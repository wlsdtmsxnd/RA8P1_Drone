#include "icm42688.h"

#include <limits.h>
#include <string.h>

/* ICM42688 User Bank 0 寄存器。 */
#define ICM42688_REG_DEVICE_CONFIG          (0x11U)
#define ICM42688_REG_ACCEL_DATA_X1          (0x1FU)
#define ICM42688_REG_INTF_CONFIG0           (0x4CU)
#define ICM42688_REG_PWR_MGMT0              (0x4EU)
#define ICM42688_REG_GYRO_CONFIG0           (0x4FU)
#define ICM42688_REG_ACCEL_CONFIG0          (0x50U)
#define ICM42688_REG_GYRO_CONFIG1           (0x51U)
#define ICM42688_REG_GYRO_ACCEL_CONFIG0     (0x52U)
#define ICM42688_REG_ACCEL_CONFIG1          (0x53U)
#define ICM42688_REG_WHO_AM_I               (0x75U)
#define ICM42688_REG_BANK_SEL               (0x76U)

/* SPI 读操作标志。 */
#define ICM42688_SPI_READ_BIT               (0x80U)

/* 单次 SPI 传输超时时间。 */
#define ICM42688_SPI_TIMEOUT_TICKS          pdMS_TO_TICKS(5U)

/* 当前使用的 FSP SPI 实例。 */
static spi_instance_t const * g_icm42688_spi = NULL;

/* 当前使用的软件片选引脚。 */
static bsp_io_port_pin_t g_icm42688_cs_pin;

/* 等待 SPI 完成的任务。 */
static volatile TaskHandle_t g_spi_wait_task = NULL;

/* 最近一次 SPI 中断事件。 */
static volatile spi_event_t g_spi_last_event = SPI_EVENT_TRANSFER_COMPLETE;


/* SPI 中断回调。 */
void spi_imu_callback(spi_callback_args_t * p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;       /* 是否需要立即切换任务。 */
    TaskHandle_t wait_task = (TaskHandle_t) g_spi_wait_task; /* 当前等待 SPI 的任务。 */

    if ((NULL != p_args) && (NULL != wait_task))
    {
        g_spi_last_event = p_args->event;

        vTaskNotifyGiveFromISR(wait_task,
                               &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}


/* 设置软件片选电平。 */
static icm42688_status_t icm42688_set_cs(bsp_io_level_t level)
{
    fsp_err_t err;    /* IOPORT 返回值。 */

    err = g_ioport.p_api->pinWrite(g_ioport.p_ctrl,
                                   g_icm42688_cs_pin,
                                   level);

    return (FSP_SUCCESS == err) ?
           ICM42688_STATUS_OK :
           ICM42688_STATUS_IOPORT_ERROR;
}


/* 执行一次 SPI 全双工传输，并等待回调。 */
static icm42688_status_t icm42688_spi_transfer(uint8_t const * p_tx_data,
                                               uint8_t * p_rx_data,
                                               uint32_t length)
{
    fsp_err_t err;          /* FSP SPI 返回值。 */
    uint32_t notified;      /* 任务通知计数。 */

    if ((NULL == g_icm42688_spi) ||
        (NULL == p_tx_data) ||
        (NULL == p_rx_data) ||
        (0U == length))
    {
        return ICM42688_STATUS_ARGUMENT_ERROR;
    }

    /*
     * 先登记等待任务，再启动异步 SPI。
     * 即使中断很快到达，任务通知计数也不会丢失。
     */
    g_spi_wait_task = xTaskGetCurrentTaskHandle();
    g_spi_last_event = SPI_EVENT_TRANSFER_COMPLETE;
    (void) ulTaskNotifyTake(pdTRUE, 0U);

    err = g_icm42688_spi->p_api->writeRead(
        g_icm42688_spi->p_ctrl,
        p_tx_data,
        p_rx_data,
        length,
        SPI_BIT_WIDTH_8_BITS);

    if (FSP_SUCCESS != err)
    {
        g_spi_wait_task = NULL;
        return ICM42688_STATUS_SPI_TRANSFER_ERROR;
    }

    notified = ulTaskNotifyTake(pdTRUE,
                                ICM42688_SPI_TIMEOUT_TICKS);

    g_spi_wait_task = NULL;

    if (0U == notified)
    {
        return ICM42688_STATUS_SPI_TIMEOUT;
    }

    if (SPI_EVENT_TRANSFER_COMPLETE != g_spi_last_event)
    {
        return ICM42688_STATUS_SPI_TRANSFER_ERROR;
    }

    return ICM42688_STATUS_OK;
}


/* 写入一个寄存器。 */
static icm42688_status_t icm42688_write_register(uint8_t register_address,
                                                 uint8_t register_value)
{
    uint8_t tx_buffer[2] = {register_address, register_value}; /* SPI 发送缓冲区。 */
    uint8_t rx_buffer[2] = {0U, 0U};                           /* SPI 接收缓冲区。 */
    icm42688_status_t status;                                  /* 驱动状态。 */

    status = icm42688_set_cs(BSP_IO_LEVEL_LOW);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    status = icm42688_spi_transfer(tx_buffer,
                                   rx_buffer,
                                   sizeof(tx_buffer));

    if (ICM42688_STATUS_OK != icm42688_set_cs(BSP_IO_LEVEL_HIGH))
    {
        return ICM42688_STATUS_IOPORT_ERROR;
    }

    return status;
}


/* 从指定地址连续读取多个寄存器。 */
static icm42688_status_t icm42688_read_registers(uint8_t register_address,
                                                uint8_t * p_data,
                                                uint32_t data_length)
{
    uint8_t tx_buffer[16] = {0U};    /* 首字节为寄存器地址。 */
    uint8_t rx_buffer[16] = {0U};    /* 首字节为无效返回值。 */
    uint32_t data_index;             /* 数据复制索引。 */
    icm42688_status_t status;        /* 驱动状态。 */

    if ((NULL == p_data) ||
        (0U == data_length) ||
        ((data_length + 1U) > sizeof(tx_buffer)))
    {
        return ICM42688_STATUS_ARGUMENT_ERROR;
    }

    tx_buffer[0] = register_address | ICM42688_SPI_READ_BIT;

    status = icm42688_set_cs(BSP_IO_LEVEL_LOW);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    status = icm42688_spi_transfer(tx_buffer,
                                   rx_buffer,
                                   data_length + 1U);

    if (ICM42688_STATUS_OK != icm42688_set_cs(BSP_IO_LEVEL_HIGH))
    {
        return ICM42688_STATUS_IOPORT_ERROR;
    }

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    for (data_index = 0U; data_index < data_length; data_index++)
    {
        p_data[data_index] = rx_buffer[data_index + 1U];
    }

    return ICM42688_STATUS_OK;
}


/* 读取 WHO_AM_I。 */
icm42688_status_t icm42688_read_who_am_i(uint8_t * p_device_id)
{
    if (NULL == p_device_id)
    {
        return ICM42688_STATUS_ARGUMENT_ERROR;
    }

    return icm42688_read_registers(ICM42688_REG_WHO_AM_I,
                                   p_device_id,
                                   1U);
}


/* 初始化 ICM42688。 */
icm42688_status_t icm42688_init(spi_instance_t const * p_spi_instance,
                                bsp_io_port_pin_t chip_select_pin)
{
    fsp_err_t err;                  /* FSP 模块返回值。 */
    uint8_t device_id = 0U;         /* WHO_AM_I 读取值。 */
    icm42688_status_t status;       /* 驱动状态。 */

    if (NULL == p_spi_instance)
    {
        return ICM42688_STATUS_ARGUMENT_ERROR;
    }

    g_icm42688_spi = p_spi_instance;
    g_icm42688_cs_pin = chip_select_pin;

    /*
     * IOPORT 只需要打开一次。
     * 如果其他任务已经打开，则接受 FSP_ERR_ALREADY_OPEN。
     */
    err = g_ioport.p_api->open(g_ioport.p_ctrl,
                               g_ioport.p_cfg);

    if ((FSP_SUCCESS != err) &&
        (FSP_ERR_ALREADY_OPEN != err))
    {
        return ICM42688_STATUS_IOPORT_ERROR;
    }

    status = icm42688_set_cs(BSP_IO_LEVEL_HIGH);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    err = g_icm42688_spi->p_api->open(
        g_icm42688_spi->p_ctrl,
        g_icm42688_spi->p_cfg);

    if ((FSP_SUCCESS != err) &&
        (FSP_ERR_ALREADY_OPEN != err))
    {
        return ICM42688_STATUS_SPI_OPEN_ERROR;
    }

    /* 上电后等待寄存器接口稳定。 */
    vTaskDelay(pdMS_TO_TICKS(10U));

    status = icm42688_read_who_am_i(&device_id);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    if (ICM42688_WHO_AM_I_VALUE != device_id)
    {
        return ICM42688_STATUS_ID_ERROR;
    }

    /* 软件复位，复位后至少等待 1 ms。 */
    status = icm42688_write_register(ICM42688_REG_DEVICE_CONFIG,
                                     0x01U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    vTaskDelay(pdMS_TO_TICKS(10U));

    /* 选择 User Bank 0。 */
    status = icm42688_write_register(ICM42688_REG_BANK_SEL,
                                     0x00U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 保持传感器数据为大端格式，并禁用 I2C。
     * CS 必须在上电阶段保持高电平，避免设备误进入其他接口状态。
     */
    status = icm42688_write_register(ICM42688_REG_INTF_CONFIG0,
                                     0x33U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 陀螺仪：±2000 dps，ODR = 1 kHz。
     */
    status = icm42688_write_register(ICM42688_REG_GYRO_CONFIG0,
                                     0x06U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 加速度计：±8 g，ODR = 1 kHz。
     */
    status = icm42688_write_register(ICM42688_REG_ACCEL_CONFIG0,
                                     0x26U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 陀螺仪 UI 滤波器使用二阶配置。
     */
    status = icm42688_write_register(ICM42688_REG_GYRO_CONFIG1,
                                     0x16U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 加速度计 UI 滤波器使用二阶配置。
     */
    status = icm42688_write_register(ICM42688_REG_ACCEL_CONFIG1,
                                     0x0DU);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 1 kHz ODR 下，陀螺仪和加速度计 UI 低通带宽约为 125 Hz。
     * 上层仍保留 500 Hz / 40 Hz 二阶巴特沃斯滤波器。
     */
    status = icm42688_write_register(ICM42688_REG_GYRO_ACCEL_CONFIG0,
                                     0x33U);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /*
     * 加速度计和陀螺仪进入 Low Noise 模式。
     */
    status = icm42688_write_register(ICM42688_REG_PWR_MGMT0,
                                     0x0FU);

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    /* 陀螺仪启动后等待至少 45 ms。 */
    vTaskDelay(pdMS_TO_TICKS(50U));

    return ICM42688_STATUS_OK;
}


/* 一次突发读取六轴原始数据。 */
icm42688_status_t icm42688_read_raw(icm42688_raw_data_t * p_raw_data)
{
    uint8_t raw_buffer[12];          /* 六轴寄存器原始字节。 */
    icm42688_status_t status;        /* 驱动状态。 */

    if (NULL == p_raw_data)
    {
        return ICM42688_STATUS_ARGUMENT_ERROR;
    }

    /*
     * 从 ACCEL_DATA_X1 连续读取：
     * AccX、AccY、AccZ、GyroX、GyroY、GyroZ。
     */
    status = icm42688_read_registers(ICM42688_REG_ACCEL_DATA_X1,
                                     raw_buffer,
                                     sizeof(raw_buffer));

    if (ICM42688_STATUS_OK != status)
    {
        return status;
    }

    p_raw_data->accel_x =
        (int16_t) (((uint16_t) raw_buffer[0] << 8U) | raw_buffer[1]);

    p_raw_data->accel_y =
        (int16_t) (((uint16_t) raw_buffer[2] << 8U) | raw_buffer[3]);

    p_raw_data->accel_z =
        (int16_t) (((uint16_t) raw_buffer[4] << 8U) | raw_buffer[5]);

    p_raw_data->gyro_x =
        (int16_t) (((uint16_t) raw_buffer[6] << 8U) | raw_buffer[7]);

    p_raw_data->gyro_y =
        (int16_t) (((uint16_t) raw_buffer[8] << 8U) | raw_buffer[9]);

    p_raw_data->gyro_z =
        (int16_t) (((uint16_t) raw_buffer[10] << 8U) | raw_buffer[11]);

    /* 保留未解析的寄存器字节，供异常首读/复读诊断逐字节比较。 */
    memcpy(p_raw_data->bytes, raw_buffer, sizeof(raw_buffer));

    /*
     * -32768 可能代表无效采样。
     */
    if ((INT16_MIN == p_raw_data->accel_x) ||
        (INT16_MIN == p_raw_data->accel_y) ||
        (INT16_MIN == p_raw_data->accel_z) ||
        (INT16_MIN == p_raw_data->gyro_x) ||
        (INT16_MIN == p_raw_data->gyro_y) ||
        (INT16_MIN == p_raw_data->gyro_z))
    {
        return ICM42688_STATUS_DATA_INVALID;
    }

    return ICM42688_STATUS_OK;
}
