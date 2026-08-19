#include "tpf_flow.h"
#include "code/project_config.h"

#include <math.h>
#include <stddef.h>

/* 软件 I2C 引脚档位；每组第一个为 SCL，第二个为 SDA。 */
#if (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P411_P412)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_11
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_12
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P413_P414)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_13
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_14
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P408_P409)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_08
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_09
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P400_P401)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_00
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_01
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P402_P403)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_02
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_03
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P404_P405)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_04
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_05
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P406_P407)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_06
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_07
#elif (TPF_FLOW_PIN_PAIR == TPF_FLOW_PIN_PAIR_P406_P409)
#define TPF_FLOW_SCL_PIN                 BSP_IO_PORT_04_PIN_06
#define TPF_FLOW_SDA_PIN                 BSP_IO_PORT_04_PIN_09
#else
#error "Unsupported TPF flow pin pair"
#endif

/* 与 RA6M5 实机相同的器件地址和数据寄存器。 */
#define TPF_FLOW_ADDRESS_WRITE           (0xC4U)
#define TPF_FLOW_ADDRESS_READ            (0xC5U)
#define TPF_FLOW_DATA_REGISTER           (0x33U)
#define TPF_FLOW_DATA_LENGTH             (5U)

#define TPF_FLOW_MIN_DISTANCE_MM         (20U)
#define TPF_FLOW_MAX_DISTANCE_MM         (3000U)
#define TPF_FLOW_MIN_SCALE_HEIGHT_MM     (100.0f)
#define TPF_FLOW_MIN_TILT_COSINE         (0.707f)
#define TPF_FLOW_DISTANCE_SCALE          (3400.0f)
#define TPF_FLOW_VELOCITY_SCALE          (30.0f)
#define TPF_FLOW_GYRO_COMPENSATION       (0.05f)
#define TPF_FLOW_FILTER_OLD_WEIGHT       (0.85f)
#define TPF_FLOW_FILTER_NEW_WEIGHT       (0.15f)
#define TPF_FLOW_DEG_TO_RAD              (0.01745329251994329577f)

#define TPF_FLOW_HALF_PERIOD_US          (2U)
#define TPF_FLOW_CLOCK_STRETCH_LIMIT     (50U)
#define TPF_FLOW_BUS_RECOVERY_CLOCKS     (9U)

static bool g_tpf_flow_initialized = false;

volatile tpf_flow_data_t g_tpf_flow_data = {0};

static void tpf_flow_delay(void)
{
    R_BSP_SoftwareDelay(TPF_FLOW_HALF_PERIOD_US,
                        BSP_DELAY_UNITS_MICROSECONDS);
}

static void tpf_flow_scl_write(bsp_io_level_t level)
{
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl, TPF_FLOW_SCL_PIN, level);
}

static void tpf_flow_sda_write(bsp_io_level_t level)
{
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl, TPF_FLOW_SDA_PIN, level);
}

static bool tpf_flow_pin_is_high(bsp_io_port_pin_t pin)
{
    bsp_io_level_t level = BSP_IO_LEVEL_LOW;

    if (FSP_SUCCESS != R_IOPORT_PinRead(&g_ioport_ctrl, pin, &level))
    {
        return false;
    }

    return (BSP_IO_LEVEL_HIGH == level);
}

static bool tpf_flow_release_scl(void)
{
    uint32_t wait_count;

    tpf_flow_scl_write(BSP_IO_LEVEL_HIGH);

    for (wait_count = 0U;
         wait_count < TPF_FLOW_CLOCK_STRETCH_LIMIT;
         wait_count++)
    {
        if (tpf_flow_pin_is_high(TPF_FLOW_SCL_PIN))
        {
            tpf_flow_delay();
            return true;
        }

        tpf_flow_delay();
    }

    return false;
}

static bool tpf_flow_i2c_start(void)
{
    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);

    if (!tpf_flow_release_scl())
    {
        return false;
    }

    if (!tpf_flow_pin_is_high(TPF_FLOW_SDA_PIN))
    {
        return false;
    }

    tpf_flow_sda_write(BSP_IO_LEVEL_LOW);
    tpf_flow_delay();
    tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
    tpf_flow_delay();
    return true;
}

static bool tpf_flow_i2c_stop(void)
{
    tpf_flow_sda_write(BSP_IO_LEVEL_LOW);
    tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
    tpf_flow_delay();

    if (!tpf_flow_release_scl())
    {
        return false;
    }

    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);
    tpf_flow_delay();
    return tpf_flow_pin_is_high(TPF_FLOW_SDA_PIN);
}

static bool tpf_flow_i2c_write_byte(uint8_t value)
{
    uint32_t bit_index;
    bool acknowledged;

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        if (0U != (value & 0x80U))
        {
            tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);
        }
        else
        {
            tpf_flow_sda_write(BSP_IO_LEVEL_LOW);
        }

        tpf_flow_delay();

        if (!tpf_flow_release_scl())
        {
            return false;
        }

        tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
        tpf_flow_delay();
        value = (uint8_t) (value << 1U);
    }

    /* 释放 SDA，让从机在第九个时钟拉低表示 ACK。 */
    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);
    tpf_flow_delay();

    if (!tpf_flow_release_scl())
    {
        return false;
    }

    acknowledged = !tpf_flow_pin_is_high(TPF_FLOW_SDA_PIN);
    tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
    tpf_flow_delay();
    return acknowledged;
}

static bool tpf_flow_i2c_read_byte(uint8_t * p_value, bool send_ack)
{
    uint32_t bit_index;
    uint8_t value = 0U;

    if (NULL == p_value)
    {
        return false;
    }

    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        value = (uint8_t) (value << 1U);

        if (!tpf_flow_release_scl())
        {
            return false;
        }

        if (tpf_flow_pin_is_high(TPF_FLOW_SDA_PIN))
        {
            value |= 1U;
        }

        tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
        tpf_flow_delay();
    }

    tpf_flow_sda_write(send_ack ? BSP_IO_LEVEL_LOW : BSP_IO_LEVEL_HIGH);
    tpf_flow_delay();

    if (!tpf_flow_release_scl())
    {
        return false;
    }

    tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);
    tpf_flow_delay();
    *p_value = value;
    return true;
}

static bool tpf_flow_recover_bus(void)
{
    uint32_t clock_index;

    tpf_flow_sda_write(BSP_IO_LEVEL_HIGH);
    tpf_flow_scl_write(BSP_IO_LEVEL_HIGH);
    tpf_flow_delay();

    if (!tpf_flow_pin_is_high(TPF_FLOW_SCL_PIN))
    {
        return false;
    }

    if (!tpf_flow_pin_is_high(TPF_FLOW_SDA_PIN))
    {
        for (clock_index = 0U;
             clock_index < TPF_FLOW_BUS_RECOVERY_CLOCKS;
             clock_index++)
        {
            tpf_flow_scl_write(BSP_IO_LEVEL_LOW);
            tpf_flow_delay();

            if (!tpf_flow_release_scl())
            {
                return false;
            }
        }
    }

    return tpf_flow_i2c_stop();
}

static bool tpf_flow_read_frame(uint8_t * p_buffer,
                                uint32_t * p_ack_error_count,
                                uint32_t * p_bus_error_count)
{
    uint32_t byte_index;
    bool transaction_started = false;

    if ((NULL == p_buffer) ||
        (NULL == p_ack_error_count) ||
        (NULL == p_bus_error_count))
    {
        return false;
    }

    if (!tpf_flow_i2c_start())
    {
        (*p_bus_error_count)++;
        (void) tpf_flow_recover_bus();
        return false;
    }

    transaction_started = true;

    if (!tpf_flow_i2c_write_byte(TPF_FLOW_ADDRESS_WRITE))
    {
        (*p_ack_error_count)++;
        goto transaction_failed;
    }

    if (!tpf_flow_i2c_write_byte(TPF_FLOW_DATA_REGISTER))
    {
        (*p_ack_error_count)++;
        goto transaction_failed;
    }

    if (!tpf_flow_i2c_start())
    {
        (*p_bus_error_count)++;
        goto transaction_failed;
    }

    if (!tpf_flow_i2c_write_byte(TPF_FLOW_ADDRESS_READ))
    {
        (*p_ack_error_count)++;
        goto transaction_failed;
    }

    for (byte_index = 0U; byte_index < TPF_FLOW_DATA_LENGTH; byte_index++)
    {
        if (!tpf_flow_i2c_read_byte(&p_buffer[byte_index],
                                    byte_index < (TPF_FLOW_DATA_LENGTH - 1U)))
        {
            (*p_bus_error_count)++;
            goto transaction_failed;
        }
    }

    if (!tpf_flow_i2c_stop())
    {
        (*p_bus_error_count)++;
        return false;
    }

    return true;

transaction_failed:
    if (transaction_started && !tpf_flow_i2c_stop())
    {
        (*p_bus_error_count)++;
    }

    return false;
}

fsp_err_t tpf_flow_init(void)
{
    fsp_err_t err;
    uint32_t pin_cfg;

    pin_cfg = ((uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT |
               (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH |
               (uint32_t) IOPORT_CFG_NMOS_ENABLE |
               (uint32_t) IOPORT_CFG_PULLUP_ENABLE);

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, TPF_FLOW_SCL_PIN, pin_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_IOPORT_PinCfg(&g_ioport_ctrl, TPF_FLOW_SDA_PIN, pin_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    taskENTER_CRITICAL();
    g_tpf_flow_data = (tpf_flow_data_t) {0};
    taskEXIT_CRITICAL();
    g_tpf_flow_initialized = true;

    /*
     * 即使上电时总线被拉低，也保留周期恢复和错误遥测能力；调用者仍可
     * 通过返回值知道首次恢复失败，但不必重新初始化才能看到故障计数。
     */
    if (!tpf_flow_recover_bus())
    {
        taskENTER_CRITICAL();
        g_tpf_flow_data.bus_error_count = 1U;
        taskEXIT_CRITICAL();
        return FSP_ERR_TIMEOUT;
    }

    return FSP_SUCCESS;
}

void tpf_flow_poll(float roll_deg,
                   float pitch_deg,
                   float gyro_x_dps,
                   float gyro_y_dps)
{
    uint8_t buffer[TPF_FLOW_DATA_LENGTH];
    uint16_t distance_mm;
    float roll_cosine;
    float pitch_cosine;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float compensated_velocity_x;
    float compensated_velocity_y;
    tpf_flow_data_t data;

    if (!g_tpf_flow_initialized)
    {
        return;
    }

    taskENTER_CRITICAL();
    data = g_tpf_flow_data;
    taskEXIT_CRITICAL();

    if (!tpf_flow_read_frame(buffer,
                             &data.ack_error_count,
                             &data.bus_error_count))
    {
        taskENTER_CRITICAL();
        g_tpf_flow_data.ack_error_count = data.ack_error_count;
        g_tpf_flow_data.bus_error_count = data.bus_error_count;
        taskEXIT_CRITICAL();
        return;
    }

    distance_mm = (uint16_t) (((uint16_t) buffer[0] << 8U) |
                              (uint16_t) buffer[1]);

    if ((distance_mm <= TPF_FLOW_MIN_DISTANCE_MM) ||
        (distance_mm >= TPF_FLOW_MAX_DISTANCE_MM))
    {
        data.valid = false;
        data.distance_mm = distance_mm;
        data.quality = buffer[4];
        data.filtered_velocity_x_mm_s *= TPF_FLOW_FILTER_OLD_WEIGHT;
        data.filtered_velocity_y_mm_s *= TPF_FLOW_FILTER_OLD_WEIGHT;

        taskENTER_CRITICAL();
        g_tpf_flow_data = data;
        taskEXIT_CRITICAL();
        return;
    }

    data.distance_mm = distance_mm;
    data.raw_velocity_x = (int8_t) buffer[2];
    data.raw_velocity_y = (int8_t) buffer[3];
    data.quality = buffer[4];

    roll_cosine = cosf(roll_deg * TPF_FLOW_DEG_TO_RAD);
    pitch_cosine = cosf(pitch_deg * TPF_FLOW_DEG_TO_RAD);

    if (roll_cosine < TPF_FLOW_MIN_TILT_COSINE)
    {
        roll_cosine = TPF_FLOW_MIN_TILT_COSINE;
    }

    if (pitch_cosine < TPF_FLOW_MIN_TILT_COSINE)
    {
        pitch_cosine = TPF_FLOW_MIN_TILT_COSINE;
    }

    data.tilt_height_mm = (float) distance_mm * roll_cosine * pitch_cosine;

    if (data.tilt_height_mm > TPF_FLOW_MIN_SCALE_HEIGHT_MM)
    {
        data.scaled_velocity_x_mm_s =
            (float) data.raw_velocity_x *
            (data.tilt_height_mm / TPF_FLOW_DISTANCE_SCALE) *
            TPF_FLOW_VELOCITY_SCALE;
        data.scaled_velocity_y_mm_s =
            (float) data.raw_velocity_y *
            (data.tilt_height_mm / TPF_FLOW_DISTANCE_SCALE) *
            TPF_FLOW_VELOCITY_SCALE;

        gyro_x_rad_s = gyro_x_dps * TPF_FLOW_DEG_TO_RAD;
        gyro_y_rad_s = gyro_y_dps * TPF_FLOW_DEG_TO_RAD;
        compensated_velocity_x =
            data.scaled_velocity_x_mm_s -
            (gyro_y_rad_s * data.tilt_height_mm *
             TPF_FLOW_GYRO_COMPENSATION);
        compensated_velocity_y =
            data.scaled_velocity_y_mm_s -
            (gyro_x_rad_s * data.tilt_height_mm *
             TPF_FLOW_GYRO_COMPENSATION);
        data.filtered_velocity_x_mm_s =
            (data.filtered_velocity_x_mm_s * TPF_FLOW_FILTER_OLD_WEIGHT) +
            (compensated_velocity_x * TPF_FLOW_FILTER_NEW_WEIGHT);
        data.filtered_velocity_y_mm_s =
            (data.filtered_velocity_y_mm_s * TPF_FLOW_FILTER_OLD_WEIGHT) +
            (compensated_velocity_y * TPF_FLOW_FILTER_NEW_WEIGHT);
    }
    else
    {
        data.scaled_velocity_x_mm_s = 0.0f;
        data.scaled_velocity_y_mm_s = 0.0f;
        data.filtered_velocity_x_mm_s *= TPF_FLOW_FILTER_OLD_WEIGHT;
        data.filtered_velocity_y_mm_s *= TPF_FLOW_FILTER_OLD_WEIGHT;
    }

    data.sample_count++;
    data.last_sample_tick = xTaskGetTickCount();
    data.valid = true;

    taskENTER_CRITICAL();
    g_tpf_flow_data = data;
    taskEXIT_CRITICAL();
}

void tpf_flow_get_data(tpf_flow_data_t * p_data)
{
    TickType_t current_tick;

    if (NULL == p_data)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_data = g_tpf_flow_data;
    taskEXIT_CRITICAL();

    current_tick = xTaskGetTickCount();

    if ((!p_data->valid) ||
        ((current_tick - p_data->last_sample_tick) >
         pdMS_TO_TICKS(TPF_FLOW_SIGNAL_TIMEOUT_MS)))
    {
        p_data->valid = false;
    }
}

bool tpf_flow_is_ready(void)
{
    tpf_flow_data_t data;

    tpf_flow_get_data(&data);
    return data.valid;
}
