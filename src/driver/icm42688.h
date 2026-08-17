#ifndef DRIVER_ICM42688_H_
#define DRIVER_ICM42688_H_

#include "hal_data.h"
#include "r_spi_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

/* 当前配置：加速度计 ±8 g，灵敏度 4096 LSB/g。 */
#define ICM42688_ACCEL_LSB_PER_G       (4096.0f)

/* 当前配置：陀螺仪 ±2000 dps，灵敏度 16.4 LSB/(dps)。 */
#define ICM42688_GYRO_LSB_PER_DPS      (16.4f)

/* ICM42688-P 的 WHO_AM_I 值。 */
#define ICM42688_WHO_AM_I_VALUE        (0x47U)

/* ICM42688 驱动返回状态。 */
typedef enum
{
    ICM42688_STATUS_OK = 0,
    ICM42688_STATUS_ARGUMENT_ERROR,
    ICM42688_STATUS_IOPORT_ERROR,
    ICM42688_STATUS_SPI_OPEN_ERROR,
    ICM42688_STATUS_SPI_TRANSFER_ERROR,
    ICM42688_STATUS_SPI_TIMEOUT,
    ICM42688_STATUS_ID_ERROR,
    ICM42688_STATUS_DATA_INVALID
} icm42688_status_t;

/* 一次同步读取的六轴原始数据。 */
typedef struct
{
    int16_t accel_x;    /* X 轴加速度原始值。 */
    int16_t accel_y;    /* Y 轴加速度原始值。 */
    int16_t accel_z;    /* Z 轴加速度原始值。 */
    int16_t gyro_x;     /* X 轴角速度原始值。 */
    int16_t gyro_y;     /* Y 轴角速度原始值。 */
    int16_t gyro_z;     /* Z 轴角速度原始值。 */
} icm42688_raw_data_t;

/*
 * SPI 中断回调。
 * FSP 中 g_spi_imu 的 Callback 必须填写 spi_imu_callback。
 */
void spi_imu_callback(spi_callback_args_t * p_args);

/*
 * 初始化 ICM42688。
 *
 * p_spi_instance：FSP 生成的 SPI 实例，例如 &g_spi_imu。
 * chip_select_pin：Pins 页面配置的普通 GPIO 片选引脚。
 */
icm42688_status_t icm42688_init(spi_instance_t const * p_spi_instance,
                                bsp_io_port_pin_t chip_select_pin);

/* 一次突发读取 AccX/Y/Z 和 GyroX/Y/Z。 */
icm42688_status_t icm42688_read_raw(icm42688_raw_data_t * p_raw_data);

/* 读取 WHO_AM_I，便于检查 SPI 接线。 */
icm42688_status_t icm42688_read_who_am_i(uint8_t * p_device_id);

#endif /* DRIVER_ICM42688_H_ */
