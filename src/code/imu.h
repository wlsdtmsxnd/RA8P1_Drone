#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "hal_data.h"
#include "r_spi_api.h"

#include <stdbool.h>
#include <stdint.h>

/* IMU 姿态算法状态。 */
typedef enum
{
    IMU_STATUS_OK = 0,
    IMU_STATUS_DRIVER_ERROR,
    IMU_STATUS_NOT_READY,
    IMU_STATUS_CALIBRATION_MOTION
} imu_status_t;

/* 启动静止标定的状态，用于调试器和数传观察。 */
typedef enum
{
    IMU_CALIBRATION_NOT_STARTED = 0,
    IMU_CALIBRATION_IN_PROGRESS,
    IMU_CALIBRATION_SUCCESS,
    IMU_CALIBRATION_MOTION,
    IMU_CALIBRATION_DRIVER_ERROR
} imu_calibration_state_t;

/* 启动标定结果。零偏和最大波动单位均为度每秒。 */
typedef struct
{
    float gyro_offset_x_dps;
    float gyro_offset_y_dps;
    float gyro_offset_z_dps;
    float gyro_span_max_dps;
    uint32_t sample_count;
    imu_calibration_state_t state;
} imu_calibration_t;

/* 对外发布的欧拉角。 */
typedef struct
{
    float roll_deg;      /* 横滚角，单位：度。 */
    float pitch_deg;     /* 俯仰角，单位：度。 */
    float yaw_deg;       /* 航向角，单位：度。 */
    float gyro_x_dps;    /* X 轴滤波角速度，单位：度每秒。 */
    float gyro_y_dps;    /* Y 轴滤波角速度，单位：度每秒。 */
    float gyro_z_dps;    /* Z 轴滤波角速度，单位：度每秒。 */
} imu_attitude_t;

/*
 * 初始化 ICM42688，并完成静止零偏标定。
 */
imu_status_t imu_init(spi_instance_t const * p_spi_instance,
                      bsp_io_port_pin_t chip_select_pin);

/* 执行一次 2 ms 姿态更新。 */
imu_status_t imu_update(void);

/* 获取三个角度的一致快照。 */
void imu_get_attitude(imu_attitude_t * p_attitude);

/* 获取启动标定结果的一致快照。 */
void imu_get_calibration(imu_calibration_t * p_calibration);

/* 查询 IMU 是否初始化完成。 */
bool imu_is_ready(void);

/* 将当前航向角设为新的零点，在解锁瞬间调用。 */
void imu_zero_yaw(void);

#endif /* CODE_IMU_H_ */
