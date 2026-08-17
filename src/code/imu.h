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
    IMU_STATUS_NOT_READY
} imu_status_t;

/* 对外发布的欧拉角。 */
typedef struct
{
    float roll_deg;      /* 横滚角，单位：度。 */
    float pitch_deg;     /* 俯仰角，单位：度。 */
    float yaw_deg;       /* 航向角，单位：度。 */
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

/* 查询 IMU 是否初始化完成。 */
bool imu_is_ready(void);

/* 将当前航向角设为新的零点，在解锁瞬间调用。 */
void imu_zero_yaw(void);

#endif /* CODE_IMU_H_ */
