#ifndef CODE_FLIGHT_SNAPSHOT_H_
#define CODE_FLIGHT_SNAPSHOT_H_

#include "actuator_manager.h"
#include "flight_control.h"
#include "flight_safety.h"
#include "imu.h"
#include "rc_command.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sequence;
    uint32_t tick_ms;
    uint32_t actuator_us[ACTUATOR_MANAGER_COUNT];
    imu_attitude_t attitude;
    rc_command_t command;
    flight_control_status_t control;
    flight_safety_state_t safety_state;
    flight_safety_stop_reason_t stop_reason;
    bool imu_healthy;
    bool valid;
} flight_snapshot_t;

/* 只能在 500 Hz IMU/控制任务完成本周期处理后发布。 */
void flight_snapshot_publish(bool imu_healthy);
void flight_snapshot_get(flight_snapshot_t * p_snapshot);

#endif /* CODE_FLIGHT_SNAPSHOT_H_ */
