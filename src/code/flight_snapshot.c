#include "flight_snapshot.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

static flight_snapshot_t g_flight_snapshot =
{
    .actuator_us =
    {
        ACTUATOR_MANAGER_MIN_US,
        ACTUATOR_MANAGER_MIN_US,
        ACTUATOR_MANAGER_MIN_US,
        ACTUATOR_MANAGER_MIN_US
    },
    .safety_state = FLIGHT_SAFETY_FAILSAFE,
    .stop_reason = FLIGHT_SAFETY_STOP_NONE,
    .imu_healthy = false,
    .valid = false
};


void flight_snapshot_publish(bool imu_healthy)
{
    flight_snapshot_t next = {0};
    uint32_t actuator_index;

    taskENTER_CRITICAL();
    next.sequence = g_flight_snapshot.sequence + 1U;
    taskEXIT_CRITICAL();

    next.tick_ms = (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (actuator_index = 0U;
         actuator_index < ACTUATOR_MANAGER_COUNT;
         actuator_index++)
    {
        next.actuator_us[actuator_index] =
            actuator_manager_get_us(actuator_index);
    }

    imu_get_attitude(&next.attitude);
    rc_command_get(&next.command);
    flight_control_get_status(&next.control);
    next.safety_state = flight_safety_get_state();
    next.stop_reason = flight_safety_get_stop_reason();
    next.imu_healthy = imu_healthy;
    next.valid = true;

    taskENTER_CRITICAL();
    g_flight_snapshot = next;
    taskEXIT_CRITICAL();
}


void flight_snapshot_get(flight_snapshot_t * p_snapshot)
{
    if (NULL == p_snapshot)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_snapshot = g_flight_snapshot;
    taskEXIT_CRITICAL();
}
