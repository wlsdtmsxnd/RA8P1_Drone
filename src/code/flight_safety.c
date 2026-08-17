#include "flight_safety.h"

#include "imu.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define RC_ARM_HOLD_TIME_TICKS        pdMS_TO_TICKS(1000U)

volatile flight_safety_state_t g_flight_safety_state =
    FLIGHT_SAFETY_FAILSAFE;

static TickType_t g_arm_start_tick = 0U;
static bool g_seen_arm_switch_low = false;


static void flight_safety_stop(flight_safety_state_t state)
{
    g_flight_safety_state = state;
    g_arm_start_tick = 0U;
    motor_output_all_stop();
}


void flight_safety_init(void)
{
    g_seen_arm_switch_low = false;
    flight_safety_stop(FLIGHT_SAFETY_FAILSAFE);
}


void flight_safety_update(bool imu_healthy)
{
    rc_command_t command;
    TickType_t now;

    rc_command_get(&command);

    if ((false == imu_healthy) ||
        (false == command.connected))
    {
        g_seen_arm_switch_low = false;
        flight_safety_stop(FLIGHT_SAFETY_FAILSAFE);
        return;
    }

    if (true == command.arm_switch_low)
    {
        g_seen_arm_switch_low = true;
        flight_safety_stop(FLIGHT_SAFETY_DISARMED);
        return;
    }

    if (FLIGHT_SAFETY_ARMED == g_flight_safety_state)
    {
        if (false == command.arm_switch_high)
        {
            flight_safety_stop(FLIGHT_SAFETY_DISARMED);
        }

        return;
    }

    if (false == g_seen_arm_switch_low)
    {
        flight_safety_stop(FLIGHT_SAFETY_DISARMED);
        return;
    }

    if ((true == command.arm_switch_high) &&
        (true == command.throttle_low))
    {
        now = xTaskGetTickCount();

        if (FLIGHT_SAFETY_ARMING_WAIT != g_flight_safety_state)
        {
            g_flight_safety_state = FLIGHT_SAFETY_ARMING_WAIT;
            g_arm_start_tick = now;
            motor_output_all_stop();
        }
        else if ((now - g_arm_start_tick) >=
                 RC_ARM_HOLD_TIME_TICKS)
        {
            g_flight_safety_state = FLIGHT_SAFETY_ARMED;
            imu_zero_yaw();
        }

        return;
    }

    flight_safety_stop(FLIGHT_SAFETY_DISARMED);
}


flight_safety_state_t flight_safety_get_state(void)
{
    return g_flight_safety_state;
}


bool flight_safety_is_armed(void)
{
    return FLIGHT_SAFETY_ARMED == g_flight_safety_state;
}
