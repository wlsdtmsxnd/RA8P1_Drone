#include "flight_safety.h"

#include "imu.h"
#include "../driver/crsf.h"
#include "../driver/motor_output.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define RC_THROTTLE_CHANNEL_INDEX     (2U)
#define RC_ARM_CHANNEL_INDEX          (4U)
#define RC_THROTTLE_LOW_MAX_US        (1050U)
#define RC_ARM_LOW_MAX_US             (1300U)
#define RC_ARM_HIGH_MIN_US            (1700U)
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
    crsf_data_t rc_data;
    uint16_t throttle_us;
    uint16_t arm_us;
    TickType_t now;

    crsf_get_data(&rc_data);

    if ((false == imu_healthy) ||
        (false == rc_data.connected))
    {
        g_seen_arm_switch_low = false;
        flight_safety_stop(FLIGHT_SAFETY_FAILSAFE);
        return;
    }

    throttle_us = rc_data.channel_us[RC_THROTTLE_CHANNEL_INDEX];
    arm_us = rc_data.channel_us[RC_ARM_CHANNEL_INDEX];

    if (arm_us <= RC_ARM_LOW_MAX_US)
    {
        g_seen_arm_switch_low = true;
        flight_safety_stop(FLIGHT_SAFETY_DISARMED);
        return;
    }

    if (FLIGHT_SAFETY_ARMED == g_flight_safety_state)
    {
        if (arm_us < RC_ARM_HIGH_MIN_US)
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

    if ((arm_us >= RC_ARM_HIGH_MIN_US) &&
        (throttle_us <= RC_THROTTLE_LOW_MAX_US))
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
