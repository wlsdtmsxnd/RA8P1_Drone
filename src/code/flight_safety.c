#include "flight_safety.h"

#include "imu.h"
#include "project_config.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdint.h>

#define RC_ARM_HOLD_TIME_TICKS        pdMS_TO_TICKS(1000U)

#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
#define TETHERED_ARM_LEVEL_LIMIT_DEG  (3.0f)
#define TETHERED_ARM_RATE_LIMIT_DPS   (10.0f)
#define TETHERED_ARM_STICK_EPSILON    (0.001f)
#endif

volatile flight_safety_state_t g_flight_safety_state =
    FLIGHT_SAFETY_FAILSAFE;

static TickType_t g_arm_start_tick = 0U;
static bool g_seen_arm_switch_low = false;


#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
static bool flight_safety_tethered_arm_attitude_valid(void)
{
    imu_attitude_t attitude;

    imu_get_attitude(&attitude);

    return (0 != isfinite(attitude.roll_deg)) &&
           (0 != isfinite(attitude.pitch_deg)) &&
           (0 != isfinite(attitude.gyro_x_dps)) &&
           (0 != isfinite(attitude.gyro_y_dps)) &&
           (0 != isfinite(attitude.gyro_z_dps)) &&
           (fabsf(attitude.roll_deg) <= TETHERED_ARM_LEVEL_LIMIT_DEG) &&
           (fabsf(attitude.pitch_deg) <= TETHERED_ARM_LEVEL_LIMIT_DEG) &&
           (fabsf(attitude.gyro_x_dps) <= TETHERED_ARM_RATE_LIMIT_DPS) &&
           (fabsf(attitude.gyro_y_dps) <= TETHERED_ARM_RATE_LIMIT_DPS) &&
           (fabsf(attitude.gyro_z_dps) <= TETHERED_ARM_RATE_LIMIT_DPS);
}
#endif


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
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        /* 首次离地仅允许水平且三个姿态摇杆处于软件死区时解锁。 */
        if ((fabsf(command.roll) > TETHERED_ARM_STICK_EPSILON) ||
            (fabsf(command.pitch) > TETHERED_ARM_STICK_EPSILON) ||
            (fabsf(command.yaw) > TETHERED_ARM_STICK_EPSILON) ||
            (false == flight_safety_tethered_arm_attitude_valid()))
        {
            flight_safety_stop(FLIGHT_SAFETY_DISARMED);
            return;
        }
#endif
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
