#include "flight_control.h"

#include "flight_safety.h"
#include "imu.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

#include <math.h>
#include <stdint.h>

/* 初次闭环台架参数：只用于无桨方向验证，不能带桨飞行。 */
#define CONTROL_DT_S                         (0.002f)
#define CONTROL_MAX_ANGLE_DEG                (10.0f)
#define CONTROL_MAX_YAW_RATE_DPS             (45.0f)
#define CONTROL_MAX_RATE_SETPOINT_DPS        (80.0f)
#define CONTROL_MAX_CORRECTION_US            (60.0f)
#define CONTROL_ARMED_IDLE_US                (1150U)
#define CONTROL_MAX_OUTPUT_US                (1300U)
#define CONTROL_TILT_CUTOFF_DEG              (35.0f)

#define CONTROL_ANGLE_ROLL_KP                (4.0f)
#define CONTROL_ANGLE_PITCH_KP               (4.0f)
#define CONTROL_RATE_ROLL_KP                 (0.25f)
#define CONTROL_RATE_PITCH_KP                (0.25f)
#define CONTROL_RATE_YAW_KP                  (0.20f)

typedef struct
{
    float angle_roll_rate_dps;
    float angle_pitch_rate_dps;
} flight_control_state_t;

static flight_control_state_t g_control_state = {0};


static float control_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static uint32_t control_float_to_us(float value)
{
    value = control_clampf(value,
                           (float) MOTOR_OUTPUT_MIN_US,
                           (float) CONTROL_MAX_OUTPUT_US);

    return (uint32_t) (value + 0.5f);
}


static void control_reset(void)
{
    g_control_state = (flight_control_state_t) {0};
    motor_output_all_stop();
}


void flight_control_init(void)
{
    control_reset();
}


void flight_control_update(bool imu_healthy)
{
    imu_attitude_t attitude;
    rc_command_t command;
    float roll_target_deg;
    float pitch_target_deg;
    float yaw_rate_target_dps;
    float roll_rate_target_dps;
    float pitch_rate_target_dps;
    float roll_correction_us;
    float pitch_correction_us;
    float yaw_correction_us;
    float base_us;
    float motor_us[MOTOR_OUTPUT_COUNT];

    if ((false == imu_healthy) || (false == flight_safety_is_armed()))
    {
        control_reset();
        return;
    }

    rc_command_get(&command);

    if ((false == command.connected) || (true == command.throttle_low))
    {
        control_reset();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > CONTROL_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > CONTROL_TILT_CUTOFF_DEG))
    {
        control_reset();
        return;
    }

    roll_target_deg = command.roll * CONTROL_MAX_ANGLE_DEG;
    pitch_target_deg = command.pitch * CONTROL_MAX_ANGLE_DEG;
    yaw_rate_target_dps = command.yaw * CONTROL_MAX_YAW_RATE_DPS;

    g_control_state.angle_roll_rate_dps =
        CONTROL_ANGLE_ROLL_KP * (roll_target_deg - attitude.roll_deg);
    g_control_state.angle_pitch_rate_dps =
        CONTROL_ANGLE_PITCH_KP * (pitch_target_deg - attitude.pitch_deg);

    roll_rate_target_dps =
        control_clampf(g_control_state.angle_roll_rate_dps,
                       -CONTROL_MAX_RATE_SETPOINT_DPS,
                       CONTROL_MAX_RATE_SETPOINT_DPS);
    pitch_rate_target_dps =
        control_clampf(g_control_state.angle_pitch_rate_dps,
                       -CONTROL_MAX_RATE_SETPOINT_DPS,
                       CONTROL_MAX_RATE_SETPOINT_DPS);

    roll_correction_us =
        CONTROL_RATE_ROLL_KP * (roll_rate_target_dps - attitude.gyro_x_dps);
    pitch_correction_us =
        CONTROL_RATE_PITCH_KP * (pitch_rate_target_dps - attitude.gyro_y_dps);
    yaw_correction_us =
        CONTROL_RATE_YAW_KP * (yaw_rate_target_dps - attitude.gyro_z_dps);

    roll_correction_us = control_clampf(roll_correction_us,
                                        -CONTROL_MAX_CORRECTION_US,
                                        CONTROL_MAX_CORRECTION_US);
    pitch_correction_us = control_clampf(pitch_correction_us,
                                         -CONTROL_MAX_CORRECTION_US,
                                         CONTROL_MAX_CORRECTION_US);
    yaw_correction_us = control_clampf(yaw_correction_us,
                                       -CONTROL_MAX_CORRECTION_US,
                                       CONTROL_MAX_CORRECTION_US);

    base_us = (float) CONTROL_ARMED_IDLE_US +
              (command.throttle *
               ((float) CONTROL_MAX_OUTPUT_US -
                (float) CONTROL_ARMED_IDLE_US));

    /*
     * Quad-X: M1 左前，M2 右前，M3 右后，M4 左后。
     * 后排公式按 RA8P1 实测电机位置排列，避免沿用 M3/M4 互换的参考顺序。
     */
    motor_us[0] = base_us + pitch_correction_us +
                  roll_correction_us + yaw_correction_us;
    motor_us[1] = base_us + pitch_correction_us -
                  roll_correction_us - yaw_correction_us;
    motor_us[2] = base_us - pitch_correction_us -
                  roll_correction_us + yaw_correction_us;
    motor_us[3] = base_us - pitch_correction_us +
                  roll_correction_us - yaw_correction_us;

    (void) motor_output_set_us(0U, control_float_to_us(motor_us[0]));
    (void) motor_output_set_us(1U, control_float_to_us(motor_us[1]));
    (void) motor_output_set_us(2U, control_float_to_us(motor_us[2]));
    (void) motor_output_set_us(3U, control_float_to_us(motor_us[3]));

    (void) CONTROL_DT_S;
}
