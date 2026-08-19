#include "rc_attitude_bench_test.h"

#include "actuator_manager.h"
#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "quad_x_mixer.h"
#include "rc_command.h"

#include <math.h>
#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE)
/* 拆桨遥控姿态串级测试限幅，不作为飞行参数。 */
#define RC_ATT_TEST_BASE_MIN_US              (1150.0f)
#define RC_ATT_TEST_BASE_MAX_US              (1250.0f)
#define RC_ATT_TEST_TARGET_ANGLE_DEG         (10.0f)
#define RC_ATT_TEST_ANGLE_KP_DPS_PER_DEG     (3.0f)
#define RC_ATT_TEST_RATE_KP_US_PER_DPS       (0.25f)
#define RC_ATT_TEST_RATE_TARGET_LIMIT_DPS    (45.0f)
#define RC_ATT_TEST_CORRECTION_LIMIT_US      (20.0f)
#define RC_ATT_TEST_OUTPUT_MIN_US            (1100.0f)
#define RC_ATT_TEST_OUTPUT_MAX_US            (1300.0f)
#define RC_ATT_TEST_TILT_CUTOFF_DEG          (20.0f)
#define RC_ATT_TEST_RATE_CUTOFF_DPS          (150.0f)
#define RC_ATT_TEST_YAW_NEUTRAL_LIMIT        (0.12f)


static float rc_att_test_clampf(float value,
                                float minimum,
                                float maximum)
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


static uint32_t rc_att_test_to_us(float value)
{
    value = rc_att_test_clampf(value,
                               RC_ATT_TEST_OUTPUT_MIN_US,
                               RC_ATT_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void rc_attitude_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE)
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
    float target_roll_deg;
    float target_pitch_deg;
    float roll_target_rate_dps;
    float pitch_target_rate_dps;
    float roll_correction_us;
    float pitch_correction_us;
    float motor_us[ACTUATOR_MANAGER_COUNT];
    uint32_t actuator_us[ACTUATOR_MANAGER_COUNT];
    uint32_t motor_index;

    if ((false == imu_healthy) ||
        (false == flight_safety_is_armed()))
    {
        (void) actuator_manager_stop();
        return;
    }

    rc_command_get(&command);

    if ((false == command.connected) ||
        (true == command.throttle_low) ||
        (fabsf(command.yaw) > RC_ATT_TEST_YAW_NEUTRAL_LIMIT))
    {
        (void) actuator_manager_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > RC_ATT_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > RC_ATT_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > RC_ATT_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > RC_ATT_TEST_RATE_CUTOFF_DPS))
    {
        (void) actuator_manager_stop();
        return;
    }

    base_us = RC_ATT_TEST_BASE_MIN_US +
              (command.throttle *
               (RC_ATT_TEST_BASE_MAX_US - RC_ATT_TEST_BASE_MIN_US));
    target_roll_deg = rc_att_test_clampf(command.roll, -1.0f, 1.0f) *
                      RC_ATT_TEST_TARGET_ANGLE_DEG;
    target_pitch_deg = rc_att_test_clampf(command.pitch, -1.0f, 1.0f) *
                       RC_ATT_TEST_TARGET_ANGLE_DEG;

    roll_target_rate_dps = rc_att_test_clampf(
        (target_roll_deg - attitude.roll_deg) *
        RC_ATT_TEST_ANGLE_KP_DPS_PER_DEG,
        -RC_ATT_TEST_RATE_TARGET_LIMIT_DPS,
        RC_ATT_TEST_RATE_TARGET_LIMIT_DPS);
    pitch_target_rate_dps = rc_att_test_clampf(
        (target_pitch_deg - attitude.pitch_deg) *
        RC_ATT_TEST_ANGLE_KP_DPS_PER_DEG,
        -RC_ATT_TEST_RATE_TARGET_LIMIT_DPS,
        RC_ATT_TEST_RATE_TARGET_LIMIT_DPS);

    roll_correction_us = rc_att_test_clampf(
        (roll_target_rate_dps - attitude.gyro_x_dps) *
        RC_ATT_TEST_RATE_KP_US_PER_DPS,
        -RC_ATT_TEST_CORRECTION_LIMIT_US,
        RC_ATT_TEST_CORRECTION_LIMIT_US);
    pitch_correction_us = rc_att_test_clampf(
        (pitch_target_rate_dps - attitude.gyro_y_dps) *
        RC_ATT_TEST_RATE_KP_US_PER_DPS,
        -RC_ATT_TEST_CORRECTION_LIMIT_US,
        RC_ATT_TEST_CORRECTION_LIMIT_US);

    quad_x_mixer_apply(base_us,
                       roll_correction_us,
                       pitch_correction_us,
                       0.0f,
                       motor_us);

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        actuator_us[motor_index] = rc_att_test_to_us(motor_us[motor_index]);
    }

    if (ACTUATOR_MANAGER_STATUS_OK !=
        actuator_manager_apply_us(actuator_us))
    {
        flight_safety_force_failsafe(
            FLIGHT_SAFETY_STOP_MOTOR_OUTPUT_ERROR);
    }
#else
    (void) imu_healthy;
#endif
}
