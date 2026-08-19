#include "rc_yaw_rate_bench_test.h"

#include "actuator_manager.h"
#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "quad_x_mixer.h"
#include "rc_command.h"

#include <math.h>
#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE)
/* 拆桨 Yaw 目标角速度测试限幅，不作为飞行参数。 */
#define YAW_RATE_TEST_BASE_MIN_US             (1150.0f)
#define YAW_RATE_TEST_BASE_MAX_US             (1250.0f)
#define YAW_RATE_TEST_TARGET_LIMIT_DPS        (30.0f)
#define YAW_RATE_TEST_KP_US_PER_DPS           (0.25f)
#define YAW_RATE_TEST_CORRECTION_LIMIT_US     (20.0f)
#define YAW_RATE_TEST_OUTPUT_MIN_US           (1100.0f)
#define YAW_RATE_TEST_OUTPUT_MAX_US           (1300.0f)
#define YAW_RATE_TEST_TILT_CUTOFF_DEG         (20.0f)
#define YAW_RATE_TEST_RATE_CUTOFF_DPS         (150.0f)
#define YAW_RATE_TEST_STICK_NEUTRAL_LIMIT     (0.12f)


static float yaw_rate_test_clampf(float value,
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


static uint32_t yaw_rate_test_to_us(float value)
{
    value = yaw_rate_test_clampf(value,
                                 YAW_RATE_TEST_OUTPUT_MIN_US,
                                 YAW_RATE_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void rc_yaw_rate_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE)
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
    float yaw_target_rate_dps;
    float yaw_correction_us;
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
        (fabsf(command.roll) > YAW_RATE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.pitch) > YAW_RATE_TEST_STICK_NEUTRAL_LIMIT))
    {
        (void) actuator_manager_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > YAW_RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > YAW_RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_z_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS))
    {
        (void) actuator_manager_stop();
        return;
    }

    base_us = YAW_RATE_TEST_BASE_MIN_US +
              (command.throttle *
               (YAW_RATE_TEST_BASE_MAX_US - YAW_RATE_TEST_BASE_MIN_US));
    yaw_target_rate_dps = yaw_rate_test_clampf(command.yaw, -1.0f, 1.0f) *
                          YAW_RATE_TEST_TARGET_LIMIT_DPS;
    yaw_correction_us = yaw_rate_test_clampf(
        (yaw_target_rate_dps - attitude.gyro_z_dps) *
        YAW_RATE_TEST_KP_US_PER_DPS,
        -YAW_RATE_TEST_CORRECTION_LIMIT_US,
        YAW_RATE_TEST_CORRECTION_LIMIT_US);

    quad_x_mixer_apply(base_us,
                       0.0f,
                       0.0f,
                       yaw_correction_us,
                       motor_us);

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        actuator_us[motor_index] = yaw_rate_test_to_us(motor_us[motor_index]);
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
