#include "rc_yaw_rate_bench_test.h"

#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

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
    float motor_us[MOTOR_OUTPUT_COUNT];
    uint32_t motor_index;

    if ((false == imu_healthy) ||
        (false == flight_safety_is_armed()))
    {
        motor_output_all_stop();
        return;
    }

    rc_command_get(&command);

    if ((false == command.connected) ||
        (true == command.throttle_low) ||
        (fabsf(command.roll) > YAW_RATE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.pitch) > YAW_RATE_TEST_STICK_NEUTRAL_LIMIT))
    {
        motor_output_all_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > YAW_RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > YAW_RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_z_dps) > YAW_RATE_TEST_RATE_CUTOFF_DPS))
    {
        motor_output_all_stop();
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

    motor_us[0] = base_us - yaw_correction_us;
    motor_us[1] = base_us + yaw_correction_us;
    motor_us[2] = base_us - yaw_correction_us;
    motor_us[3] = base_us + yaw_correction_us;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        if (MOTOR_OUTPUT_STATUS_OK !=
            motor_output_set_us(motor_index,
                                yaw_rate_test_to_us(motor_us[motor_index])))
        {
            motor_output_all_stop();
            return;
        }
    }
#else
    (void) imu_healthy;
#endif
}
