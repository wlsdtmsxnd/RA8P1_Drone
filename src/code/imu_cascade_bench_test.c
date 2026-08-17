#include "imu_cascade_bench_test.h"

#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

#include <math.h>
#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_CASCADE)
/* 拆桨串级自稳测试限幅，不作为飞行参数。 */
#define CASCADE_TEST_BASE_MIN_US              (1150.0f)
#define CASCADE_TEST_BASE_MAX_US              (1250.0f)
#define CASCADE_TEST_ANGLE_KP_DPS_PER_DEG     (3.0f)
#define CASCADE_TEST_RATE_KP_US_PER_DPS       (0.25f)
#define CASCADE_TEST_RATE_TARGET_LIMIT_DPS    (45.0f)
#define CASCADE_TEST_CORRECTION_LIMIT_US      (20.0f)
#define CASCADE_TEST_OUTPUT_MIN_US            (1100.0f)
#define CASCADE_TEST_OUTPUT_MAX_US            (1300.0f)
#define CASCADE_TEST_TILT_CUTOFF_DEG          (20.0f)
#define CASCADE_TEST_RATE_CUTOFF_DPS          (150.0f)
#define CASCADE_TEST_STICK_NEUTRAL_LIMIT      (0.12f)


static float cascade_test_clampf(float value,
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


static uint32_t cascade_test_to_us(float value)
{
    value = cascade_test_clampf(value,
                                CASCADE_TEST_OUTPUT_MIN_US,
                                CASCADE_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void imu_cascade_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_CASCADE)
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
    float roll_target_rate_dps;
    float pitch_target_rate_dps;
    float roll_correction_us;
    float pitch_correction_us;
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
        (fabsf(command.roll) > CASCADE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.pitch) > CASCADE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.yaw) > CASCADE_TEST_STICK_NEUTRAL_LIMIT))
    {
        motor_output_all_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > CASCADE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > CASCADE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > CASCADE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > CASCADE_TEST_RATE_CUTOFF_DPS))
    {
        motor_output_all_stop();
        return;
    }

    base_us = CASCADE_TEST_BASE_MIN_US +
              (command.throttle *
               (CASCADE_TEST_BASE_MAX_US - CASCADE_TEST_BASE_MIN_US));

    /* 外环以水平为目标，只输出有限的 Roll/Pitch 目标角速度。 */
    roll_target_rate_dps = cascade_test_clampf(
        -attitude.roll_deg * CASCADE_TEST_ANGLE_KP_DPS_PER_DEG,
        -CASCADE_TEST_RATE_TARGET_LIMIT_DPS,
        CASCADE_TEST_RATE_TARGET_LIMIT_DPS);
    pitch_target_rate_dps = cascade_test_clampf(
        -attitude.pitch_deg * CASCADE_TEST_ANGLE_KP_DPS_PER_DEG,
        -CASCADE_TEST_RATE_TARGET_LIMIT_DPS,
        CASCADE_TEST_RATE_TARGET_LIMIT_DPS);

    /* 内环 P 控制角速度误差；本阶段不使用积分、微分或 Yaw 控制。 */
    roll_correction_us = cascade_test_clampf(
        (roll_target_rate_dps - attitude.gyro_x_dps) *
        CASCADE_TEST_RATE_KP_US_PER_DPS,
        -CASCADE_TEST_CORRECTION_LIMIT_US,
        CASCADE_TEST_CORRECTION_LIMIT_US);
    pitch_correction_us = cascade_test_clampf(
        (pitch_target_rate_dps - attitude.gyro_y_dps) *
        CASCADE_TEST_RATE_KP_US_PER_DPS,
        -CASCADE_TEST_CORRECTION_LIMIT_US,
        CASCADE_TEST_CORRECTION_LIMIT_US);

    motor_us[0] = base_us + pitch_correction_us + roll_correction_us;
    motor_us[1] = base_us + pitch_correction_us - roll_correction_us;
    motor_us[2] = base_us - pitch_correction_us - roll_correction_us;
    motor_us[3] = base_us - pitch_correction_us + roll_correction_us;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        if (MOTOR_OUTPUT_STATUS_OK !=
            motor_output_set_us(motor_index,
                                cascade_test_to_us(motor_us[motor_index])))
        {
            motor_output_all_stop();
            return;
        }
    }
#else
    (void) imu_healthy;
#endif
}
