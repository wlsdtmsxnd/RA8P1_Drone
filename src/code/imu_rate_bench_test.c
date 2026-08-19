#include "imu_rate_bench_test.h"

#include "actuator_manager.h"
#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "quad_x_mixer.h"
#include "rc_command.h"

#include <math.h>
#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_RATE)
/* 拆桨角速度阻尼测试限幅，不作为飞行参数。 */
#define RATE_TEST_BASE_MIN_US            (1150.0f)
#define RATE_TEST_BASE_MAX_US            (1250.0f)
#define RATE_TEST_KP_US_PER_DPS          (0.25f)
#define RATE_TEST_CORRECTION_LIMIT_US    (20.0f)
#define RATE_TEST_OUTPUT_MIN_US          (1100.0f)
#define RATE_TEST_OUTPUT_MAX_US          (1300.0f)
#define RATE_TEST_TILT_CUTOFF_DEG        (20.0f)
#define RATE_TEST_RATE_CUTOFF_DPS        (150.0f)
#define RATE_TEST_STICK_NEUTRAL_LIMIT    (0.12f)


static float rate_test_clampf(float value, float minimum, float maximum)
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


static uint32_t rate_test_to_us(float value)
{
    value = rate_test_clampf(value,
                             RATE_TEST_OUTPUT_MIN_US,
                             RATE_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void imu_rate_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_RATE)
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
    float roll_correction_us;
    float pitch_correction_us;
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
        (fabsf(command.roll) > RATE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.pitch) > RATE_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.yaw) > RATE_TEST_STICK_NEUTRAL_LIMIT))
    {
        (void) actuator_manager_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > RATE_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > RATE_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_z_dps) > RATE_TEST_RATE_CUTOFF_DPS))
    {
        (void) actuator_manager_stop();
        return;
    }

    base_us = RATE_TEST_BASE_MIN_US +
              (command.throttle *
               (RATE_TEST_BASE_MAX_US - RATE_TEST_BASE_MIN_US));

    roll_correction_us = rate_test_clampf(
        -attitude.gyro_x_dps * RATE_TEST_KP_US_PER_DPS,
        -RATE_TEST_CORRECTION_LIMIT_US,
        RATE_TEST_CORRECTION_LIMIT_US);
    pitch_correction_us = rate_test_clampf(
        -attitude.gyro_y_dps * RATE_TEST_KP_US_PER_DPS,
        -RATE_TEST_CORRECTION_LIMIT_US,
        RATE_TEST_CORRECTION_LIMIT_US);
    yaw_correction_us = rate_test_clampf(
        -attitude.gyro_z_dps * RATE_TEST_KP_US_PER_DPS,
        -RATE_TEST_CORRECTION_LIMIT_US,
        RATE_TEST_CORRECTION_LIMIT_US);

    quad_x_mixer_apply(base_us,
                       roll_correction_us,
                       pitch_correction_us,
                       yaw_correction_us,
                       motor_us);

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        actuator_us[motor_index] = rate_test_to_us(motor_us[motor_index]);
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
