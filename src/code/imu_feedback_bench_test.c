#include "imu_feedback_bench_test.h"

#include "actuator_manager.h"
#include "flight_safety.h"
#include "imu.h"
#include "project_config.h"
#include "quad_x_mixer.h"
#include "rc_command.h"

#include <math.h>
#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_LEVEL)
/* 拆桨角度反馈测试限幅，不作为飞行参数。 */
#define IMU_TEST_BASE_MIN_US             (1150.0f)
#define IMU_TEST_BASE_MAX_US             (1250.0f)
#define IMU_TEST_ANGLE_KP_US_PER_DEG     (1.5f)
#define IMU_TEST_CORRECTION_LIMIT_US     (20.0f)
#define IMU_TEST_OUTPUT_MIN_US           (1100.0f)
#define IMU_TEST_OUTPUT_MAX_US           (1300.0f)
#define IMU_TEST_TILT_CUTOFF_DEG         (20.0f)
#define IMU_TEST_STICK_NEUTRAL_LIMIT     (0.12f)


static float imu_test_clampf(float value, float minimum, float maximum)
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


static uint32_t imu_test_to_us(float value)
{
    value = imu_test_clampf(value,
                            IMU_TEST_OUTPUT_MIN_US,
                            IMU_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void imu_feedback_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_LEVEL)
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
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
        (fabsf(command.roll) > IMU_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.pitch) > IMU_TEST_STICK_NEUTRAL_LIMIT) ||
        (fabsf(command.yaw) > IMU_TEST_STICK_NEUTRAL_LIMIT))
    {
        (void) actuator_manager_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((fabsf(attitude.roll_deg) > IMU_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > IMU_TEST_TILT_CUTOFF_DEG))
    {
        (void) actuator_manager_stop();
        return;
    }

    base_us = IMU_TEST_BASE_MIN_US +
              (command.throttle *
               (IMU_TEST_BASE_MAX_US - IMU_TEST_BASE_MIN_US));

    /* 水平目标为 0°；正修正量产生正 Roll/Pitch 力矩。 */
    roll_correction_us = imu_test_clampf(
        -attitude.roll_deg * IMU_TEST_ANGLE_KP_US_PER_DEG,
        -IMU_TEST_CORRECTION_LIMIT_US,
        IMU_TEST_CORRECTION_LIMIT_US);
    pitch_correction_us = imu_test_clampf(
        -attitude.pitch_deg * IMU_TEST_ANGLE_KP_US_PER_DEG,
        -IMU_TEST_CORRECTION_LIMIT_US,
        IMU_TEST_CORRECTION_LIMIT_US);

    quad_x_mixer_apply(base_us,
                       roll_correction_us,
                       pitch_correction_us,
                       0.0f,
                       motor_us);

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        actuator_us[motor_index] = imu_test_to_us(motor_us[motor_index]);
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
