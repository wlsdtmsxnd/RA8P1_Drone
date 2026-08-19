#include "mixer_bench_test.h"

#include "actuator_manager.h"
#include "flight_safety.h"
#include "project_config.h"
#include "quad_x_mixer.h"
#include "rc_command.h"

#include <stdint.h>

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_STICK_MIXER)
/* 拆桨波形测试限幅，不作为飞行参数。 */
#define MIXER_TEST_BASE_MIN_US       (1150.0f)
#define MIXER_TEST_BASE_MAX_US       (1250.0f)
#define MIXER_TEST_AXIS_DELTA_US     (30.0f)
#define MIXER_TEST_OUTPUT_MIN_US     (1100.0f)
#define MIXER_TEST_OUTPUT_MAX_US     (1300.0f)


static float mixer_test_clampf(float value, float minimum, float maximum)
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


static uint32_t mixer_test_to_us(float value)
{
    value = mixer_test_clampf(value,
                              MIXER_TEST_OUTPUT_MIN_US,
                              MIXER_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


void mixer_bench_test_update(bool imu_healthy)
{
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_STICK_MIXER)
    rc_command_t command;
    float base_us;
    float roll_us;
    float pitch_us;
    float yaw_us;
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
        (true == command.throttle_low))
    {
        (void) actuator_manager_stop();
        return;
    }

    base_us = MIXER_TEST_BASE_MIN_US +
              (command.throttle *
               (MIXER_TEST_BASE_MAX_US - MIXER_TEST_BASE_MIN_US));
    roll_us = command.roll * MIXER_TEST_AXIS_DELTA_US;
    pitch_us = command.pitch * MIXER_TEST_AXIS_DELTA_US;
    yaw_us = command.yaw * MIXER_TEST_AXIS_DELTA_US;

    quad_x_mixer_apply(base_us, roll_us, pitch_us, yaw_us, motor_us);

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        actuator_us[motor_index] = mixer_test_to_us(motor_us[motor_index]);
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
