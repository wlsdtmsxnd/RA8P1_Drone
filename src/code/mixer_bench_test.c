#include "mixer_bench_test.h"

#include "flight_safety.h"
#include "project_config.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

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
        (true == command.throttle_low))
    {
        motor_output_all_stop();
        return;
    }

    base_us = MIXER_TEST_BASE_MIN_US +
              (command.throttle *
               (MIXER_TEST_BASE_MAX_US - MIXER_TEST_BASE_MIN_US));
    roll_us = command.roll * MIXER_TEST_AXIS_DELTA_US;
    pitch_us = command.pitch * MIXER_TEST_AXIS_DELTA_US;
    yaw_us = command.yaw * MIXER_TEST_AXIS_DELTA_US;

    /*
     * FRD / Quad-X：M1 左前，M2 右前，M3 右后，M4 左后。
     * 目标旋向：M1/M3 CW，M2/M4 CCW（从上方看）。
     */
    motor_us[0] = base_us + pitch_us + roll_us - yaw_us;
    motor_us[1] = base_us + pitch_us - roll_us + yaw_us;
    motor_us[2] = base_us - pitch_us - roll_us - yaw_us;
    motor_us[3] = base_us - pitch_us + roll_us + yaw_us;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        if (MOTOR_OUTPUT_STATUS_OK !=
            motor_output_set_us(motor_index,
                                mixer_test_to_us(motor_us[motor_index])))
        {
            motor_output_all_stop();
            return;
        }
    }
#else
    (void) imu_healthy;
#endif
}
