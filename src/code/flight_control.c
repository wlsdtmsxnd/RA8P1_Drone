#include "flight_control.h"

#include "flight_safety.h"
#include "imu.h"
#include "pid_controller.h"
#include "project_config.h"
#include "rc_command.h"
#include "../driver/motor_output.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#if (FLIGHT_CONTROL_MOTOR_COUNT != MOTOR_OUTPUT_COUNT)
#error "Flight control and motor output counts must match"
#endif

static flight_control_status_t g_flight_control_status =
{
    {1000.0f, 1000.0f, 1000.0f, 1000.0f},
    1000.0f,
    0.0f,
    0.0f,
    0.0f,
    false
};

#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL))
/* 首次三轴综合台架参数，仅用于拆桨验证，不作为飞行参数。 */
#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL))
#define FLIGHT_TEST_BASE_MIN_US               (1150.0f)
#define FLIGHT_TEST_BASE_MAX_US               (1220.0f)
#define FLIGHT_TEST_TARGET_ANGLE_DEG          (8.0f)
#define FLIGHT_TEST_YAW_RATE_LIMIT_DPS        (24.0f)
#define FLIGHT_TEST_CORRECTION_LIMIT_US       (12.0f)
#define FLIGHT_TEST_OUTPUT_MAX_US             (1280.0f)
#define FLIGHT_TEST_TILT_CUTOFF_DEG           (15.0f)
#define FLIGHT_TEST_RATE_CUTOFF_DPS           (100.0f)
#else
#define FLIGHT_TEST_BASE_MIN_US               (1150.0f)
#define FLIGHT_TEST_BASE_MAX_US               (1250.0f)
#define FLIGHT_TEST_TARGET_ANGLE_DEG          (10.0f)
#define FLIGHT_TEST_YAW_RATE_LIMIT_DPS        (30.0f)
#define FLIGHT_TEST_CORRECTION_LIMIT_US       (20.0f)
#define FLIGHT_TEST_OUTPUT_MAX_US             (1300.0f)
#define FLIGHT_TEST_TILT_CUTOFF_DEG           (20.0f)
#define FLIGHT_TEST_RATE_CUTOFF_DPS           (150.0f)
#endif
#define FLIGHT_TEST_ANGLE_KP_DPS_PER_DEG      (3.0f)
#define FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S    (0.0f)
#define FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG    (0.0f)
#define FLIGHT_TEST_RATE_KP_US_PER_DPS        (0.25f)
#define FLIGHT_TEST_RATE_KI_US_PER_DEG         (0.0f)
#define FLIGHT_TEST_RATE_KD_US_S_PER_DPS       (0.0f)
#define FLIGHT_TEST_RATE_TARGET_LIMIT_DPS     (45.0f)
#define FLIGHT_TEST_OUTPUT_MIN_US             (1100.0f)
#define FLIGHT_CONTROL_PERIOD_S                (0.002f)
#define FLIGHT_CONTROL_DERIVATIVE_ALPHA        (0.20f)

static pid_controller_t g_roll_angle_controller;
static pid_controller_t g_pitch_angle_controller;
static pid_controller_t g_roll_rate_controller;
static pid_controller_t g_pitch_rate_controller;
static pid_controller_t g_yaw_rate_controller;
static bool g_controllers_configured = false;

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
#define FLIGHT_TEST_OUTPUT_RISE_US_PER_UPDATE (1.0f)
#define FLIGHT_TEST_OUTPUT_FALL_US_PER_UPDATE (2.0f)

static float g_powered_output_us[MOTOR_OUTPUT_COUNT] =
{
    1000.0f,
    1000.0f,
    1000.0f,
    1000.0f
};
#endif


static float flight_test_clampf(float value,
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


static void flight_control_configure_controllers(void)
{
    if (true == g_controllers_configured)
    {
        return;
    }

    pid_controller_configure(&g_roll_angle_controller,
                             FLIGHT_TEST_ANGLE_KP_DPS_PER_DEG,
                             FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S,
                             FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG,
                             0.0f,
                             FLIGHT_TEST_RATE_TARGET_LIMIT_DPS,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_pitch_angle_controller,
                             FLIGHT_TEST_ANGLE_KP_DPS_PER_DEG,
                             FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S,
                             FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG,
                             0.0f,
                             FLIGHT_TEST_RATE_TARGET_LIMIT_DPS,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_roll_rate_controller,
                             FLIGHT_TEST_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             0.0f,
                             FLIGHT_TEST_CORRECTION_LIMIT_US,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_pitch_rate_controller,
                             FLIGHT_TEST_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             0.0f,
                             FLIGHT_TEST_CORRECTION_LIMIT_US,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_yaw_rate_controller,
                             FLIGHT_TEST_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             0.0f,
                             FLIGHT_TEST_CORRECTION_LIMIT_US,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    g_controllers_configured = true;
}


static void flight_control_reset_controllers(void)
{
    pid_controller_reset(&g_roll_angle_controller);
    pid_controller_reset(&g_pitch_angle_controller);
    pid_controller_reset(&g_roll_rate_controller);
    pid_controller_reset(&g_pitch_rate_controller);
    pid_controller_reset(&g_yaw_rate_controller);
}


#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL)
static uint32_t flight_test_to_us(float value)
{
    value = flight_test_clampf(value,
                               FLIGHT_TEST_OUTPUT_MIN_US,
                               FLIGHT_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


static void flight_control_publish_stopped(void)
{
    uint32_t motor_index;

    taskENTER_CRITICAL();
    for (motor_index = 0U;
         motor_index < FLIGHT_CONTROL_MOTOR_COUNT;
         motor_index++)
    {
        g_flight_control_status.motor_us[motor_index] = 1000.0f;
    }
    g_flight_control_status.base_us = 1000.0f;
    g_flight_control_status.roll_correction_us = 0.0f;
    g_flight_control_status.pitch_correction_us = 0.0f;
    g_flight_control_status.yaw_correction_us = 0.0f;
    g_flight_control_status.valid = false;
    taskEXIT_CRITICAL();
}


static void flight_control_publish_active(
    const float motor_us[FLIGHT_CONTROL_MOTOR_COUNT],
    float base_us,
    float roll_correction_us,
    float pitch_correction_us,
    float yaw_correction_us)
{
    uint32_t motor_index;

    taskENTER_CRITICAL();
    for (motor_index = 0U;
         motor_index < FLIGHT_CONTROL_MOTOR_COUNT;
         motor_index++)
    {
        g_flight_control_status.motor_us[motor_index] = motor_us[motor_index];
    }
    g_flight_control_status.base_us = base_us;
    g_flight_control_status.roll_correction_us = roll_correction_us;
    g_flight_control_status.pitch_correction_us = pitch_correction_us;
    g_flight_control_status.yaw_correction_us = yaw_correction_us;
    g_flight_control_status.valid = true;
    taskEXIT_CRITICAL();
}


#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
static void flight_test_reset_powered_output(void)
{
    uint32_t motor_index;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        g_powered_output_us[motor_index] = 1000.0f;
    }
}


static uint32_t flight_test_slew_to_us(uint32_t motor_index,
                                       float target_us)
{
    float delta_us;

    target_us = flight_test_clampf(target_us,
                                   FLIGHT_TEST_OUTPUT_MIN_US,
                                   FLIGHT_TEST_OUTPUT_MAX_US);
    delta_us = target_us - g_powered_output_us[motor_index];

    if (delta_us > FLIGHT_TEST_OUTPUT_RISE_US_PER_UPDATE)
    {
        delta_us = FLIGHT_TEST_OUTPUT_RISE_US_PER_UPDATE;
    }
    else if (delta_us < -FLIGHT_TEST_OUTPUT_FALL_US_PER_UPDATE)
    {
        delta_us = -FLIGHT_TEST_OUTPUT_FALL_US_PER_UPDATE;
    }
    else
    {
        /* 目标已经在本周期允许的变化范围内。 */
    }

    g_powered_output_us[motor_index] += delta_us;

    return (uint32_t) (g_powered_output_us[motor_index] + 0.5f);
}
#endif
#endif


void flight_control_update(bool imu_healthy)
{
#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL))
    imu_attitude_t attitude;
    rc_command_t command;
    float base_us;
    float target_roll_deg;
    float target_pitch_deg;
    float roll_target_rate_dps;
    float pitch_target_rate_dps;
    float yaw_target_rate_dps;
    float roll_correction_us;
    float pitch_correction_us;
    float yaw_correction_us;
    float motor_us[MOTOR_OUTPUT_COUNT];
    uint32_t motor_index;

    flight_control_configure_controllers();

    if ((false == imu_healthy) ||
        (false == flight_safety_is_armed()))
    {
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

    rc_command_get(&command);

    if ((false == command.connected) ||
        (true == command.throttle_low))
    {
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

    imu_get_attitude(&attitude);

    if ((0 == isfinite(attitude.roll_deg)) ||
        (0 == isfinite(attitude.pitch_deg)) ||
        (0 == isfinite(attitude.gyro_x_dps)) ||
        (0 == isfinite(attitude.gyro_y_dps)) ||
        (0 == isfinite(attitude.gyro_z_dps)) ||
        (fabsf(attitude.roll_deg) > FLIGHT_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > FLIGHT_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > FLIGHT_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > FLIGHT_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_z_dps) > FLIGHT_TEST_RATE_CUTOFF_DPS))
    {
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

    base_us = FLIGHT_TEST_BASE_MIN_US +
              (command.throttle *
               (FLIGHT_TEST_BASE_MAX_US - FLIGHT_TEST_BASE_MIN_US));
    target_roll_deg = flight_test_clampf(command.roll, -1.0f, 1.0f) *
                      FLIGHT_TEST_TARGET_ANGLE_DEG;
    target_pitch_deg = flight_test_clampf(command.pitch, -1.0f, 1.0f) *
                       FLIGHT_TEST_TARGET_ANGLE_DEG;

    roll_target_rate_dps = pid_controller_update(
        &g_roll_angle_controller,
        target_roll_deg,
        attitude.roll_deg,
        FLIGHT_CONTROL_PERIOD_S);
    pitch_target_rate_dps = pid_controller_update(
        &g_pitch_angle_controller,
        target_pitch_deg,
        attitude.pitch_deg,
        FLIGHT_CONTROL_PERIOD_S);
    yaw_target_rate_dps = flight_test_clampf(command.yaw, -1.0f, 1.0f) *
                          FLIGHT_TEST_YAW_RATE_LIMIT_DPS;

    roll_correction_us = pid_controller_update(
        &g_roll_rate_controller,
        roll_target_rate_dps,
        attitude.gyro_x_dps,
        FLIGHT_CONTROL_PERIOD_S);
    pitch_correction_us = pid_controller_update(
        &g_pitch_rate_controller,
        pitch_target_rate_dps,
        attitude.gyro_y_dps,
        FLIGHT_CONTROL_PERIOD_S);
    yaw_correction_us = pid_controller_update(
        &g_yaw_rate_controller,
        yaw_target_rate_dps,
        attitude.gyro_z_dps,
        FLIGHT_CONTROL_PERIOD_S);

    motor_us[0] = base_us + pitch_correction_us +
                  roll_correction_us - yaw_correction_us;
    motor_us[1] = base_us + pitch_correction_us -
                  roll_correction_us + yaw_correction_us;
    motor_us[2] = base_us - pitch_correction_us -
                  roll_correction_us - yaw_correction_us;
    motor_us[3] = base_us - pitch_correction_us +
                  roll_correction_us + yaw_correction_us;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        motor_us[motor_index] = flight_test_clampf(
            motor_us[motor_index],
            FLIGHT_TEST_OUTPUT_MIN_US,
            FLIGHT_TEST_OUTPUT_MAX_US);
    }

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL)
    /* 影子模式只观察控制计算，实际四路 GPT 始终保持停机脉宽。 */
    motor_output_all_stop();
    flight_control_publish_active(motor_us,
                                  base_us,
                                  roll_correction_us,
                                  pitch_correction_us,
                                  yaw_correction_us);
#else
    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        if (MOTOR_OUTPUT_STATUS_OK !=
            motor_output_set_us(motor_index,
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
                                flight_test_slew_to_us(motor_index,
                                                       motor_us[motor_index])))
#else
                                flight_test_to_us(motor_us[motor_index])))
#endif
        {
            motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
            flight_test_reset_powered_output();
#endif
            flight_control_reset_controllers();
            flight_control_publish_stopped();
            return;
        }
    }
    flight_control_publish_active(motor_us,
                                  base_us,
                                  roll_correction_us,
                                  pitch_correction_us,
                                  yaw_correction_us);
#endif
#else
    (void) imu_healthy;
#endif
}


void flight_control_get_status(flight_control_status_t * p_status)
{
    if (NULL == p_status)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_status = g_flight_control_status;
    taskEXIT_CRITICAL();
}
