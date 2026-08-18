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
    0.0f,
    0.0f,
    0.0f,
    FLIGHT_CONTROL_FAULT_NONE,
    false
};

#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
static flight_control_fault_reason_t g_flight_control_fault_reason =
    FLIGHT_CONTROL_FAULT_NONE;
#endif

/*
 * 桨载振动基线只给四路相同的受限脉宽，不调用 PID 或混控。
 * 该函数始终参与编译以便默认安全构建也能检查源码，但只有独立的
 * PROP_LOAD_TEST_MODE 才会从 IMU 任务调用它。
 */
#define PROP_LOAD_TEST_OUTPUT_RANGE_US         (180.0f)
#define PROP_LOAD_TEST_OUTPUT_MAX_US           (1180.0f)
#define PROP_LOAD_TEST_OUTPUT_RISE_US_PER_UPDATE (1.0f)
#define PROP_LOAD_TEST_OUTPUT_FALL_US_PER_UPDATE (2.0f)
#define PROP_LOAD_TEST_TILT_CUTOFF_DEG         (10.0f)
#define PROP_LOAD_TEST_RATE_CUTOFF_DPS         (100.0f)

static float g_prop_load_output_us = (float) MOTOR_OUTPUT_MIN_US;


static void flight_control_prop_load_stop(void)
{
    g_prop_load_output_us = (float) MOTOR_OUTPUT_MIN_US;
    motor_output_all_stop();
}


static uint32_t flight_control_prop_load_slew(float target_us)
{
    float delta_us;

    if (target_us < (float) MOTOR_OUTPUT_MIN_US)
    {
        target_us = (float) MOTOR_OUTPUT_MIN_US;
    }
    else if (target_us > PROP_LOAD_TEST_OUTPUT_MAX_US)
    {
        target_us = PROP_LOAD_TEST_OUTPUT_MAX_US;
    }

    delta_us = target_us - g_prop_load_output_us;

    if (delta_us > PROP_LOAD_TEST_OUTPUT_RISE_US_PER_UPDATE)
    {
        delta_us = PROP_LOAD_TEST_OUTPUT_RISE_US_PER_UPDATE;
    }
    else if (delta_us < -PROP_LOAD_TEST_OUTPUT_FALL_US_PER_UPDATE)
    {
        delta_us = -PROP_LOAD_TEST_OUTPUT_FALL_US_PER_UPDATE;
    }
    else
    {
        /* 目标已经在本周期允许的变化范围内。 */
    }

    g_prop_load_output_us += delta_us;

    return (uint32_t) (g_prop_load_output_us + 0.5f);
}


void flight_control_prop_load_vibration_update(bool imu_healthy)
{
    imu_attitude_t attitude;
    rc_command_t command;
    float throttle;
    float target_us;
    uint32_t output_us;
    uint32_t motor_index;

#if (PROP_LOAD_TEST_MODE != PROP_LOAD_TEST_MODE_VIBRATION_BASELINE)
    /* 即使以后误调用，本模式未启用时也只能写入停机脉宽。 */
    (void) imu_healthy;
    flight_control_prop_load_stop();
    return;
#endif

    if ((false == imu_healthy) ||
        (false == flight_safety_is_armed()))
    {
        flight_control_prop_load_stop();
        return;
    }

    rc_command_get(&command);

    if ((false == command.connected) ||
        (true == command.throttle_low))
    {
        flight_control_prop_load_stop();
        return;
    }

    imu_get_attitude(&attitude);

    if ((0 == isfinite(attitude.roll_deg)) ||
        (0 == isfinite(attitude.pitch_deg)) ||
        (0 == isfinite(attitude.gyro_x_dps)) ||
        (0 == isfinite(attitude.gyro_y_dps)) ||
        (0 == isfinite(attitude.gyro_z_dps)) ||
        (fabsf(attitude.roll_deg) > PROP_LOAD_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.pitch_deg) > PROP_LOAD_TEST_TILT_CUTOFF_DEG) ||
        (fabsf(attitude.gyro_x_dps) > PROP_LOAD_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_y_dps) > PROP_LOAD_TEST_RATE_CUTOFF_DPS) ||
        (fabsf(attitude.gyro_z_dps) > PROP_LOAD_TEST_RATE_CUTOFF_DPS))
    {
        flight_control_prop_load_stop();
        return;
    }

    throttle = command.throttle;

    if (throttle < 0.0f)
    {
        throttle = 0.0f;
    }
    else if (throttle > 1.0f)
    {
        throttle = 1.0f;
    }

    target_us = (float) MOTOR_OUTPUT_MIN_US +
                (throttle * PROP_LOAD_TEST_OUTPUT_RANGE_US);
    output_us = flight_control_prop_load_slew(target_us);

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        if (MOTOR_OUTPUT_STATUS_OK !=
            motor_output_set_us(motor_index, output_us))
        {
            flight_control_prop_load_stop();
            return;
        }
    }
}

#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW) || \
     (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP))
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
/*
 * 10 英寸本机首次系留短跳参数。只使用 P 项；慢斜坡仅作用于基础油门，
 * 姿态修正保持 500 Hz 实时响应。1550 us 是本轮基础油门硬上限。
 */
#define FLIGHT_TEST_BASE_MIN_US               (1150.0f)
#define FLIGHT_TEST_BASE_MAX_US               (1550.0f)
#define FLIGHT_TEST_TARGET_ANGLE_DEG          (3.0f)
#define FLIGHT_TEST_YAW_RATE_LIMIT_DPS        (10.0f)
#define FLIGHT_TEST_CORRECTION_LIMIT_US       (20.0f)
#define FLIGHT_TEST_OUTPUT_MIN_US             (1000.0f)
#define FLIGHT_TEST_OUTPUT_MAX_US             (1570.0f)
#define FLIGHT_TEST_TILT_CUTOFF_DEG           (10.0f)
#define FLIGHT_TEST_RATE_CUTOFF_DPS           (100.0f)
#define FLIGHT_TEST_RATE_IMMEDIATE_DPS        (300.0f)
#define TETHERED_RATE_CONFIRM_UPDATES          (10U) /* 500 Hz 下 20 ms。 */
#define TETHERED_RATE_IMMEDIATE_UPDATES        (2U)  /* 500 Hz 下 4 ms。 */
#define FLIGHT_TEST_ROLL_ANGLE_KP_DPS_PER_DEG (1.0f)
#define FLIGHT_TEST_PITCH_ANGLE_KP_DPS_PER_DEG (1.0f)
#define FLIGHT_TEST_ROLL_RATE_KP_US_PER_DPS   (0.30f)
#define FLIGHT_TEST_PITCH_RATE_KP_US_PER_DPS  (0.40f)
#define FLIGHT_TEST_YAW_RATE_KP_US_PER_DPS    (0.20f)
#define FLIGHT_TEST_ROLL_RATE_TARGET_LIMIT_DPS  (10.0f)
#define FLIGHT_TEST_PITCH_RATE_TARGET_LIMIT_DPS (20.0f)
#else
/* 历史三轴综合台架参数，仅用于拆桨验证，不作为飞行参数。 */
#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW))
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
#define FLIGHT_TEST_ROLL_ANGLE_KP_DPS_PER_DEG (3.0f)
#define FLIGHT_TEST_PITCH_ANGLE_KP_DPS_PER_DEG (3.0f)
#define FLIGHT_TEST_ROLL_RATE_KP_US_PER_DPS   (0.25f)
#define FLIGHT_TEST_PITCH_RATE_KP_US_PER_DPS  (0.25f)
#define FLIGHT_TEST_YAW_RATE_KP_US_PER_DPS    (0.25f)
#define FLIGHT_TEST_ROLL_RATE_TARGET_LIMIT_DPS  (45.0f)
#define FLIGHT_TEST_PITCH_RATE_TARGET_LIMIT_DPS (45.0f)
#define FLIGHT_TEST_OUTPUT_MIN_US             (1100.0f)
#endif
#define FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S    (0.0f)
#define FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG    (0.0f)
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW)
#define FLIGHT_TEST_RATE_KI_US_PER_DEG         (0.02f)
#define FLIGHT_TEST_RATE_INTEGRATOR_LIMIT_US   (2.0f)
#else
#define FLIGHT_TEST_RATE_KI_US_PER_DEG         (0.0f)
#define FLIGHT_TEST_RATE_INTEGRATOR_LIMIT_US   (0.0f)
#endif
#define FLIGHT_TEST_RATE_KD_US_S_PER_DPS       (0.0f)
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

#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
/* 500 Hz 下分别对应基础油门上升 200 us/s、普通下降 500 us/s。 */
#define TETHERED_BASE_RISE_US_PER_UPDATE       (0.4f)
#define TETHERED_BASE_FALL_US_PER_UPDATE       (1.0f)

static float g_tethered_base_us = (float) MOTOR_OUTPUT_MIN_US;
static bool g_tethered_fault_latched = false;
static uint32_t g_tethered_rate_count[3] = {0U, 0U, 0U};
static uint32_t g_tethered_rate_immediate_count[3] = {0U, 0U, 0U};
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
                             FLIGHT_TEST_ROLL_ANGLE_KP_DPS_PER_DEG,
                             FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S,
                             FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG,
                             0.0f,
                             FLIGHT_TEST_ROLL_RATE_TARGET_LIMIT_DPS,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_pitch_angle_controller,
                             FLIGHT_TEST_PITCH_ANGLE_KP_DPS_PER_DEG,
                             FLIGHT_TEST_ANGLE_KI_DPS_PER_DEG_S,
                             FLIGHT_TEST_ANGLE_KD_DPS_S_PER_DEG,
                             0.0f,
                             FLIGHT_TEST_PITCH_RATE_TARGET_LIMIT_DPS,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_roll_rate_controller,
                             FLIGHT_TEST_ROLL_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             FLIGHT_TEST_RATE_INTEGRATOR_LIMIT_US,
                             FLIGHT_TEST_CORRECTION_LIMIT_US,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_pitch_rate_controller,
                             FLIGHT_TEST_PITCH_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             FLIGHT_TEST_RATE_INTEGRATOR_LIMIT_US,
                             FLIGHT_TEST_CORRECTION_LIMIT_US,
                             FLIGHT_CONTROL_DERIVATIVE_ALPHA);
    pid_controller_configure(&g_yaw_rate_controller,
                             FLIGHT_TEST_YAW_RATE_KP_US_PER_DPS,
                             FLIGHT_TEST_RATE_KI_US_PER_DEG,
                             FLIGHT_TEST_RATE_KD_US_S_PER_DPS,
                             FLIGHT_TEST_RATE_INTEGRATOR_LIMIT_US,
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


#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
     (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP))
static uint32_t flight_test_to_us(float value)
{
    value = flight_test_clampf(value,
                               FLIGHT_TEST_OUTPUT_MIN_US,
                               FLIGHT_TEST_OUTPUT_MAX_US);

    return (uint32_t) (value + 0.5f);
}
#endif


#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
static void flight_control_reset_tethered_base(void)
{
    g_tethered_base_us = (float) MOTOR_OUTPUT_MIN_US;
    g_tethered_rate_count[0] = 0U;
    g_tethered_rate_count[1] = 0U;
    g_tethered_rate_count[2] = 0U;
    g_tethered_rate_immediate_count[0] = 0U;
    g_tethered_rate_immediate_count[1] = 0U;
    g_tethered_rate_immediate_count[2] = 0U;
}


static float flight_control_slew_tethered_base(float target_us)
{
    float delta_us;

    target_us = flight_test_clampf(target_us,
                                   FLIGHT_TEST_BASE_MIN_US,
                                   FLIGHT_TEST_BASE_MAX_US);
    delta_us = target_us - g_tethered_base_us;

    if (delta_us > TETHERED_BASE_RISE_US_PER_UPDATE)
    {
        delta_us = TETHERED_BASE_RISE_US_PER_UPDATE;
    }
    else if (delta_us < -TETHERED_BASE_FALL_US_PER_UPDATE)
    {
        delta_us = -TETHERED_BASE_FALL_US_PER_UPDATE;
    }
    else
    {
        /* 目标已经在本周期允许的变化范围内。 */
    }

    g_tethered_base_us += delta_us;
    return g_tethered_base_us;
}


/*
 * 单个桨载振动尖峰不再直接停机：100..300 dps 需持续 20 ms，
 * 超过 300 dps 也需连续两帧。恢复到 100 dps 内立即清零该轴计数。
 */
static bool flight_control_tethered_rate_fault(float rate_dps,
                                                uint32_t axis_index)
{
    float absolute_rate_dps = fabsf(rate_dps);

    if (absolute_rate_dps <= FLIGHT_TEST_RATE_CUTOFF_DPS)
    {
        g_tethered_rate_count[axis_index] = 0U;
        g_tethered_rate_immediate_count[axis_index] = 0U;
        return false;
    }

    if (g_tethered_rate_count[axis_index] <
        TETHERED_RATE_CONFIRM_UPDATES)
    {
        g_tethered_rate_count[axis_index]++;
    }

    if (absolute_rate_dps > FLIGHT_TEST_RATE_IMMEDIATE_DPS)
    {
        if (g_tethered_rate_immediate_count[axis_index] <
            TETHERED_RATE_IMMEDIATE_UPDATES)
        {
            g_tethered_rate_immediate_count[axis_index]++;
        }
    }
    else
    {
        g_tethered_rate_immediate_count[axis_index] = 0U;
    }

    return ((g_tethered_rate_count[axis_index] >=
             TETHERED_RATE_CONFIRM_UPDATES) ||
            (g_tethered_rate_immediate_count[axis_index] >=
             TETHERED_RATE_IMMEDIATE_UPDATES));
}


static flight_control_fault_reason_t flight_control_tethered_fault_reason(
    const imu_attitude_t * p_attitude)
{
    bool roll_rate_fault;
    bool pitch_rate_fault;
    bool yaw_rate_fault;

    if ((0 == isfinite(p_attitude->roll_deg)) ||
        (0 == isfinite(p_attitude->pitch_deg)) ||
        (0 == isfinite(p_attitude->gyro_x_dps)) ||
        (0 == isfinite(p_attitude->gyro_y_dps)) ||
        (0 == isfinite(p_attitude->gyro_z_dps)))
    {
        return FLIGHT_CONTROL_FAULT_NONFINITE;
    }

    if (fabsf(p_attitude->roll_deg) > FLIGHT_TEST_TILT_CUTOFF_DEG)
    {
        return FLIGHT_CONTROL_FAULT_ROLL_TILT;
    }

    if (fabsf(p_attitude->pitch_deg) > FLIGHT_TEST_TILT_CUTOFF_DEG)
    {
        return FLIGHT_CONTROL_FAULT_PITCH_TILT;
    }

    roll_rate_fault = flight_control_tethered_rate_fault(
        p_attitude->gyro_x_dps,
        0U);
    pitch_rate_fault = flight_control_tethered_rate_fault(
        p_attitude->gyro_y_dps,
        1U);
    yaw_rate_fault = flight_control_tethered_rate_fault(
        p_attitude->gyro_z_dps,
        2U);

    if (true == roll_rate_fault)
    {
        return FLIGHT_CONTROL_FAULT_ROLL_RATE;
    }

    if (true == pitch_rate_fault)
    {
        return FLIGHT_CONTROL_FAULT_PITCH_RATE;
    }

    if (true == yaw_rate_fault)
    {
        return FLIGHT_CONTROL_FAULT_YAW_RATE;
    }

    return FLIGHT_CONTROL_FAULT_NONE;
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
    g_flight_control_status.roll_integrator_us = 0.0f;
    g_flight_control_status.pitch_integrator_us = 0.0f;
    g_flight_control_status.yaw_integrator_us = 0.0f;
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    g_flight_control_status.fault_reason = g_flight_control_fault_reason;
#else
    g_flight_control_status.fault_reason = FLIGHT_CONTROL_FAULT_NONE;
#endif
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
    g_flight_control_status.roll_integrator_us =
        g_roll_rate_controller.integrator;
    g_flight_control_status.pitch_integrator_us =
        g_pitch_rate_controller.integrator;
    g_flight_control_status.yaw_integrator_us =
        g_yaw_rate_controller.integrator;
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    g_flight_control_status.fault_reason = g_flight_control_fault_reason;
#else
    g_flight_control_status.fault_reason = FLIGHT_CONTROL_FAULT_NONE;
#endif
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
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW) || \
     (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP))
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
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    flight_control_fault_reason_t fault_reason;
#endif

    flight_control_configure_controllers();

    if ((false == imu_healthy) ||
        (false == flight_safety_is_armed()))
    {
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        flight_control_reset_tethered_base();
        /* 只有CH5撤防/失联后才允许清除上一次保护锁存。 */
        g_tethered_fault_latched = false;
        g_flight_control_fault_reason = FLIGHT_CONTROL_FAULT_NONE;
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    if (true == g_tethered_fault_latched)
    {
        motor_output_all_stop();
        flight_control_reset_tethered_base();
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }
#endif

    rc_command_get(&command);

    if ((false == command.connected) ||
        (true == command.throttle_low))
    {
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        flight_control_reset_tethered_base();
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

    imu_get_attitude(&attitude);

#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    fault_reason = flight_control_tethered_fault_reason(&attitude);

    if (FLIGHT_CONTROL_FAULT_NONE != fault_reason)
#else
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
#endif
    {
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        /* 倾角/角速度保护触发后禁止在CH5仍为高档时自动重新启动。 */
        g_tethered_fault_latched = true;
        g_flight_control_fault_reason = fault_reason;
#endif
        motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
        flight_test_reset_powered_output();
#endif
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        flight_control_reset_tethered_base();
#endif
        flight_control_reset_controllers();
        flight_control_publish_stopped();
        return;
    }

    base_us = FLIGHT_TEST_BASE_MIN_US +
              (command.throttle *
               (FLIGHT_TEST_BASE_MAX_US - FLIGHT_TEST_BASE_MIN_US));
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    /* 只缓升基础油门，不能延迟角速度闭环的差动修正。 */
    base_us = flight_control_slew_tethered_base(base_us);
#endif
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

#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW))
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
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
            g_tethered_fault_latched = true;
            g_flight_control_fault_reason =
                FLIGHT_CONTROL_FAULT_MOTOR_OUTPUT;
#endif
            motor_output_all_stop();
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL)
            flight_test_reset_powered_output();
#endif
#if (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
            flight_control_reset_tethered_base();
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
