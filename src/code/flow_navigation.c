#include "flow_navigation.h"

#include "project_config.h"
#include "../driver/up_tof.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>

#define FLOW_NAV_TOF_MIN_MM                     (200.0f)
#define FLOW_NAV_TOF_MAX_MM                   (20000.0f)
#define FLOW_NAV_MINIMUM_TILT_COSINE              (0.819152f)
#define FLOW_NAV_TOF_JUMP_LIMIT_MM              (200.0f)
#define FLOW_NAV_TOF_REACCEPT_COUNT                (5U)
#define FLOW_NAV_VELOCITY_FILTER_ALPHA             (0.15f)
#define FLOW_NAV_GYRO_COMPENSATION_GAIN            (0.05f)
#define FLOW_NAV_HEIGHT_TIMEOUT_S                   (0.20f)
#define FLOW_NAV_FLOW_TIMEOUT_S                     (0.12f)
#define FLOW_NAV_VERTICAL_ACCEL_DEADBAND_MM_S2     (40.0f)
#define FLOW_NAV_VERTICAL_ACCEL_LIMIT_MM_S2      (5000.0f)
#define FLOW_NAV_VERTICAL_VELOCITY_DAMPING          (0.999f)
#define FLOW_NAV_HEIGHT_POSITION_CORRECTION         (0.25f)
#define FLOW_NAV_HEIGHT_VELOCITY_CORRECTION         (0.04f)

static flow_navigation_core_t g_flow_navigation_core;
static flow_navigation_state_t g_flow_navigation_state;
static bool g_flow_navigation_initialized = false;


static void flow_navigation_initialize(void)
{
    flow_navigation_config_t config =
    {
        .tof_min_mm = FLOW_NAV_TOF_MIN_MM,
        .tof_max_mm = FLOW_NAV_TOF_MAX_MM,
        .minimum_tilt_cosine = FLOW_NAV_MINIMUM_TILT_COSINE,
        .tof_jump_limit_mm = FLOW_NAV_TOF_JUMP_LIMIT_MM,
        .tof_reaccept_count = FLOW_NAV_TOF_REACCEPT_COUNT,
        .velocity_filter_alpha = FLOW_NAV_VELOCITY_FILTER_ALPHA,
        .gyro_compensation_gain = FLOW_NAV_GYRO_COMPENSATION_GAIN,
        .sensor_x_sign = FLOW_NAV_SENSOR_X_SIGN,
        .sensor_y_sign = FLOW_NAV_SENSOR_Y_SIGN,
        .swap_sensor_axes = (FLOW_NAV_SENSOR_SWAP_XY == 1U),
        .height_timeout_s = FLOW_NAV_HEIGHT_TIMEOUT_S,
        .flow_timeout_s = FLOW_NAV_FLOW_TIMEOUT_S,
        .vertical_accel_deadband_mm_s2 =
            FLOW_NAV_VERTICAL_ACCEL_DEADBAND_MM_S2,
        .vertical_accel_limit_mm_s2 =
            FLOW_NAV_VERTICAL_ACCEL_LIMIT_MM_S2,
        .vertical_velocity_damping =
            FLOW_NAV_VERTICAL_VELOCITY_DAMPING,
        .height_position_correction =
            FLOW_NAV_HEIGHT_POSITION_CORRECTION,
        .height_velocity_correction =
            FLOW_NAV_HEIGHT_VELOCITY_CORRECTION
    };

    flow_navigation_core_init(&g_flow_navigation_core, &config);
    g_flow_navigation_state = (flow_navigation_state_t) {0};
    g_flow_navigation_initialized = true;
}


void flow_navigation_update(const imu_attitude_t * p_attitude,
                            bool imu_healthy,
                            float dt_s)
{
    up_tof_data_t sensor_data;
    flow_navigation_input_t input = {0};

    if (false == g_flow_navigation_initialized)
    {
        flow_navigation_initialize();
    }

    up_tof_get_data(&sensor_data);

    if (NULL != p_attitude)
    {
        input.roll_deg = p_attitude->roll_deg;
        input.pitch_deg = p_attitude->pitch_deg;
        input.yaw_deg = p_attitude->yaw_deg;
        input.gyro_x_dps = p_attitude->gyro_x_dps;
        input.gyro_y_dps = p_attitude->gyro_y_dps;
        input.vertical_accel_mm_s2 =
            p_attitude->vertical_accel_mm_s2;
    }

    input.flow_x_integral = sensor_data.flow_x_integral;
    input.flow_y_integral = sensor_data.flow_y_integral;
    input.velocity_x_cm_s = sensor_data.velocity_x_cm_s;
    input.velocity_y_cm_s = sensor_data.velocity_y_cm_s;
    input.integration_us = sensor_data.integration_us;
    input.distance_mm = sensor_data.distance_mm;
    input.frame_count = sensor_data.frame_count;
    input.imu_valid = imu_healthy && (NULL != p_attitude);
    input.frame_valid = sensor_data.frame_valid;
    input.flow_valid = sensor_data.flow_valid;
    input.tof_valid = sensor_data.tof_valid;
    input.velocity_valid = sensor_data.velocity_valid;

    flow_navigation_core_update(&g_flow_navigation_core, &input, dt_s);

    taskENTER_CRITICAL();
    g_flow_navigation_state = g_flow_navigation_core.state;
    taskEXIT_CRITICAL();
}


void flow_navigation_get_state(flow_navigation_state_t * p_state)
{
    if (NULL == p_state)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_state = g_flow_navigation_state;
    taskEXIT_CRITICAL();
}


void flow_navigation_reset_position(void)
{
    if (false == g_flow_navigation_initialized)
    {
        flow_navigation_initialize();
    }

    flow_navigation_core_reset_position(&g_flow_navigation_core);

    taskENTER_CRITICAL();
    g_flow_navigation_state = g_flow_navigation_core.state;
    taskEXIT_CRITICAL();
}
