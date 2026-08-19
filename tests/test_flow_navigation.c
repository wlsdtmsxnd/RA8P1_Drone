#include "code/flow_hold_controller.h"
#include "code/flow_navigation_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_true(int condition, const char * message)
{
    if (!condition)
    {
        (void) fprintf(stderr, "FAIL: %s\n", message);
        exit(EXIT_FAILURE);
    }
}


static void require_near(float actual,
                         float expected,
                         float tolerance,
                         const char * message)
{
    if (fabsf(actual - expected) > tolerance)
    {
        (void) fprintf(stderr,
                       "FAIL: %s (actual %.3f expected %.3f)\n",
                       message,
                       (double) actual,
                       (double) expected);
        exit(EXIT_FAILURE);
    }
}


static flow_navigation_config_t navigation_config(void)
{
    const flow_navigation_config_t config =
    {
        .tof_min_mm = 200.0f,
        .tof_max_mm = 20000.0f,
        .minimum_tilt_cosine = 0.819152f,
        .tof_jump_limit_mm = 200.0f,
        .tof_reaccept_count = 5U,
        .velocity_filter_alpha = 0.15f,
        .gyro_compensation_gain = 0.05f,
        .sensor_x_sign = 1.0f,
        .sensor_y_sign = 1.0f,
        .swap_sensor_axes = false,
        .height_timeout_s = 0.20f,
        .flow_timeout_s = 0.12f,
        .vertical_accel_deadband_mm_s2 = 40.0f,
        .vertical_accel_limit_mm_s2 = 5000.0f,
        .vertical_velocity_damping = 0.999f,
        .height_position_correction = 0.25f,
        .height_velocity_correction = 0.04f
    };

    return config;
}


static flow_navigation_input_t valid_navigation_input(void)
{
    const flow_navigation_input_t input =
    {
        .integration_us = 12480U,
        .distance_mm = 500U,
        .frame_count = 1U,
        .imu_valid = true,
        .frame_valid = true,
        .flow_valid = true,
        .tof_valid = true,
        .velocity_valid = true
    };

    return input;
}


static void test_navigation_uses_module_velocity(void)
{
    flow_navigation_core_t core;
    flow_navigation_config_t config = navigation_config();
    flow_navigation_input_t input = valid_navigation_input();
    uint32_t frame;

    flow_navigation_core_init(&core, &config);
    input.velocity_x_cm_s = 100;
    input.flow_x_integral = 30000;

    for (frame = 1U; frame <= 20U; frame++)
    {
        input.frame_count = frame;
        flow_navigation_core_update(&core, &input, 0.002f);
    }

    require_true(core.state.height_valid, "TOF height becomes valid");
    require_true(core.state.flow_valid, "flow velocity becomes valid");
    require_near(core.state.height_mm, 500.0f, 0.5f,
                 "level TOF initializes height");
    require_true((core.state.velocity_x_mm_s > 900.0f) &&
                 (core.state.velocity_x_mm_s < 1000.0f),
                 "module velocity is converted from cm/s to mm/s");
    require_true(core.state.position_x_mm > 20.0f,
                 "filtered velocity integrates into position");
}


static void test_ra6m5_filter_and_timeout(void)
{
    flow_navigation_core_t core;
    flow_navigation_config_t config = navigation_config();
    flow_navigation_input_t input = valid_navigation_input();
    uint32_t index;

    flow_navigation_core_init(&core, &config);
    flow_navigation_core_update(&core, &input, 0.002f);

    input.frame_count++;
    input.distance_mm = 2000U;
    flow_navigation_core_update(&core, &input, 0.002f);
    require_near(core.state.filtered_tof_mm, 500.0f, 0.5f,
                 "single TOF jump is rejected");
    require_true(core.state.tof_reject_count == 1U,
                 "TOF rejection is counted");

    for (index = 0U; index < 70U; index++)
    {
        flow_navigation_core_update(&core, &input, 0.002f);
    }

    require_true(!core.state.flow_valid,
                 "stale flow is invalid after timeout");
}


static void test_ra6m5_gyro_compensation(void)
{
    flow_navigation_core_t core;
    flow_navigation_config_t config = navigation_config();
    flow_navigation_input_t input = valid_navigation_input();

    flow_navigation_core_init(&core, &config);
    input.gyro_y_dps = 10.0f;
    flow_navigation_core_update(&core, &input, 0.002f);

    require_near(core.state.velocity_x_mm_s, -37.5f, 0.1f,
                 "RA6M5 gyro compensation and 0.15 LPF are applied");
}


static flow_hold_config_t hold_config(void)
{
    const flow_hold_config_t config =
    {
        .height_position_kp = 1.0f,
        .height_velocity_kp = 0.08f,
        .height_velocity_ki = 0.02f,
        .height_velocity_integrator_limit_us = 80.0f,
        .height_velocity_limit_mm_s = 300.0f,
        .height_correction_limit_us = 150.0f,
        .position_kp = 0.5f,
        .velocity_kp = 0.02f,
        .velocity_ki = 0.002f,
        .velocity_integrator_limit_deg = 2.0f,
        .position_velocity_limit_mm_s = 300.0f,
        .angle_limit_deg = 5.0f,
        .height_target_min_mm = 200.0f,
        .height_target_max_mm = 2500.0f,
        .throttle_deadband = 0.05f,
        .throttle_height_rate_mm_s = 600.0f,
        .stick_position_rate_mm_s = 400.0f
    };

    return config;
}


static void test_hold_modes_and_signs(void)
{
    flow_hold_controller_t controller;
    flow_hold_config_t config = hold_config();
    flow_hold_input_t input =
    {
        .manual_base_us = 1504.0f,
        .manual_roll_target_deg = 2.0f,
        .manual_pitch_target_deg = -3.0f,
        .throttle = 0.84f,
        .height_mm = 500.0f,
        .height_valid = true,
        .flow_valid = true,
        .enabled = true
    };
    flow_hold_output_t output;

    flow_hold_controller_init(&controller, &config);
    input.requested_mode = FLOW_HOLD_MODE_MANUAL;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    require_near(output.roll_target_deg, 2.0f, 0.001f,
                 "manual mode preserves roll target");
    require_near(output.base_us, 1504.0f, 0.001f,
                 "manual mode preserves base throttle");

    input.requested_mode = FLOW_HOLD_MODE_ALTITUDE;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    require_true(output.altitude_active, "middle mode enables altitude hold");
    input.height_mm = 450.0f;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    require_true(output.height_correction_us > 0.0f,
                 "altitude loss increases base throttle");

    input.height_mm = 500.0f;
    input.position_x_mm = 100.0f;
    input.position_y_mm = 100.0f;
    input.requested_mode = FLOW_HOLD_MODE_POSITION;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    input.position_x_mm = 150.0f;
    input.position_y_mm = 150.0f;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    require_true(output.position_active, "high mode enables position hold");
    require_true(output.pitch_target_deg > 0.0f,
                 "positive X drift commands RA6M5 pitch braking sign");
    require_true(output.roll_target_deg > 0.0f,
                 "positive Y drift commands RA6M5 roll braking sign");

    input.flow_valid = false;
    flow_hold_controller_update(&controller, &input, 0.002f, &output);
    require_true(output.altitude_active && !output.position_active,
                 "flow loss degrades position hold to altitude hold");
}


int main(void)
{
    test_navigation_uses_module_velocity();
    test_ra6m5_filter_and_timeout();
    test_ra6m5_gyro_compensation();
    test_hold_modes_and_signs();
    (void) puts("Flow navigation and hold tests passed");
    return EXIT_SUCCESS;
}
