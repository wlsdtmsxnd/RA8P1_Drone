#include "flow_hold_controller.h"

#include <math.h>
#include <stddef.h>

static float flow_hold_clampf(float value,
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


static void flow_hold_reset_pid_state(flow_hold_controller_t * p_controller)
{
    pid_controller_reset(&p_controller->height_position_controller);
    pid_controller_reset(&p_controller->height_velocity_controller);
    pid_controller_reset(&p_controller->position_x_controller);
    pid_controller_reset(&p_controller->position_y_controller);
    pid_controller_reset(&p_controller->velocity_x_controller);
    pid_controller_reset(&p_controller->velocity_y_controller);
}


void flow_hold_controller_init(flow_hold_controller_t * p_controller,
                               const flow_hold_config_t * p_config)
{
    if ((NULL == p_controller) || (NULL == p_config))
    {
        return;
    }

    *p_controller = (flow_hold_controller_t) {0};
    p_controller->config = *p_config;

    pid_controller_configure(&p_controller->height_position_controller,
                             p_config->height_position_kp,
                             0.0f,
                             0.0f,
                             0.0f,
                             p_config->height_velocity_limit_mm_s,
                             0.0f);
    pid_controller_configure(&p_controller->height_velocity_controller,
                             p_config->height_velocity_kp,
                             p_config->height_velocity_ki,
                             0.0f,
                             p_config->height_velocity_integrator_limit_us,
                             p_config->height_correction_limit_us,
                             0.0f);
    pid_controller_configure(&p_controller->position_x_controller,
                             p_config->position_kp,
                             0.0f,
                             0.0f,
                             0.0f,
                             p_config->position_velocity_limit_mm_s,
                             0.0f);
    pid_controller_configure(&p_controller->position_y_controller,
                             p_config->position_kp,
                             0.0f,
                             0.0f,
                             0.0f,
                             p_config->position_velocity_limit_mm_s,
                             0.0f);
    pid_controller_configure(&p_controller->velocity_x_controller,
                             p_config->velocity_kp,
                             p_config->velocity_ki,
                             0.0f,
                             p_config->velocity_integrator_limit_deg,
                             p_config->angle_limit_deg,
                             0.0f);
    pid_controller_configure(&p_controller->velocity_y_controller,
                             p_config->velocity_kp,
                             p_config->velocity_ki,
                             0.0f,
                             p_config->velocity_integrator_limit_deg,
                             p_config->angle_limit_deg,
                             0.0f);
    p_controller->active_mode = FLOW_HOLD_MODE_MANUAL;
    p_controller->initialized = true;
}


void flow_hold_controller_reset(flow_hold_controller_t * p_controller)
{
    if ((NULL == p_controller) || (false == p_controller->initialized))
    {
        return;
    }

    flow_hold_reset_pid_state(p_controller);
    p_controller->active_mode = FLOW_HOLD_MODE_MANUAL;
    p_controller->target_height_mm = 0.0f;
    p_controller->target_position_x_mm = 0.0f;
    p_controller->target_position_y_mm = 0.0f;
    p_controller->base_reference_us = 0.0f;
    p_controller->throttle_reference = 0.0f;
}


static void flow_hold_capture(flow_hold_controller_t * p_controller,
                              const flow_hold_input_t * p_input,
                              flow_hold_mode_t active_mode)
{
    flow_hold_reset_pid_state(p_controller);
    p_controller->active_mode = active_mode;
    p_controller->target_height_mm = p_input->height_mm;
    p_controller->target_position_x_mm = p_input->position_x_mm;
    p_controller->target_position_y_mm = p_input->position_y_mm;
    p_controller->base_reference_us = p_input->manual_base_us;
    p_controller->throttle_reference = p_input->throttle;
}


void flow_hold_controller_update(flow_hold_controller_t * p_controller,
                                 const flow_hold_input_t * p_input,
                                 float dt_s,
                                 flow_hold_output_t * p_output)
{
    flow_hold_mode_t effective_mode;
    float throttle_delta;
    float target_height_rate_mm_s;
    float target_vertical_velocity_mm_s;
    float target_body_velocity_x_mm_s;
    float target_body_velocity_y_mm_s;
    float target_velocity_x_mm_s;
    float target_velocity_y_mm_s;
    float angle_x_deg;
    float angle_y_deg;

    if ((NULL == p_controller) || (NULL == p_input) ||
        (NULL == p_output) || (false == p_controller->initialized) ||
        (dt_s <= 0.0f) || (0 == isfinite(dt_s)))
    {
        return;
    }

    *p_output = (flow_hold_output_t)
    {
        .active_mode = FLOW_HOLD_MODE_MANUAL,
        .base_us = p_input->manual_base_us,
        .roll_target_deg = p_input->manual_roll_target_deg,
        .pitch_target_deg = p_input->manual_pitch_target_deg
    };

    if ((!p_input->enabled) ||
        (FLOW_HOLD_MODE_MANUAL == p_input->requested_mode) ||
        (!p_input->height_valid))
    {
        flow_hold_controller_reset(p_controller);
        return;
    }

    effective_mode = p_input->requested_mode;

    if ((FLOW_HOLD_MODE_POSITION == effective_mode) &&
        (!p_input->flow_valid))
    {
        /* 光流失效时保留定高，Roll/Pitch 立即交还飞手。 */
        effective_mode = FLOW_HOLD_MODE_ALTITUDE;
    }

    if (effective_mode != p_controller->active_mode)
    {
        flow_hold_capture(p_controller, p_input, effective_mode);
    }

    throttle_delta = p_input->throttle - p_controller->throttle_reference;

    if (fabsf(throttle_delta) <= p_controller->config.throttle_deadband)
    {
        throttle_delta = 0.0f;
    }

    target_height_rate_mm_s = flow_hold_clampf(
        throttle_delta * p_controller->config.throttle_height_rate_mm_s,
        -p_controller->config.height_velocity_limit_mm_s,
        p_controller->config.height_velocity_limit_mm_s);
    p_controller->target_height_mm = flow_hold_clampf(
        p_controller->target_height_mm +
        (target_height_rate_mm_s * dt_s),
        p_controller->config.height_target_min_mm,
        p_controller->config.height_target_max_mm);

    target_vertical_velocity_mm_s = pid_controller_update(
        &p_controller->height_position_controller,
        p_controller->target_height_mm,
        p_input->height_mm,
        dt_s);
    p_output->height_correction_us = pid_controller_update(
        &p_controller->height_velocity_controller,
        target_vertical_velocity_mm_s,
        p_input->vertical_velocity_mm_s,
        dt_s);
    p_output->base_us = p_controller->base_reference_us +
                        p_output->height_correction_us;
    p_output->active_mode = effective_mode;
    p_output->altitude_active = true;

    if (FLOW_HOLD_MODE_POSITION == effective_mode)
    {
        /* 后拉 Pitch 为正，对应机体系向后（-X）的目标速度。 */
        target_body_velocity_x_mm_s =
            -p_input->pitch_stick *
            p_controller->config.stick_position_rate_mm_s;
        target_body_velocity_y_mm_s =
            p_input->roll_stick *
            p_controller->config.stick_position_rate_mm_s;
        p_controller->target_position_x_mm +=
            target_body_velocity_x_mm_s * dt_s;
        p_controller->target_position_y_mm +=
            target_body_velocity_y_mm_s * dt_s;

        target_velocity_x_mm_s = pid_controller_update(
            &p_controller->position_x_controller,
            p_controller->target_position_x_mm,
            p_input->position_x_mm,
            dt_s);
        target_velocity_y_mm_s = pid_controller_update(
            &p_controller->position_y_controller,
            p_controller->target_position_y_mm,
            p_input->position_y_mm,
            dt_s);
        angle_x_deg = pid_controller_update(
            &p_controller->velocity_x_controller,
            target_velocity_x_mm_s,
            p_input->velocity_x_mm_s,
            dt_s);
        angle_y_deg = pid_controller_update(
            &p_controller->velocity_y_controller,
            target_velocity_y_mm_s,
            p_input->velocity_y_mm_s,
            dt_s);

        p_output->pitch_target_deg = -angle_x_deg;
        p_output->roll_target_deg = -angle_y_deg;
        p_output->position_active = true;
    }

    p_output->target_height_mm = p_controller->target_height_mm;
    p_output->target_position_x_mm =
        p_controller->target_position_x_mm;
    p_output->target_position_y_mm =
        p_controller->target_position_y_mm;
}
