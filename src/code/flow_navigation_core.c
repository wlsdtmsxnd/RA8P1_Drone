#include "flow_navigation_core.h"

#include <math.h>
#include <stddef.h>

#define FLOW_NAVIGATION_DEG_TO_RAD          (0.01745329251994329577f)


static float flow_navigation_clampf(float value,
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


static float flow_navigation_tof_average(
    const flow_navigation_core_t * p_core)
{
    float sum = 0.0f;
    uint32_t index;

    for (index = 0U; index < FLOW_NAVIGATION_TOF_WINDOW_SIZE; index++)
    {
        sum += p_core->tof_window[index];
    }

    return sum / (float) FLOW_NAVIGATION_TOF_WINDOW_SIZE;
}


static void flow_navigation_fill_tof_window(
    flow_navigation_core_t * p_core,
    float height_mm)
{
    uint32_t index;

    for (index = 0U; index < FLOW_NAVIGATION_TOF_WINDOW_SIZE; index++)
    {
        p_core->tof_window[index] = height_mm;
    }

    p_core->tof_window_index = 0U;
    p_core->tof_window_initialized = true;
}


static bool flow_navigation_accept_tof(flow_navigation_core_t * p_core,
                                       float measured_height_mm,
                                       float * p_filtered_height_mm)
{
    float average_mm;

    if ((NULL == p_filtered_height_mm) ||
        (0 == isfinite(measured_height_mm)))
    {
        return false;
    }

    if (false == p_core->tof_window_initialized)
    {
        flow_navigation_fill_tof_window(p_core, measured_height_mm);
        p_core->tof_candidate_count = 0U;
        *p_filtered_height_mm = measured_height_mm;
        return true;
    }

    average_mm = flow_navigation_tof_average(p_core);

    if (fabsf(measured_height_mm - average_mm) >
        p_core->config.tof_jump_limit_mm)
    {
        p_core->state.tof_reject_count++;

        if (p_core->tof_candidate_count < UINT8_MAX)
        {
            p_core->tof_candidate_count++;
        }
        else
        {
            /* 计数已饱和，继续等待接受条件。 */
        }

        if (p_core->tof_candidate_count <
            p_core->config.tof_reaccept_count)
        {
            return false;
        }

        flow_navigation_fill_tof_window(p_core, measured_height_mm);
        p_core->tof_candidate_count = 0U;
        *p_filtered_height_mm = measured_height_mm;
        return true;
    }

    p_core->tof_candidate_count = 0U;
    p_core->tof_window[p_core->tof_window_index] = measured_height_mm;
    p_core->tof_window_index++;

    if (p_core->tof_window_index >= FLOW_NAVIGATION_TOF_WINDOW_SIZE)
    {
        p_core->tof_window_index = 0U;
    }

    *p_filtered_height_mm = flow_navigation_tof_average(p_core);
    return true;
}


void flow_navigation_core_init(
    flow_navigation_core_t * p_core,
    const flow_navigation_config_t * p_config)
{
    if ((NULL == p_core) || (NULL == p_config))
    {
        return;
    }

    *p_core = (flow_navigation_core_t) {0};
    p_core->config = *p_config;
}


void flow_navigation_core_update(
    flow_navigation_core_t * p_core,
    const flow_navigation_input_t * p_input,
    float dt_s)
{
    flow_navigation_state_t * p_state;
    float vertical_accel_mm_s2;
    float roll_cosine;
    float pitch_cosine;
    float tilt_cosine;
    float measured_height_mm;
    float filtered_height_mm;
    float height_error_mm;
    float sensor_velocity_x_mm_s;
    float sensor_velocity_y_mm_s;
    float body_velocity_x_mm_s;
    float body_velocity_y_mm_s;
    bool new_frame;

    if ((NULL == p_core) || (NULL == p_input) ||
        (dt_s <= 0.0f) || (0 == isfinite(dt_s)))
    {
        return;
    }

    p_state = &p_core->state;
    p_state->height_age_s += dt_s;
    p_state->flow_age_s += dt_s;
    p_state->vertical_accel_mm_s2 = 0.0f;
    p_state->raw_distance_mm = p_input->distance_mm;
    p_state->frame_count = p_input->frame_count;

    if (p_input->imu_valid)
    {
        vertical_accel_mm_s2 = flow_navigation_clampf(
            p_input->vertical_accel_mm_s2,
            -p_core->config.vertical_accel_limit_mm_s2,
            p_core->config.vertical_accel_limit_mm_s2);

        if (fabsf(vertical_accel_mm_s2) <
            p_core->config.vertical_accel_deadband_mm_s2)
        {
            vertical_accel_mm_s2 = 0.0f;
        }

        p_state->vertical_accel_mm_s2 = vertical_accel_mm_s2;

        if (p_core->tof_window_initialized)
        {
            p_state->vertical_velocity_mm_s =
                (p_state->vertical_velocity_mm_s *
                 p_core->config.vertical_velocity_damping) +
                (vertical_accel_mm_s2 * dt_s);
            p_state->height_mm +=
                p_state->vertical_velocity_mm_s * dt_s;
        }
    }

    new_frame = p_input->frame_valid &&
                ((!p_core->frame_count_initialized) ||
                 (p_input->frame_count != p_core->last_frame_count));

    if (new_frame)
    {
        p_core->frame_count_initialized = true;
        p_core->last_frame_count = p_input->frame_count;

        roll_cosine = cosf(p_input->roll_deg * FLOW_NAVIGATION_DEG_TO_RAD);
        pitch_cosine = cosf(p_input->pitch_deg * FLOW_NAVIGATION_DEG_TO_RAD);
        tilt_cosine = roll_cosine * pitch_cosine;

        if (tilt_cosine < p_core->config.minimum_tilt_cosine)
        {
            tilt_cosine = p_core->config.minimum_tilt_cosine;
        }

        if (p_input->tof_valid &&
            ((float) p_input->distance_mm >= p_core->config.tof_min_mm) &&
            ((float) p_input->distance_mm <= p_core->config.tof_max_mm) &&
            (fabsf(p_input->roll_deg) <= 35.0f) &&
            (fabsf(p_input->pitch_deg) <= 35.0f))
        {
            measured_height_mm = (float) p_input->distance_mm * tilt_cosine;
            if (flow_navigation_accept_tof(
                p_core,
                measured_height_mm,
                &filtered_height_mm))

            {
                p_state->filtered_tof_mm = filtered_height_mm;
                p_state->height_age_s = 0.0f;

                if (false == p_state->height_valid)
                {
                    p_state->height_mm = filtered_height_mm;
                    p_state->vertical_velocity_mm_s = 0.0f;
                }
                else
                {
                    height_error_mm = flow_navigation_clampf(
                        filtered_height_mm - p_state->height_mm,
                        -p_core->config.tof_jump_limit_mm,
                        p_core->config.tof_jump_limit_mm);
                    p_state->height_mm +=
                        height_error_mm *
                        p_core->config.height_position_correction;
                    p_state->vertical_velocity_mm_s +=
                        height_error_mm *
                        p_core->config.height_velocity_correction;
                }
            }
        }
        else
        {
            p_state->tof_reject_count++;
        }

        if (p_input->velocity_valid &&
            (p_state->height_age_s <= p_core->config.height_timeout_s) &&
            p_input->imu_valid)
        {
            /*
             * UP-T301 已按 UPIX 积分时间和距离给出物理速度。这里与 RA6M5
             * 控制流程一致，只做安装方向、角速度补偿和 0.85/0.15 低通。
             */
            sensor_velocity_x_mm_s =
                (float) p_input->velocity_x_cm_s * 10.0f;
            sensor_velocity_y_mm_s =
                (float) p_input->velocity_y_cm_s * 10.0f;

            sensor_velocity_x_mm_s *= p_core->config.sensor_x_sign;
            sensor_velocity_y_mm_s *= p_core->config.sensor_y_sign;

            if (p_core->config.swap_sensor_axes)
            {
                body_velocity_x_mm_s = sensor_velocity_y_mm_s;
                body_velocity_y_mm_s = sensor_velocity_x_mm_s;
            }
            else
            {
                body_velocity_x_mm_s = sensor_velocity_x_mm_s;
                body_velocity_y_mm_s = sensor_velocity_y_mm_s;
            }

            body_velocity_x_mm_s -=
                p_input->gyro_y_dps *
                p_state->filtered_tof_mm *
                p_core->config.gyro_compensation_gain;
            body_velocity_y_mm_s -=
                p_input->gyro_x_dps *
                p_state->filtered_tof_mm *
                p_core->config.gyro_compensation_gain;

            p_state->velocity_x_mm_s +=
                p_core->config.velocity_filter_alpha *
                (body_velocity_x_mm_s - p_state->velocity_x_mm_s);
            p_state->velocity_y_mm_s +=
                p_core->config.velocity_filter_alpha *
                (body_velocity_y_mm_s - p_state->velocity_y_mm_s);
            p_state->flow_age_s = 0.0f;
            p_core->velocity_initialized = true;
        }
        else
        {
            p_state->flow_reject_count++;
        }
    }

    p_state->height_valid =
        p_core->tof_window_initialized && p_input->imu_valid &&
        (p_state->height_age_s <= p_core->config.height_timeout_s);
    p_state->flow_valid =
        p_core->velocity_initialized && p_input->imu_valid &&
        (p_state->flow_age_s <= p_core->config.flow_timeout_s);

    if (p_state->flow_valid)
    {
        p_state->position_x_mm += p_state->velocity_x_mm_s * dt_s;
        p_state->position_y_mm += p_state->velocity_y_mm_s * dt_s;
    }
    else
    {
        p_state->velocity_x_mm_s = 0.0f;
        p_state->velocity_y_mm_s = 0.0f;
        p_core->velocity_initialized = false;
    }

    if (!p_state->height_valid)
    {
        p_state->vertical_velocity_mm_s = 0.0f;
    }

    p_state->navigation_ready =
        p_state->height_valid && p_state->flow_valid;
}


void flow_navigation_core_reset_position(flow_navigation_core_t * p_core)
{
    if (NULL == p_core)
    {
        return;
    }

    p_core->state.position_x_mm = 0.0f;
    p_core->state.position_y_mm = 0.0f;
}
