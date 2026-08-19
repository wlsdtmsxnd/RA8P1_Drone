#ifndef CODE_FLOW_HOLD_CONTROLLER_H_
#define CODE_FLOW_HOLD_CONTROLLER_H_

#include "pid_controller.h"

#include <stdbool.h>

typedef enum
{
    FLOW_HOLD_MODE_MANUAL = 0,
    FLOW_HOLD_MODE_ALTITUDE,
    FLOW_HOLD_MODE_POSITION
} flow_hold_mode_t;

typedef struct
{
    float height_position_kp;
    float height_velocity_kp;
    float height_velocity_ki;
    float height_velocity_integrator_limit_us;
    float height_velocity_limit_mm_s;
    float height_correction_limit_us;
    float position_kp;
    float velocity_kp;
    float velocity_ki;
    float velocity_integrator_limit_deg;
    float position_velocity_limit_mm_s;
    float angle_limit_deg;
    float height_target_min_mm;
    float height_target_max_mm;
    float throttle_deadband;
    float throttle_height_rate_mm_s;
    float stick_position_rate_mm_s;
} flow_hold_config_t;

typedef struct
{
    flow_hold_mode_t requested_mode;
    float manual_base_us;
    float manual_roll_target_deg;
    float manual_pitch_target_deg;
    float throttle;
    float roll_stick;
    float pitch_stick;
    float yaw_deg;
    float height_mm;
    float vertical_velocity_mm_s;
    float position_x_mm;
    float position_y_mm;
    float velocity_x_mm_s;
    float velocity_y_mm_s;
    bool enabled;
    bool height_valid;
    bool flow_valid;
} flow_hold_input_t;

typedef struct
{
    flow_hold_mode_t active_mode;
    float base_us;
    float roll_target_deg;
    float pitch_target_deg;
    float height_correction_us;
    float target_height_mm;
    float target_position_x_mm;
    float target_position_y_mm;
    bool altitude_active;
    bool position_active;
} flow_hold_output_t;

typedef struct
{
    flow_hold_config_t config;
    pid_controller_t height_position_controller;
    pid_controller_t height_velocity_controller;
    pid_controller_t position_x_controller;
    pid_controller_t position_y_controller;
    pid_controller_t velocity_x_controller;
    pid_controller_t velocity_y_controller;
    flow_hold_mode_t active_mode;
    float target_height_mm;
    float target_position_x_mm;
    float target_position_y_mm;
    float base_reference_us;
    float throttle_reference;
    bool initialized;
} flow_hold_controller_t;

void flow_hold_controller_init(flow_hold_controller_t * p_controller,
                               const flow_hold_config_t * p_config);
void flow_hold_controller_reset(flow_hold_controller_t * p_controller);
void flow_hold_controller_update(flow_hold_controller_t * p_controller,
                                 const flow_hold_input_t * p_input,
                                 float dt_s,
                                 flow_hold_output_t * p_output);

#endif /* CODE_FLOW_HOLD_CONTROLLER_H_ */
