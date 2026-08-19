#ifndef CODE_FLOW_NAVIGATION_CORE_H_
#define CODE_FLOW_NAVIGATION_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#define FLOW_NAVIGATION_TOF_WINDOW_SIZE    (5U)

typedef struct
{
    float tof_min_mm;
    float tof_max_mm;
    float minimum_tilt_cosine;
    float tof_jump_limit_mm;
    uint8_t tof_reaccept_count;
    float velocity_filter_alpha;
    float gyro_compensation_gain;
    float sensor_x_sign;
    float sensor_y_sign;
    bool swap_sensor_axes;
    float height_timeout_s;
    float flow_timeout_s;
    float vertical_accel_deadband_mm_s2;
    float vertical_accel_limit_mm_s2;
    float vertical_velocity_damping;
    float height_position_correction;
    float height_velocity_correction;
} flow_navigation_config_t;

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_x_dps;
    float gyro_y_dps;
    float vertical_accel_mm_s2;
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    int32_t velocity_x_cm_s;
    int32_t velocity_y_cm_s;
    uint16_t integration_us;
    uint16_t distance_mm;
    uint32_t frame_count;
    bool imu_valid;
    bool frame_valid;
    bool flow_valid;
    bool tof_valid;
    bool velocity_valid;
} flow_navigation_input_t;

typedef struct
{
    float height_mm;
    float filtered_tof_mm;
    float vertical_velocity_mm_s;
    float vertical_accel_mm_s2;
    float velocity_x_mm_s;
    float velocity_y_mm_s;
    float position_x_mm;
    float position_y_mm;
    float height_age_s;
    float flow_age_s;
    uint16_t raw_distance_mm;
    uint32_t frame_count;
    uint32_t tof_reject_count;
    uint32_t flow_reject_count;
    bool height_valid;
    bool flow_valid;
    bool navigation_ready;
} flow_navigation_state_t;

typedef struct
{
    flow_navigation_config_t config;
    flow_navigation_state_t state;
    float tof_window[FLOW_NAVIGATION_TOF_WINDOW_SIZE];
    uint32_t last_frame_count;
    uint8_t tof_window_index;
    uint8_t tof_candidate_count;
    bool tof_window_initialized;
    bool frame_count_initialized;
    bool velocity_initialized;
} flow_navigation_core_t;

void flow_navigation_core_init(
    flow_navigation_core_t * p_core,
    const flow_navigation_config_t * p_config);
void flow_navigation_core_update(
    flow_navigation_core_t * p_core,
    const flow_navigation_input_t * p_input,
    float dt_s);
void flow_navigation_core_reset_position(flow_navigation_core_t * p_core);

#endif /* CODE_FLOW_NAVIGATION_CORE_H_ */
