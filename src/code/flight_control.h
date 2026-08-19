#ifndef CODE_FLIGHT_CONTROL_H_
#define CODE_FLIGHT_CONTROL_H_

#include <stdbool.h>

#define FLIGHT_CONTROL_MOTOR_COUNT    (4U)

typedef enum
{
    FLIGHT_CONTROL_FAULT_NONE = 0,
    FLIGHT_CONTROL_FAULT_NONFINITE,
    FLIGHT_CONTROL_FAULT_ROLL_TILT,
    FLIGHT_CONTROL_FAULT_PITCH_TILT,
    FLIGHT_CONTROL_FAULT_ROLL_RATE,
    FLIGHT_CONTROL_FAULT_PITCH_RATE,
    FLIGHT_CONTROL_FAULT_YAW_RATE,
    FLIGHT_CONTROL_FAULT_MOTOR_OUTPUT
} flight_control_fault_reason_t;

typedef struct st_flight_control_status
{
    float motor_us[FLIGHT_CONTROL_MOTOR_COUNT];
    float base_us;
    float yaw_target_rate_dps;
    float roll_correction_us;
    float pitch_correction_us;
    float yaw_correction_us;
    float roll_integrator_us;
    float pitch_integrator_us;
    float yaw_integrator_us;
    flight_control_fault_reason_t fault_reason;
    bool valid;
} flight_control_status_t;


void flight_control_update(bool imu_healthy);
void flight_control_prop_load_vibration_update(bool imu_healthy);
void flight_control_get_status(flight_control_status_t * p_status);


#endif /* CODE_FLIGHT_CONTROL_H_ */
