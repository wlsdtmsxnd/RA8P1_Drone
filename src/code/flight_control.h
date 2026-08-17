#ifndef CODE_FLIGHT_CONTROL_H_
#define CODE_FLIGHT_CONTROL_H_

#include <stdbool.h>

#define FLIGHT_CONTROL_MOTOR_COUNT    (4U)

typedef struct st_flight_control_status
{
    float motor_us[FLIGHT_CONTROL_MOTOR_COUNT];
    float base_us;
    float roll_correction_us;
    float pitch_correction_us;
    float yaw_correction_us;
    bool valid;
} flight_control_status_t;


void flight_control_update(bool imu_healthy);
void flight_control_get_status(flight_control_status_t * p_status);


#endif /* CODE_FLIGHT_CONTROL_H_ */
