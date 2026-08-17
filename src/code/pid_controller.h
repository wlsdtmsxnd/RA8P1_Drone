#ifndef CODE_PID_CONTROLLER_H_
#define CODE_PID_CONTROLLER_H_

#include <stdbool.h>

typedef struct st_pid_controller
{
    float kp;
    float ki;
    float kd;
    float integrator;
    float integrator_limit;
    float output_limit;
    float previous_measurement;
    float derivative_state;
    float derivative_alpha;
    bool measurement_initialized;
} pid_controller_t;


void pid_controller_configure(pid_controller_t * p_controller,
                              float kp,
                              float ki,
                              float kd,
                              float integrator_limit,
                              float output_limit,
                              float derivative_alpha);
void pid_controller_reset(pid_controller_t * p_controller);
float pid_controller_update(pid_controller_t * p_controller,
                            float setpoint,
                            float measurement,
                            float dt_s);


#endif /* CODE_PID_CONTROLLER_H_ */
