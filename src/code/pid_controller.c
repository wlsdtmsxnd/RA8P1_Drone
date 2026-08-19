#include "pid_controller.h"

#include <math.h>
#include <stddef.h>


static float pid_controller_clampf(float value,
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


void pid_controller_configure(pid_controller_t * p_controller,
                              float kp,
                              float ki,
                              float kd,
                              float integrator_limit,
                              float output_limit,
                              float derivative_alpha)
{
    if (NULL == p_controller)
    {
        return;
    }

    p_controller->kp = kp;
    p_controller->ki = ki;
    p_controller->kd = kd;
    p_controller->integrator_limit =
        (integrator_limit > 0.0f) ? integrator_limit : 0.0f;
    p_controller->output_limit =
        (output_limit > 0.0f) ? output_limit : 0.0f;
    p_controller->derivative_alpha = pid_controller_clampf(
        derivative_alpha,
        0.0f,
        1.0f);
    pid_controller_reset(p_controller);
}


void pid_controller_reset(pid_controller_t * p_controller)
{
    if (NULL == p_controller)
    {
        return;
    }

    p_controller->integrator = 0.0f;
    p_controller->previous_measurement = 0.0f;
    p_controller->derivative_state = 0.0f;
    p_controller->measurement_initialized = false;
}


float pid_controller_update(pid_controller_t * p_controller,
                            float setpoint,
                            float measurement,
                            float dt_s)
{
    float error;
    float candidate_integrator;
    float raw_derivative;
    float unconstrained_output;
    float output;
    bool integrate;

    if ((NULL == p_controller) ||
        (dt_s <= 0.0f) ||
        (0 == isfinite(dt_s)) ||
        (0 == isfinite(setpoint)) ||
        (0 == isfinite(measurement)))
    {
        pid_controller_reset(p_controller);
        return 0.0f;
    }

    error = setpoint - measurement;

    if (false == p_controller->measurement_initialized)
    {
        p_controller->previous_measurement = measurement;
        p_controller->derivative_state = 0.0f;
        p_controller->measurement_initialized = true;
    }
    else
    {
        raw_derivative =
            (measurement - p_controller->previous_measurement) / dt_s;
        p_controller->derivative_state +=
            p_controller->derivative_alpha *
            (raw_derivative - p_controller->derivative_state);
        p_controller->previous_measurement = measurement;
    }

    candidate_integrator = pid_controller_clampf(
        p_controller->integrator + (p_controller->ki * error * dt_s),
        -p_controller->integrator_limit,
        p_controller->integrator_limit);
    unconstrained_output =
        (p_controller->kp * error) + candidate_integrator -
        (p_controller->kd * p_controller->derivative_state);

    if (p_controller->output_limit > 0.0f)
    {
        integrate = ((unconstrained_output <= p_controller->output_limit) &&
                     (unconstrained_output >= -p_controller->output_limit)) ||
                    ((unconstrained_output > p_controller->output_limit) &&
                     (error < 0.0f)) ||
                    ((unconstrained_output < -p_controller->output_limit) &&
                     (error > 0.0f));
    }
    else
    {
        /* output_limit == 0：与RA6M5一致，不限制P+I+D总输出。 */
        integrate = true;
    }

    if (true == integrate)
    {
        p_controller->integrator = candidate_integrator;
    }

    output = (p_controller->kp * error) + p_controller->integrator -
             (p_controller->kd * p_controller->derivative_state);

    if (p_controller->output_limit > 0.0f)
    {
        return pid_controller_clampf(output,
                                     -p_controller->output_limit,
                                     p_controller->output_limit);
    }

    return output;
}
