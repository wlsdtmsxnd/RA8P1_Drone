#include "code/pid_controller.h"
#include "code/quad_x_mixer.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


static void require_near(float actual, float expected, float tolerance,
                         char const * p_name)
{
    if (fabsf(actual - expected) > tolerance)
    {
        (void) fprintf(stderr,
                       "%s: expected %.6f, got %.6f\n",
                       p_name,
                       (double) expected,
                       (double) actual);
        exit(EXIT_FAILURE);
    }
}


static void test_quad_x_mixer(void)
{
    float output[QUAD_X_MIXER_MOTOR_COUNT];

    quad_x_mixer_apply(1200.0f, 10.0f, 20.0f, 5.0f, output);
    require_near(output[0], 1225.0f, 0.001f, "mixer m1");
    require_near(output[1], 1215.0f, 0.001f, "mixer m2");
    require_near(output[2], 1165.0f, 0.001f, "mixer m3");
    require_near(output[3], 1195.0f, 0.001f, "mixer m4");

    quad_x_mixer_apply(1200.0f, -10.0f, -20.0f, -5.0f, output);
    require_near(output[0], 1175.0f, 0.001f, "reverse mixer m1");
    require_near(output[1], 1185.0f, 0.001f, "reverse mixer m2");
    require_near(output[2], 1235.0f, 0.001f, "reverse mixer m3");
    require_near(output[3], 1205.0f, 0.001f, "reverse mixer m4");
}


static void test_pid_limits_and_reset(void)
{
    pid_controller_t controller;
    float output = 0.0f;
    uint32_t update_index;

    pid_controller_configure(&controller,
                             0.0f,
                             1.0f,
                             0.0f,
                             0.5f,
                             0.0f,
                             0.2f);

    for (update_index = 0U; update_index < 20U; update_index++)
    {
        output = pid_controller_update(&controller, 1.0f, 0.0f, 0.1f);
    }

    require_near(output, 0.5f, 0.001f, "integrator limit");
    require_near(controller.integrator, 0.5f, 0.001f, "integrator state");

    output = pid_controller_update(&controller, NAN, 0.0f, 0.1f);
    require_near(output, 0.0f, 0.001f, "nonfinite output");
    require_near(controller.integrator, 0.0f, 0.001f, "nonfinite reset");

    pid_controller_configure(&controller,
                             2.0f,
                             0.0f,
                             0.0f,
                             0.0f,
                             3.0f,
                             0.2f);
    output = pid_controller_update(&controller, 10.0f, 0.0f, 0.01f);
    require_near(output, 3.0f, 0.001f, "output limit");
}


int main(void)
{
    test_quad_x_mixer();
    test_pid_limits_and_reset();
    (void) puts("control math tests passed");
    return EXIT_SUCCESS;
}
