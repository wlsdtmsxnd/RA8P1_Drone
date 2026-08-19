#include "quad_x_mixer.h"

#include <stddef.h>


void quad_x_mixer_apply(float base,
                        float roll_correction,
                        float pitch_correction,
                        float yaw_correction,
                        float output[QUAD_X_MIXER_MOTOR_COUNT])
{
    if (NULL == output)
    {
        return;
    }

    output[0] = base + pitch_correction + roll_correction - yaw_correction;
    output[1] = base + pitch_correction - roll_correction + yaw_correction;
    output[2] = base - pitch_correction - roll_correction - yaw_correction;
    output[3] = base - pitch_correction + roll_correction + yaw_correction;
}
