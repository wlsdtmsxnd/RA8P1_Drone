#ifndef CODE_QUAD_X_MIXER_H_
#define CODE_QUAD_X_MIXER_H_

#define QUAD_X_MIXER_MOTOR_COUNT    (4U)

/*
 * FRD / Quad-X：M1 左前、M2 右前、M3 右后、M4 左后。
 * 正修正分别代表正 Roll、正 Pitch、正 Yaw 力矩。
 */
void quad_x_mixer_apply(float base,
                        float roll_correction,
                        float pitch_correction,
                        float yaw_correction,
                        float output[QUAD_X_MIXER_MOTOR_COUNT]);

#endif /* CODE_QUAD_X_MIXER_H_ */
