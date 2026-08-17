#ifndef CODE_VOFA_JUSTFLOAT_H_
#define CODE_VOFA_JUSTFLOAT_H_

#include <stdint.h>

/* 当前 VOFA+ 数据通道数量：Roll、Pitch、Yaw。 */
#define VOFA_CHANNEL_COUNT          (3U)

/* JustFloat 帧长度：3 个 float 加 4 字节帧尾。 */
#define VOFA_FRAME_SIZE             ((VOFA_CHANNEL_COUNT * sizeof(float)) + 4U)

/*
 * 生成 VOFA+ JustFloat 数据帧。
 *
 * channel_data：按顺序存放所有通道数据。
 * frame_buffer：长度至少为 VOFA_FRAME_SIZE。
 */
void vofa_justfloat_build(float const channel_data[VOFA_CHANNEL_COUNT],
                          uint8_t frame_buffer[VOFA_FRAME_SIZE]);

#endif /* CODE_VOFA_JUSTFLOAT_H_ */
