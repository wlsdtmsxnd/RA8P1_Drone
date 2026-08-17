#include "vofa_justfloat.h"

#include <stddef.h>
#include <string.h>

/* VOFA+ JustFloat 帧尾。 */
static uint8_t const g_vofa_frame_tail[4] =
{
    0x00U,
    0x00U,
    0x80U,
    0x7FU
};

/* 确认当前编译器使用 32 位 float。 */
_Static_assert(sizeof(float) == 4U,
               "VOFA JustFloat requires 32-bit float");


/* 生成一帧 JustFloat 数据。 */
void vofa_justfloat_build(float const channel_data[VOFA_CHANNEL_COUNT],
                          uint8_t frame_buffer[VOFA_FRAME_SIZE])
{
    uint32_t data_length;    /* 所有浮点通道占用的字节数。 */

    if ((NULL == channel_data) ||
        (NULL == frame_buffer))
    {
        return;
    }

    data_length = VOFA_CHANNEL_COUNT * sizeof(float);

    /*
     * RA8P1 为小端处理器，直接复制 float 即符合 JustFloat 字节序。
     */
    memcpy(frame_buffer,
           channel_data,
           data_length);

    memcpy(&frame_buffer[data_length],
           g_vofa_frame_tail,
           sizeof(g_vofa_frame_tail));
}
