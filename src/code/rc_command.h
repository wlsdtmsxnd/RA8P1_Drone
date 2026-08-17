#ifndef CODE_RC_COMMAND_H_
#define CODE_RC_COMMAND_H_

#include <stdbool.h>
#include <stdint.h>

/* CH6 三档飞行模式开关。 */
typedef enum
{
    RC_MODE_LOW = 0,
    RC_MODE_MIDDLE,
    RC_MODE_HIGH
} rc_mode_t;

/* 供安全逻辑和后续控制器使用的归一化遥控指令。 */
typedef struct
{
    float roll;                 /* [-1, 1]，右打杆为正。 */
    float pitch;                /* [-1, 1]，后拉杆为正。 */
    float throttle;             /* [0, 1]，最低油门为 0。 */
    float yaw;                  /* [-1, 1]，右打杆为正。 */
    rc_mode_t mode;             /* CH6 三档位置。 */
    bool connected;             /* CRSF 通道帧是否持续有效。 */
    bool throttle_low;          /* 是否满足安全解锁的低油门条件。 */
    bool arm_switch_low;        /* CH5 是否明确处于低档。 */
    bool arm_switch_high;       /* CH5 是否明确处于高档。 */
} rc_command_t;

/* 获取一次 CRSF 快照并转换为归一化飞控指令。 */
void rc_command_get(rc_command_t * p_command);

#endif /* CODE_RC_COMMAND_H_ */
