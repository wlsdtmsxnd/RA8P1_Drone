#ifndef DRIVER_MOTOR_OUTPUT_H_
#define DRIVER_MOTOR_OUTPUT_H_

#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_OUTPUT_COUNT          (4U)
#define MOTOR_OUTPUT_PWM_HZ         (50U)
#define MOTOR_OUTPUT_MIN_US         (1000U)
#define MOTOR_OUTPUT_MAX_US         (2000U)

typedef enum
{
    MOTOR_OUTPUT_STATUS_OK = 0,
    MOTOR_OUTPUT_STATUS_NOT_READY,
    MOTOR_OUTPUT_STATUS_ARGUMENT_ERROR,
    MOTOR_OUTPUT_STATUS_FSP_ERROR
} motor_output_status_t;

/* 初始化 GPT5/GPT10/GPT6，并以 1000 us 安全脉宽启动四路输出。 */
motor_output_status_t motor_output_init(void);

/* motor_index 为 0..3，分别对应 M1..M4。 */
motor_output_status_t motor_output_set_us(uint32_t motor_index,
                                          uint32_t pulse_us);

/* 返回最近一次成功写入该通道的脉宽；索引非法时返回 0。 */
uint32_t motor_output_get_us(uint32_t motor_index);

void motor_output_all_stop(void);
bool motor_output_is_ready(void);

#endif /* DRIVER_MOTOR_OUTPUT_H_ */
