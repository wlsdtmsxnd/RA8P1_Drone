#ifndef CODE_ACTUATOR_MANAGER_H_
#define CODE_ACTUATOR_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

#define ACTUATOR_MANAGER_COUNT       (4U)
#define ACTUATOR_MANAGER_MIN_US      (1000U)
#define ACTUATOR_MANAGER_MAX_US      (2000U)

typedef enum
{
    ACTUATOR_MANAGER_STATUS_OK = 0,
    ACTUATOR_MANAGER_STATUS_NOT_READY,
    ACTUATOR_MANAGER_STATUS_INHIBITED,
    ACTUATOR_MANAGER_STATUS_ARGUMENT_ERROR,
    ACTUATOR_MANAGER_STATUS_RANGE_ERROR,
    ACTUATOR_MANAGER_STATUS_DRIVER_ERROR
} actuator_manager_status_t;

/* 初始化底层 PWM。除显式 ESC 台架 profile 外，初始状态始终禁止动力输出。 */
actuator_manager_status_t actuator_manager_init(void);

/* 只有统一安全状态机应调用 authorize；任何故障路径都可调用 inhibit。 */
actuator_manager_status_t actuator_manager_authorize(void);
actuator_manager_status_t actuator_manager_inhibit(void);

/* 一次提交完整四路命令，禁止各控制模块分别写单路 PWM。 */
actuator_manager_status_t actuator_manager_apply_us(
    uint32_t const output_us[ACTUATOR_MANAGER_COUNT]);

/* 写入四路停机脉宽并返回真实执行结果。 */
actuator_manager_status_t actuator_manager_stop(void);

uint32_t actuator_manager_get_us(uint32_t actuator_index);
bool actuator_manager_is_ready(void);
bool actuator_manager_is_authorized(void);
bool actuator_manager_has_latched_fault(void);

#endif /* CODE_ACTUATOR_MANAGER_H_ */
