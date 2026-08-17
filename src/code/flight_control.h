#ifndef CODE_FLIGHT_CONTROL_H_
#define CODE_FLIGHT_CONTROL_H_

#include <stdbool.h>

/* 初始化保守双环控制器。 */
void flight_control_init(void);

/* 500 Hz 调用；未解锁或保护触发时输出 1000 us 并清空控制器状态。 */
void flight_control_update(bool imu_healthy);

#endif /* CODE_FLIGHT_CONTROL_H_ */
