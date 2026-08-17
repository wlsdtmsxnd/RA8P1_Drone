#ifndef CODE_FLIGHT_SAFETY_H_
#define CODE_FLIGHT_SAFETY_H_

#include <stdbool.h>

typedef enum
{
    FLIGHT_SAFETY_DISARMED = 0,
    FLIGHT_SAFETY_ARMING_WAIT = 1,
    FLIGHT_SAFETY_ARMED = 2,
    FLIGHT_SAFETY_FAILSAFE = 3
} flight_safety_state_t;

void flight_safety_init(void);

/* 每个 IMU 周期调用；IMU 读失败会立即进入 FAILSAFE。 */
void flight_safety_update(bool imu_healthy);

flight_safety_state_t flight_safety_get_state(void);
bool flight_safety_is_armed(void);

/* 调试器可直接观察，量产逻辑仍应通过访问函数读取。 */
extern volatile flight_safety_state_t g_flight_safety_state;

#endif /* CODE_FLIGHT_SAFETY_H_ */
