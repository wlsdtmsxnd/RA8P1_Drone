#ifndef CODE_FLOW_NAVIGATION_H_
#define CODE_FLOW_NAVIGATION_H_

#include "flow_navigation_core.h"
#include "imu.h"

#include <stdbool.h>

void flow_navigation_update(const imu_attitude_t * p_attitude,
                            bool imu_healthy,
                            float dt_s);
void flow_navigation_get_state(flow_navigation_state_t * p_state);
void flow_navigation_reset_position(void);

#endif /* CODE_FLOW_NAVIGATION_H_ */
