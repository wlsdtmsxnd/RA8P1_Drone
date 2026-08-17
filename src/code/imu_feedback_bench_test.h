#ifndef CODE_IMU_FEEDBACK_BENCH_TEST_H_
#define CODE_IMU_FEEDBACK_BENCH_TEST_H_

#include <stdbool.h>

/* 500 Hz 调用；仅验证 Roll/Pitch 回水平角度反馈，不含角速度内环。 */
void imu_feedback_bench_test_update(bool imu_healthy);

#endif /* CODE_IMU_FEEDBACK_BENCH_TEST_H_ */
