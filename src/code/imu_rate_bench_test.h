#ifndef CODE_IMU_RATE_BENCH_TEST_H_
#define CODE_IMU_RATE_BENCH_TEST_H_

#include <stdbool.h>

/* 500 Hz 调用；仅验证三轴角速度阻尼，不含角度环。 */
void imu_rate_bench_test_update(bool imu_healthy);

#endif /* CODE_IMU_RATE_BENCH_TEST_H_ */
