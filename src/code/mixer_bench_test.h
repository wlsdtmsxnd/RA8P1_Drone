#ifndef CODE_MIXER_BENCH_TEST_H_
#define CODE_MIXER_BENCH_TEST_H_

#include <stdbool.h>

/* 500 Hz 调用；仅输出纯摇杆混控，不使用姿态或陀螺反馈。 */
void mixer_bench_test_update(bool imu_healthy);

#endif /* CODE_MIXER_BENCH_TEST_H_ */
