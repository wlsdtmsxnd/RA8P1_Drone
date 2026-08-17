#ifndef CODE_ESC_BENCH_TEST_H_
#define CODE_ESC_BENCH_TEST_H_

#include <stdint.h>

typedef enum
{
    ESC_BENCH_PHASE_DISABLED = 0,
    ESC_BENCH_PHASE_SEQUENCE_WAIT_LOW,
    ESC_BENCH_PHASE_SEQUENCE_WAIT_ARM,
    ESC_BENCH_PHASE_SEQUENCE_COUNTDOWN,
    ESC_BENCH_PHASE_MOTOR_RUNNING,
    ESC_BENCH_PHASE_MOTOR_GAP,
    ESC_BENCH_PHASE_COMPLETE,
    ESC_BENCH_PHASE_OUTPUT_ERROR,
    ESC_BENCH_PHASE_CALIBRATION_HIGH,
    ESC_BENCH_PHASE_CALIBRATION_LOW
} esc_bench_phase_t;

typedef struct
{
    esc_bench_phase_t phase;
    uint32_t active_motor; /* 0 表示无，1..4 表示 M1..M4。 */
    uint32_t elapsed_ms;
} esc_bench_status_t;

void esc_bench_test_init(void);
void esc_bench_test_update(void);
void esc_bench_test_get_status(esc_bench_status_t * p_status);

#endif /* CODE_ESC_BENCH_TEST_H_ */
