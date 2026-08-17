#include "esc_bench_test.h"

#include "project_config.h"
#include "../driver/motor_output.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define ESC_CALIBRATION_HIGH_TIME_MS     (5000U)
#define MOTOR_TEST_STARTUP_TIME_MS       (10000U)
#define MOTOR_TEST_RUN_TIME_MS           (3000U)
#define MOTOR_TEST_GAP_TIME_MS           (2000U)
#define MOTOR_TEST_PULSE_US              (1200U)

static TickType_t g_bench_start_tick;
static uint32_t g_active_motor;


#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
static void motor_output_all_set_us(uint32_t pulse_us)
{
    uint32_t motor_index;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        (void) motor_output_set_us(motor_index,
                                   pulse_us);
    }
}
#endif


void esc_bench_test_init(void)
{
    g_bench_start_tick = xTaskGetTickCount();
    g_active_motor = MOTOR_OUTPUT_COUNT;

#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
    motor_output_all_set_us(MOTOR_OUTPUT_MAX_US);
#else
    motor_output_all_stop();
#endif
}


void esc_bench_test_update(void)
{
#if (ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED)
    motor_output_all_stop();
#else
    uint32_t elapsed_ms;

    elapsed_ms =
        (uint32_t) ((xTaskGetTickCount() - g_bench_start_tick) *
                    portTICK_PERIOD_MS);

#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
    if (elapsed_ms >= ESC_CALIBRATION_HIGH_TIME_MS)
    {
        motor_output_all_stop();
    }
#elif (ESC_BENCH_MODE == ESC_BENCH_MODE_MOTOR_SEQUENCE)
    uint32_t sequence_elapsed_ms;
    uint32_t motor_slot_ms;
    uint32_t motor_index;

    if (elapsed_ms < MOTOR_TEST_STARTUP_TIME_MS)
    {
        return;
    }

    sequence_elapsed_ms = elapsed_ms - MOTOR_TEST_STARTUP_TIME_MS;
    motor_slot_ms = MOTOR_TEST_RUN_TIME_MS + MOTOR_TEST_GAP_TIME_MS;
    motor_index = sequence_elapsed_ms / motor_slot_ms;

    if (motor_index >= MOTOR_OUTPUT_COUNT)
    {
        if (g_active_motor != MOTOR_OUTPUT_COUNT)
        {
            motor_output_all_stop();
            g_active_motor = MOTOR_OUTPUT_COUNT;
        }

        return;
    }

    if ((sequence_elapsed_ms % motor_slot_ms) <
        MOTOR_TEST_RUN_TIME_MS)
    {
        if (g_active_motor != motor_index)
        {
            motor_output_all_stop();
            (void) motor_output_set_us(motor_index,
                                       MOTOR_TEST_PULSE_US);
            g_active_motor = motor_index;
        }
    }
    else if (g_active_motor != MOTOR_OUTPUT_COUNT)
    {
        motor_output_all_stop();
        g_active_motor = MOTOR_OUTPUT_COUNT;
    }
#endif
#endif
}
