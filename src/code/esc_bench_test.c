#include "esc_bench_test.h"

#include "actuator_manager.h"
#include "project_config.h"
#include "rc_command.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define ESC_CALIBRATION_HIGH_TIME_MS     (5000U)
#define MOTOR_TEST_START_DELAY_MS        (3000U)
#define MOTOR_TEST_RUN_TIME_MS           (3000U)
#define MOTOR_TEST_GAP_TIME_MS           (2000U)
#define MOTOR_TEST_PULSE_US              (1200U)

static TickType_t g_bench_start_tick;
static volatile esc_bench_phase_t g_bench_phase =
    ESC_BENCH_PHASE_DISABLED;
static volatile uint32_t g_active_motor;
static volatile uint32_t g_elapsed_ms;
#if (ESC_BENCH_MODE == ESC_BENCH_MODE_MOTOR_SEQUENCE)
static TickType_t g_sequence_start_tick;
static bool g_seen_arm_switch_low;
static bool g_sequence_started;
#endif


#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
static bool esc_bench_all_set_us(uint32_t pulse_us)
{
    uint32_t output_us[ACTUATOR_MANAGER_COUNT];
    uint32_t motor_index;

    for (motor_index = 0U;
         motor_index < ACTUATOR_MANAGER_COUNT;
         motor_index++)
    {
        output_us[motor_index] = pulse_us;
    }

    return ACTUATOR_MANAGER_STATUS_OK ==
           actuator_manager_apply_us(output_us);
}
#endif


#if (ESC_BENCH_MODE != ESC_BENCH_MODE_DISABLED)
static void esc_bench_output_error(void)
{
    (void) actuator_manager_stop();
    g_active_motor = 0U;
    g_bench_phase = ESC_BENCH_PHASE_OUTPUT_ERROR;
}
#endif


void esc_bench_test_init(void)
{
    g_bench_start_tick = xTaskGetTickCount();
    g_active_motor = 0U;
    g_elapsed_ms = 0U;

#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
    if (true == esc_bench_all_set_us(ACTUATOR_MANAGER_MAX_US))
    {
        g_bench_phase = ESC_BENCH_PHASE_CALIBRATION_HIGH;
    }
    else
    {
        esc_bench_output_error();
    }
#elif (ESC_BENCH_MODE == ESC_BENCH_MODE_MOTOR_SEQUENCE)
    (void) actuator_manager_stop();
    g_sequence_start_tick = 0U;
    g_seen_arm_switch_low = false;
    g_sequence_started = false;
    g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_LOW;
#else
    (void) actuator_manager_stop();
    g_bench_phase = ESC_BENCH_PHASE_DISABLED;
#endif
}


void esc_bench_test_update(void)
{
#if (ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED)
    (void) actuator_manager_stop();
    g_active_motor = 0U;
    g_elapsed_ms = 0U;
    g_bench_phase = ESC_BENCH_PHASE_DISABLED;
#else
    uint32_t elapsed_ms;

    elapsed_ms =
        (uint32_t) ((xTaskGetTickCount() - g_bench_start_tick) *
                    portTICK_PERIOD_MS);
    g_elapsed_ms = elapsed_ms;

    if (ESC_BENCH_PHASE_OUTPUT_ERROR == g_bench_phase)
    {
        (void) actuator_manager_stop();
        return;
    }

#if (ESC_BENCH_MODE == ESC_BENCH_MODE_CALIBRATION)
    if ((elapsed_ms >= ESC_CALIBRATION_HIGH_TIME_MS) &&
        (ESC_BENCH_PHASE_CALIBRATION_HIGH == g_bench_phase))
    {
        if (true == esc_bench_all_set_us(ACTUATOR_MANAGER_MIN_US))
        {
            g_bench_phase = ESC_BENCH_PHASE_CALIBRATION_LOW;
        }
        else
        {
            esc_bench_output_error();
        }
    }
#elif (ESC_BENCH_MODE == ESC_BENCH_MODE_MOTOR_SEQUENCE)
    rc_command_t command;
    uint32_t sequence_elapsed_ms;
    uint32_t motor_slot_ms;
    uint32_t motor_index;
    uint32_t output_us[ACTUATOR_MANAGER_COUNT];

    rc_command_get(&command);

    if (false == command.connected)
    {
        (void) actuator_manager_stop();
        g_active_motor = 0U;
        g_seen_arm_switch_low = false;
        g_sequence_started = false;
        g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_LOW;
        return;
    }

    if (ESC_BENCH_PHASE_COMPLETE == g_bench_phase)
    {
        if (true == command.arm_switch_low)
        {
            g_seen_arm_switch_low = true;
            g_sequence_started = false;
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_ARM;
        }

        return;
    }

    if (false == g_sequence_started)
    {
        (void) actuator_manager_stop();
        g_active_motor = 0U;

        if (true == command.arm_switch_low)
        {
            g_seen_arm_switch_low = true;
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_ARM;
        }
        else if ((true == g_seen_arm_switch_low) &&
                 (true == command.arm_switch_high) &&
                 (true == command.throttle_low))
        {
            g_sequence_start_tick = xTaskGetTickCount();
            g_sequence_started = true;
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_COUNTDOWN;
        }
        else if (false == g_seen_arm_switch_low)
        {
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_LOW;
        }

        return;
    }

    if ((false == command.arm_switch_high) ||
        (false == command.throttle_low))
    {
        (void) actuator_manager_stop();
        g_active_motor = 0U;
        g_sequence_started = false;

        if (true == command.arm_switch_low)
        {
            g_seen_arm_switch_low = true;
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_ARM;
        }
        else
        {
            g_seen_arm_switch_low = false;
            g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_WAIT_LOW;
        }

        return;
    }

    sequence_elapsed_ms =
        (uint32_t) ((xTaskGetTickCount() - g_sequence_start_tick) *
                    portTICK_PERIOD_MS);

    if (sequence_elapsed_ms < MOTOR_TEST_START_DELAY_MS)
    {
        g_active_motor = 0U;
        g_bench_phase = ESC_BENCH_PHASE_SEQUENCE_COUNTDOWN;
        return;
    }

    sequence_elapsed_ms -= MOTOR_TEST_START_DELAY_MS;
    motor_slot_ms = MOTOR_TEST_RUN_TIME_MS + MOTOR_TEST_GAP_TIME_MS;
    motor_index = sequence_elapsed_ms / motor_slot_ms;

    if (motor_index >= ACTUATOR_MANAGER_COUNT)
    {
        if (ESC_BENCH_PHASE_COMPLETE != g_bench_phase)
        {
            (void) actuator_manager_stop();
            g_active_motor = 0U;
            g_bench_phase = ESC_BENCH_PHASE_COMPLETE;
        }

        return;
    }

    if ((sequence_elapsed_ms % motor_slot_ms) <
        MOTOR_TEST_RUN_TIME_MS)
    {
        if ((g_active_motor != (motor_index + 1U)) ||
            (ESC_BENCH_PHASE_MOTOR_RUNNING != g_bench_phase))
        {
            for (uint32_t output_index = 0U;
                 output_index < ACTUATOR_MANAGER_COUNT;
                 output_index++)
            {
                output_us[output_index] = ACTUATOR_MANAGER_MIN_US;
            }
            output_us[motor_index] = MOTOR_TEST_PULSE_US;

            if (ACTUATOR_MANAGER_STATUS_OK ==
                actuator_manager_apply_us(output_us))
            {
                g_active_motor = motor_index + 1U;
                g_bench_phase = ESC_BENCH_PHASE_MOTOR_RUNNING;
            }
            else
            {
                esc_bench_output_error();
            }
        }
    }
    else if (ESC_BENCH_PHASE_MOTOR_GAP != g_bench_phase)
    {
        (void) actuator_manager_stop();
        g_active_motor = 0U;
        g_bench_phase = ESC_BENCH_PHASE_MOTOR_GAP;
    }
#endif
#endif
}


void esc_bench_test_get_status(esc_bench_status_t * p_status)
{
    if (NULL == p_status)
    {
        return;
    }

    p_status->phase = g_bench_phase;
    p_status->active_motor = g_active_motor;
    p_status->elapsed_ms = g_elapsed_ms;
}
