#include "actuator_manager.h"

#include "project_config.h"
#include "../driver/motor_output.h"

#include <stddef.h>

_Static_assert(ACTUATOR_MANAGER_COUNT == MOTOR_OUTPUT_COUNT,
               "Actuator manager and motor driver counts must match");
_Static_assert(ACTUATOR_MANAGER_MIN_US == MOTOR_OUTPUT_MIN_US,
               "Actuator manager and motor driver minima must match");
_Static_assert(ACTUATOR_MANAGER_MAX_US == MOTOR_OUTPUT_MAX_US,
               "Actuator manager and motor driver maxima must match");

static bool g_actuator_ready = false;
static bool g_actuator_authorized = false;
static bool g_actuator_fault_latched = false;


static actuator_manager_status_t actuator_manager_map_driver_status(
    motor_output_status_t status)
{
    switch (status)
    {
        case MOTOR_OUTPUT_STATUS_OK:
            return ACTUATOR_MANAGER_STATUS_OK;

        case MOTOR_OUTPUT_STATUS_NOT_READY:
            return ACTUATOR_MANAGER_STATUS_NOT_READY;

        case MOTOR_OUTPUT_STATUS_ARGUMENT_ERROR:
            return ACTUATOR_MANAGER_STATUS_ARGUMENT_ERROR;

        case MOTOR_OUTPUT_STATUS_FSP_ERROR:
        default:
            return ACTUATOR_MANAGER_STATUS_DRIVER_ERROR;
    }
}


actuator_manager_status_t actuator_manager_init(void)
{
    motor_output_status_t driver_status;

    g_actuator_ready = false;
    g_actuator_authorized = false;
    g_actuator_fault_latched = false;

    driver_status = motor_output_init();

    if (MOTOR_OUTPUT_STATUS_OK != driver_status)
    {
        g_actuator_fault_latched = true;
        return actuator_manager_map_driver_status(driver_status);
    }

    g_actuator_ready = true;

#if (ESC_BENCH_MODE != ESC_BENCH_MODE_DISABLED)
    /* ESC 台架模式使用自己的物理触发状态机，且已由编译期确认位保护。 */
    g_actuator_authorized = true;
#endif

    return ACTUATOR_MANAGER_STATUS_OK;
}


actuator_manager_status_t actuator_manager_authorize(void)
{
    if (false == g_actuator_ready)
    {
        return ACTUATOR_MANAGER_STATUS_NOT_READY;
    }

    if (true == g_actuator_fault_latched)
    {
        return ACTUATOR_MANAGER_STATUS_DRIVER_ERROR;
    }

    g_actuator_authorized = true;
    return ACTUATOR_MANAGER_STATUS_OK;
}


actuator_manager_status_t actuator_manager_stop(void)
{
    motor_output_status_t driver_status;

    if (false == g_actuator_ready)
    {
        return ACTUATOR_MANAGER_STATUS_NOT_READY;
    }

    driver_status = motor_output_all_stop();

    if (MOTOR_OUTPUT_STATUS_OK != driver_status)
    {
        g_actuator_fault_latched = true;
    }

    return actuator_manager_map_driver_status(driver_status);
}


actuator_manager_status_t actuator_manager_inhibit(void)
{
    g_actuator_authorized = false;
    return actuator_manager_stop();
}


actuator_manager_status_t actuator_manager_apply_us(
    uint32_t const output_us[ACTUATOR_MANAGER_COUNT])
{
    uint32_t actuator_index;
    motor_output_status_t driver_status;

    if (NULL == output_us)
    {
        return ACTUATOR_MANAGER_STATUS_ARGUMENT_ERROR;
    }

    if (false == g_actuator_ready)
    {
        return ACTUATOR_MANAGER_STATUS_NOT_READY;
    }

    if ((false == g_actuator_authorized) ||
        (true == g_actuator_fault_latched))
    {
        (void) actuator_manager_stop();
        return (true == g_actuator_fault_latched) ?
               ACTUATOR_MANAGER_STATUS_DRIVER_ERROR :
               ACTUATOR_MANAGER_STATUS_INHIBITED;
    }

    for (actuator_index = 0U;
         actuator_index < ACTUATOR_MANAGER_COUNT;
         actuator_index++)
    {
        if ((output_us[actuator_index] < ACTUATOR_MANAGER_MIN_US) ||
            (output_us[actuator_index] > ACTUATOR_MANAGER_MAX_US))
        {
            g_actuator_authorized = false;
            (void) actuator_manager_stop();
            return ACTUATOR_MANAGER_STATUS_RANGE_ERROR;
        }
    }

    for (actuator_index = 0U;
         actuator_index < ACTUATOR_MANAGER_COUNT;
         actuator_index++)
    {
        driver_status = motor_output_set_us(actuator_index,
                                            output_us[actuator_index]);

        if (MOTOR_OUTPUT_STATUS_OK != driver_status)
        {
            g_actuator_authorized = false;
            g_actuator_fault_latched = true;
            (void) motor_output_all_stop();
            return actuator_manager_map_driver_status(driver_status);
        }
    }

    return ACTUATOR_MANAGER_STATUS_OK;
}


uint32_t actuator_manager_get_us(uint32_t actuator_index)
{
    return motor_output_get_us(actuator_index);
}


bool actuator_manager_is_ready(void)
{
    return g_actuator_ready && motor_output_is_ready();
}


bool actuator_manager_is_authorized(void)
{
    return g_actuator_authorized;
}


bool actuator_manager_has_latched_fault(void)
{
    return g_actuator_fault_latched;
}
