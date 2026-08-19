#include "gps_thread.h"

#include "driver/gps_nmea.h"
#include "driver/up_tof.h"

/* 该 FSP UART 实例当前由 telemetry 线程栈生成，但采集归本任务所有。 */
extern uart_instance_t const g_uart_flow_tof;

void gps_thread_entry(void * pvParameters)
{
    fsp_err_t err;
    bool gps_available;
    bool up_tof_available;
    TickType_t last_wake_time;

    FSP_PARAMETER_NOT_USED(pvParameters);

    err = gps_init(&g_uart_gps);
    gps_available = (FSP_SUCCESS == err);

    err = up_tof_init(&g_uart_flow_tof);
    up_tof_available = (FSP_SUCCESS == err);

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (true == gps_available)
        {
            gps_process();
        }

        if (true == up_tof_available)
        {
            up_tof_process();
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2U));
    }
}
