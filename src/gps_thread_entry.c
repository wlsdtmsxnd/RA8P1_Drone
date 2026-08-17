#include "gps_thread.h"

#include "driver/gps_nmea.h"

void gps_thread_entry(void * pvParameters)
{
    fsp_err_t err;

    FSP_PARAMETER_NOT_USED(pvParameters);

    err = gps_init(&g_uart_gps);

    if (FSP_SUCCESS != err)
    {
        vTaskSuspend(NULL);
    }

    while (1)
    {
        gps_process();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}
