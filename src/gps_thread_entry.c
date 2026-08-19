#include "gps_thread.h"

#include "code/imu.h"
#include "code/project_config.h"
#include "driver/gps_nmea.h"
#include "driver/tpf_flow.h"
#include "driver/up_tof.h"

/* 该 FSP UART 实例当前由 telemetry 线程栈生成，但采集归本任务所有。 */
extern uart_instance_t const g_uart_flow_tof;

void gps_thread_entry(void * pvParameters)
{
    fsp_err_t err;
    bool gps_available;
    bool up_tof_available;
    TickType_t last_wake_time;
#if (TPF_FLOW_DIAGNOSTIC_ENABLED == 1U)
    TickType_t last_tpf_poll_tick;
    imu_attitude_t attitude;
#endif

    FSP_PARAMETER_NOT_USED(pvParameters);

    err = gps_init(&g_uart_gps);
    gps_available = (FSP_SUCCESS == err);

    err = up_tof_init(&g_uart_flow_tof);
    up_tof_available = (FSP_SUCCESS == err);

#if (TPF_FLOW_DIAGNOSTIC_ENABLED == 1U)
    /* 首次恢复失败也保留后续周期恢复和错误计数。 */
    (void) tpf_flow_init();
    last_tpf_poll_tick = xTaskGetTickCount();
#endif

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

#if (TPF_FLOW_DIAGNOSTIC_ENABLED == 1U)
        if ((xTaskGetTickCount() - last_tpf_poll_tick) >=
            pdMS_TO_TICKS(30U))
        {
            last_tpf_poll_tick += pdMS_TO_TICKS(30U);
            imu_get_attitude(&attitude);
            tpf_flow_poll(attitude.roll_deg,
                          attitude.pitch_deg,
                          attitude.gyro_x_dps,
                          attitude.gyro_y_dps);
        }
#endif

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2U));
    }
}
