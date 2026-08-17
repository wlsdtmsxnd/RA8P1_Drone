#include "uart_thread.h"
#include "code/imu.h"

#include <stdio.h>

/* UART9 发送完成后通知的任务句柄。 */
static TaskHandle_t g_uart_task_handle = NULL;


/* UART9 中断回调。 */
void uart9_callback(uart_callback_args_t * p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;   /* 是否立即切换任务。 */

    if ((NULL != p_args) &&
        (UART_EVENT_TX_COMPLETE == p_args->event) &&
        (NULL != g_uart_task_handle))
    {
        vTaskNotifyGiveFromISR(g_uart_task_handle,
                               &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}


/* UART 任务入口：通过板载 J-Link 串口输出三个角度。 */
void uart_thread_entry(void * pvParameters)
{
    fsp_err_t err;                       /* UART FSP 返回值。 */
    imu_attitude_t attitude;             /* 三个角度快照。 */
    static char tx_buffer[128];          /* 异步发送缓冲区。 */
    int message_length;                  /* 格式化后的字符串长度。 */
    uint32_t notified;                   /* 发送完成通知计数。 */

    int32_t roll_cdeg;                   /* Roll，单位 0.01 度。 */
    int32_t pitch_cdeg;                  /* Pitch，单位 0.01 度。 */
    int32_t yaw_cdeg;                    /* Yaw，单位 0.01 度。 */

    uint32_t roll_abs;                   /* Roll 绝对值。 */
    uint32_t pitch_abs;                  /* Pitch 绝对值。 */
    uint32_t yaw_abs;                    /* Yaw 绝对值。 */

    char roll_sign;                      /* Roll 符号。 */
    char pitch_sign;                     /* Pitch 符号。 */
    char yaw_sign;                       /* Yaw 符号。 */

    FSP_PARAMETER_NOT_USED(pvParameters);

    g_uart_task_handle = xTaskGetCurrentTaskHandle();

    err = R_SCI_B_UART_Open(&g_uart9_ctrl,
                            &g_uart9_cfg);

    if ((FSP_SUCCESS != err) &&
        (FSP_ERR_ALREADY_OPEN != err))
    {
        vTaskSuspend(NULL);
    }

    while (1)
    {
        if (imu_is_ready())
        {
            imu_get_attitude(&attitude);

            /*
             * 使用 0.01 度定点格式输出，
             * 避免 newlib-nano 未开启浮点 printf 时无法显示。
             */
            roll_cdeg = (int32_t) (attitude.roll_deg * 100.0f);
            pitch_cdeg = (int32_t) (attitude.pitch_deg * 100.0f);
            yaw_cdeg = (int32_t) (attitude.yaw_deg * 100.0f);

            roll_sign = (roll_cdeg < 0) ? '-' : '+';
            pitch_sign = (pitch_cdeg < 0) ? '-' : '+';
            yaw_sign = (yaw_cdeg < 0) ? '-' : '+';

            roll_abs =
                (uint32_t) ((roll_cdeg < 0) ? -roll_cdeg : roll_cdeg);

            pitch_abs =
                (uint32_t) ((pitch_cdeg < 0) ? -pitch_cdeg : pitch_cdeg);

            yaw_abs =
                (uint32_t) ((yaw_cdeg < 0) ? -yaw_cdeg : yaw_cdeg);

            message_length = snprintf(
                tx_buffer,
                sizeof(tx_buffer),
                "Roll=%c%lu.%02lu  Pitch=%c%lu.%02lu  Yaw=%c%lu.%02lu\r\n",
                roll_sign,
                (unsigned long) (roll_abs / 100U),
                (unsigned long) (roll_abs % 100U),
                pitch_sign,
                (unsigned long) (pitch_abs / 100U),
                (unsigned long) (pitch_abs % 100U),
                yaw_sign,
                (unsigned long) (yaw_abs / 100U),
                (unsigned long) (yaw_abs % 100U));
        }
        else
        {
            message_length = snprintf(
                tx_buffer,
                sizeof(tx_buffer),
                "ICM42688 calibrating, keep still...\r\n");
        }

        if ((message_length > 0) &&
            ((size_t) message_length < sizeof(tx_buffer)))
        {
            (void) ulTaskNotifyTake(pdTRUE, 0U);

            err = R_SCI_B_UART_Write(
                &g_uart9_ctrl,
                (uint8_t const *) tx_buffer,
                (uint32_t) message_length);

            if (FSP_SUCCESS == err)
            {
                notified = ulTaskNotifyTake(pdTRUE,
                                            pdMS_TO_TICKS(100U));

                if (0U == notified)
                {
                    /*
                     * 本次串口发送超时，直接跳过，不影响 IMU 任务。
                     */
                }
            }
        }

        /*
         * 串口显示 20 Hz。
         * 不要以 500 Hz 输出文本，否则会浪费大量 CPU 和串口带宽。
         */
        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}
