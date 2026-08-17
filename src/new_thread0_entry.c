#include "new_thread0.h"

#define LED_PIN    BSP_IO_PORT_01_PIN_10

void new_thread0_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);

    fsp_err_t err;
    bsp_io_level_t led_level = BSP_IO_LEVEL_HIGH;

    /*
     * 初始化 IOPORT，并应用 Pins 页面生成的配置。
     */
    err = R_IOPORT_Open(&g_ioport_ctrl, &g_bsp_pin_cfg);

    if ((FSP_SUCCESS != err) &&
        (FSP_ERR_ALREADY_OPEN != err))
    {
        /*
         * 初始化失败，挂起当前任务。
         */
        vTaskSuspend(NULL);
    }

    while (1)
    {
        err = R_IOPORT_PinWrite(&g_ioport_ctrl,
                                LED_PIN,
                                led_level);

        if (FSP_SUCCESS != err)
        {
            vTaskSuspend(NULL);
        }

        /*
         * 计算下一次输出电平。
         */
        if (BSP_IO_LEVEL_HIGH == led_level)
        {
            led_level = BSP_IO_LEVEL_LOW;
        }
        else
        {
            led_level = BSP_IO_LEVEL_HIGH;
        }

        /*
         * 当前任务阻塞 500 ms，让其他任务运行。
         */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
