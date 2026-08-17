#include "rc_thread.h"

#include "driver/crsf.h"


/* ELRS/CRSF 遥控接收线程。 */
void rc_thread_entry(void * pvParameters)
{
    fsp_err_t err;    /* CRSF UART 初始化结果。 */

    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 打开接收机使用的 UART0。 */
    err = crsf_init(&g_uart_rc);

    if (FSP_SUCCESS != err)
    {
        /*
         * UART 初始化失败时暂停当前线程。
         * 调试时可以在这里打断点查看 err。
         */
        vTaskSuspend(NULL);
    }

    while (1)
    {
        /*
         * UART 中断只负责收字节，本线程负责帧同步、CRC 校验和通道解包。
         */
        crsf_process();

        /*
         * 1 ms 处理周期足以覆盖 CRSF 的高速连续数据，同时不会长期占用 CPU。
         */
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}
