#include "radio_3dr.h"

/* 数传接收环形缓冲区大小。 */
#define RADIO_3DR_RX_BUFFER_SIZE       (256U)

/* 当前使用的 UART 实例。 */
static uart_instance_t const * g_radio_uart = NULL;

/* 当前等待发送完成的任务。 */
static volatile TaskHandle_t g_radio_tx_wait_task = NULL;

/* 接收环形缓冲区。 */
static uint8_t g_radio_rx_buffer[RADIO_3DR_RX_BUFFER_SIZE];

/* 接收缓冲区写入位置。 */
static volatile uint16_t g_radio_rx_head = 0U;

/* 接收缓冲区读取位置。 */
static volatile uint16_t g_radio_rx_tail = 0U;

/* 接收缓冲区溢出计数。 */
static volatile uint32_t g_radio_rx_overflow_count = 0U;


/* FSP UART 中断回调。 */
void radio_3dr_callback(uart_callback_args_t * p_args)
{
    BaseType_t higher_priority_task_woken = pdFALSE;         /* 是否需要立即切换任务。 */
    TaskHandle_t tx_wait_task;                               /* 等待发送完成的任务。 */
    uint16_t next_head;                                      /* 接收缓冲区下一个写入位置。 */

    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
        {
            tx_wait_task = (TaskHandle_t) g_radio_tx_wait_task;

            if (NULL != tx_wait_task)
            {
                vTaskNotifyGiveFromISR(tx_wait_task,
                                       &higher_priority_task_woken);

                portYIELD_FROM_ISR(higher_priority_task_woken);
            }

            break;
        }

        case UART_EVENT_RX_CHAR:
        {
            next_head = (uint16_t) ((g_radio_rx_head + 1U) %
                                    RADIO_3DR_RX_BUFFER_SIZE);

            if (next_head != g_radio_rx_tail)
            {
                g_radio_rx_buffer[g_radio_rx_head] =
                    (uint8_t) p_args->data;

                g_radio_rx_head = next_head;
            }
            else
            {
                /*
                 * 缓冲区已满时丢弃最新字节。
                 * 正式协议应在上层增加帧校验和超时恢复。
                 */
                g_radio_rx_overflow_count++;
            }

            break;
        }

        default:
        {
            /*
             * UART 溢出、帧错误和校验错误可在后续故障管理中统计。
             */
            break;
        }
    }
}


/* 初始化数传 UART。 */
radio_3dr_status_t radio_3dr_init(uart_instance_t const * p_uart_instance)
{
    fsp_err_t err;    /* FSP UART 返回值。 */

    if (NULL == p_uart_instance)
    {
        return RADIO_3DR_STATUS_ARGUMENT_ERROR;
    }

    g_radio_uart = p_uart_instance;

    err = g_radio_uart->p_api->open(g_radio_uart->p_ctrl,
                                    g_radio_uart->p_cfg);

    if ((FSP_SUCCESS != err) &&
        (FSP_ERR_ALREADY_OPEN != err))
    {
        g_radio_uart = NULL;
        return RADIO_3DR_STATUS_OPEN_ERROR;
    }

    return RADIO_3DR_STATUS_OK;
}


/* 发送一帧数据并等待发送完成。 */
radio_3dr_status_t radio_3dr_write(uint8_t const * p_data,
                                  uint32_t length,
                                  TickType_t timeout_ticks)
{
    fsp_err_t err;          /* FSP UART 返回值。 */
    uint32_t notified;      /* 发送完成通知计数。 */

    if ((NULL == g_radio_uart) ||
        (NULL == p_data) ||
        (0U == length))
    {
        return RADIO_3DR_STATUS_ARGUMENT_ERROR;
    }

    /*
     * 当前设计由 telemetry_thread 独占数传 UART。
     * 先登记等待任务，再启动异步发送，避免完成中断过早到达。
     */
    g_radio_tx_wait_task = xTaskGetCurrentTaskHandle();
    (void) ulTaskNotifyTake(pdTRUE, 0U);

    err = g_radio_uart->p_api->write(g_radio_uart->p_ctrl,
                                     p_data,
                                     length);

    if (FSP_SUCCESS != err)
    {
        g_radio_tx_wait_task = NULL;
        return RADIO_3DR_STATUS_WRITE_ERROR;
    }

    notified = ulTaskNotifyTake(pdTRUE,
                                timeout_ticks);

    g_radio_tx_wait_task = NULL;

    if (0U == notified)
    {
        return RADIO_3DR_STATUS_TX_TIMEOUT;
    }

    return RADIO_3DR_STATUS_OK;
}


/* 从接收环形缓冲区读取数据。 */
uint32_t radio_3dr_read(uint8_t * p_data,
                       uint32_t max_length)
{
    uint32_t read_length = 0U;    /* 本次实际读取长度。 */

    if ((NULL == p_data) || (0U == max_length))
    {
        return 0U;
    }

    taskENTER_CRITICAL();

    while ((read_length < max_length) &&
           (g_radio_rx_tail != g_radio_rx_head))
    {
        p_data[read_length] = g_radio_rx_buffer[g_radio_rx_tail];

        g_radio_rx_tail =
            (uint16_t) ((g_radio_rx_tail + 1U) %
                        RADIO_3DR_RX_BUFFER_SIZE);

        read_length++;
    }

    taskEXIT_CRITICAL();

    return read_length;
}


/* 获取接收溢出计数。 */
uint32_t radio_3dr_get_rx_overflow_count(void)
{
    return g_radio_rx_overflow_count;
}
