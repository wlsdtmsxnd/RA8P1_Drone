#ifndef DRIVER_RADIO_3DR_H_
#define DRIVER_RADIO_3DR_H_

#include "hal_data.h"
#include "r_uart_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

/* 3DR 数传驱动状态。 */
typedef enum
{
    RADIO_3DR_STATUS_OK = 0,
    RADIO_3DR_STATUS_ARGUMENT_ERROR,
    RADIO_3DR_STATUS_OPEN_ERROR,
    RADIO_3DR_STATUS_WRITE_ERROR,
    RADIO_3DR_STATUS_TX_TIMEOUT
} radio_3dr_status_t;

/*
 * FSP UART 回调。
 * g_uart_radio 的 Callback 必须填写 radio_3dr_callback。
 */
void radio_3dr_callback(uart_callback_args_t * p_args);

/* 初始化数传 UART。 */
radio_3dr_status_t radio_3dr_init(uart_instance_t const * p_uart_instance);

/*
 * 阻塞当前任务，等待一帧 UART 数据发送完成。
 * 只能由数传任务调用，避免多个任务同时访问同一个 UART。
 */
radio_3dr_status_t radio_3dr_write(uint8_t const * p_data,
                                  uint32_t length,
                                  TickType_t timeout_ticks);

/* 从接收环形缓冲区读取已有数据，返回实际读取字节数。 */
uint32_t radio_3dr_read(uint8_t * p_data,
                       uint32_t max_length);

/* 获取因缓冲区满而丢弃的接收字节数。 */
uint32_t radio_3dr_get_rx_overflow_count(void);

#endif /* DRIVER_RADIO_3DR_H_ */
