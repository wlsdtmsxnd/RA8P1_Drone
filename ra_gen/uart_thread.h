/* generated thread header file - do not edit */
#ifndef UART_THREAD_H_
#define UART_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#ifdef __cplusplus
                extern "C" void uart_thread_entry(void * pvParameters);
                #else
extern void uart_thread_entry(void *pvParameters);
#endif
#include "r_sci_b_uart.h"
#include "r_uart_api.h"
FSP_HEADER
/** UART on SCI Instance. */
extern const uart_instance_t g_uart9;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_b_uart_instance_ctrl_t g_uart9_ctrl;
extern const uart_cfg_t g_uart9_cfg;
extern const sci_b_uart_extended_cfg_t g_uart9_cfg_extend;

#ifndef uart9_callback
void uart9_callback(uart_callback_args_t *p_args);
#endif
FSP_FOOTER
#endif /* UART_THREAD_H_ */
