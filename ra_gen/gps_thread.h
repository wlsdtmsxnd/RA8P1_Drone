/* generated-like thread header file */
#ifndef GPS_THREAD_H_
#define GPS_THREAD_H_
#include "bsp_api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hal_data.h"
#ifdef __cplusplus
extern "C" void gps_thread_entry(void * pvParameters);
#else
extern void gps_thread_entry(void * pvParameters);
#endif
#include "r_sci_b_uart.h"
#include "r_uart_api.h"
FSP_HEADER
extern const uart_instance_t g_uart_gps;
extern sci_b_uart_instance_ctrl_t g_uart_gps_ctrl;
extern const uart_cfg_t g_uart_gps_cfg;
extern const sci_b_uart_extended_cfg_t g_uart_gps_cfg_extend;

#ifndef gps_uart_callback
void gps_uart_callback(uart_callback_args_t * p_args);
#endif
FSP_FOOTER
#endif /* GPS_THREAD_H_ */
