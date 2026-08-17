/* generated-like thread source file */
#include "gps_thread.h"

#if 1
static StaticTask_t gps_thread_memory;
#if defined(__ARMCC_VERSION)
static uint8_t gps_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#else
static uint8_t gps_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.gps_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#endif
#endif
TaskHandle_t gps_thread;
void gps_thread_create(void);
static void gps_thread_func(void * pvParameters);
void rtos_startup_err_callback(void * p_instance, void * p_data);
void rtos_startup_common_init(void);
sci_b_uart_instance_ctrl_t g_uart_gps_ctrl;

sci_b_baud_setting_t g_uart_gps_baud_setting =
{
    .baudrate_bits_b.abcse = 0,
    .baudrate_bits_b.abcs = 0,
    .baudrate_bits_b.bgdm = 1,
    .baudrate_bits_b.cks = 0,
    .baudrate_bits_b.brr = 64,
    .baudrate_bits_b.mddr = (uint8_t) 256,
    .baudrate_bits_b.brme = false
};

const sci_b_uart_extended_cfg_t g_uart_gps_cfg_extend =
{ .clock = SCI_B_UART_CLOCK_INT, .rx_edge_start = SCI_B_UART_START_BIT_FALLING_EDGE, .noise_cancel =
          SCI_B_UART_NOISE_CANCELLATION_DISABLE,
  .rx_fifo_trigger = SCI_B_UART_RX_FIFO_TRIGGER_1, .p_baud_setting = &g_uart_gps_baud_setting, .flow_control =
          SCI_B_UART_FLOW_CONTROL_RTS,
  .flow_control_pin = (bsp_io_port_pin_t) UINT16_MAX,
  .rs485_setting =
  { .enable = SCI_B_UART_RS485_DISABLE,
    .polarity = SCI_B_UART_RS485_DE_POLARITY_HIGH,
    .assertion_time = 1,
    .negation_time = 1, },
  .delay_cycles = 0, };

const uart_cfg_t g_uart_gps_cfg =
{ .channel = 4, .data_bits = UART_DATA_BITS_8, .parity = UART_PARITY_OFF, .stop_bits = UART_STOP_BITS_1, .p_callback =
          gps_uart_callback,
  .p_context = NULL, .p_extend = &g_uart_gps_cfg_extend,
  .p_transfer_tx = NULL,
  .p_transfer_rx = NULL,
  .rxi_ipl = (12),
  .txi_ipl = (12), .tei_ipl = (12), .eri_ipl = (12),
#if defined(VECTOR_NUMBER_SCI4_RXI)
  .rxi_irq = VECTOR_NUMBER_SCI4_RXI,
#else
  .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_TXI)
  .txi_irq = VECTOR_NUMBER_SCI4_TXI,
#else
  .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_TEI)
  .tei_irq = VECTOR_NUMBER_SCI4_TEI,
#else
  .tei_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI4_ERI)
  .eri_irq = VECTOR_NUMBER_SCI4_ERI,
#else
  .eri_irq = FSP_INVALID_VECTOR,
#endif
        };

const uart_instance_t g_uart_gps =
{ .p_ctrl = &g_uart_gps_ctrl, .p_cfg = &g_uart_gps_cfg, .p_api = &g_uart_on_sci_b };
extern uint32_t g_fsp_common_thread_count;

const rm_freertos_port_parameters_t gps_thread_parameters =
{ .p_context = (void *) NULL, };

void gps_thread_create(void)
{
    g_fsp_common_thread_count++;

#if 1
    gps_thread = xTaskCreateStatic(
#else
    BaseType_t gps_thread_create_err = xTaskCreate(
#endif
        gps_thread_func,
        (const char *) "GPS",
        2048 / 4,
        (void *) &gps_thread_parameters,
        3,
#if 1
        (StackType_t *) &gps_thread_stack,
        (StaticTask_t *) &gps_thread_memory
#else
        &gps_thread
#endif
        );

#if 1
    if (NULL == gps_thread)
    {
        rtos_startup_err_callback(gps_thread, 0);
    }
#else
    if (pdPASS != gps_thread_create_err)
    {
        rtos_startup_err_callback(gps_thread, 0);
    }
#endif
}

static void gps_thread_func(void * pvParameters)
{
    rtos_startup_common_init();

#if (1 == BSP_TZ_NONSECURE_BUILD) && (1 == 1)
    portALLOCATE_SECURE_CONTEXT(0);
#endif

    gps_thread_entry(pvParameters);
}
