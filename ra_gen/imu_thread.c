/* generated thread source file - do not edit */
#include "imu_thread.h"

#if 1
                static StaticTask_t imu_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t imu_thread_stack[4096] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t imu_thread_stack[4096] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.imu_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
                TaskHandle_t imu_thread;
                void imu_thread_create(void);
                static void imu_thread_func(void * pvParameters);
                void rtos_startup_err_callback(void * p_instance, void * p_data);
                void rtos_startup_common_init(void);
#define RA_NOT_DEFINED (UINT32_MAX)
#if (RA_NOT_DEFINED) != (RA_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void spi_b_tx_dmac_callback(spi_b_instance_ctrl_t const * const p_ctrl);

void g_spi_imu_tx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    spi_b_tx_dmac_callback(&g_spi_imu_ctrl);
}
#endif

#if (RA_NOT_DEFINED) != (RA_NOT_DEFINED)

/* If the transfer module is DMAC, define a DMAC transfer callback. */
#include "r_dmac.h"
extern void spi_b_rx_dmac_callback(spi_b_instance_ctrl_t const * const p_ctrl);

void g_spi_imu_rx_transfer_callback (dmac_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    spi_b_rx_dmac_callback(&g_spi_imu_ctrl);
}
#endif
#undef RA_NOT_DEFINED

spi_b_instance_ctrl_t g_spi_imu_ctrl;

/** SPI extended configuration for SPI HAL driver */
const spi_b_extended_cfg_t g_spi_imu_ext_cfg =
{
    .spi_clksyn         = SPI_B_SSL_MODE_CLK_SYN,
    .spi_comm           = SPI_B_COMMUNICATION_FULL_DUPLEX,
    .ssl_polarity        = SPI_B_SSLP_LOW,
    .ssl_select          = SPI_B_SSL_SELECT_SSL0,
    .mosi_idle           = SPI_B_MOSI_IDLE_VALUE_FIXING_DISABLE,
    .parity              = SPI_B_PARITY_MODE_DISABLE,
    .byte_swap           = SPI_B_BYTE_SWAP_DISABLE,
    .clock_source        = SPI_B_CLOCK_SOURCE_PCLK,
    .spck_div            = {
        /* Actual calculated bitrate: 7812500. */ .spbr = 7, .brdv = 0
    },
    .spck_delay          = SPI_B_DELAY_COUNT_1,
    .ssl_negation_delay  = SPI_B_DELAY_COUNT_1,
    .next_access_delay   = SPI_B_DELAY_COUNT_1,
    .burst_interframe_delay = SPI_B_BURST_TRANSFER_WITH_DELAY

 };

/** SPI configuration for SPI HAL driver */
const spi_cfg_t g_spi_imu_cfg =
{
    .channel             = 1,

#if defined(VECTOR_NUMBER_SPI1_RXI)
    .rxi_irq             = VECTOR_NUMBER_SPI1_RXI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_TXI)
    .txi_irq             = VECTOR_NUMBER_SPI1_TXI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_TEI)
    .tei_irq             = VECTOR_NUMBER_SPI1_TEI,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SPI1_ERI)
    .eri_irq             = VECTOR_NUMBER_SPI1_ERI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif

    .rxi_ipl             = (12),
    .txi_ipl             = (12),
    .tei_ipl             = (12),
    .eri_ipl             = (12),

    .operating_mode      = SPI_MODE_MASTER,

    .clk_phase           = SPI_CLK_PHASE_EDGE_ODD,
    .clk_polarity        = SPI_CLK_POLARITY_LOW,

    .mode_fault          = SPI_MODE_FAULT_ERROR_DISABLE,
    .bit_order           = SPI_BIT_ORDER_MSB_FIRST,
    .p_transfer_tx       = g_spi_imu_P_TRANSFER_TX,
    .p_transfer_rx       = g_spi_imu_P_TRANSFER_RX,
    .p_callback          = spi_imu_callback,

    .p_context           = NULL,
    .p_extend            = (void *)&g_spi_imu_ext_cfg,
};

/* Instance structure to use this module. */
const spi_instance_t g_spi_imu =
{
    .p_ctrl        = &g_spi_imu_ctrl,
    .p_cfg         = &g_spi_imu_cfg,
    .p_api         = &g_spi_on_spi_b
};
extern uint32_t g_fsp_common_thread_count;

                const rm_freertos_port_parameters_t imu_thread_parameters =
                {
                    .p_context = (void *) NULL,
                };

                void imu_thread_create (void)
                {
                    /* Increment count so we will know the number of threads created in the RA Configuration editor. */
                    g_fsp_common_thread_count++;

                    /* Initialize each kernel object. */
                    

                    #if 1
                    imu_thread = xTaskCreateStatic(
                    #else
                    BaseType_t imu_thread_create_err = xTaskCreate(
                    #endif
                        imu_thread_func,
                        (const char *)"IMU Thread",
                        4096/4, // In words, not bytes
                        (void *) &imu_thread_parameters, //pvParameters
                        8,
                        #if 1
                        (StackType_t *)&imu_thread_stack,
                        (StaticTask_t *)&imu_thread_memory
                        #else
                        & imu_thread
                        #endif
                    );

                    #if 1
                    if (NULL == imu_thread)
                    {
                        rtos_startup_err_callback(imu_thread, 0);
                    }
                    #else
                    if (pdPASS != imu_thread_create_err)
                    {
                        rtos_startup_err_callback(imu_thread, 0);
                    }
                    #endif
                }
                static void imu_thread_func (void * pvParameters)
                {
                    /* Initialize common components */
                    rtos_startup_common_init();

                    /* Initialize each module instance. */
                    

                    #if (1 == BSP_TZ_NONSECURE_BUILD) && (1 == 1)
                    /* When FreeRTOS is used in a non-secure TrustZone application, portALLOCATE_SECURE_CONTEXT must be called prior
                     * to calling any non-secure callable function in a thread. The parameter is unused in the FSP implementation.
                     * If no slots are available then configASSERT() will be called from vPortSVCHandler_C(). If this occurs, the
                     * application will need to either increase the value of the "Process Stack Slots" Property in the rm_tz_context
                     * module in the secure project or decrease the number of threads in the non-secure project that are allocating
                     * a secure context. Users can control which threads allocate a secure context via the Properties tab when
                     * selecting each thread. Note that the idle thread in FreeRTOS requires a secure context so the application
                     * will need at least 1 secure context even if no user threads make secure calls. */
                     portALLOCATE_SECURE_CONTEXT(0);
                    #endif

                    /* Enter user code for this thread. Pass task handle. */
                    imu_thread_entry(pvParameters);
                }
