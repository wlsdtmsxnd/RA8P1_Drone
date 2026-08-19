/* generated thread header file - do not edit */
#ifndef IMU_THREAD_H_
#define IMU_THREAD_H_
#include "bsp_api.h"
                #include "FreeRTOS.h"
                #include "task.h"
                #include "semphr.h"
                #include "hal_data.h"
                #ifdef __cplusplus
                extern "C" void imu_thread_entry(void * pvParameters);
                #else
                extern void imu_thread_entry(void * pvParameters);
                #endif
#include "r_spi_b.h"
FSP_HEADER
/** SPI on SPI Instance. */
extern const spi_instance_t g_spi_imu;

/** Access the SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern spi_b_instance_ctrl_t g_spi_imu_ctrl;
extern const spi_cfg_t g_spi_imu_cfg;

/** Callback used by SPI Instance. */
#ifndef spi_imu_callback
void spi_imu_callback(spi_callback_args_t * p_args);
#endif


#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
    #define g_spi_imu_P_TRANSFER_TX (NULL)
#else
    #define g_spi_imu_P_TRANSFER_TX (&RA_NOT_DEFINED)
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
    #define g_spi_imu_P_TRANSFER_RX (NULL)
#else
    #define g_spi_imu_P_TRANSFER_RX (&RA_NOT_DEFINED)
#endif
#undef RA_NOT_DEFINED
FSP_FOOTER
#endif /* IMU_THREAD_H_ */
