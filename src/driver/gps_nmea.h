#ifndef DRIVER_GPS_NMEA_H_
#define DRIVER_GPS_NMEA_H_

#include "hal_data.h"
#include "r_uart_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

#define GPS_SIGNAL_TIMEOUT_MS          (1500U)

typedef struct
{
    int32_t latitude_deg_e7;
    int32_t longitude_deg_e7;
    int32_t altitude_cm;
    int32_t ground_speed_cm_s;
    uint16_t course_cdeg;
    uint8_t fix_quality;
    uint8_t satellites;
    uint16_t hdop_centi;
    uint32_t sentence_count;
    uint32_t checksum_error_count;
    uint32_t parse_error_count;
    uint32_t uart_error_count;
    uint32_t rx_overflow_count;
    TickType_t last_fix_tick;
    bool valid;
} gps_data_t;

extern volatile gps_data_t g_gps_data;

void gps_uart_callback(uart_callback_args_t * p_args);
fsp_err_t gps_init(uart_instance_t const * p_uart_instance);
void gps_process(void);
void gps_get_data(gps_data_t * p_data);
bool gps_is_ready(void);

#endif /* DRIVER_GPS_NMEA_H_ */
