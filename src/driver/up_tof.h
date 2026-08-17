#ifndef DRIVER_UP_TOF_H_
#define DRIVER_UP_TOF_H_

#include "hal_data.h"
#include "r_uart_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

#define UP_TOF_SIGNAL_TIMEOUT_MS       (200U)
#define UP_TOF_FLOW_VALID_VALUE        (0xF5U)

typedef struct
{
    int16_t flow_x_integral;
    int16_t flow_y_integral;
    uint16_t integration_us;
    uint16_t distance_mm;
    uint8_t flow_valid_raw;
    uint8_t tof_confidence;
    int32_t displacement_x_mm;
    int32_t displacement_y_mm;
    int32_t velocity_x_cm_s;
    int32_t velocity_y_cm_s;
    uint32_t frame_count;
    uint32_t checksum_error_count;
    uint32_t parse_error_count;
    uint32_t uart_error_count;
    uint32_t rx_overflow_count;
    TickType_t last_frame_tick;
    bool flow_valid;
    bool valid;
} up_tof_data_t;

extern volatile up_tof_data_t g_up_tof_data;

void up_tof_uart_callback(uart_callback_args_t * p_args);
fsp_err_t up_tof_init(uart_instance_t const * p_uart_instance);
void up_tof_process(void);
void up_tof_get_data(up_tof_data_t * p_data);
bool up_tof_is_ready(void);

#endif /* DRIVER_UP_TOF_H_ */
