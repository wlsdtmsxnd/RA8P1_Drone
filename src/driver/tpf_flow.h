#ifndef DRIVER_TPF_FLOW_H_
#define DRIVER_TPF_FLOW_H_

#include "hal_data.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * RA6M5 同款 TPF/LC08 光流测距模块。
 * 软件 I2C 引脚由 project_config.h 选择，当前为 P406/P409；本驱动与
 * 现有 UART up_tof 驱动相互独立。
 */
#define TPF_FLOW_SIGNAL_TIMEOUT_MS       (200U)

typedef struct
{
    uint16_t distance_mm;
    int8_t raw_velocity_x;
    int8_t raw_velocity_y;
    uint8_t quality;
    float tilt_height_mm;
    float scaled_velocity_x_mm_s;
    float scaled_velocity_y_mm_s;
    float filtered_velocity_x_mm_s;
    float filtered_velocity_y_mm_s;
    uint32_t sample_count;
    uint32_t ack_error_count;
    uint32_t bus_error_count;
    TickType_t last_sample_tick;
    bool valid;
} tpf_flow_data_t;

extern volatile tpf_flow_data_t g_tpf_flow_data;

fsp_err_t tpf_flow_init(void);
void tpf_flow_poll(float roll_deg,
                   float pitch_deg,
                   float gyro_x_dps,
                   float gyro_y_dps);
void tpf_flow_get_data(tpf_flow_data_t * p_data);
bool tpf_flow_is_ready(void);

#endif /* DRIVER_TPF_FLOW_H_ */
