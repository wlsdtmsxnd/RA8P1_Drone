#ifndef DRIVER_UP_TOF_PROTOCOL_H_
#define DRIVER_UP_TOF_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UP_TOF_PROTOCOL_FRAME_SIZE          (14U)
#define UP_TOF_PROTOCOL_PAYLOAD_SIZE        (10U)
#define UP_TOF_PROTOCOL_HEADER_BYTE         (0xFEU)
#define UP_TOF_PROTOCOL_LENGTH_BYTE         (0x0AU)
#define UP_TOF_PROTOCOL_TAIL_BYTE           (0x55U)
#define UP_TOF_PROTOCOL_FLOW_VALID_VALUE    (0xF5U)
#define UP_TOF_PROTOCOL_DISTANCE_INVALID    (0xFFFFU)
#define UP_TOF_PROTOCOL_DISTANCE_MIN_MM     (200U)
#define UP_TOF_PROTOCOL_DISTANCE_MAX_MM     (20000U)

typedef enum
{
    UP_TOF_PROTOCOL_DECODE_OK = 0,
    UP_TOF_PROTOCOL_DECODE_ARGUMENT_ERROR,
    UP_TOF_PROTOCOL_DECODE_FORMAT_ERROR,
    UP_TOF_PROTOCOL_DECODE_CHECKSUM_ERROR
} up_tof_protocol_decode_status_t;

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
    bool flow_valid;
    bool tof_valid;
    bool velocity_valid;
} up_tof_protocol_sample_t;

up_tof_protocol_decode_status_t up_tof_protocol_decode_frame(
    uint8_t const * p_frame,
    size_t frame_size,
    up_tof_protocol_sample_t * p_sample);

#endif /* DRIVER_UP_TOF_PROTOCOL_H_ */
