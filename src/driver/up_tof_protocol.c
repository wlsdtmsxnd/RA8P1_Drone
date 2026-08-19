#include "up_tof_protocol.h"

#include <limits.h>

static uint16_t up_tof_protocol_read_u16_le(uint8_t const * p_data)
{
    return (uint16_t) (((uint16_t) p_data[1] << 8U) |
                       (uint16_t) p_data[0]);
}

static int16_t up_tof_protocol_read_i16_le(uint8_t const * p_data)
{
    return (int16_t) up_tof_protocol_read_u16_le(p_data);
}

static int64_t up_tof_protocol_divide_round_nearest(int64_t numerator,
                                                     int64_t denominator)
{
    if (numerator >= 0LL)
    {
        numerator += denominator / 2LL;
    }
    else
    {
        numerator -= denominator / 2LL;
    }

    return numerator / denominator;
}

static int32_t up_tof_protocol_saturate_i32(int64_t value)
{
    if (value > (int64_t) INT32_MAX)
    {
        return INT32_MAX;
    }

    if (value < (int64_t) INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t) value;
}

static int32_t up_tof_protocol_displacement_mm(int16_t flow_integral,
                                                uint16_t distance_mm)
{
    int64_t numerator = (int64_t) flow_integral * (int64_t) distance_mm;

    return (int32_t) up_tof_protocol_divide_round_nearest(numerator, 10000LL);
}

static int32_t up_tof_protocol_velocity_cm_s(int16_t flow_integral,
                                              uint16_t distance_mm,
                                              uint16_t integration_us)
{
    int64_t numerator = (int64_t) flow_integral *
                        (int64_t) distance_mm * 10LL;
    int64_t velocity = up_tof_protocol_divide_round_nearest(
        numerator,
        (int64_t) integration_us);

    return up_tof_protocol_saturate_i32(velocity);
}

up_tof_protocol_decode_status_t up_tof_protocol_decode_frame(
    uint8_t const * p_frame,
    size_t frame_size,
    up_tof_protocol_sample_t * p_sample)
{
    up_tof_protocol_sample_t sample = {0};
    uint8_t checksum = 0U;
    size_t payload_index;

    if ((NULL == p_frame) || (NULL == p_sample) ||
        (UP_TOF_PROTOCOL_FRAME_SIZE != frame_size))
    {
        return UP_TOF_PROTOCOL_DECODE_ARGUMENT_ERROR;
    }

    if ((UP_TOF_PROTOCOL_HEADER_BYTE != p_frame[0]) ||
        (UP_TOF_PROTOCOL_LENGTH_BYTE != p_frame[1]) ||
        (UP_TOF_PROTOCOL_TAIL_BYTE !=
         p_frame[UP_TOF_PROTOCOL_FRAME_SIZE - 1U]))
    {
        return UP_TOF_PROTOCOL_DECODE_FORMAT_ERROR;
    }

    for (payload_index = 2U;
         payload_index < (2U + UP_TOF_PROTOCOL_PAYLOAD_SIZE);
         payload_index++)
    {
        checksum ^= p_frame[payload_index];
    }

    if (checksum != p_frame[UP_TOF_PROTOCOL_FRAME_SIZE - 2U])
    {
        return UP_TOF_PROTOCOL_DECODE_CHECKSUM_ERROR;
    }

    sample.flow_x_integral = up_tof_protocol_read_i16_le(&p_frame[2]);
    sample.flow_y_integral = up_tof_protocol_read_i16_le(&p_frame[4]);
    sample.integration_us = up_tof_protocol_read_u16_le(&p_frame[6]);
    sample.distance_mm = up_tof_protocol_read_u16_le(&p_frame[8]);
    sample.flow_valid_raw = p_frame[10];
    sample.tof_confidence = p_frame[11];
    sample.flow_valid =
        (UP_TOF_PROTOCOL_FLOW_VALID_VALUE == sample.flow_valid_raw);
    sample.tof_valid =
        ((sample.distance_mm >= UP_TOF_PROTOCOL_DISTANCE_MIN_MM) &&
         (sample.distance_mm <= UP_TOF_PROTOCOL_DISTANCE_MAX_MM));
    sample.velocity_valid = sample.flow_valid && sample.tof_valid &&
                            (sample.integration_us > 0U);

    if (sample.flow_valid && sample.tof_valid)
    {
        sample.displacement_x_mm = up_tof_protocol_displacement_mm(
            sample.flow_x_integral,
            sample.distance_mm);
        sample.displacement_y_mm = up_tof_protocol_displacement_mm(
            sample.flow_y_integral,
            sample.distance_mm);
    }

    if (sample.velocity_valid)
    {
        sample.velocity_x_cm_s = up_tof_protocol_velocity_cm_s(
            sample.flow_x_integral,
            sample.distance_mm,
            sample.integration_us);
        sample.velocity_y_cm_s = up_tof_protocol_velocity_cm_s(
            sample.flow_y_integral,
            sample.distance_mm,
            sample.integration_us);
    }

    *p_sample = sample;
    return UP_TOF_PROTOCOL_DECODE_OK;
}
