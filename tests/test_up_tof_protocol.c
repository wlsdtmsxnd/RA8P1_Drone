#include "driver/up_tof_protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t const g_manual_frame_1[UP_TOF_PROTOCOL_FRAME_SIZE] =
{
    0xFEU, 0x0AU, 0x02U, 0x00U, 0xFEU, 0xFFU, 0x20U,
    0x4EU, 0xFDU, 0x09U, 0xF5U, 0x00U, 0x6CU, 0x55U
};

static uint8_t const g_manual_frame_2[UP_TOF_PROTOCOL_FRAME_SIZE] =
{
    0xFEU, 0x0AU, 0xFEU, 0xFFU, 0x04U, 0x00U, 0xBFU,
    0x30U, 0x84U, 0x02U, 0xF5U, 0x64U, 0x9DU, 0x55U
};

static uint8_t const g_manual_tof_invalid_frame[UP_TOF_PROTOCOL_FRAME_SIZE] =
{
    0xFEU, 0x0AU, 0x00U, 0x00U, 0x00U, 0x00U, 0xBFU,
    0x30U, 0xFFU, 0xFFU, 0xF5U, 0x64U, 0x1EU, 0x55U
};

static void require_true(bool condition, char const * p_name)
{
    if (!condition)
    {
        (void) fprintf(stderr, "%s: condition is false\n", p_name);
        exit(EXIT_FAILURE);
    }
}

static void require_i32(int32_t actual, int32_t expected,
                        char const * p_name)
{
    if (actual != expected)
    {
        (void) fprintf(stderr,
                       "%s: expected %ld, got %ld\n",
                       p_name,
                       (long) expected,
                       (long) actual);
        exit(EXIT_FAILURE);
    }
}

static void require_status(up_tof_protocol_decode_status_t actual,
                           up_tof_protocol_decode_status_t expected,
                           char const * p_name)
{
    if (actual != expected)
    {
        (void) fprintf(stderr,
                       "%s: expected status %d, got %d\n",
                       p_name,
                       (int) expected,
                       (int) actual);
        exit(EXIT_FAILURE);
    }
}

static void update_checksum(uint8_t frame[UP_TOF_PROTOCOL_FRAME_SIZE])
{
    uint8_t checksum = 0U;
    size_t index;

    for (index = 2U;
         index < (2U + UP_TOF_PROTOCOL_PAYLOAD_SIZE);
         index++)
    {
        checksum ^= frame[index];
    }

    frame[UP_TOF_PROTOCOL_FRAME_SIZE - 2U] = checksum;
}

static void set_distance(uint8_t frame[UP_TOF_PROTOCOL_FRAME_SIZE],
                         uint16_t distance_mm)
{
    frame[8] = (uint8_t) (distance_mm & 0xFFU);
    frame[9] = (uint8_t) (distance_mm >> 8U);
    update_checksum(frame);
}

static void test_manual_frames(void)
{
    up_tof_protocol_sample_t sample;
    up_tof_protocol_decode_status_t status;

    status = up_tof_protocol_decode_frame(g_manual_frame_1,
                                          sizeof(g_manual_frame_1),
                                          &sample);
    require_status(status, UP_TOF_PROTOCOL_DECODE_OK, "manual frame 1");
    require_i32(sample.flow_x_integral, 2, "manual frame 1 flow x");
    require_i32(sample.flow_y_integral, -2, "manual frame 1 flow y");
    require_i32(sample.integration_us, 20000, "manual frame 1 integration");
    require_i32(sample.distance_mm, 2557, "manual frame 1 distance");
    require_i32(sample.displacement_x_mm, 1, "manual frame 1 displacement x");
    require_i32(sample.displacement_y_mm, -1, "manual frame 1 displacement y");
    require_i32(sample.velocity_x_cm_s, 3, "manual frame 1 velocity x");
    require_i32(sample.velocity_y_cm_s, -3, "manual frame 1 velocity y");
    require_true(sample.flow_valid, "manual frame 1 flow valid");
    require_true(sample.tof_valid, "manual frame 1 TOF valid");
    require_true(sample.velocity_valid, "manual frame 1 velocity valid");

    status = up_tof_protocol_decode_frame(g_manual_frame_2,
                                          sizeof(g_manual_frame_2),
                                          &sample);
    require_status(status, UP_TOF_PROTOCOL_DECODE_OK, "manual frame 2");
    require_i32(sample.flow_x_integral, -2, "manual frame 2 flow x");
    require_i32(sample.flow_y_integral, 4, "manual frame 2 flow y");
    require_i32(sample.integration_us, 12479, "manual frame 2 integration");
    require_i32(sample.distance_mm, 644, "manual frame 2 distance");
    require_i32(sample.velocity_x_cm_s, -1, "manual frame 2 velocity x");
    require_i32(sample.velocity_y_cm_s, 2, "manual frame 2 velocity y");

    status = up_tof_protocol_decode_frame(g_manual_tof_invalid_frame,
                                          sizeof(g_manual_tof_invalid_frame),
                                          &sample);
    require_status(status, UP_TOF_PROTOCOL_DECODE_OK,
                   "manual TOF invalid frame");
    require_true(sample.flow_valid, "TOF invalid frame flow valid");
    require_true(!sample.tof_valid, "TOF invalid frame TOF invalid");
    require_true(!sample.velocity_valid,
                 "TOF invalid frame velocity invalid");
    require_i32(sample.velocity_x_cm_s, 0,
                "TOF invalid frame velocity zero");
}

static void test_validity_boundaries(void)
{
    uint8_t frame[UP_TOF_PROTOCOL_FRAME_SIZE];
    up_tof_protocol_sample_t sample;

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    set_distance(frame, UP_TOF_PROTOCOL_DISTANCE_MIN_MM);
    require_status(up_tof_protocol_decode_frame(frame, sizeof(frame), &sample),
                   UP_TOF_PROTOCOL_DECODE_OK,
                   "minimum distance decode");
    require_true(sample.tof_valid, "minimum distance valid");

    set_distance(frame, UP_TOF_PROTOCOL_DISTANCE_MAX_MM);
    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_true(sample.tof_valid, "maximum distance valid");

    set_distance(frame, UP_TOF_PROTOCOL_DISTANCE_MIN_MM - 1U);
    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_true(!sample.tof_valid, "below minimum distance invalid");

    set_distance(frame, UP_TOF_PROTOCOL_DISTANCE_MAX_MM + 1U);
    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_true(!sample.tof_valid, "above maximum distance invalid");

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[10] = 0x00U;
    update_checksum(frame);
    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_true(!sample.flow_valid, "flow flag invalid");
    require_true(sample.tof_valid, "TOF independent from flow flag");
    require_true(!sample.velocity_valid, "invalid flow blocks velocity");

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[6] = 0U;
    frame[7] = 0U;
    update_checksum(frame);
    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_true(sample.flow_valid, "zero integration flow valid");
    require_true(sample.tof_valid, "zero integration TOF valid");
    require_true(!sample.velocity_valid, "zero integration velocity invalid");
}

static void test_format_and_checksum_errors(void)
{
    uint8_t frame[UP_TOF_PROTOCOL_FRAME_SIZE];
    up_tof_protocol_sample_t sample;

    require_status(up_tof_protocol_decode_frame(NULL,
                                                UP_TOF_PROTOCOL_FRAME_SIZE,
                                                &sample),
                   UP_TOF_PROTOCOL_DECODE_ARGUMENT_ERROR,
                   "null frame");
    require_status(up_tof_protocol_decode_frame(g_manual_frame_1,
                                                UP_TOF_PROTOCOL_FRAME_SIZE - 1U,
                                                &sample),
                   UP_TOF_PROTOCOL_DECODE_ARGUMENT_ERROR,
                   "wrong frame size");

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[0] = 0U;
    require_status(up_tof_protocol_decode_frame(frame, sizeof(frame), &sample),
                   UP_TOF_PROTOCOL_DECODE_FORMAT_ERROR,
                   "bad header");

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[UP_TOF_PROTOCOL_FRAME_SIZE - 1U] = 0U;
    require_status(up_tof_protocol_decode_frame(frame, sizeof(frame), &sample),
                   UP_TOF_PROTOCOL_DECODE_FORMAT_ERROR,
                   "bad tail");

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[2] ^= 0x01U;
    require_status(up_tof_protocol_decode_frame(frame, sizeof(frame), &sample),
                   UP_TOF_PROTOCOL_DECODE_CHECKSUM_ERROR,
                   "bad checksum");
}

static void test_velocity_saturation(void)
{
    uint8_t frame[UP_TOF_PROTOCOL_FRAME_SIZE];
    up_tof_protocol_sample_t sample;

    (void) memcpy(frame, g_manual_frame_1, sizeof(frame));
    frame[2] = 0xFFU;
    frame[3] = 0x7FU;
    frame[4] = 0x00U;
    frame[5] = 0x80U;
    frame[6] = 0x01U;
    frame[7] = 0x00U;
    set_distance(frame, UP_TOF_PROTOCOL_DISTANCE_MAX_MM);

    (void) up_tof_protocol_decode_frame(frame, sizeof(frame), &sample);
    require_i32(sample.velocity_x_cm_s, INT32_MAX,
                "positive velocity saturation");
    require_i32(sample.velocity_y_cm_s, INT32_MIN,
                "negative velocity saturation");
}

int main(void)
{
    test_manual_frames();
    test_validity_boundaries();
    test_format_and_checksum_errors();
    test_velocity_saturation();
    (void) puts("UPIX UP-T301 protocol tests passed");
    return EXIT_SUCCESS;
}
