#include "up_tof.h"

#include <stddef.h>

#define UP_TOF_RX_BUFFER_SIZE          (256U)

typedef enum
{
    UP_TOF_PARSE_WAIT_HEADER = 0,
    UP_TOF_PARSE_WAIT_LENGTH,
    UP_TOF_PARSE_PAYLOAD,
    UP_TOF_PARSE_CHECKSUM,
    UP_TOF_PARSE_TAIL
} up_tof_parse_state_t;

static uart_instance_t const * g_up_tof_uart = NULL;
static volatile uint8_t g_up_tof_rx_buffer[UP_TOF_RX_BUFFER_SIZE];
static volatile uint16_t g_up_tof_rx_head = 0U;
static volatile uint16_t g_up_tof_rx_tail = 0U;
static uint8_t g_up_tof_frame[UP_TOF_PROTOCOL_FRAME_SIZE];
static uint8_t g_up_tof_frame_index = 0U;
static uint8_t g_up_tof_xor = 0U;
static up_tof_parse_state_t g_up_tof_parse_state = UP_TOF_PARSE_WAIT_HEADER;

volatile up_tof_data_t g_up_tof_data = {0};

_Static_assert((UP_TOF_RX_BUFFER_SIZE & (UP_TOF_RX_BUFFER_SIZE - 1U)) == 0U,
               "UP_TOF_RX_BUFFER_SIZE must be a power of two");

static void up_tof_reset_parser(void)
{
    g_up_tof_parse_state = UP_TOF_PARSE_WAIT_HEADER;
    g_up_tof_frame_index = 0U;
    g_up_tof_xor = 0U;
}

static void up_tof_publish_frame(void)
{
    up_tof_protocol_sample_t sample;
    up_tof_protocol_decode_status_t status;
    up_tof_data_t data;

    status = up_tof_protocol_decode_frame(g_up_tof_frame,
                                          sizeof(g_up_tof_frame),
                                          &sample);
    if (UP_TOF_PROTOCOL_DECODE_OK != status)
    {
        if (UP_TOF_PROTOCOL_DECODE_CHECKSUM_ERROR == status)
        {
            g_up_tof_data.checksum_error_count++;
        }
        else
        {
            g_up_tof_data.parse_error_count++;
        }

        return;
    }

    taskENTER_CRITICAL();
    data = g_up_tof_data;
    data.flow_x_integral = sample.flow_x_integral;
    data.flow_y_integral = sample.flow_y_integral;
    data.integration_us = sample.integration_us;
    data.distance_mm = sample.distance_mm;
    data.flow_valid_raw = sample.flow_valid_raw;
    data.tof_confidence = sample.tof_confidence;
    data.displacement_x_mm = sample.displacement_x_mm;
    data.displacement_y_mm = sample.displacement_y_mm;
    data.velocity_x_cm_s = sample.velocity_x_cm_s;
    data.velocity_y_cm_s = sample.velocity_y_cm_s;
    data.frame_valid = true;
    data.flow_valid = sample.flow_valid;
    data.tof_valid = sample.tof_valid;
    data.velocity_valid = sample.velocity_valid;
    data.frame_count++;
    data.last_frame_tick = xTaskGetTickCount();
    g_up_tof_data = data;
    taskEXIT_CRITICAL();
}

static void up_tof_parse_byte(uint8_t received_byte)
{
    switch (g_up_tof_parse_state)
    {
        case UP_TOF_PARSE_WAIT_HEADER:
        {
            if (UP_TOF_PROTOCOL_HEADER_BYTE == received_byte)
            {
                g_up_tof_frame[0] = received_byte;
                g_up_tof_frame_index = 1U;
                g_up_tof_parse_state = UP_TOF_PARSE_WAIT_LENGTH;
            }

            break;
        }

        case UP_TOF_PARSE_WAIT_LENGTH:
        {
            if (UP_TOF_PROTOCOL_LENGTH_BYTE == received_byte)
            {
                g_up_tof_frame[1] = received_byte;
                g_up_tof_frame_index = 2U;
                g_up_tof_xor = 0U;
                g_up_tof_parse_state = UP_TOF_PARSE_PAYLOAD;
            }
            else
            {
                g_up_tof_data.parse_error_count++;

                if (UP_TOF_PROTOCOL_HEADER_BYTE == received_byte)
                {
                    g_up_tof_frame[0] = received_byte;
                    g_up_tof_frame_index = 1U;
                }
                else
                {
                    up_tof_reset_parser();
                }
            }

            break;
        }

        case UP_TOF_PARSE_PAYLOAD:
        {
            g_up_tof_frame[g_up_tof_frame_index] = received_byte;
            g_up_tof_frame_index++;
            g_up_tof_xor ^= received_byte;

            if ((2U + UP_TOF_PROTOCOL_PAYLOAD_SIZE) ==
                g_up_tof_frame_index)
            {
                g_up_tof_parse_state = UP_TOF_PARSE_CHECKSUM;
            }

            break;
        }

        case UP_TOF_PARSE_CHECKSUM:
        {
            g_up_tof_frame[UP_TOF_PROTOCOL_FRAME_SIZE - 2U] = received_byte;

            if (received_byte == g_up_tof_xor)
            {
                g_up_tof_parse_state = UP_TOF_PARSE_TAIL;
            }
            else
            {
                g_up_tof_data.checksum_error_count++;
                up_tof_reset_parser();
            }

            break;
        }

        case UP_TOF_PARSE_TAIL:
        {
            g_up_tof_frame[UP_TOF_PROTOCOL_FRAME_SIZE - 1U] = received_byte;

            if (UP_TOF_PROTOCOL_TAIL_BYTE == received_byte)
            {
                up_tof_publish_frame();
            }
            else
            {
                g_up_tof_data.parse_error_count++;
            }

            up_tof_reset_parser();
            break;
        }

        default:
        {
            up_tof_reset_parser();
            break;
        }
    }
}

void up_tof_uart_callback(uart_callback_args_t * p_args)
{
    uint16_t next_head;

    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            next_head = (uint16_t) ((g_up_tof_rx_head + 1U) &
                                    (UP_TOF_RX_BUFFER_SIZE - 1U));

            if (next_head != g_up_tof_rx_tail)
            {
                g_up_tof_rx_buffer[g_up_tof_rx_head] = (uint8_t) p_args->data;
                g_up_tof_rx_head = next_head;
            }
            else
            {
                g_up_tof_data.rx_overflow_count++;
            }

            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        {
            g_up_tof_data.uart_error_count++;
            break;
        }

        default:
        {
            break;
        }
    }
}

fsp_err_t up_tof_init(uart_instance_t const * p_uart_instance)
{
    fsp_err_t err;

    if (NULL == p_uart_instance)
    {
        return FSP_ERR_ASSERTION;
    }

    g_up_tof_uart = p_uart_instance;
    g_up_tof_rx_head = 0U;
    g_up_tof_rx_tail = 0U;
    g_up_tof_data = (up_tof_data_t) {0};
    up_tof_reset_parser();

    err = g_up_tof_uart->p_api->open(g_up_tof_uart->p_ctrl,
                                     g_up_tof_uart->p_cfg);

    if (FSP_ERR_ALREADY_OPEN == err)
    {
        return FSP_SUCCESS;
    }

    if (FSP_SUCCESS != err)
    {
        g_up_tof_uart = NULL;
    }

    return err;
}

void up_tof_process(void)
{
    uint8_t received_byte;

    while (g_up_tof_rx_tail != g_up_tof_rx_head)
    {
        received_byte = g_up_tof_rx_buffer[g_up_tof_rx_tail];
        g_up_tof_rx_tail = (uint16_t) ((g_up_tof_rx_tail + 1U) &
                                       (UP_TOF_RX_BUFFER_SIZE - 1U));

        up_tof_parse_byte(received_byte);
    }
}

void up_tof_get_data(up_tof_data_t * p_data)
{
    TickType_t current_tick;

    if (NULL == p_data)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_data = g_up_tof_data;
    taskEXIT_CRITICAL();

    current_tick = xTaskGetTickCount();

    if ((!p_data->frame_valid) ||
        ((current_tick - p_data->last_frame_tick) >
         pdMS_TO_TICKS(UP_TOF_SIGNAL_TIMEOUT_MS)))
    {
        p_data->frame_valid = false;
        p_data->flow_valid = false;
        p_data->tof_valid = false;
        p_data->velocity_valid = false;
        p_data->displacement_x_mm = 0;
        p_data->displacement_y_mm = 0;
        p_data->velocity_x_cm_s = 0;
        p_data->velocity_y_cm_s = 0;
    }
}

bool up_tof_is_ready(void)
{
    up_tof_data_t data;

    up_tof_get_data(&data);
    return data.frame_valid;
}
