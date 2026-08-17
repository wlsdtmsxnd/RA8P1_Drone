#include "up_tof.h"

#include <stddef.h>

#define UP_TOF_RX_BUFFER_SIZE          (256U)
#define UP_TOF_PAYLOAD_SIZE            (10U)
#define UP_TOF_HEADER_BYTE             (0xFEU)
#define UP_TOF_LENGTH_BYTE             (0x0AU)
#define UP_TOF_TAIL_BYTE               (0x55U)

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
static uint8_t g_up_tof_payload[UP_TOF_PAYLOAD_SIZE];
static uint8_t g_up_tof_payload_index = 0U;
static uint8_t g_up_tof_xor = 0U;
static up_tof_parse_state_t g_up_tof_parse_state = UP_TOF_PARSE_WAIT_HEADER;

volatile up_tof_data_t g_up_tof_data = {0};

_Static_assert((UP_TOF_RX_BUFFER_SIZE & (UP_TOF_RX_BUFFER_SIZE - 1U)) == 0U,
               "UP_TOF_RX_BUFFER_SIZE must be a power of two");

static uint16_t up_tof_read_u16_le(uint8_t const * p_data)
{
    return (uint16_t) (((uint16_t) p_data[1] << 8U) | (uint16_t) p_data[0]);
}

static int16_t up_tof_read_i16_le(uint8_t const * p_data)
{
    return (int16_t) up_tof_read_u16_le(p_data);
}

static int32_t up_tof_scale_displacement_mm(int16_t flow_integral,
                                            uint16_t distance_mm)
{
    int32_t numerator = (int32_t) flow_integral * (int32_t) distance_mm;

    if (numerator >= 0)
    {
        numerator += 5000;
    }
    else
    {
        numerator -= 5000;
    }

    return numerator / 10000;
}

static int32_t up_tof_scale_velocity_cm_s(int32_t displacement_mm,
                                          uint16_t integration_us)
{
    int64_t numerator;

    if (0U == integration_us)
    {
        return 0;
    }

    numerator = (int64_t) displacement_mm * 100000LL;

    if (numerator >= 0)
    {
        numerator += (int64_t) integration_us / 2LL;
    }
    else
    {
        numerator -= (int64_t) integration_us / 2LL;
    }

    return (int32_t) (numerator / (int64_t) integration_us);
}

static void up_tof_reset_parser(void)
{
    g_up_tof_parse_state = UP_TOF_PARSE_WAIT_HEADER;
    g_up_tof_payload_index = 0U;
    g_up_tof_xor = 0U;
}

static void up_tof_publish_payload(void)
{
    up_tof_data_t data;

    data = g_up_tof_data;
    data.flow_x_integral = up_tof_read_i16_le(&g_up_tof_payload[0]);
    data.flow_y_integral = up_tof_read_i16_le(&g_up_tof_payload[2]);
    data.integration_us = up_tof_read_u16_le(&g_up_tof_payload[4]);
    data.distance_mm = up_tof_read_u16_le(&g_up_tof_payload[6]);
    data.flow_valid_raw = g_up_tof_payload[8];
    data.tof_confidence = g_up_tof_payload[9];
    data.displacement_x_mm =
        up_tof_scale_displacement_mm(data.flow_x_integral, data.distance_mm);
    data.displacement_y_mm =
        up_tof_scale_displacement_mm(data.flow_y_integral, data.distance_mm);
    data.velocity_x_cm_s =
        up_tof_scale_velocity_cm_s(data.displacement_x_mm, data.integration_us);
    data.velocity_y_cm_s =
        up_tof_scale_velocity_cm_s(data.displacement_y_mm, data.integration_us);
    data.flow_valid = (UP_TOF_FLOW_VALID_VALUE == data.flow_valid_raw);
    data.valid = data.flow_valid;
    data.frame_count++;
    data.last_frame_tick = xTaskGetTickCount();

    taskENTER_CRITICAL();
    g_up_tof_data = data;
    taskEXIT_CRITICAL();
}

static void up_tof_parse_byte(uint8_t received_byte)
{
    switch (g_up_tof_parse_state)
    {
        case UP_TOF_PARSE_WAIT_HEADER:
        {
            if (UP_TOF_HEADER_BYTE == received_byte)
            {
                g_up_tof_parse_state = UP_TOF_PARSE_WAIT_LENGTH;
            }

            break;
        }

        case UP_TOF_PARSE_WAIT_LENGTH:
        {
            if (UP_TOF_LENGTH_BYTE == received_byte)
            {
                g_up_tof_parse_state = UP_TOF_PARSE_PAYLOAD;
                g_up_tof_payload_index = 0U;
                g_up_tof_xor = 0U;
            }
            else
            {
                g_up_tof_data.parse_error_count++;
                up_tof_reset_parser();
            }

            break;
        }

        case UP_TOF_PARSE_PAYLOAD:
        {
            g_up_tof_payload[g_up_tof_payload_index] = received_byte;
            g_up_tof_payload_index++;
            g_up_tof_xor ^= received_byte;

            if (UP_TOF_PAYLOAD_SIZE == g_up_tof_payload_index)
            {
                g_up_tof_parse_state = UP_TOF_PARSE_CHECKSUM;
            }

            break;
        }

        case UP_TOF_PARSE_CHECKSUM:
        {
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
            if (UP_TOF_TAIL_BYTE == received_byte)
            {
                up_tof_publish_payload();
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

    if ((true == p_data->valid) &&
        ((current_tick - p_data->last_frame_tick) <=
         pdMS_TO_TICKS(UP_TOF_SIGNAL_TIMEOUT_MS)))
    {
        p_data->valid = true;
    }
    else
    {
        p_data->valid = false;
    }
}

bool up_tof_is_ready(void)
{
    up_tof_data_t data;

    up_tof_get_data(&data);
    return data.valid;
}
