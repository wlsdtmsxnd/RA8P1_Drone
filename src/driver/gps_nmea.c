#include "gps_nmea.h"

#include <stddef.h>

#define GPS_RX_BUFFER_SIZE             (512U)
#define GPS_SENTENCE_BUFFER_SIZE       (96U)
#define GPS_NMEA_MAX_FIELDS            (24U)

static uart_instance_t const * g_gps_uart = NULL;
static volatile uint8_t g_gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static volatile uint16_t g_gps_rx_head = 0U;
static volatile uint16_t g_gps_rx_tail = 0U;
static char g_gps_sentence[GPS_SENTENCE_BUFFER_SIZE];
static uint8_t g_gps_sentence_index = 0U;

volatile gps_data_t g_gps_data = {0};

_Static_assert((GPS_RX_BUFFER_SIZE & (GPS_RX_BUFFER_SIZE - 1U)) == 0U, "GPS_RX_BUFFER_SIZE must be a power of two");

static bool gps_is_digit(char c)
{
    return (c >= '0') && (c <= '9');
}

static bool gps_starts_with(char const * p_text, char const * p_prefix)
{
    while ('\0' != *p_prefix)
    {
        if (*p_text != *p_prefix)
        {
            return false;
        }

        p_text++;
        p_prefix++;
    }

    return true;
}

static uint8_t gps_hex_value(char c)
{
    if ((c >= '0') && (c <= '9'))
    {
        return (uint8_t) (c - '0');
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        return (uint8_t) (c - 'A' + 10);
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        return (uint8_t) (c - 'a' + 10);
    }

    return 0xFFU;
}

static bool gps_check_checksum(char const * p_sentence)
{
    uint8_t checksum = 0U;
    uint8_t received;
    uint8_t high_nibble;
    uint8_t low_nibble;

    if ('$' != p_sentence[0])
    {
        return false;
    }

    p_sentence++;

    while (('*' != *p_sentence) && ('\0' != *p_sentence))
    {
        checksum ^= (uint8_t) *p_sentence;
        p_sentence++;
    }

    if ('*' != *p_sentence)
    {
        return false;
    }

    high_nibble = gps_hex_value(p_sentence[1]);
    low_nibble = gps_hex_value(p_sentence[2]);

    if ((high_nibble > 0x0FU) || (low_nibble > 0x0FU))
    {
        return false;
    }

    received = (uint8_t) ((high_nibble << 4U) | low_nibble);
    return received == checksum;
}

static uint8_t gps_split_fields(char * p_sentence,
                                char * p_fields[],
                                uint8_t max_fields)
{
    uint8_t field_count = 0U;
    char * p_cursor = p_sentence;

    if ('$' == *p_cursor)
    {
        p_cursor++;
    }

    while ((field_count < max_fields) && ('\0' != *p_cursor))
    {
        p_fields[field_count] = p_cursor;
        field_count++;

        while ((',' != *p_cursor) && ('*' != *p_cursor) && ('\0' != *p_cursor))
        {
            p_cursor++;
        }

        if (('\0' == *p_cursor) || ('*' == *p_cursor))
        {
            *p_cursor = '\0';
            break;
        }

        *p_cursor = '\0';
        p_cursor++;
    }

    return field_count;
}

static bool gps_parse_unsigned_scaled(char const * p_text,
                                      uint32_t scale,
                                      uint32_t * p_value)
{
    uint32_t value = 0U;
    uint32_t frac_scale = scale;
    bool saw_digit = false;

    while (gps_is_digit(*p_text))
    {
        value = (value * 10U) + (uint32_t) (*p_text - '0');
        saw_digit = true;
        p_text++;
    }

    value *= scale;

    if ('.' == *p_text)
    {
        p_text++;

        while (gps_is_digit(*p_text) && (frac_scale > 1U))
        {
            frac_scale /= 10U;
            value += ((uint32_t) (*p_text - '0')) * frac_scale;
            saw_digit = true;
            p_text++;
        }
    }

    if (false == saw_digit)
    {
        return false;
    }

    *p_value = value;
    return true;
}

static bool gps_parse_u8(char const * p_text, uint8_t * p_value)
{
    uint32_t value = 0U;
    bool saw_digit = false;

    while (gps_is_digit(*p_text))
    {
        value = (value * 10U) + (uint32_t) (*p_text - '0');
        saw_digit = true;
        p_text++;
    }

    if ((false == saw_digit) || (value > 255U))
    {
        return false;
    }

    *p_value = (uint8_t) value;
    return true;
}

static bool gps_parse_lat_lon(char const * p_value,
                              char const * p_hemisphere,
                              int32_t * p_deg_e7)
{
    uint32_t value_x1e7;
    uint32_t integer_part;
    uint32_t degrees;
    uint32_t minutes_whole;
    uint32_t minutes_x1e7;
    uint32_t result;

    if ((NULL == p_value) || (NULL == p_hemisphere) || ('\0' == p_value[0]) || ('\0' == p_hemisphere[0]))
    {
        return false;
    }

    if (false == gps_parse_unsigned_scaled(p_value, 10000000U, &value_x1e7))
    {
        return false;
    }

    integer_part = value_x1e7 / 10000000U;
    degrees = integer_part / 100U;
    minutes_whole = integer_part % 100U;
    minutes_x1e7 = (minutes_whole * 10000000U) + (value_x1e7 % 10000000U);
    result = (degrees * 10000000U) + ((minutes_x1e7 + 30U) / 60U);

    if (('S' == p_hemisphere[0]) || ('W' == p_hemisphere[0]))
    {
        *p_deg_e7 = -(int32_t) result;
    }
    else
    {
        *p_deg_e7 = (int32_t) result;
    }

    return true;
}

static void gps_parse_gga(char * p_fields[], uint8_t field_count)
{
    gps_data_t data;
    uint32_t scaled_value;

    if (field_count < 10U)
    {
        g_gps_data.parse_error_count++;
        return;
    }

    data = g_gps_data;

    if (false == gps_parse_u8(p_fields[6], &data.fix_quality))
    {
        data.fix_quality = 0U;
    }

    if (false == gps_parse_u8(p_fields[7], &data.satellites))
    {
        data.satellites = 0U;
    }

    if (gps_parse_unsigned_scaled(p_fields[8], 100U, &scaled_value))
    {
        data.hdop_centi = (uint16_t) scaled_value;
    }

    if (gps_parse_unsigned_scaled(p_fields[9], 100U, &scaled_value))
    {
        data.altitude_cm = (int32_t) scaled_value;
    }

    if ((data.fix_quality > 0U) &&
        gps_parse_lat_lon(p_fields[2], p_fields[3], &data.latitude_deg_e7) &&
        gps_parse_lat_lon(p_fields[4], p_fields[5], &data.longitude_deg_e7))
    {
        data.valid = true;
        data.last_fix_tick = xTaskGetTickCount();
    }
    else
    {
        data.valid = false;
    }

    data.sentence_count++;

    taskENTER_CRITICAL();
    g_gps_data = data;
    taskEXIT_CRITICAL();
}

static void gps_parse_rmc(char * p_fields[], uint8_t field_count)
{
    gps_data_t data;
    uint32_t scaled_value;

    if (field_count < 9U)
    {
        g_gps_data.parse_error_count++;
        return;
    }

    data = g_gps_data;

    if ('A' == p_fields[2][0])
    {
        if (gps_parse_lat_lon(p_fields[3], p_fields[4], &data.latitude_deg_e7) &&
            gps_parse_lat_lon(p_fields[5], p_fields[6], &data.longitude_deg_e7))
        {
            data.valid = true;
            data.last_fix_tick = xTaskGetTickCount();
        }
    }
    else
    {
        data.valid = false;
    }

    if (gps_parse_unsigned_scaled(p_fields[7], 1000U, &scaled_value))
    {
        data.ground_speed_cm_s = (int32_t) ((scaled_value * 51444U) / 1000000U);
    }

    if (gps_parse_unsigned_scaled(p_fields[8], 100U, &scaled_value))
    {
        data.course_cdeg = (uint16_t) scaled_value;
    }

    data.sentence_count++;

    taskENTER_CRITICAL();
    g_gps_data = data;
    taskEXIT_CRITICAL();
}

static void gps_parse_sentence(void)
{
    char * p_fields[GPS_NMEA_MAX_FIELDS];
    uint8_t field_count;

    if (false == gps_check_checksum(g_gps_sentence))
    {
        g_gps_data.checksum_error_count++;
        return;
    }

    field_count = gps_split_fields(g_gps_sentence, p_fields, GPS_NMEA_MAX_FIELDS);

    if (0U == field_count)
    {
        g_gps_data.parse_error_count++;
        return;
    }

    if (gps_starts_with(p_fields[0], "GNGGA") || gps_starts_with(p_fields[0], "GPGGA"))
    {
        gps_parse_gga(p_fields, field_count);
    }
    else if (gps_starts_with(p_fields[0], "GNRMC") || gps_starts_with(p_fields[0], "GPRMC"))
    {
        gps_parse_rmc(p_fields, field_count);
    }
}

void gps_uart_callback(uart_callback_args_t * p_args)
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
            next_head = (uint16_t) ((g_gps_rx_head + 1U) & (GPS_RX_BUFFER_SIZE - 1U));

            if (next_head != g_gps_rx_tail)
            {
                g_gps_rx_buffer[g_gps_rx_head] = (uint8_t) p_args->data;
                g_gps_rx_head = next_head;
            }
            else
            {
                g_gps_data.rx_overflow_count++;
            }

            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        {
            g_gps_data.uart_error_count++;
            break;
        }

        default:
        {
            break;
        }
    }
}

fsp_err_t gps_init(uart_instance_t const * p_uart_instance)
{
    fsp_err_t err;

    if (NULL == p_uart_instance)
    {
        return FSP_ERR_ASSERTION;
    }

    g_gps_uart = p_uart_instance;
    g_gps_rx_head = 0U;
    g_gps_rx_tail = 0U;
    g_gps_sentence_index = 0U;
    g_gps_data = (gps_data_t) {0};

    err = g_gps_uart->p_api->open(g_gps_uart->p_ctrl, g_gps_uart->p_cfg);

    if (FSP_ERR_ALREADY_OPEN == err)
    {
        return FSP_SUCCESS;
    }

    if (FSP_SUCCESS != err)
    {
        g_gps_uart = NULL;
    }

    return err;
}

void gps_process(void)
{
    uint8_t received_byte;

    while (g_gps_rx_tail != g_gps_rx_head)
    {
        received_byte = g_gps_rx_buffer[g_gps_rx_tail];
        g_gps_rx_tail = (uint16_t) ((g_gps_rx_tail + 1U) & (GPS_RX_BUFFER_SIZE - 1U));

        if ('$' == received_byte)
        {
            g_gps_sentence_index = 0U;
            g_gps_sentence[g_gps_sentence_index] = (char) received_byte;
            g_gps_sentence_index++;
            continue;
        }

        if (0U == g_gps_sentence_index)
        {
            continue;
        }

        if (('\r' == received_byte) || ('\n' == received_byte))
        {
            g_gps_sentence[g_gps_sentence_index] = '\0';
            gps_parse_sentence();
            g_gps_sentence_index = 0U;
            continue;
        }

        if (g_gps_sentence_index < (GPS_SENTENCE_BUFFER_SIZE - 1U))
        {
            g_gps_sentence[g_gps_sentence_index] = (char) received_byte;
            g_gps_sentence_index++;
        }
        else
        {
            g_gps_sentence_index = 0U;
            g_gps_data.parse_error_count++;
        }
    }
}

void gps_get_data(gps_data_t * p_data)
{
    TickType_t current_tick;

    if (NULL == p_data)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_data = g_gps_data;
    taskEXIT_CRITICAL();

    current_tick = xTaskGetTickCount();

    if ((true == p_data->valid) &&
        ((current_tick - p_data->last_fix_tick) <= pdMS_TO_TICKS(GPS_SIGNAL_TIMEOUT_MS)))
    {
        p_data->valid = true;
    }
    else
    {
        p_data->valid = false;
    }
}

bool gps_is_ready(void)
{
    gps_data_t data;

    gps_get_data(&data);
    return data.valid;
}
