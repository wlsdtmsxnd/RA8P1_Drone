#include "crsf.h"

/* CRSF 最大完整帧长度。 */
#define CRSF_MAX_FRAME_SIZE             (64U)

/* CRSF 软件接收环形缓冲区大小，必须为 2 的整数次幂。 */
#define CRSF_RX_BUFFER_SIZE             (512U)

/* CRSF 串行同步字节和常用设备地址。 */
#define CRSF_ADDRESS_BROADCAST          (0x00U)
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  (0xC8U)
#define CRSF_ADDRESS_RADIO_TRANSMITTER  (0xEAU)
#define CRSF_ADDRESS_RECEIVER           (0xECU)

/* CRSF 帧类型。 */
#define CRSF_FRAME_TYPE_LINK_STATISTICS (0x14U)
#define CRSF_FRAME_TYPE_RC_CHANNELS     (0x16U)

/* CRSF CRC8 多项式。 */
#define CRSF_CRC_POLYNOMIAL             (0xD5U)

/* CRSF 遥控通道帧负载长度。 */
#define CRSF_RC_PAYLOAD_SIZE            (22U)

/* 当前使用的 UART 实例。 */
static uart_instance_t const * g_crsf_uart = NULL;

/* UART 中断写入、CRSF 任务读取的环形缓冲区。 */
static volatile uint8_t g_crsf_rx_buffer[CRSF_RX_BUFFER_SIZE];

/* 环形缓冲区写入位置。 */
static volatile uint16_t g_crsf_rx_head = 0U;

/* 环形缓冲区读取位置。 */
static volatile uint16_t g_crsf_rx_tail = 0U;

/* 当前正在接收的 CRSF 帧。 */
static uint8_t g_crsf_frame_buffer[CRSF_MAX_FRAME_SIZE];

/* 当前 CRSF 帧已经接收的字节数。 */
static uint8_t g_crsf_frame_index = 0U;

/* 当前 CRSF 帧期望接收的完整字节数。 */
static uint8_t g_crsf_expected_frame_size = 0U;

/* 对外发布的 CRSF 数据。 */
volatile crsf_data_t g_crsf_data = {0};

_Static_assert((CRSF_RX_BUFFER_SIZE & (CRSF_RX_BUFFER_SIZE - 1U)) == 0U, "CRSF_RX_BUFFER_SIZE must be a power of two");


/* 计算 CRSF CRC8，CRC 范围包含帧类型和负载。 */
static uint8_t crsf_crc8(uint8_t const * p_data, uint8_t length)
{
    uint8_t crc = 0U;           /* 当前 CRC 值。 */
    uint8_t byte_index;         /* 当前字节索引。 */
    uint8_t bit_index;          /* 当前位索引。 */

    for (byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= p_data[byte_index];

        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            if (0U != (crc & 0x80U))
            {
                crc = (uint8_t) ((crc << 1U) ^ CRSF_CRC_POLYNOMIAL);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}


/* 校验并解析一帧完整的 CRSF 数据。 */
static void crsf_parse_frame(void)
{
    uint8_t frame_length;                /* 不包含同步字节和长度字节的帧长度。 */
    uint8_t frame_type;                  /* 当前 CRSF 帧类型。 */
    uint8_t received_crc;                /* 帧中携带的 CRC。 */
    uint8_t calculated_crc;              /* 本地计算得到的 CRC。 */
    uint8_t const * p_payload;           /* CRSF 帧负载首地址。 */
    uint8_t channel_index;               /* 当前遥控通道索引。 */
    uint8_t payload_index = 0U;          /* 当前负载字节索引。 */
    uint8_t bit_count = 0U;              /* 位缓存中有效位数量。 */
    uint32_t bit_buffer = 0U;            /* 通道解包使用的位缓存。 */
    uint16_t channel_raw;                /* 当前通道原始值。 */
    int32_t channel_us;                  /* 当前通道换算值，单位 us。 */
    uint8_t active_antenna;              /* 当前活动接收天线编号。 */

    frame_length = g_crsf_frame_buffer[1U];
    frame_type = g_crsf_frame_buffer[2U];
    received_crc = g_crsf_frame_buffer[frame_length + 1U];
    calculated_crc = crsf_crc8(&g_crsf_frame_buffer[2U], (uint8_t) (frame_length - 1U));

    if (received_crc != calculated_crc)
    {
        g_crsf_data.crc_error_count++;
        return;
    }

    p_payload = &g_crsf_frame_buffer[3U];

    if ((CRSF_FRAME_TYPE_RC_CHANNELS == frame_type) && (frame_length >= (CRSF_RC_PAYLOAD_SIZE + 2U)))
    {
        /*
         * 0x16 帧把 16 路通道连续压缩为 22 字节，每路占 11 位。
         * 位缓存按 CRSF 通道负载的低位优先方式依次取出通道值。
         */
        for (channel_index = 0U; channel_index < CRSF_CHANNEL_COUNT; channel_index++)
        {
            while (bit_count < 11U)
            {
                bit_buffer |= ((uint32_t) p_payload[payload_index]) << bit_count;
                payload_index++;
                bit_count = (uint8_t) (bit_count + 8U);
            }

            channel_raw = (uint16_t) (bit_buffer & 0x07FFU);
            bit_buffer >>= 11U;
            bit_count = (uint8_t) (bit_count - 11U);

            channel_us = ((((int32_t) channel_raw - 992) * 5) / 8) + 1500;

            g_crsf_data.channel_raw[channel_index] = channel_raw;
            g_crsf_data.channel_us[channel_index] = (uint16_t) channel_us;
        }

        g_crsf_data.rc_frame_count++;
        g_crsf_data.last_rc_frame_tick = xTaskGetTickCount();
    }
    else if ((CRSF_FRAME_TYPE_LINK_STATISTICS == frame_type) && (frame_length >= 12U))
    {
        /*
         * 0x14 链路状态帧包含两根天线的 RSSI、链路质量、SNR、射频模式和发射功率。
         */
        active_antenna = p_payload[4U];

        if (0U == active_antenna)
        {
            g_crsf_data.rssi_dbm = -(int16_t) p_payload[0U];
        }
        else
        {
            g_crsf_data.rssi_dbm = -(int16_t) p_payload[1U];
        }

        g_crsf_data.link_quality = p_payload[2U];
        g_crsf_data.snr_db = (int8_t) p_payload[3U];
        g_crsf_data.rf_mode = p_payload[5U];
        g_crsf_data.tx_power_index = p_payload[6U];
        g_crsf_data.link_frame_count++;
    }
}


/* FSP UART 中断回调函数。 */
void rc_uart_callback(uart_callback_args_t * p_args)
{
    uint16_t next_head;    /* 环形缓冲区下一个写入位置。 */

    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            next_head = (uint16_t) ((g_crsf_rx_head + 1U) & (CRSF_RX_BUFFER_SIZE - 1U));

            if (next_head != g_crsf_rx_tail)
            {
                g_crsf_rx_buffer[g_crsf_rx_head] = (uint8_t) p_args->data;
                g_crsf_rx_head = next_head;
            }
            else
            {
                g_crsf_data.rx_overflow_count++;
            }

            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        {
            g_crsf_data.uart_error_count++;
            break;
        }

        default:
        {
            break;
        }
    }
}


/* 初始化 CRSF 使用的 UART。 */
fsp_err_t crsf_init(uart_instance_t const * p_uart_instance)
{
    fsp_err_t err;    /* FSP UART 返回值。 */

    if (NULL == p_uart_instance)
    {
        return FSP_ERR_ASSERTION;
    }

    g_crsf_uart = p_uart_instance;
    g_crsf_rx_head = 0U;
    g_crsf_rx_tail = 0U;
    g_crsf_frame_index = 0U;
    g_crsf_expected_frame_size = 0U;
    g_crsf_data = (crsf_data_t) {0};

    err = g_crsf_uart->p_api->open(g_crsf_uart->p_ctrl, g_crsf_uart->p_cfg);

    if (FSP_ERR_ALREADY_OPEN == err)
    {
        return FSP_SUCCESS;
    }

    if (FSP_SUCCESS != err)
    {
        g_crsf_uart = NULL;
    }

    return err;
}


/* 处理 UART 环形缓冲区中的 CRSF 字节流。 */
void crsf_process(void)
{
    uint8_t received_byte;    /* 当前从环形缓冲区取出的字节。 */

    while (g_crsf_rx_tail != g_crsf_rx_head)
    {
        received_byte = g_crsf_rx_buffer[g_crsf_rx_tail];
        g_crsf_rx_tail = (uint16_t) ((g_crsf_rx_tail + 1U) & (CRSF_RX_BUFFER_SIZE - 1U));

        if (0U == g_crsf_frame_index)
        {
            /*
             * 接收机发往飞控的帧通常以 0xC8 开始，同时接受广播和常用 CRSF 设备地址。
             */
            if ((CRSF_ADDRESS_FLIGHT_CONTROLLER == received_byte) || (CRSF_ADDRESS_RECEIVER == received_byte) || (CRSF_ADDRESS_RADIO_TRANSMITTER == received_byte) || (CRSF_ADDRESS_BROADCAST == received_byte))
            {
                g_crsf_frame_buffer[0U] = received_byte;
                g_crsf_frame_index = 1U;
            }

            continue;
        }

        if (1U == g_crsf_frame_index)
        {
            /*
             * CRSF 长度字段有效范围为 2～62，完整帧长度等于长度字段加 2。
             */
            if ((received_byte < 2U) || (received_byte > 62U))
            {
                g_crsf_frame_index = 0U;
                g_crsf_expected_frame_size = 0U;
                continue;
            }

            g_crsf_frame_buffer[1U] = received_byte;
            g_crsf_expected_frame_size = (uint8_t) (received_byte + 2U);
            g_crsf_frame_index = 2U;
            continue;
        }

        g_crsf_frame_buffer[g_crsf_frame_index] = received_byte;
        g_crsf_frame_index++;

        if (g_crsf_frame_index >= g_crsf_expected_frame_size)
        {
            crsf_parse_frame();
            g_crsf_frame_index = 0U;
            g_crsf_expected_frame_size = 0U;
        }
    }
}


/* 获取一份完整的 CRSF 数据快照。 */
void crsf_get_data(crsf_data_t * p_data)
{
    TickType_t current_tick;    /* 当前 FreeRTOS 系统节拍。 */

    if (NULL == p_data)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_data = g_crsf_data;
    taskEXIT_CRITICAL();

    current_tick = xTaskGetTickCount();

    if ((0U != p_data->rc_frame_count) && ((current_tick - p_data->last_rc_frame_tick) <= pdMS_TO_TICKS(CRSF_SIGNAL_TIMEOUT_MS)))
    {
        p_data->connected = true;
    }
    else
    {
        p_data->connected = false;
    }
}
