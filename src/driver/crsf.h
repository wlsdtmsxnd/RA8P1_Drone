#ifndef DRIVER_CRSF_H_
#define DRIVER_CRSF_H_

#include "hal_data.h"
#include "r_uart_api.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdint.h>

/* CRSF 遥控通道数量。 */
#define CRSF_CHANNEL_COUNT              (16U)

/* 超过该时间没有收到遥控通道帧，认为遥控链路断开。 */
#define CRSF_SIGNAL_TIMEOUT_MS          (300U)

/* CRSF 接收数据。 */
typedef struct
{
    uint16_t channel_raw[CRSF_CHANNEL_COUNT];    /* 16 路 CRSF 原始通道值。 */
    uint16_t channel_us[CRSF_CHANNEL_COUNT];     /* 16 路换算后的脉宽值，单位 us。 */
    int16_t rssi_dbm;                            /* 当前接收信号强度，单位 dBm。 */
    uint8_t link_quality;                        /* 当前链路质量，范围 0～100。 */
    int8_t snr_db;                               /* 当前信噪比，单位 dB。 */
    uint8_t rf_mode;                             /* 接收机报告的射频模式编号。 */
    uint8_t tx_power_index;                      /* 接收机报告的发射功率编号。 */
    uint32_t rc_frame_count;                     /* 已通过 CRC 校验的遥控通道帧数量。 */
    uint32_t link_frame_count;                   /* 已通过 CRC 校验的链路状态帧数量。 */
    uint32_t crc_error_count;                    /* CRSF 帧 CRC 错误数量。 */
    uint32_t uart_error_count;                   /* UART 溢出、帧错误和校验错误数量。 */
    uint32_t rx_overflow_count;                  /* 软件接收环形缓冲区溢出数量。 */
    TickType_t last_rc_frame_tick;               /* 最近一次遥控通道帧的系统节拍。 */
    bool connected;                              /* 当前遥控链路是否有效。 */
} crsf_data_t;

/*
 * CRSF 全局调试数据。
 * 调试器可以直接观察该变量；其他任务读取数据时建议调用 crsf_get_data()。
 */
extern volatile crsf_data_t g_crsf_data;

/* FSP UART 回调函数。 */
void rc_uart_callback(uart_callback_args_t * p_args);

/* 初始化 CRSF 使用的 UART。 */
fsp_err_t crsf_init(uart_instance_t const * p_uart_instance);

/* 处理 UART 接收缓冲区中的 CRSF 数据。 */
void crsf_process(void);

/* 获取一份完整的 CRSF 数据快照。 */
void crsf_get_data(crsf_data_t * p_data);

#endif /* DRIVER_CRSF_H_ */
