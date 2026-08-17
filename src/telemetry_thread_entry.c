#include "telemetry_thread.h"

#include "code/flight_safety.h"
#include "code/imu.h"
#include "code/project_config.h"
#include "driver/radio_3dr.h"
#include "driver/crsf.h"

#include <string.h>

/* 遥测发送周期：20 ms，即 50 Hz。 */
#define TELEMETRY_PERIOD_TICKS       pdMS_TO_TICKS(20U)

/* 3DR UART 单帧发送超时时间。 */
#define TELEMETRY_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(20U)

/* 6 个数据通道，加安全状态和 IMU 就绪标志。 */
#define TELEMETRY_CHANNEL_COUNT      (8U)

/* JustFloat 帧长度：8 个 float 加 4 字节帧尾。 */
#define TELEMETRY_FRAME_SIZE         ((TELEMETRY_CHANNEL_COUNT * sizeof(float)) + sizeof(uint32_t))

/* VOFA+ JustFloat 帧尾。 */
#define VOFA_JUSTFLOAT_TAIL          (0x7F800000UL)


/* 通过 3DR 数传向 VOFA+ 发送前 6 路 CRSF 遥控通道。 */
void telemetry_thread_entry(void * pvParameters)
{
    TickType_t last_wake_time;                         /* 遥测任务周期基准。 */
    radio_3dr_status_t radio_status;                   /* 3DR 数传操作状态。 */
#if (TELEMETRY_SOURCE == TELEMETRY_SOURCE_EULER)
    imu_attitude_t attitude;                           /* IMU 姿态快照。 */
#else
    crsf_data_t rc_data;                               /* CRSF 遥控数据快照。 */
#endif
    float channel_data[TELEMETRY_CHANNEL_COUNT];       /* VOFA+ 数据通道。 */
    uint8_t frame_buffer[TELEMETRY_FRAME_SIZE];        /* JustFloat 数据帧。 */
    uint32_t frame_tail = VOFA_JUSTFLOAT_TAIL;         /* JustFloat 帧尾。 */
    uint32_t channel_index;                            /* 遥控通道索引。 */

    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 初始化 3DR 数传使用的 UART。 */
    radio_status = radio_3dr_init(&g_uart_radio);

    if (RADIO_3DR_STATUS_OK != radio_status)
    {
        /* 数传 UART 初始化失败时暂停当前任务。 */
        vTaskSuspend(NULL);
    }

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        for (channel_index = 0U;
             channel_index < TELEMETRY_CHANNEL_COUNT;
             channel_index++)
        {
            channel_data[channel_index] = 0.0f;
        }

#if (TELEMETRY_SOURCE == TELEMETRY_SOURCE_EULER)
        imu_get_attitude(&attitude);
        channel_data[0] = attitude.roll_deg;
        channel_data[1] = attitude.pitch_deg;
        channel_data[2] = attitude.yaw_deg;
#else
        /* 获取同一时刻的一组 CRSF 遥控数据。 */
        crsf_get_data(&rc_data);

        if (true == rc_data.connected)
        {
            /* CH1～CH6 依次发送到 VOFA+ 的 I0～I5。 */
            for (channel_index = 0U;
                 channel_index < 6U;
                 channel_index++)
            {
                channel_data[channel_index] = (float) rc_data.channel_us[channel_index];
            }
        }
#endif

        channel_data[6] = (float) flight_safety_get_state();
        channel_data[7] = imu_is_ready() ? 1.0f : 0.0f;

        /* I0-I5 为所选数据，I6 为安全状态，I7 为 IMU 就绪。 */
        memcpy(frame_buffer, channel_data, sizeof(channel_data));
        memcpy(&frame_buffer[sizeof(channel_data)], &frame_tail, sizeof(frame_tail));

        radio_status = radio_3dr_write(frame_buffer,
                                      TELEMETRY_FRAME_SIZE,
                                      TELEMETRY_TX_TIMEOUT_TICKS);

        if (RADIO_3DR_STATUS_OK != radio_status)
        {
            /*
             * 数传发送失败时丢弃当前帧。
             * 不阻塞更高优先级的遥控接收和 IMU 任务。
             */
        }

        vTaskDelayUntil(&last_wake_time, TELEMETRY_PERIOD_TICKS);
    }
}
