#include "telemetry_thread.h"

#include "code/esc_bench_test.h"
#include "code/flight_control.h"
#include "code/flight_safety.h"
#include "code/imu.h"
#include "code/project_config.h"
#include "code/rc_command.h"
#include "driver/radio_3dr.h"
#include "driver/crsf.h"
#include "driver/gps_nmea.h"
#include "driver/motor_output.h"
#include "driver/up_tof.h"

#include <string.h>
#include <math.h>

/* 桨载振动模式记录控制器实际看到的 40 Hz 带限角速度，提升到 100 Hz。 */
#if (IMU_DIAGNOSTIC_MODE == IMU_DIAGNOSTIC_MODE_RAW_REREAD)
#define TELEMETRY_PERIOD_TICKS       pdMS_TO_TICKS(50U)
#define TELEMETRY_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(40U)
#elif (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_VIBRATION_BASELINE)
#define TELEMETRY_PERIOD_TICKS       pdMS_TO_TICKS(10U)
#define TELEMETRY_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(10U)
#else
/* 其他遥测保持 20 ms，即 50 Hz。 */
#define TELEMETRY_PERIOD_TICKS       pdMS_TO_TICKS(20U)

/* 3DR UART 单帧发送超时时间。 */
#define TELEMETRY_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(20U)
#endif

#if (IMU_DIAGNOSTIC_MODE == IMU_DIAGNOSTIC_MODE_RAW_REREAD)
/* 20 Hz、51 通道：原始首读/复读、是否替换以及滤波后三轴。 */
#define TELEMETRY_CHANNEL_COUNT      (51U)
#elif (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
/* 系留短跳以 50 Hz 发送精简的 14 通道关键数据。 */
#define TELEMETRY_CHANNEL_COUNT      (14U)
#else
/* 普通模式使用 8 个数据通道。 */
#define TELEMETRY_CHANNEL_COUNT      (8U)
#endif

/* JustFloat 帧长度：数据 float 加 4 字节帧尾。 */
#define TELEMETRY_FRAME_SIZE         ((TELEMETRY_CHANNEL_COUNT * sizeof(float)) + sizeof(uint32_t))

/* VOFA+ JustFloat 帧尾。 */
#define VOFA_JUSTFLOAT_TAIL          (0x7F800000UL)

/* 等待 3DR 数传模块完成上电启动后再发送首帧。 */
#define TELEMETRY_POWER_STABLE_DELAY_MS    (1500U)


/* 通过 3DR 数传向 VOFA+ 发送前 6 路 CRSF 遥控通道。 */
void telemetry_thread_entry(void * pvParameters)
{
    TickType_t last_wake_time;                         /* 遥测任务周期基准。 */
    radio_3dr_status_t radio_status;                   /* 3DR 数传操作状态。 */
#if (IMU_DIAGNOSTIC_MODE == IMU_DIAGNOSTIC_MODE_RAW_REREAD)
    imu_raw_diagnostic_t raw_diagnostic;               /* 原始异常捕获快照。 */
    imu_attitude_t attitude;                            /* 守卫后的滤波角速度。 */
#elif (ESC_BENCH_MODE != ESC_BENCH_MODE_DISABLED)
    esc_bench_status_t bench_status;                   /* ESC 台架状态。 */
#elif (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
    imu_attitude_t attitude;                           /* 系留姿态与角速度快照。 */
    flight_control_status_t control_status;            /* 系留控制器状态。 */
    rc_command_t command;                              /* 系留遥控指令。 */
    imu_raw_diagnostic_t raw_diagnostic;               /* 原始守卫累计状态。 */
#elif (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_VIBRATION_BASELINE)
    imu_attitude_t attitude;                           /* 桨载滤波角速度快照。 */
#else
#if ((TELEMETRY_SOURCE == TELEMETRY_SOURCE_EULER) || \
     (TELEMETRY_SOURCE == TELEMETRY_SOURCE_IMU_CALIBRATION))
    imu_attitude_t attitude;                           /* IMU 姿态快照。 */
#if (TELEMETRY_SOURCE == TELEMETRY_SOURCE_IMU_CALIBRATION)
    imu_calibration_t calibration;                     /* IMU 标定结果。 */
#endif
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_RC_COMMAND)
    rc_command_t command;                              /* 归一化遥控指令。 */
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_GPS)
    gps_data_t gps_data;                               /* GPS 定位数据快照。 */
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_FLOW_TOF)
    up_tof_data_t flow_data;                           /* 光流 TOF 数据快照。 */
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_FLIGHT_CONTROL)
    flight_control_status_t control_status;            /* 影子控制计算快照。 */
#if (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_PID_I_SHADOW)
    rc_command_t command;                              /* 归一化遥控指令。 */
#endif
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_MOTOR_OUTPUT)
#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_LEVEL) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_RATE) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_CASCADE) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE))
    imu_attitude_t attitude;                           /* IMU 台架测试快照。 */
#if ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE) || \
     (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE))
    rc_command_t command;                              /* 遥控姿态测试指令。 */
#endif
#else
    rc_command_t command;                              /* 归一化遥控指令。 */
#endif
#else
    crsf_data_t rc_data;                               /* CRSF 遥控数据快照。 */
#endif
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

    vTaskDelay(pdMS_TO_TICKS(TELEMETRY_POWER_STABLE_DELAY_MS));

    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        for (channel_index = 0U;
             channel_index < TELEMETRY_CHANNEL_COUNT;
             channel_index++)
        {
            channel_data[channel_index] = 0.0f;
        }

#if (IMU_DIAGNOSTIC_MODE == IMU_DIAGNOSTIC_MODE_RAW_REREAD)
        imu_get_raw_diagnostic(&raw_diagnostic);
        imu_get_attitude(&attitude);
        channel_data[0] = raw_diagnostic.valid ? 1.0f : 0.0f;
        channel_data[1] = raw_diagnostic.used_reread ? 1.0f : 0.0f;
        channel_data[2] = (float) raw_diagnostic.capture_tick / 1000.0f;
        channel_data[3] = (float) raw_diagnostic.trigger_mask;
        channel_data[4] = (float) raw_diagnostic.reread_status;
        channel_data[5] = (float) raw_diagnostic.differing_byte_count;

        channel_data[6] = (float) raw_diagnostic.current.accel_x;
        channel_data[7] = (float) raw_diagnostic.current.accel_y;
        channel_data[8] = (float) raw_diagnostic.current.accel_z;
        channel_data[9] = (float) raw_diagnostic.current.gyro_x;
        channel_data[10] = (float) raw_diagnostic.current.gyro_y;
        channel_data[11] = (float) raw_diagnostic.current.gyro_z;

        channel_data[12] = (float) raw_diagnostic.first.accel_x;
        channel_data[13] = (float) raw_diagnostic.first.accel_y;
        channel_data[14] = (float) raw_diagnostic.first.accel_z;
        channel_data[15] = (float) raw_diagnostic.first.gyro_x;
        channel_data[16] = (float) raw_diagnostic.first.gyro_y;
        channel_data[17] = (float) raw_diagnostic.first.gyro_z;

        channel_data[18] = (float) raw_diagnostic.reread.accel_x;
        channel_data[19] = (float) raw_diagnostic.reread.accel_y;
        channel_data[20] = (float) raw_diagnostic.reread.accel_z;
        channel_data[21] = (float) raw_diagnostic.reread.gyro_x;
        channel_data[22] = (float) raw_diagnostic.reread.gyro_y;
        channel_data[23] = (float) raw_diagnostic.reread.gyro_z;

        for (channel_index = 0U;
             channel_index < ICM42688_RAW_BYTE_COUNT;
             channel_index++)
        {
            channel_data[24U + channel_index] =
                (float) raw_diagnostic.first.bytes[channel_index];
            channel_data[36U + channel_index] =
                (float) raw_diagnostic.reread.bytes[channel_index];
        }
        channel_data[48] = attitude.gyro_x_dps;
        channel_data[49] = attitude.gyro_y_dps;
        channel_data[50] = attitude.gyro_z_dps;
#elif (ESC_BENCH_MODE != ESC_BENCH_MODE_DISABLED)
        esc_bench_test_get_status(&bench_status);

        for (channel_index = 0U;
             channel_index < MOTOR_OUTPUT_COUNT;
             channel_index++)
        {
            channel_data[channel_index] =
                (float) motor_output_get_us(channel_index);
        }

        channel_data[4] = (float) bench_status.phase;
        channel_data[5] = (float) bench_status.active_motor;
        channel_data[6] = motor_output_is_ready() ? 1.0f : 0.0f;
        channel_data[7] = (float) bench_status.elapsed_ms / 1000.0f;
#elif (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        for (channel_index = 0U;
             channel_index < MOTOR_OUTPUT_COUNT;
             channel_index++)
        {
            channel_data[channel_index] =
                (float) motor_output_get_us(channel_index);
        }

        imu_get_attitude(&attitude);
        flight_control_get_status(&control_status);
        rc_command_get(&command);
        imu_get_raw_diagnostic(&raw_diagnostic);
        channel_data[4] = command.throttle;
        channel_data[5] = attitude.roll_deg;
        channel_data[6] = attitude.pitch_deg;
        channel_data[7] = attitude.yaw_deg;
        channel_data[8] = fmaxf(fabsf(attitude.gyro_x_dps),
                                fmaxf(fabsf(attitude.gyro_y_dps),
                                      fabsf(attitude.gyro_z_dps)));
        channel_data[9] = (float) flight_safety_get_state();
        channel_data[10] = (float) control_status.fault_reason;
        channel_data[11] = (float) raw_diagnostic.replacement_count;
        channel_data[12] = (float) raw_diagnostic.reread_failure_count;
        channel_data[13] = control_status.valid ? 1.0f : 0.0f;
#elif (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_VIBRATION_BASELINE)
        for (channel_index = 0U;
             channel_index < MOTOR_OUTPUT_COUNT;
             channel_index++)
        {
            channel_data[channel_index] =
                (float) motor_output_get_us(channel_index);
        }

        imu_get_attitude(&attitude);
        channel_data[4] = attitude.gyro_x_dps;
        channel_data[5] = attitude.gyro_y_dps;
        channel_data[6] = attitude.gyro_z_dps;
        channel_data[7] = (float) flight_safety_get_state();
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_IMU_CALIBRATION)
        imu_get_calibration(&calibration);
        imu_get_attitude(&attitude);
        channel_data[0] = calibration.gyro_offset_x_dps;
        channel_data[1] = calibration.gyro_offset_y_dps;
        channel_data[2] = calibration.gyro_offset_z_dps;
        channel_data[3] = attitude.roll_deg;
        channel_data[4] = attitude.pitch_deg;
        channel_data[5] = attitude.yaw_deg;
        channel_data[6] = (float) calibration.state;
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_EULER)
        imu_get_attitude(&attitude);
        channel_data[0] = attitude.roll_deg;
        channel_data[1] = attitude.pitch_deg;
        channel_data[2] = attitude.yaw_deg;
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_RC_COMMAND)
        rc_command_get(&command);
        channel_data[0] = command.roll;
        channel_data[1] = command.pitch;
        channel_data[2] = command.throttle;
        channel_data[3] = command.yaw;
        channel_data[4] = command.arm_switch_high ? 1.0f : 0.0f;
        channel_data[5] = (float) command.mode;
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_GPS)
        gps_get_data(&gps_data);
        channel_data[0] = (float) gps_data.latitude_deg_e7 / 10000000.0f;
        channel_data[1] = (float) gps_data.longitude_deg_e7 / 10000000.0f;
        channel_data[2] = (float) gps_data.altitude_cm / 100.0f;
        channel_data[3] = (float) gps_data.ground_speed_cm_s / 100.0f;
        channel_data[4] = (float) gps_data.satellites;
        channel_data[5] = gps_data.valid ? 1.0f : 0.0f;
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_FLOW_TOF)
        up_tof_get_data(&flow_data);
        channel_data[0] = (float) flow_data.distance_mm / 1000.0f;
        channel_data[1] = (float) flow_data.velocity_x_cm_s;
        channel_data[2] = (float) flow_data.velocity_y_cm_s;
        channel_data[3] = (float) flow_data.flow_x_integral;
        channel_data[4] = (float) flow_data.flow_y_integral;
        channel_data[5] = flow_data.valid ? (float) flow_data.tof_confidence : 0.0f;
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_FLIGHT_CONTROL)
        flight_control_get_status(&control_status);
#if (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_PID_I_SHADOW)
        rc_command_get(&command);
#endif
        channel_data[0] = control_status.roll_correction_us;
        channel_data[1] = control_status.pitch_correction_us;
        channel_data[2] = control_status.yaw_correction_us;
        channel_data[3] = control_status.base_us;
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW)
        channel_data[4] = control_status.roll_integrator_us;
        channel_data[5] = control_status.pitch_integrator_us;
        channel_data[6] = control_status.yaw_integrator_us;
#else
        channel_data[4] = command.roll;
        channel_data[5] = command.pitch;
        channel_data[6] = command.yaw;
#endif
        channel_data[7] = (float) flight_safety_get_state();
#elif (TELEMETRY_SOURCE == TELEMETRY_SOURCE_MOTOR_OUTPUT)
        for (channel_index = 0U;
             channel_index < MOTOR_OUTPUT_COUNT;
             channel_index++)
        {
            channel_data[channel_index] =
                (float) motor_output_get_us(channel_index);
        }
#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_LEVEL)
        imu_get_attitude(&attitude);
        channel_data[4] = attitude.roll_deg;
        channel_data[5] = attitude.pitch_deg;
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_RATE)
        imu_get_attitude(&attitude);
        channel_data[4] = attitude.gyro_x_dps;
        channel_data[5] = attitude.gyro_y_dps;
        channel_data[6] = attitude.gyro_z_dps;
        channel_data[7] = (float) flight_safety_get_state();
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_CASCADE)
        imu_get_attitude(&attitude);
        channel_data[4] = attitude.roll_deg;
        channel_data[5] = attitude.pitch_deg;
        channel_data[6] = attitude.gyro_x_dps;
        channel_data[7] = attitude.gyro_y_dps;
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE)
        rc_command_get(&command);
        imu_get_attitude(&attitude);
        channel_data[4] = command.roll;
        channel_data[5] = attitude.roll_deg;
        channel_data[6] = command.pitch;
        channel_data[7] = attitude.pitch_deg;
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE)
        rc_command_get(&command);
        imu_get_attitude(&attitude);
        channel_data[4] = command.yaw;
        channel_data[5] = attitude.gyro_z_dps;
        channel_data[6] = (float) flight_safety_get_state();
        channel_data[7] = imu_is_ready() ? 1.0f : 0.0f;
#elif ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
       (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL))
        rc_command_get(&command);
        channel_data[4] = command.roll;
        channel_data[5] = command.pitch;
        channel_data[6] = command.yaw;
        channel_data[7] = (float) flight_safety_get_state();
#else
        rc_command_get(&command);
        channel_data[4] = command.throttle;
        channel_data[5] = command.arm_switch_high ? 1.0f : 0.0f;
#endif
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

#if ((ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED) && \
     (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_DISABLED) && \
     (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_DISABLED) && \
     (TELEMETRY_SOURCE != TELEMETRY_SOURCE_IMU_CALIBRATION) && \
     (TELEMETRY_SOURCE != TELEMETRY_SOURCE_FLIGHT_CONTROL) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_IMU_RATE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_IMU_CASCADE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_RC_ATTITUDE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_RC_YAW_RATE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_FULL_CONTROL) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_POWERED_CONTROL))
        channel_data[6] = (float) flight_safety_get_state();
#endif
#if ((ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED) && \
     (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_DISABLED) && \
     (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_DISABLED) && \
     (TELEMETRY_SOURCE != TELEMETRY_SOURCE_FLIGHT_CONTROL) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_IMU_RATE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_IMU_CASCADE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_RC_ATTITUDE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_RC_YAW_RATE) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_FULL_CONTROL) && \
     (CONTROL_BENCH_MODE != CONTROL_BENCH_MODE_POWERED_CONTROL))
        channel_data[7] = imu_is_ready() ? 1.0f : 0.0f;
#endif

        /* 专用测试模式优先显示测试数据；正常模式显示所选遥测源。 */
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
