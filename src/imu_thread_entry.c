#include "imu_thread.h"
#include "code/actuator_manager.h"
#include "code/esc_bench_test.h"
#include "code/flight_safety.h"
#include "code/flight_control.h"
#include "code/flight_snapshot.h"
#include "code/flow_navigation.h"
#include "code/imu.h"
#include "code/imu_feedback_bench_test.h"
#include "code/imu_cascade_bench_test.h"
#include "code/imu_rate_bench_test.h"
#include "code/mixer_bench_test.h"
#include "code/project_config.h"
#include "code/rc_attitude_bench_test.h"
#include "code/rc_yaw_rate_bench_test.h"

/* ICM42688 软件片选引脚：P710。 */
#define ICM42688_CS_PIN    BSP_IO_PORT_07_PIN_10

/* ICM42688 由 5V_EXT 供电，等待外设电源和板载稳压稳定。 */
#define ICM42688_POWER_STABLE_DELAY_MS    (1000U)

/*
 * 主电源冷启动时，ESC 上电提示音会通过电机/桨叶传到 IMU。
 * 只对“静止标定期间检测到运动”做有限次重试；SPI、设备 ID
 * 或寄存器错误仍立即停机，不用重试掩盖硬件故障。
 */
#define IMU_CALIBRATION_MAX_ATTEMPTS      (3U)
#define IMU_CALIBRATION_RETRY_DELAY_MS    (1000U)


/* IMU 任务入口：固定 2 ms 周期运行姿态解算。 */
void imu_thread_entry(void * pvParameters)
{
    TickType_t last_wake_time;       /* 上一次周期唤醒时刻。 */
#if (ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED)
    imu_status_t imu_status;         /* IMU 初始化或更新状态。 */
    uint32_t imu_calibration_attempt; /* 静止标定尝试次数。 */
    imu_attitude_t navigation_attitude;
#endif
    actuator_manager_status_t actuator_status;

    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 先启动 1000 us 安全脉宽，再初始化任何可能失败的传感器。 */
    actuator_status = actuator_manager_init();

    if (ACTUATOR_MANAGER_STATUS_OK != actuator_status)
    {
        vTaskSuspend(NULL);
    }

#if (ESC_BENCH_MODE != ESC_BENCH_MODE_DISABLED)
    esc_bench_test_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        esc_bench_test_update();
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(2U));
    }
#else
    vTaskDelay(pdMS_TO_TICKS(ICM42688_POWER_STABLE_DELAY_MS));

    imu_calibration_attempt = 0U;

    do
    {
        imu_calibration_attempt++;
        imu_status = imu_init(&g_spi_imu,
                              ICM42688_CS_PIN);

        if ((IMU_STATUS_CALIBRATION_MOTION == imu_status) &&
            (imu_calibration_attempt < IMU_CALIBRATION_MAX_ATTEMPTS))
        {
            /* 重试等待期间始终保持四路 1000 us。 */
            (void) actuator_manager_stop();
            vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_RETRY_DELAY_MS));
        }
    }
    while ((IMU_STATUS_CALIBRATION_MOTION == imu_status) &&
           (imu_calibration_attempt < IMU_CALIBRATION_MAX_ATTEMPTS));

    if (IMU_STATUS_OK != imu_status)
    {
        /*
         * 初始化失败。
         * 调试时观察 imu_status，并在 icm42688_init 内查看具体返回位置。
         */
        (void) actuator_manager_inhibit();

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    flight_safety_init();
    last_wake_time = xTaskGetTickCount();

    while (1)
    {
        imu_status = imu_update();

        imu_get_attitude(&navigation_attitude);
        flow_navigation_update(&navigation_attitude,
                               IMU_STATUS_OK == imu_status,
                               0.002f);

        /* IMU 错误、遥控失联或未满足解锁条件都会强制 1000 us。 */
        flight_safety_update(IMU_STATUS_OK == imu_status);

#if (IMU_DIAGNOSTIC_MODE == IMU_DIAGNOSTIC_MODE_RAW_REREAD)
        /* 诊断模式不运行任何控制器，并在每个 2 ms 周期重申安全输出。 */
        (void) actuator_manager_inhibit();
#elif (TETHERED_FLIGHT_MODE == TETHERED_FLIGHT_MODE_FIRST_HOP)
        flight_control_update(IMU_STATUS_OK == imu_status);
#elif (PROP_LOAD_TEST_MODE == PROP_LOAD_TEST_MODE_VIBRATION_BASELINE)
        flight_control_prop_load_vibration_update(
            IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_STICK_MIXER)
        mixer_bench_test_update(IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_LEVEL)
        imu_feedback_bench_test_update(IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_RATE)
        imu_rate_bench_test_update(IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_IMU_CASCADE)
        imu_cascade_bench_test_update(IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_ATTITUDE)
        rc_attitude_bench_test_update(IMU_STATUS_OK == imu_status);
#elif (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_RC_YAW_RATE)
        rc_yaw_rate_bench_test_update(IMU_STATUS_OK == imu_status);
#elif ((CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_FULL_CONTROL) || \
       (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_POWERED_CONTROL) || \
       (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL) || \
       (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_PID_I_SHADOW))
        flight_control_update(IMU_STATUS_OK == imu_status);
#endif

        /* 遥测只读取这一份控制周期末尾快照，避免跨周期拼接。 */
        flight_snapshot_publish(IMU_STATUS_OK == imu_status);

        /*
         * 绝对周期延时，减少任务执行时间引起的周期累计误差。
         * FreeRTOS Tick Rate 需要配置为 1000 Hz。
         */
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(2U));
    }
#endif
}
