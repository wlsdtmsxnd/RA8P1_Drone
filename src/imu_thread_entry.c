#include "imu_thread.h"
#include "code/esc_bench_test.h"
#include "code/flight_safety.h"
#include "code/flight_control.h"
#include "code/imu.h"
#include "code/imu_feedback_bench_test.h"
#include "code/imu_cascade_bench_test.h"
#include "code/imu_rate_bench_test.h"
#include "code/mixer_bench_test.h"
#include "code/project_config.h"
#include "code/rc_attitude_bench_test.h"
#include "code/rc_yaw_rate_bench_test.h"
#include "driver/motor_output.h"

/* ICM42688 软件片选引脚：P710。 */
#define ICM42688_CS_PIN    BSP_IO_PORT_07_PIN_10

/* ICM42688 由 5V_EXT 供电，等待外设电源和板载稳压稳定。 */
#define ICM42688_POWER_STABLE_DELAY_MS    (100U)


/* IMU 任务入口：固定 2 ms 周期运行姿态解算。 */
void imu_thread_entry(void * pvParameters)
{
    TickType_t last_wake_time;       /* 上一次周期唤醒时刻。 */
#if (ESC_BENCH_MODE == ESC_BENCH_MODE_DISABLED)
    imu_status_t imu_status;         /* IMU 初始化或更新状态。 */
#endif
    motor_output_status_t motor_status;

    FSP_PARAMETER_NOT_USED(pvParameters);

    /* 先启动 1000 us 安全脉宽，再初始化任何可能失败的传感器。 */
    motor_status = motor_output_init();

    if (MOTOR_OUTPUT_STATUS_OK != motor_status)
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

    imu_status = imu_init(&g_spi_imu,
                          ICM42688_CS_PIN);

    if (IMU_STATUS_OK != imu_status)
    {
        /*
         * 初始化失败。
         * 调试时观察 imu_status，并在 icm42688_init 内查看具体返回位置。
         */
        motor_output_all_stop();

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

        /* IMU 错误、遥控失联或未满足解锁条件都会强制 1000 us。 */
        flight_safety_update(IMU_STATUS_OK == imu_status);

#if (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_STICK_MIXER)
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
       (CONTROL_BENCH_MODE == CONTROL_BENCH_MODE_SHADOW_CONTROL))
        flight_control_update(IMU_STATUS_OK == imu_status);
#endif

        /*
         * 绝对周期延时，减少任务执行时间引起的周期累计误差。
         * FreeRTOS Tick Rate 需要配置为 1000 Hz。
         */
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(2U));
    }
#endif
}
