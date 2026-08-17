#include "imu.h"
#include "../driver/icm42688.h"

#include "FreeRTOS.h"
#include "task.h"

#include <limits.h>
#include <math.h>

/* 姿态算法周期：2 ms，即 500 Hz。 */
#define IMU_SAMPLE_PERIOD_S               (0.002f)

/* 角度转换常数。 */
#define IMU_PI_F                          (3.14159265358979323846f)
#define IMU_DEG_TO_RAD                    (IMU_PI_F / 180.0f)
#define IMU_RAD_TO_DEG                    (180.0f / IMU_PI_F)

/* 加速度计一阶低通滤波系数。 */
#define IMU_ACCEL_LPF_ALPHA               (0.30f)

/* Mahony 比例和积分增益。 */
#define IMU_MAHONY_KP                     (1.50f)
#define IMU_MAHONY_KI                     (0.004f)

/* 陀螺仪静止零偏标定参数。 */
#define IMU_CALIBRATION_SETTLE_MS          (1000U)
#define IMU_CALIBRATION_SAMPLES            (2000U)
#define IMU_CALIBRATION_DELAY_MS          (2U)
#define IMU_CALIBRATION_GYRO_SPAN_COUNTS  (100)
#define IMU_CALIBRATION_ACCEL_NORM_MIN    (0.64f)
#define IMU_CALIBRATION_ACCEL_NORM_MAX    (1.44f)

/* 拆装 IMU 后先保持为 0，根据本次水平静置结果重新填写。 */
#define IMU_LEVEL_ROLL_TRIM_DEG           (0.0f)
#define IMU_LEVEL_PITCH_TRIM_DEG          (-0.97f)

/*
 * 二阶巴特沃斯：
 * 采样频率 500 Hz，截止频率 40 Hz。
 */
#define IMU_BUTTER_B0                     (0.0461318f)
#define IMU_BUTTER_B1                     (0.0922636f)
#define IMU_BUTTER_B2                     (0.0461318f)
#define IMU_BUTTER_A1                     (-1.3072850f)
#define IMU_BUTTER_A2                     (0.4918122f)

/* 二阶滤波器历史状态。 */
typedef struct
{
    float x1;     /* 上一次输入。 */
    float x2;     /* 上上次输入。 */
    float y1;     /* 上一次输出。 */
    float y2;     /* 上上次输出。 */
} imu_butterworth_state_t;

/* 陀螺仪三轴零偏，单位为原始计数。 */
static float g_gyro_offset_x = 0.0f;
static float g_gyro_offset_y = 0.0f;
static float g_gyro_offset_z = 0.0f;

/* 加速度计一阶低通输出，单位为 g。 */
static float g_accel_lpf_x = 0.0f;
static float g_accel_lpf_y = 0.0f;
static float g_accel_lpf_z = 0.0f;

/* 陀螺仪三轴二阶巴特沃斯状态。 */
static imu_butterworth_state_t g_gyro_filter_x = {0};
static imu_butterworth_state_t g_gyro_filter_y = {0};
static imu_butterworth_state_t g_gyro_filter_z = {0};

/* Mahony 四元数。 */
static float g_q0 = 1.0f;
static float g_q1 = 0.0f;
static float g_q2 = 0.0f;
static float g_q3 = 0.0f;

/* 相对航向参考，在首次更新或解锁时重置。 */
static float g_yaw_absolute_deg = 0.0f;
static float g_yaw_reference_deg = 0.0f;
static bool g_yaw_reference_valid = false;

/* Mahony 误差积分。 */
static float g_integral_x = 0.0f;
static float g_integral_y = 0.0f;
static float g_integral_z = 0.0f;

/* 对外发布的姿态。 */
static imu_attitude_t g_attitude = {0};

/* 对外发布的启动标定状态和结果。 */
static imu_calibration_t g_calibration = {0};

/* IMU 初始化完成标志。 */
static bool g_imu_ready = false;


/* 二阶巴特沃斯滤波。 */
static float imu_butterworth_update(imu_butterworth_state_t * p_state,
                                    float input)
{
    float output;     /* 本次滤波输出。 */

    output = (IMU_BUTTER_B0 * input) +
             (IMU_BUTTER_B1 * p_state->x1) +
             (IMU_BUTTER_B2 * p_state->x2) -
             (IMU_BUTTER_A1 * p_state->y1) -
             (IMU_BUTTER_A2 * p_state->y2);

    p_state->x2 = p_state->x1;
    p_state->x1 = input;
    p_state->y2 = p_state->y1;
    p_state->y1 = output;

    return output;
}


/* 将角度限制到 [-180, 180]。 */
static float imu_wrap_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}


/* Mahony 四元数更新。 */
static void imu_mahony_update(float gyro_x_rad_s,
                              float gyro_y_rad_s,
                              float gyro_z_rad_s,
                              float accel_x_g,
                              float accel_y_g,
                              float accel_z_g)
{
    float accel_norm_squared;    /* 加速度模长平方。 */
    float accel_norm_inv;        /* 加速度模长倒数。 */
    float gravity_x;             /* 估计重力 X 分量。 */
    float gravity_y;             /* 估计重力 Y 分量。 */
    float gravity_z;             /* 估计重力 Z 分量。 */
    float error_x;               /* 重力叉乘误差 X。 */
    float error_y;               /* 重力叉乘误差 Y。 */
    float error_z;               /* 重力叉乘误差 Z。 */
    float q_dot0;                /* 四元数导数 q0。 */
    float q_dot1;                /* 四元数导数 q1。 */
    float q_dot2;                /* 四元数导数 q2。 */
    float q_dot3;                /* 四元数导数 q3。 */
    float quaternion_norm_inv;   /* 四元数模长倒数。 */

    accel_norm_squared = (accel_x_g * accel_x_g) +
                         (accel_y_g * accel_y_g) +
                         (accel_z_g * accel_z_g);

    /*
     * 只有加速度模长接近 1 g 时才用重力方向纠偏。
     * 大幅平移、撞击或剧烈振动时，暂时只积分陀螺仪。
     */
    if ((accel_norm_squared > 0.49f) &&
        (accel_norm_squared < 1.69f))
    {
        accel_norm_inv = 1.0f / sqrtf(accel_norm_squared);

        accel_x_g *= accel_norm_inv;
        accel_y_g *= accel_norm_inv;
        accel_z_g *= accel_norm_inv;

        gravity_x = 2.0f * ((g_q1 * g_q3) - (g_q0 * g_q2));
        gravity_y = 2.0f * ((g_q0 * g_q1) + (g_q2 * g_q3));
        gravity_z = (g_q0 * g_q0) - (g_q1 * g_q1) -
                    (g_q2 * g_q2) + (g_q3 * g_q3);

        error_x = (accel_y_g * gravity_z) - (accel_z_g * gravity_y);
        error_y = (accel_z_g * gravity_x) - (accel_x_g * gravity_z);
        error_z = (accel_x_g * gravity_y) - (accel_y_g * gravity_x);

        g_integral_x += IMU_MAHONY_KI * error_x * IMU_SAMPLE_PERIOD_S;
        g_integral_y += IMU_MAHONY_KI * error_y * IMU_SAMPLE_PERIOD_S;
        g_integral_z += IMU_MAHONY_KI * error_z * IMU_SAMPLE_PERIOD_S;

        gyro_x_rad_s += (IMU_MAHONY_KP * error_x) + g_integral_x;
        gyro_y_rad_s += (IMU_MAHONY_KP * error_y) + g_integral_y;
        gyro_z_rad_s += (IMU_MAHONY_KP * error_z) + g_integral_z;
    }

    /*
     * 使用同一组旧四元数计算四个导数，
     * 避免逐项原地更新引入误差。
     */
    q_dot0 = 0.5f * ((-g_q1 * gyro_x_rad_s) -
                     ( g_q2 * gyro_y_rad_s) -
                     ( g_q3 * gyro_z_rad_s));

    q_dot1 = 0.5f * (( g_q0 * gyro_x_rad_s) +
                     ( g_q2 * gyro_z_rad_s) -
                     ( g_q3 * gyro_y_rad_s));

    q_dot2 = 0.5f * (( g_q0 * gyro_y_rad_s) -
                     ( g_q1 * gyro_z_rad_s) +
                     ( g_q3 * gyro_x_rad_s));

    q_dot3 = 0.5f * (( g_q0 * gyro_z_rad_s) +
                     ( g_q1 * gyro_y_rad_s) -
                     ( g_q2 * gyro_x_rad_s));

    g_q0 += q_dot0 * IMU_SAMPLE_PERIOD_S;
    g_q1 += q_dot1 * IMU_SAMPLE_PERIOD_S;
    g_q2 += q_dot2 * IMU_SAMPLE_PERIOD_S;
    g_q3 += q_dot3 * IMU_SAMPLE_PERIOD_S;

    quaternion_norm_inv = 1.0f /
        sqrtf((g_q0 * g_q0) +
              (g_q1 * g_q1) +
              (g_q2 * g_q2) +
              (g_q3 * g_q3));

    g_q0 *= quaternion_norm_inv;
    g_q1 *= quaternion_norm_inv;
    g_q2 *= quaternion_norm_inv;
    g_q3 *= quaternion_norm_inv;
}


/* 四元数转换为正常定义的 Roll、Pitch、Yaw。 */
static void imu_update_euler_angles(void)
{
    imu_attitude_t attitude;      /* 本次姿态结果。 */
    float pitch_sine;             /* Pitch 反正弦输入。 */

    attitude.roll_deg =
        atan2f(2.0f * ((g_q0 * g_q1) + (g_q2 * g_q3)),
               1.0f - (2.0f * ((g_q1 * g_q1) + (g_q2 * g_q2)))) *
        IMU_RAD_TO_DEG - IMU_LEVEL_ROLL_TRIM_DEG;

    pitch_sine = 2.0f * ((g_q0 * g_q2) - (g_q1 * g_q3));

    if (pitch_sine >= 1.0f)
    {
        attitude.pitch_deg = 90.0f;
    }
    else if (pitch_sine <= -1.0f)
    {
        attitude.pitch_deg = -90.0f;
    }
    else
    {
        attitude.pitch_deg = asinf(pitch_sine) * IMU_RAD_TO_DEG;
    }
    attitude.pitch_deg -= IMU_LEVEL_PITCH_TRIM_DEG;

    g_yaw_absolute_deg =
        atan2f(2.0f * ((g_q0 * g_q3) + (g_q1 * g_q2)),
               1.0f - (2.0f * ((g_q2 * g_q2) + (g_q3 * g_q3)))) *
        IMU_RAD_TO_DEG;

    if (false == g_yaw_reference_valid)
    {
        g_yaw_reference_deg = g_yaw_absolute_deg;
        g_yaw_reference_valid = true;
    }

    attitude.yaw_deg = imu_wrap_angle_180(g_yaw_absolute_deg -
                                          g_yaw_reference_deg);

    /*
     * 三个角度作为一组发布，避免串口任务读到不同周期的数据。
     */
    taskENTER_CRITICAL();
    g_attitude = attitude;
    taskEXIT_CRITICAL();
}


/* 初始化 IMU 并标定陀螺仪零偏。 */
imu_status_t imu_init(spi_instance_t const * p_spi_instance,
                      bsp_io_port_pin_t chip_select_pin)
{
    uint32_t sample_index;                  /* 标定采样索引。 */
    int64_t gyro_sum_x = 0;                 /* X 轴累计值。 */
    int64_t gyro_sum_y = 0;                 /* Y 轴累计值。 */
    int64_t gyro_sum_z = 0;                 /* Z 轴累计值。 */
    int16_t gyro_min_x = INT16_MAX;          /* X 轴标定最小值。 */
    int16_t gyro_min_y = INT16_MAX;          /* Y 轴标定最小值。 */
    int16_t gyro_min_z = INT16_MAX;          /* Z 轴标定最小值。 */
    int16_t gyro_max_x = INT16_MIN;          /* X 轴标定最大值。 */
    int16_t gyro_max_y = INT16_MIN;          /* Y 轴标定最大值。 */
    int16_t gyro_max_z = INT16_MIN;          /* Z 轴标定最大值。 */
    int32_t gyro_span_x;                     /* X 轴标定波动范围。 */
    int32_t gyro_span_y;                     /* Y 轴标定波动范围。 */
    int32_t gyro_span_z;                     /* Z 轴标定波动范围。 */
    int32_t gyro_span_max;                   /* 三轴最大波动范围。 */
    float accel_x_g;                         /* 标定期间 X 轴加速度。 */
    float accel_y_g;                         /* 标定期间 Y 轴加速度。 */
    float accel_z_g;                         /* 标定期间 Z 轴加速度。 */
    float accel_norm_squared;                /* 加速度模长平方。 */
    bool calibration_motion = false;         /* 标定期间移动标志。 */
    icm42688_raw_data_t raw_data;           /* 六轴原始数据。 */
    icm42688_status_t driver_status;        /* 底层驱动状态。 */

    g_imu_ready = false;
    g_yaw_reference_deg = 0.0f;
    g_yaw_reference_valid = false;

    taskENTER_CRITICAL();
    g_calibration = (imu_calibration_t) {0};
    taskEXIT_CRITICAL();

    driver_status = icm42688_init(p_spi_instance,
                                  chip_select_pin);

    if (ICM42688_STATUS_OK != driver_status)
    {
        taskENTER_CRITICAL();
        g_calibration.state = IMU_CALIBRATION_DRIVER_ERROR;
        taskEXIT_CRITICAL();
        return IMU_STATUS_DRIVER_ERROR;
    }

    /*
     * 先等待传感器和机架稳定，再进行约 4 秒静止采样。
     * 标定期间飞控板必须水平放稳，不能触碰。
     */
    taskENTER_CRITICAL();
    g_calibration.state = IMU_CALIBRATION_IN_PROGRESS;
    taskEXIT_CRITICAL();

    vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_SETTLE_MS));

    for (sample_index = 0U;
         sample_index < IMU_CALIBRATION_SAMPLES;
         sample_index++)
    {
        driver_status = icm42688_read_raw(&raw_data);

        if (ICM42688_STATUS_OK != driver_status)
        {
            taskENTER_CRITICAL();
            g_calibration.state = IMU_CALIBRATION_DRIVER_ERROR;
            taskEXIT_CRITICAL();
            return IMU_STATUS_DRIVER_ERROR;
        }

        gyro_sum_x += raw_data.gyro_x;
        gyro_sum_y += raw_data.gyro_y;
        gyro_sum_z += raw_data.gyro_z;

        if (raw_data.gyro_x < gyro_min_x)
        {
            gyro_min_x = raw_data.gyro_x;
        }
        if (raw_data.gyro_x > gyro_max_x)
        {
            gyro_max_x = raw_data.gyro_x;
        }
        if (raw_data.gyro_y < gyro_min_y)
        {
            gyro_min_y = raw_data.gyro_y;
        }
        if (raw_data.gyro_y > gyro_max_y)
        {
            gyro_max_y = raw_data.gyro_y;
        }
        if (raw_data.gyro_z < gyro_min_z)
        {
            gyro_min_z = raw_data.gyro_z;
        }
        if (raw_data.gyro_z > gyro_max_z)
        {
            gyro_max_z = raw_data.gyro_z;
        }

        accel_x_g = (float) raw_data.accel_x /
                    ICM42688_ACCEL_LSB_PER_G;
        accel_y_g = (float) raw_data.accel_y /
                    ICM42688_ACCEL_LSB_PER_G;
        accel_z_g = (float) raw_data.accel_z /
                    ICM42688_ACCEL_LSB_PER_G;
        accel_norm_squared = (accel_x_g * accel_x_g) +
                             (accel_y_g * accel_y_g) +
                             (accel_z_g * accel_z_g);

        if ((accel_norm_squared < IMU_CALIBRATION_ACCEL_NORM_MIN) ||
            (accel_norm_squared > IMU_CALIBRATION_ACCEL_NORM_MAX))
        {
            calibration_motion = true;
        }

        if (0U == ((sample_index + 1U) % 50U))
        {
            taskENTER_CRITICAL();
            g_calibration.sample_count = sample_index + 1U;
            taskEXIT_CRITICAL();
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_CALIBRATION_DELAY_MS));
    }

    g_gyro_offset_x =
        (float) gyro_sum_x / (float) IMU_CALIBRATION_SAMPLES;

    g_gyro_offset_y =
        (float) gyro_sum_y / (float) IMU_CALIBRATION_SAMPLES;

    g_gyro_offset_z =
        (float) gyro_sum_z / (float) IMU_CALIBRATION_SAMPLES;

    gyro_span_x = (int32_t) gyro_max_x - (int32_t) gyro_min_x;
    gyro_span_y = (int32_t) gyro_max_y - (int32_t) gyro_min_y;
    gyro_span_z = (int32_t) gyro_max_z - (int32_t) gyro_min_z;
    gyro_span_max = gyro_span_x;

    if (gyro_span_y > gyro_span_max)
    {
        gyro_span_max = gyro_span_y;
    }
    if (gyro_span_z > gyro_span_max)
    {
        gyro_span_max = gyro_span_z;
    }

    if (gyro_span_max > IMU_CALIBRATION_GYRO_SPAN_COUNTS)
    {
        calibration_motion = true;
    }

    taskENTER_CRITICAL();
    g_calibration.gyro_offset_x_dps =
        g_gyro_offset_x / ICM42688_GYRO_LSB_PER_DPS;
    g_calibration.gyro_offset_y_dps =
        g_gyro_offset_y / ICM42688_GYRO_LSB_PER_DPS;
    g_calibration.gyro_offset_z_dps =
        g_gyro_offset_z / ICM42688_GYRO_LSB_PER_DPS;
    g_calibration.gyro_span_max_dps =
        (float) gyro_span_max / ICM42688_GYRO_LSB_PER_DPS;
    g_calibration.sample_count = IMU_CALIBRATION_SAMPLES;
    g_calibration.state = calibration_motion ?
                          IMU_CALIBRATION_MOTION :
                          IMU_CALIBRATION_SUCCESS;
    taskEXIT_CRITICAL();

    if (true == calibration_motion)
    {
        return IMU_STATUS_CALIBRATION_MOTION;
    }

    /*
     * 用一次静止加速度作为低通初值，避免启动时从 0 缓慢收敛。
     */
    driver_status = icm42688_read_raw(&raw_data);

    if (ICM42688_STATUS_OK != driver_status)
    {
        taskENTER_CRITICAL();
        g_calibration.state = IMU_CALIBRATION_DRIVER_ERROR;
        taskEXIT_CRITICAL();
        return IMU_STATUS_DRIVER_ERROR;
    }

    g_accel_lpf_x =
        -(float) raw_data.accel_x / ICM42688_ACCEL_LSB_PER_G;

    g_accel_lpf_y =
        -(float) raw_data.accel_y / ICM42688_ACCEL_LSB_PER_G;

    g_accel_lpf_z =
        -(float) raw_data.accel_z / ICM42688_ACCEL_LSB_PER_G;

    g_imu_ready = true;

    return IMU_STATUS_OK;
}


/* 执行一次 500 Hz 姿态更新。 */
imu_status_t imu_update(void)
{
    icm42688_raw_data_t raw_data;        /* 六轴原始数据。 */
    icm42688_status_t driver_status;     /* 底层驱动状态。 */
    float accel_x_g;                     /* X 轴加速度，单位 g。 */
    float accel_y_g;                     /* Y 轴加速度，单位 g。 */
    float accel_z_g;                     /* Z 轴加速度，单位 g。 */
    float gyro_x_rad_s;                  /* X 轴角速度，单位 rad/s。 */
    float gyro_y_rad_s;                  /* Y 轴角速度，单位 rad/s。 */
    float gyro_z_rad_s;                  /* Z 轴角速度，单位 rad/s。 */

    if (false == g_imu_ready)
    {
        return IMU_STATUS_NOT_READY;
    }

    driver_status = icm42688_read_raw(&raw_data);

    if (ICM42688_STATUS_OK != driver_status)
    {
        return IMU_STATUS_DRIVER_ERROR;
    }

    /*
     * 同一机架在 RT1064 工程中的实测结果为：传感器 +X 向前、+Y 向右、
     * +Z 向下。加速度计输出为比力，方向与重力相反，因此三个轴取反后
     * 再送入 Mahony 重力校正；陀螺仪轴不交换也不取反。
     */
    accel_x_g = -(float) raw_data.accel_x /
                ICM42688_ACCEL_LSB_PER_G;

    accel_y_g = -(float) raw_data.accel_y /
                ICM42688_ACCEL_LSB_PER_G;

    accel_z_g = -(float) raw_data.accel_z /
                ICM42688_ACCEL_LSB_PER_G;

    g_accel_lpf_x = (IMU_ACCEL_LPF_ALPHA * accel_x_g) +
                    ((1.0f - IMU_ACCEL_LPF_ALPHA) * g_accel_lpf_x);

    g_accel_lpf_y = (IMU_ACCEL_LPF_ALPHA * accel_y_g) +
                    ((1.0f - IMU_ACCEL_LPF_ALPHA) * g_accel_lpf_y);

    g_accel_lpf_z = (IMU_ACCEL_LPF_ALPHA * accel_z_g) +
                    ((1.0f - IMU_ACCEL_LPF_ALPHA) * g_accel_lpf_z);

    gyro_x_rad_s =
        (((float) raw_data.gyro_x - g_gyro_offset_x) /
         ICM42688_GYRO_LSB_PER_DPS) *
        IMU_DEG_TO_RAD;

    gyro_y_rad_s =
        (((float) raw_data.gyro_y - g_gyro_offset_y) /
         ICM42688_GYRO_LSB_PER_DPS) *
        IMU_DEG_TO_RAD;

    gyro_z_rad_s =
        (((float) raw_data.gyro_z - g_gyro_offset_z) /
         ICM42688_GYRO_LSB_PER_DPS) *
        IMU_DEG_TO_RAD;

    gyro_x_rad_s = imu_butterworth_update(&g_gyro_filter_x,
                                          gyro_x_rad_s);

    gyro_y_rad_s = imu_butterworth_update(&g_gyro_filter_y,
                                          gyro_y_rad_s);

    gyro_z_rad_s = imu_butterworth_update(&g_gyro_filter_z,
                                          gyro_z_rad_s);

    imu_mahony_update(gyro_x_rad_s,
                      gyro_y_rad_s,
                      gyro_z_rad_s,
                      g_accel_lpf_x,
                      g_accel_lpf_y,
                      g_accel_lpf_z);

    imu_update_euler_angles();

    return IMU_STATUS_OK;
}


/* 获取三个角度快照。 */
void imu_get_attitude(imu_attitude_t * p_attitude)
{
    if (NULL == p_attitude)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_attitude = g_attitude;
    taskEXIT_CRITICAL();
}


/* 查询 IMU 是否就绪。 */
bool imu_is_ready(void)
{
    return g_imu_ready;
}


/* 获取启动标定结果快照。 */
void imu_get_calibration(imu_calibration_t * p_calibration)
{
    if (NULL == p_calibration)
    {
        return;
    }

    taskENTER_CRITICAL();
    *p_calibration = g_calibration;
    taskEXIT_CRITICAL();
}


/* 将当前绝对航向设为新的相对零点。 */
void imu_zero_yaw(void)
{
    taskENTER_CRITICAL();
    g_yaw_reference_deg = g_yaw_absolute_deg;
    g_yaw_reference_valid = true;
    g_attitude.yaw_deg = 0.0f;
    taskEXIT_CRITICAL();
}
