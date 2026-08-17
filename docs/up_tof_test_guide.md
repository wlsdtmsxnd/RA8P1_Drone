# UP-T301/UP_T3-001 光流 TOF 接入与测试指南

## 用户请求与资料说明

你的请求是：在 RA8P1 飞控工程中接入 UP_T3-001 光流+TOF 模块，使用飞控端引脚 P905/P310，并提供测试指南。

资料中的关键说明只作为驱动依据：UPIXELS 光流+TOF 帧为 `0xFE 0x0A + 10 字节 payload + XOR + 0x55`；payload 依次是 `flow_x_integral`、`flow_y_integral`、`integration_timespan`、`distance_mm`、`valid`、`tof_confidence`。其中 `flow_*_integral` 是 `radians * 10000`，`valid == 0xF5` 表示光流可用。

注意：UP-T301 产品规格书写默认通信速率为 460800 bps；协议介绍手册写 T1/T2 固定 115200 bps、302GS 固定 460800 bps。你使用的型号是 UP_T3-001，建议先用厂家 FLOW_TOOL 确认当前协议和波特率，再在 FSP 中填同一个值。若不确定，优先试 460800 bps，再试 115200 bps。

## 已加入的驱动

- `src/driver/up_tof.c`
- `src/driver/up_tof.h`

驱动提供：

- `up_tof_uart_callback()`：填到 FSP UART Callback。
- `up_tof_init(&g_uart_xxx)`：打开 UART 并清空解析状态。
- `up_tof_process()`：在线程中周期调用，解析接收环形缓冲区。
- `up_tof_get_data()`：读取距离、速度、原始积分、质量和错误计数快照。
- `up_tof_is_ready()`：最近 200 ms 内收到 `valid == 0xF5` 的有效帧时返回 true。

速度计算：

```text
displacement_mm = flow_integral / 10000 * distance_mm
velocity_cm_s = displacement_mm * 100000 / integration_us
```

## FSP 硬件配置

当前工程还没有 P905/P310 的 pin mux，也没有光流专用 UART 实例。请在 e2 studio 的 FSP 配置中新增一个 SCI UART：

1. 打开 `configuration.xml`。
2. 在 Stacks 中新增 `UART (r_sci_b_uart)`，命名建议 `g_uart_flow_tof`。
3. Channel 选择必须以 Pins 页面里 P905/P310 可分配到同一组 SCI 为准。
4. Callback 填 `up_tof_uart_callback`。
5. UART 格式：8 data bits，no parity，1 stop bit，no flow control。
6. Baud rate 填 FLOW_TOOL 确认的值：通常先试 `460800`，若无帧再试 `115200`。
7. Pins 页面把 P905/P310 配为该 SCI 的 RXD/TXD。模块 TX 接 MCU RX，模块 RX 接 MCU TX。
8. 重新 Generate Project Content。

模块供电和接线：

```text
UP_T3-001 VCC -> 3.7-5.0 V
UP_T3-001 GND -> 飞控 GND
UP_T3-001 TX  -> RA8P1 UART RX
UP_T3-001 RX  -> RA8P1 UART TX
```

## 线程接入

生成 UART 实例后，新增或复用一个 FreeRTOS 线程，入口中调用：

```c
#include "driver/up_tof.h"

void flow_tof_thread_entry(void * pvParameters)
{
    fsp_err_t err;

    FSP_PARAMETER_NOT_USED(pvParameters);

    err = up_tof_init(&g_uart_flow_tof);

    if (FSP_SUCCESS != err)
    {
        vTaskSuspend(NULL);
    }

    while (1)
    {
        up_tof_process();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}
```

不要把这段直接填进 `ra_gen` 生成文件里；让 FSP 生成线程壳，业务代码放到 `src/*_entry.c`。

## VOFA 遥测测试

我已在 `src/code/project_config.h` 加了 `TELEMETRY_SOURCE_FLOW_TOF`。要用 3DR 数传看数据时，把当前遥测源：

```c
#define TELEMETRY_SOURCE                 TELEMETRY_SOURCE_...
```

临时改成：

```c
#define TELEMETRY_SOURCE                 TELEMETRY_SOURCE_FLOW_TOF
```

VOFA JustFloat 通道含义：

| 通道 | 含义 |
|---|---|
| I0 | TOF 距离，m |
| I1 | X 方向速度，cm/s |
| I2 | Y 方向速度，cm/s |
| I3 | `flow_x_integral` 原始值 |
| I4 | `flow_y_integral` 原始值 |
| I5 | 有效帧的 TOF confidence，无效时为 0 |
| I6 | 飞控安全状态 |
| I7 | IMU ready |

## 分步验收

1. 先拆桨，只给模块和飞控上电。
2. 用 FLOW_TOOL 连接模块，确认当前输出协议为 `upixels`，并记录波特率。
3. 在 FSP 配好 UART 后编译烧录。
4. 打开 VOFA，选择 JustFloat，确认 I0 距离随高度变化。
5. 模块静止放在纹理明显、光照充足的地面上方，I1/I2 应接近 0。
6. 水平缓慢移动模块，I1/I2 应有正负变化；若方向与机体坐标相反，先记录，再在控制融合层做轴向修正。
7. 遮挡镜头或对着弱纹理/过暗表面时，I5 应降为 0 或有效性变差。
8. 若 I0 正常但 I1/I2 长期为 0，检查光照、纹理和模块离地高度是否在 0.2-20 m 范围内。
9. 若完全无数据，优先检查波特率、TX/RX 是否交叉、P905/P310 是否真的映射到同一个 SCI UART。

## 后续接入控制前的安全要求

当前驱动只发布传感器数据，没有把光流接入位置控制。接入定点前至少需要完成：

- 确认模块安装方向和机体 FRD 坐标的 X/Y 符号。
- 增加低质量、超时、距离异常时的降级逻辑。
- 只在解锁前或低速安全状态下验证闭环参数。
- 光流定点首次测试必须拆桨或固定台架验证，再上桨低高度试飞。
