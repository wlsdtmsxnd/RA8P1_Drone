# RA8P1_Drone

基于瑞萨 **RA8P1**（Arm Cortex-M85）的 10 寸四旋翼飞控固件工程，使用
**e2 studio + Renesas FSP** 构建，跑 **FreeRTOS**。

目标机架与 `RT1064_Drone_10inch` 相同，IMU 安装方向、CRSF、数传、电机顺序
和安全逻辑均已从 RT1064 实机验证结果同步迁移。

## 目录结构

```text
src/            应用源码
  code/           应用模块：姿态解算、飞控安全、电调测试、VOFA 数传
  driver/         器件驱动：ICM42688、CRSF 接收机、3DR 数传、电机输出
  *_entry.c       FreeRTOS 线程入口（imu / rc / telemetry / uart 等）
  hal_warmstart.c 启动与安全初始化
ra/             Renesas FSP 库（CMSIS_6、FreeRTOS、FSP 驱动）
ra_gen/         FSP 生成的 main / hal_data / 各线程与向量表
ra_cfg/         FSP 配置头文件
script/         链接脚本（fsp.ld）
docs/           迁移、硬件、调试笔记
```

## 硬件映射（摘要）

| 功能 | 引脚 | 外设 |
|---|---|---|
| ICM42688 SCK / MOSI / MISO | P415 / P708 / P709 | SPI1 |
| ICM42688 CS | P710 | GPIO，低有效 |
| CRSF RX / TX | P602 / P603 | SCI0，420000 baud |
| 3DR 数传 RX / TX | P802 / P801 | SCI2，57600 baud |
| M1 / M2 | P700 / P701 | GPT5 A / B |
| M3 / M4 | P109 / P702 | GPT10 A / GPT6 A |

电机顺序沿用 RT1064 实机记录：M1 左前、M2 右前、M3 右后、M4 左后。
四路 ESC PWM 为 50 Hz、1000–2000 us，上电及任何故障状态保持 1000 us。

## 当前状态

- ICM42688 SPI 驱动、500 Hz Mahony 姿态解算与静止陀螺零偏标定。
- CRSF 16 通道解析、300 ms 失联判定与 3DR/VOFA JustFloat 数传。
- 四路 GPT PWM 输出与电机脉宽限幅。
- CH5 解锁 / 停机安全逻辑。
- 编译期保护的电调行程校准与 M1–M4 无桨顺序测试。
- UP_T3-001 / UP-T301 光流 TOF 的 UPIXELS 协议驱动与 VOFA 测试遥测源。

角度环、角速度环 PID 尚未接入电机输出（正常模式 ARMED 后四路仍保持
1000 us）。详见 [docs/migration_status.md](docs/migration_status.md)。
光流 TOF 的 FSP UART 配置和验收流程见
[docs/up_tof_test_guide.md](docs/up_tof_test_guide.md)。

## 构建

用 e2 studio 打开工程（`ra8p1_project1`），使用 FSP 生成器确认外设配置后
编译即可。构建产物输出到 `Debug/`（已在 `.gitignore` 中忽略）。

> 安全提示：任何电调测试模式（`src/code/project_config.h` 的
> `ESC_BENCH_MODE`）都必须在拆除全部螺旋桨后方可启用。
