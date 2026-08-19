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
| UP-T301 TX / RX | P905 / P310 | SCI3，460800 baud |
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
- UP_T3-001 / UP-T301 UPIX 驱动、RA6M5 同结构的高度融合、光流速度积分位置。
- CH6 中档定高、高档定高+定点的串级闭环；光流失效降级为定高，TOF
  失效退回手动。旧 TPF/LC08 软件 I2C 驱动已移除。
- Roll/Pitch 姿态—角速度串级控制与 Yaw 角速度控制已具备系留测试 profile。
- 所有动力命令统一经过执行器仲裁层；控制故障会进入全局 FAILSAFE。

仓库默认构建是 `SAFE`：即使 ARMED，控制与测试代码也没有提高 PWM 的调用
路径。系留和台架固件必须通过显式编译器 profile 构建，不能通过提交一个
长期为 `1U` 的授权位启用。当前架构边界见
[docs/architecture.md](docs/architecture.md)，迁移状态见
[docs/migration_status.md](docs/migration_status.md)。

## 构建

用 e2 studio 打开工程（`ra8p1_project1`），使用 FSP 生成器确认外设配置后
编译即可。构建产物输出到 `Debug/`（已在 `.gitignore` 中忽略）。

不增加任何预处理宏时生成 `PROJECT_BUILD_PROFILE_SAFE`。只有重新完成现场
安全确认后，才可在专用构建配置中同时增加：

```text
PROJECT_BUILD_PROFILE=1U
PROJECT_DANGEROUS_BUILD_ACK=0x54455448UL
```

这会选择 `PROJECT_BUILD_PROFILE_TETHERED_FIRST_HOP`。测试结束后必须切回
无额外宏的 SAFE 配置，执行 `clean all` 并重新烧录。

拆桨验证导航闭环时只增加 `PROJECT_BUILD_PROFILE=3U`，进入四路始终为
1000 us 的 `FLOW_HOLD_SHADOW`，且不得携带危险确认宏。完整流程见
[docs/flow_navigation_test_guide.md](docs/flow_navigation_test_guide.md)。

当前 `Debug` 和 `Release` 都不包含上述两个宏，均构建默认 SAFE。以后新建
系留专用配置时再显式加入，并在烧录前核对实际选择的构建配置。

代码变更后可在 PowerShell 中运行：

```powershell
tests\run_host_tests.ps1
tests\run_profile_syntax_checks.ps1
tests\verify_safe_elf.ps1
```

> 安全提示：编译确认值只用于阻止误构建，不能替代拆桨、系留、隔离区、
> 急停和独立断电等现场确认。
