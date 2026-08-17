# RA8P1 飞控迁移状态

## 迁移基线

- 目标机架与 `RT1064_Drone_10inch` 相同。
- `RT1064_Drone_10inch` 提供已经验证的 IMU 安装方向、CRSF、数传、
  电机顺序和安全逻辑。
- `D:\Renesas\old_fly` 的 RA6M5 工程仅作为完整四轴控制流程参考，
  不复制其 FSP 生成文件和针对小机架的 PID 参数。

## 硬件映射

| 功能 | RA8P1 引脚 | 外设 |
|---|---|---|
| ICM42688 SCK | P415 | SPI1 RSPCK |
| ICM42688 MOSI | P708 | SPI1 MOSI |
| ICM42688 MISO | P709 | SPI1 MISO |
| ICM42688 CS | P710 | GPIO，低有效 |
| CRSF RX / TX | P602 / P603 | SCI0，420000 baud |
| 3DR 数传 RX / TX | P802 / P801 | SCI2，57600 baud |
| M1 / M2 | P700 / P701 | GPT5 A / B |
| M3 | P109 | GPT10 A |
| M4 | P702 | GPT6 A |

电机顺序沿用 RT1064 实机记录：M1 左前、M2 右前、M3 右后、M4 左后。
四路 ESC PWM 为 50 Hz，范围 1000-2000 us，上电和任何故障状态均保持
1000 us。

## 已完成

- ICM42688 SPI 驱动、500 Hz Mahony 姿态解算和静止陀螺零偏标定。
- 同步 RT1064 实测 FRD 轴向：X 前、Y 右、Z 下；加速度比力取反后参与
  重力校正。
- CRSF 16 通道解析、300 ms 失联判定和 3DR/VOFA JustFloat 数传。
- 四路 GPT PWM、电机脉宽限幅和上电安全输出。
- CH5 解锁：启动后必须先看到低档；CH3 低于 1050 us 且 CH5 高档保持
  1 秒才进入 ARMED。CH5 低、遥控失联或 IMU 更新失败立即停机。
- 编译期保护的电调行程校准和 M1-M4 无桨顺序测试。

## 尚未完成

- RT1064 工程当前也尚未把角度环、角速度环 PID 输出接入电机，因此
  RA8P1 的正常模式即使进入 ARMED，四路仍保持 1000 us。
- 必须先完成无桨方向、PWM、姿态符号和失联测试，再参考 RA6M5 的
  `control.c`/`pid.c` 移植双环控制。
- RA6M5 的机架尺寸和动力系统不同，其 PID 数值不能直接使用。
- 定高、光流、低电压和传感器冗余保护尚未迁移。
