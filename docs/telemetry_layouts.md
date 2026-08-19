# 分阶段遥测通道布局

当前 3DR 串口为 57600 baud。遥测按实验阶段拆分，避免长期发送无关通道，
也避免 37 通道、50 Hz 超出串口吞吐量。

## SAFE 默认：UP-T301 UPIX UART 验证

16 通道、50 Hz；JustFloat 单帧 68 字节，串口发送约 11.8 ms。SAFE profile
不授权执行器输出，详细验收步骤见 `up_tof_test_guide.md`。

| 通道 | 数据 | 单位 |
|---|---|---|
| I0 | TOF 原始距离 | m |
| I1/I2 | X/Y 光流速度 | cm/s |
| I3/I4 | X/Y 光流积分 | raw |
| I5 | 积分时间 | us |
| I6～I9 | 帧/光流/TOF/速度有效标志 | 0/1 |
| I10 | TOF confidence | raw |
| I11 | 正确帧累计数 | count |
| I12～I15 | XOR/格式/UART/缓冲溢出累计数 | count |

## SAFE 可选：TPF/LC08 软件 I2C 验证

在 SAFE 构建中显式设置 `TELEMETRY_SOURCE=0U` 后启用 17 通道、50 Hz TPF
布局。具体 I0～I16 定义和测试方法见 `tpf_flow_test_guide.md`。

## TETHERED_FIRST_HOP：系留起飞与悬停

18 通道、50 Hz；JustFloat 单帧 76 字节，在 57600 baud 下发送约
13.2 ms，20 ms 周期内保留约 6.8 ms 调度余量。当前阶段光流不采集、
不参与控制，也不占用飞行遥测。

| 通道 | 数据 |
|---|---|
| I0～I3 | M1～M4 实际执行器命令，us |
| I4 | 油门指令 |
| I5～I7 | Roll/Pitch/Yaw 摇杆指令 |
| I8/I9 | Roll/Pitch，deg |
| I10～I12 | Gyro X/Y/Z，deg/s |
| I13～I15 | Roll/Pitch/Yaw 修正，us |
| I16 | 安全状态 |
| I17 | 停机原因 |

Yaw 积分和光流数据只在对应专项调参时临时加入，不长期占用飞行遥测。
高频姿态环调参也应使用更少通道、更高采样率的专用布局。
