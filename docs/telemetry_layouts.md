# 分阶段遥测通道布局

3DR 串口为 57600 baud。SAFE 导航遥测为 50 Hz；28 通道系留遥测降为
25 Hz，避免占满串口。

## SAFE 默认：UP-T301 导航状态

无额外宏时使用 `TELEMETRY_SOURCE_FLOW_NAVIGATION`，16 通道、50 Hz，电机
始终为 1000 us。

| 通道 | 数据 | 单位 |
|---|---|---|
| I0 | 融合高度 | mm |
| I1 | 垂直速度 | mm/s |
| I2/I3 | X/Y 积分位置 | mm |
| I4/I5 | X/Y 滤波速度 | mm/s |
| I6 | UP-T301 原始距离 | mm |
| I7/I8/I9 | 高度有效/光流有效/导航就绪 | 0/1 |
| I10/I11 | 定点 Roll/Pitch 目标；SAFE 下为 0 | deg |
| I12 | 定高油门修正；SAFE 下为 0 | us |
| I13 | 闭环模式：0 手动、1 定高、2 定点 | enum |
| I14 | UPIX 正确帧累计数 | count |
| I15 | TOF 与光流拒绝累计数 | count |

原始协议复测可显式选择 `TELEMETRY_SOURCE_FLOW_TOF`，其 16 通道定义见
`up_tof_test_guide.md`。

## FLOW_HOLD_SHADOW：拆桨闭环影子测试

设置 `PROJECT_BUILD_PROFILE=3U`，通道与 SAFE 默认相同。完成正常解锁流程后
控制器会计算 I10～I13，但执行器由影子模式强制保持 1000 us。

## TETHERED_FIRST_HOP：系留闭环

28 通道、25 Hz：

| 通道 | 数据 |
|---|---|
| I0～I3 | M1～M4 实际执行器命令，us |
| I4/I5 | 油门、CH6 模式 |
| I6～I8 | Roll/Pitch/Yaw 摇杆 |
| I9/I10 | Roll/Pitch，deg |
| I11～I13 | Gyro X/Y/Z，deg/s |
| I14～I16 | Roll/Pitch/Yaw 修正，us |
| I17 | 基础油门，us |
| I18/I19 | 高度、垂直速度，mm 与 mm/s |
| I20/I21 | X/Y 位置，mm |
| I22/I23 | X/Y 速度，mm/s |
| I24/I25 | 定点 Roll/Pitch 目标，deg |
| I26 | 实际闭环模式：0/1/2 |
| I27 | 停机原因 |
