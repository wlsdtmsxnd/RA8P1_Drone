# RA8P1 无人机分阶段调试

所有电机相关测试都必须拆除桨叶并固定机架。先用示波器或逻辑分析仪
确认波形，再连接 ESC 信号线。

## 1. 上电与 IMU

1. 保持 `src/code/project_config.h` 中 `ESC_BENCH_MODE_DISABLED`。
2. 拆除桨叶，将机架放在坚硬、水平、无振动的桌面上。编译、烧录后，
   手离开机架再按复位键。
3. 当前 `TELEMETRY_SOURCE_IMU_CALIBRATION` 模式会先等待 1 秒，再连续
   采样约 4 秒。整个 5 秒内不能触碰机架。
4. VOFA+ JustFloat 建立 8 个通道：I0/I1/I2 为 X/Y/Z 陀螺零偏（deg/s），
   I3/I4/I5 为 Roll/Pitch/Yaw，I6 为标定状态，I7 为 IMU 就绪标志。
5. I6 状态：0 未开始、1 标定中、2 成功、3 检测到移动、4 驱动通信错误。
   只有 I6=2 且 I7=1 才能继续。I6=3 时保持静止并重新复位；I6=4 时检查
   SPI 接线与供电。
6. 成功后继续静置 60 秒。Roll/Pitch 应稳定在约 +/-0.5 度内，Yaw 初次
   台架目标为 60 秒漂移不超过 3 度；超过 5 度应重新标定并检查温漂、
   电源和桌面振动。
7. 当前水平修正为 Roll 0.0 度、Pitch -0.97 度。若再次拆装 IMU，记录静止
   10～20 秒内 Roll/Pitch 的平均值，分别填回 `IMU_LEVEL_ROLL_TRIM_DEG`
   和 `IMU_LEVEL_PITCH_TRIM_DEG`，重新编译后水平值应接近 0。
8. 右侧压低 Roll 应为正，机头抬起 Pitch 应为正。若方向不符，停止后续
   测试并重新核对安装方向；本次拆装方向未变时不应修改轴映射。
9. 标定和姿态确认完成后，将 `TELEMETRY_SOURCE` 恢复为
   `TELEMETRY_SOURCE_CRSF`，再进行遥控与电机测试。

## 2. 遥控与保护

将 `TELEMETRY_SOURCE` 恢复为 `TELEMETRY_SOURCE_CRSF`。I0-I5 对应
CH1-CH6，I6 的状态定义为：0 DISARMED、1 ARMING_WAIT、2 ARMED、
3 FAILSAFE；I7 为 IMU 就绪。

RadioMaster Pocket 的对频只负责建立 ELRS 链路，通道来源和端点需要在
EdgeTX 当前模型中设置。推荐先使用 AETR 顺序：CH1 Roll、CH2 Pitch、
CH3 Throttle、CH4 Yaw、CH5 Arm。CH5 必须分配给一个自锁开关，并保证
低位约 1000 us、高位约 2000 us；不要使用瞬时 SE 按键作为 Arm。

1. 短按 `MDL` 进入当前模型设置，用 `PAGE>` 找到 `Mixes`。
2. 确认 CH1-CH4 的 Source 分别为 Ail、Ele、Thr、Rud，Weight 为 100、
   Offset 为 0。
3. 将 CH5 Source 设为选定的自锁开关，Weight 为 100、Offset 为 0。
4. 在 `Outputs` 中保持 Subtrim=0、Min=-100、Max=100、PPM Center=1500，
   首次测试不要启用 Extended Limits。
5. 长按 `TELE` 打开通道监视器，先确认遥控器本机显示 -100/0/+100，
   再与 VOFA 的约 988/1500/2012 us 对照。ELRS 的 CH5/AUX1 是专用两档
   解锁通道，约 1000/2000 us；CH6 及之后在 Hybrid/Wide 模式下可能出现
   量化后的档位值，这是协议行为。

- 关闭遥控器或拔掉接收机后，300 ms 内 I0-I5 应清零、I6 变为 3。
- 上电时 CH5 即使在高档也不能解锁；必须先拨到低档。
- CH3 最低、CH5 高档保持 1 秒后 I6 才能从 1 变为 2。
- CH5 拨低后 I6 应立即变为 0。

## 3. 遥控归一化

原始通道测试通过后，将 `TELEMETRY_SOURCE` 设置为
`TELEMETRY_SOURCE_RC_COMMAND`。该模式只验证飞控指令预处理，不会把
油门直接送给电机。

VOFA+ 的 8 个通道定义如下：

- I0：Roll，范围 -1～+1，中位死区内为 0。
- I1：Pitch，范围 -1～+1，中位死区内为 0。
- I2：Throttle，范围 0～1，1050 us 以下保持为 0。
- I3：Yaw，范围 -1～+1，中位死区内为 0。
- I4：CH5 Arm，高档为 1，低档为 0。
- I5：CH6 Mode，低/中/高档分别为 0/1/2。
- I6：安全状态 0 DISARMED、1 ARMING_WAIT、2 ARMED、3 FAILSAFE。
- I7：IMU 就绪标志。

1. 拆除桨叶并断开 ESC 动力电源，CH5 低档、油门最低后上电。
2. 等待 I7=1、I6=0。摇杆全部释放时 I0/I1/I3 应稳定为 0，I2 应为 0。
3. Roll 右打杆应到 +1，左打杆应到 -1；Yaw 右打杆应到 +1，左打杆
   应到 -1。
4. Pitch 后拉杆应到 +1，前推杆应到 -1；Throttle 从最低到最高应单调
   从 0 增长到 1。
5. CH6 三个档位依次检查 I5=0/1/2。CH5 高档时 I4=1，低档时 I4=0。
6. 关闭遥控器后，I0-I5 应全部为 0、I6 应在 300 ms 内变为 3；重连且
   CH5 保持低档后，I6 应恢复为 0。
7. 记录完整 CSV，重点检查中位是否稳定为 0、端点是否能达到 +/-1、油门
   是否连续单调，以及任何通道是否存在方向相反或跳变。

当前归一化使用本机实测端点：Roll 989/1502/2011 us、Pitch
989/1490/2001 us、Yaw 990/1506/2011 us；三个居中通道均使用 20 us
死区。更换遥控器模型、修改 EdgeTX 端点或重新校准摇杆后必须重新采集
原始通道数据并复核这些参数。

## 4. PWM 波形

在 P700、P701、P109、P702 分别测量：周期应为 20 ms，正常模式高电平
应为 1.000 ms。四路都正确后才连接 ESC，且 MCU 与 ESC 必须共地。

启用任一 ESC 台架模式后，VOFA+ 会自动切换为 PWM 监视，无需修改
`TELEMETRY_SOURCE`：I0～I3 为 M1～M4 命令脉宽（us），I4 为台架阶段，
I5 为活动电机（0 无、1～4 对应 M1～M4），I6 为 PWM 驱动就绪标志，
I7 为台架上电时间（s）。阶段定义为：0 关闭、1 等待 CH5 低档、2 等待
CH5 高档、3 启动倒计时、4 电机运行、5 电机间隔、6 完成、7 输出错误、
8 校准高电平、9 校准低电平。

## 5. 电机顺序

确认无桨后，在 `src/code/project_config.h` 中设置：

```c
#define ESC_BENCH_MODE                   ESC_BENCH_MODE_MOTOR_SEQUENCE
#define ESC_BENCH_SAFETY_ACKNOWLEDGED    (1U)
```

烧录后四路持续保持 1000 us，不会自动启动。接收机连接、油门最低后，
必须先将 CH5 拨到低档，再拨到高档；3 秒倒计时后 M1、M2、M3、M4
各以 1200 us 运行 3 秒，中间停 2 秒。任意时刻关闭 CH5、抬高油门或
遥控失联都会立即将四路恢复为 1000 us。核对位置和旋向后，立即恢复
`ESC_BENCH_MODE_DISABLED` 和 `0U` 并重新编译烧录。

顺序模式的 VOFA 预期为：启动后 I0～I3 均为 1000；CH5 低档时 I4=2，
CH5 高档且油门最低时 I4=3。M1 运行时 I0=1200、I4=4、I5=1，
其余三路为 1000。随后 I5 按 2、3、4 递增，
每次运行之间 I4=5、I5=0；全部完成后 I4=6、I0～I3 均回到 1000。
任何时刻 I4=7 或 I6=0，都应立即断开 ESC 动力并停止测试。

## 6. 闭环前检查点

- IMU 静止时角度不发散，快速晃动后能回到水平零点附近。
- 四路 PWM 顺序、旋向和 1000-2000 us 限幅正确。
- 遥控失联、IMU 读失败和 CH5 低档均能强制 1000 us。
- 记录本机架的水平零偏、悬停油门和每次 PID 修改；不要直接套用
  RA6M5 小机架参数。
