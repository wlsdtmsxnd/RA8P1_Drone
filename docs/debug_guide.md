# RA8P1 无人机分阶段调试

所有电机相关测试都必须拆除桨叶并固定机架。先用示波器或逻辑分析仪
确认波形，再连接 ESC 信号线。

## 1. 上电与 IMU

1. 保持 `src/code/project_config.h` 中 `ESC_BENCH_MODE_DISABLED`。
2. 编译、烧录并复位，前约 2 秒不要移动机架，等待陀螺仪标定。
3. 将 `TELEMETRY_SOURCE` 改为 `TELEMETRY_SOURCE_EULER`，重新烧录。
4. VOFA+ JustFloat 建立 8 个通道：I0/I1/I2 为 Roll/Pitch/Yaw，I6 为
   安全状态，I7 为 IMU 就绪标志。
5. 水平时 Roll/Pitch 应接近 0；右侧压低 Roll 应为正，机头抬起 Pitch
   应为正。若方向不符，停止后续测试并重新核对安装方向。

## 2. 遥控与保护

将 `TELEMETRY_SOURCE` 恢复为 `TELEMETRY_SOURCE_CRSF`。I0-I5 对应
CH1-CH6，I6 的状态定义为：0 DISARMED、1 ARMING_WAIT、2 ARMED、
3 FAILSAFE；I7 为 IMU 就绪。

- 关闭遥控器或拔掉接收机后，300 ms 内 I0-I5 应清零、I6 变为 3。
- 上电时 CH5 即使在高档也不能解锁；必须先拨到低档。
- CH3 最低、CH5 高档保持 1 秒后 I6 才能从 1 变为 2。
- CH5 拨低后 I6 应立即变为 0。

## 3. PWM 波形

在 P700、P701、P109、P702 分别测量：周期应为 20 ms，正常模式高电平
应为 1.000 ms。四路都正确后才连接 ESC，且 MCU 与 ESC 必须共地。

## 4. 电机顺序

确认无桨后，在 `src/code/project_config.h` 中设置：

```c
#define ESC_BENCH_MODE                   ESC_BENCH_MODE_MOTOR_SEQUENCE
#define ESC_BENCH_SAFETY_ACKNOWLEDGED    (1U)
```

烧录后先保持 1000 us 10 秒，然后 M1、M2、M3、M4 各以 1200 us 运行
3 秒，中间停 2 秒。核对位置和旋向后，立即恢复
`ESC_BENCH_MODE_DISABLED` 和 `0U` 并重新编译烧录。

## 5. 闭环前检查点

- IMU 静止时角度不发散，快速晃动后能回到水平零点附近。
- 四路 PWM 顺序、旋向和 1000-2000 us 限幅正确。
- 遥控失联、IMU 读失败和 CH5 低档均能强制 1000 us。
- 记录本机架的水平零偏、悬停油门和每次 PID 修改；不要直接套用
  RA6M5 小机架参数。
