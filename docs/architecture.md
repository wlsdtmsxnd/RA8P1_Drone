# 固件架构与不可绕过边界

## 运行链路

```text
UART/SPI 驱动与中断
        ↓
RC、IMU、导航传感器采集任务
        ↓ 一致数据快照
统一飞行安全状态机
        ↓ 授权/禁止
控制器或受限台架模式
        ↓ 完整四路命令帧
actuator_manager（唯一仲裁者）
        ↓
motor_output（GPT 硬件驱动）
```

遥测是上述快照的消费者，不得初始化、轮询或拥有控制所需的传感器。UART
UP-T301 UPIX 串口由低优先级导航采集任务维护；IMU 任务以 500 Hz 运行
`flow_navigation`，发布高度、水平速度和积分位置快照。3DR 失败不影响采集
和控制。

## 必须保持的约束

1. `motor_output.h` 只允许 `actuator_manager.c` 包含；其他应用模块不能直接
   写 GPT。
2. 四路输出必须由 `actuator_manager_apply_us()` 整帧提交。命令越界、任一
   GPT 写失败或未授权都会拒绝输出并尝试写入四路停机脉宽。
3. CH5、RC、IMU、控制器和执行器故障最终都必须体现在
   `flight_safety_state_t`；禁止出现长期“安全状态 ARMED、执行器内部停机”
   的双状态源。
4. 仓库无额外编译宏时必须生成 SAFE 固件。危险 profile 只能存在于专用
   IDE 构建配置中，不能通过修改并提交长期授权位来启用。
5. 遥测使用控制周期末尾发布的 `flight_snapshot_t`。新增调参通道时优先从
   该快照取值，避免独立读取电机、姿态、RC 和控制器造成跨周期拼接。
6. FreeRTOS 栈检查、Cortex-M85 PSPLIM 和系统故障处理不得在异常后
   挂死并保持最后一帧 PWM：软件栈溢出先尝试停机，内核故障则直接复位。

## 模块职责

- `driver/`：只处理器件协议和硬件错误，不决定飞行模式。
- `imu.c`：原始采样守卫、标定、滤波和姿态估计。
- `flow_navigation_core.c`：RA6M5 路线的 TOF/垂直加速度互补、光流低通、
  陀螺补偿和速度积分位置；不依赖 FSP/FreeRTOS。
- `flow_hold_controller.c`：定高与定点串级闭环、CH6 模式切换和失效降级。
- `flight_safety.c`：解锁、失联、故障锁存和执行器授权。
- `flight_control.c`：目标生成、PID、混控和控制故障检测；不直接访问 GPT。
- `actuator_manager.c`：最终授权、整帧范围检查、PWM 提交和停机结果聚合。
- `flight_snapshot.c`：在 500 Hz 控制周期末尾发布遥测一致快照。
- `freertos_hooks.c`：栈溢出和 Cortex-M 系统故障的失效安全复位。
- `*_thread_entry.c`：任务调度和模块编排，不承载控制算法。

## 自动验证

- `tests/run_host_tests.ps1`：在主机端验证 PID、Quad-X、UPIX 协议、导航融合、
  定高/定点方向和失效降级。
- `tests/run_profile_syntax_checks.ps1`：对 SAFE、系留、全部台架和诊断 profile
  的全部应用源码执行 `-Werror` 语法回归。
- `tests/verify_safe_elf.ps1`：确认默认 ELF 中不存在执行器应用、飞控更新和
  混控调用路径，且停机与故障处理符号存在。

## 后续演进

定高和定点已通过纯算法模块接入，下一步只按实机记录逐级调整本机架增益，
不得直接复制 RA6M5 的 PID 数值。硬件 bench/影子模式继续复用生产控制器，
只替换执行器输出授权。
