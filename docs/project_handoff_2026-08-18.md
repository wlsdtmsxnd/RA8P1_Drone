# RA8P1 四轴无人机项目交接

更新时间：2026-08-18

本文记录当前磁盘上的实际工程状态、已经完成的验证、尚未完成的工作和
接手时必须遵守的安全边界。后续人员应先核对本文与源码，不要仅根据历史
聊天或某一份旧 CSV 判断固件状态。

## 1. 工程与参考项目

- 主工程：`D:\Renesas\RA8P1_Drone`
- RT1064 同机架参考：`D:\Renesas\RT1064_Drone_10inch`
- RA6M5 完整控制流程参考：`D:\Renesas\old_fly`
- 最新 VOFA 数据：`D:\Renesas\VOFA_Date\vofa+.csv`
- IDE：Renesas e2 studio v2026-04.2
- FSP：6.5.0
- 芯片：R7KA8P1KF_CPU0
- 编译器：GCC ARM Embedded 13.2.rel1
- 调试器：SEGGER J-Link，SWD，当前速度 4000 kHz

目标机架和动力系统与 RT1064 工程相同。RA6M5 工程的机架尺寸和动力系统
不同，只能参考控制流程、状态管理和模块划分，禁止复制其 PID 数值，也
禁止把其 FSP 生成文件复制到本工程。

## 2. 当前 Git 与工作区状态

- 当前分支：`main`
- 当前 HEAD：`7005f50 上桨叶前虚拟测试`
- 工作区包含未提交且有效的修改，禁止 reset、checkout 或覆盖。
- 当前修改文件：
  - `docs/debug_guide.md`
  - `docs/migration_status.md`
  - `src/code/flight_control.c`
  - `src/code/flight_control.h`
  - `src/code/project_config.h`
  - `src/imu_thread_entry.c`
  - `src/telemetry_thread_entry.c`
- `src/code/pid_controller.c/.h` 和 `src/code/rc_command.c/.h` 已被 Git 跟踪，
  不要误判为可删除的临时文件。

本轮交接前曾讨论下一阶段 D 项影子测试，但用户中止了该轮。D 项测试模式、
D 参数和 D 遥测均没有写入源码，不能把讨论内容当成已实现功能。

## 3. 当前默认安全固件

2026-08-18，用户已明确确认固定工装、桨叶、桨盘隔离和主动力紧急断电
条件，并授权本次固定工装桨载振动基线测试安装桨叶、接通 ESC 动力。
该基线测试已完成并结束授权；`src/code/project_config.h` 当前已恢复为：

```c
#define ESC_BENCH_MODE                    ESC_BENCH_MODE_DISABLED
#define ESC_BENCH_SAFETY_ACKNOWLEDGED     (0U)
#define CONTROL_BENCH_MODE                CONTROL_BENCH_MODE_DISABLED
#define CONTROL_BENCH_SAFETY_ACKNOWLEDGED (0U)
#define CONTROL_BENCH_ESC_POWER_ACKNOWLEDGED (0U)
#define PROP_LOAD_TEST_MODE               PROP_LOAD_TEST_MODE_DISABLED
#define PROP_LOAD_TEST_AIRFRAME_RESTRAINED_ACKNOWLEDGED (0U)
#define PROP_LOAD_TEST_PROPELLERS_ACKNOWLEDGED (0U)
#define PROP_LOAD_TEST_ESC_POWER_ACKNOWLEDGED (0U)
#define TELEMETRY_SOURCE                  TELEMETRY_SOURCE_RC_COMMAND
```

该配置下即使进入 ARMED，四路电机仍保持 1000 us。链接符号检查中
`flight_control_prop_load_vibration_update`、`motor_output_set_us` 和
`pid_controller_update` 均不在当前安全 ELF 中。

当前默认安全固件 `clean all` 成功：

- `text=35348, data=176, bss=22032`
- SREC SHA256：
  `9935A3D25593350D2BD86133A39D2908ED2276673E56FF26532F4943A85417AC`
- ELF SHA256：
  `A9ACE2C4344FD0F59948E7B1FF0668BD70831170E1E15C764A53B09B35866E28`

注意：当前工程中的 ELF/SREC 已被默认安全构建覆盖，不再是可驱动
桨载电机的专用测试固件。

### 已完成的专用桨载测试固件记录

已完成的专用固件尺寸为 `text=35860, data=180, bss=22028`，SREC
SHA256 为 `112609105E638DCF0C5B1A3EFB8DDFE10707084DB70E68199AC5DBAA9AE8D1EC`，
ELF SHA256 为 `3EBE5105EE7268EC5DD26BE7EA3D9FC4437C586896796D848025481F30489342`。
该固件已作废，如后续确有需要必须重新取得独立授权并构建。

### 测试前的默认安全固件记录

启用本次专用模式前，默认安全固件的记录为：

- ELF：`Debug\ra8p1_project1.elf`
- SREC：`Debug\ra8p1_project1.srec`
- `text=35308, data=176, bss=22032`
- SREC SHA256：
  `CF1AEB2745EA1E3298E90D36854CDE8BC5D9D3A98D7B24176D6EA2CBDCE5F36C`
- ELF SHA256：
  `89062D3F8979C4653BCE90A372465EEE69B261C24C1A1183701E7CD0F7E60FE6`
- 安全配置的链接符号检查只保留 `motor_output_all_stop`，控制/PID 输出路径
  被编译期裁剪，没有 `motor_output_set_us` 控制调用。

新增默认禁用的桨载振动基线源码后，安全 SREC 和 text/data/bss 与上一版
完全相同；ELF 哈希变化来自新增源码对应的 DWARF 调试行信息，不代表安全
机器码输出路径发生变化。`flight_control_prop_load_vibration_update`、
`motor_output_set_us` 和 `pid_controller_update` 均不在安全 ELF 符号表中。

## 4. 构建方法与环境陷阱

完整构建命令：

```powershell
& 'C:\Renesas\RA\e2studio_v2026-04.2_fsp_v6.5.0\eclipse\plugins\com.renesas.ide.exttools.gnumake.win32.x86_64_4.3.1.v20240909-0854\mk\make.exe' -C Debug clean all
```

若出现 `arm-none-eabi-gcc not found`，检查工具链：

```text
C:\Renesas\RA\e2studio_v2026-04.2_fsp_v6.5.0\toolchains\gcc_arm\13.2.rel1\bin
```

以前无界面 e2 studio 扫描曾覆盖 `Debug/makefile.init` 中的 PATH。不要为
解决普通编译问题随意重新生成工程内容，也不要触碰 FSP 生成文件。修改后
必须执行全量构建，不能只依赖增量结果。

## 5. 硬件映射

| 功能 | 引脚/外设 |
|---|---|
| ICM42688 SCK | P415 / SPI1 RSPCK |
| ICM42688 MOSI | P708 / SPI1 MOSI |
| ICM42688 MISO | P709 / SPI1 MISO |
| ICM42688 CS | P710 / GPIO 低有效 |
| CRSF RX/TX | P602/P603 / SCI0 / 420000 baud |
| 3DR RX/TX | P802/P801 / SCI2 / 57600 baud |
| GPS RX/TX | P715/P714 / SCI4 / 115200 baud |
| GPS compass SDA/SCL | P514/P515 / IIC2 |
| M1 左前 | P700 / GPT5 A |
| M2 右前 | P701 / GPT5 B |
| M3 右后 | P109 / GPT10 A |
| M4 左后 | P702 / GPT6 A |

ESC PWM 为 50 Hz，范围 1000-2000 us；上电、撤防和任何故障状态默认
1000 us。M1-M4 电机顺序已经通过拆桨实机测试，但本文没有记录桨叶型号、
桨叶安装方向或各电机物理旋转方向，安装桨叶前必须重新建立独立检查表。

桨叶包装信息现已记录到 `docs/propeller_record_2026-08-18.md`：候选桨叶为
GEMFAN 1045、CF Nylon、10 英寸双叶桨，每包两只且标签为 1 CCW + 1 CW；
1045 型号对应的标称螺距按 4.5 英寸记录。包装照片已复制到
`docs/evidence/`。该证据只确认型号和单包组合，尚未确认现有至少两包、
四只实物的外观/中心孔、M1-M4 实际旋向、安装位置和紧固，因此不能把桨叶
确认位改为 `1U`。

## 6. IMU 与姿态状态

- ICM42688 SPI 驱动完成。
- IMU 任务 500 Hz。
- Mahony 姿态解算完成。
- 开机静止陀螺零偏标定完成。
- 机体系使用 FRD：X 前、Y 右、Z 下。
- 加速度比力取反后参与重力校正。
- 当前水平修正以源码为准：
  - `IMU_LEVEL_ROLL_TRIM_DEG = 0.0f`
  - `IMU_LEVEL_PITCH_TRIM_DEG = -0.97f`

如果旧说明写着“水平修正已清零”，属于过时描述。不要把 Pitch 修正改回
零，除非重新完成水平基准测量并保存证据。

## 7. 遥控归一化与安全状态机

当前通道：Roll CH1、Pitch CH2、Throttle CH3、Yaw CH4、Arm CH5、Mode
CH6。归一化输出：

| VOFA 通道 | 含义 | 范围 |
|---|---|---|
| I0 | Roll | -1 到 +1 |
| I1 | Pitch | -1 到 +1 |
| I2 | Throttle | 0 到 1 |
| I3 | Yaw | -1 到 +1 |
| I4 | Arm | 0/1 |
| I5 | Mode | 0/1/2 |
| I6 | 安全状态 | 0/1/2/3 |
| I7 | IMU 就绪 | 0/1 |

实测端点：

- Roll：989 / 1502 / 2011 us
- Pitch：989 / 1490 / 2001 us
- Yaw：990 / 1506 / 2011 us
- Throttle：1050-2010 us
- 中位死区：20 us

安全状态：0 DISARMED、1 ARMING_WAIT、2 ARMED、3 FAILSAFE。

已经实测通过的规则：启动后必须先看到 CH5 低档；低油门并将 CH5 置高
保持 1 秒才解锁；CH5 低档、遥控失联超过约 300 ms 或 IMU 异常立即
停机；重连后不会自动解锁；高油门打开 CH5 不会解锁。

早期 Yaw 记录只到约 -0.123 到 +0.173，后续补测确认是测试操作理解问题，
不是归一化参数问题。后续记录中 Yaw 已覆盖正负满杆。禁止通过修改端点
掩盖 EdgeTX 输出、CRSF 原始通道或操作流程问题。

## 8. 控制结构与当前参数

`flight_control.c` 已形成统一控制路径：

1. Roll/Pitch 摇杆生成目标角度。
2. 角度外环生成目标角速度。
3. Roll/Pitch/Yaw 角速度内环生成三轴修正量。
4. 四轴 X 型混控生成 M1-M4 目标脉宽。
5. 专用通电模式再执行软启动、限幅和保护。

混控关系：

```text
M1 = base + pitch + roll - yaw
M2 = base + pitch - roll + yaw
M3 = base - pitch - roll - yaw
M4 = base - pitch + roll + yaw
```

独立 `pid_controller` 已实现：显式 `dt`、输出限幅、积分限幅、测量值微分
滤波、条件积分抗饱和、统一复位以及 NaN/Inf 拒绝。

当前正式 P-only 参数仅用于拆桨台架验证，不是可飞参数：

- Roll/Pitch 角度外环 `Kp=3.0`
- Roll/Pitch/Yaw 角速度内环 `Kp=0.25`
- 正式通电复测中 `Ki=0`、`Kd=0`
- Roll/Pitch 最大目标角度：8 deg
- Yaw 最大目标角速度：24 deg/s
- 基础脉宽：1150-1220 us
- 单轴修正限幅：+/-12 us
- 输出上限：1280 us
- 倾角保护：15 deg
- 角速度保护：100 deg/s
- 启动上升斜率：每 2 ms 最多 1 us，即 500 us/s
- 普通下降斜率：每 2 ms 最多 2 us
- 撤防、失联、IMU 异常、低油门和保护触发均立即 1000 us，不走斜坡

独立积分影子模式曾使用 `Ki=0.02`、积分限幅 +/-2 us。它只证明积分器的
数学行为和复位路径正确，不是已经批准通电或飞行的 Ki。

## 9. 已完成验证时间线

以下项目均已完成；“波形测试”表示拆桨、固定机架、ESC 动力断开：

1. ICM42688 通信、姿态方向、500 Hz 更新和静止标定。
2. CRSF 16 通道、遥控端点、失联检测、VOFA JustFloat 遥测。
3. 四路 GPT 50 Hz/1000-2000 us、安全默认输出。
4. M1-M4 无桨电机顺序和四电机稳定启动。
5. 纯摇杆混控六方向波形。
6. Roll/Pitch 单角度回水平反馈四方向。
7. X/Y/Z 单角速度阻尼六方向。
8. Roll/Pitch 串级自稳波形。
9. Roll/Pitch 受限遥控目标角度。
10. Yaw 受限目标角速度。
11. 三轴综合断电波形及组合输入。
12. 三轴综合无桨通电测试。
13. 正式统一控制器影子运行，示波器确认实际四路始终 1000 us。
14. P-only PID 内核替换后的数值等效性复测。
15. 极小角速度积分影子测试。
16. 正式 P-only PID 无桨通电复测。

最终 P-only 通电复测的用户机械确认结果全部为“是”：

- 四个电机同步稳定启动，无某一路延迟或停转。
- CH5 拨低后四个电机全部立即停转。
- Roll/Pitch/Yaw 六个摇杆方向的实际快慢变化正确。
- 右侧压低、机头抬高的实际自稳反馈方向正确。
- 未出现抖动、异响、失步或机架松动。

## 10. 最新 VOFA 数据结论

文件：`D:\Renesas\VOFA_Date\vofa+.csv`

- 文件时间：2026-08-18 00:25:28
- 文件大小：668054 字节
- CSV 是追加记录，前 3463 行约 69.26 秒属于旧积分影子测试。
- 正式 P-only 通电有效段从原始第 3463 行附近开始。
- 有效通电段：4678 行，约 93.56 秒，无 NaN/Inf。
- 状态完整覆盖 DISARMED、ARMING_WAIT、ARMED，并正常撤防。
- 1491 个非 ARMED 样本的 M1-M4 全部严格为 1000 us。
- 五次启动斜率：497.2-500.0 us/s。
- 每次四路启动差不超过 1 us。
- 两次 CH5 撤防均在下一遥测帧恢复四路 1000 us。
- 最大命令：M1 1192、M2 1184、M3 1192、M4 1180 us。
- 没有命令越过 1000-1280 us 台架范围。
- Roll/Pitch/Yaw 隔离指令的差动符号一致率均为 100%。
- Roll 右：M1/M4 增，Roll 左：M2/M3 增。
- Pitch 后拉：M1/M2 增，Pitch 前推：M3/M4 增。
- Yaw 右：M2/M4 增，Yaw 左：M1/M3 增。
- 右侧压低产生负 Roll 修正；机头抬高产生负 Pitch 修正，均为回水平方向。

CSV 第一行可能包含 VOFA 上一次连接残留样本，不能直接把整份文件的第一行
当成当前固件起点。分析新测试时应先按状态和四路 PWM 范围切分有效段。

## 11. 尚未完成与禁止推断

- 尚未建立可飞 PI/PID 参数。
- D 项尚未启用，也没有完成静止噪声或电机振动下的微分输出评估。
- `Ki=0.02` 只通过断电影子测试，没有获准接入通电电机控制。
- 尚未完成带桨静态推力、桨叶方向/紧固检查、系留、首次离地和实际调参。
- 定高、光流、低电压、电池监控和传感器冗余保护尚未形成可飞闭环。
- 不能因为所有无桨测试通过就声称项目可以安装桨叶飞行。
- 不得直接采用 RA6M5 PID 参数。
- 不得在未确认桨叶拆除时启用 ESC 校准或电机顺序测试。

## 12. 重新评估后的下一阶段

2026-08-18 复核源码和现有证据后，决定不把独立的无桨、断电 D 项影子测试
作为下一阶段，也不新增该测试模式。当前 PID 内核已经采用测量值微分：
正角加速度产生负 D 阻尼，首次采样不产生微分冲击，撤防和保护路径会统一
复位微分状态；当前 `Kd=0`。断电手动转动只能再次检查这些软件行为和桌面
静止噪声，不能覆盖电机/桨叶振动、机架共振、推力响应、闭环相位裕度，
因此不能据此确定真实 `Kd` 或微分滤波参数。

更有价值的推进顺序调整为：

1. 保持当前默认安全固件，不再增加 I/D 断电影子阶段。
2. 在任何带桨测试前，先补齐独立的桨叶型号和正反桨位置、M1-M4 实际旋向、
   紧固方式、动力电池、场地隔离、固定/系留、紧急停机和数据记录检查表。
3. 先准备能够同时记录实际四路输出、三轴滤波陀螺和安全状态的调参数据，
   并将正常飞行输出与历史台架模式明确隔离；默认配置仍必须保持停机。
4. 只有用户再次明确允许接通 ESC 动力和安装桨叶后，才进行固定机架的低能量
   桨载振动/动力基线采样。该阶段仍保持姿态 `Ki=Kd=0`，只用于确认机械、
   振动、油门工作区和记录链路，不从固定机架数据宣称得到可飞 PID。
5. 后续受限动态测试先调角速度内环 P，再根据真实振荡和阻尼表现决定是否
   引入 D，最后处理 I；角度外环在角速度内环可控后再调。每次只改变一项，
   参数必须来自本机架实测，不复制 RA6M5 数值。

上述顺序不等于当前已获准安装桨叶。任何 ESC 动力、桨载、固定推力、系留
或离地阶段都必须重新取得用户明确确认，并在测试后恢复 `DISABLED/0U`、
正常遥测源和安全固件。

软件准备现已完成第一步：新增独立的
`PROP_LOAD_TEST_MODE_VIBRATION_BASELINE`，但默认仍为 `DISABLED`。该模式
不调用 PID 或混控，只把归一化油门映射为四路相同的 1000-1180 us 目标，
并以 500 us/s 缓升；CH5 低、低油门、失联、IMU 异常、倾角超过 10 度或
任一滤波角速度超过 100 deg/s 都会立即恢复 1000 us。专用遥测为 100 Hz：
I0-I3 是实际 M1-M4 脉宽，I4-I6 是三轴 40 Hz 带限角速度，I7 是安全状态。

该模式有固定工装、桨叶检查、ESC 动力三项独立确认位，并与 ESC 台架模式、
控制台架模式互斥。当前三项确认必须全部保持 `0U`；只有用户再次逐项确认
后才能构建可运行的桨载测试固件。详细操作与判定标准见 `debug_guide.md`
第 20 节。

## 13. 接手后的首轮检查清单

1. 阅读本文件、`docs/migration_status.md` 和 `docs/debug_guide.md`。
2. 执行 `git status --short` 和 `git diff --check`，保留现有有效修改。
3. 检查 `project_config.h` 是否仍为全部禁用、确认位 0U、RC 遥测。
4. 对当前状态执行一次 `clean all`。
5. 记录 ELF/SREC 体积和 SHA256；若与本文不同，先解释差异再烧录。
6. 任何波形模式均要求拆桨、固定机架、ESC 动力断开。
7. 任何无桨通电模式均要求用户重新明确授权，且只能使用独立动力确认位。
8. 测试结束必须恢复安全配置并再次全量编译。
9. 保存每次 VOFA CSV，不覆盖上一阶段证据；注明固件配置、时间和操作序列。
10. 不得把当前项目描述为可带桨飞行状态。

## 14. 当前专用系留短跳固件

用户已确认系留现场条件，并要求跳过重复的无桨硬件验证。源码新增独立
`TETHERED_FLIGHT_MODE_FIRST_HOP`，当前该模式和系留、桨叶、隔离/停机、
ESC 动力四项确认位均已启用；ESC/控制台架和桨载基线模式仍为禁用。

首次参数为角度外环 Kp=0.40，Roll/Pitch 角速度内环 Kp=0.22，Yaw 角速度
内环 Kp=0.20，全部 Ki=Kd=0；目标倾角正负 3 度，Yaw 正负 10 deg/s，
第一轮 1400 us 未离地且数据正常；第二轮基础油门改为 1150-1450 us，
总输出 1000-1470 us，单轴修正正负 20 us。基础
油门以 200 us/s 缓升，姿态差动保持 500 Hz 实时响应。解锁要求水平正负
3 度、三轴角速度正负 10 deg/s 和姿态摇杆中位；10 度/100 deg/s 保护及
GPT 错误会锁存停机，CH5 拨低前不会自动恢复。

专用 VOFA 为 17 通道、50 Hz，I16 新增锁存故障原因：1 非有限数、2/3
Roll/Pitch 倾角、4/5/6 X/Y/Z 角速度、7 GPT 写入错误，故障码保持到 CH5
拨低。详细定义和操作判据见 `debug_guide.md` 第 21 节。当前构建为
`text=37844, data=232, bss=22240`；ELF SHA256：
`E217F09B7E2F46726FF77968B5CA644FDB319BC6A7090A4B6BBEBAED694E3BEF`；SREC
SHA256：`92532B1FCF6D11F2F970E26762AEF5536FA1B89FE69136BF6275DEE0AF108B4F`。
该产物只用于首次系留短跳，不是自由飞行固件。测试结束和数据读取后必须
恢复 `DISABLED/0U` 并重新构建、烧录默认安全固件。

## 15. 当前 IMU 首读/复读诊断固件

最新 VOFA 记录确认系留停机原因为 Y 轴角速度保护，并在撤防、电机全停时
发现更大的孤立六轴尖峰。当前已暂停系留阶段，系留模式和四个确认位恢复为
`DISABLED/0U`。新增并启用 `IMU_DIAGNOSTIC_MODE_RAW_REREAD`：滤波前异常
触发后立即复读完整六轴寄存器，每次上电只锁存第一组首读/复读和 24 个原始
字节。主板供电已捕获首读 Z 陀螺 12287、立即复读 -7 counts，且加速度三轴
完全相同，确认是单事务读取瞬态。随后新增永久原始采样守卫：只有复读成功、
字节不同且复读恢复正常时才用复读替换首读；两次都异常时仍保留原保护。
专用遥测为 51 通道、20 Hz，定义和操作见 `debug_guide.md` 第 22 节。

该固件每 2 ms 强制四路 1000 us，不包含 PID、飞控更新或提高电机脉宽路径；
只允许在 ESC 主动力断开条件下比较 J-Link 5 V 和主板 4.99 V 供电。取得数据
后必须恢复诊断模式 `DISABLED`、正常 RC 遥测并重建默认安全固件。

## 16. 当前守卫版精简系留固件

用户要求在软件守卫生效后继续短跳。当前诊断模式已关闭，系留模式及系留、
桨叶、隔离/停机和ESC动力四项确认位已恢复 `FIRST_HOP/1U`。原始采样守卫
不依赖诊断模式，仍会在滤波前对异常首读立即复读，并仅在复读可证实首读为
单事务瞬态时替换整组六轴。

当前系留遥测为15通道、50 Hz：I0-I3四电机，I4油门，I5/I6为Roll/Pitch，
I7为归一化Roll指令，I8为三轴最大滤波角速度，I9安全状态，I10故障码，
I11守卫成功替换次数，I12复读事务失败次数，I13控制有效标志，I14锁存撤防
原因。上一轮1450 us保持约8秒仍未离地，因此本轮只将
基础/总输出上限提高至1550/1570 us；P-only参数、10度/100 dps保护和
200 us/s基础缓升均未改变。详细操作见 `debug_guide.md` 第21节。

首次接近离地时出现连续前倾，并在 Pitch=-10.378 度触发故障码3。
数据确认 Pitch 混控方向正确，但保护前差动仅增加到约11 us。第二次
只将 Pitch 角速度内环 Kp 从0.22提高到0.30 us/(deg/s)；Roll=0.22、
Yaw=0.20、角度外环0.40、Ki=Kd=0、1550/1570 us限幅、10度/100 dps保护
和200 us/s缓升保持不变。

该第二次短跳专用固件已执行 `clean all` 并成功：`text=38940`、
`data=232`、`bss=22368`。ELF SHA256 为
`274143261AD513F7C258C61133E85718614629AA76DAC91F868243192114DBDC`，
SREC SHA256 为
`9E3B3B764AB40DF5365CF0BDE16CC8D0E8551ABE5565A1E47B1B9F35EBEB4900`。

第二次仍在接近离地时向前倾。参数变更已把 -8.5 度附近的 Pitch 差动
由约11 us提高到14 us，最大角速度由47.2降到43.6 deg/s，但仍无法制止
持续前倾。第三次已将角度外环拆成独立 Roll/Pitch 参数：Pitch 外环
Kp=3.0，Roll 外环 Kp=0.40；Pitch 内环仍为0.30，目标角速度限幅仍为
正负10 deg/s，其他参数与保护均不变。

第三次短跳专用固件已执行 `clean all` 并成功：`text=38940`、
`data=232`、`bss=22368`。ELF SHA256 为
`0201209FDDB1714CBC82C8E206AECEF52D23302D808E7D3E56927D027ED0AB8B`，
SREC SHA256 为
`13211AFE1019285BF90D71B71CB9B29425038CDF9DC59276BBFEEBE5A65D6243`。

电池后移后，接近离地段 Pitch/Roll 已稳定在 -0.27..+1.45/-0.96..+0.15
度，之前的持续前倾消失。本轮由孤立 Pitch/Y 角速度尖峰 236.12 deg/s
触发故障码5，下一遥测帧恢复到9.49 deg/s。系留角速度保护已改为：
100..300 dps连续20 ms才停机，超过300 dps连续4 ms才停机。恢复至100 dps
内立即清计数；10度倾角、非有限数、GPT故障和撤防逻辑不延时。

该持续时间保护版固件已执行 `clean all` 并成功：`text=39196`、
`data=232`、`bss=22400`。ELF SHA256 为
`48F95FE09049EE8D4AFFACF3F6864F946088522D4D64FE7D0D77C701E42C8035`，
SREC SHA256 为
`8A2AC2EF4E6B1A3E0586DAC3322E60E2FE2EA2E77060A8A4E56BA837B3969F7E`。

持续保护版首次尝试在约1266 us时无故障码地由ARMED直接转为
DISARMED，证明CH5离开高档范围。第二次已离地2至3 cm，但发送左Roll后
左倾至-7.59度并触地；左右电机差动方向正确，Roll外环0.40回正过弱。
已只将Roll角度外环Kp提高到3.0，Roll内环仍为0.22，其他参数和保护
不变。系留遥测改为15通道：I7为归一化Roll摇杆指令，I14为锁存
撤防原因（0无、1 IMU、2 CRSF失联、3 CH5低、4 CH5离开高档但未到低档）。

该Roll外环与撤防诊断版固件已执行 `clean all` 并成功：`text=39308`、
`data=232`、`bss=22400`。ELF SHA256 为
`2BA51BB1B45C897CE5FC27D9ACB6F9DBE84642A88491DC4DD5DC7F22F08907B4`，
SREC SHA256 为
`1CE1F4E33478B2BAFC77CD98B6C37140D24024CC7672A70B19F4E47DA0F66E52`。
