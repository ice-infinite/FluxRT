# DengFOC ESP32 + AS5600 板级适配

## 1. 结论与证据边界

本适配以本机参考工程 `E:\File\PlatformIO\DengFOC` 的当前源码为事实来源，目标是把
DengFOC 板作为 FluxRT 的另一套硬件平台，并先用板载 AS5600 为 HFI、SMO/BEMF 和换相角
提供独立角度真值。

当前已经完成：

- 固化当前参考工程的 ESP32、PWM、ADC、使能、双 I2C 和 AS5600 板级配置；
- 补齐 FluxRT `AbsoluteEncoder` 的路由、质量门、方向/零偏标定和配置事务；
- 实现 AS5600 12 bit 解码、跨零点展开、速度/电角度换算和失效清零；
- 建立不依赖 Arduino/ESP-IDF 的 I2C 端口，平台绑定只实现 `begin/read_register`；
- Host 测试核对两个 Axis 的引脚、I2C 寄存器访问、跨零点、时间戳和 fail-zero。
- 建立 `targets/esp32_dengfoc` PlatformIO/Arduino USB-only 诊断目标；它不初始化 LEDC，
  PWM 引脚保持输入态，并在 `setup/loop` 中持续把 GPIO12 压在禁用电平；
- 增加硬件无关电流ADC零点统计器，启动时对M0 A/B各取1000点并输出均值、极值、标准差、
  零点电压和名义电流噪声；
- 增加默认关闭功率事务与Arduino LEDC/ADC/GPIO薄绑定；诊断策略不调用PWM准备，双重许可均为0；
- ESP32 目标构建通过；并新增独立 MCPWM timing 目标，以单定时器 up-down 计数建立
  29,985 Hz 中心对齐时基和最小周期 ISR。

当前尚未完成：PWM/ADC硬件同步、硬件快速关断、电流比例/极性和正式HFI功率门。H1已取得
正反方向各一次完整AS5600证据，但同方向历史上也出现过0.4719 A软件关断，证明异步采样
非确定性仍存在。因此DengFOC的正式职责已冻结为 **AS5600独立真值 + 低压commissioning**；
正式H2迁移到STM32G431/IHM16M1。DengFOC不得作为第二逆变器与G431同时连接同一台电机。
详见[H1收口与H2平台迁移门](../performance/2026-10-04-P5.5-H1收口与H2平台迁移门.md)。

2026-10-04 已在用户确认只接USB、12 V断开后，把诊断固件烧入COM13对应的DengFOC板。
ESP32识别为D0WD-V3 revision 3.1、8 MB Flash；启动回读`disabled=1`、`axis0_init=1`，
AS5600连续`valid=1/fail=0`。当前角度保持5.758564 rad，没有捕获到人工转动，因此只关闭了
静态通信与应用关断S4子门，方向、跨零和整圈连续性仍未验证。

随后45 s动态窗口捕获到双向累计运动7.697526 rad、跨零2次，机械多圈角连续且`fail=0`，
说明双向变化和跨零展开有效。但单方向最大展开跨度只有4.313558 rad；追加的35 s“两整圈”
窗口没有捕获到转动，因此完整单方向一圈、物理正方向和4096 count覆盖仍待验证。

最终同步窗口由用户从电机轴伸出端看连续顺时针转动两圈：890个有效遥测、`fail=0`、
跨零2次，净正向展开12.813305 rad（约2.04圈），正向累计12.962083 rad，反向小量
0.148778 rad，结果为`ONE_DIRECTION_2TURN_OK`。因此当前`direction=+1`与该物理顺时针定义
一致，单圈域、跨零和多圈展开门已关闭。`electrical_offset_rad=0`仍只是占位值，没有完成转子
磁极d轴零偏标定，不能据此直接进入有感FOC。

随后USB-only诊断固件增加M0电流ADC零点采样。硬复位后1000点结果为：A/B均值
1859.874/1860.289 count，标准差2.229/2.270 count，均值相差0.415 count；按参考工程名义
`0.01 Ω × 50`换算，RMS噪声约3.593/3.659 mA。采样前后`GPIO12 disabled=1`，没有初始化
LEDC，AS5600仍`valid=1/fail=0`。这关闭了静态零点可读性子门，但没有已知电流源，因此不能
证明A/count、极性、运行期同步采样或功率级共模影响。

默认关闭功率执行器镜像随后在相同USB-only条件下完成S4。最终BIN SHA-256为
`A1092C854C2396BD4AEE28AE816E74C31A28C047CE8190EABB653EAC7DB6B599`，启动回读
`power_stage_init=1,state=1,allow_actuation=0,pwm_configured=0`。12 s观察窗239/239帧AS5600
有效、I2C失败0、FATAL 0；因此当前板上固件仍不配置LEDC、不允许驱动使能。

独立`dengfoc_usb_pwm_prepare`构建档随后完成USB-only S4：它只在GPIO12确认禁用后配置
M0三路LEDC为30 kHz/8 bit并写入0/0/0，验证arm被拒绝并每100 ms复核频率、duty和安全
状态。板上当前BIN SHA-256为
`181EC942E4C402FBA14DB9002683D8849326B6382C19B0EC30F5C56288A9BCE6`，启动读回
`state=2,disabled=1,configured=1,freq=29996/29996/29996,duty=0/0/0,arm_refused=1`；
14 s主观察窗及最终镜像6 s复核均无FATAL且AS5600失败计数为0。该结果证明LEDC定时器配置与软件安全门，不证明
中心对齐/ADC同步，也不授权12 V或电机运行。0% duty的物理引脚没有边沿，LA1010只能验证
低电平，若要直接量载波频率必须另行审批禁驱动的非零诊断波形。

在用户暂时没有逻辑分析仪后，新增的`dengfoc_usb_mcpwm_timing`继续完成USB-only S4。
该镜像把M0三相放在同一个MCPWM up-down定时器下，读回29,985 Hz，并用最小full事件ISR
提供快环时基；每100 ms检查事件计数必须前进。三路compare先写shadow，再由经典ESP32
MCPWM的group-wide flush一次同步生效。板上固件SHA-256为
`4193461F937D121006EF037F05DF9CB60EDD392D6B7980A9B0BA22EB9DB3005C`。独立硬复位后读回
`caps=0x47,missing_runtime=0x38,coherent=1`，12 s内无FATAL，GPIO12仍禁用、三相被永久
强制为低、非零duty和arm均拒绝。它证明中心对齐时基、同步shadow提交及周期ISR运行，
不证明相位锁定ADC、ADC overrun检测或硬件快速关断；详见
[MCPWM同步执行器软件门](../performance/2026-10-04-P5.5-H1-DengFOC-MCPWM同步执行器软件门.md)。

在此基础上又完成ESP32 Rust静态库和编码器实时FOC的S1/S2准备。新ABI不再让C依次调用
兼容`speed_step/fast_step`，而是由Rust在单一逐拍入口内完成2 kHz速度/电流控制、角度补偿
和SVPWM；C只拥有MCPWM、ADC continuous和AS5600采集。USB-only探针同时记录ADC DMA
overrun/read error、丢失窗口及PWM full事件后的样本延迟。该ADC与MCPWM仍为异步时钟域，
所以不能声明硬件相位锁定。新BIN已构建但当前未检测到COM口，尚未烧录；详情见
[ESP32 Rust编码器闭环准备门](../performance/2026-10-04-P5.5-H1-ESP32-Rust编码器闭环准备门.md)。

## 2. 从参考工程提取的硬件配置

| 对象 | 当前参考值 | 来源 |
|---|---:|---|
| MCU | ESP32-D0WD-V3，双核 240 MHz | `DengFOC双环控制与神经网络自动调参开发记录.md` |
| 母线 | 12 V | `src/main.cpp` |
| PWM | 30 kHz，8 bit | `src/DengFOC.cpp` |
| 当前参考控制频率 | 2 kHz | `src/main.cpp` |
| 驱动总使能 | GPIO12，高有效 | `src/DengFOC.cpp` |
| M0 PWM A/B/C | GPIO32/33/25 | `src/DengFOC.cpp` |
| M1 PWM A/B/C | GPIO26/27/14 | `src/DengFOC.cpp` |
| M0 电流 ADC A/B | GPIO39/36 | `src/InlineCurrent.cpp` |
| M1 电流 ADC A/B | GPIO35/34 | `src/InlineCurrent.cpp` |
| 电流采样 | 0.01 Ω，增益 50，V4 正极性 | `src/InlineCurrent.cpp` |
| M0 AS5600 | I2C0，SDA19/SCL18，400 kHz | `src/DengFOC.cpp` |
| M1 AS5600 | I2C1，SDA23/SCL5，400 kHz | `src/DengFOC.cpp` |
| AS5600 | 地址 `0x36`，角度寄存器 `0x0C`，4096 count/rev | `src/AS5600.cpp` |
| 默认启用 Axis | 只启用 M0 | `kEnableMotor1Runtime=false` |

2026-10-04实板照片没有显示GPIO4通用排针，因此GPIO4不能仅凭“源码未占用”作为接线依据。
照片显示两组传感器插座；用户实物目视确认同步候选插座丝印依次包含
`VCC / GND / IO23 / IO5 / IO21`。当前V0.4 profile把M1 I2C映射为SDA23/SCL5，因而
`IO23=SDA1`、`IO5=SCL1`与源码一致，`IO21`不参与当前同步。H3同步诊断因此只在
Axis1关闭时把SDA1/GPIO23作为RMT RX候选，并由
`foc_time_sync_dengfoc_input_resource_valid()`拒绝所有PWM/ADC/GPIO12冲突及Axis1同时启用。
实物丝印已经闭合针名/顺序证据；未做电气连续性测量，首次接线仍只允许USB供电、
Axis1传感器断开、VCC/IO5/IO21悬空，并以RMT单帧接收作为功能复核。

板端功率相丝印使用 `A/B/C`，不是 `U/V/W`。本适配固定采用
`A=U`、`B=V`、`C=W` 作为命名别名；LCR 线间测量因此记录为 `A-B`、`B-C`、`C-A`。
该别名只统一软件和文档命名，不能替代后续低压限流下的电气相序与电角度零偏标定。

2026-10-04 H1 首个无功率 LCR 测点使用 LT1 的 1 kHz/0.6 V 条件。连续测得：

| 线间 | Q | 交流等效R/Ω | 线间L/mH | `L/2`诊断值/mH |
|---|---:|---:|---:|---:|
| A-B | 0.895 | 16.35 | 2.320 | 1.160 |
| B-C | 0.854 | 16.32 | 2.230 | 1.115 |
| C-A | 1.024 | 16.39 | 2.666 | 1.333 |

三对交流等效R极差仅0.07 Ω（均值的0.43%），而线间L极差0.436 mH（均值的18.13%）。
测量结束后立即通过AS5600捕获的机械角为0.360485～0.362019 rad，I2C失败0。该角度只作为
“位置A”的事后索引；未同步记录每次换线时的角度，因此本轮只能建立位置敏感性候选，不能
直接给出`Ld/Lq`。`L/2`也只是在星形/平衡近似下便于比较的诊断值，不能写入正式电机参数。
LCR显示的R包含1 kHz交流损耗，同样不能直接作为FOC的直流相电阻`Rs`。

随后保持C-A线对并手转粗扫，在用户报告的最小点测得`Q=0.875`、`R=16.39 Ω`、
`L=2.287 mH`。停止后立即采集54个有效AS5600样本，机械角全部为5.632778 rad，
I2C失败0；因此该点建立为首个同步极值`CA-min-A`。按星形/平衡近似的`L/2=1.1435 mH`
仍只作为诊断值。C-A最大点及其同步角度尚待测量。

C-A手转粗扫的近似最大点测得`Q=1.011`、`R=16.31 Ω`、`L=2.638 mH`；停止后54个
AS5600有效样本全部为4.390253 rad，I2C失败0，记为`CA-max-A-candidate`。与同步最小点相比，
线间L增加0.351 mH，`Lmax/Lmin=1.1535`。这说明该线对存在约15.35%的粗扫变化，具有继续
评估HFI适用性的价值，但因最大值由手动观察取得且尚未复核，不能据此批准`Ld/Lq`或HFI。

在4.390253 rad保持不动并换线复核，A-B=`Q 0.889/R 16.30 Ω/L 2.309 mH`，
B-C=`0.846/16.28 Ω/2.189 mH`，C-A沿用`1.011/16.31 Ω/2.638 mH`。换线后再次采集
54个AS5600样本，角度仍全部为4.390253 rad、I2C失败0，因此三组属于同一机械位置。
该位置三对R极差仅0.03 Ω（均值的0.18%），线间L极差0.449 mH（均值的18.88%）；
差异不支持“某一绕组电阻明显异常”，但仍需在C-A最小位置完成同样三相复核，才能区分
随转子角度轮换的凸极响应与固定相别差异。

第二个位置最终稳定在5.464040 rad，A-B/B-C/C-A分别为2.235/2.695/2.284 mH；最高值
从P1的C-A转移到P2的B-C。按平衡星形与正弦凸极近似反算，两点分别得到
`Ld/Lq=1.0551/1.3235 mH`和`1.0565/1.3482 mH`，均值为1.0558/1.3359 mH，凸极比
约1.265。详细公式和证据边界见
[P5.5-H1 LCR凸极初筛](../performance/2026-10-04-P5.5-H1-LCR凸极初筛.md)。这些值只进入
未审批的小信号候选，不修改正式profile。

板级常量位于：

- `foc/platform/esp32_dengfoc/foc_board_dengfoc_v04.h/.c`
- `foc/platform/esp32_dengfoc/foc_dengfoc_as5600_port.h/.c`

注意：参考目录同时提到 V0.2/V0.4，当前源码的有效电流极性分支是 **V4 正极性**。实际
板卡若是 V3P 或不同修订版，电流极性可能为负，必须核对丝印/原理图后新建独立 profile，
不能直接改写 V0.4 profile。

当前参考工程`include/src`和FluxRT profile均没有给出独立driver fault/over-current输入，
只有GPIO12高有效总使能。因此GPIO12软件关断不能被报告为MCPWM硬件快速关断；如果实物
原理图存在未使用的故障网络，需先核对网络和有效电平，再为对应PCB revision增加专用profile。

## 3. 分层与数据流

```text
ESP32 Arduino Wire / ESP-IDF I2C
  └─ 只实现 begin + read_register
       └─ foc_dengfoc_as5600_port
            ├─ 选择 Axis 对应 I2C/引脚
            ├─ 读取 AS5600 0x0C 两字节
            └─ foc_as5600_tracker
                 ├─ 12 bit 解码与跨圈展开
                 ├─ 机械角/机械速度
                 ├─ pole_pairs + direction + offset → 电角度
                 └─ foc_feedback_adapt_absolute_encoder
                      └─ Rust FeedbackRouter / 配置事务
```

约束：

- I2C 是阻塞且速度远低于 12 kHz ADC/PWM 快环，不能在 STM32G431 ADC ISR 或未来 ESP32
  高频 PWM ISR 内直接读取；
- ESP32 平台应在独立传感器任务中采样，并通过固定容量 mailbox 提交最新样本；
- AS5600 是单圈绝对编码器，不需要增量编码器的 Index 门；多圈位置只由连续采样展开，
  掉电后不保持；
- 第一个样本以及超出最大采样间隔后的第一个样本只建立基线，不输出有效速度；
- I2C 失败、非法时间戳和非法配置均清零输出，不沿用旧角度冒充新样本。

## 4. 两种使用方式

### 4.1 有感控制

配置 `FeedbackMode::AbsoluteEncoder` 为 primary，完成方向和电角度零偏标定后，AS5600 可直接
提供有感 FOC 角度。它不需要 Index，但必须具备方向、极对数、零偏、新鲜时间戳、连续序列
和 calibrated 质量位。

### 4.2 HFI 独立真值

HFI/SMO 仍作为实际控制角度来源，AS5600 只记录：

```text
angle_error = wrap(HFI_angle - AS5600_electrical_angle)
```

这种模式用于统计零速/低速角误差、180°极性错误和 HFI→BEMF 切换跳变。真值通道不能反向
修改 HFI 输出，否则测试会变成同源自证。

## 5. 后续实机门

1. 确认实物板修订版及电流极性；
2. ~~增加 ESP32 构建目标和 Arduino I2C 薄绑定~~（已完成S2构建）；
3. USB 供电、驱动禁用时读取默认M0 AS5600的静态值、双向变化、跨零连续性和单方向两圈
   展开（已通过；物理顺时针对应`direction=+1`）；
4. 只启用 M0，验证 ADC 静态零偏及默认关闭功率执行器持续关断（USB-only S4已通过）；
   MCPWM中心对齐时基与周期ISR的无主电S4已通过。逻辑分析仪物理波形证据延期；下一步先
   三相同拍shadow更新已通过软件及板端状态门；下一步关闭`0x38`所代表的相位锁定ADC、
   overrun检测和硬件快速关断能力缺口；
5. 低压限流完成静态电角度零偏标定；
6. 先把 AS5600 用作 truth-only，完成 HFI 静态角度/极性对拍；
7. 证据通过后，才允许有感闭环或 HFI 功率注入。

H3动态真值另有独立`dengfoc_usb_time_sync_rx`目标。它保持GPIO12低和六路PWM输入态，
只运行M0 AS5600、GPIO23上升沿时间戳和RMT完整帧解码。当前已完成构建S2和USB-only
烧录/启动S4：经典ESP32的192-symbol RMT分配、M0真值链与诊断串口均在实板运行成功；
两板仍未接线，因此没有端到端帧、CRC或时间残差证据。RMT毛刺过滤阈值已按经典ESP32
80 MHz过滤时钟由5 us修正为1 us，仍远短于协议最短10 us有效脉冲。

任何 Host PASS 都不授权 GPIO12 拉高、LEDC 输出或电机运行。

## 6. ESP32 USB-only诊断目标

目标位于 `targets/esp32_dengfoc/`。日常从仓库根目录执行：

```powershell
.\targets\esp32_dengfoc\build.ps1

# 单独构建无主电PWM准备档；不会上传
.\targets\esp32_dengfoc\build.ps1 -Environment dengfoc_usb_pwm_prepare

# 单独构建中心对齐MCPWM时序诊断档；永久零输出且不会上传
.\targets\esp32_dengfoc\build.ps1 -Environment dengfoc_usb_mcpwm_timing
```

也可以在 VS Code 运行“FluxRT: DengFOC USB-only 角度诊断编译”或“FluxRT: DengFOC
USB-only PWM准备编译”或“FluxRT: DengFOC USB-only MCPWM时序编译”。这些任务都只编译，
不自动上传。当前固定配置为：

- `FLUXRT_DENGFOC_DIAGNOSTIC_ONLY=1`，若删除或改成0，编译直接失败；
- `FLUXRT_DENGFOC_ENABLE_AXIS1=0`，默认只访问参考工程实际启用的M0；
- 串口115200 bit/s，AS5600每5 ms采样、每50 ms输出一行；
- 启动时两路电流ADC各采1000点，只做零点统计；
- 功率事务`allow_actuation=0`且Arduino端二次许可为false；angle diagnostic不调用`prepare`，
  PWM prepare只允许配置LEDC并保持0 duty，两者都不能arm；
- MCPWM timing档只报告真实具备的`0x47`能力，保持发生器永久强制低，并因缺少`0x38`
  自动拒绝成为电流闭环执行器；
- 方向暂用`+1`、极对数7、零偏0且`calibrated=0`，只用于观察，不能直接视为可闭环配置。

预期串口格式：

```text
axis=0,valid=1,seq=123,mech=1.234000,multi=7.517185,vel=0.120000,elec=2.354000,fail=0
```

烧录前必须再次确认：只连接USB、12 V主电源断开、GPIO12高有效事实与板版次一致。上传和
实物回读属于S4，必须另记固件SHA-256、串口、角度连续性和结束安全状态。

后续采集统一使用仓库工具，而不是临时串口脚本：

```powershell
# 静态10秒，生成CSV及同名.summary.json
.\targets\esp32_dengfoc\capture.ps1 -Port COM13 -DurationSeconds 10 -Name static

# 单方向两圈验收：至少11.5 rad、反向小于0.8 rad、至少跨零2次
.\targets\esp32_dengfoc\capture.ps1 -Port COM13 -DurationSeconds 45 `
  -Name clockwise-2turn -RequireMotionRad 11.5 `
  -RequireOneDirectionRad 11.5 -MaxReverseRad 0.8 -RequireWraps 2
```

产物默认写入被Git忽略的`targets/esp32_dengfoc/captures/`。CSV保留逐帧主机时间、序列、
机械角、多圈角、速度、电角度和累计I2C失败；JSON给出跨度、标准差、RMS、正反累计、跨零和
验收结论。MATLAB/Python后续只消费这套格式。
