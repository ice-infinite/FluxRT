# EXP-B3 设计：MCU 注入 `Ls(I)` 辨识（实现前安全契约）

> 状态：**S0～S4.1 已通过；S5 禁止**
> 目的：判定 GBM2804H 在实际电流工作点的增量电感，解释 1 kHz LCR 小信号
> `1.114 mH` 与 Motor Profiler 约 `0.893 mH` 的差异。
> 边界：本文不是运行许可；没有通过下述 S0～S4 门之前，不得向电机注入辨识脉冲。

## 1. 为什么现在必须补这一项

rev7 把小信号 `Ls=1.114 mH` 直接代入控制器后：PC 正/反转空载能闭环，部分指标改善；
但轻载和死区补偿场景出现混合或退化结果。说明“小信号 LCR 值 = 0.8 A 工作点值”这个
假设尚未成立。继续猜 L 或直接烧录 rev7 都不能形成可靠证据。

目标输出不是一个自动写入固件的参数，而是三档增量电感：

```text
Ls(0.2 A), Ls(0.4 A), Ls(0.6 A)
```

0.8 A 是电机额定电流且接近当前启动电流，只能在前三档稳定、保护余量和温升均通过后
单独批准；不得默认自动跑到 0.8 A。

## 2. 架构职责

| 层 | 负责 | 禁止 |
|---|---|---|
| C 平台层 | PWM/ADC 同步、硬件 break、母线/电流/驱动故障、立即关断 | 计算/审批电机参数 |
| C 应用层 | 辨识状态机、Shell 命令、互锁、超时、原始样本环形缓冲 | 从 Shell 接受任意 duty/任意电压 |
| Rust 实时数学层 | 对固定长度样本做有界拟合；无分配、无日志、无阻塞 | 直接操作寄存器或解除保护 |
| PC 工具 | 重算、残差/重复性统计、生成未审批候选 | 自动置 approval 或改固件默认值 |

算法库已有的 HF/pulse injection 只能复用数学思想，**不能直接作为功率级入口**。唯一的
PWM 写入者仍是 STM32G431 平台适配层；辨识模式必须拥有独立的编译期和运行时门。

## 3. 状态机与互锁

```text
IDLE
  -> PREFLIGHT       检查 Identification 构建、母线 7~18 V、无 fault
  -> OFFSET_CAL      PWM 关闭，采电流零偏
  -> BIAS_SETTLE     受限 DC 偏置，单档由低到高，禁止自动跨档
  -> PULSE_PAIR      +ΔV/-ΔV 对称小扰动，固定拍数，原始 ADC 同步采样
  -> COOLDOWN        PWM 关闭，确认电流回零
  -> COMPLETE        只允许 dump/stop；下一档必须重新显式 arm

任意状态 -> ABORT   break/fault/过流/母线越界/超时/串口 abort -> 同一 ISR 内中性占空并关栅极
```

强制门：

- 仅专用 Identification 构建开放；Production 永久拒绝，Calibration 仍保持现有“禁止 arm”。
- `start` 必须带一次性确认参数，不能上电自动运行，也不能从 profile 自动触发。
- 软件过流门保持 `1.15 A` 或更低；硬件 break 优先级不变。
- 单次有功注入总时长先限制在 50 ms 内，全流程硬超时 500 ms。
- 每档结束必须 PWM off、电流回零、无 fault；不得在一个命令里连续跑三档。
- 电机空载、可自由轻微转动、无桨叶；用户必须能立即断开 12.3 V/2 A 电源。

### 3.1 S1 Host 状态机实现

当前纯状态机落在应用层：

- `applications/foc_lsi_identification.h`：版本化配置、输入、状态、abort 原因和输出请求；
- `applications/foc_lsi_identification.c`：无 RTOS/HAL/寄存器/堆分配的确定性状态迁移；
- `tests/host/test_foc_lsi_identification.c`：完整序列、所有可运行阶段人工 abort、fault、
  software trip、过流、母线越界、非法输入、总超时和手工复位测试。

`drive_request` 只是给后续平台适配器的请求，Host PASS 不表示 PWM 已经输出。所有非驱动
状态以及全部 abort 路径都强制 `force_safe_output=1`、`drive_request=OFF`、请求电流/扰动
电压清零。完成后保持 `COMPLETE`，不会自动换到 0.4 A/0.6 A；必须显式 reset 并再次带
确认字 start。

S2 已把文件纳入 `applications/SConscript` 的通配构建，并增加独立 Identification 档。
只有该档的编译期能力位允许 `request_start`；Diagnostic、Calibration、Production 即使
直接调用也会保持 `IDLE`。S2 当时没有 Shell/管理入口或 PWM/ADC 适配器，因此状态机虽在
目标对象中产生 1,350 B `.text`，但被最终链接器完整丢弃；下述 S4 才使只读/关断管理路径
进入最终 ELF。

### 3.2 S2 Target 构建门

构建档从三档扩展为四档：

- `Identification` 只开放 `FLUXRT_BUILD_CAP_LSI_IDENTIFICATION`；
- `Calibration` 与 `Identification` 都定义 `FLUXRT_MOTOR_ARM_DISABLED_BUILD`；
- 普通 `foc_start` 在应用层拒绝，`foc_platform_control_start()` 在平台层再次拒绝；
- Diagnostic/Calibration/Production 均不能启动辨识状态机。

四档 `RustOptLevel=s` 交叉构建结果：

| Profile | ROM | RAM | BIN SHA-256 |
|---|---:|---:|---|
| Diagnostic | 130,880 B | 23,120 B | `624D440BE3A0B93C4582D3FC23CDC4E28536E98049F6D1F97E74A7140B95AC53` |
| Calibration | 100,304 B | 13,072 B | `3BDE71847184AB596FC2ECFEF712E96FA05AA5B6F295C48CCBA96C3D1BCACE4F` |
| Identification | 97,292 B | 8,952 B | `DAC031A739F9356901F721B17C38CB771A55E13F40044902197FD243ED69AD59` |
| Production | 98,396 B | 8,952 B | `0D27BF43F80CEB4E9093099EB8ADDE21061474BA54DA57B692D81C3906F3E390` |

Identification 与 Production 的 `ADC1_2_IRQHandler` 都是 1,208 B，反汇编文本 SHA-256
完全相同：`4BEB9573D77E101D84C281C5A8353B9E3F76D548E96E80C9C3CCC8FD0E86FA62`。
这只证明 S2 修改没有改变该 ISR 的机器码；板端 WCET 仍必须在后续 S4/S5 单独实测。

Identification/Calibration 最终 ELF 均不包含正常 `foc_platform_control_start` 的可执行
路径；Production 仍包含 600 B 的正常实现。Identification 状态机没有调用入口，最终
链接符号数为 0。本轮没有烧录、串口或电机运行。

### 3.3 S4 无功率管理入口（PASS）

S4 新增三层而不接实时路径：

- `foc_lsi_management.c/.h`：固定 0.2 A 候选配置、状态快照、幂等 abort 和安全 reset；
- `foc_lsi_shell.c`：只在 Identification 编译 `foc_lsi_status/dump/abort/reset`；
- `test_foc_lsi_management.c`：覆盖 IDLE、活动态中止、强制复位和模拟重启清零。

**没有 `foc_lsi_start` 命令**。通用 `foc_start` 仍出现在 Shell 帮助中，但只链接
Identification 的编译期拒绝桩；平台层也再次返回 `FOC_STATUS_DISABLED`。四条 S4 命令中
`abort/reset` 先执行 `foc_platform_emergency_stop()`，随后才更新管理状态。

机器接口：

```text
FLSI_STATUS,<ver>,<result>,<authorized>,<state>,<abort>,<safe>,<drive>,<lsi_start>,<pwm>,<adc>,<abort_count>,<reset_count>,<start_count>,<start_consumed>
FLSI_CONFIG,<ver>,<sample_hz>,<offset_ticks>,<bias_ticks>,<pulse_ticks>,<pairs>,<cooldown_ticks>,<active_cap>,<total_cap>,<bias_mA>,<delta_mV>,<trip_mA>,<cool_mA>,<vmin_mV>,<vmax_mV>
FLSI_ABORT,...  # 后续字段与 STATUS 相同
FLSI_RESET,...  # 后续字段与 STATUS 相同
```

当前候选只用于 S4 dump：12 kHz、256 点偏置、10 ms bias settle、`4+4` 拍×6 对脉冲、
2 ms 回零、50 ms active cap、500 ms total cap、0.2 A / 0.4 V、1.15 A trip、7～18 V。
这些值不构成 S5 审批。

Host 全量回归通过，Identification 目标为 99,940 B ROM / 9,080 B RAM，BIN SHA-256
`AED75B3A447D5C44B03E0884718E815F0ADE2F9A65F4AB64ECDF0EE86875F7C3`。ELF 只导出上述
四个 `foc_lsi_*` Shell 符号，没有 `foc_lsi_start`；其它三个 Profile 中 `foc_lsi_*` 符号
计数仍为 0。Identification 与 Production 的 ADC ISR 反汇编保持逐行相同。

2026-09-27 首次板端预检时旧 CM4.4 Diagnostic 回读 Vbus 12.288 V，故停在烧录前。
用户关闭主电源后，母线自然下降至 0.877 V；随后 Identification 下载/verify/reset 成功。
四条管理命令、通用 `foc_start` 拒绝、软件复位和明确 `-hardRst` 硬件复位均通过。最终
管理计数恢复 0/0、state/drive=`IDLE/OFF`、duty 0、step/error/miss 0、Vbus 0.193 V，
全过程没有电机运行或功率注入。

### 3.4 S4.1 原始证据窗与 applied-voltage 纯算法门（PASS）

本阶段仍不接 PWM/ADC，不增加 start，只补齐后续适配器必须依赖的两个纯组件：

- `foc/include/foc_lsi_raw_capture.h` + `foc/runtime/foc_lsi_raw_capture.c`：固定 256 拍、
  24 B/拍、总计 6 KiB 的原始窗；
  每拍只保存同步 U/V 电流 ADC、**同一注入序列**的母线 ADC、当拍实际生效的三相 CCR、
  周期和状态标志。ISR 写入为 O(1)，线程在停止后才能读；满窗锁存 `OVERFLOW`，不覆盖旧
  样本，未来平台适配器必须在同一控制拍触发 abort；
- `foc-algorithm/src/inverter.rs::reconstruct_applied_phase_voltage_v()`：复用既有死区、器件
  压降和电流极性模型，在去除桥臂损失后再消除浮动中性点共模，返回实际平均三相
  line-to-neutral 电压。非法 duty/Vbus/电流/参数直接 `None`，不钳位伪造证据。

这里明确禁止把 `g_foc_diagnostics.bus_voltage_raw` 的慢速旧值直接写进辨识窗。后续平台
适配器必须为 Identification 建立与电流同序列的 Vbus 采样，并在写新 CCR 之前保存当前
实际生效 CCR；否则 applied-voltage 输入虽然字段齐全，时间上仍是错拍的。

Host 全量回归通过：Rust `75+32+18+17`、C Host `10/10`。四档目标重新构建后 ROM/RAM
仍为 130,880/23,120、100,304/13,072、99,940/9,080、98,396/8,952 B。新增 C 缓冲对象
已交叉编译，`record_isr` 对象代码 72 B/31 条指令，但尚未实例化或接入最终 ELF；四档最终
ELF 均无其符号，也无 applied-voltage 新符号。Identification 与 Production 的 ADC ISR
均为 404 条语义指令、逐条一致，因此本阶段没有改变当前板端实时路径。

当前新 Identification BIN 为 `BAA538DD...98D853`，只属于 S2 本地构建，**未烧录**；
板上仍是 S4 的 `AED75B3A...875F7C3`。本阶段没有串口、下载、上电或电机运行。

### 3.5 S4.2 同序列 Vbus 无功率子门（PASS）

Identification 已把 ADC1 injected sequence 扩为两 rank：rank1 继续采 U 相电流，rank2 以
47.5 cycles 采 PA0/Vbus；ADC2 rank1 的 V 相电流保持不变。ADC1 JEOS 在 rank2 完成后才
进入 ISR，因此 `JDR1/JDR2` 是同序列证据。只读 `foc_lsi_hw_status` 报告配置位、有效位、
样本计数、Vbus raw 和 DWT 监测时序，不 arm、不写 CCR。

全回归 C Host 已增至 11/11。最终 Identification 为 100,320 B ROM / 9,096 B RAM，BIN
`34A45DDC...63C967`、HEX `F01CE41B...417DC5`；`foc_lsi_start` / request 符号仍为 0。
主电源关闭条件下下载/校验成功，同步样本计数持续按 12 kHz 增长，Vbus raw 0～3 counts，
未 arm monitor ISR max 333 cycles；普通 `foc_start 582` 被编译门拒绝，状态保持 IDLE/OFF。
完整证据见
[`2026-09-27-EXP-B3-S4.2-同步Vbus无功率门.md`](../performance/2026-09-27-EXP-B3-S4.2-同步Vbus无功率门.md)。

本子门没有接 active CCR/ARR，也没有实例化 6 KiB 固定窗；下一步仍是无功率适配器骨架，
不是 S5 许可。

### 3.6 S4.3 固定原始窗目标板无功率门（PASS）

固定窗和契约已迁入 `foc/runtime/`，由 Identification ISR 在任何新 CCR 写入之前读取
TIM1 CCR1/2/3 与 ARR，并与同序列 U/V 电流、Vbus 和故障标志组成 24 B 样本。固定
`capture service` 负责连续性/量程/状态检查和 256 点存储；契约或存储错误锁存平台错误位，
随后走 C 侧快速关断。采集中重复 arm 明确拒绝且不能清掉前一拍状态，Host 已覆盖该边界。

无功率 Shell 只暴露 `foc_lsi_capture_arm/status/dump`。arm 要求 gate safe、同步采样有效、
无输出/故障且 Vbus raw `<=80`；这些命令不能提交 duty、使能 MOE 或启动电机。最终板端
256 点 sequence/tick 连续，U raw 1946～1957、V raw 1930～1935、Vbus 0～6、CCR 三相全 0、
ARR 全 7083、flags 全为 ADC_VALID；overflow/contract/storage error 均为 0。采集路径 monitor
ISR max 1,201 cycles，普通 `foc_start 582` 继续拒绝。完整证据见
[`2026-09-27-EXP-B3-S4.3-固定原始窗无功率门.md`](../performance/2026-09-27-EXP-B3-S4.3-固定原始窗无功率门.md)。

本门只在输出关闭条件下验证寄存器快照和固定窗，不能证明输出开启时 CCR 的 active/preload
时序。下一步先完成专用 PWM 注入适配器的静态/Host/S2 设计门，并使窗满也先快速关断；
在单独安全评审前仍不增加 `foc_lsi_start`，不进入 S5。

### 3.7 S4.4 Rust 注入计划静态门（PASS：S1/S2）

新增 `foc-control::plan_lsi_actuation()`：按 4.9666667 Ω/相筛查 Rs 计算
`Rs*I_bias+V_perturb`，再映射为三相和为零、公共模 0.5 的 duty。配置冻结为 12 kHz、
1 control tick preload、0.2 A、±0.4 V、7～18 V、1.15 A 和 3%～97%；无效枚举、方向
错配、NaN、capture 未就绪/已满、fault、电流/母线/duty 越界均先返回安全输出，不钳位
继续。新增 ABI V18 的 48/52/40 B 结构和 `source_tick→expected_active_tick` 账本。

Identification 初始化只做 OFF/BIAS 纯计算自检，不写 CCR、CCER、MOE 或栅极。全回归、
四档构建和链接隔离通过，计划符号只在 Identification，ADC ISR 指令数仍为
539/436/548/404。本轮未烧录；板上继续运行 S4.3 固件且主电源关闭。完整证据见
[`2026-09-27-EXP-B3-S4.4-Rust注入计划静态门.md`](../performance/2026-09-27-EXP-B3-S4.4-Rust注入计划静态门.md)。

### 3.8 S4.5 C 执行契约静态门（PASS：S1/S2）

新增 `foc_lsi_actuation_executor`，把状态请求/capture 门、同步 raw、板级 offset/counts 标定、
Rust plan、duty→CCR 四舍五入和下一拍账本收进无 HAL 的 C runtime。账本必须同时匹配
expected tick、U/V/W compare、`DRIVE_ACTIVE` 和正负 pulse flag；任何 capture/raw/fault/NaN/
越界/账本错误都返回 SAFE 并清 pending。目标 ISR 同时补齐固定窗 256 点正常完成也立即
快速关断，只有 contract/storage 错误额外锁存 `LSI_CAPTURE_ERROR`。

启动期使用实际板级换算常量完成一组 BIAS→OFF 合成自检，但不写 TIM1。全回归、Host
13/13 和四档目标构建通过；executor/plan 只在 Identification，ADC ISR 550 条指令（相对
S4.4 +2，仅来自窗满关断分支）。本轮未烧录，`WRITE_PRELOAD` 仍是未施加决策，实际 CCR
writer、私有 arm token、gate/MOE 和 `foc_lsi_start` 都不存在。完整证据见
[`2026-09-27-EXP-B3-S4.5-C执行契约静态门.md`](../performance/2026-09-27-EXP-B3-S4.5-C执行契约静态门.md)。

下一门 S4.6 在独立复核后实现私有会话协调器和物理 TIM1 preload sink；仍不开放 Shell
start、不烧录、不使能栅极。

### 3.9 S4.6 私有会话与 TIM1 preload sink（PASS：S1/S2）

在 `foc/platform/stm32g431/` 新增不导出的 session/permit 与 register sink。permit 绑定
generation、source/expected tick、U/V/W compare 和 ARR，只允许消费一次；旧许可、输出篡改、
tick/duty/compare 不一致均拒绝。STM32 wrapper 在写前重新检查 TIM1 时钟、OC preload 位、
ARR、gate、CCER、MOE、Break/driver fault 和 software trip。

启动自检现在会在所有输出关闭时把 BIAS compare 真实写入 CCR1/2/3、读回，用读回值完成
下一拍 ledger，再清零。它没有 active/powered 模式，不进入 ADC ISR，也没有 start/gate API。
全回归 Host 14/14、四档 S2 和链接隔离通过；Identification ROM/RAM 109,588/15,496 B，
ADC ISR 保持 550 条。本轮未烧录，完整证据见
[`2026-09-27-EXP-B3-S4.6-私有会话与TIM1预装载静态门.md`](../performance/2026-09-27-EXP-B3-S4.6-私有会话与TIM1预装载静态门.md)。

### 3.10 S4.7～S4.8 板端 preload 与 bounded active permit（PASS）

S4.7 在主功率断开时验证真实 G431 的 outputs-disabled preload 写/读/ledger/清零和 bit 23；
S4.8 将私有 permit 扩展为 generation/mode/sequence/tick/CCR/ARR 逐拍绑定，硬限制 600 写、
6000 总拍，并冻结首笔 outputs-off、后续 outputs-on 的硬件事实门。目标启动只执行首笔 off
自检并置 bit 24；固定窗仍为 256 点连续、CCR=0，普通 start 继续拒绝。

详细证据见
[`S4.7 报告`](../performance/2026-09-27-EXP-B3-S4.7-TIM1预装载无功率板端门.md)和
[`S4.8 报告`](../performance/2026-09-27-EXP-B3-S4.8-bounded-active-session静态与无功率门.md)。

### 3.11 S4.9 一次性会话与 ADC ISR（PASS：S1/S2/无功率 S4）

应用管理层 v2 持有一次性 token 和共享状态机；平台新增唯一固定入口
`foc_lsi_start LSI1`。live preflight 在消费 token 前检查 7～18 V 同步 Vbus、0.05 A 近零
电流、输出全关、能力/故障位和 capture 状态。ADC ISR 执行 256 拍 offset、120 拍 bias、
6 对 `4+4` 拍脉冲和 24 拍 cooldown；active raw 窗不记录 offset。首笔 preload 以 TRGO
屏蔽的 UG 装载后才开 gate，后续逐拍 ledger；任一错误同拍快速关断。

全回归和 Host 14/14 通过，start/active symbols 只在 Identification。无主功率板端：错误
口令拒绝；正确口令因 Vbus=0 返回 NOT_CONFIGURED，token 保持 0/0；normal start 拒绝；
256 点 sequence/tick 连续、Vbus 0～7、CCR=0、无错误，monitor max 1,606 cycles。该值不是
active WCET，S5.0 仍需单次带电实测。详细证据见
[`S4.9 报告`](../performance/2026-09-27-EXP-B3-S4.9-一次性会话与ISR无功率门.md)。

### 3.12 S5.0 单次 0.2 A 实机筛查（执行 PASS，参数 REJECT）

用户重新确认 12.3 V/2 A、空载可自由旋转且可立即断电后，只执行一次固定 start。状态机
449 拍后自动 COMPLETE，raw 192 点连续，最大名义相电流 0.2155 A，fault/miss/capture
错误均为 0；完整 ISR WCET 11,078/12,750 cycles，最终 OFF、duty 0、token 1/1。

applied-voltage 固定 Rs 拟合约 1.740 mH，但六个脉冲对极差/均值 15.68%，与 LCR 折算
1.113 mH 相差 56.29%。因此本门只证明 bounded 硬件链安全执行，参数筛查拒绝升级；详细
证据见[`S5.0 报告`](../performance/2026-09-27-EXP-B3-S5.0-0.2A一次性实机筛查.md)。

### 3.13 S5.1 模型比例与实测噪声审查（PASS：S0/S3；旧序列降级）

ST MCSDK 参考工程中的 0.33 Ω 分流、1.53 增益、0.0625 分压和 525/550 ns 死区与
FluxRT 名义模型一致；4095/4096 的比例差只有约 0.0244%，按官方比例重算仍为
1.740 mH。Rs/L 联合拟合为 4.79 Ω/1.726 mH，固定器件压降扫描只会让残差变差。因此
换算常数不能解释与 LCR 相差 56.29%，参数仍拒绝。

S5.0 cooldown U 相标准差为 1.761 count，旧 PC 模型的 0.5 count 假设被实测证据否定。
1.76 count 下旧 4 拍/极性×6 对最低分类率 93.33%，Host 门降级为 FAIL；离线 6 拍/极性
×6 对候选在 9 工况×30 次中分类率均 100%，最大极差/均值 4.44%。因此 S5.2 只在代码中
把 bias 120→24 拍、pulse 4→6 拍，pairs 保持 6；raw 窗 192→120 点，有功时间 14→8 ms。
详细证据见
[`S5.1 报告`](../performance/2026-09-27-EXP-B3-S5.1-模型比例与实测噪声审查.md)。

## 4. 激励和计算口径

静止时 BEMF≈0，单轴等效模型为：

```text
v = R*i + L*di/dt
i(t) = V/R * (1 - exp(-t*R/L))
```

使用正负对称小扰动围绕偏置电流采样，优先用整段指数拟合 `R/L`，并用
`L=(v-Ri)/(di/dt)` 作交叉检查。正负样本平均可降低 ADC 零偏、死区和器件压降的一阶
影响；实际端电压必须由母线采样、占空比和已验证逆变器模型重构，不能把 duty 当伏特。

当前 `L/R≈0.22 ms`，12 kHz 只有约 2.7 个样本/时间常数，因此实现前必须在 PC 中加入
12 kHz 采样、ADC 量化、零偏和死区，证明拟合可辨识；若不可辨识，选择重复脉冲平均或
专用更高采样率构建，不能通过提高电压硬凑信噪比。

## 5. 证据格式

每次只产生原始证据和筛查报告：

```text
profiles/identification/evidence/lsi-YYYYMMDD/
  run_<current>_<index>.csv       # tick, phase ADC, bus ADC, duty, flags
  run_<current>_<index>.log       # 完整命令/状态/故障/停机回显
  run_<current>_<index>.json      # 固件/profile/CRC/HEX hash/条件/原始文件 hash
  repeatability_summary.json      # L、残差、温升、极差/均值和接受/拒绝原因
```

每档至少三次；只有三次都无 fault、拟合残差过门且 `(max-min)/mean <10%`，该档才可作为
筛查结果。报告不得携带 approval，候选工具仍需另行人工评审。

## 6. 实施门

| 门 | 必须证明 | 当前 |
|---|---|---|
| S0 架构 | 单一 PWM 所有者、状态机、接口和失败语义评审 | **PASS**：状态机归应用层，平台保持唯一 PWM 所有者 |
| S1 Host | 状态迁移、任意中止均关断、边界/超时/CRC、拟合合成波形 | **PASS**：完整 Host CTest 14/14（含 C executor、private preload sink、固定原始窗、样本契约和 capture service）、Rust 75+38+21+17；重复 arm/permit 不破坏安全状态；状态机/管理恢复/构建授权/normal-arm 双层拒绝均通过；候选 CRC 140 tests、拟合 3/3 和 PC plant 门沿用现有证据 |
| S3 PC plant | 12 kHz+量化+死区下能区分 0.893/1.058/1.114 mH | **旧 4×6 方案 FAIL；新 6×6 候选 PASS**：S5.0 实测 1.76 count 噪声代理下旧方案最低分类率 93.33%；新方案 9 工况×30 次均 100%，极差/均值最高 4.44%。仍无完整噪声频谱 |
| S2 Target | 专用构建 size/ISR 预算，非 Identification 拒绝、normal-arm 双层拒绝 | **PASS（静态）**：四档交叉构建通过；S4.9 Identification ROM/RAM 113,112/15,552 B；executor/plan/preload/start 仅该档链接；其它三档无 start，normal arm 禁用门通过 |
| S4 干跑 | 不 arm 的命令/状态/dump、断电与复位恢复 | **PASS**：Vbus 0.877 V 后下载/校验；四命令与 arm 拒绝通过；软件/硬件复位均恢复计数 0/0、IDLE/OFF，最终 Vbus 0.193 V |
| S4.1 数据链静态门 | 固定原始窗、实际施加电压纯算法、四档隔离、ISR 不变 | **PASS（Host/S2 静态）**：256×24 B 契约和溢出锁存通过；重构复用统一逆变器模型；四档构建通过，新增模块未链接，ADC ISR 404 条语义指令一致；未烧录 |
| S4.2 同步 Vbus 子门 | 同序列 Vbus、只读状态、无功率 WCET、启动继续拒绝 | **PASS（S4 无功率）**：ADC1 rank1/2=U 电流/Vbus；计数连续，raw 0～3；monitor ISR max 333 cycles；normal start 拒绝，输出关闭；active CCR/原始窗目标实例仍未完成 |
| S4.3 固定原始窗子门 | 目标 ISR 同拍 raw 窗、契约/错误关断、四档隔离、无功率 WCET | **PASS（S4 无功率）**：256 点 sequence/tick 连续，Vbus 0～6、CCR=0、ARR=7083、无错误；采集 ISR max 1,201 cycles；capture 符号只在 Identification；normal start 拒绝。powered active/preload 语义和 PWM adapter 仍未完成 |
| S4.4 Rust 注入计划静态门 | OFF/BIAS/±PULSE 到 duty 的纯计算、ABI、包络和 preload 账本 | **PASS（S1/S2）**：ABI V18；12 kHz/1 tick、0.2 A、±0.4 V、7～18 V、1.15 A、3%～97% 冻结；Host fault/满窗/NaN/越界均安全；计划符号只在 Identification，ISR 指令数不变。没有 CCR 写入器/start，未烧录 |
| S4.5 C 执行契约静态门 | raw→SI、Rust plan、CCR 量化、下一拍 tick/CCR/极性账本、满窗快速关断 | **PASS（S1/S2）**：Host 13/13、全回归、四档构建/链接隔离；Identification ISR 550 条，executor 不在 ISR。只返回未施加的 `WRITE_PRELOAD`，没有物理 CCR writer/start，未烧录 |
| S4.6 私有 preload 静态门 | one-shot permit、真实 TIM1 CCR 写/读/清、最终硬件复核、链接隔离 | **PASS（S1/S2）**：Host 14/14；启动路径只允许 outputs-disabled validation write，preload 符号只在 Identification，ISR 550 条不变；无 active session/start，未烧录 |
| S4.7～S4.8 preload/active permit 无功率门 | 真实 startup preload、bounded permit、首拍 off/后续 on 门、固定窗不退化 | **PASS（无功率 S4）**：bits 23/24、CCR 清零、256 点连续和 normal-start 拒绝通过；未执行 active output |
| S4.9 一次性 ISR 会话无功率门 | 固定 start、live preflight、token、状态机/ISR/fail-close、断电拒绝 | **PASS（S1/S2/无功率 S4）**：错误/正确口令拒绝且 token 0/0；256 点 CCR=0；Identification ISR 静态 1,053 条；active WCET 未测 |
| S5.0 实机 | 重新确认后只执行一次 0.2 A，取得 active raw/WCET，再决定是否拟合 | **硬件执行 PASS、参数 REJECT**：449 拍 COMPLETE，192 点连续，0 fault/miss，WCET 11,078/12,750；1.740 mH 与 LCR 口径冲突，未批准 |
| S5.1 离线审查 | 官方比例/死区/压降/Rs-L 灵敏度，实测噪声回灌与序列重设计 | **PASS（只到 S0/S3）**：官方常数不能解释冲突；旧 4×6 门降级；6×6 候选通过。硬件 repeat 仍 `not-authorized` |
| S5.2 目标整改门 | 24 bias + 6×6 pulse + 24 cooldown，Host/四档 S2 | **PASS（S1/S2）**：全回归和四档构建通过，代码为 120 点窗/377 拍会话；后续无功率实机证据归 S5.3 |
| S5.3 无功率板端门 | 新固件身份/配置、拒绝/token、独立安全窗和最终关断 | **PASS（S4 无功率）**：配置 24/6/6/24；wrong/low-bus/normal start 均拒绝，token 0/0；256 点连续、Vbus 0～5、CCR=0、0 capture error；未运行 active 会话 |
| S5.4 锁转子 6×6 实机门 | 固定转子后一次 0.2 A 会话、120 点 active raw/WCET 与离线模型复核 | **硬件执行/窗内重复性 PASS，参数 REJECT**：377 拍 COMPLETE、120 点连续、WCET 11,053/12,750；六对极差/均值 6.73%，但固定 Rs 动态 L=1.950 mH、相对 LCR +75.15%；再次运行未授权 |
| S5.5A 锁转子 LCR 同角度门 | 无主电源、同角度三轮正反序 Q/R/L，区分随机接触与稳定线对差异 | **同线对重复性 PASS，参数 REJECT**：WV/VU/WU 极差/均值最大 0.375%；VU=2.8207 mH、WV/WU≈2.13 mH，跨线对 29.12%。只支持角度/凸极或相别效应假设，尚未证明角度依赖 |
| S5.5B LCR 手动角度粗扫 | 多方向手动转动，仅记录观察极值，判断是否值得继续密集逐角度测量 | **定性筛查 PASS，量化延期**：用户报告约1.96～2.60 mH；转子位置影响有工程量级，不再要求手持逐5°测量。无角度/线对映射，不能生成 `Ld/Lq` |
| S5.5C 动态真值与参数包络 | 比例区分、0.98～1.4103 mH Rust包络和最小独立真值契约 | **PC比例审计/功能包络 PASS，参数 REJECT**：±5%和0.1～3倍相对比例均不能把1.9503 mH拉入包络；10个Rust场景全部完成。下一次powered门缺独立动态电流和差分PWM电压 |

## 7. 当前决定

- rev7 保持未审批、未集成、未烧录。
- 纯 Host 状态机、S2、无功率 S4～S4.9、powered S5.0 和锁转子 S5.4 均有证据；两次
  powered 门都证明执行链
  安全，但没有证明参数值可批准。旧 0.5 count PC 可辨识性证据已被 S5.1 降级。
- S5.4 已排除“自由转子未固定”这一直接缺口；6×6 窗内极差/均值改善到 6.73%，但动态
  1.950 mH 与 LCR 冲突扩大到 75.15%。名义比例、Rs/L 联合、常量桥压降和理想中心对齐
  子周期 PWM 均不能解释该冲突。
- S5.5A 已在用户确认主电源未连接、转子保持角度 A 时完成三轮 LCR 重复；主电源物理断开
  现已确认。S5.5B 随后的手动多方向粗扫观察到约1.96～2.60 mH，足以定性确认位置敏感，
  精细角度图延期。S5.5C 已进一步排除简单乘法比例作为单一解释，并证明现有控制骨架在
  0.98～1.41 mH的PC空载包络内不失效。A26下一次powered门等待独立动态电流/差分PWM
  电压仪器；期间转入A22/A23纯软件质量门和ABI，revision/approval不变。
- 如果 12 kHz 对三种候选电感不可分辨，优先改采样方案或使用带 DC bias 的 LCR 仪器，
  不增加注入幅值突破现有电流保护。

PC 证据见 `profiles/identification/evidence/lsi-20260927/`。特别注意：command voltage
未扣死区时仿真产生约 −1.75%～−2.77% 系统误差，因此 applied-voltage 重构是后续实现
的硬门，不是可选优化。
