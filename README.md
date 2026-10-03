# FluxRT：RT-Thread + Rust FOC 实时电机控制平台

这是一个面向后续 FOC 开发的 RT-Thread 工程。当前验证目标是 `NUCLEO-G431RB + X-NUCLEO-IHM16M1 + GBM2804H-100T`；代码按 C 硬件平台层、Rust 控制层和纯算法层分离，便于后续替换国产 MCU。

## 当前状态

- RT-Thread + STM32G4 HAL 构建骨架。
- 主时钟：NUCLEO-G431RB 板载 X3 24 MHz HSE 经 PLL 到 170 MHz（`M=6, N=85, R=2`）。
- 低速时钟：外部 32.768 kHz LSE，已启用并预选为 RTC 时钟源。
- HSE 或 LSE 起振失败时进入 `Error_Handler()`，不会静默换用另一套控制时基。
- 临时控制台：LPUART1，PA2/TX、PA3/RX。
- FOC 控制核心已切换为 Rust `no_std` 静态库，RT-Thread/HAL/ISR 保留在 C。
- 已迁入完整 Rust 算法库，并通过独立 C ABI bridge 调用 ST 参考速度环/电流环。
- 已加入逆变器 + PMSM plant + 假传感器的 PC 闭环仿真和可替换硬件 ports。
- 已接入可配置的 STM32G4 混合快环数学后端：`sin/cos`、`atan2` 使用 CORDIC，矢量模长使用 FPU `VSQRT.F32`；硬件调用失败时自动用 CPU 重算。
- RT-Thread RTC 设备驱动暂未启用；当前只完成 LSE 和 RTC 时钟源配置。
- TIM1 已按中心对齐 12 kHz 运行，ADC1/ADC2 由 TIM1 TRGO 同步注入采样；PA11/Break2、1.15 A 软件过流和驱动器故障关断已接入。
- 默认上电保持 CH1～CH3、MOE 和 PB13/PB14/PB15 关闭；只有串口显式执行 `foc_start` 才会 arm，`foc_stop` 立即关断。
- Rust 已运行对齐、强制角度升速、Id/Iq PI、圆限幅、SVPWM 和 SMO。默认 `SMO=启用`、`closed_loop=0`；闭环只能在停机后通过 Shell 临时打开。
- 已建立完整 ISR 同拍 WCET，并完成过 Rust `3/s/z` 实机矩阵；当前四档统一使用
  Rust `s`。2026-09-28 审计整改后的 Diagnostic 固件已重新烧录，trace-off 三次
  5 s 开环最坏为 9,763 cycles（物理周期 68.92%），0 error/miss/fault；trace-on、
  闭环和故障注入仍需分别复验。
- P0.1 审计整改基线已收口：全量 Host/Rust/Python 回归和四档目标构建通过，构建脚本
  固定 `SOURCE_DATE_EPOCH` 后完整重生成哈希可复现。本次收口没有重新烧录或运行电机，
  P0.2～P0.5 的目标安全注入、耐久和实时性复验仍待执行。
- P1.1～P1.3 与 P3.1 软件基础已完成：产品公共契约V1/bridge当前V21、固定容量命令仲裁与来源权限、
  以及Rust PC + MATLAB共享的scenario/profile/trace/comparison契约已经建立；524 rpm场景的
  完整D0身份和七字段D1生命周期严格对拍通过。P3.1又建立可恢复配置、CRC、迁移、掉电恢复
  和回滚纯逻辑。P3.2A建立token化设备端参数事务、分组patch、应用等级、
  volatile apply/rollback和commit回读确认；P2.5A2已将配置schema/管理ABI升级为V4：七组配置、
  768 B当前双槽兼容V0～V3旧记录、4 KiB上下文，384 B `ExternalIoConfig`包含88 B三输入标定；
  V3迁移保留transport/link但强制关闭缺少标定的simple input；
  Platform→Axis→Management执行器和Fake Flash故障门。它们仍未接RT-Thread服务或真实Flash，
  既有V19快环和默认运行行为未改。P4.1A 已新增纯 Rust 统一反馈与标定核心；P4.1B1 又冻结
  独立反馈管理ABI，补齐 Sensorless/ABZ/Hall 目标中立C adapter，并把批准后的标定更新接到
  既有配置事务的 pending Axis/Calibration 组。P4.1B2 已加入 Rust PC 与 MATLAB 两套独立编码器/Hall模型，量化、整拍
  延迟、丢脉冲、Index缺失、Hall错序、回退/恢复和feature-off共5场景80行轨迹通过
  D0/D2/D3/D4对拍。P4.1C又在应用层加入默认关闭的单owner管理服务，完成超时、取消/回滚、
  批准后事务暂存和commit/readback确认门；它没有注册Shell/RT线程，没有自动apply/写Flash，
  也没有接默认ISR或真实传感器。P4.2A/B/C现已建立默认关闭的通用参考规划与级联控制：
  覆盖Torque/Velocity/Position、ramp/filter/trapezoidal trajectory、Position P、Velocity PI、
  Torque→Iq、前馈后的最终限幅、anti-windup和实测Iq切换预置；P4.2C又补齐schema V2
  配置映射、独立motion V1 ABI、C适配器和Diagnostic候选。P4.2D又完成Rust PC与MATLAB独立
  motion模型：8类场景、192行trace的D0～D4/feature-off严格对拍通过，15个连续通道最大
  绝对差均为0；修复后全回归和默认关闭四档构建/map也通过。Cargo/Kconfig仍默认关闭，既有
  ISR未改。P4.2E0又新增默认关闭、单执行上下文的motion owner。P4.2E1现已落位ADC ISR
  单写者dispatcher、136 B固定快照/urgent请求和Rust combined realtime单入口，并修复动态
  `f32::clamp`误带约20 KiB浮点格式化链的问题。P4.2E1B现已增加Diagnostic-only、不能arm的
  无功率活调用点、mode 0/1增量测量和configure/combined/commit六点故障注入；candidate `s`
  为130,340 B ROM/20,664 B RAM。无电机电源板测已完成：mode 0/1各1,200拍，motion完整ISR
  最坏12,157/12,750 cycles、0 miss，六个注入点全部fail-closed且duty为0。候选随后关闭并
  烧回默认Diagnostic（110,660 B/22,472 B）。P4.2E2先完成逐模式S3软件门：在同一契约中
  增加Torque/Velocity/Position三个动态机械plant场景，Rust/MATLAB共11场景31,392行严格
  对拍及模式阈值全部通过。Torque随后又增加固定0.002 N·m/0.08 A/100 ms状态机、专用
  candidate能力档和唯一arm入口；板端低母线拒绝、mode 0/1无功率WCET及六点注入通过，
  mode 1最坏12,233/12,750 cycles、0 miss。源码已恢复candidate默认关闭，板上暂留输出关闭的
  候选等待一次powered S5；这些证据仍不等于真实Torque、观察器获取或仿真实机一致。
- 已建立 Diagnostic / Calibration / Identification / Production 四个隔离构建档。Identification 的 EXP-B3 S4～S5.4硬件链通过；S5.5A/B确认LCR线对/位置差异，S5.5C又证明简单乘法比例不能把动态1.9503 mH拉入0.98～1.4103 mH诊断包络。Rust在该包络10个PC闭环工况全部完成，但现有仪器缺独立动态电流/差分PWM电压，精确 `Ld/Lq`、参数更新及再次powered run仍未授权。
- G431RB 已建立独立于上述主构建档的五种互斥产品组合：Basic Drive、Advanced Lab、
  Motion Lab、Power Lab、Connected Lab。只有 Basic 可配非 Diagnostic；各 Lab 仍默认关闭、
  不自动启用运行时能力。全功能同镜像明确不受支持，非法组合由 Kconfig、CMake 和 C 头
  三层拒绝。
- A19 已在目标板完成 Calibration 首次下载、启动和 `foc_start` 拒绝门，并取得停机 256 拍连续原始码；当前缺可信万用表多点参考，仍未形成电压标定参数。
- A17 已接通 PC0/PC3/PC1 三路 BEMF ADC 原始码的 12 kHz、256 拍只读固定窗；无功率实机采集通过，但尚未完成电压标定、动态相序、示波器对拍或观察器接入。
- A22.3 已完成 adapter→V19 的硬件中立组装门：名义/无模型与质量状态联合校验、回绕 age、
  失败88 B清零和128拍legacy等价通过；它尚未接入目标ISR，Measured仍关闭。A21.1 已用
  严格完整-token十进制解析替换应用层36处libc数字转换，Diagnostic ROM由129,952 B降至
  106,416 B，回收23,536 B、剩余24,656 B；V19 + `z` 的板端完整ISR WCET仍待复测。
- 已完成 CORDIC `sin/cos`、`atan2` 停机分项基准；专用 `foc_math_bench` 由默认关闭的 Kconfig 控制，不占用日常 Diagnostic/Production Flash。
- 已完成“6 NOP + 单次 RRDY 检查”候选 A/B，并补齐实时健康计数与单次未就绪故障注入：两类注入均恰好回退/恢复 1 次、后续约 6.36 万拍成功、0 miss/error/fault；因收益仅 69 cycles 且依赖时钟/工具链，候选仍默认关闭，Production 保留有界轮询。
- 已实现可移植 Rust CPU 快速 `sin/cos`/`atan2` 候选：Host 密集误差回归、PC 闭环仿真和实机同镜像 A/B 通过；三轮最坏完整 ISR 比 CORDIC 少 204 cycles，但因实机仍为开环且没有编码器真值，G431 默认继续使用 CORDIC/FPU。
- 闭环重复性仍未通过：同条件复测存在 `OBSERVER_LOST`，因此该问题与构建优化分开处理，默认闭环继续关闭。

> 当前只证明低压、限流、空载短时运行、构建档 WCET 和“单次 CORDIC 未就绪”故障恢复；没有编码器/测速仪独立真值，也未完成其它保护故障、长时间和多工况验证。上电默认仍为 `closedloop 0`，Production 只是候选构建档，不得误读为量产通过。

## 初始验证硬件

- 控制板：`NUCLEO-G431RB`，板载 `STM32G431RBT6`。
- 功率板：`X-NUCLEO-IHM16M1`，计划采用三分流采样配置。
- 电机：`GIMBAL GBM2804H-100T`。
- 对应 ST 套件：`P-NUCLEO-IHM03`。

当前保留参考工程的电机参数、连续时间 PI、圆限幅和无感启动时序；PWM/电流环按实机 WCET 从参考的 30 kHz 降到 12 kHz。电流符号和短时观察器接管已由实机验证；独立真值校准、故障注入和长时间多工况验证仍待完成。

## 架构

```text
applications/ + RT-Thread     C：启动、线程、通信和管理
        ↓ 固定 C ABI
rust/crates/foc-rt-bridge/    Rust：控制状态机、参数验证、快环入口
        ↓ Rust API
rust/crates/foc-control/      Rust：参考规划、速度/电流环、启动、观测器和硬件 ports
        ↓ 纯算法
rust/crates/foc-algorithm/    Rust：纯算法库（no_std、无 HAL）
        ↑ 反馈 / ↓ 占空比
foc/platform/stm32g431/       C：ADC/PWM/DMA/保护与硬件关断
        ↓
board/ + STM32 HAL            C：时钟、中断和芯片启动
```

以后替换国产 MCU 时，应新增 `foc/platform/<new_mcu>/` 和新的 `board/`，保持 Rust 控制与算法 crate 不依赖厂商头文件。

## 构建

首次构建会拉取 STM32G4 CMSIS/HAL 包：

```powershell
cd E:\File\RT-Thread\projects\FluxRT
.\build.ps1 -UpdatePackages -Regenerate
```

普通增量构建：

```powershell
.\build.ps1 -BuildOnly
```

Production 候选构建：

```powershell
.\build.ps1 -Regenerate -Profile Production -RustOptLevel s
```

Calibration 停机采集构建（禁止电机 arm）：

```powershell
.\build.ps1 -Regenerate -Profile Calibration -RustOptLevel s
```

Identification 参数辨识构建（S5.5C离线真值与包络门已完成；下一次powered参数门等待独立测量能力并须重新人工确认）：

```powershell
.\build.ps1 -Regenerate -Profile Identification -RustOptLevel s
```

Diagnostic、Calibration、Identification、Production 输出分别位于 `cmake-build`、
`cmake-build-calibration`、`cmake-build-identification`、`cmake-build-production`，Rust 静态库也按档位和优化等级
隔离。A21.1 回收浮点文本解析链、A21.2 选定 `s` 后，A21.3 对 V19 常态路径做了
数值等价裁剪。A21.3 历史基线为 9,627 cycles；加入功率安全事务和同步 Vbus 后，
先升至 10,344 cycles，经保持语义不变的热路径内联后，当前 Diagnostic + `s`
三次 trace-off 最坏为 9,763 cycles，占 12 kHz 物理周期 68.92%，重新通过 70%
阶段门。四档省略参数时仍统一默认 `s`；`z`、`3` 仅用于显式回归比较。这不代表
trace-on、闭环、故障注入、长测或产品验收已通过。详见
[构建档与优化等级](docs/构建档与优化等级.md)。

Rust 算法/桥接、Clippy、交叉编译和 C 平台安全测试：

```powershell
.\test.ps1
```

Python 核心回归只使用标准库，依赖集由 `requirements-core.lock` 显式记录为空。
需要连接串口硬件时，单独安装已锁定的硬件工具依赖：

```powershell
python -m pip install -r .\requirements-hardware.lock
```

生成文件位于 `cmake-build`：

- `rtthread.elf`（RT-Thread CMake 生成器的默认 ELF 名称）
- `fluxrt.bin`
- `fluxrt.hex`
- `fluxrt.map`

## 开发顺序

1. 已完成参考板时钟、引脚、12 kHz PWM/ADC 同步时基和默认安全失能。
2. 已完成 Rust 电流环、强制角度 Rev-Up、90% 母线利用率及空载实机运行。
3. 已完成两轮限流、空载、5 秒的 `closedloop 1` 短时接管试验；试验后均恢复默认 `closedloop 0`。
4. 当前继续用独立测速真值校验 Rs/Ls、电压模型、相序和 SMO/PLL，再扩展到冷启动、负载和长时间闭环。
5. 做 PA11/Break2、软件过流、采样丢失和截止超时故障注入。
6. 用与 PC/MATLAB 相同工况、采样和可观测量进行仿真/实机对比。

当前 24 kHz PWM + 12 kHz 控制只完成 PC/Rust 与 MATLAB/Simulink 设计门；目标
TIM1/ADC、固件默认和板上镜像仍保持 12/12 kHz，详见 A7 报告。

详细说明：

- [A0～A28历史、P0～P6统一后续任务表与验收清单](docs/后续任务阶段计划.md)
- [项目操作记录、当前交接与 Git 追溯规则](docs/工程操作日志.md)
- [P0.1 审计整改基线收口、四档尺寸与可重复哈希](docs/performance/2026-09-28-P0.1-审计整改基线收口.md)
- [产品公共契约 V1：Axis、模式、SI 命令、快照和 C/Rust ABI](docs/产品公共契约V1.md)
- [P1.1 产品公共契约冻结与四档构建证据](docs/performance/2026-09-28-P1.1-产品公共契约冻结.md)
- [命令仲裁、来源权限、sequence、lease和timeout契约V1](docs/命令仲裁与租约契约V1.md)
- [Rust PC与MATLAB共享的双仿真共同契约V1](docs/双仿真共同契约V1.md)
- [P1.2/P1.3命令仲裁与双仿真D0/D1验证报告](docs/performance/2026-09-28-P1.2-P1.3-命令仲裁与双仿真共同契约.md)
- [可恢复配置核心V1：七组V4配置、兼容双槽、CRC、迁移、掉电与回滚](docs/可恢复配置核心V1.md)
- [P3.1可恢复配置核心验证报告](docs/performance/2026-09-28-P3.1-可恢复配置核心.md)
- [并行任务的文件隔离、串行集成与证据规则](docs/并行任务执行与集成规则.md)
- [A22.1/A24.1 并行软件门与 A23 V19 输入 ABI 冻结](docs/performance/2026-09-27-A22.1-A24.1-并行软件门与A23-ABI冻结.md)
- [A23.1～A24.3 V19 完整输入、Rust 传感器模型与 MATLAB 独立对拍](docs/performance/2026-09-27-A23.1-A24.3-V19输入与双仿真门.md)
- [A22.2/A24.4 未标定平台锁门与 PC 完整相电压链](docs/performance/2026-09-28-A22.2-A24.4-平台锁门与PC完整链.md)
- [A21.0/A22.3 Diagnostic 容量归因与 V19 组装门](docs/performance/2026-09-28-A21.0-A22.3-容量归因与V19组装门.md)
- [A21.1 严格整数 CLI、输入拒绝契约与实际容量回收](docs/performance/2026-09-28-A21.1-严格整数CLI与容量回收.md)
- [Diagnostic / Calibration / Identification / Production 构建档与 Rust 优化等级](docs/构建档与优化等级.md)
- [STM32G431RB五种互斥产品组合Profile与切换方法](docs/STM32G431RB产品组合Profile.md)
- [A18 Calibration 构建档、能力隔离与标定契约](docs/performance/2026-09-24-A18-Calibration构建档与标定契约.md)
- [A19 Calibration 板端安全门与静态原始码基线](docs/performance/2026-09-24-A19-Calibration板端安全门与静态基线.md)
- [阶段 1.1 构建优化矩阵与 Production 候选报告](docs/performance/2026-09-23-A1构建档矩阵.md)
- [阶段 1.3 混合 CORDIC/FPU 模长优化与实机 WCET](docs/performance/2026-09-24-A3混合CORDIC与FPU.md)
- [阶段 1.3 CORDIC sin/cos 与 atan2 分项基准](docs/performance/2026-09-24-A3-CORDIC分项基准.md)
- [阶段 1.3 CORDIC 固定延迟读取候选 A/B](docs/performance/2026-09-24-A4-CORDIC固定延迟对比.md)
- [阶段 1.3 CORDIC 健康计数与单次未就绪故障恢复](docs/performance/2026-09-24-A5-CORDIC健康计数与故障恢复.md)
- [阶段 1.3 可移植 CPU 快速数学候选 A/B](docs/performance/2026-09-24-A6-CPU快速数学候选对比.md)
- [阶段 2：24 kHz PWM + 12 kHz 控制多速率仿真设计门](docs/performance/2026-09-24-A7-多速率仿真设计门.md)
- [C / Rust FOC 混合架构与后续开发规则](docs/C与Rust混合架构.md)
- [FOC 算法组合、应用场景与工程落地指南](docs/FOC算法组合与应用场景.md)
- [FOC 整改工程架构分配与代码落位规范](docs/整改架构与职责分配.md)
- [借鉴 ST MCSDK 与 VESC 的工程修正、补全和验证路线](docs/ST与VESC工程改进路线图.md)
- [FluxRT 通用 FOC 产品架构、U0～U11 路线与 Rust/MATLAB 双仿真契约](docs/FluxRT通用FOC产品架构与双仿真路线图.md)
- [可选外部控制、通信、设备接口与最终上位机边界](docs/可选外部控制与通信架构.md)
- [FluxRT Native V1 帧层协议、CRC、消息编号与安全边界](docs/FluxRT%20Native协议V1.md)
- [P2.6B Native定宽载荷、ProductCommand映射与只读Fake服务](docs/performance/2026-10-01-P2.6B-Native载荷与只读Fake服务.md)
- [P2.5A1 PWM/Analog/Step-Dir简单输入归一化核心](docs/performance/2026-10-01-P2.5A1-简单输入归一化核心.md)
- [P2.5A2 输入持久化配置、V3迁移与服务接线](docs/performance/2026-10-01-P2.5A2-输入配置与服务接线.md)
- [第三方协议官方上游隔离、升级规则与 PX4 接入策略](docs/第三方协议依赖与PX4接入策略.md)
- [FluxRT Studio 上位机、在线调参、实时可视化与协议设计](docs/FluxRTStudio上位机架构与调参可视化设计.md)
- [ST MCSDK 参考参数、硬件接口与 PC 闭环仿真](docs/ST_MCSDK参考参数与仿真.md)
- [CORDIC 硬件数学加速、CPU 回退与换芯片方法](docs/硬件数学加速与CPU回退.md)
- [Rust FOC 与 MATLAB 一键联合仿真和绘图](docs/MATLAB联合仿真.md)
- [2026-09-22 NUCLEO-G431RB 实机烧录与安全状态记录](docs/2026-09-22实机烧录记录.md)
- [独立 RT-Thread 空工程创建与复用手册](docs/创建空白RT-Thread工程手册.md)
- [架构与安全边界](docs/架构与安全边界.md)
- [更换 MCU 的移植步骤](docs/更换MCU移植步骤.md)
- [硬件信息待办表](docs/硬件信息待办表.md)
