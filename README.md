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
- 已建立完整 ISR 同拍 WCET，并完成 Rust `3/s/z` 实机矩阵；默认为 `s`。A17 Diagnostic BIN 为 128,600 B，Flash 仅余 2,472 B；A17 新增 ADC 序列后的运行态 WCET 尚待复测，不能沿用旧数值。
- 已建立 Diagnostic / Calibration / Production 三个隔离构建档。A18 Calibration + `s` 为 97,128 B，只保留停机相电压固定窗并在应用/平台两层禁止 arm；Production + `s` 为 95,708 B，不含在线调参、trace 或 4 KiB 固定窗。
- A19 已在目标板完成 Calibration 首次下载、启动和 `foc_start` 拒绝门，并取得停机 256 拍连续原始码；当前缺可信万用表多点参考，仍未形成电压标定参数。
- A17 已接通 PC0/PC3/PC1 三路 BEMF ADC 原始码的 12 kHz、256 拍只读固定窗；无功率实机采集通过，但尚未完成电压标定、动态相序、示波器对拍或观察器接入。
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
rust/crates/foc-control/      Rust：速度/电流环、启动、观测器和硬件 ports
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

Diagnostic、Calibration、Production 输出分别位于 `cmake-build`、
`cmake-build-calibration`、`cmake-build-production`，Rust 静态库也按档位和优化等级
隔离。默认 Rust 优化等级为实机比较后的 `s`；`3` 和 `z` 仅用于回归矩阵。详见
[构建档与优化等级](docs/构建档与优化等级.md)。

Rust 算法/桥接、Clippy、交叉编译和 C 平台安全测试：

```powershell
.\test.ps1
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

- [A0～A28 已完成历史、后续任务阶段与验收清单](docs/后续任务阶段计划.md)
- [项目操作记录、当前交接与 Git 追溯规则](docs/工程操作日志.md)
- [Diagnostic / Calibration / Production 构建档与 Rust 优化等级](docs/构建档与优化等级.md)
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
- [ST MCSDK 参考参数、硬件接口与 PC 闭环仿真](docs/ST_MCSDK参考参数与仿真.md)
- [CORDIC 硬件数学加速、CPU 回退与换芯片方法](docs/硬件数学加速与CPU回退.md)
- [Rust FOC 与 MATLAB 一键联合仿真和绘图](docs/MATLAB联合仿真.md)
- [2026-09-22 NUCLEO-G431RB 实机烧录与安全状态记录](docs/2026-09-22实机烧录记录.md)
- [独立 RT-Thread 空工程创建与复用手册](docs/创建空白RT-Thread工程手册.md)
- [架构与安全边界](docs/架构与安全边界.md)
- [更换 MCU 的移植步骤](docs/更换MCU移植步骤.md)
- [硬件信息待办表](docs/硬件信息待办表.md)
