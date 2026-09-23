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
- 已接入可配置的 STM32G4 CORDIC 快环数学后端；无加速器或调用失败时自动用 CPU 计算。
- RT-Thread RTC 设备驱动暂未启用；当前只完成 LSE 和 RTC 时钟源配置。
- TIM1 已按中心对齐 12 kHz 运行，ADC1/ADC2 由 TIM1 TRGO 同步注入采样；PA11/Break2、1.15 A 软件过流和驱动器故障关断已接入。
- 默认上电保持 CH1～CH3、MOE 和 PB13/PB14/PB15 关闭；只有串口显式执行 `foc_start` 才会 arm，`foc_stop` 立即关断。
- Rust 已运行对齐、强制角度升速、Id/Iq PI、圆限幅、SVPWM 和 SMO。默认 `SMO=启用`、`closed_loop=0`；闭环只能在停机后通过 Shell 临时打开。
- 已建立完整 ISR 同拍 WCET，并完成 Rust `3/s/z` 实机矩阵；默认改为 `s`，Diagnostic BIN 为 122,812 B，开环 WCET 为 8,257/12,500 cycles。
- 已建立 Diagnostic 与 Production 两个构建档。Production + `s` 裁掉在线浮点调参和 trace 后 BIN 为 93,004 B，开环 WCET 为 8,160/12,500 cycles；安全启停、状态和保护仍保留。
- 闭环重复性仍未通过：同条件复测存在 `OBSERVER_LOST`，因此该问题与构建优化分开处理，默认闭环继续关闭。

> 当前只证明低压、限流、空载短时运行和构建档 WCET；没有编码器/测速仪独立真值，也未完成故障注入和长时间多工况验证。上电默认仍为 `closedloop 0`，Production 只是候选构建档，不得误读为量产通过。

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

Diagnostic 输出位于 `cmake-build`，Production 输出位于
`cmake-build-production`。默认 Rust 优化等级为实机比较后的 `s`；`3` 和 `z` 仅用于
回归矩阵。详见[构建档与优化等级](docs/BUILD_PROFILES.md)。

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

详细说明：

- [项目操作记录、当前交接与 Git 追溯规则](docs/PROJECT_OPERATION_LOG.md)
- [Diagnostic / Production 构建档与 Rust 优化等级](docs/BUILD_PROFILES.md)
- [阶段 1.1 构建优化矩阵与 Production 候选报告](docs/performance/2026-09-23-a1-build-profile-matrix.md)
- [C / Rust FOC 混合架构与后续开发规则](docs/RUST_C_FOC_ARCHITECTURE.md)
- [FOC 算法组合、应用场景与工程落地指南](docs/FOC_ALGORITHM_COMBINATIONS_AND_SCENARIOS.md)
- [FOC 整改工程架构分配与代码落位规范](docs/REMEDIATION_ARCHITECTURE_ALLOCATION.md)
- [借鉴 ST MCSDK 与 VESC 的工程修正、补全和验证路线](docs/ST_VESC_ENGINEERING_IMPROVEMENT_ROADMAP.md)
- [ST MCSDK 参考参数、硬件接口与 PC 闭环仿真](docs/ST_MCSDK_REFERENCE_AND_SIMULATION.md)
- [CORDIC 硬件数学加速、CPU 回退与换芯片方法](docs/HARDWARE_MATH_ACCELERATION.md)
- [Rust FOC 与 MATLAB 一键联合仿真和绘图](docs/MATLAB_SIMULATION.md)
- [2026-09-22 NUCLEO-G431RB 实机烧录与安全状态记录](docs/HARDWARE_BRINGUP_2026-09-22.md)
- [独立 RT-Thread 空工程创建与复用手册](docs/CREATE_EMPTY_RTTHREAD_PROJECT.md)
- [架构与安全边界](docs/ARCHITECTURE.md)
- [更换 MCU 的移植步骤](docs/PORTING.md)
- [硬件信息待办表](docs/HARDWARE_TODO.md)
