#![no_std]
#![forbid(unsafe_code)]

//! FluxRT —— 与芯片无关的电机控制组合层与硬件端口。
//! FluxRT - board-independent motor-control composition and hardware ports.
//!
//! 本 crate 是 FluxRT 的控制组合层：把电流环、速度环、预启动（rev-up）时序、
//! 转子观测器与参数量组合成一套与芯片无关的控制律，并用 trait 描述硬件端口。
//! 同一套代码因此既跑在 STM32G431 上，也跑在 PC 仿真里。
//! This crate contains the control topology shared by the STM32 target and the
//! host simulation. It deliberately has no RTOS, HAL, allocator, or MCU types.
//! Concretely it is the control-composition layer: current loop, speed loop,
//! rev-up sequencer, rotor observers and parameter sets, plus the hardware-port
//! traits, so the same control law runs on the STM32G431 and in host simulation.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C, RT-Thread)
//!     -> foc/platform/stm32g431 (C 硬件适配层)
//!     -> foc-rt-bridge (整个 Rust 栈里唯一的 C ABI 边界)
//!     -> `foc-control`（本 crate，控制组合）
//!     -> foc-algorithm（纯数学，`no_std`）
//!   本 crate 不认识 RTOS、HAL、堆或任何 MCU 类型，所以是 `#![no_std]` 且
//!   `#![forbid(unsafe_code)]`；所有 C ABI 与 `unsafe` 都留在 `foc-rt-bridge`。
//!   The C application and platform adapter reach this crate only through
//!   `foc-rt-bridge`. No RTOS, HAL, heap or MCU types appear here: the crate is
//!   `no_std` and forbids `unsafe`.
//!
//! 实时约束 / Real-time constraints:
//!   本 crate 的函数被 12 kHz ADC ISR 里的快环逐拍调用：所有路径都不得动态
//!   分配、不得阻塞、不得打日志、不得等待 Mutex。这里只做纯计算与状态机推进；
//!   需要三角函数或开方时一律走 [`ControlMath`] 后端（实机为 CORDIC）。
//!   Called every sample from the 12 kHz ADC ISR fast loop: no path may allocate,
//!   block, log or wait on a mutex. Only pure computation and state advance happen
//!   here; trig and square root go through the `ControlMath` backend.
//!
//! 量纲 / Units: 本 crate 不设统一量纲，各结构体字段用后缀与注释自带单位
//!   （`[V] [A] [rad] [rad/s] [rpm] [s]`），占空比与归一化比值为无量纲。全部是
//!   `f32`；本 crate 不使用 Q 格式，定点只存在于 CORDIC 平台适配器内部。
//!   Each field documents its own unit (`[V] [A] [rad] [rad/s] [rpm] [s]`); duty
//!   and normalized ratios are dimensionless. Everything is `f32`; no Q formats
//!   here, fixed point lives inside the CORDIC adapter.
//!
//! 参考 / Reference: docs/架构与安全边界.md, docs/C与Rust混合架构.md

// 模块分工 / Module map:
//   types      控制器与端口之间的纯数据结构（量纲见字段名）
//   math       可替换的数学后端 trait（sin/cos、幅值、atan2）
//   params     被控对象与 PI 参数集，含来源标注与"未辨识"警告
//   ports      反馈 / 功率级 / 安全 三个硬件端口 trait
//   controller 电流环、速度环与串级控制器（MCSDK 参考拓扑）
//   observer   SMO+PLL 与浮点反电势观测器，含可靠性门控
//   voltage    观察器电压来源策略；当前默认/唯一批准路径为 CommandModel
//   startup    开环预启动时序与无感切换的角度渐变
//   runtime    端口 + 控制器的有状态"一拍"外壳（PC 仿真主用）
//   `types` holds the plain data contract, `math` the replaceable math backend,
//   `params` the plant and PI sets, `ports` the three hardware traits,
//   `controller` the cascaded loops, `observer` the sensorless estimators,
//   `startup` the rev-up sequencer and `runtime` the stateful one-tick shell.
pub mod controller;
pub mod math;
pub mod observer;
pub mod params;
pub mod ports;
pub mod runtime;
pub mod startup;
pub mod types;
pub mod voltage;

// 全部重导出，使调用方写 `foc_control::Xxx` 而不必写子模块路径；
// `foc-rt-bridge` 与 `foc-sim` 都依赖这个扁平命名空间。
// Flat re-exports so callers write `foc_control::Xxx`; `foc-rt-bridge` and
// `foc-sim` both rely on this namespace.
pub use controller::*;
pub use math::*;
pub use observer::*;
pub use params::*;
pub use ports::*;
pub use runtime::*;
pub use startup::*;
pub use types::*;
pub use voltage::*;
