#![no_std]
#![forbid(unsafe_code)]

//! `lib` 目录中 C 版 FOC 算法库的 Rust `no_std` 等价实现。
//!
//! 当前版本完整覆盖相邻 C 库的 63 个生产模块，包括基础 FOC、调制、
//! 观测器、状态估计器、高级控制器和优化辅助算法。所有实时状态由调用方持有，
//! 不使用堆、不使用全局可变状态，也不包含芯片 HAL 依赖。
//!
//! # 架构位置 / Place in the architecture
//!
//! ```text
//! applications/ (C + RT-Thread 应用、Shell、状态机)
//!   -> foc-rt-bridge (固定 C ABI，唯一跨语言边界)
//!     -> foc-control (板级无关的电流/速度/位置环组合)
//!       -> foc-algorithm (本 crate，纯 no_std 数学)
//! ```
//!
//! 依赖方向是单向的：本 crate 只依赖 `libm`，不得反向依赖 `foc-control`、
//! `foc-rt-bridge`、RT-Thread 或 STM32 HAL。换 MCU 时本 crate 保持不变。
//! The dependency direction is one-way: this crate depends only on `libm` and
//! must never depend on `foc-control`, `foc-rt-bridge`, RT-Thread or the STM32
//! HAL. Porting to another MCU leaves this crate untouched.
//!
//! # crate 级契约 / Crate-level contract
//!
//! - `#![no_std]`：不链接标准库；三角函数/开方走 `libm`。
//! - `#![forbid(unsafe_code)]`：全 crate 无 `unsafe`，指针解引用只能出现在
//!   上层 FFI 包装 crate，不属于本 crate 的责任。
//! - 无堆分配：所有状态是定长标量或定长结构体，容量在编译期固定。
//! - 无全局可变状态：不存在 `static mut`，参数与状态都由调用方持有并传入。
//! - 无 HAL：不认识 ADC、PWM、GPIO、DMA 或任何寄存器。
//!
//! - `#![no_std]`: no standard library; trig and square root come from `libm`.
//! - `#![forbid(unsafe_code)]`: no `unsafe` anywhere in this crate.
//! - No heap allocation: every state is a fixed-size scalar or struct.
//! - No global mutable state: all parameters and states are caller-owned.
//! - No HAL: no knowledge of ADC, PWM, GPIO, DMA or any register.
//!
//! # 实时边界 / Real-time boundaries
//!
//! 本 crate 中的算法都会运行在 **12 kHz 的 ADC 注入中断**（STM32G431，
//! 周期约 83.3 us）内，或由该中断链路间接调用。因此所有函数都禁止动态分配、
//! 阻塞、日志和 Mutex 等待；状态对象必须在初始化阶段静态创建。
//! Every algorithm here runs inside (or is called from) the 12 kHz ADC injected
//! interrupt on the STM32G431, so no function may allocate, block, log or wait on
//! a mutex, and every state object must be created statically at init time.
//!
//! 按可放入快环的程度分三类 / Three classes by fast-loop suitability:
//!
//! - 快环主力，逐拍调用 / core per-sample fast loop:
//!   `math`、`transform`、`utility`、`angle`、`commutation`。
//! - 快环可用但需实测 WCET / usable but needs a measured WCET:
//!   `controller`、`filter`、`modulation`、`foc`、`observer`、`optimization`。
//! - 设计期/离线专用，不得进快环 / design-time or offline only:
//!   `adrc`、`advanced_control`、`learning`、`genetic`。
//!
//! 第三类只保证算法行为与 C 参考一致，不保证在 ISR 时间窗内可执行；把它们放进
//! 12 kHz 快环前必须用 DWT 实测最坏执行周期。相关记录见 `算法库实时性说明.md`。
//! The third class only guarantees behavioural equivalence with the C reference;
//! it says nothing about fitting the ISR window. Measure the WCET with DWT
//! before putting any of it into the 12 kHz fast loop (see `算法库实时性说明.md`).
//!
//! 第二类中的 `foc`、`observer`、`optimization` 目前经 `foc-control` 间接进入
//! ISR；`foc` 模块本身是整链参考实现，目标固件的电流环走 `foc-control`，两者
//! 不要混用。`Observer`/`Optimization` 的 WCET 证据尚不完整。
//! The second class reaches the ISR indirectly through `foc-control`; the `foc`
//! module is a full-chain reference implementation and the target firmware's
//! current loop lives in `foc-control`, so do not mix them.
//!
//! # 门面与重导出 / Facade and re-exports
//!
//! 下列 `pub use` 把 16 个模块的全部公共项扁平化到 crate 根，使上层可以
//! `use foc_algorithm::{FocBasicInput, clarke, ...}`，而不必关心模块划分。
//! 这也意味着**新增公共项会自动成为 crate 的公共 API**：重命名或删除模块内
//! 的公共标量会直接破坏 `foc-control`、`foc-sim` 和 `foc-rt-bridge`。
//! The `pub use` block below flattens all 15 modules into the crate root, so an
//! added public item silently becomes public API and a renamed one breaks
//! `foc-control`, `foc-sim` and `foc-rt-bridge`.
//!
//! # 数值与安全约定 / Numerical and safety conventions
//!
//! - 角度一律 `[rad]`，角速度 `[rad/s]`，采样周期 `[s]`；输入应为有限浮点数，
//!   `NaN/Inf` 不属于有效输入域。
//! - 硬件过流、欠压/过压、过温必须由独立保护链处理，不能用算法返回值代替。
//! - `#[repr(C)]` 只为稳定数据布局准备，当前没有导出 `extern "C"` 符号。
//!
//! - Angles are `[rad]`, angular speed `[rad/s]`, sample periods `[s]`; inputs
//!   must be finite, `NaN/Inf` is outside the valid domain.
//! - Over-current, over/under-voltage and over-temperature need an independent
//!   protection path; algorithm return values cannot replace it.
//! - `#[repr(C)]` only prepares a stable layout; no `extern "C"` symbol is
//!   exported today.
//!
//! 完整模块清单与逐项迁移状态见 `算法库移植状态.md` 和
//! `算法库总览与对接指南.md`。
//! See `算法库移植状态.md` for the per-module migration status.
//!
//! 参考 / Reference: docs/架构与安全边界.md, docs/C与Rust混合架构.md

// 模块按功能划分，不按原 C 目录划分；模块间只允许依赖 `math` 这一叶子模块。
// Modules are grouped by function rather than by the original C directory tree;
// cross-module dependency is limited to the leaf module `math`.
/// ADRC：自抗扰控制（TD、ESO、非线性误差反馈与带宽整定）。
/// ADRC: active disturbance rejection control (TD, ESO, nonlinear feedback).
///
/// 属于控制理论研究/设计期工具。库内不限制循环次数，进快环前必须实测 WCET。
/// A design-time/offline control-theory tool; measure the WCET before using it
/// inside the fast loop.
pub mod adrc;
/// 高级控制：自适应、反步、模糊、H∞、LQR、MPC、MRAC、滑模。
/// Advanced control: adaptive, backstepping, fuzzy, H-infinity, LQR, MPC,
/// MRAC and sliding mode.
///
/// `MpcState::update` 的运算次数随候选数线性增长，是设计/离线算法的典型代表。
/// `MpcState::update` scales with the candidate count and is a typical
/// design-time/offline algorithm.
pub mod advanced_control;
/// CORDIC 角度与幅值：软件参考实现，与硬件 CORDIC 后端约定同一输出量纲。
/// CORDIC angle and magnitude: the software reference implementation, sharing
/// one output convention with the hardware CORDIC backend.
pub mod angle;
/// 六步换相：BLDC 方波主链的桥臂状态表，与正弦 FOC 是互斥的两条链路。
/// Six-step commutation: the BLDC trapezoidal main chain, mutually exclusive
/// with the sinusoidal FOC chain.
pub mod commutation;
/// 控制器：P/PI/PD/PID、速度环、位置环、级联环与前馈。
/// Controllers: P/PI/PD/PID, speed loop, position loop, cascade and
/// feed-forward.
pub mod controller;
/// 滤波器：一阶低通、一阶高通和带滤波微分器。
/// Filters: first-order low pass, first-order high pass and filtered
/// differentiator.
pub mod filter;
/// 基础有感 FOC：Clarke/Park -> 双 PI -> 逆 Park -> SVPWM 的整链示例。
/// Basic sensored FOC: the full Clarke/Park -> dual PI -> inverse Park -> SVPWM
/// chain.
pub mod foc;
/// 遗传算法辅助函数：单变量交叉、变异与最优候选维护，随机信号由调用方提供。
/// Genetic-algorithm helpers: single-variable crossover, mutation and
/// best-candidate bookkeeping; the random signal is supplied by the caller.
///
/// 离线/设计期专用，不得放入快环。
/// Offline/design-time only; must not run in the fast loop.
pub mod genetic;
/// 逆变器平均损失：死区、器件压降与过零软带的纯数学模型。
/// Average inverter loss: pure dead-time, device-drop and zero-band equations.
pub mod inverter;
/// 固定拓扑神经网络与固定容量 Q-learning 表。
/// Fixed-topology neural network and fixed-capacity Q-learning table.
///
/// 只做推理，不做训练；训练、量化和安全认证都在本库之外。
/// Inference only, never training; training, quantisation and safety
/// certification live outside this crate.
pub mod learning;
/// 基础数学：限幅、角度环绕与 `atan2`，是全库唯一的叶子依赖模块。
/// Base math: clamping, angle wrapping and `atan2`; the only leaf dependency of
/// the whole crate.
pub mod math;
/// 调制：正弦 PWM、SVPWM 与 DPWM，把电压指令换成三相占空比。
/// Modulation: sine PWM, SVPWM and DPWM, turning a voltage command into
/// three-phase duties.
pub mod modulation;
/// 观测器入口：PLL 以及各观测器子模块的重导出。
/// Observer entry point: the PLL plus re-exports of the observer submodules.
pub mod observer;
/// 电机工作区优化：MTPA、MTPV、V/f 与弱磁。
/// Operating-region optimisation: MTPA, MTPV, V/f and field weakening.
pub mod optimization;
/// 坐标变换：Clarke、逆 Clarke、Park 与逆 Park。
/// Coordinate transforms: Clarke, inverse Clarke, Park and inverse Park.
pub mod transform;
/// 限幅与斜坡：输出安全边界和目标变化率限制。
/// Limiting and ramping: output safety bounds and target slew-rate limits.
pub mod utility;

pub use adrc::*;
pub use advanced_control::*;
pub use angle::*;
pub use commutation::*;
pub use controller::*;
pub use filter::*;
pub use foc::*;
pub use genetic::*;
pub use inverter::*;
pub use learning::*;
pub use math::*;
pub use modulation::*;
pub use observer::*;
pub use optimization::*;
pub use transform::*;
pub use utility::*;
