//! FluxRT —— 控制器面向硬件的端口抽象（反馈 / 功率级 / 安全）。
//! FluxRT - the hardware-facing ports the controller talks to.
//!
//! 这三个 trait 是控制律与具体 MCU/仿真之间的唯一接口：实机由 C 平台适配层
//! 实现，PC 仿真由 `foc-sim` 的 `SimHardware` 用理想转子反馈实现，
//! `foc-control` 自己完全不知道 PWM 寄存器或 ADC 的存在。
//! These three traits are the only interface between the control law and a
//! concrete MCU or simulation: the C platform adapter implements them on target,
//! `SimHardware` implements them in `foc-sim`, and `foc-control` never sees a PWM
//! register or an ADC.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc/platform/stm32g431 (C) 或 foc-sim -> `foc-control`（本模块）
//!   依赖方向是"硬件实现 trait"，而不是"控制依赖硬件"；新增板卡只需实现这三个
//!   trait，不必改动控制律。
//!   Hardware implements the traits rather than the control law depending on
//!   hardware, so a new board only has to implement these three traits.
//!
//! 实时约束 / Real-time constraints:
//!   `read_feedback` 与 `apply_pwm` 在 12 kHz ADC ISR 上下文内被调用，实现方
//!   不得动态分配、不得阻塞、不得打日志。`disable_pwm` 必须能在任意时刻安全地
//!   重复调用（故障路径上会被调用多次）。
//!   `read_feedback` and `apply_pwm` run inside the 12 kHz ADC ISR, so
//!   implementations must not allocate, block or log. `disable_pwm` must be safe
//!   to call repeatedly at any time, since fault paths call it more than once.
//!
//! 安全语义 / Safety semantics:
//!   每个 `Err` 路径都要求调用方关断栅极，`ControlRuntime` 已经这样做了；
//!   端口实现自身不要"尽力而为地继续输出"，也不要依赖上层来兜底。
//!   Every `Err` path requires the caller to disable the gate drive and
//!   `ControlRuntime` does so; implementations must not keep driving.
//!
//! 量纲 / Units: 由 [`crate::PwmCommand`] 与 [`crate::FeedbackSnapshot`] 定义
//!   （占空比无量纲、电流 `[A]`、电压 `[V]`、角度 `[rad]`、角速度 `[rad/s]`）。
//!   本模块全为 `f32`，不使用 Q 格式。
//!   Units come from the command and snapshot types; everything is `f32`.
//!
//! 参考 / Reference: docs/架构与安全边界.md（安全边界一节）

use crate::{FeedbackSnapshot, HardwareFault, PwmCommand};

/// 每个 PWM 周期提供一份同步的 ADC/位置快照。
/// Provides one synchronized ADC/position snapshot for a PWM period.
///
/// 实现方必须保证三相电流、母线电压与转子角度来自**同一次**采样触发；否则
/// Park 变换会把不同时刻的量混在一起，产生与转速同频的 dq 纹波。
/// The implementation must guarantee that currents, bus voltage and rotor angle
/// come from the SAME sampling trigger, otherwise Park mixes different instants
/// and produces speed-synchronous dq ripple.
///
/// 返回 `Err` 表示本拍反馈不可用，调用方会关断 PWM；不要用"上一次的值"顶替，
/// 那会让控制律在失去反馈后继续带载运行。
/// `Err` means this sample is unusable and the caller disables PWM; never
/// substitute a stale value, which would keep the loop driving without feedback.
pub trait FeedbackPort {
    /// 读取本拍反馈；单位见 [`crate::FeedbackSnapshot`] 各字段。
    /// Reads this sample's feedback; see `FeedbackSnapshot` for units.
    fn read_feedback(&mut self) -> Result<FeedbackSnapshot, HardwareFault>;
}

/// 唯一允许把占空比命令送到功率级的端口。
/// The only port allowed to pass a duty command to a power-stage adapter.
///
/// `apply_pwm` 必须在写寄存器前复核命令合法性，并在失败时自行关断；返回 `Err`
/// 之后调用方还会再调用一次 [`PwmPort::disable_pwm`]，所以实现必须幂等。
/// `apply_pwm` must re-check the command before writing registers and disable on
/// failure. The caller calls `disable_pwm` again afterwards, so it must be
/// idempotent.
pub trait PwmPort {
    /// 应用三相占空比；占空比无量纲 `[0, 1]`，0.5 为零电压矢量。
    /// Applies the three-phase duty; duty is dimensionless `[0, 1]`, 0.5 is zero
    /// voltage.
    fn apply_pwm(&mut self, command: PwmCommand) -> Result<(), HardwareFault>;

    /// 立即关断栅极并回到安全输出；必须可重复调用且不得失败。
    /// Immediately disables the gate drive; must be repeatable and infallible.
    fn disable_pwm(&mut self);
}

/// 表示异步硬件保护（刹车输入、比较器、栅极故障）。
/// Represents asynchronous hardware protection (break, comparator, gate fault).
///
/// 这是**查询**而不是中断：C 侧的保护逻辑已经在硬件里关断了栅极，这里只是让
/// Rust 的状态机跟上。不要把它当作唯一的保护手段，也不要指望它及时到拍。
/// This is a query, not an interrupt: the C side has already tripped the gates in
/// hardware and this only lets the Rust state machine follow. Never treat it as
/// the only protection.
pub trait SafetyPort {
    /// 功率级当前是否处于故障态；为真时控制律不得再输出。
    /// Whether the power stage is faulted right now; if true, stop driving.
    fn power_stage_faulted(&self) -> bool;
}

/// 一个真实或仿真功率级必须同时具备的三个端口。
/// The three ports a real or simulated power stage must provide together.
///
/// 由下面的 blanket impl 自动实现：任何同时实现三个 trait 的类型都自动成为
/// `MotorHardware`，所以接入新硬件不需要改动本模块或控制器。
/// A blanket impl makes any type implementing all three automatically
/// `MotorHardware`, so new hardware needs no change here or in the controller.
pub trait MotorHardware: FeedbackPort + PwmPort + SafetyPort {}

impl<T> MotorHardware for T where T: FeedbackPort + PwmPort + SafetyPort {}
