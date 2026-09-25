//! FluxRT —— 端口 + 控制器的有状态运行时（"一拍"的完整外壳）。
//! FluxRT - the stateful runtime that wraps ports plus a controller for one tick.
//!
//! 本类型把控制器与硬件端口组合成固定顺序的一条流水线：使能检查 → 读反馈 →
//! 数值校验 → 控制计算 → 输出校验 → 写 PWM，并在任意失败路径上关断功率级并
//! 复位控制器状态。
//! The type composes a controller with hardware ports into one fixed pipeline:
//! enable check, read feedback, validate, compute, validate output, write PWM, and
//! on any failure disable the power stage and reset the controller.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-sim -> `foc-control`（本模块）
//!   `foc-sim` 的 `run_reference_simulation` 直接使用本类型；实机快环走
//!   `foc-rt-bridge` 的 `foc_rust_realtime_step()`，后者自己组合同一批部件而不
//!   经过本类型，因为 C ABI 还需要手动状态机与观测器门控。
//!   `foc-sim` uses this type directly; the on-target fast loop goes through
//!   `foc_rust_realtime_step()`, which composes the same pieces itself.
//!
//! 实时约束 / Real-time constraints:
//!   `tick()` 会在 12 kHz 快环里被调用：无动态分配、无阻塞、无日志。本类型只
//!   保存 `Copy` 状态，尺寸固定，因此不需要堆。
//!   `tick()` runs in the 12 kHz fast loop: no allocation, blocking or logging, and
//!   only `Copy` state is stored.
//!
//! 安全语义 / Safety semantics:
//!   每个 `Err` 返回前都已经调用 `disable_pwm()` 并 `reset()` 控制器，调用方不
//!   需要再做任何补救动作。这是本类型存在的核心理由：失败路径不允许被遗漏。
//!   Every `Err` return has already disabled PWM and reset the controller, so the
//!   caller needs no recovery step. That is the core reason this type exists:
//!   failure paths must not be forgettable.
//!
//! 量纲 / Units: 电流 `[A]`、电压 `[V]`、角度 `[rad]`、角速度 `[rad/s]`、
//!   转速 `[rpm]`、时间 `[s]`；占空比无量纲 `[0,1]`。全部为 `f32`，不使用 Q 格式。
//!   Currents `[A]`, voltages `[V]`, angles `[rad]`, speed `[rad/s]` or `[rpm]`,
//!   time `[s]`, duty dimensionless. Everything is `f32`; no Q formats.
//!
//! 参考 / Reference: docs/架构与安全边界.md

use crate::{
    ControlTelemetry, HardwareFault, MotorHardware, PwmCommand, SpeedCommand, StReferenceController,
};

/// 控制器 + 硬件端口的组合，带一个显式的软件使能位。
/// A controller plus hardware ports, with an explicit software enable bit.
///
/// `enabled` 是软件侧意愿，`SafetyPort::power_stage_faulted()` 是硬件侧事实；
/// `tick()` 要求两者同时成立才会输出，任一不成立都走关断路径。
/// `enabled` is the software intent and `power_stage_faulted()` is the hardware
/// fact; `tick()` requires both, and either failing takes the disable path.
pub struct ControlRuntime<H> {
    /// 串级控制器（电流内环 + 速度外环）。
    /// The cascaded controller (current inner loop, speed outer loop).
    controller: StReferenceController,
    /// 硬件端口集合：反馈、功率级与安全。
    /// The hardware port set: feedback, power stage and safety.
    hardware: H,
    /// 软件使能位；`false` 时 `tick()` 直接关断而不做任何计算。
    /// Software enable bit; when false `tick()` disables without computing.
    enabled: bool,
}

impl<H: MotorHardware> ControlRuntime<H> {
    /// 组合控制器与端口；初始状态是**未使能**，必须显式调用 [`Self::enable`]。
    /// Combines controller and ports; the initial state is NOT enabled, so
    /// [`Self::enable`] must be called explicitly.
    ///
    /// 不在这里使能是刻意的：上电时功率级可能还处于未知状态，启动必须由上层
    /// 在确认硬件就绪后单独触发。
    /// Not enabling here is deliberate: the power stage state is unknown at
    /// power-up, so the upper layer triggers the start explicitly.
    pub fn new(controller: StReferenceController, hardware: H) -> Self {
        Self {
            controller,
            hardware,
            enabled: false,
        }
    }

    /// 使能控制输出；功率级已有故障时立即关断并返回故障。
    /// Enables control output; if the power stage is already faulted it disables
    /// immediately and reports the fault.
    ///
    /// 使能前先 `reset()` 控制器，清除上一次运行的 PI 积分与速度计数器；
    /// 否则残留积分会在第一个周期直接推出饱和电压。
    /// The controller is reset first so leftover PI integrals and the speed counter
    /// from a previous run cannot saturate the very first sample.
    pub fn enable(&mut self) -> Result<(), HardwareFault> {
        if self.hardware.power_stage_faulted() {
            self.hardware.disable_pwm();
            return Err(HardwareFault::PowerStageFault);
        }
        self.controller.reset();
        self.enabled = true;
        Ok(())
    }

    /// 停机：清使能位、复位控制器并关断栅极。可重复调用且不会失败。
    /// Stops: clears enable, resets the controller and disables the gate drive.
    /// Repeatable and infallible.
    pub fn disable(&mut self) {
        self.enabled = false;
        self.controller.reset();
        self.hardware.disable_pwm();
    }

    /// 跑一拍完整控制：校验 → 计算 → 输出。
    /// Runs one full control tick: validate, compute, output.
    ///
    /// 返回的 `Err` 表示本拍已经安全停机（栅极已关断、控制器已复位）；调用方
    /// 可以把它当作"需要上报/重试的故障"，而不是"需要补救的半完成状态"。
    /// An `Err` means this sample already shut down safely (gates off, controller
    /// reset), so the caller treats it as a fault to report rather than a partial
    /// state to recover.
    ///
    /// 校验发生在计算之前：`dc_bus_voltage` 必须为正（否则 SVPWM 归一化除零），
    /// 电流、角度与转速必须有限。缺失这些校验会让 `NaN` 直接传到占空比上。
    /// Validation precedes computation: `dc_bus_voltage` must be positive or SVPWM
    /// normalization divides by zero, and currents, angle and speed must be finite.
    pub fn tick(&mut self, command: SpeedCommand) -> Result<ControlTelemetry, HardwareFault> {
        if !self.enabled || self.hardware.power_stage_faulted() {
            self.disable();
            return Err(HardwareFault::PowerStageFault);
        }
        let feedback = match self.hardware.read_feedback() {
            Ok(feedback) => feedback,
            Err(fault) => {
                self.disable();
                return Err(fault);
            }
        };
        if !feedback.dc_bus_voltage.is_finite()
            || feedback.dc_bus_voltage <= 0.0
            || !feedback.currents.a.is_finite()
            || !feedback.currents.b.is_finite()
            || !feedback.currents.c.is_finite()
            || !feedback.rotor.electrical_angle_rad.is_finite()
            || !feedback.rotor.mechanical_speed_rad_s.is_finite()
        {
            self.disable();
            return Err(HardwareFault::InvalidFeedback);
        }
        let (pwm, telemetry) = self.controller.update(&feedback, command);
        // 输出闸门：算法给出的占空比必须有限且落在 [0,1]，否则关断而不是钳位。
        // Output gate: the duty must be finite and inside [0,1]; disable rather
        // than clamp.
        if !pwm.is_valid() {
            self.disable();
            return Err(HardwareFault::OutputRejected);
        }
        if let Err(fault) = self.hardware.apply_pwm(pwm) {
            self.disable();
            return Err(fault);
        }
        Ok(telemetry)
    }

    /// 只读访问硬件端口（遥测、日志与仿真取状态用）。
    /// Read-only access to the hardware ports (telemetry and simulation state).
    pub fn hardware(&self) -> &H {
        &self.hardware
    }

    /// 可写访问硬件端口（仿真注入故障、负载或改变母线电压用）。
    /// Mutable access to the hardware ports; `foc-sim` uses it to inject faults,
    /// load torque or bus changes.
    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.hardware
    }

    /// 控制器周期 `[s]`，即 `1 / pwm_frequency_hz`；仿真用它作为积分步长。
    /// Controller period in `[s]`; the simulation uses it as the plant step.
    ///
    /// 它取的是**电流环**周期，不是速度环周期；仿真必须以这个值推进被控对象，
    /// 否则多速率调度与实机不一致。
    /// This is the CURRENT-loop period, not the speed-loop period; the simulation
    /// must step the plant with it or the multi-rate timing differs from target.
    pub fn controller_period_s(&self) -> f32 {
        1.0 / self.controller.parameters().pwm_frequency_hz as f32
    }

    /// 安全停机输出（三相 0.5 的零电压矢量），不依赖任何实例状态。
    /// The safe stop output (0.5 on all phases, zero voltage vector), independent
    /// of any instance state.
    ///
    /// 是关联函数而不是方法，因为它要在"还没有 runtime 可用"的早期初始化与
    /// 故障恢复路径里使用。
    /// It is an associated function rather than a method so the earliest
    /// initialization and fault-recovery paths can use it before a runtime exists.
    pub fn last_safe_pwm() -> PwmCommand {
        PwmCommand::default()
    }
}
