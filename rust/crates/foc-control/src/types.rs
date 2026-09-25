//! FluxRT —— 控制器与硬件端口之间的数据契约（量纲见各字段）。
//! FluxRT - the data contract between the controller and the hardware ports.
//!
//! 本模块只放"纯数据"结构体：一个 PWM 周期的反馈快照、速度/电流指令、PWM
//! 输出、遥测以及硬件故障枚举。它们全部 `#[repr(C)]` 且 `Copy`，因为
//! `foc-rt-bridge` 会逐字段映射到 C ABI 结构体，也会在 ISR 里按值传递。
//! This module holds pure data: a PWM-period feedback snapshot, speed and current
//! commands, the PWM output, telemetry and the hardware-fault enum. All are
//! `#[repr(C)]` and `Copy` because `foc-rt-bridge` maps them field-by-field onto C
//! ABI structs and they are passed by value inside the ISR.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge (C ABI) -> `foc-control`（本模块）-> foc-algorithm
//!   本模块不依赖任何端口或算法，只借用 `foc-algorithm` 的坐标系类型。
//!   Nothing here depends on a port or an algorithm; only the coordinate-frame
//!   types come from `foc-algorithm`.
//!
//! 实时约束 / Real-time constraints:
//!   这些结构体在 12 kHz ADC ISR 的栈上构造与拷贝，因此必须保持小尺寸；
//!   不要在这里加入 `Vec`、`String` 或任何需要析构的字段（本 crate 是 `no_std`）。
//!   These structs are built and copied on the 12 kHz ISR stack, so keep them
//!   small and never add owning fields: this crate is `no_std`.
//!
//! 量纲 / Units: 字段名自带量纲后缀（`_a` 安培 `[A]`、`_v` 伏特 `[V]`、
//!   `_rpm` `[rpm]`、`_rad` `[rad]`、`_rad_s` `[rad/s]`、`_s` 秒 `[s]`），
//!   占空比是 0..1 的无量纲比值。全部为 `f32`，不使用 Q 格式。
//!   Field suffixes carry the unit: `_a` `[A]`, `_v` `[V]`, `_rpm` `[rpm]`,
//!   `_rad` `[rad]`, `_rad_s` `[rad/s]`, `_s` `[s]`; duty is a dimensionless 0..1
//!   ratio. Everything is `f32`; no Q formats.
//!
//! 参考 / Reference: foc/include/foc_rust_bridge.h

use foc_algorithm::{AlphaBeta, Dq};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 一个 PWM 周期的三相电流快照，单位安培 `[A]`。
/// One PWM-period current snapshot in amperes.
///
/// 三相必须来自**同一采样时刻**（一次 ADC 注入序列）。分开读取会引入相间时间
/// 偏斜，在 dq 上表现为与转速同频的纹波，而电流环看上去仍然"稳定"。
/// All three must come from the same sampling instant (one injected ADC
/// sequence); reading them separately adds inter-phase skew that shows up as
/// speed-synchronous ripple on dq while the loop still looks stable.
pub struct PhaseCurrents {
    /// A 相电流 `[A]`，极性约定为 `(offset - raw)`，与 MCSDK/IHM16M1 参考一致。
    /// Phase A current in `[A]`; polarity is `(offset - raw)`, as in MCSDK.
    pub a: f32,
    /// B 相电流 `[A]`，与 A 相同极性约定。
    /// Phase B current in `[A]`, same polarity convention as A.
    pub b: f32,
    /// C 相电流 `[A]`；三相一般由同一 ADC 注入序列采样得到。
    /// Phase C current in `[A]`; usually captured in the same injected sequence.
    pub c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 转子反馈：电角度与机械角速度。
/// Rotor feedback: electrical angle and mechanical angular speed.
///
/// `electrical_angle_rad` 必须是**电角度**（机械角度 × 极对数 + 零位偏置），
/// 不是机械角度；传机械角度会让极对数大于 1 的电机在高转速下失控。低速小角度
/// 下用错角度往往仍能"闭环稳定"，所以必须用示波器或已知转向验证符号与零位。
/// `electrical_angle_rad` must be the ELECTRICAL angle, not the mechanical one;
/// feeding the mechanical angle makes any motor with more than one pole pair
/// uncontrollable at speed, while a low-speed test can still look stable.
pub struct RotorFeedback {
    /// 电角度 `[rad]`，通常归一化在 `[0, 2*pi)`。
    /// Electrical angle in `[rad]`, normally wrapped into `[0, 2*pi)`.
    pub electrical_angle_rad: f32,
    /// 机械角速度 `[rad/s]`；转 rpm 需乘 `30/pi`。
    /// Mechanical angular speed in `[rad/s]`; multiply by `30/pi` for rpm.
    pub mechanical_speed_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 一个 PWM 周期的完整反馈快照：三相电流 + 母线电压 + 转子状态。
/// One complete PWM-period feedback snapshot: currents, DC bus and rotor.
///
/// 电流环、观测器与遥测共享这一份快照，避免同一拍内多次读取硬件造成不一致。
/// `dc_bus_voltage` 必须大于 0：为 0 会让 SVPWM 归一化除零，`ControlRuntime`
/// 会把它判为 [`crate::HardwareFault::InvalidFeedback`]。
/// The current loop, observer and telemetry share one snapshot so a single sample
/// cannot be read inconsistently. `dc_bus_voltage` must be > 0 or SVPWM
/// normalization divides by zero; `ControlRuntime` rejects that as
/// `InvalidFeedback`.
pub struct FeedbackSnapshot {
    /// 三相电流 `[A]`。
    /// Three-phase currents in `[A]`.
    pub currents: PhaseCurrents,
    /// 直流母线电压 `[V]`，必须大于 0；SVPWM 与电压圆限幅都用它做标度。
    /// DC bus voltage in `[V]`, must be > 0; used to scale SVPWM and the limiter.
    pub dc_bus_voltage: f32,
    /// 转子状态（电角度 `[rad]`、机械角速度 `[rad/s]`）。
    /// Rotor state (electrical angle `[rad]`, speed `[rad/s]`).
    pub rotor: RotorFeedback,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 速度环指令：目标转速与 d 轴电流给定。
/// Speed-loop command: target speed and d-axis current reference.
///
/// `target_rpm` 是**机械**转速 `[rpm]`（不是电频率）；`id_ref_a` 单位安培 `[A]`，
/// 表贴式永磁电机通常给 0，只有弱磁或利用凸极时才给负值。
/// `target_rpm` is MECHANICAL speed in `[rpm]`, not electrical frequency;
/// `id_ref_a` is in `[A]`, normally 0 for a surface PMSM and negative only for
/// field weakening or intentional saliency use.
pub struct SpeedCommand {
    /// 目标机械转速 `[rpm]`。
    /// Target mechanical speed in `[rpm]`.
    pub target_rpm: f32,
    /// d 轴电流给定 `[A]`，速度环原样透传，不做闭环调节。
    /// d-axis current reference in `[A]`, passed through by the speed loop.
    pub id_ref_a: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 电流环指令：d/q 轴电流给定，单位安培 `[A]`。
/// Current-loop command: d/q current references in amperes `[A]`.
///
/// 这是速度环的输出，也是 `foc-rt-bridge` 里 slew-rate 限幅的对象：限幅直接
/// 作用在本结构体上，所以无感切换瞬间不会产生转矩阶跃。
/// This is the speed loop's output and the object of the bridge's slew-rate limit,
/// which is what prevents a torque step at sensorless handoff.
pub struct CurrentCommand {
    /// d 轴电流给定 `[A]`。
    /// d-axis current reference in `[A]`.
    pub id_ref_a: f32,
    /// q 轴电流给定 `[A]`，正比于转矩（表贴式电机）。
    /// q-axis current reference in `[A]`, proportional to torque for a surface PM.
    pub iq_ref_a: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
/// 三相占空比输出，无量纲，范围 `[0, 1]`，0.5 表示 50% 占空比。
/// Three-phase duty output, dimensionless in `[0, 1]`; 0.5 means 50 percent.
///
/// `Default` 给三相 0.5，对应零电压矢量（三相共模相等、差模为零），因此它是
/// 安全停止值而不是 0：三相全 0 会把三个下桥臂一起打开，形成制动短路。
/// `Default` is 0.5 on all phases, the zero-voltage vector, and is therefore the
/// safe stop value rather than 0: all-zero duty would turn on all low-side
/// devices and brake the motor through a short circuit.
pub struct PwmCommand {
    /// A 相占空比，无量纲 `[0, 1]`。
    /// Phase A duty, dimensionless `[0, 1]`.
    pub duty_a: f32,
    /// B 相占空比，无量纲 `[0, 1]`。
    /// Phase B duty, dimensionless `[0, 1]`.
    pub duty_b: f32,
    /// C 相占空比，无量纲 `[0, 1]`。
    /// Phase C duty, dimensionless `[0, 1]`.
    pub duty_c: f32,
}

impl Default for PwmCommand {
    /// 三相居中 0.5，即零电压矢量，是"停机/未使能"时的安全输出。
    /// Centred 0.5 on all phases, the zero-voltage vector used when idle.
    fn default() -> Self {
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
        }
    }
}

impl PwmCommand {
    /// 输出闸门：三相占空比必须有限且各自落在 `[0, 1]` 内。
    /// Output gate: every duty must be finite and inside `[0, 1]`.
    ///
    /// 任何一项不满足，都说明上游出现了 `NaN`/`Inf`（通常是除零或某个参数为
    /// 0），或者 SVPWM 过调制越界。此时调用方必须关断栅极，而不是钳位后继续
    /// 输出：钳位会掩盖已经失效的控制律，让电机带着错误的电压矢量继续转。
    /// A failure means `NaN`/`Inf` appeared upstream (usually a divide by zero or a
    /// zero parameter) or SVPWM over-modulated. The caller must disable the gate
    /// drive instead of clamping and continuing: clamping would hide a control law
    /// that has already failed.
    pub fn is_valid(&self) -> bool {
        self.duty_a.is_finite()
            && self.duty_b.is_finite()
            && self.duty_c.is_finite()
            && (0.0..=1.0).contains(&self.duty_a)
            && (0.0..=1.0).contains(&self.duty_b)
            && (0.0..=1.0).contains(&self.duty_c)
    }
}

/// 一次控制周期的可观测输出，供 C 侧遥测与上位机波形使用。
/// Per-cycle observable output for C-side telemetry and host traces.
///
/// `current_dq`/`voltage_dq` 是 Park 后的实测电流与 PI 输出电压（`[A]`/`[V]`），
/// `voltage_alpha_beta` 是逆 Park 后交给 SVPWM 的 `[V]` 矢量，
/// `measured_speed_rpm` 是观测器机械转速 `[rpm]`。
/// `voltage_limited` 表示本拍电压圆限幅是否生效：它持续为真说明指令已超出母线
/// 能力，此时电流环失去线性度，转矩不再跟随电流给定。
/// `current_dq`/`voltage_dq` are the Park-transformed measured current and PI
/// output voltage (`[A]`/`[V]`), `voltage_alpha_beta` is the `[V]` vector handed to
/// SVPWM and `measured_speed_rpm` is observer speed in `[rpm]`. `voltage_limited`
/// staying true means the command exceeds the bus capability and the current loop
/// has lost linearity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlTelemetry {
    /// 本拍实际作用于电流环的 d/q 电流给定 `[A]`（已经过 slew 限幅）。
    /// The d/q reference actually applied to the current loop this sample `[A]`.
    pub current_reference: CurrentCommand,
    /// Park 后的实测 d/q 电流 `[A]`。
    /// Measured d/q current after Park, in `[A]`.
    pub current_dq: Dq,
    /// 电流环 PI 输出的 d/q 电压 `[V]`（已过电压圆限幅）。
    /// d/q voltage out of the current PI, in `[V]`, after circle limiting.
    pub voltage_dq: Dq,
    /// 逆 Park 后交给 SVPWM 的静止坐标系电压 `[V]`。
    /// Stationary-frame voltage in `[V]` handed to SVPWM after inverse Park.
    pub voltage_alpha_beta: AlphaBeta,
    /// 观测器给出的机械转速 `[rpm]`。
    /// Observer mechanical speed in `[rpm]`.
    pub measured_speed_rpm: f32,
    /// 本拍电压圆限幅是否生效；持续为真说明已超出母线能力。
    /// Whether the voltage circle limiter engaged this sample.
    pub voltage_limited: bool,
}

/// 硬件侧故障，决定快环是继续、关断还是锁存。
/// Hardware-side faults that decide whether the fast loop continues or trips.
///
/// 四个变体对应四条失败路径：拿不到反馈、反馈不可信、功率级已故障、算法输出被
/// 拒绝。四条路径都会让 `ControlRuntime` 关断 PWM；区分它们主要是为了遥测定位。
/// Four variants cover the four failure paths: no feedback, untrustworthy
/// feedback, a faulted power stage and a rejected algorithm output. All of them
/// disable PWM in `ControlRuntime`; the distinction is for telemetry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HardwareFault {
    /// 端口本拍没有可用反馈（ADC/编码器未就绪、采样超时）。
    /// No usable feedback this sample (ADC/encoder not ready, sampling timeout).
    FeedbackUnavailable,
    /// 反馈存在但数值非法（`NaN`/`Inf` 或母线电压不为正）。
    /// Feedback exists but is invalid (`NaN`/`Inf`, or a non-positive DC bus).
    InvalidFeedback,
    /// 功率级已进入硬件故障（刹车输入、比较器、栅极故障），不允许再输出。
    /// The power stage has tripped (break input, comparator, gate fault).
    PowerStageFault,
    /// 算法输出被拒绝（占空比非有限或越界，或端口拒绝写入）。
    /// The algorithm output was rejected (non-finite or out-of-range duty).
    OutputRejected,
}
