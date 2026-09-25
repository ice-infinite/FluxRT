//! P/PI/PD/PID、速度环和位置环控制器。
//! P/PI/PD/PID controllers, speed loop, position loop, cascade and feed-forward.
//!
//! 职责 / Responsibility:
//!   - 位置环 -> 速度环 -> d/q 电流环 -> SVPWM 的级联链路
//!   - 目标速度与目标加速度的前馈补偿
//!   - 与 C 版逐项对照、带抗饱和的 PI/PID
//!   - position -> speed -> d/q current -> SVPWM cascade
//!   - velocity and acceleration feed-forward
//!   - anti-windup PI/PID kept equivalent to the C implementation
//!
//! 架构位置 / Architecture position:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!     -> foc-control -> foc-algorithm (本文件 / this file)
//! 依赖方向 / Dependency direction:
//!   只依赖 crate 内的 `math`、`foc`、`modulation`；不依赖 HAL、RTOS 或
//!   任何具体 MCU。本文件是纯 `no_std` 数学层，状态由调用方持有。
//!   Depends only on `math`, `foc` and `modulation` inside this crate; no HAL,
//!   no RTOS, no MCU specifics. Pure `no_std` math layer, all state caller-owned.
//!
//! 实时约束 / Real-time constraints:
//!   所有 `update` 只做固定次数的标量 `f32` 运算：无分配、无阻塞、无日志、
//!   无 Mutex 等待，可在 12 kHz ADC 中断内调用。
//!   Every `update` is a fixed number of scalar `f32` operations: no allocation,
//!   blocking or logging. Safe inside the 12 kHz ADC ISR. Division by `ts` is a
//!   soft-float call on `thumbv7em-none-eabi`.
//!
//! 实时路径使用情况 / Realtime-path usage:
//!   本文件中只有 `PiState` 真正进入本工程实时路径：`foc.rs` 的 d/q 电流环和
//!   foc-control 的电流环/速度环都用它。
//!   `PState`、`PdState`、`PidState`、`SpeedLoopState`、`PositionLoopState`、
//!   `FeedForwardState`、`CascadeState` 目前只被本文件的 C 参考测试和
//!   `examples/footprint.rs` 引用，属于库级兼容实现，实时固件不调用它们。
//!   Only `PiState` is on this project's realtime path (used by `foc.rs` and by
//!   foc-control's current and speed loops). The remaining controllers are
//!   library-level ports kept for C-reference equivalence; the realtime firmware
//!   does not call them, only this file's tests and `examples/footprint.rs` do.
//!
//! 采样周期 / Sample period:
//!   本文件不含定时器和分频器，`ts` 必须等于真实调用周期；本工程中电流 PI
//!   用 `1/12000` `[s]`，速度 PI 用 `1/1000` `[s]`。
//!   No timer or divider lives here: `ts` must equal the real invocation period.
//!   Here the current PI uses `1/12000` `[s]` and the speed PI `1/1000` `[s]`;
//!   the multi-rate split is the caller's responsibility.
//!
//! 常数来源 / Constant provenance:
//!   本文件不硬编码任何增益、限幅或 `ts`，全部由 `*Param` 传入，因此这里
//!   没有需要标 `[HW]`/`[ST]` 的常数；本工程实机参数来自 foc-control
//!   `params.rs`（转写自 ST MCSDK 6.4.1 生成工程，`[ST]`）。
//!   No gain, limit or `ts` is hard-coded here; all arrive through `*Param`.
//!   The project values come from foc-control `params.rs`, transcribed from the
//!   generated ST MCSDK 6.4.1 project (`[ST]`).
//!
//! 定点说明 / Fixed point:
//!   本文件全部量都是 `f32`，不存在 Q1.15/Q1.31 定点站点。
//!   Every quantity here is `f32`; this file has no Q1.15/Q1.31 site.

use crate::math::{clamp, wrap_angle_minus_pi_to_pi};

/// 纯比例控制器的参数。
/// Parameters of the pure proportional controller.
///
/// 本结构不含 `ts`：比例项没有时间量纲，与采样周期无关。
/// No `ts` field: a proportional term carries no time dimension, so the result
/// is independent of the sample period.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PParam {
    /// 比例增益，单位为 (输出单位)/(输入单位)；例如电流误差 `[A]` 到电压 `[V]`
    /// 时为 `[V/A]`，速度误差 `[rad/s]` 到 `iq` `[A]` 时为 `[A*s/rad]`。
    /// Proportional gain in output-units per input-unit: `[V/A]` for a current
    /// loop, `[A*s/rad]` for a speed loop.
    pub kp: f32,
    /// 输出下限，与输出同量纲（通常 `[V]` 或 `[A]`）。
    /// Output lower saturation limit, in the output unit (`[V]` or `[A]`).
    pub out_min: f32,
    /// 输出上限；若误填成 `out_min > out_max`，`math::clamp` 会自动交换两者。
    /// Output upper limit. `math::clamp` silently swaps the pair when
    /// `out_min > out_max`, so a mis-ordered parameter still saturates safely
    /// instead of panicking.
    pub out_max: f32,
}

/// 纯比例控制器的运行状态。
/// Runtime state of the pure proportional controller.
///
/// 只有误差和输出，没有积分器，因此不存在积分饱和，也就不需要抗饱和回算。
/// Holds only the error and the output. Without an integrator there is nothing
/// to wind up, so no back-calculation is needed.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PState {
    /// 最近一次误差 `reference - feedback`，与输入同量纲。
    /// Last error `reference - feedback`, in the input unit.
    pub error: f32,
    /// 最近一次限幅后的输出，与 `out_min`/`out_max` 同量纲。
    /// Last clamped output, in the `out_min`/`out_max` unit.
    pub output: f32,
}

impl PState {
    /// 把误差和输出清零，用于故障复位或控制模式切换。
    /// Clears the error and the output; used on fault reset or mode change.
    ///
    /// 这里不做无扰切换：输出直接跳回 0，调用方必须先关断 PWM 或自行加斜坡。
    /// This is not a bumpless transfer: the output jumps to zero, so the caller
    /// must disable the PWM or insert its own ramp first.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 执行一次比例控制，返回限幅后的输出。
    /// Runs one proportional step and returns the clamped output.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，与 `feedback` 同量纲
    ///   feedback  - 反馈值，与 `reference` 同量纲
    ///
    /// 返回 / Returns: 已钳位到 `out_min..out_max` 的输出。
    /// The output already clamped to `out_min..out_max`.
    ///
    /// 与 C 版等价 / C reference equivalence: `p_matches_c_reference` 锁定的是
    /// “先算误差、再乘 `kp`、最后限幅”这一顺序。
    /// `p_matches_c_reference` pins the order error -> `kp*error` -> clamp.
    ///
    /// 上下文 / Context: 固定执行时间、无分配，可在 12 kHz 电流环 ISR 内调用。
    /// Fixed cost, no allocation: allowed inside the 12 kHz current-loop ISR.
    #[inline]
    pub fn update(&mut self, param: &PParam, reference: f32, feedback: f32) -> f32 {
        self.error = reference - feedback;
        // 输出只做钳位，没有积分项需要回算；`clamp` 会在 min > max 时交换边界。
        // The output is only clamped, and `clamp` swaps the bounds when
        // min > max, so a swapped parameter pair cannot produce an empty range.
        self.output = clamp(param.kp * self.error, param.out_min, param.out_max);
        self.output
    }
}

/// 带积分限幅和输出反算抗饱和的 PI 控制器参数。
/// Parameters of a PI controller with integral clamping and output
/// back-calculation anti-windup.
///
/// 抗饱和语义 / Anti-windup semantics（顺序不可改动 / do not reorder）:
///   1. `integrator += ki * ts * error`，再钳位到
///      `integrator_min..integrator_max`；
///   2. `unclamped = kp * error + integrator`，钳位到 `out_min..out_max`；
///   3. 仅当输出确实被钳位（`unclamped != output`）时，才把积分器反算为
///      `output - kp * error`，并按积分限幅再钳一次。
///   1. integrate, then clamp the integrator to `integrator_min..integrator_max`;
///   2. form `kp*error + integrator` and clamp to `out_min..out_max`;
///   3. only when the output really saturated, back-calculate the integrator to
///      `output - kp*error` and clamp it to the integral limits again.
///
/// 这个顺序、以及第 3 步用浮点比较 `unclamped != output` 判断“是否饱和”，正是
/// `pi_matches_c_reference` 通过的原因；改成正负分支判断或调整运算顺序会让该
/// 参考测试失败。README“与 C 基线”一节也要求保持 C 版抗饱和逻辑。
/// This exact order, including the floating-point `unclamped != output`
/// saturation test, is why `pi_matches_c_reference` passes. Replacing it with
/// sign-based saturation branches or reordering the arithmetic breaks that test.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PiParam {
    /// 比例增益。电流环 `[V/A]`，速度环 `[A*s/rad]`；本工程取值 `[ST]`。
    /// Proportional gain: `[V/A]` in the current loop, `[A*s/rad]` in the speed
    /// loop. This project's values are `[ST]`, from MCSDK 6.4.1.
    pub kp: f32,
    /// 积分增益，单位为 (输出单位)/(输入单位 * s)：电流环 `[V/(A*s)]`，
    /// 速度环 `[A/rad]`。与 `ts` 相乘后才得到每一步的积分增量。
    /// Integral gain in output per (input * second), so `[V/(A*s)]` for the
    /// current loop. `ki * ts` is the per-sample increment. `[ST]` derived: the
    /// MCSDK per-tick gain is kept continuous-time and re-scaled by this
    /// project's `ts`, see foc-control `params.rs`.
    pub ki: f32,
    /// 采样周期 `[s]`，必须等于真实调用周期；错配会按比例改变积分增益。
    /// Sample period `[s]`; it must equal the real invocation period, otherwise
    /// the integral gain is scaled by the mismatch factor.
    pub ts: f32,
    /// 输出电压下限 `[V]`。电流环把 SVPWM 线性区电压上限写在这里，
    /// 因此“输出限幅”同时也是 SVPWM 过调制保护。
    /// Output lower limit `[V]`. In the current loop these bounds express the
    /// SVPWM linear-range voltage ceiling, so the output clamp doubles as
    /// over-modulation protection.
    pub out_min: f32,
    /// 输出电压上限 `[V]`。
    /// Output upper limit `[V]`.
    pub out_max: f32,
    /// 积分器下限，量纲与输出相同 `[V]`；误差长期存在时限制积分累加。
    /// Integrator lower bound in the output unit `[V]`; bounds the stored
    /// integral while a persistent error remains.
    pub integrator_min: f32,
    /// 积分器上限 `[V]`。可以等于 `out_min`/`out_max`，也可以更紧以留抗饱和余量。
    /// Integrator upper bound `[V]`. It may equal `out_min`/`out_max` or be
    /// tighter to keep anti-windup headroom.
    pub integrator_max: f32,
}

/// PI 控制器的运行状态。
/// Runtime state of the PI controller.
///
/// 全部为 `f32`。注意 `integrator` 已经是“输出量纲”的值（已经含 `ki*ts` 的
/// 缩放），不是误差的累加和；`preload_output` 的反算正是建立在这个约定上。
/// All fields are `f32`. `integrator` is expressed in output units (it already
/// carries the `ki*ts` scaling), not as an accumulated error; `preload_output`
/// relies on that convention.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PiState {
    /// 积分器累加值，量纲为输出 `[V]`，已按积分限幅钳位。
    /// Accumulated integral in output units `[V]`, already clamped.
    pub integrator: f32,
    /// 最近一次误差 `reference - feedback`。
    /// Last error `reference - feedback`.
    pub error: f32,
    /// 最近一次限幅后的输出 `[V]`。
    /// Last clamped output `[V]`.
    pub output: f32,
}

impl PiState {
    /// 清零积分器、误差和输出。
    /// Clears the integrator, the error and the output.
    ///
    /// 复位后第一次 `update` 只累加一步 `ki*ts*error`，所以从零启动会有一次
    /// 比例加单步积分的阶跃。需要从已有执行器指令无扰接管时用 `preload_output`，
    /// 不要用 `reset`。
    /// After a reset the first `update` contributes only one `ki*ts*error` step,
    /// so starting from zero produces a proportional-plus-one-step transient.
    /// Use `preload_output` for a bumpless handoff instead of `reset`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 用当前执行器指令预装积分器，使控制器从已有输出开始，而不是首次更新
    /// 从零跳变。
    /// 用于无感切换进闭环时的无扰投入，对应 MCSDK 的 `SWITCH_OVER` 积分初始化。
    /// Preloads the integral term so the controller starts from an existing
    /// actuator command instead of stepping from zero on the first update.
    /// This is the bumpless handoff used at MCSDK `SWITCH_OVER`.
    ///
    /// 反算顺序 / Back-calculation order:
    ///   1. `output = clamp(desired_output, out_min, out_max)`；
    ///   2. `integrator = clamp(output - kp*error, integrator_min, integrator_max)`；
    ///   3. 再用 `clamp(kp*error + integrator, out_min, out_max)` 复核返回值。
    ///
    ///   1. clamp the requested output; 2. back-calculate and clamp the
    ///      integrator; 3. re-clamp `kp*error + integrator` for the return value.
    ///
    /// 因此当积分限幅比输出限幅更紧时，返回的不是 `desired_output`。
    /// When the integral limits are tighter than the output limits the
    /// returned value is intentionally not `desired_output`.
    ///
    /// 量纲 / Units: 本工程调用点把速度误差放在 `reference`/`feedback`
    /// （机械 `[rad/s]`），`desired_output` 是 `iq` 电流 `[A]`。
    /// At the one in-tree call site `reference`/`feedback` are mechanical
    /// `[rad/s]` speed errors and `desired_output` is the `iq` current `[A]`.
    ///
    /// 调用点 / Call sites: foc-rt-bridge 在无感切入闭环的第一个周期调用，
    /// 预装量为 `iq_ref * speed_pi_preload_ratio`；本工程 `[FW]` 默认 0.0，
    /// 即默认不做积分预装。
    /// foc-rt-bridge calls this on the first closed-loop sample with
    /// `current_iq_a * speed_pi_preload_ratio`; that `[FW]` ratio defaults to
    /// 0.0 in this project, i.e. no preload by default.
    #[inline]
    pub fn preload_output(
        &mut self,
        param: &PiParam,
        reference: f32,
        feedback: f32,
        desired_output: f32,
    ) -> f32 {
        self.error = reference - feedback;
        let proportional = param.kp * self.error;
        self.output = clamp(desired_output, param.out_min, param.out_max);
        self.integrator = clamp(
            self.output - proportional,
            param.integrator_min,
            param.integrator_max,
        );
        self.output = clamp(proportional + self.integrator, param.out_min, param.out_max);
        self.output
    }

    /// 执行一次 PI 运算：积分 -> 积分限幅 -> 输出限幅 -> 饱和时反算积分器。
    /// Runs one PI step: integrate, clamp the integral, clamp the output, and
    /// back-calculate the integrator when the output saturated.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，与 `feedback` 同量纲（电流环为 `[A]`，速度环为
    ///               `[rad/s]`）；same unit as `feedback`
    ///   feedback  - 反馈值，与 `reference` 同量纲；same unit as `reference`
    ///
    /// 返回 / Returns: 已钳位到 `out_min..out_max` 的输出。d/q 电流环中单位是
    /// `[V]`（送逆 Park/SVPWM），速度环中单位是 `iq` 参考电流 `[A]`。
    /// The output clamped to `out_min..out_max`: `[V]` in the d/q current loop
    /// (fed to inverse Park/SVPWM), the `iq` reference `[A]` in the speed loop.
    ///
    /// 与 C 版等价 / C reference equivalence: 运算顺序和浮点饱和判断都是
    /// `pi_matches_c_reference` 的判据，改动即破坏该测试。
    /// The operation order and the float saturation test are exactly what
    /// `pi_matches_c_reference` checks; changing them breaks that test.
    ///
    /// 上下文 / Context: 12 kHz 电流环 ISR 内可调用；固定执行时间、无分配、
    /// 无阻塞、无日志。
    /// Callable inside the 12 kHz current-loop ISR: fixed cost, no allocation,
    /// no blocking, no logging.
    ///
    /// 注意 / Caveat: `clamp` 会把 `NaN` 原样透传，而 `NaN != NaN` 为真，
    /// 于是 `NaN` 会被写进积分器并永久保留；`NaN/Inf` 不属于有效输入域。
    /// `clamp` passes `NaN` through and `NaN != NaN` is true, so a NaN input is
    /// written into the integrator and stays there. `NaN/Inf` are outside the
    /// documented input domain.
    #[inline]
    pub fn update(&mut self, param: &PiParam, reference: f32, feedback: f32) -> f32 {
        self.error = reference - feedback;
        let proportional = param.kp * self.error;

        // 先积分再钳位：积分限幅是第一道抗饱和防线，先于输出限幅生效。
        // Integrate first, then clamp: the integral limit is the first
        // anti-windup layer and acts before the output limit.
        self.integrator += param.ki * param.ts * self.error;
        self.integrator = clamp(self.integrator, param.integrator_min, param.integrator_max);

        let unclamped = proportional + self.integrator;
        self.output = clamp(unclamped, param.out_min, param.out_max);
        // 只有输出真的被钳位才回算积分器；否则每个周期都会重写积分状态，
        // 未饱和时也会引入额外的舍入误差。
        // Back-calculate only when the output really saturated; otherwise the
        // stored integral would be rewritten every sample and pick up extra
        // rounding noise even while linear.
        if unclamped != self.output {
            // 回算把积分器拉到“刚好让输出停在限幅上”的值，饱和解除瞬间不跳变。
            // The back-calculation moves the integrator to the value that just
            // holds the output on the limit, so releasing the limit is smooth.
            self.integrator = clamp(
                self.output - proportional,
                param.integrator_min,
                param.integrator_max,
            );
        }
        self.output
    }
}

/// 比例 + 一阶滤波微分的 PD 控制器参数。
/// Parameters of a PD controller with a first-order filtered derivative.
///
/// 微分作用在误差上而不是测量值上，所以参考值阶跃会产生微分冲击（derivative
/// kick）。这是 C 版语义，`pd_matches_c_reference` 的期望值依赖它。
/// The derivative acts on the error rather than on the measurement, so a
/// reference step produces derivative kick. That is the C semantics that
/// `pd_matches_c_reference` pins down.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdParam {
    /// 比例增益，量纲为 输出/输入（速度阻尼环常见 `[A*s/rad]`）。
    /// Proportional gain in output/input units (`[A*s/rad]` for a damping loop).
    pub kp: f32,
    /// 微分增益，量纲为 输出/(输入/s)；例如输出 `[A]`、输入 `[rad/s]` 时为
    /// `[A*s^2/rad]`，等效于为机械系统提供阻尼。
    /// Derivative gain in output per (input per second), e.g. `[A*s^2/rad]` when
    /// the output is `[A]` and the input `[rad/s]`; it supplies damping.
    pub kd: f32,
    /// 采样周期 `[s]`，用于把误差差分换算成变化率。`<= 0` 时 `update` 直接
    /// 返回 0.0，避免除零得到 `Inf`。
    /// Sample period `[s]` used to turn the error difference into a rate. When
    /// `ts <= 0.0` the update returns 0.0 instead of dividing by zero.
    pub ts: f32,
    /// 微分低通系数，逐次钳位到 `[0,1]`。1.0 表示不滤波，编码器/电流噪声会
    /// 直接乘上 `kd` 放大。
    /// Derivative low-pass coefficient, clamped to `[0,1]`. 1.0 means no
    /// filtering, so encoder or current noise is amplified by `kd` directly.
    pub derivative_alpha: f32,
    /// 输出下限，与 `kp*error + kd*derivative` 同量纲（通常 `[V]` 或 `[A]`）。
    /// Output lower limit, same unit as `kp*error + kd*derivative`.
    pub out_min: f32,
    /// 输出上限。
    /// Output upper limit.
    pub out_max: f32,
}

/// PD 控制器的运行状态。
/// Runtime state of the PD controller.
///
/// 没有积分器，所以不存在积分饱和；`initialized` 只负责抑制首次采样的微分冲击。
/// There is no integrator and therefore no windup; `initialized` only suppresses
/// the derivative kick on the very first sample.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdState {
    /// 最近一次误差。
    /// Last error.
    pub error: f32,
    /// 上一次误差，用于差分求变化率。
    /// Previous error, used for the difference quotient.
    pub last_error: f32,
    /// 一阶低通后的误差变化率，量纲为 输入/s。
    /// First-order-filtered error rate, in input units per second.
    pub derivative: f32,
    /// 最近一次限幅后的输出。
    /// Last clamped output.
    pub output: f32,
    /// 与 C 版 `int initialized` 保持布局和语义一致：0 为未初始化。
    /// Kept as `i32` to match the C `int initialized` in layout and meaning;
    /// 0 means the derivative base has not been seeded yet.
    pub initialized: i32,
}

impl PdState {
    /// 复位为全零，含 `initialized = 0`。
    /// Resets every field to zero, including `initialized`.
    ///
    /// 下一次 `update` 会用当前误差重新植入差分基准，所以复位后的第一次调用
    /// 微分项为 0、不会冲击；但 `output` 字段会立刻掉到 0，运行中调用需先关 PWM。
    /// The next `update` re-seeds the difference base from the current error, so
    /// there is no derivative kick. The `output` field does drop to 0
    /// immediately, so disable the PWM before calling this while running.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 执行一次 PD 运算：比例 + 一阶滤波微分，然后输出限幅。
    /// Runs one PD step: proportional plus filtered derivative, then clamps.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，与 `feedback` 同量纲
    ///   feedback  - 反馈值，与 `reference` 同量纲
    ///
    /// 返回 / Returns: 限幅后的输出；`param.ts <= 0.0` 时返回 0.0 且完全不改状态。
    /// Clamped output. When `param.ts <= 0.0` it returns 0.0 and leaves `error`,
    /// `derivative` and `output` untouched.
    ///
    /// 与 C 版等价 / C reference equivalence: “先把当前误差植入 `last_error`，
    /// 再做差分”的顺序使首次调用微分项恒为 0；`pd_matches_c_reference` 的
    /// 第二个期望值正是靠这个顺序算出来的。
    /// Seeding `last_error` from the current error *before* the difference
    /// quotient is what makes the first-step derivative zero, which is what
    /// `pd_matches_c_reference` expects.
    ///
    /// 上下文 / Context: 机械/速度外环通常 1 kHz；也可以在 12 kHz 内调用，
    /// 但要重新核对 `derivative_alpha` 带来的相位滞后。
    /// Usually a 1 kHz outer loop. It may run at 12 kHz, but then the phase lag
    /// introduced by `derivative_alpha` has to be re-checked.
    #[inline]
    pub fn update(&mut self, param: &PdParam, reference: f32, feedback: f32) -> f32 {
        // ts 非法时直接返回 0，避免除零得到 Inf；代价是状态保留上一次的值，
        // 调用方不应把返回的 0.0 当成“输出已归零”。
        // Guards against division by zero. The state keeps its previous values,
        // so a returned 0.0 here does not mean the stored output is zero.
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.error = reference - feedback;
        if self.initialized == 0 {
            self.last_error = self.error;
            self.initialized = 1;
        }
        let raw_derivative = (self.error - self.last_error) / param.ts;
        // alpha 逐次钳到 [0,1]：小于 0 会让一阶极点失稳，大于 1 会过冲振荡。
        // `alpha` is clamped to `[0,1]`: a negative value destabilises the pole
        // and a value above 1.0 overshoots.
        let alpha = clamp(param.derivative_alpha, 0.0, 1.0);
        // 一阶 IIR 低通（后向欧拉形式）；alpha = 1.0 时退化为无滤波差分，
        // 也就是 C 参考测试用的配置。
        // First-order IIR low-pass (backward-Euler form). `alpha = 1.0` degrades
        // to the unfiltered difference quotient, which is what the C reference
        // test uses.
        self.derivative += alpha * (raw_derivative - self.derivative);
        self.last_error = self.error;
        // PD 没有积分状态，输出饱和不会累积误差，因此这里不需要反算抗饱和。
        // A PD holds no integral state, so a saturated output cannot accumulate
        // error and no back-calculation is required.
        self.output = clamp(
            param.kp * self.error + param.kd * self.derivative,
            param.out_min,
            param.out_max,
        );
        self.output
    }
}

/// 带积分限幅和输出反算抗饱和的完整 PID 参数。
/// Parameters of the full PID with integral clamping and output
/// back-calculation anti-windup.
///
/// 抗饱和顺序与 `PiParam` 完全相同，区别只在回算时还要扣掉微分项：
/// `integrator = output - kp*error - kd*derivative`。
/// The anti-windup order is the same as `PiParam`; only the back-calculation
/// also subtracts the derivative: `output - kp*error - kd*derivative`.
///
/// `pid_matches_c_reference` 固定了运算顺序、`ki*ts*error` 的分组方式以及
/// “饱和才回算”的判断，改动其中任何一项都会让该参考测试失败。
/// `pid_matches_c_reference` pins the operation order, the `ki*ts*error`
/// grouping and the saturate-then-back-calculate rule; changing any of them
/// fails that test.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidParam {
    /// 比例增益，量纲为 输出/输入。
    /// Proportional gain in output/input units.
    pub kp: f32,
    /// 积分增益，量纲为 输出/(输入*s)；与 `ts` 相乘后才落到每一步。
    /// Integral gain in output per (input * second); `ts` turns it into a
    /// per-sample increment.
    pub ki: f32,
    /// 微分增益，量纲为 输出/(输入/s)。作用在误差上，所以目标值阶跃会带来
    /// 微分冲击。
    /// Derivative gain in output per (input per second). It acts on the error,
    /// so a reference step produces derivative kick.
    pub kd: f32,
    /// 采样周期 `[s]`；`ts <= 0.0` 时 `update` 返回 0.0 且不改状态。
    /// Sample period `[s]`; a non-positive value makes `update` return 0.0
    /// without touching the state.
    pub ts: f32,
    /// 微分低通系数，逐次钳位到 `[0,1]`；1.0 为不滤波。
    /// Derivative low-pass coefficient, clamped to `[0,1]`; 1.0 disables it.
    pub derivative_alpha: f32,
    /// 输出下限，与执行器输出同量纲（`[V]` 或 `[A]`）。
    /// Output lower limit in the actuator unit (`[V]` or `[A]`).
    pub out_min: f32,
    /// 输出上限。
    /// Output upper limit.
    pub out_max: f32,
    /// 积分器下限，量纲与输出相同；比输出限幅更紧时由回算保证两者自洽。
    /// Integrator lower bound in output units. When it is tighter than the
    /// output limits the back-calculation keeps the two consistent.
    pub integrator_min: f32,
    /// 积分器上限。
    /// Integrator upper bound.
    pub integrator_max: f32,
}

/// PID 控制器的运行状态。
/// Runtime state of the PID controller.
///
/// 全部为 `f32`；`integrator` 与 `PiState` 一样是输出量纲，不是误差累加和。
/// All fields are `f32`. As in `PiState`, `integrator` is in output units rather
/// than an accumulated error.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidState {
    /// 最近一次误差 `reference - feedback`。
    /// Last error `reference - feedback`.
    pub error: f32,
    /// 上一次误差，用于差分求变化率。
    /// Previous error, used for the difference quotient.
    pub last_error: f32,
    /// 已按积分限幅钳位的积分累加值，量纲为输出。
    /// Clamped accumulated integral, in output units.
    pub integrator: f32,
    /// 一阶低通后的误差变化率，量纲为 输入/s。
    /// First-order-filtered error rate, in input units per second.
    pub derivative: f32,
    /// 最近一次限幅后的输出。
    /// Last clamped output.
    pub output: f32,
    /// 与 C 版 `int initialized` 一致：0 表示微分差分基准尚未植入。
    /// Matches the C `int initialized`: 0 means the derivative base has not been
    /// seeded yet.
    pub initialized: i32,
}

impl PidState {
    /// 复位为全零，含积分器；下一次 `update` 的微分项从 0 开始。
    /// Resets every field including the integrator; the next `update` starts with
    /// a zero derivative term.
    ///
    /// 应在闭环使能前调用；运行中调用会让输出跳变，必须先关断 PWM 或另做斜坡。
    /// Call it before enabling the closed loop. Calling it while running produces
    /// an output step, so the PWM must be disabled or ramped first.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 执行一次 PID 运算：积分 -> 限幅 -> 微分滤波 -> 输出限幅 -> 反算抗饱和。
    /// Runs one PID step: integrate, clamp, filter the derivative, clamp the
    /// output, then back-calculate the integrator.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，与 `feedback` 同量纲
    ///   feedback  - 反馈值，与 `reference` 同量纲
    ///
    /// 返回 / Returns: 限幅后的输出；`param.ts <= 0.0` 时返回 0.0 且不改状态。
    /// Clamped output; returns 0.0 and leaves the state untouched when
    /// `param.ts <= 0.0`.
    ///
    /// 与 C 版等价 / C reference equivalence: 回算中 `- kd*derivative` 的位置和
    /// `ki*ts*error` 的分组是 `pid_matches_c_reference` 的判据，不可重排。
    /// The position of `- kd*derivative` in the back-calculation and the
    /// `ki*ts*error` grouping are what `pid_matches_c_reference` checks.
    ///
    /// 上下文 / Context: ISR 安全（固定次数标量运算、无分配、无阻塞）；可用于
    /// 12 kHz 电流环，也可用于更慢的单环速度/位置控制。
    /// ISR-safe (fixed scalar op count, no allocation or blocking). Usable in the
    /// 12 kHz current loop or in a slower single-loop speed/position controller.
    #[inline]
    pub fn update(&mut self, param: &PidParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.error = reference - feedback;
        if self.initialized == 0 {
            self.last_error = self.error;
            self.initialized = 1;
        }
        // 比例项只算一次，限幅回算复用同一个值，避免浮点重算带来的末位差异。
        // The proportional term is computed once and reused by the
        // back-calculation, so no floating-point re-evaluation difference.
        let proportional = param.kp * self.error;
        self.integrator += param.ki * param.ts * self.error;
        self.integrator = clamp(self.integrator, param.integrator_min, param.integrator_max);

        let raw_derivative = (self.error - self.last_error) / param.ts;
        let alpha = clamp(param.derivative_alpha, 0.0, 1.0);
        self.derivative += alpha * (raw_derivative - self.derivative);
        self.last_error = self.error;

        // 三项按 `比例 + 积分 + 微分` 相加；这个顺序是复现 C 参考向量的条件，
        // 交换顺序会改变浮点舍入，可能让 `pid_matches_c_reference` 失败。
        // The three terms are summed as proportional + integral + derivative; this
        // order is what reproduces the C reference vector, and swapping it changes
        // the rounding and can fail `pid_matches_c_reference`.
        let unclamped = proportional + self.integrator + param.kd * self.derivative;
        self.output = clamp(unclamped, param.out_min, param.out_max);
        if unclamped != self.output {
            // 微分项已经计入被饱和的和，回算时必须一起扣掉，否则限幅解除瞬间
            // 输出会跳变。
            // The derivative is part of the saturated sum, so it has to be
            // subtracted here as well; otherwise the output jumps when the limit
            // releases.
            self.integrator = clamp(
                self.output - proportional - param.kd * self.derivative,
                param.integrator_min,
                param.integrator_max,
            );
        }
        self.output
    }
}

/// 速度环参数：把速度误差变成 `iq` 参考电流。
/// Speed-loop parameters: turn the speed error into the `iq` reference current.
///
/// 速度环没有独立的限幅字段，直接复用内层 `PiParam`，因此
/// `pi.out_min`/`pi.out_max` 就是转矩电流限幅（本工程 `[ST]` 参考参数为
/// ±0.8 A，等于电机额定电流 `rated_current_a`）。
/// The speed loop has no limit fields of its own; it reuses the inner `PiParam`,
/// so `pi.out_min`/`pi.out_max` *are* the torque-current limit (the `[ST]`
/// reference values here are ±0.8 A, equal to the rated current).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpeedLoopParam {
    /// 内层 PI 参数。`ts` 必须是真实速度环周期（本工程 `1/1000` `[s]`），
    /// 不是电流环的 `1/12000` `[s]`；把两者搞反会让每步积分增量小 12 倍。
    /// Inner PI parameters. `ts` must be the real speed-loop period
    /// (`1/1000` `[s]` here), not the current-loop `1/12000` `[s]`; swapping them
    /// makes every integral increment 12 times smaller.
    pub pi: PiParam,
}

/// 速度环状态：内层 PI 状态加最近一次的 `iq` 参考值。
/// Speed-loop state: the inner PI state plus the last `iq` reference.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpeedLoopState {
    /// 内层 PI 状态。误差为速度 `[rad/s]`，积分器与输出为电流 `[A]`。
    /// Inner PI state. The error is a speed `[rad/s]`; the integrator and the
    /// output are currents `[A]`.
    pub pi: PiState,
    /// 最近一次 `iq` 参考电流 `[A]`，直接作为电流环 q 轴给定。
    /// Last `iq` reference current `[A]`, fed straight to the current-loop q axis.
    pub iq_ref: f32,
}

impl SpeedLoopState {
    /// 复位速度环 PI，`iq_ref` 同时归零。
    /// Resets the speed PI and zeroes `iq_ref`.
    ///
    /// 归零等价于撤销转矩指令，所以必须在 PWM 关断状态下调用，或由上层先做斜坡。
    /// Zeroing it removes the torque command, so call it with the PWM disabled or
    /// ramp the reference down in the caller first.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 由速度误差生成 `iq` 参考电流 `[A]`。
    /// Produces the `iq` reference current `[A]` from the speed error.
    ///
    /// 参数 / Parameters:
    ///   speed_ref      - 目标机械角速度 `[rad/s]`
    ///   speed_feedback - 实测机械角速度 `[rad/s]`（不是电角速度）
    ///   Both speeds are mechanical `[rad/s]`, not electrical. `[rpm]` has to be
    ///   converted first (`rad/s = rpm * pi / 30`); foc-control already does that,
    ///   so this function never sees `[rpm]`.
    ///
    /// 返回 / Returns: 由 `param.pi.out_min..out_max` 限幅后的 `iq` 参考 `[A]`。
    /// Clamped `iq` reference `[A]`; the clamp comes from `param.pi.out_*`.
    ///
    /// 调用周期 / Rate: 本类型没有内部计数器，`param.pi.ts` 必须与真实调用周期
    /// 一致。本工程的实机速度环是 foc-control 的 `SpeedLoop`（自身带 12 分频，
    /// 按 1 kHz 运行）；若改用本类型，必须由调用方提供同样的分频，否则既不能
    /// 自动保持上一次的 `iq_ref`，`ts` 也会与真实周期不符。
    /// This type holds no counter, so `param.pi.ts` must equal the real call
    /// period. The firmware's speed loop is foc-control's own `SpeedLoop`, which
    /// divides 12 kHz down to 1 kHz itself; a caller switching to this type must
    /// supply the same divider, otherwise `iq_ref` is not latched between ticks
    /// and `ts` no longer matches the real period.
    ///
    /// 上下文 / Context: 固定执行时间、无分配、无阻塞，可在 ISR 内调用。
    /// Fixed cost, no allocation, no blocking: ISR-safe.
    #[inline]
    pub fn update(&mut self, param: &SpeedLoopParam, speed_ref: f32, speed_feedback: f32) -> f32 {
        self.iq_ref = self.pi.update(&param.pi, speed_ref, speed_feedback);
        self.iq_ref
    }
}

/// 位置环参数：把位置误差变成速度目标（纯比例，无积分）。
/// Position-loop parameters: turn the position error into a speed target. Pure
/// proportional, no integral term, so there is no windup to manage.
///
/// 级联结构里阻尼由内层速度环提供，所以这里没有 `kd`；位置环的输出直接是
/// `[rad/s]` 速度给定。
/// Damping comes from the cascaded speed loop, which is why no `kd` exists here:
/// the output is used directly as a `[rad/s]` speed setpoint.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionLoopParam {
    /// 比例增益，量纲为 (`[rad/s]`)/(位置单位)；位置用 `[rad]` 表示时为 `[1/s]`，
    /// 用编码器 `[counts]` 表示时为 `([rad/s])/[counts]`。
    /// Proportional gain in (`[rad/s]`) per position unit: `[1/s]` when the
    /// position is in `[rad]`, `([rad/s])/[counts]` when it is in `[counts]`.
    pub kp: f32,
    /// 速度目标下限 `[rad/s]`，通常对应机械行程的安全速度（软限位）。
    /// Lower speed target `[rad/s]`, normally the safe speed for the mechanical
    /// travel limit.
    pub speed_min: f32,
    /// 速度目标上限 `[rad/s]`。
    /// Upper speed target `[rad/s]`.
    pub speed_max: f32,
    /// 与 C 版 `int wrap_error` 一致；非 0 表示启用角度误差环绕。
    /// Kept as `i32` to match the C `int wrap_error`; non-zero enables angle
    /// error wrapping around one revolution.
    ///
    /// 只有单圈旋转轴才应置非 0：环绕按 1 圈 = 2*pi 计算，丝杠、多圈绝对
    /// 位置会被错误折叠成最短路径。线性或多圈场合必须置 0。
    /// Enable it only for a single-turn rotary axis: the wrap assumes one turn
    /// equals 2*pi and would fold a linear or multi-turn position onto the short
    /// path. Use 0 for those cases.
    pub wrap_error: i32,
}

/// 位置环状态。
/// Position-loop state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionLoopState {
    /// 最近一次位置误差；`wrap_error != 0` 时已折叠到 `-pi..pi`。
    /// Last position error, already folded into `-pi..pi` when
    /// `wrap_error != 0`.
    pub error: f32,
    /// 最近一次限幅后的速度目标 `[rad/s]`。
    /// Last clamped speed target `[rad/s]`.
    pub speed_ref: f32,
}

impl PositionLoopState {
    /// 复位误差和速度目标。
    /// Clears the error and the speed target.
    ///
    /// 位置环是纯比例环，没有积分状态，所以复位不涉及抗饱和；但速度目标会
    /// 立刻归零，运行中调用需先把 PWM 关断或让外环接管。
    /// Being a pure proportional loop it has no integral state, so no anti-windup
    /// reset is involved. The speed target does drop to zero immediately, so
    /// disable the PWM or let the outer loop take over first.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 由位置误差生成速度目标 `[rad/s]`。
    /// Produces the speed target `[rad/s]` from the position error.
    ///
    /// 参数 / Parameters:
    ///   position_ref      - 目标位置；`wrap_error != 0` 时为 `[rad]`
    ///   position_feedback - 实测位置，与 `position_ref` 同量纲
    ///   Target and measured position share one unit; it must match `kp`.
    ///
    /// 返回 / Returns: 钳位到 `speed_min..speed_max` 的速度目标 `[rad/s]`。
    /// Speed target clamped to `speed_min..speed_max`.
    ///
    /// 环绕语义 / Wrapping: `wrap_error != 0` 时误差经
    /// `wrap_angle_minus_pi_to_pi` 折叠到 `[-pi, pi)`，跨 0/2*pi 时走最短路径。
    /// 这就是 `outer_loops_match_c_reference` 里 `0.1 - (2*pi - 0.1)` 得到
    /// `+0.2` 而不是约 `-6.08` 的原因，改动环绕方式会破坏该参考测试。
    /// With `wrap_error != 0` the error is folded into `[-pi, pi)` so crossing
    /// 0/2*pi takes the short way round; that is why the reference test expects
    /// `+0.2` instead of roughly `-6.08`.
    ///
    /// 上下文 / Context: 外环，通常 1 kHz 或更慢；无分配、无阻塞，可在 ISR 内调用。
    /// Outer loop, normally 1 kHz or slower; no allocation, ISR-safe.
    #[inline]
    pub fn update(
        &mut self,
        param: &PositionLoopParam,
        position_ref: f32,
        position_feedback: f32,
    ) -> f32 {
        let mut error = position_ref - position_feedback;
        if param.wrap_error != 0 {
            error = wrap_angle_minus_pi_to_pi(error);
        }
        self.error = error;
        // wrap_error 只影响误差计算，限幅永远作用在绝对速度目标上，两者互不耦合。
        // `wrap_error` only affects the error; the clamp always applies to the
        // absolute speed target, so the two are independent.
        self.speed_ref = clamp(param.kp * error, param.speed_min, param.speed_max);
        self.speed_ref
    }
}

/// 前馈补偿参数：由目标速度、目标加速度和偏置直接算出执行器指令。
/// Feed-forward parameters: velocity, acceleration and bias terms that directly
/// produce an actuator command.
///
/// 前馈是开环项，没有状态也没有积分器，所以不存在饱和累积；它通常与反馈环
/// 输出相加后统一限幅。量纲由 `kv`/`ka` 决定，本结构不做任何单位换算。
/// Feed-forward is open-loop with no integral state, so it cannot wind up. It is
/// normally summed with the feedback output and clamped once. Units follow from
/// `kv`/`ka`; nothing is converted here.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedForwardParam {
    /// 速度前馈增益。输出为 `[V]`、速度为 `[rad/s]` 时为 `[V*s/rad]`；
    /// 输出为 `[A]` 时对应粘滞摩擦/阻尼补偿项。
    /// Velocity feed-forward gain: `[V*s/rad]` for a `[V]` output, or the
    /// viscous-friction/damping term when the output is `[A]`.
    pub kv: f32,
    /// 加速度前馈增益。输出为 `[V]`、加速度为 `[rad/s^2]` 时为 `[V*s^2/rad]`，
    /// 用于补偿负载惯量。
    /// Acceleration feed-forward gain: `[V*s^2/rad]` for a `[V]` output and
    /// `[rad/s^2]` acceleration; it compensates load inertia.
    pub ka: f32,
    /// 恒定偏置，与输出同量纲（`[V]` 或 `[A]`），用于补偿静摩擦或已知死区。
    /// Constant bias in output units (`[V]` or `[A]`), e.g. static friction or a
    /// known dead zone.
    pub bias: f32,
    /// 输出下限，与 `bias` 同量纲。
    /// Output lower limit, same unit as `bias`.
    pub out_min: f32,
    /// 输出上限。
    /// Output upper limit.
    pub out_max: f32,
}

/// 前馈补偿状态：只保存最近一次输出。
/// Feed-forward state: only the last output is stored.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedForwardState {
    /// 最近一次限幅后的前馈输出。
    /// Last clamped feed-forward output.
    pub output: f32,
}

impl FeedForwardState {
    /// 清零输出。
    /// Zeroes the output.
    ///
    /// 没有积分器或历史差分，复位就是一次纯赋值，不涉及抗饱和或微分基准。
    /// With no integrator or history this is a plain assignment: no anti-windup
    /// or derivative base has to be re-seeded.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 计算一次前馈输出并限幅。
    /// Computes one feed-forward output and clamps it.
    ///
    /// 参数 / Parameters:
    ///   velocity_ref     - 目标速度，量纲必须与 `kv` 匹配（通常 `[rad/s]`）
    ///   acceleration_ref - 目标加速度，量纲必须与 `ka` 匹配（通常 `[rad/s^2]`）
    ///   Reference units must match `kv` and `ka`; a mismatch silently changes
    ///   the gain.
    ///
    /// 返回 / Returns: 钳位到 `out_min..out_max` 的前馈量。
    /// The feed-forward value clamped to `out_min..out_max`.
    ///
    /// 与 C 版等价 / C reference equivalence: 三项相加顺序
    /// `kv*v + ka*a + bias` 是 `feed_forward_matches_c_reference` 的判据。
    /// The `kv*v + ka*a + bias` summation order is what
    /// `feed_forward_matches_c_reference` checks.
    ///
    /// 上下文 / Context: 可在 ISR 内调用。调用方必须自己保证 `velocity_ref` 与
    /// `acceleration_ref` 已经过斜坡和限幅，否则前馈会先于反馈环饱和。
    /// ISR-safe. The caller must ramp and limit the references itself, otherwise
    /// feed-forward saturates before the feedback loop does.
    pub fn update(
        &mut self,
        param: &FeedForwardParam,
        velocity_ref: f32,
        acceleration_ref: f32,
    ) -> f32 {
        let output = param.kv * velocity_ref + param.ka * acceleration_ref + param.bias;
        self.output = clamp(output, param.out_min, param.out_max);
        self.output
    }
}

// 级联环复用基础 FOC 电流环。`foc.rs` 反过来又用本模块的 `PiState`，两个模块
// 互相引用；Rust 允许模块级互相引用，这里不存在编译期循环依赖。
// The cascade reuses the basic FOC current loop while `foc.rs` in turn uses this
// module's `PiState`. The two modules refer to each other, which Rust permits at
// module level.
use crate::foc::{FocBasicInput, FocBasicParam, FocBasicState};
use crate::modulation::SvpwmOutput;

/// 位置-速度-电流三环级联参数。
/// Parameters of the position/speed/current cascade.
///
/// 三环共用同一次 `update` 调用，本结构没有任何分频字段。若以 12 kHz 调用，
/// 速度环和电流环的 `ts` 都必须按 12 kHz 填写；要保留 1 kHz 速度环，就必须由
/// 调用方自己分频并分别调用各环。
/// All three loops advance on every `update` and no divider field exists here.
/// Calling it at 12 kHz requires both the speed and current `ts` to be set for
/// 12 kHz; keeping a 1 kHz speed loop means the caller must implement the rate
/// split itself.
///
/// 单位约定 / Units: 位置 `[rad]`，速度 `[rad/s]`，电流 `[A]`，SVPWM 前电压 `[V]`。
/// position `[rad]`, speed `[rad/s]`, current `[A]`, pre-SVPWM voltage `[V]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeParam {
    /// 位置环参数（纯 P，输出速度目标 `[rad/s]`）。
    /// Position loop (pure P, output `[rad/s]`).
    pub position: PositionLoopParam,
    /// 速度环参数（PI，输出 `iq` 参考 `[A]`）。
    /// Speed loop (PI, output the `iq` reference `[A]`).
    pub speed: SpeedLoopParam,
    /// 基础 FOC 电流环参数，含 d/q 双 PI 和 SVPWM 母线电压 `[V]`。
    /// Basic FOC current-loop parameters: the d/q PIs and the SVPWM bus voltage.
    pub foc: FocBasicParam,
    /// d 轴电流给定 `[A]`；表贴式永磁同步电机通常给 0，弱磁时给负值。
    /// d-axis current reference `[A]`: 0 for a surface PMSM, negative for flux
    /// weakening. It is a parameter, not an input, so it cannot change within a
    /// control tick.
    pub id_ref: f32,
}

/// 级联环的单次采样输入。
/// One sample of cascade inputs.
///
/// 所有量必须取自同一个采样时刻的快照；把不同时刻的角度和电流混在一起，会在
/// 高速下引入额外的 d/q 交叉耦合误差。
/// Everything must come from one ADC snapshot; mixing epochs of angle and current
/// adds avoidable d/q cross-coupling at speed.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeInput {
    /// 目标位置；`position.wrap_error != 0` 时按 `[rad]` 环绕。
    /// Target position, wrapped as `[rad]` when `position.wrap_error != 0`.
    pub position_ref: f32,
    /// 实测位置，与 `position_ref` 同量纲（`[rad]` 或 `[counts]`），必须与
    /// `position.kp` 的量纲一致。
    /// Measured position in the same unit as `position_ref`; it has to match the
    /// unit used by `position.kp`.
    pub position_feedback: f32,
    /// 实测机械角速度 `[rad/s]`，来自位置差分或观测器，不是电角速度。
    /// Measured mechanical speed `[rad/s]` from a position difference or an
    /// observer; not the electrical speed.
    pub speed_feedback: f32,
    /// A 相电流 `[A]`，由 ADC 注入采样换算得到。
    /// Phase A current `[A]`, converted from the injected ADC sample.
    pub ia: f32,
    /// B 相电流 `[A]`。
    /// Phase B current `[A]`.
    pub ib: f32,
    /// C 相电流 `[A]`。
    /// Phase C current `[A]`.
    pub ic: f32,
    /// 电角度 `[rad]` = 机械角度 * 极对数 + 零位偏置。
    /// Electrical angle `[rad]` = mechanical angle * pole pairs + offset.
    pub theta_e_rad: f32,
}

/// 三环级联状态。
/// Cascade state.
///
/// 尺寸固定、不含堆指针；`算法库实时性说明.md` 记录为 120 字节，可静态分配并由
/// 唯一控制上下文修改。
/// Fixed size with no heap pointer; `算法库实时性说明.md` records 120 bytes, so it
/// can be allocated statically and owned by a single control context.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CascadeState {
    /// 位置环状态。
    /// Position-loop state.
    pub position: PositionLoopState,
    /// 速度环状态。
    /// Speed-loop state.
    pub speed: SpeedLoopState,
    /// 基础 FOC 电流环状态，含 `id_pi`/`iq_pi` 两个 `PiState`。
    /// Basic FOC current-loop state, including both d/q `PiState`s.
    pub foc: FocBasicState,
    /// 本周期位置环给出的速度目标 `[rad/s]`。
    /// Speed target `[rad/s]` produced by the position loop this tick.
    pub speed_ref: f32,
    /// 本周期速度环给出的 `iq` 参考 `[A]`。
    /// `iq` reference `[A]` produced by the speed loop this tick.
    pub iq_ref: f32,
    /// 本周期 SVPWM 输出的三相占空比，范围 `[0,1]`。
    /// Three-phase SVPWM duty for this tick, in `[0,1]`.
    pub pwm: SvpwmOutput,
}

impl Default for CascadeState {
    /// 全零初始化，与 C 版级联复位等价。
    /// All-zero initialization, equivalent to the C cascade reset.
    ///
    /// 注意占空比不是 0 而是 `SvpwmOutput::default()` 的 0.5/0.5/0.5 中点，
    /// 即线电压为零的安全状态；栅极使能仍必须由平台层单独关断。
    /// The duty is not zero but the `SvpwmOutput::default()` 0.5 mid-point, i.e.
    /// zero line-to-line voltage. Gate enabling must still be handled separately
    /// by the platform layer.
    fn default() -> Self {
        Self {
            position: PositionLoopState::default(),
            speed: SpeedLoopState::default(),
            foc: FocBasicState::default(),
            speed_ref: 0.0,
            iq_ref: 0.0,
            pwm: SvpwmOutput::default(),
        }
    }
}

impl CascadeState {
    /// 复位三环全部状态和占空比输出。
    /// Resets all three loops and the duty outputs.
    ///
    /// 应在 PWM/栅极关断状态下调用：复位不是无扰切换，占空比会立刻回到中点。
    /// Call it with the PWM/gate drive disabled: this is not a bumpless transfer
    /// and the duty returns to the mid-point immediately.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 执行一次完整级联：位置 -> 速度 -> d/q 电流 -> 逆 Park -> SVPWM。
    /// Runs one full cascade: position -> speed -> d/q current -> inverse Park
    /// -> SVPWM.
    ///
    /// 参数 / Parameters:
    ///   input - 同一采样时刻的位置、速度、三相电流和电角度快照
    ///           one coherent snapshot of position, speed, currents and angle
    ///
    /// 返回 / Returns: 三相占空比 `[0,1]`。`foc.svpwm.v_bus <= 0` 时
    /// `svpwm_update` 退化为 0.5/0.5/0.5 的安全中点（见 `modulation.rs`）。
    /// Three-phase duty in `[0,1]`. With `foc.svpwm.v_bus <= 0` the SVPWM call
    /// degrades to the safe 0.5 mid-point, see `modulation.rs`.
    ///
    /// 时序 / Timing: 位置环和速度环的结果在本周期就被内环消费，没有额外的一拍
    /// 延迟；如果调用方按不同频率分别调用各环，必须自己保持 `iq_ref`。
    /// The outer-loop results are consumed by the inner loop in the same tick, so
    /// there is no extra one-sample delay. A caller that splits the rates must
    /// latch `iq_ref` itself.
    ///
    /// 上下文 / Context: 12 kHz ADC ISR 内可调用；无分配、无阻塞、无日志、
    /// 无 Mutex 等待。
    /// Callable inside the 12 kHz ADC ISR: no allocation, blocking, logging or
    /// mutex wait.
    ///
    /// 与 C 版等价 / C reference equivalence: 三环的嵌套顺序，以及 `id_ref`
    /// 取自参数而不是输入，是 `cascade_matches_c_reference` 的判据。
    /// The loop nesting order and the fact that `id_ref` comes from the parameter
    /// rather than the input are what `cascade_matches_c_reference` checks.
    pub fn update(&mut self, param: &CascadeParam, input: &CascadeInput) -> SvpwmOutput {
        // 位置环输出直接作为速度环给定：级联结构让速度环天然提供阻尼，
        // 因此位置环自己不需要微分项。
        // The position output is used directly as the speed setpoint: the cascade
        // gives the speed loop its damping role, so the position loop needs no
        // derivative term of its own.
        self.speed_ref =
            self.position
                .update(&param.position, input.position_ref, input.position_feedback);
        self.iq_ref = self
            .speed
            .update(&param.speed, self.speed_ref, input.speed_feedback);
        // `id_ref` 来自参数（一个周期内不变），不是来自输入；改这个来源会破坏
        // `cascade_matches_c_reference`。
        // `id_ref` comes from the parameter, constant within a tick, not from the
        // input; changing that source breaks `cascade_matches_c_reference`.
        self.pwm = self.foc.update(
            &param.foc,
            &FocBasicInput {
                ia: input.ia,
                ib: input.ib,
                ic: input.ic,
                id_ref: param.id_ref,
                iq_ref: self.iq_ref,
                theta_e_rad: input.theta_e_rad,
            },
        );
        self.pwm
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::TWO_PI;

    /// 浮点近似比较助手；容差按 C 版参考向量的打印精度选取，
    /// 纯比例/纯积分路径用 1e-6，含除法或角度环绕的路径放宽到 1e-5。
    /// Floating-point comparison helper. Tolerances match the printed precision of
    /// the C reference vectors: 1e-6 for pure proportional/integral paths, 1e-5
    /// where a division or angle wrap is involved.
    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 锁定 C 版 `Controller_P` 的参考向量：`kp=2`、误差 2 -> 4，第二次误差 10
    /// 被 `out_max=5` 钳位。测试向量取自相邻 C 模块原有测试（见 README）。
    /// Pins the C `Controller_P` vector, including the output clamp.
    #[test]
    fn p_matches_c_reference() {
        let p = PParam {
            kp: 2.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut s = PState::default();
        assert_near(s.update(&p, 3.0, 1.0), 4.0, 1e-6);
        assert_near(s.update(&p, 10.0, 0.0), 5.0, 1e-6);
    }

    /// 锁定 C 版 `Controller_PI` 的参考向量，重点是抗饱和：
    /// `e=1` 时 `2*1 + 10*0.001 = 2.01` 被 `out_max=1.0` 钳位，积分器随后被
    /// 反算到不超过 `integrator_max`；复位后 `e=0.1` 得 `0.2 + 0.001`。
    /// Pins the C `Controller_PI` vector: output clamp plus the back-calculated
    /// integrator, then the post-reset `0.201`.
    #[test]
    fn pi_matches_c_reference() {
        let p = PiParam {
            kp: 2.0,
            ki: 10.0,
            ts: 0.001,
            out_min: -1.0,
            out_max: 1.0,
            integrator_min: -0.5,
            integrator_max: 0.5,
        };
        let mut s = PiState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.0, 1e-6);
        assert!(s.integrator <= 0.5);
        s.reset();
        assert_near(s.update(&p, 0.1, 0.0), 0.201, 1e-6);
    }

    /// 无扰切换验证：用 `desired_output` 预装积分器后，立刻以零误差更新一次，
    /// 输出必须仍然是 `0.8`，即预装确实把控制器放在了已有执行器指令上。
    /// Bumpless-handoff check: after preloading, an immediate zero-error update
    /// must still return `0.8`.
    #[test]
    fn pi_preload_preserves_existing_output() {
        let p = PiParam {
            kp: 0.2,
            ki: 1.0,
            ts: 0.001,
            out_min: -1.0,
            out_max: 1.0,
            integrator_min: -1.0,
            integrator_max: 1.0,
        };
        let mut s = PiState::default();
        assert_near(s.preload_output(&p, 10.0, 10.0, 0.8), 0.8, 1e-6);
        assert_near(s.update(&p, 10.0, 10.0), 0.8, 1e-6);
    }

    /// 锁定 C 版 `Controller_PD` 的参考向量：`derivative_alpha=1.0` 表示不滤波，
    /// 第一次更新微分项为 0（得 1.0），第二次误差从 1 跳到 2 得 `1 + 0.1*100`，
    /// 最后把 `out_max` 改小以验证输出限幅。
    /// Pins the C `Controller_PD` vector: zero first-step derivative, the
    /// unfiltered step response, and the output clamp.
    #[test]
    fn pd_matches_c_reference() {
        let mut p = PdParam {
            kp: 1.0,
            kd: 0.1,
            ts: 0.01,
            derivative_alpha: 1.0,
            out_min: -100.0,
            out_max: 100.0,
        };
        let mut s = PdState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.0, 1e-6);
        assert_near(s.update(&p, 2.0, 0.0), 12.0, 1e-5);
        p.out_max = 5.0;
        assert_near(s.update(&p, 20.0, 0.0), 5.0, 1e-6);
    }

    /// 锁定 C 版 `Controller_PID` 的参考向量：首步 `1 + 2*0.01 + 0 = 1.02`，
    /// 第二步误差阶跃使微分项贡献 `0.1*100`，得 `2 + 0.06 + 10 = 12.06`；最后
    /// 把 `out_max` 压到 3.0，验证输出限幅生效且积分器不超过 `integrator_max`。
    /// Pins the C `Controller_PID` vector, including the derivative kick and the
    /// clamped integrator.
    #[test]
    fn pid_matches_c_reference() {
        let mut p = PidParam {
            kp: 1.0,
            ki: 2.0,
            kd: 0.1,
            ts: 0.01,
            derivative_alpha: 1.0,
            out_min: -10.0,
            out_max: 20.0,
            integrator_min: -1.0,
            integrator_max: 1.0,
        };
        let mut s = PidState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.02, 1e-6);
        assert_near(s.update(&p, 2.0, 0.0), 12.06, 1e-5);
        p.out_max = 3.0;
        assert_near(s.update(&p, 100.0, 0.0), 3.0, 1e-6);
        assert!(s.integrator <= 1.0);
    }

    /// 锁定 C 版速度环和位置环的参考向量。位置环部分故意把当前位置放在
    /// `2*pi - 0.1`、目标放在 `0.1`：只有启用 `wrap_error` 并按最短路径折叠，
    /// 误差才是 `+0.2`，否则会是约 `-6.08`，`speed_ref` 也会被限幅吃掉。
    /// Pins the C speed/position vectors, including the wrapped short-path error
    /// that turns `0.1 - (2*pi - 0.1)` into `+0.2` rather than roughly `-6.08`.
    #[test]
    fn outer_loops_match_c_reference() {
        let speed_param = SpeedLoopParam {
            pi: PiParam {
                kp: 0.1,
                ki: 1.0,
                ts: 0.01,
                out_min: -3.0,
                out_max: 3.0,
                integrator_min: -1.0,
                integrator_max: 1.0,
            },
        };
        let mut speed = SpeedLoopState::default();
        assert_near(speed.update(&speed_param, 10.0, 0.0), 1.1, 1e-6);
        assert_near(speed.update(&speed_param, 1000.0, 0.0), 3.0, 1e-6);

        let mut position_param = PositionLoopParam {
            kp: 2.0,
            speed_min: -5.0,
            speed_max: 5.0,
            wrap_error: 0,
        };
        let mut position = PositionLoopState::default();
        assert_near(position.update(&position_param, 2.0, 1.5), 1.0, 1e-6);
        assert_near(position.update(&position_param, 10.0, 0.0), 5.0, 1e-6);
        position_param.wrap_error = 1;
        assert_near(
            position.update(&position_param, 0.1, TWO_PI - 0.1),
            0.4,
            1e-5,
        );
        assert_near(position.error, 0.2, 1e-5);
    }

    /// 锁定 C 版 `Controller_FeedForward` 的参考向量：
    /// `0.5*4 + 0.25*2 + 1 = 3.5`，第二项把速度拉到 100 以验证 `out_max` 限幅。
    /// Pins the C feed-forward vector: the three-term sum and the output clamp.
    #[test]
    fn feed_forward_matches_c_reference() {
        let p = FeedForwardParam {
            kv: 0.5,
            ka: 0.25,
            bias: 1.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut state = FeedForwardState::default();
        assert_near(state.update(&p, 4.0, 2.0), 3.5, 1e-6);
        assert_near(state.update(&p, 100.0, 0.0), 5.0, 1e-6);
    }

    /// 锁定 C 版 `Controller_Cascade` 的全链路参考向量：
    /// 位置误差 2.0 -> `speed_ref` 4.0 -> `iq_ref` 1.5 -> `vq` 1.5 -> 合法占空比。
    /// 这里 `id_pi`/`iq_pi` 的 `ki = 0`，所以测试只覆盖比例与限幅路径，
    /// 积分抗饱和由 `pi_matches_c_reference` 单独覆盖。
    /// Pins the whole C cascade vector. With `ki = 0` in both current PIs this
    /// test covers only the proportional and limiting paths; the integral
    /// anti-windup is covered by `pi_matches_c_reference`.
    #[test]
    fn cascade_matches_c_reference() {
        use crate::modulation::SvpwmParam;

        let current_pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        let param = CascadeParam {
            position: PositionLoopParam {
                kp: 2.0,
                speed_min: -20.0,
                speed_max: 20.0,
                wrap_error: 0,
            },
            speed: SpeedLoopParam {
                pi: PiParam {
                    kp: 0.5,
                    ki: 0.0,
                    ts: 0.001,
                    out_min: -5.0,
                    out_max: 5.0,
                    integrator_min: -2.0,
                    integrator_max: 2.0,
                },
            },
            foc: FocBasicParam {
                id_pi: current_pi,
                iq_pi: current_pi,
                svpwm: SvpwmParam { v_bus: 24.0 },
            },
            id_ref: 0.0,
        };
        let input = CascadeInput {
            position_ref: 3.0,
            position_feedback: 1.0,
            speed_feedback: 1.0,
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            theta_e_rad: 0.0,
        };
        let mut state = CascadeState::default();
        let output = state.update(&param, &input);
        assert_near(state.speed_ref, 4.0, 1e-6);
        assert_near(state.iq_ref, 1.5, 1e-6);
        assert_near(state.foc.voltage_dq.d, 0.0, 1e-6);
        assert_near(state.foc.voltage_dq.q, 1.5, 1e-6);
        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
    }
}
