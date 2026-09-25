//! ADRC、非线性 `fal`、快速跟踪微分器与带宽整定。
//!
//! 职责：二阶线性自抗扰控制器（跟踪微分器 TD + 扩张状态观测器 ESO + 扰动补偿）、
//! 非线性误差函数 `fal`、韩京清式快速跟踪微分器、复用同一状态类型的非线性 ADRC 变体，
//! 以及按带宽整定的参数生成助手。
//! Responsibility: second-order linear ADRC (tracking differentiator + extended state
//! observer + disturbance compensation), the nonlinear `fal` error function, the fast tracking
//! differentiator, the nonlinear ADRC variant that reuses the same state type, and the
//! bandwidth-parameterisation tuning helper.
//!
//! 架构位置：applications/ 下的 C 代码与 RT-Thread 经 foc-rt-bridge 的 C ABI 进入
//! foc-control 的控制周期，foc-control 再调用本 crate；本文件只做与芯片无关的纯数学，
//! 依赖方向单向朝内（本文件 -> `crate::math`、`libm`），不反向依赖上层。
//! Architecture position: C code under applications/ and RT-Thread enter the control tick
//! through the foc-rt-bridge C ABI and foc-control, which calls this crate; this file is
//! chip-independent pure math whose dependencies point inward only (`crate::math`, `libm`).
//!
//! 实时性：全部是定长 `f32` 算术，无堆分配、无阻塞、无日志、无 Mutex 等待。
//! `AdrcState::update` 是固定开销路径，原则上可放进 12 kHz ADC 中断，但放入快环前必须
//! 复核输出饱和是否覆盖真实执行器边界并实测 WCET；`adrc_fal` 与 `update_nonlinear` 会调用
//! `libm::powf`，其 WCET 必须实测；`tune_adrc_bandwidth`、`new` 与 `reset` 属于初始化/设计期
//! 函数，不在 ISR 内调用。
//! Real time: fixed-size `f32` arithmetic only, no heap allocation, blocking, logging or mutex
//! wait. `AdrcState::update` is fixed-cost and may go into the 12 kHz ADC interrupt, but its
//! output saturation must cover the real actuator range and its WCET must be measured first;
//! `adrc_fal` and `update_nonlinear` call `libm::powf`, so their WCET must be measured;
//! `tune_adrc_bandwidth`, `new` and `reset` are setup- or design-time helpers, not ISR code.
//!
//! 数值格式：本文件不含定点量，所有量都是 `f32` 的 SI 值，这里不做任何 Q1.15/Q1.31 换算；
//! ST MCSDK 的定点增益换算留在 C 侧参考与 foc-control 的参数转换里。
//! Numeric format: no fixed-point quantity appears in this file - every value is an `f32` SI
//! value and no Q1.15/Q1.31 scaling happens here; the MCSDK fixed-point gain conversion stays
//! on the C reference side and in the foc-control parameter conversion.
//!
//! 常数来源：本检出树内没有对应的 C 基线源文件（已检索 `*.c`），因此这些整定常数的物理
//! 出处无法在树内核实，只能确认它们被 `adrc_matches_c_reference`、`tuning_matches_c_reference`
//! 等测试锁定；改名或改形式会破坏与 C 基线的数值可互换性。
//! Provenance: the C baseline sources are absent from this checkout (searched `*.c`), so the
//! physical origin of these tuning constants cannot be verified in-tree; they are only known to
//! be pinned by tests such as `adrc_matches_c_reference` and `tuning_matches_c_reference`, and
//! changing their form breaks numerical interchangeability with the C baseline.

use crate::math::clamp;

/// 二阶线性 ADRC 参数；状态在 `AdrcState`，本结构只放整定值。
/// Second-order linear ADRC parameters; state lives in `AdrcState` and this struct holds tuning
/// values only.
///
/// 量纲约定：设被控量单位为 `[U]`（速度环 `[U] = [rad/s]`，电流环 `[U] = [A]`），则
/// `v1`/`z1` 为 `[U]`、`v2`/`z2` 为 `[U/s]`、`z3` 为 `[U/s^2]`；各字段单位见下。
/// `[U]` 由使用本结构的控制环决定，本模块不做单位换算，调用方必须保证整定值与真实物理量一致。
/// Dimension convention: with plant output unit `[U]` (`[rad/s]` for a speed loop, `[A]` for a
/// current loop), `v1`/`z1` are `[U]`, `v2`/`z2` are `[U/s]` and `z3` is `[U/s^2]`; per-field
/// units are listed below. The caller decides what `[U]` is and must keep the tuning consistent
/// with the physical quantity, because this module performs no unit conversion.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcParam {
    /// 采样周期 `[s]`，必须等于真实调用周期；`<= 0` 时 `update` 返回 0.0 且不修改状态。
    /// Sample period `[s]`, which must equal the real call period; `<= 0` makes `update`
    /// return 0.0 without touching the state.
    pub ts: f32,
    /// TD 速率系数 `[1/s^2]`：同一系数兼作刚度项与阻尼项，量纲推导见 `update`。
    /// TD rate coefficient `[1/s^2]`: the same coefficient acts as both the stiffness and the
    /// damping term, see `update` for the dimensional reasoning.
    pub td_r: f32,
    /// ESO 一阶增益 `[1/s]`（按带宽整定时为 `3*wo`）。
    /// ESO first-order gain `[1/s]` (`3*wo` under bandwidth tuning).
    pub beta01: f32,
    /// ESO 二阶增益 `[1/s^2]`（按带宽整定时为 `3*wo^2`）。
    /// ESO second-order gain `[1/s^2]` (`3*wo^2` under bandwidth tuning).
    pub beta02: f32,
    /// ESO 三阶增益 `[1/s^3]`（按带宽整定时为 `wo^3`）。
    /// ESO third-order gain `[1/s^3]` (`wo^3` under bandwidth tuning).
    pub beta03: f32,
    /// 误差比例增益 `[1/s^2]`（按带宽整定时为 `wc^2`）。
    /// Error proportional gain `[1/s^2]` (`wc^2` under bandwidth tuning).
    pub kp: f32,
    /// 误差微分增益 `[1/s]`（按带宽整定时为 `2*wc`）。
    /// Error derivative gain `[1/s]` (`2*wc` under bandwidth tuning).
    pub kd: f32,
    /// 输入增益 `[U/(s^2 * 输出单位)]`，把控制量换算成被控量的二阶导；0.0 会被当作 1.0。
    /// Input gain `[U/(s^2 * output unit)]` converting the control effort into the second
    /// derivative of the plant output; a 0.0 value is treated as 1.0.
    pub b0: f32,
    /// 输出下限，单位同本控制环的输出量。
    /// Lower output limit, in the output unit of this control loop.
    pub out_min: f32,
    /// 输出上限，单位同上；`math::clamp` 在 min > max 时会自动交换两者。
    /// Upper output limit, same unit; `math::clamp` silently swaps the two bounds when min > max.
    pub out_max: f32,
}

/// ADRC 实时状态；线性版与非线性版共用此类型，字段含义相同。
/// ADRC runtime state shared by the linear and the nonlinear variant, with identical field meaning.
///
/// 全部字段由 `update`/`update_nonlinear` 原地更新，不含堆指针；对象大小见 `算法库实时性说明.md`
/// 中的 `AdrcState`（32 字节），可静态创建后由唯一控制上下文持有。
/// Every field is updated in place by `update`/`update_nonlinear` and there is no heap pointer;
/// the object size is the 32 bytes recorded for `AdrcState` in `算法库实时性说明.md`, so it can be
/// statically created and owned by a single control context.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcState {
    /// TD 平滑后的参考 `[U]`。
    /// TD-smoothed reference `[U]`.
    pub v1: f32,
    /// 参考的一阶导 `[U/s]`。
    /// First derivative of the reference `[U/s]`.
    pub v2: f32,
    /// 被控量估计 `[U]`。
    /// Estimated plant output `[U]`.
    pub z1: f32,
    /// 被控量一阶导估计 `[U/s]`。
    /// Estimated first derivative of the plant output `[U/s]`.
    pub z2: f32,
    /// 总扰动估计 `[U/s^2]`，包含模型误差、未建模动态和外部负载。
    /// Total disturbance estimate `[U/s^2]`, covering model error, unmodelled dynamics and
    /// external load.
    pub z3: f32,
    /// 最近一次的跟踪误差 `v1 - z1` `[U]`，供诊断与上层使用。
    /// Last tracking error `v1 - z1` `[U]`, exposed for diagnostics and upper layers.
    pub error: f32,
    /// 最近一次的未补偿控制量 `[U/s^2]`（扰动补偿并除以 `b0` 之前）。
    /// Last uncompensated control effort `[U/s^2]`, before disturbance compensation and division
    /// by `b0`.
    pub u0: f32,
    /// 最近一次的限幅输出，单位为本控制环的输出单位。
    /// Last saturated output, in the output unit of this control loop.
    pub output: f32,
}

impl AdrcState {
    /// 清零全部状态，等价于 `*self = Self::default()`。
    /// Clears every state field, equivalent to `*self = Self::default()`.
    ///
    /// 停机、控制模式切换或控制器替换时必须调用：`z3` 里保存着上一工况的扰动估计，
    /// 不清理会把旧工况的负载/摩擦估计直接注入新工况的第一拍。
    /// Must be called on stop, control-mode switch or controller swap: `z3` holds the previous
    /// disturbance estimate and keeping it injects the old load and friction estimate into the
    /// first tick of the new operating point.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一拍二阶线性 ADRC，返回限幅后的控制量。
    /// Advances one tick of the second-order linear ADRC and returns the saturated output.
    ///
    /// 参数：`reference` 与 `feedback` 都是 `[U]`；`param.ts` 必须等于真实调用周期 `[s]`。
    /// Parameters: `reference` and `feedback` are `[U]`; `param.ts` must equal the real call
    /// period `[s]`.
    ///
    /// 返回：被限幅到 `[out_min, out_max]` 的控制量；`ts <= 0` 时返回 0.0，这是静默零输出
    /// 而不是错误码，调用方必须自己校验参数。
    /// Returns the output limited to `[out_min, out_max]`; with `ts <= 0` it returns 0.0, which is
    /// a silent zero output rather than an error code, so parameter validation is the caller's job.
    ///
    /// 上下文：定长算术、无分配、无阻塞，可用于 12 kHz ADC 中断；但放入快环前必须复核
    /// 限幅是否覆盖真实执行器边界并实测 WCET。ESO 带宽越高对采样噪声越敏感，`ts` 越大
    /// 越容易数值发散，`wo*ts` 必须由调用方限制。
    /// Context: fixed-cost arithmetic with no allocation or blocking, usable from the 12 kHz ADC
    /// interrupt, but the limits must cover the real actuator range and the WCET must be measured.
    /// A higher ESO bandwidth is more sensitive to sampling noise, and a larger `ts` diverges more
    /// easily, so `wo*ts` must be bounded by the caller.
    pub fn update(&mut self, param: &AdrcParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }

        // b0 为 0.0 时替换成 1.0，避免 (u0 - z3) / b0 变成 inf/NaN；代价是漏配 b0 不会报错，
        // 只会让扰动补偿的缩放与实际物理增益不一致。
        // A b0 of 0.0 is replaced by 1.0 so (u0 - z3) / b0 cannot become inf/NaN; the price is
        // that a missing b0 is never reported and only scales the compensation wrongly.
        let b0 = if param.b0 == 0.0 { 1.0 } else { param.b0 };
        // 跟踪微分器的一步前向欧拉：同一个 td_r 同时出现在刚度项和阻尼项，因此 td_r 的量纲是
        // `[1/s^2]`（由 `td_r * [U] -> [U/s^2]` 推出），它不等于"TD 带宽"本身，闭环极点是
        // `s^2 + 2*td_r*s + td_r = 0` 的根而不是 `-td_r` 二重极点。改动这一形式会改变
        // `adrc_matches_c_reference` 的断言值。
        // One forward-Euler step of the tracking differentiator: the same td_r appears in the
        // stiffness and the damping term, so td_r is `[1/s^2]` (from `td_r * [U] -> [U/s^2]`) and
        // is not the TD bandwidth itself; the closed-loop poles are the roots of
        // `s^2 + 2*td_r*s + td_r = 0`, not a double pole at `-td_r`. Changing this form moves the
        // values asserted by `adrc_matches_c_reference`.
        let td_acc = param.td_r * (reference - self.v1) - 2.0 * param.td_r * self.v2;
        self.v1 += param.ts * self.v2;
        self.v2 += param.ts * td_acc;

        // ESO 误差取 z1 - feedback（而不是教材常见的 y - z1），所以下面三个 beta 项带负号；
        // 两者只差整体符号，但改符号会让观测器发散。
        // The ESO error is z1 - feedback rather than the textbook y - z1, hence the negative beta
        // terms below; the two differ only by an overall sign, and flipping it diverges.
        let eso_error = self.z1 - feedback;
        let z1_dot = self.z2 - param.beta01 * eso_error;
        // 这里的 self.output 是上一拍的值（本拍 output 在函数末尾才写出），即 ESO 使用的是
        // 滞后一拍、且已限幅的控制量；改成先算 output 再用当拍值会改变
        // `adrc_matches_c_reference` 的断言值，等于改变与 C 基线可互换的离散形式。
        // self.output here is the previous tick's value because this tick's output is only written
        // at the end, so the ESO consumes a one-sample-delayed, already saturated effort; computing
        // output first and using the current tick would move the `adrc_matches_c_reference`
        // assertions and change the discrete form shared with the C baseline.
        let z2_dot = self.z3 - param.beta02 * eso_error + b0 * self.output;
        let z3_dot = -param.beta03 * eso_error;
        self.z1 += param.ts * z1_dot;
        self.z2 += param.ts * z2_dot;
        self.z3 += param.ts * z3_dot;

        self.error = self.v1 - self.z1;
        // 误差反馈用 TD 输出的 v1/v2，而不是原始 reference 与 feedback 之差，这样参考阶跃
        // 先被 TD 平滑，微分项不会把阶跃放大成冲击。
        // The error feedback uses the TD outputs v1/v2 instead of raw reference and feedback, so a
        // reference step is smoothed by the TD first and the derivative term cannot amplify it
        // into a kick.
        self.u0 = param.kp * (self.v1 - self.z1) + param.kd * (self.v2 - self.z2);
        // 扰动补偿：先减去总扰动估计 z3，再除以输入增益 b0；clamp 是本函数唯一的限幅点，
        // 也是防止扰动估计把执行器推出物理边界的最后一道约束（注意 clamp 在 out_min > out_max
        // 时会自动交换，配错的限幅不会报错）。
        // Disturbance compensation: subtract the total disturbance estimate z3 and divide by the
        // input gain b0; clamp is the only limiting point in this function and the last guard that
        // keeps the actuator inside its physical range (note that clamp swaps the bounds silently
        // when out_min > out_max).
        self.output = clamp((self.u0 - self.z3) / b0, param.out_min, param.out_max);
        self.output
    }
}

/// `adrc_fal` 的参数。
/// Parameters of `adrc_fal`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFalParam {
    /// 非线性度（无量纲），有效域 `[0,1]`；超出范围被静默截断，不报错。
    /// Nonlinearity exponent (dimensionless) with valid range `[0,1]`; out-of-range values are
    /// silently clamped and never reported.
    pub alpha: f32,
    /// 线性区半宽 `[U]`（`U` 为误差单位，例如速度环的 `[rad/s]`）；`<= 0` 时取 `1e-6`。
    /// Half-width of the linear region `[U]` (`U` being the error unit, e.g. `[rad/s]`);
    /// a value `<= 0` is replaced by `1e-6`.
    pub delta: f32,
}

/// 三值符号函数：正数返回 1.0，负数返回 -1.0，其余（0.0、-0.0、NaN）返回 0.0。
/// Three-valued sign: 1.0 for positive, -1.0 for negative and 0.0 for everything else
/// (0.0, -0.0 and NaN, because every comparison with NaN is false).
///
/// 不能用 `f32::signum()` 替代：后者对 ±0.0 返回 ±1.0，会让 `fal` 和快速 TD 的分段判据在
/// 零点跳变。纯算术且 `#[inline]`，可在实时路径调用。
/// Do not replace it with `f32::signum()`, which returns ±1.0 for ±0.0 and would step the
/// piecewise decisions inside `fal` and the fast TD at zero. Pure arithmetic and `#[inline]`, so
/// it is usable on the real-time path.
#[inline]
fn sign(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// ADRC 非线性误差函数 `fal`：小误差按线性放大，大误差按幂律压缩。
/// The ADRC nonlinear error function `fal`: linear for small errors and power-law compressed for
/// large ones.
///
/// 参数：`error` `[U]`；`param.alpha` 无量纲、`param.delta` `[U]`。
/// Parameters: `error` in `[U]`; `param.alpha` is dimensionless and `param.delta` is `[U]`.
///
/// 返回 `[U^alpha]`，不是 `[U]`。线性支除以 `delta^(1-alpha)`（增益 `delta^(alpha-1)`）后
/// 量纲同样是 `[U^alpha]`，两支在 `|error| = delta` 处连续且都等于 `delta^alpha * sign(error)`，
/// 所以线性区不是"截断"而是"接续"，作用是消除 `|e|^alpha`（`alpha < 1`）在零点斜率无穷大的问题。
/// Returns `[U^alpha]`, not `[U]`: the linear branch divides by `delta^(1-alpha)` (gain
/// `delta^(alpha-1)`) and is dimensionally `[U^alpha]` too, so both branches are continuous at
/// `|error| = delta` with the common value `delta^alpha * sign(error)`. The linear region therefore
/// continues rather than truncates the curve and removes the infinite slope that `|e|^alpha` has at
/// zero for `alpha < 1`.
///
/// 由于输出量纲含 `alpha`，非线性 ADRC 的 `kp`/`kd` 整定值不能直接照搬线性版的数值。
/// Because the output unit carries `alpha`, the `kp`/`kd` values of a nonlinear ADRC cannot be
/// copied from the linear version.
///
/// 上下文：调用 `libm::powf`（软件实现，两条分支各自只调用一次），WCET 必须实测；无分配、
/// 无阻塞。本函数没有定点版本，不存在 Q1.15/Q1.31 实现。
/// Context: it calls `libm::powf` (a software implementation, once per branch), so its WCET must
/// be measured; no allocation or blocking. There is no fixed-point variant, hence no Q1.15/Q1.31
/// implementation of this function.
pub fn adrc_fal(error: f32, param: &AdrcFalParam) -> f32 {
    // alpha 超出 `[0,1]` 被静默截断：配错只会改变非线性度，不会报错。
    // alpha is silently clamped into `[0,1]`: a misconfiguration only changes the nonlinearity
    // and is never reported.
    let alpha = clamp(param.alpha, 0.0, 1.0);
    // delta <= 0 被替换为 1e-6：既避免 `delta^(1-alpha)` 为 0 造成除零，也避免线性区退化成
    // 单点。测试 `fal_matches_c_reference_and_sanitizes_parameters` 用 delta = 0.0、alpha = 2.0
    // 锁定"非法参数仍要给出有限输出"这一行为。
    // A delta <= 0 is replaced by 1e-6 so `delta^(1-alpha)` cannot divide by zero and the linear
    // region does not collapse to a point; the test
    // `fal_matches_c_reference_and_sanitizes_parameters` pins this finite-output-for-illegal-input
    // behaviour with delta = 0.0 and alpha = 2.0.
    let delta = if param.delta <= 0.0 {
        1.0e-6
    } else {
        param.delta
    };
    let abs_error = error.abs();
    if abs_error <= delta {
        // 线性支：增益 `delta^(alpha-1)`，在 `|e| = delta` 处与幂律支连续。
        // Linear branch with gain `delta^(alpha-1)`, continuous with the power-law branch at
        // `|e| = delta`.
        error / libm::powf(delta, 1.0 - alpha)
    } else {
        // 幂律支：`|e|^alpha * sign(e)`。NaN 误差不在有效输入域内，且不会在这里被清零，
        // 而是继续以 NaN 传播（NaN 与任何值比较均为 false，因此走这一支）。
        // Power-law branch: `|e|^alpha * sign(e)`. A NaN error is outside the documented valid
        // input domain and is not zeroed here but propagated as NaN (NaN takes this branch because
        // it compares false against everything).
        libm::powf(abs_error, alpha) * sign(error)
    }
}

/// 快速跟踪微分器（韩京清离散最速控制形式）的参数。
/// Parameters of the fast tracking differentiator (Han's discrete time-optimal form).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFastTdParam {
    /// 积分步长 `[s]`，必须等于真实调用周期；`<= 0` 时 `update` 返回 0.0 且不修改状态。
    /// Integration step `[s]`, which must equal the real call period; `<= 0` makes `update`
    /// return 0.0 without touching the state.
    pub ts: f32,
    /// 最大加速度 `[U/s^2]`（`U` 为参考量单位）；越大跟踪越快，也越容易抖振与过冲。
    /// Maximum acceleration `[U/s^2]`; a larger value tracks faster and chatters more.
    pub r: f32,
    /// 滤波因子（判据时间尺度）`[s]`：只进入最速控制的分段判据，不参与积分；
    /// 过小会让分段判据在噪声下频繁跳变，工程上通常取 `ts` 的数倍。
    /// Filtering factor (decision time scale) `[s]`: it only enters the piecewise time-optimal law
    /// and is not used for integration. Too small a value makes the decision jump under noise, so
    /// it is typically a small multiple of `ts`.
    pub h0: f32,
}

/// 快速跟踪微分器状态：`v1` 是平滑参考，`v2` 是它的一阶导。
/// Fast tracking differentiator state: `v1` is the smoothed reference and `v2` its derivative.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcFastTdState {
    /// 平滑后的参考 `[U]`，`update` 的返回值就是这个量。
    /// Smoothed reference `[U]`; this is what `update` returns.
    pub v1: f32,
    /// 参考的一阶导 `[U/s]`，通常作为 ADRC 的 `v2` 或前馈项使用。
    /// First derivative of the reference `[U/s]`, normally used as ADRC's `v2` or as a
    /// feed-forward term.
    pub v2: f32,
    /// 本拍算出的最速控制加速度 `[U/s^2]`，保留在状态里便于观测与诊断。
    /// Time-optimal acceleration computed this tick `[U/s^2]`, kept in the state for inspection
    /// and diagnostics.
    pub fh: f32,
}

impl AdrcFastTdState {
    /// 以给定参考构造状态：`v1 = initial_ref`、`v2 = 0`、`fh = 0`。
    /// Builds the state with `v1 = initial_ref`, `v2 = 0` and `fh = 0`.
    ///
    /// 让 `v1` 从当前参考起步可以避免上电第一拍出现 `reference - 0` 的巨大初始误差，
    /// 否则最速控制会先给出一段大幅加速，形成启动冲击；属于初始化期函数，不在 ISR 内调用。
    /// Starting `v1` at the current reference avoids the huge first-tick `reference - 0` error
    /// that would otherwise make the time-optimal law produce a large startup acceleration kick;
    /// setup-time function, not called from the ISR.
    pub fn new(initial_ref: f32) -> Self {
        Self {
            v1: initial_ref,
            ..Self::default()
        }
    }

    /// 复位到新参考，等价于 `*self = Self::new(reference)`；停机或模式切换时调用。
    /// Resets to a new reference, equivalent to `*self = Self::new(reference)`; call on stop or
    /// mode switch.
    pub fn reset(&mut self, reference: f32) {
        *self = Self::new(reference);
    }

    /// 推进一拍最速跟踪微分器，返回平滑后的参考 `v1` `[U]`。
    /// Advances one tick of the time-optimal TD and returns the smoothed reference `v1` `[U]`.
    ///
    /// 返回：`v1`；参数非法（`ts`/`r`/`h0` 任一 `<= 0`）时返回 0.0 且状态保持不变，
    /// 由 `fast_td_matches_c_reference_and_rejects_invalid_timing` 锁定。
    /// Returns `v1`; with an illegal parameter (any of `ts`/`r`/`h0` `<= 0`) it returns 0.0 and
    /// leaves the state untouched, as pinned by
    /// `fast_td_matches_c_reference_and_rejects_invalid_timing`.
    ///
    /// 上下文：定长算术、无分配，每拍一次 `libm::sqrtf`，WCET 需实测后决定能否放进 12 kHz 快环；
    /// 导数 `v2` 由调用方从状态读取，不由返回值给出。
    /// Context: fixed-cost arithmetic with no allocation and one `libm::sqrtf` per tick, so its
    /// WCET must be measured before it goes into the 12 kHz loop; the derivative `v2` is read from
    /// the state by the caller and is not part of the return value.
    pub fn update(&mut self, param: &AdrcFastTdParam, reference: f32) -> f32 {
        if param.ts <= 0.0 || param.r <= 0.0 || param.h0 <= 0.0 {
            return 0.0;
        }

        // d = r * h0^2 是 h0 时间内的最大位移尺度 `[U]`，用作最速控制的分段阈值；
        // 上面的守卫已排除 r、h0 <= 0，因此 d != 0，后面的 a / d 不会除零。
        // d = r * h0^2 is the maximum displacement scale over h0 in `[U]` and serves as the
        // piecewise threshold; the guard above excludes r, h0 <= 0 so d != 0 and a / d is safe.
        let d = param.r * param.h0 * param.h0;
        let a0 = param.h0 * self.v2;
        let y = self.v1 - reference + a0;
        // a1 = sqrt(d*(d + 8|y|)) 是最速控制解析解的判别量，量纲为 `[U]`。
        // a1 = sqrt(d*(d + 8|y|)) is the discriminator of the analytic time-optimal
        // solution, in `[U]`.
        let a1 = libm::sqrtf(d * (d + 8.0 * y.abs()));
        let a2 = a0 + sign(y) * (a1 - d) * 0.5;
        // sy/sa 由三值 sign 构成的区间选择器；sa 相当于 |a| < d 的判据，必须在边界上给出 0/1
        // 而不是 ±1，否则分段律会在切换点跳变。
        // sy/sa are interval selectors built from the three-valued sign; sa is effectively the
        // test |a| < d and must yield 0/1 rather than ±1 at the boundary, otherwise the piecewise
        // law jumps at the switching point.
        let sy = (sign(y + d) - sign(y - d)) * 0.5;
        let a = (a0 + y - a2) * sy + a2;
        let sa = (sign(a + d) - sign(a - d)) * 0.5;
        // 最速控制律：|a| < d 时取线性段 -r*a/d，否则取饱和段 -r*sign(a)，量纲都是 `[U/s^2]`。
        // Time-optimal law: the linear segment -r*a/d for |a| < d and the saturated segment
        // -r*sign(a) otherwise, both in `[U/s^2]`.
        self.fh = -param.r * (a / d - sign(a)) * sa - param.r * sign(a);
        // 积分只用真实步长 ts，h0 不参与积分；把 h0 当积分步长会改变数值并破坏
        // `fast_td_matches_c_reference_and_rejects_invalid_timing`。
        // Integration uses the real step ts only and never h0; using h0 as the integration step
        // changes the numbers and breaks
        // `fast_td_matches_c_reference_and_rejects_invalid_timing`.
        self.v1 += param.ts * self.v2;
        self.v2 += param.ts * self.fh;
        self.v1
    }
}

/// 非线性 ADRC 参数：线性参数集加两条误差通道各自的 `fal` 配置。
/// Nonlinear ADRC parameters: the linear parameter set plus one `fal` configuration
/// per error channel.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcNonlinearParam {
    /// 线性部分参数，`ts`、`td_r`、`beta01..03`、`b0` 和限幅全部复用。
    /// The linear parameter set; `ts`, `td_r`, `beta01..03`, `b0` and the limits are all reused.
    pub base: AdrcParam,
    /// 跟踪误差通道 `e1 = v1 - z1` 的 `fal` 参数（C 参考测试向量用 `alpha = 0.5`）。
    /// `fal` configuration of the tracking error channel `e1 = v1 - z1` (the C reference vector
    /// uses `alpha = 0.5`).
    pub e1_fal: AdrcFalParam,
    /// 误差微分通道 `e2 = v2 - z2` 的 `fal` 参数（C 参考测试向量用 `alpha = 0.25`）。
    /// `fal` configuration of the derivative error channel `e2 = v2 - z2` (the C reference vector
    /// uses `alpha = 0.25`).
    pub e2_fal: AdrcFalParam,
}

impl AdrcState {
    /// 推进一拍非线性 ADRC，返回限幅后的控制量。
    /// Advances one tick of the nonlinear ADRC and returns the saturated output.
    ///
    /// 与 `update` 共用同一个 `AdrcState` 类型和字段语义（TD 状态、ESO 状态、上一拍输出），
    /// 因此两种形式可以在同一对象上切换；但切换不会复位任何状态，新形式会继续使用 `z3` 和
    /// 上一拍 `output`，模式切换应显式调用 `reset()`。
    /// Shares the `AdrcState` type and field semantics with `update` (TD state, ESO state, previous
    /// output), so both forms can run on the same object; however a switch resets nothing and the
    /// new form keeps consuming `z3` and the previous `output`, so a mode change should call
    /// `reset()` explicitly.
    ///
    /// 参数、返回与失败语义同 `update`：入参都是 `[U]`，`base.ts <= 0` 时返回 0.0 且不改状态。
    /// Parameters, return value and failure semantics match `update`: the inputs are `[U]` and a
    /// `base.ts <= 0` returns 0.0 without touching the state.
    ///
    /// WCET：每次调用执行 2 次 `adrc_fal`，每条通道最多 1 次 `libm::powf`，没有循环；
    /// 实测前不要放进 12 kHz 快环。
    /// WCET: two `adrc_fal` calls per invocation with at most one `libm::powf` each and no loop;
    /// measure it before using this in the 12 kHz loop.
    pub fn update_nonlinear(
        &mut self,
        param: &AdrcNonlinearParam,
        reference: f32,
        feedback: f32,
    ) -> f32 {
        let base = &param.base;
        if base.ts <= 0.0 {
            return 0.0;
        }

        // 与线性版相同的 b0 保护：0.0 被替换为 1.0，避免除以零。
        // The same b0 protection as the linear version: 0.0 becomes 1.0 to avoid division by zero.
        let b0 = if base.b0 == 0.0 { 1.0 } else { base.b0 };
        // TD 与线性版逐字相同（同一个 td_r 兼作刚度与阻尼项，量纲 `[1/s^2]`）。
        // The TD is identical to the linear version (one td_r acting as both stiffness and
        // damping, unit `[1/s^2]`).
        let td_acc = base.td_r * (reference - self.v1) - 2.0 * base.td_r * self.v2;
        self.v1 += base.ts * self.v2;
        self.v2 += base.ts * td_acc;

        // ESO 仍使用 z1 - feedback 的误差符号，也仍然读取上一拍的 self.output。
        // The ESO keeps the z1 - feedback error sign and still reads the previous self.output.
        let eso_error = self.z1 - feedback;
        self.z1 += base.ts * (self.z2 - base.beta01 * eso_error);
        self.z2 += base.ts * (self.z3 - base.beta02 * eso_error + b0 * self.output);
        self.z3 += base.ts * (-base.beta03 * eso_error);

        // e2 只在 u0 里使用，不写回状态；`error` 字段只保存 e1，供上层诊断。
        // e2 is used inside u0 only and never written back; the `error` field keeps e1 alone for
        // upper-layer diagnostics.
        let e1 = self.v1 - self.z1;
        let e2 = self.v2 - self.z2;
        self.error = e1;
        // 非线性误差反馈：用 fal(e1)/fal(e2) 取代线性的 kp*e1/kd*e2。因为 fal 的输出量纲是
        // `[U^alpha]`（见 `adrc_fal`），这里的 kp/kd 数值与线性版不可直接互换。
        // Nonlinear error feedback: fal(e1)/fal(e2) replace the linear kp*e1/kd*e2; since fal
        // returns `[U^alpha]` (see `adrc_fal`), these kp/kd values are not interchangeable with the
        // linear version.
        self.u0 = base.kp * adrc_fal(e1, &param.e1_fal) + base.kd * adrc_fal(e2, &param.e2_fal);
        // 扰动补偿与限幅语义与线性版完全一致。
        // Disturbance compensation and saturation semantics are identical to the linear version.
        self.output = clamp((self.u0 - self.z3) / b0, base.out_min, base.out_max);
        self.output
    }
}

/// 带宽整定输入：把控制带宽与观测带宽映射成 `AdrcParam` 的增益。
/// Bandwidth tuning input: maps the control and observer bandwidths onto `AdrcParam` gains.
///
/// 本结构由设计期/初始化期填写，来源是整定意图而不是实测值；`ts`、`b0`、`td_r` 与限幅
/// 必须与真实控制环一致，否则得到的增益只是纸面数值。
/// It is filled at design or setup time from tuning intent rather than measurements; `ts`, `b0`,
/// `td_r` and the limits must match the real control loop or the gains stay on paper only.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdrcTuningParam {
    /// 采样周期 `[s]`，原样透传给 `AdrcParam`；本函数不校验 `wo*ts` 的离散稳定性。
    /// Sample period `[s]`, passed through to `AdrcParam`; this helper does not check the discrete
    /// stability of `wo*ts`.
    pub ts: f32,
    /// 控制带宽 `wc` `[rad/s]`；负值被静默抬到 0.0（得到零增益，不报错）。
    /// Control bandwidth `wc` `[rad/s]`; a negative value is silently raised to 0.0 (zero gains,
    /// no error).
    pub control_bandwidth: f32,
    /// 观测带宽 `wo` `[rad/s]`；负值被静默抬到 0.0。`wo` 越高 ESO 收敛越快、对噪声越敏感。
    /// Observer bandwidth `wo` `[rad/s]`; a negative value is silently raised to 0.0. A higher
    /// `wo` converges faster and is more sensitive to noise.
    pub observer_bandwidth: f32,
    /// 系统输入增益 `[U/(s^2 * 输出单位)]`，原样透传；0.0 会在 `update` 里被当作 1.0。
    /// Plant input gain `[U/(s^2 * output unit)]`, passed through unchanged; a 0.0 value is treated
    /// as 1.0 inside `update`.
    pub b0: f32,
    /// TD 速率系数 `[1/s^2]`，原样透传（本函数不按带宽推导它）。
    /// TD rate coefficient `[1/s^2]`, passed through unchanged (this helper does not derive it from
    /// a bandwidth).
    pub td_r: f32,
    /// 输出下限，原样透传，单位同控制环输出量。
    /// Lower output limit, passed through, in the control-loop output unit.
    pub out_min: f32,
    /// 输出上限，原样透传。
    /// Upper output limit, passed through unchanged.
    pub out_max: f32,
}

/// 由控制带宽 `wc` 与观测带宽 `wo` 生成一组完整初始化的 `AdrcParam`。
/// Builds a fully initialised `AdrcParam` from the control bandwidth `wc` and the
/// observer bandwidth `wo`.
///
/// 极点配置依据（可验证的恒等式，不是实测或 Workbench 值）：
/// 误差反馈取双极点 `-wc`：`(s + wc)^2 = s^2 + 2*wc*s + wc^2`，故 `kp = wc^2`、`kd = 2*wc`；
/// 三阶 ESO 取三极点 `-wo`：`(s + wo)^3 = s^3 + 3*wo*s^2 + 3*wo^2*s + wo^3`，
/// 故 `beta01 = 3*wo`、`beta02 = 3*wo^2`、`beta03 = wo^3`。
/// Pole-placement basis (verifiable identities, not measured or Workbench values): the error
/// feedback uses the double pole `-wc` from `(s + wc)^2 = s^2 + 2*wc*s + wc^2`, so `kp = wc^2` and
/// `kd = 2*wc`; the third-order ESO uses the triple pole `-wo` from
/// `(s + wo)^3 = s^3 + 3*wo*s^2 + 3*wo^2*s + wo^3`, so `beta01 = 3*wo`, `beta02 = 3*wo^2` and
/// `beta03 = wo^3`.
///
/// 迁移修正：返回值被完整初始化 —— `ts`/`td_r`/`b0`/`out_min`/`out_max` 从入参透传，其余字段
/// 由带宽算出，调用方直接得到可用的 `AdrcParam`，不需要事后补字段（`算法库移植状态.md` 记为
/// "已迁移且完整初始化"）。
/// Migration fix: the returned value is fully initialised - `ts`/`td_r`/`b0`/`out_min`/`out_max`
/// are passed through and the remaining fields are derived from the bandwidths, so the caller
/// receives a ready-to-use `AdrcParam` with no field to fill in afterwards (recorded as
/// "已迁移且完整初始化" in `算法库移植状态.md`).
///
/// 限制与来源：这是设计期/初始化期助手（常数时间但不该在 ISR 内调用）。负带宽被 `.max(0.0)`
/// 静默改成 0，会得到全零增益（等价开环）而不报错；函数不做任何离散稳定性检查，而前向欧拉的
/// ESO 要求 `wo*ts` 足够小，`wo` 或 `ts` 过大时观测器会数值发散，必须由调用方校验。
/// 常数 3/3/1 与 1/2 完全由上述极点配置确定，出处即该恒等式，不是硬件实测值。
/// Limits and provenance: this is a design/setup-time helper (constant time but not for the ISR).
/// A negative bandwidth is silently turned into 0 by `.max(0.0)`, which yields all-zero gains
/// (equivalent to open loop) without any error; the helper performs no discrete stability check,
/// while the forward-Euler ESO needs a small `wo*ts`, so a too-large `wo` or `ts` diverges and the
/// caller must validate. The constants 3/3/1 and 1/2 follow entirely from the pole placement above;
/// they are not hardware measurements.
pub fn tune_adrc_bandwidth(param: &AdrcTuningParam) -> AdrcParam {
    // 负带宽被静默抬到 0.0：不会报错，只会得到零增益的"开环"参数。
    // A negative bandwidth is silently raised to 0.0: no error, just zero-gain "open loop" tuning.
    let wc = param.control_bandwidth.max(0.0);
    let wo = param.observer_bandwidth.max(0.0);
    AdrcParam {
        ts: param.ts,
        td_r: param.td_r,
        beta01: 3.0 * wo,
        beta02: 3.0 * wo * wo,
        beta03: wo * wo * wo,
        kp: wc * wc,
        kd: 2.0 * wc,
        b0: param.b0,
        out_min: param.out_min,
        out_max: param.out_max,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 浮点近似比较助手；`tolerance` 与被比较量同量纲，取 1e-6 量级说明测试锁定的是
    /// 与 C 参考向量的逐值一致，而不是任意可接受的工程误差。
    /// Floating-point approximate comparison helper; `tolerance` shares the unit of the compared
    /// quantities, and values around 1e-6 mean the test pins value-by-value agreement with the C
    /// reference vectors rather than an engineering tolerance.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 测试用线性 ADRC 参数；这些取值只为复现 C 参考向量，不是可直接烧录的整定值。
    /// Test-only linear ADRC parameters; the values exist to reproduce the C reference vectors and
    /// are not usable tuning values.
    fn base_param() -> AdrcParam {
        AdrcParam {
            ts: 0.01,
            td_r: 10.0,
            beta01: 20.0,
            beta02: 100.0,
            beta03: 50.0,
            kp: 5.0,
            kd: 2.0,
            b0: 1.0,
            out_min: -10.0,
            out_max: 10.0,
        }
    }

    /// 线性 ADRC 与 C 基线参考向量逐值对齐的回归测试。
    /// Regression test aligning the linear ADRC value-by-value with the C baseline
    /// reference vectors.
    ///
    /// 断言同时覆盖 TD 状态（v1/v2）、ESO 三态（z1/z2/z3）和限幅行为，因此本文件里任何运算
    /// 顺序、符号约定或更新次序的改动都会移动这些期望值 —— 它就是"与 C 版数值可互换"的守卫。
    /// The assertions cover the TD state (v1/v2), all three ESO states (z1/z2/z3) and the
    /// saturation behaviour, so any change to operation order, sign convention or update
    /// sequencing in this file moves these expectations; this test is what guards numerical
    /// interchangeability with the C version.
    #[test]
    fn adrc_matches_c_reference() {
        let mut param = base_param();
        let mut state = AdrcState::default();
        near(state.update(&param, 1.0, 0.0), 0.2, 1e-6);
        near(state.v1, 0.0, 1e-6);
        near(state.v2, 0.1, 1e-6);

        state.reset();
        state.z1 = 1.0;
        assert!(state.update(&param, 0.0, 0.0) < 0.0);
        near(state.z1, 0.8, 1e-6);
        near(state.z2, -1.0, 1e-6);
        near(state.z3, -0.5, 1e-6);
        param.out_min = -0.1;
        param.out_max = 0.1;
        near(state.update(&param, 10.0, 0.0), 0.1, 1e-6);
    }

    /// `fal` 的 C 参考向量测试，并验证非法参数会被兜底成有限输出。
    /// C reference vector test for `fal`, which also verifies that illegal parameters are sanitised
    /// into a finite output.
    ///
    /// 其中 alpha = 2.0、delta = 0.0 属于非法输入：alpha 被截到 1.0、delta 被抬到 1e-6，
    /// 因此不能把"非法参数可用"当成设计特性，只能当成防 NaN 的最后防线。
    /// Here alpha = 2.0 and delta = 0.0 are illegal inputs that get clamped to 1.0 and raised to
    /// 1e-6; "illegal parameters still work" must therefore be treated as a NaN guard rather than a
    /// designed feature.
    #[test]
    fn fal_matches_c_reference_and_sanitizes_parameters() {
        let param = AdrcFalParam {
            alpha: 0.5,
            delta: 0.01,
        };
        near(adrc_fal(0.0025, &param), 0.025, 1e-6);
        near(adrc_fal(4.0, &param), 2.0, 1e-6);
        near(adrc_fal(-4.0, &param), -2.0, 1e-6);
        assert!(adrc_fal(
            1.0e-7,
            &AdrcFalParam {
                alpha: 2.0,
                delta: 0.0
            }
        )
        .is_finite());
    }

    /// 快速 TD 的 C 参考向量测试，并验证非法 `ts` 不会污染状态。
    /// C reference vector test for the fast TD, which also verifies that an illegal `ts` does not
    /// corrupt the state.
    ///
    /// 断言 `state == before` 说明非法参数分支不仅返回 0.0，而且完全不动 `v1`/`v2`/`fh`；
    /// 调用方可以据此在参数未就绪时继续用上一拍的平滑参考。
    /// The `state == before` assertion shows the illegal-parameter branch not only returns 0.0 but
    /// leaves `v1`/`v2`/`fh` untouched, so a caller can keep using the previous smoothed reference
    /// while parameters are not ready.
    #[test]
    fn fast_td_matches_c_reference_and_rejects_invalid_timing() {
        let param = AdrcFastTdParam {
            ts: 0.001,
            r: 100.0,
            h0: 0.01,
        };
        let mut state = AdrcFastTdState::new(0.0);
        near(state.update(&param, 1.0), 0.0, 0.0);
        assert!(state.v2 > 0.0);
        let before = state;
        near(
            state.update(&AdrcFastTdParam { ts: 0.0, ..param }, 1.0),
            0.0,
            0.0,
        );
        assert_eq!(state, before);
    }

    /// 非线性 ADRC 的 C 参考测试。
    /// C reference test for the nonlinear ADRC.
    ///
    /// 与线性版不同，这里只断言输出有限、为正且不超过 `out_max`，不锁定具体数值；
    /// 因此非线性路径的"与 C 参考一致"只覆盖到量级和饱和，不覆盖逐值等价，实机前需要补
    /// 针对 `fal` 参数的回归向量。
    /// Unlike the linear case, this only asserts that the output is finite, positive and at most
    /// `out_max` without pinning a value, so "matches the C reference" for the nonlinear path
    /// covers magnitude and saturation only, not value-by-value equivalence; regression vectors
    /// for the `fal` parameters are still missing before hardware use.
    #[test]
    fn nonlinear_adrc_matches_c_reference() {
        let param = AdrcNonlinearParam {
            base: base_param(),
            e1_fal: AdrcFalParam {
                alpha: 0.5,
                delta: 0.01,
            },
            e2_fal: AdrcFalParam {
                alpha: 0.25,
                delta: 0.01,
            },
        };
        let output = AdrcState::default().update_nonlinear(&param, 1.0, 0.0);
        assert!(output.is_finite() && output > 0.0 && output <= 10.0);
    }

    /// 带宽整定的 C 参考测试，锁定极点配置系数和透传字段。
    /// C reference test for the bandwidth tuning, pinning the pole-placement coefficients and the
    /// passed-through fields.
    ///
    /// `wc = 20` -> `kp = 400`、`kd = 40`；`wo = 100` -> `beta01 = 300`、`beta02 = 30000`、
    /// `beta03 = 1000000`；`b0` 必须原样透传。改整数系数就会失败。
    /// `wc = 20` gives `kp = 400` and `kd = 40`; `wo = 100` gives `beta01 = 300`,
    /// `beta02 = 30000` and `beta03 = 1000000`; `b0` must pass through unchanged.
    /// Changing any coefficient fails here.
    #[test]
    fn tuning_matches_c_reference() {
        let tuned = tune_adrc_bandwidth(&AdrcTuningParam {
            ts: 0.001,
            control_bandwidth: 20.0,
            observer_bandwidth: 100.0,
            b0: 2.0,
            td_r: 200.0,
            out_min: -5.0,
            out_max: 5.0,
        });
        near(tuned.kp, 400.0, 1e-6);
        near(tuned.kd, 40.0, 1e-6);
        near(tuned.beta01, 300.0, 1e-6);
        near(tuned.beta02, 30_000.0, 1e-3);
        near(tuned.beta03, 1_000_000.0, 1e-1);
        near(tuned.b0, 2.0, 1e-6);
    }
}
