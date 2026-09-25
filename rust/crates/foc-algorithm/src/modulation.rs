//! SinePWM、SVPWM 与 DPWM 的等价实现。
//!
//! 职责 / Responsibility:
//!   - SPWM：由电角度和调制度生成三相正弦 duty，不注入零序
//!   - SVPWM：由 αβ 电压生成三相 duty，用 min-max 零序注入提高母线利用率
//!   - DPWM：把某一相钳到母线轨的不连续调制，减少该相开关次数
//!   - 输入是 αβ 电压 `[V]` 或电角度，输出是 `[0,1]` 的三相 duty
//!   - 不做软启动、死区补偿、最小脉宽、过调制补偿和故障处理
//! Three final-stage modulators: SPWM (sine duties from an angle and index, no
//! injection), SVPWM (αβ voltage with min-max zero-sequence injection) and DPWM
//! (discontinuous modulation that parks one phase on a DC rail). Inputs are αβ voltage
//! in `[V]` or an angle; output is three duties in `[0,1]`. Soft-start, dead-time
//! compensation, minimum pulse width, over-modulation compensation and fault handling
//! are out of scope.
//!
//! 架构位置 / Position in the architecture:
//!   - applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control -> 本文件
//!   - 依赖方向只向下：只依赖本 crate 的 `math`、`transform` 和 `libm`
//!   - `foc.rs` 的 `FocBasicState` 内部固定以 `svpwm_update` 收尾；`foc-control` 的
//!     电流环也直接调用 `svpwm_update`
//! Leaf of applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control ->
//! this file. Dependencies point downward only: `math`, `transform` and `libm`.
//! `FocBasicState` in `foc.rs` always ends in `svpwm_update`, and the `foc-control`
//! current loop calls `svpwm_update` directly as well.
//!
//! 单位与约定 / Units and conventions:
//!   - 电压 `[V]`、母线电压 `[V]`、角速度 `[rad/s]`；duty 与调制度无量纲
//!   - duty 是上桥臂导通比例：`0.0` = 该相接负母线、`1.0` = 接正母线、`0.5` = 中点
//!   - 三相 duty 是共模无关量：整体加减同一个偏移不改变电机看到的线电压
//!   - 本文件全部是 f32；没有 Q1.15/Q1.31 定点缩放
//! Voltage `[V]`, DC-link voltage `[V]`, angular velocity `[rad/s]`; duty and the
//! modulation index are dimensionless. Duty is the high-side on-time ratio: `0.0` ties
//! the phase to the negative rail, `1.0` to the positive rail, `0.5` is the midpoint.
//! The three duties are common-mode free: shifting all of them by the same offset does
//! not change the line voltages seen by the motor. Everything is f32; no Q-format
//! scaling.
//!
//! 实时约束 / Real-time constraints:
//!   - 在 12 kHz ADC 中断内调用；无分配、无阻塞、无日志、无 Mutex 等待
//!   - 每个函数计算量固定：`sinf` 次数、比较次数与输入无关，没有循环
//! Called inside the 12 kHz ADC interrupt: no allocation, no blocking, no logging, no
//! mutex waits. Every function has a fixed cost: the number of `sinf` calls and
//! comparisons does not depend on the arguments, and there are no loops.
//!
//! 平台侧约束 / Platform-side constraints:
//!   - 平台层把 duty 换算成 TIM1 比较值，写寄存器前还要按 3%～97% 的窗口和
//!     `minimum_duty`/`maximum_duty` 再检查一次，`NaN` duty 直接被拒绝
//!   - 遥测里的 `duty_*_per_mille` 是 duty × 1000 的 per-mille 记录量
//! The platform converts duty into TIM1 compare values and re-checks the 3%..97% window
//! and `minimum_duty`/`maximum_duty` before writing; `NaN` duties are rejected. Telemetry
//! records `duty_*_per_mille` as duty × 1000.
//!
//! 迁移与等价性 / Migration and equivalence:
//!   - 对应原 C 库的 `Modulation_SinePWM`、`Modulation_SVPWM`、`Modulation_DPWM`
//!   - 三个 `*_matches_c_reference` 测试沿用原 C 测试向量；正是这里固定的运算顺序和
//!     比例系数让它们通过，改动顺序或系数会破坏该等价性
//!   - 原 C 参考库未随本仓库提供，因此库内常数的来源类别（`[HW]`/`[ST]`/`[FW]`/
//!     `[VESC]`）无法在本仓库内确认
//! Corresponds to the C modules `Modulation_SinePWM`, `Modulation_SVPWM` and
//! `Modulation_DPWM`. The three `*_matches_c_reference` tests reuse those C test
//! vectors, which pass only because of the exact operation order and scaling used here.
//! The C reference library is not shipped in this repository, so the provenance class of
//! the constants (`[HW]`/`[ST]`/`[FW]`/`[VESC]`) cannot be confirmed in-tree.

use crate::math::{clamp, PI, SQRT_3};
use crate::transform::AlphaBeta;

/// SPWM 的三相占空比输出，无量纲 `[0,1]`。
/// Three-phase duty output of SPWM, dimensionless in `[0,1]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SinePwmOutput {
    /// a 相 duty，已钳位到 `[0,1]`。
    /// Phase-a duty, clamped to `[0,1]`.
    pub duty_a: f32,
    /// b 相 duty，已钳位到 `[0,1]`。
    /// Phase-b duty, clamped to `[0,1]`.
    pub duty_b: f32,
    /// c 相 duty，已钳位到 `[0,1]`。
    /// Phase-c duty, clamped to `[0,1]`.
    pub duty_c: f32,
}

/// 由电角度和调制度生成三相正弦占空比（SPWM，不注入零序）。
/// Builds three sine duties from an electrical angle and a modulation index (SPWM, no
/// zero-sequence injection).
///
/// 参数 / Parameters:
///   theta_rad - 电角度 `[rad]`，三相相差 ±120°；不要求落在 `[0,2π)`，直接进 `sinf`，
///               但数值过大时正弦精度会下降
///               electrical angle in `[rad]` with the phases 120° apart; it need not be
///               wrapped, but very large values lose precision in `sinf`
///   modulation - 调制度，先钳位到 `[0,1]`；1.0 时 duty 峰值到达 1.0/0.0
///                modulation index, clamped to `[0,1]`; at 1.0 the duty peaks reach
///                1.0/0.0
///
/// 返回 / Returns: 三相 duty，逐相钳位到 `[0,1]`
///                 three duties, each clamped to `[0,1]`
///
/// 用途 / Use: SPWM 的线性区比 SVPWM 小 `1/(2/√3) ≈ 13%`（即 SVPWM 多出约 15% 的母线
/// 利用率），板级电流环默认用 SVPWM；本函数保留用于与原 C 模块对照和离线比较。
/// SPWM's linear range is `1/(2/sqrt(3)) ≈ 13%` smaller than SVPWM's (equivalently SVPWM
/// gains about 15% more DC-link utilisation), and the board current loop uses SVPWM by
/// default; this function is kept for equivalence with the C module and offline
/// comparison.
#[inline]
pub fn sine_pwm_update(theta_rad: f32, modulation: f32) -> SinePwmOutput {
    // 调制度先限幅：负数或 >1 会让 duty 直接撞到 `[0,1]` 边界，失去与给定电压的比例关系。
    // Clamp the index first: a negative or >1 value would slam the duty into the `[0,1]`
    // rails and break the proportionality to the commanded voltage.
    let m = clamp(modulation, 0.0, 1.0);
    // 每相 duty = 0.5 + 0.5·m·sinθ：0.5 是中点（零电压），±0.5 的摆幅对应 ±v_bus/2 的
    // 相电压；三相共用同一个 m，没有零序注入，因此线性区小于 SVPWM。
    // Each phase is duty = 0.5 + 0.5*m*sin(theta): 0.5 is the midpoint (zero volts) and
    // the ±0.5 swing spans ±v_bus/2 of phase voltage. All phases share one `m`, so there
    // is no zero-sequence injection and the linear range is smaller than SVPWM's.
    SinePwmOutput {
        duty_a: clamp(0.5 + 0.5 * m * libm::sinf(theta_rad), 0.0, 1.0),
        duty_b: clamp(
            0.5 + 0.5 * m * libm::sinf(theta_rad - 2.0 * PI / 3.0),
            0.0,
            1.0,
        ),
        duty_c: clamp(
            0.5 + 0.5 * m * libm::sinf(theta_rad + 2.0 * PI / 3.0),
            0.0,
            1.0,
        ),
    }
}

/// SVPWM 参数：母线电压。
/// SVPWM parameters: the DC-link voltage.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SvpwmParam {
    /// 直流母线电压 `[V]`。duty = 0.5 + (v_phase - v_offset)/v_bus，所以它是电压与 duty
    /// 之间唯一的比例尺；`<= 0` 时直接返回 0.5/0.5/0.5 的安全中点，避免除零产生的
    /// `NaN`/`Inf` duty 流到功率级。
    /// DC-link voltage in `[V]`, the only scale between volts and duty. When it is
    /// `<= 0`, SVPWM returns the safe 0.5/0.5/0.5 midpoint instead of dividing by zero
    /// and letting `NaN`/`Inf` duties reach the power stage.
    pub v_bus: f32,
}

/// SVPWM 输出：三相 duty 与扇区号。
/// SVPWM output: the three duties plus the sector index.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SvpwmOutput {
    /// a 相 duty，已钳位到 `[0,1]`。
    /// Phase-a duty, clamped to `[0,1]`.
    pub duty_a: f32,
    /// b 相 duty，已钳位到 `[0,1]`。
    /// Phase-b duty, clamped to `[0,1]`.
    pub duty_b: f32,
    /// c 相 duty，已钳位到 `[0,1]`。
    /// Phase-c duty, clamped to `[0,1]`.
    pub duty_c: f32,
    /// 扇区号：`1..=6` 对应六个 60° 扇区，`0` 表示输入退化（零矢量或三相同号）。
    /// 用 `i32` 是为了和 C ABI 的整数字段对齐，它不是 Q1.31 定点数；扇区只用于遥测与
    /// 换相，不参与 duty 计算。
    /// Sector index: `1..=6` for the six 60° sectors, `0` for a degenerate input (zero
    /// vector or all three phases with the same sign). It is `i32` to match the C ABI
    /// integer field and is not a Q1.31 fixed-point value; it is used for telemetry and
    /// commutation only, never for the duty computation.
    pub sector: i32,
}

/// 默认值是安全中点 0.5/0.5/0.5（零电压矢量）加扇区 0，用于 `v_bus <= 0` 这类无法换算
/// 的输入。
/// The default is the safe 0.5/0.5/0.5 midpoint (zero-voltage vector) with sector 0,
/// used for inputs that cannot be converted, such as `v_bus <= 0`.
impl Default for SvpwmOutput {
    fn default() -> Self {
        // 三相同为 0.5 时母线上没有净电压输出，是母线电压未知时最稳妥的固定输出。
        // With all three phases at 0.5 the bridge produces no net motor voltage, the
        // safest fixed output when the DC link is unknown.
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
            sector: 0,
        }
    }
}

/// 由三相电压符号得到 `1..=6` 的扇区号；三相同号（含全零输入）时返回 0。
/// Derives the `1..=6` sector index from the signs of the three phase voltages and
/// returns 0 when all three share a sign (including the all-zero input).
///
/// 参数 / Parameters: `va`/`vb`/`vc` 是**未减去零序**的三相电压 `[V]`。
/// `va`/`vb`/`vc` are the three phase voltages in `[V]` BEFORE the zero-sequence
/// removal.
///
/// 用未减零序的量判扇区是安全的：min-max 注入只改变共模，不移动扇区边界。
/// Using the pre-injection values is safe because the min-max injection only shifts the
/// common mode and does not move the sector boundaries.
///
/// 只有 `> 0.0` 才算正，`-0.0` 与 `0.0` 都算非正，且比较不带容差：输入有噪声时扇区号
/// 可能在边界抖动，因此它不能当作控制量使用。
/// Only `> 0.0` counts as positive, so `-0.0` and `0.0` are non-positive and the
/// comparison has no tolerance: a noisy input can toggle the index at a boundary, which
/// is why it must not be used as a control quantity.
#[inline]
fn svpwm_sector(va: f32, vb: f32, vc: f32) -> i32 {
    // 位 0/1/2 分别表示 a/b/c 相符号；三相对称时只有六种组合可达。
    // Bits 0/1/2 are the signs of phases a/b/c; only six codes are reachable for a
    // balanced three-phase set.
    let mut code = 0;
    if va > 0.0 {
        code |= 1;
    }
    if vb > 0.0 {
        code |= 2;
    }
    if vc > 0.0 {
        code |= 4;
    }
    // 3 = a+b-、1 = a+、5 = a+c-、4 = c-、6 = b+c-、2 = b+，对应六个 60° 扇区；
    // `_` 覆盖 0（全零或全非正）与 7（三相全正），它们不构成有效扇区。
    // 3 = a+b-, 1 = a+, 5 = a+c-, 4 = c-, 6 = b+c-, 2 = b+ for the six 60° sectors; `_`
    // covers 0 (all zero or all non-positive) and 7 (all positive), which are not valid
    // sectors.
    match code {
        3 => 1,
        1 => 2,
        5 => 3,
        4 => 4,
        6 => 5,
        2 => 6,
        _ => 0,
    }
}

/// αβ 电压 → 三相 duty（SVPWM，min-max 零序注入）。
/// αβ voltage to three-phase duties (SVPWM with min-max zero-sequence injection).
///
/// 参数 / Parameters:
///   v_alpha_beta - 定子电压矢量 `[V]`，幅值不变形式：`alpha` 等于 a 相相对母线的
///                  相电压，`beta` 由 b/c 相合成
///                  stator voltage vector in `[V]`, amplitude-invariant: `alpha` equals
///                  phase a line-to-neutral and `beta` is the b/c combination
///   param - 只含 `v_bus` `[V]`
///           only holds `v_bus` in `[V]`
///
/// 返回 / Returns: 三相 duty `[0,1]` 与扇区号；`v_bus <= 0` 时返回
///                 `SvpwmOutput::default()` 即 0.5/0.5/0.5
///                 three duties in `[0,1]` plus the sector; when `v_bus <= 0` it returns
///                 `SvpwmOutput::default()`, i.e. 0.5/0.5/0.5
///
/// 线性区与过调制 / Linear range and over-modulation: 只有 `|v_alpha_beta| <= v_bus/√3`
/// （≈ 0.577·v_bus）时 duty 与给定电压成线性关系。超出后 duty 被钳位，实际输出电压不再
/// 跟随给定，本函数既不做过调制补偿也不返回饱和标志；电压幅值必须由调用方自己限制
/// （ST 参考参数用 `0.95·v_bus/√3` 的圆限幅）。
/// The mapping is linear only while `|v_alpha_beta| <= v_bus/sqrt(3)` (≈ 0.577·v_bus).
/// Beyond that the clamped duties over-modulate and the applied voltage no longer follows
/// the command; this function neither compensates for over-modulation nor reports
/// saturation, so the caller must limit the magnitude itself (the ST-derived parameters
/// use a `0.95 * v_bus / sqrt(3)` circle limit).
///
/// 实时 / Real-time: 单周期固定计算量，无循环、无 `sinf`。
/// Fixed single-cycle cost: no loops, no `sinf`.
#[inline]
pub fn svpwm_update(v_alpha_beta: AlphaBeta, param: &SvpwmParam) -> SvpwmOutput {
    // 母线电压非法时返回安全中点：宁可输出零电压矢量，也不能让除零产生的 NaN/Inf duty
    // 流到比较寄存器。平台层会独立再校验 duty 的有限性和范围。
    // On an invalid DC link, return the safe midpoint: a zero-voltage vector is better
    // than letting a divide-by-zero NaN/Inf duty reach the compare registers. The
    // platform independently re-validates finiteness and range.
    if param.v_bus <= 0.0 {
        return SvpwmOutput::default();
    }
    // 幅值不变逆 Clarke：αβ 电压 `[V]` → 三相相对母线的相电压 `[V]`。
    // Amplitude-invariant inverse Clarke: αβ voltage `[V]` to the three line-to-neutral
    // phase voltages `[V]`.
    let va = v_alpha_beta.alpha;
    let vb = -0.5 * v_alpha_beta.alpha + 0.5 * SQRT_3 * v_alpha_beta.beta;
    let vc = -0.5 * v_alpha_beta.alpha - 0.5 * SQRT_3 * v_alpha_beta.beta;
    // 零序取 (v_max + v_min)/2，等价于把三相电压的中点搬到母线中点：这样每相偏离中点
    // 的最大幅度最小，线性区因此比 SPWM 大 2/√3 ≈ 1.155 倍。共模电压不产生电机电流，
    // 所以注入不改变线电压。
    // The injected zero sequence is (v_max + v_min)/2, which centres the three phase
    // voltages inside the DC link and minimises the largest deviation from the midpoint;
    // that is why the linear range is 2/sqrt(3) ≈ 1.155 times SPWM's. Common mode drives
    // no motor current, so the line voltages are unchanged.
    let v_max = va.max(vb).max(vc);
    let v_min = va.min(vb).min(vc);
    let v_offset = 0.5 * (v_max + v_min);
    // duty = 0.5 + (v - v_offset)/v_bus，再逐相钳位：钳位只削减越界部分，不改变相位，
    // 三相共用同一次 v_bus 除法，结果与原 C 版逐相比值一致。
    // duty = 0.5 + (v - v_offset)/v_bus, clamped per phase. Clamping only trims the
    // excursion without changing the phase, and the shared division keeps the result
    // identical to the C version's per-phase ratio.
    SvpwmOutput {
        duty_a: clamp(0.5 + (va - v_offset) / param.v_bus, 0.0, 1.0),
        duty_b: clamp(0.5 + (vb - v_offset) / param.v_bus, 0.0, 1.0),
        duty_c: clamp(0.5 + (vc - v_offset) / param.v_bus, 0.0, 1.0),
        sector: svpwm_sector(va, vb, vc),
    }
}

/// DPWM 的钳位模式：选择把哪一侧的母线轨当作钳位目标。
/// DPWM clamp mode: which DC rail one phase is parked on.
#[repr(i32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum DpwmMode {
    /// 把电压最高的那一相钳到正母线（duty = 1）：该相下桥臂在整个扇区内不开关。
    /// Parks the highest phase on the positive rail (duty = 1), so that phase's low-side
    /// switch does not switch for the whole sector.
    #[default]
    ClampMax = 0,
    /// 把电压最低的那一相钳到负母线（duty = 0），该相上桥臂不开关。
    /// Parks the lowest phase on the negative rail (duty = 0), so that phase's high-side
    /// switch does not switch.
    ClampMin = 1,
}

/// DPWM 参数。
/// DPWM parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DpwmParam {
    /// 母线电压 `[V]`，语义与 `SvpwmParam::v_bus` 相同；`<= 0` 时返回 0.5/0.5/0.5。
    /// DC-link voltage in `[V]` with the same meaning as `SvpwmParam::v_bus`; `<= 0`
    /// returns 0.5/0.5/0.5.
    pub v_bus: f32,
    /// 钳位侧选择。运行中切换会让钳位相跳变一次，建议在低速或零电压矢量附近切换，
    /// 并重核电流采样窗口。
    /// Chooses the clamped side. Switching it while running steps the clamped phase once,
    /// so prefer to switch at low speed or near a zero-voltage vector, and re-check the
    /// current-sampling window.
    pub mode: DpwmMode,
}

/// DPWM 输出：三相 duty 与本拍注入的零序量。
/// DPWM output: the three duties plus the zero sequence injected this cycle.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DpwmOutput {
    /// a 相 duty `[0,1]`。
    /// Phase-a duty in `[0,1]`.
    pub duty_a: f32,
    /// b 相 duty `[0,1]`。
    /// Phase-b duty in `[0,1]`.
    pub duty_b: f32,
    /// c 相 duty `[0,1]`。
    /// Phase-c duty in `[0,1]`.
    pub duty_c: f32,
    /// 本拍加到三路上的零序量（duty 域，无量纲）：`ClampMin` 时是 `-d_min`，
    /// `ClampMax` 时是 `1 - d_max`。它已包含在上面的 duty 里，只用于遥测，不能重复叠加。
    /// Zero sequence added to all three phases this cycle (duty domain, dimensionless):
    /// `-d_min` for `ClampMin` and `1 - d_max` for `ClampMax`. It is already included in
    /// the duties above; it is telemetry only and must not be added again.
    pub zero_sequence: f32,
}

/// 默认值与 `SvpwmOutput::default()` 一样是 0.5/0.5/0.5 中点、零序为 0，用于
/// `v_bus <= 0` 的退化输入。
/// The default matches `SvpwmOutput::default()`: the 0.5/0.5/0.5 midpoint with zero
/// injected sequence, used for the degenerate `v_bus <= 0` input.
impl Default for DpwmOutput {
    fn default() -> Self {
        // 零序字段是 0 而不是“某个钳位值”：没有有效母线电压时不做任何注入。
        // The zero-sequence field is 0 rather than a clamp value: with no valid DC link
        // nothing is injected.
        Self {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
            zero_sequence: 0.0,
        }
    }
}

/// αβ 电压 → 三相 duty（DPWM：把一相钳到母线轨的不连续调制）。
/// αβ voltage to three-phase duties (DPWM: discontinuous modulation that parks one phase
/// on a DC rail).
///
/// 参数 / Parameters: `v_alpha_beta` 是幅值不变 αβ 电压 `[V]`，`param.v_bus` `[V]`，
/// `param.mode` 选择钳位侧。
/// `v_alpha_beta` is the amplitude-invariant αβ voltage in `[V]`, `param.v_bus` is in
/// `[V]` and `param.mode` selects the clamped side.
///
/// 返回 / Returns: 三相 duty `[0,1]` 与本拍零序量；`v_bus <= 0` 时返回默认中点。
/// Three duties in `[0,1]` plus this cycle's zero sequence; `v_bus <= 0` returns the
/// default midpoint.
///
/// 与 SVPWM 的关系 / Relation to SVPWM: 线性区相同（`|v| <= v_bus/√3`），但零序是把一相
/// “顶到轨”而不是把三相居中，因此每个 60° 扇区内总有一相 duty 停在 0 或 1，该相一个
/// 开关周期内不再开关，开关损耗下降；代价是谐波、共模与电流采样窗口改变。
/// Same linear range as SVPWM (`|v| <= v_bus/sqrt(3)`), but the injection pushes one phase
/// onto a rail instead of centring the set, so inside every 60° sector one duty sits at
/// 0 or 1: that phase stops switching for the cycle and switching loss drops. The price
/// is extra harmonics, a changed common mode and a changed current-sampling window.
///
/// 电流采样窗口 / Current-sampling window: 钳位相的 duty 贴在 0/1 上时，该相低侧（或
/// 高侧）导通时间趋近整周期，留给有效矢量的采样时间被压缩；低侧分流采样只在低侧导通
/// 期间有效，所以 DPWM 下必须重新核对采样点与窗口，窗口不足时不能沿用上一次的电流。
/// When the clamped duty sits on 0/1, that phase's low-side (or high-side) on-time
/// approaches the whole period and the time left for the active vectors shrinks.
/// Low-side shunt sampling is only valid while the low side conducts, so the sampling
/// instant and window must be re-verified for DPWM; when the window is too short the
/// stale current must not be reused.
///
/// 实时 / Real-time: 固定计算量，无 `sinf`、无循环。
/// Fixed cost: no `sinf`, no loops.
#[inline]
pub fn dpwm_update(v_alpha_beta: AlphaBeta, param: &DpwmParam) -> DpwmOutput {
    // 与 SVPWM 相同的防御：母线电压非法时输出零电压矢量，不产生 NaN/Inf duty。
    // Same guard as SVPWM: an invalid DC link yields a zero-voltage vector instead of
    // NaN/Inf duties.
    if param.v_bus <= 0.0 {
        return DpwmOutput::default();
    }
    // 幅值不变逆 Clarke：αβ 电压 `[V]` → 三相相电压 `[V]`。
    // Amplitude-invariant inverse Clarke: αβ voltage `[V]` to the three phase voltages
    // `[V]`.
    let va = v_alpha_beta.alpha;
    let vb = -0.5 * v_alpha_beta.alpha + 0.5 * SQRT_3 * v_alpha_beta.beta;
    let vc = -0.5 * v_alpha_beta.alpha - 0.5 * SQRT_3 * v_alpha_beta.beta;
    // 先算不注入的居中 duty d0 = 0.5 + v/v_bus（即 SPWM 的落点），再看离轨距离。
    // Compute the centred duties d0 = 0.5 + v/v_bus first (the SPWM location), then look
    // at the distance to the rails.
    let d0_a = 0.5 + va / param.v_bus;
    let d0_b = 0.5 + vb / param.v_bus;
    let d0_c = 0.5 + vc / param.v_bus;
    let d_max = d0_a.max(d0_b).max(d0_c);
    let d_min = d0_a.min(d0_b).min(d0_c);
    // 零序把最靠近负轨的相推到 0（ClampMin），或把最靠近正轨的相推到 1（ClampMax）：
    // 两种模式都只搬移共模，不改变线电压。
    // The zero sequence either pushes the lowest phase onto 0 (`ClampMin`) or the highest
    // onto 1 (`ClampMax`); both move the common mode only and leave the line voltages
    // unchanged.
    let zero = match param.mode {
        DpwmMode::ClampMin => -d_min,
        DpwmMode::ClampMax => 1.0 - d_max,
    };
    // 钳位后仍可能有浮点舍入或过调制造成的轻微越界，平台层写 CCR 前会再检查一遍。
    // After clamping, rounding or over-modulation can still leave a tiny excursion, which
    // the platform re-checks before writing the compare registers.
    DpwmOutput {
        duty_a: clamp(d0_a + zero, 0.0, 1.0),
        duty_b: clamp(d0_b + zero, 0.0, 1.0),
        duty_c: clamp(d0_c + zero, 0.0, 1.0),
        zero_sequence: zero,
    }
}

/// 主机单元测试：核对三个调制器与原 C 参考向量的等价性。
/// Host unit tests checking the three modulators against the original C reference
/// vectors.
///
/// 这些测试只覆盖数值等价，不验证死区、最小脉宽、采样窗口或功率级行为。
/// They cover numerical equivalence only: dead time, minimum pulse width, sampling
/// window and power-stage behaviour are not covered.
#[cfg(test)]
mod tests {
    use super::*;

    /// duty 是否落在 `[0,1]` 闭区间（允许端点）。
    /// Whether a duty lies in the closed `[0,1]` interval (endpoints allowed).
    fn in_duty_range(value: f32) -> bool {
        (0.0..=1.0).contains(&value)
    }

    /// SPWM 与 C 参考向量的等价性测试。
    /// Equivalence test for SPWM against the C reference vector.
    ///
    /// 调制度 0 必须给出严格中点 0.5，调制度 1 在正弦峰值处必须到 1.0：这两点同时锁定
    /// `0.5 + 0.5·m·sinθ` 的偏移和比例，改动任一系数都会破坏等价性。
    /// Index 0 must give exactly the 0.5 midpoint and index 1 must reach 1.0 at the sine
    /// peak; those two points pin down the offset and scale of `0.5 + 0.5*m*sin(theta)`,
    /// so changing either breaks the equivalence.
    #[test]
    fn sine_pwm_matches_c_reference() {
        // 调制度为 0 时三相都应停在 0.5（零电压矢量），与角度无关。
        // With index 0 all phases must stay at 0.5 (zero-voltage vector) for any angle.
        let zero = sine_pwm_update(0.0, 0.0);
        assert!((zero.duty_a - 0.5).abs() <= 1e-6);
        assert!((zero.duty_b - 0.5).abs() <= 1e-6);
        assert!((zero.duty_c - 0.5).abs() <= 1e-6);
        // m = 1 且 θ = 90° 时 a 相正弦为 1，duty 到达上限 1.0；另两相虽被钳位但仍留在
        // `[0,1]`，说明钳位只削减越界部分。
        // At m = 1 and theta = 90° the a-phase sine is 1 so its duty reaches the 1.0 rail;
        // the other two are clamped but stay inside `[0,1]`, showing that clamping only
        // trims the excursion.
        let peak = sine_pwm_update(PI * 0.5, 1.0);
        assert!((peak.duty_a - 1.0).abs() <= 1e-6);
        assert!(in_duty_range(peak.duty_b) && in_duty_range(peak.duty_c));
    }

    /// SVPWM 与 C 参考向量的等价性测试。
    /// Equivalence test for SVPWM against the C reference vector.
    ///
    /// 零输入必须逐字段等于默认值（中点 + 扇区 0），而 `alpha = 6 V, beta = 0` 必须落在
    /// 有效扇区并使 a 相 duty 最大：两条断言一起锁定零序注入和扇区映射。
    /// A zero input must equal the default field by field (midpoint, sector 0), while
    /// `alpha = 6 V, beta = 0` must land in a valid sector with the largest a-phase duty;
    /// together they pin down the injection and the sector mapping.
    #[test]
    fn svpwm_matches_c_reference() {
        let p = SvpwmParam { v_bus: 24.0 };
        // 零输入落到 1..=6 之外的 `_` 分支，扇区为 0，duty 为中点。
        // A zero input falls into the `_` arm outside 1..=6, giving sector 0 and midpoint
        // duties.
        let zero = svpwm_update(AlphaBeta::default(), &p);
        assert_eq!(zero, SvpwmOutput::default());
        // 24 V 母线上 6 V 仍有裕量，duty 应留在 `[0,1]` 内；`alpha` 最大的一相 duty 最大。
        // 6 V on a 24 V DC link still has headroom, so the duties stay in `[0,1]` and the
        // phase with the largest `alpha` gets the largest duty.
        let out = svpwm_update(
            AlphaBeta {
                alpha: 6.0,
                beta: 0.0,
            },
            &p,
        );
        assert!(in_duty_range(out.duty_a));
        assert!(in_duty_range(out.duty_b));
        assert!(in_duty_range(out.duty_c));
        assert!((1..=6).contains(&out.sector));
        assert!(out.duty_a > out.duty_b && out.duty_a > out.duty_c);
    }

    /// DPWM 与 C 参考向量的等价性测试。
    /// Equivalence test for DPWM against the C reference vector.
    ///
    /// 同一输入在两种模式下必须分别把 a 相顶到 1.0（`ClampMax`）、把 b/c 相顶到 0.0
    /// （`ClampMin`），这同时验证零序方向没有写反、两种模式互为镜像。
    /// The same input must push phase a to 1.0 under `ClampMax` and phases b/c to 0.0 under
    /// `ClampMin`, verifying that the injection direction is right and that the two modes
    /// mirror each other.
    #[test]
    fn dpwm_matches_c_reference() {
        // 电压矢量与 SVPWM 测试相同（alpha = 6 V），便于两路调制结果对照。
        // The voltage vector matches the SVPWM test (alpha = 6 V) so the two modulators can
        // be compared directly.
        let voltage = AlphaBeta {
            alpha: 6.0,
            beta: 0.0,
        };
        let mut p = DpwmParam {
            v_bus: 24.0,
            mode: DpwmMode::ClampMax,
        };
        let max = dpwm_update(voltage, &p);
        assert!((max.duty_a - 1.0).abs() <= 1e-6);
        // 只改模式、不改输入：另一种钳位方向也必须把对应相贴到母线轨。
        // Only the mode changes, so the other clamp direction must park its phase on the
        // rail for the same input.
        p.mode = DpwmMode::ClampMin;
        let min = dpwm_update(voltage, &p);
        assert!((min.duty_b - 0.0).abs() <= 1e-6);
        assert!((min.duty_c - 0.0).abs() <= 1e-6);
    }
}
