//! MTPA、MTPV、V/f 与弱磁控制。
//!
//! 职责：电流工作区优化 —— 按电流幅值在电流圆上做 MTPA 的 `id`/`iq` 分配、按电压椭圆搜索
//! 可行且转矩分数高的 MTPV、开环 V/f 的电压幅值与电角度生成，以及电压超限时给出负 d 轴电流
//! 的弱磁。
//! Responsibility: current operating-point optimisation - MTPA allocation of `id`/`iq` on the
//! current circle, MTPV search for a feasible high-torque point on the voltage ellipse, open-loop
//! V/f generation of voltage magnitude and electrical angle, and field weakening that produces a
//! negative d-axis current when the voltage limit is exceeded.
//!
//! 架构位置：applications/ 下的 C 代码与 RT-Thread 经 foc-rt-bridge 的 C ABI 进入 foc-control
//! 的控制周期，由 foc-control 决定何时调用本模块产生 `id_ref`/`iq_ref`；本文件只做与芯片无关
//! 的纯数学，依赖方向单向朝内（本文件 -> `crate::math`、`crate::transform`、`libm`）。
//! Architecture position: C code under applications/ and RT-Thread reach the foc-control tick
//! through the foc-rt-bridge C ABI, and foc-control decides when to call this module for
//! `id_ref`/`iq_ref`; this file is chip-independent pure math whose dependencies point inward only
//! (`crate::math`, `crate::transform`, `libm`).
//!
//! 实时性分级（12 kHz ADC 中断内：无分配、无阻塞、无日志、无 Mutex 等待）：
//! - `VfState::update`：常数时间、无 `sqrt`/`powf`，可直接放进快环；
//! - `WeakeningState::update`：常数时间、一次 `libm::sqrtf`，可直接放进快环；
//! - `MtpaState::update`、`MtpvState::update`：定长但循环次数由参数决定（`search_steps + 1`
//!   次，每次一次 `libm::sqrtf`），属于"实时可用但必须先限制 `search_steps` 并实测 WCET"；
//! - `new`/`reset` 是初始化/停机期函数。
//! 本模块当前没有接入 foc-control 的 12 kHz 主链，接线必须由工作区管理器负责（MTPA 与弱磁、
//! MTPV 不应同时各自改写 `id_ref`/`iq_ref`）。
//! Real-time grading (inside the 12 kHz ADC interrupt: no allocation, blocking, logging or mutex
//! wait): `VfState::update` is constant time with no `sqrt`/`powf` and may go straight into the
//! fast loop; `WeakeningState::update` is constant time with one `libm::sqrtf` and may do the same;
//! `MtpaState::update` and `MtpvState::update` are bounded loops whose trip count follows the
//! parameter (`search_steps + 1` iterations with one `libm::sqrtf` each), so they are real-time
//! capable only after `search_steps` is limited and the WCET measured; `new`/`reset` are setup- and
//! stop-time helpers. This module is not currently wired into the foc-control 12 kHz path, and the
//! wiring must be owned by a region manager (MTPA, weakening and MTPV must not each rewrite
//! `id_ref`/`iq_ref` independently).
//!
//! 单位与数值格式：所有量都是 `f32` 的 SI 值，本文件不做任何 Q1.15/Q1.31 定点换算；整数字段
//! （`search_steps`、`valid`）只是计数和标志，不是定标小数。
//! Units and numeric format: every value is an `f32` SI value and no Q1.15/Q1.31 fixed-point
//! scaling happens here; the integer fields (`search_steps`, `valid`) are counts and flags, not
//! scaled fractions.
//!
//! 有意差异：`MtpvState::update` 的行为与 C 基线不同，见该函数的说明以及 crate `算法库总览.md`
//! 的「与 C 基线的有意差异」一节。
//! Intentional divergence: `MtpvState::update` deliberately differs from the C baseline; see its
//! own
//! documentation and the "与 C 基线的有意差异" section of the crate `算法库总览.md`.
//!
//! 常数来源：本检出树内没有对应的 C 基线源文件（已检索 `*.c`），`search_steps` 的兜底值
//! 32/64 与门槛 8 出处未在树内标注，不能当成 MCSDK 或实测常数；电机参数（`flux_pm`、`ld`、
//! `lq`）与限制值（`current_limit`、`voltage_limit`、`id_min`/`id_max`、`v_per_hz`）由调用方
//! 按固件配置提供 `[FW]`，必须先辨识再使用。
//! Provenance: the C baseline sources are absent from this checkout (searched `*.c`), so the origin
//! of the `search_steps` fallbacks 32/64 and of the threshold 8 is not identified in-tree and they
//! must not be presented as MCSDK or measured constants; the motor parameters (`flux_pm`, `ld`,
//! `lq`) and the limits (`current_limit`, `voltage_limit`, `id_min`/`id_max`, `v_per_hz`) are
//! supplied by the caller from firmware configuration `[FW]` and must be identified before use.

use crate::math::{clamp, wrap_angle_0_to_2pi, TWO_PI};
use crate::transform::Dq;

/// MTPA 参数。
/// MTPA parameters.
///
/// 转矩模型为 `T = 1.5 * pole_pairs * (flux_pm * iq + (ld - lq) * id * iq)`；本模块只用括号内
/// 与转矩成正比的部分排序，所以所有 `torque_score` 都缺少 `1.5 * pole_pairs` 因子，不是物理
/// 转矩值，只能在同一条电机参数下相互比较。
/// The torque model is `T = 1.5 * pole_pairs * (flux_pm * iq + (ld - lq) * id * iq)`; this module
/// ranks with the bracketed part alone, so every `torque_score` is missing the `1.5 * pole_pairs`
/// factor, is not a physical torque and is comparable only under one fixed motor parameter set.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpaParam {
    /// 永磁磁链 `[Wb]`；SPMSM 的 MTPA 收益几乎全部来自它。
    /// Permanent-magnet flux linkage `[Wb]`; on an SPMSM it accounts for nearly all of the MTPA
    /// benefit.
    pub flux_pm: f32,
    /// d 轴电感 `[H]`。
    /// d-axis inductance `[H]`.
    pub ld: f32,
    /// q 轴电感 `[H]`；`ld - lq` 的符号决定磁阻转矩方向（IPMSM 通常 `lq > ld`）。
    /// q-axis inductance `[H]`; the sign of `ld - lq` sets the reluctance-torque direction (an
    /// IPMSM normally has `lq > ld`).
    pub lq: f32,
    /// 电流圆搜索的等分段数（无量纲计数）；`< 8`（含负数与未配置的 0）时兜底为 32。
    /// Number of equal segments on the current circle (a count, dimensionless); values `< 8`
    /// (including negatives and an unset 0) fall back to 32.
    pub search_steps: i32,
}

/// MTPA 状态。
/// MTPA state.
///
/// 只有一个标量字段、无堆指针；`算法库实时性说明.md` 未收录本类型，但对象就是 4 字节标量，
/// 可静态创建。
/// A single scalar field with no heap pointer; `算法库实时性说明.md` does not list this type, but the
/// object is a 4-byte scalar and can be created statically.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpaState {
    /// 最近一次搜索得到的最优转矩分数 `[Wb*A]`（比例量，见 `MtpaParam`）；退化分支返回
    /// `flux_pm * iq`。
    /// Best torque score of the last search in `[Wb*A]` (a relative figure, see `MtpaParam`); the
    /// degenerate branch stores `flux_pm * iq`.
    pub torque_score: f32,
}

impl MtpaState {
    /// 清零转矩分数；切换电机参数、停机或切换工作区时调用。
    /// Clears the torque score; call it on a motor-parameter change, on stop or when the operating
    /// region changes.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 按电流幅值 `current_mag` `[A]` 在电流圆上分配一对 `id`/`iq` `[A]`。
    /// Allocates a pair of `id`/`iq` `[A]` on the current circle for the current amplitude
    /// `current_mag` `[A]`.
    ///
    /// 电流圆约束 `id^2 + iq^2 = current_mag^2` 由构造保证（`iq` 由 `id` 反解），搜索只沿一维
    /// `id` 从 0 扫到 `-current_mag`；`score = (flux_pm + (ld - lq) * id) * iq` 是转矩的比例量：
    /// `[Wb] * [A] = [N*m]`，不含 `1.5 * pole_pairs`。
    /// The current-circle constraint `id^2 + iq^2 = current_mag^2` holds by construction (`iq` is
    /// back-solved from `id`) and the search is one-dimensional over `id` from 0 to `-current_mag`;
    /// `score = (flux_pm + (ld - lq) * id) * iq` is the relative torque figure `[Wb] * [A] = [N*m]`
    /// without the `1.5 * pole_pairs` factor.
    ///
    /// 返回：`[A]` 的 `id`/`iq`。零电流或 `|ld - lq| < 1e-9` 时直接返回 `id = 0`、
    /// `iq = |current_mag|`（SPMSM 的 MTPA 解就是 `id = 0`，在浮点噪声上搜索没有意义）；
    /// 负的 `current_mag` 只翻转 `iq` 的符号（转矩反向），`id` 仍为负。
    /// Returns `id`/`iq` in `[A]`. Zero current or `|ld - lq| < 1e-9` returns `id = 0` and
    /// `iq = |current_mag|` directly (an SPMSM's MTPA solution is `id = 0`, so searching over
    /// floating-point noise is pointless); a negative `current_mag` only flips the sign of `iq`
    /// (reverse torque) while `id` stays negative.
    ///
    /// WCET：循环 `search_steps + 1` 次、每次一次 `libm::sqrtf`、无分配，可实时调用；
    /// 但 `search_steps` 越大循环线性变长，产品必须设上界并实测 WCET。
    /// WCET: the loop runs `search_steps + 1` times with one `libm::sqrtf` each and no allocation,
    /// so it is real-time capable; the trip count grows linearly with `search_steps`, so a product
    /// must bound it and measure the WCET.
    pub fn update(&mut self, param: &MtpaParam, current_mag: f32) -> Dq {
        // abs() 取幅值：负幅值代表反向转矩，方向在函数末尾统一恢复。
        // abs() takes the magnitude: a negative amplitude means reverse torque and the direction is
        // restored once at the end of the function.
        let abs_current = current_mag.abs();
        let mut output = Dq {
            d: 0.0,
            q: abs_current,
        };
        let delta_l = param.ld - param.lq;
        // 两个退化保护：零电流无需分配；`|ld - lq|` 接近 0（SPMSM 或参数缺失）时磁阻转矩可忽略，
        // 在浮点噪声上搜索只会得到随机的最优点。这里用绝对值判据，因此 `ld > lq` 的电机同样适用。
        // Two degenerate guards: zero current needs no allocation, and a near-zero `|ld - lq|`
        // (SPMSM or missing parameters) makes the reluctance torque negligible, so searching would
        // only pick a random point out of floating-point noise. The test uses an absolute value, so
        // motors with `ld > lq` are covered as well.
        if abs_current <= 0.0 || delta_l.abs() < 1e-9 {
            self.torque_score = param.flux_pm * output.q;
            return output;
        }

        // 兜底：search_steps < 8（含负数和未配置的 0）时用 32 段。32 这个兜底值与门槛 8 的出处
        // 未在树内标注，不是 MCSDK 常数，也不是实测值。
        // Fallback: `search_steps < 8` (including negatives and an unset 0) uses 32 segments. The
        // origin of this 32 fallback and of the threshold 8 is not identified in-tree; they are
        // neither MCSDK nor measured constants.
        let steps = if param.search_steps < 8 {
            32
        } else {
            param.search_steps
        } as usize;
        // 哨兵初值 -1e30：保证第一个采样点必然成为初始最优，避免"合法但为负"的分数与 0 比较。
        // The -1e30 sentinel makes the first sample the initial best and avoids comparing a
        // legitimate negative score against 0.
        let mut best_score = -1.0e30;
        for i in 0..=steps {
            let ratio = i as f32 / steps as f32;
            let id = -abs_current * ratio;
            // iq 由电流圆反解；`.max(0.0)` 吸收 ratio = 1 时浮点减法可能产生的极小负值，
            // 否则 libm::sqrtf 会返回 NaN 并污染后面的分数比较。
            // iq is back-solved from the current circle; `.max(0.0)` absorbs the tiny negative
            // value that floating-point subtraction can produce at ratio = 1, which would otherwise
            // make libm::sqrtf return NaN and poison the score comparisons.
            let iq = libm::sqrtf((abs_current * abs_current - id * id).max(0.0));
            // score 与转矩成正比，量纲 `[Wb] * [A] = [N*m]`（缺少 1.5 * pole_pairs 因子）。
            // score is proportional to torque, dimensionally `[Wb] * [A] = [N*m]` (the 1.5 *
            // pole_pairs factor is missing).
            let score = (param.flux_pm + delta_l * id) * iq;
            if score > best_score {
                best_score = score;
                output = Dq { d: id, q: iq };
            }
        }
        self.torque_score = best_score;
        // 反向转矩只翻转 iq：`id` 保持负值才是 MTPA 解，翻转方向不会改变磁阻转矩项的大小。
        // Reverse torque only flips iq: keeping id negative is what makes this the MTPA solution,
        // and flipping the direction does not change the magnitude of the reluctance-torque term.
        if current_mag < 0.0 {
            output.q = -output.q;
        }
        output
    }
}

/// MTPV 参数。
/// MTPV parameters.
///
/// 可行判据是忽略定子电阻的稳态 dq 电压幅值 `|v| <= voltage_limit`，代价函数与 MTPA 同形。
/// The feasibility test is the steady-state dq voltage magnitude with the stator resistance
/// neglected, `|v| <= voltage_limit`, and the cost function has the same shape as MTPA's.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpvParam {
    /// 永磁磁链 `[Wb]`。
    /// Permanent-magnet flux linkage `[Wb]`.
    pub flux_pm: f32,
    /// d 轴电感 `[H]`。
    /// d-axis inductance `[H]`.
    pub ld: f32,
    /// q 轴电感 `[H]`。
    /// q-axis inductance `[H]`.
    pub lq: f32,
    /// 电流圆半径上限 `[A]`；函数内部取绝对值，负值按正数处理。
    /// Current-circle radius limit `[A]`; the function takes its absolute value, so a negative
    /// number is treated as positive.
    pub current_limit: f32,
    /// 电压椭圆半径上限 `[V]`（相电压幅值，不是母线电压）；`<= 0` 走参数非法分支。
    /// Voltage-ellipse radius limit `[V]` (phase voltage magnitude, not the DC bus voltage); `<= 0`
    /// takes the illegal-parameter branch.
    pub voltage_limit: f32,
    /// 搜索段数（无量纲计数）；`< 8`（含负数）时兜底为 64，循环次数为 `search_steps + 1`。
    /// Number of search segments (a count, dimensionless); values `< 8` (including negatives) fall
    /// back to 64, and the loop runs `search_steps + 1` times.
    pub search_steps: i32,
}

/// MTPV 状态：最近一次搜索结果与有效性标志。
/// MTPV state: the last search result together with the validity flag.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpvState {
    /// 最近一次被接受搜索点（或退化回退点）的电压幅值 `[V]`。
    /// Voltage magnitude `[V]` of the last accepted search point (or of the degenerate fallback).
    pub voltage_mag: f32,
    /// 最近一次的转矩分数 `[Wb*A]`（比例量）；无可行点时被置 0。
    /// Last torque score in `[Wb*A]` (a relative figure); set to 0 when no point is feasible.
    pub torque_score: f32,
    /// 0/1 标志：本次搜索是否找到满足电压约束的点。这是与 C 基线有意不同的字段，见 `update`。
    /// 0/1 flag telling whether this search found a point inside the voltage limit; this is the
    /// field whose behaviour intentionally differs from the C baseline, see `update`.
    pub valid: i32,
}

impl MtpvState {
    /// 清零电压幅值、转矩分数与 `valid`；在无扰切换或停机时调用。
    /// Clears the voltage magnitude, torque score and `valid`; call it for a bumpless transfer or
    /// on stop.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 在电压椭圆内搜索可行且转矩分数最高的 `id`/`iq` `[A]`。
    /// Searches inside the voltage ellipse for the feasible `id`/`iq` `[A]` with the highest torque
    /// score.
    ///
    /// 电压模型（忽略定子电阻 `R`）：`vd = -omega * lq * iq`、`vq = omega * (ld * id + flux_pm)`，
    /// 量纲 `[rad/s] * [H] * [A] = [V]`（`Wb = H * A`），幅值 `|v| = sqrt(vd^2 + vq^2)` `[V]`。
    /// 省略 `R * i` 项意味着低转速大电流时估算的 `|v|` 偏低，"未超压"的判断会偏乐观，高速区
    /// 也依然依赖调用方的工作区调度与母线裕量。
    /// Voltage model with the stator resistance `R` neglected: `vd = -omega * lq * iq` and
    /// `vq = omega * (ld * id + flux_pm)`, dimensionally `[rad/s] * [H] * [A] = [V]`
    /// (`Wb = H * A`), with magnitude `|v| = sqrt(vd^2 + vq^2)` in `[V]`. Dropping the `R * i`
    /// term underestimates `|v|` at low speed and high current, so "inside the limit" is
    /// optimistic; in the high-speed region the caller's region scheduling and bus margin still
    /// carry the responsibility.
    ///
    /// 参数：`omega_e_rad_s` 是电角速度 `[rad/s]`（内部取绝对值，高速区应传入正的电角速度大小）；
    /// `iq_sign` 决定 `iq` 的符号，`< 0.0` 取 -1，否则取 +1（包括 0.0）。
    /// Parameters: `omega_e_rad_s` is the electrical angular velocity `[rad/s]` (its absolute value
    /// is used internally); `iq_sign` selects the sign of `iq` and maps `< 0.0` to -1 and
    /// everything else (including 0.0) to +1.
    ///
    /// 返回：`[A]` 的 `id`/`iq`。`current_limit <= 0` 或 `voltage_limit <= 0` 时调用 `reset()` 并返回
    /// `Dq::default()`（`id = 0`、`iq = 0`），注意这与"搜索无可行点"的返回值不同。
    /// Returns `id`/`iq` in `[A]`. With `current_limit <= 0` or `voltage_limit <= 0` it calls
    /// `reset()` and returns `Dq::default()` (`id = 0`, `iq = 0`), which is deliberately different
    /// from the no-feasible-point return value.
    ///
    /// 与 C 基线的有意差异（见 crate `算法库总览.md`「与 C 基线的有意差异」）：本实现每次搜索前都先
    /// `self.valid = 0`，因此失败的搜索不会残留上一次的 `valid = 1`；C 版缺少这一步，可能在本次
    /// 搜索无可行点时仍报有效。无可行点时 Rust 版返回退化点 `Id = -current_limit, Iq = 0`
    /// （`voltage_mag` 置为该点的电压估计、`torque_score` 置 0），调用方必须据此回退到受限弱磁或
    /// 降转矩，而**不能**沿用上一次结果。这是有意的、已在文档中记录的行为差异，不是缺陷。
    /// Intentional divergence from the C baseline (see "与 C 基线的有意差异" in the crate `算法库总览.md`):
    /// this implementation sets `self.valid = 0` before every search, so a failed search cannot
    /// leave the previous `valid = 1` behind, whereas the C version could still report valid after
    /// a search that found nothing. With no feasible point the Rust version returns the degenerate
    /// point `Id = -current_limit, Iq = 0` (with `voltage_mag` set to that point's voltage estimate
    /// and `torque_score` set to 0), and the caller must fall back to limited weakening or reduced
    /// torque rather than reuse the previous result. This is a deliberate, documented behavioural
    /// difference, not a defect.
    ///
    /// WCET：循环 `search_steps + 1` 次、每次一次 `libm::sqrtf`；`search_steps` 未设上界时（例如
    /// `i32::MAX`）会退化成超长循环，产品必须限制该参数并实测 WCET 后才能放进 12 kHz 快环。
    /// WCET: the loop runs `search_steps + 1` times with one `libm::sqrtf` each; an unbounded
    /// `search_steps` (for example `i32::MAX`) degenerates into a very long loop, so a product must
    /// bound this parameter and measure the WCET before using it in the 12 kHz loop.
    pub fn update(&mut self, param: &MtpvParam, omega_e_rad_s: f32, iq_sign: f32) -> Dq {
        let current_limit = param.current_limit.abs();
        let omega_abs = omega_e_rad_s.abs();
        let sign = if iq_sign < 0.0 { -1.0 } else { 1.0 };
        let steps = if param.search_steps < 8 {
            64
        } else {
            param.search_steps
        } as usize;
        // 参数非法分支：清空状态并返回零电流（不是退化点 Id = -current_limit），调用方需要区分
        // "参数没配好"和"本次搜索失败"两种 0 结果。
        // Illegal-parameter branch: clear the state and return zero current (not the degenerate
        // `Id = -current_limit` point); the caller has to distinguish "parameters not configured"
        // from "this search failed", both of which can look like zero.
        if current_limit <= 0.0 || param.voltage_limit <= 0.0 {
            self.reset();
            return Dq::default();
        }

        // C 版没有在每次搜索前清除此标志，可能把上一次结果误报为有效。
        self.valid = 0;
        let mut output = Dq::default();
        let mut best_score = -1.0e30;
        let mut best_voltage = 0.0;
        for i in 0..=steps {
            let ratio = i as f32 / steps as f32;
            let id = -current_limit * ratio;
            // iq 由电流圆反解，`.max(0.0)` 的作用与 MTPA 相同：吸收 ratio = 1 时的极小负值。
            // iq is back-solved from the current circle; `.max(0.0)` has the same purpose as in
            // MTPA, absorbing the tiny negative value possible at ratio = 1.
            let iq = sign * libm::sqrtf((current_limit * current_limit - id * id).max(0.0));
            // 稳态电压（忽略 R）：vd 来自 q 轴电流与 lq 的交叉耦合，vq 来自 d 轴磁链，
            // 两者都是 `[V]`。
            // Steady-state voltage with R neglected: vd comes from the cross-coupling of the q-axis
            // current with lq and vq from the d-axis flux linkage, both in `[V]`.
            let vd = -omega_abs * param.lq * iq;
            let vq = omega_abs * (param.ld * id + param.flux_pm);
            let voltage_mag = libm::sqrtf(vd * vd + vq * vq);
            // 代价函数与 MTPA 同形（用 |iq| 保证反向转矩不影响排序），量纲 `[Wb] * [A] = [N*m]`。
            // The cost function has the same shape as MTPA's (using |iq| so reverse torque does not
            // affect the ranking) and is dimensionally `[Wb] * [A] = [N*m]`.
            let score = (param.flux_pm + (param.ld - param.lq) * id) * iq.abs();
            // 只在电压约束内接受候选点；由于 id 从 0 向负方向扫描，可行区通常从某个负 id 开始，
            // 这里的比较就是"约束内转矩最大"的离散近似。
            // A candidate is accepted only inside the voltage constraint; because id is scanned
            // from 0 towards negative values the feasible region normally starts at some negative
            // id, and this comparison is the discrete approximation of "maximum torque inside the
            // constraint".
            if voltage_mag <= param.voltage_limit && score > best_score {
                best_score = score;
                best_voltage = voltage_mag;
                output = Dq { d: id, q: iq };
                self.valid = 1;
            }
        }
        // 无可行点：返回纯 d 轴退化点并明确置 valid = 0，绝不返回上一次的搜索结果。
        // No feasible point: return the pure d-axis degenerate point and explicitly leave valid =
        // 0, never the previous search result.
        if self.valid == 0 {
            output = Dq {
                d: -current_limit,
                q: 0.0,
            };
            // iq = 0 时 vd = 0，所以该点电压幅值为 `omega * |flux_pm - ld * current_limit|` `[V]`，
            // 与循环内的电压模型一致。
            // With iq = 0 the value of vd is 0, so this point's voltage magnitude is
            // `omega * |flux_pm - ld * current_limit|` `[V]`, consistent with the loop model.
            best_voltage = omega_abs * (param.flux_pm - param.ld * current_limit).abs();
            best_score = 0.0;
        }
        self.voltage_mag = best_voltage;
        self.torque_score = best_score;
        output
    }
}

/// V/f 参数（开环恒压频比控制，通常用于启动或低成本驱动）。
/// V/f parameters (open-loop constant volts-per-hertz control, normally used for start-up or
/// low-cost drives).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VfParam {
    /// 采样周期 `[s]`；`<= 0` 时 `update` 返回全零输出且**不修改内部状态**。
    /// Sample period `[s]`; `<= 0` makes `update` return an all-zero output and leave the internal
    /// state **untouched**.
    pub ts: f32,
    /// 压频比 `[V/Hz]`，乘的是频率的绝对值。
    /// Volts-per-hertz slope `[V/Hz]`, multiplied by the absolute frequency.
    pub v_per_hz: f32,
    /// 低频偏置 `[V]`：`f = 0` 时仍输出该电压，用于补偿定子电阻压降（低频转矩不足）。
    /// Low-frequency boost `[V]`: it is still output at `f = 0` to compensate the stator resistance
    /// drop, which would otherwise starve low-frequency torque.
    pub v_min: f32,
    /// 电压上限 `[V]`；输出电压还额外被钳在 `>= 0`。
    /// Voltage ceiling `[V]`; the output is additionally clamped to `>= 0`.
    pub v_max: f32,
    /// 电频率下限 `[Hz]`，可为负（反转）。
    /// Lower electrical-frequency bound `[Hz]`, may be negative (reverse rotation).
    pub freq_min: f32,
    /// 电频率上限 `[Hz]`；`math::clamp` 在 min > max 时会自动交换两者，配错不会报错。
    /// Upper electrical-frequency bound `[Hz]`; `math::clamp` silently swaps the two bounds when
    /// min > max, so a swapped configuration is not reported.
    pub freq_max: f32,
}

/// V/f 输出状态，同时也是 `update` 的返回类型。
/// V/f output state, which is also the return type of `update`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VfState {
    /// 电角度 `[rad]`，始终 wrap 在 `[0, 2*pi)` 内。
    /// Electrical angle `[rad]`, always wrapped into `[0, 2*pi)`.
    pub theta_rad: f32,
    /// 本拍输出电压幅值 `[V]`。
    /// Output voltage magnitude for this tick `[V]`.
    pub voltage: f32,
    /// 本拍限幅后的电频率 `[Hz]`，带符号，负值表示反转。
    /// Limited electrical frequency for this tick `[Hz]`, signed, where a negative value means
    /// reverse rotation.
    pub freq_hz: f32,
}

/// 命名别名：`VfOutput` 与 `VfState` 是同一类型，`update` 直接按值返回状态副本。
/// Naming alias: `VfOutput` and `VfState` are the same type, and `update` returns a copy of the
/// state by value.
pub type VfOutput = VfState;

impl VfState {
    /// 以给定电角度 `[rad]` 构造：先 wrap 到 `[0, 2*pi)`，电压与频率保持 0。
    /// Builds with the given electrical angle `[rad]` wrapped into `[0, 2*pi)` and with zero
    /// voltage and frequency.
    ///
    /// `wrap_angle_0_to_2pi` 对 NaN 与 ±Inf 都返回 NaN，因此非法角度不会在这里被静默变成 0；
    /// 属于初始化期函数，不在 ISR 内调用。
    /// `wrap_angle_0_to_2pi` returns NaN for NaN and for ±Inf, so an illegal angle is not silently
    /// turned into 0 here; setup-time function, not called from the ISR.
    pub fn new(theta_rad: f32) -> Self {
        Self {
            theta_rad: wrap_angle_0_to_2pi(theta_rad),
            ..Self::default()
        }
    }

    /// 复位到新电角度，等价于 `*self = Self::new(theta_rad)`；停机或重启开环启动时调用。
    /// Resets to a new electrical angle, equivalent to `*self = Self::new(theta_rad)`; call it on
    /// stop or when restarting an open-loop start-up.
    pub fn reset(&mut self, theta_rad: f32) {
        *self = Self::new(theta_rad);
    }

    /// 由电频率 `freq_hz` `[Hz]` 生成开环电压幅值 `[V]` 与下一个电角度 `[rad]`。
    /// Generates the open-loop voltage magnitude `[V]` and the next electrical angle `[rad]` from
    /// the electrical frequency `freq_hz` `[Hz]`.
    ///
    /// 输入必须是**电频率**：机械频率要先乘极对数。角度积分使用带符号的限幅频率，电压使用其
    /// 绝对值，所以反转时电压幅值不会变负而角度方向会正确反向；角度增量 `2*pi * f * ts` 的量纲是
    /// `[rad]`（`[Hz] * [s]` 为圈数，乘 `2*pi` 得弧度）。
    /// The input must be the *electrical* frequency: a mechanical frequency has to be multiplied by
    /// the pole pairs first. The angle integrates the signed limited frequency while the voltage
    /// uses its magnitude, so reversing rotation keeps the voltage positive and reverses the angle
    /// correctly; the angle increment `2*pi * f * ts` is `[rad]` (`[Hz] * [s]` is a count of
    /// revolutions, times `2*pi` gives radians).
    ///
    /// 返回：整份状态副本（`VfOutput = VfState`），按值拷贝，无分配。
    /// 失败语义：`ts <= 0` 时返回全零的 `VfOutput::default()`，内部状态不变 —— 调用方会看到一拍
    /// 零角度、零电压，这不是复位，恢复合法 `ts` 后角度会从原值继续。
    /// Returns a copy of the whole state (`VfOutput = VfState`), copied by value with no
    /// allocation. Failure semantics: with `ts <= 0` it returns an all-zero `VfOutput::default()`
    /// and leaves the internal state unchanged, so the caller sees one tick of zero angle and zero
    /// voltage; this is not a reset, and the angle resumes from its old value once `ts` is legal
    /// again.
    ///
    /// 实时性：常数时间、无 `sqrt`/`powf`/循环、无分配，可直接放进 12 kHz 快环。
    /// Real time: constant time with no `sqrt`/`powf`/loop and no allocation, so it may go straight
    /// into the 12 kHz loop.
    pub fn update(&mut self, param: &VfParam, freq_hz: f32) -> VfOutput {
        if param.ts <= 0.0 {
            return VfOutput::default();
        }
        // 频率限幅保留符号（允许反转），与电压侧取绝对值配合，方向语义一致。
        // The frequency clamp keeps the sign (reverse rotation allowed) which, together with the
        // voltage magnitude, keeps the direction semantics consistent.
        let limited_freq = clamp(freq_hz, param.freq_min, param.freq_max);
        // V/f 直线加上低频偏置 v_min，再钳到 `[0, v_max]`：下限固定为 0，因此即使 v_min 配成负值
        // 也不会输出负电压；v_max 之上则直接削顶，避免超出调制能力。
        // The V/f line plus the low-frequency boost v_min, clamped into `[0, v_max]`: the lower
        // bound is fixed at 0 so even a negative v_min cannot produce a negative voltage, and the
        // v_max ceiling clips the demand before it exceeds the modulation capability.
        let voltage = param.v_min + param.v_per_hz * limited_freq.abs();
        self.freq_hz = limited_freq;
        self.voltage = clamp(voltage, 0.0, param.v_max);
        // 电角度积分并 wrap 到 `[0, 2*pi)`；用限幅后（而不是原始）频率，保证输出电压与角度推进一致。
        // Integrates the electrical angle and wraps it into `[0, 2*pi)`; the limited frequency is
        // used rather than the raw one so the output voltage and the angle advance stay consistent.
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_rad + TWO_PI * limited_freq * param.ts);
        *self
    }
}

/// 弱磁参数。
/// Field-weakening parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeakeningParam {
    /// 电压误差到 d 轴电流的比例增益 `[A/V]`；只在超压时起作用。
    /// Proportional gain from voltage error to d-axis current `[A/V]`; it acts only when the
    /// voltage is over the limit.
    pub kp: f32,
    /// 允许的 dq 电压幅值上限 `[V]`（相电压幅值，不是母线电压），超出部分才产生弱磁电流。
    /// Allowed dq voltage magnitude limit `[V]` (phase voltage magnitude, not the DC bus voltage);
    /// only the excess produces a weakening current.
    pub voltage_limit: f32,
    /// `id_ref` 下限 `[A]`；弱磁通常给负值，绝对值受退磁能力与电流能力约束。
    /// Lower bound of `id_ref` `[A]`; weakening normally uses a negative value whose magnitude is
    /// bounded by demagnetisation and current capability.
    pub id_min: f32,
    /// `id_ref` 上限 `[A]`；结果还会被 `.min(0.0)` 再钳一次，因此永远不会输出正 `id`。
    /// Upper bound of `id_ref` `[A]`; the result is additionally clamped by `.min(0.0)`, so a
    /// positive `id` is never returned.
    pub id_max: f32,
}

/// 弱磁状态。
/// Field-weakening state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeakeningState {
    /// 最近一次输出的 d 轴电流参考 `[A]`，恒 `<= 0`。
    /// Last d-axis current reference `[A]`, always `<= 0`.
    pub id_ref: f32,
    /// 最近一次由 dq 电压算出的电压幅值 `[V]`。
    /// Voltage magnitude `[V]` computed from the dq voltages on the last call.
    pub voltage_mag: f32,
    /// 电压误差 `voltage_mag - voltage_limit` `[V]`；只有正值才触发弱磁。
    /// Voltage error `voltage_mag - voltage_limit` `[V]`; only a positive value triggers weakening.
    pub voltage_error: f32,
}

impl WeakeningState {
    /// 清零弱磁状态；退出高速弱磁区、停机或切换工作区时调用，避免把上一段的 `id_ref` 带进新工况。
    /// Clears the weakening state; call it when leaving the high-speed weakening region, on stop or
    /// when the operating region changes, so the previous `id_ref` does not leak into the new
    /// operating point.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 电压超限时给出负的 d 轴电流参考 `[A]`。
    /// Produces a negative d-axis current reference `[A]` when the voltage limit is exceeded.
    ///
    /// 参数：`voltage_dq` 是 dq 电压 `[V]`（只用幅值，方向无关）；`base_id_ref` 是上游（例如 MTPA
    /// 或 Id=0 策略）给出的 d 轴参考 `[A]`。
    /// Parameters: `voltage_dq` is the dq voltage `[V]` (only the magnitude matters, not the
    /// direction) and `base_id_ref` is the d-axis reference `[A]` supplied upstream, for example by
    /// MTPA or by an Id=0 strategy.
    ///
    /// 返回：钳位后的 `id_ref` `[A]`，恒 `<= 0`；未超压时等于 `base_id_ref` 的钳位结果。
    /// Returns the clamped `id_ref` `[A]`, always `<= 0`; while the voltage is inside the limit it
    /// equals the clamped `base_id_ref`.
    ///
    /// 语义与限制：只有比例作用、没有积分项，因此会保留稳态电压误差；也没有速率限制，工作区切入
    /// 切出时 `id_ref` 可能阶跃，必须由工作区管理器加斜坡或滞环，并按 FOC 组合文档要求先设好退出
    /// 阈值再依赖本函数。
    /// Semantics and limits: proportional only with no integral term, so a steady-state voltage
    /// error remains; there is also no rate limit, so `id_ref` can step when the region changes and
    /// the region manager must add ramping or hysteresis, setting the exit threshold before relying
    /// on this function.
    ///
    /// 实时性：常数时间、一次 `libm::sqrtf`、无分配、无阻塞，可用于 12 kHz 电流参考路径。
    /// Real time: constant time with one `libm::sqrtf`, no allocation and no blocking, so it is
    /// usable on the 12 kHz current-reference path.
    pub fn update(&mut self, param: &WeakeningParam, voltage_dq: Dq, base_id_ref: f32) -> f32 {
        self.voltage_mag = libm::sqrtf(voltage_dq.d * voltage_dq.d + voltage_dq.q * voltage_dq.q);
        self.voltage_error = self.voltage_mag - param.voltage_limit;
        let mut id_ref = base_id_ref;
        // 只在超压时把 id 推得更负；不超压时保持上游参考，避免无谓的铜耗和额外的退磁风险。
        // id is only pushed more negative when the voltage is over the limit; otherwise the
        // upstream reference is kept, avoiding pointless copper loss and extra demagnetisation
        // risk.
        if self.voltage_error > 0.0 {
            id_ref -= param.kp * self.voltage_error;
        }
        // 双重限幅：先按 `[id_min, id_max]` 钳（注意 clamp 在 min > max 时会交换两者），再用
        // `.min(0.0)` 保证弱磁只能向负方向增磁，永远不会意外抬高电压。
        // Double clamping: first into `[id_min, id_max]` (note that clamp swaps the bounds when min
        // > max) and then `.min(0.0)` so weakening can only push id negative and can never raise
        // the voltage.
        self.id_ref = clamp(id_ref, param.id_min, param.id_max).min(0.0);
        self.id_ref
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 浮点近似比较助手；`tolerance` 与被比较量同量纲。
    /// Floating-point approximate comparison helper; `tolerance` shares the unit of the compared
    /// values.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// MTPA 的 C 参考向量测试，同时验证 SPMSM 退化行为与电流圆约束。
    /// C reference vector test for MTPA, which also verifies the SPMSM degenerate behaviour and the
    /// current-circle constraint.
    ///
    /// `spm` 的 `ld == lq` 断言结果精确等于 `id = 0`、`iq = 10.0`（不经过搜索）；`ipm` 断言
    /// `id < 0` 且 `sqrt(id^2 + iq^2) = 10.0`（容差 1e-4，因为 iq 由 sqrt 反解）；最后断言负
    /// 电流幅值只翻转 `iq` 符号。改搜索起点、步数或打分公式都会移动这些期望值。
    /// The `spm` case with `ld == lq` asserts exactly `id = 0` and `iq = 10.0` without any search,
    /// the `ipm` case asserts `id < 0` and `sqrt(id^2 + iq^2) = 10.0` within 1e-4 (because iq is
    /// back-solved through sqrt), and the last assertion checks that a negative amplitude only
    /// flips the sign of iq. Changing the search start, the step count or the scoring formula moves
    /// these expectations.
    #[test]
    fn mtpa_matches_c_reference() {
        let spm = MtpaParam {
            flux_pm: 0.05,
            ld: 0.001,
            lq: 0.001,
            search_steps: 32,
        };
        let ipm = MtpaParam {
            flux_pm: 0.02,
            ld: 0.0005,
            lq: 0.0015,
            search_steps: 128,
        };
        let mut state = MtpaState::default();
        assert_eq!(state.update(&spm, 10.0), Dq { d: 0.0, q: 10.0 });
        let out = state.update(&ipm, 10.0);
        assert!(out.d < 0.0);
        near(libm::sqrtf(out.d * out.d + out.q * out.q), 10.0, 1e-4);
        assert!(state.update(&ipm, -10.0).q < 0.0);
    }

    /// MTPV 的 C 参考向量测试，并锁定"清除上次 valid"这一有意差异。
    /// C reference vector test for MTPV, which also pins the intentional "clear stale valid"
    /// divergence.
    ///
    /// 断言顺序说明：`valid = 1` 且 `voltage_mag <= voltage_limit`、电流不超过 `current_limit`；
    /// 反向 `iq_sign` 得到 `q <= 0`；把 `voltage_limit` 压到 0.1 后本次搜索无可行点，于是
    /// `valid = 0`、返回退化点 `d = -10.0`、`q = 0.0`。C 版在最后一步可能仍报 `valid = 1`，
    /// 所以这个测试就是该有意差异的守卫，不能被"对齐 C"改回去。
    /// Assertion order: `valid = 1` together with `voltage_mag <= voltage_limit` and a current
    /// inside `current_limit`; a reversed `iq_sign` yields `q <= 0`; after squeezing
    /// `voltage_limit` to 0.1 the search has no feasible point, so `valid = 0` with the degenerate
    /// point `d = -10.0`, `q = 0.0`. The C version could still report `valid = 1` at that last
    /// step, so this test guards the intentional divergence and must not be "aligned back" to C.
    #[test]
    fn mtpv_matches_c_reference_and_clears_stale_valid() {
        let mut param = MtpvParam {
            flux_pm: 0.02,
            ld: 0.0005,
            lq: 0.0015,
            current_limit: 10.0,
            voltage_limit: 20.0,
            search_steps: 128,
        };
        let mut state = MtpvState::default();
        let out = state.update(&param, 1000.0, 1.0);
        assert_eq!(state.valid, 1);
        assert!(state.voltage_mag <= param.voltage_limit + 1e-5);
        assert!(libm::sqrtf(out.d * out.d + out.q * out.q) <= param.current_limit + 1e-5);
        assert!(state.update(&param, 1000.0, -1.0).q <= 0.0);
        param.voltage_limit = 0.1;
        let fallback = state.update(&param, 1000.0, 1.0);
        near(fallback.q, 0.0, 1e-6);
        assert_eq!(state.valid, 0);
        near(fallback.d, -10.0, 1e-6);
    }

    /// V/f 的 C 参考向量测试。
    /// C reference vector test for V/f.
    ///
    /// `ts = 0.01`、`v_per_hz = 0.2`、`v_min = 1.0`、频率 10 Hz：电压 `1.0 + 0.2 * 10 = 3.0` `[V]`，
    /// 角度增量 `2*pi * 10 * 0.01 = 0.2*pi` `[rad]`；频率 100 Hz 超过 `freq_max = 50` 后
    /// 被钳到 50 Hz、电压削到 `v_max = 10.0`。它锁定限幅与角度积分的具体形式。
    /// With `ts = 0.01`, `v_per_hz = 0.2`, `v_min = 1.0` and 10 Hz the voltage is
    /// `1.0 + 0.2 * 10 = 3.0` `[V]` and the angle increment is `2*pi * 10 * 0.01 = 0.2*pi` `[rad]`;
    /// a 100 Hz command above `freq_max = 50` is clamped to 50 Hz and the voltage to
    /// `v_max = 10.0`. This pins the exact limiting and integration form.
    #[test]
    fn vf_matches_c_reference() {
        let p = VfParam {
            ts: 0.01,
            v_per_hz: 0.2,
            v_min: 1.0,
            v_max: 10.0,
            freq_min: -50.0,
            freq_max: 50.0,
        };
        let mut state = VfState::default();
        let out = state.update(&p, 10.0);
        near(out.voltage, 3.0, 1e-6);
        near(out.theta_rad, TWO_PI * 0.1, 1e-5);
        let out = state.update(&p, 100.0);
        near(out.freq_hz, 50.0, 1e-6);
        near(out.voltage, 10.0, 1e-6);
    }

    /// 弱磁的 C 参考向量测试。
    /// C reference vector test for field weakening.
    ///
    /// `d = 6.0`、`q = 8.0` 时 `|v| = 10.0` 正好等于 `voltage_limit`，误差为 0 不产生弱磁，返回 0.0；
    /// `q = 12.0` 时 `|v| = 13.416...`，误差 3.416...，乘以 `kp = 0.1` 得到 `-0.341641`；
    /// 最后 `kp = 10.0` 时结果被 `id_min = -5.0` 钳住，说明下限确实生效。
    /// With `d = 6.0`, `q = 8.0` the magnitude `|v| = 10.0` equals `voltage_limit`, so the error is
    /// 0, no weakening happens and the result is 0.0; with `q = 12.0` the magnitude is `13.416...`,
    /// the error is `3.416...` and `kp = 0.1` gives `-0.341641`; finally `kp = 10.0` is clamped by
    /// `id_min = -5.0`, showing that the lower bound is effective.
    #[test]
    fn weakening_matches_c_reference() {
        let mut p = WeakeningParam {
            kp: 0.1,
            voltage_limit: 10.0,
            id_min: -5.0,
            id_max: 0.0,
        };
        let mut state = WeakeningState::default();
        near(state.update(&p, Dq { d: 6.0, q: 8.0 }, 0.0), 0.0, 1e-6);
        near(
            state.update(&p, Dq { d: 6.0, q: 12.0 }, 0.0),
            -0.341_641,
            1e-5,
        );
        p.kp = 10.0;
        near(state.update(&p, Dq { d: 6.0, q: 12.0 }, 0.0), -5.0, 1e-6);
    }
}
