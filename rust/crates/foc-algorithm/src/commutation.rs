//! Commutation_SixStep 的等价实现。
//!
//! FluxRT —— BLDC 六步（方波）换相（`foc-algorithm` 的换相层）。
//! FluxRT - BLDC six-step (trapezoidal) commutation, the commutation layer of
//! `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - 由扇区号或电角度查出一个 60° 区间的桥臂状态：哪一相上桥臂导通、哪一相
//!     下桥臂导通、哪一相浮空。
//!   - 输出的不是占空比，而是三态桥臂状态；配合 BEMF 过零检测即可完成无感
//!     六步换相定时。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `commutation`（本模块，只依赖 `crate::math`）
//!
//! 与正弦 FOC 的关系（重要）/ Relation to sinusoidal FOC (important):
//!   六步与 FOC 是**两条互斥的主链**：六步模式不要同时调用 Park、d/q PI 和
//!   SVPWM。同一产品可以同时支持两种模式，但必须停机切换并分别验证采样、
//!   换相和保护时序（见 `docs/FOC算法组合与应用场景.md`）。
//!   Six-step and FOC are mutually exclusive main chains; a product may support
//!   both, but only with a stopped-mode switch and separately verified timing.
//!
//! 实时约束 / Real-time constraints:
//!   两个函数都是查表/算术，无分配、无阻塞、无日志，可用于 12 kHz 快环；
//!   `six_step_from_angle()` 会调用 `wrap_angle_0_to_2pi()`，其循环次数随
//!   |角度| 线性增长，因此传给它的角度必须有界。
//!   Both functions are table/arithmetic only and fast-loop safe, but
//!   `six_step_from_angle()` inherits the linear-in-|angle| wrap cost, so its
//!   input must be bounded.
//!
//! 参考 / Reference: `算法库移植状态.md`（Commutation_SixStep 行）

use crate::math::{wrap_angle_0_to_2pi, PI};

/// 六步桥臂状态：三相各一个三态值，附带归一化后的扇区号。
/// Six-step bridge state: one tri-state value per phase plus the normalised
/// sector number.
///
/// 用 `i32` 而不是 `bool`，是因为每相有三种合法状态（上桥臂导通 / 下桥臂导通 /
/// 浮空），单一布尔无法表达；这也是与 C 参考结构体逐字段对齐的布局。
/// `i32` rather than `bool` is required because each phase has three legal states
/// (high-side on / low-side on / floating).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SixStepOutput {
    /// A 相桥臂状态：`+1` 上桥臂导通，`-1` 下桥臂导通，`0` 浮空。
    /// Phase A bridge state: `+1` high side on, `-1` low side on, `0` floating.
    pub phase_a: i32,
    /// B 相桥臂状态，取值含义同 `phase_a`（`+1`/`-1`/`0`）。
    /// Phase B bridge state, same `+1`/`-1`/`0` encoding as `phase_a`.
    pub phase_b: i32,
    /// C 相桥臂状态，取值含义同 `phase_a`（`+1`/`-1`/`0`）。
    /// Phase C bridge state, same `+1`/`-1`/`0` encoding as `phase_a`.
    pub phase_c: i32,
    /// 归一化后的扇区号，取值 `1..=6`，每步对应 60° 电角度。
    /// Normalised sector number in `1..=6`; each step spans 60 electrical
    /// degrees.
    ///
    /// 扇区号按换相顺序递增，所以它同时也是"电角度 ÷ 60° + 1"的可读表示，
    /// 可直接用于遥测和换相调试。
    /// Sectors increase in commutation order, so this doubles as a readable
    /// representation of the electrical angle.
    pub sector: i32,
}

/// 按扇区号查表得到六步桥臂状态。
/// Looks up the six-step bridge state for a sector number.
///
/// 参数 / Parameters:
///   sector 扇区号，任意 `i32`：`1..=6` 为有效扇区，其它值经 `rem_euclid` 环绕，
///          `0` 视作 `6`。
///
/// 返回 / Returns: `SixStepOutput`，`sector` 字段一定是归一化后的 `1..=6`。
///
/// 算法与陷阱 / Algorithm and pitfalls:
///   - 归一化写成 `(sector - 1).rem_euclid(6) + 1`：先减 1 再取模再加 1，使
///     "1 基"的扇区号在负数和 0 上也能落在 `1..=6`。若改用 `sector % 6`，
///     `0` 会得到扇区 0 而不是 6，换相顺序会整体错一拍。
///   - 表格序列 `(1,-1,0) (1,0,-1) (0,1,-1) (-1,1,0) (-1,0,1) (0,-1,1)` 是标准的
///     60° 换相序列，每步只切换两相，因此任何时刻都有一相浮空——浮空相正是
///     BEMF 过零检测要观察的那一相。
///   - `_ => (0, 0, 0)` 分支在归一化之后不可达，保留它只是为了保持与 C 版
///     `switch` 语句结构一致。三相同为 0 表示"全部关断"，是安全但无转矩的状态。
///   - The 1-based normalisation must stay as `(sector - 1).rem_euclid(6) + 1`;
///     `sector % 6` would map 0 to sector 0 and shift the whole sequence.
///   - The table is the standard 60° sequence: exactly one phase floats at any
///     time, which is what BEMF zero-cross detection observes.
///   - The unreachable `_` arm mirrors the C `switch` structure; all-zero means
///     "all off", a safe but torque-free state.
pub fn six_step_from_sector(sector: i32) -> SixStepOutput {
    let normalized = (sector - 1).rem_euclid(6) + 1;
    let (phase_a, phase_b, phase_c) = match normalized {
        1 => (1, -1, 0),
        2 => (1, 0, -1),
        3 => (0, 1, -1),
        4 => (-1, 1, 0),
        5 => (-1, 0, 1),
        6 => (0, -1, 1),
        _ => (0, 0, 0),
    };
    SixStepOutput {
        phase_a,
        phase_b,
        phase_c,
        sector: normalized,
    }
}

/// 由电角度得到六步桥臂状态。
/// Derives the six-step bridge state from an electrical angle.
///
/// 参数 / Parameters:
///   theta_rad 电角度 `[rad]`（机械角度 × 极对数 + 零位偏置）。
///
/// 返回 / Returns: 该角度所在 60° 扇区的六步输出，扇区号 `1..=6`。
///
/// 算法与边界 / Algorithm and boundaries:
///   `wrapped / (π/3)` 先把 `[0, 2π)` 映射到 `[0, 6)`，`as i32` 是**截断取整**
///   （不是四舍五入），所以 `θ = 0`、`θ = π/6` 都落在扇区 1，扇区边界与
///   `π/3` 的整数倍对齐——这正是六步换相希望的边界。
///   `wrapped / (π/3)` maps `[0, 2π)` onto `[0, 6)` and `as i32` truncates, so
///   `θ = 0` and `θ = π/6` both fall in sector 1 and sector boundaries align with
///   integer multiples of `π/3`.
///
/// 陷阱 / Pitfalls:
///   - `.min(6)` 只压上限，没有 `.max(1)`；正常情况下截断结果已是 `0..=5`，
///     加 1 后必然是 `1..=6`，所以钳位不会触发。它只是边界防护，不是把非法
///     输入映射成合法扇区的手段。
///   - `theta_rad` 为非有限值时 `wrap_angle_0_to_2pi` 返回 `NaN`，而 Rust 的
///     浮点转整数是**饱和转换**（`NaN` 转为 0），因此本函数会静默返回扇区 1，
///     不会 panic 也不会未定义。调用方若需要区分"无效角度"，必须自己先判
///     `is_finite()`。
///   - `.min(6)` clamps only the upper bound; the truncation already yields
///     `0..=5`. A non-finite angle becomes `NaN`, saturates to 0 on the cast and
///     silently returns sector 1, so validate `is_finite()` if that matters.
pub fn six_step_from_angle(theta_rad: f32) -> SixStepOutput {
    let wrapped = wrap_angle_0_to_2pi(theta_rad);
    let sector = ((wrapped / (PI / 3.0)) as i32 + 1).min(6);
    six_step_from_sector(sector)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 与 C 版 `Commutation_SixStep` 原测试向量逐项对照。
    /// Replays the original C `Commutation_SixStep` test vectors.
    ///
    /// 三项断言分别锁定：扇区 1 的桥臂组合 `(A+, B-, C 浮空)`、`sector = 7`
    /// 必须环绕回扇区 1（`rem_euclid` 归一化）、以及 `π/3 + 0.01` 必须落在扇区 2
    /// 且该扇区 C 相为下桥臂（`-1`）。改换相表顺序或改归一化方式都会失配。
    /// The three assertions pin the sector-1 bridge pattern, the wrap of sector 7
    /// back to 1, and the sector-2 pattern at `π/3 + 0.01`.
    #[test]
    fn matches_c_reference_vectors() {
        assert_eq!(
            six_step_from_sector(1),
            SixStepOutput {
                phase_a: 1,
                phase_b: -1,
                phase_c: 0,
                sector: 1
            }
        );
        assert_eq!(six_step_from_sector(7).sector, 1);
        let out = six_step_from_angle(PI / 3.0 + 0.01);
        assert_eq!(out.sector, 2);
        assert_eq!(out.phase_c, -1);
    }
}
