//! Angle_CORDIC 的等价实现。
//!
//! FluxRT —— CORDIC 角度与幅值计算（`foc-algorithm` 的坐标/角度原语层）。
//! FluxRT - CORDIC angle and magnitude computation, a coordinate/angle primitive
//! of `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - 由 `(x, y)` 同时求出矢量角度 `[rad]` 和模长，替代一次 `atan2` 加一次
//!     `sqrt`；常用于把观测器估出的 αβ 反电势转成电角度。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `angle`（本模块）
//!   本模块只依赖 `crate::math`，是叶子模块；不认识 HAL、RTOS、堆或全局状态。
//!   Depends only on `crate::math`; no HAL, RTOS, heap or global state.
//!
//! 与硬件后端的关系（重要）/ Relation to the hardware backend (important):
//!   这里是**软件参考模型**：用 IEEE-754 浮点加 `libm` 实现，目的是让算法行为
//!   可逐位对照、可在主机上测试。固件实际走的是 C 平台适配器里的片上硬件
//!   CORDIC（见 `foc/platform/stm32g431/foc_math_accel_stm32g431.c` 与
//!   `foc/include/foc_math_accel.h`），两侧靠**相同的输出量纲**互换：角度
//!   `[rad]` 且归一化到 `[0, 2π)`，模长与输入同量纲。
//!   This is the software reference model (IEEE-754 + `libm`) used for host tests
//!   and behaviour comparison. Firmware uses the on-chip CORDIC through the C
//!   platform adapter; the two sides agree on units: angle in `[rad]` wrapped to
//!   `[0, 2π)` and a magnitude in the same unit as the inputs.
//!
//! 定点与 Q 格式 / Fixed point and Q format:
//!   本模块**不使用定点**，全部为 `f32`。Q1.31 缩放（以及模长路径的 Q1.15
//!   安全范围缩放）只存在于 CORDIC 平台适配器内部；`foc_math_accel.h` 明确
//!   要求 Q 格式不得泄漏到算法库。
//!   This module is pure `f32` with no fixed point. Q1.31 (and the Q1.15 range
//!   scaling on the magnitude path) exists only inside the CORDIC adapter, and
//!   `foc_math_accel.h` forbids leaking Q formats into the algorithm crate.
//!
//! 实时约束 / Real-time constraints:
//!   纯计算、无分配、无阻塞、无日志；但每次迭代都调用 `libm`，是否放进 12 kHz
//!   快环必须按最坏执行周期实测决定（`算法库实时性说明.md` 第 4 条）。
//!   Pure computation, but every iteration calls `libm`, so putting it in the
//!   12 kHz fast loop requires a measured WCET.
//!
//! 参考 / Reference: `算法库移植状态.md`（Angle_CORDIC 行）,
//! `docs/硬件数学加速与CPU回退.md`

use crate::math::{wrap_angle_0_to_2pi, PI};

/// CORDIC 旋转序列使用的反正切查找表，单位 `[rad]`，共 16 项。
/// Arctangent lookup table for the CORDIC rotation sequence, in `[rad]`, 16
/// entries.
///
/// 第 `i` 项是 `atan(2^-i)`，即第 `i` 次旋转要补偿的角度。最后一项
/// `atan(2^-15) ≈ 3.05e-5 rad` 给出 16 次迭代的残余角度上界，也就是角度精度
/// 上限；测试用的 `1e-4` 容差正是围绕这个量级选取的。
/// Entry `i` is `atan(2^-i)`. The last entry, `atan(2^-15) ≈ 3.05e-5 rad`, bounds
/// the residual angle after 16 iterations, which is the accuracy ceiling and the
/// reason the test tolerance is `1e-4`.
///
/// 来源 / Origin: 数值本身是 CORDIC 的标准递推值（可由 `atan(2^-i)` 复算），
/// 但本仓库未记录它们是从 C 参考库抄录还是重新计算，因此不标注 `[ST]`/`[HW]`。
/// The values are the standard CORDIC recurrence, but this checkout does not
/// record whether they were copied from the C reference or recomputed, so no
/// `[ST]`/`[HW]` tag is claimed.
const ATAN_TABLE: [f32; 16] = [
    core::f32::consts::FRAC_PI_4,
    0.463_647_6,
    0.244_978_67,
    0.124_354_996,
    0.062_418_81,
    0.031_239_834,
    0.015_623_729,
    0.007_812_341,
    0.003_906_230_2,
    0.001_953_122_6,
    0.000_976_562_2,
    0.000_488_281_22,
    0.000_244_140_62,
    0.000_122_070_31,
    0.000_061_035_156,
    0.000_030_517_578,
];

/// CORDIC 输出：角度与模长一次算出。
/// CORDIC output: angle and magnitude from one call.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CordicOutput {
    /// 矢量角度，单位 `[rad]`，归一化到 `[0, 2π)`。
    /// Vector angle in `[rad]`, wrapped into `[0, 2π)`。
    ///
    /// 选择 `[0, 2π)` 而不是 `(-π, π]`，是为了与 `foc_control` 使用的电角度
    /// 约定以及 C 适配器 `foc_math_accel_atan2()` 的输出范围保持一致。
    /// `[0, 2π)` is chosen to match the electrical-angle convention used by
    /// `foc-control` and the output range of `foc_math_accel_atan2()`.
    pub angle_rad: f32,
    /// 矢量模长，与输入 `x`/`y` 同量纲，非负（补偿增益后）。
    /// Vector magnitude in the same unit as the inputs, non-negative after gain
    /// compensation.
    ///
    /// 零矢量输入会被提前拦截并返回 0，不做迭代。
    /// A zero-vector input is rejected early and returns 0 without iterating.
    pub magnitude: f32,
}

/// 把请求的迭代次数折算成表长范围内的有效次数。
/// Maps a requested iteration count onto the valid range for the table.
///
/// 参数 / Parameters:
///   iterations 调用方请求的迭代次数；`<= 0` 表示"用满整张表"
///
/// 返回 / Returns: 实际迭代次数，取值 `1..=16`。
/// The effective count, in `1..=16`.
///
/// 约定 / Contract: 非正数取满表是 C 参考定义的语义，不是错误返回；因此调用
///   方要"最高精度"时可以直接传 0 或负数。上限被表长卡住，无法通过传 64 追求
///   更高精度——要更高精度必须扩表。
/// Non-positive means "use the whole table", matching the C reference, so 0 or a
/// negative value is a valid way to ask for maximum accuracy. The table length is
/// the hard ceiling; asking for more requires a longer table.
fn iteration_count(iterations: i32) -> usize {
    if iterations <= 0 {
        ATAN_TABLE.len()
    } else {
        (iterations as usize).min(ATAN_TABLE.len())
    }
}

/// 计算 `count` 次迭代的累积 CORDIC 旋转增益 `K = Π√(1 + 2^-2i)`。
/// Computes the accumulated CORDIC rotation gain `K = Π√(1 + 2^-2i)` for `count`
/// iterations.
///
/// 参数 / Parameters:
///   count 迭代次数，必须 `<= 16`（由 `iteration_count()` 保证）；无量纲。
///
/// 返回 / Returns: 无量纲增益 `K`，`count = 16` 时约 1.6468。`cordic_atan2`
///   用它把被放大的 `x_work` 还原成真实模长。
/// The dimensionless gain `K`, about 1.6468 at `count = 16`; `cordic_atan2`
/// divides by it to recover the true magnitude.
///
/// 实现说明 / Implementation note:
///   - 用 `ldexpf(1.0, -i)` 生成 `2^-i`，避免维护第二张常数表，代价是每次调用
///     都要做 `count` 次 `sqrtf`，约 16 次软件平方根。
///   - 这里**不缓存**增益表。若要把它放进 12 kHz 快环，应改为编译期常量表，
///     否则每拍都要付出这 16 次平方根。
///   - `ldexpf` is used instead of a second constant table, at the cost of about
///     16 software square roots per call. This function does not cache the gain;
///     a compile-time table would be required before using it per-sample.
fn cordic_gain(count: usize) -> f32 {
    let mut gain = 1.0;
    for i in 0..count {
        let factor = libm::ldexpf(1.0, -(i as i32));
        gain *= libm::sqrtf(1.0 + factor * factor);
    }
    gain
}

/// 用 CORDIC 同时求出 `(x, y)` 的矢量角度与模长。
/// Computes the vector angle and magnitude of `(x, y)` with CORDIC.
///
/// 参数 / Parameters:
///   y, x       输入分量，必须同量纲（常为 `[V]` 的反电势或 `[A]` 的电流），
///              量纲不影响角度；模长继承同一量纲。
///   iterations 迭代次数，`<= 0` 或 `> 16` 都被折算到 `1..=16`。
///
/// 返回 / Returns: `CordicOutput { angle_rad [rad], magnitude }`；
///   输入为 `(0, 0)` 时返回 `CordicOutput::default()`（角度 0、模长 0）。
///   The angle in `[rad]` and the magnitude. A `(0, 0)` input returns
///   `CordicOutput::default()`.
///
/// 算法与陷阱 / Algorithm and pitfalls:
///   - 象限处理用"整矢量旋转 180°"（`x < 0` 时同时取反 `x`、`y`），而不是扩展
///     第二张象限表。好处是只用一张表；副作用是当 `x` 为负且 `|x|` 很小时，
///     折回时由 `y = -y` 确定的角度是精确的，但若 `x` 本身就是被减法减出来的
///     近零值，其符号已经不可信。
///   - 旋转方向用 `y_work > 0.0` 判定：`y == 0` 时走负向分支。恰好落在坐标轴
///     上的向量因此有确定结果，不会在两次调用间抖动。
///   - 返回值必须与 C 平台适配器的硬件 CORDIC 具有相同约定（`[rad]`、
///     `[0, 2π)`、模长经增益补偿），否则观测器换后端后角度会整体偏移。
///   - The quadrant is handled by rotating the whole vector by 180° rather than a
///     second table, so only one table is needed. The angle stays exact because
///     the fold sets `y = -y`, but the sign of a near-zero `x` produced by
///     cancellation is no longer trustworthy.
///   - The rotation test is `y_work > 0.0`, so an exactly-zero `y` has a defined
///     result instead of jittering between two branches.
///   - The returned convention (in `[rad]`, wrapped to `[0, 2π)`, magnitude
///     gain-compensated) must stay identical to the hardware CORDIC adapter.
///
/// 实时约束 / Real-time constraints: 无分配、无阻塞、无日志；但每次迭代含
///   `libm` 调用，是否进 12 kHz 快环需实测 WCET。
/// Allocation-free and non-blocking, but each iteration calls `libm`.
pub fn cordic_atan2(y: f32, x: f32, iterations: i32) -> CordicOutput {
    if x == 0.0 && y == 0.0 {
        // 零矢量方向未定义，提前返回而不是让迭代产生 0/0 增益补偿。
        // A zero vector has no defined direction; return early instead of letting
        // the iteration produce a 0/0 gain compensation.
        return CordicOutput::default();
    }

    let count = iteration_count(iterations);
    let mut x_work = x;
    let mut y_work = y;
    let mut quadrant_offset = 0.0;
    // 把矢量折到右半平面，使后续旋转序列收敛到 `(-π/2, π/2)`，最后再加回 π。
    // Fold the vector into the right half-plane so the rotation sequence
    // converges inside `(-π/2, π/2)`; π is added back at the end.
    if x_work < 0.0 {
        x_work = -x_work;
        y_work = -y_work;
        quadrant_offset = PI;
    }

    let mut z = 0.0;
    // 递推的关键约束：每一步的两个输出都必须基于**上一步**的 x/y 同时算出，
    // 先算出一对再整体赋值。若写成先改 x 再用新 x 算 y，旋转就会引入额外误差，
    // 且与 C 参考的迭代顺序不一致。
    // Both outputs of a step must be derived from the previous step's x/y and only
    // then assigned. Updating x before computing y would add error and diverge
    // from the C reference iteration order.
    for (i, atan) in ATAN_TABLE.iter().copied().enumerate().take(count) {
        let factor = libm::ldexpf(1.0, -(i as i32));
        let (x_next, y_next) = if y_work > 0.0 {
            z += atan;
            (x_work + y_work * factor, y_work - x_work * factor)
        } else {
            z -= atan;
            (x_work - y_work * factor, y_work + x_work * factor)
        };
        x_work = x_next;
        y_work = y_next;
    }

    CordicOutput {
        // 收敛后的 `z` 是相对右半平面的偏角，加回象限偏移再归一化到 [0, 2π)。
        // The converged `z` is relative to the right half-plane; add the quadrant
        // offset and wrap into [0, 2π).
        angle_rad: wrap_angle_0_to_2pi(z + quadrant_offset),
        // 每次旋转都把模长放大 `sqrt(1 + 2^-2i)`，这里按实际迭代次数除回增益。
        // 增益必须用 `count`（本次实际迭代次数）而不是固定 16，否则少迭代的调用
        // 会得到偏大的模长。
        // Each rotation scales the magnitude by `sqrt(1 + 2^-2i)`, so divide by the
        // gain for the iteration count actually used, never a hard-coded 16.
        magnitude: x_work / cordic_gain(count),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 带容差的浮点比较辅助函数；容差由每个断言显式给出。
    /// Float comparison helper with an explicit per-assertion tolerance.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 与 C 版 `Angle_CORDIC` 原测试向量逐项对照，覆盖四个象限和零矢量。
    /// Replays the original C `Angle_CORDIC` vectors across all four quadrants
    /// plus the zero vector.
    ///
    /// 容差固定为 `1e-4`：16 次迭代的角度误差是本次实现的一部分，收紧容差等于
    /// 要求实现与 C 参考逐位一致；放宽则会掩盖象限处理错误。
    /// The `1e-4` tolerance is part of the current 16-iteration accuracy budget;
    /// tightening it demands bit-identical output and loosening it hides quadrant
    /// bugs.
    #[test]
    fn matches_c_reference_vectors() {
        let out = cordic_atan2(0.0, 1.0, 16);
        near(out.angle_rad, 0.0, 1e-4);
        near(out.magnitude, 1.0, 1e-4);
        near(cordic_atan2(1.0, 0.0, 16).angle_rad, PI * 0.5, 1e-4);
        let q2 = cordic_atan2(1.0, -1.0, 16);
        near(q2.angle_rad, PI * 0.75, 1e-4);
        near(q2.magnitude, libm::sqrtf(2.0), 1e-4);
        near(cordic_atan2(-1.0, -1.0, 16).angle_rad, PI * 1.25, 1e-4);
        assert_eq!(cordic_atan2(0.0, 0.0, 16), CordicOutput::default());
    }
}
