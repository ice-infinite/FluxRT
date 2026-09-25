//! Common/foc_math 的等价实现。
//!
//! FluxRT —— 基础数学与角度处理（`foc-algorithm` 的叶子模块）。
//! FluxRT - base math and angle handling, the leaf module of `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - 提供全库共用的浮点限幅、角度归一化和 `atan2` 包装。
//!   - 定义 `PI`/`TWO_PI`/`SQRT_3`/`INV_SQRT_3` 这些被变换、调制、观测器
//!     反复引用的常量，避免各模块各写一份数值。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `math`（本模块，位于最底层）
//!   本模块不依赖本 crate 的任何其他模块，也不认识 HAL、RTOS、堆或全局状态。
//!   These functions sit at the bottom of the stack and depend on no other
//!   module of this crate; they know nothing about the HAL, the RTOS, the heap
//!   or any global state.
//!
//! 实时约束 / Real-time constraints:
//!   本模块函数会被 12 kHz 的 ADC 注入中断逐拍调用（Clarke/Park、观测器角度
//!   归一化）。全部为纯计算：不分配、不阻塞、不打日志、无锁。
//!   Called every 12 kHz ADC sample by the transform and observer paths. Pure
//!   computation: no allocation, no blocking, no logging, no locking.
//!
//! 量纲约定 / Unit conventions:
//!   角度 `[rad]`，其它标量为无量纲浮点；本模块不引入任何定点 Q 格式。
//!   Angles are `[rad]`; other scalars are dimensionless floats. No fixed-point
//!   Q format appears in this module.
//!
//! 参考 / Reference: docs/架构与安全边界.md, `算法库移植状态.md`（Common 行）

/// 圆周率 π，单位 `[rad]`。
/// Pi in `[rad]`.
///
/// 直接取自 `core::f32::consts`，保证与标准库/编译器常量为同一字面量，
/// 避免自己写 3.14159 造成与 C 参考的末位差异。
/// Taken from `core::f32::consts` so the literal matches the compiler constant
/// exactly instead of a hand-typed approximation.
pub const PI: f32 = core::f32::consts::PI;
/// 2π，一个完整电周期对应的角度，单位 `[rad]`。
/// Two pi, one full electrical revolution, in `[rad]`.
///
/// 用 `TAU` 而不是 `2.0 * PI`，是为了让角度环绕的边界在编译期就是精确常量。
/// Using `TAU` rather than `2.0 * PI` keeps the wrap boundary an exact
/// compile-time constant.
pub const TWO_PI: f32 = core::f32::consts::TAU;
/// √3，无量纲。用于三相/两相变换中的线电压与相电压换算。
/// Square root of three, dimensionless; used by the three-phase/two-phase
/// conversions.
///
/// 来源 / Origin: 本仓库未记录该常量的推导或实测出处；数值等于 √3 在 `f32` 中
/// 可表示的最近值。C 参考库不在本检出范围内，因此无法在此声明两侧共用同一
/// 字面量。
/// The origin of this literal is not recorded in-tree. It equals the nearest
/// `f32` value of √3. The C reference library is not part of this checkout, so a
/// shared literal cannot be claimed here.
pub const SQRT_3: f32 = 1.732_050_8;
/// 1/√3，无量纲。Clarke 变换的幅值不变系数，参见 `crate::transform::clarke`。
/// One over square root of three, dimensionless: the amplitude-invariant Clarke
/// coefficient.
///
/// 与 `SQRT_3` 同为库外来源的常量，本仓库未记录其推导或实测出处；改动它会让
/// `transform::tests::matches_c_reference_vectors` 的参考向量失配。
/// Same unidentified in-tree origin as `SQRT_3`; changing it breaks the reference
/// vectors in `transform::tests::matches_c_reference_vectors`.
pub const INV_SQRT_3: f32 = 0.577_350_26;

/// 限幅：把 `value` 夹到 `[min_value, max_value]`。
/// Clamps `value` into `[min_value, max_value]`.
///
/// 参数 / Parameters:
///   value     任意有限浮点（量纲由调用方决定）
///   min_value 下界（含），与 `max_value` 顺序颠倒时会被自动交换
///   max_value 上界（含）
///
/// 行为与陷阱 / Behaviour and pitfalls:
///   - 上下界写反不会报错，函数内部交换后仍然返回合法结果；
///     测试里的 `clamp(2.0, 1.0, -1.0) == 1.0` 就是为锁定这个行为而存在的。
///   - `NaN` 会穿透所有比较并原样返回，调用方必须自己判 `is_finite()`。
///   - 不做舍入，也不涉及定点；用于电流/电压/占空比的安全边界。
///   - Swapped bounds are silently tolerated; the first three assertions of
///     `matches_c_reference_vectors` pin that behaviour down.
///   - `NaN` passes every comparison and is returned unchanged.
///   - No rounding and no fixed point; this is the safety bound for currents,
///     voltages and duties.
#[inline]
pub fn clamp(value: f32, mut min_value: f32, mut max_value: f32) -> f32 {
    if min_value > max_value {
        core::mem::swap(&mut min_value, &mut max_value);
    }
    if value < min_value {
        min_value
    } else if value > max_value {
        max_value
    } else {
        value
    }
}

/// 把角度归一到 `[0, 2π)`，单位 `[rad]`。
/// Wraps an angle into `[0, 2π)`, in `[rad]`.
///
/// 参数 / Parameters:
///   angle_rad 输入电/机械角度 `[rad]`
///
/// 返回 / Returns:
///   归一化角度 `[rad]`；输入为 `NaN` 或 `±Inf` 时返回 `NaN`。
///   The wrapped angle in `[rad]`, or `NaN` when the input is `NaN`/`±Inf`.
///
/// 行为与陷阱 / Behaviour and pitfalls:
///   - 用**逐次减/加 `2π`** 的循环而不是 `rem_euclid`。这样做与 C 参考实现的
///     舍入行为一致，也让结果在整数倍 `2π` 处精确落在 0（测试依赖这一点）。
///   - 代价是迭代次数随 |角度| 线性增长：`1e6 rad` 需要约 16 万次循环。
///     这是潜在的最坏执行时间（WCET）风险，调用方必须保证输入有界；观测器/
///     PLL 输出应当先做一次粗归一化再调用本函数。
///   - 非有限输入直接返回 `NaN`，避免 `NaN` 静默变成一个"看起来合法"的角度。
///   - Repeated subtraction/addition of `2π` instead of `rem_euclid`, so the
///     rounding behaviour matches the C reference and integer multiples of `2π`
///     land exactly on 0 (the tests rely on this).
///   - The iteration count grows linearly with |angle|: `1e6 rad` needs about
///     160k loops. That is a real WCET risk, so callers must bound the input.
///   - Non-finite input returns `NaN` instead of a plausible-looking angle.
#[inline]
pub fn wrap_angle_0_to_2pi(angle_rad: f32) -> f32 {
    if !angle_rad.is_finite() {
        return f32::NAN;
    }
    let mut wrapped = angle_rad;
    // 两个方向分开处理：先把 >= 2π 的部分减掉，再补齐负角度。
    // 两段都是纯加/减 `2π` 的循环，没有取模，所以精度只受浮点加减的累积误差影响。
    // Two separate directions: first drain the >= 2π part, then top up negatives.
    // Both are plain add/subtract loops, so accuracy depends only on accumulated
    // float add/subtract error.
    while wrapped >= TWO_PI {
        wrapped -= TWO_PI;
    }
    if wrapped < 0.0 {
        while wrapped < 0.0 {
            wrapped += TWO_PI;
        }
    }
    wrapped
}

/// 把角度归一到 `[-π, π)`，单位 `[rad]`。适用于误差/偏差类角度。
/// Wraps an angle into `[-π, π)`, in `[rad]`; meant for error-like angles.
///
/// 参数 / Parameters:
///   angle_rad 输入角度 `[rad]`
///
/// 返回 / Returns: 归一化角度 `[rad]`；非有限输入返回 `NaN`（由内部调用传播）。
/// The wrapped angle in `[rad]`; non-finite input yields `NaN`.
///
/// 实现说明 / Implementation note:
///   先平移 `+π` 再复用 `[0, 2π)` 归一化，最后减回 `π`。这样只需要维护一套
///   环绕逻辑，与 C 参考实现同构；代价是同样继承 `wrap_angle_0_to_2pi` 的
///   循环次数随 |角度| 线性增长，输入必须有界。
///   Shift by `+π`, reuse the `[0, 2π)` wrapper, shift back by `π`, so only one
///   wrapping rule has to be maintained and the C reference stays matched. It
///   inherits the same linear-in-|angle| loop count, so bound the input.
#[inline]
pub fn wrap_angle_minus_pi_to_pi(angle_rad: f32) -> f32 {
    wrap_angle_0_to_2pi(angle_rad + PI) - PI
}

/// `atan2(y, x)` 包装，结果归一到 `[0, 2π)`，单位 `[rad]`。
/// Wraps `atan2(y, x)` into `[0, 2π)`, in `[rad]`.
///
/// 参数 / Parameters:
///   y 矢量 y 分量（量纲任意，只需与 `x` 同量纲）
///   x 矢量 x 分量
///
/// 返回 / Returns: 角度 `[rad]`，落在 `[0, 2π)`；`x == y == 0` 时 `libm` 返回
///   0，经环绕后仍是 0（C 参考同样未定义零矢量方向，调用方需自行排除）。
///   The angle in `[rad]` inside `[0, 2π)`. `x == y == 0` yields 0.
///
/// 量纲 / Units: 输入无单位要求，输出恒为 `[rad]`；常用于把观测器估出的
///   αβ 反电势矢量转成电角度。
/// The inputs are dimensionless and the output is always `[rad]`; typically used
/// to turn the observer's αβ back-EMF vector into an electrical angle.
///
/// 一致性 / Consistency: 使用 `libm::atan2f` 而非宿主数学库，保证主机测试、
///   交叉编译目标和 `no_std` 构建拿到同一实现；换用 `f32::atan2` 会让
///   `matches_c_reference_vectors` 的容差假设失效。
/// Uses `libm::atan2f` so host tests, cross builds and the `no_std` firmware all
/// run the same implementation.
#[inline]
pub fn atan2_angle_0_to_2pi(y: f32, x: f32) -> f32 {
    wrap_angle_0_to_2pi(libm::atan2f(y, x))
}

/// 快速近似 `sin/cos`，用于没有 CORDIC 的 MCU 候选后端。
/// Fast `sin/cos` approximation for MCU targets without CORDIC.
///
/// 输入先归一到 `[-pi, pi)`，再利用象限对称性缩到 `[-pi/2, pi/2]`，最后
/// 分别计算 9 阶正弦与 8 阶余弦多项式。该函数只做纯 `f32` 运算，不调用
/// `libm`、不分配内存，也不包含芯片相关代码。它是一个明确的精度/周期候选，
/// 不是 [`atan2_angle_0_to_2pi`] 等精确软件基线的替代品。
/// The angle is reduced to `[-pi/2, pi/2]` before evaluating ninth/eighth-order
/// sine/cosine polynomials. This is pure `f32` arithmetic with no `libm`, heap,
/// or target dependency. It is an explicit speed/accuracy candidate, not a new
/// definition of the portable reference path.
///
/// 非有限输入返回 `(NaN, NaN)`。实时调用方仍须把输入限制在合理角度范围，
/// 因为底层角度归一化采用有界场景下的逐次加减。
/// Non-finite input yields `(NaN, NaN)`. Realtime callers must still keep the
/// input angle reasonably bounded because wrapping uses repeated add/subtract.
#[inline]
pub fn fast_sin_cos(angle_rad: f32) -> (f32, f32) {
    if !angle_rad.is_finite() {
        return (f32::NAN, f32::NAN);
    }

    let mut reduced = wrap_angle_minus_pi_to_pi(angle_rad);
    let mut cosine_sign = 1.0;
    if reduced > core::f32::consts::FRAC_PI_2 {
        reduced = PI - reduced;
        cosine_sign = -1.0;
    } else if reduced < -core::f32::consts::FRAC_PI_2 {
        reduced = -PI - reduced;
        cosine_sign = -1.0;
    }

    let squared = reduced * reduced;
    let sine = reduced
        * (1.0
            + squared
                * (-1.0 / 6.0
                    + squared
                        * (1.0 / 120.0
                            + squared * (-1.0 / 5_040.0 + squared * (1.0 / 362_880.0)))));
    let cosine = cosine_sign
        * (1.0
            + squared
                * (-1.0 / 2.0
                    + squared
                        * (1.0 / 24.0 + squared * (-1.0 / 720.0 + squared * (1.0 / 40_320.0)))));
    (sine, cosine)
}

/// 快速近似 `atan2(y, x)`，结果落在 `[0, 2*pi)`。
/// Fast approximation of `atan2(y, x)`, wrapped to `[0, 2*pi)`.
///
/// 先把比值限制在 `[0, 1]`，用奇多项式逼近第一象限的 `atan`，再恢复八分区
/// 和象限。这样避免 `libm::atan2f`，同时不会在极陡比值上除以接近零的数。
/// The ratio is restricted to `[0, 1]`, an odd polynomial approximates the
/// first-octant arctangent, and the octant/quadrant is restored afterwards.
/// This avoids both `libm::atan2f` and division by a near-zero minor axis.
///
/// 零矢量沿用基线语义返回 0；任一输入非有限则返回 `NaN`。
/// A zero vector follows the reference convention and returns 0; non-finite
/// inputs return `NaN`.
#[inline]
pub fn fast_atan2_angle_0_to_2pi(y: f32, x: f32) -> f32 {
    if !x.is_finite() || !y.is_finite() {
        return f32::NAN;
    }

    let abs_x = x.abs();
    let abs_y = y.abs();
    if abs_x == 0.0 && abs_y == 0.0 {
        return 0.0;
    }

    #[inline]
    fn atan_first_octant(ratio: f32) -> f32 {
        let squared = ratio * ratio;
        ratio
            * (0.999_866
                + squared
                    * (-0.330_299_5
                        + squared * (0.180_141 + squared * (-0.085_133 + squared * 0.020_835_1))))
    }

    let first_quadrant = if abs_x >= abs_y {
        atan_first_octant(abs_y / abs_x)
    } else {
        core::f32::consts::FRAC_PI_2 - atan_first_octant(abs_x / abs_y)
    };
    let upper_half = if x < 0.0 {
        PI - first_quadrant
    } else {
        first_quadrant
    };
    if y < 0.0 {
        TWO_PI - upper_half
    } else {
        upper_half
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 带容差的浮点比较辅助函数；容差由每个断言显式给出，不使用统一阈值。
    /// Float comparison helper with an explicit per-assertion tolerance.
    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 与 C 版 `Common` 模块原测试向量逐项对照，验证移植的数值等价性。
    /// Replays the original C `Common` test vectors to prove numerical
    /// equivalence of the port.
    ///
    /// 注意 `clamp(2.0, 1.0, -1.0)` 与 `wrap_angle_0_to_2pi(-0.5)` 两项分别锁定
    /// "上下界自动交换"和"负角度按 +2π 补齐"这两个行为；改动实现前先看这里。
    /// Changing either behaviour breaks this test.
    #[test]
    fn matches_c_reference_vectors() {
        assert_near(clamp(2.0, -1.0, 1.0), 1.0, 1e-6);
        assert_near(clamp(-2.0, -1.0, 1.0), -1.0, 1e-6);
        assert_near(clamp(2.0, 1.0, -1.0), 1.0, 1e-6);
        assert_near(wrap_angle_0_to_2pi(TWO_PI), 0.0, 1e-5);
        assert_near(wrap_angle_0_to_2pi(-0.5), TWO_PI - 0.5, 1e-5);
        assert_near(wrap_angle_minus_pi_to_pi(PI + 0.1), -PI + 0.1, 1e-5);
        assert_near(atan2_angle_0_to_2pi(1.0, 0.0), PI * 0.5, 1e-5);
        assert_near(atan2_angle_0_to_2pi(-1.0, 0.0), PI * 1.5, 1e-5);
    }

    /// 在完整电周期上密集对拍快速 sin/cos。阈值不是“宣称等价”，而是锁定
    /// 当前多项式的误差包络，后续换系数时必须显式重新评估。
    #[test]
    fn fast_sin_cos_stays_inside_declared_error_envelope() {
        for index in -16_384..=16_384 {
            let angle = (index as f32) * TWO_PI / 16_384.0;
            let (sin, cos) = fast_sin_cos(angle);
            assert_near(sin, libm::sinf(angle), 4.0e-6);
            assert_near(cos, libm::cosf(angle), 2.6e-5);
        }
        let invalid = fast_sin_cos(f32::INFINITY);
        assert!(invalid.0.is_nan() && invalid.1.is_nan());
    }

    /// 密集扫描单位圆并覆盖轴线/零矢量，锁定快速 atan2 的象限和最大角误差。
    #[test]
    fn fast_atan2_stays_inside_declared_error_envelope() {
        for index in 0..=65_536 {
            let angle = (index as f32) * TWO_PI / 65_536.0;
            let y = libm::sinf(angle);
            let x = libm::cosf(angle);
            let fast = fast_atan2_angle_0_to_2pi(y, x);
            let reference = atan2_angle_0_to_2pi(y, x);
            let mut error = (fast - reference).abs();
            if error > PI {
                error = TWO_PI - error;
            }
            assert!(error <= 1.3e-5, "angle={angle} error={error}");
        }
        assert_eq!(fast_atan2_angle_0_to_2pi(0.0, 0.0), 0.0);
        assert_near(fast_atan2_angle_0_to_2pi(-1.0, 0.0), 1.5 * PI, 1.3e-5);
        assert!(fast_atan2_angle_0_to_2pi(f32::NAN, 1.0).is_nan());
    }
}
