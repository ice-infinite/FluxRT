//! FluxRT —— 硬实时电流环使用的可替换数学运算后端。
//! FluxRT - replaceable math operations used by the hard real-time current loop.
//!
//! 本模块只定义一个极小的 trait，把"控制算法"与"具体 MCU 的数学外设"解耦：
//! 主机测试与不带数学外设的 MCU 用 [`CpuMath`]（`libm` 软件实现），
//! STM32G431 由平台适配层用 CORDIC + FPU 实现同一套契约。
//! The control algorithm depends on this small trait instead of an MCU. Host
//! tests and MCUs without a math peripheral use [`CpuMath`]. A board adapter may
//! implement the same contract with CORDIC, an FPU, another accelerator, or a DSP library.
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   foc-rt-bridge -> `foc-control`（本模块）-> foc-algorithm
//!   `foc-rt-bridge` 里的 `PlatformMath` 是本 trait 的实机实现，并负责在
//!   CORDIC 不可用或返回非有限值时回退到 [`CpuMath`]。
//!   `PlatformMath` in `foc-rt-bridge` is the on-target implementation and falls
//!   back to [`CpuMath`] when CORDIC is unavailable or returns non-finite values.
//!
//! 实时约束 / Real-time constraints:
//!   每个电流控制周期调用一次 `sin_cos` 与一次 `magnitude`，`angle_0_to_2pi`
//!   只在观测器里用到。全部必须无分配、无阻塞、无日志。软件 `sinf`/`cosf` 是
//!   快环里最贵的操作（数百周期），所以 Park/逆 Park 共用一组 sin/cos，不要在
//!   这里叠加额外三角函数调用。
//!   One sin/cos pair and one magnitude per current sample; `angle_0_to_2pi` is
//!   used by the observer only. All must be allocation-free, non-blocking and
//!   log-free. Software `sinf`/`cosf` dominate fast-loop cost, so Park and inverse
//!   Park share one pair instead of adding extra trig calls.
//!
//! 量纲 / Units: 角度一律为弧度 `[rad]`；`magnitude` 的输入与输出同量纲，快环
//!   里是电压 `[V]`。本模块全为 `f32`，不使用 Q 格式。
//!   Angles are radians `[rad]`; `magnitude` preserves the caller's unit, volts
//!   `[V]` in the fast loop. Everything is `f32`; no Q formats.
//!
//! 参考 / Reference: docs/硬件数学加速与CPU回退.md

/// 每个电流控制周期需要一次的数学运算契约。
/// Math required once per current-control sample.
///
/// 参数与返回值都是 `f32`，且实现不得 panic：`no_std` 目标上没有可依赖的展开
/// 路径，后端遇到不可用外设时要么自行回退，要么返回 `NaN` 让调用方的有限性
/// 检查来拒绝这一拍。
/// All arguments and results are `f32` and implementations must not panic: there
/// is no dependable unwinding path on the `no_std` target, so a backend either
/// falls back internally or returns `NaN` for the caller's finiteness check.
pub trait ControlMath {
    /// 返回 `(sin(angle), cos(angle))`，`angle` 单位 `[rad]`。
    /// Returns `(sin(angle), cos(angle))`.
    ///
    /// Park 与逆 Park 共用这一组结果，因此每个采样周期只应调用一次。
    /// Park and inverse Park share one result, so call it once per sample.
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32);

    /// 返回 `sqrt(x*x + y*y)`，量纲与 `x`/`y` 相同（快环里是电压 `[V]`）。
    /// Returns `sqrt(x*x + y*y)`.
    ///
    /// 用于电压圆限幅：结果与限幅值比较，超限时按同一比例缩放 d/q 电压。
    /// Used by the voltage circle limiter, which compares the result against the
    /// limit and scales the d/q voltage by that ratio.
    fn magnitude(&mut self, x: f32, y: f32) -> f32;

    /// 返回 `atan2(y, x)`，并归一化到 `[0, 2*pi)`，单位 `[rad]`。
    /// Returns `atan2(y, x)` wrapped to `[0, 2*pi)`.
    ///
    /// 观测器需要"反电势矢量方向"这个绝对角度；用 atan2 而不是增量角度，
    /// 是为了让 PLL 的相位误差可以用两个角度直接相减得到。
    /// The observer needs the absolute EMF vector direction so the PLL phase error
    /// is a plain subtraction of two angles.
    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32;
}

/// 默认实现，也是硬件后端不可用时的回退路径。
/// Portable implementation used by default and as the hardware fallback.
///
/// 基于 `libm` 的软件浮点，在不带数学加速外设时是快环最贵的部分；
/// 实机只在 CORDIC 返回非有限值时才走这条路径。
/// Based on software `libm` floats, the most expensive part of the fast loop
/// without a math peripheral; the target only reaches it when CORDIC returns a
/// non-finite value.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuMath;

impl ControlMath for CpuMath {
    /// 见 [`ControlMath::sin_cos`]；这里是一次 `libm` 软件实现。
    /// See [`ControlMath::sin_cos`]; this is the software `libm` implementation.
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        (libm::sinf(angle_rad), libm::cosf(angle_rad))
    }

    /// 见 [`ControlMath::magnitude`]；这里是一次 `libm` 软件开方。
    /// See [`ControlMath::magnitude`]; this is the software `libm` square root.
    fn magnitude(&mut self, x: f32, y: f32) -> f32 {
        libm::sqrtf(x * x + y * y)
    }

    /// 见 [`ControlMath::angle_0_to_2pi`]；复用 `foc-algorithm` 的归一化约定，
    /// 避免本 crate 再实现一份角度回绕逻辑。
    /// See [`ControlMath::angle_0_to_2pi`]; reuses `foc-algorithm`'s wrapping
    /// convention so this crate does not fork that logic.
    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
        foc_algorithm::atan2_angle_0_to_2pi(y, x)
    }
}

/// 不依赖数学外设的快速近似后端。
/// Portable fast-approximation backend that needs no math peripheral.
///
/// `sin/cos` 与 `atan2` 使用 `foc-algorithm` 中经过误差回归的纯浮点多项式；
/// 模长仍保留 `sqrtf` 基线，目标平台可继续用 FPU `VSQRT.F32` 覆盖该操作。
/// 它只在显式候选配置中使用，默认控制路径仍为 [`CpuMath`] 或平台 CORDIC。
/// `sin/cos` and `atan2` use the error-regressed pure-float polynomials from
/// `foc-algorithm`; magnitude retains the portable `sqrtf` baseline and may be
/// overridden by a target FPU adapter. This backend is opt-in only.
#[derive(Clone, Copy, Debug, Default)]
pub struct FastApproxMath;

impl ControlMath for FastApproxMath {
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        foc_algorithm::fast_sin_cos(angle_rad)
    }

    fn magnitude(&mut self, x: f32, y: f32) -> f32 {
        libm::sqrtf(x * x + y * y)
    }

    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
        foc_algorithm::fast_atan2_angle_0_to_2pi(y, x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 锁定 CPU 后端的象限、符号与量纲：`sin(pi/2) = 1`、`cos(pi/2) = 0`、
    /// `|(3,4)| = 5`、`atan2(1,0) = pi/2`。CORDIC 后端必须给出同样的结果，
    /// 因此这是"加速器与回退路径一致"的基线。
    /// Pins the CPU backend's quadrant, sign and magnitude: `sin(pi/2) = 1`,
    /// `cos(pi/2) = 0`, `|(3,4)| = 5` and `atan2(1,0) = pi/2`. The CORDIC backend
    /// must agree with this, making it the accelerator/fallback baseline.
    #[test]
    fn cpu_backend_matches_expected_quadrant_and_magnitude() {
        let mut math = CpuMath;
        let (sin, cos) = math.sin_cos(core::f32::consts::FRAC_PI_2);
        assert!((sin - 1.0).abs() < 1e-6);
        assert!(cos.abs() < 1e-6);
        assert!((math.magnitude(3.0, 4.0) - 5.0).abs() < 1e-6);
        assert!((math.angle_0_to_2pi(1.0, 0.0) - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn fast_backend_matches_cpu_contract_with_bounded_error() {
        let mut fast = FastApproxMath;
        let mut reference = CpuMath;
        for index in 0..=8_192 {
            let angle = (index as f32) * core::f32::consts::TAU / 8_192.0;
            let (fast_sin, fast_cos) = fast.sin_cos(angle);
            let (reference_sin, reference_cos) = reference.sin_cos(angle);
            assert!((fast_sin - reference_sin).abs() <= 4.0e-6);
            assert!((fast_cos - reference_cos).abs() <= 2.6e-5);

            let fast_angle = fast.angle_0_to_2pi(reference_sin, reference_cos);
            let mut angle_error = (fast_angle - angle).abs();
            if angle_error > core::f32::consts::PI {
                angle_error = core::f32::consts::TAU - angle_error;
            }
            assert!(angle_error <= 1.3e-5);
        }
        assert!((fast.magnitude(3.0, 4.0) - 5.0).abs() < 1e-6);
    }
}
