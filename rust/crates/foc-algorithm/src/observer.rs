//! Observer_PLL 的等价实现。
//!
//! 职责 / Responsibility:
//!   - 作为 `observer` 子模块的入口：声明并重导出各观测器子模块
//!   - 提供全库共用的 PLL：对测量角度锁相，输出平滑电角度和电角速度
//! Module entry for the `observer` submodules (declaration plus re-export) and home of
//! the crate-wide PLL, which locks onto a measured angle and outputs a smoothed
//! electrical angle and angular velocity.
//!
//! 架构位置 / Position in the architecture:
//!   - applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control -> 本文件
//!   - `PllState` 被同目录的观测器复用（`observer::smo`、`observer::bemf`），
//!     `foc-control` 也用它做无感后端的角度/速度输出级
//!   - 依赖方向只向下：只依赖本 crate 的 `math` 和 `libm`；无 `unsafe`、无堆、无
//!     HAL/RTOS 依赖
//! End of applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control -> this
//! file. `PllState` is reused by the sibling observers (`observer::smo`,
//! `observer::bemf`) and by `foc-control` as the angle/speed output stage of a
//! sensorless backend. Dependencies point downward only: `math` and `libm`; no `unsafe`,
//! no heap, no HAL/RTOS types.
//!
//! 单位与角度约定 / Units and angle convention:
//!   - 角度 `[rad]`、角速度 `[rad/s]`、采样周期 `[s]`
//!   - `theta_rad` 与 `omega_rad_s` 都是**电角度**量：机械量必须乘/除以极对数
//!     （`foc-control` 用 `omega_rad_s × 30/(π × pole_pairs)` 换算成 `[rpm]`）
//!   - `theta_rad` 始终环回 `[0,2π)`；`omega_min`/`omega_max` 限幅电角速度 `[rad/s]`
//!   - 本文件全部是 f32；没有 Q1.15/Q1.31 定点缩放
//! Angles in `[rad]`, angular velocity in `[rad/s]`, sample period in `[s]`. `theta_rad`
//! and `omega_rad_s` are ELECTRICAL quantities: mechanical values must be multiplied or
//! divided by the pole pairs (`foc-control` converts with
//! `omega_rad_s * 30 / (pi * pole_pairs)` into `[rpm]`). `theta_rad` is always wrapped
//! into `[0, 2*pi)` and `omega_min`/`omega_max` limit the electrical speed in `[rad/s]`.
//! Everything is f32; no Q-format scaling.
//!
//! 实时约束 / Real-time constraints:
//!   - 可在 12 kHz 电流环 ISR 内调用（每拍一次 `sinf`）；无分配、无阻塞、无日志
//!   - 状态必须由唯一上下文持有；观测器与 PLL 在同一拍内串联运行
//! Callable inside the 12 kHz current-loop ISR (one `sinf` per cycle). No allocation, no
//! blocking, no logging. One owner context must own the state; the observer and the PLL
//! run back to back within the same cycle.
//!
//! 迁移与等价性 / Migration and equivalence:
//!   - 对应原 C 库的 `Observer_PLL`；测试 `matches_c_reference_vectors` 沿用原 C 测试
//!     向量（含 `2π` 附近跨界锁定），固定的运算顺序与系数正是等价性成立的原因
//!   - 原 C 参考库未随本仓库提供，因此库内常数的来源类别（`[HW]`/`[ST]`/`[FW]`/
//!     `[VESC]`）无法在本仓库内确认
//! Corresponds to the C module `Observer_PLL`; `matches_c_reference_vectors` reuses its
//! test vectors, including the lock across the `2*pi` wrap, which hold only because of the
//! exact operation order and scaling used here. The C reference library is not shipped in
//! this repository, so the provenance class of the constants
//! (`[HW]`/`[ST]`/`[FW]`/`[VESC]`) cannot be confirmed in-tree.

/// 自适应、高阶与超螺旋滑模观测器。
/// Adaptive, higher-order and super-twisting sliding-mode observers.
pub mod advanced_smo;
/// 反电势、反电势积分、过零检测与 BEMF+PLL 观测器。
/// Back-EMF, integrated BEMF, zero-crossing and BEMF+PLL observers.
pub mod bemf;
/// Kalman、Luenberger、EKF、EKF+FOC 与 UKF 状态估计器。
/// Kalman, Luenberger, EKF, EKF+FOC and UKF state estimators.
pub mod estimator;
/// 电压模型、电流模型、混合与改进积分磁链观测器。
/// Voltage-model, current-model, hybrid and improved-integrator flux observers.
pub mod flux;
/// 高频注入、旋转高频、脉冲注入与 HF/BEMF 融合观测器。
/// High-frequency injection, rotating HF, pulse injection and HF/BEMF fusion observers.
pub mod injection;
/// 滑模观测器、SMO+PLL 与磁链/SMO 融合观测器。
/// Sliding-mode observer, SMO+PLL and flux/SMO fusion observers.
pub mod smo;

// 全部子模块在这里再导出：`observer::smo::SmoState` 与 `observer::SmoState` 指同一个类型。
// Every submodule is re-exported here, so `observer::smo::SmoState` and
// `observer::SmoState` name the same type.
pub use advanced_smo::*;
pub use bemf::*;
pub use estimator::*;
pub use flux::*;
pub use injection::*;
pub use smo::*;

use crate::math::{clamp, wrap_angle_0_to_2pi};

/// PLL 参数：PI 增益、采样周期与电角速度限幅。
/// PLL parameters: PI gains, sample period and electrical-speed limits.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PllParam {
    /// 比例增益 `[1/s]`：相位误差是经 `sin` 的无量纲量，所以增益量纲是 1/时间。
    /// 它决定相位检测器的瞬时刚度；过大时角度噪声会被放大成速度抖动。
    /// Proportional gain in `[1/s]`: the phase error is a dimensionless `sin`, so the gain
    /// has 1/time dimension. It sets the instantaneous stiffness of the detector; too
    /// large a value turns angle noise into speed jitter.
    pub kp: f32,
    /// 积分增益 `[1/s²]`：它把环变成二型环，使稳态角度误差为零。
    /// 纯比例环（`ki = 0`）匀速跟踪时会留下约 `ω/kp` 的固定角度滞后。
    /// Integral gain in `[1/s^2]`, which makes the loop type-II so the steady-state angle
    /// error is zero. A purely proportional loop (`ki = 0`) keeps a constant lag of about
    /// `omega/kp` while tracking a constant speed.
    pub ki: f32,
    /// 采样周期 `[s]`；积分项按 `ki * ts * error` 累加，必须是真实调用周期，否则等效
    /// 积分带宽会被按比例改掉（例如把 12 kHz 的 `ts` 写成 30 kHz 的）。
    /// Sample period in `[s]`. The integrator accumulates `ki * ts * error`, so it must be
    /// the real call period, otherwise the effective integral bandwidth is rescaled (for
    /// example by passing a 30 kHz period to a 12 kHz loop).
    pub ts: f32,
    /// 电角速度下限 `[rad/s]`，通常是最大反转速度的负值。
    /// Lower electrical-speed limit in `[rad/s]`, normally the negative of the maximum
    /// reverse speed.
    pub omega_min: f32,
    /// 电角速度上限 `[rad/s]`。限幅同时充当抗饱和：限幅一旦生效，积分器会被反算回
    /// `omega_clamped - proportional`，不会继续累积。
    /// Upper electrical-speed limit in `[rad/s]`. The clamp also acts as anti-windup: once
    /// it binds, the integrator is back-calculated to `omega_clamped - proportional`
    /// instead of accumulating further.
    pub omega_max: f32,
}

/// PLL 状态：估计角度、估计电角速度、速度积分器与上一拍相位误差。
/// PLL state: estimated angle, estimated electrical speed, the speed integrator and the
/// last phase error.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PllState {
    /// 估计电角度 `[rad]`，始终环回 `[0,2π)`。
    /// Estimated electrical angle in `[rad]`, always wrapped into `[0, 2*pi)`.
    pub theta_rad: f32,
    /// 估计电角速度 `[rad/s]`（限幅后的值）；机械转速还要除以极对数。
    /// Estimated electrical angular velocity in `[rad/s]` (after clamping); divide by the
    /// pole pairs for the mechanical speed.
    pub omega_rad_s: f32,
    /// 速度环 PI 的积分项 `[rad/s]`（不是角度积分），饱和时由反算逻辑修正。
    /// Integral term of the speed PI in `[rad/s]` (not an angle integral), corrected by the
    /// back-calculation at saturation.
    pub integrator: f32,
    /// 上一拍的相位误差 `sin(θ_meas - θ_est)`，无量纲、范围 `[-1,1]`；用于遥测和锁定判据。
    /// Last phase error `sin(theta_meas - theta_est)`, dimensionless in `[-1,1]`, used for
    /// telemetry and lock detection.
    pub phase_error: f32,
}

/// PLL 的状态操作。
/// State operations of the PLL.
impl PllState {
    /// 以给定电角度建立 PLL：角度环回后用 `reset` 初始化，速度与积分器为零。
    /// Creates a PLL at the given electrical angle: the angle is wrapped and initialised
    /// through `reset`, with zero speed and integrator.
    ///
    /// 参数 / Parameters: `initial_theta_rad` `[rad]`，任意值都会被环回 `[0,2π)`。
    /// `initial_theta_rad` in `[rad]`; any value is wrapped into `[0, 2*pi)`.
    #[inline]
    pub fn new(initial_theta_rad: f32) -> Self {
        let mut state = Self::default();
        state.reset(initial_theta_rad);
        state
    }

    /// 复位到指定电角度：环回角度并清零角速度、积分器和相位误差。
    /// Resets to the given electrical angle: the angle is wrapped and the speed, integrator
    /// and phase error are cleared.
    ///
    /// 参数 / Parameters: `theta_rad` `[rad]`，不需要预先环回。
    /// `theta_rad` in `[rad]`; it need not be pre-wrapped.
    ///
    /// 观测器切换、启动和故障恢复后必须调用，否则残留积分器会让 PLL 从错误速度起跑。
    /// Call it after observer switching, startup and fault recovery, otherwise the stale
    /// integrator makes the PLL start from a wrong speed.
    #[inline]
    pub fn reset(&mut self, theta_rad: f32) {
        self.theta_rad = wrap_angle_0_to_2pi(theta_rad);
        self.omega_rad_s = 0.0;
        self.integrator = 0.0;
        self.phase_error = 0.0;
    }

    /// 用一个测量电角度更新 PLL，返回新的估计电角度。
    /// Updates the PLL with one measured electrical angle and returns the new angle
    /// estimate.
    ///
    /// 参数 / Parameters: `param`（`kp` `[1/s]`、`ki` `[1/s²]`、`ts` `[s]`、速度限幅
    /// `[rad/s]`），`theta_meas_rad` `[rad]`，可以是任意实数（差值的正弦是周期函数，
    /// 不需要预先解缠）。
    /// `param` (`kp` in `[1/s]`, `ki` in `[1/s^2]`, `ts` in `[s]`, speed limits in
    /// `[rad/s]`) and `theta_meas_rad` in `[rad]`, which may be any real number: the sine
    /// of the difference is periodic, so no pre-unwrapping is needed.
    ///
    /// 返回 / Returns: 估计电角度 `[rad]`，落在 `[0,2π)`；估计电角速度留在
    /// `PllState::omega_rad_s`。
    /// Estimated electrical angle in `[rad]` inside `[0, 2*pi)`; the estimated speed is
    /// left in `PllState::omega_rad_s`.
    ///
    /// 锁定特性 / Lock behaviour: 相位检测器是 `sin`，锁定点增益为 1，但在 `±π/2` 处增益
    /// 降到 0，所以捕获范围有限：必须先用开环/Rev-Up 把误差拉进 `±π/2` 再切闭环。`sin`
    /// 的周期性同时让 `±2π` 跨界不需要额外处理。
    /// The detector is a `sin` with unity gain at lock but zero gain at `±pi/2`, so the
    /// pull-in range is limited: open loop or Rev-Up must bring the error inside `±pi/2`
    /// before closing the loop. The periodicity of `sin` also removes any need to handle
    /// the `±2*pi` wrap.
    ///
    /// 输出合法性 / Output validity: 本函数既不判断锁定也不校验有限性，调用方必须用
    /// 幅值/方差等可信度判据（见 `foc-control` 的 `ObserverReliabilityConfig`）决定估计
    /// 角度何时可用。
    /// This function neither judges lock nor checks finiteness; the caller must gate the
    /// estimate on an amplitude/variance reliability criterion (see `foc-control`'s
    /// `ObserverReliabilityConfig`).
    #[inline]
    pub fn update(&mut self, param: &PllParam, theta_meas_rad: f32) -> f32 {
        // 相位检测器用 sin(θ_meas - θ_est) 而不是原始角度差：小误差下等价于线性检测，
        // 又天然处理 ±2π 跨界；代价是 ±π/2 附近检测增益下降。
        // The detector uses sin(theta_meas - theta_est) instead of the raw difference: it is
        // linear for small errors and handles the ±2*pi wrap for free, at the cost of
        // reduced gain near ±pi/2.
        self.phase_error = libm::sinf(theta_meas_rad - self.theta_rad);
        // 积分项按 ki*ts 前向欧拉累加；`ts` 必须等于真实调用周期。
        // The integral accumulates ki*ts per sample (forward Euler), so `ts` must be the
        // real call period.
        self.integrator += param.ki * param.ts * self.phase_error;
        let proportional = param.kp * self.phase_error;
        let omega = proportional + self.integrator;
        // 速度限幅 + 反算抗饱和：限幅生效时把积分器改写为“限幅输出 - 比例项”，与
        // `PiState::update` 的处理一致，避免退出限幅时的过冲。
        // Speed clamp plus back-calculation anti-windup: when the clamp binds, the
        // integrator is rewritten as clamped_output - proportional, matching
        // `PiState::update`, so leaving saturation does not overshoot.
        self.omega_rad_s = clamp(omega, param.omega_min, param.omega_max);
        if omega != self.omega_rad_s {
            self.integrator = self.omega_rad_s - proportional;
        }
        // 角度按估计电角速度积分并环回 `[0,2π)`；环回只改变数值域，不改变物理角度。
        // Integrates the angle with the estimated electrical speed and wraps it into
        // `[0, 2*pi)`; wrapping changes the numeric range only, not the physical angle.
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_rad + self.omega_rad_s * param.ts);
        self.theta_rad
    }
}

/// 主机单元测试：核对 PLL 与原 C 参考向量（含 `2π` 附近跨界）的等价性。
/// Host unit tests checking the PLL against the original C reference vectors, including
/// the wrap near `2*pi`.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::TWO_PI;

    /// `kp = 20, ki = 0` 的纯比例 PLL 参考向量。
    /// Reference vector for a purely proportional PLL with `kp = 20, ki = 0`.
    ///
    /// `ki = 0` 让积分器恒为零，结果只由 `kp`、`sin` 和环回逻辑决定；`ts = 0.001` `[s]`
    /// 与 `±200` `[rad/s]` 的速度限幅沿用 C 测试向量，不是整定建议。
    /// `ki = 0` keeps the integrator at zero, so the result depends only on `kp`, `sin` and
    /// the wrap logic. `ts = 0.001` `[s]` and the `±200` `[rad/s]` speed limits come from
    /// the C test vector and are not tuning recommendations.
    #[test]
    fn matches_c_reference_vectors() {
        let p = PllParam {
            kp: 20.0,
            ki: 0.0,
            ts: 0.001,
            omega_min: -200.0,
            omega_max: 200.0,
        };
        let mut state = PllState::new(0.0);
        let before = state.theta_rad;
        // 误差 0.5 rad 时 sin(0.5) > 0，比例项给出正速度，角度必须前进。
        // With a 0.5 rad error, sin(0.5) > 0, so the proportional term gives a positive
        // speed and the angle must advance.
        let after = state.update(&p, 0.5);
        assert!(after > before);
        assert!(state.omega_rad_s > 0.0);
        // 从 2π - 0.05 复位、测量 0.05：跨界后的角度差仍为正的小误差，
        // 验证 `sin` 检测器不会把一整圈当成相位误差。
        // Reset to 2*pi - 0.05 and measure 0.05: after the wrap the difference is still a
        // small positive error, proving the `sin` detector never turns a full revolution
        // into a phase error.
        state.reset(TWO_PI - 0.05);
        state.update(&p, 0.05);
        assert!(state.phase_error > 0.0);
    }
}
