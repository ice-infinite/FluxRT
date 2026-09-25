//! 电压模型、定子电流模型、混合磁链与改进积分器。
//! Voltage model, stator current model, hybrid flux and the improved integrator.
//!
//! 职责 / Responsibility:
//!   - `FluxState`：电压模型，积分 `v - Rs*i` 得到 αβ 磁链（带泄漏和逐轴限幅）
//!   - `FluxCurrentState`：电流模型，用 d/q 电流与 `Ld`/`Lq`/`psi_pm` 算 d/q 磁链
//!   - `FluxHybridState`：按转速在两种模型之间线性加权融合
//!   - `FluxImprovedState`：改进积分器（泄漏 + 矢量幅值限制 + 输出低通）抑制直流漂移
//!   - voltage model, current model, speed-scheduled blend, improved integrator
//!
//! 架构位置 / Architecture position:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!     -> foc-control -> foc-algorithm (本文件 / this file，纯 `no_std` 数学层)
//! 依赖方向 / Dependency direction:
//!   只依赖 crate 内的 `math` 和 `transform`；不认识 ADC/PWM/寄存器，不做单位换算，
//!   不分配内存、不加锁、不打印日志。
//!   Depends only on `math` and `transform`; no HAL, no RTOS, no heap, no locks.
//!
//! 实时约束 / Real-time constraints:
//!   本文件的四个观测器在当前仓库内都没有实时调用方（`smo.rs` 的 `FluxSmoState`
//!   复用了 `FluxParam`/`FluxState`，它同样不在 12 kHz 快环里），目前只有移植测试
//!   覆盖。接入快环前必须按 `算法库实时性说明.md` 用 DWT 实测最坏执行周期。
//!   None of these observers has a real-time caller in this checkout yet, so the WCET
//!   must be measured before they are put into the ISR.
//!
//! 量纲 / Units: 磁链 `[Wb]`、电压 `[V]`、电流 `[A]`、电阻 `[ohm]`、电感 `[H]`、
//!   泄漏系数 `[1/s]`、角度 `[rad]`；融合器的转速单位由调用方约定，见
//!   `FluxHybridParam`。
//! 定点说明 / Fixed point:
//!   本文件全部量都是 `f32`，不存在 Q1.15/Q1.31 定点站点。
//!   Every quantity here is `f32`; this file has no Q1.15/Q1.31 site.
//!
//! 参数来源 / Provenance: ST 参考只记录了 `Rs = 5.29 ohm`、`Ld = Lq = 1.058 mH` 和
//!   线电压常数 5.0 V RMS/kRPM（`[ST]`，见 `docs/ST_MCSDK参考参数与仿真.md`）；
//!   `psi_pm` 的实际辨识值和各限幅的整定值未在仓库内记录（测试里出现的 0.05 只是
//!   测试向量，不是实机参数）。
//!   The ST baseline documents `Rs`, `Ld`/`Lq` and `Ke` only; the identified `psi_pm`
//!   and every limiter value are not recorded in-tree.
//!
//! 参考 / Reference: `算法库移植状态.md`（Observer_Flux* 行）、`算法库实时性说明.md`、
//!   `算法库总览与对接指南.md` §4.4

use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::{AlphaBeta, Dq};

/// 电压模型磁链观测器参数（带泄漏的积分器）。
/// Parameters of the voltage-model flux observer (leaky integrator).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm。
    /// Per-phase stator resistance in `[ohm]`.
    ///
    /// 这是电压模型最大的误差源：偏差 ε 被积分成 `ε*∫i dt`，低频/直流电流分量使
    /// 它单调漂移，直到被 `flux_min`/`flux_max` 截断，而截断期间磁链角是错的。
    /// The dominant error source: a bias integrates into `ε*∫i dt` and drifts until
    /// the clamp clips it.
    pub rs: f32,
    /// 积分步长 `[s]`；写错等于同时改掉积分增益和泄漏截止频率。
    /// Integration step in `[s]`; a wrong value rescales gain and leakage cutoff.
    pub ts: f32,
    /// 泄漏（去直流）系数，量纲 `[1/s]`。
    /// Leakage / DC-rejection coefficient in `[1/s]`.
    ///
    /// 泄漏把纯积分 `1/s` 变成一阶低通 `1/(s + leakage)`，直流增益由无穷降为
    /// `1/leakage`；`0.0` 表示纯积分。泄漏越大漂移越小，但低频的幅值误差和相位
    /// 误差越大。
    /// Turns the pure integrator into a first-order low-pass with DC gain `1/leakage`;
    /// `0.0` means a pure integrator.
    pub leakage: f32,
    /// α/β 磁链下限 `[Wb]`（含端点）。
    /// Lower α/β flux limit in `[Wb]` (inclusive).
    ///
    /// 逐轴限幅不是矢量幅值限幅：某一轴削顶会改变矢量方向，饱和期间角度偏移。
    /// 限幅至少要覆盖真实磁链幅值（`ψpm + Ld*|id|`），否则正常运行就会削顶。
    /// Per-axis clamping rotates the vector; the limits must cover the real flux.
    pub flux_min: f32,
    /// α/β 磁链上限 `[Wb]`（含端点），说明见 `flux_min`。
    /// Upper α/β flux limit in `[Wb]` (inclusive); see `flux_min`.
    pub flux_max: f32,
}

/// 电压模型磁链观测器状态。
/// Voltage-model flux observer state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxState {
    /// αβ 磁链 `[Wb]`（已限幅），也是 `update` 的返回值。
    /// Clamped αβ flux linkage in `[Wb]`; also the value `update` returns.
    pub flux: AlphaBeta,
    /// 磁链矢量角 `[rad]`，范围 `[0, 2π)`。
    /// Flux-vector angle in `[rad]`, in `[0, 2π)`.
    ///
    /// 永磁磁链矢量沿转子 d 轴，所以 `atan2(ψβ, ψα)` 可以直接当电角度用；它与反电势
    /// 观测器给出的角度相差 90°，两者不能混用。
    /// The PM flux vector lies on the rotor d-axis, so this is the electrical angle
    /// directly; it is 90° away from a BEMF-based angle.
    pub theta_flux_rad: f32,
}

/// 一拍输入：αβ 电压 `[V]` 和 αβ 电流 `[A]`。
/// One-sample input: αβ voltage in `[V]` and αβ current in `[A]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxInput {
    /// αβ 相电压 `[V]`；实时路径上通常由 PWM 占空比和母线电压重构，含死区误差和
    /// 一拍延时。
    /// αβ phase voltage in `[V]`, usually reconstructed from PWM duty and the DC bus.
    pub voltage: AlphaBeta,
    /// αβ 电流 `[A]`，实测相电流经 Clarke 变换得到。
    /// αβ current in `[A]` from the measured phase currents (Clarke).
    pub current: AlphaBeta,
}

/// `FluxState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `FluxState`.
impl FluxState {
    /// 清零磁链与角度；应在电机静止、电流为零时复位。
    /// Clears flux and angle; reset at standstill with zero current.
    ///
    /// 电压模型没有对消项，复位后积分从零重建磁链，期间角度不可用。
    /// There is no cancellation term: after reset the angle stays unusable until the
    /// integrator rebuilds the flux.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一步显式欧拉积分 `dψ/dt = v - Rs*i - leakage*ψ`，限幅后求磁链角。
    /// One explicit-Euler step of `dψ/dt = v - Rs*i - leakage*ψ`, clamp, then angle.
    ///
    /// 参数 / Parameters: param - 见 `FluxParam`；input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: 限幅后的 αβ 磁链 `[Wb]`；`param.ts <= 0.0` 时返回零向量且
    ///   不改状态（"参数未配置"的安全返回）。
    ///
    /// 陷阱 / Pitfalls:
    ///   - 逐轴 `clamp` 会改变矢量方向，饱和期间角度偏差；`FluxImprovedState` 用
    ///     矢量幅值限制避免这一点。
    ///   - 使用的电压是上一拍的重构值，不含死区和管压降补偿，低速时相对误差最大。
    ///   - `leakage = 0` 时没有直流反馈路径，任何直流偏置都会积分到限幅为止。
    ///   - Per-axis clamping rotates the vector, the reconstructed voltage lacks
    ///     dead-time compensation, and a pure integrator has no DC feedback path.
    ///
    /// 实时约束 / Real-time: 每拍 4 次乘法、2 次 `clamp`、1 次 `libm::atan2f`。
    pub fn update(&mut self, param: &FluxParam, input: &FluxInput) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.flux.alpha += param.ts
            * (input.voltage.alpha
                - param.rs * input.current.alpha
                - param.leakage * self.flux.alpha);
        self.flux.beta += param.ts
            * (input.voltage.beta - param.rs * input.current.beta - param.leakage * self.flux.beta);
        self.flux.alpha = clamp(self.flux.alpha, param.flux_min, param.flux_max);
        self.flux.beta = clamp(self.flux.beta, param.flux_min, param.flux_max);
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

/// 电流模型磁链参数（d/q 坐标下的 PMSM 磁链模型）。
/// Current-model parameters: the PMSM flux model in dq coordinates.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxCurrentParam {
    /// d 轴电感 `[H]`；`[ST]` 基准 1.058 mH。
    /// d-axis inductance in `[H]`.
    ///
    /// 电感随电流饱和、随温度漂移，而本模型不做在线辨识，所以大电流或高温下模型
    /// 自身就有偏差——这正好与电压模型的 `Rs` 误差互补。
    /// Inductance saturates with current and drifts with temperature; no online
    /// identification is done here, which complements the voltage model's `Rs` error.
    pub ld: f32,
    /// q 轴电感 `[H]`；`Ld != Lq` 时才有磁阻转矩，所以两轴必须分开建模。
    /// q-axis inductance in `[H]`; saliency (`Ld != Lq`) is what creates reluctance
    /// torque, so the two axes must stay separate.
    pub lq: f32,
    /// 永磁磁链 `[Wb]`（`ψpm`）；它不是线电压常数 `Ke`，两者不能混填。
    /// Permanent-magnet flux linkage in `[Wb]`; this is not the line-voltage constant `Ke`.
    ///
    /// ST 参考只给出线电压常数 5.0 V RMS/kRPM（`[ST]`），仓库内没有记录辨识出的
    /// `ψpm`；测试向量里的 0.05 Wb 是测试值，不是实机参数。填错会让 d 轴磁链、
    /// MTPA/MTPV 和转矩估算一起错。
    /// Only `Ke` is documented in-tree; the identified `ψpm` is not, and 0.05 Wb appears
    /// only as a test vector.
    pub flux_pm: f32,
    /// d/q 磁链下限 `[Wb]`（含端点）。
    /// Lower dq flux limit in `[Wb]` (inclusive).
    ///
    /// 两轴共用这一对限幅：d 轴磁链在 `ψpm` 附近，q 轴在 `Lq*iq` 附近，区间必须同时
    /// 覆盖两者，否则负载时 q 轴会被削顶。
    /// Both axes share this pair, so the range must cover `ψpm` and `Lq*iq` together.
    pub flux_min: f32,
    /// d/q 磁链上限 `[Wb]`（含端点），说明见 `flux_min`。
    /// Upper dq flux limit in `[Wb]` (inclusive); see `flux_min`.
    pub flux_max: f32,
}

/// 电流模型磁链状态（d/q 磁链与 dq 坐标系内的磁链相位）。
/// Current-model state: dq flux linkage and the flux phase inside the dq frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxCurrentState {
    /// d/q 磁链 `[Wb]`（已限幅），也是 `update` 的返回值。
    /// Clamped dq flux linkage in `[Wb]`; also the value `update` returns.
    pub flux_dq: Dq,
    /// `atan2(ψq, ψd)`，单位 `[rad]`：dq 坐标系内的磁链相位，不是 αβ 电角度。
    /// `atan2(ψq, ψd)` in `[rad]`: the flux phase inside the dq frame, not an αβ angle.
    ///
    /// 只有调用方做 Park 变换用的角度与真实转子一致时它才接近 0；它反映的是角度估计
    /// 误差，可以当相位误差信号用，但不能直接当电角度输出。
    /// It only approaches zero when the dq frame is aligned with the rotor, so it is an
    /// angle-error signal rather than an electrical angle.
    pub theta_flux_rad: f32,
}

/// `FluxCurrentState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `FluxCurrentState`.
impl FluxCurrentState {
    /// 清零 d/q 磁链与相位。
    /// Clears the dq flux and the phase.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 按 `ψd = Ld*id + ψpm`、`ψq = Lq*iq` 更新 d/q 磁链并求相位。
    /// Updates `ψd = Ld*id + ψpm` and `ψq = Lq*iq`, then computes the phase.
    ///
    /// 参数 / Parameters: param - 见 `FluxCurrentParam`；current_dq - 本拍 d/q 电流 `[A]`。
    /// 返回 / Returns: 限幅后的 d/q 磁链 `[Wb]`。
    ///
    /// 特性 / Properties: 不含 `Rs`、不含积分，所以既没有积分漂移也对电阻误差不敏感；
    /// 代价是完全依赖 `Ld`/`Lq`/`ψpm`，且只在中低速准确（高速时电压模型更可信）。
    /// 它输出的是 dq 量：融合前必须由调用方用同一个角度做反 Park，否则两路矢量不在
    /// 同一坐标系里。
    /// No `Rs` and no integration, so it is drift-free but depends on `Ld`/`Lq`/`ψpm`;
    /// being a dq quantity, it must be inverse-Parked before any αβ blend.
    ///
    /// 实时约束 / Real-time: 每拍 2 次乘法、2 次 `clamp`、1 次 `libm::atan2f`。
    pub fn update(&mut self, param: &FluxCurrentParam, current_dq: Dq) -> Dq {
        self.flux_dq.d = clamp(
            param.ld * current_dq.d + param.flux_pm,
            param.flux_min,
            param.flux_max,
        );
        self.flux_dq.q = clamp(param.lq * current_dq.q, param.flux_min, param.flux_max);
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux_dq.q, self.flux_dq.d);
        self.flux_dq
    }
}

/// 混合磁链融合器的速度调度参数。
/// Speed-scheduling parameters of the hybrid flux blender.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxHybridParam {
    /// 过渡带下边界：低于该速度时权重为 0，完全采用电流模型。
    /// Lower edge of the transition band: below it the current model gets all the weight.
    ///
    /// 速度单位由调用方约定，但 `speed_low`、`speed_high` 和传入的 `speed_abs` 必须
    /// 同单位（`[rpm]` 或 `[rad/s]` 都可以，本文件既不换算也不校验）。
    /// The speed unit is caller-defined but must be identical across all three inputs.
    pub speed_low: f32,
    /// 过渡带上边界：达到或高于该速度时权重为 1，完全采用电压模型。
    /// Upper edge: at or above it the voltage model gets all the weight.
    ///
    /// `speed_high <= speed_low` 时退化为在 `speed_high` 处的硬切换：没有过渡带，
    /// 两个模型的直流漂移不同，切换瞬间磁链矢量和角度可能出现跃变。
    /// `speed_high <= speed_low` degenerates into a hard switch that can step the angle.
    pub speed_high: f32,
}

/// 混合融合器状态。
/// Hybrid blender state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxHybridState {
    /// 融合后的 αβ 磁链 `[Wb]`，也是 `update` 的返回值。
    /// Blended αβ flux linkage in `[Wb]`; also the value `update` returns.
    pub flux: AlphaBeta,
    /// 电压模型权重，无量纲 `[0, 1]`；暴露给上层做可观性/置信度判断。
    /// Voltage-model weight in `[0, 1]`, also exposed for observability logic.
    pub weight_voltage: f32,
    /// 融合后磁链矢量角 `[rad]`，范围 `[0, 2π)`。
    /// Blended flux-vector angle in `[rad]`, in `[0, 2π)`.
    pub theta_flux_rad: f32,
}

/// `FluxHybridState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `FluxHybridState`.
impl FluxHybridState {
    /// 清零磁链、权重和角度。
    /// Clears the flux, the weight and the angle.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 按 `speed_abs` 在两个模型的 αβ 磁链之间线性加权，并输出融合角。
    /// Linearly blends the two models' αβ flux by `speed_abs` and outputs the angle.
    ///
    /// 参数 / Parameters:
    ///   param - 见 `FluxHybridParam`
    ///   flux_voltage - 电压模型磁链 `[Wb]`（通常来自 `FluxState`）
    ///   flux_current - 电流模型磁链 `[Wb]`（电流模型输出 dq，需先反 Park 到 αβ）
    ///   speed_abs - 有符号速度（内部取绝对值），单位与 `speed_low`/`speed_high` 一致
    ///
    /// 返回 / Returns: 融合后的 αβ 磁链 `[Wb]`。
    ///
    /// 权重 / Weight: `w = clamp((|speed| - speed_low)/(speed_high - speed_low), 0, 1)`。
    /// 两个模型的适用区互补——低速时电流模型无漂移，高速时电压模型不受电感饱和
    /// 影响——所以融合在矢量层面是连续的，角度不会跳。真正的不连续出现在退化分支
    /// （`speed_high <= speed_low`）以及两路模型本身不一致时，而低速的电压模型漂移
    /// 正是最不一致的地方。
    /// The two models are complementary, so the αβ blend is continuous as long as both
    /// agree; the discontinuous cases are the degenerate branch and model disagreement.
    ///
    /// 陷阱 / Pitfalls:
    ///   - `clamp` 不拦 `NaN`（见 `crate::math::clamp`）：`speed_abs` 为 NaN 时权重和
    ///     融合结果都会变成 NaN，调用方必须先判有限性。
    ///   - 本函数不做模型有效性检查，也不做权重的速率限制。
    ///   - `NaN` passes through `clamp`, and no model-validity check is performed.
    pub fn update(
        &mut self,
        param: &FluxHybridParam,
        flux_voltage: AlphaBeta,
        flux_current: AlphaBeta,
        speed_abs: f32,
    ) -> AlphaBeta {
        let speed = speed_abs.abs();
        let denom = param.speed_high - param.speed_low;
        self.weight_voltage = if denom <= 0.0 {
            if speed >= param.speed_high {
                1.0
            } else {
                0.0
            }
        } else {
            clamp((speed - param.speed_low) / denom, 0.0, 1.0)
        };
        self.flux.alpha = self.weight_voltage * flux_voltage.alpha
            + (1.0 - self.weight_voltage) * flux_current.alpha;
        self.flux.beta = self.weight_voltage * flux_voltage.beta
            + (1.0 - self.weight_voltage) * flux_current.beta;
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

/// 改进积分器参数：泄漏 + 矢量幅值限制 + 输出低通。
/// Improved-integrator parameters: leakage, vector-magnitude limiting, output low-pass.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxImprovedParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm，误差仍按 `Rs*i` 进入积分。
    /// Per-phase stator resistance in `[ohm]`; its error still enters as `Rs*i`.
    pub rs: f32,
    /// 积分步长 `[s]`；必须等于真实调用周期。
    /// Integration step in `[s]`; it must equal the real call period.
    pub ts: f32,
    /// 泄漏（去直流）系数，量纲 `[1/s]`，作用同 `FluxParam::leakage`。
    /// Leakage coefficient in `[1/s]`, same role as in `FluxParam`.
    pub leakage: f32,
    /// 磁链矢量幅值下限 `[Wb]`；`<= 0.0` 表示不设下限。
    /// Lower bound on the flux vector magnitude in `[Wb]`; `<= 0.0` disables it.
    ///
    /// 幅值为 0 时函数直接返回，所以下限无法把原点处的磁链"拉起来"；它只在幅值已
    /// 非零时防止矢量缩到噪声底以下。
    /// A zero vector returns early, so the floor cannot pull the vector off the origin.
    pub flux_mag_min: f32,
    /// 磁链矢量幅值上限 `[Wb]`；`<= 0.0` 表示不设上限。
    /// Upper bound on the flux vector magnitude in `[Wb]`; `<= 0.0` disables it.
    ///
    /// 超限时按比例缩放 α/β 两个分量，因此保持矢量方向——这是它相对逐轴 `clamp`
    /// 的关键区别，限幅期间角度仍然可用。
    /// Scaling both components preserves the direction, unlike per-axis clamping.
    pub flux_mag_max: f32,
    /// 输出低通系数，无量纲 `[0, 1]`；`1.0` 表示不做平滑。
    /// Output low-pass coefficient in `[0, 1]`; `1.0` disables the smoothing.
    ///
    /// 低通在限幅之后：它滤掉积分的高频纹波，代价是把滞后直接加到输出的磁链角上
    /// （时间常数 `τ = -ts/ln(1-α)`）。
    /// It sits after the limiter and adds lag `τ = -ts/ln(1-α)` to the reported angle.
    pub output_alpha: f32,
}

/// 改进积分器状态。
/// Improved-integrator state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxImprovedState {
    /// 限幅后的积分器状态 `[Wb]`。
    /// Integrator state after limiting, in `[Wb]`.
    ///
    /// 限幅结果直接写回这里（而不是只在输出侧限制），等价于抗积分饱和，所以它不能
    /// 当作"未限幅真值"用于诊断。
    /// The limiter writes back into the integrator, which acts as anti-windup, so this
    /// is not the unlimited "true" value for diagnostics.
    pub raw_flux: AlphaBeta,
    /// 低通后的输出磁链 `[Wb]`，也是 `update` 的返回值；角度取自它。
    /// Filtered output flux in `[Wb]`; the returned value and the angle source.
    pub flux: AlphaBeta,
    /// 输出磁链矢量角 `[rad]`，范围 `[0, 2π)`；与 `FluxState` 一样是 d 轴方向。
    /// Output flux-vector angle in `[rad]`, in `[0, 2π)`, again the d-axis direction.
    pub theta_flux_rad: f32,
}

/// 改进积分器的输入类型别名（复用电压模型的 αβ 电压 `[V]` / 电流 `[A]` 输入）。
/// Input alias for the improved integrator, reusing the voltage model's αβ input.
pub type FluxImprovedInput = FluxInput;

/// 按矢量幅值等比缩放 `value`，把模长夹到 `[min_mag, max_mag]`。
/// Scales `value` to clamp its magnitude into `[min_mag, max_mag]`.
///
/// `min_mag`/`max_mag` 为 `<= 0.0` 时分别表示该侧不限制；幅值为 0 时直接返回，既
/// 避免除零，也意味着原点处的矢量不会被下限抬起。
/// Non-positive bounds disable that side; a zero vector returns early to avoid a
/// division by zero.
///
/// `NaN` 不会被拦截：所有比较都为假，最终 `scale` 也是 `NaN`，所以调用方必须保证
/// 电压/电流是有限值。
/// `NaN` is not intercepted; the caller must pass finite values.
fn limit_vector_magnitude(value: &mut AlphaBeta, min_mag: f32, max_mag: f32) {
    let magnitude = libm::sqrtf(value.alpha * value.alpha + value.beta * value.beta);
    if magnitude <= 0.0 {
        return;
    }
    let mut target = magnitude;
    if max_mag > 0.0 && target > max_mag {
        target = max_mag;
    }
    if min_mag > 0.0 && target < min_mag {
        target = min_mag;
    }
    let scale = target / magnitude;
    value.alpha *= scale;
    value.beta *= scale;
}

/// `FluxImprovedState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `FluxImprovedState`.
impl FluxImprovedState {
    /// 清零积分状态、输出磁链和角度。
    /// Clears the integrator state, the output flux and the angle.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一步"泄漏积分 + 矢量幅值限制 + 输出低通"。
    /// One step of leaky integration, vector-magnitude limiting and output low-pass.
    ///
    /// 参数 / Parameters: param - 见 `FluxImprovedParam`；input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: 低通后的 αβ 磁链 `[Wb]`；`param.ts <= 0.0` 时返回零向量且
    ///   不改状态。
    ///
    /// 顺序 / Order: 先积分 `v - Rs*i - leakage*ψ`，再对积分状态做矢量幅值限制，最后
    /// 经一阶低通输出。顺序有意义：限幅写回积分状态等价于抗积分饱和（限幅期间不会
    /// 继续累积），低通放在限幅之后保证角度不跟随削顶的瞬时跳变。
    /// The limiter writes back into the integrator (anti-windup) and the LPF follows it.
    ///
    /// 与 `FluxState` 的差别 / Difference from `FluxState`: 用矢量幅值限制代替逐轴
    /// `clamp`（保方向），并额外用泄漏 + 输出低通压制直流漂移；代价是相位滞后和幅值
    /// 误差——`leakage`、`output_alpha` 越大滞后越小、漂移抑制越弱，需要按工作转速
    /// 权衡。
    /// Magnitude limiting preserves direction; the leak and output LPF suppress DC drift
    /// at the cost of lag and amplitude error.
    ///
    /// 实时约束 / Real-time: 每拍一次 `libm::sqrtf`、一次 `libm::atan2f` 和若干乘加，
    ///   无分配、无阻塞。
    pub fn update(&mut self, param: &FluxImprovedParam, input: &FluxImprovedInput) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.raw_flux.alpha += param.ts
            * (input.voltage.alpha
                - param.rs * input.current.alpha
                - param.leakage * self.raw_flux.alpha);
        self.raw_flux.beta += param.ts
            * (input.voltage.beta
                - param.rs * input.current.beta
                - param.leakage * self.raw_flux.beta);
        limit_vector_magnitude(&mut self.raw_flux, param.flux_mag_min, param.flux_mag_max);
        let alpha = clamp(param.output_alpha, 0.0, 1.0);
        self.flux.alpha += alpha * (self.raw_flux.alpha - self.flux.alpha);
        self.flux.beta += alpha * (self.raw_flux.beta - self.flux.beta);
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    /// 浮点近似比较助手：容差由调用方给出，用来吸收最后一位的舍入差异。
    /// Approximate-float comparison helper; the tolerance absorbs last-bit rounding.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 对照 C 参考向量的电压模型积分测试，并验证 `flux_max` 处的逐轴限幅。
    /// C reference vectors for the voltage-model integration and its per-axis clamp.
    #[test]
    fn flux_matches_c_reference() {
        let p = FluxParam {
            rs: 0.5,
            ts: 0.01,
            leakage: 0.0,
            flux_min: -1.0,
            flux_max: 1.0,
        };
        let mut input = FluxInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 1.0,
            },
            current: AlphaBeta {
                alpha: 1.0,
                beta: 0.0,
            },
        };
        let mut state = FluxState::default();
        let out = state.update(&p, &input);
        near(out.alpha, 0.015, 1e-6);
        near(out.beta, 0.01, 1e-6);
        input.voltage.alpha = 1000.0;
        near(state.update(&p, &input).alpha, 1.0, 1e-6);
    }

    /// 对照 C 参考向量的电流模型测试：验证 `ψd = Ld*id + ψpm`、`ψq = Lq*iq` 与限幅。
    /// C reference vectors for the dq current model and its clamp.
    #[test]
    fn current_model_matches_c_reference() {
        let p = FluxCurrentParam {
            ld: 0.001,
            lq: 0.002,
            flux_pm: 0.05,
            flux_min: -0.1,
            flux_max: 0.1,
        };
        let mut state = FluxCurrentState::default();
        let out = state.update(&p, Dq { d: 10.0, q: 20.0 });
        near(out.d, 0.06, 1e-6);
        near(out.q, 0.04, 1e-6);
        near(state.update(&p, Dq { d: 1000.0, q: 20.0 }).d, 0.1, 1e-6);
    }

    /// 对照 C 参考向量的混合融合测试：低速全电流模型、过渡带中点各半、高速全电压模型。
    /// C reference vectors for the blend: 100 % current model, 50/50, 100 % voltage model.
    #[test]
    fn hybrid_matches_c_reference() {
        let p = FluxHybridParam {
            speed_low: 100.0,
            speed_high: 300.0,
        };
        let voltage = AlphaBeta {
            alpha: 1.0,
            beta: 0.0,
        };
        let current = AlphaBeta {
            alpha: 0.0,
            beta: 1.0,
        };
        let mut state = FluxHybridState::default();
        assert_eq!(state.update(&p, voltage, current, 50.0), current);
        let mid = state.update(&p, voltage, current, 200.0);
        near(mid.alpha, 0.5, 1e-6);
        near(mid.beta, 0.5, 1e-6);
        near(state.theta_flux_rad, PI * 0.25, 1e-6);
        assert_eq!(state.update(&p, voltage, current, 400.0), voltage);
    }

    /// 对照 C 参考向量的改进积分器测试，锁住"限幅写回积分状态、低通在限幅之后"的顺序。
    /// C reference vectors that pin the limiter/low-pass ordering.
    #[test]
    fn improved_integrator_matches_c_reference() {
        let p = FluxImprovedParam {
            rs: 0.5,
            ts: 0.01,
            leakage: 0.0,
            flux_mag_min: 0.0,
            flux_mag_max: 0.02,
            output_alpha: 0.5,
        };
        let mut input = FluxImprovedInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 0.0,
            },
            current: AlphaBeta::default(),
        };
        let mut state = FluxImprovedState::default();
        let out = state.update(&p, &input);
        near(state.raw_flux.alpha, 0.02, 1e-6);
        near(out.alpha, 0.01, 1e-6);
        input.voltage.alpha = 100.0;
        let out = state.update(&p, &input);
        near(state.raw_flux.alpha, 0.02, 1e-6);
        near(out.alpha, 0.015, 1e-6);
    }
}
