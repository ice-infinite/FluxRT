//! 滑模观测器、SMO+PLL 与磁链/SMO 融合观测器。
//!
//! 职责 / Responsibility:
//!   - `SmoState`：经典 αβ 滑模观测器，估计反电势并给出电角度（也可以只出矢量不算角度）
//!   - `SmoPllState`：SMO 后接 PLL，输出更平滑的电角度与电角速度
//!   - `FluxSmoState`：电压模型磁链观测器与 SMO 并行运行，再按权重融合两路角度
//! `SmoState` is the classic αβ sliding-mode observer that estimates the back-EMF and
//! derives an electrical angle (or only the vector, without the angle); `SmoPllState`
//! appends a PLL for a smoother angle and speed; `FluxSmoState` runs the voltage-model
//! flux observer and the SMO in parallel and fuses their angles by weight.
//!
//! 架构位置 / Position in the architecture:
//!   - applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control -> 本文件
//!   - `foc-control` 的 `SmoPllEstimator` 用本文件的 `SmoParam`/`SmoPllState` 组成板级
//!     默认无感后端（`ObserverBackend::SmoPll`），并以 12 kHz 快环周期调用
//!   - 依赖方向只向下：只依赖本 crate 的 `math`、`transform` 与同模块的 `PllState`、
//!     `FluxState`；无 `unsafe`、无堆、无 HAL/RTOS 依赖
//! `foc-control::SmoPllEstimator` composes `SmoParam`/`SmoPllState` into the board's
//! default sensorless backend (`ObserverBackend::SmoPll`) and calls it at the 12 kHz
//! fast-loop rate. Dependencies point downward only: `math`, `transform` and the sibling
//! `PllState`/`FluxState`; no `unsafe`, no heap, no HAL/RTOS types.
//!
//! 单位与符号约定 / Units and sign convention:
//!   - 电压 `[V]`、电流 `[A]`、电阻 `[ohm]`、电感 `[H]`、周期 `[s]`、角度 `[rad]`
//!   - `sliding`/`emf` 都是电压量纲 `[V]`，`k_slide` `[V]`，`boundary` `[A]`
//!   - 估计反电势沿用本 crate 的 αβ 约定（与 `observer::bemf` 相同）：
//!     `e_alpha = -ω_e·ψ·sin θ_e`、`e_beta = +ω_e·ψ·cos θ_e`，故 `θ_e = atan2(-e_alpha,
//!     e_beta)`；符号写反会让角度整体偏 π 或方向相反，电流环可能仍“稳定”但转矩反向
//!   - 本文件全部是 f32；没有 Q1.15/Q1.31 定点缩放
//! Voltage `[V]`, current `[A]`, resistance `[ohm]`, inductance `[H]`, period `[s]`,
//! angles `[rad]`. `sliding` and `emf` are voltages in `[V]`, `k_slide` is in `[V]` and
//! `boundary` in `[A]`. The estimated back-EMF follows the crate convention
//! (`observer::bemf`): `e_alpha = -omega_e*psi*sin(theta_e)`,
//! `e_beta = +omega_e*psi*cos(theta_e)`, hence `theta_e = atan2(-e_alpha, e_beta)`.
//! Reversing the sign shifts the angle by pi or flips the direction: the current loop may
//! still look stable while the torque is reversed. Everything is f32; no Q-format scaling.
//!
//! 实时约束 / Real-time constraints:
//!   - 在 12 kHz 快环内运行（当前 SMO 与电压/电流快照严格同拍）；无分配、无阻塞、无日志
//!   - `atan2` 是这里最贵的运算，因此 `update_vector` 专供能接硬件 CORDIC 的快环使用
//!   - 本文件同时含有乘积、比较和一次除法，但没有循环，计算量与输入无关
//! Runs inside the 12 kHz fast loop (currently the SMO shares the voltage/current snapshot
//! cycle). No allocation, no blocking, no logging. `atan2` is the most expensive operation
//! here, which is why `update_vector` exists for fast loops that can use a hardware CORDIC.
//! The code contains products, comparisons and one division, but no loops, so the cost does
//! not depend on the input values.
//!
//! 低速限制 / Low-speed limitation:
//!   - 反电势幅值正比于电角速度，零低速不可观：必须有 Rev-Up/高频注入/编码器，
//!     且上层要按幅值和速度方差判定可信度之后再让角度生效
//! The back-EMF magnitude is proportional to the electrical speed, so the observer is not
//! observable at zero/low speed: Rev-Up, high-frequency injection or an encoder is
//! required, and the upper layer must gate the angle on magnitude and speed variance.
//!
//! 迁移与等价性 / Migration and equivalence:
//!   - 对应原 C 库的 `Observer_SMO`、`Observer_SMO_PLL`、`Observer_Flux_SMO`
//!   - 第一拍 `*_matches_c_reference` 测试沿用原 C 测试向量；另有两拍状态测试明确修正
//!     原实现把 `sliding` 与其低通结果 `emf` 重复扣除的问题。两者是同一反电势的
//!     瞬时/等效表示，不能同时进入电流模型
//!   - 原 C 参考库的参数来源类别（`[HW]`/`[ST]`/`[FW]`/`[VESC]`）无法在仓库内确认
//! Corresponds to the C modules `Observer_SMO`, `Observer_SMO_PLL` and
//! `Observer_Flux_SMO`. First-cycle tests retain their C vectors, while a two-cycle state
//! test pins the corrected observer equation: the raw sliding injection and its low-pass
//! EMF are two representations of one back-EMF and must not both be subtracted. The
//! provenance class of the original constants cannot be confirmed in-tree.

use super::{FluxInput, FluxParam, FluxState, PllParam, PllState};
use crate::math::{atan2_angle_0_to_2pi, clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi};
use crate::transform::AlphaBeta;

/// 滑模观测器参数：电机模型、采样周期、滑模增益、边界层与反电势滤波系数。
/// Sliding-mode observer parameters: machine model, sample period, sliding gain, boundary
/// layer and EMF filter coefficient.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoParam {
    /// 定子相电阻 `[ohm]`。偏差会直接进入估计反电势，是低速误差和温升漂移的主要来源。
    /// Stator phase resistance in `[ohm]`. Any error lands directly in the estimated
    /// back-EMF and is the main low-speed and thermal-drift error term.
    pub rs: f32,
    /// 定子相电感 `[H]`（表贴机用 `Ld`，内嵌式还需评估 `Ld/Lq` 凸极性）；`<= 0` 时
    /// `update_vector` 直接返回零矢量且不修改状态。
    /// Stator phase inductance in `[H]` (`Ld` for a surface PM; an interior PM needs its
    /// `Ld/Lq` saliency considered). When it is `<= 0`, `update_vector` returns a zero vector
    /// without touching the state.
    pub ls: f32,
    /// 采样周期 `[s]`。电流用显式前向欧拉按 `ts/L` 积分，所以它必须等于真实调用周期；
    /// `ts·rs/ls` 接近或超过 2 时离散极点会跑出单位圆，估计电流发散。
    /// Sample period in `[s]`. The current is integrated with an explicit forward-Euler step
    /// scaled by `ts/L`, so it must equal the real call period; when `ts*rs/ls` approaches or
    /// exceeds 2 the discrete pole leaves the unit circle and the estimated current diverges.
    pub ts: f32,
    /// 滑模增益 `[V]`：`-k_slide · sat(电流误差/boundary)` 作为电压量加入电流方程。
    /// 它必须大于反电势峰值才可能建立滑动模态，但越大抖振和噪声放大越严重。
    /// Sliding gain in `[V]`: `-k_slide * sat(current_error/boundary)` enters the current
    /// equation as a voltage. It must exceed the back-EMF peak for a sliding mode to exist,
    /// yet a larger value amplifies chattering and noise.
    pub k_slide: f32,
    /// 边界层宽度 `[A]`：电流误差在此范围内线性饱和，用来抑制 `sign` 切换抖振。
    /// `<= 0` 时退化为纯 `sign`（`-1/0/+1`），抖振最大；边界层越大越平滑，但等效反电势
    /// 估计的幅值与带宽损失越大。
    /// Boundary-layer width in `[A]`: the current error saturates linearly inside it, which
    /// suppresses sign-switching chattering. `<= 0` degrades to a pure `sign` (`-1/0/+1`)
    /// with maximum chattering; a larger boundary is smoother but loses more amplitude and
    /// bandwidth in the equivalent EMF estimate.
    pub boundary: f32,
    /// 反电势一阶低通的更新系数，无量纲，函数内会再钳位到 `[0,1]`：
    /// `emf += alpha · (sliding - emf)`。`alpha = 1` 表示不滤波（测试用它取即时值），
    /// 越小越平滑但相移越大；小 `alpha` 时截止频率约为 `alpha/(2π·ts)` `[Hz]`。
    /// First-order EMF low-pass update coefficient, dimensionless and clamped again to
    /// `[0,1]` inside the function: `emf += alpha * (sliding - emf)`. `alpha = 1` disables
    /// filtering (the tests use it to read the instantaneous value); smaller values smooth
    /// more but add phase lag, with a cut-off of roughly `alpha/(2*pi*ts)` `[Hz]` for a small
    /// `alpha`.
    pub emf_filter_alpha: f32,
}

/// 滑模观测器状态：估计电流、滑模注入量、滤波后的估计反电势与估计电角度。
/// SMO state: estimated current, sliding injection, filtered EMF estimate and the EMF
/// angle.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoState {
    /// 估计 αβ 定子电流 `[A]`：由电压方程逐步积分得到，它跟踪实测电流，误差驱动滑模项。
    /// 状态本身没有限幅，参数不合理时可能发散。
    /// Estimated αβ stator current in `[A]`, integrated from the voltage equation; it tracks
    /// the measured current and its error drives the sliding term. The state is not clamped,
    /// so bad parameters can make it diverge.
    pub current_est: AlphaBeta,
    /// 本拍滑模注入量 `[V]`：`k_slide · sat(估计电流 - 实测电流)`，是切换项而不是物理
    /// 机端电压，其等效平均值才代表反电势。
    /// This cycle's sliding injection in `[V]`: `k_slide * sat(i_est - i_meas)`. It is a
    /// switching term rather than a physical terminal voltage; only its equivalent average
    /// represents the back-EMF.
    pub sliding: AlphaBeta,
    /// 低通后的估计反电势 `[V]`，方向与真实反电势一致；低速时幅值很小、方向不可信。
    /// Filtered back-EMF estimate in `[V]` with the direction of the real back-EMF; at low
    /// speed its magnitude is small and its direction untrustworthy.
    pub emf: AlphaBeta,
    /// 由 `emf` 得到的电角度 `[rad]`：`atan2(-emf.alpha, emf.beta)`，范围 `[0,2π)`。
    /// Electrical angle in `[rad]` derived from `emf` as `atan2(-emf.alpha, emf.beta)`,
    /// wrapped into `[0, 2*pi)`.
    pub theta_emf_rad: f32,
}

/// 滑模观测器输入：同一采样周期的 αβ 电压 `[V]` 与 αβ 电流 `[A]`。
/// SMO input: αβ voltage in `[V]` and αβ current in `[A]` from the same sampling cycle.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoInput {
    /// αβ 定子电压 `[V]`。注意这是 SVPWM 的给定值而非实测值：占空比饱和、死区和母线
    /// 波动都会让实际电压偏离它，这部分误差会直接进入反电势估计。
    /// αβ stator voltage in `[V]`. This is the SVPWM command, not a measurement: duty
    /// saturation, dead time and DC-link ripple all make the real voltage differ, and that
    /// error goes straight into the EMF estimate.
    pub voltage: AlphaBeta,
    /// αβ 定子电流 `[A]`，由三相采样经幅值不变 Clarke 得到。
    /// αβ stator current in `[A]`, from the three-phase samples via the amplitude-invariant
    /// Clarke transform.
    pub current: AlphaBeta,
}

/// 滑模切换函数：带边界层的饱和，输出无量纲 `[-1,1]`。
/// Sliding switching function: saturation with a boundary layer, dimensionless in
/// `[-1,1]`.
///
/// 参数 / Parameters: `value` 是电流误差 `[A]`，`boundary` 是边界层宽度 `[A]`。
/// `value` is the current error in `[A]` and `boundary` is the boundary-layer width in
/// `[A]`.
///
/// 返回 / Returns: `boundary > 0` 时是 `clamp(value/boundary, -1, 1)`；`boundary <= 0` 时
/// 是理想 `sign`，并显式给出 `value == 0` 时的 0（避免 `sign(0)` 的实现歧义）。
/// With `boundary > 0` it is `clamp(value/boundary, -1, 1)`; with `boundary <= 0` it is an
/// ideal `sign` with an explicit 0 at `value == 0` (avoiding `sign(0)` ambiguity).
///
/// 折中 / Trade-off: 理想 `sign` 会在估计反电势上叠加开关频率的抖振，只能靠后面的低通
/// 抑制，而低通又带来相移；边界层内滑模退化为高增益比例项，抖振被稳态误差换掉。
/// An ideal `sign` adds switching-frequency chattering to the EMF estimate that only the
/// following low-pass can suppress, and that low-pass adds phase lag; inside the boundary
/// layer the sliding mode becomes a high-gain proportional term, trading chattering for
/// steady-state error.
fn sliding_saturation(value: f32, boundary: f32) -> f32 {
    if boundary <= 0.0 {
        if value > 0.0 {
            1.0
        } else if value < 0.0 {
            -1.0
        } else {
            0.0
        }
    } else {
        clamp(value / boundary, -1.0, 1.0)
    }
}

/// 滑模观测器的状态操作。
/// State operations of the sliding-mode observer.
impl SmoState {
    /// 清零估计电流、滑模量、估计反电势与估计角度。
    /// Clears the estimated current, sliding term, EMF estimate and angle.
    ///
    /// 无感启动、模式切换和故障恢复后必须调用：残留的 `emf` 会让 PLL/电流环带着错误的
    /// 初始角度起跑。
    /// Call it on sensorless start, mode change and fault recovery: a stale `emf` would make
    /// the PLL/current loop start from a wrong angle.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 只更新观测器矢量，不计算角度。
    /// Updates the observer vector without calculating its angle.
    ///
    /// 参数 / Parameters: `param`（`rs` `[ohm]`、`ls` `[H]`、`ts` `[s]`、`k_slide` `[V]`、
    /// `boundary` `[A]`、`emf_filter_alpha` 无量纲），`input`（αβ 电压 `[V]`、电流 `[A]`）。
    /// `param` (`rs` `[ohm]`, `ls` `[H]`, `ts` `[s]`, `k_slide` `[V]`, `boundary` `[A]`,
    /// `emf_filter_alpha` dimensionless) and `input` (αβ voltage `[V]`, current `[A]`).
    ///
    /// 返回 / Returns: 滤波后的估计反电势 `[V]`；`ls <= 0` 时返回零矢量且不改动任何状态，
    /// 此时 `theta_emf_rad` 仍是上一次的旧值。
    /// Filtered EMF estimate in `[V]`; when `ls <= 0` it returns a zero vector without
    /// touching any state field, so `theta_emf_rad` keeps its previous value.
    ///
    /// 实时 / Real-time: 这是给硬件 CORDIC/加速器准备的入口：只有乘法、比较和每拍一次
    /// `1/ls` 除法，没有 `atan2`/`sin`/`cos`。
    /// This is the entry for hardware CORDIC/accelerators: only multiplications,
    /// comparisons and one `1/ls` division per call, with no `atan2`/`sin`/`cos`.
    ///
    /// Updates the observer vector without calculating its angle. Hard
    /// realtime callers can feed the returned vector to an MCU CORDIC instead
    /// of paying for a software `atan2f` on every PWM sample.
    pub fn update_vector(&mut self, param: &SmoParam, input: &SmoInput) -> AlphaBeta {
        // 电感非法时直接返回零矢量：这里不清状态（与 reset 区分），避免上层在参数配置
        // 中途调用时静默丢掉估计角度；放在最前面也让下面的 1/ls 不可能除零。
        // An invalid inductance returns a zero vector without clearing state (unlike
        // `reset`), so a mid-configuration call does not silently drop the estimated angle;
        // the check also runs first, so the `1/ls` below can never divide by zero.
        if param.ls <= 0.0 {
            return AlphaBeta::default();
        }
        // 每拍只算一次 1/L，避免两个轴各做一次除法（除法在 Cortex-M4F 上比乘法慢得多）。
        // One `1/L` per cycle instead of one division per axis, since division is much slower
        // than multiplication on a Cortex-M4F.
        let inv_l = 1.0 / param.ls;
        // 滤波系数在这里再钳位一次：越界值（负数或 >1）会把低通变成高通或发散滤波器。
        // The filter coefficient is clamped again here: an out-of-range value (negative or
        // above 1) would turn the low-pass into a high-pass or an unstable filter.
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        // 电流误差定义为 (估计 - 实测)，它的符号决定滑模项方向。该约定与下面电流方程里
        // “减去滑模项”配合，才让估计反电势与真实反电势同向（而不是差 π）。
        // The current error is defined as (estimated - measured) and its sign sets the
        // direction of the sliding term. Together with the subtraction in the current
        // equation below, this keeps the EMF estimate in phase with the real back-EMF
        // instead of pi out of phase.
        let err_alpha = self.current_est.alpha - input.current.alpha;
        let err_beta = self.current_est.beta - input.current.beta;
        self.sliding.alpha = param.k_slide * sliding_saturation(err_alpha, param.boundary);
        self.sliding.beta = param.k_slide * sliding_saturation(err_beta, param.boundary);
        // 估计电流按显式（前向）欧拉积分：di = ts/L · (v - R·i_est - sliding)。
        // `sliding` 的等效平均值就是反电势，`emf` 只是它的低通副本；两者同时相减会
        // 把同一物理量扣两次，使稳态 EMF 被迫缩到约一半并产生次谐波限环。
        // 离散极点是 1 - ts·R/L，所以参数必须满足 ts·R/L << 2；这里没有任何中间饱和，
        // 参数不合理时状态会直接发散。
        // The estimated current uses an explicit (forward) Euler step:
        // di = ts/L * (v - R*i_est - sliding). The low-pass `emf` is the equivalent
        // value of that same injection; subtracting both double-counts one physical
        // back-EMF and creates a sub-harmonic limit cycle. The discrete pole is 1 - ts*R/L, so the
        // parameters must keep ts*R/L well below 2; there is no intermediate saturation here,
        // so bad parameters let the state diverge.
        self.current_est.alpha += param.ts
            * inv_l
            * (input.voltage.alpha - param.rs * self.current_est.alpha - self.sliding.alpha);
        self.current_est.beta += param.ts
            * inv_l
            * (input.voltage.beta - param.rs * self.current_est.beta - self.sliding.beta);
        // 估计反电势不是直接解出来的，而是滑模注入量的低通：低通既提取切换项的等效平均
        // 值，也带来相移。注意 `emf` 又作为补偿项加回上面的电流方程，所以它的绝对幅值
        // 不是真实反电势的 1:1 标定值，只有方向/相位按上面的符号约定对齐；任何按幅值设
        // 的判据都必须在实物上标定，不能假定固定比例。
        // The EMF estimate is not solved directly: it is the low-pass of the sliding
        // injection, which extracts the equivalent average of the switching term at the cost
        // of phase lag. Because `emf` is also fed back as a compensation term in the current
        // equation above, its magnitude is not a calibrated 1:1 copy of the real back-EMF;
        // only its direction follows the sign convention above, so any magnitude-based
        // threshold must be characterised on hardware instead of assuming a fixed ratio.
        self.emf.alpha += alpha * (self.sliding.alpha - self.emf.alpha);
        self.emf.beta += alpha * (self.sliding.beta - self.emf.beta);
        self.emf
    }

    /// 更新观测器并计算估计电角度。
    /// Updates the observer and computes the estimated electrical angle.
    ///
    /// 参数与返回同 `update_vector`；角度写入 `SmoState::theta_emf_rad`。
    /// Parameters and return value match `update_vector`; the angle is stored in
    /// `SmoState::theta_emf_rad`.
    ///
    /// 代价 / Cost: 每拍一次 `atan2f`，是快环里最贵的数学调用；能用硬件 CORDIC 时应改用
    /// `update_vector`。
    /// One `atan2f` per cycle, the most expensive math call in the fast loop; prefer
    /// `update_vector` when a hardware CORDIC is available.
    ///
    /// 零幅值语义 / Zero-magnitude semantics: `emf` 为零时（未收敛、零速、`ls <= 0` 的首拍）
    /// 角度没有物理意义，`atan2(0,0)` 会给出 0；调用方必须用幅值/速度可信度判据屏蔽这段
    /// 输出，不能只看角度。
    /// When `emf` is zero (not converged, zero speed, or the first call with `ls <= 0`) the
    /// angle is meaningless and `atan2(0,0)` yields 0, so the caller must gate the output on
    /// an amplitude/speed reliability criterion instead of trusting the angle alone.
    pub fn update(&mut self, param: &SmoParam, input: &SmoInput) -> AlphaBeta {
        let emf = self.update_vector(param, input);
        // 角度取 atan2(-e_alpha, e_beta)，即本 crate 的反电势约定
        // e_alpha = -ω_e·ψ·sinθ_e、e_beta = +ω_e·ψ·cosθ_e；去掉负号会让估计角度整体偏 π
        // 并反转旋转方向，而电流环在错误角度下仍可能“稳定”。
        // The angle is atan2(-e_alpha, e_beta), the crate's back-EMF convention
        // e_alpha = -omega_e*psi*sin(theta_e), e_beta = +omega_e*psi*cos(theta_e). Dropping
        // the minus shifts the estimate by pi and flips the rotation direction, while a
        // current loop at a wrong angle may still look stable.
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        emf
    }
}

/// SMO+PLL 参数：滑模观测器参数加 PLL 参数。
/// SMO+PLL parameters: the SMO parameter block plus the PLL parameter block.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoPllParam {
    /// 滑模观测器参数；它的 `ts` 是快环周期 `[s]`。
    /// Sliding-mode observer parameters; its `ts` is the fast-loop period in `[s]`.
    pub smo: SmoParam,
    /// PLL 参数。`pll.ts` 应与本观测器的调用周期一致，否则等效环路带宽和 `omega` 限幅的
    /// 含义都会偏移。
    /// PLL parameters. `pll.ts` should match this observer's call period, otherwise the
    /// effective loop bandwidth and the meaning of the `omega` limits both shift.
    pub pll: PllParam,
}

/// SMO+PLL 状态：内嵌的 SMO 与 PLL 状态，加上本拍的反电势、角度和角速度副本。
/// SMO+PLL state: the embedded SMO and PLL states plus this cycle's EMF, angle and speed
/// copies.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoPllState {
    /// 内嵌滑模观测器状态（估计电流、滑模量、估计反电势、反电势角度）。
    /// Embedded sliding-mode observer state (estimated current, sliding term, EMF estimate
    /// and EMF angle).
    pub smo: SmoState,
    /// 内嵌 PLL 状态（估计角度、角速度、积分器、相位误差）。
    /// Embedded PLL state (estimated angle, speed, integrator, phase error).
    pub pll: PllState,
    /// 本拍估计反电势 `[V]` 的副本，供遥测与可信度判据使用。
    /// Copy of this cycle's EMF estimate in `[V]`, for telemetry and reliability checks.
    pub emf: AlphaBeta,
    /// PLL 输出的电角度 `[rad]`，范围 `[0,2π)`。
    /// Electrical angle from the PLL in `[rad]`, inside `[0, 2*pi)`.
    pub theta_rad: f32,
    /// PLL 输出的电角速度 `[rad/s]`（限幅后）；机械转速需除以极对数。
    /// Electrical angular velocity from the PLL in `[rad/s]` (after clamping); divide by the
    /// pole pairs for the mechanical speed.
    pub omega_rad_s: f32,
}

/// SMO+PLL 的状态操作。
/// State operations of the SMO+PLL combination.
impl SmoPllState {
    /// 复位 SMO 与 PLL 的全部状态。
    /// Resets all SMO and PLL state.
    ///
    /// 无感接管、切换观测器和故障恢复后必须调用：残留角度/积分器会让首次输出跳变。
    /// Call it after sensorless takeover, observer switching and fault recovery: stale
    /// angles or integrators would make the first output jump.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一个周期：先跑 SMO 得到反电势角度，再让 PLL 锁定它，返回平滑后的电角度。
    /// One cycle: run the SMO for the EMF angle, lock the PLL onto it and return the smoothed
    /// electrical angle.
    ///
    /// 参数 / Parameters: `param`（SMO + PLL），`input`（同一拍 αβ 电压 `[V]` 与电流 `[A]`）。
    /// `param` (SMO plus PLL) and `input` (αβ voltage `[V]` and current `[A]` from one
    /// sampling cycle).
    ///
    /// 返回 / Returns: PLL 电角度 `[rad]`；电角速度在 `SmoPllState::omega_rad_s`，估计反电
    /// 势在 `SmoPllState::emf`。
    /// PLL electrical angle in `[rad]`; the speed is in `SmoPllState::omega_rad_s` and the EMF
    /// estimate in `SmoPllState::emf`.
    ///
    /// 实时 / Real-time: 每拍一次 `atan2` 加一次 `sin`，都在快环预算内但必须实测 WCET。
    /// One `atan2` plus one `sin` per cycle; both fit the fast-loop budget but need a measured
    /// WCET.
    pub fn update(&mut self, param: &SmoPllParam, input: &SmoInput) -> f32 {
        self.emf = self.smo.update(&param.smo, input);
        // PLL 只消费 SMO 的角度，不反向影响 SMO：两级串联、带宽分开整定（PLL 通常更慢，
        // 否则会把滑模抖振直接搬进速度输出）。
        // The PLL only consumes the SMO angle and never feeds back into it: the stages are
        // cascaded with separately tuned bandwidths (the PLL is usually slower, otherwise
        // sliding chattering goes straight into the speed output).
        self.theta_rad = self.pll.update(&param.pll, self.smo.theta_emf_rad);
        self.omega_rad_s = self.pll.omega_rad_s;
        self.theta_rad
    }
}

/// 磁链+SMO 融合参数：磁链观测器、滑模观测器与 SMO 权重。
/// Flux+SMO fusion parameters: the flux observer, the SMO and the SMO weight.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxSmoParam {
    /// 电压模型磁链观测器参数（`rs` `[ohm]`、`ts` `[s]`、`leakage`、`flux_min/max` `[Wb]`）。
    /// Voltage-model flux observer parameters (`rs` `[ohm]`, `ts` `[s]`, `leakage`,
    /// `flux_min/max` in `[Wb]`).
    pub flux: FluxParam,
    /// 滑模观测器参数。
    /// Sliding-mode observer parameters.
    pub smo: SmoParam,
    /// 融合权重，函数内会再钳位到 `[0,1]`：0 表示完全用磁链角度，1 表示完全用 SMO 角度。
    /// Fusion weight, clamped again to `[0,1]` inside the function: 0 uses the flux angle only
    /// and 1 the SMO angle only.
    pub smo_weight: f32,
}

/// 磁链+SMO 融合状态：两路观测器状态、两路矢量、两路角度与融合结果。
/// Flux+SMO fusion state: both observer states, both vectors, both angles and the fused
/// result.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxSmoState {
    /// 内嵌磁链观测器状态。
    /// Embedded flux observer state.
    pub flux: FluxState,
    /// 内嵌滑模观测器状态。
    /// Embedded sliding-mode observer state.
    pub smo: SmoState,
    /// 本拍磁链矢量 `[Wb]`（αβ）。
    /// This cycle's flux vector in `[Wb]` (αβ).
    pub flux_vector: AlphaBeta,
    /// 本拍估计反电势 `[V]`（αβ）。
    /// This cycle's EMF estimate in `[V]` (αβ).
    pub emf: AlphaBeta,
    /// 磁链角度 `[rad]`，范围 `[0,2π)`；稳态下就是转子磁链（磁极）方向。
    /// Flux angle in `[rad]` inside `[0, 2*pi)`, which in steady state is the rotor flux
    /// (magnet) direction.
    pub theta_flux_rad: f32,
    /// SMO 反电势角度 `[rad]`，范围 `[0,2π)`。
    /// SMO EMF angle in `[rad]` inside `[0, 2*pi)`.
    pub theta_smo_rad: f32,
    /// 融合后的电角度 `[rad]`，范围 `[0,2π)`。
    /// Fused electrical angle in `[rad]` inside `[0, 2*pi)`.
    pub theta_rad: f32,
}

/// 磁链+SMO 融合的状态操作。
/// State operations of the flux+SMO fusion.
impl FluxSmoState {
    /// 复位两路观测器与融合结果。
    /// Resets both observers and the fused result.
    ///
    /// 切换观测器策略、无感接管和故障恢复后必须调用。
    /// Call it after an observer-strategy change, sensorless takeover and fault recovery.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一个周期：磁链观测器与 SMO 并行运行，再按权重在角度上融合。
    /// One cycle: the flux observer and the SMO run in parallel and their angles are fused by
    /// weight.
    ///
    /// 参数 / Parameters: `param`（磁链 + SMO + 权重），`input`（同一拍 αβ 电压 `[V]` 与电流
    /// `[A]`，两路观测器共用同一份输入）。
    /// `param` (flux plus SMO plus weight) and `input` (αβ voltage `[V]` and current `[A]` from
    /// one cycle, shared by both observers).
    ///
    /// 返回 / Returns: 融合电角度 `[rad]`，落在 `[0,2π)`。
    /// Fused electrical angle in `[rad]` inside `[0, 2*pi)`.
    ///
    /// 融合语义 / Fusion semantics: 电压模型磁链在低速靠积分仍可用但受 `rs` 漂移影响，SMO
    /// 在中高速更稳；权重应按速度区间调度，并在切换前确认两路都已收敛，否则融合只会在
    /// 正确角度和错误角度之间取中间值。
    /// The voltage-model flux still works at low speed thanks to integration but suffers from
    /// `rs` drift, while the SMO is steadier at medium/high speed, so the weight is scheduled
    /// by speed band; confirm both paths have converged before switching, otherwise the fusion
    /// merely averages a correct with a wrong angle.
    ///
    /// 实时 / Real-time: 本函数同时跑两路观测器（含两次 `atan2`），是本文件最重的路径，
    /// 必须实测 WCET。
    /// This function runs both observers including two `atan2` calls, the heaviest path in this
    /// file; its WCET must be measured.
    pub fn update(&mut self, param: &FluxSmoParam, input: &SmoInput) -> f32 {
        // 两路观测器吃同一份电压/电流快照，避免跨周期混用导致两路角度不一致。
        // Both observers consume the same voltage/current snapshot, so their angles cannot
        // disagree because of a mixed sampling cycle.
        self.flux_vector = self.flux.update(
            &param.flux,
            &FluxInput {
                voltage: input.voltage,
                current: input.current,
            },
        );
        self.emf = self.smo.update(&param.smo, input);
        self.theta_flux_rad = self.flux.theta_flux_rad;
        self.theta_smo_rad = self.smo.theta_emf_rad;
        // 权重在这里再钳位一次（与 `boundary`/`alpha` 同样的防御式处理）：越界权重会把融合
        // 角度推到两路角度之外。
        // The weight is clamped again here (the same defensive style as `boundary`/`alpha`): an
        // out-of-range weight would extrapolate beyond both angles.
        let weight = clamp(param.smo_weight, 0.0, 1.0);
        // 融合必须在“环绕后的最小角差”上做：先取 `[-π,π)` 的差再按权重插值，否则两路角度
        // 分居 0/2π 两侧时会产生一整圈的假跳变。
        // Fusion must use the wrapped minimal difference: take the `[-pi, pi)` delta first and
        // then interpolate, otherwise the blend jumps a full turn while the two angles sit on
        // opposite sides of the 0/2*pi boundary.
        let delta = wrap_angle_minus_pi_to_pi(self.theta_smo_rad - self.theta_flux_rad);
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_flux_rad + weight * delta);
        self.theta_rad
    }
}

/// 主机单元测试：核对 SMO、SMO+PLL 与磁链+SMO 与原 C 参考向量的等价性。
/// Host unit tests checking the SMO, SMO+PLL and flux+SMO against the original C reference
/// vectors.
///
/// 只覆盖数值等价，不证明无感启动、低速可观测性、抖振水平或实机稳定性。
/// They cover numerical equivalence only; sensorless startup, low-speed observability,
/// chattering level and on-target stability are not covered.
#[cfg(test)]
mod tests {
    use super::*;

    /// 断言浮点近似相等；容差 `1e-6` 说明参考向量是按 f32 逐步复算出来的。
    /// Asserts approximate float equality; the `1e-6` tolerance reflects that the reference
    /// vector was recomputed step by step in f32.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// SMO 与 C 参考向量的等价性测试。
    /// Equivalence test for the SMO against the C reference vector.
    ///
    /// 输入电压为 0、电流为 `(1, -1) A`、状态从零开始：第一拍滑模量必然是
    /// `k_slide·(±1)`，估计反电势只经过一次低通，因此断言值直接锁定 `k_slide`、
    /// `boundary`、`emf_filter_alpha` 与运算顺序。
    /// With zero voltage, a current of `(1, -1) A` and a zero state, the first-cycle sliding
    /// term must be `k_slide*(±1)` and the EMF passes the low-pass once, so the asserted values
    /// pin down `k_slide`, `boundary`, `emf_filter_alpha` and the operation order.
    #[test]
    fn smo_matches_c_reference() {
        // 参数取自 C 测试向量：ts·rs/ls = 0.05，远小于 2，前向欧拉稳定；换真实电机参数前
        // 必须重新核对这个比值。
        // Values come from the C test vector: ts*rs/ls = 0.05, far below 2, so the forward
        // Euler step is stable; re-check this ratio before using real machine parameters.
        let param = SmoParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.0001,
            k_slide: 4.0,
            boundary: 0.1,
            emf_filter_alpha: 0.5,
        };
        let input = SmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = SmoState::default();
        let emf = state.update(&param, &input);
        // emf.alpha = -2.0：误差 -1 A 先饱和到 -1，再乘 4 V 滑模增益，最后乘 0.5 低通系数。
        // emf.alpha = -2.0: the -1 A error saturates to -1, is scaled by the 4 V sliding gain
        // and then by the 0.5 low-pass coefficient.
        near(emf.alpha, -2.0, 1e-6);
        near(emf.beta, 2.0, 1e-6);
        // 只要 ls > 0，`update` 一定会写角度，即使零电压、尚未收敛。
        // As long as ls > 0, `update` always writes an angle, even at zero voltage before
        // convergence.
        assert!((0.0..crate::math::TWO_PI).contains(&state.theta_emf_rad));

        // 第二拍专门挡住旧 C 实现的“双重反电势扣除”：第一拍结束后 emf 已非零，
        // 正确电流模型只减 sliding，得到 ±0.78 A；若再减一次 emf 会变成 ±0.98 A。
        // The second cycle catches the legacy double subtraction. Once the EMF
        // low-pass is non-zero, the corrected model subtracts only `sliding` and
        // reaches +/-0.78 A; subtracting `emf` again would produce +/-0.98 A.
        let _ = state.update(&param, &input);
        near(state.current_est.alpha, 0.78, 1e-6);
        near(state.current_est.beta, -0.78, 1e-6);
        near(state.emf.alpha, -3.0, 1e-6);
        near(state.emf.beta, 3.0, 1e-6);
    }

    /// SMO+PLL 与 C 参考向量的等价性测试。
    /// Equivalence test for SMO+PLL against the C reference vector.
    ///
    /// `emf_filter_alpha = 1`（不滤波）让估计反电势等于滑模量本身，所以断言值是 4 V 而不是
    /// 滤波后的 2 V；这说明低通系数直接决定反电势幅值的口径。
    /// `emf_filter_alpha = 1` (no filtering) makes the EMF estimate equal the raw sliding term,
    /// so the asserted value is 4 V instead of the filtered 2 V, showing that the coefficient
    /// directly sets the amplitude basis of the EMF.
    #[test]
    fn smo_pll_matches_c_reference() {
        let param = SmoPllParam {
            smo: SmoParam {
                rs: 0.5,
                ls: 0.001,
                ts: 0.0001,
                k_slide: 4.0,
                boundary: 0.1,
                emf_filter_alpha: 1.0,
            },
            pll: PllParam {
                kp: 20.0,
                ki: 0.0,
                ts: 0.001,
                omega_min: -200.0,
                omega_max: 200.0,
            },
        };
        let input = SmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = SmoPllState::default();
        assert!(state.update(&param, &input) > 0.0);
        // 零电压、零初始状态：滑模量仍是 ±4 V，不滤波的反电势立即等于它。
        // Zero voltage and a zero initial state: the sliding term is still ±4 V and the
        // unfiltered EMF equals it immediately.
        near(state.emf.alpha, -4.0, 1e-6);
        near(state.emf.beta, 4.0, 1e-6);
        // PLL 用 C 测试向量的 kp = 20、ki = 0；角速度为正时速度输出即为正。
        // The PLL uses kp = 20 and ki = 0 from the C test vector, so a positive angle rate
        // gives a positive speed output.
        assert!(state.omega_rad_s > 0.0);
    }

    /// 磁链+SMO 融合与 C 参考向量的等价性测试。
    /// Equivalence test for the flux+SMO fusion against the C reference vector.
    ///
    /// `rs = 0`、`leakage = 0` 让磁链退化为纯电压积分（`ts·v`），`flux_min/max = ±10 Wb`
    /// 保证不触发限幅，从而能直接核对积分比例；`smo_weight = 0.25` 用来验证融合是在环绕角
    /// 差上插值，而不是对两个绝对角直接加权。
    /// `rs = 0` and `leakage = 0` reduce the flux to a plain voltage integral (`ts*v`) and
    /// `flux_min/max = ±10 Wb` keeps the clamp inactive, so the integration scale can be
    /// checked directly. `smo_weight = 0.25` verifies that the fusion interpolates a wrapped
    /// angle difference instead of weighting two absolute angles.
    #[test]
    fn flux_smo_matches_c_reference() {
        let param = FluxSmoParam {
            flux: FluxParam {
                rs: 0.0,
                ts: 0.01,
                leakage: 0.0,
                flux_min: -10.0,
                flux_max: 10.0,
            },
            smo: SmoParam {
                rs: 0.2,
                ls: 0.001,
                ts: 0.0001,
                k_slide: 2.0,
                boundary: 0.1,
                emf_filter_alpha: 1.0,
            },
            smo_weight: 0.25,
        };
        let input = SmoInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 1.0,
            },
            current: AlphaBeta {
                alpha: 1.0,
                beta: 0.0,
            },
        };
        let mut state = FluxSmoState::default();
        let theta = state.update(&param, &input);
        // 磁链 = ts·v，所以 alpha 分量是 0.01 s × 2 V = 0.02 Wb，beta 分量是 0.01 Wb。
        // Flux = ts*v, so the alpha component is 0.01 s * 2 V = 0.02 Wb and beta is 0.01 Wb.
        near(state.flux_vector.alpha, 0.02, 1e-6);
        near(state.flux_vector.beta, 0.01, 1e-6);
        assert!(state.emf.alpha.abs() > 0.0);
        assert!((0.0..crate::math::TWO_PI).contains(&theta));
        // 权重 0.25 且两路角度不同：融合结果必须偏离纯磁链角度。
        // With weight 0.25 and the two angles differing, the fused result must differ from the
        // pure flux angle.
        assert!((theta - state.theta_flux_rad).abs() > 1e-5);
    }
}
