//! 自适应、高阶和超扭曲滑模观测器。
//! Adaptive, higher-order and super-twisting sliding-mode observers.
//!
//! 职责 / Responsibility:
//!   - `AdaptiveSmoState`：滑模增益按电流估计误差在线自适应（误差大 -> 增益大）
//!   - `HigherOrderSmoState`：滑模面加入误差导数反馈（PD 型），改善收敛瞬态
//!   - `SuperTwistingSmoState`：超螺旋二阶滑模，用
//!     `k1*|e|^0.5*sign(e) + ∫k2*sign(e)dt` 替代单符号项，降低 `sign()` 抖振
//!   - adaptive gain, error-derivative feedback, super-twisting chattering reduction
//!
//! 共用骨架 / Shared skeleton: `Ls*dî/dt = v - Rs*î - ê - u`，其中 `ê` 是滑模项 `u`
//! 的一阶低通（等效控制法），电角度由 `atan2(-ê_alpha, ê_beta)` 给出（反电势矢量
//! 超前转子 d 轴 90° 电角度，与 `bemf.rs` 的约定一致）。
//! All three share one current observer: the EMF is the low-pass filtered sliding term.
//!
//! 架构位置 / Architecture position:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!     -> foc-control -> foc-algorithm (本文件 / this file，纯 `no_std` 数学层)
//! 依赖方向 / Dependency direction:
//!   只依赖 crate 内的 `math` 和 `transform`；状态是定长标量，不认识 HAL、RTOS、堆，
//!   不做单位换算、不分配内存、不加锁、不打印日志。
//!   Depends only on `math` and `transform`; fixed-size caller-owned state, no HAL.
//!
//! 实时约束 / Real-time constraints:
//!   本文件三个类型在当前仓库内没有实时调用方（`foc-control` 的默认后端是 `smo.rs`
//!   的基础 `SmoState` + PLL），属于由 C 参考库移植来的可选观测器。接入 12 kHz 快环前
//!   必须先用 DWT 实测 WCET，尤其是高阶型多出的除法/微分和超螺旋型的 `libm::sqrtf`。
//!   状态对象尺寸见 `算法库实时性说明.md`（40 / 44 / 36 字节）。
//!   No real-time caller in this checkout yet; measure the WCET before ISR use.
//!
//! 离散化与稳定性 / Discretisation and stability:
//!   三者都是固定步长显式欧拉 `î += ts/Ls*(...)`。显式欧拉的绝对稳定域要求
//!   `ts*Rs/Ls < 2`：本项目 ST 基准 `Rs = 5.29 ohm`、`Ld = 1.058 mH`（`[ST]`）给出
//!   `L/R ≈ 200 us`，12 kHz（`ts ≈ 83.3 us`，`[FW]`）时 `ts*Rs/Ls ≈ 0.42`——仍在
//!   稳定域内，但一步只剩约 0.58 倍衰减，离散误差不可忽略；提高 PWM 频率或换更小
//!   电感的电机都必须重算。此外滑模项各自还有增益条件，见各结构体说明。
//!   Fixed-step explicit Euler with `ts*Rs/Ls < 2`; ~0.42 for the ST baseline at
//!   12 kHz, so stability holds but discretisation error is not negligible.
//!
//! 抖振与滤波的权衡 / Chattering versus filter lag:
//!   滑模项在穿越滑模面时本质上是切换的，抖振主要靠两处压制：切换函数用
//!   `saturation` 的边界层代替 `sign()`，以及反电势低通。滤波越重（`emf_filter_alpha`
//!   越小）角度越平滑，但相位滞后 `τ = -ts/ln(1-α)` 直接进入角度，本文件不做相位
//!   补偿，所以"降抖振"和"减滞后"只能按转速取舍。
//!   Chattering is traded against phase lag: heavier EMF filtering smooths the angle but
//!   adds uncompensated lag `τ = -ts/ln(1-α)`.
//!
//! 量纲 / Units: 电压 `[V]`、电流 `[A]`、电流变化率 `[A/s]`、电阻 `[ohm]`、电感 `[H]`、
//!   时间 `[s]`、角度 `[rad]`、电角速度 `[rad/s]`。
//! 定点说明 / Fixed point:
//!   本文件全部量都是 `f32`，不存在 Q1.15/Q1.31 定点站点。
//!   Every quantity here is `f32`; this file has no Q1.15/Q1.31 site.
//!
//! 参考 / Reference: `算法库移植状态.md`（Observer_AdaptiveSMO / HigherOrderSMO /
//!   SuperTwistingSMO 行）、`算法库实时性说明.md`、`算法库总览与对接指南.md` §4.5、
//!   `docs/FOC算法组合与应用场景.md` §3.7

use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::AlphaBeta;

/// 带边界层的饱和函数：把 `value/boundary` 夹到 `[-1, 1]`。
/// Saturation with a boundary layer: clamps `value/boundary` into `[-1, 1]`.
///
/// `boundary <= 0.0` 时退化为符号函数（正 1 / 负 -1 / 零 0）。
/// With `boundary <= 0.0` it degenerates to the sign function.
///
/// 用饱和代替 `sign()` 是为了抑制抖振：`|value| < boundary` 时切换项变成线性，
/// 输出连续；代价是这一段没有理想滑模的有限时间收敛保证，边界越大越平滑也越"软"。
/// Replacing `sign()` with saturation trades finite-time exactness for smoothness.
///
/// 注意 / Note: 边界的量纲必须与输入一致（电流 `[A]` 或电流变化率 `[A/s]`），写错
/// 量级等于悄悄关掉或过度加宽边界层；另外内部的 `clamp` 不拦 `NaN`。
/// The boundary shares the input's unit, and `clamp` does not stop `NaN`.
fn saturation(value: f32, boundary: f32) -> f32 {
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

/// 三值符号函数：正数返回 1、负数返回 -1、恰好 0 返回 0。
/// Three-valued sign: 1 for positive, -1 for negative, 0 at exactly zero.
///
/// 超螺旋项用它而不是 `signum`，是为了让误差恰好为零时积分项停止累加。`NaN` 的两个
/// 比较都为假，因此返回 0，即 NaN 误差不会推动积分项（但不代表安全，NaN 仍会通过
/// 电流观测器继续扩散）。
/// Used so the super-twisting integral stops at exactly zero error; `NaN` returns 0.
fn sign_zero(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// 自适应滑模观测器参数。
/// Adaptive sliding-mode observer parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm。
    /// Per-phase stator resistance in `[ohm]`.
    pub rs: f32,
    /// 定子电感 `[H]`；α/β 两轴共用一个值（不区分 `Ld`/`Lq`），`ls <= 0.0` 时
    /// `update` 返回零向量。
    /// Stator inductance in `[H]` for both axes; `update` no-ops when `ls <= 0.0`.
    pub ls: f32,
    /// 调用周期 `[s]`。注意本类型只校验 `ls`，不校验 `ts`（见 `update` 的说明）。
    /// Sample period in `[s]`; note that only `ls` is validated for this observer.
    pub ts: f32,
    /// 滑模增益初值 `[V]`；`reset` 会用 `clamp(k_initial, k_min, k_max)` 装入。
    /// Initial sliding gain in `[V]`, clamped into `[k_min, k_max]` by `reset`.
    pub k_initial: f32,
    /// 滑模增益下限 `[V]`。
    /// Lower sliding-gain bound in `[V]`.
    ///
    /// 增益必须大于反电势幅值上限（滑模可达条件），低于该量级时滑模面不可达；低增益
    /// 抖振小但收敛慢。
    /// The gain must exceed the EMF magnitude to reach the sliding surface.
    pub k_min: f32,
    /// 滑模增益上限 `[V]`；这是自适应律唯一的限幅，防止大误差把增益越推越高。
    /// Upper sliding-gain bound in `[V]`; the only clamp on the adaptation.
    pub k_max: f32,
    /// 自适应速率，量纲 `[V/(A*s)]`（等效"电阻/秒"）。
    /// Adaptation rate in `[V/(A*s)]`.
    ///
    /// 增益按 `k += adapt_rate*(|err| - target_error)*ts` 变化：误差大于目标就加增益，
    /// 小于目标就减。速率太大时只要一拍到几拍就能把 `k` 顶到 `k_max`（本文件测试就是
    /// 用 `adapt_rate = 100000` 验证这一点），等于退化成固定增益上限运行。
    /// The gain is driven toward `target_error`; a large rate rails `k` at `k_max`.
    pub adapt_rate: f32,
    /// 电流估计误差的目标幅值 `[A]`，自适应律的工作点。
    /// Target current-error magnitude in `[A]`, the adaptation operating point.
    ///
    /// 取 0 会把增益一直往上推（直到 `k_max`），因为噪声和离散误差决定误差不可能真的
    /// 为零；工程上应取的略大于稳态误差的噪声底。
    /// Zero drives the gain to `k_max` because the error can never reach zero.
    pub target_error: f32,
    /// 电流误差的饱和边界 `[A]`，用法见 `saturation`。
    /// Current-error saturation boundary in `[A]`; see `saturation`.
    pub boundary: f32,
    /// 反电势低通系数，无量纲 `[0, 1]`：`1.0` 不滤波（角度跟随滑模抖振），越小越平滑
    /// 但相位滞后 `τ = -ts/ln(1-α)` 越大。
    /// EMF low-pass coefficient in `[0, 1]`; smaller values trade chattering for lag.
    pub emf_filter_alpha: f32,
}

/// 自适应滑模观测器状态。
/// Adaptive sliding-mode observer state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoState {
    /// αβ 估计电流 `[A]`；`reset` 把它清零而不是用实测电流初始化。
    /// Estimated αβ current in `[A]`; `reset` zeroes it instead of seeding it.
    pub current_est: AlphaBeta,
    /// 本拍滑模（切换）项 `[V]`，也就是未滤波的反电势等效量。
    /// This sample's sliding (switching) term in `[V]`, the raw EMF equivalent.
    pub sliding: AlphaBeta,
    /// 滤波后的 αβ 反电势 `[V]`，也是 `update` 的返回值。
    /// Filtered αβ EMF in `[V]`; also the value `update` returns.
    pub emf: AlphaBeta,
    /// 当前滑模增益 `[V]`，由自适应律在 `[k_min, k_max]` 内修改。
    /// Current sliding gain in `[V]`, adapted inside `[k_min, k_max]`.
    pub k_slide: f32,
    /// 本拍电流估计误差幅值 `[A]`（`sqrt(e_alpha^2 + e_beta^2)`），自适应律的输入。
    /// Current-error magnitude in `[A]`, the adaptation input.
    pub error_abs: f32,
    /// 由滤波反电势推出的电角度 `[rad]`，范围 `[0, 2π)`。
    /// Electrical angle in `[rad]` from the filtered EMF, in `[0, 2π)`.
    pub theta_emf_rad: f32,
    /// 初始化标志：0 时 `update` 会先调用 `reset`（本观测器的 `reset` 需要参数）。
    /// Init flag: `update` calls `reset` when it is 0.
    ///
    /// 保留 `i32` 与 C 参考里的 `int` 宽度和含义一致。
    /// Kept as `i32` to match the C `int` in width and meaning.
    pub initialized: i32,
}

/// 自适应滑模观测器一拍输入（αβ 静止坐标系）。
/// One-sample input of the adaptive SMO in the stationary αβ frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoInput {
    /// αβ 相电压 `[V]`；实时路径上由 PWM 占空比和母线电压重构，含死区误差。
    /// αβ phase voltage in `[V]`, reconstructed from PWM duty and the DC bus.
    pub voltage: AlphaBeta,
    /// 实测 αβ 电流 `[A]`，作为滑模面的跟踪目标。
    /// Measured αβ current in `[A]`, the sliding-surface tracking target.
    pub current: AlphaBeta,
}

/// `AdaptiveSmoState` 的构造、复位与逐拍更新入口。
/// Construction, reset and per-sample update entry points of `AdaptiveSmoState`.
impl AdaptiveSmoState {
    /// 新建一个只装入滑模增益和初始化标志的默认状态。
    /// Creates a default state with only the gain and the init flag seeded.
    pub fn new(param: &AdaptiveSmoParam) -> Self {
        let mut state = Self::default();
        state.reset(param);
        state
    }

    /// 把估计电流、反电势和滑模项清零，并把增益复位到 `k_initial`。
    /// Clears the estimates and re-seeds the gain from `k_initial`.
    ///
    /// 增益被夹在 `[k_min, k_max]`：配置越界不会报错，而是被静默拉到边界。
    /// 清零 `current_est` 意味着复位后的前几拍误差接近满量程，自适应律会把增益快速
    /// 推向 `k_max`；需要温和启动时应先把 `current_est` 设为实测电流。
    /// The gain is clamped silently, and a zeroed `current_est` pushes it hard toward
    /// `k_max` during the first samples.
    pub fn reset(&mut self, param: &AdaptiveSmoParam) {
        *self = Self {
            k_slide: clamp(param.k_initial, param.k_min, param.k_max),
            initialized: 1,
            ..Self::default()
        };
    }

    /// 一拍自适应 SMO：先按误差调增益，再更新电流观测器和反电势低通。
    /// One adaptive-SMO step: adapt the gain, then update the current observer and EMF LPF.
    ///
    /// 参数 / Parameters: param - 见 `AdaptiveSmoParam`；input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: 滤波后的 αβ 反电势 `[V]`；`param.ls <= 0.0` 时返回零向量且
    ///   不改状态。
    ///
    /// 稳定性 / Stability: 滑模可达条件要求切换项幅值大于反电势幅值（本模型里就是
    /// `k_slide > |e|`），自适应律把 `|err|` 驱动到 `target_error`，本质上是在线搜索
    /// 这个下界。该律是积分型正反馈（误差 -> 增益 -> 误差），本文件没有稳定性证明也
    /// 没有参数校验，越界只靠 `k_min`/`k_max` 兜住。
    /// Reachability needs the switching term to dominate the EMF; the adaptation searches
    /// for that bound online and is protected only by `k_min`/`k_max`.
    ///
    /// 陷阱 / Pitfalls:
    ///   - 只校验 `ls` 不校验 `ts`：`ts <= 0.0` 时电流状态不前进（或倒退），而增益项
    ///     会随 `ts` 的符号反向调整，属于未定义配置（`HigherOrderSmoState` 两者都校验）。
    ///   - `emf_filter_alpha` 越大角度越快但抖振越大，本文件不做相位补偿。
    ///   - `saturation` 内部走 `clamp`，`NaN` 会穿透并污染 `sliding`/`emf`。
    ///   - Only `ls` is validated here, and `NaN` is not filtered out.
    pub fn update(&mut self, param: &AdaptiveSmoParam, input: &AdaptiveSmoInput) -> AlphaBeta {
        if param.ls <= 0.0 {
            return AlphaBeta::default();
        }
        if self.initialized == 0 {
            self.reset(param);
        }
        let inv_l = 1.0 / param.ls;
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        let err_alpha = self.current_est.alpha - input.current.alpha;
        let err_beta = self.current_est.beta - input.current.beta;
        // 误差取幅值（标量）：自适应律只关心总体跟踪质量，不区分 α/β 分量。
        // The magnitude is used because the adaptation only cares about tracking quality.
        self.error_abs = libm::sqrtf(err_alpha * err_alpha + err_beta * err_beta);
        // 积分型自适应律：误差大于目标就加增益。`k_slide` 的单位是 [V]，所以
        // adapt_rate 的量纲是 [V/(A*s)]。
        // Integral adaptation; `k_slide` is in [V], hence `adapt_rate` is [V/(A*s)].
        self.k_slide += param.adapt_rate * (self.error_abs - param.target_error) * param.ts;
        self.k_slide = clamp(self.k_slide, param.k_min, param.k_max);
        // 切换项量纲是 [V]：增益 [V] 乘以无量纲的饱和函数。
        // The switching term is in [V]: a [V] gain times a dimensionless saturation.
        self.sliding.alpha = self.k_slide * saturation(err_alpha, param.boundary);
        self.sliding.beta = self.k_slide * saturation(err_beta, param.boundary);
        // 显式欧拉：î += ts/L*(v - R*î - ê - u)。-ê 是 -u 的滤波版本，两者共同构成
        // 校正，不能各自按满幅理解；ts/L 的稳定域见文件头。
        // Explicit Euler; `ê` is the filtered version of the switching term `u`.
        self.current_est.alpha += param.ts
            * inv_l
            * (input.voltage.alpha
                - param.rs * self.current_est.alpha
                - self.emf.alpha
                - self.sliding.alpha);
        self.current_est.beta += param.ts
            * inv_l
            * (input.voltage.beta
                - param.rs * self.current_est.beta
                - self.emf.beta
                - self.sliding.beta);
        // 等效控制法：把切换项低通后当作反电势估计。α = 1 时 ê = u，没有滞后但把
        // 抖振全部带进角度；α -> 0 则平滑但滞后大（见文件头"抖振与滤波的权衡"）。
        // Equivalent-control method: the filtered switching term is the EMF estimate.
        self.emf.alpha += alpha * (self.sliding.alpha - self.emf.alpha);
        self.emf.beta += alpha * (self.sliding.beta - self.emf.beta);
        // 与 `bemf.rs` 同一约定：反电势矢量旋转 -90° 得到 d 轴电角度。
        // Same convention as `bemf.rs`: rotate the EMF vector by -90°.
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        self.emf
    }
}

/// 高阶滑模观测器参数（滑模面含误差导数项）。
/// Higher-order SMO parameters (the sliding surface includes the error derivative).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm。
    /// Per-phase stator resistance in `[ohm]`.
    pub rs: f32,
    /// 定子电感 `[H]`；α/β 两轴共用一个值；`ls <= 0.0` 或 `ts <= 0.0` 时 `update`
    /// 返回零向量（本类型两个都校验）。
    /// Stator inductance in `[H]`; both `ls` and `ts` are validated here.
    pub ls: f32,
    /// 调用周期 `[s]`；误差导数用 `Δerr/ts` 计算，写错会按比例改变该增益的等效值。
    /// Sample period in `[s]`; the derivative term uses `Δerr/ts`.
    pub ts: f32,
    /// 误差比例项增益 `[V]`（乘以无量纲饱和函数）。
    /// Proportional (error) gain in `[V]`.
    pub k_slide: f32,
    /// 误差导数项增益，量纲 `[V*s]`：`k_dot * sat(err_dot)` 的结果必须是 `[V]`。
    /// Derivative gain with the dimension `[V*s]`, so `k_dot*sat(err_dot)` is `[V]`.
    ///
    /// 这一项给滑模面加入超前（PD 型）作用，能缩短到达滑模面的时间；代价是误差导数
    /// 由后向差分得到，噪声增益为 `1/ts`（12 kHz 时 12000），所以 `dot_boundary` 必须
    /// 足够大才能把噪声限制在边界层内。
    /// The derivative term adds lead action but amplifies noise by `1/ts`.
    pub k_dot: f32,
    /// 电流误差的饱和边界 `[A]`，用法见 `saturation`。
    /// Current-error saturation boundary in `[A]`; see `saturation`.
    pub boundary: f32,
    /// 误差导数的饱和边界 `[A/s]`；取太小会让电流采样噪声直接进入滑模项。
    /// Derivative saturation boundary in `[A/s]`; too small lets noise through.
    pub dot_boundary: f32,
    /// 反电势低通系数，无量纲 `[0, 1]`；越小越平滑、滞后越大。
    /// EMF low-pass coefficient in `[0, 1]`; smaller means smoother but laggier.
    pub emf_filter_alpha: f32,
}

/// 高阶滑模观测器状态。
/// Higher-order SMO state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoState {
    /// αβ 估计电流 `[A]`。
    /// Estimated αβ current in `[A]`.
    pub current_est: AlphaBeta,
    /// 上一拍的电流估计误差 `[A]`，用于后向差分。
    /// Previous current-error sample in `[A]` for the backward difference.
    pub last_error: AlphaBeta,
    /// 本拍误差导数 `[A/s]`；仅作为滑模面输入和诊断量保存，本文件内没有对它限幅。
    /// This sample's error derivative in `[A/s]`; it is not clamped in this file.
    pub error_dot: AlphaBeta,
    /// 本拍滑模（切换）项 `[V]`，比例项与导数项之和。
    /// This sample's sliding term in `[V]`: the sum of both feedback terms.
    pub sliding: AlphaBeta,
    /// 滤波后的 αβ 反电势 `[V]`，也是 `update` 的返回值。
    /// Filtered αβ EMF in `[V]`; also the value `update` returns.
    pub emf: AlphaBeta,
    /// 由滤波反电势推出的电角度 `[rad]`，范围 `[0, 2π)`。
    /// Electrical angle in `[rad]` from the filtered EMF, in `[0, 2π)`.
    pub theta_emf_rad: f32,
}

/// 高阶滑模观测器一拍输入（αβ 静止坐标系）。
/// One-sample input of the higher-order SMO in the stationary αβ frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoInput {
    /// αβ 相电压 `[V]`；实时路径上由 PWM 占空比和母线电压重构，含死区误差。
    /// αβ phase voltage in `[V]`, reconstructed from PWM duty and the DC bus.
    pub voltage: AlphaBeta,
    /// 实测 αβ 电流 `[A]`。
    /// Measured αβ current in `[A]`.
    pub current: AlphaBeta,
}

/// `HigherOrderSmoState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `HigherOrderSmoState`.
impl HigherOrderSmoState {
    /// 清零估计电流、误差历史、滑模项和反电势。
    /// Clears the current estimate, the error history, the sliding term and the EMF.
    ///
    /// `last_error` 归零意味着复位后第一拍的误差导数等于 `-err/ts`（本文件测试就是
    /// 用 `-10000 A/s` 验证这一点的），是一个量级很大的瞬态值。
    /// A zeroed `last_error` makes the first derivative sample equal to `-err/ts`.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一拍高阶 SMO：误差及其导数共同构成滑模面。
    /// One higher-order SMO step: the error and its derivative form the sliding surface.
    ///
    /// 参数 / Parameters: param - 见 `HigherOrderSmoParam`；
    ///   input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: 滤波后的 αβ 反电势 `[V]`；`ls <= 0.0` 或 `ts <= 0.0` 时返回
    ///   零向量且不改状态。
    ///
    /// 与 `AdaptiveSmoState::update` 的差别 / Difference: 两个增益都是固定常数，没有
    /// 自适应律；校验同时覆盖 `ls` 和 `ts`。
    /// Both gains are fixed constants here and both `ls` and `ts` are validated.
    ///
    /// 陷阱 / Pitfalls: 微分项对电流采样噪声和量化误差最敏感，后向差分在阶跃/换相
    /// 瞬间会产出 `Δi/ts` 量级的尖峰（本文件测试第一拍即为 `-10000 A/s`），只有
    /// `dot_boundary` 能限制它进入滑模项；`error_dot` 本身没有限幅。
    /// Only `dot_boundary` limits the derivative spike; `error_dot` itself is unclamped.
    pub fn update(
        &mut self,
        param: &HigherOrderSmoParam,
        input: &HigherOrderSmoInput,
    ) -> AlphaBeta {
        if param.ls <= 0.0 || param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        let inv_l = 1.0 / param.ls;
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        let error = AlphaBeta {
            alpha: self.current_est.alpha - input.current.alpha,
            beta: self.current_est.beta - input.current.beta,
        };
        // 后向差分求误差导数：噪声增益 1/ts，见 `dot_boundary` 的说明。
        // Backward-difference derivative: noise gain 1/ts.
        self.error_dot.alpha = (error.alpha - self.last_error.alpha) / param.ts;
        self.error_dot.beta = (error.beta - self.last_error.beta) / param.ts;
        // 滑模面 = 比例项 + 导数项，两项都先经各自的边界层饱和，量纲都是 [V]。
        // Sliding surface = proportional + derivative terms, both saturated to [V].
        self.sliding.alpha = param.k_slide * saturation(error.alpha, param.boundary)
            + param.k_dot * saturation(self.error_dot.alpha, param.dot_boundary);
        self.sliding.beta = param.k_slide * saturation(error.beta, param.boundary)
            + param.k_dot * saturation(self.error_dot.beta, param.dot_boundary);
        self.current_est.alpha += param.ts
            * inv_l
            * (input.voltage.alpha
                - param.rs * self.current_est.alpha
                - self.emf.alpha
                - self.sliding.alpha);
        self.current_est.beta += param.ts
            * inv_l
            * (input.voltage.beta
                - param.rs * self.current_est.beta
                - self.emf.beta
                - self.sliding.beta);
        self.emf.alpha += alpha * (self.sliding.alpha - self.emf.alpha);
        self.emf.beta += alpha * (self.sliding.beta - self.emf.beta);
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        // 保存本拍误差，供下一拍的差分使用（放在最后，确保用的是"上一拍"值）。
        // Stored last so the next sample differentiates against the previous error.
        self.last_error = error;
        self.emf
    }
}

/// 超螺旋滑模观测器参数。
/// Super-twisting SMO parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm。
    /// Per-phase stator resistance in `[ohm]`.
    pub rs: f32,
    /// 定子电感 `[H]`；α/β 两轴共用一个值；`ls <= 0.0` 时 `update` 返回零向量
    /// （本类型只校验 `ls`，不校验 `ts`）。
    /// Stator inductance in `[H]`; only `ls` is validated for this observer.
    pub ls: f32,
    /// 调用周期 `[s]`；积分项按 `k2*sign(e)*ts` 累加，写错会直接改变积分增益。
    /// Sample period in `[s]`; the integral term accumulates `k2*sign(e)*ts`.
    pub ts: f32,
    /// 代数项增益，量纲 `[V/√A]`：`k1*|e|^0.5*sign(e)` 的结果是 `[V]`。
    /// Algebraic-term gain in `[V/√A]` so `k1*sqrt(|e|)*sign(e)` is `[V]`.
    ///
    /// `|e|^0.5` 是非线性增益：`|e| < 1` 时相对增益比线性项更大（收敛快），`|e|` 大时
    /// 相对增益更小（幅值不爆炸），这正是超螺旋能兼顾收敛速度与抖振的原因。
    /// The square-root law is why super-twisting converges fast without exploding.
    pub k1: f32,
    /// 积分项增益，量纲 `[V/s]`：`∫k2*sign(e)dt` 累加成 `[V]`。
    /// Integral-term gain in `[V/s]`; `∫k2*sign(e)dt` accumulates into `[V]`.
    ///
    /// 经典超螺旋的充分条件（公开控制理论结论）大致是 `k1 > 0` 且 `k2` 大于反电势
    /// 导数的界；本文件两个增益都不校验也不夹紧，配错会让观测器发散。另一个实际陷阱
    /// 是积分项没有抗饱和：起转或复位后误差长时间同号会把 `integral` 积到远大于反电势
    /// 的值，之后只能按 `k2*ts` 每拍慢慢退回。
    /// Public control theory bounds `k2` by the EMF derivative; nothing is clamped here,
    /// and the integral term has no anti-windup.
    pub k2: f32,
    /// 反电势低通系数，无量纲 `[0, 1]`；超螺旋本身抖振较小，可以取比单符号项更大的
    /// 值来换取更小的相位滞后。
    /// EMF low-pass coefficient in `[0, 1]`; super-twisting allows a larger value (less
    /// lag) than a plain `sign()` observer for the same angle quality.
    pub emf_filter_alpha: f32,
}

/// 超螺旋滑模观测器状态。
/// Super-twisting SMO state.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoState {
    /// αβ 估计电流 `[A]`。
    /// Estimated αβ current in `[A]`.
    pub current_est: AlphaBeta,
    /// 超螺旋积分项 `[V]`；本文件不对它夹紧，也没有抗饱和回退。
    /// The super-twisting integral term in `[V]`; it is not clamped anywhere.
    pub integral: AlphaBeta,
    /// 本拍滑模（切换）控制量 `[V]`，代数项与积分项之和。
    /// This sample's sliding control in `[V]`: the algebraic term plus the integral.
    pub sliding: AlphaBeta,
    /// 滤波后的 αβ 反电势 `[V]`，也是 `update` 的返回值。
    /// Filtered αβ EMF in `[V]`; also the value `update` returns.
    pub emf: AlphaBeta,
    /// 由滤波反电势推出的电角度 `[rad]`，范围 `[0, 2π)`。
    /// Electrical angle in `[rad]` from the filtered EMF, in `[0, 2π)`.
    pub theta_emf_rad: f32,
}

/// 超螺旋滑模观测器一拍输入（αβ 静止坐标系）。
/// One-sample input of the super-twisting SMO in the stationary αβ frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoInput {
    /// αβ 相电压 `[V]`；实时路径上由 PWM 占空比和母线电压重构，含死区误差。
    /// αβ phase voltage in `[V]`, reconstructed from PWM duty and the DC bus.
    pub voltage: AlphaBeta,
    /// 实测 αβ 电流 `[A]`。
    /// Measured αβ current in `[A]`.
    pub current: AlphaBeta,
}

/// `SuperTwistingSmoState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `SuperTwistingSmoState`.
impl SuperTwistingSmoState {
    /// 清零估计电流、积分项、滑模项和反电势。
    /// Clears the current estimate, the integral, the sliding term and the EMF.
    ///
    /// 积分项归零后需要重新建立，起转期间误差符号长期不变会把积分项推得很大。
    /// The integral restarts from zero and can wind up while the startup error keeps
    /// one sign.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 一拍超螺旋 SMO：`u = k1*|e|^0.5*sign(e) + ∫k2*sign(e)dt`。
    /// One super-twisting SMO step: `u = k1*sqrt(|e|)*sign(e) + ∫k2*sign(e)dt`.
    ///
    /// 参数 / Parameters: param - 见 `SuperTwistingSmoParam`；
    ///   input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: 滤波后的 αβ 反电势 `[V]`；`param.ls <= 0.0` 时返回零向量且
    ///   不改状态（与自适应型一样只校验 `ls`，不校验 `ts`）。
    ///   Returns the filtered EMF; only `ls` is validated, not `ts`.
    ///
    /// 相对单符号项 SMO 的收益 / Why: 控制量在 `e` 变化时是连续的（只有 `e` 过零那
    /// 一拍才跳变），所以反电势低通可以少滤一点、角度滞后更小；代价是多一个积分状态、
    /// 每轴多一次 `libm::sqrtf`，并且离散化后抖振仍在采样率上存在。
    /// The control is continuous except at `e = 0`, so less EMF filtering is needed; the
    /// cost is one extra state and one `sqrtf` per axis.
    pub fn update(
        &mut self,
        param: &SuperTwistingSmoParam,
        input: &SuperTwistingSmoInput,
    ) -> AlphaBeta {
        if param.ls <= 0.0 {
            return AlphaBeta::default();
        }
        let inv_l = 1.0 / param.ls;
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        let err_alpha = self.current_est.alpha - input.current.alpha;
        let err_beta = self.current_est.beta - input.current.beta;
        let sign_alpha = sign_zero(err_alpha);
        let sign_beta = sign_zero(err_beta);
        // 积分项（"连续"部分）：按 k2*sign(e)*ts 累加成 [V]，没有抗饱和。
        // The integral part accumulates into [V] with no anti-windup.
        self.integral.alpha += param.k2 * sign_alpha * param.ts;
        self.integral.beta += param.k2 * sign_beta * param.ts;
        // 控制量 = 代数项 + 积分项；代数项用 |e| 的平方根做非线性增益（见 `k1`）。
        // Control = algebraic term + integral term, with the square-root nonlinear gain.
        self.sliding.alpha =
            param.k1 * libm::sqrtf(err_alpha.abs()) * sign_alpha + self.integral.alpha;
        self.sliding.beta = param.k1 * libm::sqrtf(err_beta.abs()) * sign_beta + self.integral.beta;
        self.current_est.alpha += param.ts
            * inv_l
            * (input.voltage.alpha
                - param.rs * self.current_est.alpha
                - self.emf.alpha
                - self.sliding.alpha);
        self.current_est.beta += param.ts
            * inv_l
            * (input.voltage.beta
                - param.rs * self.current_est.beta
                - self.emf.beta
                - self.sliding.beta);
        self.emf.alpha += alpha * (self.sliding.alpha - self.emf.alpha);
        self.emf.beta += alpha * (self.sliding.beta - self.emf.beta);
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        self.emf
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::TWO_PI;

    /// 浮点近似比较助手：容差由调用方给出，用来吸收最后一位的舍入差异。
    /// Approximate-float comparison helper; the tolerance absorbs last-bit rounding.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 对照 C 参考向量的自适应 SMO 测试：验证增益按误差幅值调整并被 `k_max` 夹住。
    /// C reference vectors for the adaptive SMO gain update and its `k_max` clamp.
    ///
    /// 期望值 2.009142 来自 `100*(sqrt(2) - 0.5)*1e-4`；把 `adapt_rate` 提到 100000
    /// 后第二段断言验证增益确实被顶到 `k_max = 5.0`。
    /// The two halves pin the adaptation law and the `k_max` rail respectively.
    #[test]
    fn adaptive_smo_matches_c_reference() {
        let mut param = AdaptiveSmoParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.0001,
            k_initial: 2.0,
            k_min: 1.0,
            k_max: 5.0,
            adapt_rate: 100.0,
            target_error: 0.5,
            boundary: 0.1,
            emf_filter_alpha: 1.0,
        };
        let input = AdaptiveSmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = AdaptiveSmoState::new(&param);
        let emf = state.update(&param, &input);
        near(state.k_slide, 2.009_142, 1e-5);
        near(emf.alpha, -2.009_142, 1e-5);
        near(emf.beta, 2.009_142, 1e-5);
        assert!((0.0..TWO_PI).contains(&state.theta_emf_rad));
        param.adapt_rate = 100_000.0;
        state.update(&param, &input);
        near(state.k_slide, 5.0, 1e-6);
    }

    /// 对照 C 参考向量的高阶 SMO 测试。
    /// C reference vectors for the higher-order SMO.
    ///
    /// 首拍 `last_error` 为零，所以误差导数等于 `-err/ts`（断言 `-10000 A/s`）；
    /// `-5.0 V` 是比例项 `4*(-1)` 与导数项 `1*(-1)` 都进入饱和后的和，用来锁住两项
    /// 各自的边界层与运算顺序。
    /// The first sample derives `-err/ts`, and `-5 V` pins both saturated terms.
    #[test]
    fn higher_order_smo_matches_c_reference() {
        let param = HigherOrderSmoParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.0001,
            k_slide: 4.0,
            k_dot: 1.0,
            boundary: 0.1,
            dot_boundary: 10.0,
            emf_filter_alpha: 1.0,
        };
        let input = HigherOrderSmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = HigherOrderSmoState::default();
        let emf = state.update(&param, &input);
        near(emf.alpha, -5.0, 1e-6);
        near(emf.beta, 5.0, 1e-6);
        near(state.error_dot.alpha, -10_000.0, 1e-3);
    }

    /// 对照 C 参考向量的超螺旋测试，锁住该律的离散化顺序。
    /// C reference vectors that pin the discretisation order of the super-twisting law.
    ///
    /// 首拍积分项为 `k2*(-1)*ts = -0.001 V`，与代数项 `-2.0 V` 相加得到 `-2.001 V`；
    /// 若把积分项写在代数项之后累加，或先更新积分再算代数项，期望值就会变。
    /// The first integral step plus the algebraic term gives `-2.001 V`; reordering the
    /// two updates changes that value.
    #[test]
    fn super_twisting_smo_matches_c_reference() {
        let param = SuperTwistingSmoParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.0001,
            k1: 2.0,
            k2: 10.0,
            emf_filter_alpha: 1.0,
        };
        let input = SuperTwistingSmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = SuperTwistingSmoState::default();
        let emf = state.update(&param, &input);
        near(emf.alpha, -2.001, 1e-6);
        near(emf.beta, 2.001, 1e-6);
    }
}
