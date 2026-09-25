//! 反电势、反电势积分、过零检测及 BEMF+PLL 组合观测器。
//! Back-EMF, integral BEMF, zero-cross detection and the combined BEMF + PLL observer.
//!
//! 职责 / Responsibility:
//!   - `BemfState`：由 αβ 电压/电流和 `Rs`/`Ls` 反解瞬时反电势 `e = v - Rs*i - Ls*di/dt`
//!   - `BemfIntegralState`：对 `v - Rs*i` 积分得到定子磁链，再由磁链求电角度
//!   - `BemfZeroCrossState`：六步无感 BLDC 的浮空相反电势过零检测（消隐带去抖）
//!   - `BemfPllState`：把反电势角度送入 PLL，输出平滑电角度和电角速度
//!   - instantaneous BEMF estimate, leaky flux integral, zero-cross detection, BEMF+PLL
//!
//! 架构位置 / Architecture position:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!     -> foc-control -> foc-algorithm (本文件 / this file，纯 `no_std` 数学层)
//! 依赖方向 / Dependency direction:
//!   只依赖 crate 内的 `math`、`transform` 和同层的 `PllState`；不认识 ADC/PWM/
//!   寄存器，不做单位换算，不分配内存、不加锁、不打印日志。
//!   Depends only on `math`, `transform` and the sibling `PllState`; no ADC, PWM or
//!   register knowledge, no allocation, no locking, no logging.
//!
//! 实时约束 / Real-time constraints:
//!   `BemfPllState` 由 `foc-control` 的 `BemfPllEstimator`
//!   （`ObserverBackend::FloatBemfPll` 后端）逐拍调用，`ts = 1/12000` `[s]`，每拍一次
//!   `libm::atan2f`，可以放进 12 kHz ADC 中断，但必须计入 ISR 时间预算。
//!   `BemfIntegralState` 和 `BemfZeroCrossState` 在当前仓库内没有实时调用方，
//!   只有移植测试覆盖，接入快环前要自己实测 WCET。
//!   `BemfPllState` is driven per sample by `foc-control`'s `BemfPllEstimator` when the
//!   `FloatBemfPll` backend is selected; the other two types have no real-time caller.
//!
//! 可观性 / Observability:
//!   反电势幅值正比于转速，零速时为零，低速时被电流采样噪声、`Rs`/`Ls` 参数误差和
//!   逆变器死区电压淹没。本文件的观测器只覆盖中高速无感，启动必须另配开环升速
//!   （RevUp）、有感切换或高频注入路径。
//!   EMF amplitude scales with speed and vanishes at standstill, so these observers
//!   only cover medium/high speed; startup needs RevUp, a sensor or HF injection.
//!
//! 量纲 / Units: 电压 `[V]`、电流 `[A]`、电阻 `[ohm]`、电感 `[H]`、时间 `[s]`、
//!   磁链 `[Wb]`、角度 `[rad]`、电角速度 `[rad/s]`。
//! 定点说明 / Fixed point:
//!   本文件全部量都是 `f32`，不存在 Q1.15/Q1.31 定点站点。
//!   Every quantity here is `f32`; this file has no Q1.15/Q1.31 site.
//!
//! 参考 / Reference: `算法库移植状态.md`（Observer_BEMF* 行）、`算法库实时性说明.md`、
//!   `算法库总览与对接指南.md` §4.4、
//!   `docs/FOC算法组合与应用场景.md` §3.7（角度与速度估计层）

use super::{PllParam, PllState};
use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::AlphaBeta;

/// 瞬时反电势观测器的电机参数与滤波系数。
/// Motor and filter parameters of the instantaneous BEMF observer.
///
/// 全部字段都是 `f32`，`Default` 给出全 0；`ts = 0` 会让 `update` 直接返回零向量，
/// 所以默认值代表"未配置"，不是可用配置。
/// All fields are `f32` and `Default` yields zeros; a zero `ts` makes `update`
/// return a zero vector, so the default value means "not configured".
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm（见
    /// `docs/ST_MCSDK参考参数与仿真.md` 的参数基线表）。
    /// Per-phase stator resistance in `[ohm]`.
    ///
    /// 误差按 `Rs*i` 直接进入反电势结果，是低速精度的主要限制。
    /// An error here enters the EMF result directly as `Rs*i`.
    pub rs: f32,
    /// 定子同步电感 `[H]`；α/β 两轴共用一个值，不区分 `Ld`/`Lq`。
    /// Stator inductance in `[H]`, one value for both axes; saliency is ignored.
    ///
    /// `foc-control` 的 `BemfPllEstimator` 用 `Ld` 填入本字段（`[FW]`），所以凸极
    /// 电机在大电流下存在模型误差。
    /// `foc-control` fills this field with `Ld`, so salient machines carry model error.
    pub ls: f32,
    /// 调用周期 `[s]`；必须等于真实调用间隔（本项目快环为 `1/12000` `[s]`）。
    /// Sample period in `[s]`; it must equal the real call interval.
    ///
    /// `di/dt` 用后向差分 `Δi/ts` 计算：`ts` 写大会按比例低估电感压降，写小会放大
    /// 电流噪声（噪声增益为 `Ls/ts`）。
    /// `di/dt` uses a backward difference, so a wrong `ts` mis-scales the inductive term.
    pub ts: f32,
    /// 反电势一阶低通系数，无量纲，有效范围 `(0, 1]`；`1.0` 表示不滤波。
    /// First-order low-pass coefficient for the EMF estimate, dimensionless `(0, 1]`.
    ///
    /// 本文件会把它 `clamp` 到 `[0, 1]`，防止误配置让滤波器过冲或发散；代价是相位
    /// 滞后 `τ = -ts/ln(1-α)`（α = 0.05、12 kHz 时约 1.6 ms，折合约 100 Hz 截止），
    /// 本文件不做滞后补偿，PLL 只能被动跟随。
    /// The clamp stops a bad config from destabilising the filter, but the lag
    /// `τ = -ts/ln(1-α)` is not compensated anywhere in this file.
    ///
    /// 运行值来源：C 侧配置结构 `foc_rust_bridge.h` 的 `observer_emf_filter_alpha`
    /// （`[FW]`，`foc-sim` 默认 0.05）；`foc-control` 的 BEMF 路径硬编码 0.08。
    pub emf_filter_alpha: f32,
}

/// 瞬时反电势观测器状态。
/// State of the instantaneous BEMF observer.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfState {
    /// 上一拍的 αβ 电流 `[A]`，只用于后向差分求 `di/dt`，不是滤波器状态。
    /// Previous sample's αβ current in `[A]`, used only for the backward difference.
    pub last_current: AlphaBeta,
    /// 滤波后的 αβ 反电势估计 `[V]`，也是 `update` 的返回值。
    /// Filtered αβ BEMF estimate in `[V]`; also the value `update` returns.
    pub emf: AlphaBeta,
    /// 由反电势矢量推出的电角度 `[rad]`，范围 `[0, 2π)`。
    /// Electrical angle in `[rad]` derived from the EMF vector, in `[0, 2π)`.
    ///
    /// 取的是反电势矢量旋转 -90° 后的方向（`atan2(-e_alpha, e_beta)`）：非凸极
    /// PMSM 的反电势矢量超前转子 d 轴 90° 电角度，所以这个值就是 d 轴电角度，可以
    /// 直接送 Park。反电势符号或电流极性写反会让它整体偏 π，闭环看上去仍"收敛"，
    /// 只是转矩方向相反。
    /// The EMF vector rotated by -90°, which is the rotor d-axis angle under this
    /// crate's convention; a sign error biases it by π without breaking lock.
    pub theta_emf_rad: f32,
    /// 首次调用标志：0 表示 `last_current` 还没有有效历史。
    /// First-call flag: 0 means `last_current` has no valid history yet.
    ///
    /// 保留 `i32` 与 C 参考里的 `int` 宽度和含义一致。
    /// Kept as `i32` to match the C `int` in width and meaning.
    ///
    /// 为 0 时先把当前电流装进 `last_current`，使第一拍的 `di/dt` 为 0，避免用零
    /// 初值算出一个虚假的巨大电感压降。
    /// The first call seeds `last_current` so `di/dt` is zero for one sample.
    pub initialized: i32,
}

/// 观测器一拍输入（αβ 静止坐标系）。
/// One-sample observer input in the stationary αβ frame.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfInput {
    /// αβ 相电压 `[V]`；实时路径上由上一拍 PWM 占空比和母线电压重构，不是实测值，
    /// 因此含有死区/管压降误差和一拍延时。
    /// αβ phase voltage in `[V]`, reconstructed from the previous PWM command and the
    /// DC bus, so it carries dead-time error and one sample of delay.
    pub voltage: AlphaBeta,
    /// αβ 电流 `[A]`，实测相电流经 Clarke 变换得到。
    /// αβ current in `[A]` from the measured phase currents (Clarke).
    pub current: AlphaBeta,
}

/// `BemfState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `BemfState`.
impl BemfState {
    /// 把状态整体清零，等价于回到"首次调用"状态。
    /// Clears the whole state, i.e. goes back to the "first call" condition.
    ///
    /// 清零后 `last_current = 0` 而不是当前电流：从停机状态复位这正是想要的，从旋转
    /// 状态热复位时第一拍会少一个电感项。
    /// `last_current` is cleared rather than seeded, so the first sample after a hot
    /// reset misses the inductive term.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 用 `e = v - Rs*i - Ls*di/dt` 估计一拍反电势并做一阶滤波。
    /// Estimates one sample of BEMF via `e = v - Rs*i - Ls*di/dt` and low-pass filters it.
    ///
    /// 参数 / Parameters:
    ///   param - 电机参数与滤波系数，见 `BemfParam`
    ///   input - 本拍 αβ 电压 `[V]` 与 αβ 电流 `[A]`
    ///
    /// 返回 / Returns:
    ///   滤波后的 αβ 反电势 `[V]`（`self.emf`），不是未滤波的瞬时差值。
    ///   The filtered αβ BEMF in `[V]`, not the raw instantaneous difference.
    ///   `param.ts <= 0.0` 时返回零向量且不改动状态：这是"参数未配置"的安全返回，
    ///   调用方不能把零向量当成有效的零反电势。
    ///   With `ts <= 0.0` it returns zero and leaves the state untouched.
    ///
    /// 陷阱 / Pitfalls:
    ///   - 模型是 α/β 两轴独立的 RL 电路，没有 dq 交叉耦合项，也没有凸极修正。
    ///   - `di/dt` 是后向差分（一阶、滞后一拍），噪声增益 `Ls/ts`：12 kHz、
    ///     `Ls = 1.058 mH`（`[ST]`）时约 12.7 ohm，10 mA 电流噪声折算约 0.13 V。
    ///   - `Rs` 误差按 `Rs*i` 直接影响结果，`Ls` 误差只在电流变化剧烈时明显。
    ///   - 反电势幅值正比于转速，低速不可观（见文件头"可观性"）。
    ///   - Independent per-axis RL model; the backward difference amplifies current
    ///     noise by `Ls/ts`, and the estimate is unobservable near zero speed.
    ///
    /// 实时约束 / Real-time: 每拍一次 `libm::atan2f` 加少量乘加，无分配、无阻塞。
    pub fn update(&mut self, param: &BemfParam, input: &BemfInput) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        if self.initialized == 0 {
            self.last_current = input.current;
            self.initialized = 1;
        }
        // 后向差分 di/dt：一阶近似、滞后一拍，噪声增益为 1/ts。
        // Backward-difference di/dt: first order, one-sample lag, noise gain 1/ts.
        let di_alpha = (input.current.alpha - self.last_current.alpha) / param.ts;
        let di_beta = (input.current.beta - self.last_current.beta) / param.ts;
        // 从相电压里依次扣除电阻压降 Rs*i 和电感压降 Ls*di/dt，剩下的才是反电势。
        // The resistive and inductive drops are removed from the applied voltage.
        let raw = AlphaBeta {
            alpha: input.voltage.alpha - param.rs * input.current.alpha - param.ls * di_alpha,
            beta: input.voltage.beta - param.rs * input.current.beta - param.ls * di_beta,
        };
        // 低通系数夹到 [0, 1]：α > 1 会过冲振荡，α < 0 会发散；`clamp` 不拦 NaN
        // （见 `crate::math::clamp`），所以参数必须由上层保证有限。
        // Clamped to [0, 1]; NaN passes through `clamp`, so the caller must validate.
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        self.emf.alpha += alpha * (raw.alpha - self.emf.alpha);
        self.emf.beta += alpha * (raw.beta - self.emf.beta);
        // 反电势矢量再旋转 -90° 得到 d 轴电角度，见 `theta_emf_rad` 的字段说明。
        // Rotate the EMF vector by -90° to obtain the d-axis electrical angle.
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        self.last_current = input.current;
        self.emf
    }
}

/// 积分型（定子磁链）观测器的参数。
/// Parameters of the integral / stator flux-linkage observer.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfIntegralParam {
    /// 每相定子电阻 `[ohm]`；`[ST]` 基准 5.29 ohm。
    /// Per-phase stator resistance in `[ohm]`.
    ///
    /// 纯积分对 `Rs` 误差没有抑制：偏差 ε 会在磁链上累积 `ε*∫i dt`，直流或低频电流
    /// 分量会把它推成斜坡，直到撞上限幅，而截断期间给出的角度是错的。
    /// A pure integrator has no rejection of `Rs` error: a bias accumulates `ε*∫i dt`.
    pub rs: f32,
    /// 积分步长 `[s]`；必须等于真实调用周期。
    /// Integration step in `[s]`; it must equal the real call period.
    pub ts: f32,
    /// 泄漏（去直流）系数，量纲 `[1/s]`，等价于一个角频率截止。
    /// Leakage / DC-rejection coefficient in `[1/s]`, i.e. an angular cutoff.
    ///
    /// 它把纯积分 `1/s` 变成一阶低通 `1/(s + leakage)`：直流增益由无穷降为
    /// `1/leakage`，从而抑制积分漂移；代价是截止频率附近的幅值衰减和相位超前。
    /// `0.0` 表示退回纯积分，漂移风险由调用方承担。
    /// Turns the pure integrator into a first-order low-pass with DC gain `1/leakage`;
    /// `0.0` degenerates back to a pure integrator.
    pub leakage: f32,
    /// α/β 磁链下限 `[Wb]`（含端点）。
    /// Lower α/β flux limit in `[Wb]` (inclusive).
    ///
    /// 这是逐轴限幅而不是矢量幅值限幅：某一轴被截断会改变合成矢量方向，饱和期间
    /// 角度出现偏差；需要保方向请用 `crate::observer::FluxImprovedState`。
    /// Per-axis, not vector-magnitude limiting: clipping one axis rotates the vector.
    ///
    /// 限幅至少要覆盖真实磁链幅值（`ψpm + Ld*|id|`），否则正常运行就会削顶。
    /// It must cover the real flux magnitude or normal operation clips.
    pub flux_min: f32,
    /// α/β 磁链上限 `[Wb]`（含端点），说明见 `flux_min`。
    /// Upper α/β flux limit in `[Wb]` (inclusive); see `flux_min`.
    pub flux_max: f32,
}

/// 积分型观测器状态：αβ 定子磁链和由它得到的电角度。
/// Integral observer state: αβ stator flux linkage and the angle derived from it.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfIntegralState {
    /// αβ 定子磁链 `[Wb]`（已限幅），也是 `update` 的返回值。
    /// Clamped αβ stator flux linkage in `[Wb]`; also the value `update` returns.
    pub flux: AlphaBeta,
    /// 磁链矢量角 `[rad]`，范围 `[0, 2π)`。
    /// Flux-vector angle in `[rad]`, in `[0, 2π)`.
    ///
    /// `atan2(ψβ, ψα)` 直接给出转子 d 轴（永磁磁链）方向，与 `BemfState` 的反电势
    /// 角度相差 90°：两者不能互换使用。
    /// This is the d-axis angle and is 90° away from the BEMF-based angle; the two
    /// must not be swapped.
    pub theta_rad: f32,
}

/// 积分型观测器复用瞬时观测器的输入类型（αβ 电压 `[V]` / 电流 `[A]`）。
/// The integral observer reuses the instantaneous observer's αβ input type.
pub type BemfIntegralInput = BemfInput;

/// `BemfIntegralState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `BemfIntegralState`.
impl BemfIntegralState {
    /// 清零磁链与角度；应在电机静止、电流为零时复位，否则积分从非零残值开始。
    /// Clears flux and angle; reset at standstill with zero current.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 对 `dψ/dt = v - Rs*i - leakage*ψ` 做一步显式欧拉积分，限幅后求磁链角。
    /// One explicit-Euler step of `dψ/dt = v - Rs*i - leakage*ψ`, then clamp and angle.
    ///
    /// 参数 / Parameters: param - 见 `BemfIntegralParam`；
    ///   input - αβ 电压 `[V]` 与 αβ 电流 `[A]`。
    /// 返回 / Returns: 限幅后的 αβ 磁链 `[Wb]`；`param.ts <= 0.0` 时返回零向量且
    ///   不改状态（"参数未配置"的安全返回）。
    ///
    /// 陷阱 / Pitfalls:
    ///   - 积分器在低频段有 -90° 相移，加上 `Rs` 误差和逆变器死区电压，低速精度
    ///     明显差于电流模型，这正是电流模型/混合方案存在的原因。
    ///   - 显式欧拉的截断误差随 `ts` 增大而增大，`ts` 必须与真实周期一致。
    ///   - The integrator's -90° phase and `Rs` error make this model weak at low
    ///     speed, which is why the current model and hybrid variants exist.
    ///
    /// 实时约束 / Real-time: 每拍 4 次乘法、2 次 `clamp` 和 1 次 `libm::atan2f`，
    ///   无分配、无阻塞。
    pub fn update(&mut self, param: &BemfIntegralParam, input: &BemfIntegralInput) -> AlphaBeta {
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
        self.theta_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

/// 六步无感 BLDC 浮空相反电势过零检测参数。
/// Parameters of floating-phase BEMF zero-cross detection for six-step BLDC control.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfZeroCrossParam {
    /// 过零消隐带宽度 `[V]`，内部取绝对值使用。
    /// Zero-cross blanking band in `[V]`, used as a magnitude.
    ///
    /// `|bemf| <= threshold` 一律判为"无符号"（返回 0），用来抑制过零点附近的噪声
    /// 抖动和 PWM 开关噪声。取值必须远小于反电势幅值，否则真实过零会被吞掉；太小
    /// 则每拍都可能翻转，换来虚假换相事件。
    /// A wider band suppresses noise but can swallow a real crossing.
    ///
    /// 标定值来源未在仓库内记录：本类型在当前仓库内没有实时调用方，C 侧配置结构
    /// `foc_rust_bridge.h` 里也没有对应的过零阈值字段。
    /// The calibration origin is not recorded in-tree; there is no runtime caller and
    /// no C-side config field for this threshold.
    pub threshold: f32,
}

/// 过零检测状态，含去抖用的上次符号。
/// Zero-cross state, including the last sign used for debouncing.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfZeroCrossState {
    /// 上一次非零符号（-1/0/+1）；信号停留在消隐带内时保持不变。
    /// Last non-zero sign (-1/0/+1); held while the signal stays inside the band.
    ///
    /// 只在 `current_sign != 0` 时更新，所以消隐带在这里充当滞环记忆：代价是一次
    /// 真实过零最多被推迟"信号停留在带内的时间"，六步换相的定时刻度直接受它影响。
    /// Updated only on a non-zero sign, so the band acts as hysteresis memory and can
    /// delay the reported crossing.
    pub last_sign: i32,
    /// 本拍是否发生过零（1/0）；每次 `update` 开头清零，是单拍脉冲而不是电平。
    /// One-shot crossing flag for this sample; cleared at the start of every `update`.
    ///
    /// 保留 `i32` 与 C 参考里的 `int` 宽度和含义一致。
    /// Kept as `i32` to match the C `int` in width and meaning.
    ///
    /// 调用方必须按固定节拍调用（至少每个 PWM 周期一次），漏拍就会漏掉过零事件。
    /// Callers must call it every PWM period or crossings are lost.
    pub crossing: i32,
    /// 本拍过零后的符号（+1 表示由负到正，-1 表示由正到负，0 表示无事件）。
    /// Sign after the crossing (+1 rising, -1 falling, 0 no event).
    pub direction: i32,
    /// 上一拍的反电势 `[V]`，仅作诊断/遥测；本文件内没有读取点。
    /// Previous BEMF sample in `[V]` kept for diagnostics; nothing reads it in-file.
    pub last_bemf: f32,
}

/// 带消隐带的三值符号函数：带外返回 ±1，带内返回 0。
/// Three-valued sign with a blanking band: ±1 outside the band, 0 inside.
///
/// `threshold` 取绝对值，所以误配负数不会被当成"反向带"；`NaN` 的两个比较都为假，
/// 结果按带内处理（返回 0），也就是 NaN 永远不会触发过零，也不会污染 `last_sign`。
/// `threshold` is used as a magnitude; `NaN` falls into the band and never triggers.
fn bemf_sign(value: f32, threshold: f32) -> i32 {
    let threshold = threshold.abs();
    if value > threshold {
        1
    } else if value < -threshold {
        -1
    } else {
        0
    }
}

/// `BemfZeroCrossState` 的复位与逐拍检测入口。
/// Reset and per-sample detection entry points of `BemfZeroCrossState`.
impl BemfZeroCrossState {
    /// 清零符号、事件和方向；下一次 `update` 会重新建立 `last_sign`。
    /// Clears sign, event and direction; the next `update` rebuilds `last_sign`.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 检测一次过零事件，必要时更新 `last_sign`。
    /// Detects one zero-crossing event and updates `last_sign` when appropriate.
    ///
    /// 参数 / Parameters: param - 消隐带 `[V]`；bemf - 本拍浮空相反电势 `[V]`。
    /// 返回 / Returns: 本拍发生过零返回 1，否则返回 0；方向在 `self.direction`。
    ///   1 on the sample where a crossing is detected, otherwise 0.
    ///
    /// 使用方式 / Usage: 六步换相的 30° 延迟要由调用方在过零后用定时器/计数延时
    /// 自己安排，本函数只报告事件，不生成换相时刻。输入最好取浮空相端电压减去中性
    /// 点电压；用重构电压会引入占空比误差和死区误差。
    /// The 30° commutation delay must be scheduled by the caller; this function only
    /// reports the event.
    ///
    /// 实时约束 / Real-time: 每拍只有比较和赋值，无除法、无查表，可放在 ISR 内。
    pub fn update(&mut self, param: &BemfZeroCrossParam, bemf: f32) -> i32 {
        let current_sign = bemf_sign(bemf, param.threshold);
        self.crossing = 0;
        self.direction = 0;
        if current_sign != 0 && self.last_sign != 0 && current_sign != self.last_sign {
            self.crossing = 1;
            self.direction = current_sign;
        }
        if current_sign != 0 {
            self.last_sign = current_sign;
        }
        self.last_bemf = bemf;
        self.crossing
    }
}

/// 反电势观测器与 PLL 的组合参数。
/// Combined parameters of the BEMF observer and the PLL.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfPllParam {
    /// 内层反电势观测器参数，见 `BemfParam`。
    /// Inner BEMF observer parameters.
    pub bemf: BemfParam,
    /// 角度环 PLL 参数，见 `crate::observer::PllParam`（`kp`/`ki`/`ts`/`omega_min`/
    /// `omega_max`，其中 `kp` 作用在 `sin(Δθ)` 上）。
    /// PLL parameters; `kp` acts on `sin(Δθ)`.
    ///
    /// PLL 的输入是本文件给出的反电势角，因此它同时继承低通滞后和反电势符号约定：
    /// PLL 带宽开得比反电势滤波截止频率还高，只会把噪声放大成角度抖动。
    /// The PLL inherits the EMF filter lag and sign convention; a bandwidth wider than
    /// the EMF filter cutoff only amplifies noise into angle jitter.
    pub pll: PllParam,
}

/// 反电势 + PLL 组合观测器状态。
/// State of the combined BEMF + PLL observer.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfPllState {
    /// 内层反电势观测器状态。
    /// Inner BEMF observer state.
    pub bemf: BemfState,
    /// 内层 PLL 状态（角度、角速度、积分器、相位误差）。
    /// Inner PLL state (angle, speed, integrator, phase error).
    pub pll: PllState,
    /// 本拍滤波后的 αβ 反电势 `[V]`，是 `bemf.emf` 的副本，便于上层取用。
    /// Copy of the filtered αβ EMF in `[V]` for upper layers.
    pub emf: AlphaBeta,
    /// PLL 平滑后的电角度 `[rad]`，范围 `[0, 2π)`。
    /// Smoothed electrical angle in `[rad]`, in `[0, 2π)`.
    pub theta_rad: f32,
    /// PLL 输出的电角速度 `[rad/s]`（受 PLL 的 `omega_min`/`omega_max` 限幅）。
    /// Estimated electrical angular speed in `[rad/s]`, clamped by the PLL limits.
    ///
    /// 机械转速需除以极对数，`foc-control` 的 `RotorFeedback` 就是这么换算的。
    /// Divide by the pole pairs for mechanical speed.
    pub omega_rad_s: f32,
}

/// `BemfPllState` 的复位与逐拍更新入口。
/// Reset and per-sample update entry points of `BemfPllState`.
impl BemfPllState {
    /// 把内层观测器和 PLL 一起清零。
    /// Clears the inner observer and the PLL together.
    ///
    /// PLL 复位后角度为 0，需要角度初值时由调用方另外设置 `pll.theta_rad`
    /// （`foc-control` 用 `PllState::reset(initial_theta_rad)` 达到同样目的）。
    /// The PLL restarts at angle 0; callers needing an initial angle must set it.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 依次运行反电势估计和 PLL，返回平滑电角度。
    /// Runs the BEMF estimate and the PLL in sequence and returns the smoothed angle.
    ///
    /// 参数 / Parameters: param - 见 `BemfPllParam`；input - αβ 电压 `[V]`、电流 `[A]`。
    /// 返回 / Returns: PLL 平滑后的电角度 `[rad]`；本拍电角速度见 `omega_rad_s`。
    ///
    /// 说明 / Note: PLL 的输入是角度而不是相位误差，相位检测在 `PllState` 内部用
    /// `sin(θ_meas - θ)` 完成，跨零由它自己的角度环绕处理，调用方不需要预处理。
    /// 本文件不做反电势幅值可信度判断：反电势很小或符号错时 PLL 仍会输出一个看起来
    /// 正常的角度，可靠性判据必须由上层另外实现。
    /// The PLL takes an angle, not a phase error; no EMF-magnitude reliability check
    /// exists here, so that judgement belongs to the caller.
    pub fn update(&mut self, param: &BemfPllParam, input: &BemfInput) -> f32 {
        self.emf = self.bemf.update(&param.bemf, input);
        self.theta_rad = self.pll.update(&param.pll, self.bemf.theta_emf_rad);
        self.omega_rad_s = self.pll.omega_rad_s;
        self.theta_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 浮点近似比较助手：容差由调用方给出，用来吸收最后一位的舍入差异。
    /// Approximate-float comparison helper; the tolerance absorbs last-bit rounding.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 对照 C 参考向量的瞬时反电势测试。
    /// C reference vectors for the instantaneous BEMF estimate.
    ///
    /// 期望值 9.0 / 7.5 依赖"先扣电阻项、再扣电感项、首拍 `di/dt = 0`"的运算顺序，
    /// 改动运算顺序或首拍分支就会失配——这个测试存在的意义正是锁住这份等价性。
    /// The expected values pin the operation order and the first-call branch.
    #[test]
    fn bemf_matches_c_reference() {
        let param = BemfParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.001,
            emf_filter_alpha: 1.0,
        };
        let mut input = BemfInput {
            voltage: AlphaBeta {
                alpha: 10.0,
                beta: -4.0,
            },
            current: AlphaBeta {
                alpha: 2.0,
                beta: -2.0,
            },
        };
        let mut state = BemfState::default();
        let emf = state.update(&param, &input);
        near(emf.alpha, 9.0, 1e-6);
        near(emf.beta, -3.0, 1e-6);
        input.current = AlphaBeta {
            alpha: 3.0,
            beta: -3.0,
        };
        let emf = state.update(&param, &input);
        near(emf.alpha, 7.5, 1e-6);
        near(emf.beta, -1.5, 1e-6);
    }

    /// 对照 C 参考向量的积分磁链测试，并验证 `flux_max` 处的逐轴限幅行为。
    /// C reference vectors for the integral flux, including the `flux_max` clamp.
    #[test]
    fn integral_matches_c_reference() {
        let param = BemfIntegralParam {
            rs: 0.5,
            ts: 0.01,
            leakage: 0.0,
            flux_min: -1.0,
            flux_max: 1.0,
        };
        let mut input = BemfIntegralInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 1.0,
            },
            current: AlphaBeta {
                alpha: 1.0,
                beta: 0.0,
            },
        };
        let mut state = BemfIntegralState::default();
        let flux = state.update(&param, &input);
        near(flux.alpha, 0.015, 1e-6);
        near(flux.beta, 0.01, 1e-6);
        input.voltage.alpha = 1000.0;
        near(state.update(&param, &input).alpha, 1.0, 1e-6);
    }

    /// 对照 C 参考的过零序列：带内样本不产生事件，符号真正改变才置位 `crossing`。
    /// C reference sequence for zero-cross detection (blanking band, then sign change).
    #[test]
    fn zero_cross_matches_c_reference() {
        let param = BemfZeroCrossParam { threshold: 0.1 };
        let mut state = BemfZeroCrossState::default();
        assert_eq!(state.update(&param, -1.0), 0);
        assert_eq!(state.update(&param, 0.05), 0);
        assert_eq!(state.update(&param, 1.0), 1);
        assert_eq!(state.direction, 1);
        assert_eq!(state.update(&param, -1.0), 1);
        assert_eq!(state.direction, -1);
    }

    /// 对照 C 参考的 BEMF+PLL 测试：验证 PLL 能跟上非零速旋转的反电势矢量。
    /// C reference vectors for the BEMF + PLL combination at non-zero speed.
    #[test]
    fn bemf_pll_matches_c_reference() {
        let param = BemfPllParam {
            bemf: BemfParam {
                rs: 0.5,
                ls: 0.001,
                ts: 0.001,
                emf_filter_alpha: 1.0,
            },
            pll: PllParam {
                kp: 10.0,
                ki: 0.0,
                ts: 0.001,
                omega_min: -100.0,
                omega_max: 100.0,
            },
        };
        let mut input = BemfInput {
            voltage: AlphaBeta {
                alpha: 0.0,
                beta: 10.0,
            },
            current: AlphaBeta::default(),
        };
        let mut state = BemfPllState::default();
        near(state.update(&param, &input), 0.0, 1e-6);
        near(state.emf.beta, 10.0, 1e-6);
        input.voltage = AlphaBeta {
            alpha: -10.0,
            beta: 0.0,
        };
        assert!(state.update(&param, &input) > 0.0);
        assert!(state.omega_rad_s > 0.0);
    }
}
