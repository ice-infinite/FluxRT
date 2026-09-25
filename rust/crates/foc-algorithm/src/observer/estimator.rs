//! 标量 Kalman、二阶 Luenberger、EKF、EKF+FOC 与一维 UKF。
//! Scalar Kalman, second-order discrete Luenberger, EKF, EKF+FOC and a
//! one-dimensional UKF.
//!
//! EKF/UKF 模型使用可空的 `extern "C" fn`，不使用堆分配或 trait object。
//! EKF/UKF models arrive as nullable `extern "C" fn` pointers: no heap allocation
//! and no trait object / dynamic dispatch.
//!
//! FluxRT —— 通用状态估计器（`foc-algorithm` 的 `observer` 子模块）。
//! FluxRT - generic state estimators, an `observer` sub-module of `foc-algorithm`.
//!
//! 职责 / Responsibility:
//!   - `KalmanState`：一维线性 Kalman 的预测与量测校正两步。
//!   - `LuenbergerState`：二阶离散线性 Luenberger 观测器。
//!   - `EkfState`：一维 EKF；状态函数 `f`、状态雅可比、量测函数 `h`、量测雅可比
//!     全部由回调提供。
//!   - `EkfFocState`：EKF 角度估计与基础 FOC 电流环组合成的一拍链路。
//!   - `UkfState`：一维 UKF，用固定 3 个 sigma 点传播非线性模型。
//!
//! 架构位置与依赖方向 / Place in the architecture and dependency direction:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!     -> foc-algorithm -> `observer::estimator`（本模块）
//!   除 `crate::math`（角度环绕）外，还依赖同 crate 的 `crate::foc`（`EkfFocState`
//!   内嵌 `FocBasicState`）与 `crate::modulation::SvpwmOutput`；依赖方向仍然是
//!   向下的纯算法依赖，本模块不认识 HAL、RTOS、堆，也没有全局可变状态。
//!   Besides `crate::math` this module uses `crate::foc` and `crate::modulation`
//!   for the combined chain; it still knows nothing about the HAL, the RTOS or the
//!   heap.
//!
//! 实时约束 / Real-time constraints:
//!   可能被 12 kHz 的 ADC 注入中断逐拍调用。全部为纯计算：不分配、不阻塞、
//!   不打日志、无锁、无 trait object。但 EKF/UKF 的执行时间由**调用方提供的回调**
//!   决定：EKF 每拍调用 4 个回调，UKF 每拍调用 `f` 三次、`h` 三次；每个回调都必须
//!   可重入、固定时间、不阻塞，并且不得在控制 ISR 里分配内存。是否把估计器放进
//!   快环，必须由目标板上的 WCET 实测决定（见 `算法库实时性说明.md`）。
//!   Pure computation with no allocation or locking, but the cost of EKF/UKF is set
//!   by the caller-supplied callbacks (4 per EKF step, 6 per UKF step), so in-ISR
//!   placement must be decided by measured WCET.
//!
//! 量纲约定 / Unit conventions:
//!   本模块是标量滤波器，量纲完全由调用方的状态定义决定。若状态为电角度，则 `x`
//!   与 `p` 的单位是 `[rad]` 与 `[rad²]`，`q` 是每步过程噪声方差 `[rad²]`，`r` 是
//!   量测噪声方差（量测为角度时也是 `[rad²]`）；若状态为转速，则对应
//!   `[rad/s]` 与 `[(rad/s)²]`。`LuenbergerParam` 的 `a/b/c/l` 必须是**已经离散化**
//!   的矩阵元素，其量纲由状态选择决定。本模块没有任何定点 Q 格式，全部为 `f32`。
//!   These are scalar filters: units follow the caller's state definition. `q` is a
//!   per-step process-noise variance and `r` a measurement-noise variance; the
//!   Luenberger matrices must already be discrete.
//!
//! 常数来源 / Constant provenance:
//!   本模块不含任何带 `[HW]`/`[ST]`/`[FW]`/`[VESC]` 标签的标定常数：`a`/`h`/`q`/`r`、
//!   Luenberger 的 `A/B/C/L`、EKF/UKF 的 `q`/`r` 与 `alpha`/`beta`/`kappa` 全部由
//!   调用方整定，仓库内没有记录这些数值的来源。`Default` 全为 0 只表示"未配置"，
//!   不是可用默认值；UKF 内部使用的 0.001/1.0 兜底常量同样没有可查的来源记录。
//!   No calibrated constants with provenance tags appear here: every coefficient is
//!   caller-tuned and the fallback literals have no recorded in-tree origin.
//!
//! 参考 / Reference: `算法库移植状态.md` 第 43～47 行、`算法库总览与对接指南.md`
//!   §4.6、`算法库实时性说明.md`（状态对象尺寸、UKF 临时数据、EKF/UKF 回调）。

use crate::foc::{FocBasicInput, FocBasicParam, FocBasicState};
use crate::math::wrap_angle_0_to_2pi;
use crate::modulation::SvpwmOutput;

/// 一维线性 Kalman 滤波参数。
/// Parameters of the one-dimensional linear Kalman filter.
///
/// 模型 / Model: `x[k+1] = a·x[k] + b·u[k]`，`y[k] = h·x[k]`。
/// 参数 / Parameters:
///   a 离散状态转移系数（每步）。它必须是**离散**值，例如一阶惯性的
///     `a = exp(-ts/tau)` 或恒值模型的 `a = 1.0`；本库不含 `ts`，也不做离散化。
///     Discrete state-transition coefficient per step; the library performs no
///     discretisation, so `ts` has to be baked in by the caller.
///   b 输入增益，量纲 `[x]/[u]`；不需要输入时置 0。
///     Input gain in `[x]/[u]`; set 0 when there is no control input.
///   h 量测系数，量纲 `[y]/[x]`；恒等量测取 1。
///     Measurement coefficient in `[y]/[x]`.
///   q 每步过程噪声方差，量纲 `[x²]`，必须 `>= 0`。调大表示更相信量测。
///     Per-step process-noise variance in `[x²]`, must be `>= 0`.
///   r 量测噪声方差，量纲 `[y²]`，必须 `> 0`；它是唯一防止增益溢出的项。
///     Measurement-noise variance in `[y²]`, must be `> 0`.
///     具体数值来源未在仓库内标注，必须按实际采样噪声实测整定。
///     The concrete values are not identified in-tree and must be tuned from
///     measured noise.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanParam {
    pub a: f32,
    pub b: f32,
    pub h: f32,
    pub q: f32,
    pub r: f32,
}

/// 一维 Kalman 滤波状态。
/// State of the one-dimensional Kalman filter.
///
/// 字段 / Fields:
///   x        当前（后验）状态估计，量纲 `[x]`。
///            Current a-posteriori state estimate in `[x]`.
///   p        估计误差协方差，量纲 `[x²]`。它必须保持非负：负值意味着"方差"失去
///            意义，之后增益和修正方向都会出错。
///            Estimation-error covariance in `[x²]`; it must stay non-negative.
///   k        最近一次的 Kalman 增益，量纲 `[x]/[y]`（状态与量测同量纲时是无量纲
///            比值）。保存下来用于诊断，例如判断滤波器是否已经"不信任量测"。
///            Last Kalman gain in `[x]/[y]`; kept for diagnostics.
///   residual 最近一次的创新量（量测减去先验预测）`[y]`，是滤波器健康度与故障
///            检测最直接的信号。
///            Last innovation (measurement minus a-priori prediction) in `[y]`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct KalmanState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

/// 一维 Kalman 滤波器（`Observer_Kalman`）。
/// The one-dimensional Kalman filter (`Observer_Kalman`).
///
/// 定位 / Placement: 这是标量辅助级滤波器，用于角度平滑、速度或单参数
/// 估计；电流环的逐拍控制不应依赖它。
/// A scalar helper filter for angle smoothing, speed or single-parameter
/// estimation; the fast current loop should not depend on it.
impl KalmanState {
    /// 构造一个滤波器状态。
    /// Builds a filter state.
    ///
    /// 参数 / Parameters:
    ///   initial_x 初始状态估计 `[x]`
    ///   initial_p 初始误差协方差 `[x²]`，必须 `>= 0`
    ///
    /// `const fn` 是刻意的：状态对象可以在初始化阶段（甚至编译期常量）建立，
    /// 不必也不应该出现在 12 kHz ISR 里。`initial_p = 0` 且 `q = 0` 会让增益恒为 0，
    /// 估计值永远停在 `initial_x`，这是最常见的"滤波器不工作"原因。
    /// `const fn` on purpose: the state object belongs to initialisation, never to
    /// the ISR. `initial_p = 0` together with `q = 0` freezes the estimate at
    /// `initial_x` because the gain stays 0.
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    /// 复位滤波器：把状态与协方差设为给定初值，并清空增益与创新量。
    /// Resets the filter to the given state and covariance, clearing the gain and
    /// the innovation.
    ///
    /// 复位会一并清掉 `k` 与 `residual`（由 `new` 完成），所以不会残留上一次运行的
    /// 增益；重新使能、换模式或长时间失联后必须复位，否则旧协方差会让滤波器
    /// 对新工况过度自信。
    /// Reset also clears `k` and `residual`, so no stale gain survives.
    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    /// 推进一拍 Kalman 滤波（预测 + 量测校正），返回后验估计。
    /// Runs one Kalman step (predict then correct) and returns the a-posteriori
    /// estimate.
    ///
    /// 参数 / Parameters:
    ///   param       滤波参数（见 `KalmanParam`）
    ///   u           控制/输入量 `[u]`（量纲由 `b` 决定，`b = 0` 时可传任意有限值）
    ///   measurement 量测值 `[y]`（量纲由 `h` 决定）
    ///
    /// 返回 / Returns: 后验估计 `self.x`，量纲 `[x]`。
    ///   The a-posteriori estimate `self.x` in `[x]`.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 先验协方差为 `a·p·a + q`，全程是**离散**递推；连续模型的离散化与 `q`
    ///     的折算完全由调用方负责，采样周期改了却不同步改参数是常见错误。
    ///   - `s = h·p_pred·h + r` 恰为 0 时被强制成 1.0：这是 C 参考实现的除零保护，
    ///     只覆盖"精确等于 0"。`r < 0` 且 `|h²·p_pred| < |r|` 会让 `s`、`k` 变号，
    ///     滤波器会朝错误方向修正并发散，所以 `r` 必须 `> 0`。
    ///   - 只要初始 `p >= 0`、`q >= 0`、`r > 0`，就有 `k·h < 1`，协方差
    ///     `p = (1 - k·h)·p_pred` 不会变负；反之传入负的 `q` 或负的初始 `p` 会让
    ///     协方差失去方差含义。
    ///   - 不判 `NaN/Inf`、不限幅：量测异常会直接污染 `x` 与 `p`，送入前必须做
    ///     有效性检查（本工程要求量测/参数应为有限浮点）。
    ///   - 运算顺序与 C 参考实现一致，改动会让 `tests::kalman_matches_c_reference`
    ///     失配。
    pub fn update(&mut self, param: &KalmanParam, u: f32, measurement: f32) -> f32 {
        let x_pred = param.a * self.x + param.b * u;
        let p_pred = param.a * self.p * param.a + param.q;
        let mut s = param.h * p_pred * param.h + param.r;
        if s == 0.0 {
            s = 1.0;
        }
        self.k = p_pred * param.h / s;
        self.residual = measurement - param.h * x_pred;
        self.x = x_pred + self.k * self.residual;
        self.p = (1.0 - self.k * param.h) * p_pred;
        self.x
    }
}

/// 二阶离散线性 Luenberger 观测器参数。
/// Parameters of the second-order discrete linear Luenberger observer.
///
/// 模型 / Model: `x[k+1] = A·x[k] + B·u[k] + L·(y - C·x[k])`，`y_hat = C·x[k]`。
///
/// 参数 / Parameters:
///   a00/a01/a10/a11 状态矩阵 `A` 的元素（已经是离散值）。矩阵元素的量纲由状态
///                   选择决定：例如 `x0 = 角度 [rad]`、`x1 = 角速度 [rad/s]` 时，
///                   `a01` 是每步 `[rad/s] -> [rad]` 的耦合，`a10` 通常是 0。
///                   Discrete state-matrix entries; their units follow the state
///                   choice.
///   b0/b1           输入矩阵 `B` 的元素，量纲 `[x]/[u]`；没有输入时置 0。
///                   Input-matrix entries in `[x]/[u]`.
///   c0/c1           输出矩阵 `C` 的元素，量纲 `[y]/[x]`；只测 `x0` 时取
///                   `c0 = 1, c1 = 0`。
///                   Output-matrix entries in `[y]/[x]`.
///   l0/l1           观测器增益 `L`。它必须让 `(A - L·C)` 的特征值落在单位圆内，
///                   本库**不做**极点配置也不做稳定性检查：整定属于离线/设计期
///                   工作，错误的增益会导致观测器发散而不是报错。
///                   Observer gains; `(A - L·C)` must be Schur-stable. This library
///                   performs no pole placement and no stability check.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LuenbergerParam {
    pub a00: f32,
    pub a01: f32,
    pub a10: f32,
    pub a11: f32,
    pub b0: f32,
    pub b1: f32,
    pub c0: f32,
    pub c1: f32,
    pub l0: f32,
    pub l1: f32,
}

/// 二阶 Luenberger 观测器状态。
/// State of the second-order Luenberger observer.
///
/// 字段 / Fields:
///   x0       第一个状态（常见选择：角度 `[rad]`）。
///            First state (a common choice is an angle in `[rad]`).
///   x1       第二个状态（常见选择：角速度 `[rad/s]`）。
///            Second state (often an angular speed in `[rad/s]`).
///   y_hat    最近一次输出的**后验**估计 `[y]`（由更新后的状态重新算出）。
///            Last a-posteriori output estimate in `[y]`.
///   error    最近一次的量测偏差 `[y]`，且是**先验**偏差（用更新前的状态算得）。
///            注意 `error` 与 `y_hat` 不是同一时刻的量：前者是先验、后者是后验，
///            做诊断时不要用 `measured_y - y_hat` 反推 `error`。
///            Last innovation in `[y]`, computed a-priori while `y_hat` is
///            a-posteriori; the two do not belong to the same instant.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LuenbergerState {
    pub x0: f32,
    pub x1: f32,
    pub y_hat: f32,
    pub error: f32,
}

/// 二阶离散 Luenberger 观测器（`Observer_Luenberger`）。
/// The second-order discrete Luenberger observer (`Observer_Luenberger`).
impl LuenbergerState {
    /// 构造观测器状态。
    /// Builds the observer state.
    ///
    /// 参数 / Parameters:
    ///   x0 第一个状态初值（量纲由参数矩阵定义）
    ///   x1 第二个状态初值
    ///
    /// 输出估计与偏差都从 0 开始；首拍会用一个明显偏离的量测做一次较大修正，
    /// 因此上电后应先等待若干拍再判定观测器是否收敛。
    /// The output estimate and the error start at 0; expect a large correction on
    /// the first step.
    pub const fn new(x0: f32, x1: f32) -> Self {
        Self {
            x0,
            x1,
            y_hat: 0.0,
            error: 0.0,
        }
    }

    /// 复位观测器状态：设定两个状态初值，并把输出估计与偏差清零。
    /// Resets the observer: sets both states and clears the output estimate and the
    /// error.
    ///
    /// 观测器没有协方差，收敛速度只由增益 `L` 决定；复位后同样需要若干拍才能
    /// 让估计追上真实量测。
    /// There is no covariance here: only the gains `L` set the convergence speed.
    pub fn reset(&mut self, x0: f32, x1: f32) {
        *self = Self::new(x0, x1);
    }

    /// 推进一拍二阶 Luenberger 观测器，返回输出的后验估计。
    /// Runs one observer step and returns the a-posteriori output estimate.
    ///
    /// 参数 / Parameters:
    ///   param      观测器参数（见 `LuenbergerParam`）
    ///   input_u    输入量 `[u]`
    ///   measured_y 量测值 `[y]`
    ///
    /// 返回 / Returns: 用**更新后**状态算出的输出估计 `y_hat` `[y]`。
    ///   The output estimate `y_hat` in `[y]`, computed from the updated state.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 两个状态的新值先算进局部变量再整体写回，所以它们用的是**同一拍**的旧
    ///     状态，是真正的"同步更新"。若改成先写 `x0` 再算 `x1`，`x1` 会用到已更新
    ///     的 `x0`，等价于偷偷改变了 `A` 矩阵，观测器极点随之偏移——这是移植这类
    ///     递推时最容易引入的错误。
    ///   - `y_hat` 在返回前被重算为后验值，而 `error` 保留先验值；上层诊断时不要
    ///     假设 `measured_y - y_hat == error`。
    ///   - 不做限幅、不判 `NaN/Inf`，也不检查 `(A - L·C)` 的稳定性；量测异常或增益
    ///     不当会直接体现在发散的状态上。
    ///   - 运算顺序与 C 参考实现一致，改动会让
    ///     `tests::luenberger_matches_c_reference` 失配。
    pub fn update(&mut self, param: &LuenbergerParam, input_u: f32, measured_y: f32) -> f32 {
        self.y_hat = param.c0 * self.x0 + param.c1 * self.x1;
        self.error = measured_y - self.y_hat;
        let next_x0 =
            param.a00 * self.x0 + param.a01 * self.x1 + param.b0 * input_u + param.l0 * self.error;
        let next_x1 =
            param.a10 * self.x0 + param.a11 * self.x1 + param.b1 * input_u + param.l1 * self.error;
        self.x0 = next_x0;
        self.x1 = next_x1;
        self.y_hat = param.c0 * self.x0 + param.c1 * self.x1;
        self.y_hat
    }
}

/// EKF 状态回调类型：`f(x, u) -> f32`，用于状态转移 `f` 及其雅可比 `df/dx`。
/// EKF state callback type, used for both the transition `f` and its Jacobian.
///
/// 契约 / Contract（`算法库实时性说明.md` 与 README 的硬性要求）:
///   - 模型必须是**已经离散化**的一步映射，`ts` 由回调自己引入；本库不再做积分。
///   - 必须可重入、固定时间、不阻塞、不分配内存、不打日志；它会被 12 kHz 快环
///     逐拍调用，是否放进 ISR 要用 WCET 实测决定。
///   - 不得 panic（`no_std` 目标上没有可依赖的展开路径），也不得跨边界传递 Rust
///     引用、切片或拥有析构逻辑的类型；参数与返回值都是普通 `f32`。
///   - 雅可比必须与 `f` 的解析导数一致，本库不做数值校验；写错会直接表现为滤波
///     器发散或对量测过度自信。
///   - `extern "C"` 是为了与 C 参考实现/桥接层保持同一 ABI：没有闭包、没有捕获
///     环境、没有虚表，函数地址存放在参数结构体里。
///
///   Must be re-entrant, fixed-time, non-blocking, allocation-free and panic-free;
///   the Jacobian must be the analytic derivative of `f`.
pub type EkfStateFunction = extern "C" fn(x: f32, u: f32) -> f32;
/// EKF 量测回调类型：`h(x) -> f32`，用于量测函数 `h` 及其雅可比 `dh/dx`。
/// EKF measurement callback type, used for both `h` and its Jacobian `dh/dx`.
///
/// 契约与 `EkfStateFunction` 相同。注意量测函数 `h` 与其雅可比都在**先验状态**
/// `x_pred` 处求值，两者必须用同一状态点，否则线性化误差会被当成模型失配。
/// Same contract as `EkfStateFunction`; both `h` and `dh/dx` are evaluated at the
/// a-priori state, so they must use the same linearisation point.
pub type EkfMeasureFunction = extern "C" fn(x: f32) -> f32;

/// 一维扩展 Kalman 滤波参数。
/// Parameters of the one-dimensional extended Kalman filter.
///
/// 模型 / Model: `x[k+1] = f(x[k], u[k])`，`y[k] = h(x[k])`，其中 `f`、`h` 及其
/// 雅可比全部由回调提供。四个回调都是 `Option<extern "C" fn>`：未配置完整模型时
/// `update` 返回 `0.0` 并保持估计状态不变（见
/// `tests::ekf_rejects_missing_model_without_state_change`）。
/// All four callbacks are optional; if any of them is missing, `update` returns
/// `0.0` and leaves the state untouched.
///
/// 参数 / Parameters:
///   f      状态转移函数 `f(x, u)`
///   df_dx  状态雅可比 `∂f/∂x`，必须与 `f` 一致
///   h      量测函数 `h(x)`
///   dh_dx  量测雅可比 `∂h/∂x`
///   q      每步过程噪声方差 `[x²]`，必须 `>= 0`。因为 `f` 已经是离散步，`q` 就是
///          每步方差而不是谱密度：它不会随 `ts` 自动缩放，改变调用周期后必须重新
///          整定。
///          Per-step process-noise variance in `[x²]`, not a spectral density; it
///          does not scale with `ts` on its own.
///   r      量测噪声方差 `[y²]`，必须 `> 0`。
///          Measurement-noise variance in `[y²]`, must be `> 0`.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct EkfParam {
    pub f: Option<EkfStateFunction>,
    pub df_dx: Option<EkfStateFunction>,
    pub h: Option<EkfMeasureFunction>,
    pub dh_dx: Option<EkfMeasureFunction>,
    pub q: f32,
    pub r: f32,
}

/// 一维 EKF 状态。
/// State of the one-dimensional EKF.
///
/// 字段 / Fields:
///   x        状态估计 `[x]`（角度状态即 `[rad]`）。
///            State estimate in `[x]`.
///   p        估计误差协方差 `[x²]`，必须保持非负。EKF 的线性化只保证局部有效，
///            模型强非线性或 `q` 过小时 `p` 会塌缩，滤波器随即"听不进量测"。
///            Estimation-error covariance in `[x²]`, must stay non-negative.
///   k        最近一次的 Kalman 增益 `[x]/[y]`。
///            Last Kalman gain in `[x]/[y]`.
///   residual 最近一次的先验创新量 `measurement - h(x_pred)` `[y]`。它是判断模型
///            失配/量测异常最直接的量，但**没有做角度环绕**：角度状态跨过 `0/2π`
///            边界时它会出现接近 `±2π` 的尖峰，见 `update` 的说明。
///            Last a-priori innovation in `[y]`; it is not angle-wrapped.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EkfState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

/// 一维 EKF（`Observer_EKF`）。
/// The one-dimensional EKF (`Observer_EKF`).
impl EkfState {
    /// 构造 EKF 状态。
    /// Builds the EKF state.
    ///
    /// 参数 / Parameters:
    ///   initial_x 初始状态估计 `[x]`（角度状态即 `[rad]`，建议先用
    ///             `wrap_angle_0_to_2pi` 归一）
    ///   initial_p 初始误差协方差 `[x²]`，必须 `>= 0`
    ///
    /// 与 `KalmanState::new` 一样是 `const fn`：状态对象属于初始化阶段，不应在
    /// 中断里构造。
    /// Like `KalmanState::new` this is a `const fn`: the state belongs to
    /// initialisation, not to an interrupt.
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    /// 复位 EKF：设定状态与协方差初值，并清空增益与创新量。
    /// Resets the EKF to the given state and covariance, clearing the gain and the
    /// innovation.
    ///
    /// 模型回调不在这里、也不能在这里更换：`f`/`h` 保存在 `EkfParam` 中，复位只
    /// 影响数值状态。
    /// The model callbacks live in `EkfParam` and are not touched here.
    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    /// 推进一拍 EKF（先验预测 + 量测校正），返回后验估计。
    /// Runs one EKF step (predict then correct) and returns the a-posteriori
    /// estimate.
    ///
    /// 参数 / Parameters:
    ///   param       滤波参数与模型回调（见 `EkfParam`）
    ///   u           控制/输入量 `[u]`，原样传给 `f` 与 `df_dx`
    ///   measurement 量测值 `[y]`，原样用于计算创新量
    ///
    /// 返回 / Returns: 后验估计 `self.x` `[x]`；**任一回调缺失时返回 `0.0` 且估计
    ///   状态完全不变**。因此 `0.0` 是"模型没配好"的哨兵值，而不是有效估计：调用
    ///   方不能只看返回值就当作角度/速度结果使用。
    ///   The a-posteriori estimate, or `0.0` with the state left completely
    ///   unchanged when any required callback is missing. `0.0` is a sentinel, not a
    ///   valid estimate.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - 缺回调的检查发生在任何状态写入**之前**，所以 `x`、`p`、`k`、`residual`
    ///     全部保持原值（测试 `ekf_rejects_missing_model_without_state_change` 断言
    ///     了这一点）。
    ///   - 状态雅可比在**先验状态** `self.x` 处求值，量测函数与量测雅可比在
    ///     `x_pred` 处求值。因为 `f` 已经是离散映射，雅可比在 `x[k]` 处取值才是
    ///     正确的一阶线性化；把它改到 `x_pred` 处会改变滤波行为并让
    ///     `tests::ekf_matches_c_reference` 失配。
    ///   - 创新量 `measurement - h(x_pred)` **不做角度环绕**。对角度状态，量测跨过
    ///     `0/2π` 边界时创新量可接近 `±2π`，滤波器会用一个巨大的修正把状态拉过去；
    ///     工程上要么在 `h` 回调里返回角度差，要么在量测预处理里消环绕。
    ///   - `s`（创新方差）恰为 0 时被强制成 1.0，是 C 参考的除零保护；`r < 0` 或
    ///     `q < 0` 仍会让协方差失去方差含义，本库不做检查。
    ///   - 每拍调用 4 个回调（`f`、`df_dx`、`h`、`dh_dx`），而回调是外部函数、无法
    ///     被内联进本 crate：快环 WCET 主要由这些回调决定，必须实测。
    pub fn update(&mut self, param: &EkfParam, u: f32, measurement: f32) -> f32 {
        let (Some(f), Some(df_dx), Some(h), Some(dh_dx)) =
            (param.f, param.df_dx, param.h, param.dh_dx)
        else {
            return 0.0;
        };
        let x_pred = f(self.x, u);
        let f_jac = df_dx(self.x, u);
        let p_pred = f_jac * self.p * f_jac + param.q;
        let h_value = h(x_pred);
        let h_jac = dh_dx(x_pred);
        let mut s = h_jac * p_pred * h_jac + param.r;
        if s == 0.0 {
            s = 1.0;
        }
        self.k = p_pred * h_jac / s;
        self.residual = measurement - h_value;
        self.x = x_pred + self.k * self.residual;
        self.p = (1.0 - self.k * h_jac) * p_pred;
        self.x
    }
}

/// EKF + FOC 组合链参数。
/// Parameters of the combined EKF + FOC chain.
///
/// 参数 / Parameters:
///   ekf EKF 参数与模型回调（见 `EkfParam`）。它的状态就是电角度，所以 `q`/`r`
///       的量纲是 `[rad²]`。
///       EKF parameters; its scalar state is an electrical angle, so `q`/`r` are
///       in `[rad²]`.
///   foc 基础电流环参数（d/q PI 与母线电压，见 `FocBasicParam`：`v_bus` 单位 `[V]`）。
///       Basic current-loop parameters (d/q PI gains and the DC bus in `[V]`).
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct EkfFocParam {
    pub ekf: EkfParam,
    pub foc: FocBasicParam,
}

/// EKF + FOC 组合链的一拍输入。
/// One-step input of the combined EKF + FOC chain.
///
/// 字段 / Fields:
///   ekf_u                    EKF 的输入量 `[u]`，含义由 `f(x, u)` 回调定义。
///                            EKF input, its meaning is defined by `f(x, u)`.
///   theta_measurement_rad    量测到的电角度 `[rad]`，直接作为 EKF 的量测。
///                            注意它必须是**电**角度，且未做角度差环绕（见
///                            `EkfState::update`）。
///                            Measured electrical angle in `[rad]`.
///   foc                      基础 FOC 输入（三相电流 `[A]` 与 `id/iq` 给定 `[A]`）。
///                            其中 `theta_e_rad` 会被 EKF 的估计角度**覆盖**。
///                            Basic FOC input; its `theta_e_rad` is overridden by
///                            the EKF estimate.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EkfFocInput {
    pub ekf_u: f32,
    pub theta_measurement_rad: f32,
    pub foc: FocBasicInput,
}

/// EKF + FOC 组合链状态（`Observer_EKF_FOC`）。
/// State of the combined EKF + FOC chain (`Observer_EKF_FOC`).
///
/// 字段 / Fields:
///   ekf       EKF 子状态（角度状态 `[rad]` 与协方差 `[rad²]`）。
///             EKF sub-state.
///   foc       基础电流环子状态（PI 积分器与最近一次 d/q 电压 `[V]`）。
///             Basic current-loop sub-state.
///   theta_rad 本拍交给电流环使用的电角度 `[rad]`，范围 `[0, 2π)`；它是 EKF 估计
///             值归一化后的结果，也是诊断"角度源"的直接依据。
///             Electrical angle in `[rad]` handed to the current loop, inside
///             `[0, 2π)`.
///   pwm       本拍 SVPWM 输出：`duty_a/b/c` 为 `[0,1]` 的占空比（不是 per-mille），
///             由 C 平台层在复核故障与范围后写入比较寄存器。
///             SVPWM output of this step: duties in `[0,1]`, written to the compare
///             registers by the C platform layer after its own checks.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfFocState {
    pub ekf: EkfState,
    pub foc: FocBasicState,
    pub theta_rad: f32,
    pub pwm: SvpwmOutput,
}

/// 手写 `Default`：EKF 与 FOC 子状态取各自默认值，角度为 0，PWM 输出全 0。
/// Hand-written `Default`: default sub-states, zero angle and zero duties.
///
/// 占空比全 0 是"没有输出"的数值默认值，**不是**安全使能状态：是否允许写栅极由
/// C 平台层在故障检查之后决定（见 docs/C与Rust混合架构.md 的 ABI 规则）。
/// Zero duties are a numeric default, not an enable signal: the C platform layer
/// decides whether the gate driver may be written at all.
impl Default for EkfFocState {
    fn default() -> Self {
        Self {
            ekf: EkfState::default(),
            foc: FocBasicState::default(),
            theta_rad: 0.0,
            pwm: SvpwmOutput::default(),
        }
    }
}

/// EKF + FOC 组合链（`Observer_EKF_FOC`）。
/// The combined EKF + FOC chain (`Observer_EKF_FOC`).
impl EkfFocState {
    /// 构造组合链状态：给定角度与协方差初值并复位。
    /// Builds the chain with the given angle and covariance, then resets it.
    ///
    /// 参数 / Parameters:
    ///   initial_theta_rad 初始电角度 `[rad]`
    ///   initial_p         初始角度协方差 `[rad²]`
    pub fn new(initial_theta_rad: f32, initial_p: f32) -> Self {
        let mut state = Self::default();
        state.reset(initial_theta_rad, initial_p);
        state
    }

    /// 复位组合链：EKF（角度 `[rad]` 与协方差 `[rad²]`）、FOC 电流环、输出角度与
    /// PWM 输出一并复位。
    /// Resets the EKF (angle in `[rad]`, covariance in `[rad²]`), the current loop,
    /// the output angle and the PWM output.
    ///
    /// FOC 状态的复位会清掉 d/q PI 的积分器，因此复位同时是一次"输出归零"动作：
    /// 停机、故障恢复或切换控制模式时必须调用它，避免带着旧积分器重新使能。
    /// Resetting the FOC state clears the d/q PI integrators, so this doubles as an
    /// output-clearing action before re-enabling.
    pub fn reset(&mut self, theta_rad: f32, p: f32) {
        self.ekf.reset(theta_rad, p);
        self.foc.reset();
        self.theta_rad = wrap_angle_0_to_2pi(theta_rad);
        self.pwm = SvpwmOutput::default();
    }

    /// 推进一拍"EKF 估角 + 基础 FOC 电流环"，返回 SVPWM 占空比。
    /// Runs one step of "EKF angle estimation + basic FOC current loop" and returns
    /// the SVPWM duties.
    ///
    /// 参数 / Parameters:
    ///   param 组合链参数（EKF 模型回调 + FOC 参数）
    ///   input 本拍输入（量测角度、EKF 输入、三相电流与 d/q 给定）
    ///
    /// 返回 / Returns: 本拍 SVPWM 输出，`duty_a/b/c` 为 `[0,1]`（不是 per-mille）。
    ///   SVPWM output with duties in `[0,1]`.
    ///
    /// 行为与陷阱 / Behaviour and pitfalls:
    ///   - `input.foc.theta_e_rad` 被**忽略并覆盖**为 EKF 的估计角度：本链的语义
    ///     就是"由 EKF 提供角度"。调用方仍必须填写该字段（结构体要求），但填什么都
    ///     不影响结果——这一点容易误判为"角度没生效"。
    ///   - EKF 的返回值先经 `wrap_angle_0_to_2pi` 归一，再送入 Park/SVPWM，保证
    ///     电流环拿到的电角度始终在 `[0, 2π)`。
    ///   - 若 EKF 模型回调缺失，`ekf.update` 返回 `0.0`，本链会把角度固定到 `0 rad`
    ///     而不是报错或保持上一拍角度：上电自检必须确认回调已配置，否则电机会在
    ///     错误角度下励磁。
    ///   - 每拍执行 EKF 的 4 个回调加整条 FOC 电流环（Clarke/Park、双 PI、逆 Park、
    ///     SVPWM），是快环里最重的组合之一；是否放进 12 kHz ISR 必须实测 WCET，
    ///     并确认回调是可重入、固定时间的。
    ///   - 本链不做硬件保护：过流、欠压、过压、过温必须由独立保护路径与 C 平台层
    ///     处理，算法返回值不能替代关断。
    pub fn update(&mut self, param: &EkfFocParam, input: &EkfFocInput) -> SvpwmOutput {
        self.theta_rad = wrap_angle_0_to_2pi(self.ekf.update(
            &param.ekf,
            input.ekf_u,
            input.theta_measurement_rad,
        ));
        let mut foc_input = input.foc;
        foc_input.theta_e_rad = self.theta_rad;
        self.pwm = self.foc.update(&param.foc, &foc_input);
        self.pwm
    }
}

/// UKF 状态回调类型：`f(x, u) -> f32`。
/// UKF state callback type.
///
/// 契约与 `EkfStateFunction` 相同（可重入、固定时间、不阻塞、不分配、不 panic），
/// 区别在于 UKF **不需要雅可比**：非线性由 3 个 sigma 点直接传播。因此同一个
/// `extern "C" fn(f32, f32) -> f32` 实现既可以当 EKF 的状态函数，也可以当 UKF 的
/// 状态函数；两个类型别名签名完全相同，编译器不会阻止把它们混用，命名区分只是
/// 为了可读性与文档归属。
/// Same contract as `EkfStateFunction`, but no Jacobian is required; the two aliases
/// have identical signatures, so keeping them distinct is only a documentation aid.
pub type UkfStateFunction = extern "C" fn(x: f32, u: f32) -> f32;
/// UKF 量测回调类型：`h(x) -> f32`。契约与 `EkfMeasureFunction` 相同。
/// UKF measurement callback type; same contract as `EkfMeasureFunction`.
pub type UkfMeasureFunction = extern "C" fn(x: f32) -> f32;

/// 一维无迹 Kalman 滤波参数。
/// Parameters of the one-dimensional unscented Kalman filter.
///
/// 模型 / Model: `x[k+1] = f(x[k], u[k])`，`y[k] = h(x[k])`；`f`、`h` 为可空回调，
/// 缺任一必需回调时 `update` 返回 `0.0` 并保持状态不变。
/// Both callbacks are optional; a missing one makes `update` return `0.0` without
/// touching the state.
///
/// 参数 / Parameters:
///   f          状态转移回调 `f(x, u)`
///   h          量测回调 `h(x)`
///   q          每步过程噪声方差 `[x²]`，必须 `>= 0`；因为 `f` 已是离散步，`q` 是
///              每步方差，不随 `ts` 自动缩放。
///              Per-step process-noise variance in `[x²]`.
///   r          量测噪声方差 `[y²]`，必须 `> 0`；`r = 0` 时创新方差可能为 0，
///              `update` 会把它强制成 1.0，增益随即失去意义。
///              Measurement-noise variance in `[y²]`, must be `> 0`.
///   alpha      sigma 点相对均值的散布，无量纲。常用 `1e-3`～`1`。本库对
///              `alpha <= 0` 用 0.001 兜底；**取值来源未在仓库内标注**。注意
///              `alpha` 远小于 1 时第 0 个权重会变成很大的负数，均值与协方差靠
///              大数相减得到，在 `f32` 下会明显丢精度。
///              Sigma-point spread, dimensionless; the `<= 0` fallback 0.001 has no
///              identified origin in-tree.
///   beta       先验分布知识参数，无量纲；高斯假设下取 2 最优（会加大第 0 个协
///              方差权重）。
///              Prior-knowledge parameter, dimensionless; 2 is optimal for a
///              Gaussian.
///   kappa      次级散布参数，无量纲；一维状态常见取 0（或 `3 - n`，即 2）。
///              Secondary scaling parameter, dimensionless.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct UkfParam {
    pub f: Option<UkfStateFunction>,
    pub h: Option<UkfMeasureFunction>,
    pub q: f32,
    pub r: f32,
    pub alpha: f32,
    pub beta: f32,
    pub kappa: f32,
}

/// 一维 UKF 状态。
/// State of the one-dimensional UKF.
///
/// 字段 / Fields:
///   x        状态估计 `[x]`。sigma 点按 `±sqrt(scale·p)` 对称展开，`p` 为负时
///            `sqrtf` 返回 `NaN`，`x` 会被永久污染且无法自恢复。
///            State estimate in `[x]`; a negative `p` poisons it irrecoverably.
///   p        估计误差协方差 `[x²]`，**必须保持非负**。UKF 的第 0 个协方差权重在
///            `alpha < 1` 时为负，因此 `p` 不像线性 Kalman 那样天然非负，本库也不
///            做下限保护。
///            Estimation-error covariance in `[x²]`; it must stay non-negative, but
///            neither the UKF formulation nor this library guarantees that.
///   k        最近一次的 Kalman 增益 `[x]/[y]`。
///            Last Kalman gain in `[x]/[y]`.
///   residual 最近一次的先验创新量 `measurement - z_pred` `[y]`；同样不做角度环绕。
///            Last a-priori innovation in `[y]`, not angle-wrapped.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UkfState {
    pub x: f32,
    pub p: f32,
    pub k: f32,
    pub residual: f32,
}

/// 一维 UKF（`Observer_UKF`）。
/// The one-dimensional UKF (`Observer_UKF`).
impl UkfState {
    /// 构造 UKF 状态。
    /// Builds the UKF state.
    ///
    /// 参数 / Parameters:
    ///   initial_x 初始状态估计 `[x]`
    ///   initial_p 初始误差协方差 `[x²]`，必须 `> 0`：它决定 sigma 点的展开半径
    ///             `sqrt(scale·p)`，取 0 会让三个 sigma 点重合、模型非线性完全
    ///             无法被感知，取负值直接产生 `NaN`。
    ///             Initial covariance in `[x²]`; it must be `> 0` because it sets the
    ///             sigma-point spread.
    pub const fn new(initial_x: f32, initial_p: f32) -> Self {
        Self {
            x: initial_x,
            p: initial_p,
            k: 0.0,
            residual: 0.0,
        }
    }

    /// 复位 UKF：设定状态与协方差初值，并清空增益与创新量。
    /// Resets the UKF to the given state and covariance, clearing the gain and the
    /// innovation.
    ///
    /// 协方差初值同时是 sigma 点的展开半径，所以复位不只是"清历史"：它决定了
    /// 滤波器起步时对非线性的采样范围。
    /// The initial covariance also sets the sigma-point spread.
    pub fn reset(&mut self, x: f32, p: f32) {
        *self = Self::new(x, p);
    }

    /// 推进一拍一维 UKF，返回后验估计。
    /// Runs one UKF step and returns the a-posteriori estimate.
    ///
    /// 参数 / Parameters:
    ///   param       滤波参数与模型回调（见 `UkfParam`）
    ///   u           控制/输入量 `[u]`，原样传给 `f`
    ///   measurement 量测值 `[y]`
    ///
    /// 返回 / Returns: 后验估计 `self.x` `[x]`；任一必需回调缺失时返回 `0.0` 且状态
    ///   完全不变（与 `EkfState::update` 同一约定）。
    ///   The a-posteriori estimate, or `0.0` with the state untouched when a
    ///   required callback is missing.
    ///
    /// 实现说明 / Implementation notes:
    ///   - 只有 **3 个固定 sigma 点**：`[x, x + sqrt(scale·p), x - sqrt(scale·p)]`，
    ///     所以一维情况下不需要 Cholesky 分解，也不需要矩阵运算。
    ///   - 临时数据是三个 `[f32; 3]` 数组（`sigma`、`sigma_pred`、`z_sigma`），数组
    ///     数据合计 36 字节；此外还有若干局部标量。编译器可能把部分值放进寄存器、
    ///     内联调用或重排栈帧，因此 **36 字节不是最终最坏栈占用**，实机栈水位必须
    ///     用链接器 map/栈涂色实测（见 `算法库实时性说明.md` 的 UKF 临时数据一节）。
    ///     The three `[f32; 3]` arrays total 36 bytes of array data, which is not the
    ///     final worst-case stack figure.
    ///   - 每拍调用 `f` 三次、`h` 三次，共 6 次外部回调，是本模块里调用次数最多的
    ///     路径；快环 WCET 由这些回调决定。
    ///   - `alpha <= 0` 与 `scale <= 0` 用 0.001 兜底，`pz == 0` 用 1.0 兜底：这些都是
    ///     静默替换而非报错。参数非法时滤波器不会停机，只是悄悄换了一套散布/增益
    ///     语义，因此参数必须在上电自检里校验（`alpha > 0` 且 `alpha²(1+kappa) > 0`
    ///     且 `r > 0`）。
    ///     The `0.001` and `1.0` fallbacks silently retune the filter instead of
    ///     failing, so the parameters must be validated at start-up.
    ///   - `alpha < 1` 时 `lambda = alpha²(1+kappa) - 1 < 0`，第 0 个权重
    ///     `wm0 = lambda/scale` 是绝对值很大的负数（`alpha = 1e-3`、`kappa = 0` 时约
    ///     为 `-1e6`，而 `wi = 0.5/scale` 约为 `5e5`）：均值与协方差由量级相近的大数
    ///     相减得到，`f32` 下会明显丢精度；同时 `wc0` 也为负，协方差 `p` 可能变负，
    ///     下一次 `sqrtf(scale·p)` 直接返回 `NaN`，而本函数**没有任何 `NaN` 检查**，
    ///     估计会永久失效。这是本模块需要专门测试的边界（见 `算法库实时性说明.md`
    ///     第 5 条）。小 `alpha` 的整定必须先在目标精度下验证数值行为。
    ///     With `alpha < 1` the zeroth weights are large in magnitude and negative, so
    ///     the mean and covariance come from subtracting nearby large numbers; `p` can
    ///     turn negative and `sqrtf` then poisons the state with `NaN`.
    ///   - `q`/`r` 是每步方差 `[x²]`/`[y²]`，不是谱密度；`f` 已经是离散映射。
    ///   - 运算顺序与 C 参考实现一致，改动会让 `tests::ukf_matches_c_reference`
    ///     失配。
    pub fn update(&mut self, param: &UkfParam, u: f32, measurement: f32) -> f32 {
        let (Some(f), Some(h)) = (param.f, param.h) else {
            return 0.0;
        };
        let alpha = if param.alpha <= 0.0 {
            0.001
        } else {
            param.alpha
        };
        let lambda = alpha * alpha * (1.0 + param.kappa) - 1.0;
        let mut scale = 1.0 + lambda;
        if scale <= 0.0 {
            scale = 0.001;
        }
        let sqrt_scale_p = libm::sqrtf(scale * self.p);
        let sigma = [self.x, self.x + sqrt_scale_p, self.x - sqrt_scale_p];
        let sigma_pred = [f(sigma[0], u), f(sigma[1], u), f(sigma[2], u)];
        let wm0 = lambda / scale;
        let wc0 = wm0 + (1.0 - alpha * alpha + param.beta);
        let wi = 0.5 / scale;
        let x_pred = wm0 * sigma_pred[0] + wi * sigma_pred[1] + wi * sigma_pred[2];
        let mut p_pred = param.q;
        p_pred += wc0 * (sigma_pred[0] - x_pred) * (sigma_pred[0] - x_pred);
        p_pred += wi * (sigma_pred[1] - x_pred) * (sigma_pred[1] - x_pred);
        p_pred += wi * (sigma_pred[2] - x_pred) * (sigma_pred[2] - x_pred);
        let z_sigma = [h(sigma_pred[0]), h(sigma_pred[1]), h(sigma_pred[2])];
        let z_pred = wm0 * z_sigma[0] + wi * z_sigma[1] + wi * z_sigma[2];
        let mut pz = param.r;
        pz += wc0 * (z_sigma[0] - z_pred) * (z_sigma[0] - z_pred);
        pz += wi * (z_sigma[1] - z_pred) * (z_sigma[1] - z_pred);
        pz += wi * (z_sigma[2] - z_pred) * (z_sigma[2] - z_pred);
        let mut pxz = wc0 * (sigma_pred[0] - x_pred) * (z_sigma[0] - z_pred);
        pxz += wi * (sigma_pred[1] - x_pred) * (z_sigma[1] - z_pred);
        pxz += wi * (sigma_pred[2] - x_pred) * (z_sigma[2] - z_pred);
        if pz == 0.0 {
            pz = 1.0;
        }
        self.k = pxz / pz;
        self.residual = measurement - z_pred;
        self.x = x_pred + self.k * self.residual;
        self.p = p_pred - self.k * pz * self.k;
        self.x
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller::PiParam;
    use crate::modulation::SvpwmParam;

    /// 浮点近似比较辅助：容差只为吸收 `f32` 舍入差异，不掩盖模型误差。
    /// Approximate float comparison helper; the tolerance only absorbs `f32`
    /// rounding.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 测试用状态模型 `f(x, u) = x + u`（`extern "C"`，无捕获、无分配），同时用于
    /// EKF 与 UKF 的状态函数。
    /// Test state model `f(x, u) = x + u`, used as both the EKF and the UKF state
    /// function.
    extern "C" fn add_input(x: f32, u: f32) -> f32 {
        x + u
    }

    /// `add_input` 的解析雅可比 `∂f/∂x = 1`。常量雅可比正是 C 参考向量能给出
    /// 闭式期望值的原因。
    /// Analytic Jacobian of `add_input`; the constant Jacobian is why the C reference
    /// vector has a closed-form expectation.
    extern "C" fn state_jacobian(_x: f32, _u: f32) -> f32 {
        1.0
    }

    /// 测试用非线性量测 `h(x) = x²`，用来检验 EKF 的线性化路径。
    /// Test non-linear measurement `h(x) = x²`.
    extern "C" fn square_measurement(x: f32) -> f32 {
        x * x
    }

    /// `square_measurement` 的解析雅可比 `∂h/∂x = 2x`。
    /// Analytic Jacobian of `square_measurement`.
    extern "C" fn square_jacobian(x: f32) -> f32 {
        2.0 * x
    }

    /// 测试用恒等量测 `h(x) = x`，让 EKF/UKF 的期望值可以手算验证。
    /// Test identity measurement `h(x) = x`.
    extern "C" fn identity_measurement(x: f32) -> f32 {
        x
    }

    /// `identity_measurement` 的解析雅可比，恒为 1。
    /// Analytic Jacobian of `identity_measurement`, constant 1.
    extern "C" fn identity_jacobian(_x: f32) -> f32 {
        1.0
    }

    /// C 参考向量：一维 Kalman 的预测/校正两步与增益、协方差数值。
    /// C reference vector for the 1-D Kalman predict/correct step, the gain and the
    /// covariance.
    ///
    /// 断言的是与 C 参考实现逐值一致；改动运算顺序、除零保护（`s == 0 -> 1.0`）
    /// 或协方差更新式 `(1 - k·h)·p_pred` 都会让它失配。
    /// Numerical equivalence with the C reference: changing the operation order or
    /// the covariance update breaks it.
    #[test]
    fn kalman_matches_c_reference() {
        let param = KalmanParam {
            a: 1.0,
            b: 1.0,
            h: 1.0,
            q: 1.0,
            r: 1.0,
        };
        let mut state = KalmanState::new(0.0, 1.0);
        near(state.update(&param, 1.0, 2.0), 1.666_666_7, 1e-5);
        near(state.k, 0.666_666_7, 1e-5);
        near(state.p, 0.666_666_7, 1e-5);
    }

    /// C 参考向量：二阶 Luenberger 的同步更新顺序与输出估计。
    /// C reference vector for the second-order Luenberger observer: the simultaneous
    /// update order and the output estimate.
    ///
    /// 期望值对"先把两个新状态算完再写回"这一顺序敏感；改成先写 `x0` 再算 `x1`
    /// 会得到不同结果并让本测试失配。
    /// The expectation is sensitive to the simultaneous-update order.
    #[test]
    fn luenberger_matches_c_reference() {
        let param = LuenbergerParam {
            a00: 1.0,
            a01: 0.1,
            a10: 0.0,
            a11: 1.0,
            b0: 0.0,
            b1: 0.1,
            c0: 1.0,
            c1: 0.0,
            l0: 0.5,
            l1: 0.2,
        };
        let mut state = LuenbergerState::default();
        near(state.update(&param, 1.0, 2.0), 1.0, 1e-6);
        near(state.x0, 1.0, 1e-6);
        near(state.x1, 0.5, 1e-6);
    }

    /// C 参考向量：带非线性量测 `h(x) = x²` 的一维 EKF 后验估计、增益与创新量。
    /// C reference vector for the 1-D EKF with the non-linear measurement
    /// `h(x) = x²`: estimate, gain and innovation.
    ///
    /// 该向量同时固定了"状态雅可比在先验 `x` 处求值、量测雅可比在 `x_pred` 处求值"
    /// 这一线性化点选择；把任一处换到另一个状态点都会失配。
    /// The vector also pins down the linearisation points.
    #[test]
    fn ekf_matches_c_reference() {
        let param = EkfParam {
            f: Some(add_input),
            df_dx: Some(state_jacobian),
            h: Some(square_measurement),
            dh_dx: Some(square_jacobian),
            q: 0.1,
            r: 0.5,
        };
        let mut state = EkfState::new(1.0, 1.0);
        near(state.update(&param, 1.0, 5.0), 2.243_094, 1e-5);
        near(state.k, 0.243_093_9, 1e-5);
        near(state.residual, 1.0, 1e-6);
    }

    /// 缺模型回调时的失败语义：`update` 返回 `0.0`，且 `x`、`p`、`k`、`residual`
    /// 一字不变。
    /// Failure semantics with missing model callbacks: `update` returns `0.0` and no
    /// field changes.
    ///
    /// 这是上层"回调没配好"与"估计真的是 0"之间唯一的区分手段：返回值本身不足以
    /// 判断，必须另外检查 `EkfParam` 的四个 `Option` 是否都已配置。
    /// The only way to tell a missing model from a genuine 0 estimate is to check the
    /// four `Option` fields.
    #[test]
    fn ekf_rejects_missing_model_without_state_change() {
        let mut state = EkfState::new(2.0, 3.0);
        assert_eq!(state.update(&EkfParam::default(), 1.0, 4.0), 0.0);
        assert_eq!(state, EkfState::new(2.0, 3.0));
    }

    /// C 参考向量：EKF 估角 + 基础 FOC 电流环的一拍输出。
    /// C reference vector for one step of the EKF + basic FOC chain.
    ///
    /// 验证三件事：EKF 把量测 `0.5 rad` 收敛到 `0.49～0.51`、电流环在该角度下把
    /// `iq` 误差送到电压（`kp = 1`、`ki = 0`、未饱和时 `vq = iq_ref - iq = 2.0`）、
    /// 以及 duty 全部落在 `[0,1]`。它证明的是数值链路与 C 一致，不证明实机闭环稳定。
    /// Validates the numeric chain only, not real-machine stability.
    #[test]
    fn ekf_foc_matches_c_reference() {
        let pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        let param = EkfFocParam {
            ekf: EkfParam {
                f: Some(add_input),
                df_dx: Some(state_jacobian),
                h: Some(identity_measurement),
                dh_dx: Some(identity_jacobian),
                q: 0.0,
                r: 0.01,
            },
            foc: FocBasicParam {
                id_pi: pi,
                iq_pi: pi,
                svpwm: SvpwmParam { v_bus: 24.0 },
            },
        };
        let input = EkfFocInput {
            ekf_u: 0.0,
            theta_measurement_rad: 0.5,
            foc: FocBasicInput {
                ia: 0.0,
                ib: 0.0,
                ic: 0.0,
                id_ref: 0.0,
                iq_ref: 2.0,
                theta_e_rad: 0.0,
            },
        };
        let mut state = EkfFocState::new(0.0, 1.0);
        let output = state.update(&param, &input);
        assert!((0.49..0.51).contains(&state.theta_rad));
        near(state.foc.voltage_dq.q, 2.0, 1e-6);
        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
    }

    /// C 参考向量：`alpha = 1`、`kappa = 0` 时三个 sigma 点的权重、均值与协方差。
    /// C reference vector for the UKF weights, mean and covariance with `alpha = 1`
    /// and `kappa = 0`.
    ///
    /// 该组参数下 `lambda = 0`、第 0 个权重为 0，均值退化成两个外侧 sigma 点的
    /// 等权平均，期望值因此可以手算核对；换成 `alpha << 1` 的常用整定后权重会变成
    /// 大数相减，测试无法固定，所以本向量**不覆盖**那种数值脆弱工况。
    /// This vector does not cover the numerically fragile `alpha << 1` tuning.
    #[test]
    fn ukf_matches_c_reference() {
        let param = UkfParam {
            f: Some(add_input),
            h: Some(identity_measurement),
            q: 1.0,
            r: 1.0,
            alpha: 1.0,
            beta: 2.0,
            kappa: 0.0,
        };
        let mut state = UkfState::new(0.0, 1.0);
        near(state.update(&param, 1.0, 2.0), 1.5, 1e-5);
        near(state.k, 0.5, 1e-5);
        near(state.p, 1.5, 1e-5);
    }

    /// 状态对象尺寸契约：三个估计器状态都必须是 16 字节。
    /// State-size contract: all three estimator states must remain 16 bytes.
    ///
    /// 尺寸与 `算法库实时性说明.md` 的表格一起对外承诺：加减字段会让这里和文档同时
    /// 失配，也会影响 `#[repr(C)]` 布局在 C 侧静态断言下的对应关系。
    /// These sizes are published in `算法库实时性说明.md`; changing fields breaks both
    /// the table and the `#[repr(C)]` layout expectations.
    #[test]
    fn estimator_state_sizes_are_fixed() {
        assert_eq!(core::mem::size_of::<KalmanState>(), 16);
        assert_eq!(core::mem::size_of::<EkfState>(), 16);
        assert_eq!(core::mem::size_of::<UkfState>(), 16);
    }
}
