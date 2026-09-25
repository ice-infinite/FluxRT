//! Filter_LowPass 的等价实现。
//! Equivalent implementation of `Filter_LowPass` plus the digital filters.
//!
//! 职责 / Responsibility:
//!   - `LowPassState`：一阶低通，平滑电流、速度、磁链等估计量
//!   - `HighPassState`：一阶高通（隔直），抑制测量零漂和偏置
//!   - `DifferentiatorState`：带一阶滤波的微分器，由位置/速度求变化率
//!   - first-order low-pass, first-order high-pass (DC blocker) and a filtered
//!     differentiator
//!
//! 架构位置 / Architecture position:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI)
//!     -> foc-control -> foc-algorithm (本文件 / this file)
//! 依赖方向 / Dependency direction: 只用到 crate 内的 `math::clamp`，
//! 不依赖 HAL、RTOS、定时器、堆或日志；本文件是纯 `no_std` 数学层。
//! Depends only on `math::clamp` inside this crate; no HAL, RTOS, timer, heap or
//! logging. This is the pure `no_std` math layer.
//!
//! 实时约束 / Real-time constraints:
//!   三个 `update` 都是固定次数的 `f32` 乘加，无分配、无阻塞、无 Mutex 等待，
//!   可以放进 12 kHz ADC 中断；`DifferentiatorState` 另外含一次除法。
//!   Every `update` is a fixed number of `f32` multiply-adds with no allocation,
//!   blocking or mutex wait, so the 12 kHz ADC ISR may call them.
//!   `DifferentiatorState` additionally performs one division.
//!   注意 / Note: crate 内的观测器（`observer/bemf.rs`、`observer/smo.rs` 等）
//!   各自内联了等价的一阶滤波，没有复用本文件的类型，所以当前除本文件的测试外
//!   没有其它调用点；这三个类型用于与 C 版行为逐项对照。
//!   The in-crate observers inline their own equivalent first-order filter instead
//!   of reusing these types, so today only this file's tests call them. They exist
//!   to stay behaviour-comparable with the C `Filter_LowPass`/`Filter_Digital`.
//!
//! 常数来源 / Constant provenance:
//!   本文件不硬编码任何 `alpha` 或 `ts`，全部由 `*Param` 传入，因此这里没有需要
//!   标 `[HW]`/`[ST]` 的常数；本工程 `[FW]` 侧使用的滤波系数在 foc-control 配置。
//!   No `alpha` or `ts` is hard-coded here; all arrive through `*Param`, so this
//!   file has no `[HW]`/`[ST]` constant to tag.
//!
//! 定点说明 / Fixed point:
//!   状态与系数全部为 `f32`，不存在 Q1.15/Q1.31 定点站点。
//!   All state and coefficients are `f32`; no Q1.15/Q1.31 site here.

use crate::math::clamp;

/// 一阶低通的参数。
/// Parameters of the first-order low-pass filter.
///
/// 差分方程为 `y[n] = y[n-1] + alpha * (x[n] - y[n-1])`，即后向欧拉一阶 IIR；
/// `alpha` 是每一步的平滑系数，不是截止频率。
/// Difference equation: `y[n] = y[n-1] + alpha * (x[n] - y[n-1])`, a first-order
/// backward-Euler IIR. `alpha` is the per-sample smoothing factor, not a cutoff
/// frequency.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowPassParam {
    /// 平滑系数，无量纲，逐次钳位到 `[0,1]`。0 表示输出冻结（最慢），
    /// 1 表示直通（不滤波）。
    /// Smoothing coefficient, dimensionless, clamped to `[0,1]`. 0 freezes the
    /// output (slowest) and 1 passes the input through unfiltered.
    ///
    /// 与截止频率的关系：`alpha ~= 1 - exp(-2*pi*fc*ts) ~= 2*pi*fc*ts`，
    /// 第二个近似只在 `fc << 1/ts` 时成立。本工程电流环 `ts = 1/12000` `[s]`，
    /// 例如 `fc = 500` `[Hz]` 时 `alpha` 约 0.23（线性近似给 0.26）。
    /// Relation to the cutoff: `alpha ~= 1 - exp(-2*pi*fc*ts)`, with the linear
    /// form `2*pi*fc*ts` valid only for `fc << 1/ts`. With the 12 kHz current loop
    /// (`ts = 1/12000` `[s]`) a 500 `[Hz]` cutoff gives `alpha` about 0.23, or
    /// 0.26 from the linear approximation.
    pub alpha: f32,
}

/// 一阶低通的运行状态。
/// Runtime state of the first-order low-pass filter.
///
/// 只有 `output` 一个记忆量，所以这是一阶（一状态）滤波器；`initialized`
/// 与本 crate 其它模块一致，用 `i32` 而不是 `bool`。
/// A single memory element, hence a first-order filter. `initialized` is an
/// `i32` rather than a `bool`, consistent with the other modules in this crate.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowPassState {
    /// 上一步滤波输出，量纲与输入相同。
    /// Previous filter output, in the same unit as the input.
    pub output: f32,
    /// 0 表示尚未注入初值，下一次 `update` 会直接采纳输入。
    /// 0 means no seed value yet; the next `update` adopts the input directly.
    pub initialized: i32,
}

impl LowPassState {
    /// 用给定初值构造，并把 `initialized` 置 1。
    /// Builds the state with a given initial output and sets `initialized`.
    ///
    /// 给初值可以避免上电后滤波器从 0 缓慢爬升到实际工作点；对电流/磁链这类
    /// 直接进入控制律的量，爬升过程会被误当成真实动态。
    /// Seeding avoids a slow ramp from zero to the real operating point at power
    /// up; for quantities that feed the control law directly (current, flux) that
    /// ramp would be mistaken for real dynamics.
    #[inline]
    pub const fn new(initial_output: f32) -> Self {
        Self {
            output: initial_output,
            initialized: 1,
        }
    }

    /// 把输出设为给定值并标记为已初始化。
    /// Sets the output to the given value and marks the filter as initialized.
    ///
    /// 这是无扰复位：它直接把滤波器放到工作点上，而不是清到 0。
    /// This is a bumpless reset: it places the filter on an operating point
    /// instead of clearing it to zero.
    #[inline]
    pub fn reset(&mut self, output: f32) {
        self.output = output;
        self.initialized = 1;
    }

    /// 执行一次一阶低通，返回新的滤波值。
    /// Runs one first-order low-pass step and returns the new output.
    ///
    /// 参数 / Parameters:
    ///   input - 原始采样，量纲与 `output` 相同
    ///           raw sample, same unit as `output`
    ///
    /// 返回 / Returns: 滤波后的值，量纲与输入相同。首次调用（`initialized == 0`）
    /// 直接返回输入本身，不产生从 0 开始的爬升。
    /// The filtered value. On the first call (`initialized == 0`) it returns the
    /// input itself, so there is no ramp from zero.
    ///
    /// 与 C 版等价 / C reference equivalence: `matches_c_reference_vectors`
    /// 固定了低通的差分形式和 `alpha` 的钳位行为（`alpha = 2.0` 被钳到 1.0，
    /// 输出等于输入），改动即破坏该参考向量。
    /// `matches_c_reference_vectors` pins this difference form and the `alpha`
    /// clamp for the low-pass; changing either breaks the vector.
    ///
    /// 上下文 / Context: 无分配、固定执行时间，可在 12 kHz ADC ISR 内调用。
    /// No allocation, fixed cost: safe inside the 12 kHz ADC ISR.
    #[inline]
    pub fn update(&mut self, param: &LowPassParam, input: f32) -> f32 {
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        // alpha 逐次钳位到 [0,1]：小于 0 会让极点跑到单位圆外（发散），
        // 大于 1 会让输出反向过冲并振荡。
        // `alpha` is clamped to `[0,1]`: below 0 the pole leaves the unit circle
        // and the filter diverges, above 1 the output overshoots and rings.
        let alpha = clamp(param.alpha, 0.0, 1.0);
        self.output += alpha * (input - self.output);
        self.output
    }
}

/// 一阶高通（隔直）滤波器的参数。
/// Parameters of the first-order high-pass (DC blocker) filter.
///
/// 差分方程为 `y[n] = alpha * (y[n-1] + x[n] - x[n-1])`，传递函数
/// `H(z) = alpha * (1 - z^-1) / (1 - alpha * z^-1)`：直流增益恒为 0，
/// 用于去掉测量里的固定偏置或缓慢零漂。
/// Difference equation `y[n] = alpha * (y[n-1] + x[n] - x[n-1])`, i.e.
/// `H(z) = alpha * (1 - z^-1) / (1 - alpha * z^-1)`. The DC gain is exactly
/// zero, so it removes a constant offset or slow drift.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighPassParam {
    /// 极点系数，无量纲，逐次钳位到 `[0,1]`。越接近 1 通带越完整、但直流
    /// 收敛越慢；越小则连有用的低频成分也一起被压掉。
    /// Pole coefficient, dimensionless, clamped to `[0,1]`. The closer to 1, the
    /// more of the passband survives but the slower the DC settles; smaller
    /// values also attenuate wanted low-frequency content.
    pub alpha: f32,
}

/// 一阶高通滤波器的运行状态。
/// Runtime state of the first-order high-pass filter.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HighPassState {
    /// 上一步输入，量纲与输入相同；用于构造 `x[n] - x[n-1]`。
    /// Previous input, same unit as the input; it forms `x[n] - x[n-1]`.
    pub last_input: f32,
    /// 上一步滤波输出，量纲与输入相同。
    /// Previous filter output, same unit as the input.
    pub output: f32,
    /// 0 表示尚未注入初值；下一次 `update` 只同步历史并返回 0。
    /// 0 means no seed value yet: the next `update` only syncs the history and
    /// returns 0.
    pub initialized: i32,
}

impl HighPassState {
    /// 用给定初值构造：历史输入取 `initial_input`，输出为 0。
    /// Builds the state with `last_input = initial_input` and `output = 0`.
    ///
    /// 输出从 0 开始是有意的：高通在稳态下本来就应输出 0，这样上电时不会把
    /// 一个直流偏置当成有用信号送进控制律。
    /// Starting from 0 is deliberate: a high-pass is supposed to output 0 in
    /// steady state, so a DC offset is not mistaken for signal at power up.
    pub const fn new(initial_input: f32) -> Self {
        Self {
            last_input: initial_input,
            output: 0.0,
            initialized: 1,
        }
    }

    /// 用给定输入重新植入历史并把输出清零。
    /// Re-seeds the input history from the given value and clears the output.
    ///
    /// 复位后第一次 `update` 的 `x[n] - x[n-1]` 为 0，因此不会产生复位冲击。
    /// After a reset the first `update` sees `x[n] - x[n-1] == 0`, so no reset
    /// transient is produced.
    pub fn reset(&mut self, input: f32) {
        *self = Self::new(input);
    }

    /// 执行一次一阶高通，返回新的滤波值。
    /// Runs one high-pass step and returns the new output.
    ///
    /// 参数 / Parameters:
    ///   input - 原始采样，量纲与输出相同
    ///           raw sample, same unit as the output
    ///
    /// 返回 / Returns: 高通后的值；首次调用（`initialized == 0`）返回 0.0。
    /// The high-passed value; the first call returns 0.0.
    ///
    /// 与 C 版等价 / C reference equivalence: `digital_filters_match_c_reference`
    /// 用阶跃输入锁定高通的前两步（0.5、0.25），固定了差分形式和状态更新顺序。
    /// `digital_filters_match_c_reference` pins the first two high-pass outputs for
    /// a step input, i.e. the difference form and the state-update order.
    ///
    /// 上下文 / Context: 无分配、固定执行时间，可在 12 kHz ADC ISR 内调用。
    /// No allocation, fixed cost: safe inside the 12 kHz ADC ISR.
    pub fn update(&mut self, param: &HighPassParam, input: f32) -> f32 {
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        // 与低通同样的理由：alpha 必须在 [0,1] 内，否则极点失稳或输出振荡。
        // Same reasoning as the low-pass: `alpha` must stay inside `[0,1]` or the
        // pole becomes unstable or the output rings.
        let alpha = clamp(param.alpha, 0.0, 1.0);
        self.output = alpha * (self.output + input - self.last_input);
        self.last_input = input;
        self.output
    }
}

/// 带一阶滤波的微分器参数。
/// Parameters of the filtered differentiator.
///
/// 先做差分 `(x[n] - x[n-1]) / ts`，再过一阶低通，因此等效于“微分 + 低通”。
/// 单独的差分会把量化噪声按 `1/ts` 放大，滤波是必须的。
/// The raw difference `(x[n] - x[n-1]) / ts` is followed by a first-order
/// low-pass. Plain differencing amplifies quantisation noise by `1/ts`, so the
/// smoothing is not optional.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DifferentiatorParam {
    /// 采样周期 `[s]`，用于把输入差分换算成变化率（量纲 输入/s）。
    /// `ts <= 0.0` 时 `update` 返回 0.0 且不改状态，避免除零得到 `Inf`。
    /// Sample period `[s]` that converts the input difference into a rate. When
    /// `ts <= 0.0` the update returns 0.0 without touching state, so it cannot
    /// divide by zero.
    pub ts: f32,
    /// 微分低通系数，无量纲，逐次钳位到 `[0,1]`。1.0 表示不做平滑，噪声会
    /// 全部进入输出，设成 1.0 前必须先确认编码器/测量分辨率。
    /// Derivative low-pass coefficient, clamped to `[0,1]`. 1.0 disables the
    /// smoothing and passes all noise; verify the measurement resolution first.
    pub alpha: f32,
}

/// 带一阶滤波的微分器的运行状态。
/// Runtime state of the filtered differentiator.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DifferentiatorState {
    /// 上一步输入，与输入同量纲。
    /// Previous input, in the input unit.
    pub last_input: f32,
    /// 上一步微分输出（已滤波），量纲为 输入/s。
    /// Previous filtered derivative output, in input units per second.
    pub output: f32,
    /// 0 表示尚未注入初值；下一次 `update` 只同步历史并返回 0。
    /// 0 means no seed value yet: the next `update` only syncs the history and
    /// returns 0.
    pub initialized: i32,
}

impl DifferentiatorState {
    /// 用给定初值构造：历史输入取 `initial_input`，微分输出为 0。
    /// Builds the state with `last_input = initial_input` and `output = 0`.
    ///
    /// 初值必须来自实际闭环前的采样点，否则第一次更新会算出一个虚假的巨大
    /// 变化率。
    /// The seed must come from a real pre-closed-loop sample, otherwise the first
    /// update would report a fictitious rate.
    pub const fn new(initial_input: f32) -> Self {
        Self {
            last_input: initial_input,
            output: 0.0,
            initialized: 1,
        }
    }

    /// 用给定输入重新植入历史并把微分输出清零。
    /// Re-seeds the input history from the given value and clears the output.
    ///
    /// 复位后第一次 `update` 的差分为 0，所以不会产生微分冲击。
    /// After a reset the first `update` sees a zero difference, so there is no
    /// derivative kick.
    pub fn reset(&mut self, input: f32) {
        *self = Self::new(input);
    }

    /// 执行一次滤波微分，返回新的变化率。
    /// Runs one filtered differentiation step and returns the new rate.
    ///
    /// 参数 / Parameters:
    ///   input - 原始采样（通常是位置 `[rad]` 或速度 `[rad/s]`）
    ///           raw sample, typically a position `[rad]` or a speed `[rad/s]`
    ///
    /// 返回 / Returns: 变化率，量纲为 输入单位/s。`param.ts <= 0.0` 时返回 0.0
    /// 且不改状态；首次调用（`initialized == 0`）也返回 0.0。
    /// The rate in input units per second. It returns 0.0 and leaves the state
    /// untouched when `param.ts <= 0.0`, and 0.0 on the first call too.
    ///
    /// 与 C 版等价 / C reference equivalence: `digital_filters_match_c_reference`
    /// 锁定了差分、滤波和状态更新的顺序；把 `last_input` 的赋值提前会让该参考
    /// 向量失败。
    /// `digital_filters_match_c_reference` pins the order of the difference, the
    /// filter and the history update; moving the `last_input` assignment earlier
    /// breaks that vector.
    ///
    /// 上下文 / Context: 无分配、固定执行时间（含一次除法），可在 12 kHz ADC
    /// 中断内调用；在 `thumbv7em-none-eabi` 上这次除法是一次软件浮点调用。
    /// No allocation, fixed cost including one division: safe in the 12 kHz ADC
    /// ISR. On `thumbv7em-none-eabi` that division is a soft-float call.
    pub fn update(&mut self, param: &DifferentiatorParam, input: f32) -> f32 {
        // ts 非法时直接返回 0，避免除零得到 Inf；状态保留上一次的值，
        // 调用方不应把返回的 0.0 理解成“微分器已被清零”。
        // Guards against division by zero. The state keeps its previous values,
        // so a returned 0.0 does not mean the differentiator was cleared.
        if param.ts <= 0.0 {
            return 0.0;
        }
        if self.initialized == 0 {
            self.reset(input);
            return self.output;
        }
        // 与低通/高通一致，alpha 必须在 [0,1] 内。
        // As with the low-pass and high-pass, `alpha` must stay inside `[0,1]`.
        let alpha = clamp(param.alpha, 0.0, 1.0);
        let raw = (input - self.last_input) / param.ts;
        self.output += alpha * (raw - self.output);
        self.last_input = input;
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 锁定 C 版 `Filter_LowPass` 的参考向量：`alpha=0.25` 时两步为 0.25、
    /// 0.4375；随后把 `alpha` 设成 2.0 以验证系数被钳到 1.0（输出直接等于输入），
    /// 即非法系数不会让滤波器发散。
    /// Pins the C `Filter_LowPass` vector, including the `alpha` clamp that keeps an
    /// out-of-range coefficient from destabilising the filter.
    #[test]
    fn matches_c_reference_vectors() {
        let mut p = LowPassParam { alpha: 0.25 };
        let mut s = LowPassState::new(0.0);
        assert!((s.update(&p, 1.0) - 0.25).abs() <= 1e-6);
        assert!((s.update(&p, 1.0) - 0.4375).abs() <= 1e-6);
        p.alpha = 2.0;
        assert!((s.update(&p, 1.0) - 1.0).abs() <= 1e-6);
    }

    /// `Default`（即 `initialized == 0`）状态下第一次更新必须原样返回输入，
    /// 保证上电后滤波器不产生从 0 爬升的虚假暂态。
    /// A default-constructed filter must adopt the first sample verbatim, so no
    /// fictitious ramp from zero appears at power up.
    #[test]
    fn default_state_initializes_from_first_sample() {
        let mut s = LowPassState::default();
        let p = LowPassParam { alpha: 0.1 };
        assert_eq!(s.update(&p, 3.0), 3.0);
    }

    /// 锁定 C 版 `Filter_Digital` 的参考向量：高通在阶跃下给出 0.5、0.25；
    /// 微分器 `ts=0.1`、`alpha=0.5` 时先给出 `10 * alpha = 5.0`，再乘 `alpha`
    /// 得 2.5。两个向量一起固定了本文件的差分形式和状态更新顺序。
    /// Pins the C `Filter_Digital` vectors for the high-pass and the
    /// differentiator, fixing both difference forms and the update order.
    #[test]
    fn digital_filters_match_c_reference() {
        let hp_param = HighPassParam { alpha: 0.5 };
        let mut hp = HighPassState::new(0.0);
        assert!((hp.update(&hp_param, 1.0) - 0.5).abs() <= 1e-6);
        assert!((hp.update(&hp_param, 1.0) - 0.25).abs() <= 1e-6);

        let diff_param = DifferentiatorParam {
            ts: 0.1,
            alpha: 0.5,
        };
        let mut diff = DifferentiatorState::new(0.0);
        assert!((diff.update(&diff_param, 1.0) - 5.0).abs() <= 1e-6);
        assert!((diff.update(&diff_param, 1.0) - 2.5).abs() <= 1e-6);
    }
}
