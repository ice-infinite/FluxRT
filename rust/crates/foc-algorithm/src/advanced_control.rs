//! 自适应、反步、模糊、H∞、LQR、MPC、MRAC 与滑模控制器。
//!
//! 职责 / Responsibility:
//!   - 八个基础标量/小模型高级控制器：自适应增益、二阶反步、3x3 模糊规则表、
//!     标量 H∞ 鲁棒状态反馈加扰动补偿、二状态固定增益 LQR、一阶离散模型 MPC、
//!     参考模型误差驱动的 MRAC、滑模面加边界层饱和的 SMC。
//!   - 全部是纯浮点数学：不访问硬件、不分配内存、不持有全局可变状态。
//!
//! 架构位置 / Position in the architecture:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!   -> foc-algorithm（本文件：芯片无关的纯 `no_std` 数学层）。
//!
//! 依赖方向 / Dependency direction:
//!   - 只依赖 `crate::math::clamp`；不依赖 HAL、RTOS、日志或其他控制模块。
//!   - 调用方负责单位换算、电角度与机械角度的区分、安全包络和复位时机。
//!   - 本文件只使用 `f32`，不涉及定点：既没有 `i16` Q1.15，也没有 `i32` Q1.31
//!     的隐式换算。生产代码里除了数学归一化常数（±1）和 `max(candidates, 2)`
//!     之外没有魔数，因此没有 `[HW]`/`[ST]`/`[FW]`/`[VESC]` 来源的标定值。
//!   - `#[repr(C)]` 只为稳定布局和后续 C 互操作准备；本 crate 只生成 rlib，
//!     尚未导出 C ABI 符号。
//!
//! 实时性与定位 / Real-time status:
//!   - 默认定位是离线整定、仿真和设计评审，不是 12 kHz ADC 电流环的实时代码。
//!   - 若确需放进快环：本文件无分配、无阻塞、无日志、无 Mutex 等待，但必须
//!     先在目标板上单独实测 WCET 和栈占用（见 `算法库实时性说明.md`）。
//!   - 越界参数不会被拒绝，只会被限幅或回退到默认值；`NaN` 不在有效输入域内。
//!
//! 范围与验证边界 / Scope and verification limits:
//!   - 这些是与原 C 参考库一致的基础标量/小模型实现，不是通用矩阵求解框架；
//!     H∞、LQR、MPC 都不做矩阵运算，MPC 只是有限候选枚举，不是 QP 求解器。
//!   - 单元测试只证明实现与 C 参考向量数值一致，不证明真实电机上的闭环稳定性；
//!     投产前必须补仿真、HIL 和低压限流实机验证。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 自适应比例控制器的配置参数。
/// Configuration of the adaptive proportional controller.
///
/// 控制律为 `gain += learning_rate * error * reference * ts`，`output = gain * error`。
/// 这是梯度型（MIT 规则形式）的标量自适应律，回归量取 `reference` 而不是 `error`；
/// 换成 `error * error` 就不再与 C 参考向量等价。
/// A gradient-type (MIT-rule form) scalar adaptation law whose regressor is
/// `reference`; substituting `error * error` would break C-reference equivalence.
///
/// 定位 / Status: 设计期与离线整定使用；放进 12 kHz 电流环前必须实测 WCET。
/// Design-time/offline by default; measure WCET before fast-loop use.
pub struct AdaptiveParam {
    /// 自适应律步长，量纲为 `1/(error单位 * reference单位 * s)`；
    /// 它与误差、回归量、`ts` 相乘后直接累加到增益上，所以量纲随被控量而变。
    /// Adaptation step; dimension is 1/(error unit * reference unit * s).
    pub learning_rate: f32,
    /// 调用周期 `[s]`，必须与真实控制周期一致，多速率环要分别设置；
    /// `<= 0` 时 `update` 立即返回 0.0 且不改状态。
    /// Call period `[s]`; a non-positive value disables `update` entirely.
    pub ts: f32,
    /// 自适应增益下界（投影边界），防止自适应律把增益拉到发散方向。
    /// Lower projection bound for the adaptive gain.
    pub gain_min: f32,
    /// 自适应增益上界；取值应使最坏工作点仍留有相位裕度，不能取到不稳定区。
    /// Upper projection bound for the adaptive gain.
    pub gain_max: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit; the unit follows the controlled variable.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者，不会 panic。
    /// Output upper limit; `clamp` swaps the bounds instead of panicking.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 自适应控制器的运行状态。
/// Runtime state of the adaptive controller.
///
/// 三个字段都是"最近一次调用"的快照，供诊断和上位机观测使用；
/// 只有 `gain` 会参与下一周期的运算。
/// All fields are snapshots of the last call; only `gain` carries over.
pub struct AdaptiveState {
    /// 当前自适应增益；误差与 `reference` 同量纲时它是无量纲的。
    /// Current adaptive gain; dimensionless when the two inputs share a unit.
    pub gain: f32,
    /// 最近一次 `reference - feedback`，注意符号是"参考减反馈"。
    /// Last `reference - feedback`; note the sign convention.
    pub error: f32,
    /// 最近一次限幅后的输出，单位由被控量决定。
    /// Last clamped output.
    pub output: f32,
}

impl AdaptiveState {
    /// 按给定初始增益建立状态，其余字段清零。
    /// Creates the state with an explicit initial gain; the rest is zeroed.
    ///
    /// 初始增益不做投影：越界初值会在第一次 `update` 调用中被
    /// `gain_min`/`gain_max` 拉回区间内。
    /// The initial gain is not projected here; the first `update` clamps it.
    pub fn new(initial_gain: f32) -> Self {
        Self {
            gain: initial_gain,
            ..Self::default()
        }
    }

    /// 等价于用新初值重建状态，用于故障恢复、模式切换或重新使能。
    /// Same as rebuilding the state, for fault recovery or mode changes.
    pub fn reset(&mut self, gain: f32) {
        *self = Self::new(gain);
    }

    /// 推进一个自适应控制周期并返回本周期输出。
    /// Advances one adaptive control period and returns the output.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，同时充当自适应律的回归量，单位由被控量决定
    ///               target value; it doubles as the adaptation regressor
    ///   feedback  - 反馈值，与 `reference` 同单位
    ///               feedback value, same unit as `reference`
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量；`ts <= 0` 时返回
    /// 0.0 且完全不修改状态（连 `error` 都不更新）。
    ///
    /// 关键顺序 / Key ordering: 先用本周期误差更新增益，再用"更新后的增益
    /// 与本周期误差"计算输出，所以输出用的不是上一周期的误差。这个顺序是
    /// `adaptive_matches_c_reference` 能通过的必要条件。
    /// Gain is updated before the output is formed, using the current-cycle error.
    ///
    /// 失败语义 / Failure semantics: 不做安全检查，也不清零外部输出；
    /// `reference`/`feedback` 含 `NaN` 会污染 `gain` 并让后续输出持续为 `NaN`。
    ///
    /// 上下文 / Context: 设计期或低优先级线程；快环使用需先实测 WCET。
    pub fn update(&mut self, param: &AdaptiveParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.error = reference - feedback;
        self.gain += param.learning_rate * self.error * reference * param.ts;
        self.gain = clamp(self.gain, param.gain_min, param.gain_max);
        self.output = clamp(self.gain * self.error, param.out_min, param.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 二阶反步控制器的配置参数。
/// Configuration of the second-order backstepping controller.
///
/// 被控对象按积分链假设 `x1' = x2`、`x2' = u`；控制律为
/// `alpha = reference_dot - k1*e1`，
/// `u = reference_ddot - k1*(x2 - reference_dot) - k2*e2`，最后限幅。
/// Assumes the integrator chain `x1' = x2`, `x2' = u`; the law above is applied
/// and then clamped. This is a scalar design, not a general backstepping toolkit.
///
/// 定位 / Status: 依赖模型结构的设计期方法；单元测试只覆盖参考向量，
/// 不证明带摩擦、延迟和电流环动态的真实对象稳定。
/// Design-time only; passing the reference-vector test does not imply stability
/// on a real plant with friction, delay and current-loop dynamics.
pub struct BacksteppingParam {
    /// 第一级（虚拟控制）增益，量纲 `[1/s]`（x1 为位置、x2 为速度时）；
    /// 标准稳定性论证要求 `k1 > 0`，本函数不校验符号。
    /// First-stage virtual-control gain `[1/s]`; the standard argument needs
    /// `k1 > 0`, but the sign is not validated here.
    pub k1: f32,
    /// 第二级误差增益，量纲 `[1/s]`；同样要求 `k2 > 0`。
    /// Second-stage error gain `[1/s]`; `k2 > 0` is required by the same argument.
    pub k2: f32,
    /// 输出下限；单位是 u 的单位，对二阶机械对象通常是角加速度 `[rad/s^2]`
    /// 或力/转矩指令 `[N*m]`。
    /// Output lower limit, in the unit of u.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 反步控制器的运行状态。
/// Runtime state of the backstepping controller.
///
/// 四个字段都只由 `update` 写出；`alpha` 与两个误差保留下来是为了诊断
/// "输出是被限幅压住，还是被虚拟控制带偏"。
/// All four fields are written by `update`; keeping `alpha` and both errors makes
/// saturation issues diagnosable.
pub struct BacksteppingState {
    /// 第一级跟踪误差 `x1 - reference`，单位同 `x1`。
    /// First-stage tracking error `x1 - reference`, in the unit of `x1`.
    pub e1: f32,
    /// 第二级误差 `x2 - alpha`，即"实际变化率 - 虚拟控制"，单位同 `x2`。
    /// Second-stage error `x2 - alpha`, in the unit of `x2`.
    pub e2: f32,
    /// 虚拟控制（期望变化率），单位同 `x2`。
    /// Virtual control (the desired rate), in the unit of `x2`.
    pub alpha: f32,
    /// 最近一次限幅后的控制量。
    /// Last clamped control output.
    pub output: f32,
}

impl BacksteppingState {
    /// 清零全部状态。
    /// Clears all state.
    ///
    /// 切换目标轨迹、重新使能或故障恢复时必须调用：`e1`/`e2` 的残留会让
    /// 新轨迹的第一拍产生额外控制量冲击。
    /// Call it on target changes or fault recovery: stale errors cause a first-cycle
    /// kick on the new trajectory.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一个反步控制周期并返回本周期输出。
    /// Advances one backstepping period and returns the output.
    ///
    /// 参数 / Parameters:
    ///   x1            - 第一级状态（通常为位置 `[rad]`），单位由调用方定义
    ///   x2            - 第二级状态（通常为角速度 `[rad/s]`），是 x1 的导数
    ///   reference     - 目标轨迹值，单位同 `x1`
    ///   reference_dot - 目标一阶导数，单位 `[x1单位/s]`
    ///   reference_ddot- 目标二阶导数，单位 `[x1单位/s^2]`
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量，单位 `[x1单位/s^2]`。
    ///
    /// 为什么这样写 / Rationale: `k1*(x2 - reference_dot)` 这一项用状态本身
    /// 替换了 `e1` 的解析导数，省掉一次求导；代价是完全依赖 `reference_dot`
    /// 与 `reference` 的一致性。若 `reference_dot` 由带滞后的差分得到，
    /// 这个抵消项就会带入相位误差。
    /// The `k1*(x2 - reference_dot)` term substitutes the state for the analytic
    /// derivative of `e1`; it therefore inherits any lag in `reference_dot`.
    ///
    /// 上下文 / Context: 设计期/离线；快环使用需先实测 WCET。
    /// Design-time/offline; measure WCET before fast-loop use.
    pub fn update(
        &mut self,
        param: &BacksteppingParam,
        x1: f32,
        x2: f32,
        reference: f32,
        reference_dot: f32,
        reference_ddot: f32,
    ) -> f32 {
        self.e1 = x1 - reference;
        self.alpha = reference_dot - param.k1 * self.e1;
        self.e2 = x2 - self.alpha;
        let output = reference_ddot - param.k1 * (x2 - reference_dot) - param.k2 * self.e2;
        self.output = clamp(output, param.out_min, param.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 3x3 模糊控制器的配置参数。
/// Configuration of the 3x3 fuzzy controller.
///
/// 两个输入（误差与误差变化量）先各自归一化，再取三角隶属度得到
/// "负/零/正"三档权重，最后对 9 条规则的后件做加权平均。
/// Two inputs are normalised, turned into triangular membership weights and then
/// combined as a weighted average over the 9 rule consequents.
///
/// 因为后件是常数而不是模糊集合，这里的去模糊化是单点（singleton）加权平均，
/// 不是重心法；整张规则表在 `update` 里被完整遍历，代价与输入无关。
/// Consequents are constants, so defuzzification is a singleton weighted average
/// (not a centre-of-gravity integral); the full table is always traversed.
pub struct FuzzyParam {
    /// 误差归一化因子，单位同误差；内部按 `error / e_scale` 归一。
    /// 配成 0.0 会被当作 1.0 处理（静默回退，不报错），因为下面要用它做除数。
    /// Error normalisation factor; `0.0` silently falls back to `1.0` because it is
    /// used as a divisor.
    pub e_scale: f32,
    /// 误差变化量归一化因子，单位是"误差单位 / 调用周期"；
    /// 本库不对误差求差分，`error_delta` 必须由调用方按同一周期算好。
    /// Delta normalisation factor; the library never differentiates, so the caller
    /// must supply `error_delta` on the same period.
    pub de_scale: f32,
    /// 规则表输出的比例因子，把无量纲后件映射到控制量单位。
    /// Rule-table output scale, mapping dimensionless consequents to the output unit.
    pub out_scale: f32,
    /// 3x3 规则表，索引为 `[误差档][变化量档]`，档位顺序是 负 / 零 / 正；
    /// 后件是常数（单点），典型取值在 -1..1，再由 `out_scale` 放大。
    /// 3x3 rule table indexed as `[error tier][delta tier]`, tiers are neg/zero/pos.
    pub rules: [[f32; 3]; 3],
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 模糊控制器的运行状态。
/// Runtime state of the fuzzy controller.
pub struct FuzzyState {
    /// 最近一次限幅后的输出。
    /// Last clamped output.
    pub output: f32,
    /// 9 条规则的权重和（去模糊化的分母）。
    /// Sum of the 9 rule weights, i.e. the defuzzification denominator.
    ///
    /// 三角隶属度三档之和在有限输入下恒为 1，所以这个值通常正好是 1.0；
    /// 它保留下来只是为了诊断。输入含 `NaN` 时它为 `NaN`，此时输出被强制
    /// 置 0.0（见 `update` 的 `denominator > 0.0` 判断）。
    /// It is normally 1.0 because the membership weights sum to unity; a `NaN`
    /// input makes it `NaN`, and the output is then forced to 0.0.
    pub weight_sum: f32,
}

/// 单个输入的三角隶属度函数，返回 `[负, 零, 正]` 三档权重。
/// Triangular membership of one input, returning `[negative, zero, positive]`.
///
/// 输入先被钳位到 -1..1，因此超过 ±1 的误差与恰好 ±1 完全等价：这是饱和，
/// 不是外推。三档权重之和为 1，这就是"归一化输入"的含义。
/// The input is clamped to -1..1 first, so this saturates rather than extrapolates,
/// and the three weights always sum to unity.
///
/// 这里的 ±1 是数学归一化边界，不是 `[HW]`/`[ST]`/`[FW]`/`[VESC]` 来源的标定值。
/// The ±1 limits are mathematical normalisation bounds, not calibrated constants.
fn fuzzy_membership(x: f32) -> [f32; 3] {
    let value = clamp(x, -1.0, 1.0);
    [
        if value < 0.0 { -value } else { 0.0 },
        1.0 - value.abs(),
        if value > 0.0 { value } else { 0.0 },
    ]
}

impl FuzzyState {
    /// 清零 `output` 与 `weight_sum`；规则表在 `param` 里，不受复位影响。
    /// Clears `output` and `weight_sum`; the rule table lives in `param`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 用误差与误差变化量求一次模糊控制输出。
    /// Computes one fuzzy control output from the error and its delta.
    ///
    /// 参数 / Parameters:
    ///   error       - 被控量误差，单位由被控量决定
    ///   error_delta - 误差的变化量（本周期减上周期），单位是"误差单位/周期"；
    ///                 不要传已经除以 `ts` 的变化率，否则 `de_scale` 的含义
    ///                 会随采样周期改变
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量。
    ///
    /// 边界语义 / Edge cases:
    ///   - `e_scale` 或 `de_scale` 为 0.0 时静默按 1.0 处理；
    ///   - 权重和 `<= 0`（只可能来自 `NaN` 输入）时返回 0.0 而不是 `NaN`，
    ///     相当于"无有效规则"时输出中性；
    ///   - 输入含 `NaN` 不会 panic，但输出会退化为 0.0，调用方需自行判定
    ///     这是"中性"还是"失去控制"。
    ///
    /// 上下文 / Context: 设计期/离线；固定 9 次乘加，无分配，无函数调用。
    pub fn update(&mut self, param: &FuzzyParam, error: f32, error_delta: f32) -> f32 {
        let e_scale = if param.e_scale == 0.0 {
            1.0
        } else {
            param.e_scale
        };
        let de_scale = if param.de_scale == 0.0 {
            1.0
        } else {
            param.de_scale
        };
        let e_mu = fuzzy_membership(error / e_scale);
        let de_mu = fuzzy_membership(error_delta / de_scale);
        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for (i, e_weight) in e_mu.iter().enumerate() {
            for (j, de_weight) in de_mu.iter().enumerate() {
                let weight = e_weight * de_weight;
                numerator += weight * param.rules[i][j];
                denominator += weight;
            }
        }
        self.weight_sum = denominator;
        self.output = if denominator > 0.0 {
            clamp(
                numerator / denominator * param.out_scale,
                param.out_min,
                param.out_max,
            )
        } else {
            0.0
        };
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 标量 H∞ 鲁棒状态反馈的配置参数。
/// Configuration of the scalar H-infinity robust state feedback.
///
/// 控制律为 `robust_term = -(kw/gamma)*disturbance`，
/// `u = feedforward - kx*x + robust_term`，最后限幅。
/// Law: `robust_term = -(kw/gamma)*disturbance` and
/// `u = feedforward - kx*x + robust_term`, then clamped.
///
/// 范围 / Scope: 这是与原 C 参考库一致的基础标量形式，不是 Riccati 方程求解器，
/// 也不含状态观测器；没有矩阵、没有 γ 迭代，`gamma` 只是给定的性能水平。
/// Scalar form matching the C reference: no Riccati solve, no observer, no matrix
/// work; `gamma` is simply a given performance level.
pub struct HinfParam {
    /// 状态反馈增益，量纲为 `u 单位 / x 单位`。
    /// State-feedback gain, in u units per x unit.
    pub kx: f32,
    /// 扰动通道增益，量纲为 `u 单位 / disturbance 单位`。
    /// Disturbance-channel gain, in u units per disturbance unit.
    pub kw: f32,
    /// H∞ 性能水平，无量纲；它出现在分母上，越小鲁棒补偿越强。
    /// 代码对 `<= 0.0` 静默按 1.0 处理，避免除零。
    /// Performance level; it divides the compensation term, so smaller means
    /// stronger compensation. Non-positive values silently fall back to 1.0.
    pub gamma: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// H∞ 控制器的运行状态。
/// Runtime state of the H-infinity controller.
pub struct HinfState {
    /// 最近一次限幅后的控制量。
    /// Last clamped control output.
    pub u: f32,
    /// 最近一次算出的鲁棒补偿项 `-(kw/gamma)*disturbance`，量纲同 `u`。
    /// 单独保留它是为了判断输出是被反馈项主导还是被补偿项顶到限幅。
    /// Last robust term, in the unit of `u`; kept so saturation can be attributed
    /// to the feedback or the compensation path.
    pub robust_term: f32,
}

impl HinfState {
    /// 清零 `u` 与 `robust_term`；配置参数在 `param` 里，不受复位影响。
    /// Clears `u` and `robust_term`; the configuration lives in `param`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 用标量 H∞ 状态反馈加扰动补偿求一次控制量。
    /// Computes one control output with scalar H-infinity state feedback plus
    /// disturbance compensation.
    ///
    /// 参数 / Parameters:
    ///   x           - 标量状态，通常是角度 `[rad]` 或速度 `[rad/s]`
    ///                 scalar state, typically an angle or a speed
    ///   disturbance - 扰动估计值，单位要与 `kw` 的量纲约定一致。
    ///                 本库不含扰动观测器，该值必须由调用方提供
    ///                 （ADRC 的 ESO 在 `adrc.rs` 中）
    ///                 disturbance estimate; supplied by the caller
    ///   feedforward - 前馈项，单位同 u，直接叠加在反馈之上
    ///                 feedforward term in the unit of u
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量 `u`。
    ///
    /// 为什么这样写 / Rationale: `gamma` 在作除数前先做 `<= 0.0` 检查并回退到
    /// 1.0，所以 `gamma = 0.0` 不会算出 `inf`；但很小的正 `gamma` 仍会把补偿项
    /// 放大到顶住限幅，那是整定问题而不是数值问题。
    /// `gamma` is guarded against zero, but a tiny positive value can still saturate
    /// the compensation path; that is a tuning fault, not a numerical one.
    ///
    /// 上下文 / Context: 设计期/离线；快环使用需先实测 WCET。
    pub fn update(&mut self, param: &HinfParam, x: f32, disturbance: f32, feedforward: f32) -> f32 {
        let gamma = if param.gamma <= 0.0 { 1.0 } else { param.gamma };
        self.robust_term = -(param.kw / gamma) * disturbance;
        self.u = clamp(
            feedforward - param.kx * x + self.robust_term,
            param.out_min,
            param.out_max,
        );
        self.u
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 二状态固定增益 LQR 的配置参数。
/// Configuration of the two-state fixed-gain LQR.
///
/// 控制律为 `u = feedforward - k0*x0 - k1*x1`，然后限幅。
/// Law: `u = feedforward - k0*x0 - k1*x1`, then clamped.
///
/// 范围 / Scope: 本库只做反馈综合的最后一步：增益矩阵 K 必须在库外离线求解
/// （Riccati/DLQR 或手工极点配置）。这里没有代价函数、没有权重矩阵 Q/R，
/// 也没有迭代求解器，因此它既不能检查也不能改善闭环性能。
/// The gain matrix K must be solved offline outside this library; there is no cost
/// function, no Q/R weighting and no solver here.
pub struct LqrParam {
    /// `x0` 的反馈增益，量纲为 `u 单位 / x0 单位`；`x0` 通常是位置 `[rad]`。
    /// Gain for `x0`, in u units per x0 unit.
    pub k0: f32,
    /// `x1` 的反馈增益，量纲为 `u 单位 / x1 单位`；`x1` 通常是速度 `[rad/s]`。
    /// Gain for `x1`, in u units per x1 unit.
    pub k1: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// LQR 控制器的运行状态。
/// Runtime state of the LQR controller.
pub struct LqrState {
    /// 最近一次限幅后的控制量。
    /// Last clamped control output.
    pub u: f32,
}

impl LqrState {
    /// 清零输出；反馈增益在 `param` 里，不受复位影响。
    /// Clears the output; the gains live in `param`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 求一次二状态固定增益状态反馈。
    /// Computes one two-state fixed-gain state-feedback step.
    ///
    /// 参数 / Parameters:
    ///   x0          - 第一个状态，单位由 `k0` 的量纲约定决定
    ///                 first state, unit defined by the convention behind `k0`
    ///   x1          - 第二个状态，单位由 `k1` 的量纲约定决定
    ///                 second state, unit defined by the convention behind `k1`
    ///   feedforward - 前馈项，单位同 u
    ///                 feedforward term in the unit of u
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量。
    ///
    /// 注意 / Pitfall: 纯状态反馈没有积分项，对常值扰动和模型偏差会留下稳态
    /// 误差，要消差必须外加积分环或前馈；限幅是唯一的保护，本函数不含
    /// 抗饱和回算。
    /// Pure state feedback has no integral action, so constant disturbances leave a
    /// steady-state error; the clamp is the only protection here.
    ///
    /// 上下文 / Context: 设计期/离线；快环使用需先实测 WCET。
    pub fn update(&mut self, param: &LqrParam, x0: f32, x1: f32, feedforward: f32) -> f32 {
        self.u = clamp(
            feedforward - param.k0 * x0 - param.k1 * x1,
            param.out_min,
            param.out_max,
        );
        self.u
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 一阶离散模型 MPC 的配置参数。
/// Configuration of the first-order discrete-model MPC.
///
/// 预测模型 `x[k+1] = a*x[k] + b*u[k]`，单步代价
/// `cost = q*(x_pred - reference)^2 + r*u^2`。
/// Model `x[k+1] = a*x[k] + b*u[k]`; cost `q*(x_pred - ref)^2 + r*u^2`.
///
/// 求解方式是在 `[u_min, u_max]` 上等距枚举 `max(candidates, 2)` 个候选并取代价
/// 最小者：这是有限候选搜索，不是 QP、梯度法或多步滚动优化，没有终端代价，
/// 唯一的约束就是候选区间本身。
/// A finite candidate-grid search, not a QP solver: no multi-step horizon, no
/// terminal cost, and the candidate interval is the only constraint.
pub struct MpcParam {
    /// 离散状态系数，无量纲；由调用方辨识或按模型离散化得到，本库不辨识。
    /// Discrete state coefficient (dimensionless); never identified in-library.
    pub a: f32,
    /// 离散输入系数，量纲为 `x 单位 / u 单位`；离散化时应已包含采样周期 `ts`。
    /// Discrete input coefficient, in x units per u unit; it should already embed
    /// the sampling period.
    pub b: f32,
    /// 状态误差权重，量纲 `1/(x 单位^2)`；它与 `r` 的相对大小直接决定最优候选
    /// 偏向跟踪还是偏向小控制量。
    /// State-error weight, dimension 1/(x unit^2).
    pub q: f32,
    /// 控制量权重，量纲 `1/(u 单位^2)`；取 0.0 时代价只惩罚跟踪误差，
    /// 最优候选会贴向使预测等于参考的那一端（可能是 `u_min` 或 `u_max`）。
    /// Control weight, dimension 1/(u unit^2); zero removes any effort penalty.
    pub r: f32,
    /// 候选控制量下限，单位同 u。
    /// Lower bound of the candidate control values, in the unit of u.
    pub u_min: f32,
    /// 候选控制量上限；与 `u_min` 配反时本函数不会纠正（这里没用到 `clamp`），
    /// 候选网格只是反向铺开。
    /// Upper bound of the candidates; a swapped pair is not corrected, the grid is
    /// simply laid out in reverse.
    pub u_max: f32,
    /// 候选控制量个数（`i32`）。库内取下界 `max(candidates, 2)` 以避免
    /// `candidates - 1` 变成 0（除零），但**不设上界**：循环次数等于
    /// `max(candidates, 2)`，执行时间随它线性增长，产品配置必须自行限制，
    /// 并实测 WCET（`算法库实时性说明.md` 记录的是同一结论）。
    /// Candidate count, floored at 2 to avoid a zero divisor and deliberately not
    /// capped; the loop runs `max(candidates, 2)` times.
    pub candidates: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// MPC 控制器的运行状态。
/// Runtime state of the MPC controller.
pub struct MpcState {
    /// 本周期选中的控制量，取自离散候选网格，没有在候选之间做插值。
    /// Selected control value, taken from the grid without interpolation.
    pub u: f32,
    /// 选中候选对应的一步预测状态 `a*x + b*u`，供诊断。
    /// One-step prediction of the selected candidate, kept for diagnostics.
    pub predicted_x: f32,
    /// 选中候选的加权代价；`NaN`/`inf` 说明参数里含非有限值。
    /// Cost of the selected candidate; non-finite means the parameters are bad.
    pub cost: f32,
}

impl MpcState {
    /// 清零 `u`、`predicted_x` 与 `cost`；配置在 `param` 里，不受复位影响。
    /// Clears the state; the configuration lives in `param`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 穷举次数由 `candidates` 决定；实时应用必须在配置阶段限制其上界。
    /// The number of enumerated candidates is driven by `candidates`; a real-time
    /// application must bound it at configuration time.
    ///
    /// 参数 / Parameters:
    ///   x         - 当前状态，单位与预测模型的 x 一致
    ///               current state, in the unit used by the prediction model
    ///   reference - 目标状态，单位同 x
    ///               target state, same unit as x
    ///
    /// 返回 / Returns: 选中的控制量。若没有任何候选的有限代价能小于
    /// `f32::MAX`（例如参数含 `NaN`），`best_u` 保持初值，函数**静默返回
    /// 0.0**：调用方应检查 `cost.is_finite()`，以区分"最优就是 0"和"求解失败"。
    /// If every candidate cost is non-finite the function silently returns 0.0, so
    /// check `cost.is_finite()` to tell "zero is optimal" from "the solve failed".
    ///
    /// 网格 / Grid: `ratio = i/(candidates-1)`，`i = 0` 对应 `u_min`、
    /// `i = candidates-1` 对应 `u_max`，两个端点都包含；`candidates = 2` 时只能
    /// 在两个极端之间二选一，控制量必然是 bang-bang。
    /// Both endpoints are included, so two candidates degenerate to bang-bang.
    ///
    /// 注意 / Pitfall: 代价只惩罚 `u` 本身而不惩罚 `Δu`，相邻周期的选中候选
    /// 可以突跳；需要平滑时应自己加变化率惩罚或后置斜坡限幅。代价里也没有
    /// 母线电压/电流约束，安全包络必须由外层负责。
    /// The cost does not penalise `Δu`, so the selection can jump between periods,
    /// and no bus-voltage or current constraint is represented here.
    ///
    /// 上下文 / Context: 设计期/离线；执行时间与 `candidates` 成正比，
    /// 快环使用前必须先固定上界并实测 WCET。
    pub fn update(&mut self, param: &MpcParam, x: f32, reference: f32) -> f32 {
        let candidates = param.candidates.max(2);
        let mut best_cost = f32::MAX;
        let mut best_u = 0.0;
        let mut best_x = 0.0;
        for i in 0..candidates {
            let ratio = i as f32 / (candidates - 1) as f32;
            let u = param.u_min + ratio * (param.u_max - param.u_min);
            let predicted = param.a * x + param.b * u;
            let error = predicted - reference;
            let cost = param.q * error * error + param.r * u * u;
            if cost < best_cost {
                best_cost = cost;
                best_u = u;
                best_x = predicted;
            }
        }
        self.u = best_u;
        self.predicted_x = best_x;
        self.cost = best_cost;
        self.u
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 参考模型自适应控制（MRAC）的配置参数。
/// Configuration of the model-reference adaptive controller (MRAC).
///
/// 参考模型是一阶连续系统 `y' = -model_a*y + model_b*reference`，用前向欧拉离散：
/// `y += ts*(-model_a*y + model_b*reference)`；可调参数只有一个标量 `theta`，
/// 输出为 `theta * reference`。
/// The reference model is integrated with forward Euler; a single scalar `theta` is
/// adapted and the output is `theta * reference`.
pub struct MracParam {
    /// 调用周期 `[s]`，必须与真实控制周期一致（它同时是前向欧拉的步长）；
    /// `<= 0` 时 `update` 立即返回 0.0 且不修改状态。
    /// Call period `[s]`, also the forward-Euler step; `<= 0` disables `update`.
    pub ts: f32,
    /// 参考模型极点，量纲 `[1/s]`；必须 `> 0` 参考模型才稳定。前向欧拉在
    /// `ts > 2/model_a` 时会数值发散，本库不检查这个条件。
    /// Reference-model pole `[1/s]`; forward Euler diverges once `ts > 2/model_a`
    /// and that condition is not checked here.
    pub model_a: f32,
    /// 参考模型输入增益，量纲 `y 单位 / (reference 单位 * s)`。
    /// Reference-model input gain.
    pub model_b: f32,
    /// 自适应增益，量纲 `1/(y 单位 * reference 单位 * s)`；越大收敛越快，
    /// 也越容易在测量噪声下振荡。本库不校验其符号。
    /// Adaptation gain; larger converges faster and oscillates more easily.
    pub gamma: f32,
    /// `theta` 的下投影边界，用于防止自适应参数漂移；建议按被控对象增益的
    /// 物理范围设定。
    /// Lower projection bound for `theta`.
    pub theta_min: f32,
    /// `theta` 的上投影边界。
    /// Upper projection bound for `theta`.
    pub theta_max: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// MRAC 的运行状态。
/// Runtime state of the MRAC.
pub struct MracState {
    /// 参考模型输出的当前值，单位同 y；它是误差的比较基准，所以复位时被清零
    /// 而不是置成目标值。
    /// Current reference-model output; it is the baseline for the error, so `reset`
    /// clears it.
    pub model_output: f32,
    /// 当前可调参数 `theta`，量纲为 `输出单位 / reference 单位`。
    /// The adapted scalar parameter.
    pub theta: f32,
    /// 最近一次 `model_output - feedback`；符号是"模型减反馈"，
    /// 与 `AdaptiveState::error` 的"参考减反馈"相反，两套符号不要混用。
    /// Last `model_output - feedback`; its sign is opposite to `AdaptiveState`.
    pub error: f32,
    /// 最近一次限幅后的输出。
    /// Last clamped output.
    pub output: f32,
}

impl MracState {
    /// 按给定初始 `theta` 建立状态，其余字段清零。
    /// Creates the state with an explicit initial `theta`.
    ///
    /// 初始 `theta` 不做投影：越界初值会在第一次 `update` 调用中被
    /// `theta_min`/`theta_max` 拉回区间内。
    /// The initial value is not projected; the first `update` clamps it.
    pub fn new(initial_theta: f32) -> Self {
        Self {
            theta: initial_theta,
            ..Self::default()
        }
    }

    /// 等价于用新初值重建状态；`model_output` 与 `error` 同时清零，
    /// 避免旧参考模型状态在新模式下继续参与误差计算。
    /// Same as rebuilding the state; this also clears the stale model output.
    pub fn reset(&mut self, theta: f32) {
        *self = Self::new(theta);
    }

    /// 推进一个 MRAC 周期并返回本周期输出。
    /// Advances one MRAC period and returns the output.
    ///
    /// 参数 / Parameters:
    ///   reference - 目标值，同时是参考模型的输入和自适应律的回归量，
    ///               单位由被控量决定
    ///               target value; input to the reference model and regressor
    ///   feedback  - 实际反馈值，单位同参考模型输出 y
    ///               measured feedback, in the unit of the model output
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的 `theta * reference`；
    /// `ts <= 0` 时返回 0.0 且完全不修改状态（连 `model_output` 都不推进）。
    ///
    /// 为什么这样写 / Rationale: 参数更新项和输出都正比于 `reference`，
    /// 所以在 `reference == 0` 时自适应停止、输出也为 0：零指令下没有持续激励，
    /// 参数不会收敛。这是教科书 MRAC 的已知性质，不是本实现的缺陷。
    /// Both the update term and the output are proportional to `reference`, so at zero
    /// reference there is no excitation and adaptation stalls.
    ///
    /// 上下文 / Context: 设计期/离线；快环使用需先实测 WCET。
    pub fn update(&mut self, param: &MracParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.model_output +=
            param.ts * (-param.model_a * self.model_output + param.model_b * reference);
        self.error = self.model_output - feedback;
        self.theta += param.gamma * self.error * reference * param.ts;
        self.theta = clamp(self.theta, param.theta_min, param.theta_max);
        self.output = clamp(self.theta * reference, param.out_min, param.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 滑模控制器（SMC）的配置参数。
/// Configuration of the sliding-mode controller (SMC).
///
/// 滑模面 `s = c*error + error_dot`，切换项 `-k*sat(s/boundary)`，
/// 输出 `equivalent + switching` 再限幅；等效控制 `equivalent` 由调用方提供。
/// Surface `s = c*error + error_dot`, switching term `-k*sat(s/boundary)`, output
/// `equivalent + switching`; the equivalent control comes from the caller.
///
/// 定位 / Status: 设计期/离线。抖振抑制只靠边界层，没有观测器、没有自适应增益，
/// 单元测试只覆盖参考向量，不证明实机抖振和噪声可接受。
/// Design-time/offline: chattering is limited by the boundary layer only.
pub struct SmcParam {
    /// 滑模面系数，量纲 `[1/s]`（使 `c*error` 与 `error_dot` 同量纲）；
    /// 越大越强调误差本身，收敛更慢但对微分噪声更不敏感。
    /// Surface coefficient `[1/s]`; larger weights the error more and noise less.
    pub c: f32,
    /// 切换增益，量纲与滑模面相同（`error 单位/s`）。按到达条件，它必须大于
    /// 扰动与模型误差的上界，否则滑模面不可达、误差不会收敛到边界层内。
    /// Switching gain in the unit of the surface; it must exceed the disturbance
    /// bound for the reaching condition to hold. The sign is not validated.
    pub k: f32,
    /// 边界层半宽，量纲同滑模面 `s`；`<= 0` 时退化为理想符号切换，
    /// 抖动最大，离散实现下容易进入极限环。
    /// Boundary-layer half-width; a non-positive value degenerates to ideal switching.
    pub boundary: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 滑模控制器的运行状态。
/// Runtime state of the sliding-mode controller.
pub struct SmcState {
    /// 最近一次的滑模面值 `s`，量纲同 `error_dot`；
    /// 它是判断"是否已进入边界层"的唯一依据。
    /// Last surface value; it tells whether the state is inside the boundary layer.
    pub surface: f32,
    /// 最近一次的切换项 `-k*sat(s/boundary)`，量纲同输出 u。
    /// Last switching term, in the unit of the output.
    pub switching: f32,
    /// 最近一次限幅后的输出。
    /// Last clamped output.
    pub output: f32,
}

/// 边界层饱和函数 `sat(s/boundary)`，返回 -1..1。
/// Boundary-layer saturation `sat(s/boundary)`, returning -1..1.
///
/// `boundary <= 0` 时退化为符号函数，并且对 `value == 0.0` 明确返回 0.0：
/// 这既避免 `sign(0)` 的歧义，也让理想切换在滑模面上不产生无意义的翻转。
/// A non-positive boundary degenerates to the sign function with `sign(0) == 0.0`,
/// which keeps ideal switching from toggling on the surface itself.
///
/// 饱和限 ±1 同样是数学归一化常数，与硬件实测、ST MCSDK 或 VESC 策略无关。
/// The ±1 saturation limits are mathematical constants, not calibration data.
fn smc_saturate(value: f32, boundary: f32) -> f32 {
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

impl SmcState {
    /// 清零滑模面、切换项与输出；配置在 `param` 里，不受复位影响。
    /// Clears the surface, switching term and output.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 推进一个滑模控制周期并返回本周期输出。
    /// Advances one sliding-mode period and returns the output.
    ///
    /// 参数 / Parameters:
    ///   error      - 被控量误差，单位由被控量决定
    ///   error_dot  - 误差的导数，单位 `[error单位/s]`；离散实现里通常来自
    ///                差分或滤波微分器，其量化噪声会直接进入滑模面 `s`
    ///   equivalent - 等效控制（基于模型的前馈/标称控制），单位同 u；
    ///                由调用方计算，本库不估计它
    ///                equivalent control, computed by the caller
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的控制量。
    ///
    /// 为什么这样写 / Rationale: 用边界层饱和而不是硬符号函数，是为了把不连续
    /// 切换变成边界层内的线性比例，代价是稳态误差带约为 `boundary` 的宽度；
    /// 边界层越大越平滑，跟踪精度越低。`k` 必须覆盖扰动上界，否则边界层内
    /// 的比例控制力不足以把状态推回滑模面。
    /// The boundary layer trades tracking accuracy for chattering reduction, and
    /// `k` must still cover the disturbance bound.
    ///
    /// 上下文 / Context: 设计期/离线；快环使用需先实测 WCET。
    pub fn update(&mut self, param: &SmcParam, error: f32, error_dot: f32, equivalent: f32) -> f32 {
        self.surface = param.c * error + error_dot;
        self.switching = -param.k * smc_saturate(self.surface, param.boundary);
        self.output = clamp(equivalent + self.switching, param.out_min, param.out_max);
        self.output
    }
}

#[cfg(test)]
mod tests {
    // 主机单元测试：不在目标板上运行，也不属于 12 kHz 控制路径。
    // Host-only unit tests; they never run on the target or inside the control ISR.
    //
    // 命名后缀 `_matches_c_reference` 的用例锁定了"与原 C 参考实现数值可互换"
    // 这一契约：运算顺序、限幅位置、增益与参数更新的先后都是契约的一部分；
    // 改动其中任何一步都会让这些向量失败，而失败本身并不说明 C 版是错的。
    // The `_matches_c_reference` cases pin numerical interchangeability with the C
    // reference; the exact operation order and clamping position are part of it.
    //
    // 通过这些用例只证明与参考向量一致，不证明闭环稳定性，见
    // `算法库移植状态.md` 的迁移验收门槛第 7 条。
    // Passing them proves reference-vector agreement, not closed-loop stability.
    use super::*;

    /// 近似比较辅助函数，容差由调用点给出；1e-6 是 `f32` 在参考向量量级下的
    /// 比较余量，不是物理误差指标。
    /// Approximate comparison helper; the tolerance is a `f32` rounding allowance,
    /// not a physical error bound.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 锁定自适应律的更新顺序：先用本周期误差更新增益，再用更新后的增益与
    /// 本周期误差计算输出；增益的投影发生在算输出之前。
    /// Pins the update order and that the gain is projected before being used.
    #[test]
    fn adaptive_matches_c_reference() {
        let mut param = AdaptiveParam {
            learning_rate: 1.0,
            ts: 0.1,
            gain_min: 0.0,
            gain_max: 2.0,
            out_min: -10.0,
            out_max: 10.0,
        };
        let mut state = AdaptiveState::new(1.0);
        near(state.update(&param, 2.0, 1.0), 1.2, 1e-6);
        near(state.gain, 1.2, 1e-6);
        param.learning_rate = 100.0;
        state.update(&param, 2.0, 0.0);
        near(state.gain, 2.0, 1e-6);
    }

    /// 锁定虚拟控制 `alpha`、两级误差的定义，以及输出被 `out_min` 饱和的行为。
    /// Pins the virtual control, both error definitions and output saturation.
    #[test]
    fn backstepping_matches_c_reference() {
        let p = BacksteppingParam {
            k1: 2.0,
            k2: 3.0,
            out_min: -10.0,
            out_max: 10.0,
        };
        let mut state = BacksteppingState::default();
        near(state.update(&p, 1.0, 0.0, 0.0, 0.0, 0.0), -6.0, 1e-6);
        near(state.alpha, -2.0, 1e-6);
        near(state.e2, 2.0, 1e-6);
        near(state.update(&p, 10.0, 0.0, 0.0, 0.0, 0.0), -10.0, 1e-6);
    }

    /// 锁定三角隶属度、9 条规则的加权平均，以及 `out_scale` 的施加位置
    /// （先缩放后限幅）。
    /// Pins the membership functions, the weighted average and the scale position.
    #[test]
    fn fuzzy_matches_c_reference() {
        let p = FuzzyParam {
            e_scale: 1.0,
            de_scale: 1.0,
            out_scale: 2.0,
            rules: [[-1.0, -0.5, 0.0], [-0.5, 0.0, 0.5], [0.0, 0.5, 1.0]],
            out_min: -2.0,
            out_max: 2.0,
        };
        let mut state = FuzzyState::default();
        near(state.update(&p, 1.0, 1.0), 2.0, 1e-6);
        near(state.update(&p, 0.0, 0.0), 0.0, 1e-6);
        near(state.update(&p, -1.0, -1.0), -2.0, 1e-6);
    }

    /// 锁定鲁棒补偿项的符号与缩放关系 `-(kw/gamma)*disturbance`。
    /// Pins the sign and scaling of the robust compensation term.
    #[test]
    fn hinf_matches_c_reference() {
        let p = HinfParam {
            kx: 2.0,
            kw: 4.0,
            gamma: 2.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut state = HinfState::default();
        near(state.update(&p, 1.0, 0.5, 1.0), -2.0, 1e-6);
        near(state.robust_term, -1.0, 1e-6);
        near(state.update(&p, 10.0, 0.0, 0.0), -5.0, 1e-6);
    }

    /// 锁定二状态反馈的符号约定 `u = feedforward - k0*x0 - k1*x1`。
    /// Pins the sign convention of the two-state feedback law.
    #[test]
    fn lqr_matches_c_reference() {
        let p = LqrParam {
            k0: 2.0,
            k1: 0.5,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut state = LqrState::default();
        near(state.update(&p, 1.0, 2.0, 0.5), -2.5, 1e-6);
        near(state.update(&p, 10.0, 0.0, 0.0), -5.0, 1e-6);
    }

    /// 锁定候选网格的铺法与二次代价，并锁定 `candidates = 0` 时仍按 2 个候选
    /// 计算的降级行为（结果必须有限，而不是 `NaN`）。
    /// Pins the candidate grid, the quadratic cost and the two-candidate floor.
    #[test]
    fn mpc_matches_c_reference_and_has_two_candidate_floor() {
        let mut p = MpcParam {
            a: 1.0,
            b: 1.0,
            q: 1.0,
            r: 0.0,
            u_min: -2.0,
            u_max: 2.0,
            candidates: 5,
        };
        let mut state = MpcState::default();
        near(state.update(&p, 0.0, 1.0), 1.0, 1e-6);
        near(state.predicted_x, 1.0, 1e-6);
        p.r = 10.0;
        near(state.update(&p, 0.0, 1.0), 0.0, 1e-6);
        p.candidates = 0;
        assert!(state.update(&p, 0.0, 1.0).is_finite());
    }

    /// 锁定参考模型的前向欧拉步进顺序与 `theta` 的投影边界。
    /// Pins the forward-Euler step order and the `theta` projection bounds.
    #[test]
    fn mrac_matches_c_reference() {
        let mut p = MracParam {
            ts: 0.1,
            model_a: 1.0,
            model_b: 2.0,
            gamma: 1.0,
            theta_min: 0.0,
            theta_max: 5.0,
            out_min: -10.0,
            out_max: 10.0,
        };
        let mut state = MracState::new(1.0);
        near(state.update(&p, 1.0, 0.0), 1.02, 1e-6);
        near(state.model_output, 0.2, 1e-6);
        near(state.theta, 1.02, 1e-6);
        p.gamma = 100.0;
        state.update(&p, 10.0, 0.0);
        near(state.theta, 5.0, 1e-6);
    }

    /// 锁定滑模面、边界层饱和与输出限幅的先后关系。
    /// Pins the order of surface, boundary-layer saturation and output clamp.
    #[test]
    fn smc_matches_c_reference() {
        let mut p = SmcParam {
            c: 2.0,
            k: 3.0,
            boundary: 2.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut state = SmcState::default();
        near(state.update(&p, 0.5, 0.0, 1.0), -0.5, 1e-6);
        near(state.surface, 1.0, 1e-6);
        near(state.update(&p, 10.0, 0.0, 0.0), -3.0, 1e-6);
        p.out_min = -2.0;
        near(state.update(&p, 10.0, 0.0, 0.0), -2.0, 1e-6);
    }
}
