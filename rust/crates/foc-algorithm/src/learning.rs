//! 固定拓扑神经网络与固定容量 Q-learning 表。
//!
//! 职责 / Responsibility:
//!   - 固定拓扑前馈神经网络推理：2 输入 -> 3 个 `tanh` 隐层节点 -> 1 输出。
//!   - 固定容量 8x4 Q 表的 Q-learning：确定性贪心选动作 + 单步 TD 更新。
//!
//! 架构位置 / Position in the architecture:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!   -> foc-algorithm（本文件：芯片无关的纯 `no_std` 数学层）。
//!
//! 依赖方向 / Dependency direction:
//!   - 只依赖 `crate::math::clamp` 与 `libm::tanhf`；无 HAL、无 RTOS、无堆。
//!   - 权重训练、量化、模型签名、奖励设计和探索策略都在本库之外完成。
//!   - 本文件只使用 `f32` 与 `usize`/`i32` 索引，不涉及定点：没有 `i16` Q1.15
//!     或 `i32` Q1.31 的换算。容量常量 8 与 4 的来源见 `RL_MAX_STATES`。
//!   - `#[repr(C)]` 只为稳定布局和后续 C 互操作准备；本 crate 只生成 rlib，
//!     尚未导出 C ABI 符号。
//!
//! 实时性与定位 / Real-time status:
//!   - 默认定位是离线/设计期与低优先级线程：`tanhf` 在目标板上的 WCET 尚未
//!     实测，不应假设可以直接放进 12 kHz ADC 电流环。
//!   - 推理与 Q 更新都是定长循环：不分配内存、不阻塞、不打日志、不等 Mutex。
//!     真要放进快环，必须另行实测 `tanhf` 和整表扫描的最坏时间。
//!
//! 安全边界 / Safety boundary:
//!   - 在线 Q 更新必须被电流、电压、速度和状态机安全包络限制；本库只提供表格
//!     运算，不提供任何安全校验、动作许可或故障关断逻辑。
//!   - 硬件过流、过压、过温和驱动器故障必须由独立保护路径处理，不能依赖
//!     本文件的返回值代替。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 固定拓扑前馈神经网络的配置参数（已训练并量化好的权重）。
/// Configuration of the fixed-topology feed-forward network.
///
/// 拓扑固定为 2 输入 -> 3 个 `tanh` 隐层节点 -> 1 线性输出；这也是
/// `算法库实时性说明.md` 记录"单次推理固定执行 3 次 `tanh`"的原因。
/// Topology is fixed at 2 inputs -> 3 `tanh` hidden nodes -> 1 linear output, which is
/// why a single inference always executes exactly 3 `tanh` calls.
///
/// 范围 / Scope: 本库不做训练、不做量化、不做输入归一化，也不做模型校验；
/// 权重必须来自库外的训练流程，权重有效域也必须在库外确认。
/// No training, quantisation, input normalisation or model validation happens here.
pub struct NeuralNetworkParam {
    /// 输入层到隐层的权重，索引为 `[隐层节点][输入]`，共 3x2 个。
    /// 权重本身无量纲，前提是输入已在库外归一化。
    /// Input-to-hidden weights indexed `[hidden node][input]`; dimensionless only if
    /// the inputs are normalised outside.
    pub w1: [[f32; 2]; 3],
    /// 隐层偏置，共 3 个；它决定各 `tanh` 的工作点，偏置过大会让节点长期饱和。
    /// Hidden biases; they set the `tanh` operating point.
    pub b1: [f32; 3],
    /// 隐层到输出的权重，共 3 个。
    /// Hidden-to-output weights.
    pub w2: [f32; 3],
    /// 输出偏置；输出层是线性的，没有激活函数，所以未训练的权重会直接
    /// 产生大幅输出，只能靠 `out_min`/`out_max` 兜住。
    /// Output bias; the output layer is linear, so the clamp is the only bound.
    pub b2: f32,
    /// 输出下限，单位由被控量决定。
    /// Output lower limit.
    pub out_min: f32,
    /// 输出上限；上下限配反时 `clamp` 会先交换两者。
    /// Output upper limit; `clamp` swaps the bounds instead of misbehaving.
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 神经网络的运行状态：最近一次推理的中间量与输出。
/// Runtime state of the network: the last inference's activations and output.
pub struct NeuralNetworkState {
    /// 最近一次隐层激活值，每个都是 `tanh` 输出，范围 `(-1, 1)`；
    /// 保留它便于诊断"是否所有节点都饱和、网络是否已失去分辨力"。
    /// Last hidden activations in `(-1, 1)`; saturation of all nodes is visible here.
    pub hidden: [f32; 3],
    /// 最近一次限幅后的网络输出。
    /// Last clamped network output.
    pub output: f32,
}

impl NeuralNetworkState {
    /// 清零隐层激活与输出；权重在 `param` 里，不受复位影响。
    /// Clears the activations and the output; the weights live in `param`.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 固定拓扑前向推理一次，返回限幅后的输出。
    /// Runs one forward pass and returns the clamped output.
    ///
    /// 参数 / Parameters:
    ///   input0, input1 - 两个输入特征；必须在库外完成归一化和限幅，
    ///                     本函数不做任何输入校验
    ///                     two features; normalise and clamp them outside
    ///
    /// 返回 / Returns: 限幅到 `[out_min, out_max]` 的线性输出。
    ///
    /// 为什么这样写 / Rationale: 隐层循环固定 3 次迭代，累加顺序为
    /// `b2` -> `w2[0]*h0` -> `w2[1]*h1` -> `w2[2]*h2`；这个顺序与 `libm::tanhf`
    /// 的选择一起决定浮点结果，改动后就不再与 C 参考向量一致
    /// （见 `neural_network_matches_c_reference`）。
    /// The loop count is a compile-time constant, and the exact accumulation order is
    /// what keeps C-reference equivalence.
    ///
    /// 上下文 / Context: 低优先级线程或设计期；放进快环前必须实测 `tanhf` 的 WCET。
    pub fn update(&mut self, param: &NeuralNetworkParam, input0: f32, input1: f32) -> f32 {
        let mut output = param.b2;
        for i in 0..3 {
            let sum = param.w1[i][0] * input0 + param.w1[i][1] * input1 + param.b1[i];
            self.hidden[i] = libm::tanhf(sum);
            output += param.w2[i] * self.hidden[i];
        }
        self.output = clamp(output, param.out_min, param.out_max);
        self.output
    }
}

/// Q 表支持的状态数上界，固定为 8。
/// Upper bound on the number of Q-table states, fixed at 8.
///
/// 容量 8x4 来自原 C 参考库的固定表（`算法库移植状态.md` 记为"已迁移，固定
/// 8x4"）。为什么恰好是 8 和 4，树内文档没有给出依据，也没有按实物辨识过；
/// 改动它等于改变与原 C 版本的行为对照关系。
/// The 8x4 capacity comes from the fixed C reference table. The in-tree documents do
/// not explain why 8 and 4, so treat it as an unidentified fixed capacity.
pub const RL_MAX_STATES: usize = 8;
/// Q 表支持的动作数上界，固定为 4；容量来源同上。
/// Upper bound on the number of Q-table actions, fixed at 4; same provenance note.
pub const RL_MAX_ACTIONS: usize = 4;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 固定容量 Q-learning 的配置参数。
/// Configuration of the fixed-capacity Q-learning.
///
/// `state_count`/`action_count` 只是"有效子表"的大小，物理容量由
/// `RL_MAX_STATES`/`RL_MAX_ACTIONS` 固定；策略是确定性贪心，不含探索随机数。
/// The two counts select an active sub-table inside the fixed physical capacity; the
/// policy is deterministic greedy with no exploration RNG.
pub struct ReinforcementLearningParam {
    /// 有效状态数，必须满足 `0 < state_count <= RL_MAX_STATES`。
    /// 类型是 `i32` 且只在运行时校验：越界不是截断，而是整次更新被拒绝。
    /// Active state count; out-of-range values are rejected rather than truncated.
    pub state_count: i32,
    /// 有效动作数，必须满足 `0 < action_count <= RL_MAX_ACTIONS`，校验方式同上。
    /// Active action count, validated the same way.
    pub action_count: i32,
    /// 学习率 `alpha`，无量纲，工程上取 `(0, 1]`。
    /// 本库不校验范围：`alpha > 1` 会让单步 TD 更新过冲甚至发散。
    /// Learning rate; not validated, and `alpha > 1` can diverge.
    pub alpha: f32,
    /// 折扣因子 `gamma`，无量纲，通常取 `[0, 1)`。
    /// 本库不校验范围：在没有终止状态的表格上持续更新时，`gamma >= 1` 会让
    /// Q 值无界增长。本实现也没有回合结束/终止状态的概念。
    /// Discount factor; not validated. There is no terminal-state handling here.
    pub gamma: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 固定容量 Q 表的运行状态。
/// Runtime state of the fixed-capacity Q-table.
///
/// 占用是定长且可静态核算的：`q` 是 8x4 个 `f32`（128 字节），加 `best_action`
/// （`i32`，4 字节）与 `last_td_error`（`f32`，4 字节），合计 136 字节，
/// 与 `算法库实时性说明.md` 记录一致；结构里没有指针、`Vec` 或 trait object。
/// Fixed footprint: 128 bytes of Q values plus 8 bytes of bookkeeping, matching
/// `算法库实时性说明.md`; no pointers, no `Vec`, no trait objects.
pub struct ReinforcementLearningState {
    /// Q 值表，索引为 `[状态][动作]`，物理容量固定 8x4；
    /// 只有 `state_count`/`action_count` 划定的子表会被读写。
    /// Q-value table indexed `[state][action]`; only the active sub-table is used.
    pub q: [[f32; RL_MAX_ACTIONS]; RL_MAX_STATES],
    /// 最近一次 `select_action` 选出的动作索引；索引非法时保持原值不变。
    /// Last action chosen by `select_action`; untouched when the index is invalid.
    pub best_action: i32,
    /// 最近一次的 TD 误差 `target - Q(s,a)`，量纲同奖励 `reward`。
    /// Last TD error, in the unit of the reward.
    pub last_td_error: f32,
}

/// 校验状态/动作索引是否落在有效子表内。
/// Validates that a state/action index lies inside the active sub-table.
///
/// 这是整张固定数组唯一的边界防线：先要求两个计数都在 `1..=RL_MAX_*` 之内，
/// 再要求索引非负且小于计数。只有全部成立，`q` 的下标才保证不越界，
/// 所以这里的任何一个 `&&` 条件都不能省。
/// This is the only bounds gate for the fixed arrays: the counts must be within
/// capacity and the indices must be non-negative and below those counts.
fn valid_index(param: &ReinforcementLearningParam, state: i32, action: i32) -> bool {
    param.state_count > 0
        && param.state_count <= RL_MAX_STATES as i32
        && param.action_count > 0
        && param.action_count <= RL_MAX_ACTIONS as i32
        && state >= 0
        && state < param.state_count
        && action >= 0
        && action < param.action_count
}

impl ReinforcementLearningState {
    /// 清零整张 Q 表与两个簿记字段（136 字节，一次定长写入，无分配）。
    /// Clears the whole table and both bookkeeping fields; fixed size, no allocation.
    ///
    /// 注意 / Pitfall: 全零初始化下所有动作的 Q 值相等，贪心选择会恒返回动作 0。
    /// 调用方必须先灌入先验值或先做离线训练，否则策略等价于"永远执行动作 0"。
    /// With an all-zero table every action ties, so greedy selection always returns
    /// action 0; seed the table before use.
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 采用与 C 版一致的确定性贪心选择；相等时保留最小动作索引。
    /// Deterministic greedy selection matching the C version; ties keep the lowest
    /// action index.
    ///
    /// 比较用的是严格大于 `>`：Q 值相等时不替换 `best`，因此选择结果可重复，
    /// 不含探索随机数，也不是 ε-greedy。索引非法时返回 0，并且**不修改**
    /// `best_action`。
    /// A strict `>` comparison keeps the choice reproducible; invalid indices return 0
    /// without touching `best_action`.
    ///
    /// 上下文 / Context: 低优先级线程或设计期；代价是定长的 `action_count` 次比较。
    pub fn select_action(&mut self, param: &ReinforcementLearningParam, state_index: i32) -> i32 {
        if !valid_index(param, state_index, 0) {
            return 0;
        }
        let state = state_index as usize;
        let mut best = 0usize;
        let mut best_value = self.q[state][0];
        for action in 1..param.action_count as usize {
            if self.q[state][action] > best_value {
                best_value = self.q[state][action];
                best = action;
            }
        }
        self.best_action = best as i32;
        self.best_action
    }

    /// 执行一次单步 Q-learning 更新，无返回值。
    /// Performs one single-step Q-learning update; nothing is returned.
    ///
    /// 参数 / Parameters:
    ///   state_index      - 当前状态索引
    ///   action_index     - 本次实际执行的动作索引
    ///   reward           - 即时奖励，单位由调用方的奖励设计决定
    ///   next_state_index - 转移后的状态索引
    ///
    /// 语义 / Semantics: `target = reward + gamma * max_a' Q(s',a')`，
    /// `last_td_error = target - Q(s,a)`，最后 `Q(s,a) += alpha * last_td_error`。
    /// `next_state` 的最大值在写 `Q(s,a)` 之前读出，所以即使
    /// `next_state == state` 用的也是更新前的值：这是标准 Q-learning（离线策略），
    /// 不是 SARSA。
    /// The next-state maximum is read before the write, which makes this off-policy
    /// Q-learning rather than SARSA.
    ///
    /// 失败语义 / Failure semantics: `state`、`action`、`next_state` 任一非法就整体
    /// 静默丢弃，不做部分更新，也不会 panic；这是唯一的越界防护，也是唯一的
    /// 输入校验。
    ///
    /// 安全与稳定性 / Safety and stability: 本函数不检查动作是否在电流、电压、
    /// 速度或状态机安全包络内，也不对 Q 值限幅；奖励长期为正时 Q 值可能增长到
    /// `inf`。在线学习必须由调用方的安全包络、奖励设计和定期复位约束。
    /// No safety envelope and no Q clamping here: long-running positive rewards can
    /// drive the values to infinity, so the caller must gate actions and rewards.
    pub fn update(
        &mut self,
        param: &ReinforcementLearningParam,
        state_index: i32,
        action_index: i32,
        reward: f32,
        next_state_index: i32,
    ) {
        if !valid_index(param, state_index, action_index)
            || next_state_index < 0
            || next_state_index >= param.state_count
        {
            return;
        }

        let state = state_index as usize;
        let action = action_index as usize;
        let next_state = next_state_index as usize;
        let mut next_best = self.q[next_state][0];
        for next_action in 1..param.action_count as usize {
            if self.q[next_state][next_action] > next_best {
                next_best = self.q[next_state][next_action];
            }
        }
        let target = reward + param.gamma * next_best;
        self.last_td_error = target - self.q[state][action];
        self.q[state][action] += param.alpha * self.last_td_error;
    }
}

#[cfg(test)]
mod tests {
    // 主机单元测试：不在目标板上运行，也不属于 12 kHz 控制路径。
    // Host-only unit tests; they never run on the target or inside the control ISR.
    //
    // 命名后缀 `_matches_c_reference` 的用例锁定"与原 C 参考实现数值可互换"：
    // 累加顺序、`tanhf` 的实现选择、比较运算是否严格都必须是契约的一部分。
    // The `_matches_c_reference` cases pin numerical interchangeability with the C
    // reference, including accumulation order and strictness of comparisons.
    use super::*;

    /// 近似比较辅助函数，容差由调用点给出；1e-6 是 `f32` 在参考向量量级下的
    /// 比较余量，不是控制精度指标。
    /// Approximate comparison helper; the tolerance is a `f32` rounding allowance,
    /// not a control-accuracy figure.
    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    /// 锁定隐层累加顺序与 `libm::tanhf` 的选择：期望值由测试用同样的 `tanhf`
    /// 自行拼出，因此这里比较的是实现路径而不是近似公式。
    /// Pins the hidden-layer accumulation order and the `libm::tanhf` choice.
    #[test]
    fn neural_network_matches_c_reference() {
        let mut param = NeuralNetworkParam {
            w1: [[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]],
            b1: [0.0; 3],
            w2: [1.0, -1.0, 0.5],
            b2: 0.1,
            out_min: -10.0,
            out_max: 10.0,
        };
        let expected = libm::tanhf(1.0) - libm::tanhf(2.0) + 0.5 * libm::tanhf(3.0) + 0.1;
        let mut state = NeuralNetworkState::default();
        near(state.update(&param, 1.0, 2.0), expected, 1e-6);
        param.out_max = 0.0;
        near(state.update(&param, 1.0, 2.0), 0.0, 1e-6);
    }

    /// 锁定单步 TD 更新的读写顺序（先读下一状态最大值再写当前项）与
    /// `alpha`/`gamma` 的施加位置，以及贪心选择的严格大于语义。
    /// Pins the TD update order, the `alpha`/`gamma` placement and the strict `>`
    /// comparison used by greedy selection.
    #[test]
    fn reinforcement_learning_matches_c_reference() {
        let param = ReinforcementLearningParam {
            state_count: 3,
            action_count: 3,
            alpha: 0.5,
            gamma: 0.9,
        };
        let mut state = ReinforcementLearningState::default();
        state.q[1][2] = 4.0;
        state.update(&param, 0, 1, 1.0, 1);
        near(state.q[0][1], 2.3, 1e-6);
        near(state.last_td_error, 4.6, 1e-6);
        state.q[0][2] = 3.0;
        assert_eq!(state.select_action(&param, 0), 2);
    }

    /// 锁定容量校验的拒绝语义：`state_count` 超过 `RL_MAX_STATES` 时 `update`
    /// 必须整表不动，`select_action` 必须返回 0 而不是越界读取。
    /// Pins the rejection semantics for over-capacity counts: no table change and no
    /// out-of-bounds read.
    #[test]
    fn reinforcement_learning_rejects_out_of_capacity_indices() {
        let invalid = ReinforcementLearningParam {
            state_count: 9,
            action_count: 4,
            alpha: 1.0,
            gamma: 1.0,
        };
        let mut state = ReinforcementLearningState::default();
        state.update(&invalid, 0, 0, 10.0, 0);
        assert_eq!(state, ReinforcementLearningState::default());
        assert_eq!(state.select_action(&invalid, 0), 0);
    }
}
