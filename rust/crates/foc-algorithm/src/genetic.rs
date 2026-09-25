//! 单变量遗传算法辅助函数。
//!
//! 职责 / Responsibility:
//!   - 单个标量基因的算术交叉 + 加性变异（`crossover_mutate`）。
//!   - 只维护"当前最优标量 + 最优适应度"两个字段，不持有种群。
//!
//! 架构位置 / Position in the architecture:
//!   applications/ (C + RT-Thread) -> foc-rt-bridge (C ABI) -> foc-control
//!   -> foc-algorithm（本文件：芯片无关的纯 `no_std` 数学层）。
//!
//! 依赖方向 / Dependency direction:
//!   - 只依赖 `crate::math::clamp`；没有随机数源、没有迭代与评估循环、
//!     没有种群容器，所以调用方要自己提供变异信号并驱动选择与淘汰。
//!   - 本文件只使用 `f32`，不涉及定点：没有 `i16` Q1.15 或 `i32` Q1.31 的换算；
//!     交叉权值 `0.5` 是算法定义的一部分，不是硬件或 SDK 标定值。
//!   - `#[repr(C)]` 只为稳定布局和后续 C 互操作准备；本 crate 只生成 rlib，
//!     尚未导出 C ABI 符号。
//!
//! 实时性与定位 / Real-time status:
//!   - 仅用于离线整定、参数搜索和仿真；禁止放进 12 kHz 控制 ISR。
//!   - 适应度评估通常本身就是高开销的仿真或实机试探，必须在低优先级上下文
//!     完成，绝不能在控制中断里做。
//!   - `mutation_signal` 必须由调用方提供（`算法库移植状态.md` 记作"已迁移，
//!     外部变异信号"）：本库不绑定 RNG，随机源质量与重复性由调用方负责。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 单变量遗传算法的配置参数。
/// Configuration of the single-variable genetic algorithm.
pub struct GeneticAlgorithmParam {
    /// 基因下界，与被整定参数同单位；交叉变异的结果会被钳位到此边界。
    /// Lower bound of the gene; the child is clamped to it.
    pub value_min: f32,
    /// 基因上界；上下限配反时 `clamp` 会先交换两者。
    /// Upper bound of the gene; `clamp` swaps the bounds instead of misbehaving.
    pub value_max: f32,
    /// 变异幅度，与基因同单位；它乘以调用方给的 `mutation_signal` 才是实际偏移量。
    /// `mutation_signal` 应是无量纲的（例如 `[-1, 1]` 上的均匀采样），否则实际
    /// 步长会被额外缩放；若传标准正态，约 68% 的样本落在 ±1 个 `mutation_step`。
    /// Mutation magnitude; the caller's signal must be dimensionless or the effective
    /// step is rescaled.
    pub mutation_step: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// 遗传算法的运行状态：只记录当前最优候选。
/// Runtime state: only the current best candidate is kept.
pub struct GeneticAlgorithmState {
    /// 当前最优基因值，单位与被整定参数相同。
    /// Best gene value found so far.
    pub best_value: f32,
    /// 当前最优适应度，单位由调用方的适应度函数定义；本库只做"越大越好"的比较。
    /// Best fitness so far; the library always maximises.
    pub best_fitness: f32,
}

impl GeneticAlgorithmState {
    /// 用给定的初始最优值与初始适应度建立状态。
    /// Creates the state with an explicit initial best value and fitness.
    ///
    /// 初值不做区间钳位：调用方要保证传入的初值本身合法，否则"最优值"会一开始
    /// 就落在 `value_min`/`value_max` 之外。
    /// The initial value is not clamped to the configured range.
    pub fn new(initial_value: f32, initial_fitness: f32) -> Self {
        Self {
            best_value: initial_value,
            best_fitness: initial_fitness,
        }
    }

    /// 等价于用新的初值重建状态，用于开始新一轮搜索。
    /// Same as rebuilding the state to start a new search round.
    pub fn reset(&mut self, value: f32, fitness: f32) {
        *self = Self::new(value, fitness);
    }

    /// 用一个新的候选更新"当前最优"记录。
    /// Feeds one new candidate into the best-so-far bookkeeping.
    ///
    /// 参数 / Parameters:
    ///   candidate_value   - 候选基因值，单位同 `best_value`
    ///   candidate_fitness - 候选适应度，单位同 `best_fitness`
    ///
    /// 语义 / Semantics: 只有适应度**严格大于**当前最优时才替换，相等时不替换；
    /// 因此重复提交同一批候选是幂等的，最优值不会来回跳动。
    /// 适应度按"越大越好"解释：若目标是最小化，调用方必须自己取负。
    /// Only a strictly larger fitness replaces the incumbent, which makes repeated
    /// submissions idempotent; minimise by negating the fitness outside.
    ///
    /// 上下文 / Context: 离线/设计期；本函数无分配、无循环。
    pub fn apply_candidate(&mut self, candidate_value: f32, candidate_fitness: f32) {
        if candidate_fitness > self.best_fitness {
            self.best_fitness = candidate_fitness;
            self.best_value = candidate_value;
        }
    }
}

/// `mutation_signal` 由调用方提供，算法层不依赖随机数源。
/// `mutation_signal` is supplied by the caller; this layer has no RNG.
///
/// 交叉是算术平均（权重固定 0.5，没有随机交叉点），变异是加性的
/// （没有乘性变异、也没有边界反射），最后统一钳位到 `value_min`/`value_max`。
/// Crossover is a fixed 0.5-weighted arithmetic mean, mutation is additive, and the
/// child is clamped afterwards.
///
/// 为什么这样写 / Rationale: 两个父代都贴在 `value_max` 且 `mutation_signal > 0`
/// 时，变异会被钳位吃掉，子代停在边界上，算法可能因此停滞。这是与原 C 参考
/// 向量一致的行为（见 `genetic_algorithm_matches_c_reference` 的第二个断言），
/// 不是可以顺手"修好"的缺陷。
/// Clamping can swallow the mutation at a bound; that is intentional C-equivalent
/// behaviour, not a defect to be patched here.
///
/// 参数 / Parameters:
///   parent_a, parent_b - 两个父代基因值，与被整定参数同单位
///                        the two parents, in the tuned parameter's unit
///   mutation_signal    - 无量纲变异信号，由调用方提供
///                        dimensionless mutation signal from the caller
///
/// 返回 / Returns: 钳位后的子代基因值，单位同父代。
/// The clamped child gene, in the unit of the parents.
///
/// 上下文 / Context: 仅离线/设计期使用，禁止放进控制 ISR。
/// Offline/design-time only; never call this from the control ISR.
pub fn crossover_mutate(
    param: &GeneticAlgorithmParam,
    parent_a: f32,
    parent_b: f32,
    mutation_signal: f32,
) -> f32 {
    let child = 0.5 * (parent_a + parent_b) + param.mutation_step * mutation_signal;
    clamp(child, param.value_min, param.value_max)
}

#[cfg(test)]
mod tests {
    // 主机单元测试：锁定与原 C 参考向量的一致性，重点在边界钳位行为。
    // Host-only test pinning C-reference agreement, especially at the bounds.
    use super::*;

    /// 近似比较辅助函数；容差是 `f32` 舍入余量，不是搜索精度指标。
    /// Approximate comparison helper; the tolerance is a `f32` rounding allowance.
    fn near(actual: f32, expected: f32) {
        assert!((actual - expected).abs() <= 1e-6, "{actual} != {expected}");
    }

    /// 锁定交叉权值 0.5、变异为加性偏移，以及两种情况下的钳位：
    /// 正常范围内的截断，和两父代都在上界时的边界停滞。
    /// Pins the 0.5 crossover weight, additive mutation and both clamp outcomes.
    #[test]
    fn genetic_algorithm_matches_c_reference() {
        let param = GeneticAlgorithmParam {
            value_min: 0.0,
            value_max: 10.0,
            mutation_step: 2.0,
        };
        let mut state = GeneticAlgorithmState::new(1.0, 0.5);
        near(crossover_mutate(&param, 2.0, 4.0, 0.5), 4.0);
        near(crossover_mutate(&param, 10.0, 10.0, 2.0), 10.0);
        state.apply_candidate(3.0, 0.4);
        near(state.best_value, 1.0);
        state.apply_candidate(5.0, 0.8);
        near(state.best_value, 5.0);
        near(state.best_fitness, 0.8);
    }
}
