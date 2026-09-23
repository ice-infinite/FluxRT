//! 单变量遗传算法辅助函数。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneticAlgorithmParam {
    pub value_min: f32,
    pub value_max: f32,
    pub mutation_step: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneticAlgorithmState {
    pub best_value: f32,
    pub best_fitness: f32,
}

impl GeneticAlgorithmState {
    pub fn new(initial_value: f32, initial_fitness: f32) -> Self {
        Self {
            best_value: initial_value,
            best_fitness: initial_fitness,
        }
    }

    pub fn reset(&mut self, value: f32, fitness: f32) {
        *self = Self::new(value, fitness);
    }

    pub fn apply_candidate(&mut self, candidate_value: f32, candidate_fitness: f32) {
        if candidate_fitness > self.best_fitness {
            self.best_fitness = candidate_fitness;
            self.best_value = candidate_value;
        }
    }
}

/// `mutation_signal` 由调用方提供，算法层不依赖随机数源。
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
    use super::*;

    fn near(actual: f32, expected: f32) {
        assert!((actual - expected).abs() <= 1e-6, "{actual} != {expected}");
    }

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
