//! 固定拓扑神经网络与固定容量 Q-learning 表。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NeuralNetworkParam {
    pub w1: [[f32; 2]; 3],
    pub b1: [f32; 3],
    pub w2: [f32; 3],
    pub b2: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NeuralNetworkState {
    pub hidden: [f32; 3],
    pub output: f32,
}

impl NeuralNetworkState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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

pub const RL_MAX_STATES: usize = 8;
pub const RL_MAX_ACTIONS: usize = 4;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReinforcementLearningParam {
    pub state_count: i32,
    pub action_count: i32,
    pub alpha: f32,
    pub gamma: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ReinforcementLearningState {
    pub q: [[f32; RL_MAX_ACTIONS]; RL_MAX_STATES],
    pub best_action: i32,
    pub last_td_error: f32,
}

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
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 采用与 C 版一致的确定性贪心选择；相等时保留最小动作索引。
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
    use super::*;

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

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
