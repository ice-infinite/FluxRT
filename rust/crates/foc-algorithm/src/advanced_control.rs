//! 自适应、反步、模糊、H∞、LQR、MPC、MRAC 与滑模控制器。

use crate::math::clamp;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveParam {
    pub learning_rate: f32,
    pub ts: f32,
    pub gain_min: f32,
    pub gain_max: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveState {
    pub gain: f32,
    pub error: f32,
    pub output: f32,
}

impl AdaptiveState {
    pub fn new(initial_gain: f32) -> Self {
        Self {
            gain: initial_gain,
            ..Self::default()
        }
    }

    pub fn reset(&mut self, gain: f32) {
        *self = Self::new(gain);
    }

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
pub struct BacksteppingParam {
    pub k1: f32,
    pub k2: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BacksteppingState {
    pub e1: f32,
    pub e2: f32,
    pub alpha: f32,
    pub output: f32,
}

impl BacksteppingState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
pub struct FuzzyParam {
    pub e_scale: f32,
    pub de_scale: f32,
    pub out_scale: f32,
    pub rules: [[f32; 3]; 3],
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FuzzyState {
    pub output: f32,
    pub weight_sum: f32,
}

fn fuzzy_membership(x: f32) -> [f32; 3] {
    let value = clamp(x, -1.0, 1.0);
    [
        if value < 0.0 { -value } else { 0.0 },
        1.0 - value.abs(),
        if value > 0.0 { value } else { 0.0 },
    ]
}

impl FuzzyState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
pub struct HinfParam {
    pub kx: f32,
    pub kw: f32,
    pub gamma: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HinfState {
    pub u: f32,
    pub robust_term: f32,
}

impl HinfState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
pub struct LqrParam {
    pub k0: f32,
    pub k1: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LqrState {
    pub u: f32,
}

impl LqrState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
pub struct MpcParam {
    pub a: f32,
    pub b: f32,
    pub q: f32,
    pub r: f32,
    pub u_min: f32,
    pub u_max: f32,
    pub candidates: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MpcState {
    pub u: f32,
    pub predicted_x: f32,
    pub cost: f32,
}

impl MpcState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// 穷举次数由 `candidates` 决定；实时应用必须在配置阶段限制其上界。
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
pub struct MracParam {
    pub ts: f32,
    pub model_a: f32,
    pub model_b: f32,
    pub gamma: f32,
    pub theta_min: f32,
    pub theta_max: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MracState {
    pub model_output: f32,
    pub theta: f32,
    pub error: f32,
    pub output: f32,
}

impl MracState {
    pub fn new(initial_theta: f32) -> Self {
        Self {
            theta: initial_theta,
            ..Self::default()
        }
    }

    pub fn reset(&mut self, theta: f32) {
        *self = Self::new(theta);
    }

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
pub struct SmcParam {
    pub c: f32,
    pub k: f32,
    pub boundary: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmcState {
    pub surface: f32,
    pub switching: f32,
    pub output: f32,
}

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
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &SmcParam, error: f32, error_dot: f32, equivalent: f32) -> f32 {
        self.surface = param.c * error + error_dot;
        self.switching = -param.k * smc_saturate(self.surface, param.boundary);
        self.output = clamp(equivalent + self.switching, param.out_min, param.out_max);
        self.output
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
