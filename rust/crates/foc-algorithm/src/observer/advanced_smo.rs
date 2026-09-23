//! 自适应、高阶和超扭曲滑模观测器。

use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::AlphaBeta;

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

fn sign_zero(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoParam {
    pub rs: f32,
    pub ls: f32,
    pub ts: f32,
    pub k_initial: f32,
    pub k_min: f32,
    pub k_max: f32,
    pub adapt_rate: f32,
    pub target_error: f32,
    pub boundary: f32,
    pub emf_filter_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoState {
    pub current_est: AlphaBeta,
    pub sliding: AlphaBeta,
    pub emf: AlphaBeta,
    pub k_slide: f32,
    pub error_abs: f32,
    pub theta_emf_rad: f32,
    pub initialized: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdaptiveSmoInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

impl AdaptiveSmoState {
    pub fn new(param: &AdaptiveSmoParam) -> Self {
        let mut state = Self::default();
        state.reset(param);
        state
    }

    pub fn reset(&mut self, param: &AdaptiveSmoParam) {
        *self = Self {
            k_slide: clamp(param.k_initial, param.k_min, param.k_max),
            initialized: 1,
            ..Self::default()
        };
    }

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
        self.error_abs = libm::sqrtf(err_alpha * err_alpha + err_beta * err_beta);
        self.k_slide += param.adapt_rate * (self.error_abs - param.target_error) * param.ts;
        self.k_slide = clamp(self.k_slide, param.k_min, param.k_max);
        self.sliding.alpha = self.k_slide * saturation(err_alpha, param.boundary);
        self.sliding.beta = self.k_slide * saturation(err_beta, param.boundary);
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoParam {
    pub rs: f32,
    pub ls: f32,
    pub ts: f32,
    pub k_slide: f32,
    pub k_dot: f32,
    pub boundary: f32,
    pub dot_boundary: f32,
    pub emf_filter_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoState {
    pub current_est: AlphaBeta,
    pub last_error: AlphaBeta,
    pub error_dot: AlphaBeta,
    pub sliding: AlphaBeta,
    pub emf: AlphaBeta,
    pub theta_emf_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HigherOrderSmoInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

impl HigherOrderSmoState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
        self.error_dot.alpha = (error.alpha - self.last_error.alpha) / param.ts;
        self.error_dot.beta = (error.beta - self.last_error.beta) / param.ts;
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
        self.last_error = error;
        self.emf
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoParam {
    pub rs: f32,
    pub ls: f32,
    pub ts: f32,
    pub k1: f32,
    pub k2: f32,
    pub emf_filter_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoState {
    pub current_est: AlphaBeta,
    pub integral: AlphaBeta,
    pub sliding: AlphaBeta,
    pub emf: AlphaBeta,
    pub theta_emf_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SuperTwistingSmoInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

impl SuperTwistingSmoState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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
        self.integral.alpha += param.k2 * sign_alpha * param.ts;
        self.integral.beta += param.k2 * sign_beta * param.ts;
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

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

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
