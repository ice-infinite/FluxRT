//! 反电势、反电势积分、过零检测及 BEMF+PLL 组合观测器。

use super::{PllParam, PllState};
use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::AlphaBeta;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfParam {
    pub rs: f32,
    pub ls: f32,
    pub ts: f32,
    pub emf_filter_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfState {
    pub last_current: AlphaBeta,
    pub emf: AlphaBeta,
    pub theta_emf_rad: f32,
    pub initialized: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

impl BemfState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &BemfParam, input: &BemfInput) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        if self.initialized == 0 {
            self.last_current = input.current;
            self.initialized = 1;
        }
        let di_alpha = (input.current.alpha - self.last_current.alpha) / param.ts;
        let di_beta = (input.current.beta - self.last_current.beta) / param.ts;
        let raw = AlphaBeta {
            alpha: input.voltage.alpha - param.rs * input.current.alpha - param.ls * di_alpha,
            beta: input.voltage.beta - param.rs * input.current.beta - param.ls * di_beta,
        };
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        self.emf.alpha += alpha * (raw.alpha - self.emf.alpha);
        self.emf.beta += alpha * (raw.beta - self.emf.beta);
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        self.last_current = input.current;
        self.emf
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfIntegralParam {
    pub rs: f32,
    pub ts: f32,
    pub leakage: f32,
    pub flux_min: f32,
    pub flux_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfIntegralState {
    pub flux: AlphaBeta,
    pub theta_rad: f32,
}

pub type BemfIntegralInput = BemfInput;

impl BemfIntegralState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfZeroCrossParam {
    pub threshold: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfZeroCrossState {
    pub last_sign: i32,
    pub crossing: i32,
    pub direction: i32,
    pub last_bemf: f32,
}

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

impl BemfZeroCrossState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfPllParam {
    pub bemf: BemfParam,
    pub pll: PllParam,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BemfPllState {
    pub bemf: BemfState,
    pub pll: PllState,
    pub emf: AlphaBeta,
    pub theta_rad: f32,
    pub omega_rad_s: f32,
}

impl BemfPllState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

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

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

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
