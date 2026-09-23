//! 滑模观测器、SMO+PLL 与磁链/SMO 融合观测器。

use super::{FluxInput, FluxParam, FluxState, PllParam, PllState};
use crate::math::{atan2_angle_0_to_2pi, clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi};
use crate::transform::AlphaBeta;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoParam {
    pub rs: f32,
    pub ls: f32,
    pub ts: f32,
    pub k_slide: f32,
    pub boundary: f32,
    pub emf_filter_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoState {
    pub current_est: AlphaBeta,
    pub sliding: AlphaBeta,
    pub emf: AlphaBeta,
    pub theta_emf_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

fn sliding_saturation(value: f32, boundary: f32) -> f32 {
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

impl SmoState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Updates the observer vector without calculating its angle. Hard
    /// realtime callers can feed the returned vector to an MCU CORDIC instead
    /// of paying for a software `atan2f` on every PWM sample.
    pub fn update_vector(&mut self, param: &SmoParam, input: &SmoInput) -> AlphaBeta {
        if param.ls <= 0.0 {
            return AlphaBeta::default();
        }
        let inv_l = 1.0 / param.ls;
        let alpha = clamp(param.emf_filter_alpha, 0.0, 1.0);
        let err_alpha = self.current_est.alpha - input.current.alpha;
        let err_beta = self.current_est.beta - input.current.beta;
        self.sliding.alpha = param.k_slide * sliding_saturation(err_alpha, param.boundary);
        self.sliding.beta = param.k_slide * sliding_saturation(err_beta, param.boundary);
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
        self.emf
    }

    pub fn update(&mut self, param: &SmoParam, input: &SmoInput) -> AlphaBeta {
        let emf = self.update_vector(param, input);
        self.theta_emf_rad = atan2_angle_0_to_2pi(-self.emf.alpha, self.emf.beta);
        emf
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoPllParam {
    pub smo: SmoParam,
    pub pll: PllParam,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SmoPllState {
    pub smo: SmoState,
    pub pll: PllState,
    pub emf: AlphaBeta,
    pub theta_rad: f32,
    pub omega_rad_s: f32,
}

impl SmoPllState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &SmoPllParam, input: &SmoInput) -> f32 {
        self.emf = self.smo.update(&param.smo, input);
        self.theta_rad = self.pll.update(&param.pll, self.smo.theta_emf_rad);
        self.omega_rad_s = self.pll.omega_rad_s;
        self.theta_rad
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxSmoParam {
    pub flux: FluxParam,
    pub smo: SmoParam,
    pub smo_weight: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxSmoState {
    pub flux: FluxState,
    pub smo: SmoState,
    pub flux_vector: AlphaBeta,
    pub emf: AlphaBeta,
    pub theta_flux_rad: f32,
    pub theta_smo_rad: f32,
    pub theta_rad: f32,
}

impl FluxSmoState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &FluxSmoParam, input: &SmoInput) -> f32 {
        self.flux_vector = self.flux.update(
            &param.flux,
            &FluxInput {
                voltage: input.voltage,
                current: input.current,
            },
        );
        self.emf = self.smo.update(&param.smo, input);
        self.theta_flux_rad = self.flux.theta_flux_rad;
        self.theta_smo_rad = self.smo.theta_emf_rad;
        let weight = clamp(param.smo_weight, 0.0, 1.0);
        let delta = wrap_angle_minus_pi_to_pi(self.theta_smo_rad - self.theta_flux_rad);
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_flux_rad + weight * delta);
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
    fn smo_matches_c_reference() {
        let param = SmoParam {
            rs: 0.5,
            ls: 0.001,
            ts: 0.0001,
            k_slide: 4.0,
            boundary: 0.1,
            emf_filter_alpha: 0.5,
        };
        let input = SmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = SmoState::default();
        let emf = state.update(&param, &input);
        near(emf.alpha, -2.0, 1e-6);
        near(emf.beta, 2.0, 1e-6);
        assert!((0.0..crate::math::TWO_PI).contains(&state.theta_emf_rad));
    }

    #[test]
    fn smo_pll_matches_c_reference() {
        let param = SmoPllParam {
            smo: SmoParam {
                rs: 0.5,
                ls: 0.001,
                ts: 0.0001,
                k_slide: 4.0,
                boundary: 0.1,
                emf_filter_alpha: 1.0,
            },
            pll: PllParam {
                kp: 20.0,
                ki: 0.0,
                ts: 0.001,
                omega_min: -200.0,
                omega_max: 200.0,
            },
        };
        let input = SmoInput {
            voltage: AlphaBeta::default(),
            current: AlphaBeta {
                alpha: 1.0,
                beta: -1.0,
            },
        };
        let mut state = SmoPllState::default();
        assert!(state.update(&param, &input) > 0.0);
        near(state.emf.alpha, -4.0, 1e-6);
        near(state.emf.beta, 4.0, 1e-6);
        assert!(state.omega_rad_s > 0.0);
    }

    #[test]
    fn flux_smo_matches_c_reference() {
        let param = FluxSmoParam {
            flux: FluxParam {
                rs: 0.0,
                ts: 0.01,
                leakage: 0.0,
                flux_min: -10.0,
                flux_max: 10.0,
            },
            smo: SmoParam {
                rs: 0.2,
                ls: 0.001,
                ts: 0.0001,
                k_slide: 2.0,
                boundary: 0.1,
                emf_filter_alpha: 1.0,
            },
            smo_weight: 0.25,
        };
        let input = SmoInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 1.0,
            },
            current: AlphaBeta {
                alpha: 1.0,
                beta: 0.0,
            },
        };
        let mut state = FluxSmoState::default();
        let theta = state.update(&param, &input);
        near(state.flux_vector.alpha, 0.02, 1e-6);
        near(state.flux_vector.beta, 0.01, 1e-6);
        assert!(state.emf.alpha.abs() > 0.0);
        assert!((0.0..crate::math::TWO_PI).contains(&theta));
        assert!((theta - state.theta_flux_rad).abs() > 1e-5);
    }
}
