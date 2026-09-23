//! 高频注入、旋转高频注入、脉冲注入及 HF/BEMF 融合。

use super::{BemfInput, BemfParam, BemfState};
use crate::math::{
    atan2_angle_0_to_2pi, clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi, TWO_PI,
};
use crate::transform::AlphaBeta;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfInjectionParam {
    pub amplitude: f32,
    pub freq_hz: f32,
    pub ts: f32,
    pub axis_angle_rad: f32,
    pub demod_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfInjectionState {
    pub carrier_angle_rad: f32,
    pub carrier: f32,
    pub voltage: AlphaBeta,
    pub demod_response: f32,
}

impl HfInjectionState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &HfInjectionParam, current_response: f32) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.carrier_angle_rad =
            wrap_angle_0_to_2pi(self.carrier_angle_rad + TWO_PI * param.freq_hz * param.ts);
        self.carrier = libm::sinf(self.carrier_angle_rad);
        self.voltage.alpha = param.amplitude * self.carrier * libm::cosf(param.axis_angle_rad);
        self.voltage.beta = param.amplitude * self.carrier * libm::sinf(param.axis_angle_rad);
        let response = current_response * self.carrier;
        let alpha = clamp(param.demod_alpha, 0.0, 1.0);
        self.demod_response += alpha * (response - self.demod_response);
        self.voltage
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotatingHfParam {
    pub amplitude: f32,
    pub freq_hz: f32,
    pub ts: f32,
    pub demod_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RotatingHfState {
    pub carrier_angle_rad: f32,
    pub voltage: AlphaBeta,
    pub demod: AlphaBeta,
    pub theta_est_rad: f32,
}

impl RotatingHfState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &RotatingHfParam, current_response: AlphaBeta) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.carrier_angle_rad =
            wrap_angle_0_to_2pi(self.carrier_angle_rad + TWO_PI * param.freq_hz * param.ts);
        let c = libm::cosf(self.carrier_angle_rad);
        let s = libm::sinf(self.carrier_angle_rad);
        self.voltage.alpha = param.amplitude * c;
        self.voltage.beta = param.amplitude * s;
        let alpha = clamp(param.demod_alpha, 0.0, 1.0);
        self.demod.alpha += alpha * (current_response.alpha * c - self.demod.alpha);
        self.demod.beta += alpha * (current_response.beta * s - self.demod.beta);
        self.theta_est_rad =
            wrap_angle_0_to_2pi(0.5 * atan2_angle_0_to_2pi(self.demod.beta, self.demod.alpha));
        self.voltage
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PulseInjectionParam {
    pub amplitude: f32,
    pub axis_angle_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PulseInjectionState {
    pub polarity: i32,
    pub voltage: AlphaBeta,
    pub positive_response: f32,
    pub negative_response: f32,
    pub saliency: f32,
}

impl Default for PulseInjectionState {
    fn default() -> Self {
        Self {
            polarity: 1,
            voltage: AlphaBeta::default(),
            positive_response: 0.0,
            negative_response: 0.0,
            saliency: 0.0,
        }
    }
}

impl PulseInjectionState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &PulseInjectionParam, current_response: f32) -> AlphaBeta {
        if self.polarity >= 0 {
            self.positive_response = current_response;
            self.polarity = -1;
        } else {
            self.negative_response = current_response;
            self.polarity = 1;
        }
        self.saliency = self.positive_response - self.negative_response;
        let polarity = self.polarity as f32;
        self.voltage.alpha = param.amplitude * polarity * libm::cosf(param.axis_angle_rad);
        self.voltage.beta = param.amplitude * polarity * libm::sinf(param.axis_angle_rad);
        self.voltage
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfParam {
    pub hf: RotatingHfParam,
    pub bemf: BemfParam,
    pub speed_low_abs: f32,
    pub speed_high_abs: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfInput {
    pub bemf: BemfInput,
    pub hf_current_response: AlphaBeta,
    pub speed_abs: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfBemfState {
    pub hf: RotatingHfState,
    pub bemf: BemfState,
    pub hf_voltage: AlphaBeta,
    pub bemf_emf: AlphaBeta,
    pub theta_hf_rad: f32,
    pub theta_bemf_rad: f32,
    pub theta_rad: f32,
    pub bemf_weight: f32,
}

impl HfBemfState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &HfBemfParam, input: &HfBemfInput) -> f32 {
        self.hf_voltage = self.hf.update(&param.hf, input.hf_current_response);
        self.bemf_emf = self.bemf.update(&param.bemf, &input.bemf);
        self.theta_hf_rad = self.hf.theta_est_rad;
        self.theta_bemf_rad = self.bemf.theta_emf_rad;
        let span = param.speed_high_abs - param.speed_low_abs;
        self.bemf_weight = if span <= 0.0 {
            if input.speed_abs >= param.speed_high_abs {
                1.0
            } else {
                0.0
            }
        } else {
            clamp((input.speed_abs - param.speed_low_abs) / span, 0.0, 1.0)
        };
        let delta = wrap_angle_minus_pi_to_pi(self.theta_bemf_rad - self.theta_hf_rad);
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_hf_rad + self.bemf_weight * delta);
        self.theta_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::PI;

    fn near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn hf_injection_matches_c_reference() {
        let mut param = HfInjectionParam {
            amplitude: 2.0,
            freq_hz: 25.0,
            ts: 0.01,
            axis_angle_rad: 0.0,
            demod_alpha: 1.0,
        };
        let mut state = HfInjectionState::default();
        let voltage = state.update(&param, 3.0);
        near(state.carrier_angle_rad, PI * 0.5, 1e-5);
        near(voltage.alpha, 2.0, 1e-5);
        near(voltage.beta, 0.0, 1e-5);
        near(state.demod_response, 3.0, 1e-5);
        param.axis_angle_rad = PI * 0.5;
        let voltage = state.update(&param, 1.0);
        near(state.carrier, 0.0, 1e-5);
        near(voltage.alpha, 0.0, 1e-5);
    }

    #[test]
    fn rotating_hf_matches_c_reference() {
        let param = RotatingHfParam {
            amplitude: 3.0,
            freq_hz: 25.0,
            ts: 0.01,
            demod_alpha: 1.0,
        };
        let mut state = RotatingHfState::default();
        let voltage = state.update(
            &param,
            AlphaBeta {
                alpha: 4.0,
                beta: 2.0,
            },
        );
        near(state.carrier_angle_rad, PI * 0.5, 1e-5);
        near(voltage.alpha, 0.0, 1e-5);
        near(voltage.beta, 3.0, 1e-5);
        near(state.demod.alpha, 0.0, 1e-5);
        near(state.demod.beta, 2.0, 1e-5);
        near(state.theta_est_rad, PI * 0.25, 1e-5);
    }

    #[test]
    fn pulse_injection_matches_c_reference() {
        let param = PulseInjectionParam {
            amplitude: 2.0,
            axis_angle_rad: 0.0,
        };
        let mut state = PulseInjectionState::default();
        let voltage = state.update(&param, 5.0);
        assert_eq!(state.polarity, -1);
        near(state.positive_response, 5.0, 1e-6);
        near(voltage.alpha, -2.0, 1e-6);
        let voltage = state.update(&param, 3.0);
        assert_eq!(state.polarity, 1);
        near(state.negative_response, 3.0, 1e-6);
        near(state.saliency, 2.0, 1e-6);
        near(voltage.alpha, 2.0, 1e-6);
    }

    #[test]
    fn hf_bemf_matches_c_reference() {
        let param = HfBemfParam {
            hf: RotatingHfParam {
                amplitude: 2.0,
                freq_hz: 100.0,
                ts: 0.001,
                demod_alpha: 1.0,
            },
            bemf: BemfParam {
                rs: 0.0,
                ls: 0.0,
                ts: 0.001,
                emf_filter_alpha: 1.0,
            },
            speed_low_abs: 10.0,
            speed_high_abs: 100.0,
        };
        let mut input = HfBemfInput {
            bemf: BemfInput {
                voltage: AlphaBeta {
                    alpha: -10.0,
                    beta: 0.0,
                },
                current: AlphaBeta::default(),
            },
            hf_current_response: AlphaBeta {
                alpha: 1.0,
                beta: 0.5,
            },
            speed_abs: 0.0,
        };
        let mut state = HfBemfState::default();
        let low = state.update(&param, &input);
        near(state.bemf_weight, 0.0, 1e-6);
        near(low, state.theta_hf_rad, 1e-6);
        assert!(state.hf_voltage.alpha.abs() > 0.0 || state.hf_voltage.beta.abs() > 0.0);
        input.speed_abs = 150.0;
        let high = state.update(&param, &input);
        near(state.bemf_weight, 1.0, 1e-6);
        near(high, state.theta_bemf_rad, 1e-6);
    }
}
