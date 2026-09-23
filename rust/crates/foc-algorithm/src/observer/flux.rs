//! 电压模型、定子电流模型、混合磁链与改进积分器。

use crate::math::{atan2_angle_0_to_2pi, clamp};
use crate::transform::{AlphaBeta, Dq};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxParam {
    pub rs: f32,
    pub ts: f32,
    pub leakage: f32,
    pub flux_min: f32,
    pub flux_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxState {
    pub flux: AlphaBeta,
    pub theta_flux_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxInput {
    pub voltage: AlphaBeta,
    pub current: AlphaBeta,
}

impl FluxState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &FluxParam, input: &FluxInput) -> AlphaBeta {
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
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxCurrentParam {
    pub ld: f32,
    pub lq: f32,
    pub flux_pm: f32,
    pub flux_min: f32,
    pub flux_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxCurrentState {
    pub flux_dq: Dq,
    pub theta_flux_rad: f32,
}

impl FluxCurrentState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &FluxCurrentParam, current_dq: Dq) -> Dq {
        self.flux_dq.d = clamp(
            param.ld * current_dq.d + param.flux_pm,
            param.flux_min,
            param.flux_max,
        );
        self.flux_dq.q = clamp(param.lq * current_dq.q, param.flux_min, param.flux_max);
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux_dq.q, self.flux_dq.d);
        self.flux_dq
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxHybridParam {
    pub speed_low: f32,
    pub speed_high: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxHybridState {
    pub flux: AlphaBeta,
    pub weight_voltage: f32,
    pub theta_flux_rad: f32,
}

impl FluxHybridState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(
        &mut self,
        param: &FluxHybridParam,
        flux_voltage: AlphaBeta,
        flux_current: AlphaBeta,
        speed_abs: f32,
    ) -> AlphaBeta {
        let speed = speed_abs.abs();
        let denom = param.speed_high - param.speed_low;
        self.weight_voltage = if denom <= 0.0 {
            if speed >= param.speed_high {
                1.0
            } else {
                0.0
            }
        } else {
            clamp((speed - param.speed_low) / denom, 0.0, 1.0)
        };
        self.flux.alpha = self.weight_voltage * flux_voltage.alpha
            + (1.0 - self.weight_voltage) * flux_current.alpha;
        self.flux.beta = self.weight_voltage * flux_voltage.beta
            + (1.0 - self.weight_voltage) * flux_current.beta;
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxImprovedParam {
    pub rs: f32,
    pub ts: f32,
    pub leakage: f32,
    pub flux_mag_min: f32,
    pub flux_mag_max: f32,
    pub output_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FluxImprovedState {
    pub raw_flux: AlphaBeta,
    pub flux: AlphaBeta,
    pub theta_flux_rad: f32,
}

pub type FluxImprovedInput = FluxInput;

fn limit_vector_magnitude(value: &mut AlphaBeta, min_mag: f32, max_mag: f32) {
    let magnitude = libm::sqrtf(value.alpha * value.alpha + value.beta * value.beta);
    if magnitude <= 0.0 {
        return;
    }
    let mut target = magnitude;
    if max_mag > 0.0 && target > max_mag {
        target = max_mag;
    }
    if min_mag > 0.0 && target < min_mag {
        target = min_mag;
    }
    let scale = target / magnitude;
    value.alpha *= scale;
    value.beta *= scale;
}

impl FluxImprovedState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &FluxImprovedParam, input: &FluxImprovedInput) -> AlphaBeta {
        if param.ts <= 0.0 {
            return AlphaBeta::default();
        }
        self.raw_flux.alpha += param.ts
            * (input.voltage.alpha
                - param.rs * input.current.alpha
                - param.leakage * self.raw_flux.alpha);
        self.raw_flux.beta += param.ts
            * (input.voltage.beta
                - param.rs * input.current.beta
                - param.leakage * self.raw_flux.beta);
        limit_vector_magnitude(&mut self.raw_flux, param.flux_mag_min, param.flux_mag_max);
        let alpha = clamp(param.output_alpha, 0.0, 1.0);
        self.flux.alpha += alpha * (self.raw_flux.alpha - self.flux.alpha);
        self.flux.beta += alpha * (self.raw_flux.beta - self.flux.beta);
        self.theta_flux_rad = atan2_angle_0_to_2pi(self.flux.beta, self.flux.alpha);
        self.flux
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
    fn flux_matches_c_reference() {
        let p = FluxParam {
            rs: 0.5,
            ts: 0.01,
            leakage: 0.0,
            flux_min: -1.0,
            flux_max: 1.0,
        };
        let mut input = FluxInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 1.0,
            },
            current: AlphaBeta {
                alpha: 1.0,
                beta: 0.0,
            },
        };
        let mut state = FluxState::default();
        let out = state.update(&p, &input);
        near(out.alpha, 0.015, 1e-6);
        near(out.beta, 0.01, 1e-6);
        input.voltage.alpha = 1000.0;
        near(state.update(&p, &input).alpha, 1.0, 1e-6);
    }

    #[test]
    fn current_model_matches_c_reference() {
        let p = FluxCurrentParam {
            ld: 0.001,
            lq: 0.002,
            flux_pm: 0.05,
            flux_min: -0.1,
            flux_max: 0.1,
        };
        let mut state = FluxCurrentState::default();
        let out = state.update(&p, Dq { d: 10.0, q: 20.0 });
        near(out.d, 0.06, 1e-6);
        near(out.q, 0.04, 1e-6);
        near(state.update(&p, Dq { d: 1000.0, q: 20.0 }).d, 0.1, 1e-6);
    }

    #[test]
    fn hybrid_matches_c_reference() {
        let p = FluxHybridParam {
            speed_low: 100.0,
            speed_high: 300.0,
        };
        let voltage = AlphaBeta {
            alpha: 1.0,
            beta: 0.0,
        };
        let current = AlphaBeta {
            alpha: 0.0,
            beta: 1.0,
        };
        let mut state = FluxHybridState::default();
        assert_eq!(state.update(&p, voltage, current, 50.0), current);
        let mid = state.update(&p, voltage, current, 200.0);
        near(mid.alpha, 0.5, 1e-6);
        near(mid.beta, 0.5, 1e-6);
        near(state.theta_flux_rad, PI * 0.25, 1e-6);
        assert_eq!(state.update(&p, voltage, current, 400.0), voltage);
    }

    #[test]
    fn improved_integrator_matches_c_reference() {
        let p = FluxImprovedParam {
            rs: 0.5,
            ts: 0.01,
            leakage: 0.0,
            flux_mag_min: 0.0,
            flux_mag_max: 0.02,
            output_alpha: 0.5,
        };
        let mut input = FluxImprovedInput {
            voltage: AlphaBeta {
                alpha: 2.0,
                beta: 0.0,
            },
            current: AlphaBeta::default(),
        };
        let mut state = FluxImprovedState::default();
        let out = state.update(&p, &input);
        near(state.raw_flux.alpha, 0.02, 1e-6);
        near(out.alpha, 0.01, 1e-6);
        input.voltage.alpha = 100.0;
        let out = state.update(&p, &input);
        near(state.raw_flux.alpha, 0.02, 1e-6);
        near(out.alpha, 0.015, 1e-6);
    }
}
