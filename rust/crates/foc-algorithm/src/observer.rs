//! Observer_PLL 的等价实现。

pub mod advanced_smo;
pub mod bemf;
pub mod estimator;
pub mod flux;
pub mod injection;
pub mod smo;

pub use advanced_smo::*;
pub use bemf::*;
pub use estimator::*;
pub use flux::*;
pub use injection::*;
pub use smo::*;

use crate::math::{clamp, wrap_angle_0_to_2pi};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PllParam {
    pub kp: f32,
    pub ki: f32,
    pub ts: f32,
    pub omega_min: f32,
    pub omega_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PllState {
    pub theta_rad: f32,
    pub omega_rad_s: f32,
    pub integrator: f32,
    pub phase_error: f32,
}

impl PllState {
    #[inline]
    pub fn new(initial_theta_rad: f32) -> Self {
        let mut state = Self::default();
        state.reset(initial_theta_rad);
        state
    }

    #[inline]
    pub fn reset(&mut self, theta_rad: f32) {
        self.theta_rad = wrap_angle_0_to_2pi(theta_rad);
        self.omega_rad_s = 0.0;
        self.integrator = 0.0;
        self.phase_error = 0.0;
    }

    #[inline]
    pub fn update(&mut self, param: &PllParam, theta_meas_rad: f32) -> f32 {
        self.phase_error = libm::sinf(theta_meas_rad - self.theta_rad);
        self.integrator += param.ki * param.ts * self.phase_error;
        let proportional = param.kp * self.phase_error;
        let omega = proportional + self.integrator;
        self.omega_rad_s = clamp(omega, param.omega_min, param.omega_max);
        if omega != self.omega_rad_s {
            self.integrator = self.omega_rad_s - proportional;
        }
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_rad + self.omega_rad_s * param.ts);
        self.theta_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::TWO_PI;

    #[test]
    fn matches_c_reference_vectors() {
        let p = PllParam {
            kp: 20.0,
            ki: 0.0,
            ts: 0.001,
            omega_min: -200.0,
            omega_max: 200.0,
        };
        let mut state = PllState::new(0.0);
        let before = state.theta_rad;
        let after = state.update(&p, 0.5);
        assert!(after > before);
        assert!(state.omega_rad_s > 0.0);
        state.reset(TWO_PI - 0.05);
        state.update(&p, 0.05);
        assert!(state.phase_error > 0.0);
    }
}
