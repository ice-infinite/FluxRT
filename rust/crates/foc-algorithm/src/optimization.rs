//! MTPA、MTPV、V/f 与弱磁控制。

use crate::math::{clamp, wrap_angle_0_to_2pi, TWO_PI};
use crate::transform::Dq;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpaParam {
    pub flux_pm: f32,
    pub ld: f32,
    pub lq: f32,
    pub search_steps: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpaState {
    pub torque_score: f32,
}

impl MtpaState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &MtpaParam, current_mag: f32) -> Dq {
        let abs_current = current_mag.abs();
        let mut output = Dq {
            d: 0.0,
            q: abs_current,
        };
        let delta_l = param.ld - param.lq;
        if abs_current <= 0.0 || delta_l.abs() < 1e-9 {
            self.torque_score = param.flux_pm * output.q;
            return output;
        }

        let steps = if param.search_steps < 8 {
            32
        } else {
            param.search_steps
        } as usize;
        let mut best_score = -1.0e30;
        for i in 0..=steps {
            let ratio = i as f32 / steps as f32;
            let id = -abs_current * ratio;
            let iq = libm::sqrtf((abs_current * abs_current - id * id).max(0.0));
            let score = (param.flux_pm + delta_l * id) * iq;
            if score > best_score {
                best_score = score;
                output = Dq { d: id, q: iq };
            }
        }
        self.torque_score = best_score;
        if current_mag < 0.0 {
            output.q = -output.q;
        }
        output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpvParam {
    pub flux_pm: f32,
    pub ld: f32,
    pub lq: f32,
    pub current_limit: f32,
    pub voltage_limit: f32,
    pub search_steps: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MtpvState {
    pub voltage_mag: f32,
    pub torque_score: f32,
    pub valid: i32,
}

impl MtpvState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &MtpvParam, omega_e_rad_s: f32, iq_sign: f32) -> Dq {
        let current_limit = param.current_limit.abs();
        let omega_abs = omega_e_rad_s.abs();
        let sign = if iq_sign < 0.0 { -1.0 } else { 1.0 };
        let steps = if param.search_steps < 8 {
            64
        } else {
            param.search_steps
        } as usize;
        if current_limit <= 0.0 || param.voltage_limit <= 0.0 {
            self.reset();
            return Dq::default();
        }

        // C 版没有在每次搜索前清除此标志，可能把上一次结果误报为有效。
        self.valid = 0;
        let mut output = Dq::default();
        let mut best_score = -1.0e30;
        let mut best_voltage = 0.0;
        for i in 0..=steps {
            let ratio = i as f32 / steps as f32;
            let id = -current_limit * ratio;
            let iq = sign * libm::sqrtf((current_limit * current_limit - id * id).max(0.0));
            let vd = -omega_abs * param.lq * iq;
            let vq = omega_abs * (param.ld * id + param.flux_pm);
            let voltage_mag = libm::sqrtf(vd * vd + vq * vq);
            let score = (param.flux_pm + (param.ld - param.lq) * id) * iq.abs();
            if voltage_mag <= param.voltage_limit && score > best_score {
                best_score = score;
                best_voltage = voltage_mag;
                output = Dq { d: id, q: iq };
                self.valid = 1;
            }
        }
        if self.valid == 0 {
            output = Dq {
                d: -current_limit,
                q: 0.0,
            };
            best_voltage = omega_abs * (param.flux_pm - param.ld * current_limit).abs();
            best_score = 0.0;
        }
        self.voltage_mag = best_voltage;
        self.torque_score = best_score;
        output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VfParam {
    pub ts: f32,
    pub v_per_hz: f32,
    pub v_min: f32,
    pub v_max: f32,
    pub freq_min: f32,
    pub freq_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VfState {
    pub theta_rad: f32,
    pub voltage: f32,
    pub freq_hz: f32,
}

pub type VfOutput = VfState;

impl VfState {
    pub fn new(theta_rad: f32) -> Self {
        Self {
            theta_rad: wrap_angle_0_to_2pi(theta_rad),
            ..Self::default()
        }
    }

    pub fn reset(&mut self, theta_rad: f32) {
        *self = Self::new(theta_rad);
    }

    pub fn update(&mut self, param: &VfParam, freq_hz: f32) -> VfOutput {
        if param.ts <= 0.0 {
            return VfOutput::default();
        }
        let limited_freq = clamp(freq_hz, param.freq_min, param.freq_max);
        let voltage = param.v_min + param.v_per_hz * limited_freq.abs();
        self.freq_hz = limited_freq;
        self.voltage = clamp(voltage, 0.0, param.v_max);
        self.theta_rad = wrap_angle_0_to_2pi(self.theta_rad + TWO_PI * limited_freq * param.ts);
        *self
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeakeningParam {
    pub kp: f32,
    pub voltage_limit: f32,
    pub id_min: f32,
    pub id_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WeakeningState {
    pub id_ref: f32,
    pub voltage_mag: f32,
    pub voltage_error: f32,
}

impl WeakeningState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &WeakeningParam, voltage_dq: Dq, base_id_ref: f32) -> f32 {
        self.voltage_mag = libm::sqrtf(voltage_dq.d * voltage_dq.d + voltage_dq.q * voltage_dq.q);
        self.voltage_error = self.voltage_mag - param.voltage_limit;
        let mut id_ref = base_id_ref;
        if self.voltage_error > 0.0 {
            id_ref -= param.kp * self.voltage_error;
        }
        self.id_ref = clamp(id_ref, param.id_min, param.id_max).min(0.0);
        self.id_ref
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
    fn mtpa_matches_c_reference() {
        let spm = MtpaParam {
            flux_pm: 0.05,
            ld: 0.001,
            lq: 0.001,
            search_steps: 32,
        };
        let ipm = MtpaParam {
            flux_pm: 0.02,
            ld: 0.0005,
            lq: 0.0015,
            search_steps: 128,
        };
        let mut state = MtpaState::default();
        assert_eq!(state.update(&spm, 10.0), Dq { d: 0.0, q: 10.0 });
        let out = state.update(&ipm, 10.0);
        assert!(out.d < 0.0);
        near(libm::sqrtf(out.d * out.d + out.q * out.q), 10.0, 1e-4);
        assert!(state.update(&ipm, -10.0).q < 0.0);
    }

    #[test]
    fn mtpv_matches_c_reference_and_clears_stale_valid() {
        let mut param = MtpvParam {
            flux_pm: 0.02,
            ld: 0.0005,
            lq: 0.0015,
            current_limit: 10.0,
            voltage_limit: 20.0,
            search_steps: 128,
        };
        let mut state = MtpvState::default();
        let out = state.update(&param, 1000.0, 1.0);
        assert_eq!(state.valid, 1);
        assert!(state.voltage_mag <= param.voltage_limit + 1e-5);
        assert!(libm::sqrtf(out.d * out.d + out.q * out.q) <= param.current_limit + 1e-5);
        assert!(state.update(&param, 1000.0, -1.0).q <= 0.0);
        param.voltage_limit = 0.1;
        let fallback = state.update(&param, 1000.0, 1.0);
        near(fallback.q, 0.0, 1e-6);
        assert_eq!(state.valid, 0);
        near(fallback.d, -10.0, 1e-6);
    }

    #[test]
    fn vf_matches_c_reference() {
        let p = VfParam {
            ts: 0.01,
            v_per_hz: 0.2,
            v_min: 1.0,
            v_max: 10.0,
            freq_min: -50.0,
            freq_max: 50.0,
        };
        let mut state = VfState::default();
        let out = state.update(&p, 10.0);
        near(out.voltage, 3.0, 1e-6);
        near(out.theta_rad, TWO_PI * 0.1, 1e-5);
        let out = state.update(&p, 100.0);
        near(out.freq_hz, 50.0, 1e-6);
        near(out.voltage, 10.0, 1e-6);
    }

    #[test]
    fn weakening_matches_c_reference() {
        let mut p = WeakeningParam {
            kp: 0.1,
            voltage_limit: 10.0,
            id_min: -5.0,
            id_max: 0.0,
        };
        let mut state = WeakeningState::default();
        near(state.update(&p, Dq { d: 6.0, q: 8.0 }, 0.0), 0.0, 1e-6);
        near(
            state.update(&p, Dq { d: 6.0, q: 12.0 }, 0.0),
            -0.341_641,
            1e-5,
        );
        p.kp = 10.0;
        near(state.update(&p, Dq { d: 6.0, q: 12.0 }, 0.0), -5.0, 1e-6);
    }
}
