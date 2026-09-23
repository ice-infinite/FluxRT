//! FOC_Basic 的等价实现：Clarke -> Park -> Id/Iq PI -> 逆 Park -> SVPWM。

use crate::controller::{PiParam, PiState};
use crate::modulation::{svpwm_update, SvpwmOutput, SvpwmParam};
use crate::transform::{clarke, inverse_park, park, Abc, AlphaBeta, Dq};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocBasicParam {
    pub id_pi: PiParam,
    pub iq_pi: PiParam,
    pub svpwm: SvpwmParam,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocBasicInput {
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    pub id_ref: f32,
    pub iq_ref: f32,
    pub theta_e_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FocBasicState {
    pub id_pi: PiState,
    pub iq_pi: PiState,
    pub current_alpha_beta: AlphaBeta,
    pub current_dq: Dq,
    pub voltage_dq: Dq,
    pub voltage_alpha_beta: AlphaBeta,
    pub pwm: SvpwmOutput,
}

impl Default for FocBasicState {
    fn default() -> Self {
        // 与 C 版 FOC_Basic_Reset 的 memset(0) 行为保持一致。
        Self {
            id_pi: PiState::default(),
            iq_pi: PiState::default(),
            current_alpha_beta: AlphaBeta::default(),
            current_dq: Dq::default(),
            voltage_dq: Dq::default(),
            voltage_alpha_beta: AlphaBeta::default(),
            pwm: SvpwmOutput {
                duty_a: 0.0,
                duty_b: 0.0,
                duty_c: 0.0,
                sector: 0,
            },
        }
    }
}

impl FocBasicState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(&mut self, param: &FocBasicParam, input: &FocBasicInput) -> SvpwmOutput {
        self.current_alpha_beta = clarke(Abc {
            a: input.ia,
            b: input.ib,
            c: input.ic,
        });
        self.current_dq = park(self.current_alpha_beta, input.theta_e_rad);
        self.voltage_dq.d = self
            .id_pi
            .update(&param.id_pi, input.id_ref, self.current_dq.d);
        self.voltage_dq.q = self
            .iq_pi
            .update(&param.iq_pi, input.iq_ref, self.current_dq.q);
        self.voltage_alpha_beta = inverse_park(self.voltage_dq, input.theta_e_rad);
        self.pwm = svpwm_update(self.voltage_alpha_beta, &param.svpwm);
        self.pwm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_c_reference_vector() {
        let pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        let param = FocBasicParam {
            id_pi: pi,
            iq_pi: pi,
            svpwm: SvpwmParam { v_bus: 24.0 },
        };
        let input = FocBasicInput {
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            id_ref: 0.0,
            iq_ref: 2.0,
            theta_e_rad: 0.0,
        };
        let mut state = FocBasicState::default();
        let out = state.update(&param, &input);
        assert!(state.current_dq.d.abs() <= 1e-6);
        assert!(state.current_dq.q.abs() <= 1e-6);
        assert!(state.voltage_dq.d.abs() <= 1e-6);
        assert!((state.voltage_dq.q - 2.0).abs() <= 1e-6);
        assert!((0.0..=1.0).contains(&out.duty_a));
        assert!((0.0..=1.0).contains(&out.duty_b));
        assert!((0.0..=1.0).contains(&out.duty_c));
    }
}
