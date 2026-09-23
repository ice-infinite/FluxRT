//! P/PI/PD/PID、速度环和位置环控制器。

use crate::math::{clamp, wrap_angle_minus_pi_to_pi};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PParam {
    pub kp: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PState {
    pub error: f32,
    pub output: f32,
}

impl PState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(&mut self, param: &PParam, reference: f32, feedback: f32) -> f32 {
        self.error = reference - feedback;
        self.output = clamp(param.kp * self.error, param.out_min, param.out_max);
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PiParam {
    pub kp: f32,
    pub ki: f32,
    pub ts: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub integrator_min: f32,
    pub integrator_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PiState {
    pub integrator: f32,
    pub error: f32,
    pub output: f32,
}

impl PiState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Preloads the integral term so the controller starts from an existing
    /// actuator command instead of stepping from zero on the first update.
    #[inline]
    pub fn preload_output(
        &mut self,
        param: &PiParam,
        reference: f32,
        feedback: f32,
        desired_output: f32,
    ) -> f32 {
        self.error = reference - feedback;
        let proportional = param.kp * self.error;
        self.output = clamp(desired_output, param.out_min, param.out_max);
        self.integrator = clamp(
            self.output - proportional,
            param.integrator_min,
            param.integrator_max,
        );
        self.output = clamp(proportional + self.integrator, param.out_min, param.out_max);
        self.output
    }

    #[inline]
    pub fn update(&mut self, param: &PiParam, reference: f32, feedback: f32) -> f32 {
        self.error = reference - feedback;
        let proportional = param.kp * self.error;

        self.integrator += param.ki * param.ts * self.error;
        self.integrator = clamp(self.integrator, param.integrator_min, param.integrator_max);

        let unclamped = proportional + self.integrator;
        self.output = clamp(unclamped, param.out_min, param.out_max);
        if unclamped != self.output {
            self.integrator = clamp(
                self.output - proportional,
                param.integrator_min,
                param.integrator_max,
            );
        }
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdParam {
    pub kp: f32,
    pub kd: f32,
    pub ts: f32,
    pub derivative_alpha: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PdState {
    pub error: f32,
    pub last_error: f32,
    pub derivative: f32,
    pub output: f32,
    /// 与 C 版 `int initialized` 保持布局和语义一致：0 为未初始化。
    pub initialized: i32,
}

impl PdState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(&mut self, param: &PdParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.error = reference - feedback;
        if self.initialized == 0 {
            self.last_error = self.error;
            self.initialized = 1;
        }
        let raw_derivative = (self.error - self.last_error) / param.ts;
        let alpha = clamp(param.derivative_alpha, 0.0, 1.0);
        self.derivative += alpha * (raw_derivative - self.derivative);
        self.last_error = self.error;
        self.output = clamp(
            param.kp * self.error + param.kd * self.derivative,
            param.out_min,
            param.out_max,
        );
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidParam {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub ts: f32,
    pub derivative_alpha: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub integrator_min: f32,
    pub integrator_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidState {
    pub error: f32,
    pub last_error: f32,
    pub integrator: f32,
    pub derivative: f32,
    pub output: f32,
    pub initialized: i32,
}

impl PidState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(&mut self, param: &PidParam, reference: f32, feedback: f32) -> f32 {
        if param.ts <= 0.0 {
            return 0.0;
        }
        self.error = reference - feedback;
        if self.initialized == 0 {
            self.last_error = self.error;
            self.initialized = 1;
        }
        let proportional = param.kp * self.error;
        self.integrator += param.ki * param.ts * self.error;
        self.integrator = clamp(self.integrator, param.integrator_min, param.integrator_max);

        let raw_derivative = (self.error - self.last_error) / param.ts;
        let alpha = clamp(param.derivative_alpha, 0.0, 1.0);
        self.derivative += alpha * (raw_derivative - self.derivative);
        self.last_error = self.error;

        let unclamped = proportional + self.integrator + param.kd * self.derivative;
        self.output = clamp(unclamped, param.out_min, param.out_max);
        if unclamped != self.output {
            self.integrator = clamp(
                self.output - proportional - param.kd * self.derivative,
                param.integrator_min,
                param.integrator_max,
            );
        }
        self.output
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpeedLoopParam {
    pub pi: PiParam,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpeedLoopState {
    pub pi: PiState,
    pub iq_ref: f32,
}

impl SpeedLoopState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(&mut self, param: &SpeedLoopParam, speed_ref: f32, speed_feedback: f32) -> f32 {
        self.iq_ref = self.pi.update(&param.pi, speed_ref, speed_feedback);
        self.iq_ref
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionLoopParam {
    pub kp: f32,
    pub speed_min: f32,
    pub speed_max: f32,
    /// 与 C 版 `int wrap_error` 一致；非 0 表示启用角度误差环绕。
    pub wrap_error: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionLoopState {
    pub error: f32,
    pub speed_ref: f32,
}

impl PositionLoopState {
    #[inline]
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    #[inline]
    pub fn update(
        &mut self,
        param: &PositionLoopParam,
        position_ref: f32,
        position_feedback: f32,
    ) -> f32 {
        let mut error = position_ref - position_feedback;
        if param.wrap_error != 0 {
            error = wrap_angle_minus_pi_to_pi(error);
        }
        self.error = error;
        self.speed_ref = clamp(param.kp * error, param.speed_min, param.speed_max);
        self.speed_ref
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedForwardParam {
    pub kv: f32,
    pub ka: f32,
    pub bias: f32,
    pub out_min: f32,
    pub out_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedForwardState {
    pub output: f32,
}

impl FeedForwardState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(
        &mut self,
        param: &FeedForwardParam,
        velocity_ref: f32,
        acceleration_ref: f32,
    ) -> f32 {
        let output = param.kv * velocity_ref + param.ka * acceleration_ref + param.bias;
        self.output = clamp(output, param.out_min, param.out_max);
        self.output
    }
}

use crate::foc::{FocBasicInput, FocBasicParam, FocBasicState};
use crate::modulation::SvpwmOutput;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeParam {
    pub position: PositionLoopParam,
    pub speed: SpeedLoopParam,
    pub foc: FocBasicParam,
    pub id_ref: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeInput {
    pub position_ref: f32,
    pub position_feedback: f32,
    pub speed_feedback: f32,
    pub ia: f32,
    pub ib: f32,
    pub ic: f32,
    pub theta_e_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CascadeState {
    pub position: PositionLoopState,
    pub speed: SpeedLoopState,
    pub foc: FocBasicState,
    pub speed_ref: f32,
    pub iq_ref: f32,
    pub pwm: SvpwmOutput,
}

impl Default for CascadeState {
    fn default() -> Self {
        Self {
            position: PositionLoopState::default(),
            speed: SpeedLoopState::default(),
            foc: FocBasicState::default(),
            speed_ref: 0.0,
            iq_ref: 0.0,
            pwm: SvpwmOutput::default(),
        }
    }
}

impl CascadeState {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn update(&mut self, param: &CascadeParam, input: &CascadeInput) -> SvpwmOutput {
        self.speed_ref =
            self.position
                .update(&param.position, input.position_ref, input.position_feedback);
        self.iq_ref = self
            .speed
            .update(&param.speed, self.speed_ref, input.speed_feedback);
        self.pwm = self.foc.update(
            &param.foc,
            &FocBasicInput {
                ia: input.ia,
                ib: input.ib,
                ic: input.ic,
                id_ref: param.id_ref,
                iq_ref: self.iq_ref,
                theta_e_rad: input.theta_e_rad,
            },
        );
        self.pwm
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::TWO_PI;

    fn assert_near(actual: f32, expected: f32, tolerance: f32) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} != {expected}"
        );
    }

    #[test]
    fn p_matches_c_reference() {
        let p = PParam {
            kp: 2.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut s = PState::default();
        assert_near(s.update(&p, 3.0, 1.0), 4.0, 1e-6);
        assert_near(s.update(&p, 10.0, 0.0), 5.0, 1e-6);
    }

    #[test]
    fn pi_matches_c_reference() {
        let p = PiParam {
            kp: 2.0,
            ki: 10.0,
            ts: 0.001,
            out_min: -1.0,
            out_max: 1.0,
            integrator_min: -0.5,
            integrator_max: 0.5,
        };
        let mut s = PiState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.0, 1e-6);
        assert!(s.integrator <= 0.5);
        s.reset();
        assert_near(s.update(&p, 0.1, 0.0), 0.201, 1e-6);
    }

    #[test]
    fn pi_preload_preserves_existing_output() {
        let p = PiParam {
            kp: 0.2,
            ki: 1.0,
            ts: 0.001,
            out_min: -1.0,
            out_max: 1.0,
            integrator_min: -1.0,
            integrator_max: 1.0,
        };
        let mut s = PiState::default();
        assert_near(s.preload_output(&p, 10.0, 10.0, 0.8), 0.8, 1e-6);
        assert_near(s.update(&p, 10.0, 10.0), 0.8, 1e-6);
    }

    #[test]
    fn pd_matches_c_reference() {
        let mut p = PdParam {
            kp: 1.0,
            kd: 0.1,
            ts: 0.01,
            derivative_alpha: 1.0,
            out_min: -100.0,
            out_max: 100.0,
        };
        let mut s = PdState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.0, 1e-6);
        assert_near(s.update(&p, 2.0, 0.0), 12.0, 1e-5);
        p.out_max = 5.0;
        assert_near(s.update(&p, 20.0, 0.0), 5.0, 1e-6);
    }

    #[test]
    fn pid_matches_c_reference() {
        let mut p = PidParam {
            kp: 1.0,
            ki: 2.0,
            kd: 0.1,
            ts: 0.01,
            derivative_alpha: 1.0,
            out_min: -10.0,
            out_max: 20.0,
            integrator_min: -1.0,
            integrator_max: 1.0,
        };
        let mut s = PidState::default();
        assert_near(s.update(&p, 1.0, 0.0), 1.02, 1e-6);
        assert_near(s.update(&p, 2.0, 0.0), 12.06, 1e-5);
        p.out_max = 3.0;
        assert_near(s.update(&p, 100.0, 0.0), 3.0, 1e-6);
        assert!(s.integrator <= 1.0);
    }

    #[test]
    fn outer_loops_match_c_reference() {
        let speed_param = SpeedLoopParam {
            pi: PiParam {
                kp: 0.1,
                ki: 1.0,
                ts: 0.01,
                out_min: -3.0,
                out_max: 3.0,
                integrator_min: -1.0,
                integrator_max: 1.0,
            },
        };
        let mut speed = SpeedLoopState::default();
        assert_near(speed.update(&speed_param, 10.0, 0.0), 1.1, 1e-6);
        assert_near(speed.update(&speed_param, 1000.0, 0.0), 3.0, 1e-6);

        let mut position_param = PositionLoopParam {
            kp: 2.0,
            speed_min: -5.0,
            speed_max: 5.0,
            wrap_error: 0,
        };
        let mut position = PositionLoopState::default();
        assert_near(position.update(&position_param, 2.0, 1.5), 1.0, 1e-6);
        assert_near(position.update(&position_param, 10.0, 0.0), 5.0, 1e-6);
        position_param.wrap_error = 1;
        assert_near(
            position.update(&position_param, 0.1, TWO_PI - 0.1),
            0.4,
            1e-5,
        );
        assert_near(position.error, 0.2, 1e-5);
    }

    #[test]
    fn feed_forward_matches_c_reference() {
        let p = FeedForwardParam {
            kv: 0.5,
            ka: 0.25,
            bias: 1.0,
            out_min: -5.0,
            out_max: 5.0,
        };
        let mut state = FeedForwardState::default();
        assert_near(state.update(&p, 4.0, 2.0), 3.5, 1e-6);
        assert_near(state.update(&p, 100.0, 0.0), 5.0, 1e-6);
    }

    #[test]
    fn cascade_matches_c_reference() {
        use crate::modulation::SvpwmParam;

        let current_pi = PiParam {
            kp: 1.0,
            ki: 0.0,
            ts: 0.001,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        let param = CascadeParam {
            position: PositionLoopParam {
                kp: 2.0,
                speed_min: -20.0,
                speed_max: 20.0,
                wrap_error: 0,
            },
            speed: SpeedLoopParam {
                pi: PiParam {
                    kp: 0.5,
                    ki: 0.0,
                    ts: 0.001,
                    out_min: -5.0,
                    out_max: 5.0,
                    integrator_min: -2.0,
                    integrator_max: 2.0,
                },
            },
            foc: FocBasicParam {
                id_pi: current_pi,
                iq_pi: current_pi,
                svpwm: SvpwmParam { v_bus: 24.0 },
            },
            id_ref: 0.0,
        };
        let input = CascadeInput {
            position_ref: 3.0,
            position_feedback: 1.0,
            speed_feedback: 1.0,
            ia: 0.0,
            ib: 0.0,
            ic: 0.0,
            theta_e_rad: 0.0,
        };
        let mut state = CascadeState::default();
        let output = state.update(&param, &input);
        assert_near(state.speed_ref, 4.0, 1e-6);
        assert_near(state.iq_ref, 1.5, 1e-6);
        assert_near(state.foc.voltage_dq.d, 0.0, 1e-6);
        assert_near(state.foc.voltage_dq.q, 1.5, 1e-6);
        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
    }
}
