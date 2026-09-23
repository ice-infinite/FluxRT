use core::f32::consts::PI;

use foc_algorithm::{clarke, svpwm_update, Abc, AlphaBeta, Dq, PiState, SvpwmParam};

use crate::{
    ControlMath, ControlParameters, ControlTelemetry, CpuMath, CurrentCommand, FeedbackSnapshot,
    PwmCommand, SpeedCommand,
};

const SQRT_3: f32 = 1.732_050_8;

#[derive(Clone, Copy, Debug, Default)]
pub struct CurrentLoop {
    id_pi: PiState,
    iq_pi: PiState,
}

impl CurrentLoop {
    pub fn reset(&mut self) {
        self.id_pi.reset();
        self.iq_pi.reset();
    }

    pub fn update(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        reference: CurrentCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.update_with_math(params, feedback, reference, &mut CpuMath)
    }

    /// Runs one current-loop sample using a replaceable math backend.
    pub fn update_with_math<M: ControlMath>(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        reference: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        let current_alpha_beta = clarke(Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        self.update_from_alpha_beta_with_math(params, feedback, current_alpha_beta, reference, math)
    }

    /// Runs one current-loop sample from a Clarke result already calculated by
    /// the realtime composition layer. This keeps Clarke a once-per-sample
    /// operation when the observer and current controller consume the same ADC
    /// snapshot.
    pub fn update_from_alpha_beta_with_math<M: ControlMath>(
        &mut self,
        params: &ControlParameters,
        feedback: &FeedbackSnapshot,
        current_alpha_beta: AlphaBeta,
        reference: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        // One sin/cos pair is shared by Park and inverse Park. This reduces the
        // CPU fallback cost as well as the number of accelerator transactions.
        let (sin, cos) = math.sin_cos(feedback.rotor.electrical_angle_rad);
        let current_dq = Dq {
            d: current_alpha_beta.alpha * cos + current_alpha_beta.beta * sin,
            q: -current_alpha_beta.alpha * sin + current_alpha_beta.beta * cos,
        };
        let mut voltage_dq = Dq {
            d: self
                .id_pi
                .update(&params.id_pi, reference.id_ref_a, current_dq.d),
            q: self
                .iq_pi
                .update(&params.iq_pi, reference.iq_ref_a, current_dq.q),
        };
        let limit = params.voltage_utilization * feedback.dc_bus_voltage / SQRT_3;
        let magnitude = math.magnitude(voltage_dq.d, voltage_dq.q);
        let voltage_limited = magnitude > limit && magnitude > 0.0;
        if voltage_limited {
            let scale = limit / magnitude;
            voltage_dq.d *= scale;
            voltage_dq.q *= scale;
        }
        let voltage_alpha_beta = AlphaBeta {
            alpha: voltage_dq.d * cos - voltage_dq.q * sin,
            beta: voltage_dq.d * sin + voltage_dq.q * cos,
        };
        let pwm = svpwm_update(
            voltage_alpha_beta,
            &SvpwmParam {
                v_bus: feedback.dc_bus_voltage,
            },
        );
        (
            PwmCommand {
                duty_a: pwm.duty_a,
                duty_b: pwm.duty_b,
                duty_c: pwm.duty_c,
            },
            ControlTelemetry {
                current_reference: reference,
                current_dq,
                voltage_dq,
                voltage_alpha_beta,
                measured_speed_rpm: feedback.rotor.mechanical_speed_rad_s * 30.0 / PI,
                voltage_limited,
            },
        )
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpeedLoop {
    pi: PiState,
}

impl SpeedLoop {
    pub fn reset(&mut self) {
        self.pi.reset();
    }

    /// Matches the speed PI output to the torque-producing current already
    /// flowing at sensorless handoff. This mirrors MCSDK's speed-integral
    /// initialization in `SWITCH_OVER`.
    pub fn preload(
        &mut self,
        params: &ControlParameters,
        command: SpeedCommand,
        mechanical_speed_rad_s: f32,
        current_iq_a: f32,
    ) -> CurrentCommand {
        let target = command.target_rpm * PI / 30.0;
        CurrentCommand {
            id_ref_a: command.id_ref_a,
            iq_ref_a: self.pi.preload_output(
                &params.speed_pi,
                target,
                mechanical_speed_rad_s,
                current_iq_a,
            ),
        }
    }

    pub fn update(
        &mut self,
        params: &ControlParameters,
        command: SpeedCommand,
        mechanical_speed_rad_s: f32,
    ) -> CurrentCommand {
        let target = command.target_rpm * PI / 30.0;
        CurrentCommand {
            id_ref_a: command.id_ref_a,
            iq_ref_a: self
                .pi
                .update(&params.speed_pi, target, mechanical_speed_rad_s),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StReferenceController {
    params: ControlParameters,
    current: CurrentLoop,
    speed: SpeedLoop,
    current_reference: CurrentCommand,
    speed_divider: u32,
    speed_counter: u32,
}

impl StReferenceController {
    pub fn new(params: ControlParameters) -> Self {
        let speed_divider = (params.pwm_frequency_hz / params.speed_loop_frequency_hz).max(1);
        Self {
            params,
            current: CurrentLoop::default(),
            speed: SpeedLoop::default(),
            current_reference: CurrentCommand::default(),
            speed_divider,
            speed_counter: 0,
        }
    }

    pub fn parameters(&self) -> &ControlParameters {
        &self.params
    }

    pub fn reset(&mut self) {
        self.current.reset();
        self.speed.reset();
        self.current_reference = CurrentCommand::default();
        self.speed_counter = 0;
    }

    pub fn update(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: SpeedCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.update_with_math(feedback, command, &mut CpuMath)
    }

    pub fn update_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: SpeedCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        if self.speed_counter == 0 {
            self.current_reference =
                self.speed
                    .update(&self.params, command, feedback.rotor.mechanical_speed_rad_s);
        }
        self.speed_counter += 1;
        if self.speed_counter >= self.speed_divider {
            self.speed_counter = 0;
        }
        self.current
            .update_with_math(&self.params, feedback, self.current_reference, math)
    }

    pub fn update_current(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: CurrentCommand,
    ) -> (PwmCommand, ControlTelemetry) {
        self.current.update(&self.params, feedback, command)
    }

    pub fn update_current_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        command: CurrentCommand,
        math: &mut M,
    ) -> (PwmCommand, ControlTelemetry) {
        self.current
            .update_with_math(&self.params, feedback, command, math)
    }
}

pub fn pwm_to_phase_voltage(pwm: PwmCommand, dc_bus_voltage: f32) -> Abc {
    let common = (pwm.duty_a + pwm.duty_b + pwm.duty_c) / 3.0;
    Abc {
        a: (pwm.duty_a - common) * dc_bus_voltage,
        b: (pwm.duty_b - common) * dc_bus_voltage,
        c: (pwm.duty_c - common) * dc_bus_voltage,
    }
}

pub fn pwm_to_alpha_beta(pwm: PwmCommand, dc_bus_voltage: f32) -> AlphaBeta {
    clarke(pwm_to_phase_voltage(pwm, dc_bus_voltage))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{st_gbm2804_reference_parameters, PhaseCurrents, RotorFeedback};

    #[test]
    fn zero_error_produces_centered_pwm() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let (pwm, telemetry) = loop_.update(&params, &feedback, CurrentCommand::default());
        assert!((pwm.duty_a - 0.5).abs() < 1e-6);
        assert!(!telemetry.voltage_limited);
    }

    #[test]
    fn circle_limit_bounds_voltage_vector() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let (pwm, telemetry) = loop_.update(
            &params,
            &feedback,
            CurrentCommand {
                id_ref_a: 0.8,
                iq_ref_a: 0.8,
            },
        );
        assert!(pwm.is_valid());
        assert!(telemetry.voltage_limited);
    }

    #[derive(Default)]
    struct CountingMath {
        sin_cos_calls: u32,
        magnitude_calls: u32,
    }

    impl ControlMath for CountingMath {
        fn sin_cos(&mut self, _angle_rad: f32) -> (f32, f32) {
            self.sin_cos_calls += 1;
            (0.0, 1.0)
        }

        fn magnitude(&mut self, x: f32, y: f32) -> f32 {
            self.magnitude_calls += 1;
            libm::sqrtf(x * x + y * y)
        }

        fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
            foc_algorithm::atan2_angle_0_to_2pi(y, x)
        }
    }

    #[test]
    fn current_loop_uses_replaceable_math_backend_once_per_sample() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback::default(),
        };
        let mut loop_ = CurrentLoop::default();
        let mut math = CountingMath::default();
        let _ = loop_.update_with_math(&params, &feedback, CurrentCommand::default(), &mut math);
        assert_eq!(math.sin_cos_calls, 1);
        assert_eq!(math.magnitude_calls, 1);
    }

    #[test]
    fn precomputed_clarke_path_matches_regular_current_loop() {
        let params = st_gbm2804_reference_parameters();
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents {
                a: 0.37,
                b: -0.22,
                c: -0.15,
            },
            dc_bus_voltage: 13.0,
            rotor: RotorFeedback {
                electrical_angle_rad: 1.25,
                mechanical_speed_rad_s: 41.0,
            },
        };
        let reference = CurrentCommand {
            id_ref_a: 0.1,
            iq_ref_a: 0.6,
        };
        let current_alpha_beta = clarke(Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        let mut regular = CurrentLoop::default();
        let mut precomputed = CurrentLoop::default();
        let regular_output = regular.update(&params, &feedback, reference);
        let precomputed_output = precomputed.update_from_alpha_beta_with_math(
            &params,
            &feedback,
            current_alpha_beta,
            reference,
            &mut CpuMath,
        );

        assert_eq!(precomputed_output, regular_output);
    }

    #[test]
    fn speed_loop_preload_is_bumpless_at_equal_speed() {
        let params = st_gbm2804_reference_parameters();
        let speed_rpm = 524.0;
        let speed_rad_s = speed_rpm * PI / 30.0;
        let command = SpeedCommand {
            target_rpm: speed_rpm,
            id_ref_a: 0.0,
        };
        let mut loop_ = SpeedLoop::default();
        let preloaded = loop_.preload(&params, command, speed_rad_s, 0.63);
        let first = loop_.update(&params, command, speed_rad_s);
        assert!((preloaded.iq_ref_a - 0.63).abs() < 1e-6);
        assert!((first.iq_ref_a - preloaded.iq_ref_a).abs() < 1e-6);
    }
}
