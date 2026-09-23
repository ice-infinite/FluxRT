use foc_algorithm::{
    clamp, wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi, BemfInput, BemfParam, BemfPllParam,
    BemfPllState, PllParam, SmoInput, SmoParam, SmoPllParam, SmoPllState,
};

use crate::{
    pwm_to_alpha_beta, ControlMath, CpuMath, FeedbackSnapshot, MotorParameters, PwmCommand,
    RotorFeedback,
};

pub trait RotorEstimator {
    fn reset(&mut self, initial_electrical_angle_rad: f32);
    fn update_with_math<M: ControlMath>(
        &mut self,
        currents_and_bus: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback;

    fn update(
        &mut self,
        currents_and_bus: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
    ) -> RotorFeedback {
        self.update_with_math(currents_and_bus, previous_pwm, &mut CpuMath)
    }

    /// The estimator may calculate an angle before it is trustworthy enough
    /// to close the speed loop. Startup logic must explicitly check this bit.
    fn is_reliable(&self) -> bool;
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ObserverBackend {
    /// Portable sliding-mode observer followed by a PLL. This is the default
    /// hardware-independent sensorless backend.
    SmoPll = 0,
    /// Portable floating-point BEMF + PLL, useful for host simulation and
    /// non-ST targets.
    FloatBemfPll = 1,
    /// Reserved boundary for the exact ST MCSDK fixed-point STO-PLL backend.
    StStoPll = 2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoPllTuning {
    pub k_slide_v: f32,
    pub boundary_a: f32,
    pub emf_filter_alpha: f32,
    pub pll_kp: f32,
    pub pll_ki: f32,
}

impl SmoPllTuning {
    pub fn for_motor(motor: MotorParameters) -> Self {
        Self {
            // Hardware-correlated starting point for the high-resistance
            // GBM2804 class. All five values remain runtime-configurable.
            k_slide_v: motor.nominal_bus_voltage_v * (4.0 / 13.0),
            boundary_a: motor.rated_current_a * 0.20,
            emf_filter_alpha: 0.05,
            pll_kp: 80.0,
            pll_ki: 1_000.0,
        }
    }

    pub fn is_valid(&self) -> bool {
        self.k_slide_v.is_finite()
            && self.k_slide_v > 0.0
            && self.boundary_a.is_finite()
            && self.boundary_a > 0.0
            && self.emf_filter_alpha.is_finite()
            && (0.0001..=1.0).contains(&self.emf_filter_alpha)
            && self.pll_kp.is_finite()
            && self.pll_kp >= 0.0
            && self.pll_ki.is_finite()
            && self.pll_ki >= 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverReliabilityConfig {
    pub minimum_speed_rpm: f32,
    pub minimum_bemf_v: f32,
    /// Maximum normalized variance: `variance / mean_speed^2`.
    pub speed_variance_ratio: f32,
    pub consecutive_samples: u16,
}

impl ObserverReliabilityConfig {
    pub fn for_motor(motor: MotorParameters) -> Self {
        Self {
            minimum_speed_rpm: motor.max_speed_rpm / 3.0,
            minimum_bemf_v: 0.25,
            speed_variance_ratio: 0.01,
            consecutive_samples: 2,
        }
    }

    pub fn is_valid(&self) -> bool {
        self.minimum_speed_rpm.is_finite()
            && self.minimum_speed_rpm > 0.0
            && self.minimum_bemf_v.is_finite()
            && self.minimum_bemf_v > 0.0
            && self.speed_variance_ratio.is_finite()
            && (0.000_001..=1.0).contains(&self.speed_variance_ratio)
            && self.consecutive_samples > 0
    }
}

impl ObserverBackend {
    pub const fn from_raw(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::SmoPll),
            1 => Some(Self::FloatBemfPll),
            2 => Some(Self::StStoPll),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SmoPllEstimator {
    motor: MotorParameters,
    params: SmoPllParam,
    reliability: ObserverReliabilityConfig,
    state: SmoPllState,
    speed_fifo: [f32; 64],
    speed_index: usize,
    valid_samples: u32,
    reliable_samples: u16,
    speed_sum: f32,
    speed_square_sum: f32,
    reliability_decimator: u8,
    reliability_decimator_limit: u8,
}

impl SmoPllEstimator {
    pub fn new(motor: MotorParameters, sample_time_s: f32) -> Self {
        Self::new_with_tuning_and_reliability(
            motor,
            sample_time_s,
            SmoPllTuning::for_motor(motor),
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    pub fn new_with_tuning(
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
    ) -> Self {
        Self::new_with_tuning_and_reliability(
            motor,
            sample_time_s,
            tuning,
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    pub fn new_with_tuning_and_reliability(
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
        reliability: ObserverReliabilityConfig,
    ) -> Self {
        let tuning = if tuning.is_valid() {
            tuning
        } else {
            SmoPllTuning::for_motor(motor)
        };
        let reliability = if reliability.is_valid()
            && reliability.minimum_speed_rpm < motor.max_speed_rpm * 1.10
        {
            reliability
        } else {
            ObserverReliabilityConfig::for_motor(motor)
        };
        Self {
            motor,
            params: SmoPllParam {
                smo: SmoParam {
                    rs: motor.stator_resistance_ohm,
                    ls: motor.ld_h,
                    ts: sample_time_s,
                    k_slide: tuning.k_slide_v,
                    boundary: tuning.boundary_a,
                    emf_filter_alpha: tuning.emf_filter_alpha,
                },
                pll: PllParam {
                    kp: tuning.pll_kp,
                    ki: tuning.pll_ki,
                    ts: sample_time_s,
                    omega_min: -2_000.0,
                    omega_max: 2_000.0,
                },
            },
            reliability,
            state: SmoPllState::default(),
            speed_fifo: [0.0; 64],
            speed_index: 0,
            valid_samples: 0,
            reliable_samples: 0,
            speed_sum: 0.0,
            speed_square_sum: 0.0,
            reliability_decimator: 0,
            reliability_decimator_limit: ((0.001 / sample_time_s) as u32).clamp(1, 255) as u8,
        }
    }

    fn update_reliability(&mut self) {
        self.reliability_decimator = self.reliability_decimator.wrapping_add(1);
        if self.reliability_decimator < self.reliability_decimator_limit {
            return;
        }
        self.reliability_decimator = 0;
        let rpm =
            self.state.omega_rad_s * 30.0 / (core::f32::consts::PI * self.motor.pole_pairs as f32);
        let oldest = self.speed_fifo[self.speed_index];
        self.speed_sum += rpm - oldest;
        self.speed_square_sum += rpm * rpm - oldest * oldest;
        self.speed_fifo[self.speed_index] = rpm;
        self.speed_index = (self.speed_index + 1) & 63;
        self.valid_samples = self.valid_samples.saturating_add(1);
        if self.valid_samples < 64 {
            self.reliable_samples = 0;
            return;
        }
        let mean = self.speed_sum / 64.0;
        let variance = (self.speed_square_sum / 64.0 - mean * mean).max(0.0);
        let emf_sq =
            self.state.emf.alpha * self.state.emf.alpha + self.state.emf.beta * self.state.emf.beta;
        let stable = rpm.is_finite()
            && mean > self.reliability.minimum_speed_rpm
            && mean < self.motor.max_speed_rpm * 1.10
            && emf_sq > self.reliability.minimum_bemf_v * self.reliability.minimum_bemf_v
            && variance < mean * mean * self.reliability.speed_variance_ratio;
        self.reliable_samples = if stable {
            self.reliable_samples.saturating_add(1)
        } else {
            0
        };
    }
}

impl RotorEstimator for SmoPllEstimator {
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        self.state.reset();
        self.state.pll.reset(initial_electrical_angle_rad);
        self.speed_fifo = [0.0; 64];
        self.speed_index = 0;
        self.valid_samples = 0;
        self.reliable_samples = 0;
        self.speed_sum = 0.0;
        self.speed_square_sum = 0.0;
        self.reliability_decimator = 0;
    }

    fn update_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback {
        let voltage = pwm_to_alpha_beta(previous_pwm, feedback.dc_bus_voltage);
        let current = foc_algorithm::clarke(foc_algorithm::Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        self.state.emf = self
            .state
            .smo
            .update_vector(&self.params.smo, &SmoInput { voltage, current });
        self.state.smo.theta_emf_rad =
            math.angle_0_to_2pi(-self.state.emf.alpha, self.state.emf.beta);
        self.state.pll.phase_error =
            wrap_angle_minus_pi_to_pi(self.state.smo.theta_emf_rad - self.state.pll.theta_rad);
        self.state.pll.integrator +=
            self.params.pll.ki * self.params.pll.ts * self.state.pll.phase_error;
        let proportional = self.params.pll.kp * self.state.pll.phase_error;
        let raw_speed = proportional + self.state.pll.integrator;
        self.state.pll.omega_rad_s = clamp(
            raw_speed,
            self.params.pll.omega_min,
            self.params.pll.omega_max,
        );
        if raw_speed != self.state.pll.omega_rad_s {
            self.state.pll.integrator = self.state.pll.omega_rad_s - proportional;
        }
        self.state.pll.theta_rad = wrap_angle_0_to_2pi(
            self.state.pll.theta_rad + self.state.pll.omega_rad_s * self.params.pll.ts,
        );
        self.state.theta_rad = self.state.pll.theta_rad;
        self.state.omega_rad_s = self.state.pll.omega_rad_s;
        self.update_reliability();
        RotorFeedback {
            electrical_angle_rad: self.state.theta_rad,
            mechanical_speed_rad_s: self.state.omega_rad_s / self.motor.pole_pairs as f32,
        }
    }

    fn is_reliable(&self) -> bool {
        // Reliability is evaluated at 1 kHz after the 64-sample speed window.
        // The required consecutive windows are runtime-configurable.
        self.reliable_samples >= self.reliability.consecutive_samples
    }
}

/// Floating-point BEMF + PLL adapter with the same observer topology as the
/// reference MCSDK STO-PLL. It is intentionally separate from the current
/// controller so an encoder, resolver, or a later bit-equivalent STO port can
/// be substituted without changing the FOC loop.
#[derive(Clone, Copy, Debug)]
pub struct BemfPllEstimator {
    motor: MotorParameters,
    params: BemfPllParam,
    state: BemfPllState,
    valid_samples: u32,
}

impl BemfPllEstimator {
    pub fn new(motor: MotorParameters, sample_time_s: f32) -> Self {
        Self {
            motor,
            params: BemfPllParam {
                bemf: BemfParam {
                    rs: motor.stator_resistance_ohm,
                    ls: motor.ld_h,
                    ts: sample_time_s,
                    emf_filter_alpha: 0.08,
                },
                // Float-domain PLL bandwidth for host/target experimentation.
                // The generated fixed-point gains 195/16384 and 5/65535 are
                // retained in docs; they are not dimensionally interchangeable.
                pll: PllParam {
                    kp: 220.0,
                    ki: 12_000.0,
                    ts: sample_time_s,
                    omega_min: -2_000.0,
                    omega_max: 2_000.0,
                },
            },
            state: BemfPllState::default(),
            valid_samples: 0,
        }
    }

    pub fn electrical_speed_rad_s(&self) -> f32 {
        self.state.omega_rad_s
    }
}

impl RotorEstimator for BemfPllEstimator {
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        self.state.reset();
        self.state.pll.reset(initial_electrical_angle_rad);
        self.valid_samples = 0;
    }

    fn update_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
        _math: &mut M,
    ) -> RotorFeedback {
        let voltage = pwm_to_alpha_beta(previous_pwm, feedback.dc_bus_voltage);
        let current = foc_algorithm::clarke(foc_algorithm::Abc {
            a: feedback.currents.a,
            b: feedback.currents.b,
            c: feedback.currents.c,
        });
        let electrical_angle_rad = self
            .state
            .update(&self.params, &BemfInput { voltage, current });
        self.valid_samples = self.valid_samples.saturating_add(1);
        RotorFeedback {
            electrical_angle_rad,
            mechanical_speed_rad_s: self.state.omega_rad_s / self.motor.pole_pairs as f32,
        }
    }

    fn is_reliable(&self) -> bool {
        // This deliberately is not an MCSDK convergence claim. It only keeps
        // the portable backend from being consumed during its initial FIFO/
        // PLL transient. The ST-compatible backend will supply its own
        // variance and BEMF-consistency decision.
        self.valid_samples >= 2_048 && self.state.omega_rad_s.is_finite()
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ConfigurableObserver {
    backend: ObserverBackend,
    smo: SmoPllEstimator,
    bemf: BemfPllEstimator,
}

impl ConfigurableObserver {
    pub fn new(backend: ObserverBackend, motor: MotorParameters, sample_time_s: f32) -> Self {
        Self::new_with_smo_tuning(
            backend,
            motor,
            sample_time_s,
            SmoPllTuning::for_motor(motor),
        )
    }

    pub fn new_with_smo_tuning(
        backend: ObserverBackend,
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
    ) -> Self {
        Self::new_with_smo_tuning_and_reliability(
            backend,
            motor,
            sample_time_s,
            tuning,
            ObserverReliabilityConfig::for_motor(motor),
        )
    }

    pub fn new_with_smo_tuning_and_reliability(
        backend: ObserverBackend,
        motor: MotorParameters,
        sample_time_s: f32,
        tuning: SmoPllTuning,
        reliability: ObserverReliabilityConfig,
    ) -> Self {
        Self {
            backend,
            smo: SmoPllEstimator::new_with_tuning_and_reliability(
                motor,
                sample_time_s,
                tuning,
                reliability,
            ),
            bemf: BemfPllEstimator::new(motor, sample_time_s),
        }
    }

    pub const fn backend(&self) -> ObserverBackend {
        self.backend
    }
}

impl RotorEstimator for ConfigurableObserver {
    fn reset(&mut self, initial_electrical_angle_rad: f32) {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.reset(initial_electrical_angle_rad),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => {
                self.bemf.reset(initial_electrical_angle_rad)
            }
        }
    }

    fn update_with_math<M: ControlMath>(
        &mut self,
        feedback: &FeedbackSnapshot,
        previous_pwm: PwmCommand,
        math: &mut M,
    ) -> RotorFeedback {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.update_with_math(feedback, previous_pwm, math),
            ObserverBackend::StStoPll | ObserverBackend::FloatBemfPll => {
                self.bemf.update_with_math(feedback, previous_pwm, math)
            }
        }
    }

    fn is_reliable(&self) -> bool {
        match self.backend {
            ObserverBackend::SmoPll => self.smo.is_reliable(),
            // The portable equations are not yet bit-equivalent to MCSDK's
            // fixed-point STO-PLL, so the ST selection is telemetry-only for
            // this first hardware milestone and must not close the loop.
            ObserverBackend::StStoPll => false,
            ObserverBackend::FloatBemfPll => self.bemf.is_reliable(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{st_gbm2804_reference_parameters, PhaseCurrents};
    use foc_algorithm::{svpwm_update, AlphaBeta, SvpwmParam};

    #[test]
    fn bemf_pll_tracks_a_rotating_nonzero_speed_vector() {
        let control = st_gbm2804_reference_parameters();
        let dt = 1.0 / control.pwm_frequency_hz as f32;
        let mut estimator = BemfPllEstimator::new(control.motor, dt);
        estimator.reset(0.0);
        let electrical_speed = 200.0;
        let mut theta: f32 = 0.0;
        let feedback = FeedbackSnapshot {
            currents: PhaseCurrents::default(),
            dc_bus_voltage: control.motor.nominal_bus_voltage_v,
            rotor: RotorFeedback::default(),
        };
        for _ in 0..control.pwm_frequency_hz {
            theta = (theta + electrical_speed * dt).rem_euclid(2.0 * core::f32::consts::PI);
            let voltage = AlphaBeta {
                alpha: -2.0 * libm::sinf(theta),
                beta: 2.0 * libm::cosf(theta),
            };
            let pwm = svpwm_update(
                voltage,
                &SvpwmParam {
                    v_bus: feedback.dc_bus_voltage,
                },
            );
            estimator.update(
                &feedback,
                PwmCommand {
                    duty_a: pwm.duty_a,
                    duty_b: pwm.duty_b,
                    duty_c: pwm.duty_c,
                },
            );
        }
        assert!((estimator.electrical_speed_rad_s() - electrical_speed).abs() < 5.0);
    }
}
