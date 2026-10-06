//! Bounded pulse sequence for HFI magnet-polarity identification.
//!
//! The rotating HFI angle is inherently valid only modulo `pi`. This state
//! machine requests matched positive/negative voltage pulses on one candidate
//! axis and compares their peak current responses. It owns sequencing, current
//! limits, settle timeouts, retry count and fail-closed output; the C platform
//! still owns hardware Break/over-current and whether injection is permitted.

use core::mem::size_of;

use foc_algorithm::AlphaBeta;

pub const HFI_POLARITY_CONFIG_VERSION: u32 = 1;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HfiPolarityState {
    #[default]
    Idle = 0,
    PositivePulse = 1,
    PositiveOff = 2,
    NegativePulse = 3,
    NegativeOff = 4,
    Resolved = 5,
    Failed = 6,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HfiPolarityFailure {
    #[default]
    None = 0,
    InvalidConfig = 1,
    InjectionNotPermitted = 2,
    InvalidInput = 3,
    OverCurrent = 4,
    ResidualCurrentNotSettled = 5,
    InsufficientResponse = 6,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HfiPolarityConfig {
    pub struct_size: u32,
    pub version: u32,
    pub pulse_voltage_v: f32,
    pub pulse_ticks: u32,
    pub minimum_off_ticks: u32,
    pub settle_timeout_ticks: u32,
    pub maximum_current_a: f32,
    pub maximum_residual_current_a: f32,
    pub minimum_response_delta_a: f32,
    pub maximum_pulse_pairs: u32,
}

impl Default for HfiPolarityConfig {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: HFI_POLARITY_CONFIG_VERSION,
            pulse_voltage_v: 0.5,
            pulse_ticks: 2,
            minimum_off_ticks: 2,
            settle_timeout_ticks: 24,
            maximum_current_a: 0.5,
            maximum_residual_current_a: 0.05,
            minimum_response_delta_a: 0.02,
            maximum_pulse_pairs: 3,
        }
    }
}

impl HfiPolarityConfig {
    pub fn is_valid(&self) -> bool {
        self.struct_size == size_of::<Self>() as u32
            && self.version == HFI_POLARITY_CONFIG_VERSION
            && finite_positive(self.pulse_voltage_v)
            && self.pulse_ticks > 0
            && self.minimum_off_ticks > 0
            && self.settle_timeout_ticks >= self.minimum_off_ticks
            && finite_positive(self.maximum_current_a)
            && finite_nonnegative(self.maximum_residual_current_a)
            && self.maximum_residual_current_a < self.maximum_current_a
            && finite_positive(self.minimum_response_delta_a)
            && self.minimum_response_delta_a < self.maximum_current_a
            && self.maximum_pulse_pairs > 0
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfiPolarityInput {
    pub request_start: bool,
    pub request_clear: bool,
    pub injection_permitted: bool,
    pub candidate_axis_rad: f32,
    pub measured_current_alpha_beta: AlphaBeta,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfiPolarityOutput {
    pub injection_voltage_alpha_beta: AlphaBeta,
    pub state: HfiPolarityState,
    pub failure: HfiPolarityFailure,
    pub completed_pairs: u32,
    pub response_delta_a: f32,
    pub polarity_resolved: bool,
    /// Branch decision consumed by `SensorlessFusionInput::hfi_add_pi`.
    pub hfi_add_pi: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HfiPolaritySupervisor {
    state: HfiPolarityState,
    failure: HfiPolarityFailure,
    axis_cos: f32,
    axis_sin: f32,
    phase_ticks: u32,
    completed_pairs: u32,
    positive_peak_a: f32,
    negative_peak_a: f32,
    response_delta_a: f32,
    hfi_add_pi: bool,
}

impl HfiPolaritySupervisor {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn step(
        &mut self,
        config: &HfiPolarityConfig,
        input: HfiPolarityInput,
    ) -> HfiPolarityOutput {
        if input.request_clear {
            self.reset();
            return self.output(AlphaBeta::default());
        }
        if !config.is_valid() {
            return self.fail(HfiPolarityFailure::InvalidConfig);
        }
        if !finite_current(input.measured_current_alpha_beta) {
            return self.fail(HfiPolarityFailure::InvalidInput);
        }
        if self.state != HfiPolarityState::Idle
            && self.state != HfiPolarityState::Resolved
            && self.state != HfiPolarityState::Failed
            && !input.injection_permitted
        {
            return self.fail(HfiPolarityFailure::InjectionNotPermitted);
        }
        let current_magnitude = magnitude(input.measured_current_alpha_beta);
        if current_magnitude > config.maximum_current_a {
            return self.fail(HfiPolarityFailure::OverCurrent);
        }

        match self.state {
            HfiPolarityState::Idle => {
                if !input.request_start {
                    return self.output(AlphaBeta::default());
                }
                if !input.injection_permitted {
                    return self.fail(HfiPolarityFailure::InjectionNotPermitted);
                }
                if !input.candidate_axis_rad.is_finite() {
                    return self.fail(HfiPolarityFailure::InvalidInput);
                }
                if current_magnitude > config.maximum_residual_current_a {
                    return self.fail(HfiPolarityFailure::ResidualCurrentNotSettled);
                }
                self.axis_cos = libm::cosf(input.candidate_axis_rad);
                self.axis_sin = libm::sinf(input.candidate_axis_rad);
                self.failure = HfiPolarityFailure::None;
                self.state = HfiPolarityState::PositivePulse;
                self.phase_ticks = 0;
                self.positive_peak_a = 0.0;
                self.negative_peak_a = 0.0;
                self.run_positive_pulse(config, input.measured_current_alpha_beta)
            }
            HfiPolarityState::PositivePulse => {
                self.run_positive_pulse(config, input.measured_current_alpha_beta)
            }
            HfiPolarityState::PositiveOff => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= config.minimum_off_ticks
                    && current_magnitude <= config.maximum_residual_current_a
                {
                    self.state = HfiPolarityState::NegativePulse;
                    self.phase_ticks = 0;
                } else if self.phase_ticks >= config.settle_timeout_ticks {
                    return self.fail(HfiPolarityFailure::ResidualCurrentNotSettled);
                }
                self.output(AlphaBeta::default())
            }
            HfiPolarityState::NegativePulse => {
                self.run_negative_pulse(config, input.measured_current_alpha_beta)
            }
            HfiPolarityState::NegativeOff => {
                self.phase_ticks = self.phase_ticks.saturating_add(1);
                if self.phase_ticks >= config.minimum_off_ticks
                    && current_magnitude <= config.maximum_residual_current_a
                {
                    return self.evaluate_pair(config);
                }
                if self.phase_ticks >= config.settle_timeout_ticks {
                    return self.fail(HfiPolarityFailure::ResidualCurrentNotSettled);
                }
                self.output(AlphaBeta::default())
            }
            HfiPolarityState::Resolved | HfiPolarityState::Failed => {
                self.output(AlphaBeta::default())
            }
        }
    }

    fn run_positive_pulse(
        &mut self,
        config: &HfiPolarityConfig,
        current: AlphaBeta,
    ) -> HfiPolarityOutput {
        self.positive_peak_a = self.positive_peak_a.max(self.project(current).max(0.0));
        self.phase_ticks = self.phase_ticks.saturating_add(1);
        let voltage = self.axis_voltage(config.pulse_voltage_v);
        if self.phase_ticks >= config.pulse_ticks {
            self.state = HfiPolarityState::PositiveOff;
            self.phase_ticks = 0;
        }
        self.output(voltage)
    }

    fn run_negative_pulse(
        &mut self,
        config: &HfiPolarityConfig,
        current: AlphaBeta,
    ) -> HfiPolarityOutput {
        self.negative_peak_a = self.negative_peak_a.max((-self.project(current)).max(0.0));
        self.phase_ticks = self.phase_ticks.saturating_add(1);
        let voltage = self.axis_voltage(-config.pulse_voltage_v);
        if self.phase_ticks >= config.pulse_ticks {
            self.state = HfiPolarityState::NegativeOff;
            self.phase_ticks = 0;
        }
        self.output(voltage)
    }

    fn evaluate_pair(&mut self, config: &HfiPolarityConfig) -> HfiPolarityOutput {
        self.completed_pairs = self.completed_pairs.saturating_add(1);
        self.response_delta_a = self.positive_peak_a - self.negative_peak_a;
        if self.response_delta_a.abs() >= config.minimum_response_delta_a {
            // Sign convention: a stronger negative-axis response selects the
            // opposite pi branch. Board current polarity and candidate-axis
            // direction must be validated together before target use.
            self.hfi_add_pi = self.response_delta_a < 0.0;
            self.state = HfiPolarityState::Resolved;
            return self.output(AlphaBeta::default());
        }
        if self.completed_pairs >= config.maximum_pulse_pairs {
            return self.fail(HfiPolarityFailure::InsufficientResponse);
        }
        self.positive_peak_a = 0.0;
        self.negative_peak_a = 0.0;
        self.phase_ticks = 0;
        self.state = HfiPolarityState::PositivePulse;
        self.output(AlphaBeta::default())
    }

    fn fail(&mut self, failure: HfiPolarityFailure) -> HfiPolarityOutput {
        self.state = HfiPolarityState::Failed;
        self.failure = failure;
        self.output(AlphaBeta::default())
    }

    fn project(&self, current: AlphaBeta) -> f32 {
        current.alpha * self.axis_cos + current.beta * self.axis_sin
    }

    fn axis_voltage(&self, amplitude: f32) -> AlphaBeta {
        AlphaBeta {
            alpha: amplitude * self.axis_cos,
            beta: amplitude * self.axis_sin,
        }
    }

    fn output(&self, voltage: AlphaBeta) -> HfiPolarityOutput {
        HfiPolarityOutput {
            injection_voltage_alpha_beta: voltage,
            state: self.state,
            failure: self.failure,
            completed_pairs: self.completed_pairs,
            response_delta_a: self.response_delta_a,
            polarity_resolved: self.state == HfiPolarityState::Resolved,
            hfi_add_pi: self.hfi_add_pi,
        }
    }
}

fn magnitude(value: AlphaBeta) -> f32 {
    libm::sqrtf(value.alpha * value.alpha + value.beta * value.beta)
}

fn finite_current(value: AlphaBeta) -> bool {
    value.alpha.is_finite() && value.beta.is_finite()
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(current_a: f32) -> HfiPolarityInput {
        HfiPolarityInput {
            request_start: true,
            request_clear: false,
            injection_permitted: true,
            candidate_axis_rad: 0.0,
            measured_current_alpha_beta: AlphaBeta {
                alpha: current_a,
                beta: 0.0,
            },
        }
    }

    fn compact_config() -> HfiPolarityConfig {
        HfiPolarityConfig {
            pulse_ticks: 2,
            minimum_off_ticks: 1,
            settle_timeout_ticks: 2,
            maximum_current_a: 1.0,
            maximum_residual_current_a: 0.05,
            minimum_response_delta_a: 0.05,
            maximum_pulse_pairs: 1,
            ..HfiPolarityConfig::default()
        }
    }

    #[test]
    fn matched_pulse_pair_resolves_branch_and_always_ends_at_zero_voltage() {
        let config = compact_config();
        let mut state = HfiPolaritySupervisor::default();
        assert_eq!(
            state.step(&config, sample(0.0)).state,
            HfiPolarityState::PositivePulse
        );
        assert_eq!(
            state.step(&config, sample(0.40)).state,
            HfiPolarityState::PositiveOff
        );
        assert_eq!(
            state.step(&config, sample(0.0)).state,
            HfiPolarityState::NegativePulse
        );
        assert_eq!(
            state.step(&config, sample(0.0)).state,
            HfiPolarityState::NegativePulse
        );
        assert_eq!(
            state.step(&config, sample(-0.20)).state,
            HfiPolarityState::NegativeOff
        );
        let result = state.step(&config, sample(0.0));
        assert_eq!(result.state, HfiPolarityState::Resolved);
        assert!(result.polarity_resolved);
        assert!(!result.hfi_add_pi);
        assert!((result.response_delta_a - 0.20).abs() < 1.0e-6);
        assert_eq!(result.injection_voltage_alpha_beta, AlphaBeta::default());
    }

    #[test]
    fn negative_response_selects_opposite_pi_branch() {
        let config = compact_config();
        let mut state = HfiPolaritySupervisor::default();
        state.step(&config, sample(0.0));
        state.step(&config, sample(0.15));
        state.step(&config, sample(0.0));
        state.step(&config, sample(0.0));
        state.step(&config, sample(-0.35));
        let result = state.step(&config, sample(0.0));
        assert!(result.polarity_resolved);
        assert!(result.hfi_add_pi);
    }

    #[test]
    fn weak_response_exhausts_bounded_pairs_and_fails_closed() {
        let config = compact_config();
        let mut state = HfiPolaritySupervisor::default();
        state.step(&config, sample(0.0));
        state.step(&config, sample(0.20));
        state.step(&config, sample(0.0));
        state.step(&config, sample(0.0));
        state.step(&config, sample(-0.18));
        let result = state.step(&config, sample(0.0));
        assert_eq!(result.state, HfiPolarityState::Failed);
        assert_eq!(result.failure, HfiPolarityFailure::InsufficientResponse);
        assert_eq!(result.injection_voltage_alpha_beta, AlphaBeta::default());
    }

    #[test]
    fn overcurrent_or_revoked_permission_fails_in_the_same_tick() {
        let config = compact_config();
        let mut state = HfiPolaritySupervisor::default();
        state.step(&config, sample(0.0));
        let overcurrent = state.step(&config, sample(1.01));
        assert_eq!(overcurrent.failure, HfiPolarityFailure::OverCurrent);
        assert_eq!(
            overcurrent.injection_voltage_alpha_beta,
            AlphaBeta::default()
        );

        let mut state = HfiPolaritySupervisor::default();
        state.step(&config, sample(0.0));
        let mut revoked = sample(0.0);
        revoked.injection_permitted = false;
        let result = state.step(&config, revoked);
        assert_eq!(result.failure, HfiPolarityFailure::InjectionNotPermitted);
        assert_eq!(result.injection_voltage_alpha_beta, AlphaBeta::default());
    }

    #[test]
    fn explicit_clear_returns_a_latched_failure_to_idle() {
        let config = compact_config();
        let mut state = HfiPolaritySupervisor::default();
        state.step(&config, sample(0.0));
        state.step(&config, sample(1.01));
        let mut clear = sample(0.0);
        clear.request_start = false;
        clear.request_clear = true;
        let output = state.step(&config, clear);
        assert_eq!(output.state, HfiPolarityState::Idle);
        assert_eq!(output.failure, HfiPolarityFailure::None);
    }
}
