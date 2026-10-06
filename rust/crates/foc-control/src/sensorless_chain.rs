//! Full-speed sensorless angle-chain composition.
//!
//! This owner sequences rotating-HFI axis acquisition, bounded magnet-polarity
//! pulses and HFI/BEMF trust fusion.  It remains board independent: the caller
//! supplies separated HF current and an existing SMO/BEMF channel, while the C
//! platform remains the only owner of PWM/ADC timing and hardware shutdown.

use core::mem::size_of;

use foc_algorithm::{AlphaBeta, RotatingHfSequenceParam, RotatingHfSequenceState};

use crate::{
    HfiPolarityConfig, HfiPolarityFailure, HfiPolarityInput, HfiPolarityState,
    HfiPolaritySupervisor, SensorlessFusionConfig, SensorlessFusionInput, SensorlessFusionOutput,
    SensorlessFusionSupervisor,
};

pub const SENSORLESS_CHAIN_CONFIG_VERSION: u32 = 1;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SensorlessChainStage {
    #[default]
    AxisAcquisition = 0,
    Polarity = 1,
    Tracking = 2,
    Failed = 3,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SensorlessChainFailure {
    #[default]
    None = 0,
    InvalidConfig = 1,
    InvalidInput = 2,
    Polarity = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorlessChainConfig {
    pub struct_size: u32,
    pub version: u32,
    pub hfi: RotatingHfSequenceParam,
    pub minimum_hfi_response_a: f32,
    pub maximum_hfi_electrical_speed_rad_s: f32,
    pub hfi_axis_stable_samples: u32,
    pub polarity: HfiPolarityConfig,
    pub fusion: SensorlessFusionConfig,
}

impl Default for SensorlessChainConfig {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: SENSORLESS_CHAIN_CONFIG_VERSION,
            hfi: RotatingHfSequenceParam {
                amplitude: 1.0,
                freq_hz: 1_000.0,
                ts: 1.0 / 12_000.0,
                demod_alpha: 0.05,
                phase_offset_rad: 0.0,
            },
            minimum_hfi_response_a: 0.01,
            maximum_hfi_electrical_speed_rad_s: 180.0,
            hfi_axis_stable_samples: 24,
            polarity: HfiPolarityConfig::default(),
            fusion: SensorlessFusionConfig::default(),
        }
    }
}

impl SensorlessChainConfig {
    pub fn is_valid(&self) -> bool {
        self.struct_size == size_of::<Self>() as u32
            && self.version == SENSORLESS_CHAIN_CONFIG_VERSION
            && finite_positive(self.hfi.amplitude)
            && finite_positive(self.hfi.freq_hz)
            && finite_positive(self.hfi.ts)
            && self.hfi.freq_hz < 0.5 / self.hfi.ts
            && self.hfi.demod_alpha.is_finite()
            && self.hfi.demod_alpha > 0.0
            && self.hfi.demod_alpha <= 1.0
            && self.hfi.phase_offset_rad.is_finite()
            && finite_positive(self.minimum_hfi_response_a)
            && finite_positive(self.maximum_hfi_electrical_speed_rad_s)
            && self.maximum_hfi_electrical_speed_rad_s >= self.fusion.blend_enter_speed_rad_s
            && self.hfi_axis_stable_samples > 0
            && self.polarity.is_valid()
            && self.fusion.is_valid()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
/// Measurement-only split of one chain step, in free-running cycle counts.
///
/// The chain costs about 7,080 cycles of a 12,750-cycle frame, so deciding
/// whether its work can be decimated needs to know *which* part is expensive.
/// The caller injects a counter and reads this back; the values are diagnostics
/// and no control decision may depend on them.
pub struct SensorlessChainTiming {
    /// From entry to the end of the stage match, i.e. the HFI update (or the
    /// polarity step) plus the surrounding bookkeeping.
    pub hfi_cycles: u32,
    /// The fusion supervisor, which advances the angle/speed estimate.
    pub fusion_cycles: u32,
    /// The high-frequency current separator, which runs in the ABI layer just
    /// before this step.  Filled by the caller, not here.
    pub separator_cycles: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessChainInput {
    pub request_reset: bool,
    pub injection_permitted: bool,
    pub high_frequency_current_alpha_beta: AlphaBeta,
    pub measured_current_alpha_beta: AlphaBeta,
    pub bemf_angle_rad: f32,
    pub bemf_valid: bool,
    pub bemf_electrical_speed_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessChainOutput {
    pub injection_voltage_alpha_beta: AlphaBeta,
    pub stage: SensorlessChainStage,
    pub failure: SensorlessChainFailure,
    pub polarity_failure: HfiPolarityFailure,
    pub hfi_angle_mod_pi_rad: f32,
    pub hfi_response_a: f32,
    pub hfi_valid: bool,
    pub polarity_resolved: bool,
    pub hfi_add_pi: bool,
    pub fusion: SensorlessFusionOutput,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessAngleChain {
    stage: SensorlessChainStage,
    failure: SensorlessChainFailure,
    hfi: RotatingHfSequenceState,
    polarity: HfiPolaritySupervisor,
    fusion: SensorlessFusionSupervisor,
    axis_stable_samples: u32,
    candidate_axis_rad: f32,
    polarity_started: bool,
    polarity_resolved: bool,
    hfi_add_pi: bool,
    tracking_samples: u32,
}

impl SensorlessAngleChain {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn step(
        &mut self,
        config: &SensorlessChainConfig,
        input: SensorlessChainInput,
    ) -> SensorlessChainOutput {
        // The counter is injected rather than read here so this crate keeps
        // touching no peripheral register; see `SensorlessChainTiming`.
        self.step_measured(config, input, || 0, &mut SensorlessChainTiming::default())
    }

    /// [`Self::step`] with the measurement split filled in.  `mark` must be a
    /// free-running cycle counter; callers that do not measure pass `|| 0`.
    pub fn step_measured(
        &mut self,
        config: &SensorlessChainConfig,
        input: SensorlessChainInput,
        mark: fn() -> u32,
        timing: &mut SensorlessChainTiming,
    ) -> SensorlessChainOutput {
        *timing = SensorlessChainTiming::default();
        let entry = mark();
        if input.request_reset {
            self.reset();
            return self.output(AlphaBeta::default(), HfiPolarityFailure::None, false);
        }
        if !config.is_valid() {
            return self.fail(
                SensorlessChainFailure::InvalidConfig,
                HfiPolarityFailure::None,
            );
        }
        if !finite_alpha_beta(input.high_frequency_current_alpha_beta)
            || !finite_alpha_beta(input.measured_current_alpha_beta)
            || !input.bemf_angle_rad.is_finite()
            || !input.bemf_electrical_speed_rad_s.is_finite()
        {
            return self.fail(
                SensorlessChainFailure::InvalidInput,
                HfiPolarityFailure::None,
            );
        }

        let speed_abs = input.bemf_electrical_speed_rad_s.abs();
        let hfi_speed_allowed = speed_abs <= config.maximum_hfi_electrical_speed_rad_s;
        let was_tracking = self.stage == SensorlessChainStage::Tracking;
        let mut voltage = AlphaBeta::default();
        let mut polarity_failure = HfiPolarityFailure::None;
        let mut hfi_valid = false;

        match self.stage {
            SensorlessChainStage::AxisAcquisition => {
                if input.injection_permitted && hfi_speed_allowed {
                    voltage = self
                        .hfi
                        .update_fast(&config.hfi, input.high_frequency_current_alpha_beta);
                    let response_good =
                        self.hfi.response_magnitude_a >= config.minimum_hfi_response_a;
                    self.axis_stable_samples =
                        stable_count(self.axis_stable_samples, response_good);
                    if self.axis_stable_samples >= config.hfi_axis_stable_samples {
                        self.candidate_axis_rad = self.hfi.theta_est_mod_pi_rad;
                        self.stage = SensorlessChainStage::Polarity;
                        self.polarity_started = false;
                        voltage = AlphaBeta::default();
                    }
                } else {
                    self.axis_stable_samples = 0;
                }
            }
            SensorlessChainStage::Polarity => {
                let polarity = self.polarity.step(
                    &config.polarity,
                    HfiPolarityInput {
                        request_start: !self.polarity_started,
                        request_clear: false,
                        injection_permitted: input.injection_permitted && hfi_speed_allowed,
                        candidate_axis_rad: self.candidate_axis_rad,
                        measured_current_alpha_beta: input.measured_current_alpha_beta,
                    },
                );
                self.polarity_started = true;
                voltage = polarity.injection_voltage_alpha_beta;
                polarity_failure = polarity.failure;
                if polarity.state == HfiPolarityState::Resolved {
                    self.polarity_resolved = true;
                    self.hfi_add_pi = polarity.hfi_add_pi;
                    self.stage = SensorlessChainStage::Tracking;
                    self.axis_stable_samples = 0;
                    self.tracking_samples = 0;
                    self.hfi.reset();
                    voltage = AlphaBeta::default();
                } else if polarity.state == HfiPolarityState::Failed {
                    return self.fail(SensorlessChainFailure::Polarity, polarity.failure);
                }
            }
            SensorlessChainStage::Tracking => {
                self.tracking_samples = self.tracking_samples.saturating_add(1);
                if input.injection_permitted && hfi_speed_allowed {
                    voltage = self
                        .hfi
                        .update_fast(&config.hfi, input.high_frequency_current_alpha_beta);
                    let response_good =
                        self.hfi.response_magnitude_a >= config.minimum_hfi_response_a;
                    self.axis_stable_samples =
                        stable_count(self.axis_stable_samples, response_good);
                    hfi_valid = self.axis_stable_samples >= config.hfi_axis_stable_samples;
                } else {
                    self.axis_stable_samples = 0;
                }
            }
            SensorlessChainStage::Failed => {}
        }

        let after_stage = mark();
        let fusion = self.fusion.step(
            &config.fusion,
            SensorlessFusionInput {
                hfi_angle_mod_pi_rad: self.hfi.theta_est_mod_pi_rad,
                hfi_valid,
                hfi_polarity_resolved: self.polarity_resolved,
                hfi_add_pi: self.hfi_add_pi,
                bemf_angle_rad: input.bemf_angle_rad,
                bemf_valid: input.bemf_valid,
                electrical_speed_abs_rad_s: speed_abs,
            },
        );
        let output = self.output_with_fusion(voltage, polarity_failure, hfi_valid, fusion);
        let after_fusion = mark();
        timing.hfi_cycles = after_stage.wrapping_sub(entry);
        timing.fusion_cycles = after_fusion.wrapping_sub(after_stage);
        let tracking_reacquisition_expired = self.tracking_samples
            >= config
                .fusion
                .invalid_timeout_samples
                .saturating_add(config.hfi_axis_stable_samples);
        if fusion.fallback_required
            && !input.bemf_valid
            && !hfi_valid
            && was_tracking
            && (!input.injection_permitted || tracking_reacquisition_expired)
        {
            // Once both angle channels have remained unavailable for the full
            // timeout, the old mod-pi branch is no longer trustworthy.  Force a
            // fresh axis and polarity sequence before HFI may become reliable.
            self.hfi.reset();
            self.polarity.reset();
            self.fusion.reset();
            self.stage = SensorlessChainStage::AxisAcquisition;
            self.axis_stable_samples = 0;
            self.polarity_started = false;
            self.polarity_resolved = false;
            self.hfi_add_pi = false;
            self.tracking_samples = 0;
        }
        output
    }

    fn fail(
        &mut self,
        failure: SensorlessChainFailure,
        polarity_failure: HfiPolarityFailure,
    ) -> SensorlessChainOutput {
        self.stage = SensorlessChainStage::Failed;
        self.failure = failure;
        self.output(AlphaBeta::default(), polarity_failure, false)
    }

    fn output(
        &self,
        voltage: AlphaBeta,
        polarity_failure: HfiPolarityFailure,
        hfi_valid: bool,
    ) -> SensorlessChainOutput {
        self.output_with_fusion(
            voltage,
            polarity_failure,
            hfi_valid,
            SensorlessFusionOutput::default(),
        )
    }

    fn output_with_fusion(
        &self,
        voltage: AlphaBeta,
        polarity_failure: HfiPolarityFailure,
        hfi_valid: bool,
        fusion: SensorlessFusionOutput,
    ) -> SensorlessChainOutput {
        SensorlessChainOutput {
            injection_voltage_alpha_beta: voltage,
            stage: self.stage,
            failure: self.failure,
            polarity_failure,
            hfi_angle_mod_pi_rad: self.hfi.theta_est_mod_pi_rad,
            hfi_response_a: self.hfi.response_magnitude_a,
            hfi_valid,
            polarity_resolved: self.polarity_resolved,
            hfi_add_pi: self.hfi_add_pi,
            fusion,
        }
    }
}

fn stable_count(current: u32, valid: bool) -> u32 {
    if valid {
        current.saturating_add(1)
    } else {
        0
    }
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_alpha_beta(value: AlphaBeta) -> bool {
    value.alpha.is_finite() && value.beta.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_algorithm::wrap_angle_minus_pi_to_pi;

    fn hfi_current(previous_voltage: AlphaBeta, theta: f32, response: f32) -> AlphaBeta {
        if previous_voltage.alpha == 0.0 && previous_voltage.beta == 0.0 {
            return AlphaBeta::default();
        }
        let carrier = libm::atan2f(previous_voltage.beta, previous_voltage.alpha);
        let phase = 2.0 * theta - carrier;
        AlphaBeta {
            alpha: response * libm::cosf(phase),
            beta: response * libm::sinf(phase),
        }
    }

    fn polarity_current(previous_voltage: AlphaBeta, axis: f32, add_pi: bool) -> AlphaBeta {
        let projection =
            previous_voltage.alpha * libm::cosf(axis) + previous_voltage.beta * libm::sinf(axis);
        if projection == 0.0 {
            return AlphaBeta::default();
        }
        let stronger_positive = !add_pi;
        let magnitude = if (projection > 0.0) == stronger_positive {
            0.12
        } else {
            0.05
        };
        AlphaBeta {
            alpha: projection.signum() * magnitude * libm::cosf(axis),
            beta: projection.signum() * magnitude * libm::sinf(axis),
        }
    }

    #[test]
    fn runs_axis_polarity_and_hfi_tracking_in_one_owner() {
        let config = SensorlessChainConfig {
            hfi_axis_stable_samples: 3,
            polarity: HfiPolarityConfig {
                pulse_ticks: 2,
                minimum_off_ticks: 1,
                settle_timeout_ticks: 4,
                ..HfiPolarityConfig::default()
            },
            fusion: SensorlessFusionConfig {
                stable_samples: 2,
                invalid_timeout_samples: 5,
                ..SensorlessFusionConfig::default()
            },
            ..SensorlessChainConfig::default()
        };
        let theta = 3.7;
        let mut chain = SensorlessAngleChain::default();
        let mut previous_voltage = AlphaBeta::default();
        let mut output = SensorlessChainOutput::default();
        for _ in 0..100 {
            let high_frequency = hfi_current(previous_voltage, theta, 0.08);
            let measured = if output.stage == SensorlessChainStage::Polarity {
                polarity_current(previous_voltage, output.hfi_angle_mod_pi_rad, true)
            } else {
                high_frequency
            };
            output = chain.step(
                &config,
                SensorlessChainInput {
                    injection_permitted: true,
                    high_frequency_current_alpha_beta: high_frequency,
                    measured_current_alpha_beta: measured,
                    bemf_angle_rad: theta,
                    bemf_valid: false,
                    bemf_electrical_speed_rad_s: 0.0,
                    ..SensorlessChainInput::default()
                },
            );
            previous_voltage = output.injection_voltage_alpha_beta;
            if output.fusion.reliable {
                break;
            }
        }
        assert_eq!(output.stage, SensorlessChainStage::Tracking);
        assert!(output.polarity_resolved);
        assert!(output.hfi_add_pi);
        assert!(output.fusion.reliable);
        assert!(wrap_angle_minus_pi_to_pi(output.fusion.angle_rad - theta).abs() < 0.05);
    }

    #[test]
    fn high_speed_bemf_can_run_while_injection_is_forbidden() {
        let config = SensorlessChainConfig {
            fusion: SensorlessFusionConfig {
                stable_samples: 2,
                invalid_timeout_samples: 4,
                ..SensorlessFusionConfig::default()
            },
            ..SensorlessChainConfig::default()
        };
        let mut chain = SensorlessAngleChain::default();
        let mut output = SensorlessChainOutput::default();
        for _ in 0..2 {
            output = chain.step(
                &config,
                SensorlessChainInput {
                    injection_permitted: false,
                    bemf_angle_rad: 1.2,
                    bemf_valid: true,
                    bemf_electrical_speed_rad_s: 200.0,
                    ..SensorlessChainInput::default()
                },
            );
        }
        assert_eq!(output.stage, SensorlessChainStage::AxisAcquisition);
        assert_eq!(output.injection_voltage_alpha_beta, AlphaBeta::default());
        assert!(output.fusion.reliable);
        assert_eq!(output.fusion.angle_rad, 1.2);
    }

    #[test]
    fn invalid_input_latches_failure_and_zeroes_injection() {
        let mut chain = SensorlessAngleChain::default();
        let output = chain.step(
            &SensorlessChainConfig::default(),
            SensorlessChainInput {
                injection_permitted: true,
                high_frequency_current_alpha_beta: AlphaBeta {
                    alpha: f32::NAN,
                    beta: 0.0,
                },
                ..SensorlessChainInput::default()
            },
        );
        assert_eq!(output.stage, SensorlessChainStage::Failed);
        assert_eq!(output.failure, SensorlessChainFailure::InvalidInput);
        assert_eq!(output.injection_voltage_alpha_beta, AlphaBeta::default());
    }
}
