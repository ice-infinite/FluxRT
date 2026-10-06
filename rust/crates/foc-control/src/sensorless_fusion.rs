//! HFI low-speed and BEMF/SMO medium/high-speed angle fusion supervisor.
//!
//! This module owns trust, hysteresis, dwell and fallback policy.  It does not
//! generate injection voltage and it does not decide whether a motor is salient;
//! those remain algorithm/configuration prerequisites.  HFI input is rejected
//! until a separate polarity-identification stage resolves the inherent `pi`
//! ambiguity.

use core::mem::size_of;

use foc_algorithm::{wrap_angle_0_to_2pi, wrap_angle_minus_pi_to_pi};

pub const SENSORLESS_FUSION_CONFIG_VERSION: u32 = 1;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SensorlessAngleSource {
    #[default]
    Unavailable = 0,
    Hfi = 1,
    Blend = 2,
    Bemf = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorlessFusionConfig {
    pub struct_size: u32,
    pub version: u32,
    /// HFI -> Blend threshold on absolute electrical speed `[rad/s]`.
    pub blend_enter_speed_rad_s: f32,
    /// Blend -> HFI threshold; strictly below `blend_enter_speed_rad_s`.
    pub hfi_reenter_speed_rad_s: f32,
    /// Blend -> BEMF threshold; strictly above `blend_enter_speed_rad_s`.
    pub bemf_enter_speed_rad_s: f32,
    /// BEMF -> Blend threshold; strictly below `bemf_enter_speed_rad_s`.
    pub bemf_exit_speed_rad_s: f32,
    pub maximum_angle_disagreement_rad: f32,
    pub weight_slew_per_sample: f32,
    pub stable_samples: u32,
    pub invalid_timeout_samples: u32,
}

impl Default for SensorlessFusionConfig {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: SENSORLESS_FUSION_CONFIG_VERSION,
            blend_enter_speed_rad_s: 80.0,
            hfi_reenter_speed_rad_s: 60.0,
            bemf_enter_speed_rad_s: 140.0,
            bemf_exit_speed_rad_s: 110.0,
            maximum_angle_disagreement_rad: 0.45,
            weight_slew_per_sample: 0.01,
            stable_samples: 24,
            invalid_timeout_samples: 120,
        }
    }
}

impl SensorlessFusionConfig {
    pub fn is_valid(&self) -> bool {
        self.struct_size == size_of::<Self>() as u32
            && self.version == SENSORLESS_FUSION_CONFIG_VERSION
            && finite_nonnegative(self.hfi_reenter_speed_rad_s)
            && finite_positive(self.blend_enter_speed_rad_s)
            && finite_positive(self.bemf_exit_speed_rad_s)
            && finite_positive(self.bemf_enter_speed_rad_s)
            && self.hfi_reenter_speed_rad_s < self.blend_enter_speed_rad_s
            && self.blend_enter_speed_rad_s < self.bemf_enter_speed_rad_s
            && self.hfi_reenter_speed_rad_s < self.bemf_exit_speed_rad_s
            && self.bemf_exit_speed_rad_s < self.bemf_enter_speed_rad_s
            && finite_positive(self.maximum_angle_disagreement_rad)
            && self.maximum_angle_disagreement_rad < core::f32::consts::PI
            && finite_positive(self.weight_slew_per_sample)
            && self.weight_slew_per_sample <= 1.0
            && self.stable_samples > 0
            && self.invalid_timeout_samples >= self.stable_samples
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessFusionInput {
    pub hfi_angle_mod_pi_rad: f32,
    pub hfi_valid: bool,
    pub hfi_polarity_resolved: bool,
    /// `true` selects the opposite `pi` branch after pulse/saturation polarity
    /// identification; it is ignored while polarity is unresolved.
    pub hfi_add_pi: bool,
    pub bemf_angle_rad: f32,
    pub bemf_valid: bool,
    pub electrical_speed_abs_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessFusionOutput {
    pub angle_rad: f32,
    pub hfi_angle_rad: f32,
    pub bemf_angle_rad: f32,
    pub bemf_weight: f32,
    pub angle_disagreement_rad: f32,
    pub source: SensorlessAngleSource,
    pub reliable: bool,
    pub fallback_required: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorlessFusionSupervisor {
    source: SensorlessAngleSource,
    bemf_weight: f32,
    hfi_stable_samples: u32,
    bemf_stable_samples: u32,
    invalid_samples: u32,
    last_angle_rad: f32,
    last_hfi_angle_rad: f32,
    hfi_branch_initialized: bool,
}

impl SensorlessFusionSupervisor {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn step(
        &mut self,
        config: &SensorlessFusionConfig,
        input: SensorlessFusionInput,
    ) -> SensorlessFusionOutput {
        if !config.is_valid() || !input.electrical_speed_abs_rad_s.is_finite() {
            return self.fail_closed(config.invalid_timeout_samples);
        }
        let hfi_finite = input.hfi_angle_mod_pi_rad.is_finite();
        let bemf_finite = input.bemf_angle_rad.is_finite();
        let hfi_good = input.hfi_valid && input.hfi_polarity_resolved && hfi_finite;
        let bemf_good = input.bemf_valid && bemf_finite;
        self.hfi_stable_samples = stable_count(self.hfi_stable_samples, hfi_good);
        self.bemf_stable_samples = stable_count(self.bemf_stable_samples, bemf_good);
        let hfi_ready = self.hfi_stable_samples >= config.stable_samples;
        let bemf_ready = self.bemf_stable_samples >= config.stable_samples;
        let bemf_angle = if bemf_good {
            wrap_angle_0_to_2pi(input.bemf_angle_rad)
        } else {
            self.last_angle_rad
        };
        let hfi_angle = if hfi_good {
            let wrapped = wrap_angle_0_to_2pi(input.hfi_angle_mod_pi_rad);
            let base = if wrapped >= core::f32::consts::PI {
                wrapped - core::f32::consts::PI
            } else {
                wrapped
            };
            let branch_zero = base;
            let branch_pi = base + core::f32::consts::PI;
            // A merely-valid BEMF sample must not choose the HFI branch while
            // HFI owns the output: a near-pi observer error would otherwise be
            // converted into apparent agreement and a pi jump.  BEMF may seed
            // the branch only while it is already the trusted source, which is
            // required when HFI is reacquired after a high-speed interval.
            let reference = if self.source == SensorlessAngleSource::Bemf && bemf_good {
                bemf_angle
            } else if self.hfi_branch_initialized {
                self.last_hfi_angle_rad
            } else {
                wrap_angle_0_to_2pi(
                    base + if input.hfi_add_pi {
                        core::f32::consts::PI
                    } else {
                        0.0
                    },
                )
            };
            let selected = if wrap_angle_minus_pi_to_pi(branch_zero - reference).abs()
                <= wrap_angle_minus_pi_to_pi(branch_pi - reference).abs()
            {
                branch_zero
            } else {
                branch_pi
            };
            self.last_hfi_angle_rad = wrap_angle_0_to_2pi(selected);
            self.hfi_branch_initialized = true;
            self.last_hfi_angle_rad
        } else {
            self.last_hfi_angle_rad
        };
        let disagreement = wrap_angle_minus_pi_to_pi(bemf_angle - hfi_angle).abs();
        let channels_agree = disagreement <= config.maximum_angle_disagreement_rad;
        let speed = input.electrical_speed_abs_rad_s.max(0.0);

        self.source = match self.source {
            SensorlessAngleSource::Unavailable => {
                if hfi_ready {
                    SensorlessAngleSource::Hfi
                } else if bemf_ready && speed >= config.bemf_enter_speed_rad_s {
                    SensorlessAngleSource::Bemf
                } else {
                    SensorlessAngleSource::Unavailable
                }
            }
            SensorlessAngleSource::Hfi => {
                if !hfi_good {
                    if bemf_ready && speed >= config.bemf_exit_speed_rad_s {
                        SensorlessAngleSource::Bemf
                    } else {
                        SensorlessAngleSource::Unavailable
                    }
                } else if bemf_ready && channels_agree && speed >= config.blend_enter_speed_rad_s {
                    SensorlessAngleSource::Blend
                } else {
                    SensorlessAngleSource::Hfi
                }
            }
            SensorlessAngleSource::Blend => {
                if speed <= config.hfi_reenter_speed_rad_s && hfi_ready {
                    SensorlessAngleSource::Hfi
                } else if bemf_ready && (speed >= config.bemf_enter_speed_rad_s || !hfi_good) {
                    SensorlessAngleSource::Bemf
                } else if !bemf_good && hfi_ready {
                    SensorlessAngleSource::Hfi
                } else if !hfi_good && !bemf_good {
                    SensorlessAngleSource::Unavailable
                } else {
                    SensorlessAngleSource::Blend
                }
            }
            SensorlessAngleSource::Bemf => {
                if !bemf_good {
                    if hfi_ready {
                        SensorlessAngleSource::Hfi
                    } else {
                        SensorlessAngleSource::Unavailable
                    }
                } else if speed <= config.bemf_exit_speed_rad_s && hfi_ready && channels_agree {
                    SensorlessAngleSource::Blend
                } else {
                    SensorlessAngleSource::Bemf
                }
            }
        };

        self.bemf_weight = match self.source {
            // A single declared source must produce that source's angle in the
            // same sample.  Marking Bemf reliable while retaining a near-zero
            // blend weight would publish a stale HFI angle as trusted.
            SensorlessAngleSource::Unavailable | SensorlessAngleSource::Hfi => 0.0,
            SensorlessAngleSource::Bemf => 1.0,
            SensorlessAngleSource::Blend => {
                let target_weight = ((speed - config.blend_enter_speed_rad_s)
                    / (config.bemf_enter_speed_rad_s - config.blend_enter_speed_rad_s))
                    .clamp(0.0, 1.0);
                move_towards(
                    self.bemf_weight,
                    target_weight,
                    config.weight_slew_per_sample,
                )
            }
        };
        let reliable = match self.source {
            SensorlessAngleSource::Hfi => hfi_good,
            SensorlessAngleSource::Blend => hfi_good && bemf_good && channels_agree,
            SensorlessAngleSource::Bemf => bemf_good,
            SensorlessAngleSource::Unavailable => false,
        };
        self.invalid_samples = if reliable {
            0
        } else {
            self.invalid_samples.saturating_add(1)
        };
        let fallback_required = self.invalid_samples >= config.invalid_timeout_samples;
        if reliable {
            let delta = wrap_angle_minus_pi_to_pi(bemf_angle - hfi_angle);
            self.last_angle_rad = wrap_angle_0_to_2pi(hfi_angle + self.bemf_weight * delta);
        }
        SensorlessFusionOutput {
            angle_rad: self.last_angle_rad,
            hfi_angle_rad: hfi_angle,
            bemf_angle_rad: bemf_angle,
            bemf_weight: self.bemf_weight,
            angle_disagreement_rad: disagreement,
            source: self.source,
            reliable,
            fallback_required,
        }
    }

    fn fail_closed(&mut self, timeout_samples: u32) -> SensorlessFusionOutput {
        self.source = SensorlessAngleSource::Unavailable;
        self.invalid_samples = self.invalid_samples.saturating_add(1);
        SensorlessFusionOutput {
            angle_rad: self.last_angle_rad,
            source: self.source,
            fallback_required: timeout_samples == 0 || self.invalid_samples >= timeout_samples,
            ..SensorlessFusionOutput::default()
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

fn move_towards(current: f32, target: f32, maximum_step: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= maximum_step {
        target
    } else if delta > 0.0 {
        current + maximum_step
    } else {
        current - maximum_step
    }
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

    fn input(speed: f32) -> SensorlessFusionInput {
        SensorlessFusionInput {
            hfi_angle_mod_pi_rad: 0.2,
            hfi_valid: true,
            hfi_polarity_resolved: true,
            hfi_add_pi: false,
            bemf_angle_rad: 0.22,
            bemf_valid: true,
            electrical_speed_abs_rad_s: speed,
        }
    }

    fn settle(
        supervisor: &mut SensorlessFusionSupervisor,
        config: &SensorlessFusionConfig,
        input: SensorlessFusionInput,
    ) -> SensorlessFusionOutput {
        let mut output = SensorlessFusionOutput::default();
        for _ in 0..config.stable_samples {
            output = supervisor.step(config, input);
        }
        output
    }

    #[test]
    fn moves_hfi_blend_bemf_and_back_with_hysteresis() {
        let config = SensorlessFusionConfig::default();
        let mut supervisor = SensorlessFusionSupervisor::default();
        let low = settle(&mut supervisor, &config, input(20.0));
        assert_eq!(low.source, SensorlessAngleSource::Hfi);
        assert!(low.reliable);

        let blend = supervisor.step(&config, input(100.0));
        assert_eq!(blend.source, SensorlessAngleSource::Blend);
        assert!(blend.bemf_weight > 0.0 && blend.bemf_weight < 1.0);

        let high = supervisor.step(&config, input(160.0));
        assert_eq!(high.source, SensorlessAngleSource::Bemf);
        let return_blend = supervisor.step(&config, input(100.0));
        assert_eq!(return_blend.source, SensorlessAngleSource::Blend);
        let return_hfi = supervisor.step(&config, input(50.0));
        assert_eq!(return_hfi.source, SensorlessAngleSource::Hfi);
    }

    #[test]
    fn unresolved_hfi_polarity_is_never_accepted() {
        let config = SensorlessFusionConfig::default();
        let mut supervisor = SensorlessFusionSupervisor::default();
        let mut value = input(0.0);
        value.bemf_valid = false;
        value.hfi_polarity_resolved = false;
        let output = settle(&mut supervisor, &config, value);
        assert_eq!(output.source, SensorlessAngleSource::Unavailable);
        assert!(!output.reliable);
    }

    #[test]
    fn disagreement_blocks_hfi_to_bemf_handover() {
        let config = SensorlessFusionConfig::default();
        let mut supervisor = SensorlessFusionSupervisor::default();
        settle(&mut supervisor, &config, input(20.0));
        let mut disagree = input(100.0);
        disagree.bemf_angle_rad = 2.0;
        let output = settle(&mut supervisor, &config, disagree);
        assert_eq!(output.source, SensorlessAngleSource::Hfi);
        assert!(output.angle_disagreement_rad > config.maximum_angle_disagreement_rad);
    }

    #[test]
    fn bemf_pi_error_cannot_flip_the_polarity_owned_hfi_branch() {
        let config = SensorlessFusionConfig {
            stable_samples: 1,
            ..SensorlessFusionConfig::default()
        };
        let mut supervisor = SensorlessFusionSupervisor::default();
        let mut value = input(20.0);
        value.bemf_valid = false;
        let acquired = supervisor.step(&config, value);
        assert_eq!(acquired.source, SensorlessAngleSource::Hfi);
        assert!(wrap_angle_minus_pi_to_pi(acquired.angle_rad - 0.2).abs() < 1e-6);

        value.electrical_speed_abs_rad_s = 100.0;
        value.bemf_valid = true;
        value.bemf_angle_rad = 0.2 + core::f32::consts::PI;
        let rejected = supervisor.step(&config, value);
        assert_eq!(rejected.source, SensorlessAngleSource::Hfi);
        assert!(rejected.angle_disagreement_rad > 3.0);
        assert!(wrap_angle_minus_pi_to_pi(rejected.angle_rad - 0.2).abs() < 1e-6);
    }

    #[test]
    fn invalid_channels_request_fallback_after_timeout() {
        let config = SensorlessFusionConfig {
            stable_samples: 2,
            invalid_timeout_samples: 4,
            ..SensorlessFusionConfig::default()
        };
        let mut supervisor = SensorlessFusionSupervisor::default();
        let mut invalid = input(0.0);
        invalid.hfi_valid = false;
        invalid.bemf_valid = false;
        let mut output = SensorlessFusionOutput::default();
        for _ in 0..4 {
            output = supervisor.step(&config, invalid);
        }
        assert!(output.fallback_required);
        assert_eq!(output.source, SensorlessAngleSource::Unavailable);
    }

    #[test]
    fn shortest_angle_blend_is_continuous_across_wrap() {
        let config = SensorlessFusionConfig {
            stable_samples: 1,
            weight_slew_per_sample: 1.0,
            maximum_angle_disagreement_rad: 0.2,
            ..SensorlessFusionConfig::default()
        };
        let mut supervisor = SensorlessFusionSupervisor::default();
        let mut value = input(100.0);
        value.hfi_angle_mod_pi_rad = 0.03;
        value.hfi_add_pi = true;
        value.bemf_angle_rad = core::f32::consts::PI + 0.01;
        let first = supervisor.step(&config, value);
        assert_eq!(first.source, SensorlessAngleSource::Hfi);
        let blend = supervisor.step(&config, value);
        assert_eq!(blend.source, SensorlessAngleSource::Blend);
        assert!(blend.angle_disagreement_rad < 0.03);
    }
}
