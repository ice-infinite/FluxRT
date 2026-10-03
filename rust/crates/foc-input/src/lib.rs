#![no_std]
#![forbid(unsafe_code)]

//! MCU-independent normalization for simple external motor-command inputs.
//!
//! This crate owns only deterministic conversion and validation. Timer capture,
//! ADC reads, GPIO direction, timeouts, source arbitration, arming and PWM remain
//! outside. A valid sample becomes a setpoint candidate; it never means arm.

use core::f32::consts::TAU;

pub const CONTROL_MODE_TORQUE: u32 = 4;
pub const CONTROL_MODE_VELOCITY: u32 = 5;
pub const CONTROL_MODE_POSITION: u32 = 6;

pub const INPUT_QUALITY_CALIBRATED: u32 = 1 << 0;
pub const INPUT_QUALITY_CENTERED: u32 = 1 << 1;
pub const INPUT_QUALITY_KNOWN_MASK: u32 = INPUT_QUALITY_CALIBRATED | INPUT_QUALITY_CENTERED;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InputError {
    InvalidConfig = 1,
    OutOfRange = 2,
    InvalidValue = 3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CenteredInputConfig {
    pub raw_min: i32,
    pub raw_neutral: i32,
    pub raw_max: i32,
    pub deadband: u32,
}

impl CenteredInputConfig {
    pub fn validate(&self) -> Result<(), InputError> {
        let negative_span = i64::from(self.raw_neutral) - i64::from(self.raw_min);
        let positive_span = i64::from(self.raw_max) - i64::from(self.raw_neutral);
        let deadband = i64::from(self.deadband);
        if negative_span <= 0
            || positive_span <= 0
            || deadband >= negative_span
            || deadband >= positive_span
        {
            return Err(InputError::InvalidConfig);
        }
        Ok(())
    }

    pub fn normalize(&self, raw: i32) -> Result<(f32, u32), InputError> {
        self.validate()?;
        if raw < self.raw_min || raw > self.raw_max {
            return Err(InputError::OutOfRange);
        }

        let raw = i64::from(raw);
        let minimum = i64::from(self.raw_min);
        let neutral = i64::from(self.raw_neutral);
        let maximum = i64::from(self.raw_max);
        let deadband = i64::from(self.deadband);
        let lower = neutral - deadband;
        let upper = neutral + deadband;
        if raw >= lower && raw <= upper {
            return Ok((0.0, INPUT_QUALITY_CALIBRATED | INPUT_QUALITY_CENTERED));
        }
        let normalized = if raw < lower {
            -((lower - raw) as f32 / (lower - minimum) as f32)
        } else {
            (raw - upper) as f32 / (maximum - upper) as f32
        };
        if !normalized.is_finite() || !(-1.0..=1.0).contains(&normalized) {
            return Err(InputError::InvalidValue);
        }
        Ok((normalized, INPUT_QUALITY_CALIBRATED))
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandMapConfig {
    pub control_mode: u32,
    /// Positive magnitude applied to normalized values in `[-1, 0)`.
    pub negative_limit_si: f32,
    /// Positive magnitude applied to normalized values in `(0, 1]`.
    pub positive_limit_si: f32,
}

impl CommandMapConfig {
    pub fn validate(&self) -> Result<(), InputError> {
        if !matches!(
            self.control_mode,
            CONTROL_MODE_TORQUE | CONTROL_MODE_VELOCITY | CONTROL_MODE_POSITION
        ) || !self.negative_limit_si.is_finite()
            || !self.positive_limit_si.is_finite()
            || self.negative_limit_si <= 0.0
            || self.positive_limit_si <= 0.0
        {
            return Err(InputError::InvalidConfig);
        }
        Ok(())
    }

    pub fn map(&self, normalized: f32) -> Result<f32, InputError> {
        self.validate()?;
        if !normalized.is_finite() || !(-1.0..=1.0).contains(&normalized) {
            return Err(InputError::InvalidValue);
        }
        Ok(if normalized < 0.0 {
            normalized * self.negative_limit_si
        } else {
            normalized * self.positive_limit_si
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CenteredInputOutput {
    pub normalized: f32,
    pub setpoint_si: f32,
    pub quality_flags: u32,
}

pub fn normalize_centered_input(
    input: CenteredInputConfig,
    mapping: CommandMapConfig,
    raw: i32,
) -> Result<CenteredInputOutput, InputError> {
    let (normalized, quality_flags) = input.normalize(raw)?;
    Ok(CenteredInputOutput {
        normalized,
        setpoint_si: mapping.map(normalized)?,
        quality_flags,
    })
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepDirConfig {
    pub full_steps_per_revolution: u32,
    pub microsteps: u32,
    /// Driven-shaft revolutions are `gear_denominator / gear_numerator`
    /// times motor revolutions.
    pub gear_numerator: u32,
    pub gear_denominator: u32,
    pub direction: i32,
    pub zero_count: i32,
    pub zero_position_rad: f32,
}

impl StepDirConfig {
    pub fn validate(&self) -> Result<(), InputError> {
        if self.full_steps_per_revolution == 0
            || self.microsteps == 0
            || self.gear_numerator == 0
            || self.gear_denominator == 0
            || !matches!(self.direction, -1 | 1)
            || !self.zero_position_rad.is_finite()
        {
            return Err(InputError::InvalidConfig);
        }
        Ok(())
    }

    pub fn position_rad(&self, count: i32) -> Result<f32, InputError> {
        self.validate()?;
        let denominator = (self.full_steps_per_revolution as f32)
            * (self.microsteps as f32)
            * (self.gear_numerator as f32);
        let radians_per_count = TAU * (self.gear_denominator as f32) / denominator;
        let delta = (i64::from(count) - i64::from(self.zero_count)) as f32;
        let position = self.zero_position_rad + delta * radians_per_count * self.direction as f32;
        if !position.is_finite() {
            return Err(InputError::InvalidValue);
        }
        Ok(position)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn centered() -> CenteredInputConfig {
        CenteredInputConfig {
            raw_min: 1_000,
            raw_neutral: 1_500,
            raw_max: 2_000,
            deadband: 20,
        }
    }

    fn torque_map() -> CommandMapConfig {
        CommandMapConfig {
            control_mode: CONTROL_MODE_TORQUE,
            negative_limit_si: 0.5,
            positive_limit_si: 0.8,
        }
    }

    #[test]
    fn asymmetric_centered_input_reaches_both_endpoints() {
        let input = centered();
        assert_eq!(input.normalize(1_000).unwrap().0, -1.0);
        assert_eq!(input.normalize(2_000).unwrap().0, 1.0);
        assert_eq!(input.normalize(1_500).unwrap().0, 0.0);
    }

    #[test]
    fn deadband_is_zero_and_marked_centered() {
        for raw in [1_480, 1_500, 1_520] {
            let (normalized, quality) = centered().normalize(raw).unwrap();
            assert_eq!(normalized, 0.0);
            assert_ne!(quality & INPUT_QUALITY_CENTERED, 0);
        }
    }

    #[test]
    fn out_of_range_and_bad_calibration_fail_closed() {
        assert_eq!(centered().normalize(999), Err(InputError::OutOfRange));
        let mut invalid = centered();
        invalid.raw_neutral = invalid.raw_min;
        assert_eq!(invalid.validate(), Err(InputError::InvalidConfig));
        invalid = centered();
        invalid.deadband = 500;
        assert_eq!(invalid.validate(), Err(InputError::InvalidConfig));
    }

    #[test]
    fn physical_mapping_uses_independent_direction_limits() {
        let negative = normalize_centered_input(centered(), torque_map(), 1_000).unwrap();
        let positive = normalize_centered_input(centered(), torque_map(), 2_000).unwrap();
        assert_eq!(negative.setpoint_si, -0.5);
        assert_eq!(positive.setpoint_si, 0.8);
    }

    #[test]
    fn mapping_rejects_unknown_modes_and_non_finite_limits() {
        let mut mapping = torque_map();
        mapping.control_mode = 99;
        assert_eq!(mapping.validate(), Err(InputError::InvalidConfig));
        mapping = torque_map();
        mapping.positive_limit_si = f32::NAN;
        assert_eq!(mapping.validate(), Err(InputError::InvalidConfig));
        assert_eq!(torque_map().map(1.1), Err(InputError::InvalidValue));
    }

    #[test]
    fn step_dir_converts_counts_to_output_radians() {
        let config = StepDirConfig {
            full_steps_per_revolution: 200,
            microsteps: 16,
            gear_numerator: 2,
            gear_denominator: 1,
            direction: 1,
            zero_count: 100,
            zero_position_rad: 0.25,
        };
        let position = config.position_rad(6_500).unwrap();
        assert!((position - (0.25 + TAU)).abs() < 1.0e-5);
    }

    #[test]
    fn step_dir_direction_and_zero_are_explicit() {
        let config = StepDirConfig {
            full_steps_per_revolution: 100,
            microsteps: 1,
            gear_numerator: 1,
            gear_denominator: 1,
            direction: -1,
            zero_count: 10,
            zero_position_rad: 1.0,
        };
        assert!((config.position_rad(10).unwrap() - 1.0).abs() < f32::EPSILON);
        assert!(config.position_rad(35).unwrap() < 1.0);
    }

    #[test]
    fn step_dir_rejects_zero_scale_and_bad_direction() {
        let mut config = StepDirConfig {
            full_steps_per_revolution: 200,
            microsteps: 0,
            gear_numerator: 1,
            gear_denominator: 1,
            direction: 1,
            zero_count: 0,
            zero_position_rad: 0.0,
        };
        assert_eq!(config.validate(), Err(InputError::InvalidConfig));
        config.microsteps = 1;
        config.direction = 0;
        assert_eq!(config.validate(), Err(InputError::InvalidConfig));
    }
}
