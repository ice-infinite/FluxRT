//! Unified feedback routing and calibration lifecycle.
//!
//! This module is board-independent and deliberately owns no timer, encoder,
//! Hall GPIO, observer, Flash, RTOS, or motor actuation. Platform adapters
//! normalize source data into [`FeedbackSourceSample`]. The router applies one
//! quality policy and materializes the existing [`ProductFeedbackSnapshot`].
//! Calibration produces a reviewed update; it never mutates active config.

use core::f32::consts::TAU;
use core::mem::size_of;

use crate::config::hall_sequence_is_valid;
use crate::{
    AxisState, ConfigApplyGuard, FeedbackMode, ProductFeedbackSnapshot, CALIBRATION_ENCODER_VALID,
    CALIBRATION_HALL_VALID, PRODUCT_FEEDBACK_QUALITY_CALIBRATED, PRODUCT_FEEDBACK_QUALITY_DEGRADED,
    PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID, PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND,
    PRODUCT_FEEDBACK_QUALITY_KNOWN_MASK, PRODUCT_FEEDBACK_QUALITY_STALE,
    PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE, PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
    PRODUCT_FEEDBACK_VALID_KNOWN_MASK, PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION,
    PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY, PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION,
    PRODUCT_FEEDBACK_VERSION,
};

const HALF_RANGE: u32 = 0x8000_0000;
const FEEDBACK_SOURCE_COUNT: usize = 3;

/// Normalized, source-local sample. All positions are radians and velocities
/// radians per second. Fields without a matching valid bit must be zero.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedbackSourceSample {
    pub axis_id: u32,
    pub mode: u32,
    pub sequence: u32,
    /// Long-running telemetry timestamp in the system millisecond domain.
    pub sampled_at_ms: u32,
    /// Fast freshness timestamp in a wrapping microsecond domain.
    pub sampled_at_us: u32,
    pub valid_flags: u32,
    pub quality_flags: u32,
    pub direction: i32,
    pub pole_pair_revision: u32,
    pub mechanical_position_rad: f32,
    pub multi_turn_position_rad: f32,
    pub mechanical_velocity_rad_s: f32,
    pub electrical_angle_rad: f32,
    pub electrical_velocity_rad_s: f32,
}

impl FeedbackSourceSample {
    fn validate_shape(&self, axis_id: u32) -> Result<FeedbackMode, FeedbackError> {
        if self.axis_id != axis_id || self.sequence == 0 || self.pole_pair_revision == 0 {
            return Err(FeedbackError::InvalidHeader);
        }
        if self.valid_flags & !PRODUCT_FEEDBACK_VALID_KNOWN_MASK != 0
            || self.quality_flags & !PRODUCT_FEEDBACK_QUALITY_KNOWN_MASK != 0
        {
            return Err(FeedbackError::UnknownFlags);
        }
        let mode = FeedbackMode::try_from(self.mode).map_err(|_| FeedbackError::UnsupportedMode)?;
        if source_index(mode).is_none() {
            return Err(FeedbackError::UnsupportedMode);
        }
        if self.quality_flags & PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID != 0 {
            if !matches!(self.direction, -1 | 1) {
                return Err(FeedbackError::InvalidDirection);
            }
        } else if self.direction != 0 {
            return Err(FeedbackError::NonCanonical);
        }

        let values = [
            (
                PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION,
                self.mechanical_position_rad,
            ),
            (
                PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION,
                self.multi_turn_position_rad,
            ),
            (
                PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY,
                self.mechanical_velocity_rad_s,
            ),
            (
                PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE,
                self.electrical_angle_rad,
            ),
            (
                PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
                self.electrical_velocity_rad_s,
            ),
        ];
        for (flag, value) in values {
            if self.valid_flags & flag != 0 {
                if !value.is_finite() {
                    return Err(FeedbackError::NonFinite);
                }
            } else if value != 0.0 {
                return Err(FeedbackError::NonCanonical);
            }
        }
        if self.valid_flags & PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE != 0
            && !(0.0..TAU).contains(&self.electrical_angle_rad)
        {
            return Err(FeedbackError::AngleOutOfRange);
        }
        Ok(mode)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedbackError {
    InvalidConfig,
    InvalidHeader,
    UnsupportedMode,
    UnknownFlags,
    InvalidDirection,
    NonFinite,
    NonCanonical,
    AngleOutOfRange,
    StaleSequence,
    StaleCycle,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedbackRouteState {
    Acquiring,
    Primary,
    Fallback,
    Lost,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedbackRouteDecision {
    pub state: FeedbackRouteState,
    pub snapshot: ProductFeedbackSnapshot,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FeedbackRouterConfig {
    pub axis_id: u32,
    pub primary_mode: u32,
    pub backup_mode: u32,
    pub fallback_enabled: bool,
    pub maximum_age_us: u32,
    pub acquire_good_samples: u16,
    pub loss_bad_samples: u16,
    pub recovery_good_samples: u16,
}

impl FeedbackRouterConfig {
    pub fn validate(&self) -> Result<(), FeedbackError> {
        let primary =
            FeedbackMode::try_from(self.primary_mode).map_err(|_| FeedbackError::InvalidConfig)?;
        let backup =
            FeedbackMode::try_from(self.backup_mode).map_err(|_| FeedbackError::InvalidConfig)?;
        if self.axis_id != 0
            || source_index(primary).is_none()
            || source_index(backup).is_none()
            || self.maximum_age_us == 0
            || self.maximum_age_us >= HALF_RANGE
            || self.acquire_good_samples == 0
            || self.loss_bad_samples == 0
            || self.recovery_good_samples == 0
            || (self.fallback_enabled && primary == backup)
            || (!self.fallback_enabled && primary != backup)
        {
            return Err(FeedbackError::InvalidConfig);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SourceHealth {
    Good,
    Bad,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SourceTracker {
    sample: FeedbackSourceSample,
    present: bool,
    last_sequence: u32,
    last_evaluated_sequence: u32,
    consecutive_good: u16,
    consecutive_bad: u16,
}

impl SourceTracker {
    fn ingest(&mut self, sample: FeedbackSourceSample) -> Result<(), FeedbackError> {
        if self.present && !sequence_is_newer(sample.sequence, self.last_sequence) {
            return Err(FeedbackError::StaleSequence);
        }
        self.sample = sample;
        self.present = true;
        self.last_sequence = sample.sequence;
        Ok(())
    }

    fn evaluate(&mut self, now_us: u32, maximum_age_us: u32) -> SourceHealth {
        let is_new = self.present && self.sample.sequence != self.last_evaluated_sequence;
        let good = self.present
            && elapsed(now_us, self.sample.sampled_at_us).is_some_and(|age| age <= maximum_age_us)
            && sample_meets_mode_gate(&self.sample);
        if good {
            self.consecutive_bad = 0;
            if is_new {
                self.consecutive_good = self.consecutive_good.saturating_add(1);
                self.last_evaluated_sequence = self.sample.sequence;
            }
            SourceHealth::Good
        } else {
            self.consecutive_good = 0;
            self.consecutive_bad = self.consecutive_bad.saturating_add(1);
            if is_new {
                self.last_evaluated_sequence = self.sample.sequence;
            }
            SourceHealth::Bad
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ActiveSource {
    None,
    Primary,
    Backup,
}

/// Fixed-capacity feedback source router for Sensorless, Hall and incremental
/// encoder sources. Call `ingest` for every new source sample and `route` once
/// per management/control selection cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedbackRouter {
    config: FeedbackRouterConfig,
    trackers: [SourceTracker; FEEDBACK_SOURCE_COUNT],
    active: ActiveSource,
    last_cycle_sequence: u32,
}

impl FeedbackRouter {
    pub fn new(config: FeedbackRouterConfig) -> Result<Self, FeedbackError> {
        config.validate()?;
        Ok(Self {
            config,
            trackers: [SourceTracker::default(); FEEDBACK_SOURCE_COUNT],
            active: ActiveSource::None,
            last_cycle_sequence: 0,
        })
    }

    pub fn ingest(&mut self, sample: FeedbackSourceSample) -> Result<(), FeedbackError> {
        let mode = sample.validate_shape(self.config.axis_id)?;
        let index = source_index(mode).ok_or(FeedbackError::UnsupportedMode)?;
        self.trackers[index].ingest(sample)
    }

    pub fn route(
        &mut self,
        cycle_sequence: u32,
        now_us: u32,
    ) -> Result<FeedbackRouteDecision, FeedbackError> {
        if cycle_sequence == 0
            || (self.last_cycle_sequence != 0
                && !sequence_is_newer(cycle_sequence, self.last_cycle_sequence))
        {
            return Err(FeedbackError::StaleCycle);
        }
        self.last_cycle_sequence = cycle_sequence;
        let primary_mode = FeedbackMode::try_from(self.config.primary_mode)
            .map_err(|_| FeedbackError::InvalidConfig)?;
        let backup_mode = FeedbackMode::try_from(self.config.backup_mode)
            .map_err(|_| FeedbackError::InvalidConfig)?;
        let primary_index = source_index(primary_mode).ok_or(FeedbackError::InvalidConfig)?;
        let backup_index = source_index(backup_mode).ok_or(FeedbackError::InvalidConfig)?;

        let primary_health =
            self.trackers[primary_index].evaluate(now_us, self.config.maximum_age_us);
        let backup_health = if backup_index == primary_index {
            primary_health
        } else {
            self.trackers[backup_index].evaluate(now_us, self.config.maximum_age_us)
        };
        let primary_acquired = primary_health == SourceHealth::Good
            && self.trackers[primary_index].consecutive_good >= self.config.acquire_good_samples;
        let primary_recovered = primary_health == SourceHealth::Good
            && self.trackers[primary_index].consecutive_good >= self.config.recovery_good_samples;
        let primary_lost = primary_health == SourceHealth::Bad
            && self.trackers[primary_index].consecutive_bad >= self.config.loss_bad_samples;
        let backup_acquired = self.config.fallback_enabled
            && backup_health == SourceHealth::Good
            && self.trackers[backup_index].consecutive_good >= self.config.acquire_good_samples;
        let backup_lost = backup_health == SourceHealth::Bad
            && self.trackers[backup_index].consecutive_bad >= self.config.loss_bad_samples;

        self.active = match self.active {
            ActiveSource::None => {
                if primary_acquired {
                    ActiveSource::Primary
                } else if backup_acquired {
                    ActiveSource::Backup
                } else {
                    ActiveSource::None
                }
            }
            ActiveSource::Primary => {
                if primary_lost {
                    if backup_acquired {
                        ActiveSource::Backup
                    } else {
                        ActiveSource::None
                    }
                } else {
                    ActiveSource::Primary
                }
            }
            ActiveSource::Backup => {
                if primary_recovered {
                    ActiveSource::Primary
                } else if backup_lost {
                    ActiveSource::None
                } else {
                    ActiveSource::Backup
                }
            }
        };

        let (state, sample, fallback, source_usable) = match self.active {
            ActiveSource::Primary => (
                FeedbackRouteState::Primary,
                Some(self.trackers[primary_index].sample),
                false,
                primary_health == SourceHealth::Good,
            ),
            ActiveSource::Backup => (
                FeedbackRouteState::Fallback,
                Some(self.trackers[backup_index].sample),
                true,
                backup_health == SourceHealth::Good,
            ),
            ActiveSource::None => {
                let primary_exhausted = self.trackers[primary_index].present && primary_lost;
                let backup_exhausted = !self.config.fallback_enabled
                    || !self.trackers[backup_index].present
                    || backup_lost;
                let state = if primary_exhausted && backup_exhausted {
                    FeedbackRouteState::Lost
                } else {
                    FeedbackRouteState::Acquiring
                };
                (state, None, false, false)
            }
        };
        Ok(FeedbackRouteDecision {
            state,
            snapshot: materialize_snapshot(
                self.config,
                cycle_sequence,
                now_us,
                sample,
                fallback,
                source_usable,
            ),
        })
    }
}

fn materialize_snapshot(
    config: FeedbackRouterConfig,
    cycle_sequence: u32,
    now_us: u32,
    sample: Option<FeedbackSourceSample>,
    fallback: bool,
    source_usable: bool,
) -> ProductFeedbackSnapshot {
    let mut output = ProductFeedbackSnapshot {
        struct_size: size_of::<ProductFeedbackSnapshot>() as u32,
        version: PRODUCT_FEEDBACK_VERSION,
        axis_id: config.axis_id,
        sequence: cycle_sequence,
        active_feedback_mode: config.primary_mode,
        backup_feedback_mode: config.backup_mode,
        ..ProductFeedbackSnapshot::default()
    };
    if let Some(value) = sample {
        output.sampled_at_ms = value.sampled_at_ms;
        output.sample_age_us = elapsed(now_us, value.sampled_at_us).unwrap_or(u32::MAX);
        if source_usable {
            output.valid_flags = value.valid_flags;
            output.quality_flags = value.quality_flags;
            output.direction = value.direction;
            output.pole_pair_revision = value.pole_pair_revision;
            output.mechanical_position_rad = value.mechanical_position_rad;
            output.multi_turn_position_rad = value.multi_turn_position_rad;
            output.mechanical_velocity_rad_s = value.mechanical_velocity_rad_s;
            output.electrical_angle_rad = value.electrical_angle_rad;
            output.electrical_velocity_rad_s = value.electrical_velocity_rad_s;
        } else {
            output.quality_flags = value.quality_flags | PRODUCT_FEEDBACK_QUALITY_DEGRADED;
            if output.sample_age_us > config.maximum_age_us {
                output.quality_flags |= PRODUCT_FEEDBACK_QUALITY_STALE;
            }
        }
        if fallback && source_usable {
            output.quality_flags |= PRODUCT_FEEDBACK_QUALITY_DEGRADED;
        }
        output.active_feedback_mode = value.mode;
    } else {
        output.quality_flags = PRODUCT_FEEDBACK_QUALITY_STALE;
    }
    output
}

fn sample_meets_mode_gate(sample: &FeedbackSourceSample) -> bool {
    if sample.quality_flags & (PRODUCT_FEEDBACK_QUALITY_STALE | PRODUCT_FEEDBACK_QUALITY_DEGRADED)
        != 0
    {
        return false;
    }
    let mode = match FeedbackMode::try_from(sample.mode) {
        Ok(value) => value,
        Err(_) => return false,
    };
    let common_valid = PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
        | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
        | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY;
    let direction = PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID;
    match mode {
        FeedbackMode::Sensorless => {
            sample.valid_flags & common_valid == common_valid
                && sample.quality_flags & direction == direction
        }
        FeedbackMode::Hall => {
            sample.valid_flags & common_valid == common_valid
                && sample.quality_flags & (direction | PRODUCT_FEEDBACK_QUALITY_CALIBRATED)
                    == (direction | PRODUCT_FEEDBACK_QUALITY_CALIBRATED)
        }
        FeedbackMode::IncrementalEncoder => {
            let required_valid = common_valid
                | PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION
                | PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION;
            let required_quality = direction
                | PRODUCT_FEEDBACK_QUALITY_CALIBRATED
                | PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND;
            sample.valid_flags & required_valid == required_valid
                && sample.quality_flags & required_quality == required_quality
        }
        _ => false,
    }
}

fn source_index(mode: FeedbackMode) -> Option<usize> {
    match mode {
        FeedbackMode::Sensorless => Some(0),
        FeedbackMode::Hall => Some(1),
        FeedbackMode::IncrementalEncoder => Some(2),
        FeedbackMode::AbsoluteEncoder | FeedbackMode::Resolver | FeedbackMode::Fused => None,
    }
}

fn sequence_is_newer(candidate: u32, previous: u32) -> bool {
    let distance = candidate.wrapping_sub(previous);
    distance != 0 && distance < HALF_RANGE
}

fn elapsed(now: u32, then: u32) -> Option<u32> {
    let value = now.wrapping_sub(then);
    (value < HALF_RANGE).then_some(value)
}

pub const FEEDBACK_CALIBRATION_STEP_DIRECTION: u32 = 1 << 0;
pub const FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX: u32 = 1 << 1;
pub const FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET: u32 = 1 << 2;
pub const FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE: u32 = 1 << 3;

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedbackCalibrationState {
    Idle = 0,
    Collecting = 1,
    ReadyForReview = 2,
    Approved = 3,
    Failed = 4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FeedbackCalibrationPolicy {
    pub minimum_direction_samples: u32,
    pub minimum_index_samples: u32,
    pub minimum_offset_samples: u32,
    pub minimum_hall_samples: u32,
    pub encoder_counts_per_revolution: u32,
}

impl FeedbackCalibrationPolicy {
    pub fn validate(&self) -> Result<(), FeedbackCalibrationError> {
        if self.minimum_direction_samples == 0
            || self.minimum_index_samples == 0
            || self.minimum_offset_samples == 0
            || self.minimum_hall_samples == 0
            || self.encoder_counts_per_revolution == 0
        {
            Err(FeedbackCalibrationError::InvalidPolicy)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FeedbackCalibrationError {
    InvalidPolicy,
    UnsupportedMode,
    UnsafeState,
    Busy,
    NoSession,
    StaleToken,
    InvalidState,
    InvalidEvidence,
    IncompleteEvidence,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedbackCalibrationUpdate {
    pub feedback_mode: u32,
    pub axis_direction: i32,
    pub calibration_flags_to_set: u32,
    pub encoder_offset_rad: f32,
    pub encoder_counts_per_revolution: u32,
    pub hall_sequence_packed: u32,
    pub evidence_steps: u32,
}

/// Review gate for external evidence. Actuation and sensing remain owned by a
/// dedicated calibration adapter; this manager only tracks completeness and
/// creates a configuration update while the Axis is safe.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FeedbackCalibrationManager {
    policy: FeedbackCalibrationPolicy,
    state: FeedbackCalibrationState,
    next_token: u32,
    token: u32,
    mode: FeedbackMode,
    completed_steps: u32,
    direction: i32,
    encoder_offset_rad: f32,
    hall_sequence_packed: u32,
}

impl FeedbackCalibrationManager {
    pub fn new(policy: FeedbackCalibrationPolicy) -> Result<Self, FeedbackCalibrationError> {
        policy.validate()?;
        Ok(Self {
            policy,
            state: FeedbackCalibrationState::Idle,
            next_token: 0,
            token: 0,
            mode: FeedbackMode::Sensorless,
            completed_steps: 0,
            direction: 0,
            encoder_offset_rad: 0.0,
            hall_sequence_packed: 0,
        })
    }

    pub const fn state(&self) -> FeedbackCalibrationState {
        self.state
    }

    pub const fn completed_steps(&self) -> u32 {
        self.completed_steps
    }

    pub fn begin(
        &mut self,
        mode: FeedbackMode,
        guard: ConfigApplyGuard,
    ) -> Result<u32, FeedbackCalibrationError> {
        if self.state != FeedbackCalibrationState::Idle {
            return Err(FeedbackCalibrationError::Busy);
        }
        if !calibration_guard_is_safe(guard) {
            return Err(FeedbackCalibrationError::UnsafeState);
        }
        if !matches!(mode, FeedbackMode::Hall | FeedbackMode::IncrementalEncoder) {
            return Err(FeedbackCalibrationError::UnsupportedMode);
        }
        self.next_token = self.next_token.wrapping_add(1);
        if self.next_token == 0 {
            self.next_token = 1;
        }
        self.token = self.next_token;
        self.mode = mode;
        self.completed_steps = 0;
        self.direction = 0;
        self.encoder_offset_rad = 0.0;
        self.hall_sequence_packed = 0;
        self.state = FeedbackCalibrationState::Collecting;
        Ok(self.token)
    }

    pub fn record_direction(
        &mut self,
        token: u32,
        direction: i32,
        sample_count: u32,
    ) -> Result<(), FeedbackCalibrationError> {
        self.check_collecting(token)?;
        if !matches!(direction, -1 | 1) || sample_count < self.policy.minimum_direction_samples {
            return self.fail(FeedbackCalibrationError::InvalidEvidence);
        }
        self.direction = direction;
        self.completed_steps |= FEEDBACK_CALIBRATION_STEP_DIRECTION;
        self.refresh_ready();
        Ok(())
    }

    pub fn record_encoder_index(
        &mut self,
        token: u32,
        sample_count: u32,
    ) -> Result<(), FeedbackCalibrationError> {
        self.check_collecting(token)?;
        if self.mode != FeedbackMode::IncrementalEncoder
            || sample_count < self.policy.minimum_index_samples
        {
            return self.fail(FeedbackCalibrationError::InvalidEvidence);
        }
        self.completed_steps |= FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX;
        self.refresh_ready();
        Ok(())
    }

    pub fn record_encoder_offset(
        &mut self,
        token: u32,
        offset_rad: f32,
        sample_count: u32,
    ) -> Result<(), FeedbackCalibrationError> {
        self.check_collecting(token)?;
        if self.mode != FeedbackMode::IncrementalEncoder
            || !offset_rad.is_finite()
            || !(-TAU..TAU).contains(&offset_rad)
            || sample_count < self.policy.minimum_offset_samples
        {
            return self.fail(FeedbackCalibrationError::InvalidEvidence);
        }
        self.encoder_offset_rad = offset_rad;
        self.completed_steps |= FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET;
        self.refresh_ready();
        Ok(())
    }

    pub fn record_hall_sequence(
        &mut self,
        token: u32,
        sequence_packed: u32,
        sample_count: u32,
    ) -> Result<(), FeedbackCalibrationError> {
        self.check_collecting(token)?;
        if self.mode != FeedbackMode::Hall
            || sample_count < self.policy.minimum_hall_samples
            || !hall_sequence_is_valid(sequence_packed)
        {
            return self.fail(FeedbackCalibrationError::InvalidEvidence);
        }
        self.hall_sequence_packed = sequence_packed;
        self.completed_steps |= FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE;
        self.refresh_ready();
        Ok(())
    }

    pub fn approve(
        &mut self,
        token: u32,
        guard: ConfigApplyGuard,
    ) -> Result<FeedbackCalibrationUpdate, FeedbackCalibrationError> {
        self.check_token(token)?;
        if self.state != FeedbackCalibrationState::ReadyForReview {
            return Err(FeedbackCalibrationError::IncompleteEvidence);
        }
        if !calibration_guard_is_safe(guard) {
            return Err(FeedbackCalibrationError::UnsafeState);
        }
        let update = match self.mode {
            FeedbackMode::IncrementalEncoder => FeedbackCalibrationUpdate {
                feedback_mode: self.mode as u32,
                axis_direction: self.direction,
                calibration_flags_to_set: CALIBRATION_ENCODER_VALID,
                encoder_offset_rad: self.encoder_offset_rad,
                encoder_counts_per_revolution: self.policy.encoder_counts_per_revolution,
                hall_sequence_packed: 0,
                evidence_steps: self.completed_steps,
            },
            FeedbackMode::Hall => FeedbackCalibrationUpdate {
                feedback_mode: self.mode as u32,
                axis_direction: self.direction,
                calibration_flags_to_set: CALIBRATION_HALL_VALID,
                encoder_offset_rad: 0.0,
                encoder_counts_per_revolution: 0,
                hall_sequence_packed: self.hall_sequence_packed,
                evidence_steps: self.completed_steps,
            },
            _ => return Err(FeedbackCalibrationError::UnsupportedMode),
        };
        self.state = FeedbackCalibrationState::Approved;
        Ok(update)
    }

    pub fn cancel(&mut self, token: u32) -> Result<(), FeedbackCalibrationError> {
        self.check_token(token)?;
        self.clear();
        Ok(())
    }

    pub fn reset_failed(
        &mut self,
        guard: ConfigApplyGuard,
    ) -> Result<(), FeedbackCalibrationError> {
        if self.state != FeedbackCalibrationState::Failed {
            return Err(FeedbackCalibrationError::InvalidState);
        }
        if !calibration_guard_is_safe(guard) {
            return Err(FeedbackCalibrationError::UnsafeState);
        }
        self.clear();
        Ok(())
    }

    pub fn finish_applied(
        &mut self,
        guard: ConfigApplyGuard,
    ) -> Result<(), FeedbackCalibrationError> {
        if self.state != FeedbackCalibrationState::Approved {
            return Err(FeedbackCalibrationError::InvalidState);
        }
        if !calibration_guard_is_safe(guard) {
            return Err(FeedbackCalibrationError::UnsafeState);
        }
        self.clear();
        Ok(())
    }

    fn check_token(&self, token: u32) -> Result<(), FeedbackCalibrationError> {
        if self.state == FeedbackCalibrationState::Idle {
            Err(FeedbackCalibrationError::NoSession)
        } else if token == 0 || token != self.token {
            Err(FeedbackCalibrationError::StaleToken)
        } else {
            Ok(())
        }
    }

    fn check_collecting(&self, token: u32) -> Result<(), FeedbackCalibrationError> {
        self.check_token(token)?;
        if self.state == FeedbackCalibrationState::Collecting {
            Ok(())
        } else {
            Err(FeedbackCalibrationError::InvalidState)
        }
    }

    fn fail<T>(&mut self, error: FeedbackCalibrationError) -> Result<T, FeedbackCalibrationError> {
        self.state = FeedbackCalibrationState::Failed;
        Err(error)
    }

    fn refresh_ready(&mut self) {
        if self.completed_steps & required_calibration_steps(self.mode)
            == required_calibration_steps(self.mode)
        {
            self.state = FeedbackCalibrationState::ReadyForReview;
        }
    }

    fn clear(&mut self) {
        self.state = FeedbackCalibrationState::Idle;
        self.token = 0;
        self.mode = FeedbackMode::Sensorless;
        self.completed_steps = 0;
        self.direction = 0;
        self.encoder_offset_rad = 0.0;
        self.hall_sequence_packed = 0;
    }
}

fn required_calibration_steps(mode: FeedbackMode) -> u32 {
    match mode {
        FeedbackMode::IncrementalEncoder => {
            FEEDBACK_CALIBRATION_STEP_DIRECTION
                | FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX
                | FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET
        }
        FeedbackMode::Hall => {
            FEEDBACK_CALIBRATION_STEP_DIRECTION | FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE
        }
        _ => 0,
    }
}

fn calibration_guard_is_safe(guard: ConfigApplyGuard) -> bool {
    guard.axis_state == AxisState::Disabled && !guard.drive_active && guard.active_fault_flags == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn router_config(primary: FeedbackMode, backup: FeedbackMode) -> FeedbackRouterConfig {
        FeedbackRouterConfig {
            axis_id: 0,
            primary_mode: primary as u32,
            backup_mode: backup as u32,
            fallback_enabled: primary != backup,
            maximum_age_us: 500,
            acquire_good_samples: 2,
            loss_bad_samples: 2,
            recovery_good_samples: 3,
        }
    }

    fn sensorless(sequence: u32, sampled_at_us: u32) -> FeedbackSourceSample {
        FeedbackSourceSample {
            mode: FeedbackMode::Sensorless as u32,
            sequence,
            sampled_at_ms: sampled_at_us / 1000,
            sampled_at_us,
            valid_flags: PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
                | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
                | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
            quality_flags: PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
            direction: 1,
            pole_pair_revision: 7,
            mechanical_velocity_rad_s: 10.0,
            electrical_angle_rad: 1.0,
            electrical_velocity_rad_s: 70.0,
            ..FeedbackSourceSample::default()
        }
    }

    fn encoder(sequence: u32, sampled_at_us: u32) -> FeedbackSourceSample {
        FeedbackSourceSample {
            mode: FeedbackMode::IncrementalEncoder as u32,
            sequence,
            sampled_at_ms: sampled_at_us / 1000,
            sampled_at_us,
            valid_flags: PRODUCT_FEEDBACK_VALID_KNOWN_MASK,
            quality_flags: PRODUCT_FEEDBACK_QUALITY_CALIBRATED
                | PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND
                | PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
            direction: 1,
            pole_pair_revision: 8,
            mechanical_position_rad: 0.25,
            multi_turn_position_rad: 12.5,
            mechanical_velocity_rad_s: 10.0,
            electrical_angle_rad: 1.75,
            electrical_velocity_rad_s: 70.0,
            ..FeedbackSourceSample::default()
        }
    }

    fn hall(sequence: u32, sampled_at_us: u32) -> FeedbackSourceSample {
        FeedbackSourceSample {
            mode: FeedbackMode::Hall as u32,
            sequence,
            sampled_at_ms: sampled_at_us / 1000,
            sampled_at_us,
            valid_flags: PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
                | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
                | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
            quality_flags: PRODUCT_FEEDBACK_QUALITY_CALIBRATED
                | PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
            direction: -1,
            pole_pair_revision: 3,
            mechanical_velocity_rad_s: -5.0,
            electrical_angle_rad: 5.0,
            electrical_velocity_rad_s: -35.0,
            ..FeedbackSourceSample::default()
        }
    }

    fn safe_guard() -> ConfigApplyGuard {
        ConfigApplyGuard::disabled()
    }

    #[test]
    fn sensorless_requires_distinct_new_good_samples_before_acquisition() {
        let mut router = FeedbackRouter::new(router_config(
            FeedbackMode::Sensorless,
            FeedbackMode::Sensorless,
        ))
        .unwrap();
        router.ingest(sensorless(1, 100)).unwrap();
        let first = router.route(1, 100).unwrap();
        assert_eq!(first.state, FeedbackRouteState::Acquiring);
        assert_eq!(first.snapshot.valid_flags, 0);
        assert_eq!(first.snapshot.struct_size, 68);

        let same = router.route(2, 110).unwrap();
        assert_eq!(same.state, FeedbackRouteState::Acquiring);
        let mut second = sensorless(2, 120);
        second.sampled_at_ms = 42_000;
        router.ingest(second).unwrap();
        let acquired = router.route(3, 120).unwrap();
        assert_eq!(acquired.state, FeedbackRouteState::Primary);
        assert_eq!(acquired.snapshot.active_feedback_mode, 0);
        assert_eq!(acquired.snapshot.sampled_at_ms, 42_000);
        assert_eq!(acquired.snapshot.mechanical_velocity_rad_s, 10.0);
    }

    #[test]
    fn encoder_loss_falls_back_then_recovery_uses_hysteresis() {
        let mut router = FeedbackRouter::new(router_config(
            FeedbackMode::IncrementalEncoder,
            FeedbackMode::Sensorless,
        ))
        .unwrap();
        for sequence in 1..=2 {
            router.ingest(encoder(sequence, sequence * 100)).unwrap();
            router.ingest(sensorless(sequence, sequence * 100)).unwrap();
            router.route(sequence, sequence * 100).unwrap();
        }
        assert_eq!(router.active, ActiveSource::Primary);

        router.ingest(sensorless(3, 701)).unwrap();
        assert_eq!(
            router.route(3, 701).unwrap().state,
            FeedbackRouteState::Primary
        );
        router.ingest(sensorless(4, 702)).unwrap();
        let fallback = router.route(4, 702).unwrap();
        assert_eq!(fallback.state, FeedbackRouteState::Fallback);
        assert_eq!(
            fallback.snapshot.active_feedback_mode,
            FeedbackMode::Sensorless as u32
        );
        assert_ne!(
            fallback.snapshot.quality_flags & PRODUCT_FEEDBACK_QUALITY_DEGRADED,
            0
        );

        for (sequence, timestamp) in [(5, 800), (6, 900)] {
            router.ingest(encoder(sequence, timestamp)).unwrap();
            router.ingest(sensorless(sequence, timestamp)).unwrap();
            assert_eq!(
                router.route(sequence, timestamp).unwrap().state,
                FeedbackRouteState::Fallback
            );
        }
        router.ingest(encoder(7, 1000)).unwrap();
        router.ingest(sensorless(7, 1000)).unwrap();
        assert_eq!(
            router.route(7, 1000).unwrap().state,
            FeedbackRouteState::Primary
        );
    }

    #[test]
    fn stale_sequence_unknown_mode_and_noncanonical_fields_are_rejected() {
        let mut router = FeedbackRouter::new(router_config(
            FeedbackMode::Sensorless,
            FeedbackMode::Sensorless,
        ))
        .unwrap();
        router.ingest(sensorless(2, 100)).unwrap();
        assert_eq!(
            router.ingest(sensorless(2, 110)),
            Err(FeedbackError::StaleSequence)
        );
        let mut bad = sensorless(3, 120);
        bad.mode = FeedbackMode::AbsoluteEncoder as u32;
        assert_eq!(router.ingest(bad), Err(FeedbackError::UnsupportedMode));
        bad = sensorless(3, 120);
        bad.valid_flags &= !PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY;
        assert_eq!(router.ingest(bad), Err(FeedbackError::NonCanonical));
        assert_eq!(
            router.route(2, 100).unwrap().state,
            FeedbackRouteState::Acquiring
        );
        assert_eq!(router.route(2, 101), Err(FeedbackError::StaleCycle));
    }

    #[test]
    fn hall_quality_and_stale_age_fail_closed() {
        let mut router =
            FeedbackRouter::new(router_config(FeedbackMode::Hall, FeedbackMode::Hall)).unwrap();
        let mut sample = hall(1, 100);
        sample.quality_flags &= !PRODUCT_FEEDBACK_QUALITY_CALIBRATED;
        router.ingest(sample).unwrap();
        assert_eq!(
            router.route(1, 100).unwrap().state,
            FeedbackRouteState::Acquiring
        );
        router.ingest(hall(2, 200)).unwrap();
        assert_eq!(
            router.route(2, 200).unwrap().state,
            FeedbackRouteState::Acquiring
        );
        router.ingest(hall(3, 300)).unwrap();
        assert_eq!(
            router.route(3, 300).unwrap().state,
            FeedbackRouteState::Primary
        );
        let grace = router.route(4, 900).unwrap();
        assert_eq!(grace.state, FeedbackRouteState::Primary);
        assert_eq!(grace.snapshot.valid_flags, 0);
        assert_ne!(
            grace.snapshot.quality_flags & PRODUCT_FEEDBACK_QUALITY_STALE,
            0
        );
        assert_ne!(
            grace.snapshot.quality_flags & PRODUCT_FEEDBACK_QUALITY_DEGRADED,
            0
        );
        assert_eq!(
            router.route(5, 901).unwrap().state,
            FeedbackRouteState::Lost
        );
    }

    #[test]
    fn encoder_calibration_requires_direction_index_and_offset_before_review() {
        let mut manager = FeedbackCalibrationManager::new(FeedbackCalibrationPolicy {
            minimum_direction_samples: 8,
            minimum_index_samples: 2,
            minimum_offset_samples: 16,
            minimum_hall_samples: 12,
            encoder_counts_per_revolution: 4096,
        })
        .unwrap();
        let token = manager
            .begin(FeedbackMode::IncrementalEncoder, safe_guard())
            .unwrap();
        manager.record_encoder_index(token, 2).unwrap();
        manager.record_direction(token, -1, 8).unwrap();
        assert_eq!(manager.state(), FeedbackCalibrationState::Collecting);
        assert_eq!(
            manager.approve(token, safe_guard()),
            Err(FeedbackCalibrationError::IncompleteEvidence)
        );
        manager.record_encoder_offset(token, 0.3, 16).unwrap();
        assert_eq!(manager.state(), FeedbackCalibrationState::ReadyForReview);
        let update = manager.approve(token, safe_guard()).unwrap();
        assert_eq!(update.axis_direction, -1);
        assert_eq!(update.encoder_offset_rad, 0.3);
        assert_eq!(update.encoder_counts_per_revolution, 4096);
        assert_eq!(update.calibration_flags_to_set, CALIBRATION_ENCODER_VALID);
        assert_eq!(manager.state(), FeedbackCalibrationState::Approved);
        manager.finish_applied(safe_guard()).unwrap();
        assert_eq!(manager.state(), FeedbackCalibrationState::Idle);
    }

    #[test]
    fn hall_calibration_validates_sequence_and_latches_bad_evidence() {
        let policy = FeedbackCalibrationPolicy {
            minimum_direction_samples: 4,
            minimum_index_samples: 1,
            minimum_offset_samples: 4,
            minimum_hall_samples: 6,
            encoder_counts_per_revolution: 4096,
        };
        let mut manager = FeedbackCalibrationManager::new(policy).unwrap();
        let token = manager.begin(FeedbackMode::Hall, safe_guard()).unwrap();
        manager.record_direction(token, 1, 4).unwrap();
        assert_eq!(
            manager.record_hall_sequence(token, 0x0054_3211, 6),
            Err(FeedbackCalibrationError::InvalidEvidence)
        );
        assert_eq!(manager.state(), FeedbackCalibrationState::Failed);
        manager.reset_failed(safe_guard()).unwrap();

        let token = manager.begin(FeedbackMode::Hall, safe_guard()).unwrap();
        manager.record_direction(token, 1, 4).unwrap();
        manager.record_hall_sequence(token, 0x0054_3210, 6).unwrap();
        let update = manager.approve(token, safe_guard()).unwrap();
        assert_eq!(update.calibration_flags_to_set, CALIBRATION_HALL_VALID);
        assert_eq!(update.hall_sequence_packed, 0x0054_3210);
    }

    #[test]
    fn calibration_rejects_unsafe_start_stale_token_and_unsupported_mode() {
        let mut manager = FeedbackCalibrationManager::new(FeedbackCalibrationPolicy {
            minimum_direction_samples: 1,
            minimum_index_samples: 1,
            minimum_offset_samples: 1,
            minimum_hall_samples: 1,
            encoder_counts_per_revolution: 1024,
        })
        .unwrap();
        let unsafe_guard = ConfigApplyGuard {
            axis_state: AxisState::ClosedLoop,
            drive_active: true,
            active_fault_flags: 0,
        };
        assert_eq!(
            manager.begin(FeedbackMode::Hall, unsafe_guard),
            Err(FeedbackCalibrationError::UnsafeState)
        );
        assert_eq!(
            manager.begin(FeedbackMode::Sensorless, safe_guard()),
            Err(FeedbackCalibrationError::UnsupportedMode)
        );
        let token = manager.begin(FeedbackMode::Hall, safe_guard()).unwrap();
        assert_eq!(
            manager.record_direction(token + 1, 1, 1),
            Err(FeedbackCalibrationError::StaleToken)
        );
        assert_eq!(manager.state(), FeedbackCalibrationState::Collecting);
        manager.cancel(token).unwrap();
        assert_eq!(manager.state(), FeedbackCalibrationState::Idle);
    }

    #[test]
    fn router_config_rejects_unsupported_or_ambiguous_fallback() {
        let mut config = router_config(FeedbackMode::Sensorless, FeedbackMode::Sensorless);
        config.fallback_enabled = true;
        assert_eq!(config.validate(), Err(FeedbackError::InvalidConfig));
        config.primary_mode = FeedbackMode::Fused as u32;
        config.fallback_enabled = false;
        config.backup_mode = FeedbackMode::Fused as u32;
        assert_eq!(config.validate(), Err(FeedbackError::InvalidConfig));
    }
}
