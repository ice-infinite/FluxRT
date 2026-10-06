//! Versioned management-plane ABI for feedback routing and calibration.
//!
//! This module never runs from the ADC ISR. C owns one aligned storage block
//! and serializes access from a management task. Hardware adapters only submit
//! normalized SI samples; routing, hysteresis, calibration evidence and config
//! staging remain in Rust.

use core::mem::{align_of, size_of};
use core::ptr;

use foc_control::{
    AxisState, ConfigApplyGuard, FeedbackCalibrationError, FeedbackCalibrationManager,
    FeedbackCalibrationPolicy, FeedbackCalibrationState, FeedbackCalibrationUpdate, FeedbackError,
    FeedbackMode, FeedbackRouteState, FeedbackRouter, FeedbackRouterConfig, FeedbackSourceSample,
    ProductFeedbackSnapshot,
};

use crate::{
    foc_rust_config_get_pending, foc_rust_config_set_axis, foc_rust_config_set_calibration,
    FocConfigApplyGuardAbi, FocConfigBundleAbi, FocConfigContextStorage, FocConfigStatus,
    FOC_CONFIG_ABI_VERSION,
};

pub const FOC_FEEDBACK_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_FEEDBACK_CONTEXT_CAPACITY: usize = 512;

const FEEDBACK_CONTEXT_MAGIC: u32 = 0x4646_424B; // "FFBK"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocFeedbackStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    InvalidConfig = 3,
    InvalidSample = 4,
    UnsupportedMode = 5,
    StaleSequence = 6,
    StaleCycle = 7,
    UnsafeState = 8,
    Busy = 9,
    NoSession = 10,
    StaleToken = 11,
    InvalidState = 12,
    InvalidEvidence = 13,
    IncompleteEvidence = 14,
    ConfigTransactionFailed = 15,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocFeedbackRouterConfigAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub axis_id: u32,
    pub primary_mode: u32,
    pub backup_mode: u32,
    pub fallback_enabled: u32,
    pub maximum_age_us: u32,
    pub acquire_good_samples: u32,
    pub loss_bad_samples: u32,
    pub recovery_good_samples: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocFeedbackSourceSampleAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub axis_id: u32,
    pub mode: u32,
    pub sequence: u32,
    pub sampled_at_ms: u32,
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocFeedbackCalibrationPolicyAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub minimum_direction_samples: u32,
    pub minimum_index_samples: u32,
    pub minimum_offset_samples: u32,
    pub minimum_hall_samples: u32,
    pub encoder_counts_per_revolution: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocFeedbackRouteDecisionAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub route_state: u32,
    pub reserved: u32,
    pub snapshot: ProductFeedbackSnapshot,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocFeedbackCalibrationUpdateAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub feedback_mode: u32,
    pub axis_direction: i32,
    pub calibration_flags_to_set: u32,
    pub encoder_offset_rad: f32,
    pub encoder_counts_per_revolution: u32,
    pub hall_sequence_packed: u32,
    pub evidence_steps: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocFeedbackStatusAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub route_state: u32,
    pub calibration_state: u32,
    pub calibration_token: u32,
    pub completed_steps: u32,
    pub last_result: u32,
    pub last_detail: u32,
    pub reserved0: u32,
    pub reserved1: u32,
}

#[repr(C, align(8))]
pub struct FocFeedbackContextStorage {
    pub bytes: [u8; FOC_FEEDBACK_CONTEXT_CAPACITY],
}

struct FeedbackAbiContext {
    magic: u32,
    router: FeedbackRouter,
    calibration: FeedbackCalibrationManager,
    approved_update: Option<FeedbackCalibrationUpdate>,
    route_state: FeedbackRouteState,
    calibration_token: u32,
    last_result: FocFeedbackStatus,
    last_detail: u32,
}

impl FeedbackAbiContext {
    fn record(&mut self, result: FocFeedbackStatus, detail: u32) -> FocFeedbackStatus {
        self.last_result = result;
        self.last_detail = detail;
        result
    }

    fn record_feedback(&mut self, result: Result<(), FeedbackError>) -> FocFeedbackStatus {
        match result {
            Ok(()) => self.record(FocFeedbackStatus::Ok, 0),
            Err(error) => {
                let (status, detail) = map_feedback_error(error);
                self.record(status, detail)
            }
        }
    }

    fn record_calibration(
        &mut self,
        result: Result<(), FeedbackCalibrationError>,
    ) -> FocFeedbackStatus {
        match result {
            Ok(()) => self.record(FocFeedbackStatus::Ok, 0),
            Err(error) => {
                let (status, detail) = map_calibration_error(error);
                self.record(status, detail)
            }
        }
    }
}

const _: () = assert!(size_of::<FocFeedbackRouterConfigAbi>() == 40);
const _: () = assert!(size_of::<FocFeedbackSourceSampleAbi>() == 64);
const _: () = assert!(size_of::<FocFeedbackCalibrationPolicyAbi>() == 28);
const _: () = assert!(size_of::<FocFeedbackRouteDecisionAbi>() == 84);
const _: () = assert!(size_of::<FocFeedbackCalibrationUpdateAbi>() == 40);
const _: () = assert!(size_of::<FocFeedbackStatusAbi>() == 40);
const _: () = assert!(size_of::<FeedbackAbiContext>() <= FOC_FEEDBACK_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<FeedbackAbiContext>() <= align_of::<FocFeedbackContextStorage>());

impl FocFeedbackRouterConfigAbi {
    fn to_config(self) -> Result<FeedbackRouterConfig, FocFeedbackStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_FEEDBACK_ABI_VERSION
            || self.fallback_enabled > 1
            || self.acquire_good_samples > u16::MAX as u32
            || self.loss_bad_samples > u16::MAX as u32
            || self.recovery_good_samples > u16::MAX as u32
        {
            return Err(FocFeedbackStatus::InvalidArgument);
        }
        Ok(FeedbackRouterConfig {
            axis_id: self.axis_id,
            primary_mode: self.primary_mode,
            backup_mode: self.backup_mode,
            fallback_enabled: self.fallback_enabled != 0,
            maximum_age_us: self.maximum_age_us,
            acquire_good_samples: self.acquire_good_samples as u16,
            loss_bad_samples: self.loss_bad_samples as u16,
            recovery_good_samples: self.recovery_good_samples as u16,
        })
    }
}

impl FocFeedbackSourceSampleAbi {
    fn to_sample(self) -> Result<FeedbackSourceSample, FocFeedbackStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_FEEDBACK_ABI_VERSION
        {
            return Err(FocFeedbackStatus::InvalidArgument);
        }
        Ok(FeedbackSourceSample {
            axis_id: self.axis_id,
            mode: self.mode,
            sequence: self.sequence,
            sampled_at_ms: self.sampled_at_ms,
            sampled_at_us: self.sampled_at_us,
            valid_flags: self.valid_flags,
            quality_flags: self.quality_flags,
            direction: self.direction,
            pole_pair_revision: self.pole_pair_revision,
            mechanical_position_rad: self.mechanical_position_rad,
            multi_turn_position_rad: self.multi_turn_position_rad,
            mechanical_velocity_rad_s: self.mechanical_velocity_rad_s,
            electrical_angle_rad: self.electrical_angle_rad,
            electrical_velocity_rad_s: self.electrical_velocity_rad_s,
        })
    }
}

impl FocFeedbackCalibrationPolicyAbi {
    fn to_policy(self) -> Result<FeedbackCalibrationPolicy, FocFeedbackStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_FEEDBACK_ABI_VERSION
        {
            return Err(FocFeedbackStatus::InvalidArgument);
        }
        Ok(FeedbackCalibrationPolicy {
            minimum_direction_samples: self.minimum_direction_samples,
            minimum_index_samples: self.minimum_index_samples,
            minimum_offset_samples: self.minimum_offset_samples,
            minimum_hall_samples: self.minimum_hall_samples,
            encoder_counts_per_revolution: self.encoder_counts_per_revolution,
        })
    }
}

impl FocFeedbackRouteDecisionAbi {
    fn from_decision(value: foc_control::FeedbackRouteDecision) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_FEEDBACK_ABI_VERSION,
            route_state: value.state as u32,
            reserved: 0,
            snapshot: value.snapshot,
        }
    }
}

impl FocFeedbackCalibrationUpdateAbi {
    fn from_update(value: FeedbackCalibrationUpdate) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_FEEDBACK_ABI_VERSION,
            feedback_mode: value.feedback_mode,
            axis_direction: value.axis_direction,
            calibration_flags_to_set: value.calibration_flags_to_set,
            encoder_offset_rad: value.encoder_offset_rad,
            encoder_counts_per_revolution: value.encoder_counts_per_revolution,
            hall_sequence_packed: value.hall_sequence_packed,
            evidence_steps: value.evidence_steps,
            reserved: 0,
        }
    }
}

unsafe fn context_mut<'a>(
    storage: *mut FocFeedbackContextStorage,
) -> Result<&'a mut FeedbackAbiContext, FocFeedbackStatus> {
    if storage.is_null() || !address_is_aligned::<FocFeedbackContextStorage>(storage as usize) {
        return Err(FocFeedbackStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<FeedbackAbiContext>() };
    if context.magic != FEEDBACK_CONTEXT_MAGIC {
        return Err(FocFeedbackStatus::NotInitialized);
    }
    Ok(context)
}

unsafe fn read_copy<T: Copy>(value: *const T) -> Result<T, FocFeedbackStatus> {
    if value.is_null() || !address_is_aligned::<T>(value as usize) {
        Err(FocFeedbackStatus::InvalidArgument)
    } else {
        Ok(unsafe { ptr::read(value) })
    }
}

unsafe fn write_copy<T: Copy>(output: *mut T, value: T) -> Result<(), FocFeedbackStatus> {
    if output.is_null() || !address_is_aligned::<T>(output as usize) {
        Err(FocFeedbackStatus::InvalidArgument)
    } else {
        unsafe { ptr::write(output, value) };
        Ok(())
    }
}

fn address_is_aligned<T>(address: usize) -> bool {
    address & (align_of::<T>() - 1) == 0
}

fn map_feedback_error(error: FeedbackError) -> (FocFeedbackStatus, u32) {
    match error {
        FeedbackError::InvalidConfig => (FocFeedbackStatus::InvalidConfig, 1),
        FeedbackError::InvalidHeader => (FocFeedbackStatus::InvalidSample, 2),
        FeedbackError::UnsupportedMode => (FocFeedbackStatus::UnsupportedMode, 3),
        FeedbackError::UnknownFlags => (FocFeedbackStatus::InvalidSample, 4),
        FeedbackError::InvalidDirection => (FocFeedbackStatus::InvalidSample, 5),
        FeedbackError::NonFinite => (FocFeedbackStatus::InvalidSample, 6),
        FeedbackError::NonCanonical => (FocFeedbackStatus::InvalidSample, 7),
        FeedbackError::AngleOutOfRange => (FocFeedbackStatus::InvalidSample, 8),
        FeedbackError::StaleSequence => (FocFeedbackStatus::StaleSequence, 9),
        FeedbackError::StaleCycle => (FocFeedbackStatus::StaleCycle, 10),
    }
}

fn map_calibration_error(error: FeedbackCalibrationError) -> (FocFeedbackStatus, u32) {
    match error {
        FeedbackCalibrationError::InvalidPolicy => (FocFeedbackStatus::InvalidConfig, 101),
        FeedbackCalibrationError::UnsupportedMode => (FocFeedbackStatus::UnsupportedMode, 102),
        FeedbackCalibrationError::UnsafeState => (FocFeedbackStatus::UnsafeState, 103),
        FeedbackCalibrationError::Busy => (FocFeedbackStatus::Busy, 104),
        FeedbackCalibrationError::NoSession => (FocFeedbackStatus::NoSession, 105),
        FeedbackCalibrationError::StaleToken => (FocFeedbackStatus::StaleToken, 106),
        FeedbackCalibrationError::InvalidState => (FocFeedbackStatus::InvalidState, 107),
        FeedbackCalibrationError::InvalidEvidence => (FocFeedbackStatus::InvalidEvidence, 108),
        FeedbackCalibrationError::IncompleteEvidence => {
            (FocFeedbackStatus::IncompleteEvidence, 109)
        }
    }
}

fn guard_from_abi(value: FocConfigApplyGuardAbi) -> Result<ConfigApplyGuard, FocFeedbackStatus> {
    if value.struct_size != size_of::<FocConfigApplyGuardAbi>() as u32
        || value.abi_version != FOC_CONFIG_ABI_VERSION
        || value.drive_active > 1
    {
        return Err(FocFeedbackStatus::InvalidArgument);
    }
    let axis_state =
        AxisState::try_from(value.axis_state).map_err(|_| FocFeedbackStatus::InvalidArgument)?;
    Ok(ConfigApplyGuard {
        axis_state,
        drive_active: value.drive_active != 0,
        active_fault_flags: value.active_fault_flags,
    })
}

fn mode_from_raw(raw: u32) -> Result<FeedbackMode, FocFeedbackStatus> {
    FeedbackMode::try_from(raw).map_err(|_| FocFeedbackStatus::UnsupportedMode)
}

fn status_snapshot(context: &FeedbackAbiContext) -> FocFeedbackStatusAbi {
    FocFeedbackStatusAbi {
        struct_size: size_of::<FocFeedbackStatusAbi>() as u32,
        abi_version: FOC_FEEDBACK_ABI_VERSION,
        route_state: context.route_state as u32,
        calibration_state: context.calibration.state() as u32,
        calibration_token: context.calibration_token,
        completed_steps: context.calibration.completed_steps(),
        last_result: context.last_result as u32,
        last_detail: context.last_detail,
        reserved0: 0,
        reserved1: 0,
    }
}

#[no_mangle]
pub extern "C" fn foc_rust_feedback_abi_version() -> u32 {
    FOC_FEEDBACK_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_feedback_context_required_size() -> u32 {
    size_of::<FeedbackAbiContext>() as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_feedback_context_required_align() -> u32 {
    align_of::<FeedbackAbiContext>() as u32
}

#[no_mangle]
/// Initialize a caller-owned management context.
///
/// # Safety
/// All pointers must be aligned, readable/writable for their declared types,
/// and `storage` must remain exclusively owned by one management task.
pub unsafe extern "C" fn foc_rust_feedback_init(
    storage: *mut FocFeedbackContextStorage,
    router_config: *const FocFeedbackRouterConfigAbi,
    calibration_policy: *const FocFeedbackCalibrationPolicyAbi,
) -> FocFeedbackStatus {
    if storage.is_null() || !address_is_aligned::<FocFeedbackContextStorage>(storage as usize) {
        return FocFeedbackStatus::InvalidArgument;
    }
    let router_config =
        match unsafe { read_copy(router_config) }.and_then(|value| value.to_config()) {
            Ok(value) => value,
            Err(status) => return status,
        };
    let calibration_policy =
        match unsafe { read_copy(calibration_policy) }.and_then(|value| value.to_policy()) {
            Ok(value) => value,
            Err(status) => return status,
        };
    let router = match FeedbackRouter::new(router_config) {
        Ok(value) => value,
        Err(error) => return map_feedback_error(error).0,
    };
    let calibration = match FeedbackCalibrationManager::new(calibration_policy) {
        Ok(value) => value,
        Err(error) => return map_calibration_error(error).0,
    };
    unsafe {
        ptr::write(
            storage.cast::<FeedbackAbiContext>(),
            FeedbackAbiContext {
                magic: FEEDBACK_CONTEXT_MAGIC,
                router,
                calibration,
                approved_update: None,
                route_state: FeedbackRouteState::Acquiring,
                calibration_token: 0,
                last_result: FocFeedbackStatus::Ok,
                last_detail: 0,
            },
        )
    };
    FocFeedbackStatus::Ok
}

#[no_mangle]
/// Copy management status.
///
/// # Safety
/// `storage` must be initialized and `output` aligned and writable.
pub unsafe extern "C" fn foc_rust_feedback_get_status(
    storage: *mut FocFeedbackContextStorage,
    output: *mut FocFeedbackStatusAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    match unsafe { write_copy(output, status_snapshot(context)) } {
        Ok(()) => FocFeedbackStatus::Ok,
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Submit one normalized source sample.
///
/// # Safety
/// Input pointers must be aligned and readable.
pub unsafe extern "C" fn foc_rust_feedback_ingest(
    storage: *mut FocFeedbackContextStorage,
    sample: *const FocFeedbackSourceSampleAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let sample = match unsafe { read_copy(sample) }.and_then(|value| value.to_sample()) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let result = context.router.ingest(sample);
    context.record_feedback(result)
}

#[no_mangle]
/// Evaluate routing once for a new management/control selection cycle.
///
/// # Safety
/// `storage` must be initialized and `output` aligned and writable.
pub unsafe extern "C" fn foc_rust_feedback_route(
    storage: *mut FocFeedbackContextStorage,
    cycle_sequence: u32,
    now_us: u32,
    output: *mut FocFeedbackRouteDecisionAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if output.is_null() || !address_is_aligned::<FocFeedbackRouteDecisionAbi>(output as usize) {
        return context.record(FocFeedbackStatus::InvalidArgument, 0);
    }
    let decision = match context.router.route(cycle_sequence, now_us) {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_feedback_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(output, FocFeedbackRouteDecisionAbi::from_decision(decision)) } {
        Ok(()) => {
            context.route_state = decision.state;
            context.record(FocFeedbackStatus::Ok, 0)
        }
        Err(status) => context.record(status, 0),
    }
}

unsafe fn read_guard(
    context: &mut FeedbackAbiContext,
    guard: *const FocConfigApplyGuardAbi,
) -> Result<ConfigApplyGuard, FocFeedbackStatus> {
    match unsafe { read_copy(guard) }.and_then(guard_from_abi) {
        Ok(value) => Ok(value),
        Err(status) => Err(context.record(status, 0)),
    }
}

#[no_mangle]
/// Begin an ABZ or Hall calibration evidence session.
///
/// # Safety
/// Pointer requirements follow [`foc_rust_feedback_init`].
pub unsafe extern "C" fn foc_rust_feedback_calibration_begin(
    storage: *mut FocFeedbackContextStorage,
    feedback_mode: u32,
    guard: *const FocConfigApplyGuardAbi,
    token_out: *mut u32,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let mode = match mode_from_raw(feedback_mode) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match context.calibration.begin(mode, guard) {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_calibration_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(token_out, token) } {
        Ok(()) => {
            context.approved_update = None;
            context.calibration_token = token;
            context.record(FocFeedbackStatus::Ok, 0)
        }
        Err(status) => {
            let _ = context.calibration.cancel(token);
            context.calibration_token = 0;
            context.record(status, 0)
        }
    }
}

macro_rules! calibration_record {
    ($name:ident, $method:ident, $value_type:ty) => {
        #[no_mangle]
        /// Record one piece of externally measured calibration evidence.
        ///
        /// # Safety
        /// `storage` must hold an initialized, exclusively owned context.
        pub unsafe extern "C" fn $name(
            storage: *mut FocFeedbackContextStorage,
            token: u32,
            value: $value_type,
            sample_count: u32,
        ) -> FocFeedbackStatus {
            let context = match unsafe { context_mut(storage) } {
                Ok(value) => value,
                Err(status) => return status,
            };
            let result = context.calibration.$method(token, value, sample_count);
            context.record_calibration(result)
        }
    };
}

calibration_record!(
    foc_rust_feedback_calibration_record_direction,
    record_direction,
    i32
);
calibration_record!(
    foc_rust_feedback_calibration_record_encoder_offset,
    record_encoder_offset,
    f32
);
calibration_record!(
    foc_rust_feedback_calibration_record_hall_sequence,
    record_hall_sequence,
    u32
);

#[no_mangle]
/// Record a stable encoder Index observation.
///
/// # Safety
/// `storage` must hold an initialized, exclusively owned context.
pub unsafe extern "C" fn foc_rust_feedback_calibration_record_encoder_index(
    storage: *mut FocFeedbackContextStorage,
    token: u32,
    sample_count: u32,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context
        .calibration
        .record_encoder_index(token, sample_count);
    context.record_calibration(result)
}

#[no_mangle]
/// Approve complete evidence and retain a retryable config update.
///
/// # Safety
/// Input/output pointers must be valid for their declared types.
pub unsafe extern "C" fn foc_rust_feedback_calibration_approve(
    storage: *mut FocFeedbackContextStorage,
    token: u32,
    guard: *const FocConfigApplyGuardAbi,
    output: *mut FocFeedbackCalibrationUpdateAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if output.is_null() || !address_is_aligned::<FocFeedbackCalibrationUpdateAbi>(output as usize) {
        return context.record(FocFeedbackStatus::InvalidArgument, 0);
    }
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let update = match context.calibration.approve(token, guard) {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_calibration_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(output, FocFeedbackCalibrationUpdateAbi::from_update(update)) } {
        Ok(()) => {
            context.approved_update = Some(update);
            context.record(FocFeedbackStatus::Ok, 0)
        }
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Stage the approved update into an existing configuration transaction.
///
/// This only replaces the pending Axis and Calibration groups. The caller must
/// still validate, apply, commit/read back, then call `finish_applied`.
///
/// # Safety
/// Both opaque contexts must be initialized and exclusively owned.
pub unsafe extern "C" fn foc_rust_feedback_stage_config_update(
    storage: *mut FocFeedbackContextStorage,
    calibration_token: u32,
    config_storage: *mut FocConfigContextStorage,
    config_token: u32,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if calibration_token == 0
        || calibration_token != context.calibration_token
        || context.calibration.state() != FeedbackCalibrationState::Approved
    {
        return context.record(FocFeedbackStatus::StaleToken, 0);
    }
    let update = match context.approved_update {
        Some(value) => value,
        None => return context.record(FocFeedbackStatus::InvalidState, 0),
    };
    let mut bundle = FocConfigBundleAbi::default();
    let mut config_status =
        unsafe { foc_rust_config_get_pending(config_storage, config_token, &mut bundle) };
    if config_status != FocConfigStatus::Ok {
        return context.record(
            FocFeedbackStatus::ConfigTransactionFailed,
            config_status as u32,
        );
    }
    bundle.axis.feedback_mode = update.feedback_mode;
    bundle.axis.direction = update.axis_direction;
    bundle.calibration.valid_flags |= update.calibration_flags_to_set;
    match FeedbackMode::try_from(update.feedback_mode) {
        Ok(FeedbackMode::IncrementalEncoder | FeedbackMode::AbsoluteEncoder) => {
            bundle.calibration.encoder_offset_rad = update.encoder_offset_rad;
            bundle.calibration.encoder_counts_per_revolution = update.encoder_counts_per_revolution;
        }
        Ok(FeedbackMode::Hall) => {
            bundle.calibration.hall_sequence_packed = update.hall_sequence_packed;
        }
        _ => return context.record(FocFeedbackStatus::UnsupportedMode, 0),
    }
    config_status = unsafe { foc_rust_config_set_axis(config_storage, config_token, &bundle.axis) };
    if config_status != FocConfigStatus::Ok {
        return context.record(
            FocFeedbackStatus::ConfigTransactionFailed,
            config_status as u32,
        );
    }
    config_status = unsafe {
        foc_rust_config_set_calibration(config_storage, config_token, &bundle.calibration)
    };
    if config_status != FocConfigStatus::Ok {
        return context.record(
            FocFeedbackStatus::ConfigTransactionFailed,
            config_status as u32,
        );
    }
    context.record(FocFeedbackStatus::Ok, 0)
}

#[no_mangle]
/// Cancel any non-idle calibration session.
///
/// # Safety
/// `storage` must hold an initialized, exclusively owned context.
pub unsafe extern "C" fn foc_rust_feedback_calibration_cancel(
    storage: *mut FocFeedbackContextStorage,
    token: u32,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context.calibration.cancel(token);
    if result.is_ok() {
        context.approved_update = None;
        context.calibration_token = 0;
    }
    context.record_calibration(result)
}

#[no_mangle]
/// Reset a failed calibration only while the Axis remains safe.
///
/// # Safety
/// Input pointers must be valid for their declared types.
pub unsafe extern "C" fn foc_rust_feedback_calibration_reset_failed(
    storage: *mut FocFeedbackContextStorage,
    guard: *const FocConfigApplyGuardAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context.calibration.reset_failed(guard);
    if result.is_ok() {
        context.approved_update = None;
        context.calibration_token = 0;
    }
    context.record_calibration(result)
}

#[no_mangle]
/// Close an approved calibration after external config apply/commit succeeds.
///
/// # Safety
/// Input pointers must be valid for their declared types.
pub unsafe extern "C" fn foc_rust_feedback_calibration_finish_applied(
    storage: *mut FocFeedbackContextStorage,
    token: u32,
    guard: *const FocConfigApplyGuardAbi,
) -> FocFeedbackStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    if token == 0 || token != context.calibration_token {
        return context.record(FocFeedbackStatus::StaleToken, 0);
    }
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context.calibration.finish_applied(guard);
    if result.is_ok() {
        context.approved_update = None;
        context.calibration_token = 0;
    }
    context.record_calibration(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        foc_rust_config_begin, foc_rust_config_confirm_apply, foc_rust_config_get_pending,
        foc_rust_config_init, foc_rust_config_prepare_apply, foc_rust_config_validate,
        FocConfigAppAbi, FocConfigApplyPlanAbi, FocConfigAxisAbi, FocConfigBoardAbi,
        FocConfigCalibrationAbi, FocConfigCommandSourceAbi, FocConfigInverterAbi,
        FocConfigMotorAbi, FOC_CONFIG_CONTEXT_CAPACITY,
    };
    use foc_control::{
        ExternalIoConfig, BOARD_CAP_ENCODER, BOARD_CAP_HALL, BOARD_CAP_THREE_SHUNT_CURRENT,
        CALIBRATION_ENCODER_VALID, COMMAND_SOURCE_PERMISSION_KNOWN_MASK, CONFIG_GROUP_VERSION,
        PRODUCT_FEEDBACK_QUALITY_CALIBRATED, PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
        PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND, PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE,
        PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY, PRODUCT_FEEDBACK_VALID_KNOWN_MASK,
        PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY,
    };

    const BOARD_ID: u32 = 0x4311_6001;
    const MOTOR_ID: u32 = 0x2804_1007;

    fn router_config() -> FocFeedbackRouterConfigAbi {
        FocFeedbackRouterConfigAbi {
            struct_size: size_of::<FocFeedbackRouterConfigAbi>() as u32,
            abi_version: FOC_FEEDBACK_ABI_VERSION,
            axis_id: 0,
            primary_mode: FeedbackMode::IncrementalEncoder as u32,
            backup_mode: FeedbackMode::Sensorless as u32,
            fallback_enabled: 1,
            maximum_age_us: 500,
            acquire_good_samples: 2,
            loss_bad_samples: 2,
            recovery_good_samples: 3,
        }
    }

    fn policy() -> FocFeedbackCalibrationPolicyAbi {
        FocFeedbackCalibrationPolicyAbi {
            struct_size: size_of::<FocFeedbackCalibrationPolicyAbi>() as u32,
            abi_version: FOC_FEEDBACK_ABI_VERSION,
            minimum_direction_samples: 1,
            minimum_index_samples: 1,
            minimum_offset_samples: 1,
            minimum_hall_samples: 1,
            encoder_counts_per_revolution: 4096,
        }
    }

    fn safe_guard() -> FocConfigApplyGuardAbi {
        FocConfigApplyGuardAbi {
            struct_size: size_of::<FocConfigApplyGuardAbi>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            axis_state: AxisState::Disabled as u32,
            drive_active: 0,
            active_fault_flags: 0,
        }
    }

    fn init_feedback() -> FocFeedbackContextStorage {
        let mut storage = FocFeedbackContextStorage {
            bytes: [0; FOC_FEEDBACK_CONTEXT_CAPACITY],
        };
        assert_eq!(
            unsafe { foc_rust_feedback_init(&mut storage, &router_config(), &policy()) },
            FocFeedbackStatus::Ok
        );
        storage
    }

    fn sample(mode: FeedbackMode, sequence: u32, timestamp: u32) -> FocFeedbackSourceSampleAbi {
        let (valid_flags, quality_flags) = match mode {
            FeedbackMode::Sensorless => (
                PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
                    | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
                    | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
                PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
            ),
            FeedbackMode::IncrementalEncoder => (
                PRODUCT_FEEDBACK_VALID_KNOWN_MASK,
                PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID
                    | PRODUCT_FEEDBACK_QUALITY_CALIBRATED
                    | PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND,
            ),
            _ => unreachable!(),
        };
        FocFeedbackSourceSampleAbi {
            struct_size: size_of::<FocFeedbackSourceSampleAbi>() as u32,
            abi_version: FOC_FEEDBACK_ABI_VERSION,
            axis_id: 0,
            mode: mode as u32,
            sequence,
            sampled_at_ms: timestamp / 1000,
            sampled_at_us: timestamp,
            valid_flags,
            quality_flags,
            direction: 1,
            pole_pair_revision: 7,
            mechanical_position_rad: if mode == FeedbackMode::IncrementalEncoder {
                0.25
            } else {
                0.0
            },
            multi_turn_position_rad: if mode == FeedbackMode::IncrementalEncoder {
                12.5
            } else {
                0.0
            },
            mechanical_velocity_rad_s: 10.0,
            electrical_angle_rad: 1.0,
            electrical_velocity_rad_s: 70.0,
        }
    }

    fn config_bundle() -> FocConfigBundleAbi {
        FocConfigBundleAbi {
            struct_size: size_of::<FocConfigBundleAbi>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            bundle_revision: 1,
            generated_by_version: 1,
            board: FocConfigBoardAbi {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT
                    | BOARD_CAP_ENCODER
                    | BOARD_CAP_HALL,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.012,
            },
            motor: FocConfigMotorAbi {
                version: CONFIG_GROUP_VERSION,
                motor_id: MOTOR_ID,
                pole_pairs: 7,
                phase_resistance_ohm: 5.0,
                d_inductance_h: 0.001,
                q_inductance_h: 0.001,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: FocConfigInverterAbi {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 1,
                maximum_phase_current_a: 2.0,
                minimum_bus_voltage_v: 7.0,
                maximum_bus_voltage_v: 18.0,
                dead_time_s: 550.0e-9,
                transistor_drop_v: 0.35,
                brake_resistance_ohm: 0.0,
                maximum_duty: 0.95,
            },
            axis: FocConfigAxisAbi {
                version: CONFIG_GROUP_VERSION,
                axis_id: 0,
                feedback_mode: FeedbackMode::Sensorless as u32,
                direction: 1,
                current_kp: 3.0,
                current_ki: 100.0,
                velocity_kp: 0.02,
                velocity_ki: 0.2,
                position_kp: 1.0,
                soft_limit_min_rad: -100.0,
                soft_limit_max_rad: 100.0,
                motion_control_enabled: 0,
                torque_ramp_rate_nm_s: 0.0,
                velocity_ramp_rate_rad_s2: 0.0,
                position_filter_bandwidth_rad_s: 0.0,
                trajectory_acceleration_rad_s2: 0.0,
                trajectory_deceleration_rad_s2: 0.0,
            },
            app: FocConfigAppAbi {
                version: CONFIG_GROUP_VERSION,
                can_node_id: 1,
                uart_baud: 115_200,
                command_source_count: 1,
                command_sources: [
                    FocConfigCommandSourceAbi {
                        source_id: 1,
                        priority: 10,
                        permissions: COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    FocConfigCommandSourceAbi::default(),
                    FocConfigCommandSourceAbi::default(),
                    FocConfigCommandSourceAbi::default(),
                ],
            },
            external_io: ExternalIoConfig::default(),
            calibration: FocConfigCalibrationAbi {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                motor_id: MOTOR_ID,
                valid_flags: 0,
                current_offset_counts: [0.0; 3],
                phase_voltage_gain: [0.0; 3],
                phase_voltage_offset_v: [0.0; 3],
                encoder_offset_rad: 0.0,
                encoder_counts_per_revolution: 0,
                hall_sequence_packed: 0,
            },
        }
    }

    #[test]
    fn abi_layout_and_routing_are_stable() {
        assert!(foc_rust_feedback_context_required_size() <= FOC_FEEDBACK_CONTEXT_CAPACITY as u32);
        assert!(foc_rust_feedback_context_required_align() <= 8);
        let mut storage = init_feedback();
        for sequence in 1..=2 {
            for mode in [FeedbackMode::IncrementalEncoder, FeedbackMode::Sensorless] {
                assert_eq!(
                    unsafe {
                        foc_rust_feedback_ingest(
                            &mut storage,
                            &sample(mode, sequence, sequence * 100),
                        )
                    },
                    FocFeedbackStatus::Ok
                );
            }
            let mut decision = FocFeedbackRouteDecisionAbi::default();
            assert_eq!(
                unsafe {
                    foc_rust_feedback_route(&mut storage, sequence, sequence * 100, &mut decision)
                },
                FocFeedbackStatus::Ok
            );
            if sequence == 2 {
                assert_eq!(decision.route_state, FeedbackRouteState::Primary as u32);
                assert_eq!(
                    decision.snapshot.active_feedback_mode,
                    FeedbackMode::IncrementalEncoder as u32
                );
            }
        }
        assert_eq!(
            unsafe {
                foc_rust_feedback_ingest(&mut storage, &sample(FeedbackMode::Sensorless, 2, 300))
            },
            FocFeedbackStatus::StaleSequence
        );
    }

    #[test]
    fn encoder_calibration_stages_two_config_groups_without_applying_them() {
        let mut feedback_storage = init_feedback();
        let mut config_storage = FocConfigContextStorage {
            bytes: [0; FOC_CONFIG_CONTEXT_CAPACITY],
        };
        assert_eq!(
            unsafe {
                foc_rust_config_init(&mut config_storage, &config_bundle(), BOARD_ID, MOTOR_ID)
            },
            FocConfigStatus::Ok
        );
        let mut config_token = 0;
        assert_eq!(
            unsafe { foc_rust_config_begin(&mut config_storage, &mut config_token) },
            FocConfigStatus::Ok
        );
        let guard = safe_guard();
        let mut calibration_token = 0;
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_begin(
                    &mut feedback_storage,
                    FeedbackMode::IncrementalEncoder as u32,
                    &guard,
                    &mut calibration_token,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_record_direction(
                    &mut feedback_storage,
                    calibration_token,
                    -1,
                    1,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_record_encoder_index(
                    &mut feedback_storage,
                    calibration_token,
                    1,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_record_encoder_offset(
                    &mut feedback_storage,
                    calibration_token,
                    0.25,
                    1,
                )
            },
            FocFeedbackStatus::Ok
        );
        let mut update = FocFeedbackCalibrationUpdateAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_approve(
                    &mut feedback_storage,
                    calibration_token,
                    &guard,
                    &mut update,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_stage_config_update(
                    &mut feedback_storage,
                    calibration_token,
                    &mut config_storage,
                    config_token,
                )
            },
            FocFeedbackStatus::Ok
        );
        let mut pending = FocConfigBundleAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_get_pending(&mut config_storage, config_token, &mut pending) },
            FocConfigStatus::Ok
        );
        assert_eq!(
            pending.axis.feedback_mode,
            FeedbackMode::IncrementalEncoder as u32
        );
        assert_eq!(pending.axis.direction, -1);
        assert_eq!(
            pending.calibration.valid_flags & CALIBRATION_ENCODER_VALID,
            CALIBRATION_ENCODER_VALID
        );
        assert_eq!(pending.calibration.encoder_offset_rad, 0.25);
        assert_eq!(pending.calibration.encoder_counts_per_revolution, 4096);
        assert_eq!(
            unsafe { foc_rust_config_validate(&mut config_storage, config_token) },
            FocConfigStatus::Ok
        );
        let mut plan = FocConfigApplyPlanAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_config_prepare_apply(&mut config_storage, config_token, &guard, &mut plan)
            },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_config_confirm_apply(&mut config_storage, config_token, &guard, &plan)
            },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_finish_applied(
                    &mut feedback_storage,
                    calibration_token,
                    &guard,
                )
            },
            FocFeedbackStatus::Ok
        );
    }

    #[test]
    fn unsafe_and_stale_paths_fail_closed_and_remain_retryable() {
        let mut storage = init_feedback();
        assert_eq!(
            unsafe {
                foc_rust_feedback_ingest(&mut storage, &sample(FeedbackMode::Sensorless, 1, 100))
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_feedback_route(&mut storage, 1, 100, ptr::null_mut()) },
            FocFeedbackStatus::InvalidArgument
        );
        let mut decision = FocFeedbackRouteDecisionAbi::default();
        assert_eq!(
            unsafe { foc_rust_feedback_route(&mut storage, 1, 100, &mut decision) },
            FocFeedbackStatus::Ok
        );

        let mut unsafe_guard = safe_guard();
        unsafe_guard.drive_active = 1;
        let mut token = 0;
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_begin(
                    &mut storage,
                    FeedbackMode::Hall as u32,
                    &unsafe_guard,
                    &mut token,
                )
            },
            FocFeedbackStatus::UnsafeState
        );

        let guard = safe_guard();
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_begin(
                    &mut storage,
                    FeedbackMode::Hall as u32,
                    &guard,
                    &mut token,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_feedback_calibration_record_direction(&mut storage, token, 1, 1) },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_record_hall_sequence(
                    &mut storage,
                    token,
                    0x0054_3210,
                    1,
                )
            },
            FocFeedbackStatus::Ok
        );
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_approve(&mut storage, token, &guard, ptr::null_mut())
            },
            FocFeedbackStatus::InvalidArgument
        );
        let mut update = FocFeedbackCalibrationUpdateAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_feedback_calibration_approve(&mut storage, token, &guard, &mut update)
            },
            FocFeedbackStatus::Ok
        );

        let mut bad_config = router_config();
        bad_config.struct_size = 0;
        assert_eq!(
            unsafe { foc_rust_feedback_init(&mut storage, &bad_config, &policy()) },
            FocFeedbackStatus::InvalidArgument
        );
    }
}
