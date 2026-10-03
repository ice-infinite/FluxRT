//! Versioned C ABI for the device-side configuration transaction manager.
//!
//! This is a management-plane sub-contract.  It is intentionally versioned
//! independently from the realtime bridge ABI so adding configuration fields
//! cannot silently change the 12 kHz input/output contract.

use core::mem::{align_of, size_of};
use core::ptr;
use foc_control::{
    AppConfig, AxisConfig, AxisState, BoardConfig, CalibrationData, CommandSourceConfig,
    ConfigApplyGuard, ConfigApplyPlan, ConfigBundle, ConfigPatch, ConfigRecordError, ConfigSlot,
    ConfigTransactionError, ConfigTransactionManager, ConfigTransactionState,
    ConfigTransactionToken, ConfigValidationError, ConfigWriteError, ExternalIoConfig,
    InverterConfig, MotorConfig, PreparedConfigWrite, CONFIG_MAX_COMMAND_SOURCES,
    CONFIG_SLOT_COUNT, CONFIG_SLOT_SIZE,
};

pub const FOC_CONFIG_ABI_VERSION: u32 = 0x0004_0000;
pub const FOC_CONFIG_CONTEXT_CAPACITY: usize = 4096;

const CONFIG_CONTEXT_MAGIC: u32 = 0x4643_4647; // "FCFG"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocConfigStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    Busy = 3,
    NoTransaction = 4,
    StaleToken = 5,
    InvalidState = 6,
    EmptyTransaction = 7,
    UnsafeToApply = 8,
    ValidationFailed = 9,
    StoragePlanFailed = 10,
    StorageRecordFailed = 11,
    StorageVerificationFailed = 12,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigBoardAbi {
    pub version: u32,
    pub board_id: u32,
    pub pwm_frequency_hz: u32,
    pub control_frequency_hz: u32,
    pub capability_flags: u32,
    pub adc_reference_v: f32,
    pub current_gain_a_per_count: f32,
    pub bus_voltage_v_per_count: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigMotorAbi {
    pub version: u32,
    pub motor_id: u32,
    pub pole_pairs: u32,
    pub phase_resistance_ohm: f32,
    pub d_inductance_h: f32,
    pub q_inductance_h: f32,
    pub flux_linkage_v_s: f32,
    pub continuous_current_a: f32,
    pub maximum_speed_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigInverterAbi {
    pub version: u32,
    pub inverter_id: u32,
    pub maximum_phase_current_a: f32,
    pub minimum_bus_voltage_v: f32,
    pub maximum_bus_voltage_v: f32,
    pub dead_time_s: f32,
    pub transistor_drop_v: f32,
    pub brake_resistance_ohm: f32,
    pub maximum_duty: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigAxisAbi {
    pub version: u32,
    pub axis_id: u32,
    pub feedback_mode: u32,
    pub direction: i32,
    pub current_kp: f32,
    pub current_ki: f32,
    pub velocity_kp: f32,
    pub velocity_ki: f32,
    pub position_kp: f32,
    pub soft_limit_min_rad: f32,
    pub soft_limit_max_rad: f32,
    pub motion_control_enabled: u32,
    pub torque_ramp_rate_nm_s: f32,
    pub velocity_ramp_rate_rad_s2: f32,
    pub position_filter_bandwidth_rad_s: f32,
    pub trajectory_acceleration_rad_s2: f32,
    pub trajectory_deceleration_rad_s2: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigCommandSourceAbi {
    pub source_id: u32,
    pub priority: u32,
    pub permissions: u32,
    pub lease_ms: u32,
    pub command_timeout_ms: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigAppAbi {
    pub version: u32,
    pub can_node_id: u32,
    pub uart_baud: u32,
    pub command_source_count: u32,
    pub command_sources: [FocConfigCommandSourceAbi; CONFIG_MAX_COMMAND_SOURCES],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigCalibrationAbi {
    pub version: u32,
    pub board_id: u32,
    pub motor_id: u32,
    pub valid_flags: u32,
    pub current_offset_counts: [f32; 3],
    pub phase_voltage_gain: [f32; 3],
    pub phase_voltage_offset_v: [f32; 3],
    pub encoder_offset_rad: f32,
    pub encoder_counts_per_revolution: u32,
    pub hall_sequence_packed: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocConfigBundleAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub bundle_revision: u32,
    pub generated_by_version: u32,
    pub board: FocConfigBoardAbi,
    pub motor: FocConfigMotorAbi,
    pub inverter: FocConfigInverterAbi,
    pub axis: FocConfigAxisAbi,
    pub app: FocConfigAppAbi,
    pub external_io: ExternalIoConfig,
    pub calibration: FocConfigCalibrationAbi,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigApplyGuardAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub axis_state: u32,
    pub drive_active: u32,
    pub active_fault_flags: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigApplyPlanAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub token: u32,
    pub from_revision: u32,
    pub to_revision: u32,
    pub changed_groups: u32,
    pub apply_class: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigTransactionStatusAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub state: u32,
    pub token: u32,
    pub active_revision: u32,
    pub pending_revision: u32,
    pub changed_groups: u32,
    pub last_result: u32,
    pub last_detail: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FocConfigSlotsAbi {
    pub bytes: [[u8; CONFIG_SLOT_SIZE]; CONFIG_SLOT_COUNT],
}

impl Default for FocConfigSlotsAbi {
    fn default() -> Self {
        Self {
            bytes: [[0xFF; CONFIG_SLOT_SIZE]; CONFIG_SLOT_COUNT],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FocConfigSlotAbi {
    pub bytes: [u8; CONFIG_SLOT_SIZE],
}

impl Default for FocConfigSlotAbi {
    fn default() -> Self {
        Self {
            bytes: [0xFF; CONFIG_SLOT_SIZE],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigCommitPlanAbi {
    pub struct_size: u32,
    pub abi_version: u32,
    pub token: u32,
    pub target_slot: u32,
    pub sequence: u32,
    pub write_step_count: u32,
    pub reserved0: u32,
    pub reserved1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct FocConfigWriteByteAbi {
    pub offset: u32,
    pub value: u32,
}

#[repr(C, align(8))]
pub struct FocConfigContextStorage {
    pub bytes: [u8; FOC_CONFIG_CONTEXT_CAPACITY],
}

struct ConfigAbiContext {
    magic: u32,
    manager: ConfigTransactionManager,
    prepared_write: Option<PreparedConfigWrite>,
    last_result: FocConfigStatus,
    last_detail: u32,
}

impl ConfigAbiContext {
    fn new(manager: ConfigTransactionManager) -> Self {
        Self {
            magic: CONFIG_CONTEXT_MAGIC,
            manager,
            prepared_write: None,
            last_result: FocConfigStatus::Ok,
            last_detail: 0,
        }
    }

    fn record(&mut self, result: FocConfigStatus, detail: u32) -> FocConfigStatus {
        self.last_result = result;
        self.last_detail = detail;
        result
    }

    fn record_transaction_result(
        &mut self,
        result: Result<(), ConfigTransactionError>,
    ) -> FocConfigStatus {
        match result {
            Ok(()) => self.record(FocConfigStatus::Ok, 0),
            Err(error) => {
                let (status, detail) = map_transaction_error(error);
                self.record(status, detail)
            }
        }
    }
}

const _: () = assert!(size_of::<FocConfigBoardAbi>() == 32);
const _: () = assert!(size_of::<FocConfigMotorAbi>() == 36);
const _: () = assert!(size_of::<FocConfigInverterAbi>() == 36);
const _: () = assert!(size_of::<FocConfigAxisAbi>() == 68);
const _: () = assert!(size_of::<FocConfigCommandSourceAbi>() == 20);
const _: () = assert!(size_of::<FocConfigAppAbi>() == 96);
const _: () = assert!(size_of::<FocConfigCalibrationAbi>() == 64);
const _: () = assert!(size_of::<FocConfigBundleAbi>() == 732);
const _: () = assert!(size_of::<FocConfigApplyGuardAbi>() == 20);
const _: () = assert!(size_of::<FocConfigApplyPlanAbi>() == 32);
const _: () = assert!(size_of::<FocConfigTransactionStatusAbi>() == 36);
const _: () = assert!(size_of::<FocConfigCommitPlanAbi>() == 32);
const _: () = assert!(size_of::<FocConfigWriteByteAbi>() == 8);
const _: () = assert!(size_of::<ConfigAbiContext>() <= FOC_CONFIG_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<ConfigAbiContext>() <= align_of::<FocConfigContextStorage>());

impl FocConfigBundleAbi {
    pub(crate) fn from_bundle(config: ConfigBundle) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            bundle_revision: config.bundle_revision,
            generated_by_version: config.generated_by_version,
            board: FocConfigBoardAbi::from_config(config.board),
            motor: FocConfigMotorAbi::from_config(config.motor),
            inverter: FocConfigInverterAbi::from_config(config.inverter),
            axis: FocConfigAxisAbi::from_config(config.axis),
            app: FocConfigAppAbi::from_config(config.app),
            external_io: config.external_io,
            calibration: FocConfigCalibrationAbi::from_config(config.calibration),
        }
    }

    pub(crate) fn to_bundle(self) -> Result<ConfigBundle, FocConfigStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_CONFIG_ABI_VERSION
        {
            return Err(FocConfigStatus::InvalidArgument);
        }
        Ok(ConfigBundle {
            bundle_revision: self.bundle_revision,
            generated_by_version: self.generated_by_version,
            board: self.board.to_config(),
            motor: self.motor.to_config(),
            inverter: self.inverter.to_config(),
            axis: self.axis.to_config(),
            app: self.app.to_config(),
            external_io: self.external_io,
            calibration: self.calibration.to_config(),
        })
    }
}

macro_rules! group_conversion {
    ($abi:ident, $config:ident, {$($field:ident),+ $(,)?}) => {
        impl $abi {
            fn from_config(value: $config) -> Self {
                Self { $($field: value.$field),+ }
            }

            fn to_config(self) -> $config {
                $config { $($field: self.$field),+ }
            }
        }
    };
}

group_conversion!(FocConfigBoardAbi, BoardConfig, {
    version, board_id, pwm_frequency_hz, control_frequency_hz, capability_flags,
    adc_reference_v, current_gain_a_per_count, bus_voltage_v_per_count
});
group_conversion!(FocConfigMotorAbi, MotorConfig, {
    version, motor_id, pole_pairs, phase_resistance_ohm, d_inductance_h,
    q_inductance_h, flux_linkage_v_s, continuous_current_a, maximum_speed_rad_s
});
group_conversion!(FocConfigInverterAbi, InverterConfig, {
    version, inverter_id, maximum_phase_current_a, minimum_bus_voltage_v,
    maximum_bus_voltage_v, dead_time_s, transistor_drop_v, brake_resistance_ohm,
    maximum_duty
});
group_conversion!(FocConfigAxisAbi, AxisConfig, {
    version, axis_id, feedback_mode, direction, current_kp, current_ki,
    velocity_kp, velocity_ki, position_kp, soft_limit_min_rad, soft_limit_max_rad,
    motion_control_enabled, torque_ramp_rate_nm_s, velocity_ramp_rate_rad_s2,
    position_filter_bandwidth_rad_s, trajectory_acceleration_rad_s2,
    trajectory_deceleration_rad_s2
});
group_conversion!(FocConfigCalibrationAbi, CalibrationData, {
    version, board_id, motor_id, valid_flags, current_offset_counts,
    phase_voltage_gain, phase_voltage_offset_v, encoder_offset_rad,
    encoder_counts_per_revolution, hall_sequence_packed
});

impl FocConfigCommandSourceAbi {
    fn from_config(value: CommandSourceConfig) -> Self {
        Self {
            source_id: value.source_id,
            priority: value.priority,
            permissions: value.permissions,
            lease_ms: value.lease_ms,
            command_timeout_ms: value.command_timeout_ms,
        }
    }

    fn to_config(self) -> CommandSourceConfig {
        CommandSourceConfig {
            source_id: self.source_id,
            priority: self.priority,
            permissions: self.permissions,
            lease_ms: self.lease_ms,
            command_timeout_ms: self.command_timeout_ms,
        }
    }
}

impl FocConfigAppAbi {
    fn from_config(value: AppConfig) -> Self {
        Self {
            version: value.version,
            can_node_id: value.can_node_id,
            uart_baud: value.uart_baud,
            command_source_count: value.command_source_count,
            command_sources: value
                .command_sources
                .map(FocConfigCommandSourceAbi::from_config),
        }
    }

    fn to_config(self) -> AppConfig {
        AppConfig {
            version: self.version,
            can_node_id: self.can_node_id,
            uart_baud: self.uart_baud,
            command_source_count: self.command_source_count,
            command_sources: self
                .command_sources
                .map(FocConfigCommandSourceAbi::to_config),
        }
    }
}

impl FocConfigApplyGuardAbi {
    fn to_guard(self) -> Result<ConfigApplyGuard, FocConfigStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_CONFIG_ABI_VERSION
            || self.drive_active > 1
        {
            return Err(FocConfigStatus::InvalidArgument);
        }
        let axis_state =
            AxisState::try_from(self.axis_state).map_err(|_| FocConfigStatus::InvalidArgument)?;
        Ok(ConfigApplyGuard {
            axis_state,
            drive_active: self.drive_active != 0,
            active_fault_flags: self.active_fault_flags,
        })
    }
}

impl FocConfigApplyPlanAbi {
    fn from_plan(plan: ConfigApplyPlan) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            token: plan.token.raw(),
            from_revision: plan.from_revision,
            to_revision: plan.to_revision,
            changed_groups: plan.changed_groups,
            apply_class: plan.apply_class as u32,
            reserved: 0,
        }
    }

    fn to_plan(self) -> Result<ConfigApplyPlan, FocConfigStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_CONFIG_ABI_VERSION
            || self.reserved != 0
        {
            return Err(FocConfigStatus::InvalidArgument);
        }
        let token =
            ConfigTransactionToken::from_raw(self.token).ok_or(FocConfigStatus::InvalidArgument)?;
        let apply_class = match self.apply_class {
            0 => foc_control::ConfigApplyClass::ManagementOnly,
            1 => foc_control::ConfigApplyClass::AxisRestart,
            2 => foc_control::ConfigApplyClass::PlatformRestart,
            _ => return Err(FocConfigStatus::InvalidArgument),
        };
        Ok(ConfigApplyPlan {
            token,
            from_revision: self.from_revision,
            to_revision: self.to_revision,
            changed_groups: self.changed_groups,
            apply_class,
        })
    }
}

unsafe fn context_mut<'a>(
    storage: *mut FocConfigContextStorage,
) -> Result<&'a mut ConfigAbiContext, FocConfigStatus> {
    if storage.is_null() || !address_is_aligned::<FocConfigContextStorage>(storage as usize) {
        return Err(FocConfigStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<ConfigAbiContext>() };
    if context.magic != CONFIG_CONTEXT_MAGIC {
        return Err(FocConfigStatus::NotInitialized);
    }
    Ok(context)
}

unsafe fn read_copy<T: Copy>(value: *const T) -> Result<T, FocConfigStatus> {
    if value.is_null() || !address_is_aligned::<T>(value as usize) {
        Err(FocConfigStatus::InvalidArgument)
    } else {
        Ok(unsafe { ptr::read(value) })
    }
}

unsafe fn write_copy<T: Copy>(output: *mut T, value: T) -> Result<(), FocConfigStatus> {
    if output.is_null() || !address_is_aligned::<T>(output as usize) {
        Err(FocConfigStatus::InvalidArgument)
    } else {
        unsafe { ptr::write(output, value) };
        Ok(())
    }
}

fn address_is_aligned<T>(address: usize) -> bool {
    address & (align_of::<T>() - 1) == 0
}

fn token(raw: u32) -> Result<ConfigTransactionToken, FocConfigStatus> {
    ConfigTransactionToken::from_raw(raw).ok_or(FocConfigStatus::InvalidArgument)
}

fn slots_from_abi(value: FocConfigSlotsAbi) -> [ConfigSlot; CONFIG_SLOT_COUNT] {
    [
        ConfigSlot::from_bytes(value.bytes[0]),
        ConfigSlot::from_bytes(value.bytes[1]),
    ]
}

fn validation_detail(error: ConfigValidationError) -> u32 {
    match error {
        ConfigValidationError::GroupVersion => 1,
        ConfigValidationError::ZeroIdentity => 2,
        ConfigValidationError::Frequency => 3,
        ConfigValidationError::BoardCapabilities => 4,
        ConfigValidationError::BoardScaling => 5,
        ConfigValidationError::MotorParameters => 6,
        ConfigValidationError::InverterParameters => 7,
        ConfigValidationError::MotorExceedsInverter => 8,
        ConfigValidationError::Axis => 9,
        ConfigValidationError::App => 10,
        ConfigValidationError::CommandSourceCount => 11,
        ConfigValidationError::CommandSourceDuplicate => 12,
        ConfigValidationError::CommandSourcePriority => 13,
        ConfigValidationError::CommandSourcePermissions => 14,
        ConfigValidationError::CommandSourceTime => 15,
        ConfigValidationError::NonCanonicalUnusedSource => 16,
        ConfigValidationError::ExternalIo => 17,
        ConfigValidationError::CalibrationFlags => 18,
        ConfigValidationError::CalibrationIdentity => 19,
        ConfigValidationError::CalibrationValue => 20,
        ConfigValidationError::FeedbackCalibrationMissing => 21,
        ConfigValidationError::HardwareIdentityMismatch => 22,
        ConfigValidationError::HardwareCapabilityMissing => 23,
    }
}

fn write_error_detail(error: ConfigWriteError) -> u32 {
    match error {
        ConfigWriteError::InvalidConfig(value) => 100 + validation_detail(value),
        ConfigWriteError::AmbiguousSequence => 123,
        ConfigWriteError::RollbackUnavailable => 124,
    }
}

fn record_error_detail(error: ConfigRecordError) -> u32 {
    match error {
        ConfigRecordError::Erased => 201,
        ConfigRecordError::CommitMissing => 202,
        ConfigRecordError::Header => 203,
        ConfigRecordError::UnsupportedStorageVersion => 204,
        ConfigRecordError::UnsupportedSchemaVersion => 205,
        ConfigRecordError::PayloadLength => 206,
        ConfigRecordError::PayloadCrc => 207,
        ConfigRecordError::RecordCrc => 208,
        ConfigRecordError::InvalidApproval => 209,
        ConfigRecordError::IdentityMismatch => 210,
        ConfigRecordError::InvalidConfig(value) => 220 + validation_detail(value),
    }
}

fn map_transaction_error(error: ConfigTransactionError) -> (FocConfigStatus, u32) {
    match error {
        ConfigTransactionError::InvalidInitial(value)
        | ConfigTransactionError::Validation(value) => {
            (FocConfigStatus::ValidationFailed, validation_detail(value))
        }
        ConfigTransactionError::HardwareIdentityMismatch => (FocConfigStatus::ValidationFailed, 22),
        ConfigTransactionError::Busy => (FocConfigStatus::Busy, 0),
        ConfigTransactionError::NoTransaction => (FocConfigStatus::NoTransaction, 0),
        ConfigTransactionError::StaleToken => (FocConfigStatus::StaleToken, 0),
        ConfigTransactionError::InvalidState => (FocConfigStatus::InvalidState, 0),
        ConfigTransactionError::EmptyTransaction => (FocConfigStatus::EmptyTransaction, 0),
        ConfigTransactionError::UnsafeToApply => (FocConfigStatus::UnsafeToApply, 0),
        ConfigTransactionError::StoragePlan(value) => (
            FocConfigStatus::StoragePlanFailed,
            write_error_detail(value),
        ),
        ConfigTransactionError::StorageRecord(value) => (
            FocConfigStatus::StorageRecordFailed,
            record_error_detail(value),
        ),
        ConfigTransactionError::StorageVerification => {
            (FocConfigStatus::StorageVerificationFailed, 0)
        }
    }
}

fn status_snapshot(context: &ConfigAbiContext) -> FocConfigTransactionStatusAbi {
    let status = context.manager.status();
    FocConfigTransactionStatusAbi {
        struct_size: size_of::<FocConfigTransactionStatusAbi>() as u32,
        abi_version: FOC_CONFIG_ABI_VERSION,
        state: status.state as u32,
        token: status.token.map_or(0, ConfigTransactionToken::raw),
        active_revision: status.active_revision,
        pending_revision: status.pending_revision.unwrap_or(0),
        changed_groups: status.changed_groups,
        last_result: context.last_result as u32,
        last_detail: context.last_detail,
    }
}

#[no_mangle]
pub extern "C" fn foc_rust_config_abi_version() -> u32 {
    FOC_CONFIG_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_config_context_required_size() -> u32 {
    size_of::<ConfigAbiContext>() as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_config_context_required_align() -> u32 {
    align_of::<ConfigAbiContext>() as u32
}

#[no_mangle]
/// Initialize caller-owned configuration context storage.
///
/// # Safety
/// All pointer arguments must identify aligned, valid objects for the duration of this call.
pub unsafe extern "C" fn foc_rust_config_init(
    storage: *mut FocConfigContextStorage,
    active: *const FocConfigBundleAbi,
    expected_board_id: u32,
    expected_motor_id: u32,
) -> FocConfigStatus {
    if storage.is_null() || !address_is_aligned::<FocConfigContextStorage>(storage as usize) {
        return FocConfigStatus::InvalidArgument;
    }
    let active = match unsafe { read_copy(active) }.and_then(FocConfigBundleAbi::to_bundle) {
        Ok(value) => value,
        Err(status) => return status,
    };
    let manager = match ConfigTransactionManager::new(active, expected_board_id, expected_motor_id)
    {
        Ok(value) => value,
        Err(error) => return map_transaction_error(error).0,
    };
    unsafe {
        ptr::write(
            storage.cast::<ConfigAbiContext>(),
            ConfigAbiContext::new(manager),
        )
    };
    FocConfigStatus::Ok
}

#[no_mangle]
/// Copy the current transaction status into caller-owned storage.
///
/// # Safety
/// `storage` must hold an initialized context and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_config_get_status(
    storage: *mut FocConfigContextStorage,
    output: *mut FocConfigTransactionStatusAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    match unsafe { write_copy(output, status_snapshot(context)) } {
        Ok(()) => FocConfigStatus::Ok,
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Copy the active configuration into caller-owned storage.
///
/// # Safety
/// `storage` must hold an initialized context and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_config_get_active(
    storage: *mut FocConfigContextStorage,
    output: *mut FocConfigBundleAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    match unsafe {
        write_copy(
            output,
            FocConfigBundleAbi::from_bundle(*context.manager.active()),
        )
    } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Copy the pending configuration for the supplied token.
///
/// # Safety
/// `storage` must hold an initialized context and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_config_get_pending(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    output: *mut FocConfigBundleAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let pending = match context.manager.pending(token) {
        Ok(value) => *value,
        Err(error) => {
            let (status, detail) = map_transaction_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(output, FocConfigBundleAbi::from_bundle(pending)) } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Begin a configuration transaction and return its token.
///
/// # Safety
/// `storage` must hold an initialized context and `token_out` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_config_begin(
    storage: *mut FocConfigContextStorage,
    token_out: *mut u32,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let created = match context.manager.begin() {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_transaction_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(token_out, created.raw()) } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

macro_rules! config_setter {
    ($name:ident, $abi:ty, $variant:ident) => {
        #[no_mangle]
        /// Replace one group in the pending configuration.
        ///
        /// # Safety
        /// `storage` must hold an initialized context and `value` must be aligned and readable.
        pub unsafe extern "C" fn $name(
            storage: *mut FocConfigContextStorage,
            raw_token: u32,
            value: *const $abi,
        ) -> FocConfigStatus {
            let context = match unsafe { context_mut(storage) } {
                Ok(value) => value,
                Err(status) => return status,
            };
            let token = match token(raw_token) {
                Ok(value) => value,
                Err(status) => return context.record(status, 0),
            };
            let value = match unsafe { read_copy(value) } {
                Ok(value) => value,
                Err(status) => return context.record(status, 0),
            };
            let result = context
                .manager
                .set(token, ConfigPatch::$variant(value.to_config()));
            context.record_transaction_result(result)
        }
    };
}

config_setter!(foc_rust_config_set_board, FocConfigBoardAbi, Board);
config_setter!(foc_rust_config_set_motor, FocConfigMotorAbi, Motor);
config_setter!(foc_rust_config_set_inverter, FocConfigInverterAbi, Inverter);
config_setter!(foc_rust_config_set_axis, FocConfigAxisAbi, Axis);
config_setter!(foc_rust_config_set_app, FocConfigAppAbi, App);
config_setter!(
    foc_rust_config_set_calibration,
    FocConfigCalibrationAbi,
    Calibration
);

#[no_mangle]
/// Replace the external-I/O group in the pending configuration.
///
/// # Safety
/// `storage` must hold an initialized context and `value` must be aligned and readable.
pub unsafe extern "C" fn foc_rust_config_set_external_io(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    value: *const ExternalIoConfig,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let value = match unsafe { read_copy(value) } {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let result = context.manager.set(token, ConfigPatch::ExternalIo(value));
    context.record_transaction_result(result)
}

#[no_mangle]
/// Validate the pending configuration.
///
/// # Safety
/// `storage` must hold an initialized configuration context.
pub unsafe extern "C" fn foc_rust_config_validate(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let result = context.manager.validate(token);
    context.record_transaction_result(result)
}

unsafe fn read_guard(
    context: &mut ConfigAbiContext,
    guard: *const FocConfigApplyGuardAbi,
) -> Result<ConfigApplyGuard, FocConfigStatus> {
    match unsafe { read_copy(guard) }.and_then(FocConfigApplyGuardAbi::to_guard) {
        Ok(value) => Ok(value),
        Err(status) => Err(context.record(status, 0)),
    }
}

#[no_mangle]
/// Prepare a non-realtime apply plan after checking the supplied safety guard.
///
/// # Safety
/// Input pointers must be aligned/readable and `output` must be aligned/writable.
pub unsafe extern "C" fn foc_rust_config_prepare_apply(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    guard: *const FocConfigApplyGuardAbi,
    output: *mut FocConfigApplyPlanAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let plan = match context.manager.prepare_volatile_apply(token, guard) {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_transaction_error(error);
            return context.record(status, detail);
        }
    };
    match unsafe { write_copy(output, FocConfigApplyPlanAbi::from_plan(plan)) } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Confirm that the external executor applied the prepared plan successfully.
///
/// # Safety
/// Input pointers must be aligned and readable; `storage` must be initialized.
pub unsafe extern "C" fn foc_rust_config_confirm_apply(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    guard: *const FocConfigApplyGuardAbi,
    plan: *const FocConfigApplyPlanAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let plan = match unsafe { read_copy(plan) }.and_then(FocConfigApplyPlanAbi::to_plan) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let result = context.manager.confirm_volatile_apply(token, guard, plan);
    context.record_transaction_result(result)
}

#[no_mangle]
/// Cancel an apply that has been prepared but not confirmed.
///
/// # Safety
/// `storage` must hold an initialized configuration context.
pub unsafe extern "C" fn foc_rust_config_cancel_apply(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let result = context.manager.cancel_volatile_apply(token);
    context.record_transaction_result(result)
}

#[no_mangle]
/// Roll the active configuration back to its last committed baseline.
///
/// # Safety
/// `storage` must be initialized and `guard` must be aligned and readable.
pub unsafe extern "C" fn foc_rust_config_rollback(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    guard: *const FocConfigApplyGuardAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context.manager.rollback(token, guard);
    context.record_transaction_result(result)
}

#[no_mangle]
/// Prepare an atomic slot write plan for the currently applied configuration.
///
/// # Safety
/// Inputs must be aligned/readable and `output` must be aligned/writable.
pub unsafe extern "C" fn foc_rust_config_prepare_commit(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    guard: *const FocConfigApplyGuardAbi,
    slots: *const FocConfigSlotsAbi,
    output: *mut FocConfigCommitPlanAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let slots = match unsafe { read_copy(slots) } {
        Ok(value) => slots_from_abi(value),
        Err(status) => return context.record(status, 0),
    };
    let write = match context.manager.prepare_commit(token, guard, &slots) {
        Ok(value) => value,
        Err(error) => {
            let (status, detail) = map_transaction_error(error);
            return context.record(status, detail);
        }
    };
    let plan = FocConfigCommitPlanAbi {
        struct_size: size_of::<FocConfigCommitPlanAbi>() as u32,
        abi_version: FOC_CONFIG_ABI_VERSION,
        token: raw_token,
        target_slot: write.target_slot as u32,
        sequence: write.sequence,
        write_step_count: write.write_step_count() as u32,
        reserved0: 0,
        reserved1: 0,
    };
    context.prepared_write = Some(write);
    match unsafe { write_copy(output, plan) } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Return one byte of the previously prepared storage write sequence.
///
/// # Safety
/// `storage` must be initialized and `output` must be aligned and writable.
pub unsafe extern "C" fn foc_rust_config_commit_write_byte(
    storage: *mut FocConfigContextStorage,
    step: u32,
    output: *mut FocConfigWriteByteAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let write = match context.prepared_write.as_ref() {
        Some(value) => value,
        None => return context.record(FocConfigStatus::InvalidState, 0),
    };
    let byte = match usize::try_from(step)
        .ok()
        .and_then(|index| write.write_byte(index))
    {
        Some(value) => value,
        None => return context.record(FocConfigStatus::InvalidArgument, 0),
    };
    let output_value = FocConfigWriteByteAbi {
        offset: u32::from(byte.offset),
        value: u32::from(byte.value),
    };
    match unsafe { write_copy(output, output_value) } {
        Ok(()) => context.record(FocConfigStatus::Ok, 0),
        Err(status) => context.record(status, 0),
    }
}

#[no_mangle]
/// Verify Flash readback and finish or fault the configuration transaction.
///
/// # Safety
/// Input pointers must be aligned/readable and `storage` must be initialized.
pub unsafe extern "C" fn foc_rust_config_confirm_commit(
    storage: *mut FocConfigContextStorage,
    raw_token: u32,
    guard: *const FocConfigApplyGuardAbi,
    stored_slot_index: u32,
    stored_slot: *const FocConfigSlotAbi,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let token = match token(raw_token) {
        Ok(value) => value,
        Err(status) => return context.record(status, 0),
    };
    let guard = match unsafe { read_guard(context, guard) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let stored_slot = match unsafe { read_copy(stored_slot) } {
        Ok(value) => ConfigSlot::from_bytes(value.bytes),
        Err(status) => return context.record(status, 0),
    };
    let index = match usize::try_from(stored_slot_index) {
        Ok(value) => value,
        Err(_) => return context.record(FocConfigStatus::InvalidArgument, 0),
    };
    let result = context
        .manager
        .confirm_commit(token, guard, index, &stored_slot);
    context.prepared_write = None;
    context.record_transaction_result(result)
}

#[no_mangle]
/// Validate whether the active configuration may pass the Axis arm gate.
///
/// # Safety
/// `storage` must hold an initialized configuration context.
pub unsafe extern "C" fn foc_rust_config_validate_active_for_arm(
    storage: *mut FocConfigContextStorage,
) -> FocConfigStatus {
    let context = match unsafe { context_mut(storage) } {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = context.manager.validate_active_for_arm();
    // Preserve the storage failure as the durable diagnostic root cause.  An
    // arm-gate poll still returns Busy, but must not erase why recovery is
    // required from the management-plane status snapshot.
    if context.manager.status().state == ConfigTransactionState::StorageFault {
        return match result {
            Ok(()) => FocConfigStatus::Ok,
            Err(error) => map_transaction_error(error).0,
        };
    }
    context.record_transaction_result(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_control::{
        prepare_config_save, ConfigApproval, FeedbackMode, BOARD_CAP_PHASE_VOLTAGE_SENSE,
        BOARD_CAP_THREE_SHUNT_CURRENT, COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
        CONFIG_GROUP_EXTERNAL_IO, CONFIG_GROUP_VERSION,
    };

    const BOARD_ID: u32 = 0x4311_6001;
    const MOTOR_ID: u32 = 0x2804_1007;

    fn bundle(revision: u32) -> ConfigBundle {
        ConfigBundle {
            bundle_revision: revision,
            generated_by_version: 1,
            board: BoardConfig {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT | BOARD_CAP_PHASE_VOLTAGE_SENSE,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.012,
            },
            motor: MotorConfig {
                version: CONFIG_GROUP_VERSION,
                motor_id: MOTOR_ID,
                pole_pairs: 7,
                phase_resistance_ohm: 4.9667,
                d_inductance_h: 0.0012,
                q_inductance_h: 0.0013,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: InverterConfig {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 0x1600_0001,
                maximum_phase_current_a: 2.0,
                minimum_bus_voltage_v: 7.0,
                maximum_bus_voltage_v: 18.0,
                dead_time_s: 550.0e-9,
                transistor_drop_v: 0.35,
                brake_resistance_ohm: 0.0,
                maximum_duty: 0.95,
            },
            axis: AxisConfig {
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
            app: AppConfig {
                version: CONFIG_GROUP_VERSION,
                can_node_id: 1,
                uart_baud: 115_200,
                command_source_count: 1,
                command_sources: [
                    CommandSourceConfig {
                        source_id: 1,
                        priority: 10,
                        permissions: COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                ],
            },
            external_io: ExternalIoConfig::default(),
            calibration: CalibrationData {
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

    fn guard() -> FocConfigApplyGuardAbi {
        FocConfigApplyGuardAbi {
            struct_size: size_of::<FocConfigApplyGuardAbi>() as u32,
            abi_version: FOC_CONFIG_ABI_VERSION,
            axis_state: AxisState::Disabled as u32,
            drive_active: 0,
            active_fault_flags: 0,
        }
    }

    fn storage() -> FocConfigContextStorage {
        FocConfigContextStorage {
            bytes: [0; FOC_CONFIG_CONTEXT_CAPACITY],
        }
    }

    fn slots_with_approved(config: ConfigBundle) -> FocConfigSlotsAbi {
        let mut slots = [ConfigSlot::erased(), ConfigSlot::erased()];
        let write = prepare_config_save(&slots, config, ConfigApproval::Approved).unwrap();
        write.apply_complete(&mut slots);
        FocConfigSlotsAbi {
            bytes: [*slots[0].as_bytes(), *slots[1].as_bytes()],
        }
    }

    unsafe fn begin_motor_change(
        storage: &mut FocConfigContextStorage,
        active: ConfigBundle,
    ) -> (u32, FocConfigApplyPlanAbi) {
        let active_abi = FocConfigBundleAbi::from_bundle(active);
        assert_eq!(
            unsafe { foc_rust_config_init(storage, &active_abi, BOARD_ID, MOTOR_ID) },
            FocConfigStatus::Ok
        );
        let mut token = 0;
        assert_eq!(
            unsafe { foc_rust_config_begin(storage, &mut token) },
            FocConfigStatus::Ok
        );
        let mut motor = FocConfigMotorAbi::from_config(active.motor);
        motor.phase_resistance_ohm = 5.1;
        assert_eq!(
            unsafe { foc_rust_config_set_motor(storage, token, &motor) },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_config_validate(storage, token) },
            FocConfigStatus::Ok
        );
        let mut plan = FocConfigApplyPlanAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_prepare_apply(storage, token, &guard(), &mut plan) },
            FocConfigStatus::Ok
        );
        (token, plan)
    }

    #[test]
    fn management_abi_layout_and_bundle_round_trip_are_stable() {
        assert_eq!(foc_rust_config_abi_version(), 0x0004_0000);
        assert!(foc_rust_config_context_required_size() <= FOC_CONFIG_CONTEXT_CAPACITY as u32);
        assert!(foc_rust_config_context_required_align() <= 8);
        let config = bundle(7);
        assert_eq!(
            FocConfigBundleAbi::from_bundle(config).to_bundle(),
            Ok(config)
        );
    }

    #[test]
    fn fake_client_runs_begin_set_validate_apply_and_commit() {
        let active = bundle(10);
        let mut storage = storage();
        let (token, plan) = unsafe { begin_motor_change(&mut storage, active) };
        assert_eq!(plan.apply_class, 1);
        assert_eq!(
            unsafe { foc_rust_config_confirm_apply(&mut storage, token, &guard(), &plan) },
            FocConfigStatus::Ok
        );

        let mut slots = slots_with_approved(active);
        let mut commit = FocConfigCommitPlanAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_config_prepare_commit(&mut storage, token, &guard(), &slots, &mut commit)
            },
            FocConfigStatus::Ok
        );
        let target = commit.target_slot as usize;
        slots.bytes[target] = [0xFF; CONFIG_SLOT_SIZE];
        for step in 0..commit.write_step_count {
            let mut byte = FocConfigWriteByteAbi::default();
            assert_eq!(
                unsafe { foc_rust_config_commit_write_byte(&mut storage, step, &mut byte) },
                FocConfigStatus::Ok
            );
            slots.bytes[target][byte.offset as usize] = byte.value as u8;
        }
        let stored = FocConfigSlotAbi {
            bytes: slots.bytes[target],
        };
        assert_eq!(
            unsafe {
                foc_rust_config_confirm_commit(
                    &mut storage,
                    token,
                    &guard(),
                    target as u32,
                    &stored,
                )
            },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_config_validate_active_for_arm(&mut storage) },
            FocConfigStatus::Ok
        );
        let mut output = FocConfigBundleAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_get_active(&mut storage, &mut output) },
            FocConfigStatus::Ok
        );
        assert_eq!(output.bundle_revision, 11);
        assert_eq!(output.motor.phase_resistance_ohm, 5.1);
    }

    #[test]
    fn external_io_group_uses_v4_abi_and_management_apply() {
        let active = bundle(10);
        let active_abi = FocConfigBundleAbi::from_bundle(active);
        let mut storage = storage();
        assert_eq!(
            unsafe { foc_rust_config_init(&mut storage, &active_abi, BOARD_ID, MOTOR_ID) },
            FocConfigStatus::Ok
        );
        let mut token = 0;
        assert_eq!(
            unsafe { foc_rust_config_begin(&mut storage, &mut token) },
            FocConfigStatus::Ok
        );
        let external = ExternalIoConfig {
            revision: 2,
            ..ExternalIoConfig::default()
        };
        assert_eq!(
            unsafe { foc_rust_config_set_external_io(&mut storage, token, &external) },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_config_validate(&mut storage, token) },
            FocConfigStatus::Ok
        );
        let mut pending = FocConfigBundleAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_get_pending(&mut storage, token, &mut pending) },
            FocConfigStatus::Ok
        );
        assert_eq!(pending.external_io, external);
        let mut plan = FocConfigApplyPlanAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_prepare_apply(&mut storage, token, &guard(), &mut plan) },
            FocConfigStatus::Ok
        );
        assert_eq!(plan.changed_groups, CONFIG_GROUP_EXTERNAL_IO);
        assert_eq!(plan.apply_class, 0);
    }

    #[test]
    fn unsafe_guard_stale_token_and_bad_fields_fail_closed() {
        let active = bundle(1);
        let mut storage = storage();
        let active_abi = FocConfigBundleAbi::from_bundle(active);
        assert_eq!(
            unsafe { foc_rust_config_init(&mut storage, &active_abi, BOARD_ID, MOTOR_ID) },
            FocConfigStatus::Ok
        );
        let mut token = 0;
        assert_eq!(
            unsafe { foc_rust_config_begin(&mut storage, &mut token) },
            FocConfigStatus::Ok
        );
        let mut motor = FocConfigMotorAbi::from_config(active.motor);
        motor.phase_resistance_ohm = 0.0;
        assert_eq!(
            unsafe { foc_rust_config_set_motor(&mut storage, token, &motor) },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_config_validate(&mut storage, token) },
            FocConfigStatus::ValidationFailed
        );
        assert_eq!(
            unsafe { foc_rust_config_set_motor(&mut storage, token + 1, &motor) },
            FocConfigStatus::StaleToken
        );

        motor.phase_resistance_ohm = 5.1;
        assert_eq!(
            unsafe { foc_rust_config_set_motor(&mut storage, token, &motor) },
            FocConfigStatus::Ok
        );
        assert_eq!(
            unsafe { foc_rust_config_validate(&mut storage, token) },
            FocConfigStatus::Ok
        );
        let mut unsafe_guard = guard();
        unsafe_guard.drive_active = 1;
        let mut plan = FocConfigApplyPlanAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_prepare_apply(&mut storage, token, &unsafe_guard, &mut plan) },
            FocConfigStatus::UnsafeToApply
        );
    }

    #[test]
    fn interrupted_fake_flash_readback_locks_arm_in_storage_fault() {
        let active = bundle(3);
        let mut storage = storage();
        let (token, plan) = unsafe { begin_motor_change(&mut storage, active) };
        assert_eq!(
            unsafe { foc_rust_config_confirm_apply(&mut storage, token, &guard(), &plan) },
            FocConfigStatus::Ok
        );
        let slots = slots_with_approved(active);
        let mut commit = FocConfigCommitPlanAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_config_prepare_commit(&mut storage, token, &guard(), &slots, &mut commit)
            },
            FocConfigStatus::Ok
        );
        let erased = FocConfigSlotAbi::default();
        assert_eq!(
            unsafe {
                foc_rust_config_confirm_commit(
                    &mut storage,
                    token,
                    &guard(),
                    commit.target_slot,
                    &erased,
                )
            },
            FocConfigStatus::StorageRecordFailed
        );
        assert_eq!(
            unsafe { foc_rust_config_validate_active_for_arm(&mut storage) },
            FocConfigStatus::Busy
        );
        let mut status = FocConfigTransactionStatusAbi::default();
        assert_eq!(
            unsafe { foc_rust_config_get_status(&mut storage, &mut status) },
            FocConfigStatus::Ok
        );
        assert_eq!(status.state, 6);
        assert_eq!(
            status.last_result,
            FocConfigStatus::StorageRecordFailed as u32
        );
        assert_eq!(status.last_detail, 201);
    }
}
