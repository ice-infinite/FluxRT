//! Independent management-plane ABI for thermal, DC-bus and regeneration policy.
//!
//! The ABI is deliberately separate from the 12 kHz realtime controller ABI.
//! It owns no ADC, brake output or PWM register. A C management owner supplies
//! calibrated SI samples and must apply the returned limits through the normal
//! MotorService and immediate C-side hardware protection chain.

use core::mem::{align_of, size_of};
use core::ptr;

use foc_control::{
    PowerSupervisor, PowerSupervisorConfig, PowerSupervisorConfigError, PowerSupervisorInput,
    PowerSupervisorOutput, PowerSupervisorResetError,
};

pub const FOC_POWER_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_POWER_CONTEXT_CAPACITY: usize = 128;

const POWER_CONTEXT_MAGIC: u32 = 0x4650_5752; // "FPWR"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocPowerStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    InvalidTemperatureWindow = 3,
    InvalidBusWindow = 4,
    InvalidSourceCapability = 5,
    InvalidSinkCapability = 6,
    InvalidBrakeCapability = 7,
    UnsafeReset = 8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocPowerRuntimeConfig {
    pub struct_size: u32,
    pub abi_version: u32,
    pub enabled: u32,
    pub temperature_sensor_required: u32,
    pub minimum_valid_temperature_c: f32,
    pub maximum_valid_temperature_c: f32,
    pub thermal_derating_release_c: f32,
    pub thermal_derating_start_c: f32,
    pub over_temperature_trip_c: f32,
    pub bus_undervoltage_trip_v: f32,
    pub bus_undervoltage_recovery_v: f32,
    pub bus_overvoltage_recovery_v: f32,
    pub bus_overvoltage_trip_v: f32,
    pub source_current_limit_a: f32,
    pub regeneration_allowed: u32,
    pub sink_current_limit_a: f32,
    pub brake_resistor_available: u32,
    pub brake_current_limit_a: f32,
    pub brake_release_voltage_v: f32,
    pub brake_engage_voltage_v: f32,
}

impl FocPowerRuntimeConfig {
    fn to_config(self) -> Result<PowerSupervisorConfig, FocPowerStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_POWER_ABI_VERSION
            || self.enabled > 1
            || self.temperature_sensor_required > 1
            || self.regeneration_allowed > 1
            || self.brake_resistor_available > 1
        {
            return Err(FocPowerStatus::InvalidArgument);
        }
        Ok(PowerSupervisorConfig {
            enabled: self.enabled != 0,
            temperature_sensor_required: self.temperature_sensor_required != 0,
            minimum_valid_temperature_c: self.minimum_valid_temperature_c,
            maximum_valid_temperature_c: self.maximum_valid_temperature_c,
            thermal_derating_release_c: self.thermal_derating_release_c,
            thermal_derating_start_c: self.thermal_derating_start_c,
            over_temperature_trip_c: self.over_temperature_trip_c,
            bus_undervoltage_trip_v: self.bus_undervoltage_trip_v,
            bus_undervoltage_recovery_v: self.bus_undervoltage_recovery_v,
            bus_overvoltage_recovery_v: self.bus_overvoltage_recovery_v,
            bus_overvoltage_trip_v: self.bus_overvoltage_trip_v,
            source_current_limit_a: self.source_current_limit_a,
            regeneration_allowed: self.regeneration_allowed != 0,
            sink_current_limit_a: self.sink_current_limit_a,
            brake_resistor_available: self.brake_resistor_available != 0,
            brake_current_limit_a: self.brake_current_limit_a,
            brake_release_voltage_v: self.brake_release_voltage_v,
            brake_engage_voltage_v: self.brake_engage_voltage_v,
        })
    }

    pub fn disabled_default() -> Self {
        let config = PowerSupervisorConfig::default();
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_POWER_ABI_VERSION,
            enabled: 0,
            temperature_sensor_required: 0,
            minimum_valid_temperature_c: config.minimum_valid_temperature_c,
            maximum_valid_temperature_c: config.maximum_valid_temperature_c,
            thermal_derating_release_c: config.thermal_derating_release_c,
            thermal_derating_start_c: config.thermal_derating_start_c,
            over_temperature_trip_c: config.over_temperature_trip_c,
            bus_undervoltage_trip_v: config.bus_undervoltage_trip_v,
            bus_undervoltage_recovery_v: config.bus_undervoltage_recovery_v,
            bus_overvoltage_recovery_v: config.bus_overvoltage_recovery_v,
            bus_overvoltage_trip_v: config.bus_overvoltage_trip_v,
            source_current_limit_a: 0.0,
            regeneration_allowed: 0,
            sink_current_limit_a: 0.0,
            brake_resistor_available: 0,
            brake_current_limit_a: 0.0,
            brake_release_voltage_v: 0.0,
            brake_engage_voltage_v: 0.0,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocPowerInput {
    pub struct_size: u32,
    pub abi_version: u32,
    pub temperature_valid: u32,
    pub source_available: u32,
    pub sink_available: u32,
    pub brake_available: u32,
    pub temperature_c: f32,
    pub dc_bus_voltage_v: f32,
    pub requested_dc_current_a: f32,
}

impl FocPowerInput {
    fn to_input(self) -> Result<PowerSupervisorInput, FocPowerStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_POWER_ABI_VERSION
            || self.temperature_valid > 1
            || self.source_available > 1
            || self.sink_available > 1
            || self.brake_available > 1
        {
            return Err(FocPowerStatus::InvalidArgument);
        }
        Ok(PowerSupervisorInput {
            temperature_valid: self.temperature_valid != 0,
            temperature_c: self.temperature_c,
            dc_bus_voltage_v: self.dc_bus_voltage_v,
            requested_dc_current_a: self.requested_dc_current_a,
            source_available: self.source_available != 0,
            sink_available: self.sink_available != 0,
            brake_available: self.brake_available != 0,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocPowerOutput {
    pub struct_size: u32,
    pub abi_version: u32,
    pub configured: u32,
    pub enabled: u32,
    pub drive_allowed: u32,
    pub shutdown_requested: u32,
    pub active_fault_flags: u32,
    pub latched_fault_flags: u32,
    pub temperature_valid: u32,
    pub thermal_derating_active: u32,
    pub thermal_current_scale: f32,
    pub maximum_source_current_a: f32,
    pub maximum_regeneration_current_a: f32,
    pub requested_dc_current_a: f32,
    pub limited_dc_current_a: f32,
    pub source_limited: u32,
    pub regeneration_limited: u32,
    pub brake_requested: u32,
    pub brake_current_limit_a: f32,
}

impl FocPowerOutput {
    fn from_output(output: PowerSupervisorOutput) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_POWER_ABI_VERSION,
            configured: u32::from(output.configured),
            enabled: u32::from(output.enabled),
            drive_allowed: u32::from(output.drive_allowed),
            shutdown_requested: u32::from(output.shutdown_requested),
            active_fault_flags: output.active_fault_flags,
            latched_fault_flags: output.latched_fault_flags,
            temperature_valid: u32::from(output.temperature_valid),
            thermal_derating_active: u32::from(output.thermal_derating_active),
            thermal_current_scale: output.thermal_current_scale,
            maximum_source_current_a: output.maximum_source_current_a,
            maximum_regeneration_current_a: output.maximum_regeneration_current_a,
            requested_dc_current_a: output.requested_dc_current_a,
            limited_dc_current_a: output.limited_dc_current_a,
            source_limited: u32::from(output.source_limited),
            regeneration_limited: u32::from(output.regeneration_limited),
            brake_requested: u32::from(output.brake_requested),
            brake_current_limit_a: output.brake_current_limit_a,
        }
    }
}

#[repr(C, align(8))]
pub struct FocPowerContextStorage {
    pub bytes: [u8; FOC_POWER_CONTEXT_CAPACITY],
}

struct PowerAbiContext {
    magic: u32,
    supervisor: PowerSupervisor,
}

impl PowerAbiContext {
    fn new() -> Self {
        Self {
            magic: POWER_CONTEXT_MAGIC,
            supervisor: PowerSupervisor::default(),
        }
    }
}

const _: () = assert!(size_of::<FocPowerRuntimeConfig>() == 80);
const _: () = assert!(size_of::<FocPowerInput>() == 36);
const _: () = assert!(size_of::<FocPowerOutput>() == 76);
const _: () = assert!(size_of::<PowerAbiContext>() <= FOC_POWER_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<PowerAbiContext>() <= align_of::<FocPowerContextStorage>());

fn context_mut<'a>(
    storage: *mut FocPowerContextStorage,
) -> Result<&'a mut PowerAbiContext, FocPowerStatus> {
    if storage.is_null() || !(storage as usize).is_multiple_of(align_of::<PowerAbiContext>()) {
        return Err(FocPowerStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<PowerAbiContext>() };
    if context.magic != POWER_CONTEXT_MAGIC {
        return Err(FocPowerStatus::NotInitialized);
    }
    Ok(context)
}

fn map_config_error(error: PowerSupervisorConfigError) -> FocPowerStatus {
    match error {
        PowerSupervisorConfigError::InvalidTemperatureWindow => {
            FocPowerStatus::InvalidTemperatureWindow
        }
        PowerSupervisorConfigError::InvalidBusWindow => FocPowerStatus::InvalidBusWindow,
        PowerSupervisorConfigError::InvalidSourceCapability => {
            FocPowerStatus::InvalidSourceCapability
        }
        PowerSupervisorConfigError::InvalidSinkCapability => FocPowerStatus::InvalidSinkCapability,
        PowerSupervisorConfigError::InvalidBrakeCapability => {
            FocPowerStatus::InvalidBrakeCapability
        }
    }
}

#[no_mangle]
pub extern "C" fn foc_rust_power_abi_version() -> u32 {
    FOC_POWER_ABI_VERSION
}

#[no_mangle]
/// Initializes a caller-owned, fixed-capacity power-supervisor context.
///
/// # Safety
///
/// `storage` must be null or point to writable, correctly aligned storage of at
/// least `size_of::<FocPowerContextStorage>()` bytes. The caller must provide
/// exclusive access for the duration of this call.
pub unsafe extern "C" fn foc_rust_power_init(
    storage: *mut FocPowerContextStorage,
) -> FocPowerStatus {
    if storage.is_null() || !(storage as usize).is_multiple_of(align_of::<PowerAbiContext>()) {
        return FocPowerStatus::InvalidArgument;
    }
    unsafe { ptr::write(storage.cast::<PowerAbiContext>(), PowerAbiContext::new()) };
    FocPowerStatus::Ok
}

#[no_mangle]
/// Writes the fail-closed, disabled runtime configuration.
///
/// # Safety
///
/// `output` must be null or point to a writable `FocPowerRuntimeConfig`. The
/// caller must provide exclusive access for the duration of this call.
pub unsafe extern "C" fn foc_rust_power_default_config(
    output: *mut FocPowerRuntimeConfig,
) -> FocPowerStatus {
    if output.is_null() {
        return FocPowerStatus::InvalidArgument;
    }
    unsafe { ptr::write(output, FocPowerRuntimeConfig::disabled_default()) };
    FocPowerStatus::Ok
}

#[no_mangle]
/// Transactionally applies a validated runtime configuration to one context.
///
/// # Safety
///
/// `storage` must reference a context initialized by `foc_rust_power_init` and
/// be exclusively borrowed for the call. `config` must be null or point to a
/// readable `FocPowerRuntimeConfig` that remains valid for the call.
pub unsafe extern "C" fn foc_rust_power_configure(
    storage: *mut FocPowerContextStorage,
    config: *const FocPowerRuntimeConfig,
) -> FocPowerStatus {
    let context = match context_mut(storage) {
        Ok(value) => value,
        Err(status) => return status,
    };
    if config.is_null() {
        return FocPowerStatus::InvalidArgument;
    }
    let config = match unsafe { ptr::read(config) }.to_config() {
        Ok(value) => value,
        Err(status) => return status,
    };
    match context.supervisor.configure(config) {
        Ok(()) => FocPowerStatus::Ok,
        Err(error) => map_config_error(error),
    }
}

#[no_mangle]
/// Advances the management-rate supervisor by one sample.
///
/// # Safety
///
/// `storage` must reference an initialized context and be exclusively borrowed
/// for the call. `input` must be null or readable as `FocPowerInput`; `output`
/// must be null or writable as `FocPowerOutput`. Non-null pointers must remain
/// valid for the entire call and must not alias in a way that violates Rust's
/// exclusive access rules.
pub unsafe extern "C" fn foc_rust_power_step(
    storage: *mut FocPowerContextStorage,
    input: *const FocPowerInput,
    output: *mut FocPowerOutput,
) -> FocPowerStatus {
    let context = match context_mut(storage) {
        Ok(value) => value,
        Err(status) => return status,
    };
    if input.is_null() || output.is_null() {
        return FocPowerStatus::InvalidArgument;
    }
    let input = match unsafe { ptr::read(input) }.to_input() {
        Ok(value) => value,
        Err(status) => return status,
    };
    let result = FocPowerOutput::from_output(context.supervisor.step(input));
    unsafe { ptr::write(output, result) };
    FocPowerStatus::Ok
}

#[no_mangle]
/// Clears latched policy faults only when the supplied conditions are safe.
///
/// # Safety
///
/// `storage` must reference an initialized context and be exclusively borrowed
/// for the call. `input` must be null or point to a readable `FocPowerInput`
/// that remains valid for the duration of the call.
pub unsafe extern "C" fn foc_rust_power_reset_faults(
    storage: *mut FocPowerContextStorage,
    input: *const FocPowerInput,
) -> FocPowerStatus {
    let context = match context_mut(storage) {
        Ok(value) => value,
        Err(status) => return status,
    };
    if input.is_null() {
        return FocPowerStatus::InvalidArgument;
    }
    let input = match unsafe { ptr::read(input) }.to_input() {
        Ok(value) => value,
        Err(status) => return status,
    };
    match context.supervisor.reset_faults(input) {
        Ok(()) => FocPowerStatus::Ok,
        Err(PowerSupervisorResetError::NotConfigured) => FocPowerStatus::NotInitialized,
        Err(PowerSupervisorResetError::InvalidInput) => FocPowerStatus::InvalidArgument,
        Err(PowerSupervisorResetError::UnsafeConditions) => FocPowerStatus::UnsafeReset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input(current: f32) -> FocPowerInput {
        FocPowerInput {
            struct_size: size_of::<FocPowerInput>() as u32,
            abi_version: FOC_POWER_ABI_VERSION,
            temperature_valid: 1,
            source_available: 1,
            temperature_c: 25.0,
            dc_bus_voltage_v: 12.0,
            requested_dc_current_a: current,
            ..FocPowerInput::default()
        }
    }

    #[test]
    fn default_is_structurally_valid_and_runtime_disabled() {
        let mut storage = FocPowerContextStorage {
            bytes: [0; FOC_POWER_CONTEXT_CAPACITY],
        };
        let mut config = FocPowerRuntimeConfig::default();
        let mut output = FocPowerOutput::default();
        unsafe {
            assert_eq!(foc_rust_power_init(&mut storage), FocPowerStatus::Ok);
            assert_eq!(
                foc_rust_power_default_config(&mut config),
                FocPowerStatus::Ok
            );
            assert_eq!(
                foc_rust_power_configure(&mut storage, &config),
                FocPowerStatus::Ok
            );
            assert_eq!(
                foc_rust_power_step(&mut storage, &input(1.0), &mut output),
                FocPowerStatus::Ok
            );
        }
        assert_eq!(output.enabled, 0);
        assert_eq!(output.drive_allowed, 0);
        assert_eq!(output.shutdown_requested, 1);
        assert_eq!(output.limited_dc_current_a, 0.0);
    }

    #[test]
    fn invalid_config_is_transactional_and_fault_reset_is_guarded() {
        let mut storage = FocPowerContextStorage {
            bytes: [0; FOC_POWER_CONTEXT_CAPACITY],
        };
        let mut config = FocPowerRuntimeConfig::disabled_default();
        unsafe { assert_eq!(foc_rust_power_init(&mut storage), FocPowerStatus::Ok) };
        config.enabled = 1;
        config.temperature_sensor_required = 1;
        config.source_current_limit_a = 2.0;
        unsafe {
            assert_eq!(
                foc_rust_power_configure(&mut storage, &config),
                FocPowerStatus::Ok
            );
        }
        let mut invalid = config;
        invalid.bus_overvoltage_trip_v = f32::NAN;
        unsafe {
            assert_eq!(
                foc_rust_power_configure(&mut storage, &invalid),
                FocPowerStatus::InvalidBusWindow
            );
        }
        let mut bad_bus = input(1.0);
        bad_bus.dc_bus_voltage_v = 18.0;
        let mut output = FocPowerOutput::default();
        unsafe {
            assert_eq!(
                foc_rust_power_step(&mut storage, &bad_bus, &mut output),
                FocPowerStatus::Ok
            );
            assert_ne!(output.latched_fault_flags, 0);
            assert_eq!(
                foc_rust_power_reset_faults(&mut storage, &input(1.0)),
                FocPowerStatus::UnsafeReset
            );
            assert_eq!(
                foc_rust_power_reset_faults(&mut storage, &input(0.0)),
                FocPowerStatus::Ok
            );
        }
    }

    #[test]
    fn noncanonical_boolean_is_rejected_before_policy_execution() {
        let mut storage = FocPowerContextStorage {
            bytes: [0; FOC_POWER_CONTEXT_CAPACITY],
        };
        let mut output = FocPowerOutput::default();
        let mut invalid = input(0.0);
        invalid.brake_available = 2;
        unsafe {
            assert_eq!(foc_rust_power_init(&mut storage), FocPowerStatus::Ok);
            assert_eq!(
                foc_rust_power_step(&mut storage, &invalid, &mut output),
                FocPowerStatus::InvalidArgument
            );
        }
    }
}
