use core::mem::size_of;

use foc_input::{
    normalize_centered_input, CenteredInputConfig, CommandMapConfig, InputError, StepDirConfig,
};

pub const FOC_INPUT_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_INPUT_CONFIG_VERSION: u32 = 1;
pub const FOC_INPUT_OUTPUT_VERSION: u32 = 1;

pub const FOC_INPUT_STATUS_OK: u32 = 0;
pub const FOC_INPUT_STATUS_INVALID_ARGUMENT: u32 = 1;
pub const FOC_INPUT_STATUS_INVALID_LAYOUT: u32 = 2;
pub const FOC_INPUT_STATUS_INVALID_CONFIG: u32 = 3;
pub const FOC_INPUT_STATUS_OUT_OF_RANGE: u32 = 4;
pub const FOC_INPUT_STATUS_INVALID_VALUE: u32 = 5;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocCenteredInputConfig {
    pub struct_size: u32,
    pub version: u32,
    pub raw_min: i32,
    pub raw_neutral: i32,
    pub raw_max: i32,
    pub deadband: u32,
    pub control_mode: u32,
    pub reserved: u32,
    pub negative_limit_si: f32,
    pub positive_limit_si: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocStepDirInputConfig {
    pub struct_size: u32,
    pub version: u32,
    pub full_steps_per_revolution: u32,
    pub microsteps: u32,
    pub gear_numerator: u32,
    pub gear_denominator: u32,
    pub direction: i32,
    pub zero_count: i32,
    pub zero_position_rad: f32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocInputNormalizationOutput {
    pub struct_size: u32,
    pub version: u32,
    pub status: u32,
    pub quality_flags: u32,
    pub raw_value: i32,
    pub normalized_value: f32,
    pub setpoint_si: f32,
}

const _: () = assert!(size_of::<FocCenteredInputConfig>() == 40);
const _: () = assert!(size_of::<FocStepDirInputConfig>() == 40);
const _: () = assert!(size_of::<FocInputNormalizationOutput>() == 28);

fn error_status(error: InputError) -> u32 {
    match error {
        InputError::InvalidConfig => FOC_INPUT_STATUS_INVALID_CONFIG,
        InputError::OutOfRange => FOC_INPUT_STATUS_OUT_OF_RANGE,
        InputError::InvalidValue => FOC_INPUT_STATUS_INVALID_VALUE,
    }
}

fn centered_config_valid(config: &FocCenteredInputConfig) -> bool {
    config.struct_size == size_of::<FocCenteredInputConfig>() as u32
        && config.version == FOC_INPUT_CONFIG_VERSION
        && config.reserved == 0
}

fn step_dir_config_valid(config: &FocStepDirInputConfig) -> bool {
    config.struct_size == size_of::<FocStepDirInputConfig>() as u32
        && config.version == FOC_INPUT_CONFIG_VERSION
        && config.reserved == 0
}

fn prepare_output(output: &mut FocInputNormalizationOutput, raw: i32) {
    *output = FocInputNormalizationOutput {
        struct_size: size_of::<FocInputNormalizationOutput>() as u32,
        version: FOC_INPUT_OUTPUT_VERSION,
        status: FOC_INPUT_STATUS_INVALID_ARGUMENT,
        raw_value: raw,
        ..FocInputNormalizationOutput::default()
    };
}

unsafe fn normalize_centered_ffi(
    config: *const FocCenteredInputConfig,
    raw: i32,
    output: *mut FocInputNormalizationOutput,
) -> u32 {
    if config.is_null() || output.is_null() {
        return FOC_INPUT_STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: pointers were checked for null; the C ABI requires naturally
    // aligned pointers to complete objects owned by the caller for this call.
    let config = unsafe { &*config };
    // SAFETY: same contract as above, with unique mutable output ownership.
    let output = unsafe { &mut *output };
    prepare_output(output, raw);
    if !centered_config_valid(config) {
        output.status = FOC_INPUT_STATUS_INVALID_LAYOUT;
        return output.status;
    }
    let result = normalize_centered_input(
        CenteredInputConfig {
            raw_min: config.raw_min,
            raw_neutral: config.raw_neutral,
            raw_max: config.raw_max,
            deadband: config.deadband,
        },
        CommandMapConfig {
            control_mode: config.control_mode,
            negative_limit_si: config.negative_limit_si,
            positive_limit_si: config.positive_limit_si,
        },
        raw,
    );
    match result {
        Ok(value) => {
            output.status = FOC_INPUT_STATUS_OK;
            output.quality_flags = value.quality_flags;
            output.normalized_value = value.normalized;
            output.setpoint_si = value.setpoint_si;
        }
        Err(error) => output.status = error_status(error),
    }
    output.status
}

#[no_mangle]
/// Normalize one PWM pulse-width sample and map it to an SI setpoint candidate.
///
/// # Safety
///
/// `config` must point to a readable, naturally aligned complete configuration
/// object. `output` must point to a writable, naturally aligned complete output
/// object exclusively owned by the caller for the duration of this call.
pub unsafe extern "C" fn foc_rust_input_normalize_pwm(
    config: *const FocCenteredInputConfig,
    pulse_width_us: u32,
    output: *mut FocInputNormalizationOutput,
) -> u32 {
    let Ok(raw) = i32::try_from(pulse_width_us) else {
        if !output.is_null() {
            // SAFETY: caller supplied a non-null output object under the ABI contract.
            let output = unsafe { &mut *output };
            prepare_output(output, i32::MAX);
            output.status = FOC_INPUT_STATUS_OUT_OF_RANGE;
        }
        return FOC_INPUT_STATUS_OUT_OF_RANGE;
    };
    // SAFETY: forwarded unchanged under this function's C ABI pointer contract.
    unsafe { normalize_centered_ffi(config, raw, output) }
}

#[no_mangle]
/// Normalize one centered ADC sample and map it to an SI setpoint candidate.
///
/// # Safety
///
/// `config` must point to a readable, naturally aligned complete configuration
/// object. `output` must point to a writable, naturally aligned complete output
/// object exclusively owned by the caller for the duration of this call.
pub unsafe extern "C" fn foc_rust_input_normalize_analog(
    config: *const FocCenteredInputConfig,
    adc_counts: i32,
    output: *mut FocInputNormalizationOutput,
) -> u32 {
    // SAFETY: forwarded unchanged under this function's C ABI pointer contract.
    unsafe { normalize_centered_ffi(config, adc_counts, output) }
}

#[no_mangle]
/// Convert one accumulated Step/Dir count into output position in radians.
///
/// # Safety
///
/// `config` must point to a readable, naturally aligned complete configuration
/// object. `output` must point to a writable, naturally aligned complete output
/// object exclusively owned by the caller for the duration of this call.
pub unsafe extern "C" fn foc_rust_input_convert_step_dir(
    config: *const FocStepDirInputConfig,
    accumulated_count: i32,
    output: *mut FocInputNormalizationOutput,
) -> u32 {
    if config.is_null() || output.is_null() {
        return FOC_INPUT_STATUS_INVALID_ARGUMENT;
    }
    // SAFETY: pointers follow the same complete-object and unique-output ABI contract.
    let config = unsafe { &*config };
    // SAFETY: output is non-null and exclusively owned for the duration of this call.
    let output = unsafe { &mut *output };
    prepare_output(output, accumulated_count);
    if !step_dir_config_valid(config) {
        output.status = FOC_INPUT_STATUS_INVALID_LAYOUT;
        return output.status;
    }
    let result = StepDirConfig {
        full_steps_per_revolution: config.full_steps_per_revolution,
        microsteps: config.microsteps,
        gear_numerator: config.gear_numerator,
        gear_denominator: config.gear_denominator,
        direction: config.direction,
        zero_count: config.zero_count,
        zero_position_rad: config.zero_position_rad,
    }
    .position_rad(accumulated_count);
    match result {
        Ok(position_rad) => {
            output.status = FOC_INPUT_STATUS_OK;
            output.quality_flags = foc_input::INPUT_QUALITY_CALIBRATED;
            output.normalized_value = position_rad;
            output.setpoint_si = position_rad;
        }
        Err(error) => output.status = error_status(error),
    }
    output.status
}

#[cfg(test)]
mod tests {
    use super::*;

    fn centered() -> FocCenteredInputConfig {
        FocCenteredInputConfig {
            struct_size: size_of::<FocCenteredInputConfig>() as u32,
            version: FOC_INPUT_CONFIG_VERSION,
            raw_min: 1_000,
            raw_neutral: 1_500,
            raw_max: 2_000,
            deadband: 20,
            control_mode: foc_input::CONTROL_MODE_VELOCITY,
            reserved: 0,
            negative_limit_si: 100.0,
            positive_limit_si: 200.0,
        }
    }

    #[test]
    fn centered_abi_maps_pwm_and_analog() {
        let config = centered();
        let mut output = FocInputNormalizationOutput::default();
        let pwm_status = unsafe { foc_rust_input_normalize_pwm(&config, 2_000, &mut output) };
        assert_eq!(pwm_status, FOC_INPUT_STATUS_OK);
        assert_eq!(output.setpoint_si, 200.0);
        let analog_status = unsafe { foc_rust_input_normalize_analog(&config, 1_000, &mut output) };
        assert_eq!(analog_status, FOC_INPUT_STATUS_OK);
        assert_eq!(output.setpoint_si, -100.0);
    }

    #[test]
    fn abi_rejects_layout_range_and_null() {
        let mut config = centered();
        let mut output = FocInputNormalizationOutput::default();
        config.version = 2;
        assert_eq!(
            unsafe { foc_rust_input_normalize_pwm(&config, 1_500, &mut output) },
            FOC_INPUT_STATUS_INVALID_LAYOUT
        );
        config = centered();
        assert_eq!(
            unsafe { foc_rust_input_normalize_analog(&config, 999, &mut output) },
            FOC_INPUT_STATUS_OUT_OF_RANGE
        );
        assert_eq!(
            unsafe { foc_rust_input_normalize_pwm(core::ptr::null(), 1_500, &mut output) },
            FOC_INPUT_STATUS_INVALID_ARGUMENT
        );
    }

    #[test]
    fn step_dir_abi_returns_position_only() {
        let config = FocStepDirInputConfig {
            struct_size: size_of::<FocStepDirInputConfig>() as u32,
            version: FOC_INPUT_CONFIG_VERSION,
            full_steps_per_revolution: 200,
            microsteps: 16,
            gear_numerator: 1,
            gear_denominator: 1,
            direction: 1,
            zero_count: 0,
            zero_position_rad: 0.0,
            reserved: 0,
        };
        let mut output = FocInputNormalizationOutput::default();
        assert_eq!(
            unsafe { foc_rust_input_convert_step_dir(&config, 3_200, &mut output) },
            FOC_INPUT_STATUS_OK
        );
        assert!((output.setpoint_si - core::f32::consts::TAU).abs() < 1.0e-5);
    }
}
