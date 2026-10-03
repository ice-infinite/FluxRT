//! Independent C ABI contract for the advanced-FOC policy layer.
//!
//! This ABI is versioned separately from the long-lived basic realtime ABI.
//! Adding the optional candidate therefore cannot silently alter the layout of
//! `FocRuntimeConfig` or `FocTelemetry`.  The controller still owns the state;
//! this module only defines fixed-layout configuration and diagnostics.

use core::mem::size_of;

use foc_control::{
    AdvancedFocConfig, AdvancedFocError, AdvancedFocOutput, AdvancedFocRegion,
    AdvancedModulationMode, CurrentCommand, FlyingStartState, MotorParameters,
    ADVANCED_FOC_CONFIG_VERSION, ADV_FOC_DECOUPLING, ADV_FOC_DPWM, ADV_FOC_FIELD_WEAKENING,
    ADV_FOC_FLYING_START, ADV_FOC_HFI, ADV_FOC_KNOWN_MASK, ADV_FOC_MTPA, ADV_FOC_MTPV,
    ADV_FOC_OVERMODULATION,
};

pub const FOC_ADVANCED_ABI_VERSION: u32 = 0x0001_0000;

/// Hardware-coupled features require an independently proven platform
/// capability.  Pure reference shaping and dq decoupling do not require bits.
pub const FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION: u32 = 1 << 0;
pub const FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE: u32 = 1 << 1;
pub const FOC_ADVANCED_CAPABILITY_HFI_INJECTION: u32 = 1 << 2;
pub const FOC_ADVANCED_CAPABILITY_PASSIVE_FLYING_START: u32 = 1 << 3;
pub const FOC_ADVANCED_CAPABILITY_KNOWN_MASK: u32 =
    FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION
        | FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE
        | FOC_ADVANCED_CAPABILITY_HFI_INJECTION
        | FOC_ADVANCED_CAPABILITY_PASSIVE_FLYING_START;

/// The current target bridge has no dynamic request transport for HFI voltage
/// injection or flying-start capture yet.  Their pure supervisor/state-machine
/// implementations are available to both simulators, but the target ABI rejects
/// them until that request path exists.
pub const FOC_ADVANCED_TARGET_RUNTIME_SUPPORTED_MASK: u32 = ADV_FOC_MTPA
    | ADV_FOC_FIELD_WEAKENING
    | ADV_FOC_MTPV
    | ADV_FOC_DECOUPLING
    | ADV_FOC_DPWM
    | ADV_FOC_OVERMODULATION;

pub const FOC_ADVANCED_STATUS_CONFIGURED: u32 = 1 << 0;
pub const FOC_ADVANCED_STATUS_CURRENT_LIMITED: u32 = 1 << 1;
pub const FOC_ADVANCED_STATUS_HFI_ANGLE_VALID: u32 = 1 << 2;
/// The advanced request was suppressed for this tick and the unchanged basic
/// current-loop reference/path was used instead.
pub const FOC_ADVANCED_STATUS_BASIC_FALLBACK: u32 = 1 << 3;
/// The advanced policy failed closed and latched `FOC_FAULT_ADVANCED_CONTROL`.
pub const FOC_ADVANCED_STATUS_FAULTED: u32 = 1 << 4;
/// A stopped-state configuration request was rejected without replacing the
/// last accepted supervisor/configuration transaction.
pub const FOC_ADVANCED_STATUS_CONFIG_REJECTED: u32 = 1 << 5;

// Stable reason bits.  They intentionally live in the existing V1
// `status_flags` word so adding the P5.3 safety diagnostics does not change the
// 68-byte C ABI or invalidate existing readers.
pub const FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE: u32 = 1 << 8;
pub const FOC_ADVANCED_REASON_INVALID_RUNTIME_INPUT: u32 = 1 << 9;
pub const FOC_ADVANCED_REASON_SUPERVISOR_STATE: u32 = 1 << 10;
pub const FOC_ADVANCED_REASON_OUTPUT_INVALID: u32 = 1 << 11;
pub const FOC_ADVANCED_REASON_CONFIG_IDENTITY: u32 = 1 << 12;
pub const FOC_ADVANCED_REASON_CONFIG_INVALID: u32 = 1 << 13;
pub const FOC_ADVANCED_REASON_CAPABILITY_MISSING: u32 = 1 << 14;
pub const FOC_ADVANCED_REASON_TARGET_UNSUPPORTED: u32 = 1 << 15;
pub const FOC_ADVANCED_REASON_CONTROLLER_STATE: u32 = 1 << 16;
pub const FOC_ADVANCED_REASON_KNOWN_MASK: u32 = FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE
    | FOC_ADVANCED_REASON_INVALID_RUNTIME_INPUT
    | FOC_ADVANCED_REASON_SUPERVISOR_STATE
    | FOC_ADVANCED_REASON_OUTPUT_INVALID
    | FOC_ADVANCED_REASON_CONFIG_IDENTITY
    | FOC_ADVANCED_REASON_CONFIG_INVALID
    | FOC_ADVANCED_REASON_CAPABILITY_MISSING
    | FOC_ADVANCED_REASON_TARGET_UNSUPPORTED
    | FOC_ADVANCED_REASON_CONTROLLER_STATE;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FocAdvancedRuntimeConfig {
    pub struct_size: u32,
    pub abi_version: u32,
    pub platform_capabilities: u32,
    pub reserved: u32,
    pub algorithm: AdvancedFocConfig,
}

impl Default for FocAdvancedRuntimeConfig {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_ADVANCED_ABI_VERSION,
            platform_capabilities: 0,
            reserved: 0,
            algorithm: AdvancedFocConfig::default(),
        }
    }
}

impl FocAdvancedRuntimeConfig {
    pub(crate) fn validation_failure_reason(
        &self,
        motor: &MotorParameters,
        control_frequency_hz: u32,
    ) -> u32 {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_ADVANCED_ABI_VERSION
            || self.reserved != 0
        {
            return FOC_ADVANCED_REASON_CONFIG_IDENTITY;
        }
        if self.platform_capabilities & !FOC_ADVANCED_CAPABILITY_KNOWN_MASK != 0
            || self.algorithm.enabled_features & !ADV_FOC_KNOWN_MASK != 0
        {
            return FOC_ADVANCED_REASON_CONFIG_INVALID;
        }
        if self.algorithm.enabled_features & !FOC_ADVANCED_TARGET_RUNTIME_SUPPORTED_MASK != 0 {
            return FOC_ADVANCED_REASON_TARGET_UNSUPPORTED;
        }
        let features = self.algorithm.enabled_features;
        if (features & ADV_FOC_DPWM != 0
            && self.platform_capabilities & FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION
                == 0)
            || (features & ADV_FOC_OVERMODULATION != 0
                && self.platform_capabilities & FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE == 0)
            || (features & ADV_FOC_HFI != 0
                && self.platform_capabilities & FOC_ADVANCED_CAPABILITY_HFI_INJECTION == 0)
            || (features & ADV_FOC_FLYING_START != 0
                && self.platform_capabilities & FOC_ADVANCED_CAPABILITY_PASSIVE_FLYING_START == 0)
        {
            return FOC_ADVANCED_REASON_CAPABILITY_MISSING;
        }
        if self
            .algorithm
            .validate(motor, control_frequency_hz)
            .is_err()
        {
            return FOC_ADVANCED_REASON_CONFIG_INVALID;
        }
        0
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocAdvancedTelemetry {
    pub struct_size: u32,
    pub abi_version: u32,
    pub active_features: u32,
    pub region: u32,
    pub modulation_mode: u32,
    pub status_flags: u32,
    pub flying_start_state: u32,
    pub id_reference_a: f32,
    pub iq_reference_a: f32,
    pub vd_feedforward_v: f32,
    pub vq_feedforward_v: f32,
    pub injection_alpha_v: f32,
    pub injection_beta_v: f32,
    pub voltage_limit_v: f32,
    pub hfi_angle_candidate_rad: f32,
    pub flying_start_angle_rad: f32,
    pub flying_start_speed_rad_s: f32,
}

impl FocAdvancedTelemetry {
    pub(crate) fn disabled() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_ADVANCED_ABI_VERSION,
            region: AdvancedFocRegion::Basic as u32,
            modulation_mode: AdvancedModulationMode::Svpwm as u32,
            flying_start_state: FlyingStartState::Idle as u32,
            ..Self::default()
        }
    }

    pub(crate) fn from_output(output: AdvancedFocOutput) -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_ADVANCED_ABI_VERSION,
            active_features: output.active_features,
            region: output.region as u32,
            modulation_mode: output.modulation_mode as u32,
            status_flags: FOC_ADVANCED_STATUS_CONFIGURED
                | if output.current_limited {
                    FOC_ADVANCED_STATUS_CURRENT_LIMITED
                } else {
                    0
                }
                | if output.hfi_angle_candidate_valid {
                    FOC_ADVANCED_STATUS_HFI_ANGLE_VALID
                } else {
                    0
                },
            flying_start_state: output.flying_start_state as u32,
            id_reference_a: output.current_reference.id_ref_a,
            iq_reference_a: output.current_reference.iq_ref_a,
            vd_feedforward_v: output.voltage_feedforward_dq.d,
            vq_feedforward_v: output.voltage_feedforward_dq.q,
            injection_alpha_v: output.injection_voltage_alpha_beta.alpha,
            injection_beta_v: output.injection_voltage_alpha_beta.beta,
            voltage_limit_v: output.voltage_limit_v,
            hfi_angle_candidate_rad: output.hfi_angle_candidate_rad,
            flying_start_angle_rad: output.flying_start_angle_rad,
            flying_start_speed_rad_s: output.flying_start_speed_rad_s,
        }
    }

    pub(crate) fn basic_fallback(
        reason: u32,
        reference: CurrentCommand,
        voltage_limit_v: f32,
    ) -> Self {
        let mut telemetry = Self::disabled();
        telemetry.status_flags = FOC_ADVANCED_STATUS_CONFIGURED
            | FOC_ADVANCED_STATUS_BASIC_FALLBACK
            | sanitize_reason(reason);
        telemetry.id_reference_a = reference.id_ref_a;
        telemetry.iq_reference_a = reference.iq_ref_a;
        telemetry.voltage_limit_v = voltage_limit_v;
        telemetry
    }

    pub(crate) fn faulted(reason: u32) -> Self {
        let mut telemetry = Self::disabled();
        telemetry.status_flags =
            FOC_ADVANCED_STATUS_CONFIGURED | FOC_ADVANCED_STATUS_FAULTED | sanitize_reason(reason);
        telemetry
    }

    pub(crate) fn config_rejected(reason: u32) -> Self {
        let mut telemetry = Self::disabled();
        telemetry.status_flags = FOC_ADVANCED_STATUS_CONFIG_REJECTED | sanitize_reason(reason);
        telemetry
    }
}

pub(crate) fn runtime_error_reason(error: AdvancedFocError) -> u32 {
    match error {
        AdvancedFocError::InvalidInput => FOC_ADVANCED_REASON_INVALID_RUNTIME_INPUT,
        AdvancedFocError::NotConfigured => FOC_ADVANCED_REASON_SUPERVISOR_STATE,
        _ => FOC_ADVANCED_REASON_SUPERVISOR_STATE,
    }
}

fn sanitize_reason(reason: u32) -> u32 {
    let reason = reason & FOC_ADVANCED_REASON_KNOWN_MASK;
    if reason == 0 {
        FOC_ADVANCED_REASON_SUPERVISOR_STATE
    } else {
        reason
    }
}

pub(crate) fn default_advanced_runtime_config(
    motor: MotorParameters,
    control_frequency_hz: u32,
) -> FocAdvancedRuntimeConfig {
    let current_limit = motor.rated_current_a;
    let maximum_electrical_speed =
        motor.max_speed_rpm * motor.pole_pairs as f32 * core::f32::consts::PI / 30.0;
    FocAdvancedRuntimeConfig {
        algorithm: AdvancedFocConfig {
            struct_size: size_of::<AdvancedFocConfig>() as u32,
            version: ADVANCED_FOC_CONFIG_VERSION,
            enabled_features: 0,
            region_update_divider: (control_frequency_hz / 1_000).max(1),
            current_limit_a: current_limit,
            mtpa_min_current_a: current_limit * 0.05,
            mtpa_search_steps: 24,
            weakening_entry_utilization: 0.92,
            weakening_exit_utilization: 0.86,
            weakening_kp_a_per_v: 0.1,
            weakening_id_min_a: -current_limit,
            weakening_slew_a_per_s: current_limit * 20.0,
            mtpv_entry_electrical_speed_rad_s: maximum_electrical_speed * 0.85,
            mtpv_exit_electrical_speed_rad_s: maximum_electrical_speed * 0.80,
            mtpv_search_steps: 24,
            decoupling_gain: 1.0,
            dpwm_entry_modulation: 0.90,
            dpwm_exit_modulation: 0.85,
            dpwm_mode: 0,
            overmodulation_max_voltage_ratio: 0.62,
            overmodulation_entry_modulation: 1.0,
            overmodulation_exit_modulation: 0.97,
            hfi_amplitude_v: motor.nominal_bus_voltage_v * 0.03,
            hfi_frequency_hz: (control_frequency_hz as f32 * 0.083_333_336).max(1.0),
            hfi_demod_alpha: 0.1,
            hfi_high_pass_alpha: 0.9,
            hfi_min_response_a: current_limit * 0.005,
            hfi_max_electrical_speed_rad_s: maximum_electrical_speed * 0.03,
            hfi_settling_samples: (control_frequency_hz / 100).max(1),
            flying_start_min_electrical_speed_rad_s: maximum_electrical_speed * 0.02,
            flying_start_stable_samples: (control_frequency_hz / 100).max(1),
            flying_start_timeout_samples: control_frequency_hz,
        },
        ..FocAdvancedRuntimeConfig::default()
    }
}

const _: () = assert!(size_of::<AdvancedFocConfig>() == 128);
const _: () = assert!(size_of::<FocAdvancedRuntimeConfig>() == 144);
const _: () = assert!(size_of::<FocAdvancedTelemetry>() == 68);

#[cfg(test)]
mod tests {
    use super::*;
    use foc_control::st_gbm2804_reference_parameters;

    #[test]
    fn disabled_default_is_valid_and_hardware_features_need_capabilities() {
        let params = st_gbm2804_reference_parameters();
        let mut config = default_advanced_runtime_config(params.motor, params.pwm_frequency_hz);
        assert_eq!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            0
        );
        config.algorithm.enabled_features = ADV_FOC_DPWM;
        assert_ne!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            0
        );
        config.platform_capabilities = FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION;
        assert_eq!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            0
        );
    }

    #[test]
    fn unsupported_target_request_features_fail_closed() {
        let params = st_gbm2804_reference_parameters();
        let mut config = default_advanced_runtime_config(params.motor, params.pwm_frequency_hz);
        config.algorithm.enabled_features = ADV_FOC_FLYING_START;
        config.platform_capabilities = FOC_ADVANCED_CAPABILITY_PASSIVE_FLYING_START;
        assert_eq!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            FOC_ADVANCED_REASON_TARGET_UNSUPPORTED
        );
    }

    #[test]
    fn validation_reasons_and_safety_telemetry_are_stable_bits() {
        let params = st_gbm2804_reference_parameters();
        let mut config = default_advanced_runtime_config(params.motor, params.pwm_frequency_hz);
        config.algorithm.enabled_features = ADV_FOC_DPWM;
        assert_eq!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            FOC_ADVANCED_REASON_CAPABILITY_MISSING
        );
        config.algorithm.enabled_features = ADV_FOC_DECOUPLING;
        config.algorithm.decoupling_gain = f32::NAN;
        assert_eq!(
            config.validation_failure_reason(&params.motor, params.pwm_frequency_hz),
            FOC_ADVANCED_REASON_CONFIG_INVALID
        );

        let fallback = FocAdvancedTelemetry::basic_fallback(
            FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE,
            CurrentCommand {
                id_ref_a: 0.1,
                iq_ref_a: 0.2,
            },
            6.0,
        );
        assert_eq!(fallback.active_features, 0);
        assert_ne!(
            fallback.status_flags & FOC_ADVANCED_STATUS_BASIC_FALLBACK,
            0
        );
        assert_ne!(
            fallback.status_flags & FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE,
            0
        );
        assert_eq!(size_of::<FocAdvancedTelemetry>(), 68);
    }
}
