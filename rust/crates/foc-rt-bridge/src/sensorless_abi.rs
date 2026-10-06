//! Versioned, fixed-layout target ABI for the full-speed sensorless angle chain.
//!
//! This context is deliberately independent from the main controller context.
//! The C platform owns PWM/ADC timing and hardware shutdown; this module owns
//! only signal conditioning, HFI/polarity/fusion state and the one-tick request
//! ledger.  Compiling the ABI does not grant a board any injection capability.

use core::mem::{align_of, size_of};
use core::ptr;

use foc_algorithm::{svpwm_update, wrap_angle_minus_pi_to_pi, AlphaBeta, SvpwmParam};
use foc_control::{
    ControlMath, HfiPolarityConfig, SensorlessAngleChain, SensorlessChainConfig,
    SensorlessChainFailure, SensorlessChainInput, SensorlessChainOutput, SensorlessFusionConfig,
};

use crate::PlatformMath;

pub const FOC_SENSORLESS_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_SENSORLESS_CONFIG_VERSION: u32 = 1;
pub const FOC_SENSORLESS_GUARD_VERSION: u32 = 1;
pub const FOC_SENSORLESS_INPUT_VERSION: u32 = 1;
/// V3 appends `chain_cycles`, a measurement-only field.  Frames that do not
/// request timing carry 0, so V1/V2 consumers keep their offsets and meaning.
pub const FOC_SENSORLESS_OUTPUT_VERSION: u32 = 3;
pub const FOC_SENSORLESS_VOLTAGE_INPUT_VERSION: u32 = 1;
pub const FOC_SENSORLESS_VOLTAGE_OUTPUT_VERSION: u32 = 1;
pub const FOC_SENSORLESS_COMPOSITE_INPUT_VERSION: u32 = 1;
/// V2 adds `chain_status`.  The combined entry collapses every sensorless-chain
/// rejection into `FocStatus::HardwareFault`, which made the rejecting check
/// indistinguishable on hardware; the chain's own status now travels out with
/// the frame that failed.  The field is appended so V1 readers keep their
/// offsets.
pub const FOC_SENSORLESS_COMPOSITE_OUTPUT_VERSION: u32 = 2;
pub const FOC_SENSORLESS_CONTEXT_CAPACITY: usize = 512;

pub const FOC_SENSORLESS_CAP_SYNCHRONIZED_CURRENT_SAMPLE: u32 = 1 << 0;
pub const FOC_SENSORLESS_CAP_CALIBRATED_ALPHA_BETA_CURRENT: u32 = 1 << 1;
pub const FOC_SENSORLESS_CAP_NEXT_PWM_VOLTAGE_INJECTION: u32 = 1 << 2;
pub const FOC_SENSORLESS_CAP_FINAL_VECTOR_LIMIT: u32 = 1 << 3;
pub const FOC_SENSORLESS_CAP_HARDWARE_FAST_SHUTDOWN: u32 = 1 << 4;
pub const FOC_SENSORLESS_CAP_BEMF_CHANNEL: u32 = 1 << 5;
pub const FOC_SENSORLESS_CAP_KNOWN_MASK: u32 = FOC_SENSORLESS_CAP_SYNCHRONIZED_CURRENT_SAMPLE
    | FOC_SENSORLESS_CAP_CALIBRATED_ALPHA_BETA_CURRENT
    | FOC_SENSORLESS_CAP_NEXT_PWM_VOLTAGE_INJECTION
    | FOC_SENSORLESS_CAP_FINAL_VECTOR_LIMIT
    | FOC_SENSORLESS_CAP_HARDWARE_FAST_SHUTDOWN
    | FOC_SENSORLESS_CAP_BEMF_CHANNEL;
pub const FOC_SENSORLESS_REQUIRED_CAPABILITIES: u32 = FOC_SENSORLESS_CAP_KNOWN_MASK;

pub const FOC_SENSORLESS_INPUT_INJECTION_PERMITTED: u32 = 1 << 0;
pub const FOC_SENSORLESS_INPUT_BEMF_VALID: u32 = 1 << 1;
pub const FOC_SENSORLESS_INPUT_RESET: u32 = 1 << 2;
/// The final vector circle scaled the preceding request before PWM preload.
pub const FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED: u32 = 1 << 3;
pub const FOC_SENSORLESS_INPUT_KNOWN_MASK: u32 = FOC_SENSORLESS_INPUT_INJECTION_PERMITTED
    | FOC_SENSORLESS_INPUT_BEMF_VALID
    | FOC_SENSORLESS_INPUT_RESET
    | FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED;

/// Accepted value of the composite input's `reserved` word.  V1 required zero;
/// V2 accepts zero (no timing) or this sentinel, which asks the combined entry
/// to attribute the frame by filling `chain_cycles`.  It lives in `reserved`
/// rather than the flag mask so the already-validated V1 callers keep working
/// unchanged and the flag namespace stays intact.
pub const FOC_SENSORLESS_COMPOSITE_TIMING_NONE: u32 = 0;
pub const FOC_SENSORLESS_COMPOSITE_TIMING_REQUESTED: u32 = 0x5449_4D45; /* "TIME" */

pub const FOC_SENSORLESS_OUTPUT_CONFIGURED: u32 = 1 << 0;
pub const FOC_SENSORLESS_OUTPUT_ENABLED: u32 = 1 << 1;
pub const FOC_SENSORLESS_OUTPUT_INJECTION_REQUESTED: u32 = 1 << 2;
pub const FOC_SENSORLESS_OUTPUT_ANGLE_RELIABLE: u32 = 1 << 3;
pub const FOC_SENSORLESS_OUTPUT_FALLBACK_REQUIRED: u32 = 1 << 4;
pub const FOC_SENSORLESS_OUTPUT_FAULT_LATCHED: u32 = 1 << 5;
pub const FOC_SENSORLESS_VOLTAGE_OUTPUT_LIMITED: u32 = 1 << 0;
pub const FOC_SENSORLESS_COMPOSITE_OUTPUT_INJECTION_LIMITED: u32 = 1 << 0;

const SENSORLESS_CONTEXT_MAGIC: u32 = 0x4653_4C53; // "FSLS"

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FocSensorlessStatus {
    Ok = 0,
    InvalidArgument = 1,
    NotInitialized = 2,
    NotConfigured = 3,
    UnsafeConfigurationState = 4,
    CapabilityMissing = 5,
    InvalidConfiguration = 6,
    SequenceMismatch = 7,
    AppliedRequestMismatch = 8,
    ChainFailure = 9,
    FaultLatched = 10,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessHfiConfig {
    pub struct_size: u32,
    pub version: u32,
    pub amplitude_v: f32,
    pub frequency_hz: f32,
    pub sample_period_s: f32,
    pub demod_alpha: f32,
    pub phase_offset_rad: f32,
    pub minimum_response_a: f32,
    pub maximum_electrical_speed_rad_s: f32,
    pub axis_stable_samples: u32,
    /// Fundamental-current LPF coefficient. HF current is `measured - LPF`.
    pub current_low_pass_alpha: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessPolarityConfig {
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessFusionConfig {
    pub struct_size: u32,
    pub version: u32,
    pub blend_enter_speed_rad_s: f32,
    pub hfi_reenter_speed_rad_s: f32,
    pub bemf_enter_speed_rad_s: f32,
    pub bemf_exit_speed_rad_s: f32,
    pub maximum_angle_disagreement_rad: f32,
    pub weight_slew_per_sample: f32,
    pub stable_samples: u32,
    pub invalid_timeout_samples: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessRuntimeConfig {
    pub struct_size: u32,
    pub abi_version: u32,
    pub config_version: u32,
    pub enabled: u32,
    pub reserved: u32,
    pub hfi: FocSensorlessHfiConfig,
    pub polarity: FocSensorlessPolarityConfig,
    pub fusion: FocSensorlessFusionConfig,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FocSensorlessConfigureGuard {
    pub struct_size: u32,
    pub version: u32,
    pub controller_stopped: u32,
    pub outputs_disabled: u32,
    pub no_faults: u32,
    pub platform_capabilities: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessRealtimeInput {
    pub struct_size: u32,
    pub version: u32,
    /// Sequence of the ADC sample being consumed now.
    pub sample_sequence: u32,
    /// Sequence of the voltage request that produced this ADC sample.
    pub applied_request_sequence: u32,
    pub platform_capabilities: u32,
    pub input_flags: u32,
    pub measured_current_alpha_a: f32,
    pub measured_current_beta_a: f32,
    pub applied_injection_alpha_v: f32,
    pub applied_injection_beta_v: f32,
    pub bemf_angle_rad: f32,
    pub bemf_electrical_speed_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessRealtimeOutput {
    pub struct_size: u32,
    pub version: u32,
    pub sample_sequence: u32,
    /// The returned voltage may only be applied by PWM sequence `N + 1`.
    pub request_apply_sequence: u32,
    pub status_flags: u32,
    pub stage: u32,
    pub failure: u32,
    pub polarity_failure: u32,
    pub angle_source: u32,
    pub angle_reliable: u32,
    pub fallback_required: u32,
    /// Fused electrical speed. HFI/Blend use a wrapped-angle derivative;
    /// reliable BEMF may seed it directly. Consumers divide by pole pairs for
    /// the mechanical speed loop.
    pub electrical_speed_rad_s: f32,
    pub injection_alpha_v: f32,
    pub injection_beta_v: f32,
    pub electrical_angle_rad: f32,
    pub hfi_angle_mod_pi_rad: f32,
    pub hfi_response_a: f32,
    pub bemf_weight: f32,
    pub angle_disagreement_rad: f32,
    pub high_frequency_current_alpha_a: f32,
    pub high_frequency_current_beta_a: f32,
    /// V3 measurement field.  Cycles the chain step itself consumed, or 0 when
    /// the caller did not request timing.  The combined entry sets it so a
    /// measured frame can be split into chain cost and core cost without
    /// instrumenting every stage; the platform measures the frame as a whole.
    pub chain_cycles: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessVoltageInput {
    pub struct_size: u32,
    pub version: u32,
    pub request_apply_sequence: u32,
    pub pwm_sequence: u32,
    pub base_voltage_alpha_v: f32,
    pub base_voltage_beta_v: f32,
    pub injection_alpha_v: f32,
    pub injection_beta_v: f32,
    pub dc_bus_voltage_v: f32,
    pub voltage_limit_v: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessVoltageOutput {
    pub struct_size: u32,
    pub version: u32,
    pub pwm_sequence: u32,
    pub status_flags: u32,
    pub final_voltage_alpha_v: f32,
    pub final_voltage_beta_v: f32,
    pub applied_injection_alpha_v: f32,
    pub applied_injection_beta_v: f32,
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
}

/// Sensorless-only extension consumed beside the normal V19 realtime input.
///
/// Phase currents, bus voltage, sample sequence and hardware fault flags are
/// deliberately not duplicated here: the combined entry derives them from the
/// same `FocRealtimeInput` that feeds the current PI.  The applied injection is
/// the previous combined output and therefore closes the N/N+1 ledger without
/// creating a second current-sample contract.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessCompositeInput {
    pub struct_size: u32,
    pub version: u32,
    pub platform_capabilities: u32,
    pub input_flags: u32,
    pub applied_request_sequence: u32,
    pub reserved: u32,
    pub applied_injection_alpha_v: f32,
    pub applied_injection_beta_v: f32,
    pub voltage_limit_v: f32,
    pub minimum_duty: f32,
    pub maximum_duty: f32,
}

/// Applied part of one combined basic-FOC + sensorless transaction.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocSensorlessCompositeOutput {
    pub struct_size: u32,
    pub version: u32,
    pub pwm_sequence: u32,
    pub status_flags: u32,
    pub applied_injection_alpha_v: f32,
    pub applied_injection_beta_v: f32,
    /// Status returned by the chain step this frame ran.  V2.  Zero on a frame
    /// that never reached the chain because an envelope or ledger check
    /// rejected the tick first.
    pub chain_status: u32,
}

#[repr(C, align(8))]
pub struct FocSensorlessContextStorage {
    pub bytes: [u8; FOC_SENSORLESS_CONTEXT_CAPACITY],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CurrentSeparator {
    initialized: bool,
    fundamental: AlphaBeta,
}

impl CurrentSeparator {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn update(&mut self, measured: AlphaBeta, alpha: f32) -> AlphaBeta {
        if !self.initialized {
            self.fundamental = measured;
            self.initialized = true;
            return AlphaBeta::default();
        }
        self.fundamental.alpha += alpha * (measured.alpha - self.fundamental.alpha);
        self.fundamental.beta += alpha * (measured.beta - self.fundamental.beta);
        AlphaBeta {
            alpha: measured.alpha - self.fundamental.alpha,
            beta: measured.beta - self.fundamental.beta,
        }
    }
}

struct SensorlessAbiContext {
    magic: u32,
    configured: bool,
    enabled: bool,
    fault_latched: bool,
    platform_capabilities: u32,
    current_low_pass_alpha: f32,
    config: SensorlessChainConfig,
    chain: SensorlessAngleChain,
    separator: CurrentSeparator,
    sequence_initialized: bool,
    last_sample_sequence: u32,
    expected_applied: AlphaBeta,
    fused_angle_initialized: bool,
    previous_fused_angle_rad: f32,
    fused_electrical_speed_rad_s: f32,
}

impl SensorlessAbiContext {
    fn new() -> Self {
        Self {
            magic: SENSORLESS_CONTEXT_MAGIC,
            configured: false,
            enabled: false,
            fault_latched: false,
            platform_capabilities: 0,
            current_low_pass_alpha: 0.0,
            config: SensorlessChainConfig::default(),
            chain: SensorlessAngleChain::default(),
            separator: CurrentSeparator::default(),
            sequence_initialized: false,
            last_sample_sequence: 0,
            expected_applied: AlphaBeta::default(),
            fused_angle_initialized: false,
            previous_fused_angle_rad: 0.0,
            fused_electrical_speed_rad_s: 0.0,
        }
    }
}

const _: () = assert!(size_of::<FocSensorlessHfiConfig>() == 44);
const _: () = assert!(size_of::<FocSensorlessPolarityConfig>() == 40);
const _: () = assert!(size_of::<FocSensorlessFusionConfig>() == 40);
const _: () = assert!(size_of::<FocSensorlessRuntimeConfig>() == 144);
const _: () = assert!(size_of::<FocSensorlessConfigureGuard>() == 28);
const _: () = assert!(size_of::<FocSensorlessRealtimeInput>() == 48);
const _: () = assert!(size_of::<FocSensorlessRealtimeOutput>() == 88);
const _: () = assert!(size_of::<SensorlessAbiContext>() <= FOC_SENSORLESS_CONTEXT_CAPACITY);
const _: () = assert!(size_of::<FocSensorlessVoltageInput>() == 48);
const _: () = assert!(size_of::<FocSensorlessVoltageOutput>() == 44);
const _: () = assert!(size_of::<FocSensorlessCompositeInput>() == 44);
const _: () = assert!(size_of::<FocSensorlessCompositeOutput>() == 28);
const _: () =
    assert!(align_of::<SensorlessAbiContext>() <= align_of::<FocSensorlessContextStorage>());

impl FocSensorlessRuntimeConfig {
    pub fn disabled_default() -> Self {
        let chain = SensorlessChainConfig::default();
        Self {
            struct_size: size_of::<Self>() as u32,
            abi_version: FOC_SENSORLESS_ABI_VERSION,
            config_version: FOC_SENSORLESS_CONFIG_VERSION,
            enabled: 0,
            reserved: 0,
            hfi: FocSensorlessHfiConfig {
                struct_size: size_of::<FocSensorlessHfiConfig>() as u32,
                version: 1,
                amplitude_v: chain.hfi.amplitude,
                frequency_hz: chain.hfi.freq_hz,
                sample_period_s: chain.hfi.ts,
                demod_alpha: chain.hfi.demod_alpha,
                phase_offset_rad: chain.hfi.phase_offset_rad,
                minimum_response_a: chain.minimum_hfi_response_a,
                maximum_electrical_speed_rad_s: chain.maximum_hfi_electrical_speed_rad_s,
                axis_stable_samples: chain.hfi_axis_stable_samples,
                current_low_pass_alpha: 0.15,
            },
            polarity: FocSensorlessPolarityConfig {
                struct_size: size_of::<FocSensorlessPolarityConfig>() as u32,
                version: chain.polarity.version,
                pulse_voltage_v: chain.polarity.pulse_voltage_v,
                pulse_ticks: chain.polarity.pulse_ticks,
                minimum_off_ticks: chain.polarity.minimum_off_ticks,
                settle_timeout_ticks: chain.polarity.settle_timeout_ticks,
                maximum_current_a: chain.polarity.maximum_current_a,
                maximum_residual_current_a: chain.polarity.maximum_residual_current_a,
                minimum_response_delta_a: chain.polarity.minimum_response_delta_a,
                maximum_pulse_pairs: chain.polarity.maximum_pulse_pairs,
            },
            fusion: FocSensorlessFusionConfig {
                struct_size: size_of::<FocSensorlessFusionConfig>() as u32,
                version: chain.fusion.version,
                blend_enter_speed_rad_s: chain.fusion.blend_enter_speed_rad_s,
                hfi_reenter_speed_rad_s: chain.fusion.hfi_reenter_speed_rad_s,
                bemf_enter_speed_rad_s: chain.fusion.bemf_enter_speed_rad_s,
                bemf_exit_speed_rad_s: chain.fusion.bemf_exit_speed_rad_s,
                maximum_angle_disagreement_rad: chain.fusion.maximum_angle_disagreement_rad,
                weight_slew_per_sample: chain.fusion.weight_slew_per_sample,
                stable_samples: chain.fusion.stable_samples,
                invalid_timeout_samples: chain.fusion.invalid_timeout_samples,
            },
        }
    }

    fn to_chain_config(self) -> Result<(SensorlessChainConfig, f32), FocSensorlessStatus> {
        if self.struct_size != size_of::<Self>() as u32
            || self.abi_version != FOC_SENSORLESS_ABI_VERSION
            || self.config_version != FOC_SENSORLESS_CONFIG_VERSION
            || self.enabled > 1
            || self.reserved != 0
            || self.hfi.struct_size != size_of::<FocSensorlessHfiConfig>() as u32
            || self.hfi.version != 1
            || self.polarity.struct_size != size_of::<FocSensorlessPolarityConfig>() as u32
            || self.fusion.struct_size != size_of::<FocSensorlessFusionConfig>() as u32
            || !self.hfi.current_low_pass_alpha.is_finite()
            || self.hfi.current_low_pass_alpha <= 0.0
            || self.hfi.current_low_pass_alpha > 1.0
        {
            return Err(FocSensorlessStatus::InvalidConfiguration);
        }
        let config = SensorlessChainConfig {
            hfi: foc_algorithm::RotatingHfSequenceParam {
                amplitude: self.hfi.amplitude_v,
                freq_hz: self.hfi.frequency_hz,
                ts: self.hfi.sample_period_s,
                demod_alpha: self.hfi.demod_alpha,
                phase_offset_rad: self.hfi.phase_offset_rad,
            },
            minimum_hfi_response_a: self.hfi.minimum_response_a,
            maximum_hfi_electrical_speed_rad_s: self.hfi.maximum_electrical_speed_rad_s,
            hfi_axis_stable_samples: self.hfi.axis_stable_samples,
            polarity: HfiPolarityConfig {
                struct_size: self.polarity.struct_size,
                version: self.polarity.version,
                pulse_voltage_v: self.polarity.pulse_voltage_v,
                pulse_ticks: self.polarity.pulse_ticks,
                minimum_off_ticks: self.polarity.minimum_off_ticks,
                settle_timeout_ticks: self.polarity.settle_timeout_ticks,
                maximum_current_a: self.polarity.maximum_current_a,
                maximum_residual_current_a: self.polarity.maximum_residual_current_a,
                minimum_response_delta_a: self.polarity.minimum_response_delta_a,
                maximum_pulse_pairs: self.polarity.maximum_pulse_pairs,
            },
            fusion: SensorlessFusionConfig {
                struct_size: self.fusion.struct_size,
                version: self.fusion.version,
                blend_enter_speed_rad_s: self.fusion.blend_enter_speed_rad_s,
                hfi_reenter_speed_rad_s: self.fusion.hfi_reenter_speed_rad_s,
                bemf_enter_speed_rad_s: self.fusion.bemf_enter_speed_rad_s,
                bemf_exit_speed_rad_s: self.fusion.bemf_exit_speed_rad_s,
                maximum_angle_disagreement_rad: self.fusion.maximum_angle_disagreement_rad,
                weight_slew_per_sample: self.fusion.weight_slew_per_sample,
                stable_samples: self.fusion.stable_samples,
                invalid_timeout_samples: self.fusion.invalid_timeout_samples,
            },
            ..SensorlessChainConfig::default()
        };
        if !config.is_valid() {
            return Err(FocSensorlessStatus::InvalidConfiguration);
        }
        Ok((config, self.hfi.current_low_pass_alpha))
    }
}

fn context_mut<'a>(
    storage: *mut FocSensorlessContextStorage,
) -> Result<&'a mut SensorlessAbiContext, FocSensorlessStatus> {
    if storage.is_null() || !(storage as usize).is_multiple_of(align_of::<SensorlessAbiContext>()) {
        return Err(FocSensorlessStatus::InvalidArgument);
    }
    let context = unsafe { &mut *storage.cast::<SensorlessAbiContext>() };
    if context.magic != SENSORLESS_CONTEXT_MAGIC {
        return Err(FocSensorlessStatus::NotInitialized);
    }
    Ok(context)
}

fn guard_valid(guard: &FocSensorlessConfigureGuard) -> bool {
    guard.struct_size == size_of::<FocSensorlessConfigureGuard>() as u32
        && guard.version == FOC_SENSORLESS_GUARD_VERSION
        && guard.controller_stopped == 1
        && guard.outputs_disabled == 1
        && guard.no_faults == 1
        && guard.reserved == 0
        && guard.platform_capabilities & !FOC_SENSORLESS_CAP_KNOWN_MASK == 0
}

fn input_valid(input: &FocSensorlessRealtimeInput) -> bool {
    input.struct_size == size_of::<FocSensorlessRealtimeInput>() as u32
        && input.version == FOC_SENSORLESS_INPUT_VERSION
        && input.input_flags & !FOC_SENSORLESS_INPUT_KNOWN_MASK == 0
        && input.platform_capabilities & !FOC_SENSORLESS_CAP_KNOWN_MASK == 0
        && input.measured_current_alpha_a.is_finite()
        && input.measured_current_beta_a.is_finite()
        && input.applied_injection_alpha_v.is_finite()
        && input.applied_injection_beta_v.is_finite()
        && input.bemf_angle_rad.is_finite()
        && input.bemf_electrical_speed_rad_s.is_finite()
}

fn prepare_output(output: &mut FocSensorlessRealtimeOutput, sequence: u32) {
    *output = FocSensorlessRealtimeOutput {
        struct_size: size_of::<FocSensorlessRealtimeOutput>() as u32,
        version: FOC_SENSORLESS_OUTPUT_VERSION,
        sample_sequence: sequence,
        request_apply_sequence: sequence.wrapping_add(1),
        ..FocSensorlessRealtimeOutput::default()
    };
}

fn applied_request_matches(expected: AlphaBeta, applied: AlphaBeta) -> Option<bool> {
    const EPSILON: f32 = 1.0e-5;
    let expected_magnitude_sq = expected.alpha * expected.alpha + expected.beta * expected.beta;
    let applied_magnitude_sq = applied.alpha * applied.alpha + applied.beta * applied.beta;
    if expected_magnitude_sq <= EPSILON * EPSILON {
        return if applied_magnitude_sq <= EPSILON * EPSILON {
            Some(false)
        } else {
            None
        };
    }
    let dot = expected.alpha * applied.alpha + expected.beta * applied.beta;
    let cross = expected.alpha * applied.beta - expected.beta * applied.alpha;
    let scale = dot / expected_magnitude_sq;
    let direction_tolerance = EPSILON * expected_magnitude_sq.max(1.0);
    if !scale.is_finite()
        || !(-EPSILON..=1.0 + EPSILON).contains(&scale)
        || cross.abs() > direction_tolerance
        || applied_magnitude_sq > expected_magnitude_sq * (1.0 + 4.0 * EPSILON)
    {
        return None;
    }
    Some(scale < 1.0 - EPSILON)
}

fn publish_chain_output(
    output: &mut FocSensorlessRealtimeOutput,
    chain: SensorlessChainOutput,
    high_frequency_current: AlphaBeta,
) {
    output.stage = chain.stage as u32;
    output.failure = chain.failure as u32;
    output.polarity_failure = chain.polarity_failure as u32;
    output.angle_source = chain.fusion.source as u32;
    output.angle_reliable = u32::from(chain.fusion.reliable);
    output.fallback_required = u32::from(chain.fusion.fallback_required);
    output.injection_alpha_v = chain.injection_voltage_alpha_beta.alpha;
    output.injection_beta_v = chain.injection_voltage_alpha_beta.beta;
    output.electrical_angle_rad = chain.fusion.angle_rad;
    output.hfi_angle_mod_pi_rad = chain.hfi_angle_mod_pi_rad;
    output.hfi_response_a = chain.hfi_response_a;
    output.bemf_weight = chain.fusion.bemf_weight;
    output.angle_disagreement_rad = chain.fusion.angle_disagreement_rad;
    output.high_frequency_current_alpha_a = high_frequency_current.alpha;
    output.high_frequency_current_beta_a = high_frequency_current.beta;
    output.status_flags |= if chain.injection_voltage_alpha_beta != AlphaBeta::default() {
        FOC_SENSORLESS_OUTPUT_INJECTION_REQUESTED
    } else {
        0
    } | if chain.fusion.reliable {
        FOC_SENSORLESS_OUTPUT_ANGLE_RELIABLE
    } else {
        0
    } | if chain.fusion.fallback_required {
        FOC_SENSORLESS_OUTPUT_FALLBACK_REQUIRED
    } else {
        0
    };
}

#[no_mangle]
pub extern "C" fn foc_rust_sensorless_abi_version() -> u32 {
    FOC_SENSORLESS_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_sensorless_required_capabilities() -> u32 {
    FOC_SENSORLESS_REQUIRED_CAPABILITIES
}

#[no_mangle]
/// Initializes an opaque sensorless ABI context in caller-owned storage.
///
/// # Safety
///
/// `storage` must be null or point to writable, correctly aligned storage of
/// `FocSensorlessContextStorage` size for the duration of this call.
pub unsafe extern "C" fn foc_rust_sensorless_init(
    storage: *mut FocSensorlessContextStorage,
) -> FocSensorlessStatus {
    if storage.is_null() || !(storage as usize).is_multiple_of(align_of::<SensorlessAbiContext>()) {
        return FocSensorlessStatus::InvalidArgument;
    }
    unsafe {
        ptr::write(
            storage.cast::<SensorlessAbiContext>(),
            SensorlessAbiContext::new(),
        )
    };
    FocSensorlessStatus::Ok
}

#[no_mangle]
/// Writes the disabled, versioned default configuration to caller storage.
///
/// # Safety
///
/// `output` must be null or point to a writable
/// `FocSensorlessRuntimeConfig` for the duration of this call.
pub unsafe extern "C" fn foc_rust_sensorless_default_config(
    output: *mut FocSensorlessRuntimeConfig,
) -> FocSensorlessStatus {
    if output.is_null() {
        return FocSensorlessStatus::InvalidArgument;
    }
    unsafe { ptr::write(output, FocSensorlessRuntimeConfig::disabled_default()) };
    FocSensorlessStatus::Ok
}

#[no_mangle]
/// Transactionally configures a previously initialized sensorless context.
///
/// # Safety
///
/// `storage` must satisfy [`foc_rust_sensorless_init`]'s storage contract and
/// remain exclusively owned by the caller. `config` and `guard` must be null
/// or point to readable values of their declared C ABI types for this call.
pub unsafe extern "C" fn foc_rust_sensorless_configure(
    storage: *mut FocSensorlessContextStorage,
    config: *const FocSensorlessRuntimeConfig,
    guard: *const FocSensorlessConfigureGuard,
) -> FocSensorlessStatus {
    let context = match context_mut(storage) {
        Ok(value) => value,
        Err(status) => return status,
    };
    let (Some(config), Some(guard)) = (unsafe { config.as_ref() }, unsafe { guard.as_ref() })
    else {
        return FocSensorlessStatus::InvalidArgument;
    };
    if !guard_valid(guard) {
        return FocSensorlessStatus::UnsafeConfigurationState;
    }
    let (chain_config, current_low_pass_alpha) = match (*config).to_chain_config() {
        Ok(value) => value,
        Err(status) => return status,
    };
    if config.enabled != 0
        && guard.platform_capabilities & FOC_SENSORLESS_REQUIRED_CAPABILITIES
            != FOC_SENSORLESS_REQUIRED_CAPABILITIES
    {
        return FocSensorlessStatus::CapabilityMissing;
    }
    // Commit only after every identity, numeric, state and capability check has
    // passed. A rejected request leaves the prior configuration untouched.
    context.config = chain_config;
    context.current_low_pass_alpha = current_low_pass_alpha;
    context.platform_capabilities = guard.platform_capabilities;
    context.enabled = config.enabled != 0;
    context.configured = true;
    context.fault_latched = false;
    context.chain.reset();
    context.separator.reset();
    context.sequence_initialized = false;
    context.expected_applied = AlphaBeta::default();
    context.fused_angle_initialized = false;
    context.previous_fused_angle_rad = 0.0;
    context.fused_electrical_speed_rad_s = 0.0;
    FocSensorlessStatus::Ok
}

#[no_mangle]
/// Executes one ordered ADC-sample/sensorless-control transaction.
///
/// # Safety
///
/// `storage` must be an initialized, exclusively owned context. `input` must
/// be null or point to a readable `FocSensorlessRealtimeInput`; `output` must
/// be null or point to writable `FocSensorlessRealtimeOutput` storage. All
/// non-null pointers must remain valid and non-overlapping for this call.
pub unsafe extern "C" fn foc_rust_sensorless_step(
    storage: *mut FocSensorlessContextStorage,
    input: *const FocSensorlessRealtimeInput,
    output: *mut FocSensorlessRealtimeOutput,
) -> FocSensorlessStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocSensorlessStatus::InvalidArgument;
    };
    prepare_output(output, 0);
    let context = match context_mut(storage) {
        Ok(value) => value,
        Err(status) => return status,
    };
    let Some(input) = (unsafe { input.as_ref() }) else {
        return FocSensorlessStatus::InvalidArgument;
    };
    prepare_output(output, input.sample_sequence);
    if context.fault_latched {
        output.status_flags = FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
        return FocSensorlessStatus::FaultLatched;
    }
    if !input_valid(input) {
        return FocSensorlessStatus::InvalidArgument;
    }
    if !context.configured {
        return FocSensorlessStatus::NotConfigured;
    }
    output.status_flags = FOC_SENSORLESS_OUTPUT_CONFIGURED;
    if !context.enabled {
        return FocSensorlessStatus::Ok;
    }
    output.status_flags |= FOC_SENSORLESS_OUTPUT_ENABLED;
    if input.platform_capabilities != context.platform_capabilities
        || input.platform_capabilities & FOC_SENSORLESS_REQUIRED_CAPABILITIES
            != FOC_SENSORLESS_REQUIRED_CAPABILITIES
    {
        context.fault_latched = true;
        output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
        return FocSensorlessStatus::CapabilityMissing;
    }
    if context.sequence_initialized {
        if input.sample_sequence != context.last_sample_sequence.wrapping_add(1)
            || input.applied_request_sequence != input.sample_sequence
        {
            context.fault_latched = true;
            output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
            return FocSensorlessStatus::SequenceMismatch;
        }
    } else if input.applied_request_sequence != input.sample_sequence {
        return FocSensorlessStatus::SequenceMismatch;
    }
    let applied = AlphaBeta {
        alpha: input.applied_injection_alpha_v,
        beta: input.applied_injection_beta_v,
    };
    let Some(was_limited) = applied_request_matches(context.expected_applied, applied) else {
        context.fault_latched = true;
        output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
        return FocSensorlessStatus::AppliedRequestMismatch;
    };
    let limited_flag = input.input_flags & FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED != 0;
    if was_limited != limited_flag {
        context.fault_latched = true;
        output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
        return FocSensorlessStatus::AppliedRequestMismatch;
    }

    if input.input_flags & FOC_SENSORLESS_INPUT_RESET != 0 {
        context.chain.reset();
        context.separator.reset();
        context.fused_angle_initialized = false;
        context.previous_fused_angle_rad = 0.0;
        context.fused_electrical_speed_rad_s = 0.0;
    }
    let measured = AlphaBeta {
        alpha: input.measured_current_alpha_a,
        beta: input.measured_current_beta_a,
    };
    let high_frequency_current = context
        .separator
        .update(measured, context.current_low_pass_alpha);
    let chain = context.chain.step(
        &context.config,
        SensorlessChainInput {
            request_reset: input.input_flags & FOC_SENSORLESS_INPUT_RESET != 0,
            injection_permitted: input.input_flags & FOC_SENSORLESS_INPUT_INJECTION_PERMITTED != 0,
            high_frequency_current_alpha_beta: high_frequency_current,
            measured_current_alpha_beta: measured,
            bemf_angle_rad: input.bemf_angle_rad,
            bemf_valid: input.input_flags & FOC_SENSORLESS_INPUT_BEMF_VALID != 0,
            bemf_electrical_speed_rad_s: input.bemf_electrical_speed_rad_s,
        },
    );
    publish_chain_output(output, chain, high_frequency_current);
    if chain.failure != SensorlessChainFailure::None {
        context.fault_latched = true;
        output.injection_alpha_v = 0.0;
        output.injection_beta_v = 0.0;
        output.status_flags &= !FOC_SENSORLESS_OUTPUT_INJECTION_REQUESTED;
        output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
        context.expected_applied = AlphaBeta::default();
        return FocSensorlessStatus::ChainFailure;
    }
    if chain.fusion.reliable {
        let raw_speed = if context.fused_angle_initialized {
            wrap_angle_minus_pi_to_pi(chain.fusion.angle_rad - context.previous_fused_angle_rad)
                / context.config.hfi.ts
        } else if input.input_flags & FOC_SENSORLESS_INPUT_BEMF_VALID != 0 {
            input.bemf_electrical_speed_rad_s
        } else {
            0.0
        };
        if !raw_speed.is_finite() {
            context.fault_latched = true;
            output.status_flags |= FOC_SENSORLESS_OUTPUT_FAULT_LATCHED;
            return FocSensorlessStatus::ChainFailure;
        }
        if context.fused_angle_initialized {
            /* Fixed, bounded target-safe smoothing. A versioned tunable belongs
             * to the later motor-specific commissioning profile; keeping it
             * here avoids exposing an unfiltered angle derivative to the speed
             * loop before that profile exists. */
            context.fused_electrical_speed_rad_s +=
                0.2 * (raw_speed - context.fused_electrical_speed_rad_s);
        } else {
            context.fused_electrical_speed_rad_s = raw_speed;
        }
        context.previous_fused_angle_rad = chain.fusion.angle_rad;
        context.fused_angle_initialized = true;
        output.electrical_speed_rad_s = context.fused_electrical_speed_rad_s;
    } else {
        context.fused_angle_initialized = false;
        context.fused_electrical_speed_rad_s = 0.0;
        output.electrical_speed_rad_s = 0.0;
    }
    context.expected_applied = chain.injection_voltage_alpha_beta;
    context.last_sample_sequence = input.sample_sequence;
    context.sequence_initialized = true;
    FocSensorlessStatus::Ok
}

#[no_mangle]
/// Combines a basic stationary-frame voltage with one sensorless N+1 request,
/// applies the final voltage circle and returns bounded SVPWM duties.
///
/// This function is deliberately stateless: request sequencing belongs to the
/// sensorless context and hardware authorization belongs to the C platform.
/// It does not grant an injection capability or access PWM registers.
///
/// # Safety
///
/// `input` must be null or point to a readable `FocSensorlessVoltageInput`;
/// `output` must be null or point to writable, non-overlapping
/// `FocSensorlessVoltageOutput` storage for this call.
pub unsafe extern "C" fn foc_rust_sensorless_compose_voltage(
    input: *const FocSensorlessVoltageInput,
    output: *mut FocSensorlessVoltageOutput,
) -> FocSensorlessStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocSensorlessStatus::InvalidArgument;
    };
    *output = FocSensorlessVoltageOutput {
        struct_size: size_of::<FocSensorlessVoltageOutput>() as u32,
        version: FOC_SENSORLESS_VOLTAGE_OUTPUT_VERSION,
        ..FocSensorlessVoltageOutput::default()
    };
    let Some(input) = (unsafe { input.as_ref() }) else {
        return FocSensorlessStatus::InvalidArgument;
    };
    output.pwm_sequence = input.pwm_sequence;
    let finite = input.base_voltage_alpha_v.is_finite()
        && input.base_voltage_beta_v.is_finite()
        && input.injection_alpha_v.is_finite()
        && input.injection_beta_v.is_finite()
        && input.dc_bus_voltage_v.is_finite()
        && input.voltage_limit_v.is_finite()
        && input.minimum_duty.is_finite()
        && input.maximum_duty.is_finite();
    if input.struct_size != size_of::<FocSensorlessVoltageInput>() as u32
        || input.version != FOC_SENSORLESS_VOLTAGE_INPUT_VERSION
        || input.request_apply_sequence != input.pwm_sequence
        || !finite
        || input.dc_bus_voltage_v <= 0.0
        || input.voltage_limit_v <= 0.0
        || input.minimum_duty < 0.0
        || input.maximum_duty > 1.0
        || input.minimum_duty >= 0.5
        || input.maximum_duty <= 0.5
        || input.maximum_duty <= input.minimum_duty
    {
        return FocSensorlessStatus::InvalidArgument;
    }

    let window_scale = (2.0 * (0.5 - input.minimum_duty))
        .min(2.0 * (input.maximum_duty - 0.5))
        .min(1.0);
    let linear_limit = input.dc_bus_voltage_v * 0.577_350_26 * window_scale;
    let limit = input.voltage_limit_v.min(linear_limit);
    if !limit.is_finite() || limit <= 0.0 {
        return FocSensorlessStatus::InvalidArgument;
    }

    let requested_injection = AlphaBeta {
        alpha: input.injection_alpha_v,
        beta: input.injection_beta_v,
    };
    let mut final_voltage = AlphaBeta {
        alpha: input.base_voltage_alpha_v + requested_injection.alpha,
        beta: input.base_voltage_beta_v + requested_injection.beta,
    };
    let magnitude_sq =
        final_voltage.alpha * final_voltage.alpha + final_voltage.beta * final_voltage.beta;
    let mut scale = 1.0;
    if magnitude_sq > limit * limit && magnitude_sq > 0.0 {
        let mut math = PlatformMath::default();
        scale = limit / math.magnitude(final_voltage.alpha, final_voltage.beta);
        final_voltage.alpha *= scale;
        final_voltage.beta *= scale;
        output.status_flags |= FOC_SENSORLESS_VOLTAGE_OUTPUT_LIMITED;
    }
    let pwm = svpwm_update(
        final_voltage,
        &SvpwmParam {
            v_bus: input.dc_bus_voltage_v,
        },
    );
    let clamp = |value: f32| value.max(input.minimum_duty).min(input.maximum_duty);
    let duty_a = clamp(pwm.duty_a);
    let duty_b = clamp(pwm.duty_b);
    let duty_c = clamp(pwm.duty_c);
    if !duty_a.is_finite() || !duty_b.is_finite() || !duty_c.is_finite() {
        return FocSensorlessStatus::InvalidArgument;
    }
    output.final_voltage_alpha_v = final_voltage.alpha;
    output.final_voltage_beta_v = final_voltage.beta;
    output.applied_injection_alpha_v = requested_injection.alpha * scale;
    output.applied_injection_beta_v = requested_injection.beta * scale;
    output.duty_a = duty_a;
    output.duty_b = duty_b;
    output.duty_c = duty_c;
    FocSensorlessStatus::Ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::MaybeUninit;

    fn storage() -> FocSensorlessContextStorage {
        unsafe { MaybeUninit::<FocSensorlessContextStorage>::zeroed().assume_init() }
    }

    fn safe_guard(capabilities: u32) -> FocSensorlessConfigureGuard {
        FocSensorlessConfigureGuard {
            struct_size: size_of::<FocSensorlessConfigureGuard>() as u32,
            version: FOC_SENSORLESS_GUARD_VERSION,
            controller_stopped: 1,
            outputs_disabled: 1,
            no_faults: 1,
            platform_capabilities: capabilities,
            reserved: 0,
        }
    }

    fn input(sequence: u32, applied: AlphaBeta) -> FocSensorlessRealtimeInput {
        FocSensorlessRealtimeInput {
            struct_size: size_of::<FocSensorlessRealtimeInput>() as u32,
            version: FOC_SENSORLESS_INPUT_VERSION,
            sample_sequence: sequence,
            applied_request_sequence: sequence,
            platform_capabilities: FOC_SENSORLESS_REQUIRED_CAPABILITIES,
            input_flags: FOC_SENSORLESS_INPUT_INJECTION_PERMITTED,
            measured_current_alpha_a: 0.1,
            measured_current_beta_a: -0.1,
            applied_injection_alpha_v: applied.alpha,
            applied_injection_beta_v: applied.beta,
            bemf_angle_rad: 0.0,
            bemf_electrical_speed_rad_s: 0.0,
        }
    }

    #[test]
    fn default_is_disabled_and_enable_requires_all_platform_capabilities() {
        let mut storage = storage();
        assert_eq!(
            unsafe { foc_rust_sensorless_init(&mut storage) },
            FocSensorlessStatus::Ok
        );
        let mut config = FocSensorlessRuntimeConfig::disabled_default();
        config.enabled = 1;
        let missing = safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES & !(1 << 2));
        assert_eq!(
            unsafe { foc_rust_sensorless_configure(&mut storage, &config, &missing) },
            FocSensorlessStatus::CapabilityMissing
        );
        let all = safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES);
        assert_eq!(
            unsafe { foc_rust_sensorless_configure(&mut storage, &config, &all) },
            FocSensorlessStatus::Ok
        );
    }

    #[test]
    fn configuration_is_stopped_state_transactional_and_rejects_unknown_layout() {
        let mut storage = storage();
        unsafe { foc_rust_sensorless_init(&mut storage) };
        let mut config = FocSensorlessRuntimeConfig::disabled_default();
        config.enabled = 1;
        let mut unsafe_guard = safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES);
        unsafe_guard.outputs_disabled = 0;
        assert_eq!(
            unsafe { foc_rust_sensorless_configure(&mut storage, &config, &unsafe_guard) },
            FocSensorlessStatus::UnsafeConfigurationState
        );
        let mut invalid = config;
        invalid.reserved = 1;
        assert_eq!(
            unsafe {
                foc_rust_sensorless_configure(
                    &mut storage,
                    &invalid,
                    &safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES),
                )
            },
            FocSensorlessStatus::InvalidConfiguration
        );
    }

    #[test]
    fn adc_sample_consumes_applied_n_and_requests_pwm_n_plus_one() {
        let mut storage = storage();
        unsafe { foc_rust_sensorless_init(&mut storage) };
        let mut config = FocSensorlessRuntimeConfig::disabled_default();
        config.enabled = 1;
        config.hfi.axis_stable_samples = 2;
        unsafe {
            foc_rust_sensorless_configure(
                &mut storage,
                &config,
                &safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES),
            )
        };
        let mut output = FocSensorlessRealtimeOutput::default();
        assert_eq!(
            unsafe {
                foc_rust_sensorless_step(
                    &mut storage,
                    &input(41, AlphaBeta::default()),
                    &mut output,
                )
            },
            FocSensorlessStatus::Ok
        );
        assert_eq!(output.sample_sequence, 41);
        assert_eq!(output.request_apply_sequence, 42);
        let applied = AlphaBeta {
            alpha: output.injection_alpha_v,
            beta: output.injection_beta_v,
        };
        assert_eq!(
            unsafe { foc_rust_sensorless_step(&mut storage, &input(42, applied), &mut output) },
            FocSensorlessStatus::Ok
        );
        assert_eq!(output.request_apply_sequence, 43);
    }

    /// Reproduces the G431 mode 3 no-power composite probe input on the host.
    ///
    /// The board run rejects on its very first tick with FSLSO reporting
    /// `CONFIGURED | ENABLED | INJECTION_REQUESTED` and no fault-latched bit,
    /// which rules out CapabilityMissing, SequenceMismatch and the applied
    /// request checks (all of which latch).  That leaves the chain itself.
    /// This test pins the exact probe configuration -- axis acquisition held
    /// for the whole window, injection permitted, zero applied injection on the
    /// first tick and the real unpowered ADC readings -- so the returned status
    /// names the rejecting stage.
    #[test]
    fn mode3_probe_configuration_survives_axis_acquisition() {
        let mut storage = storage();
        unsafe { foc_rust_sensorless_init(&mut storage) };
        let mut config = FocSensorlessRuntimeConfig::disabled_default();
        config.enabled = 1;
        // The platform holds acquisition for the whole probe window.
        config.hfi.axis_stable_samples = u32::MAX;
        assert_eq!(
            unsafe {
                foc_rust_sensorless_configure(
                    &mut storage,
                    &config,
                    &safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES),
                )
            },
            FocSensorlessStatus::Ok
        );

        let mut output = FocSensorlessRealtimeOutput::default();
        // Tick 0: the real board reads an unpowered bus, so the currents are
        // the ~2048-count offset translated to amperes.  The applied injection
        // is zero because nothing has been requested yet.
        let mut tick_input = input(0, AlphaBeta::default());
        tick_input.input_flags = FOC_SENSORLESS_INPUT_INJECTION_PERMITTED;
        tick_input.measured_current_alpha_a = 0.0015;
        tick_input.measured_current_beta_a = 0.0013;
        let first = unsafe { foc_rust_sensorless_step(&mut storage, &tick_input, &mut output) };
        assert_eq!(
            first,
            FocSensorlessStatus::Ok,
            "mode 3 first tick rejected: status={:?} flags={:#010x} stage={} failure={}",
            first,
            output.status_flags,
            output.stage,
            output.failure
        );

        // Then run the ledger forward the way the platform does: each tick
        // applies the previous tick's request at the current sequence.
        let mut applied = AlphaBeta {
            alpha: output.injection_alpha_v,
            beta: output.injection_beta_v,
        };
        for sequence in 1..64u32 {
            let mut next = input(sequence, applied);
            next.input_flags = FOC_SENSORLESS_INPUT_INJECTION_PERMITTED;
            next.measured_current_alpha_a = 0.0015;
            next.measured_current_beta_a = 0.0013;
            let status = unsafe { foc_rust_sensorless_step(&mut storage, &next, &mut output) };
            assert_eq!(
                status,
                FocSensorlessStatus::Ok,
                "mode 3 tick {sequence} rejected: status={:?} flags={:#010x} stage={} failure={}",
                status,
                output.status_flags,
                output.stage,
                output.failure
            );
            assert_eq!(output.sample_sequence, sequence);
            assert_eq!(output.request_apply_sequence, sequence + 1);
            applied = AlphaBeta {
                alpha: output.injection_alpha_v,
                beta: output.injection_beta_v,
            };
        }
    }

    #[test]
    fn skipped_tick_or_unapplied_request_latches_fail_zero() {
        let mut storage = storage();
        unsafe { foc_rust_sensorless_init(&mut storage) };
        let mut config = FocSensorlessRuntimeConfig::disabled_default();
        config.enabled = 1;
        unsafe {
            foc_rust_sensorless_configure(
                &mut storage,
                &config,
                &safe_guard(FOC_SENSORLESS_REQUIRED_CAPABILITIES),
            )
        };
        let mut output = FocSensorlessRealtimeOutput::default();
        unsafe {
            foc_rust_sensorless_step(&mut storage, &input(7, AlphaBeta::default()), &mut output)
        };
        let bad = input(9, AlphaBeta::default());
        assert_eq!(
            unsafe { foc_rust_sensorless_step(&mut storage, &bad, &mut output) },
            FocSensorlessStatus::SequenceMismatch
        );
        assert_eq!(output.injection_alpha_v, 0.0);
        assert_eq!(output.injection_beta_v, 0.0);
        assert_ne!(output.status_flags & FOC_SENSORLESS_OUTPUT_FAULT_LATCHED, 0);
    }

    #[test]
    fn final_circle_may_scale_but_not_rotate_the_applied_injection() {
        let expected = AlphaBeta {
            alpha: 0.8,
            beta: -0.6,
        };
        assert_eq!(applied_request_matches(expected, expected), Some(false));
        assert_eq!(
            applied_request_matches(
                expected,
                AlphaBeta {
                    alpha: 0.4,
                    beta: -0.3,
                }
            ),
            Some(true)
        );
        assert_eq!(
            applied_request_matches(
                expected,
                AlphaBeta {
                    alpha: 0.6,
                    beta: 0.8,
                }
            ),
            None
        );
    }

    #[test]
    fn voltage_composer_applies_n_plus_one_request_and_final_circle() {
        let input = FocSensorlessVoltageInput {
            struct_size: size_of::<FocSensorlessVoltageInput>() as u32,
            version: FOC_SENSORLESS_VOLTAGE_INPUT_VERSION,
            request_apply_sequence: 42,
            pwm_sequence: 42,
            base_voltage_alpha_v: 3.0,
            base_voltage_beta_v: 0.0,
            injection_alpha_v: 4.0,
            injection_beta_v: 0.0,
            dc_bus_voltage_v: 12.0,
            voltage_limit_v: 5.0,
            minimum_duty: 0.03,
            maximum_duty: 0.97,
        };
        let mut output = FocSensorlessVoltageOutput::default();
        assert_eq!(
            unsafe { foc_rust_sensorless_compose_voltage(&input, &mut output) },
            FocSensorlessStatus::Ok
        );
        assert_eq!(output.pwm_sequence, 42);
        assert_ne!(
            output.status_flags & FOC_SENSORLESS_VOLTAGE_OUTPUT_LIMITED,
            0
        );
        assert!((output.final_voltage_alpha_v - 5.0).abs() < 1.0e-5);
        assert!(output.final_voltage_beta_v.abs() < 1.0e-6);
        assert!((output.applied_injection_alpha_v - 20.0 / 7.0).abs() < 1.0e-5);
        for duty in [output.duty_a, output.duty_b, output.duty_c] {
            assert!((0.03..=0.97).contains(&duty));
        }
    }

    #[test]
    fn voltage_composer_rejects_wrong_sequence_and_leaves_safe_output() {
        let input = FocSensorlessVoltageInput {
            struct_size: size_of::<FocSensorlessVoltageInput>() as u32,
            version: FOC_SENSORLESS_VOLTAGE_INPUT_VERSION,
            request_apply_sequence: 8,
            pwm_sequence: 9,
            dc_bus_voltage_v: 12.0,
            voltage_limit_v: 5.0,
            minimum_duty: 0.03,
            maximum_duty: 0.97,
            ..FocSensorlessVoltageInput::default()
        };
        let mut output = FocSensorlessVoltageOutput {
            duty_a: 0.8,
            duty_b: 0.8,
            duty_c: 0.8,
            ..FocSensorlessVoltageOutput::default()
        };
        assert_eq!(
            unsafe { foc_rust_sensorless_compose_voltage(&input, &mut output) },
            FocSensorlessStatus::InvalidArgument
        );
        assert_eq!(output.duty_a, 0.0);
        assert_eq!(output.duty_b, 0.0);
        assert_eq!(output.duty_c, 0.0);
    }
}
