//! Independent realtime ABI for sensored FOC.
//!
//! This path is deliberately separate from the sensorless rev-up/observer ABI:
//! the board owns ADC/AS5600 acquisition and protection, while Rust exclusively
//! owns the speed loop, current loop, angle compensation and modulation.

use core::mem::{offset_of, size_of};

use foc_algorithm::{clarke, wrap_angle_0_to_2pi, Abc};
use foc_control::{
    ControlAngleOffsets, CurrentCommand, FeedbackSnapshot, PhaseCurrents, PwmCommand,
    RotorFeedback, SpeedCommand,
};

use crate::{
    controller_mut, move_towards, zero_output, FocOutput, FocRustContextStorage, FocState,
    FocStatus, FocTelemetry, PlatformMath, FOC_FAULT_ALGORITHM_OUTPUT, FOC_FAULT_INVALID_FEEDBACK,
    FOC_FAULT_PLATFORM_INPUT, FOC_REALTIME_HW_FAULT_KNOWN_MASK,
};

pub const FOC_ENCODER_REALTIME_ABI_VERSION: u32 = 0x0001_0000;
pub const FOC_ENCODER_REALTIME_INPUT_VERSION: u32 = 1;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FocEncoderRealtimeInput {
    pub struct_size: u32,
    pub version: u32,
    pub control_sequence: u32,
    pub hardware_fault_flags: u32,
    pub actual_dt_s: f32,
    pub phase_current_a: f32,
    pub phase_current_b: f32,
    pub phase_current_c: f32,
    pub dc_bus_voltage_v: f32,
    pub electrical_angle_rad: f32,
    pub mechanical_speed_rad_s: f32,
    pub reserved: u32,
}

const _: () = assert!(size_of::<FocEncoderRealtimeInput>() == 48);
const _: () = assert!(offset_of!(FocEncoderRealtimeInput, actual_dt_s) == 16);
const _: () = assert!(offset_of!(FocEncoderRealtimeInput, mechanical_speed_rad_s) == 40);

#[inline(always)]
fn input_envelope_is_valid(input: &FocEncoderRealtimeInput) -> bool {
    input.struct_size == size_of::<FocEncoderRealtimeInput>() as u32
        && input.version == FOC_ENCODER_REALTIME_INPUT_VERSION
        && input.hardware_fault_flags & !FOC_REALTIME_HW_FAULT_KNOWN_MASK == 0
}

#[inline(always)]
fn physical_input_is_valid(input: &FocEncoderRealtimeInput, expected_dt_s: f32) -> bool {
    let dt_error = (input.actual_dt_s - expected_dt_s).abs();
    input.actual_dt_s.is_finite()
        && dt_error <= expected_dt_s * 0.01
        && input.phase_current_a.is_finite()
        && input.phase_current_b.is_finite()
        && input.phase_current_c.is_finite()
        && input.dc_bus_voltage_v.is_finite()
        && input.dc_bus_voltage_v > 0.0
        && input.electrical_angle_rad.is_finite()
        && input.mechanical_speed_rad_s.is_finite()
}

#[inline(always)]
fn pwm_is_valid(pwm: PwmCommand) -> bool {
    pwm.duty_a.is_finite()
        && pwm.duty_b.is_finite()
        && pwm.duty_c.is_finite()
        && (0.0..=1.0).contains(&pwm.duty_a)
        && (0.0..=1.0).contains(&pwm.duty_b)
        && (0.0..=1.0).contains(&pwm.duty_c)
}

#[no_mangle]
pub extern "C" fn foc_rust_encoder_realtime_abi_version() -> u32 {
    FOC_ENCODER_REALTIME_ABI_VERSION
}

/// Starts the encoder-owned closed loop. No alignment or sensorless observer is
/// run: the caller must already provide a calibrated electrical angle and a
/// coherent mechanical-speed sample on every control tick.
///
/// # Safety
/// `context` must point to exclusively owned storage previously initialized by
/// `foc_rust_init`; no ISR or task may access it concurrently with this call.
#[no_mangle]
pub unsafe extern "C" fn foc_rust_start_encoder_realtime(
    context: *mut FocRustContextStorage,
    platform_ready: u32,
    target_speed_rpm: f32,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if controller.state != FocState::Disabled {
        return FocStatus::Disabled;
    }
    if !controller.algorithm_configured || platform_ready == 0 {
        return FocStatus::NotConfigured;
    }
    if !target_speed_rpm.is_finite()
        || target_speed_rpm.abs() > controller.params.motor.max_speed_rpm
    {
        return FocStatus::InvalidArgument;
    }

    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.previous_pwm = PwmCommand::default();
    controller.current_reference = CurrentCommand::default();
    controller.speed_current_command = CurrentCommand::default();
    controller.target_speed_rpm = target_speed_rpm;
    controller.speed_reference_rpm = target_speed_rpm;
    controller.speed_counter = 0;
    controller.last_control_sequence = u32::MAX;
    controller.closed_loop_initialized = true;
    controller.state = FocState::Running;
    controller.telemetry = FocTelemetry::default();
    controller.telemetry.state = controller.state as u32;
    controller.telemetry.closed_loop_active = 1;
    controller.telemetry.target_speed_rpm = target_speed_rpm;
    FocStatus::Ok
}

/// Runs one complete sensored speed/current FOC tick. The output is cleared
/// before any validation, and every physical/scheduler fault is sticky.
///
/// # Safety
/// `context` must be initialized and exclusively owned by the realtime caller;
/// `input` must be readable and `output` writable. Optional `telemetry` must be
/// writable when non-null, and none of the pointed-to objects may overlap.
#[no_mangle]
pub unsafe extern "C" fn foc_rust_encoder_realtime_step(
    context: *mut FocRustContextStorage,
    input: *const FocEncoderRealtimeInput,
    output: *mut FocOutput,
    telemetry: *mut FocTelemetry,
) -> FocStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(input) = (unsafe { input.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }
    if !input_envelope_is_valid(input) {
        return FocStatus::InvalidArgument;
    }

    let expected_dt_s = 1.0 / controller.params.pwm_frequency_hz as f32;
    let expected_sequence = controller.last_control_sequence.wrapping_add(1);
    if input.control_sequence != expected_sequence
        || input.hardware_fault_flags != 0
        || !physical_input_is_valid(input, expected_dt_s)
    {
        controller.fault_flags |=
            if input.hardware_fault_flags != 0 || input.control_sequence != expected_sequence {
                FOC_FAULT_PLATFORM_INPUT
            } else {
                FOC_FAULT_INVALID_FEEDBACK
            };
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }
    controller.last_control_sequence = input.control_sequence;

    let speed_divider =
        (controller.params.pwm_frequency_hz / controller.params.speed_loop_frequency_hz).max(1);
    if controller.speed_counter == 0 {
        controller.speed_current_command = controller.speed_loop.update(
            &controller.params,
            SpeedCommand {
                target_rpm: controller.target_speed_rpm,
                id_ref_a: 0.0,
            },
            input.mechanical_speed_rad_s,
        );
    }
    controller.speed_counter = (controller.speed_counter + 1) % speed_divider;

    let current_slew = controller.runtime_config.closed_loop_current_slew_a_per_s * expected_dt_s;
    controller.current_reference.id_ref_a = move_towards(
        controller.current_reference.id_ref_a,
        controller.speed_current_command.id_ref_a,
        current_slew,
    );
    controller.current_reference.iq_ref_a = move_towards(
        controller.current_reference.iq_ref_a,
        controller.speed_current_command.iq_ref_a,
        current_slew,
    );

    let feedback = FeedbackSnapshot {
        currents: PhaseCurrents {
            a: input.phase_current_a,
            b: input.phase_current_b,
            c: input.phase_current_c,
        },
        dc_bus_voltage: input.dc_bus_voltage_v,
        rotor: RotorFeedback {
            electrical_angle_rad: wrap_angle_0_to_2pi(input.electrical_angle_rad),
            mechanical_speed_rad_s: input.mechanical_speed_rad_s,
        },
    };
    let electrical_angle_per_tick =
        input.mechanical_speed_rad_s * controller.params.motor.pole_pairs as f32 * expected_dt_s;
    let offsets = ControlAngleOffsets {
        park_rad: electrical_angle_per_tick
            * controller
                .runtime_config
                .angle_compensation
                .park_prediction_ticks,
        reverse_park_rad: electrical_angle_per_tick
            * controller
                .runtime_config
                .angle_compensation
                .reverse_park_prediction_ticks,
    };
    let current_alpha_beta = clarke(Abc {
        a: input.phase_current_a,
        b: input.phase_current_b,
        c: input.phase_current_c,
    });
    let mut math = PlatformMath::default();
    let (pwm, control) = controller
        .current_loop
        .update_from_alpha_beta_with_angle_offsets_and_math(
            &controller.params,
            &feedback,
            current_alpha_beta,
            controller.current_reference,
            offsets,
            &mut math,
        );
    if !pwm_is_valid(pwm) {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    controller.previous_pwm = pwm;
    controller.telemetry = FocTelemetry {
        state: controller.state as u32,
        observer_backend: controller.observer.backend() as u32,
        observer_reliable: 0,
        closed_loop_active: 1,
        target_speed_rpm: controller.target_speed_rpm,
        measured_speed_rpm: control.measured_speed_rpm,
        electrical_angle_rad: feedback.rotor.electrical_angle_rad,
        id_reference_a: controller.current_reference.id_ref_a,
        iq_reference_a: controller.current_reference.iq_ref_a,
        id_measured_a: control.current_dq.d,
        iq_measured_a: control.current_dq.q,
        vd_command_v: control.voltage_dq.d,
        vq_command_v: control.voltage_dq.q,
        voltage_limited: control.voltage_limited as u32,
        ..FocTelemetry::default()
    };
    if let Some(telemetry) = unsafe { telemetry.as_mut() } {
        *telemetry = controller.telemetry;
    }
    FocStatus::Ok
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{foc_rust_configure, foc_rust_default_st_config, foc_rust_init, FocRuntimeConfig};

    fn configured_context() -> FocRustContextStorage {
        let mut context = FocRustContextStorage {
            bytes: [0; crate::FOC_RUST_CONTEXT_CAPACITY],
        };
        assert_eq!(unsafe { foc_rust_init(&mut context) }, FocStatus::Ok);
        let mut config = FocRuntimeConfig::default();
        assert_eq!(
            unsafe { foc_rust_default_st_config(&mut config) },
            FocStatus::Ok
        );
        config.pwm_frequency_hz = 2_000;
        config.speed_loop_frequency_hz = 200;
        assert_eq!(
            unsafe { foc_rust_configure(&mut context, &config) },
            FocStatus::Ok
        );
        context
    }

    fn zero_input(sequence: u32) -> FocEncoderRealtimeInput {
        FocEncoderRealtimeInput {
            struct_size: size_of::<FocEncoderRealtimeInput>() as u32,
            version: FOC_ENCODER_REALTIME_INPUT_VERSION,
            control_sequence: sequence,
            actual_dt_s: 1.0 / 2_000.0,
            dc_bus_voltage_v: 12.0,
            ..FocEncoderRealtimeInput::default()
        }
    }

    #[test]
    fn zero_speed_produces_neutral_valid_pwm() {
        let mut context = configured_context();
        assert_eq!(
            unsafe { foc_rust_start_encoder_realtime(&mut context, 1, 0.0) },
            FocStatus::Ok
        );
        let input = zero_input(0);
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        assert_eq!(
            unsafe {
                foc_rust_encoder_realtime_step(&mut context, &input, &mut output, &mut telemetry)
            },
            FocStatus::Ok
        );
        assert!((output.duty_a - 0.5).abs() < 1.0e-6);
        assert!((output.duty_b - 0.5).abs() < 1.0e-6);
        assert!((output.duty_c - 0.5).abs() < 1.0e-6);
        assert_eq!(telemetry.closed_loop_active, 1);
        assert_eq!(telemetry.observer_reliable, 0);
    }

    #[test]
    fn sequence_gap_faults_and_clears_output() {
        let mut context = configured_context();
        assert_eq!(
            unsafe { foc_rust_start_encoder_realtime(&mut context, 1, 100.0) },
            FocStatus::Ok
        );
        let input = zero_input(1);
        let mut output = FocOutput {
            duty_a: 0.5,
            duty_b: 0.5,
            duty_c: 0.5,
        };
        assert_eq!(
            unsafe {
                foc_rust_encoder_realtime_step(
                    &mut context,
                    &input,
                    &mut output,
                    core::ptr::null_mut(),
                )
            },
            FocStatus::HardwareFault
        );
        assert_eq!(output, FocOutput::default());
    }
}
