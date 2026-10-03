//! PC reference for the Advanced-Lab no-power target probe.
//!
//! It deliberately hashes only discrete policy decisions, matching the target
//! `foc_advanced_wcet` command. Floating-point PWM values are not required to
//! be bit-identical between host CPU math and the STM32G4 CORDIC backend.

use core::mem::MaybeUninit;

use foc_control::{
    ADV_FOC_DECOUPLING, ADV_FOC_DPWM, ADV_FOC_FIELD_WEAKENING, ADV_FOC_MTPA, ADV_FOC_MTPV,
    ADV_FOC_OVERMODULATION,
};
use foc_rt_bridge::{
    foc_rust_configure, foc_rust_configure_advanced, foc_rust_default_advanced_config,
    foc_rust_default_st_config, foc_rust_init, foc_rust_realtime_step_advanced_no_power,
    foc_rust_start_realtime, FocAdvancedProbeInput, FocAdvancedRuntimeConfig, FocAdvancedTelemetry,
    FocFeedback, FocOutput, FocRealtimeInput, FocRuntimeConfig, FocRustContextStorage, FocStatus,
    FocTelemetry, FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION,
    FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE,
};

const DEFAULT_TICKS: u32 = 36_000;
const CONTROL_DT_S: f32 = 1.0 / 12_000.0;

fn mode_config(mode: u32) -> (u32, u32) {
    match mode {
        0 => (0, 0),
        1 => (ADV_FOC_MTPA, 0),
        2 => (ADV_FOC_FIELD_WEAKENING, 0),
        3 => (ADV_FOC_MTPV, 0),
        4 => (ADV_FOC_DECOUPLING, 0),
        5 => (
            ADV_FOC_DPWM,
            FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION,
        ),
        6 => (
            ADV_FOC_OVERMODULATION,
            FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE,
        ),
        7 => (
            ADV_FOC_MTPA
                | ADV_FOC_FIELD_WEAKENING
                | ADV_FOC_MTPV
                | ADV_FOC_DECOUPLING
                | ADV_FOC_DPWM
                | ADV_FOC_OVERMODULATION,
            FOC_ADVANCED_CAPABILITY_DPWM_CURRENT_RECONSTRUCTION
                | FOC_ADVANCED_CAPABILITY_OVERMOD_MIN_PULSE,
        ),
        _ => unreachable!("mode is bounded by the caller"),
    }
}

fn hash_decisions(signature: u32, telemetry: FocAdvancedTelemetry) -> u32 {
    [
        telemetry.active_features,
        telemetry.region,
        telemetry.modulation_mode,
        telemetry.status_flags,
    ]
    .into_iter()
    .fold(signature, |value, field| {
        (value ^ field).wrapping_mul(16_777_619)
    })
}

fn run_mode(mode: u32, ticks: u32) -> (u32, FocAdvancedTelemetry) {
    // The exported initializer is the only operation allowed to interpret this
    // opaque storage as a controller, so zeroed backing bytes are sufficient.
    let mut context = unsafe { MaybeUninit::<FocRustContextStorage>::zeroed().assume_init() };
    let mut runtime = FocRuntimeConfig::default();
    let mut advanced = FocAdvancedRuntimeConfig::default();
    let probe = FocAdvancedProbeInput {
        base_id_reference_a: 0.0,
        base_iq_reference_a: 0.6,
        electrical_angle_rad: 0.35,
        mechanical_speed_rad_s: 150.0,
        previous_vd_command_v: 0.0,
        previous_vq_command_v: 7.3,
        phase_current_a: 0.10,
        phase_current_b: -0.04,
        phase_current_c: -0.06,
        ..FocAdvancedProbeInput::default()
    };
    let feedback = FocFeedback {
        phase_current_a: probe.phase_current_a,
        phase_current_b: probe.phase_current_b,
        phase_current_c: probe.phase_current_c,
        dc_bus_voltage: 12.3,
        electrical_angle_rad: probe.electrical_angle_rad,
    };
    let mut output = FocOutput::default();
    let mut base_telemetry = FocTelemetry::default();
    let mut telemetry = FocAdvancedTelemetry::default();
    let mut signature = 2_166_136_261_u32;
    let (features, capabilities) = mode_config(mode);

    unsafe {
        assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
        assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
        assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
        assert_eq!(
            foc_rust_default_advanced_config(&mut context, &mut advanced),
            FocStatus::Ok
        );
        advanced.algorithm.enabled_features = features;
        advanced.algorithm.region_update_divider = 1;
        advanced.platform_capabilities = capabilities;
        assert_eq!(
            foc_rust_configure_advanced(&mut context, &advanced),
            FocStatus::Ok
        );
        assert_eq!(
            foc_rust_start_realtime(&mut context, 1, runtime.default_target_speed_rpm),
            FocStatus::Ok
        );

        for sequence in 0..ticks {
            let input =
                FocRealtimeInput::command_model_from_legacy(feedback, sequence, CONTROL_DT_S);
            assert_eq!(
                foc_rust_realtime_step_advanced_no_power(
                    &mut context,
                    &input,
                    &probe,
                    &mut output,
                    &mut base_telemetry,
                    &mut telemetry,
                ),
                FocStatus::Ok
            );
            signature = hash_decisions(signature, telemetry);
        }
    }
    (signature, telemetry)
}

fn main() {
    let ticks = std::env::args()
        .nth(1)
        .map(|value| value.parse::<u32>().expect("ticks must be an integer"))
        .unwrap_or(DEFAULT_TICKS);
    assert!(ticks > 0, "ticks must be positive");

    println!("mode,ticks,signature,active,status,region,modulation");
    for mode in 0..=7 {
        let (signature, telemetry) = run_mode(mode, ticks);
        println!(
            "{mode},{ticks},{signature:08x},{:02x},{:08x},{},{}",
            telemetry.active_features,
            telemetry.status_flags,
            telemetry.region,
            telemetry.modulation_mode,
        );
    }
}
