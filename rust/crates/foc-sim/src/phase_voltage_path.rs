//! A24.4 PC-only phase-voltage source-selection integration harness.
//!
//! This module deliberately does not enable the measured-voltage source in the
//! target bridge.  It drives the real V19 input gate to prove that selected
//! `Measured` data still returns `NotConfigured`, while a `Hybrid` quality
//! failure is materialised as `CommandModel` and can advance the existing
//! firmware controller.  In parallel, a PC-only shadow SMO/PLL consumes the
//! already selected voltage so measured-vs-command behaviour can be compared
//! without changing the target path.

use foc_algorithm::{
    average_inverter_phase_voltage_loss_v, clarke, Abc, AlphaBeta, InverterLossParameters,
    PllParam, SmoInput, SmoParam, SmoPllParam, SmoPllState,
};
use foc_control::{
    command_model_observer_voltage, st_gbm2804_reference_parameters, PhaseCurrents, PwmCommand,
    SmoPllTuning,
};
use foc_rt_bridge::{
    foc_rust_configure, foc_rust_default_st_config, foc_rust_init, foc_rust_realtime_step,
    foc_rust_start_realtime, FocFeedback, FocOutput, FocRealtimeInput, FocRuntimeConfig,
    FocRustContextStorage, FocStatus, FocTelemetry, FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL,
    FOC_REALTIME_OBSERVER_VOLTAGE_MEASURED, FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE,
    FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_BOARD_CALIBRATED,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_LOW_SATURATION, FOC_REALTIME_PHASE_VOLTAGE_QUALITY_STALE,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID, FOC_REALTIME_PHASE_VOLTAGE_REASON_HIGH_SATURATION,
    FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE,
    FOC_REALTIME_PHASE_VOLTAGE_REASON_LOW_SATURATION,
    FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS, FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE,
    FOC_REALTIME_VALID_PHASE_VOLTAGES, FOC_RUST_CONTEXT_CAPACITY,
};

use crate::phase_voltage_sensor::{
    PhaseVoltageFault, PhaseVoltageSensor, PhaseVoltageSensorConfig, PhaseVoltageSensorSample,
};
use crate::{InverterSimulationConfig, PmsmPlant};

const DC_BUS_VOLTAGE_V: f32 = 12.3;
const CONTROL_FREQUENCY_HZ: u32 = 12_000;
const CONTROL_DT_S: f32 = 1.0 / CONTROL_FREQUENCY_HZ as f32;
const WARMUP_TICKS: u32 = 12;
const RECOVERY_CONFIRM_SAMPLES: u32 = 3;
const MAX_SAMPLE_AGE_TICKS: u32 = 2;

/// Requested observer-voltage policy.  These are policies, not actual sources.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseVoltageRequest {
    CommandModel,
    Measured,
    Hybrid,
}

/// Actual source after the A22-equivalent quality and fallback decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseVoltageSelection {
    CommandModel,
    Measured,
    Unavailable,
}

impl PhaseVoltageSelection {
    const fn abi_value(self) -> u32 {
        match self {
            Self::CommandModel => FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL,
            Self::Measured => FOC_REALTIME_OBSERVER_VOLTAGE_MEASURED,
            Self::Unavailable => FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE,
        }
    }
}

/// Named checkpoints in the deterministic baseline/fault/recovery sequence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PhaseVoltageCheckpointKind {
    Baseline,
    Dropout,
    Stale,
    Recovery1,
    Recovery2,
    Recovered,
}

/// One PC-chain checkpoint, including the real V19 target-gate result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseVoltagePathCheckpoint {
    pub kind: PhaseVoltageCheckpointKind,
    pub control_sequence: u32,
    pub phase_voltage_sequence: u32,
    pub phase_voltage_age_ticks: u32,
    pub quality_state: u32,
    pub quality_reason_mask: u32,
    pub selection: PhaseVoltageSelection,
    pub fallback_event_count: u32,
    pub target_status: FocStatus,
    pub target_output: FocOutput,
    pub plant_voltage_alpha_beta_v: AlphaBeta,
    pub measured_voltage_alpha_beta_v: Option<AlphaBeta>,
    pub command_voltage_alpha_beta_v: AlphaBeta,
    pub shadow_voltage_alpha_beta_v: Option<AlphaBeta>,
    pub shadow_update_count: u32,
    pub shadow_angle_rad: f32,
    pub shadow_electrical_speed_rad_s: f32,
}

/// Complete PC-only result for one requested policy.
#[derive(Clone, Debug, PartialEq)]
pub struct PhaseVoltagePathRun {
    pub request: PhaseVoltageRequest,
    pub checkpoints: Vec<PhaseVoltagePathCheckpoint>,
}

#[derive(Clone, Copy, Debug)]
struct QualityDecision {
    state: u32,
    reason_mask: u32,
    selection: PhaseVoltageSelection,
    fallback_event_count: u32,
}

/// Projection of the A22 state machine needed by this integration sequence.
///
/// Saturation, unavailable samples, stale age, recovery hysteresis and the
/// Command/Measured/Hybrid policy have the same priority and output semantics as
/// `foc_phase_voltage_quality_evaluate`.  Open-wire and three-phase residual
/// detectors remain covered by the C A22 tests and are intentionally not
/// duplicated in this focused harness.
#[derive(Clone, Copy, Debug, Default)]
struct PhaseVoltageQualityProjection {
    healthy_streak: u32,
    fallback_active: bool,
    fallback_event_count: u32,
}

impl PhaseVoltageQualityProjection {
    fn evaluate(
        &mut self,
        sample: PhaseVoltageSensorSample,
        request: PhaseVoltageRequest,
    ) -> QualityDecision {
        let (state, mut reason_mask) = if !sample.configuration_valid || !sample.available {
            (
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE,
                FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE,
            )
        } else if sample.age_ticks > MAX_SAMPLE_AGE_TICKS {
            (
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_STALE,
                FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE,
            )
        } else if sample.low_rail_phase_mask != 0 {
            (
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_LOW_SATURATION,
                FOC_REALTIME_PHASE_VOLTAGE_REASON_LOW_SATURATION,
            )
        } else if sample.high_rail_phase_mask != 0 {
            (
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION,
                FOC_REALTIME_PHASE_VOLTAGE_REASON_HIGH_SATURATION,
            )
        } else {
            (FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID, 0)
        };

        if state == FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID {
            self.healthy_streak = self.healthy_streak.saturating_add(1);
        } else {
            self.healthy_streak = 0;
        }
        let measured_eligible = state == FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID
            && self.healthy_streak >= RECOVERY_CONFIRM_SAMPLES;
        if state == FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID && !measured_eligible {
            reason_mask |= FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS;
        }

        let selection = match request {
            PhaseVoltageRequest::CommandModel => PhaseVoltageSelection::CommandModel,
            PhaseVoltageRequest::Measured if measured_eligible => PhaseVoltageSelection::Measured,
            PhaseVoltageRequest::Measured => PhaseVoltageSelection::Unavailable,
            PhaseVoltageRequest::Hybrid if measured_eligible => PhaseVoltageSelection::Measured,
            PhaseVoltageRequest::Hybrid => PhaseVoltageSelection::CommandModel,
        };
        let fallback = request == PhaseVoltageRequest::Hybrid
            && selection == PhaseVoltageSelection::CommandModel;
        if fallback && !self.fallback_active {
            self.fallback_event_count = self.fallback_event_count.saturating_add(1);
        }
        self.fallback_active = fallback;

        QualityDecision {
            state,
            reason_mask,
            selection,
            fallback_event_count: self.fallback_event_count,
        }
    }
}

struct TargetHarness {
    context: FocRustContextStorage,
}

impl TargetHarness {
    fn new() -> Result<Self, FocStatus> {
        let mut harness = Self {
            context: FocRustContextStorage {
                bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
            },
        };
        let mut config = FocRuntimeConfig::default();
        // SAFETY: the context/config are aligned local ABI objects and are used
        // exclusively for this PC harness.
        unsafe {
            require_ok(foc_rust_init(&mut harness.context))?;
            require_ok(foc_rust_default_st_config(&mut config))?;
            require_ok(foc_rust_configure(&mut harness.context, &config))?;
            require_ok(foc_rust_start_realtime(&mut harness.context, 1, 524.0))?;
        }
        Ok(harness)
    }

    fn step(&mut self, input: &FocRealtimeInput) -> (FocStatus, FocOutput) {
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        // SAFETY: the context is initialized and all ABI objects are valid,
        // aligned, disjoint, and exclusively borrowed for this call.
        let status = unsafe {
            foc_rust_realtime_step(&mut self.context, input, &mut output, &mut telemetry)
        };
        (status, output)
    }
}

fn require_ok(status: FocStatus) -> Result<(), FocStatus> {
    if status == FocStatus::Ok {
        Ok(())
    } else {
        Err(status)
    }
}

#[derive(Clone, Copy, Debug)]
struct ShadowObserver {
    state: SmoPllState,
    params: SmoPllParam,
    update_count: u32,
}

impl ShadowObserver {
    fn new() -> Self {
        let motor = st_gbm2804_reference_parameters().motor;
        let tuning = SmoPllTuning::for_motor(motor);
        Self {
            state: SmoPllState::default(),
            params: SmoPllParam {
                smo: SmoParam {
                    rs: motor.stator_resistance_ohm,
                    ls: motor.ld_h,
                    ts: CONTROL_DT_S,
                    k_slide: tuning.k_slide_v,
                    boundary: tuning.boundary_a,
                    emf_filter_alpha: tuning.emf_filter_alpha,
                },
                pll: PllParam {
                    kp: tuning.pll_kp,
                    ki: tuning.pll_ki,
                    ts: CONTROL_DT_S,
                    omega_min: -2_000.0,
                    omega_max: 2_000.0,
                },
            },
            update_count: 0,
        }
    }

    fn step(&mut self, voltage: AlphaBeta, currents: PhaseCurrents) {
        self.state.update(
            &self.params,
            &SmoInput {
                voltage,
                current: clarke(Abc {
                    a: currents.a,
                    b: currents.b,
                    c: currents.c,
                }),
            },
        );
        self.update_count = self.update_count.saturating_add(1);
    }
}

fn terminal_voltages_to_alpha_beta(voltage: [f32; 3]) -> AlphaBeta {
    let common = (voltage[0] + voltage[1] + voltage[2]) / 3.0;
    clarke(Abc {
        a: voltage[0] - common,
        b: voltage[1] - common,
        c: voltage[2] - common,
    })
}

fn actual_terminal_voltages(
    pwm: PwmCommand,
    currents: PhaseCurrents,
    pwm_period_s: f32,
    inverter: InverterSimulationConfig,
) -> [f32; 3] {
    let loss = if inverter.dead_time_enabled {
        average_inverter_phase_voltage_loss_v(
            InverterLossParameters {
                dead_time_s: inverter.dead_time_s,
                pwm_period_s,
                device_drop_v: inverter.device_drop_v,
                current_zero_band_a: 0.0,
            },
            DC_BUS_VOLTAGE_V,
            Abc {
                a: currents.a,
                b: currents.b,
                c: currents.c,
            },
        )
    } else {
        Abc::default()
    };
    // Sensor and plant are derived from these exact same unipolar bridge-leg
    // voltages. Only the plant-side Clarke conversion removes common mode.
    [
        pwm.duty_a * DC_BUS_VOLTAGE_V - loss.a,
        pwm.duty_b * DC_BUS_VOLTAGE_V - loss.b,
        pwm.duty_c * DC_BUS_VOLTAGE_V - loss.c,
    ]
}

#[cfg(test)]
fn command_terminal_voltages(pwm: PwmCommand) -> [f32; 3] {
    let phase = foc_control::pwm_to_phase_voltage(pwm, DC_BUS_VOLTAGE_V);
    let common_mode_v = DC_BUS_VOLTAGE_V * 0.5;
    [
        phase.a + common_mode_v,
        phase.b + common_mode_v,
        phase.c + common_mode_v,
    ]
}

fn build_v19_input(
    currents: PhaseCurrents,
    control_sequence: u32,
    sample: PhaseVoltageSensorSample,
    decision: QualityDecision,
) -> FocRealtimeInput {
    let feedback = FocFeedback {
        phase_current_a: currents.a,
        phase_current_b: currents.b,
        phase_current_c: currents.c,
        dc_bus_voltage: DC_BUS_VOLTAGE_V,
        electrical_angle_rad: 0.0,
    };
    let mut input =
        FocRealtimeInput::command_model_from_legacy(feedback, control_sequence, CONTROL_DT_S);
    input.phase_voltage_sequence = sample.sequence;
    input.phase_voltage_age_ticks = sample.age_ticks;
    input.phase_voltage_provenance = FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_BOARD_CALIBRATED;
    input.phase_voltage_quality_state = decision.state;
    input.phase_voltage_reason_mask = decision.reason_mask;
    input.observer_voltage_selection = decision.selection.abi_value();
    input.phase_voltage_fallback_event_count = decision.fallback_event_count;
    if sample.available && sample.configuration_valid {
        input.valid_flags |= FOC_REALTIME_VALID_PHASE_VOLTAGES;
        input.phase_voltage_a_v = sample.volts[0];
        input.phase_voltage_b_v = sample.volts[1];
        input.phase_voltage_c_v = sample.volts[2];
    }
    input
}

fn selected_shadow_voltage(
    selection: PhaseVoltageSelection,
    command: AlphaBeta,
    sample: PhaseVoltageSensorSample,
) -> Option<AlphaBeta> {
    match selection {
        PhaseVoltageSelection::CommandModel => Some(command),
        PhaseVoltageSelection::Measured if sample.available => {
            Some(terminal_voltages_to_alpha_beta(sample.volts))
        }
        PhaseVoltageSelection::Measured | PhaseVoltageSelection::Unavailable => None,
    }
}

/// Runs the deterministic A24.4 baseline/dropout/stale/recovery sequence.
///
/// This is S3 PC evidence only.  It neither changes the target default nor makes
/// measured/hybrid target-capable; selected measured samples are expected to be
/// rejected by the current target bridge.
pub fn run_phase_voltage_path(
    request: PhaseVoltageRequest,
) -> Result<PhaseVoltagePathRun, FocStatus> {
    let motor = st_gbm2804_reference_parameters().motor;
    let mut plant = PmsmPlant::new(motor);
    plant.set_initial_mechanical_state(524.0, 0.3);
    let mut sensor = PhaseVoltageSensor::try_new(PhaseVoltageSensorConfig {
        full_scale_v: 18.3,
        delay_ticks: 0,
        ..PhaseVoltageSensorConfig::default()
    })
    .expect("fixed A24.4 sensor fixture must be valid");
    let inverter = InverterSimulationConfig {
        dead_time_enabled: true,
        dead_time_s: 550.0e-9,
        device_drop_v: 0.0,
    };
    let pwm_period_s = 1.0 / 24_000.0;
    let mut target = TargetHarness::new()?;
    let mut active_pwm = PwmCommand::default();
    let mut quality = PhaseVoltageQualityProjection::default();
    let mut shadow = ShadowObserver::new();

    // Warm the real bridge and plant only through the currently approved
    // CommandModel path.  Sampling still advances so V19 sequence/age coherence
    // is preserved when the requested-policy sequence starts.
    for control_sequence in 0..WARMUP_TICKS {
        let currents = plant.phase_currents();
        let terminal_voltage =
            actual_terminal_voltages(active_pwm, currents, pwm_period_s, inverter);
        let plant_voltage = terminal_voltages_to_alpha_beta(terminal_voltage);
        let _ = sensor.sample(terminal_voltage, PhaseVoltageFault::None);
        let feedback = FocFeedback {
            phase_current_a: currents.a,
            phase_current_b: currents.b,
            phase_current_c: currents.c,
            dc_bus_voltage: DC_BUS_VOLTAGE_V,
            electrical_angle_rad: 0.0,
        };
        let input =
            FocRealtimeInput::command_model_from_legacy(feedback, control_sequence, CONTROL_DT_S);
        let (status, output) = target.step(&input);
        require_ok(status)?;
        plant.step(plant_voltage, CONTROL_DT_S);
        active_pwm = PwmCommand {
            duty_a: output.duty_a,
            duty_b: output.duty_b,
            duty_c: output.duty_c,
        };
    }

    let sequence = [
        (None, PhaseVoltageFault::None),
        (None, PhaseVoltageFault::None),
        (
            Some(PhaseVoltageCheckpointKind::Baseline),
            PhaseVoltageFault::None,
        ),
        (
            Some(PhaseVoltageCheckpointKind::Dropout),
            PhaseVoltageFault::Dropout,
        ),
        (
            Some(PhaseVoltageCheckpointKind::Stale),
            PhaseVoltageFault::ForceStale {
                age_ticks: MAX_SAMPLE_AGE_TICKS + 1,
            },
        ),
        (
            Some(PhaseVoltageCheckpointKind::Recovery1),
            PhaseVoltageFault::None,
        ),
        (
            Some(PhaseVoltageCheckpointKind::Recovery2),
            PhaseVoltageFault::None,
        ),
        (
            Some(PhaseVoltageCheckpointKind::Recovered),
            PhaseVoltageFault::None,
        ),
    ];
    let mut checkpoints = Vec::with_capacity(6);
    for (offset, (kind, fault)) in sequence.into_iter().enumerate() {
        let control_sequence = WARMUP_TICKS + offset as u32;
        let currents = plant.phase_currents();
        let applied_pwm = active_pwm;
        let terminal_voltage =
            actual_terminal_voltages(applied_pwm, currents, pwm_period_s, inverter);
        let plant_voltage = terminal_voltages_to_alpha_beta(terminal_voltage);
        let sample = sensor.sample(terminal_voltage, fault);
        let decision = quality.evaluate(sample, request);
        let input = build_v19_input(currents, control_sequence, sample, decision);
        let command_voltage =
            command_model_observer_voltage(applied_pwm, DC_BUS_VOLTAGE_V).alpha_beta;
        let shadow_voltage = selected_shadow_voltage(decision.selection, command_voltage, sample);
        if let Some(voltage) = shadow_voltage {
            shadow.step(voltage, currents);
        }
        let measured_voltage = sample
            .available
            .then(|| terminal_voltages_to_alpha_beta(sample.volts));
        let (target_status, target_output) = target.step(&input);

        // A non-OK target result represents the C platform's fail-closed gate:
        // no duty is applied to the next plant tick.  `PwmCommand::default()` is
        // the centred zero vector in the PC model.
        active_pwm = if target_status == FocStatus::Ok {
            PwmCommand {
                duty_a: target_output.duty_a,
                duty_b: target_output.duty_b,
                duty_c: target_output.duty_c,
            }
        } else {
            PwmCommand::default()
        };
        plant.step(plant_voltage, CONTROL_DT_S);

        if let Some(kind) = kind {
            checkpoints.push(PhaseVoltagePathCheckpoint {
                kind,
                control_sequence,
                phase_voltage_sequence: sample.sequence,
                phase_voltage_age_ticks: sample.age_ticks,
                quality_state: decision.state,
                quality_reason_mask: decision.reason_mask,
                selection: decision.selection,
                fallback_event_count: decision.fallback_event_count,
                target_status,
                target_output,
                plant_voltage_alpha_beta_v: plant_voltage,
                measured_voltage_alpha_beta_v: measured_voltage,
                command_voltage_alpha_beta_v: command_voltage,
                shadow_voltage_alpha_beta_v: shadow_voltage,
                shadow_update_count: shadow.update_count,
                shadow_angle_rad: shadow.state.theta_rad,
                shadow_electrical_speed_rad_s: shadow.state.omega_rad_s,
            });
        }
    }

    Ok(PhaseVoltagePathRun {
        request,
        checkpoints,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checkpoint(
        run: &PhaseVoltagePathRun,
        kind: PhaseVoltageCheckpointKind,
    ) -> &PhaseVoltagePathCheckpoint {
        run.checkpoints
            .iter()
            .find(|checkpoint| checkpoint.kind == kind)
            .unwrap()
    }

    fn output_is_zero(output: FocOutput) -> bool {
        output.duty_a == 0.0 && output.duty_b == 0.0 && output.duty_c == 0.0
    }

    #[test]
    fn full_chain_is_deterministic_and_uses_plant_terminal_voltage() {
        let first = run_phase_voltage_path(PhaseVoltageRequest::CommandModel).unwrap();
        let second = run_phase_voltage_path(PhaseVoltageRequest::CommandModel).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.checkpoints.len(), 6);

        let baseline = checkpoint(&first, PhaseVoltageCheckpointKind::Baseline);
        let measured = baseline.measured_voltage_alpha_beta_v.unwrap();
        assert_eq!(baseline.phase_voltage_sequence, baseline.control_sequence);
        assert_eq!(baseline.phase_voltage_age_ticks, 0);
        assert!((measured.alpha - baseline.plant_voltage_alpha_beta_v.alpha).abs() < 0.01);
        assert!((measured.beta - baseline.plant_voltage_alpha_beta_v.beta).abs() < 0.01);
        assert!(baseline.command_voltage_alpha_beta_v.alpha.is_finite());
        assert!(baseline.command_voltage_alpha_beta_v.beta.is_finite());
    }

    #[test]
    fn mode_matrix_pins_target_measured_gate_and_hybrid_fallback() {
        let command = run_phase_voltage_path(PhaseVoltageRequest::CommandModel).unwrap();
        assert!(command.checkpoints.iter().all(|point| {
            point.selection == PhaseVoltageSelection::CommandModel
                && point.target_status == FocStatus::Ok
        }));

        let measured = run_phase_voltage_path(PhaseVoltageRequest::Measured).unwrap();
        for point in &measured.checkpoints {
            let expected_selection = match point.kind {
                PhaseVoltageCheckpointKind::Baseline | PhaseVoltageCheckpointKind::Recovered => {
                    PhaseVoltageSelection::Measured
                }
                _ => PhaseVoltageSelection::Unavailable,
            };
            assert_eq!(point.selection, expected_selection);
            assert_eq!(point.target_status, FocStatus::NotConfigured);
            assert!(output_is_zero(point.target_output));
        }

        let hybrid = run_phase_voltage_path(PhaseVoltageRequest::Hybrid).unwrap();
        for point in &hybrid.checkpoints {
            let measured_selected = matches!(
                point.kind,
                PhaseVoltageCheckpointKind::Baseline | PhaseVoltageCheckpointKind::Recovered
            );
            if measured_selected {
                assert_eq!(point.selection, PhaseVoltageSelection::Measured);
                assert_eq!(point.target_status, FocStatus::NotConfigured);
                assert!(output_is_zero(point.target_output));
            } else {
                assert_eq!(point.selection, PhaseVoltageSelection::CommandModel);
                assert_eq!(point.target_status, FocStatus::Ok);
            }
        }
        assert_eq!(
            checkpoint(&hybrid, PhaseVoltageCheckpointKind::Dropout).fallback_event_count,
            2
        );
        assert_eq!(
            checkpoint(&hybrid, PhaseVoltageCheckpointKind::Recovered).fallback_event_count,
            2
        );
    }

    #[test]
    fn dropout_stale_and_recovery_fields_match_a22_v19_semantics() {
        let measured = run_phase_voltage_path(PhaseVoltageRequest::Measured).unwrap();
        let dropout = checkpoint(&measured, PhaseVoltageCheckpointKind::Dropout);
        assert_eq!(
            dropout.quality_state,
            FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE
        );
        assert_eq!(
            dropout.quality_reason_mask,
            FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE
        );
        assert!(dropout.measured_voltage_alpha_beta_v.is_none());

        let stale = checkpoint(&measured, PhaseVoltageCheckpointKind::Stale);
        assert_eq!(
            stale.quality_state,
            FOC_REALTIME_PHASE_VOLTAGE_QUALITY_STALE
        );
        assert_eq!(
            stale.quality_reason_mask,
            FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE
        );
        assert_eq!(stale.phase_voltage_age_ticks, MAX_SAMPLE_AGE_TICKS + 1);

        for kind in [
            PhaseVoltageCheckpointKind::Recovery1,
            PhaseVoltageCheckpointKind::Recovery2,
        ] {
            let point = checkpoint(&measured, kind);
            assert_eq!(
                point.quality_state,
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID
            );
            assert_eq!(
                point.quality_reason_mask,
                FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS
            );
            assert_eq!(point.selection, PhaseVoltageSelection::Unavailable);
        }
        let recovered = checkpoint(&measured, PhaseVoltageCheckpointKind::Recovered);
        assert_eq!(
            recovered.quality_state,
            FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID
        );
        assert_eq!(recovered.quality_reason_mask, 0);
        assert_eq!(recovered.selection, PhaseVoltageSelection::Measured);
    }

    #[test]
    fn shadow_observer_consumes_only_the_resolved_source() {
        let measured = run_phase_voltage_path(PhaseVoltageRequest::Measured).unwrap();
        let baseline = checkpoint(&measured, PhaseVoltageCheckpointKind::Baseline);
        let dropout = checkpoint(&measured, PhaseVoltageCheckpointKind::Dropout);
        let recovered = checkpoint(&measured, PhaseVoltageCheckpointKind::Recovered);
        assert_eq!(baseline.shadow_update_count, 1);
        assert_eq!(dropout.shadow_update_count, 1);
        assert_eq!(recovered.shadow_update_count, 2);
        assert!(baseline.shadow_voltage_alpha_beta_v.is_some());
        assert!(dropout.shadow_voltage_alpha_beta_v.is_none());

        let hybrid = run_phase_voltage_path(PhaseVoltageRequest::Hybrid).unwrap();
        assert!(hybrid
            .checkpoints
            .iter()
            .all(|point| point.shadow_voltage_alpha_beta_v.is_some()));
        let last = checkpoint(&hybrid, PhaseVoltageCheckpointKind::Recovered);
        assert_eq!(last.shadow_update_count, 8);
        assert!(last.shadow_angle_rad.is_finite());
        assert!(last.shadow_electrical_speed_rad_s.is_finite());
    }

    #[test]
    fn source_voltage_changes_at_hybrid_fallback_and_recovery() {
        let hybrid = run_phase_voltage_path(PhaseVoltageRequest::Hybrid).unwrap();
        let dropout = checkpoint(&hybrid, PhaseVoltageCheckpointKind::Dropout);
        assert_eq!(
            dropout.shadow_voltage_alpha_beta_v,
            Some(dropout.command_voltage_alpha_beta_v)
        );
        let recovered = checkpoint(&hybrid, PhaseVoltageCheckpointKind::Recovered);
        assert_eq!(recovered.selection, PhaseVoltageSelection::Measured);
        assert_eq!(
            recovered.shadow_voltage_alpha_beta_v,
            recovered.measured_voltage_alpha_beta_v
        );
    }

    #[test]
    fn command_terminal_helper_remains_common_mode_only() {
        let pwm = PwmCommand {
            duty_a: 0.7,
            duty_b: 0.4,
            duty_c: 0.2,
        };
        let terminal = command_terminal_voltages(pwm);
        let from_terminal = terminal_voltages_to_alpha_beta(terminal);
        let expected = command_model_observer_voltage(pwm, DC_BUS_VOLTAGE_V).alpha_beta;
        assert!((from_terminal.alpha - expected.alpha).abs() < 1.0e-6);
        assert!((from_terminal.beta - expected.beta).abs() < 1.0e-6);
    }
}
