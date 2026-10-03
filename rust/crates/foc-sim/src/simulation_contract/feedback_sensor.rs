//! Independent Rust-PC feedback sensor model and D2-D4 trace generator.
//!
//! The model reads the same SI profile identity as the existing D0/D1 gate,
//! but it does not call the MATLAB implementation or consume expected traces.

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use foc_control::{
    FeedbackMode, FeedbackRouteState, FeedbackRouter, FeedbackRouterConfig, FeedbackSourceSample,
    PRODUCT_FEEDBACK_QUALITY_CALIBRATED, PRODUCT_FEEDBACK_QUALITY_DEGRADED,
    PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID, PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND,
    PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE, PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
    PRODUCT_FEEDBACK_VALID_KNOWN_MASK, PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY,
};

use super::{
    array, boolean, exact_keys, finite_number, invalid, load_simulation_contract, object,
    parse_json_bytes, required, sha256_hex, string, unsigned, SimulationContractError,
};

pub const FEEDBACK_SCENARIO_RELATIVE_PATH: &str =
    "simulation/scenarios/feedback_sensor_fault_matrix_v1.json";
pub const RUST_FEEDBACK_ENGINE_ID: &str = "fluxrt.rust.feedback_sensor.v1";

const TAU: f64 = std::f64::consts::TAU;
const SENSORLESS_MODE_NAME: &str = "Sensorless";
const ENCODER_MODE_NAME: &str = "IncrementalEncoder";
const HALL_MODE_NAME: &str = "Hall";

#[derive(Clone, Debug)]
struct RouterScenario {
    maximum_age_control_ticks: u32,
    acquire_good_samples: u16,
    loss_bad_samples: u16,
    recovery_good_samples: u16,
}

#[derive(Clone, Debug)]
struct EncoderScenario {
    counts_per_revolution: u32,
    delay_control_ticks: u32,
    electrical_offset_rad: f64,
}

#[derive(Clone, Debug)]
struct HallScenario {
    delay_control_ticks: u32,
    sector_map: [u8; 6],
}

#[derive(Clone, Debug)]
struct FaultCase {
    case_id: String,
    feature_enabled: bool,
    encoder_drop_enabled: bool,
    encoder_drop_start_tick: u32,
    encoder_drop_end_tick: u32,
    encoder_index_missing: bool,
    hall_wrong_order_enabled: bool,
    hall_wrong_order_start_tick: u32,
    hall_wrong_order_end_tick: u32,
}

impl FaultCase {
    fn encoder_dropped(&self, tick: u32) -> bool {
        self.encoder_drop_enabled
            && (self.encoder_drop_start_tick..=self.encoder_drop_end_tick).contains(&tick)
    }

    fn hall_wrong_order(&self, tick: u32) -> bool {
        self.hall_wrong_order_enabled
            && (self.hall_wrong_order_start_tick..=self.hall_wrong_order_end_tick).contains(&tick)
    }
}

#[derive(Clone, Debug)]
struct FeedbackSensorScenario {
    scenario_id: String,
    revision: u32,
    model_revision: String,
    evaluation_ticks: u32,
    initial_mechanical_position_rad: f64,
    mechanical_velocity_rad_s: f64,
    router: RouterScenario,
    encoder: EncoderScenario,
    hall: HallScenario,
    cases: Vec<FaultCase>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FeedbackTraceRow {
    pub case_id: String,
    pub control_tick: u32,
    pub time_s: f64,
    pub primary_available: bool,
    pub backup_available: bool,
    pub truth_position_rad: f64,
    pub truth_velocity_rad_s: f64,
    pub encoder_position_rad: Option<f64>,
    pub hall_electrical_angle_rad: Option<f64>,
    pub selected_position_rad: f64,
    pub selected_velocity_rad_s: f64,
    pub selected_electrical_angle_rad: f64,
    pub active_mode: &'static str,
    pub route_state: &'static str,
    pub valid_flags: u32,
    pub quality_flags: u32,
    pub sample_age_us: u32,
    pub event: &'static str,
}

#[derive(Clone, Debug)]
pub struct FeedbackSensorGateResult {
    pub scenario_id: String,
    pub scenario_revision: u32,
    pub scenario_sha256: String,
    pub model_revision: String,
    pub metadata: Vec<(String, String)>,
    pub rows: Vec<FeedbackTraceRow>,
}

impl FeedbackSensorGateResult {
    pub fn write(&self, output_dir: impl AsRef<Path>) -> Result<(), SimulationContractError> {
        let output_dir = output_dir.as_ref();
        fs::create_dir_all(output_dir).map_err(|error| {
            SimulationContractError::Io(format!("cannot create {}: {error}", output_dir.display()))
        })?;
        let metadata_path = output_dir.join("rust-feedback-d0.txt");
        let trace_path = output_dir.join("rust-feedback-trace.csv");
        let mut metadata = String::new();
        for (key, value) in &self.metadata {
            writeln!(&mut metadata, "{key}={value}").unwrap();
        }
        fs::write(&metadata_path, metadata).map_err(|error| {
            SimulationContractError::Io(format!(
                "cannot write {}: {error}",
                metadata_path.display()
            ))
        })?;

        let mut csv = String::from(
            "case_id,control_tick,time_s,feedback_primary_available,feedback_backup_available,truth_position_rad,truth_velocity_rad_s,feedback_encoder_position_rad,feedback_hall_electrical_angle_rad,feedback_selected_position_rad,feedback_selected_velocity_rad_s,feedback_selected_electrical_angle_rad,feedback_active_mode,feedback_route_state,feedback_valid_flags,feedback_quality_flags,feedback_sample_age_s,feedback_event\n",
        );
        for row in &self.rows {
            writeln!(
                &mut csv,
                "{},{},{:.9},{},{},{:.9},{:.9},{},{},{:.9},{:.9},{:.9},{},{},{},{},{:.9},{}",
                row.case_id,
                row.control_tick,
                row.time_s,
                u8::from(row.primary_available),
                u8::from(row.backup_available),
                row.truth_position_rad,
                row.truth_velocity_rad_s,
                optional_number(row.encoder_position_rad),
                optional_number(row.hall_electrical_angle_rad),
                row.selected_position_rad,
                row.selected_velocity_rad_s,
                row.selected_electrical_angle_rad,
                row.active_mode,
                row.route_state,
                row.valid_flags,
                row.quality_flags,
                f64::from(row.sample_age_us) * 1.0e-6,
                row.event,
            )
            .unwrap();
        }
        fs::write(&trace_path, csv).map_err(|error| {
            SimulationContractError::Io(format!("cannot write {}: {error}", trace_path.display()))
        })
    }
}

pub fn run_feedback_sensor_gate(
    root: impl AsRef<Path>,
) -> Result<FeedbackSensorGateResult, SimulationContractError> {
    let root = root.as_ref();
    let base = load_simulation_contract(root)?;
    let scenario_path = root.join(FEEDBACK_SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let scenario_sha256 = sha256_hex(&bytes);
    let value = parse_json_bytes(&bytes)?;
    let scenario = parse_feedback_scenario(object(&value, "feedback scenario")?)?;
    if scenario.revision == 0
        || scenario.model_revision.is_empty()
        || scenario.evaluation_ticks < 4
        || scenario.evaluation_ticks > 100_000
        || scenario.mechanical_velocity_rad_s == 0.0
        || string(object(&value, "feedback scenario")?, "profile_id")? != base.profile.profile_id
        || unsigned(object(&value, "feedback scenario")?, "profile_revision")? as u32
            != base.profile.revision
    {
        return invalid("feedback scenario identity/range mismatch".to_owned());
    }

    let mut rows = Vec::with_capacity(
        scenario.cases.len() * usize::try_from(scenario.evaluation_ticks).unwrap(),
    );
    for case in &scenario.cases {
        rows.extend(run_case(
            case,
            &scenario,
            base.profile.pole_pairs,
            base.profile.control_frequency_hz,
            base.profile.revision,
        )?);
    }
    verify_gate_semantics(&scenario, &rows)?;
    let workspace_revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    let metadata = vec![
        ("feedback_gate_result_version".to_owned(), "1".to_owned()),
        ("engine_id".to_owned(), RUST_FEEDBACK_ENGINE_ID.to_owned()),
        ("workspace_revision".to_owned(), workspace_revision),
        ("base_bundle_id".to_owned(), base.bundle_id),
        ("base_bundle_sha256".to_owned(), base.bundle_sha256),
        ("profile_id".to_owned(), base.profile.profile_id),
        (
            "profile_revision".to_owned(),
            base.profile.revision.to_string(),
        ),
        ("profile_sha256".to_owned(), base.profile_sha256),
        ("trace_schema_id".to_owned(), base.trace_schema_id),
        (
            "trace_schema_version".to_owned(),
            base.trace_schema_version.to_string(),
        ),
        ("trace_schema_sha256".to_owned(), base.trace_schema_sha256),
        ("comparison_id".to_owned(), base.comparison_id),
        (
            "comparison_version".to_owned(),
            base.comparison_version.to_string(),
        ),
        ("comparison_sha256".to_owned(), base.comparison_sha256),
        (
            "feedback_scenario_id".to_owned(),
            scenario.scenario_id.clone(),
        ),
        (
            "feedback_scenario_revision".to_owned(),
            scenario.revision.to_string(),
        ),
        (
            "feedback_scenario_sha256".to_owned(),
            scenario_sha256.clone(),
        ),
        (
            "feedback_model_revision".to_owned(),
            scenario.model_revision.clone(),
        ),
        ("case_count".to_owned(), scenario.cases.len().to_string()),
        ("row_count".to_owned(), rows.len().to_string()),
        ("d0".to_owned(), "PASS".to_owned()),
        ("d2".to_owned(), "PASS".to_owned()),
        ("d3".to_owned(), "PASS".to_owned()),
        ("d4".to_owned(), "PASS".to_owned()),
        ("feature_off".to_owned(), "PASS".to_owned()),
    ];
    Ok(FeedbackSensorGateResult {
        scenario_id: scenario.scenario_id,
        scenario_revision: scenario.revision,
        scenario_sha256,
        model_revision: scenario.model_revision,
        metadata,
        rows,
    })
}

fn run_case(
    case: &FaultCase,
    scenario: &FeedbackSensorScenario,
    pole_pairs: u32,
    control_frequency_hz: u32,
    pole_pair_revision: u32,
) -> Result<Vec<FeedbackTraceRow>, SimulationContractError> {
    let maximum_age_us = u32::try_from(
        (u64::from(scenario.router.maximum_age_control_ticks) * 1_000_000)
            .div_ceil(u64::from(control_frequency_hz)),
    )
    .map_err(|_| SimulationContractError::Invalid("maximum age overflow".to_owned()))?;
    let mut router = FeedbackRouter::new(FeedbackRouterConfig {
        axis_id: 0,
        primary_mode: FeedbackMode::IncrementalEncoder as u32,
        backup_mode: FeedbackMode::Hall as u32,
        fallback_enabled: true,
        maximum_age_us,
        acquire_good_samples: scenario.router.acquire_good_samples,
        loss_bad_samples: scenario.router.loss_bad_samples,
        recovery_good_samples: scenario.router.recovery_good_samples,
    })
    .map_err(|error| SimulationContractError::Invalid(format!("router config: {error:?}")))?;
    let mut encoder_sequence = 0_u32;
    let mut hall_sequence = 0_u32;
    let mut previous_state = "None";
    let mut previous_mode = "None";
    let mut rows = Vec::with_capacity(usize::try_from(scenario.evaluation_ticks).unwrap());
    for tick in 0..scenario.evaluation_ticks {
        let time_s = f64::from(tick) / f64::from(control_frequency_hz);
        let truth_position =
            scenario.initial_mechanical_position_rad + scenario.mechanical_velocity_rad_s * time_s;
        let true_electrical = wrap_tau(truth_position * f64::from(pole_pairs));
        if !case.feature_enabled {
            rows.push(FeedbackTraceRow {
                case_id: case.case_id.clone(),
                control_tick: tick,
                time_s,
                primary_available: false,
                backup_available: false,
                truth_position_rad: truth_position,
                truth_velocity_rad_s: scenario.mechanical_velocity_rad_s,
                encoder_position_rad: None,
                hall_electrical_angle_rad: None,
                selected_position_rad: truth_position,
                selected_velocity_rad_s: scenario.mechanical_velocity_rad_s,
                selected_electrical_angle_rad: true_electrical,
                active_mode: SENSORLESS_MODE_NAME,
                route_state: "Bypassed",
                valid_flags: PRODUCT_FEEDBACK_VALID_KNOWN_MASK,
                quality_flags: PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID,
                sample_age_us: 0,
                event: if tick == 0 { "feature_off" } else { "none" },
            });
            continue;
        }

        let encoder = build_encoder_sample(
            tick,
            case,
            scenario,
            pole_pairs,
            control_frequency_hz,
            pole_pair_revision,
            &mut encoder_sequence,
        );
        let hall = build_hall_sample(
            tick,
            case,
            scenario,
            pole_pairs,
            control_frequency_hz,
            pole_pair_revision,
            &mut hall_sequence,
        );
        if let Some(sample) = encoder {
            router.ingest(sample).map_err(|error| {
                SimulationContractError::Invalid(format!("encoder ingest: {error:?}"))
            })?;
        }
        if let Some(sample) = hall {
            router.ingest(sample).map_err(|error| {
                SimulationContractError::Invalid(format!("hall ingest: {error:?}"))
            })?;
        }
        let now_us = tick_to_us(tick, control_frequency_hz);
        let decision = router.route(tick + 1, now_us).map_err(|error| {
            SimulationContractError::Invalid(format!("feedback route: {error:?}"))
        })?;
        let state = route_state_name(decision.state);
        let active_mode = match decision.state {
            FeedbackRouteState::Primary => ENCODER_MODE_NAME,
            FeedbackRouteState::Fallback => HALL_MODE_NAME,
            FeedbackRouteState::Acquiring | FeedbackRouteState::Lost => "None",
        };
        let event = transition_event(previous_state, previous_mode, state);
        previous_state = state;
        previous_mode = active_mode;
        rows.push(FeedbackTraceRow {
            case_id: case.case_id.clone(),
            control_tick: tick,
            time_s,
            primary_available: encoder.is_some(),
            backup_available: hall.is_some(),
            truth_position_rad: truth_position,
            truth_velocity_rad_s: scenario.mechanical_velocity_rad_s,
            encoder_position_rad: encoder.map(|sample| f64::from(sample.mechanical_position_rad)),
            hall_electrical_angle_rad: hall.map(|sample| f64::from(sample.electrical_angle_rad)),
            selected_position_rad: f64::from(decision.snapshot.mechanical_position_rad),
            selected_velocity_rad_s: f64::from(decision.snapshot.mechanical_velocity_rad_s),
            selected_electrical_angle_rad: f64::from(decision.snapshot.electrical_angle_rad),
            active_mode,
            route_state: state,
            valid_flags: decision.snapshot.valid_flags,
            quality_flags: decision.snapshot.quality_flags,
            sample_age_us: decision.snapshot.sample_age_us,
            event,
        });
    }
    Ok(rows)
}

#[allow(clippy::too_many_arguments)]
fn build_encoder_sample(
    tick: u32,
    case: &FaultCase,
    scenario: &FeedbackSensorScenario,
    pole_pairs: u32,
    control_frequency_hz: u32,
    pole_pair_revision: u32,
    sequence: &mut u32,
) -> Option<FeedbackSourceSample> {
    let source_tick = tick.checked_sub(scenario.encoder.delay_control_ticks)?;
    if case.encoder_dropped(tick) {
        return None;
    }
    *sequence += 1;
    let source_time = f64::from(source_tick) / f64::from(control_frequency_hz);
    let true_position =
        scenario.initial_mechanical_position_rad + scenario.mechanical_velocity_rad_s * source_time;
    let step = TAU / f64::from(scenario.encoder.counts_per_revolution);
    let multi_turn = (true_position / step).round() * step;
    let mechanical = wrap_tau(multi_turn);
    let electrical =
        wrap_tau(mechanical * f64::from(pole_pairs) + scenario.encoder.electrical_offset_rad);
    let mut quality =
        PRODUCT_FEEDBACK_QUALITY_CALIBRATED | PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID;
    if !case.encoder_index_missing {
        quality |= PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND;
    }
    Some(FeedbackSourceSample {
        axis_id: 0,
        mode: FeedbackMode::IncrementalEncoder as u32,
        sequence: *sequence,
        sampled_at_ms: tick_to_us(source_tick, control_frequency_hz) / 1000,
        sampled_at_us: tick_to_us(source_tick, control_frequency_hz),
        valid_flags: PRODUCT_FEEDBACK_VALID_KNOWN_MASK,
        quality_flags: quality,
        direction: 1,
        pole_pair_revision,
        mechanical_position_rad: mechanical as f32,
        multi_turn_position_rad: multi_turn as f32,
        mechanical_velocity_rad_s: scenario.mechanical_velocity_rad_s as f32,
        electrical_angle_rad: electrical as f32,
        electrical_velocity_rad_s: (scenario.mechanical_velocity_rad_s * f64::from(pole_pairs))
            as f32,
    })
}

#[allow(clippy::too_many_arguments)]
fn build_hall_sample(
    tick: u32,
    case: &FaultCase,
    scenario: &FeedbackSensorScenario,
    pole_pairs: u32,
    control_frequency_hz: u32,
    pole_pair_revision: u32,
    sequence: &mut u32,
) -> Option<FeedbackSourceSample> {
    let source_tick = tick.checked_sub(scenario.hall.delay_control_ticks)?;
    *sequence += 1;
    let source_time = f64::from(source_tick) / f64::from(control_frequency_hz);
    let mechanical =
        scenario.initial_mechanical_position_rad + scenario.mechanical_velocity_rad_s * source_time;
    let electrical = wrap_tau(mechanical * f64::from(pole_pairs));
    let raw_sector = ((electrical / (TAU / 6.0)).floor() as usize).min(5);
    let mapped_sector = usize::from(scenario.hall.sector_map[raw_sector]);
    let quantized_angle = (mapped_sector as f64 + 0.5) * TAU / 6.0;
    let mut quality =
        PRODUCT_FEEDBACK_QUALITY_CALIBRATED | PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID;
    if case.hall_wrong_order(tick) {
        quality |= PRODUCT_FEEDBACK_QUALITY_DEGRADED;
    }
    Some(FeedbackSourceSample {
        axis_id: 0,
        mode: FeedbackMode::Hall as u32,
        sequence: *sequence,
        sampled_at_ms: tick_to_us(source_tick, control_frequency_hz) / 1000,
        sampled_at_us: tick_to_us(source_tick, control_frequency_hz),
        valid_flags: PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
            | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
            | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
        quality_flags: quality,
        direction: 1,
        pole_pair_revision,
        mechanical_position_rad: 0.0,
        multi_turn_position_rad: 0.0,
        mechanical_velocity_rad_s: scenario.mechanical_velocity_rad_s as f32,
        electrical_angle_rad: quantized_angle as f32,
        electrical_velocity_rad_s: (scenario.mechanical_velocity_rad_s * f64::from(pole_pairs))
            as f32,
    })
}

fn verify_gate_semantics(
    scenario: &FeedbackSensorScenario,
    rows: &[FeedbackTraceRow],
) -> Result<(), SimulationContractError> {
    let expected_rows = scenario.cases.len()
        * usize::try_from(scenario.evaluation_ticks)
            .map_err(|_| SimulationContractError::Invalid("row count overflow".to_owned()))?;
    if rows.len() != expected_rows {
        return invalid("feedback row count mismatch".to_owned());
    }
    let feature_off: Vec<_> = rows
        .iter()
        .filter(|row| row.case_id == "feature_off")
        .collect();
    if feature_off.len() != usize::try_from(scenario.evaluation_ticks).unwrap()
        || feature_off.iter().any(|row| {
            row.route_state != "Bypassed"
                || row.active_mode != SENSORLESS_MODE_NAME
                || row.selected_position_rad != row.truth_position_rad
                || row.selected_velocity_rad_s != row.truth_velocity_rad_s
        })
    {
        return invalid("feature-off path is not exactly equivalent".to_owned());
    }
    let drop_events: Vec<_> = rows
        .iter()
        .filter(|row| row.case_id == "encoder_drop_fallback")
        .filter(|row| row.event != "none")
        .map(|row| (row.control_tick, row.event))
        .collect();
    if !drop_events.contains(&(7, "fallback_entered"))
        || !drop_events.contains(&(12, "primary_recovered"))
    {
        return invalid(format!(
            "fallback/recovery timing mismatch: {drop_events:?}"
        ));
    }
    if !rows.iter().any(|row| {
        row.case_id == "encoder_index_missing"
            && row.route_state == "Fallback"
            && row.active_mode == HALL_MODE_NAME
    }) {
        return invalid("missing encoder Index did not force Hall fallback".to_owned());
    }
    if !rows.iter().any(|row| {
        row.case_id == "hall_wrong_order_and_primary_drop"
            && row.route_state == "Lost"
            && row.valid_flags == 0
    }) {
        return invalid("combined encoder/Hall fault did not fail closed".to_owned());
    }
    Ok(())
}

fn parse_feedback_scenario(
    root: &std::collections::BTreeMap<String, super::JsonValue>,
) -> Result<FeedbackSensorScenario, SimulationContractError> {
    exact_keys(root, SCENARIO_KEYS, "feedback scenario")?;
    if string(root, "contract")? != "fluxrt-feedback-sensor-scenario"
        || unsigned(root, "version")? != 1
        || string(root, "units")? != "SI"
    {
        return invalid("unknown feedback scenario contract/version/units".to_owned());
    }
    let router_object = object(required(root, "router")?, "feedback router")?;
    exact_keys(router_object, ROUTER_KEYS, "feedback router")?;
    if string(router_object, "primary_mode")? != ENCODER_MODE_NAME
        || string(router_object, "backup_mode")? != HALL_MODE_NAME
    {
        return invalid("feedback router mode tuple is not canonical".to_owned());
    }
    let router = RouterScenario {
        maximum_age_control_ticks: to_nonzero_u32(
            unsigned(router_object, "maximum_age_control_ticks")?,
            "maximum_age_control_ticks",
        )?,
        acquire_good_samples: to_nonzero_u16(
            unsigned(router_object, "acquire_good_samples")?,
            "acquire_good_samples",
        )?,
        loss_bad_samples: to_nonzero_u16(
            unsigned(router_object, "loss_bad_samples")?,
            "loss_bad_samples",
        )?,
        recovery_good_samples: to_nonzero_u16(
            unsigned(router_object, "recovery_good_samples")?,
            "recovery_good_samples",
        )?,
    };

    let encoder_object = object(required(root, "encoder")?, "encoder model")?;
    exact_keys(encoder_object, ENCODER_KEYS, "encoder model")?;
    let encoder = EncoderScenario {
        counts_per_revolution: to_nonzero_u32(
            unsigned(encoder_object, "counts_per_revolution")?,
            "counts_per_revolution",
        )?,
        delay_control_ticks: u32::try_from(unsigned(encoder_object, "delay_control_ticks")?)
            .map_err(|_| SimulationContractError::Invalid("encoder delay overflow".to_owned()))?,
        electrical_offset_rad: finite_number(encoder_object, "electrical_offset_rad")?,
    };

    let hall_object = object(required(root, "hall")?, "Hall model")?;
    exact_keys(hall_object, HALL_KEYS, "Hall model")?;
    let sector_values = array(required(hall_object, "sector_map")?, "Hall sector_map")?;
    if sector_values.len() != 6 {
        return invalid("Hall sector_map must contain six entries".to_owned());
    }
    let mut sector_map = [0_u8; 6];
    let mut sectors = BTreeSet::new();
    for (index, value) in sector_values.iter().enumerate() {
        let item = match value {
            super::JsonValue::Number(number)
                if *number >= 0.0 && *number <= 5.0 && number.fract() == 0.0 =>
            {
                *number as u8
            }
            _ => return invalid("Hall sector_map entry must be 0..5".to_owned()),
        };
        sector_map[index] = item;
        sectors.insert(item);
    }
    if sectors.len() != 6 {
        return invalid("Hall sector_map must be a permutation".to_owned());
    }
    let hall = HallScenario {
        delay_control_ticks: u32::try_from(unsigned(hall_object, "delay_control_ticks")?)
            .map_err(|_| SimulationContractError::Invalid("Hall delay overflow".to_owned()))?,
        sector_map,
    };

    let evaluation_ticks = u32::try_from(unsigned(root, "evaluation_ticks")?)
        .map_err(|_| SimulationContractError::Invalid("evaluation_ticks overflow".to_owned()))?;
    let case_values = array(required(root, "cases")?, "feedback cases")?;
    if case_values.is_empty() {
        return invalid("feedback cases must not be empty".to_owned());
    }
    let mut cases = Vec::with_capacity(case_values.len());
    let mut case_ids = BTreeSet::new();
    for value in case_values {
        let case_object = object(value, "feedback case")?;
        exact_keys(case_object, CASE_KEYS, "feedback case")?;
        let case = FaultCase {
            case_id: string(case_object, "case_id")?.to_owned(),
            feature_enabled: boolean(case_object, "feature_enabled")?,
            encoder_drop_enabled: boolean(case_object, "encoder_drop_enabled")?,
            encoder_drop_start_tick: to_u32(
                unsigned(case_object, "encoder_drop_start_tick")?,
                "encoder_drop_start_tick",
            )?,
            encoder_drop_end_tick: to_u32(
                unsigned(case_object, "encoder_drop_end_tick")?,
                "encoder_drop_end_tick",
            )?,
            encoder_index_missing: boolean(case_object, "encoder_index_missing")?,
            hall_wrong_order_enabled: boolean(case_object, "hall_wrong_order_enabled")?,
            hall_wrong_order_start_tick: to_u32(
                unsigned(case_object, "hall_wrong_order_start_tick")?,
                "hall_wrong_order_start_tick",
            )?,
            hall_wrong_order_end_tick: to_u32(
                unsigned(case_object, "hall_wrong_order_end_tick")?,
                "hall_wrong_order_end_tick",
            )?,
        };
        if case.case_id.is_empty()
            || !case_ids.insert(case.case_id.clone())
            || (case.encoder_drop_enabled
                && (case.encoder_drop_start_tick > case.encoder_drop_end_tick
                    || case.encoder_drop_end_tick >= evaluation_ticks))
            || (case.hall_wrong_order_enabled
                && (case.hall_wrong_order_start_tick > case.hall_wrong_order_end_tick
                    || case.hall_wrong_order_end_tick >= evaluation_ticks))
        {
            return invalid("invalid feedback case identity/window".to_owned());
        }
        cases.push(case);
    }
    if !cases
        .iter()
        .any(|case| case.case_id == "feature_off" && !case.feature_enabled)
    {
        return invalid("feature_off case is required".to_owned());
    }
    Ok(FeedbackSensorScenario {
        scenario_id: string(root, "scenario_id")?.to_owned(),
        revision: to_u32(unsigned(root, "revision")?, "scenario revision")?,
        model_revision: string(root, "model_revision")?.to_owned(),
        evaluation_ticks,
        initial_mechanical_position_rad: finite_number(root, "initial_mechanical_position_rad")?,
        mechanical_velocity_rad_s: finite_number(root, "mechanical_velocity_rad_s")?,
        router,
        encoder,
        hall,
        cases,
    })
}

fn tick_to_us(tick: u32, frequency_hz: u32) -> u32 {
    ((u64::from(tick) * 1_000_000) / u64::from(frequency_hz)) as u32
}

fn wrap_tau(value: f64) -> f64 {
    value.rem_euclid(TAU)
}

fn route_state_name(state: FeedbackRouteState) -> &'static str {
    match state {
        FeedbackRouteState::Acquiring => "Acquiring",
        FeedbackRouteState::Primary => "Primary",
        FeedbackRouteState::Fallback => "Fallback",
        FeedbackRouteState::Lost => "Lost",
    }
}

fn transition_event(previous_state: &str, previous_mode: &str, state: &str) -> &'static str {
    if state == "Fallback" && (previous_state != "Fallback" || previous_mode != HALL_MODE_NAME) {
        "fallback_entered"
    } else if state == "Primary" && previous_state == "Fallback" {
        "primary_recovered"
    } else if state == "Primary" && previous_state != "Primary" {
        "primary_acquired"
    } else if state == "Lost" && previous_state != "Lost" {
        "feedback_lost"
    } else {
        "none"
    }
}

fn optional_number(value: Option<f64>) -> String {
    value.map_or_else(String::new, |number| format!("{number:.9}"))
}

fn to_u32(value: u64, label: &str) -> Result<u32, SimulationContractError> {
    u32::try_from(value)
        .map_err(|_| SimulationContractError::Invalid(format!("{label} exceeds u32")))
}

fn to_nonzero_u32(value: u64, label: &str) -> Result<u32, SimulationContractError> {
    let value = to_u32(value, label)?;
    if value == 0 {
        invalid(format!("{label} must be non-zero"))
    } else {
        Ok(value)
    }
}

fn to_nonzero_u16(value: u64, label: &str) -> Result<u16, SimulationContractError> {
    let value = u16::try_from(value)
        .map_err(|_| SimulationContractError::Invalid(format!("{label} exceeds u16")))?;
    if value == 0 {
        invalid(format!("{label} must be non-zero"))
    } else {
        Ok(value)
    }
}

const SCENARIO_KEYS: &[&str] = &[
    "contract",
    "version",
    "scenario_id",
    "revision",
    "units",
    "profile_id",
    "profile_revision",
    "model_revision",
    "evaluation_ticks",
    "initial_mechanical_position_rad",
    "mechanical_velocity_rad_s",
    "router",
    "encoder",
    "hall",
    "cases",
];
const ROUTER_KEYS: &[&str] = &[
    "primary_mode",
    "backup_mode",
    "maximum_age_control_ticks",
    "acquire_good_samples",
    "loss_bad_samples",
    "recovery_good_samples",
];
const ENCODER_KEYS: &[&str] = &[
    "counts_per_revolution",
    "delay_control_ticks",
    "electrical_offset_rad",
];
const HALL_KEYS: &[&str] = &["delay_control_ticks", "sector_map"];
const CASE_KEYS: &[&str] = &[
    "case_id",
    "feature_enabled",
    "encoder_drop_enabled",
    "encoder_drop_start_tick",
    "encoder_drop_end_tick",
    "encoder_index_missing",
    "hall_wrong_order_enabled",
    "hall_wrong_order_start_tick",
    "hall_wrong_order_end_tick",
];

#[cfg(test)]
mod tests {
    use super::*;

    fn project_root() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../..")
            .canonicalize()
            .unwrap()
    }

    #[test]
    fn feedback_fault_matrix_covers_timing_numeric_and_fail_closed_paths() {
        let result = run_feedback_sensor_gate(project_root()).unwrap();
        assert_eq!(result.rows.len(), 80);
        assert!(result.rows.iter().any(|row| {
            row.case_id == "encoder_drop_fallback"
                && row.control_tick == 7
                && row.event == "fallback_entered"
        }));
        assert!(result.rows.iter().any(|row| {
            row.case_id == "encoder_drop_fallback"
                && row.control_tick == 12
                && row.event == "primary_recovered"
        }));
        assert!(result.rows.iter().any(|row| {
            row.case_id == "hall_wrong_order_and_primary_drop"
                && row.route_state == "Lost"
                && row.valid_flags == 0
        }));
    }

    #[test]
    fn malformed_or_noncanonical_feedback_scenario_is_rejected() {
        let path = project_root().join(FEEDBACK_SCENARIO_RELATIVE_PATH);
        let original = fs::read_to_string(path).unwrap();
        let missing = original.replacen("  \"evaluation_ticks\": 16,\n", "", 1);
        let value = parse_json_bytes(missing.as_bytes()).unwrap();
        assert!(parse_feedback_scenario(object(&value, "scenario").unwrap()).is_err());

        let duplicate = original.replacen(
            "\"sector_map\": [0, 1, 2, 3, 4, 5]",
            "\"sector_map\": [0, 1, 2, 3, 4, 4]",
            1,
        );
        let value = parse_json_bytes(duplicate.as_bytes()).unwrap();
        assert!(parse_feedback_scenario(object(&value, "scenario").unwrap()).is_err());
    }
}
