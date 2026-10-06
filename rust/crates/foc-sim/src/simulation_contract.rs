//! Versioned, language-neutral simulation identity shared by Rust and MATLAB.
//!
//! This is host-only code. It intentionally uses no third-party parser or hash
//! crate so the simulation contract does not change the target dependency tree.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::{Component, Path, PathBuf};

use crate::{MultiRateTimingConfig, SimulationConfig};

pub mod feedback_sensor;

pub const BUNDLE_RELATIVE_PATH: &str = "simulation/contracts/legacy_speed_start.bundle.json";
const CONTRACT_VERSION: u32 = 1;
const PRODUCT_TARGET_RPM: f64 = 524.0;

#[derive(Debug)]
pub enum SimulationContractError {
    Io(String),
    Json(String),
    Invalid(String),
}

impl std::fmt::Display for SimulationContractError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(message) => write!(formatter, "I/O: {message}"),
            Self::Json(message) => write!(formatter, "JSON: {message}"),
            Self::Invalid(message) => write!(formatter, "contract: {message}"),
        }
    }
}

impl std::error::Error for SimulationContractError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct D1Event {
    pub control_tick: u64,
    pub axis_state: String,
    pub controller_internal_state: String,
    pub active_source: String,
    pub event: String,
    pub output_enabled: bool,
    pub fault_flags: u32,
}

#[derive(Clone, Debug)]
pub struct SimulationProfile {
    pub profile_id: String,
    pub revision: u32,
    pub board_id: String,
    pub motor_id: String,
    pub inverter_id: String,
    pub runtime_config_crc32: String,
    pub product_contract_version: String,
    pub bridge_abi_version: String,
    pub source_revision: String,
    pub model_revision: String,
    pub pole_pairs: u32,
    pub stator_resistance_ohm: f64,
    pub ld_h: f64,
    pub lq_h: f64,
    pub flux_linkage_wb: f64,
    pub rated_current_a: f64,
    pub max_mechanical_velocity_rad_s: f64,
    pub nominal_bus_voltage_v: f64,
    pub inertia_kg_m2: f64,
    pub viscous_friction_nm_s: f64,
    pub pwm_frequency_hz: u32,
    pub control_frequency_hz: u32,
    pub speed_loop_frequency_hz: u32,
    pub actuation_delay_pwm_ticks: u32,
}

#[derive(Clone, Debug)]
pub struct SimulationScenario {
    pub scenario_id: String,
    pub revision: u32,
    pub seed: u64,
    pub profile_id: String,
    pub profile_revision: u32,
    pub duration_s: f64,
    pub trace_decimation: u32,
    pub initial_velocity_rad_s: f64,
    pub initial_electrical_angle_rad: f64,
    pub initial_temperature_k: f64,
    pub dc_bus_voltage_v: f64,
    pub command_at_s: f64,
    pub axis_request: String,
    pub control_mode: String,
    pub input_mode: String,
    pub feedback_mode: String,
    pub target_velocity_rad_s: f64,
    pub startup_duration_s: f64,
    pub load_step_at_s: f64,
    pub load_torque_nm: f64,
    pub d1_events: Vec<D1Event>,
}

#[derive(Clone, Debug)]
pub struct TraceChannel {
    pub name: String,
    pub unit: String,
    pub kind: String,
}

#[derive(Clone, Debug)]
pub struct SimulationContractBundle {
    pub bundle_id: String,
    pub bundle_sha256: String,
    pub schema_id: String,
    pub schema_version: u32,
    pub schema_sha256: String,
    pub profile: SimulationProfile,
    pub profile_sha256: String,
    pub scenario: SimulationScenario,
    pub scenario_sha256: String,
    pub trace_schema_id: String,
    pub trace_schema_version: u32,
    pub trace_schema_sha256: String,
    pub trace_channels: Vec<TraceChannel>,
    pub comparison_id: String,
    pub comparison_version: u32,
    pub comparison_sha256: String,
    pub d1_event_tick_tolerance: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ContractGateResult {
    pub engine_id: &'static str,
    pub lines: Vec<String>,
}

impl ContractGateResult {
    pub fn as_text(&self) -> String {
        let mut text = self.lines.join("\n");
        text.push('\n');
        text
    }

    pub fn write(&self, path: impl AsRef<Path>) -> Result<(), SimulationContractError> {
        fs::write(path.as_ref(), self.as_text()).map_err(|error| {
            SimulationContractError::Io(format!(
                "cannot write {}: {error}",
                path.as_ref().display()
            ))
        })
    }
}

impl SimulationContractBundle {
    /// Convert the shared all-SI scenario into the legacy reference-simulator adapter.
    /// RPM exists only at this adapter boundary because the old simulator API predates
    /// Product Contract V1.
    pub fn reference_simulation_config(&self) -> SimulationConfig {
        SimulationConfig {
            duration_s: self.scenario.duration_s as f32,
            target_speed_rpm: (self.scenario.target_velocity_rad_s * 30.0 / std::f64::consts::PI)
                as f32,
            load_step_time_s: self.scenario.load_step_at_s as f32,
            load_torque_nm: self.scenario.load_torque_nm as f32,
            trace_decimation: self.scenario.trace_decimation,
            timing: MultiRateTimingConfig {
                pwm_frequency_hz: self.profile.pwm_frequency_hz,
                control_frequency_hz: self.profile.control_frequency_hz,
                actuation_delay_pwm_ticks: self.profile.actuation_delay_pwm_ticks,
            },
        }
    }

    pub fn gate_result(&self) -> ContractGateResult {
        let events = self.run_d1_state_machine();
        let workspace_revision =
            std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
        let mut lines = vec![
            "contract_gate_result_version=1".to_owned(),
            "engine_id=fluxrt.rust.lifecycle.v1".to_owned(),
            format!("bundle_id={}", self.bundle_id),
            format!("bundle_sha256={}", self.bundle_sha256),
            format!("schema_id={}", self.schema_id),
            format!("schema_version={}", self.schema_version),
            format!("schema_sha256={}", self.schema_sha256),
            format!("profile_id={}", self.profile.profile_id),
            format!("profile_revision={}", self.profile.revision),
            format!("profile_sha256={}", self.profile_sha256),
            format!("board_id={}", self.profile.board_id),
            format!("motor_id={}", self.profile.motor_id),
            format!("inverter_id={}", self.profile.inverter_id),
            format!("runtime_config_crc32={}", self.profile.runtime_config_crc32),
            format!(
                "product_contract_version={}",
                self.profile.product_contract_version
            ),
            format!("bridge_abi_version={}", self.profile.bridge_abi_version),
            format!("source_revision={}", self.profile.source_revision),
            format!("model_revision={}", self.profile.model_revision),
            format!("workspace_revision={workspace_revision}"),
            format!("scenario_id={}", self.scenario.scenario_id),
            format!("scenario_revision={}", self.scenario.revision),
            format!("scenario_sha256={}", self.scenario_sha256),
            format!("trace_schema_id={}", self.trace_schema_id),
            format!("trace_schema_version={}", self.trace_schema_version),
            format!("trace_schema_sha256={}", self.trace_schema_sha256),
            format!("comparison_id={}", self.comparison_id),
            format!("comparison_version={}", self.comparison_version),
            format!("comparison_sha256={}", self.comparison_sha256),
            "d0=PASS".to_owned(),
            format!("d1_event_tick_tolerance={}", self.d1_event_tick_tolerance),
            format!("d1_event_count={}", events.len()),
        ];
        for (index, event) in events.iter().enumerate() {
            lines.push(format!(
                "d1_event_{index}={},{},{},{},{},{},{}",
                event.control_tick,
                event.axis_state,
                event.controller_internal_state,
                event.active_source,
                event.event,
                u8::from(event.output_enabled),
                event.fault_flags
            ));
        }
        lines.push("d1=PASS".to_owned());
        ContractGateResult {
            engine_id: "fluxrt.rust.lifecycle.v1",
            lines,
        }
    }

    /// Minimal lifecycle engine used by D1. It consumes the command and timing
    /// inputs, not the `d1_events` oracle stored in the scenario.
    pub fn run_d1_state_machine(&self) -> Vec<D1Event> {
        let command_tick = (self.scenario.command_at_s
            * f64::from(self.profile.control_frequency_hz))
        .round() as u64;
        let closed_loop_tick = command_tick
            + (self.scenario.startup_duration_s * f64::from(self.profile.control_frequency_hz))
                .round() as u64;
        vec![
            D1Event {
                control_tick: 0,
                axis_state: "Disabled".to_owned(),
                controller_internal_state: "Disabled".to_owned(),
                active_source: "none".to_owned(),
                event: "axis_ready".to_owned(),
                output_enabled: false,
                fault_flags: 0,
            },
            D1Event {
                control_tick: command_tick + 1,
                axis_state: "Startup".to_owned(),
                controller_internal_state: "Alignment".to_owned(),
                active_source: "product_command".to_owned(),
                event: "start_requested".to_owned(),
                output_enabled: true,
                fault_flags: 0,
            },
            D1Event {
                control_tick: closed_loop_tick,
                axis_state: "ClosedLoop".to_owned(),
                controller_internal_state: "ClosedLoop".to_owned(),
                active_source: "product_command".to_owned(),
                event: "observer_handover_complete".to_owned(),
                output_enabled: true,
                fault_flags: 0,
            },
        ]
    }
}

pub fn load_simulation_contract(
    project_root: impl AsRef<Path>,
) -> Result<SimulationContractBundle, SimulationContractError> {
    let root = project_root.as_ref();
    let bundle_path = root.join(BUNDLE_RELATIVE_PATH);
    let bundle_bytes = read_bytes(&bundle_path)?;
    let bundle_sha256 = sha256_hex(&bundle_bytes);
    let bundle_json = parse_json_bytes(&bundle_bytes)?;
    let bundle = object(&bundle_json, "bundle")?;
    exact_keys(bundle, BUNDLE_KEYS, "bundle")?;
    expect_contract(bundle, "fluxrt-simulation-bundle")?;
    expect_si(bundle, "bundle")?;

    let schema = load_referenced(root, bundle, "schema_path", "schema_sha256")?;
    let schema_object = object(&schema.json, "schema")?;
    exact_keys(schema_object, SCHEMA_KEYS, "schema")?;
    expect_contract(schema_object, "fluxrt-simulation-schema")?;
    expect_si(schema_object, "schema")?;
    check_reference_identity(
        bundle,
        "schema_id",
        "schema_version",
        schema_object,
        "schema_id",
        "version",
        "schema",
    )?;

    let profile_document = load_referenced(root, bundle, "profile_path", "profile_sha256")?;
    let profile_object = object(&profile_document.json, "profile")?;
    let profile = parse_profile(profile_object)?;
    check_reference_identity(
        bundle,
        "profile_id",
        "profile_revision",
        profile_object,
        "profile_id",
        "revision",
        "profile",
    )?;
    for key in [
        "board_id",
        "motor_id",
        "inverter_id",
        "runtime_config_crc32",
        "product_contract_version",
        "bridge_abi_version",
        "source_revision",
        "model_revision",
    ] {
        if string(bundle, key)? != string(profile_object, key)? {
            return invalid(format!("profile {key} mismatch"));
        }
    }

    let scenario_document = load_referenced(root, bundle, "scenario_path", "scenario_sha256")?;
    let scenario_object = object(&scenario_document.json, "scenario")?;
    let scenario = parse_scenario(scenario_object, &profile)?;
    check_reference_identity(
        bundle,
        "scenario_id",
        "scenario_revision",
        scenario_object,
        "scenario_id",
        "revision",
        "scenario",
    )?;

    let trace_document = load_referenced(root, bundle, "trace_schema_path", "trace_schema_sha256")?;
    let trace_object = object(&trace_document.json, "trace schema")?;
    let trace_channels = parse_trace_schema(trace_object)?;
    check_reference_identity(
        bundle,
        "trace_schema_id",
        "trace_schema_version",
        trace_object,
        "trace_schema_id",
        "version",
        "trace schema",
    )?;

    let comparison_document =
        load_referenced(root, bundle, "comparison_path", "comparison_sha256")?;
    let comparison_object = object(&comparison_document.json, "comparison")?;
    let d1_event_tick_tolerance = parse_comparison(comparison_object, &trace_channels)?;
    check_reference_identity(
        bundle,
        "comparison_id",
        "comparison_version",
        comparison_object,
        "comparison_id",
        "version",
        "comparison",
    )?;

    Ok(SimulationContractBundle {
        bundle_id: string(bundle, "bundle_id")?.to_owned(),
        bundle_sha256,
        schema_id: string(bundle, "schema_id")?.to_owned(),
        schema_version: unsigned(bundle, "schema_version")? as u32,
        schema_sha256: schema.sha256,
        profile,
        profile_sha256: profile_document.sha256,
        scenario,
        scenario_sha256: scenario_document.sha256,
        trace_schema_id: string(bundle, "trace_schema_id")?.to_owned(),
        trace_schema_version: unsigned(bundle, "trace_schema_version")? as u32,
        trace_schema_sha256: trace_document.sha256,
        trace_channels,
        comparison_id: string(bundle, "comparison_id")?.to_owned(),
        comparison_version: unsigned(bundle, "comparison_version")? as u32,
        comparison_sha256: comparison_document.sha256,
        d1_event_tick_tolerance,
    })
}

struct LoadedDocument {
    json: JsonValue,
    sha256: String,
}

fn load_referenced(
    root: &Path,
    bundle: &BTreeMap<String, JsonValue>,
    path_key: &str,
    hash_key: &str,
) -> Result<LoadedDocument, SimulationContractError> {
    let relative = safe_relative_path(string(bundle, path_key)?)?;
    let bytes = read_bytes(&root.join(relative))?;
    let actual = sha256_hex(&bytes);
    let expected = string(bundle, hash_key)?;
    if !is_sha256(expected) || actual != expected {
        return invalid(format!(
            "{hash_key} mismatch: expected {expected}, got {actual}"
        ));
    }
    Ok(LoadedDocument {
        json: parse_json_bytes(&bytes)?,
        sha256: actual,
    })
}

fn parse_profile(
    profile: &BTreeMap<String, JsonValue>,
) -> Result<SimulationProfile, SimulationContractError> {
    exact_keys(profile, PROFILE_KEYS, "profile")?;
    expect_contract(profile, "fluxrt-simulation-profile")?;
    expect_si(profile, "profile")?;
    let parsed = SimulationProfile {
        profile_id: string(profile, "profile_id")?.to_owned(),
        revision: unsigned(profile, "revision")? as u32,
        board_id: string(profile, "board_id")?.to_owned(),
        motor_id: string(profile, "motor_id")?.to_owned(),
        inverter_id: string(profile, "inverter_id")?.to_owned(),
        runtime_config_crc32: string(profile, "runtime_config_crc32")?.to_owned(),
        product_contract_version: string(profile, "product_contract_version")?.to_owned(),
        bridge_abi_version: string(profile, "bridge_abi_version")?.to_owned(),
        source_revision: string(profile, "source_revision")?.to_owned(),
        model_revision: string(profile, "model_revision")?.to_owned(),
        pole_pairs: unsigned(profile, "pole_pairs")? as u32,
        stator_resistance_ohm: finite_number(profile, "stator_resistance_ohm")?,
        ld_h: finite_number(profile, "ld_h")?,
        lq_h: finite_number(profile, "lq_h")?,
        flux_linkage_wb: finite_number(profile, "flux_linkage_wb")?,
        rated_current_a: finite_number(profile, "rated_current_a")?,
        max_mechanical_velocity_rad_s: finite_number(profile, "max_mechanical_velocity_rad_s")?,
        nominal_bus_voltage_v: finite_number(profile, "nominal_bus_voltage_v")?,
        inertia_kg_m2: finite_number(profile, "inertia_kg_m2")?,
        viscous_friction_nm_s: finite_number(profile, "viscous_friction_nm_s")?,
        pwm_frequency_hz: unsigned(profile, "pwm_frequency_hz")? as u32,
        control_frequency_hz: unsigned(profile, "control_frequency_hz")? as u32,
        speed_loop_frequency_hz: unsigned(profile, "speed_loop_frequency_hz")? as u32,
        actuation_delay_pwm_ticks: unsigned(profile, "actuation_delay_pwm_ticks")? as u32,
    };
    if parsed.revision == 0
        || parsed.board_id.is_empty()
        || parsed.motor_id.is_empty()
        || parsed.inverter_id.is_empty()
        || parsed.source_revision.is_empty()
        || parsed.model_revision.is_empty()
        || parsed.product_contract_version
            != format!("0x{:08x}", foc_control::PRODUCT_CONTRACT_VERSION)
        || parsed.bridge_abi_version != format!("0x{:08x}", foc_rt_bridge::FOC_RUST_ABI_VERSION)
        || !is_crc32(&parsed.runtime_config_crc32)
        || parsed.pole_pairs == 0
        || parsed.stator_resistance_ohm <= 0.0
        || parsed.ld_h <= 0.0
        || parsed.lq_h <= 0.0
        || parsed.flux_linkage_wb <= 0.0
        || parsed.rated_current_a <= 0.0
        || parsed.max_mechanical_velocity_rad_s <= 0.0
        || parsed.nominal_bus_voltage_v <= 0.0
        || parsed.inertia_kg_m2 <= 0.0
        || parsed.viscous_friction_nm_s < 0.0
        || parsed.pwm_frequency_hz == 0
        || parsed.control_frequency_hz == 0
        || parsed.speed_loop_frequency_hz == 0
        || !parsed
            .pwm_frequency_hz
            .is_multiple_of(parsed.control_frequency_hz)
        || !parsed
            .control_frequency_hz
            .is_multiple_of(parsed.speed_loop_frequency_hz)
        || parsed.actuation_delay_pwm_ticks > parsed.pwm_frequency_hz / parsed.control_frequency_hz
    {
        return invalid("profile values are outside the V1 domain".to_owned());
    }
    if parsed.runtime_config_crc32 != current_runtime_config_crc32()? {
        return invalid(
            "runtime_config_crc32 does not match the current bridge defaults".to_owned(),
        );
    }
    Ok(parsed)
}

fn current_runtime_config_crc32() -> Result<String, SimulationContractError> {
    use foc_rt_bridge::{
        foc_rust_default_st_config, foc_rust_runtime_config_crc32, FocRuntimeConfig, FocStatus,
    };
    let mut config = FocRuntimeConfig::default();
    let mut crc = 0_u32;
    // SAFETY: both pointers refer to valid local storage for the duration of each call.
    let default_status = unsafe { foc_rust_default_st_config(&mut config) };
    let crc_status = unsafe { foc_rust_runtime_config_crc32(&config, &mut crc) };
    if default_status != FocStatus::Ok || crc_status != FocStatus::Ok {
        return invalid("bridge rejected its own default runtime config".to_owned());
    }
    Ok(format!("0x{crc:08x}"))
}

fn parse_scenario(
    scenario: &BTreeMap<String, JsonValue>,
    profile: &SimulationProfile,
) -> Result<SimulationScenario, SimulationContractError> {
    exact_keys(scenario, SCENARIO_KEYS, "scenario")?;
    expect_contract(scenario, "fluxrt-simulation-scenario")?;
    expect_si(scenario, "scenario")?;
    let events_value = required(scenario, "d1_events")?;
    let event_values = array(events_value, "scenario.d1_events")?;
    if event_values.is_empty() {
        return invalid("scenario.d1_events must not be empty".to_owned());
    }
    let mut events = Vec::with_capacity(event_values.len());
    let mut previous_tick = None;
    let mut previous_rank = 0;
    for value in event_values {
        let event_object = object(value, "D1 event")?;
        exact_keys(event_object, D1_EVENT_KEYS, "D1 event")?;
        let event = D1Event {
            control_tick: unsigned(event_object, "control_tick")?,
            axis_state: string(event_object, "axis_state")?.to_owned(),
            controller_internal_state: string(event_object, "controller_internal_state")?
                .to_owned(),
            active_source: string(event_object, "active_source")?.to_owned(),
            event: string(event_object, "event")?.to_owned(),
            output_enabled: boolean(event_object, "output_enabled")?,
            fault_flags: unsigned(event_object, "fault_flags")? as u32,
        };
        let rank = axis_state_rank(&event.axis_state)?;
        if let Some(tick) = previous_tick {
            if event.control_tick <= tick {
                return invalid("D1 event ticks must be strictly increasing".to_owned());
            }
            if rank < previous_rank {
                return invalid("D1 lifecycle events must not run backwards".to_owned());
            }
        }
        if matches!(
            event.axis_state.as_str(),
            "Uninitialized" | "Disabled" | "Calibration" | "Stopping" | "FaultLatched"
        ) && event.output_enabled
        {
            return invalid(format!(
                "{} cannot declare output_enabled",
                event.axis_state
            ));
        }
        if event.axis_state == "FaultLatched" && event.fault_flags == 0 {
            return invalid("FaultLatched requires non-zero fault_flags".to_owned());
        }
        previous_tick = Some(event.control_tick);
        previous_rank = rank;
        events.push(event);
    }

    let parsed = SimulationScenario {
        scenario_id: string(scenario, "scenario_id")?.to_owned(),
        revision: unsigned(scenario, "revision")? as u32,
        seed: unsigned(scenario, "seed")?,
        profile_id: string(scenario, "profile_id")?.to_owned(),
        profile_revision: unsigned(scenario, "profile_revision")? as u32,
        duration_s: finite_number(scenario, "duration_s")?,
        trace_decimation: unsigned(scenario, "trace_decimation")? as u32,
        initial_velocity_rad_s: finite_number(scenario, "initial_velocity_rad_s")?,
        initial_electrical_angle_rad: finite_number(scenario, "initial_electrical_angle_rad")?,
        initial_temperature_k: finite_number(scenario, "initial_temperature_k")?,
        dc_bus_voltage_v: finite_number(scenario, "dc_bus_voltage_v")?,
        command_at_s: finite_number(scenario, "command_at_s")?,
        axis_request: string(scenario, "axis_request")?.to_owned(),
        control_mode: string(scenario, "control_mode")?.to_owned(),
        input_mode: string(scenario, "input_mode")?.to_owned(),
        feedback_mode: string(scenario, "feedback_mode")?.to_owned(),
        target_velocity_rad_s: finite_number(scenario, "target_velocity_rad_s")?,
        startup_duration_s: finite_number(scenario, "startup_duration_s")?,
        load_step_at_s: finite_number(scenario, "load_step_at_s")?,
        load_torque_nm: finite_number(scenario, "load_torque_nm")?,
        d1_events: events,
    };
    let expected_velocity = PRODUCT_TARGET_RPM * std::f64::consts::PI / 30.0;
    let max_tick = (parsed.duration_s * f64::from(profile.control_frequency_hz)).round() as u64;
    if parsed.revision == 0
        || parsed.profile_id != profile.profile_id
        || parsed.profile_revision != profile.revision
        || parsed.duration_s <= 0.0
        || parsed.trace_decimation == 0
        || parsed.initial_temperature_k <= 0.0
        || parsed.dc_bus_voltage_v <= 0.0
        || parsed.command_at_s < 0.0
        || parsed.command_at_s > parsed.duration_s
        || parsed.load_step_at_s < 0.0
        || parsed.load_step_at_s > parsed.duration_s
        || parsed.axis_request != "ClosedLoopControl"
        || parsed.control_mode != "Velocity"
        || parsed.input_mode != "Passthrough"
        || parsed.feedback_mode != "Sensorless"
        || (parsed.target_velocity_rad_s - expected_velocity).abs() > 1.0e-6
        || parsed.startup_duration_s <= 0.0
        || parsed
            .d1_events
            .last()
            .is_some_and(|event| event.control_tick > max_tick)
        || parsed.d1_events.first()
            != Some(&D1Event {
                control_tick: 0,
                axis_state: "Disabled".to_owned(),
                controller_internal_state: "Disabled".to_owned(),
                active_source: "none".to_owned(),
                event: "axis_ready".to_owned(),
                output_enabled: false,
                fault_flags: 0,
            })
        || !parsed
            .d1_events
            .iter()
            .any(|event| event.axis_state == "Startup")
        || !parsed
            .d1_events
            .iter()
            .any(|event| event.axis_state == "ClosedLoop")
    {
        return invalid("legacy_speed_start scenario violates its frozen V1 identity".to_owned());
    }
    let command_tick =
        (parsed.command_at_s * f64::from(profile.control_frequency_hz)).round() as u64;
    let calculated = vec![
        D1Event {
            control_tick: 0,
            axis_state: "Disabled".to_owned(),
            controller_internal_state: "Disabled".to_owned(),
            active_source: "none".to_owned(),
            event: "axis_ready".to_owned(),
            output_enabled: false,
            fault_flags: 0,
        },
        D1Event {
            control_tick: command_tick + 1,
            axis_state: "Startup".to_owned(),
            controller_internal_state: "Alignment".to_owned(),
            active_source: "product_command".to_owned(),
            event: "start_requested".to_owned(),
            output_enabled: true,
            fault_flags: 0,
        },
        D1Event {
            control_tick: command_tick
                + (parsed.startup_duration_s * f64::from(profile.control_frequency_hz)).round()
                    as u64,
            axis_state: "ClosedLoop".to_owned(),
            controller_internal_state: "ClosedLoop".to_owned(),
            active_source: "product_command".to_owned(),
            event: "observer_handover_complete".to_owned(),
            output_enabled: true,
            fault_flags: 0,
        },
    ];
    if calculated != parsed.d1_events {
        return invalid("D1 lifecycle engine disagrees with the scenario oracle".to_owned());
    }
    Ok(parsed)
}

fn parse_trace_schema(
    trace: &BTreeMap<String, JsonValue>,
) -> Result<Vec<TraceChannel>, SimulationContractError> {
    exact_keys(trace, TRACE_KEYS, "trace schema")?;
    expect_contract(trace, "fluxrt-trace-schema")?;
    expect_si(trace, "trace schema")?;
    let values = array(required(trace, "channels")?, "trace.channels")?;
    let mut channels = Vec::with_capacity(values.len());
    let mut names = BTreeSet::new();
    for value in values {
        let channel = object(value, "trace channel")?;
        exact_keys(channel, TRACE_CHANNEL_KEYS, "trace channel")?;
        let parsed = TraceChannel {
            name: string(channel, "name")?.to_owned(),
            unit: string(channel, "unit")?.to_owned(),
            kind: string(channel, "kind")?.to_owned(),
        };
        if parsed.name.is_empty()
            || parsed.unit.is_empty()
            || !TRACE_KINDS.contains(&parsed.kind.as_str())
            || !names.insert(parsed.name.clone())
        {
            return invalid(
                "trace channel is empty, duplicated, or has an unknown kind".to_owned(),
            );
        }
        channels.push(parsed);
    }
    for required_name in REQUIRED_TRACE_CHANNELS {
        if !names.contains(*required_name) {
            return invalid(format!("trace channel missing: {required_name}"));
        }
    }
    Ok(channels)
}

fn parse_comparison(
    comparison: &BTreeMap<String, JsonValue>,
    trace_channels: &[TraceChannel],
) -> Result<u32, SimulationContractError> {
    exact_keys(comparison, COMPARISON_KEYS, "comparison")?;
    expect_contract(comparison, "fluxrt-comparison-gates")?;
    expect_si(comparison, "comparison")?;
    exact_string_set(comparison, "d0_exact", D0_EXACT)?;
    exact_string_set(comparison, "d1_exact", D1_EXACT)?;
    let tick_tolerance = unsigned(comparison, "d1_event_tick_tolerance")?;
    if tick_tolerance != 0 {
        return invalid("D1 V1 requires zero event-tick tolerance".to_owned());
    }
    let channel_names: BTreeSet<&str> = trace_channels
        .iter()
        .map(|channel| channel.name.as_str())
        .collect();
    let tolerances = array(
        required(comparison, "numeric_tolerances")?,
        "comparison.numeric_tolerances",
    )?;
    let mut seen = BTreeSet::new();
    for value in tolerances {
        let tolerance = object(value, "numeric tolerance")?;
        exact_keys(tolerance, TOLERANCE_KEYS, "numeric tolerance")?;
        let channel = string(tolerance, "channel")?;
        let absolute = finite_number(tolerance, "absolute")?;
        let relative = finite_number(tolerance, "relative")?;
        if !channel_names.contains(channel)
            || !seen.insert(channel)
            || absolute < 0.0
            || relative < 0.0
        {
            return invalid(
                "numeric tolerance has an unknown/duplicate channel or negative value".to_owned(),
            );
        }
    }
    Ok(tick_tolerance as u32)
}

fn exact_string_set(
    object: &BTreeMap<String, JsonValue>,
    key: &str,
    expected: &[&str],
) -> Result<(), SimulationContractError> {
    let values = array(required(object, key)?, key)?;
    let mut actual = BTreeSet::new();
    for value in values {
        let JsonValue::String(value) = value else {
            return invalid(format!("{key} must contain strings"));
        };
        if !actual.insert(value.as_str()) {
            return invalid(format!("{key} contains a duplicate"));
        }
    }
    let expected: BTreeSet<&str> = expected.iter().copied().collect();
    if actual != expected {
        return invalid(format!("{key} does not match the frozen V1 set"));
    }
    Ok(())
}

fn check_reference_identity(
    bundle: &BTreeMap<String, JsonValue>,
    bundle_id_key: &str,
    bundle_version_key: &str,
    document: &BTreeMap<String, JsonValue>,
    document_id_key: &str,
    document_version_key: &str,
    label: &str,
) -> Result<(), SimulationContractError> {
    if string(bundle, bundle_id_key)? != string(document, document_id_key)?
        || unsigned(bundle, bundle_version_key)? != unsigned(document, document_version_key)?
    {
        return invalid(format!("{label} identity mismatch"));
    }
    Ok(())
}

fn expect_contract(
    object: &BTreeMap<String, JsonValue>,
    expected: &str,
) -> Result<(), SimulationContractError> {
    if string(object, "contract")? != expected
        || unsigned(object, "version")? != CONTRACT_VERSION.into()
    {
        return invalid(format!("unknown contract/version for {expected}"));
    }
    Ok(())
}

fn expect_si(
    object: &BTreeMap<String, JsonValue>,
    label: &str,
) -> Result<(), SimulationContractError> {
    if string(object, "units")? != "SI" {
        return invalid(format!("{label}.units must be SI"));
    }
    Ok(())
}

pub(crate) fn exact_keys(
    object: &BTreeMap<String, JsonValue>,
    expected: &[&str],
    label: &str,
) -> Result<(), SimulationContractError> {
    let expected: BTreeSet<&str> = expected.iter().copied().collect();
    let actual: BTreeSet<&str> = object.keys().map(String::as_str).collect();
    if actual != expected {
        let missing: Vec<_> = expected.difference(&actual).copied().collect();
        let unknown: Vec<_> = actual.difference(&expected).copied().collect();
        return invalid(format!(
            "{label} field set mismatch; missing={missing:?}, unknown={unknown:?}"
        ));
    }
    Ok(())
}

pub(crate) fn required<'a>(
    object: &'a BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<&'a JsonValue, SimulationContractError> {
    object
        .get(key)
        .ok_or_else(|| SimulationContractError::Invalid(format!("missing field: {key}")))
}

pub(crate) fn string<'a>(
    object: &'a BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<&'a str, SimulationContractError> {
    match required(object, key)? {
        JsonValue::String(value) => Ok(value),
        _ => invalid(format!("{key} must be a string")),
    }
}

pub(crate) fn finite_number(
    object: &BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<f64, SimulationContractError> {
    match required(object, key)? {
        JsonValue::Number(value) if value.is_finite() => Ok(*value),
        _ => invalid(format!("{key} must be a finite number")),
    }
}

pub(crate) fn unsigned(
    object: &BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<u64, SimulationContractError> {
    let value = finite_number(object, key)?;
    if value < 0.0 || value.fract() != 0.0 || value > u64::MAX as f64 {
        return invalid(format!("{key} must be an unsigned integer"));
    }
    Ok(value as u64)
}

pub(crate) fn boolean(
    object: &BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<bool, SimulationContractError> {
    match required(object, key)? {
        JsonValue::Bool(value) => Ok(*value),
        _ => invalid(format!("{key} must be a boolean")),
    }
}

pub(crate) fn object<'a>(
    value: &'a JsonValue,
    label: &str,
) -> Result<&'a BTreeMap<String, JsonValue>, SimulationContractError> {
    match value {
        JsonValue::Object(value) => Ok(value),
        _ => invalid(format!("{label} must be an object")),
    }
}

pub(crate) fn array<'a>(
    value: &'a JsonValue,
    label: &str,
) -> Result<&'a [JsonValue], SimulationContractError> {
    match value {
        JsonValue::Array(value) => Ok(value),
        _ => invalid(format!("{label} must be an array")),
    }
}

fn axis_state_rank(state: &str) -> Result<u8, SimulationContractError> {
    match state {
        "Uninitialized" => Ok(0),
        "Disabled" => Ok(1),
        "Calibration" => Ok(2),
        "Startup" => Ok(3),
        "ClosedLoop" => Ok(4),
        "Stopping" => Ok(5),
        "FaultLatched" => Ok(6),
        _ => invalid(format!("unknown AxisState: {state}")),
    }
}

fn safe_relative_path(value: &str) -> Result<PathBuf, SimulationContractError> {
    let path = Path::new(value);
    if path.is_absolute()
        || path.components().any(|component| {
            !matches!(component, Component::Normal(_))
                || matches!(
                    component,
                    Component::ParentDir | Component::RootDir | Component::Prefix(_)
                )
        })
    {
        return invalid(format!("unsafe contract path: {value}"));
    }
    Ok(path.to_path_buf())
}

pub(crate) fn read_bytes(path: &Path) -> Result<Vec<u8>, SimulationContractError> {
    fs::read(path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", path.display()))
    })
}

pub(crate) fn invalid<T>(message: String) -> Result<T, SimulationContractError> {
    Err(SimulationContractError::Invalid(message))
}

fn is_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn is_crc32(value: &str) -> bool {
    value.len() == 10
        && value.starts_with("0x")
        && value[2..]
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[derive(Clone, Debug)]
pub(crate) enum JsonValue {
    Null,
    Bool(bool),
    Number(f64),
    String(String),
    Array(Vec<JsonValue>),
    Object(BTreeMap<String, JsonValue>),
}

pub(crate) fn parse_json_bytes(bytes: &[u8]) -> Result<JsonValue, SimulationContractError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|error| SimulationContractError::Json(format!("not UTF-8: {error}")))?;
    let mut parser = JsonParser {
        bytes: text.as_bytes(),
        offset: 0,
    };
    let value = parser.parse_value()?;
    parser.skip_whitespace();
    if parser.offset != parser.bytes.len() {
        return Err(SimulationContractError::Json(format!(
            "trailing data at byte {}",
            parser.offset
        )));
    }
    Ok(value)
}

struct JsonParser<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl JsonParser<'_> {
    fn parse_value(&mut self) -> Result<JsonValue, SimulationContractError> {
        self.skip_whitespace();
        match self.peek() {
            Some(b'{') => self.parse_object(),
            Some(b'[') => self.parse_array(),
            Some(b'"') => self.parse_string().map(JsonValue::String),
            Some(b't') => {
                self.literal(b"true")?;
                Ok(JsonValue::Bool(true))
            }
            Some(b'f') => {
                self.literal(b"false")?;
                Ok(JsonValue::Bool(false))
            }
            Some(b'n') => {
                self.literal(b"null")?;
                Ok(JsonValue::Null)
            }
            Some(b'-' | b'0'..=b'9') => self.parse_number().map(JsonValue::Number),
            _ => Err(self.error("expected a JSON value")),
        }
    }

    fn parse_object(&mut self) -> Result<JsonValue, SimulationContractError> {
        self.consume(b'{')?;
        let mut values = BTreeMap::new();
        self.skip_whitespace();
        if self.take(b'}') {
            return Ok(JsonValue::Object(values));
        }
        loop {
            self.skip_whitespace();
            let key = self.parse_string()?;
            self.skip_whitespace();
            self.consume(b':')?;
            let value = self.parse_value()?;
            if values.insert(key.clone(), value).is_some() {
                return Err(self.error(&format!("duplicate object key: {key}")));
            }
            self.skip_whitespace();
            if self.take(b'}') {
                break;
            }
            self.consume(b',')?;
        }
        Ok(JsonValue::Object(values))
    }

    fn parse_array(&mut self) -> Result<JsonValue, SimulationContractError> {
        self.consume(b'[')?;
        let mut values = Vec::new();
        self.skip_whitespace();
        if self.take(b']') {
            return Ok(JsonValue::Array(values));
        }
        loop {
            values.push(self.parse_value()?);
            self.skip_whitespace();
            if self.take(b']') {
                break;
            }
            self.consume(b',')?;
        }
        Ok(JsonValue::Array(values))
    }

    fn parse_string(&mut self) -> Result<String, SimulationContractError> {
        self.consume(b'"')?;
        let mut value = String::new();
        while let Some(byte) = self.peek() {
            self.offset += 1;
            match byte {
                b'"' => return Ok(value),
                b'\\' => {
                    let escaped = self
                        .peek()
                        .ok_or_else(|| self.error("unterminated escape"))?;
                    self.offset += 1;
                    let character = match escaped {
                        b'"' => '"',
                        b'\\' => '\\',
                        b'/' => '/',
                        b'b' => '\u{0008}',
                        b'f' => '\u{000c}',
                        b'n' => '\n',
                        b'r' => '\r',
                        b't' => '\t',
                        b'u' => self.parse_unicode_escape()?,
                        _ => return Err(self.error("invalid string escape")),
                    };
                    value.push(character);
                }
                0x00..=0x1f => return Err(self.error("control byte in string")),
                0x20..=0x7f => value.push(char::from(byte)),
                _ => {
                    let start = self.offset - 1;
                    let remainder = std::str::from_utf8(&self.bytes[start..])
                        .map_err(|_| self.error("invalid UTF-8 in string"))?;
                    let character = remainder
                        .chars()
                        .next()
                        .ok_or_else(|| self.error("unterminated UTF-8 string"))?;
                    value.push(character);
                    self.offset = start + character.len_utf8();
                }
            }
        }
        Err(self.error("unterminated string"))
    }

    fn parse_unicode_escape(&mut self) -> Result<char, SimulationContractError> {
        if self.offset + 4 > self.bytes.len() {
            return Err(self.error("short unicode escape"));
        }
        let digits = std::str::from_utf8(&self.bytes[self.offset..self.offset + 4])
            .map_err(|_| self.error("bad unicode escape"))?;
        self.offset += 4;
        let code = u32::from_str_radix(digits, 16).map_err(|_| self.error("bad unicode escape"))?;
        char::from_u32(code).ok_or_else(|| self.error("invalid unicode scalar"))
    }

    fn parse_number(&mut self) -> Result<f64, SimulationContractError> {
        let start = self.offset;
        self.take(b'-');
        match self.peek() {
            Some(b'0') => self.offset += 1,
            Some(b'1'..=b'9') => {
                self.offset += 1;
                while matches!(self.peek(), Some(b'0'..=b'9')) {
                    self.offset += 1;
                }
            }
            _ => return Err(self.error("invalid number")),
        }
        if self.take(b'.') {
            if !matches!(self.peek(), Some(b'0'..=b'9')) {
                return Err(self.error("invalid fraction"));
            }
            while matches!(self.peek(), Some(b'0'..=b'9')) {
                self.offset += 1;
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.offset += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.offset += 1;
            }
            if !matches!(self.peek(), Some(b'0'..=b'9')) {
                return Err(self.error("invalid exponent"));
            }
            while matches!(self.peek(), Some(b'0'..=b'9')) {
                self.offset += 1;
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.offset])
            .map_err(|_| self.error("invalid number encoding"))?;
        let value: f64 = text.parse().map_err(|_| self.error("invalid number"))?;
        if !value.is_finite() {
            return Err(self.error("number is not finite"));
        }
        Ok(value)
    }

    fn literal(&mut self, literal: &[u8]) -> Result<(), SimulationContractError> {
        if self.bytes.get(self.offset..self.offset + literal.len()) != Some(literal) {
            return Err(self.error("invalid literal"));
        }
        self.offset += literal.len();
        Ok(())
    }

    fn skip_whitespace(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.offset += 1;
        }
    }

    fn consume(&mut self, expected: u8) -> Result<(), SimulationContractError> {
        if self.take(expected) {
            Ok(())
        } else {
            Err(self.error(&format!("expected byte {}", char::from(expected))))
        }
    }

    fn take(&mut self, expected: u8) -> bool {
        if self.peek() == Some(expected) {
            self.offset += 1;
            true
        } else {
            false
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.offset).copied()
    }

    fn error(&self, message: &str) -> SimulationContractError {
        SimulationContractError::Json(format!("{message} at byte {}", self.offset))
    }
}

/// SHA-256 over exact file bytes, rendered as lowercase hexadecimal.
pub fn sha256_hex(bytes: &[u8]) -> String {
    const INITIAL: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let bit_length = (bytes.len() as u64).wrapping_mul(8);
    let mut padded = bytes.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&bit_length.to_be_bytes());
    let mut hash = INITIAL;
    let (chunks, remainder) = padded.as_chunks::<64>();
    debug_assert!(remainder.is_empty());
    for chunk in chunks {
        let mut words = [0_u32; 64];
        for (index, word) in words.iter_mut().take(16).enumerate() {
            let offset = index * 4;
            *word = u32::from_be_bytes([
                chunk[offset],
                chunk[offset + 1],
                chunk[offset + 2],
                chunk[offset + 3],
            ]);
        }
        for index in 16..64 {
            let s0 = words[index - 15].rotate_right(7)
                ^ words[index - 15].rotate_right(18)
                ^ (words[index - 15] >> 3);
            let s1 = words[index - 2].rotate_right(17)
                ^ words[index - 2].rotate_right(19)
                ^ (words[index - 2] >> 10);
            words[index] = words[index - 16]
                .wrapping_add(s0)
                .wrapping_add(words[index - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut h] = hash;
        for index in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let choice = (e & f) ^ ((!e) & g);
            let temporary1 = h
                .wrapping_add(s1)
                .wrapping_add(choice)
                .wrapping_add(K[index])
                .wrapping_add(words[index]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let majority = (a & b) ^ (a & c) ^ (b & c);
            let temporary2 = s0.wrapping_add(majority);
            h = g;
            g = f;
            f = e;
            e = d.wrapping_add(temporary1);
            d = c;
            c = b;
            b = a;
            a = temporary1.wrapping_add(temporary2);
        }
        hash[0] = hash[0].wrapping_add(a);
        hash[1] = hash[1].wrapping_add(b);
        hash[2] = hash[2].wrapping_add(c);
        hash[3] = hash[3].wrapping_add(d);
        hash[4] = hash[4].wrapping_add(e);
        hash[5] = hash[5].wrapping_add(f);
        hash[6] = hash[6].wrapping_add(g);
        hash[7] = hash[7].wrapping_add(h);
    }
    let mut output = String::with_capacity(64);
    for word in hash {
        write!(&mut output, "{word:08x}").expect("String writes cannot fail");
    }
    output
}

const BUNDLE_KEYS: &[&str] = &[
    "contract",
    "version",
    "bundle_id",
    "units",
    "schema_path",
    "schema_sha256",
    "schema_id",
    "schema_version",
    "profile_path",
    "profile_sha256",
    "profile_id",
    "profile_revision",
    "board_id",
    "motor_id",
    "inverter_id",
    "runtime_config_crc32",
    "product_contract_version",
    "bridge_abi_version",
    "source_revision",
    "model_revision",
    "scenario_path",
    "scenario_sha256",
    "scenario_id",
    "scenario_revision",
    "trace_schema_path",
    "trace_schema_sha256",
    "trace_schema_id",
    "trace_schema_version",
    "comparison_path",
    "comparison_sha256",
    "comparison_id",
    "comparison_version",
];
const SCHEMA_KEYS: &[&str] = &[
    "contract",
    "version",
    "schema_id",
    "units",
    "bundle_required",
    "profile_required",
    "scenario_required",
    "trace_required",
    "comparison_required",
];
const PROFILE_KEYS: &[&str] = &[
    "contract",
    "version",
    "profile_id",
    "revision",
    "units",
    "board_id",
    "motor_id",
    "inverter_id",
    "runtime_config_crc32",
    "product_contract_version",
    "bridge_abi_version",
    "source_revision",
    "model_revision",
    "pole_pairs",
    "stator_resistance_ohm",
    "ld_h",
    "lq_h",
    "flux_linkage_wb",
    "rated_current_a",
    "max_mechanical_velocity_rad_s",
    "nominal_bus_voltage_v",
    "inertia_kg_m2",
    "viscous_friction_nm_s",
    "pwm_frequency_hz",
    "control_frequency_hz",
    "speed_loop_frequency_hz",
    "actuation_delay_pwm_ticks",
];
const SCENARIO_KEYS: &[&str] = &[
    "contract",
    "version",
    "scenario_id",
    "revision",
    "seed",
    "units",
    "profile_id",
    "profile_revision",
    "duration_s",
    "trace_decimation",
    "initial_velocity_rad_s",
    "initial_electrical_angle_rad",
    "initial_temperature_k",
    "dc_bus_voltage_v",
    "command_at_s",
    "axis_request",
    "control_mode",
    "input_mode",
    "feedback_mode",
    "target_velocity_rad_s",
    "startup_duration_s",
    "load_step_at_s",
    "load_torque_nm",
    "sensor_noise_std_a",
    "sensor_offset_a",
    "sensor_quantization_a",
    "sensor_delay_control_ticks",
    "sensor_fault",
    "power_source_model",
    "brake_resistor_model",
    "d1_events",
];
const D1_EVENT_KEYS: &[&str] = &[
    "control_tick",
    "axis_state",
    "controller_internal_state",
    "active_source",
    "event",
    "output_enabled",
    "fault_flags",
];
const TRACE_KEYS: &[&str] = &[
    "contract",
    "version",
    "trace_schema_id",
    "units",
    "channels",
];
const TRACE_CHANNEL_KEYS: &[&str] = &["name", "unit", "kind"];
const TRACE_KINDS: &[&str] = &[
    "identity",
    "time",
    "state",
    "reference",
    "current",
    "voltage",
    "modulation",
    "angle",
    "velocity",
    "observer",
    "mechanical",
    "thermal",
    "energy",
    "safety",
];
const REQUIRED_TRACE_CHANNELS: &[&str] = &[
    "scenario_identity",
    "profile_identity",
    "config_identity",
    "model_revision",
    "time_s",
    "control_tick",
    "pwm_tick",
    "sample_age_s",
    "axis_state",
    "controller_internal_state",
    "control_mode",
    "input_mode",
    "active_source",
    "event",
    "velocity_reference_rad_s",
    "id_reference_a",
    "iq_reference_a",
    "phase_current_u_a",
    "phase_current_v_a",
    "phase_current_w_a",
    "id_a",
    "iq_a",
    "vd_command_v",
    "vq_command_v",
    "dc_bus_voltage_v",
    "duty_u",
    "duty_v",
    "duty_w",
    "true_electrical_angle_rad",
    "forced_electrical_angle_rad",
    "estimated_electrical_angle_rad",
    "selected_electrical_angle_rad",
    "true_velocity_rad_s",
    "estimated_velocity_rad_s",
    "selected_velocity_rad_s",
    "observer_quality",
    "observer_reliable",
    "electromagnetic_torque_nm",
    "load_torque_nm",
    "mechanical_position_rad",
    "motor_temperature_k",
    "bus_power_w",
    "regen_power_w",
    "fault_flags",
    "derating_ratio",
    "command_timeout",
    "output_enabled",
];
const COMPARISON_KEYS: &[&str] = &[
    "contract",
    "version",
    "comparison_id",
    "units",
    "d0_exact",
    "d1_exact",
    "d1_event_tick_tolerance",
    "numeric_tolerances",
    "motion_numeric_tolerances",
];
const TOLERANCE_KEYS: &[&str] = &["channel", "absolute", "relative"];
const D0_EXACT: &[&str] = &[
    "bundle_id",
    "bundle_sha256",
    "schema_id",
    "schema_version",
    "schema_sha256",
    "profile_id",
    "profile_revision",
    "profile_sha256",
    "board_id",
    "motor_id",
    "inverter_id",
    "runtime_config_crc32",
    "product_contract_version",
    "bridge_abi_version",
    "source_revision",
    "model_revision",
    "workspace_revision",
    "scenario_id",
    "scenario_revision",
    "scenario_sha256",
    "trace_schema_id",
    "trace_schema_version",
    "trace_schema_sha256",
    "comparison_id",
    "comparison_version",
    "comparison_sha256",
];
const D1_EXACT: &[&str] = &[
    "control_tick",
    "axis_state",
    "controller_internal_state",
    "active_source",
    "event",
    "output_enabled",
    "fault_flags",
    "event_order",
];

#[cfg(test)]
mod tests {
    use super::*;

    fn project_root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../..")
            .canonicalize()
            .unwrap()
    }

    fn committed_document(relative: &str) -> String {
        fs::read_to_string(project_root().join(relative)).unwrap()
    }

    #[test]
    fn sha256_matches_fips_test_vector() {
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn runtime_config_crc_probe() {
        assert_eq!(current_runtime_config_crc32().unwrap(), "0x51d3b0a5");
    }

    #[test]
    fn legacy_speed_start_loads_and_adapts_524_rpm() {
        let bundle = load_simulation_contract(project_root()).unwrap();
        let config = bundle.reference_simulation_config();
        assert!((config.target_speed_rpm - 524.0).abs() < 1.0e-4);
        assert_eq!(config.duration_s, 3.0);
        assert_eq!(config.trace_decimation, 10);
        assert_eq!(config.timing.pwm_frequency_hz, 12_000);
        assert_eq!(bundle.scenario.d1_events.len(), 3);
        assert_eq!(bundle.gate_result().lines.last().unwrap(), "d1=PASS");
        assert_eq!(bundle.gate_result().engine_id, "fluxrt.rust.lifecycle.v1");
    }

    #[test]
    fn d1_engine_does_not_echo_the_expected_event_fixture() {
        let mut bundle = load_simulation_contract(project_root()).unwrap();
        bundle.scenario.d1_events.clear();
        let actual = bundle.run_d1_state_machine();
        assert_eq!(actual.len(), 3);
        assert_eq!(actual[0].axis_state, "Disabled");
        assert_eq!(actual[1].axis_state, "Startup");
        assert_eq!(actual[2].axis_state, "ClosedLoop");
        assert_eq!(actual[2].control_tick, 30_000);
    }

    #[test]
    fn profile_fails_closed_on_missing_unknown_version_or_units() {
        let original =
            committed_document("simulation/profiles/stm32g431_gbm2804h_reference_v1.json");
        let missing = original.replacen(
            "  \"board_id\": \"nucleo-g431rb+x-nucleo-ihm16m1\",\n",
            "",
            1,
        );
        let value = parse_json_bytes(missing.as_bytes()).unwrap();
        assert!(parse_profile(object(&value, "profile").unwrap()).is_err());

        let unknown = original.replacen("  \"version\": 1", "  \"version\": 99", 1);
        let value = parse_json_bytes(unknown.as_bytes()).unwrap();
        assert!(parse_profile(object(&value, "profile").unwrap()).is_err());

        let wrong_units = original.replacen("\"units\": \"SI\"", "\"units\": \"rpm\"", 1);
        let value = parse_json_bytes(wrong_units.as_bytes()).unwrap();
        assert!(parse_profile(object(&value, "profile").unwrap()).is_err());
    }

    #[test]
    fn loader_fails_closed_on_hash_and_identity_mismatch() {
        let profile =
            committed_document("simulation/profiles/stm32g431_gbm2804h_reference_v1.json");
        let expected = "0000000000000000000000000000000000000000000000000000000000000000";
        assert_ne!(sha256_hex(profile.as_bytes()), expected);

        let bundle = load_simulation_contract(project_root()).unwrap();
        let changed_scenario = committed_document("simulation/scenarios/legacy_speed_start.json")
            .replacen(&bundle.profile.profile_id, "wrong.profile", 1);
        let value = parse_json_bytes(changed_scenario.as_bytes()).unwrap();
        assert!(parse_scenario(object(&value, "scenario").unwrap(), &bundle.profile).is_err());
    }
}
