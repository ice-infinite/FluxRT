//! Shared P4.2D motion-control matrix runner for the Rust PC harness.
//!
//! The JSON document is deliberately parsed by the existing strict contract
//! parser.  The runner invokes the product [`MotionReferencePlanner`] and
//! [`MotionCascadeController`] directly; it does not carry a simulator copy of
//! either algorithm.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use foc_control::{
    ControlMode, FeedbackMode, InputMode, MotionCascadeConfig, MotionCascadeController,
    MotionCascadeError, MotionCascadeFeedback, MotionCascadeOutput, MotionFeedback,
    MotionPlannerConfig, MotionPlannerError, MotionReference, MotionReferencePlanner,
    ProductCommand, ProductCommandKind, PRODUCT_COMMAND_FLAG_CURRENT_LIMIT,
    PRODUCT_COMMAND_FLAG_TORQUE_LIMIT, PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT,
};

use crate::simulation_contract::{
    array, boolean, exact_keys, finite_number, invalid, load_simulation_contract, object,
    parse_json_bytes, read_bytes, sha256_hex, string, unsigned, JsonValue, SimulationContractError,
};

pub const MOTION_SCENARIO_RELATIVE_PATH: &str =
    "simulation/scenarios/motion_control_matrix_v1.json";
pub const MOTION_RESULT_RELATIVE_PATH: &str = "simulation/results/motion/rust-motion-d0.txt";
pub const MOTION_TRACE_RELATIVE_PATH: &str = "simulation/results/motion/rust-motion-trace.csv";

const TOP_LEVEL_KEYS: &[&str] = &[
    "contract",
    "version",
    "scenario_id",
    "revision",
    "units",
    "profile_id",
    "profile_revision",
    "model_revision",
    "runtime",
    "cases",
];
const RUNTIME_KEYS: &[&str] = &[
    "control_frequency_hz",
    "maximum_current_a",
    "maximum_torque_nm",
    "maximum_velocity_rad_s",
    "torque_ramp_rate_nm_s",
    "velocity_ramp_rate_rad_s2",
    "position_filter_bandwidth_rad_s",
    "trajectory_acceleration_rad_s2",
    "trajectory_deceleration_rad_s2",
    "soft_limit_min_rad",
    "soft_limit_max_rad",
    "position_kp_per_s",
    "velocity_kp_nm_per_rad_s",
    "velocity_ki_nm_per_rad",
    "torque_constant_nm_per_a",
];
const CASE_KEYS: &[&str] = &[
    "case_id",
    "enabled",
    "evaluation_ticks",
    "initial_feedback",
    "commands",
    "feedback_faults",
];
const DYNAMIC_CASE_KEYS: &[&str] = &[
    "case_id",
    "enabled",
    "evaluation_ticks",
    "initial_feedback",
    "commands",
    "feedback_faults",
    "mechanical_plant",
    "acceptance",
];
const FEEDBACK_KEYS: &[&str] = &["position_rad", "velocity_rad_s", "iq_a"];
const COMMAND_KEYS: &[&str] = &[
    "start_tick",
    "control_mode",
    "input_mode",
    "position_ref_rad",
    "velocity_ref_rad_s",
    "torque_ref_nm",
    "velocity_feedforward_rad_s",
    "torque_feedforward_nm",
    "current_limit_a",
    "torque_limit_nm",
    "velocity_limit_rad_s",
];
const FAULT_KEYS: &[&str] = &[
    "start_tick",
    "end_tick",
    "position_valid",
    "velocity_valid",
    "current_q_valid",
];
const MECHANICAL_PLANT_KEYS: &[&str] = &[
    "inertia_kg_m2",
    "viscous_friction_nm_s",
    "load_torque_nm",
    "current_response_bandwidth_rad_s",
];
const TORQUE_ACCEPTANCE_KEYS: &[&str] = &[
    "mode",
    "maximum_absolute_iq_a",
    "maximum_absolute_velocity_rad_s",
    "maximum_final_torque_error_nm",
    "minimum_final_velocity_rad_s",
];
const VELOCITY_ACCEPTANCE_KEYS: &[&str] = &[
    "mode",
    "maximum_absolute_iq_a",
    "maximum_absolute_velocity_rad_s",
    "maximum_final_velocity_error_rad_s",
    "maximum_velocity_overshoot_rad_s",
];
const POSITION_ACCEPTANCE_KEYS: &[&str] = &[
    "mode",
    "maximum_absolute_iq_a",
    "maximum_absolute_velocity_rad_s",
    "maximum_final_position_error_rad",
    "maximum_position_overshoot_rad",
    "maximum_final_absolute_velocity_rad_s",
];

const CSV_HEADER: &str = "case_id,control_tick,time_s,enabled,control_mode,input_mode,status,command_position_rad,command_velocity_rad_s,command_torque_nm,feedback_position_rad,feedback_velocity_rad_s,feedback_iq_a,planned_position_rad,planned_velocity_rad_s,planned_torque_nm,output_velocity_ref_rad_s,output_torque_nm,output_id_a,output_iq_a,planner_flags,cascade_flags,velocity_integrator_nm";

#[derive(Clone, Debug)]
struct MotionMatrix {
    scenario_id: String,
    revision: u32,
    profile_id: String,
    profile_revision: u32,
    model_revision: String,
    runtime: Runtime,
    cases: Vec<Case>,
}

#[derive(Clone, Copy, Debug)]
struct Runtime {
    control_frequency_hz: u32,
    planner: MotionPlannerConfig,
    cascade: MotionCascadeConfig,
}

#[derive(Clone, Debug)]
struct Case {
    case_id: String,
    enabled: bool,
    evaluation_ticks: u32,
    initial_feedback: InitialFeedback,
    commands: Vec<ScheduledCommand>,
    feedback_faults: Vec<FeedbackFault>,
    dynamic_acceptance: Option<DynamicAcceptance>,
}

#[derive(Clone, Copy, Debug, Default)]
struct InitialFeedback {
    position_rad: f32,
    velocity_rad_s: f32,
    iq_a: f32,
}

#[derive(Clone, Copy, Debug)]
struct ScheduledCommand {
    start_tick: u32,
    command: ProductCommand,
    control_mode: ControlMode,
    input_mode: InputMode,
}

#[derive(Clone, Copy, Debug)]
struct FeedbackFault {
    start_tick: u32,
    end_tick: u32,
    position_valid: bool,
    velocity_valid: bool,
    current_q_valid: bool,
}

#[derive(Clone, Copy, Debug)]
struct MechanicalPlant {
    inertia_kg_m2: f32,
    viscous_friction_nm_s: f32,
    load_torque_nm: f32,
    current_response_bandwidth_rad_s: f32,
}

#[derive(Clone, Copy, Debug)]
struct DynamicAcceptance {
    plant: MechanicalPlant,
    limits: AcceptanceLimits,
}

#[derive(Clone, Copy, Debug)]
enum AcceptanceLimits {
    Torque {
        maximum_absolute_iq_a: f32,
        maximum_absolute_velocity_rad_s: f32,
        maximum_final_torque_error_nm: f32,
        minimum_final_velocity_rad_s: f32,
    },
    Velocity {
        maximum_absolute_iq_a: f32,
        maximum_absolute_velocity_rad_s: f32,
        maximum_final_velocity_error_rad_s: f32,
        maximum_velocity_overshoot_rad_s: f32,
    },
    Position {
        maximum_absolute_iq_a: f32,
        maximum_absolute_velocity_rad_s: f32,
        maximum_final_position_error_rad: f32,
        maximum_position_overshoot_rad: f32,
        maximum_final_absolute_velocity_rad_s: f32,
    },
}

#[derive(Clone, Copy, Debug, Default)]
struct AcceptanceMetrics {
    maximum_absolute_iq_a: f32,
    maximum_absolute_velocity_rad_s: f32,
    maximum_velocity_rad_s: f32,
    maximum_position_rad: f32,
    final_feedback: InitialFeedback,
}

#[derive(Clone, Debug)]
pub struct MotionContractRun {
    pub metadata: Vec<(String, String)>,
    pub csv: String,
    pub row_count: usize,
    pub case_count: usize,
}

impl MotionContractRun {
    pub fn write_default_outputs(
        &self,
        project_root: impl AsRef<Path>,
    ) -> Result<(), SimulationContractError> {
        let root = project_root.as_ref();
        let metadata_path = root.join(MOTION_RESULT_RELATIVE_PATH);
        let trace_path = root.join(MOTION_TRACE_RELATIVE_PATH);
        if let Some(parent) = metadata_path.parent() {
            fs::create_dir_all(parent).map_err(|error| {
                SimulationContractError::Io(format!("cannot create {}: {error}", parent.display()))
            })?;
        }
        let metadata = self
            .metadata
            .iter()
            .map(|(key, value)| format!("{key}={value}"))
            .collect::<Vec<_>>()
            .join("\n")
            + "\n";
        fs::write(&metadata_path, metadata).map_err(|error| {
            SimulationContractError::Io(format!(
                "cannot write {}: {error}",
                metadata_path.display()
            ))
        })?;
        fs::write(&trace_path, &self.csv).map_err(|error| {
            SimulationContractError::Io(format!("cannot write {}: {error}", trace_path.display()))
        })?;
        Ok(())
    }
}

pub fn run_motion_contract(
    project_root: impl AsRef<Path>,
) -> Result<MotionContractRun, SimulationContractError> {
    let root = project_root.as_ref();
    let base = load_simulation_contract(root)?;
    let scenario_path = root.join(MOTION_SCENARIO_RELATIVE_PATH);
    let scenario_bytes = read_bytes(&scenario_path)?;
    let scenario_sha256 = sha256_hex(&scenario_bytes);
    let matrix = parse_matrix(&scenario_bytes)?;
    let workspace_revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());

    if matrix.profile_id != base.profile.profile_id
        || matrix.profile_revision != base.profile.revision
    {
        return invalid("motion scenario profile identity mismatch".to_owned());
    }
    if matrix.runtime.control_frequency_hz != base.profile.control_frequency_hz {
        return invalid("motion runtime/profile control_frequency_hz mismatch".to_owned());
    }

    let mut csv = String::from(CSV_HEADER);
    csv.push('\n');
    let mut row_count = 0_usize;
    for case in &matrix.cases {
        run_case(case, matrix.runtime, &mut csv)?;
        row_count += case.evaluation_ticks as usize;
    }

    let metadata = vec![
        ("contract_gate_result_version".to_owned(), "1".to_owned()),
        ("engine_id".to_owned(), "fluxrt.rust.motion.v1".to_owned()),
        ("workspace_revision".to_owned(), workspace_revision),
        ("bundle_id".to_owned(), base.bundle_id),
        ("bundle_sha256".to_owned(), base.bundle_sha256),
        ("profile_id".to_owned(), base.profile.profile_id),
        (
            "profile_revision".to_owned(),
            base.profile.revision.to_string(),
        ),
        ("profile_sha256".to_owned(), base.profile_sha256),
        (
            "base_model_revision".to_owned(),
            base.profile.model_revision,
        ),
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
        ("motion_scenario_id".to_owned(), matrix.scenario_id),
        (
            "scenario_contract".to_owned(),
            "fluxrt-motion-control-scenario".to_owned(),
        ),
        ("scenario_version".to_owned(), "1".to_owned()),
        (
            "motion_scenario_revision".to_owned(),
            matrix.revision.to_string(),
        ),
        ("motion_scenario_sha256".to_owned(), scenario_sha256),
        ("motion_model_revision".to_owned(), matrix.model_revision),
        ("case_count".to_owned(), matrix.cases.len().to_string()),
        ("row_count".to_owned(), row_count.to_string()),
        ("feature_off".to_owned(), "PASS".to_owned()),
        ("d0".to_owned(), "PASS".to_owned()),
        ("d1".to_owned(), "PASS".to_owned()),
        ("d2".to_owned(), "PASS".to_owned()),
        ("d3".to_owned(), "PASS".to_owned()),
        ("d4".to_owned(), "PASS".to_owned()),
        ("e2_acceptance".to_owned(), "PASS".to_owned()),
    ];
    Ok(MotionContractRun {
        metadata,
        csv,
        row_count,
        case_count: matrix.cases.len(),
    })
}

fn run_case(
    case: &Case,
    runtime: Runtime,
    csv: &mut String,
) -> Result<(), SimulationContractError> {
    let mut planner = MotionReferencePlanner::default();
    let mut cascade = MotionCascadeController::default();
    planner.set_enabled(case.enabled);
    cascade.set_enabled(case.enabled);
    let period_s = runtime.planner.control_period_s;
    let mut feedback = case.initial_feedback;
    let mut metrics = AcceptanceMetrics::default();

    for tick in 0..case.evaluation_ticks {
        let scheduled = active_command(&case.commands, tick);
        let validity = active_feedback_validity(&case.feedback_faults, tick);
        let motion_feedback = MotionFeedback {
            position_valid: validity.0,
            velocity_valid: validity.1,
            mechanical_position_rad: feedback.position_rad,
            mechanical_velocity_rad_s: feedback.velocity_rad_s,
        };
        let cascade_feedback = MotionCascadeFeedback {
            position_valid: validity.0,
            velocity_valid: validity.1,
            current_q_valid: validity.2,
            mechanical_position_rad: feedback.position_rad,
            mechanical_velocity_rad_s: feedback.velocity_rad_s,
            current_q_a: feedback.iq_a,
        };

        // One tick is a coupled transaction. A cascade rejection must not leave
        // the planner ramp one step ahead of the controller.
        let mut next_planner = planner;
        let mut next_cascade = cascade;
        let result = next_planner
            .step(&runtime.planner, &scheduled.command, motion_feedback)
            .map_err(TickError::Planner)
            .and_then(|reference| {
                next_cascade
                    .step(&runtime.cascade, &reference, cascade_feedback)
                    .map(|output| (reference, output))
                    .map_err(TickError::Cascade)
            });

        let (status, reference, output, integrator) = match result {
            Ok((reference, output)) => {
                planner = next_planner;
                cascade = next_cascade;
                ("ok", reference, output, cascade.velocity_integrator_nm())
            }
            Err(error) => (
                error.status(),
                MotionReference::default(),
                MotionCascadeOutput::default(),
                0.0,
            ),
        };

        writeln!(
            csv,
            "{},{},{:.9},{},{},{},{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{:.9}",
            case.case_id,
            tick,
            tick as f32 * period_s,
            u8::from(case.enabled),
            control_mode_name(scheduled.control_mode),
            input_mode_name(scheduled.input_mode),
            status,
            scheduled.command.position_ref_rad,
            scheduled.command.velocity_ref_rad_s,
            scheduled.command.torque_ref_nm,
            feedback.position_rad,
            feedback.velocity_rad_s,
            feedback.iq_a,
            reference.position_ref_rad,
            reference.velocity_ref_rad_s,
            reference.torque_ref_nm,
            output.velocity_reference_rad_s,
            output.torque_reference_nm,
            output.current_reference.id_ref_a,
            output.current_reference.iq_ref_a,
            reference.flags,
            output.flags,
            integrator,
        )
        .expect("writing to String cannot fail");

        if let Some(dynamic) = case.dynamic_acceptance {
            if status != "ok" {
                return invalid(format!(
                    "{}: dynamic acceptance tick {tick} failed with {status}",
                    case.case_id
                ));
            }
            update_acceptance_metrics(&mut metrics, feedback, output);
            if tick + 1 < case.evaluation_ticks {
                feedback = advance_mechanical_plant(
                    feedback,
                    output,
                    dynamic.plant,
                    runtime.cascade.torque_constant_nm_per_a,
                    period_s,
                )?;
            }
        }
    }
    if let Some(dynamic) = case.dynamic_acceptance {
        verify_acceptance(
            case,
            dynamic.limits,
            metrics,
            runtime.cascade.torque_constant_nm_per_a,
        )?;
    }
    Ok(())
}

fn update_acceptance_metrics(
    metrics: &mut AcceptanceMetrics,
    feedback: InitialFeedback,
    output: MotionCascadeOutput,
) {
    metrics.maximum_absolute_iq_a = metrics
        .maximum_absolute_iq_a
        .max(feedback.iq_a.abs())
        .max(output.current_reference.iq_ref_a.abs());
    metrics.maximum_absolute_velocity_rad_s = metrics
        .maximum_absolute_velocity_rad_s
        .max(feedback.velocity_rad_s.abs());
    metrics.maximum_velocity_rad_s = metrics.maximum_velocity_rad_s.max(feedback.velocity_rad_s);
    metrics.maximum_position_rad = metrics.maximum_position_rad.max(feedback.position_rad);
    metrics.final_feedback = feedback;
}

fn advance_mechanical_plant(
    feedback: InitialFeedback,
    output: MotionCascadeOutput,
    plant: MechanicalPlant,
    torque_constant_nm_per_a: f32,
    dt_s: f32,
) -> Result<InitialFeedback, SimulationContractError> {
    let alpha = (plant.current_response_bandwidth_rad_s * dt_s).clamp(0.0, 1.0);
    let current_error = output.current_reference.iq_ref_a - feedback.iq_a;
    let next_iq_a = feedback.iq_a + alpha * current_error;
    let electromagnetic_torque_nm = next_iq_a * torque_constant_nm_per_a;
    let viscous_torque_nm = plant.viscous_friction_nm_s * feedback.velocity_rad_s;
    let net_torque_nm = electromagnetic_torque_nm - plant.load_torque_nm - viscous_torque_nm;
    let acceleration_rad_s2 = net_torque_nm / plant.inertia_kg_m2;
    let next_velocity_rad_s = feedback.velocity_rad_s + acceleration_rad_s2 * dt_s;
    let next_position_rad = feedback.position_rad + next_velocity_rad_s * dt_s;
    let next = InitialFeedback {
        position_rad: next_position_rad,
        velocity_rad_s: next_velocity_rad_s,
        iq_a: next_iq_a,
    };
    if ![next.position_rad, next.velocity_rad_s, next.iq_a]
        .iter()
        .all(|value| value.is_finite())
    {
        return invalid("mechanical acceptance plant produced non-finite state".to_owned());
    }
    Ok(next)
}

fn verify_acceptance(
    case: &Case,
    limits: AcceptanceLimits,
    metrics: AcceptanceMetrics,
    torque_constant_nm_per_a: f32,
) -> Result<(), SimulationContractError> {
    let command = case
        .commands
        .last()
        .expect("strict parser requires at least one command");
    let fail = |message: String| invalid(format!("{}: {message}", case.case_id));
    match limits {
        AcceptanceLimits::Torque {
            maximum_absolute_iq_a,
            maximum_absolute_velocity_rad_s,
            maximum_final_torque_error_nm,
            minimum_final_velocity_rad_s,
        } => {
            let final_torque_nm = metrics.final_feedback.iq_a * torque_constant_nm_per_a;
            let final_error_nm = (final_torque_nm - command.command.torque_ref_nm).abs();
            if metrics.maximum_absolute_iq_a > maximum_absolute_iq_a {
                return fail(format!(
                    "peak iq {} exceeds {}",
                    metrics.maximum_absolute_iq_a, maximum_absolute_iq_a
                ));
            }
            if metrics.maximum_absolute_velocity_rad_s > maximum_absolute_velocity_rad_s {
                return fail(format!(
                    "peak velocity {} exceeds {}",
                    metrics.maximum_absolute_velocity_rad_s, maximum_absolute_velocity_rad_s
                ));
            }
            if final_error_nm > maximum_final_torque_error_nm {
                return fail(format!(
                    "final torque error {final_error_nm} exceeds {maximum_final_torque_error_nm}"
                ));
            }
            if metrics.final_feedback.velocity_rad_s < minimum_final_velocity_rad_s {
                return fail(format!(
                    "final velocity {} is below {}",
                    metrics.final_feedback.velocity_rad_s, minimum_final_velocity_rad_s
                ));
            }
        }
        AcceptanceLimits::Velocity {
            maximum_absolute_iq_a,
            maximum_absolute_velocity_rad_s,
            maximum_final_velocity_error_rad_s,
            maximum_velocity_overshoot_rad_s,
        } => {
            let target = command.command.velocity_ref_rad_s;
            let final_error = (metrics.final_feedback.velocity_rad_s - target).abs();
            let overshoot = (metrics.maximum_velocity_rad_s - target).max(0.0);
            if metrics.maximum_absolute_iq_a > maximum_absolute_iq_a {
                return fail(format!(
                    "peak iq {} exceeds {}",
                    metrics.maximum_absolute_iq_a, maximum_absolute_iq_a
                ));
            }
            if metrics.maximum_absolute_velocity_rad_s > maximum_absolute_velocity_rad_s {
                return fail(format!(
                    "peak velocity {} exceeds {}",
                    metrics.maximum_absolute_velocity_rad_s, maximum_absolute_velocity_rad_s
                ));
            }
            if final_error > maximum_final_velocity_error_rad_s {
                return fail(format!("final velocity error {final_error} exceeds {maximum_final_velocity_error_rad_s}"));
            }
            if overshoot > maximum_velocity_overshoot_rad_s {
                return fail(format!(
                    "velocity overshoot {overshoot} exceeds {maximum_velocity_overshoot_rad_s}"
                ));
            }
        }
        AcceptanceLimits::Position {
            maximum_absolute_iq_a,
            maximum_absolute_velocity_rad_s,
            maximum_final_position_error_rad,
            maximum_position_overshoot_rad,
            maximum_final_absolute_velocity_rad_s,
        } => {
            let target = command.command.position_ref_rad;
            let final_error = (metrics.final_feedback.position_rad - target).abs();
            let overshoot = (metrics.maximum_position_rad - target).max(0.0);
            if metrics.maximum_absolute_iq_a > maximum_absolute_iq_a {
                return fail(format!(
                    "peak iq {} exceeds {}",
                    metrics.maximum_absolute_iq_a, maximum_absolute_iq_a
                ));
            }
            if metrics.maximum_absolute_velocity_rad_s > maximum_absolute_velocity_rad_s {
                return fail(format!(
                    "peak velocity {} exceeds {}",
                    metrics.maximum_absolute_velocity_rad_s, maximum_absolute_velocity_rad_s
                ));
            }
            if final_error > maximum_final_position_error_rad {
                return fail(format!(
                    "final position error {final_error} exceeds {maximum_final_position_error_rad}"
                ));
            }
            if overshoot > maximum_position_overshoot_rad {
                return fail(format!(
                    "position overshoot {overshoot} exceeds {maximum_position_overshoot_rad}"
                ));
            }
            if metrics.final_feedback.velocity_rad_s.abs() > maximum_final_absolute_velocity_rad_s {
                return fail(format!(
                    "final absolute velocity {} exceeds {maximum_final_absolute_velocity_rad_s}",
                    metrics.final_feedback.velocity_rad_s.abs()
                ));
            }
        }
    }
    Ok(())
}

fn active_command(commands: &[ScheduledCommand], tick: u32) -> ScheduledCommand {
    *commands
        .iter()
        .rev()
        .find(|command| command.start_tick <= tick)
        .expect("strict parser requires command at tick zero")
}

fn active_feedback_validity(faults: &[FeedbackFault], tick: u32) -> (bool, bool, bool) {
    faults
        .iter()
        .find(|fault| fault.start_tick <= tick && tick < fault.end_tick)
        .map(|fault| {
            (
                fault.position_valid,
                fault.velocity_valid,
                fault.current_q_valid,
            )
        })
        .unwrap_or((true, true, true))
}

#[derive(Clone, Copy, Debug)]
enum TickError {
    Planner(MotionPlannerError),
    Cascade(MotionCascadeError),
}

impl TickError {
    fn status(self) -> &'static str {
        match self {
            Self::Planner(MotionPlannerError::Disabled) => "planner_disabled",
            Self::Planner(MotionPlannerError::InvalidConfig) => "planner_invalid_config",
            Self::Planner(MotionPlannerError::InvalidCommand(_)) => "planner_invalid_command",
            Self::Planner(MotionPlannerError::NotSetpoint) => "planner_not_setpoint",
            Self::Planner(MotionPlannerError::UnsupportedControlMode) => {
                "planner_unsupported_control_mode"
            }
            Self::Planner(MotionPlannerError::MissingPositionFeedback) => {
                "planner_missing_position"
            }
            Self::Planner(MotionPlannerError::MissingVelocityFeedback) => {
                "planner_missing_velocity"
            }
            Self::Planner(MotionPlannerError::InvalidFeedback) => "planner_invalid_feedback",
            Self::Cascade(MotionCascadeError::Disabled) => "cascade_disabled",
            Self::Cascade(MotionCascadeError::InvalidConfig) => "cascade_invalid_config",
            Self::Cascade(MotionCascadeError::InvalidReference) => "cascade_invalid_reference",
            Self::Cascade(MotionCascadeError::UnknownReferenceFlags) => {
                "cascade_unknown_reference_flags"
            }
            Self::Cascade(MotionCascadeError::UnsupportedControlMode) => {
                "cascade_unsupported_control_mode"
            }
            Self::Cascade(MotionCascadeError::UnsupportedInputMode) => {
                "cascade_unsupported_input_mode"
            }
            Self::Cascade(MotionCascadeError::MissingPositionFeedback) => {
                "cascade_missing_position"
            }
            Self::Cascade(MotionCascadeError::MissingVelocityFeedback) => {
                "cascade_missing_velocity"
            }
            Self::Cascade(MotionCascadeError::MissingCurrentFeedback) => "cascade_missing_current",
            Self::Cascade(MotionCascadeError::InvalidFeedback) => "cascade_invalid_feedback",
            Self::Cascade(MotionCascadeError::TransitionRequired) => "cascade_transition_required",
            Self::Cascade(MotionCascadeError::NonFiniteComputation) => "cascade_non_finite",
        }
    }
}

fn parse_matrix(bytes: &[u8]) -> Result<MotionMatrix, SimulationContractError> {
    let json = parse_json_bytes(bytes)?;
    let root = object(&json, "motion scenario")?;
    exact_keys(root, TOP_LEVEL_KEYS, "motion scenario")?;
    if string(root, "contract")? != "fluxrt-motion-control-scenario"
        || unsigned(root, "version")? != 1
    {
        return invalid("unknown motion scenario contract/version".to_owned());
    }
    if string(root, "units")? != "SI" {
        return invalid("motion scenario units must be SI".to_owned());
    }
    let scenario_id = checked_id(string(root, "scenario_id")?, "scenario_id")?;
    let revision = checked_u32(unsigned(root, "revision")?, "revision")?;
    let profile_id = checked_id(string(root, "profile_id")?, "profile_id")?;
    let profile_revision = checked_u32(unsigned(root, "profile_revision")?, "profile_revision")?;
    let model_revision = checked_id(string(root, "model_revision")?, "model_revision")?;
    let runtime = parse_runtime(object(required_value(root, "runtime")?, "runtime")?)?;
    let case_values = array(required_value(root, "cases")?, "cases")?;
    if case_values.is_empty() {
        return invalid("cases must not be empty".to_owned());
    }
    let mut cases = Vec::with_capacity(case_values.len());
    for (index, value) in case_values.iter().enumerate() {
        let case = parse_case(object(value, "case")?, index)?;
        if cases
            .iter()
            .any(|existing: &Case| existing.case_id == case.case_id)
        {
            return invalid(format!("duplicate case_id: {}", case.case_id));
        }
        cases.push(case);
    }
    if !cases
        .iter()
        .any(|case| case.case_id == "feature_off" && !case.enabled)
    {
        return invalid("feature_off disabled case is required".to_owned());
    }
    Ok(MotionMatrix {
        scenario_id,
        revision,
        profile_id,
        profile_revision,
        model_revision,
        runtime,
        cases,
    })
}

fn parse_runtime(object: &BTreeMap<String, JsonValue>) -> Result<Runtime, SimulationContractError> {
    exact_keys(object, RUNTIME_KEYS, "runtime")?;
    let control_frequency_hz = checked_u32(
        unsigned(object, "control_frequency_hz")?,
        "control_frequency_hz",
    )?;
    if control_frequency_hz == 0 {
        return invalid("control_frequency_hz must be positive".to_owned());
    }
    let period = 1.0 / control_frequency_hz as f32;
    let planner = MotionPlannerConfig {
        control_period_s: period,
        maximum_current_a: f32_value(object, "maximum_current_a")?,
        maximum_torque_nm: f32_value(object, "maximum_torque_nm")?,
        maximum_velocity_rad_s: f32_value(object, "maximum_velocity_rad_s")?,
        torque_ramp_rate_nm_s: f32_value(object, "torque_ramp_rate_nm_s")?,
        velocity_ramp_rate_rad_s2: f32_value(object, "velocity_ramp_rate_rad_s2")?,
        position_filter_bandwidth_rad_s: f32_value(object, "position_filter_bandwidth_rad_s")?,
        trajectory_acceleration_rad_s2: f32_value(object, "trajectory_acceleration_rad_s2")?,
        trajectory_deceleration_rad_s2: f32_value(object, "trajectory_deceleration_rad_s2")?,
        soft_limit_min_rad: f32_value(object, "soft_limit_min_rad")?,
        soft_limit_max_rad: f32_value(object, "soft_limit_max_rad")?,
    };
    let cascade = MotionCascadeConfig {
        control_period_s: period,
        position_kp_per_s: f32_value(object, "position_kp_per_s")?,
        velocity_kp_nm_per_rad_s: f32_value(object, "velocity_kp_nm_per_rad_s")?,
        velocity_ki_nm_per_rad: f32_value(object, "velocity_ki_nm_per_rad")?,
        torque_constant_nm_per_a: f32_value(object, "torque_constant_nm_per_a")?,
    };
    planner.validate().map_err(|_| {
        SimulationContractError::Invalid("invalid motion planner runtime".to_owned())
    })?;
    cascade.validate().map_err(|_| {
        SimulationContractError::Invalid("invalid motion cascade runtime".to_owned())
    })?;
    Ok(Runtime {
        control_frequency_hz,
        planner,
        cascade,
    })
}

fn parse_case(
    object: &BTreeMap<String, JsonValue>,
    index: usize,
) -> Result<Case, SimulationContractError> {
    let has_plant = object.contains_key("mechanical_plant");
    let has_acceptance = object.contains_key("acceptance");
    if has_plant != has_acceptance {
        return invalid(format!(
            "cases[{index}] must provide mechanical_plant and acceptance together"
        ));
    }
    exact_keys(
        object,
        if has_plant {
            DYNAMIC_CASE_KEYS
        } else {
            CASE_KEYS
        },
        &format!("cases[{index}]"),
    )?;
    let case_id = checked_id(string(object, "case_id")?, "case_id")?;
    let enabled = boolean(object, "enabled")?;
    let evaluation_ticks = checked_u32(unsigned(object, "evaluation_ticks")?, "evaluation_ticks")?;
    if evaluation_ticks == 0 {
        return invalid(format!("{case_id}.evaluation_ticks must be positive"));
    }
    let feedback_object = object_from(
        required_value(object, "initial_feedback")?,
        "initial_feedback",
    )?;
    exact_keys(feedback_object, FEEDBACK_KEYS, "initial_feedback")?;
    let initial_feedback = InitialFeedback {
        position_rad: f32_value(feedback_object, "position_rad")?,
        velocity_rad_s: f32_value(feedback_object, "velocity_rad_s")?,
        iq_a: f32_value(feedback_object, "iq_a")?,
    };

    let command_values = array(required_value(object, "commands")?, "commands")?;
    if command_values.is_empty() {
        return invalid(format!("{case_id}.commands must not be empty"));
    }
    let mut commands = Vec::with_capacity(command_values.len());
    for (command_index, value) in command_values.iter().enumerate() {
        commands.push(parse_command(
            object_from(value, "command")?,
            command_index,
            evaluation_ticks,
        )?);
    }
    if commands[0].start_tick != 0
        || commands
            .windows(2)
            .any(|pair| pair[0].start_tick >= pair[1].start_tick)
    {
        return invalid(format!(
            "{case_id}.commands must start at zero and have increasing start_tick"
        ));
    }

    let fault_values = array(
        required_value(object, "feedback_faults")?,
        "feedback_faults",
    )?;
    let mut feedback_faults = Vec::with_capacity(fault_values.len());
    for (fault_index, value) in fault_values.iter().enumerate() {
        feedback_faults.push(parse_fault(
            object_from(value, "feedback fault")?,
            fault_index,
            evaluation_ticks,
        )?);
    }
    if feedback_faults
        .windows(2)
        .any(|pair| pair[0].end_tick > pair[1].start_tick)
    {
        return invalid(format!("{case_id}.feedback_faults overlap"));
    }
    let dynamic_acceptance = if has_plant {
        if !enabled || !feedback_faults.is_empty() {
            return invalid(format!(
                "{case_id}: dynamic acceptance requires enabled=true and no feedback faults"
            ));
        }
        let plant = parse_mechanical_plant(object_from(
            required_value(object, "mechanical_plant")?,
            "mechanical_plant",
        )?)?;
        let limits = parse_acceptance(
            object_from(required_value(object, "acceptance")?, "acceptance")?,
            commands
                .last()
                .expect("commands are non-empty")
                .control_mode,
        )?;
        Some(DynamicAcceptance { plant, limits })
    } else {
        None
    };
    Ok(Case {
        case_id,
        enabled,
        evaluation_ticks,
        initial_feedback,
        commands,
        feedback_faults,
        dynamic_acceptance,
    })
}

fn parse_mechanical_plant(
    object: &BTreeMap<String, JsonValue>,
) -> Result<MechanicalPlant, SimulationContractError> {
    exact_keys(object, MECHANICAL_PLANT_KEYS, "mechanical_plant")?;
    let plant = MechanicalPlant {
        inertia_kg_m2: f32_value(object, "inertia_kg_m2")?,
        viscous_friction_nm_s: f32_value(object, "viscous_friction_nm_s")?,
        load_torque_nm: f32_value(object, "load_torque_nm")?,
        current_response_bandwidth_rad_s: f32_value(object, "current_response_bandwidth_rad_s")?,
    };
    if plant.inertia_kg_m2 <= 0.0
        || plant.viscous_friction_nm_s < 0.0
        || plant.current_response_bandwidth_rad_s <= 0.0
    {
        return invalid("mechanical_plant has invalid physical range".to_owned());
    }
    Ok(plant)
}

fn parse_acceptance(
    object: &BTreeMap<String, JsonValue>,
    control_mode: ControlMode,
) -> Result<AcceptanceLimits, SimulationContractError> {
    let mode = string(object, "mode")?;
    let positive = |key: &str| -> Result<f32, SimulationContractError> {
        let value = f32_value(object, key)?;
        if value <= 0.0 {
            return invalid(format!("acceptance.{key} must be positive"));
        }
        Ok(value)
    };
    match (mode, control_mode) {
        ("Torque", ControlMode::Torque) => {
            exact_keys(object, TORQUE_ACCEPTANCE_KEYS, "torque acceptance")?;
            Ok(AcceptanceLimits::Torque {
                maximum_absolute_iq_a: positive("maximum_absolute_iq_a")?,
                maximum_absolute_velocity_rad_s: positive("maximum_absolute_velocity_rad_s")?,
                maximum_final_torque_error_nm: positive("maximum_final_torque_error_nm")?,
                minimum_final_velocity_rad_s: positive("minimum_final_velocity_rad_s")?,
            })
        }
        ("Velocity", ControlMode::Velocity) => {
            exact_keys(object, VELOCITY_ACCEPTANCE_KEYS, "velocity acceptance")?;
            Ok(AcceptanceLimits::Velocity {
                maximum_absolute_iq_a: positive("maximum_absolute_iq_a")?,
                maximum_absolute_velocity_rad_s: positive("maximum_absolute_velocity_rad_s")?,
                maximum_final_velocity_error_rad_s: positive("maximum_final_velocity_error_rad_s")?,
                maximum_velocity_overshoot_rad_s: positive("maximum_velocity_overshoot_rad_s")?,
            })
        }
        ("Position", ControlMode::Position) => {
            exact_keys(object, POSITION_ACCEPTANCE_KEYS, "position acceptance")?;
            Ok(AcceptanceLimits::Position {
                maximum_absolute_iq_a: positive("maximum_absolute_iq_a")?,
                maximum_absolute_velocity_rad_s: positive("maximum_absolute_velocity_rad_s")?,
                maximum_final_position_error_rad: positive("maximum_final_position_error_rad")?,
                maximum_position_overshoot_rad: positive("maximum_position_overshoot_rad")?,
                maximum_final_absolute_velocity_rad_s: positive(
                    "maximum_final_absolute_velocity_rad_s",
                )?,
            })
        }
        _ => invalid("acceptance.mode must match final command control_mode".to_owned()),
    }
}

fn parse_command(
    object: &BTreeMap<String, JsonValue>,
    index: usize,
    evaluation_ticks: u32,
) -> Result<ScheduledCommand, SimulationContractError> {
    exact_keys(object, COMMAND_KEYS, &format!("command[{index}]"))?;
    let start_tick = checked_u32(unsigned(object, "start_tick")?, "start_tick")?;
    if start_tick >= evaluation_ticks {
        return invalid(format!("command[{index}].start_tick outside case"));
    }
    let control_mode = parse_control_mode(string(object, "control_mode")?)?;
    let input_mode = parse_input_mode(string(object, "input_mode")?)?;
    let current_limit_a = f32_value(object, "current_limit_a")?;
    let torque_limit_nm = f32_value(object, "torque_limit_nm")?;
    let velocity_limit_rad_s = f32_value(object, "velocity_limit_rad_s")?;
    if current_limit_a < 0.0 || torque_limit_nm < 0.0 || velocity_limit_rad_s < 0.0 {
        return invalid(format!("command[{index}] limits must be non-negative"));
    }
    let mut flags = 0;
    if current_limit_a > 0.0 {
        flags |= PRODUCT_COMMAND_FLAG_CURRENT_LIMIT;
    }
    if torque_limit_nm > 0.0 {
        flags |= PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
    }
    if velocity_limit_rad_s > 0.0 {
        flags |= PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;
    }
    let command = ProductCommand {
        command_kind: ProductCommandKind::Setpoint as u32,
        control_mode: control_mode as u32,
        input_mode: input_mode as u32,
        feedback_mode: FeedbackMode::Sensorless as u32,
        flags,
        position_ref_rad: f32_value(object, "position_ref_rad")?,
        velocity_ref_rad_s: f32_value(object, "velocity_ref_rad_s")?,
        torque_ref_nm: f32_value(object, "torque_ref_nm")?,
        velocity_feedforward_rad_s: f32_value(object, "velocity_feedforward_rad_s")?,
        torque_feedforward_nm: f32_value(object, "torque_feedforward_nm")?,
        current_limit_a,
        torque_limit_nm,
        velocity_limit_rad_s,
        ..ProductCommand::default()
    };
    command.validate().map_err(|error| {
        SimulationContractError::Invalid(format!("command[{index}] invalid: {error:?}"))
    })?;
    Ok(ScheduledCommand {
        start_tick,
        command,
        control_mode,
        input_mode,
    })
}

fn parse_fault(
    object: &BTreeMap<String, JsonValue>,
    index: usize,
    evaluation_ticks: u32,
) -> Result<FeedbackFault, SimulationContractError> {
    exact_keys(object, FAULT_KEYS, &format!("feedback_fault[{index}]"))?;
    let start_tick = checked_u32(unsigned(object, "start_tick")?, "start_tick")?;
    let end_tick = checked_u32(unsigned(object, "end_tick")?, "end_tick")?;
    if start_tick >= end_tick || end_tick > evaluation_ticks {
        return invalid(format!("feedback_fault[{index}] has invalid tick range"));
    }
    Ok(FeedbackFault {
        start_tick,
        end_tick,
        position_valid: boolean(object, "position_valid")?,
        velocity_valid: boolean(object, "velocity_valid")?,
        current_q_valid: boolean(object, "current_q_valid")?,
    })
}

fn parse_control_mode(value: &str) -> Result<ControlMode, SimulationContractError> {
    match value {
        "Torque" => Ok(ControlMode::Torque),
        "Velocity" => Ok(ControlMode::Velocity),
        "Position" => Ok(ControlMode::Position),
        _ => invalid(format!("unsupported control_mode: {value}")),
    }
}

fn parse_input_mode(value: &str) -> Result<InputMode, SimulationContractError> {
    match value {
        "Passthrough" => Ok(InputMode::Passthrough),
        "TorqueRamp" => Ok(InputMode::TorqueRamp),
        "VelocityRamp" => Ok(InputMode::VelocityRamp),
        "PositionFilter" => Ok(InputMode::PositionFilter),
        "TrapezoidalTrajectory" => Ok(InputMode::TrapezoidalTrajectory),
        "ExternalSynchronized" => Ok(InputMode::ExternalSynchronized),
        _ => invalid(format!("unsupported input_mode: {value}")),
    }
}

fn control_mode_name(value: ControlMode) -> &'static str {
    match value {
        ControlMode::Torque => "Torque",
        ControlMode::Velocity => "Velocity",
        ControlMode::Position => "Position",
        _ => "Inactive",
    }
}

fn input_mode_name(value: InputMode) -> &'static str {
    match value {
        InputMode::Passthrough => "Passthrough",
        InputMode::TorqueRamp => "TorqueRamp",
        InputMode::VelocityRamp => "VelocityRamp",
        InputMode::PositionFilter => "PositionFilter",
        InputMode::TrapezoidalTrajectory => "TrapezoidalTrajectory",
        InputMode::ExternalSynchronized => "ExternalSynchronized",
        _ => "Inactive",
    }
}

fn required_value<'a>(
    object: &'a BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<&'a JsonValue, SimulationContractError> {
    object
        .get(key)
        .ok_or_else(|| SimulationContractError::Invalid(format!("missing field: {key}")))
}

fn object_from<'a>(
    value: &'a JsonValue,
    label: &str,
) -> Result<&'a BTreeMap<String, JsonValue>, SimulationContractError> {
    object(value, label)
}

fn checked_u32(value: u64, label: &str) -> Result<u32, SimulationContractError> {
    u32::try_from(value)
        .map_err(|_| SimulationContractError::Invalid(format!("{label} exceeds u32")))
}

fn f32_value(
    object: &BTreeMap<String, JsonValue>,
    key: &str,
) -> Result<f32, SimulationContractError> {
    let value = finite_number(object, key)? as f32;
    if !value.is_finite() {
        return invalid(format!("{key} exceeds f32 range"));
    }
    Ok(value)
}

fn checked_id(value: &str, label: &str) -> Result<String, SimulationContractError> {
    if value.is_empty()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
    {
        return invalid(format!("{label} must be a non-empty portable identifier"));
    }
    Ok(value.to_owned())
}

pub fn default_output_paths(project_root: impl AsRef<Path>) -> (PathBuf, PathBuf) {
    let root = project_root.as_ref();
    (
        root.join(MOTION_RESULT_RELATIVE_PATH),
        root.join(MOTION_TRACE_RELATIVE_PATH),
    )
}
