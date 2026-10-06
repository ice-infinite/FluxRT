//! Full-speed sensorless angle-chain scenarios for the Rust PC engine.

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use foc_algorithm::{wrap_angle_minus_pi_to_pi, AlphaBeta};
use foc_control::{
    HfiPolarityConfig, SensorlessAngleChain, SensorlessAngleSource, SensorlessChainConfig,
    SensorlessChainInput, SensorlessChainOutput, SensorlessChainStage, SensorlessFusionConfig,
};

use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/sensorless_full_speed_chain_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/sensorless-full-speed";

#[derive(Debug)]
pub struct SensorlessFullSpeedContractResult {
    pub case_count: usize,
    pub row_count: usize,
    pub metadata_path: PathBuf,
    pub trace_path: PathBuf,
    pub metrics_path: PathBuf,
}

#[derive(Clone, Copy, Debug)]
struct Case {
    duration_s: f32,
    initial_theta_rad: f32,
    speed_initial_rad_s: f32,
    speed_peak_rad_s: f32,
    speed_final_rad_s: f32,
    ramp_up_start_s: f32,
    ramp_up_end_s: f32,
    ramp_down_start_s: f32,
    ramp_down_end_s: f32,
    bemf_dropout_start_s: f32,
    bemf_dropout_end_s: f32,
    hfi_dropout_start_s: f32,
    hfi_dropout_end_s: f32,
    expected_source_mask: u32,
    expected_transition_mask: u32,
    expected_fallback: bool,
    maximum_rms_error_rad: f32,
    maximum_angle_step_rad: f32,
}

#[derive(Clone, Copy, Debug, Default)]
struct Metrics {
    rms_angle_error_rad: f32,
    maximum_angle_error_rad: f32,
    maximum_angle_step_rad: f32,
    unreliable_fraction: f32,
    source_mask: u32,
    transition_mask: u32,
    fallback_seen: bool,
    final_reliable: bool,
}

pub fn run_sensorless_full_speed_contract(
    project_root: &Path,
) -> Result<SensorlessFullSpeedContractResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let json = parse_json_bytes(&bytes)?;
    let root = object(&json, "sensorless full-speed root")?;
    if string(root, "contract")? != "fluxrt-sensorless-full-speed-chain"
        || unsigned(root, "version")? != 1
    {
        return Err(SimulationContractError::Invalid(
            "unsupported sensorless full-speed identity".to_owned(),
        ));
    }
    let control_frequency_hz = u32_value(root, "control_frequency_hz")?;
    let trace_decimation = u32_value(root, "trace_decimation")?;
    let minimum_bemf_speed_rad_s = f32_value(root, "minimum_bemf_speed_rad_s")?;
    let hfi_response_a = f32_value(root, "hfi_response_a")?;
    let noise_a = f32_value(root, "noise_a")?;
    if control_frequency_hz == 0
        || trace_decimation == 0
        || !finite_positive(minimum_bemf_speed_rad_s)
        || !finite_positive(hfi_response_a)
        || !finite_nonnegative(noise_a)
    {
        return Err(SimulationContractError::Invalid(
            "invalid sensorless full-speed common configuration".to_owned(),
        ));
    }
    let cases = array(required(root, "cases")?, "sensorless full-speed cases")?;
    if cases.is_empty() {
        return Err(SimulationContractError::Invalid(
            "sensorless full-speed scenario has no cases".to_owned(),
        ));
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", result_dir.display()))
    })?;
    let trace_path = result_dir.join("rust-sensorless-full-speed-trace.csv");
    let metrics_path = result_dir.join("rust-sensorless-full-speed-metrics.csv");
    let metadata_path = result_dir.join("rust-sensorless-full-speed-d0.txt");
    let mut trace = fs::File::create(&trace_path).map_err(io_error_path(&trace_path))?;
    let mut metrics_file = fs::File::create(&metrics_path).map_err(io_error_path(&metrics_path))?;
    writeln!(trace, "case_id,tick,time_s,true_angle_rad,electrical_speed_rad_s,estimated_angle_rad,angle_error_rad,source,reliable,fallback_required,stage,hfi_valid,bemf_valid,injection_alpha_v,injection_beta_v")
        .map_err(io_error)?;
    writeln!(metrics_file, "case_id,rms_angle_error_rad,maximum_angle_error_rad,maximum_angle_step_rad,unreliable_fraction,source_mask,transition_mask,fallback_seen,final_reliable")
        .map_err(io_error)?;

    let dt_s = 1.0 / control_frequency_hz as f32;
    let config = chain_config(dt_s);
    let mut row_count = 0_usize;
    for value in cases {
        let object = object(value, "sensorless full-speed case")?;
        let case_id = string(object, "id")?;
        let case = parse_case(object)?;
        validate_case(case, control_frequency_hz, case_id)?;
        let (case_metrics, rows) = run_case(
            &mut trace,
            case_id,
            case,
            &config,
            control_frequency_hz,
            trace_decimation,
            minimum_bemf_speed_rad_s,
            hfi_response_a,
            noise_a,
        )?;
        row_count += rows;
        if case_metrics.source_mask & case.expected_source_mask != case.expected_source_mask
            || case_metrics.transition_mask & case.expected_transition_mask
                != case.expected_transition_mask
            || case_metrics.fallback_seen != case.expected_fallback
            || case_metrics.rms_angle_error_rad > case.maximum_rms_error_rad
            || case_metrics.maximum_angle_step_rad > case.maximum_angle_step_rad
            || !case_metrics.final_reliable
        {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} failed gate: metrics={case_metrics:?}, expected_source_mask={}, expected_transition_mask={} expected_fallback={} max_rms={} max_step={}",
                case.expected_source_mask,
                case.expected_transition_mask,
                case.expected_fallback,
                case.maximum_rms_error_rad,
                case.maximum_angle_step_rad,
            )));
        }
        writeln!(
            metrics_file,
            "{case_id},{:.9},{:.9},{:.9},{:.9},{},{},{},{}",
            case_metrics.rms_angle_error_rad,
            case_metrics.maximum_angle_error_rad,
            case_metrics.maximum_angle_step_rad,
            case_metrics.unreliable_fraction,
            case_metrics.source_mask,
            case_metrics.transition_mask,
            u32::from(case_metrics.fallback_seen),
            u32::from(case_metrics.final_reliable),
        )
        .map_err(io_error)?;
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-sensorless-full-speed-chain\nversion=1\nengine=rust-product-chain\nworkspace_revision={revision}\ncase_count={}\nrow_count={row_count}\nresult=PASS\nmodel_scope=design-not-hardware-truth\n",
            cases.len()
        ),
    )
    .map_err(|error| {
        SimulationContractError::Io(format!("cannot write {}: {error}", metadata_path.display()))
    })?;
    Ok(SensorlessFullSpeedContractResult {
        case_count: cases.len(),
        row_count,
        metadata_path,
        trace_path,
        metrics_path,
    })
}

#[allow(clippy::too_many_arguments)]
fn run_case(
    trace: &mut fs::File,
    case_id: &str,
    case: Case,
    config: &SensorlessChainConfig,
    control_frequency_hz: u32,
    trace_decimation: u32,
    minimum_bemf_speed_rad_s: f32,
    hfi_response_a: f32,
    noise_a: f32,
) -> Result<(Metrics, usize), SimulationContractError> {
    let tick_count = (case.duration_s * control_frequency_hz as f32).round() as u32;
    let dt_s = 1.0 / control_frequency_hz as f32;
    let mut chain = SensorlessAngleChain::default();
    let mut previous = SensorlessChainOutput::default();
    let mut theta = case
        .initial_theta_rad
        .rem_euclid(2.0 * std::f32::consts::PI);
    let mut squared_error_sum = 0.0_f32;
    let mut maximum_angle_error_rad = 0.0_f32;
    let mut maximum_angle_step_rad = 0.0_f32;
    let mut reliable_samples = 0_u32;
    let mut unreliable_samples = 0_u32;
    let mut source_mask = 0_u32;
    let mut transition_mask = 0_u32;
    let mut last_reliable_source = SensorlessAngleSource::Unavailable;
    let mut fallback_seen = false;
    let mut previous_reliable_angle: Option<f32> = None;
    let mut rows = 0_usize;

    for tick in 0..tick_count {
        let time_s = tick as f32 * dt_s;
        let speed = speed_profile(case, time_s);
        let hfi_permitted = !inside(time_s, case.hfi_dropout_start_s, case.hfi_dropout_end_s);
        let bemf_dropout = inside(time_s, case.bemf_dropout_start_s, case.bemf_dropout_end_s);
        let high_frequency_current =
            synthetic_hfi_current(previous, theta, hfi_response_a, noise_a, tick);
        let measured_current = synthetic_polarity_current(previous, theta);
        let bemf_valid = speed.abs() >= minimum_bemf_speed_rad_s && !bemf_dropout;
        let bemf_angle =
            (theta + 0.005 * (tick as f32 * 0.017).sin()).rem_euclid(2.0 * std::f32::consts::PI);
        let output = chain.step(
            config,
            SensorlessChainInput {
                request_reset: false,
                injection_permitted: hfi_permitted,
                high_frequency_current_alpha_beta: high_frequency_current,
                measured_current_alpha_beta: measured_current,
                bemf_angle_rad: bemf_angle,
                bemf_valid,
                bemf_electrical_speed_rad_s: speed,
            },
        );
        if output.fusion.reliable {
            let error = wrap_angle_minus_pi_to_pi(output.fusion.angle_rad - theta).abs();
            squared_error_sum += error * error;
            maximum_angle_error_rad = maximum_angle_error_rad.max(error);
            reliable_samples += 1;
            if let Some(last) = previous_reliable_angle {
                maximum_angle_step_rad = maximum_angle_step_rad
                    .max(wrap_angle_minus_pi_to_pi(output.fusion.angle_rad - last).abs());
            }
            previous_reliable_angle = Some(output.fusion.angle_rad);
            source_mask |= source_bit(output.fusion.source);
            if last_reliable_source != SensorlessAngleSource::Unavailable
                && last_reliable_source != output.fusion.source
            {
                transition_mask |= transition_bit(last_reliable_source, output.fusion.source);
            }
            last_reliable_source = output.fusion.source;
        } else {
            unreliable_samples += 1;
            previous_reliable_angle = None;
            last_reliable_source = SensorlessAngleSource::Unavailable;
        }
        fallback_seen |= output.fusion.fallback_required;
        let angle_error = if output.fusion.reliable {
            wrap_angle_minus_pi_to_pi(output.fusion.angle_rad - theta).abs()
        } else {
            0.0
        };
        if tick % trace_decimation == 0 || tick + 1 == tick_count {
            writeln!(
                trace,
                "{case_id},{tick},{time_s:.9},{theta:.9},{speed:.9},{:.9},{angle_error:.9},{},{},{},{},{},{},{:.9},{:.9}",
                output.fusion.angle_rad,
                output.fusion.source as u32,
                u32::from(output.fusion.reliable),
                u32::from(output.fusion.fallback_required),
                output.stage as u32,
                u32::from(output.hfi_valid),
                u32::from(bemf_valid),
                output.injection_voltage_alpha_beta.alpha,
                output.injection_voltage_alpha_beta.beta,
            )
            .map_err(io_error)?;
            rows += 1;
        }
        previous = output;
        theta = (theta + speed * dt_s).rem_euclid(2.0 * std::f32::consts::PI);
    }
    if reliable_samples == 0 {
        return Err(SimulationContractError::Invalid(format!(
            "case {case_id} never produced a reliable angle"
        )));
    }
    Ok((
        Metrics {
            rms_angle_error_rad: (squared_error_sum / reliable_samples as f32).sqrt(),
            maximum_angle_error_rad,
            maximum_angle_step_rad,
            unreliable_fraction: unreliable_samples as f32 / tick_count as f32,
            source_mask,
            transition_mask,
            fallback_seen,
            final_reliable: previous.fusion.reliable,
        },
        rows,
    ))
}

fn chain_config(dt_s: f32) -> SensorlessChainConfig {
    SensorlessChainConfig {
        hfi: foc_algorithm::RotatingHfSequenceParam {
            amplitude: 1.0,
            freq_hz: 1_000.0,
            ts: dt_s,
            demod_alpha: 0.2,
            phase_offset_rad: 0.0,
        },
        minimum_hfi_response_a: 0.02,
        maximum_hfi_electrical_speed_rad_s: 180.0,
        hfi_axis_stable_samples: 24,
        polarity: HfiPolarityConfig::default(),
        fusion: SensorlessFusionConfig {
            blend_enter_speed_rad_s: 81.3,
            hfi_reenter_speed_rad_s: 61.7,
            bemf_enter_speed_rad_s: 141.1,
            bemf_exit_speed_rad_s: 111.3,
            weight_slew_per_sample: 0.02,
            stable_samples: 24,
            invalid_timeout_samples: 120,
            ..SensorlessFusionConfig::default()
        },
        ..SensorlessChainConfig::default()
    }
}

fn synthetic_hfi_current(
    previous: SensorlessChainOutput,
    theta: f32,
    response_a: f32,
    noise_a: f32,
    tick: u32,
) -> AlphaBeta {
    if previous.stage == SensorlessChainStage::Polarity
        || magnitude(previous.injection_voltage_alpha_beta) < 0.75
    {
        return AlphaBeta::default();
    }
    let carrier = previous
        .injection_voltage_alpha_beta
        .beta
        .atan2(previous.injection_voltage_alpha_beta.alpha);
    let phase = 2.0 * theta - carrier;
    AlphaBeta {
        alpha: response_a * phase.cos() + noise_a * (tick as f32 * 0.731).sin(),
        beta: response_a * phase.sin() + noise_a * (tick as f32 * 0.527 + 0.4).sin(),
    }
}

fn synthetic_polarity_current(previous: SensorlessChainOutput, theta: f32) -> AlphaBeta {
    if previous.stage != SensorlessChainStage::Polarity
        || magnitude(previous.injection_voltage_alpha_beta) == 0.0
    {
        return AlphaBeta::default();
    }
    let axis = previous.hfi_angle_mod_pi_rad;
    let projection = previous.injection_voltage_alpha_beta.alpha * axis.cos()
        + previous.injection_voltage_alpha_beta.beta * axis.sin();
    let add_pi = theta.rem_euclid(2.0 * std::f32::consts::PI) >= std::f32::consts::PI;
    let stronger_positive = !add_pi;
    let magnitude = if (projection > 0.0) == stronger_positive {
        0.12
    } else {
        0.05
    };
    AlphaBeta {
        alpha: projection.signum() * magnitude * axis.cos(),
        beta: projection.signum() * magnitude * axis.sin(),
    }
}

fn speed_profile(case: Case, time_s: f32) -> f32 {
    if time_s < case.ramp_up_start_s {
        case.speed_initial_rad_s
    } else if time_s < case.ramp_up_end_s {
        lerp(
            case.speed_initial_rad_s,
            case.speed_peak_rad_s,
            ratio(time_s, case.ramp_up_start_s, case.ramp_up_end_s),
        )
    } else if time_s < case.ramp_down_start_s {
        case.speed_peak_rad_s
    } else if time_s < case.ramp_down_end_s {
        lerp(
            case.speed_peak_rad_s,
            case.speed_final_rad_s,
            ratio(time_s, case.ramp_down_start_s, case.ramp_down_end_s),
        )
    } else {
        case.speed_final_rad_s
    }
}

fn source_bit(source: SensorlessAngleSource) -> u32 {
    match source {
        SensorlessAngleSource::Unavailable => 0,
        SensorlessAngleSource::Hfi => 1,
        SensorlessAngleSource::Blend => 2,
        SensorlessAngleSource::Bemf => 4,
    }
}

fn transition_bit(from: SensorlessAngleSource, to: SensorlessAngleSource) -> u32 {
    match (from, to) {
        (SensorlessAngleSource::Hfi, SensorlessAngleSource::Blend) => 1 << 0,
        (SensorlessAngleSource::Blend, SensorlessAngleSource::Bemf) => 1 << 1,
        (SensorlessAngleSource::Bemf, SensorlessAngleSource::Blend) => 1 << 2,
        (SensorlessAngleSource::Blend, SensorlessAngleSource::Hfi) => 1 << 3,
        (SensorlessAngleSource::Hfi, SensorlessAngleSource::Bemf) => 1 << 4,
        (SensorlessAngleSource::Bemf, SensorlessAngleSource::Hfi) => 1 << 5,
        _ => 0,
    }
}

fn inside(value: f32, start: f32, end: f32) -> bool {
    value >= start && value < end
}

fn ratio(value: f32, start: f32, end: f32) -> f32 {
    ((value - start) / (end - start)).clamp(0.0, 1.0)
}

fn lerp(start: f32, end: f32, ratio: f32) -> f32 {
    start + (end - start) * ratio
}

fn magnitude(value: AlphaBeta) -> f32 {
    (value.alpha * value.alpha + value.beta * value.beta).sqrt()
}

fn validate_case(
    case: Case,
    control_frequency_hz: u32,
    case_id: &str,
) -> Result<(), SimulationContractError> {
    let tick_count = case.duration_s * control_frequency_hz as f32;
    let times_ordered = case.ramp_up_start_s >= 0.0
        && case.ramp_up_start_s < case.ramp_up_end_s
        && case.ramp_up_end_s <= case.ramp_down_start_s
        && case.ramp_down_start_s < case.ramp_down_end_s
        && case.ramp_down_end_s <= case.duration_s;
    if !finite_positive(case.duration_s)
        || tick_count > control_frequency_hz as f32 * 10.0
        || !case.initial_theta_rad.is_finite()
        || !case.speed_initial_rad_s.is_finite()
        || !case.speed_peak_rad_s.is_finite()
        || !case.speed_final_rad_s.is_finite()
        || !times_ordered
        || !finite_positive(case.maximum_rms_error_rad)
        || !finite_positive(case.maximum_angle_step_rad)
        || case.expected_source_mask == 0
        || case.expected_source_mask & !7 != 0
        || case.expected_transition_mask & !0x3f != 0
    {
        return Err(SimulationContractError::Invalid(format!(
            "case {case_id} has invalid fields"
        )));
    }
    Ok(())
}

fn parse_case(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<Case, SimulationContractError> {
    Ok(Case {
        duration_s: f32_value(object, "duration_s")?,
        initial_theta_rad: f32_value(object, "initial_theta_rad")?,
        speed_initial_rad_s: f32_value(object, "speed_initial_rad_s")?,
        speed_peak_rad_s: f32_value(object, "speed_peak_rad_s")?,
        speed_final_rad_s: f32_value(object, "speed_final_rad_s")?,
        ramp_up_start_s: f32_value(object, "ramp_up_start_s")?,
        ramp_up_end_s: f32_value(object, "ramp_up_end_s")?,
        ramp_down_start_s: f32_value(object, "ramp_down_start_s")?,
        ramp_down_end_s: f32_value(object, "ramp_down_end_s")?,
        bemf_dropout_start_s: f32_value(object, "bemf_dropout_start_s")?,
        bemf_dropout_end_s: f32_value(object, "bemf_dropout_end_s")?,
        hfi_dropout_start_s: f32_value(object, "hfi_dropout_start_s")?,
        hfi_dropout_end_s: f32_value(object, "hfi_dropout_end_s")?,
        expected_source_mask: u32_value(object, "expected_source_mask")?,
        expected_transition_mask: u32_value(object, "expected_transition_mask")?,
        expected_fallback: bool_u32(object, "expected_fallback")?,
        maximum_rms_error_rad: f32_value(object, "maximum_rms_error_rad")?,
        maximum_angle_step_rad: f32_value(object, "maximum_angle_step_rad")?,
    })
}

fn f32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<f32, SimulationContractError> {
    let value = finite_number(object, key)? as f32;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(SimulationContractError::Invalid(format!(
            "{key} does not fit float32"
        )))
    }
}

fn u32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<u32, SimulationContractError> {
    u32::try_from(unsigned(object, key)?)
        .map_err(|_| SimulationContractError::Invalid(format!("{key} does not fit uint32")))
}

fn bool_u32(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<bool, SimulationContractError> {
    match u32_value(object, key)? {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(SimulationContractError::Invalid(format!(
            "{key} must be 0 or 1"
        ))),
    }
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn io_error(error: std::io::Error) -> SimulationContractError {
    SimulationContractError::Io(error.to_string())
}

fn io_error_path(path: &Path) -> impl FnOnce(std::io::Error) -> SimulationContractError + '_ {
    move |error| SimulationContractError::Io(format!("cannot create {}: {error}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repository_sensorless_full_speed_matrix_runs() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf();
        let result = run_sensorless_full_speed_contract(&root).unwrap();
        assert_eq!(result.case_count, 9);
        assert!(result.row_count > 0);
    }
}
