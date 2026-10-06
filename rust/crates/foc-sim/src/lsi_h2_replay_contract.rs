//! Offline H2 Ls(I) replay contract over a frozen, already-converted trace.
//!
//! This module performs file I/O only on the PC.  It cannot arm a target and a
//! passing result is not permission to inject voltage.  `angle_index_valid=0`
//! deliberately keeps the historical fixture from becoming H2-5 evidence.

use std::f64::consts::TAU;
use std::fs;
use std::io::Write as _;
use std::path::{Component, Path, PathBuf};

use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned, JsonValue,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/lsi_h2_replay_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/lsi-h2-replay";
const PULSE_MASK: u32 = (1_u32 << 5) | (1_u32 << 6);

#[derive(Debug)]
pub struct LsiH2ReplayResult {
    pub case_count: usize,
    pub row_count: usize,
    pub metadata_path: PathBuf,
    pub trace_path: PathBuf,
}

#[derive(Clone, Copy, Debug)]
struct TraceSample {
    control_tick: u64,
    current_u_a: f64,
    applied_phase_u_v: f64,
    flags: u32,
}

pub fn run_lsi_h2_replay_contract(
    project_root: &Path,
) -> Result<LsiH2ReplayResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let json = parse_json_bytes(&fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?)?;
    let root = object(&json, "LSI H2 replay root")?;
    if string(root, "contract")? != "fluxrt-lsi-h2-replay" || unsigned(root, "version")? != 1 {
        return invalid("unsupported LSI H2 replay identity");
    }
    let sample_rate_hz = finite_number(root, "sample_rate_hz")?;
    let resistance_ohm = finite_number(root, "resistance_ohm")?;
    if sample_rate_hz <= 0.0 || resistance_ohm <= 0.0 {
        return invalid("sample rate and resistance must be positive");
    }
    let candidates_json = array(required(root, "candidate_inductance_h")?, "candidates")?;
    let mut candidates = Vec::with_capacity(candidates_json.len());
    for value in candidates_json {
        let candidate = match value {
            JsonValue::Number(number) if number.is_finite() && *number > 0.0 => *number,
            _ => return invalid("candidate inductance must be a positive finite number"),
        };
        candidates.push(candidate);
    }
    if candidates.is_empty() {
        return invalid("candidate set is empty");
    }
    let cases = array(required(root, "cases")?, "cases")?;
    if cases.is_empty() {
        return invalid("replay case set is empty");
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(io_error)?;
    let trace_path = result_dir.join("rust-lsi-h2-replay.csv");
    let metadata_path = result_dir.join("rust-lsi-h2-replay-d0.txt");
    let mut output = fs::File::create(&trace_path).map_err(io_error)?;
    writeln!(output, "case_id,angle_index_valid,angle_source,angle_sample_count,angle_span_counts,mechanical_angle_rad,electrical_angle_rad,candidate_inductance_h,transition_count,rmse_a,maximum_absolute_residual_a").map_err(io_error)?;

    let mut row_count = 0_usize;
    let dt_s = 1.0 / sample_rate_hz;
    for value in cases {
        let case = object(value, "replay case")?;
        let case_id = string(case, "id")?;
        validate_case_id(case_id)?;
        let relative_trace = safe_relative_path(string(case, "trace_path")?)?;
        let pole_pairs = unsigned(case, "pole_pairs")?;
        let angle_index_valid = unsigned(case, "angle_index_valid")?;
        let angle_source = string(case, "angle_source")?;
        let angle_sample_count = unsigned(case, "angle_sample_count")?;
        let angle_span_counts = unsigned(case, "angle_span_counts")?;
        let mechanical_angle_rad = finite_number(case, "mechanical_angle_rad")?;
        if pole_pairs == 0 || pole_pairs > 255 || angle_index_valid > 1 {
            return invalid("invalid pole_pairs or angle_index_valid");
        }
        if angle_index_valid == 1
            && (angle_source != "as5600-window" || angle_sample_count < 3 || angle_span_counts > 2)
        {
            return invalid("valid angle index requires a stable AS5600 time window");
        }
        if angle_index_valid == 0 && angle_source != "unavailable-historical" {
            return invalid("invalid angle index must declare unavailable-historical");
        }
        let electrical_angle_rad = if angle_index_valid == 1 {
            (mechanical_angle_rad * pole_pairs as f64).rem_euclid(TAU)
        } else {
            0.0
        };
        let samples = read_trace(&project_root.join(relative_trace))?;
        let transitions = pulse_transitions(&samples);
        if transitions.len() < 3 {
            return invalid("replay case has fewer than three contiguous pulse transitions");
        }
        for inductance_h in &candidates {
            let decay = (-resistance_ohm * dt_s / inductance_h).exp();
            let mut squared_error = 0.0_f64;
            let mut maximum_absolute_residual = 0.0_f64;
            for (current, next_current, voltage) in &transitions {
                let predicted =
                    voltage / resistance_ohm + (current - voltage / resistance_ohm) * decay;
                let residual = next_current - predicted;
                squared_error += residual * residual;
                maximum_absolute_residual = maximum_absolute_residual.max(residual.abs());
            }
            let rmse = (squared_error / transitions.len() as f64).sqrt();
            writeln!(
                output,
                "{case_id},{angle_index_valid},{angle_source},{angle_sample_count},{angle_span_counts},{mechanical_angle_rad:.12},{electrical_angle_rad:.12},{inductance_h:.12},{},{rmse:.12},{maximum_absolute_residual:.12}",
                transitions.len()
            )
            .map_err(io_error)?;
            row_count += 1;
        }
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-lsi-h2-replay\nversion=1\nengine=rust\nworkspace_revision={revision}\ncase_count={}\nrow_count={row_count}\nangle_evidence=BLOCKED_UNTIL_VALID_INDEX\nresult=PASS\n",
            cases.len()
        ),
    )
    .map_err(io_error)?;

    Ok(LsiH2ReplayResult {
        case_count: cases.len(),
        row_count,
        metadata_path,
        trace_path,
    })
}

fn read_trace(path: &Path) -> Result<Vec<TraceSample>, SimulationContractError> {
    let text = fs::read_to_string(path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", path.display()))
    })?;
    let mut lines = text.lines();
    let header = lines
        .next()
        .ok_or_else(|| SimulationContractError::Invalid("replay trace is empty".to_owned()))?;
    let columns: Vec<&str> = header.split(',').collect();
    let index = |name: &str| {
        columns
            .iter()
            .position(|column| *column == name)
            .ok_or_else(|| {
                SimulationContractError::Invalid(format!("replay trace is missing {name}"))
            })
    };
    let tick_index = index("control_tick")?;
    let current_index = index("current_u_a")?;
    let voltage_index = index("applied_phase_u_v")?;
    let flags_index = index("flags")?;
    let mut samples = Vec::new();
    for (line_number, line) in lines.enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.len() != columns.len() {
            return invalid(&format!(
                "trace row {} has wrong column count",
                line_number + 2
            ));
        }
        let sample = TraceSample {
            control_tick: parse_field(fields[tick_index], "control_tick", line_number + 2)?,
            current_u_a: parse_finite(fields[current_index], "current_u_a", line_number + 2)?,
            applied_phase_u_v: parse_finite(
                fields[voltage_index],
                "applied_phase_u_v",
                line_number + 2,
            )?,
            flags: parse_field(fields[flags_index], "flags", line_number + 2)?,
        };
        samples.push(sample);
    }
    if samples.is_empty() {
        return invalid("replay trace has no samples");
    }
    Ok(samples)
}

fn pulse_transitions(samples: &[TraceSample]) -> Vec<(f64, f64, f64)> {
    samples
        .windows(2)
        .filter_map(|pair| {
            let left = pair[0];
            let right = pair[1];
            ((left.flags & PULSE_MASK) != 0
                && (right.flags & PULSE_MASK) != 0
                && right.control_tick == left.control_tick + 1)
                .then_some((left.current_u_a, right.current_u_a, left.applied_phase_u_v))
        })
        .collect()
}

fn validate_case_id(value: &str) -> Result<(), SimulationContractError> {
    if value.is_empty()
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-'))
    {
        return invalid("case id must use ASCII alphanumeric, '_' or '-'");
    }
    Ok(())
}

fn safe_relative_path(value: &str) -> Result<PathBuf, SimulationContractError> {
    let path = Path::new(value);
    if path.is_absolute()
        || path
            .components()
            .any(|component| !matches!(component, Component::Normal(_)))
    {
        return invalid("trace_path must be a normal project-relative path");
    }
    Ok(path.to_path_buf())
}

fn parse_field<T: std::str::FromStr>(
    value: &str,
    name: &str,
    line: usize,
) -> Result<T, SimulationContractError> {
    value.parse::<T>().map_err(|_| {
        SimulationContractError::Invalid(format!("invalid {name} at trace row {line}"))
    })
}

fn parse_finite(value: &str, name: &str, line: usize) -> Result<f64, SimulationContractError> {
    let parsed = parse_field::<f64>(value, name, line)?;
    if !parsed.is_finite() {
        return invalid(&format!("non-finite {name} at trace row {line}"));
    }
    Ok(parsed)
}

fn invalid<T>(message: &str) -> Result<T, SimulationContractError> {
    Err(SimulationContractError::Invalid(message.to_owned()))
}

fn io_error(error: std::io::Error) -> SimulationContractError {
    SimulationContractError::Io(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repository_replay_fixture_runs_but_has_no_angle_evidence() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf();
        let result = run_lsi_h2_replay_contract(&root).unwrap();
        assert_eq!(result.case_count, 1);
        assert_eq!(result.row_count, 4);
        let output = fs::read_to_string(result.trace_path).unwrap();
        assert!(output.lines().skip(1).all(|line| {
            let fields: Vec<_> = line.split(',').collect();
            fields[1] == "0" && fields[2] == "unavailable-historical"
        }));
    }
}
