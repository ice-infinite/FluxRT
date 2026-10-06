//! Active rotating-HFI contract against an anisotropic stationary PMSM model.
//!
//! This is an S3 design model, not target permission.  It deliberately holds
//! the rotor stationary so the first P5.5 gate can isolate carrier generation,
//! the salient `Ld/Lq` current response and negative-sequence demodulation from
//! startup, torque production and BEMF hand-over.

use std::f32::consts::PI;
use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use foc_algorithm::{AlphaBeta, RotatingHfSequenceParam, RotatingHfSequenceState};

use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/hfi_active_plant_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/hfi";

#[derive(Debug)]
pub struct HfiContractResult {
    pub case_count: usize,
    pub metadata_path: PathBuf,
    pub trace_path: PathBuf,
}

#[derive(Clone, Copy, Debug, Default)]
struct StationarySalientPlant {
    current: AlphaBeta,
}

impl StationarySalientPlant {
    fn step(
        &mut self,
        voltage: AlphaBeta,
        theta_rad: f32,
        resistance_ohm: f32,
        ld_h: f32,
        lq_h: f32,
        dt_s: f32,
    ) {
        let c = theta_rad.cos();
        let s = theta_rad.sin();
        let laa = ld_h * c * c + lq_h * s * s;
        let lab = (ld_h - lq_h) * c * s;
        let lbb = ld_h * s * s + lq_h * c * c;
        let determinant = laa * lbb - lab * lab;
        let rhs_alpha = voltage.alpha - resistance_ohm * self.current.alpha;
        let rhs_beta = voltage.beta - resistance_ohm * self.current.beta;
        let di_alpha = (lbb * rhs_alpha - lab * rhs_beta) / determinant;
        let di_beta = (-lab * rhs_alpha + laa * rhs_beta) / determinant;
        self.current.alpha += di_alpha * dt_s;
        self.current.beta += di_beta * dt_s;
    }
}

pub fn run_hfi_contract(project_root: &Path) -> Result<HfiContractResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let json = parse_json_bytes(&bytes)?;
    let root = object(&json, "HFI scenario root")?;
    if string(root, "contract")? != "fluxrt-hfi-active-plant" || unsigned(root, "version")? != 1 {
        return Err(SimulationContractError::Invalid(
            "unsupported HFI scenario identity".to_owned(),
        ));
    }
    let control_frequency_hz = u32_value(root, "control_frequency_hz")?;
    let samples = u32_value(root, "samples")?;
    let injection = object(required(root, "injection")?, "injection")?;
    let amplitude_v = f32_value(injection, "amplitude_v")?;
    let frequency_hz = f32_value(injection, "frequency_hz")?;
    let demod_alpha = f32_value(injection, "demod_alpha")?;
    let minimum_response_a = f32_value(injection, "minimum_response_a")?;
    let minimum_relative_saliency = f32_value(injection, "minimum_relative_saliency")?;
    let maximum_angle_error_rad = f32_value(injection, "maximum_angle_error_rad")?;
    if control_frequency_hz == 0
        || samples == 0
        || samples > control_frequency_hz.saturating_mul(10)
        || !amplitude_v.is_finite()
        || amplitude_v <= 0.0
        || !frequency_hz.is_finite()
        || frequency_hz <= 0.0
        || frequency_hz >= control_frequency_hz as f32 * 0.5
        || !(0.0..=1.0).contains(&demod_alpha)
        || minimum_response_a < 0.0
        || minimum_relative_saliency < 0.0
        || maximum_angle_error_rad <= 0.0
    {
        return Err(SimulationContractError::Invalid(
            "invalid HFI common configuration".to_owned(),
        ));
    }
    let cases = array(required(root, "cases")?, "cases")?;
    if cases.is_empty() {
        return Err(SimulationContractError::Invalid(
            "HFI scenario has no cases".to_owned(),
        ));
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", result_dir.display()))
    })?;
    let trace_path = result_dir.join("rust-hfi-trace.csv");
    let metadata_path = result_dir.join("rust-hfi-d0.txt");
    let mut trace = fs::File::create(&trace_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", trace_path.display()))
    })?;
    writeln!(trace, "case_id,true_theta_mod_pi_rad,estimated_theta_mod_pi_rad,angle_error_rad,response_magnitude_a,relative_saliency,peak_current_a,valid")
        .map_err(io_error)?;

    let dt_s = 1.0 / control_frequency_hz as f32;
    for value in cases {
        let case = object(value, "HFI case")?;
        let case_id = string(case, "id")?;
        let resistance_ohm = f32_value(case, "resistance_ohm")?;
        let ld_h = f32_value(case, "ld_h")?;
        let lq_h = f32_value(case, "lq_h")?;
        let theta_rad = f32_value(case, "theta_rad")?;
        let expected_valid = bool_u32(case, "expected_valid")?;
        if resistance_ohm < 0.0 || ld_h <= 0.0 || lq_h <= 0.0 {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} has invalid motor parameters"
            )));
        }

        let phase_offset_rad =
            negative_sequence_phase(resistance_ohm, ld_h, lq_h, frequency_hz, dt_s);
        let param = RotatingHfSequenceParam {
            amplitude: amplitude_v,
            freq_hz: frequency_hz,
            ts: dt_s,
            demod_alpha,
            phase_offset_rad,
        };
        let mut observer = RotatingHfSequenceState::default();
        let mut plant = StationarySalientPlant::default();
        let mut peak_current_a = 0.0_f32;
        for _ in 0..samples {
            plant.step(
                observer.voltage,
                theta_rad,
                resistance_ohm,
                ld_h,
                lq_h,
                dt_s,
            );
            peak_current_a = peak_current_a.max(
                (plant.current.alpha * plant.current.alpha
                    + plant.current.beta * plant.current.beta)
                    .sqrt(),
            );
            observer.update(&param, plant.current);
        }
        let true_theta_mod_pi_rad = theta_rad.rem_euclid(PI);
        let angle_error_rad = mod_pi_error(observer.theta_est_mod_pi_rad, true_theta_mod_pi_rad);
        let relative_saliency = (lq_h - ld_h).abs() / ld_h.max(lq_h);
        let valid = relative_saliency >= minimum_relative_saliency
            && observer.response_magnitude_a >= minimum_response_a
            && angle_error_rad <= maximum_angle_error_rad;
        if valid != expected_valid {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} validity mismatch: expected {expected_valid}, got {valid}; error={angle_error_rad:.6}, response={:.6}",
                observer.response_magnitude_a
            )));
        }
        writeln!(
            trace,
            "{case_id},{true_theta_mod_pi_rad:.9},{:.9},{angle_error_rad:.9},{:.9},{relative_saliency:.9},{peak_current_a:.9},{}",
            observer.theta_est_mod_pi_rad,
            observer.response_magnitude_a,
            u32::from(valid),
        )
        .map_err(io_error)?;
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-hfi-active-plant\nversion=1\nengine=rust\nworkspace_revision={revision}\ncase_count={}\nresult=PASS\n",
            cases.len()
        ),
    )
    .map_err(|error| {
        SimulationContractError::Io(format!("cannot write {}: {error}", metadata_path.display()))
    })?;

    Ok(HfiContractResult {
        case_count: cases.len(),
        metadata_path,
        trace_path,
    })
}

fn negative_sequence_phase(
    resistance_ohm: f32,
    ld_h: f32,
    lq_h: f32,
    frequency_hz: f32,
    dt_s: f32,
) -> f32 {
    // Exact frequency response of the explicit-Euler plant used by this
    // contract.  Calibrating against the continuous 1/(R+jwL) response would
    // leave a deterministic one-sample/discretisation angle bias and hide it in
    // the HFI acceptance tolerance.
    let carrier_step = 2.0 * PI * frequency_hz * dt_s;
    let (hd_real, hd_imag) = discrete_axis_response(resistance_ohm, ld_h, dt_s, carrier_step);
    let (hq_real, hq_imag) = discrete_axis_response(resistance_ohm, lq_h, dt_s, carrier_step);
    // The negative-sequence coefficient is conj(Hd - Hq)/2 because the
    // stationary complex current contains the conjugated response multiplying
    // exp(j*(2*theta-carrier)).
    (-0.5 * (hd_imag - hq_imag)).atan2(0.5 * (hd_real - hq_real))
}

fn discrete_axis_response(
    resistance_ohm: f32,
    inductance_h: f32,
    dt_s: f32,
    carrier_step_rad: f32,
) -> (f32, f32) {
    let a = 1.0 - resistance_ohm * dt_s / inductance_h;
    let b = dt_s / inductance_h;
    let denominator_real = 1.0 - a * carrier_step_rad.cos();
    let denominator_imag = a * carrier_step_rad.sin();
    let norm = denominator_real * denominator_real + denominator_imag * denominator_imag;
    (b * denominator_real / norm, -b * denominator_imag / norm)
}

fn mod_pi_error(estimate: f32, truth: f32) -> f32 {
    ((estimate - truth + 0.5 * PI).rem_euclid(PI) - 0.5 * PI).abs()
}

fn u32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<u32, SimulationContractError> {
    u32::try_from(unsigned(object, key)?)
        .map_err(|_| SimulationContractError::Invalid(format!("{key} does not fit in uint32")))
}

fn f32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<f32, SimulationContractError> {
    let value = finite_number(object, key)?;
    if value < -(f32::MAX as f64) || value > f32::MAX as f64 {
        return Err(SimulationContractError::Invalid(format!(
            "{key} does not fit in float32"
        )));
    }
    Ok(value as f32)
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

fn io_error(error: std::io::Error) -> SimulationContractError {
    SimulationContractError::Io(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repository_hfi_matrix_runs() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf();
        let result = run_hfi_contract(&root).unwrap();
        assert_eq!(result.case_count, 12);
    }
}
