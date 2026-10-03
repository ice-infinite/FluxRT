//! Shared advanced-FOC policy matrix executed by the Rust PC harness.

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use foc_algorithm::{AlphaBeta, Dq};
use foc_control::{
    AdvancedFocConfig, AdvancedFocInput, AdvancedFocSupervisor, CurrentCommand, MotorParameters,
    ADVANCED_FOC_CONFIG_VERSION,
};

use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/advanced_foc_matrix_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/advanced";

#[derive(Debug)]
pub struct AdvancedContractResult {
    pub case_count: usize,
    pub metadata_path: PathBuf,
    pub trace_path: PathBuf,
}

pub fn run_advanced_contract(
    project_root: &Path,
) -> Result<AdvancedContractResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let json = parse_json_bytes(&bytes)?;
    let root = object(&json, "advanced scenario root")?;
    if string(root, "contract")? != "fluxrt-advanced-foc-scenario"
        || unsigned(root, "version")? != 1
    {
        return Err(SimulationContractError::Invalid(
            "unsupported advanced scenario identity".to_owned(),
        ));
    }
    let frequency = u32_value(root, "control_frequency_hz")?;
    let motor = parse_motor(object(required(root, "motor")?, "motor")?)?;
    let base_config = parse_config(object(required(root, "config")?, "config")?)?;
    let cases = array(required(root, "cases")?, "cases")?;
    if cases.is_empty() {
        return Err(SimulationContractError::Invalid(
            "advanced scenario has no cases".to_owned(),
        ));
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", result_dir.display()))
    })?;
    let trace_path = result_dir.join("rust-advanced-trace.csv");
    let metadata_path = result_dir.join("rust-advanced-d0.txt");
    let mut trace = fs::File::create(&trace_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", trace_path.display()))
    })?;
    writeln!(trace, "case_id,id_ref_a,iq_ref_a,vd_ff_v,vq_ff_v,injection_alpha_v,injection_beta_v,voltage_limit_v,region,modulation,active_features,current_limited,hfi_valid,flying_state,flying_angle_rad,flying_speed_rad_s")
        .map_err(io_error)?;

    for value in cases {
        let case = object(value, "advanced case")?;
        let case_id = string(case, "id")?;
        let mut config = base_config;
        config.enabled_features = u32_value(case, "features")?;
        let mut supervisor = AdvancedFocSupervisor::default();
        supervisor
            .configure(config, &motor, frequency)
            .map_err(|error| {
                SimulationContractError::Invalid(format!(
                    "case {case_id} configuration rejected: {error:?}"
                ))
            })?;
        let input = parse_input(case)?;
        let steps = u32_value(case, "steps")?;
        if steps == 0 || steps > frequency.saturating_mul(10) {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} has invalid steps"
            )));
        }
        let mut output = Default::default();
        for _ in 0..steps {
            output = supervisor.step(&motor, input).map_err(|error| {
                SimulationContractError::Invalid(format!("case {case_id} step rejected: {error:?}"))
            })?;
        }
        writeln!(
            trace,
            "{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{},{},{},{},{:.9},{:.9}",
            case_id,
            output.current_reference.id_ref_a,
            output.current_reference.iq_ref_a,
            output.voltage_feedforward_dq.d,
            output.voltage_feedforward_dq.q,
            output.injection_voltage_alpha_beta.alpha,
            output.injection_voltage_alpha_beta.beta,
            output.voltage_limit_v,
            output.region as u32,
            output.modulation_mode as u32,
            output.active_features,
            u32::from(output.current_limited),
            u32::from(output.hfi_angle_candidate_valid),
            output.flying_start_state as u32,
            output.flying_start_angle_rad,
            output.flying_start_speed_rad_s,
        )
        .map_err(io_error)?;
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-advanced-foc-scenario\nversion=1\nengine=rust\nworkspace_revision={revision}\ncase_count={}\nresult=PASS\n",
            cases.len()
        ),
    )
    .map_err(|error| {
        SimulationContractError::Io(format!("cannot write {}: {error}", metadata_path.display()))
    })?;

    Ok(AdvancedContractResult {
        case_count: cases.len(),
        metadata_path,
        trace_path,
    })
}

fn parse_motor(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<MotorParameters, SimulationContractError> {
    Ok(MotorParameters {
        pole_pairs: u32_value(object, "pole_pairs")? as u8,
        stator_resistance_ohm: f32_value(object, "stator_resistance_ohm")?,
        ld_h: f32_value(object, "ld_h")?,
        lq_h: f32_value(object, "lq_h")?,
        flux_linkage_wb: f32_value(object, "flux_linkage_wb")?,
        rated_current_a: f32_value(object, "rated_current_a")?,
        max_speed_rpm: f32_value(object, "max_speed_rpm")?,
        nominal_bus_voltage_v: f32_value(object, "nominal_bus_voltage_v")?,
        inertia_kg_m2: f32_value(object, "inertia_kg_m2")?,
        viscous_friction_nm_s: f32_value(object, "viscous_friction_nm_s")?,
    })
}

fn parse_config(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<AdvancedFocConfig, SimulationContractError> {
    Ok(AdvancedFocConfig {
        struct_size: core::mem::size_of::<AdvancedFocConfig>() as u32,
        version: ADVANCED_FOC_CONFIG_VERSION,
        enabled_features: 0,
        region_update_divider: u32_value(object, "region_update_divider")?,
        current_limit_a: f32_value(object, "current_limit_a")?,
        mtpa_min_current_a: f32_value(object, "mtpa_min_current_a")?,
        mtpa_search_steps: u32_value(object, "mtpa_search_steps")?,
        weakening_entry_utilization: f32_value(object, "weakening_entry_utilization")?,
        weakening_exit_utilization: f32_value(object, "weakening_exit_utilization")?,
        weakening_kp_a_per_v: f32_value(object, "weakening_kp_a_per_v")?,
        weakening_id_min_a: f32_value(object, "weakening_id_min_a")?,
        weakening_slew_a_per_s: f32_value(object, "weakening_slew_a_per_s")?,
        mtpv_entry_electrical_speed_rad_s: f32_value(object, "mtpv_entry_electrical_speed_rad_s")?,
        mtpv_exit_electrical_speed_rad_s: f32_value(object, "mtpv_exit_electrical_speed_rad_s")?,
        mtpv_search_steps: u32_value(object, "mtpv_search_steps")?,
        decoupling_gain: f32_value(object, "decoupling_gain")?,
        dpwm_entry_modulation: f32_value(object, "dpwm_entry_modulation")?,
        dpwm_exit_modulation: f32_value(object, "dpwm_exit_modulation")?,
        dpwm_mode: u32_value(object, "dpwm_mode")?,
        overmodulation_max_voltage_ratio: f32_value(object, "overmodulation_max_voltage_ratio")?,
        overmodulation_entry_modulation: f32_value(object, "overmodulation_entry_modulation")?,
        overmodulation_exit_modulation: f32_value(object, "overmodulation_exit_modulation")?,
        hfi_amplitude_v: f32_value(object, "hfi_amplitude_v")?,
        hfi_frequency_hz: f32_value(object, "hfi_frequency_hz")?,
        hfi_demod_alpha: f32_value(object, "hfi_demod_alpha")?,
        hfi_high_pass_alpha: f32_value(object, "hfi_high_pass_alpha")?,
        hfi_min_response_a: f32_value(object, "hfi_min_response_a")?,
        hfi_max_electrical_speed_rad_s: f32_value(object, "hfi_max_electrical_speed_rad_s")?,
        hfi_settling_samples: u32_value(object, "hfi_settling_samples")?,
        flying_start_min_electrical_speed_rad_s: f32_value(
            object,
            "flying_start_min_electrical_speed_rad_s",
        )?,
        flying_start_stable_samples: u32_value(object, "flying_start_stable_samples")?,
        flying_start_timeout_samples: u32_value(object, "flying_start_timeout_samples")?,
    })
}

fn parse_input(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<AdvancedFocInput, SimulationContractError> {
    Ok(AdvancedFocInput {
        base_reference: CurrentCommand {
            id_ref_a: f32_value(object, "base_id_a")?,
            iq_ref_a: f32_value(object, "base_iq_a")?,
        },
        measured_current_dq: Dq {
            d: f32_value(object, "measured_id_a")?,
            q: f32_value(object, "measured_iq_a")?,
        },
        current_alpha_beta: AlphaBeta {
            alpha: f32_value(object, "current_alpha_a")?,
            beta: f32_value(object, "current_beta_a")?,
        },
        previous_voltage_dq: Dq {
            d: f32_value(object, "previous_vd_v")?,
            q: f32_value(object, "previous_vq_v")?,
        },
        estimated_electrical_angle_rad: f32_value(object, "electrical_angle_rad")?,
        estimated_electrical_speed_rad_s: f32_value(object, "electrical_speed_rad_s")?,
        dc_bus_voltage_v: f32_value(object, "dc_bus_voltage_v")?,
        linear_voltage_utilization: f32_value(object, "linear_voltage_utilization")?,
        closed_loop_active: bool_u32(object, "closed_loop_active")?,
        observer_reliable: bool_u32(object, "observer_reliable")?,
        allow_voltage_injection: bool_u32(object, "allow_voltage_injection")?,
        request_flying_start: bool_u32(object, "request_flying_start")?,
    })
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
    fn repository_advanced_matrix_runs() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf();
        let result = run_advanced_contract(&root).unwrap();
        assert_eq!(result.case_count, 9);
    }
}
