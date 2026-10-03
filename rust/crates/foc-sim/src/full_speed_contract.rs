//! Shared full-speed closed-loop dynamic scenarios for the Rust PC engine.

use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use foc_control::{
    st_gbm2804_reference_parameters, AdvancedFocConfig, ADVANCED_FOC_CONFIG_VERSION,
};

use crate::dynamic_plant::{
    DynamicDcBus, DynamicInverterLimit, DynamicPmsmConfig, DynamicPmsmMotor, DynamicPmsmPlant,
    DynamicPmsmState, PmsmTopology,
};
use crate::full_speed::{FullSpeedClosedLoop, FullSpeedClosedLoopConfig};
use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/full_speed_closed_loop_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/full-speed";

#[derive(Debug)]
pub struct FullSpeedContractResult {
    pub case_count: usize,
    pub row_count: usize,
    pub metadata_path: PathBuf,
    pub trace_path: PathBuf,
    pub metrics_path: PathBuf,
}

#[derive(Clone, Copy)]
struct Case {
    duration_s: f32,
    target_initial_rpm: f32,
    target_step_time_s: f32,
    target_final_rpm: f32,
    load_initial_nm: f32,
    load_step_time_s: f32,
    load_final_nm: f32,
    source_initial_v: f32,
    source_step_time_s: f32,
    source_final_v: f32,
    advanced_features: u32,
}

pub fn run_full_speed_contract(
    project_root: &Path,
) -> Result<FullSpeedContractResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let json = parse_json_bytes(&bytes)?;
    let root = object(&json, "full-speed scenario root")?;
    if string(root, "contract")? != "fluxrt-full-speed-closed-loop"
        || unsigned(root, "version")? != 1
    {
        return Err(SimulationContractError::Invalid(
            "unsupported full-speed scenario identity".to_owned(),
        ));
    }
    let decimation = u32_value(root, "trace_decimation")?;
    if decimation == 0 {
        return Err(SimulationContractError::Invalid(
            "trace_decimation must be non-zero".to_owned(),
        ));
    }
    let cases = array(required(root, "cases")?, "full-speed cases")?;
    if cases.is_empty() {
        return Err(SimulationContractError::Invalid(
            "full-speed scenario has no cases".to_owned(),
        ));
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", result_dir.display()))
    })?;
    let trace_path = result_dir.join("rust-full-speed-trace.csv");
    let metrics_path = result_dir.join("rust-full-speed-metrics.csv");
    let metadata_path = result_dir.join("rust-full-speed-d0.txt");
    let mut trace = fs::File::create(&trace_path).map_err(io_error_path(&trace_path))?;
    let mut metrics = fs::File::create(&metrics_path).map_err(io_error_path(&metrics_path))?;
    writeln!(trace, "case_id,tick,time_s,target_rpm,speed_rpm,id_a,iq_a,id_ref_a,iq_ref_a,vd_v,vq_v,dc_bus_v,source_current_a,load_torque_nm,region,active_features,voltage_limited")
        .map_err(io_error)?;
    writeln!(metrics, "case_id,final_target_rpm,final_speed_rpm,final_speed_error_rpm,peak_current_a,min_bus_v,max_bus_v,voltage_limited_fraction")
        .map_err(io_error)?;

    let mut row_count = 0usize;
    for value in cases {
        let value = object(value, "full-speed case")?;
        let case_id = string(value, "id")?;
        let case = parse_case(value)?;
        let control = st_gbm2804_reference_parameters();
        let mut advanced = advanced_config(case.advanced_features);
        advanced.current_limit_a = control.motor.rated_current_a;
        advanced.weakening_id_min_a = -0.75 * control.motor.rated_current_a;
        let mut loop_ = FullSpeedClosedLoop::new(FullSpeedClosedLoopConfig { control, advanced })
            .map_err(|error| {
            SimulationContractError::Invalid(format!("case {case_id} control: {error:?}"))
        })?;
        let mut plant = dynamic_plant(&control, case.source_initial_v).map_err(|error| {
            SimulationContractError::Invalid(format!("case {case_id} plant: {error:?}"))
        })?;
        let tick_count = (case.duration_s * control.pwm_frequency_hz as f32).round() as u32;
        if tick_count == 0 || tick_count > control.pwm_frequency_hz.saturating_mul(20) {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} duration is outside 1 tick..20 s"
            )));
        }
        let mut peak_current_a = 0.0_f32;
        let mut min_bus_v = case.source_initial_v;
        let mut max_bus_v = case.source_initial_v;
        let mut limited_ticks = 0_u32;
        let mut final_speed_rpm = 0.0_f32;
        let mut final_target_rpm = case.target_initial_rpm;
        for tick in 0..tick_count {
            let time_s = tick as f32 / control.pwm_frequency_hz as f32;
            let target_rpm = if time_s >= case.target_step_time_s {
                case.target_final_rpm
            } else {
                case.target_initial_rpm
            };
            let load_nm = if time_s >= case.load_step_time_s {
                case.load_final_nm
            } else {
                case.load_initial_nm
            };
            let source_v = if time_s >= case.source_step_time_s {
                case.source_final_v
            } else {
                case.source_initial_v
            };
            plant.set_load_torque_nm(load_nm).map_err(|error| {
                SimulationContractError::Invalid(format!("case {case_id} load: {error:?}"))
            })?;
            if source_v != plant.source_voltage_v() {
                plant.set_source_voltage_v(source_v).map_err(|error| {
                    SimulationContractError::Invalid(format!("case {case_id} source: {error:?}"))
                })?;
            }
            let output = loop_.step(&mut plant, target_rpm).map_err(|error| {
                SimulationContractError::Invalid(format!("case {case_id} tick {tick}: {error:?}"))
            })?;
            let current = (output.plant.state.current_d_a.powi(2)
                + output.plant.state.current_q_a.powi(2))
            .sqrt();
            peak_current_a = peak_current_a.max(current);
            min_bus_v = min_bus_v.min(output.plant.state.dc_bus_voltage_v);
            max_bus_v = max_bus_v.max(output.plant.state.dc_bus_voltage_v);
            limited_ticks += u32::from(output.control.voltage_limited);
            final_speed_rpm =
                output.plant.state.mechanical_speed_rad_s * 30.0 / std::f32::consts::PI;
            final_target_rpm = target_rpm;
            if tick % decimation == 0 || tick + 1 == tick_count {
                writeln!(trace, "{case_id},{tick},{time_s:.9},{target_rpm:.9},{final_speed_rpm:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{load_nm:.9},{},{},{:}",
                    output.plant.state.current_d_a,
                    output.plant.state.current_q_a,
                    output.advanced.current_reference.id_ref_a,
                    output.advanced.current_reference.iq_ref_a,
                    output.control.voltage_dq.d,
                    output.control.voltage_dq.q,
                    output.plant.state.dc_bus_voltage_v,
                    output.plant.source_current_a,
                    output.advanced.region as u32,
                    output.advanced.active_features,
                    u32::from(output.control.voltage_limited),
                ).map_err(io_error)?;
                row_count += 1;
            }
        }
        writeln!(metrics, "{case_id},{final_target_rpm:.9},{final_speed_rpm:.9},{:.9},{peak_current_a:.9},{min_bus_v:.9},{max_bus_v:.9},{:.9}",
            final_target_rpm - final_speed_rpm,
            limited_ticks as f32 / tick_count as f32,
        ).map_err(io_error)?;
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-full-speed-closed-loop\nversion=1\nengine=rust-dynamic-rk4\nworkspace_revision={revision}\ncase_count={}\nrow_count={row_count}\nresult=PASS\nmodel_scope=design-not-hardware-truth\n",
            cases.len()
        ),
    )
    .map_err(|error| {
        SimulationContractError::Io(format!("cannot write {}: {error}", metadata_path.display()))
    })?;
    Ok(FullSpeedContractResult {
        case_count: cases.len(),
        row_count,
        metadata_path,
        trace_path,
        metrics_path,
    })
}

fn dynamic_plant(
    control: &foc_control::ControlParameters,
    source_voltage_v: f32,
) -> Result<DynamicPmsmPlant, crate::dynamic_plant::DynamicPmsmError> {
    DynamicPmsmPlant::new(
        DynamicPmsmConfig {
            step_s: 1.0 / control.pwm_frequency_hz as f32,
            motor: DynamicPmsmMotor {
                topology: PmsmTopology::Surface,
                pole_pairs: control.motor.pole_pairs as u32,
                stator_resistance_ohm: control.motor.stator_resistance_ohm,
                inductance_d_h: control.motor.ld_h,
                inductance_q_h: control.motor.lq_h,
                permanent_magnet_flux_wb: control.motor.flux_linkage_wb,
                inertia_kg_m2: control.motor.inertia_kg_m2,
                viscous_friction_nm_s_rad: control.motor.viscous_friction_nm_s,
            },
            inverter: DynamicInverterLimit::default(),
            dc_bus: DynamicDcBus {
                source_voltage_v,
                source_resistance_ohm: 0.2,
                capacitance_f: 0.002,
                minimum_voltage_v: 6.0,
                maximum_voltage_v: 20.0,
                source_can_sink: false,
            },
        },
        DynamicPmsmState::at_rest(source_voltage_v),
    )
}

fn advanced_config(features: u32) -> AdvancedFocConfig {
    AdvancedFocConfig {
        struct_size: core::mem::size_of::<AdvancedFocConfig>() as u32,
        version: ADVANCED_FOC_CONFIG_VERSION,
        enabled_features: features,
        region_update_divider: 12,
        current_limit_a: 0.8,
        weakening_entry_utilization: 0.92,
        weakening_exit_utilization: 0.82,
        weakening_kp_a_per_v: 0.08,
        weakening_id_min_a: -0.6,
        weakening_slew_a_per_s: 20.0,
        decoupling_gain: 1.0,
        ..AdvancedFocConfig::default()
    }
}

fn parse_case(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<Case, SimulationContractError> {
    Ok(Case {
        duration_s: f32_value(object, "duration_s")?,
        target_initial_rpm: f32_value(object, "target_initial_rpm")?,
        target_step_time_s: f32_value(object, "target_step_time_s")?,
        target_final_rpm: f32_value(object, "target_final_rpm")?,
        load_initial_nm: f32_value(object, "load_initial_nm")?,
        load_step_time_s: f32_value(object, "load_step_time_s")?,
        load_final_nm: f32_value(object, "load_final_nm")?,
        source_initial_v: f32_value(object, "source_initial_v")?,
        source_step_time_s: f32_value(object, "source_step_time_s")?,
        source_final_v: f32_value(object, "source_final_v")?,
        advanced_features: u32_value(object, "advanced_features")?,
    })
}

fn f32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<f32, SimulationContractError> {
    let value = finite_number(object, key)?;
    let converted = value as f32;
    if converted.is_finite() {
        Ok(converted)
    } else {
        Err(SimulationContractError::Invalid(format!(
            "{key} cannot be represented as f32"
        )))
    }
}

fn u32_value(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
    key: &str,
) -> Result<u32, SimulationContractError> {
    let value = unsigned(object, key)?;
    u32::try_from(value)
        .map_err(|_| SimulationContractError::Invalid(format!("{key} exceeds u32 range")))
}

fn io_error(error: std::io::Error) -> SimulationContractError {
    SimulationContractError::Io(error.to_string())
}

fn io_error_path(path: &Path) -> impl FnOnce(std::io::Error) -> SimulationContractError + '_ {
    move |error| SimulationContractError::Io(format!("cannot create {}: {error}", path.display()))
}
