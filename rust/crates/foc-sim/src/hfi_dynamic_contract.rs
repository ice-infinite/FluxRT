//! Dynamic rotating-HFI design contract.
//!
//! The plant linearises high-frequency current around a commanded dq operating
//! point while the rotor moves at a prescribed electrical speed.  The measured
//! current then passes through deterministic noise, a bounded sample delay and
//! a fundamental-current low-pass separator before the production negative-
//! sequence demodulator.  This remains an offline S3 model.

use std::f32::consts::PI;
use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use foc_algorithm::{
    average_inverter_phase_voltage_loss_v, clarke, inverse_clarke, inverse_park, park, Abc,
    AlphaBeta, Dq, InverterLossParameters, RotatingHfSequenceParam, RotatingHfSequenceState,
};

use crate::simulation_contract::{
    array, finite_number, object, parse_json_bytes, required, string, unsigned,
    SimulationContractError,
};

const SCENARIO_RELATIVE_PATH: &str = "simulation/scenarios/hfi_dynamic_plant_v1.json";
const RESULT_RELATIVE_DIR: &str = "simulation/results/hfi-dynamic";
const MAX_DELAY_SAMPLES: usize = 8;

#[derive(Debug)]
pub struct HfiDynamicContractResult {
    pub case_count: usize,
    pub metadata_path: PathBuf,
    pub metrics_path: PathBuf,
}

#[derive(Clone, Copy, Debug, Default)]
struct DynamicHfiPlant {
    hf_current_dq: Dq,
}

#[derive(Clone, Copy, Debug)]
struct MotorElectricalParam {
    resistance_ohm: f32,
    ld_h: f32,
    lq_h: f32,
}

impl DynamicHfiPlant {
    fn step(
        &mut self,
        voltage_alpha_beta: AlphaBeta,
        theta_rad: f32,
        electrical_speed_rad_s: f32,
        motor: MotorElectricalParam,
        dt_s: f32,
    ) {
        let voltage_dq = park(voltage_alpha_beta, theta_rad);
        let did = (voltage_dq.d - motor.resistance_ohm * self.hf_current_dq.d
            + electrical_speed_rad_s * motor.lq_h * self.hf_current_dq.q)
            / motor.ld_h;
        let diq = (voltage_dq.q
            - motor.resistance_ohm * self.hf_current_dq.q
            - electrical_speed_rad_s * motor.ld_h * self.hf_current_dq.d)
            / motor.lq_h;
        self.hf_current_dq.d += did * dt_s;
        self.hf_current_dq.q += diq * dt_s;
    }

    fn measured_current(
        &self,
        base_current_dq: Dq,
        theta_rad: f32,
        noise_a: f32,
        sample: u32,
    ) -> AlphaBeta {
        let mut current = inverse_park(
            Dq {
                d: base_current_dq.d + self.hf_current_dq.d,
                q: base_current_dq.q + self.hf_current_dq.q,
            },
            theta_rad,
        );
        // Two incommensurate deterministic tones keep the contract reproducible
        // while exercising both alpha and beta measurement paths.
        current.alpha += noise_a * (sample as f32 * 0.731).sin();
        current.beta += noise_a * (sample as f32 * 0.527 + 0.4).sin();
        current
    }
}

#[derive(Clone, Copy, Debug)]
struct DelayLine {
    samples: [AlphaBeta; MAX_DELAY_SAMPLES],
    write_index: usize,
}

impl Default for DelayLine {
    fn default() -> Self {
        Self {
            samples: [AlphaBeta::default(); MAX_DELAY_SAMPLES],
            write_index: 0,
        }
    }
}

impl DelayLine {
    fn push(&mut self, value: AlphaBeta, delay_samples: usize) -> AlphaBeta {
        self.samples[self.write_index] = value;
        let read_index = (self.write_index + MAX_DELAY_SAMPLES - delay_samples) % MAX_DELAY_SAMPLES;
        let delayed = self.samples[read_index];
        self.write_index = (self.write_index + 1) % MAX_DELAY_SAMPLES;
        delayed
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct CurrentSeparator {
    low_pass: AlphaBeta,
}

impl CurrentSeparator {
    fn update(&mut self, input: AlphaBeta, alpha: f32) -> AlphaBeta {
        self.low_pass.alpha += alpha * (input.alpha - self.low_pass.alpha);
        self.low_pass.beta += alpha * (input.beta - self.low_pass.beta);
        AlphaBeta {
            alpha: input.alpha - self.low_pass.alpha,
            beta: input.beta - self.low_pass.beta,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct DynamicCase {
    resistance_ohm: f32,
    ld_h: f32,
    lq_h: f32,
    initial_theta_rad: f32,
    electrical_speed_rad_s: f32,
    base_current_dq: Dq,
    post_step_current_dq: Dq,
    base_current_step_sample: u32,
    noise_a: f32,
    delay_samples: usize,
    dead_time_s: f32,
    expected_valid: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct DynamicMetrics {
    rms_angle_error_rad: f32,
    maximum_angle_error_rad: f32,
    mean_response_a: f32,
    peak_current_a: f32,
    valid: bool,
}

pub fn run_hfi_dynamic_contract(
    project_root: &Path,
) -> Result<HfiDynamicContractResult, SimulationContractError> {
    let scenario_path = project_root.join(SCENARIO_RELATIVE_PATH);
    let bytes = fs::read(&scenario_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot read {}: {error}", scenario_path.display()))
    })?;
    let json = parse_json_bytes(&bytes)?;
    let root = object(&json, "dynamic HFI scenario root")?;
    if string(root, "contract")? != "fluxrt-hfi-dynamic-plant" || unsigned(root, "version")? != 1 {
        return Err(SimulationContractError::Invalid(
            "unsupported dynamic HFI scenario identity".to_owned(),
        ));
    }
    let control_frequency_hz = u32_value(root, "control_frequency_hz")?;
    let samples = u32_value(root, "samples")?;
    let settling_samples = u32_value(root, "settling_samples")?;
    let injection = object(required(root, "injection")?, "injection")?;
    let amplitude_v = f32_value(injection, "amplitude_v")?;
    let injection_frequency_hz = f32_value(injection, "frequency_hz")?;
    let demod_alpha = f32_value(injection, "demod_alpha")?;
    let separator_alpha = f32_value(injection, "separator_alpha")?;
    let minimum_response_a = f32_value(injection, "minimum_response_a")?;
    let minimum_relative_saliency = f32_value(injection, "minimum_relative_saliency")?;
    let maximum_rms_error_rad = f32_value(injection, "maximum_rms_error_rad")?;
    let maximum_peak_error_rad = f32_value(injection, "maximum_peak_error_rad")?;
    let dc_bus_voltage_v = f32_value(root, "dc_bus_voltage_v")?;
    let pwm_frequency_hz = f32_value(root, "pwm_frequency_hz")?;
    if control_frequency_hz == 0
        || samples <= settling_samples
        || samples > control_frequency_hz.saturating_mul(10)
        || settling_samples == 0
        || !finite_positive(amplitude_v)
        || !finite_positive(injection_frequency_hz)
        || injection_frequency_hz >= control_frequency_hz as f32 * 0.5
        || !(0.0..=1.0).contains(&demod_alpha)
        || demod_alpha == 0.0
        || !(0.0..=1.0).contains(&separator_alpha)
        || separator_alpha == 0.0
        || !finite_nonnegative(minimum_response_a)
        || !finite_nonnegative(minimum_relative_saliency)
        || !finite_positive(maximum_rms_error_rad)
        || !finite_positive(maximum_peak_error_rad)
        || !finite_positive(dc_bus_voltage_v)
        || !finite_positive(pwm_frequency_hz)
    {
        return Err(SimulationContractError::Invalid(
            "invalid dynamic HFI common configuration".to_owned(),
        ));
    }
    let cases = array(required(root, "cases")?, "cases")?;
    if cases.is_empty() {
        return Err(SimulationContractError::Invalid(
            "dynamic HFI scenario has no cases".to_owned(),
        ));
    }

    let result_dir = project_root.join(RESULT_RELATIVE_DIR);
    fs::create_dir_all(&result_dir).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", result_dir.display()))
    })?;
    let metrics_path = result_dir.join("rust-hfi-dynamic-metrics.csv");
    let metadata_path = result_dir.join("rust-hfi-dynamic-d0.txt");
    let mut metrics_file = fs::File::create(&metrics_path).map_err(|error| {
        SimulationContractError::Io(format!("cannot create {}: {error}", metrics_path.display()))
    })?;
    writeln!(
        metrics_file,
        "case_id,rms_angle_error_rad,maximum_angle_error_rad,mean_response_a,peak_current_a,valid"
    )
    .map_err(io_error)?;

    let dt_s = 1.0 / control_frequency_hz as f32;
    for value in cases {
        let object = object(value, "dynamic HFI case")?;
        let case_id = string(object, "id")?;
        let case = parse_case(object)?;
        if case.delay_samples >= MAX_DELAY_SAMPLES
            || case.resistance_ohm < 0.0
            || !finite_positive(case.ld_h)
            || !finite_positive(case.lq_h)
            || !case.initial_theta_rad.is_finite()
            || !case.electrical_speed_rad_s.is_finite()
            || !case.base_current_dq.d.is_finite()
            || !case.base_current_dq.q.is_finite()
            || !case.post_step_current_dq.d.is_finite()
            || !case.post_step_current_dq.q.is_finite()
            || case.base_current_step_sample > samples
            || !finite_nonnegative(case.noise_a)
            || !finite_nonnegative(case.dead_time_s)
        {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} has invalid fields"
            )));
        }
        let phase_offset_rad = calibrate_phase(
            case,
            amplitude_v,
            injection_frequency_hz,
            demod_alpha,
            separator_alpha,
            dt_s,
            settling_samples,
            dc_bus_voltage_v,
            pwm_frequency_hz,
        );
        let metrics = run_case(
            case,
            amplitude_v,
            injection_frequency_hz,
            demod_alpha,
            separator_alpha,
            dt_s,
            samples,
            settling_samples,
            phase_offset_rad,
            dc_bus_voltage_v,
            pwm_frequency_hz,
            minimum_response_a,
            minimum_relative_saliency,
            maximum_rms_error_rad,
            maximum_peak_error_rad,
        );
        if metrics.valid != case.expected_valid {
            return Err(SimulationContractError::Invalid(format!(
                "case {case_id} validity mismatch: expected {}, got {}; rms={:.6}, peak={:.6}, response={:.6}",
                case.expected_valid,
                metrics.valid,
                metrics.rms_angle_error_rad,
                metrics.maximum_angle_error_rad,
                metrics.mean_response_a,
            )));
        }
        writeln!(
            metrics_file,
            "{case_id},{:.9},{:.9},{:.9},{:.9},{}",
            metrics.rms_angle_error_rad,
            metrics.maximum_angle_error_rad,
            metrics.mean_response_a,
            metrics.peak_current_a,
            u32::from(metrics.valid),
        )
        .map_err(io_error)?;
    }

    let revision =
        std::env::var("FLUXRT_WORKSPACE_REVISION").unwrap_or_else(|_| "UNRECORDED".to_owned());
    fs::write(
        &metadata_path,
        format!(
            "contract=fluxrt-hfi-dynamic-plant\nversion=1\nengine=rust\nworkspace_revision={revision}\ncase_count={}\nresult=PASS\n",
            cases.len()
        ),
    )
    .map_err(|error| {
        SimulationContractError::Io(format!("cannot write {}: {error}", metadata_path.display()))
    })?;

    Ok(HfiDynamicContractResult {
        case_count: cases.len(),
        metadata_path,
        metrics_path,
    })
}

#[allow(clippy::too_many_arguments)]
fn calibrate_phase(
    mut case: DynamicCase,
    amplitude_v: f32,
    frequency_hz: f32,
    demod_alpha: f32,
    separator_alpha: f32,
    dt_s: f32,
    samples: u32,
    dc_bus_voltage_v: f32,
    pwm_frequency_hz: f32,
) -> f32 {
    // Development calibration uses a stationary known-angle fixture. Real
    // target approval therefore requires an encoder/fixture and must not infer
    // this phase from the same observer it is meant to validate.
    case.initial_theta_rad = 0.0;
    case.electrical_speed_rad_s = 0.0;
    case.base_current_dq = Dq::default();
    case.post_step_current_dq = Dq::default();
    case.noise_a = 0.0;
    let mut observer = RotatingHfSequenceState::default();
    let mut plant = DynamicHfiPlant::default();
    let mut separator = CurrentSeparator::default();
    let mut delay = DelayLine::default();
    let param = RotatingHfSequenceParam {
        amplitude: amplitude_v,
        freq_hz: frequency_hz,
        ts: dt_s,
        demod_alpha,
        phase_offset_rad: 0.0,
    };
    for sample in 0..samples {
        let current = plant.measured_current(Dq::default(), 0.0, 0.0, sample);
        let applied = apply_inverter_loss(
            observer.voltage,
            current,
            case.dead_time_s,
            dc_bus_voltage_v,
            pwm_frequency_hz,
        );
        plant.step(
            applied,
            0.0,
            0.0,
            MotorElectricalParam {
                resistance_ohm: case.resistance_ohm,
                ld_h: case.ld_h,
                lq_h: case.lq_h,
            },
            dt_s,
        );
        let measured = plant.measured_current(Dq::default(), 0.0, 0.0, sample);
        let delayed = delay.push(measured, case.delay_samples);
        let high_frequency = separator.update(delayed, separator_alpha);
        observer.update(&param, high_frequency);
    }
    observer
        .negative_sequence
        .beta
        .atan2(observer.negative_sequence.alpha)
}

#[allow(clippy::too_many_arguments)]
fn run_case(
    case: DynamicCase,
    amplitude_v: f32,
    frequency_hz: f32,
    demod_alpha: f32,
    separator_alpha: f32,
    dt_s: f32,
    samples: u32,
    settling_samples: u32,
    phase_offset_rad: f32,
    dc_bus_voltage_v: f32,
    pwm_frequency_hz: f32,
    minimum_response_a: f32,
    minimum_relative_saliency: f32,
    maximum_rms_error_rad: f32,
    maximum_peak_error_rad: f32,
) -> DynamicMetrics {
    let demod_tracking_phase_rad =
        first_order_low_pass_phase(demod_alpha, 2.0 * case.electrical_speed_rad_s * dt_s);
    let param = RotatingHfSequenceParam {
        amplitude: amplitude_v,
        freq_hz: frequency_hz,
        ts: dt_s,
        demod_alpha,
        phase_offset_rad: phase_offset_rad + demod_tracking_phase_rad,
    };
    let mut observer = RotatingHfSequenceState::default();
    let mut plant = DynamicHfiPlant::default();
    let mut separator = CurrentSeparator::default();
    let mut delay = DelayLine::default();
    let mut theta = case.initial_theta_rad.rem_euclid(2.0 * PI);
    let mut squared_error_sum = 0.0_f32;
    let mut maximum_angle_error_rad = 0.0_f32;
    let mut response_sum = 0.0_f32;
    let mut peak_current_a = 0.0_f32;
    let mut metric_samples = 0_u32;
    for sample in 0..samples {
        let base_current_dq = case.base_current_at(sample);
        let current_before = plant.measured_current(base_current_dq, theta, 0.0, sample);
        let applied = apply_inverter_loss(
            observer.voltage,
            current_before,
            case.dead_time_s,
            dc_bus_voltage_v,
            pwm_frequency_hz,
        );
        plant.step(
            applied,
            theta,
            case.electrical_speed_rad_s,
            MotorElectricalParam {
                resistance_ohm: case.resistance_ohm,
                ld_h: case.ld_h,
                lq_h: case.lq_h,
            },
            dt_s,
        );
        theta = (theta + case.electrical_speed_rad_s * dt_s).rem_euclid(2.0 * PI);
        let measured = plant.measured_current(base_current_dq, theta, case.noise_a, sample);
        peak_current_a = peak_current_a
            .max((measured.alpha * measured.alpha + measured.beta * measured.beta).sqrt());
        let delayed = delay.push(measured, case.delay_samples);
        let high_frequency = separator.update(delayed, separator_alpha);
        observer.update(&param, high_frequency);
        if sample >= settling_samples {
            let error = mod_pi_error(observer.theta_est_mod_pi_rad, theta.rem_euclid(PI));
            squared_error_sum += error * error;
            maximum_angle_error_rad = maximum_angle_error_rad.max(error);
            response_sum += observer.response_magnitude_a;
            metric_samples += 1;
        }
    }
    let rms_angle_error_rad = (squared_error_sum / metric_samples as f32).sqrt();
    let mean_response_a = response_sum / metric_samples as f32;
    let relative_saliency = (case.lq_h - case.ld_h).abs() / case.ld_h.max(case.lq_h);
    let valid = relative_saliency >= minimum_relative_saliency
        && mean_response_a >= minimum_response_a
        && rms_angle_error_rad <= maximum_rms_error_rad
        && maximum_angle_error_rad <= maximum_peak_error_rad;
    DynamicMetrics {
        rms_angle_error_rad,
        maximum_angle_error_rad,
        mean_response_a,
        peak_current_a,
        valid,
    }
}

fn apply_inverter_loss(
    commanded: AlphaBeta,
    current: AlphaBeta,
    dead_time_s: f32,
    dc_bus_voltage_v: f32,
    pwm_frequency_hz: f32,
) -> AlphaBeta {
    if dead_time_s <= 0.0 {
        return commanded;
    }
    let current_abc = inverse_clarke(current);
    let loss = average_inverter_phase_voltage_loss_v(
        InverterLossParameters {
            dead_time_s,
            pwm_period_s: 1.0 / pwm_frequency_hz,
            device_drop_v: 0.0,
            current_zero_band_a: 0.002,
        },
        dc_bus_voltage_v,
        Abc {
            a: current_abc.a,
            b: current_abc.b,
            c: current_abc.c,
        },
    );
    let loss = clarke(loss);
    AlphaBeta {
        alpha: commanded.alpha - loss.alpha,
        beta: commanded.beta - loss.beta,
    }
}

fn parse_case(
    object: &std::collections::BTreeMap<String, crate::simulation_contract::JsonValue>,
) -> Result<DynamicCase, SimulationContractError> {
    let delay_samples = usize::try_from(unsigned(object, "delay_samples")?).map_err(|_| {
        SimulationContractError::Invalid("delay_samples does not fit usize".to_owned())
    })?;
    Ok(DynamicCase {
        resistance_ohm: f32_value(object, "resistance_ohm")?,
        ld_h: f32_value(object, "ld_h")?,
        lq_h: f32_value(object, "lq_h")?,
        initial_theta_rad: f32_value(object, "initial_theta_rad")?,
        electrical_speed_rad_s: f32_value(object, "electrical_speed_rad_s")?,
        base_current_dq: Dq {
            d: f32_value(object, "base_id_a")?,
            q: f32_value(object, "base_iq_a")?,
        },
        post_step_current_dq: Dq {
            d: f32_value(object, "post_step_id_a")?,
            q: f32_value(object, "post_step_iq_a")?,
        },
        base_current_step_sample: u32_value(object, "base_current_step_sample")?,
        noise_a: f32_value(object, "noise_a")?,
        delay_samples,
        dead_time_s: f32_value(object, "dead_time_s")?,
        expected_valid: bool_u32(object, "expected_valid")?,
    })
}

impl DynamicCase {
    fn base_current_at(self, sample: u32) -> Dq {
        if sample >= self.base_current_step_sample {
            self.post_step_current_dq
        } else {
            self.base_current_dq
        }
    }
}

fn mod_pi_error(estimate: f32, truth: f32) -> f32 {
    ((estimate - truth + 0.5 * PI).rem_euclid(PI) - 0.5 * PI).abs()
}

fn first_order_low_pass_phase(alpha: f32, input_step_rad: f32) -> f32 {
    let one_minus_alpha = 1.0 - alpha;
    let denominator_real = 1.0 - one_minus_alpha * input_step_rad.cos();
    let denominator_imag = one_minus_alpha * input_step_rad.sin();
    (-denominator_imag).atan2(denominator_real)
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

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn io_error(error: std::io::Error) -> SimulationContractError {
    SimulationContractError::Io(error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repository_dynamic_hfi_matrix_runs() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf();
        let result = run_hfi_dynamic_contract(&root).unwrap();
        assert_eq!(result.case_count, 16);
    }
}
