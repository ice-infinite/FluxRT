use std::f32::consts::PI;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use foc_algorithm::{clarke, inverse_clarke, inverse_park, park, Abc, AlphaBeta, Dq};
use foc_control::{
    pwm_to_alpha_beta, pwm_to_phase_voltage, FeedbackPort, FeedbackSnapshot, HardwareFault,
    MotorParameters, PhaseCurrents, PwmCommand, PwmPort, RotorFeedback, SafetyPort,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterSimulationConfig {
    pub dead_time_enabled: bool,
    pub dead_time_s: f32,
}

impl Default for InverterSimulationConfig {
    fn default() -> Self {
        Self {
            dead_time_enabled: false,
            dead_time_s: 550.0e-9,
        }
    }
}

fn hard_current_polarity(current_a: f32) -> f32 {
    if current_a > 0.0 {
        1.0
    } else if current_a < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Averaged inverter terminal voltage including polarity-dependent dead-time
/// loss. This matches the MATLAB plant convention used for correlation.
pub fn pwm_to_alpha_beta_with_inverter(
    pwm: PwmCommand,
    currents: PhaseCurrents,
    dc_bus_voltage: f32,
    controller_period_s: f32,
    inverter: InverterSimulationConfig,
) -> AlphaBeta {
    if !inverter.dead_time_enabled {
        return pwm_to_alpha_beta(pwm, dc_bus_voltage);
    }
    let phase = pwm_to_phase_voltage(pwm, dc_bus_voltage);
    let loss_v = 2.0 * inverter.dead_time_s / controller_period_s * dc_bus_voltage;
    clarke(Abc {
        a: phase.a - loss_v * hard_current_polarity(currents.a),
        b: phase.b - loss_v * hard_current_polarity(currents.b),
        c: phase.c - loss_v * hard_current_polarity(currents.c),
    })
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlantState {
    pub id_a: f32,
    pub iq_a: f32,
    pub mechanical_speed_rad_s: f32,
    pub mechanical_angle_rad: f32,
    pub electromagnetic_torque_nm: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct PmsmPlant {
    parameters: MotorParameters,
    state: PlantState,
    load_torque_nm: f32,
}

impl PmsmPlant {
    pub fn new(parameters: MotorParameters) -> Self {
        Self {
            parameters,
            state: PlantState::default(),
            load_torque_nm: 0.0,
        }
    }

    pub fn state(&self) -> PlantState {
        self.state
    }

    pub fn electrical_angle_rad(&self) -> f32 {
        (self.state.mechanical_angle_rad * self.parameters.pole_pairs as f32).rem_euclid(2.0 * PI)
    }

    pub fn phase_currents(&self) -> PhaseCurrents {
        let alpha_beta = inverse_park(
            Dq {
                d: self.state.id_a,
                q: self.state.iq_a,
            },
            self.electrical_angle_rad(),
        );
        let abc = inverse_clarke(alpha_beta);
        PhaseCurrents {
            a: abc.a,
            b: abc.b,
            c: abc.c,
        }
    }

    pub fn set_load_torque_nm(&mut self, load_torque_nm: f32) {
        self.load_torque_nm = load_torque_nm;
    }

    pub fn load_torque_nm(&self) -> f32 {
        self.load_torque_nm
    }

    pub fn step(&mut self, voltage_alpha_beta: foc_algorithm::AlphaBeta, dt_s: f32) {
        let electrical_angle = self.electrical_angle_rad();
        let voltage_dq = park(voltage_alpha_beta, electrical_angle);
        let electrical_speed =
            self.state.mechanical_speed_rad_s * self.parameters.pole_pairs as f32;

        let did_dt = (voltage_dq.d - self.parameters.stator_resistance_ohm * self.state.id_a
            + electrical_speed * self.parameters.lq_h * self.state.iq_a)
            / self.parameters.ld_h;
        let diq_dt = (voltage_dq.q
            - self.parameters.stator_resistance_ohm * self.state.iq_a
            - electrical_speed
                * (self.parameters.ld_h * self.state.id_a + self.parameters.flux_linkage_wb))
            / self.parameters.lq_h;

        self.state.id_a += did_dt * dt_s;
        self.state.iq_a += diq_dt * dt_s;
        self.state.electromagnetic_torque_nm = 1.5
            * self.parameters.pole_pairs as f32
            * (self.parameters.flux_linkage_wb * self.state.iq_a
                + (self.parameters.ld_h - self.parameters.lq_h)
                    * self.state.id_a
                    * self.state.iq_a);
        let acceleration = (self.state.electromagnetic_torque_nm
            - self.load_torque_nm
            - self.parameters.viscous_friction_nm_s * self.state.mechanical_speed_rad_s)
            / self.parameters.inertia_kg_m2;
        self.state.mechanical_speed_rad_s += acceleration * dt_s;
        self.state.mechanical_angle_rad = (self.state.mechanical_angle_rad
            + self.state.mechanical_speed_rad_s * dt_s)
            .rem_euclid(2.0 * PI);
    }
}

/// PC fake for every hardware-facing control port. The ideal rotor feedback is
/// intentional: it isolates controller/plant behavior from observer tuning.
/// `BemfPllEstimator` can be inserted at this port in observer-specific tests.
pub struct SimHardware {
    plant: PmsmPlant,
    dc_bus_voltage: f32,
    applied_pwm: PwmCommand,
    pwm_enabled: bool,
    faulted: bool,
    feedback_available: bool,
    reject_output: bool,
    peak_phase_current_a: f32,
}

impl SimHardware {
    pub fn new(parameters: MotorParameters) -> Self {
        Self {
            plant: PmsmPlant::new(parameters),
            dc_bus_voltage: parameters.nominal_bus_voltage_v,
            applied_pwm: PwmCommand::default(),
            pwm_enabled: false,
            faulted: false,
            feedback_available: true,
            reject_output: false,
            peak_phase_current_a: 0.0,
        }
    }

    pub fn advance(&mut self, dt_s: f32) {
        let voltage = if self.pwm_enabled {
            pwm_to_alpha_beta(self.applied_pwm, self.dc_bus_voltage)
        } else {
            foc_algorithm::AlphaBeta::default()
        };
        self.plant.step(voltage, dt_s);
        let currents = self.plant.phase_currents();
        self.peak_phase_current_a = self
            .peak_phase_current_a
            .max(currents.a.abs())
            .max(currents.b.abs())
            .max(currents.c.abs());
    }

    pub fn plant(&self) -> &PmsmPlant {
        &self.plant
    }

    pub fn plant_mut(&mut self) -> &mut PmsmPlant {
        &mut self.plant
    }

    pub fn speed_rpm(&self) -> f32 {
        self.plant.state().mechanical_speed_rad_s * 30.0 / PI
    }

    pub fn peak_phase_current_a(&self) -> f32 {
        self.peak_phase_current_a
    }

    pub fn inject_fault(&mut self) {
        self.faulted = true;
    }

    pub fn set_feedback_available(&mut self, available: bool) {
        self.feedback_available = available;
    }

    pub fn set_reject_output(&mut self, reject: bool) {
        self.reject_output = reject;
    }

    pub fn pwm_enabled(&self) -> bool {
        self.pwm_enabled
    }

    pub fn applied_pwm(&self) -> PwmCommand {
        self.applied_pwm
    }

    pub fn dc_bus_voltage(&self) -> f32 {
        self.dc_bus_voltage
    }
}

impl FeedbackPort for SimHardware {
    fn read_feedback(&mut self) -> Result<FeedbackSnapshot, HardwareFault> {
        if self.faulted {
            return Err(HardwareFault::PowerStageFault);
        }
        if !self.feedback_available {
            return Err(HardwareFault::FeedbackUnavailable);
        }
        Ok(FeedbackSnapshot {
            currents: self.plant.phase_currents(),
            dc_bus_voltage: self.dc_bus_voltage,
            rotor: RotorFeedback {
                electrical_angle_rad: self.plant.electrical_angle_rad(),
                mechanical_speed_rad_s: self.plant.state().mechanical_speed_rad_s,
            },
        })
    }
}

impl PwmPort for SimHardware {
    fn apply_pwm(&mut self, command: PwmCommand) -> Result<(), HardwareFault> {
        if self.faulted || self.reject_output || !command.is_valid() {
            self.disable_pwm();
            return Err(HardwareFault::OutputRejected);
        }
        self.applied_pwm = command;
        self.pwm_enabled = true;
        Ok(())
    }

    fn disable_pwm(&mut self) {
        self.applied_pwm = PwmCommand::default();
        self.pwm_enabled = false;
    }
}

impl SafetyPort for SimHardware {
    fn power_stage_faulted(&self) -> bool {
        self.faulted
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SimulationConfig {
    pub duration_s: f32,
    pub target_speed_rpm: f32,
    pub load_step_time_s: f32,
    pub load_torque_nm: f32,
    pub trace_decimation: u32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            duration_s: 3.0,
            target_speed_rpm: 524.0,
            load_step_time_s: 1.0,
            load_torque_nm: 0.004,
            trace_decimation: 10,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SimulationSample {
    pub time_s: f32,
    pub target_speed_rpm: f32,
    pub measured_speed_rpm: f32,
    pub phase_current_a: f32,
    pub phase_current_b: f32,
    pub phase_current_c: f32,
    pub id_ref_a: f32,
    pub iq_ref_a: f32,
    pub id_a: f32,
    pub iq_a: f32,
    pub vd_v: f32,
    pub vq_v: f32,
    pub valpha_v: f32,
    pub vbeta_v: f32,
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
    pub dc_bus_voltage_v: f32,
    pub load_torque_nm: f32,
    pub electromagnetic_torque_nm: f32,
    pub electrical_angle_rad: f32,
    pub voltage_limited: bool,
    pub pwm_enabled: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SimulationSummary {
    pub final_speed_rpm: f32,
    pub target_speed_rpm: f32,
    pub peak_phase_current_a: f32,
    pub load_torque_nm: f32,
    pub sample_count: usize,
}

#[derive(Debug)]
pub enum SimulationError {
    InvalidConfig(&'static str),
    Hardware(HardwareFault),
}

impl From<HardwareFault> for SimulationError {
    fn from(value: HardwareFault) -> Self {
        Self::Hardware(value)
    }
}

#[derive(Debug)]
pub struct SimulationRun {
    pub samples: Vec<SimulationSample>,
    pub summary: SimulationSummary,
}

impl SimulationRun {
    pub fn write_csv(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "time_s,target_speed_rpm,measured_speed_rpm,phase_current_a,phase_current_b,phase_current_c,id_ref_a,iq_ref_a,id_a,iq_a,vd_v,vq_v,valpha_v,vbeta_v,duty_a,duty_b,duty_c,dc_bus_voltage_v,load_torque_nm,electromagnetic_torque_nm,electrical_angle_rad,voltage_limited,pwm_enabled"
        )?;
        for sample in &self.samples {
            writeln!(
                writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                sample.time_s,
                sample.target_speed_rpm,
                sample.measured_speed_rpm,
                sample.phase_current_a,
                sample.phase_current_b,
                sample.phase_current_c,
                sample.id_ref_a,
                sample.iq_ref_a,
                sample.id_a,
                sample.iq_a,
                sample.vd_v,
                sample.vq_v,
                sample.valpha_v,
                sample.vbeta_v,
                sample.duty_a,
                sample.duty_b,
                sample.duty_c,
                sample.dc_bus_voltage_v,
                sample.load_torque_nm,
                sample.electromagnetic_torque_nm,
                sample.electrical_angle_rad,
                u8::from(sample.voltage_limited),
                u8::from(sample.pwm_enabled),
            )?;
        }
        writer.flush()
    }
}

/// Runs the same Rust controller and PMSM plant used by unit tests while
/// collecting a decimated trace suitable for MATLAB, Python, or spreadsheets.
pub fn run_reference_simulation(
    config: SimulationConfig,
) -> Result<SimulationRun, SimulationError> {
    if !config.duration_s.is_finite() || config.duration_s <= 0.0 {
        return Err(SimulationError::InvalidConfig(
            "duration_s must be positive",
        ));
    }
    if !config.target_speed_rpm.is_finite() {
        return Err(SimulationError::InvalidConfig(
            "target_speed_rpm must be finite",
        ));
    }
    if !config.load_step_time_s.is_finite() || config.load_step_time_s < 0.0 {
        return Err(SimulationError::InvalidConfig(
            "load_step_time_s must be non-negative",
        ));
    }
    if !config.load_torque_nm.is_finite() {
        return Err(SimulationError::InvalidConfig(
            "load_torque_nm must be finite",
        ));
    }
    if config.trace_decimation == 0 {
        return Err(SimulationError::InvalidConfig(
            "trace_decimation must be non-zero",
        ));
    }

    let params = foc_control::st_gbm2804_reference_parameters();
    let dt = 1.0 / params.pwm_frequency_hz as f32;
    let total_steps = (config.duration_s / dt).round() as u32;
    let load_step = (config.load_step_time_s / dt).round() as u32;
    let capacity = (total_steps / config.trace_decimation + 2) as usize;
    let mut samples = Vec::with_capacity(capacity);
    let mut runtime = foc_control::ControlRuntime::new(
        foc_control::StReferenceController::new(params),
        SimHardware::new(params.motor),
    );
    runtime.enable()?;

    for tick in 0..total_steps {
        if tick == load_step {
            runtime
                .hardware_mut()
                .plant_mut()
                .set_load_torque_nm(config.load_torque_nm);
        }
        let telemetry = runtime.tick(foc_control::SpeedCommand {
            target_rpm: config.target_speed_rpm,
            id_ref_a: 0.0,
        })?;

        if (tick % config.trace_decimation == 0) || (tick + 1 == total_steps) {
            let hardware = runtime.hardware();
            let currents = hardware.plant().phase_currents();
            let plant_state = hardware.plant().state();
            let pwm = hardware.applied_pwm();
            samples.push(SimulationSample {
                time_s: tick as f32 * dt,
                target_speed_rpm: config.target_speed_rpm,
                measured_speed_rpm: telemetry.measured_speed_rpm,
                phase_current_a: currents.a,
                phase_current_b: currents.b,
                phase_current_c: currents.c,
                id_ref_a: telemetry.current_reference.id_ref_a,
                iq_ref_a: telemetry.current_reference.iq_ref_a,
                id_a: telemetry.current_dq.d,
                iq_a: telemetry.current_dq.q,
                vd_v: telemetry.voltage_dq.d,
                vq_v: telemetry.voltage_dq.q,
                valpha_v: telemetry.voltage_alpha_beta.alpha,
                vbeta_v: telemetry.voltage_alpha_beta.beta,
                duty_a: pwm.duty_a,
                duty_b: pwm.duty_b,
                duty_c: pwm.duty_c,
                dc_bus_voltage_v: hardware.dc_bus_voltage(),
                load_torque_nm: hardware.plant().load_torque_nm(),
                electromagnetic_torque_nm: plant_state.electromagnetic_torque_nm,
                electrical_angle_rad: hardware.plant().electrical_angle_rad(),
                voltage_limited: telemetry.voltage_limited,
                pwm_enabled: hardware.pwm_enabled(),
            });
        }
        runtime.hardware_mut().advance(dt);
    }

    Ok(SimulationRun {
        summary: SimulationSummary {
            final_speed_rpm: runtime.hardware().speed_rpm(),
            target_speed_rpm: config.target_speed_rpm,
            peak_phase_current_a: runtime.hardware().peak_phase_current_a(),
            load_torque_nm: config.load_torque_nm,
            sample_count: samples.len(),
        },
        samples,
    })
}

/// Configuration for the correlation simulation. Unlike
/// `run_reference_simulation`, this executes the exact C-ABI controller entry
/// points used by the ADC interrupt on the STM32G431, including rev-up and the
/// selected sensorless observer.
#[derive(Clone, Copy, Debug)]
pub struct BringupSimulationConfig {
    pub duration_s: f32,
    pub target_speed_rpm: f32,
    pub closed_loop_enable: bool,
    pub dc_bus_voltage_v: f32,
    pub load_torque_nm: f32,
    pub trace_decimation: u32,
    pub observer_smo_k_slide_v: f32,
    pub observer_smo_boundary_a: f32,
    pub observer_emf_filter_alpha: f32,
    pub observer_pll_kp: f32,
    pub observer_pll_ki: f32,
    pub dead_time_enabled: bool,
    pub dead_time_ns: f32,
    pub dead_time_feedforward_enabled: bool,
    pub observer_dead_time_compensation_enabled: bool,
    pub dead_time_compensation_gain: f32,
    pub dead_time_current_zero_band_a: f32,
}

impl Default for BringupSimulationConfig {
    fn default() -> Self {
        Self {
            duration_s: 5.0,
            target_speed_rpm: 582.0,
            closed_loop_enable: false,
            dc_bus_voltage_v: 12.3,
            load_torque_nm: 0.0,
            trace_decimation: 320,
            observer_smo_k_slide_v: 4.0,
            observer_smo_boundary_a: 0.16,
            observer_emf_filter_alpha: 0.05,
            observer_pll_kp: 80.0,
            observer_pll_ki: 1_000.0,
            dead_time_enabled: false,
            dead_time_ns: 550.0,
            dead_time_feedforward_enabled: false,
            observer_dead_time_compensation_enabled: false,
            dead_time_compensation_gain: 1.0,
            dead_time_current_zero_band_a: 0.005,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BringupSimulationSample {
    pub time_s: f32,
    pub step: u32,
    pub state: u32,
    pub phase_current_a: f32,
    pub phase_current_b: f32,
    pub phase_current_c: f32,
    pub target_speed_rpm: f32,
    pub observer_speed_rpm: f32,
    pub true_speed_rpm: f32,
    pub id_ref_a: f32,
    pub iq_ref_a: f32,
    pub id_a: f32,
    pub iq_a: f32,
    pub vd_v: f32,
    pub vq_v: f32,
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
    pub dc_bus_voltage_v: f32,
    pub control_angle_rad: f32,
    pub forced_angle_rad: f32,
    pub observer_angle_rad: f32,
    pub true_angle_rad: f32,
    pub observer_reliable: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BringupSimulationSummary {
    pub final_true_speed_rpm: f32,
    pub final_observer_speed_rpm: f32,
    pub peak_phase_current_a: f32,
    pub observer_reliable_samples: usize,
    pub hold_speed_rmse_rpm: f32,
    pub hold_angle_rmse_rad: f32,
    pub transient_speed_target_rmse_rpm: f32,
    pub transient_observer_rmse_rpm: f32,
    pub steady_true_speed_mean_rpm: f32,
    pub steady_true_speed_std_rpm: f32,
    pub steady_observer_rmse_rpm: f32,
    pub steady_iq_tracking_rmse_a: f32,
    pub steady_id_rmse_a: f32,
    pub final_state: u32,
    pub sample_count: usize,
}

#[derive(Debug)]
pub enum BringupSimulationError {
    InvalidConfig(&'static str),
    Bridge(foc_rt_bridge::FocStatus),
}

#[derive(Debug)]
pub struct BringupSimulationRun {
    pub samples: Vec<BringupSimulationSample>,
    pub summary: BringupSimulationSummary,
}

impl BringupSimulationRun {
    pub fn write_csv(&self, path: impl AsRef<Path>) -> std::io::Result<()> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file);
        writeln!(
            writer,
            "time_s,step,state,phase_current_a,phase_current_b,phase_current_c,target_speed_rpm,observer_speed_rpm,true_speed_rpm,id_ref_a,iq_ref_a,id_a,iq_a,vd_v,vq_v,duty_a,duty_b,duty_c,dc_bus_voltage_v,control_angle_rad,forced_angle_rad,observer_angle_rad,true_angle_rad,observer_reliable,flags"
        )?;
        for sample in &self.samples {
            writeln!(
                writer,
                "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},0",
                sample.time_s,
                sample.step,
                sample.state,
                sample.phase_current_a,
                sample.phase_current_b,
                sample.phase_current_c,
                sample.target_speed_rpm,
                sample.observer_speed_rpm,
                sample.true_speed_rpm,
                sample.id_ref_a,
                sample.iq_ref_a,
                sample.id_a,
                sample.iq_a,
                sample.vd_v,
                sample.vq_v,
                sample.duty_a,
                sample.duty_b,
                sample.duty_c,
                sample.dc_bus_voltage_v,
                sample.control_angle_rad,
                sample.forced_angle_rad,
                sample.observer_angle_rad,
                sample.true_angle_rad,
                u8::from(sample.observer_reliable),
            )?;
        }
        writer.flush()
    }
}

/// Runs the firmware-facing rev-up/current-loop/observer path against the
/// host PMSM plant. The command path is identical to target firmware; only the
/// ADC/PWM hardware adapter and physical motor are replaced by the plant.
pub fn run_bringup_simulation(
    config: BringupSimulationConfig,
) -> Result<BringupSimulationRun, BringupSimulationError> {
    use foc_rt_bridge::{
        foc_rust_configure, foc_rust_default_st_config, foc_rust_init, foc_rust_realtime_step,
        foc_rust_set_host_dead_time_compensation, foc_rust_start_realtime, FocFeedback, FocOutput,
        FocRuntimeConfig, FocRustContextStorage, FocStatus, FocTelemetry, HostDeadTimeCompensation,
        FOC_RUST_CONTEXT_CAPACITY,
    };

    if !config.duration_s.is_finite() || config.duration_s <= 0.0 {
        return Err(BringupSimulationError::InvalidConfig(
            "duration_s must be positive",
        ));
    }
    if !config.target_speed_rpm.is_finite()
        || !config.dc_bus_voltage_v.is_finite()
        || config.dc_bus_voltage_v <= 0.0
        || !config.load_torque_nm.is_finite()
        || config.trace_decimation == 0
        || !config.observer_smo_k_slide_v.is_finite()
        || config.observer_smo_k_slide_v <= 0.0
        || !config.observer_smo_boundary_a.is_finite()
        || config.observer_smo_boundary_a <= 0.0
        || !config.observer_emf_filter_alpha.is_finite()
        || !(0.0001..=1.0).contains(&config.observer_emf_filter_alpha)
        || !config.observer_pll_kp.is_finite()
        || config.observer_pll_kp < 0.0
        || !config.observer_pll_ki.is_finite()
        || config.observer_pll_ki < 0.0
        || !config.dead_time_ns.is_finite()
        || !(0.0..=10_000.0).contains(&config.dead_time_ns)
        || !config.dead_time_compensation_gain.is_finite()
        || !(0.0..=4.0).contains(&config.dead_time_compensation_gain)
        || !config.dead_time_current_zero_band_a.is_finite()
        || !(0.0..=10.0).contains(&config.dead_time_current_zero_band_a)
        || (!config.dead_time_enabled
            && (config.dead_time_feedforward_enabled
                || config.observer_dead_time_compensation_enabled))
    {
        return Err(BringupSimulationError::InvalidConfig(
            "target, bus, load and decimation must be valid",
        ));
    }

    let params = foc_control::st_gbm2804_reference_parameters();
    let dt = 1.0 / params.pwm_frequency_hz as f32;
    let dead_time_s = config.dead_time_ns * 1.0e-9;
    let inverter = InverterSimulationConfig {
        dead_time_enabled: config.dead_time_enabled,
        dead_time_s,
    };
    let total_steps = (config.duration_s / dt).round() as u32;
    let mut plant = PmsmPlant::new(params.motor);
    plant.set_load_torque_nm(config.load_torque_nm);
    let mut context = FocRustContextStorage {
        bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
    };
    let mut runtime_config = FocRuntimeConfig::default();

    let require_ok = |status: FocStatus| {
        if status == FocStatus::Ok {
            Ok(())
        } else {
            Err(BringupSimulationError::Bridge(status))
        }
    };
    // SAFETY: all values are local, aligned ABI objects with exclusive access.
    unsafe {
        require_ok(foc_rust_init(&mut context))?;
        require_ok(foc_rust_set_host_dead_time_compensation(
            &mut context,
            HostDeadTimeCompensation {
                dead_time_s,
                current_zero_band_a: config.dead_time_current_zero_band_a,
                feedforward_gain: config.dead_time_compensation_gain,
                feedforward_enabled: config.dead_time_feedforward_enabled,
                observer_voltage_correction_enabled: config.observer_dead_time_compensation_enabled,
            },
        ))?;
        require_ok(foc_rust_default_st_config(&mut runtime_config))?;
        runtime_config.observer_enable = 1;
        runtime_config.closed_loop_enable = u32::from(config.closed_loop_enable);
        runtime_config.observer_update_divider = 1;
        runtime_config.voltage_utilization = 0.90;
        runtime_config.observer_smo_k_slide_v = config.observer_smo_k_slide_v;
        runtime_config.observer_smo_boundary_a = config.observer_smo_boundary_a;
        runtime_config.observer_emf_filter_alpha = config.observer_emf_filter_alpha;
        runtime_config.observer_pll_kp = config.observer_pll_kp;
        runtime_config.observer_pll_ki = config.observer_pll_ki;
        require_ok(foc_rust_configure(&mut context, &runtime_config))?;
        require_ok(foc_rust_start_realtime(
            &mut context,
            1,
            config.target_speed_rpm,
        ))?;
    }

    let mut samples = Vec::with_capacity((total_steps / config.trace_decimation + 2) as usize);
    let mut peak_phase_current_a = 0.0_f32;
    let mut final_telemetry = FocTelemetry::default();
    for tick in 0..total_steps {
        let currents = plant.phase_currents();
        peak_phase_current_a = peak_phase_current_a
            .max(currents.a.abs())
            .max(currents.b.abs())
            .max(currents.c.abs());
        let feedback = FocFeedback {
            phase_current_a: currents.a,
            phase_current_b: currents.b,
            phase_current_c: currents.c,
            dc_bus_voltage: config.dc_bus_voltage_v,
            electrical_angle_rad: 0.0,
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        // SAFETY: the context is initialized and all pointers reference
        // disjoint local values for the duration of this call.
        let status =
            unsafe { foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry) };
        require_ok(status)?;
        final_telemetry = telemetry;

        if (tick % config.trace_decimation == 0) || (tick + 1 == total_steps) {
            let plant_state = plant.state();
            samples.push(BringupSimulationSample {
                time_s: tick as f32 * dt,
                step: tick + 1,
                state: telemetry.state,
                phase_current_a: currents.a,
                phase_current_b: currents.b,
                phase_current_c: currents.c,
                target_speed_rpm: telemetry.target_speed_rpm,
                observer_speed_rpm: telemetry.measured_speed_rpm,
                true_speed_rpm: plant_state.mechanical_speed_rad_s * 30.0 / PI,
                id_ref_a: telemetry.id_reference_a,
                iq_ref_a: telemetry.iq_reference_a,
                id_a: telemetry.id_measured_a,
                iq_a: telemetry.iq_measured_a,
                vd_v: telemetry.vd_command_v,
                vq_v: telemetry.vq_command_v,
                duty_a: output.duty_a,
                duty_b: output.duty_b,
                duty_c: output.duty_c,
                dc_bus_voltage_v: config.dc_bus_voltage_v,
                control_angle_rad: telemetry.electrical_angle_rad,
                forced_angle_rad: telemetry.forced_electrical_angle_rad,
                observer_angle_rad: telemetry.observer_electrical_angle_rad,
                true_angle_rad: plant.electrical_angle_rad(),
                observer_reliable: telemetry.observer_reliable != 0,
            });
        }

        plant.step(
            pwm_to_alpha_beta_with_inverter(
                PwmCommand {
                    duty_a: output.duty_a,
                    duty_b: output.duty_b,
                    duty_c: output.duty_c,
                },
                currents,
                config.dc_bus_voltage_v,
                dt,
                inverter,
            ),
            dt,
        );
    }

    let final_state = plant.state();
    let hold_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| sample.time_s >= 2.5)
        .collect();
    let hold_count = hold_samples.len().max(1) as f32;
    let hold_speed_rmse_rpm = (hold_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / hold_count)
        .sqrt();
    let hold_angle_rmse_rad = (hold_samples
        .iter()
        .map(|sample| {
            let error =
                (sample.observer_angle_rad - sample.true_angle_rad + PI).rem_euclid(2.0 * PI) - PI;
            error * error
        })
        .sum::<f32>()
        / hold_count)
        .sqrt();
    let transient_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| (4.0..=5.0).contains(&sample.time_s))
        .collect();
    let transient_count = transient_samples.len().max(1) as f32;
    let transient_speed_target_rmse_rpm = (transient_samples
        .iter()
        .map(|sample| {
            let error = sample.true_speed_rpm - config.target_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / transient_count)
        .sqrt();
    let transient_observer_rmse_rpm = (transient_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / transient_count)
        .sqrt();
    let steady_start_s = (config.duration_s - 2.0).max(2.5);
    let steady_samples: Vec<&BringupSimulationSample> = samples
        .iter()
        .filter(|sample| sample.time_s >= steady_start_s)
        .collect();
    let steady_count = steady_samples.len().max(1) as f32;
    let steady_true_speed_mean_rpm = steady_samples
        .iter()
        .map(|sample| sample.true_speed_rpm)
        .sum::<f32>()
        / steady_count;
    let steady_true_speed_std_rpm = (steady_samples
        .iter()
        .map(|sample| {
            let delta = sample.true_speed_rpm - steady_true_speed_mean_rpm;
            delta * delta
        })
        .sum::<f32>()
        / (steady_samples.len().saturating_sub(1).max(1) as f32))
        .sqrt();
    let steady_observer_rmse_rpm = (steady_samples
        .iter()
        .map(|sample| {
            let error = sample.observer_speed_rpm - sample.true_speed_rpm;
            error * error
        })
        .sum::<f32>()
        / steady_count)
        .sqrt();
    let steady_iq_tracking_rmse_a = (steady_samples
        .iter()
        .map(|sample| {
            let error = sample.iq_a - sample.iq_ref_a;
            error * error
        })
        .sum::<f32>()
        / steady_count)
        .sqrt();
    let steady_id_rmse_a = (steady_samples
        .iter()
        .map(|sample| sample.id_a * sample.id_a)
        .sum::<f32>()
        / steady_count)
        .sqrt();
    Ok(BringupSimulationRun {
        summary: BringupSimulationSummary {
            final_true_speed_rpm: final_state.mechanical_speed_rad_s * 30.0 / PI,
            final_observer_speed_rpm: final_telemetry.measured_speed_rpm,
            peak_phase_current_a,
            observer_reliable_samples: samples
                .iter()
                .filter(|sample| sample.observer_reliable)
                .count(),
            hold_speed_rmse_rpm,
            hold_angle_rmse_rad,
            transient_speed_target_rmse_rpm,
            transient_observer_rmse_rpm,
            steady_true_speed_mean_rpm,
            steady_true_speed_std_rpm,
            steady_observer_rmse_rpm,
            steady_iq_tracking_rmse_a,
            steady_id_rmse_a,
            final_state: final_telemetry.state,
            sample_count: samples.len(),
        },
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_control::{
        st_gbm2804_reference_parameters, ControlRuntime, SpeedCommand, StReferenceController,
    };

    fn run_for(runtime: &mut ControlRuntime<SimHardware>, seconds: f32, target_rpm: f32) {
        let dt = runtime.controller_period_s();
        let steps = (seconds / dt) as usize;
        for _ in 0..steps {
            runtime
                .tick(SpeedCommand {
                    target_rpm,
                    id_ref_a: 0.0,
                })
                .expect("closed-loop tick");
            runtime.hardware_mut().advance(dt);
        }
    }

    #[test]
    fn speed_step_closes_controller_inverter_motor_sensor_loop() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        run_for(&mut runtime, 2.0, params.default_target_speed_rpm);
        let speed = runtime.hardware().speed_rpm();
        assert!(
            (speed - params.default_target_speed_rpm).abs() < 35.0,
            "speed={speed}"
        );
        assert!(runtime.hardware().peak_phase_current_a() < 1.2);
    }

    #[test]
    fn load_step_is_rejected_and_speed_recovers() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        run_for(&mut runtime, 1.0, 524.0);
        runtime.hardware_mut().plant_mut().set_load_torque_nm(0.004);
        run_for(&mut runtime, 2.0, 524.0);
        assert!((runtime.hardware().speed_rpm() - 524.0).abs() < 15.0);
    }

    #[test]
    fn injected_hardware_fault_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.hardware_mut().inject_fault();
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::PowerStageFault)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    #[test]
    fn lost_feedback_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.tick(SpeedCommand::default()).unwrap();
        assert!(runtime.hardware().pwm_enabled());
        runtime.hardware_mut().set_feedback_available(false);
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::FeedbackUnavailable)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    #[test]
    fn rejected_output_disables_pwm() {
        let params = st_gbm2804_reference_parameters();
        let mut runtime = ControlRuntime::new(
            StReferenceController::new(params),
            SimHardware::new(params.motor),
        );
        runtime.enable().unwrap();
        runtime.hardware_mut().set_reject_output(true);
        assert_eq!(
            runtime.tick(SpeedCommand::default()),
            Err(HardwareFault::OutputRejected)
        );
        assert!(!runtime.hardware().pwm_enabled());
    }

    #[test]
    fn reference_trace_contains_load_step_and_finite_control_data() {
        let run = run_reference_simulation(SimulationConfig {
            duration_s: 0.05,
            load_step_time_s: 0.02,
            trace_decimation: 25,
            ..SimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.len() > 10);
        assert_eq!(run.samples.first().unwrap().load_torque_nm, 0.0);
        assert_eq!(run.samples.last().unwrap().load_torque_nm, 0.004);
        assert!(run
            .samples
            .iter()
            .all(|sample| sample.measured_speed_rpm.is_finite()
                && sample.id_a.is_finite()
                && sample.iq_a.is_finite()
                && sample.duty_a.is_finite()));
    }

    #[test]
    fn inverter_model_applies_expected_dead_time_voltage_loss() {
        let pwm = PwmCommand {
            duty_a: 0.6,
            duty_b: 0.4,
            duty_c: 0.5,
        };
        let currents = PhaseCurrents {
            a: 1.0,
            b: -1.0,
            c: 0.0,
        };
        let period_s = 1.0 / 12_000.0;
        let ideal = pwm_to_alpha_beta_with_inverter(
            pwm,
            currents,
            12.3,
            period_s,
            InverterSimulationConfig::default(),
        );
        let actual = pwm_to_alpha_beta_with_inverter(
            pwm,
            currents,
            12.3,
            period_s,
            InverterSimulationConfig {
                dead_time_enabled: true,
                dead_time_s: 550.0e-9,
            },
        );
        let expected_loss_v = 2.0 * 550.0e-9 / period_s * 12.3;
        assert!((ideal.alpha - actual.alpha - expected_loss_v).abs() < 1.0e-6);
    }

    #[test]
    fn dead_time_compensation_reduces_closed_loop_error() {
        let baseline = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 7.0,
            closed_loop_enable: true,
            trace_decimation: 16,
            dead_time_enabled: true,
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        let compensated = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 7.0,
            closed_loop_enable: true,
            trace_decimation: 16,
            dead_time_enabled: true,
            dead_time_feedforward_enabled: true,
            observer_dead_time_compensation_enabled: true,
            ..BringupSimulationConfig::default()
        })
        .unwrap();

        assert_eq!(baseline.summary.final_state, 7);
        assert_eq!(compensated.summary.final_state, 7);
        // At 12 kHz the speed loop removes most steady mechanical-speed bias,
        // so phase-current distortion is the direct dead-time regression
        // metric. Observer error must still improve rather than being traded
        // for lower current error.
        assert!(
            compensated.summary.steady_iq_tracking_rmse_a
                < baseline.summary.steady_iq_tracking_rmse_a * 0.75
        );
        assert!(compensated.summary.steady_id_rmse_a < baseline.summary.steady_id_rmse_a * 0.75);
        assert!(
            compensated.summary.steady_observer_rmse_rpm
                < baseline.summary.steady_observer_rmse_rpm
        );
    }

    #[test]
    fn bringup_trace_runs_the_same_open_loop_bridge_as_firmware() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 0.8,
            trace_decimation: 160,
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.len() > 50);
        assert!(run.summary.peak_phase_current_a.is_finite());
        assert!(run.summary.final_true_speed_rpm.is_finite());
        assert!(run.samples.iter().all(|sample| {
            sample.phase_current_a.is_finite()
                && sample.iq_a.is_finite()
                && sample.duty_a.is_finite()
                && (0.0..=1.0).contains(&sample.duty_a)
        }));
    }

    #[test]
    fn bringup_bridge_reaches_closed_loop_without_iq_step() {
        let run = run_bringup_simulation(BringupSimulationConfig {
            duration_s: 4.0,
            closed_loop_enable: true,
            trace_decimation: 1,
            ..BringupSimulationConfig::default()
        })
        .unwrap();
        assert!(run.samples.iter().any(|sample| sample.state == 6));
        assert!(run.samples.iter().any(|sample| sample.state == 7));
        assert_eq!(run.summary.final_state, 7);

        let maximum_iq_step = run
            .samples
            .windows(2)
            .filter(|pair| pair[0].state >= 6 || pair[1].state >= 6)
            .map(|pair| (pair[1].iq_ref_a - pair[0].iq_ref_a).abs())
            .fold(0.0_f32, f32::max);
        assert!(maximum_iq_step < 0.02, "maximum_iq_step={maximum_iq_step}");

        let maximum_duty_step = run
            .samples
            .windows(2)
            .filter(|pair| pair[0].state >= 6 || pair[1].state >= 6)
            .map(|pair| {
                (pair[1].duty_a - pair[0].duty_a)
                    .abs()
                    .max((pair[1].duty_b - pair[0].duty_b).abs())
                    .max((pair[1].duty_c - pair[0].duty_c).abs())
            })
            .fold(0.0_f32, f32::max);
        assert!(
            maximum_duty_step < 0.05,
            "maximum_duty_step={maximum_duty_step}"
        );
    }
}
