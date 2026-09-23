#![cfg_attr(target_os = "none", no_std)]

//! RT-Thread C firmware and the pure Rust FOC algorithms meet at this crate.
//! Unsafe code is restricted to checked conversion of caller-owned C storage.

use core::f32::consts::PI;
use core::mem::{align_of, size_of};
use core::ptr;
use foc_algorithm::{
    clarke, svpwm_update, wrap_angle_0_to_2pi, Abc, AlphaBeta, PiParam, SvpwmParam,
};
use foc_control::{
    st_gbm2804_reference_parameters, ConfigurableObserver, ControlMath, ControlParameters, CpuMath,
    CurrentCommand, CurrentLoop, FeedbackSnapshot, ObserverBackend, ObserverReliabilityConfig,
    PhaseCurrents, PwmCommand, RevUpConfig, RevUpPhase, RevUpSequencer, RotorEstimator,
    RotorFeedback, SmoPllTuning, SpeedCommand, SpeedLoop,
};

pub const FOC_RUST_ABI_VERSION: u32 = 0x0007_0000;
pub const FOC_RUST_CONFIG_VERSION: u32 = 4;
pub const FOC_RUST_CONTEXT_CAPACITY: usize = 2048;
pub const FOC_FAULT_ALGORITHM_OUTPUT: u32 = 1 << 0;
pub const FOC_FAULT_INVALID_FEEDBACK: u32 = 1 << 1;
pub const FOC_FAULT_OBSERVER_STARTUP: u32 = 1 << 2;
pub const FOC_FAULT_OBSERVER_LOST: u32 = 1 << 3;

const CONTEXT_MAGIC: u32 = 0x464F_4352; // "FOCR"

#[derive(Default)]
struct PlatformMath {
    cpu: CpuMath,
}

impl ControlMath for PlatformMath {
    fn sin_cos(&mut self, angle_rad: f32) -> (f32, f32) {
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut sin = 0.0;
            let mut cos = 0.0;
            // SAFETY: the C adapter writes only the two valid stack outputs. It
            // returns zero when CORDIC is unavailable, which selects CpuMath.
            if unsafe { foc_math_accel_sin_cos(angle_rad, &mut sin, &mut cos) } != 0
                && sin.is_finite()
                && cos.is_finite()
            {
                return (sin, cos);
            }
        }
        self.cpu.sin_cos(angle_rad)
    }

    fn magnitude(&mut self, x: f32, y: f32) -> f32 {
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut magnitude = 0.0;
            // SAFETY: the C adapter writes only the valid stack output and
            // reports failure instead of returning an unchecked value.
            if unsafe { foc_math_accel_magnitude(x, y, &mut magnitude) } != 0
                && magnitude.is_finite()
                && magnitude >= 0.0
            {
                return magnitude;
            }
        }
        self.cpu.magnitude(x, y)
    }

    fn angle_0_to_2pi(&mut self, y: f32, x: f32) -> f32 {
        #[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
        {
            let mut angle = 0.0;
            // SAFETY: the C adapter writes one valid stack output and returns
            // zero when CORDIC is unavailable.
            if unsafe { foc_math_accel_atan2(y, x, &mut angle) } != 0 && angle.is_finite() {
                return angle;
            }
        }
        self.cpu.angle_0_to_2pi(y, x)
    }
}

#[cfg(all(feature = "stm32g4-cordic", target_os = "none"))]
extern "C" {
    fn foc_math_accel_sin_cos(angle_rad: f32, sin_out: *mut f32, cos_out: *mut f32) -> u32;
    fn foc_math_accel_magnitude(x: f32, y: f32, magnitude_out: *mut f32) -> u32;
    fn foc_math_accel_atan2(y: f32, x: f32, angle_out: *mut f32) -> u32;
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FocStatus {
    Ok = 0,
    Disabled = 1,
    NotConfigured = 2,
    InvalidArgument = 3,
    HardwareFault = 4,
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FocState {
    Uninitialized = 0,
    Disabled = 1,
    Running = 2,
    Alignment = 3,
    OpenLoopRamp = 4,
    OpenLoopHold = 5,
    ObserverTransition = 6,
    ClosedLoop = 7,
    Fault = 8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocFeedback {
    pub phase_current_a: f32,
    pub phase_current_b: f32,
    pub phase_current_c: f32,
    pub dc_bus_voltage: f32,
    pub electrical_angle_rad: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocReference {
    pub id_ref: f32,
    pub iq_ref: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocOutput {
    pub duty_a: f32,
    pub duty_b: f32,
    pub duty_c: f32,
}

/// Host-simulation-only dead-time compensation controls.
///
/// This is deliberately not part of the C ABI or `FocRuntimeConfig`: target
/// firmware keeps its existing ABI and behavior until hardware calibration is
/// complete. `foc-sim` can still exercise the exact realtime controller with
/// a compensated PWM command and compensated observer voltage reconstruction.
#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HostDeadTimeCompensation {
    pub dead_time_s: f32,
    pub current_zero_band_a: f32,
    pub feedforward_gain: f32,
    pub feedforward_enabled: bool,
    pub observer_voltage_correction_enabled: bool,
}

#[cfg(not(target_os = "none"))]
impl HostDeadTimeCompensation {
    fn is_valid(self) -> bool {
        self.dead_time_s.is_finite()
            && (0.0..=0.01).contains(&self.dead_time_s)
            && self.current_zero_band_a.is_finite()
            && (0.0..=10.0).contains(&self.current_zero_band_a)
            && self.feedforward_gain.is_finite()
            && (0.0..=4.0).contains(&self.feedforward_gain)
    }

    fn current_polarity(self, current_a: f32) -> f32 {
        if self.current_zero_band_a > 0.0 {
            (current_a / self.current_zero_band_a).clamp(-1.0, 1.0)
        } else if current_a > 0.0 {
            1.0
        } else if current_a < 0.0 {
            -1.0
        } else {
            0.0
        }
    }

    fn dead_time_duty(self, controller_period_s: f32) -> f32 {
        2.0 * self.dead_time_s / controller_period_s
    }

    fn observer_pwm(
        self,
        pwm: PwmCommand,
        currents: PhaseCurrents,
        controller_period_s: f32,
    ) -> PwmCommand {
        if !self.observer_voltage_correction_enabled {
            return pwm;
        }
        let loss = self.dead_time_duty(controller_period_s);
        PwmCommand {
            duty_a: pwm.duty_a - loss * self.current_polarity(currents.a),
            duty_b: pwm.duty_b - loss * self.current_polarity(currents.b),
            duty_c: pwm.duty_c - loss * self.current_polarity(currents.c),
        }
    }

    fn feedforward_pwm(
        self,
        pwm: PwmCommand,
        currents: PhaseCurrents,
        controller_period_s: f32,
    ) -> PwmCommand {
        if !self.feedforward_enabled {
            return pwm;
        }
        let correction = self.feedforward_gain * self.dead_time_duty(controller_period_s);
        PwmCommand {
            duty_a: (pwm.duty_a + correction * self.current_polarity(currents.a)).clamp(0.0, 1.0),
            duty_b: (pwm.duty_b + correction * self.current_polarity(currents.b)).clamp(0.0, 1.0),
            duty_c: (pwm.duty_c + correction * self.current_polarity(currents.c)).clamp(0.0, 1.0),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocPiConfig {
    pub kp: f32,
    pub ki: f32,
    pub ts: f32,
    pub out_min: f32,
    pub out_max: f32,
    pub integrator_min: f32,
    pub integrator_max: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocBasicConfig {
    pub id_pi: FocPiConfig,
    pub iq_pi: FocPiConfig,
    pub nominal_dc_bus_voltage: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocRuntimeConfig {
    pub struct_size: u32,
    pub config_version: u32,
    pub observer_backend: u32,
    pub observer_enable: u32,
    pub closed_loop_enable: u32,
    pub observer_update_divider: u32,
    pub id_pi: FocPiConfig,
    pub iq_pi: FocPiConfig,
    pub speed_pi: FocPiConfig,
    pub pole_pairs: u32,
    pub pwm_frequency_hz: u32,
    pub speed_loop_frequency_hz: u32,
    pub stator_resistance_ohm: f32,
    pub stator_inductance_h: f32,
    pub flux_linkage_wb: f32,
    pub rated_current_a: f32,
    pub max_speed_rpm: f32,
    pub nominal_bus_voltage_v: f32,
    pub voltage_utilization: f32,
    pub default_target_speed_rpm: f32,
    pub alignment_duration_s: f32,
    pub open_loop_ramp_duration_s: f32,
    pub observer_transition_duration_s: f32,
    pub startup_final_speed_rpm: f32,
    pub startup_current_a: f32,
    pub observer_smo_k_slide_v: f32,
    pub observer_smo_boundary_a: f32,
    pub observer_emf_filter_alpha: f32,
    pub observer_pll_kp: f32,
    pub observer_pll_ki: f32,
    pub observer_minimum_speed_rpm: f32,
    pub observer_minimum_bemf_v: f32,
    pub observer_speed_variance_ratio: f32,
    pub observer_consecutive_samples: u32,
    pub observer_acquisition_timeout_s: f32,
    pub observer_loss_timeout_s: f32,
    pub closed_loop_speed_ramp_rpm_per_s: f32,
    pub speed_pi_preload_ratio: f32,
    pub closed_loop_current_slew_a_per_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct FocTelemetry {
    pub state: u32,
    pub observer_backend: u32,
    pub observer_reliable: u32,
    pub closed_loop_active: u32,
    pub target_speed_rpm: f32,
    pub measured_speed_rpm: f32,
    pub electrical_angle_rad: f32,
    pub id_reference_a: f32,
    pub iq_reference_a: f32,
    pub id_measured_a: f32,
    pub iq_measured_a: f32,
    pub vd_command_v: f32,
    pub vq_command_v: f32,
    pub forced_electrical_angle_rad: f32,
    pub observer_electrical_angle_rad: f32,
}

#[repr(C, align(8))]
pub struct FocRustContextStorage {
    pub bytes: [u8; FOC_RUST_CONTEXT_CAPACITY],
}

struct Controller {
    magic: u32,
    state: FocState,
    fault_flags: u32,
    algorithm_configured: bool,
    params: ControlParameters,
    current_loop: CurrentLoop,
    speed_loop: SpeedLoop,
    trial_angle_rad: f32,
    runtime_config: FocRuntimeConfig,
    observer: ConfigurableObserver,
    observer_counter: u32,
    observer_feedback: RotorFeedback,
    observer_reliable: bool,
    startup: RevUpSequencer,
    previous_pwm: PwmCommand,
    current_reference: CurrentCommand,
    speed_current_command: CurrentCommand,
    target_speed_rpm: f32,
    speed_reference_rpm: f32,
    speed_counter: u32,
    observer_wait_elapsed_s: f32,
    observer_loss_elapsed_s: f32,
    closed_loop_initialized: bool,
    #[cfg(not(target_os = "none"))]
    host_dead_time_compensation: HostDeadTimeCompensation,
    telemetry: FocTelemetry,
}

impl Controller {
    fn new() -> Self {
        let params = st_gbm2804_reference_parameters();
        let runtime_config = default_st_runtime_config();
        Self {
            magic: CONTEXT_MAGIC,
            state: FocState::Disabled,
            fault_flags: 0,
            algorithm_configured: false,
            params,
            current_loop: CurrentLoop::default(),
            speed_loop: SpeedLoop::default(),
            trial_angle_rad: 0.0,
            runtime_config,
            observer: ConfigurableObserver::new(
                ObserverBackend::SmoPll,
                params.motor,
                1.0 / params.pwm_frequency_hz as f32,
            ),
            observer_counter: 0,
            observer_feedback: RotorFeedback::default(),
            observer_reliable: false,
            startup: RevUpSequencer::with_config(params.motor.pole_pairs, RevUpConfig::default()),
            previous_pwm: PwmCommand::default(),
            current_reference: CurrentCommand::default(),
            speed_current_command: CurrentCommand::default(),
            target_speed_rpm: params.default_target_speed_rpm,
            speed_reference_rpm: 0.0,
            speed_counter: 0,
            observer_wait_elapsed_s: 0.0,
            observer_loss_elapsed_s: 0.0,
            closed_loop_initialized: false,
            #[cfg(not(target_os = "none"))]
            host_dead_time_compensation: HostDeadTimeCompensation::default(),
            telemetry: FocTelemetry::default(),
        }
    }

    fn stop(&mut self) {
        self.state = FocState::Disabled;
        self.current_loop.reset();
        self.speed_loop.reset();
        self.trial_angle_rad = 0.0;
        self.observer.reset(0.0);
        self.observer_counter = 0;
        self.observer_feedback = RotorFeedback::default();
        self.observer_reliable = false;
        self.startup.reset();
        self.previous_pwm = PwmCommand::default();
        self.current_reference = CurrentCommand::default();
        self.speed_current_command = CurrentCommand::default();
        self.speed_reference_rpm = 0.0;
        self.speed_counter = 0;
        self.observer_wait_elapsed_s = 0.0;
        self.observer_loss_elapsed_s = 0.0;
        self.closed_loop_initialized = false;
        self.telemetry = FocTelemetry::default();
    }
}

const _: () = assert!(size_of::<Controller>() <= FOC_RUST_CONTEXT_CAPACITY);
const _: () = assert!(align_of::<Controller>() <= align_of::<FocRustContextStorage>());

fn zero_output(output: &mut FocOutput) {
    *output = FocOutput::default();
}

fn move_towards(current: f32, target: f32, maximum_step: f32) -> f32 {
    let delta = target - current;
    if delta.abs() <= maximum_step {
        target
    } else {
        current + maximum_step.copysign(delta)
    }
}

fn pi_is_valid(pi: &FocPiConfig) -> bool {
    pi.kp.is_finite()
        && pi.ki.is_finite()
        && pi.ts.is_finite()
        && pi.out_min.is_finite()
        && pi.out_max.is_finite()
        && pi.integrator_min.is_finite()
        && pi.integrator_max.is_finite()
        && pi.ts > 0.0
        && pi.out_min <= pi.out_max
        && pi.integrator_min <= pi.integrator_max
}

fn config_is_valid(config: &FocBasicConfig) -> bool {
    pi_is_valid(&config.id_pi)
        && pi_is_valid(&config.iq_pi)
        && config.nominal_dc_bus_voltage.is_finite()
        && config.nominal_dc_bus_voltage > 0.0
}

fn feedback_is_valid(feedback: &FocFeedback) -> bool {
    feedback.phase_current_a.is_finite()
        && feedback.phase_current_b.is_finite()
        && feedback.phase_current_c.is_finite()
        && feedback.dc_bus_voltage.is_finite()
        && feedback.dc_bus_voltage > 0.0
        && feedback.electrical_angle_rad.is_finite()
}

fn reference_is_valid(reference: &FocReference) -> bool {
    reference.id_ref.is_finite() && reference.iq_ref.is_finite()
}

fn to_pi_param(config: FocPiConfig) -> PiParam {
    PiParam {
        kp: config.kp,
        ki: config.ki,
        ts: config.ts,
        out_min: config.out_min,
        out_max: config.out_max,
        integrator_min: config.integrator_min,
        integrator_max: config.integrator_max,
    }
}

fn from_pi_param(param: PiParam) -> FocPiConfig {
    FocPiConfig {
        kp: param.kp,
        ki: param.ki,
        ts: param.ts,
        out_min: param.out_min,
        out_max: param.out_max,
        integrator_min: param.integrator_min,
        integrator_max: param.integrator_max,
    }
}

fn default_st_runtime_config() -> FocRuntimeConfig {
    let params = st_gbm2804_reference_parameters();
    let startup = RevUpConfig::default();
    let observer = SmoPllTuning::for_motor(params.motor);
    let reliability = ObserverReliabilityConfig::for_motor(params.motor);
    FocRuntimeConfig {
        struct_size: size_of::<FocRuntimeConfig>() as u32,
        config_version: FOC_RUST_CONFIG_VERSION,
        observer_backend: ObserverBackend::SmoPll as u32,
        observer_enable: 1,
        // First board runs stay in current-controlled open loop. This field is
        // explicitly enabled only after observer telemetry passes the handoff
        // gates on the actual motor.
        closed_loop_enable: 0,
        // At 12 kHz the STM32G431 has enough budget to execute the selected
        // observer and current controller in the same PWM period.
        observer_update_divider: 1,
        id_pi: from_pi_param(params.id_pi),
        iq_pi: from_pi_param(params.iq_pi),
        speed_pi: from_pi_param(params.speed_pi),
        pole_pairs: params.motor.pole_pairs as u32,
        pwm_frequency_hz: params.pwm_frequency_hz,
        speed_loop_frequency_hz: params.speed_loop_frequency_hz,
        stator_resistance_ohm: params.motor.stator_resistance_ohm,
        stator_inductance_h: params.motor.ld_h,
        flux_linkage_wb: params.motor.flux_linkage_wb,
        rated_current_a: params.motor.rated_current_a,
        max_speed_rpm: params.motor.max_speed_rpm,
        nominal_bus_voltage_v: params.motor.nominal_bus_voltage_v,
        voltage_utilization: params.voltage_utilization,
        default_target_speed_rpm: params.default_target_speed_rpm,
        alignment_duration_s: startup.alignment_s,
        open_loop_ramp_duration_s: startup.ramp_s,
        observer_transition_duration_s: startup.transition_s,
        startup_final_speed_rpm: startup.final_speed_rpm,
        startup_current_a: startup.final_current_a,
        observer_smo_k_slide_v: observer.k_slide_v,
        observer_smo_boundary_a: observer.boundary_a,
        observer_emf_filter_alpha: observer.emf_filter_alpha,
        observer_pll_kp: observer.pll_kp,
        observer_pll_ki: observer.pll_ki,
        observer_minimum_speed_rpm: params.default_target_speed_rpm,
        observer_minimum_bemf_v: reliability.minimum_bemf_v,
        observer_speed_variance_ratio: reliability.speed_variance_ratio,
        observer_consecutive_samples: reliability.consecutive_samples as u32,
        observer_acquisition_timeout_s: 0.5,
        observer_loss_timeout_s: 0.05,
        closed_loop_speed_ramp_rpm_per_s: 500.0,
        // The actual MCSDK project has PID_SPEED_INTEGRAL_INIT_DIV=0, so its
        // speed PI integral starts at zero. The applied Iq still crosses over
        // continuously through the configurable current slew below.
        speed_pi_preload_ratio: 0.0,
        closed_loop_current_slew_a_per_s: 32.0,
    }
}

fn runtime_config_is_valid(config: &FocRuntimeConfig) -> bool {
    let startup = RevUpConfig {
        alignment_s: config.alignment_duration_s,
        ramp_s: config.open_loop_ramp_duration_s,
        transition_s: config.observer_transition_duration_s,
        final_speed_rpm: config.startup_final_speed_rpm,
        final_current_a: config.startup_current_a,
    };
    let observer = SmoPllTuning {
        k_slide_v: config.observer_smo_k_slide_v,
        boundary_a: config.observer_smo_boundary_a,
        emf_filter_alpha: config.observer_emf_filter_alpha,
        pll_kp: config.observer_pll_kp,
        pll_ki: config.observer_pll_ki,
    };
    let reliability = ObserverReliabilityConfig {
        minimum_speed_rpm: config.observer_minimum_speed_rpm,
        minimum_bemf_v: config.observer_minimum_bemf_v,
        speed_variance_ratio: config.observer_speed_variance_ratio,
        consecutive_samples: config.observer_consecutive_samples.min(u16::MAX as u32) as u16,
    };
    config.struct_size == size_of::<FocRuntimeConfig>() as u32
        && config.config_version == FOC_RUST_CONFIG_VERSION
        && ObserverBackend::from_raw(config.observer_backend).is_some()
        && config.observer_enable <= 1
        && config.closed_loop_enable <= 1
        && (config.closed_loop_enable == 0 || config.observer_enable != 0)
        && (1..=32).contains(&config.observer_update_divider)
        && pi_is_valid(&config.id_pi)
        && pi_is_valid(&config.iq_pi)
        && pi_is_valid(&config.speed_pi)
        && (1..=32).contains(&config.pole_pairs)
        && config.pwm_frequency_hz >= 1_000
        && config.speed_loop_frequency_hz >= 10
        && config.speed_loop_frequency_hz <= config.pwm_frequency_hz
        && config
            .pwm_frequency_hz
            .is_multiple_of(config.speed_loop_frequency_hz)
        && config.stator_resistance_ohm.is_finite()
        && config.stator_resistance_ohm > 0.0
        && config.stator_inductance_h.is_finite()
        && config.stator_inductance_h > 0.0
        && config.flux_linkage_wb.is_finite()
        && config.flux_linkage_wb > 0.0
        && config.rated_current_a.is_finite()
        && config.rated_current_a > 0.0
        && config.max_speed_rpm.is_finite()
        && config.max_speed_rpm > 0.0
        && config.nominal_bus_voltage_v.is_finite()
        && config.nominal_bus_voltage_v > 0.0
        && config.voltage_utilization.is_finite()
        && (0.05..=0.98).contains(&config.voltage_utilization)
        && config.default_target_speed_rpm.is_finite()
        && config.default_target_speed_rpm > 0.0
        && config.default_target_speed_rpm <= config.max_speed_rpm
        && config.startup_current_a <= config.rated_current_a
        && observer.is_valid()
        && reliability.is_valid()
        && config.observer_minimum_speed_rpm < config.max_speed_rpm * 1.10
        && (1..=1_000).contains(&config.observer_consecutive_samples)
        && config.observer_acquisition_timeout_s.is_finite()
        && (0.001..=10.0).contains(&config.observer_acquisition_timeout_s)
        && config.observer_loss_timeout_s.is_finite()
        && (0.001..=5.0).contains(&config.observer_loss_timeout_s)
        && config.closed_loop_speed_ramp_rpm_per_s.is_finite()
        && config.closed_loop_speed_ramp_rpm_per_s > 0.0
        && config.speed_pi_preload_ratio.is_finite()
        && (0.0..=1.0).contains(&config.speed_pi_preload_ratio)
        && config.closed_loop_current_slew_a_per_s.is_finite()
        && (0.01..=1_000.0).contains(&config.closed_loop_current_slew_a_per_s)
        && startup.is_valid()
}

fn configure_runtime(controller: &mut Controller, config: FocRuntimeConfig) {
    let backend =
        ObserverBackend::from_raw(config.observer_backend).unwrap_or(ObserverBackend::SmoPll);
    let mut params = st_gbm2804_reference_parameters();
    params.id_pi = to_pi_param(config.id_pi);
    params.iq_pi = to_pi_param(config.iq_pi);
    params.speed_pi = to_pi_param(config.speed_pi);
    params.motor.pole_pairs = config.pole_pairs as u8;
    params.motor.stator_resistance_ohm = config.stator_resistance_ohm;
    params.motor.ld_h = config.stator_inductance_h;
    params.motor.lq_h = config.stator_inductance_h;
    params.motor.flux_linkage_wb = config.flux_linkage_wb;
    params.motor.rated_current_a = config.rated_current_a;
    params.motor.max_speed_rpm = config.max_speed_rpm;
    params.motor.nominal_bus_voltage_v = config.nominal_bus_voltage_v;
    params.pwm_frequency_hz = config.pwm_frequency_hz;
    params.speed_loop_frequency_hz = config.speed_loop_frequency_hz;
    params.voltage_utilization = config.voltage_utilization;
    params.default_target_speed_rpm = config.default_target_speed_rpm;
    let startup_config = RevUpConfig {
        alignment_s: config.alignment_duration_s,
        ramp_s: config.open_loop_ramp_duration_s,
        transition_s: config.observer_transition_duration_s,
        final_speed_rpm: config.startup_final_speed_rpm,
        final_current_a: config.startup_current_a,
    };
    controller.params = params;
    controller.runtime_config = config;
    controller.observer = ConfigurableObserver::new_with_smo_tuning_and_reliability(
        backend,
        params.motor,
        config.observer_update_divider as f32 / params.pwm_frequency_hz as f32,
        SmoPllTuning {
            k_slide_v: config.observer_smo_k_slide_v,
            boundary_a: config.observer_smo_boundary_a,
            emf_filter_alpha: config.observer_emf_filter_alpha,
            pll_kp: config.observer_pll_kp,
            pll_ki: config.observer_pll_ki,
        },
        ObserverReliabilityConfig {
            minimum_speed_rpm: config.observer_minimum_speed_rpm,
            minimum_bemf_v: config.observer_minimum_bemf_v,
            speed_variance_ratio: config.observer_speed_variance_ratio,
            consecutive_samples: config.observer_consecutive_samples as u16,
        },
    );
    controller.observer_counter = 0;
    controller.observer_feedback = RotorFeedback::default();
    controller.observer_reliable = false;
    controller.startup = RevUpSequencer::with_config(params.motor.pole_pairs, startup_config);
    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.algorithm_configured = true;
    controller.state = FocState::Disabled;
    controller.target_speed_rpm = params.default_target_speed_rpm;
    controller.previous_pwm = PwmCommand::default();
    controller.current_reference = CurrentCommand::default();
    controller.speed_current_command = CurrentCommand::default();
    controller.speed_reference_rpm = 0.0;
    controller.speed_counter = 0;
    controller.observer_wait_elapsed_s = 0.0;
    controller.observer_loss_elapsed_s = 0.0;
    controller.closed_loop_initialized = false;
    controller.telemetry = FocTelemetry::default();
}

unsafe fn controller_mut<'a>(context: *mut FocRustContextStorage) -> Option<&'a mut Controller> {
    if context.is_null() {
        return None;
    }

    // SAFETY: the C contract requires aligned, zero/static storage initialized by
    // foc_rust_init before every other call. The size/alignment assertions above
    // guarantee that Controller fits in that storage.
    let controller = unsafe { &mut *context.cast::<Controller>() };
    (controller.magic == CONTEXT_MAGIC).then_some(controller)
}

/// Enables host-only dead-time compensation without changing the target C ABI.
/// Target firmware never calls this function and therefore keeps the disabled
/// default. The setting must be installed after `foc_rust_init`.
#[cfg(not(target_os = "none"))]
pub fn foc_rust_set_host_dead_time_compensation(
    context: &mut FocRustContextStorage,
    compensation: HostDeadTimeCompensation,
) -> FocStatus {
    if !compensation.is_valid() {
        return FocStatus::InvalidArgument;
    }
    // SAFETY: the mutable reference proves exclusive, aligned storage access.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    controller.host_dead_time_compensation = compensation;
    FocStatus::Ok
}

#[no_mangle]
pub extern "C" fn foc_rust_abi_version() -> u32 {
    FOC_RUST_ABI_VERSION
}

#[no_mangle]
pub extern "C" fn foc_rust_context_required_size() -> u32 {
    size_of::<Controller>() as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_context_required_align() -> u32 {
    align_of::<Controller>() as u32
}

#[no_mangle]
/// Writes the editable configuration profile derived from the working ST
/// MCSDK project. Nothing in the realtime path reads global constants after
/// `foc_rust_configure`; the caller owns and may version this value.
///
/// # Safety
/// `config` must be null or point to writable `FocRuntimeConfig` storage.
pub unsafe extern "C" fn foc_rust_default_st_config(config: *mut FocRuntimeConfig) -> FocStatus {
    // SAFETY: the caller promises writable C ABI storage or passes null.
    let Some(config) = (unsafe { config.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *config = default_st_runtime_config();
    FocStatus::Ok
}

#[no_mangle]
/// Applies a complete versioned runtime configuration while PWM is disabled.
///
/// # Safety
/// `context` must be initialized and exclusively accessible; `config` must be
/// null or point to a readable `FocRuntimeConfig`.
pub unsafe extern "C" fn foc_rust_configure(
    context: *mut FocRustContextStorage,
    config: *const FocRuntimeConfig,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(config) = (unsafe { config.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state != FocState::Disabled || !runtime_config_is_valid(config) {
        return FocStatus::InvalidArgument;
    }
    configure_runtime(controller, *config);
    FocStatus::Ok
}

#[no_mangle]
/// Initializes caller-owned controller storage without allocating memory.
///
/// # Safety
/// `context` must be null or point to writable, correctly aligned
/// `FocRustContextStorage`. The caller must provide exclusive access during the call.
pub unsafe extern "C" fn foc_rust_init(context: *mut FocRustContextStorage) -> FocStatus {
    if context.is_null() {
        return FocStatus::InvalidArgument;
    }

    // SAFETY: context is non-null, properly aligned by its C/Rust ABI type, and
    // Controller is compile-time checked to fit in the caller-owned storage.
    unsafe { ptr::write(context.cast::<Controller>(), Controller::new()) };
    FocStatus::Ok
}

#[no_mangle]
/// Copies and validates the parameters used by the Rust `FocBasic` algorithm.
///
/// # Safety
/// `context` must have been initialized by `foc_rust_init` and be exclusively
/// accessible. `config` must be null or point to a readable `FocBasicConfig`.
pub unsafe extern "C" fn foc_rust_configure_basic(
    context: *mut FocRustContextStorage,
    config: *const FocBasicConfig,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    // SAFETY: the caller promises config points to a readable C ABI value.
    let Some(config) = (unsafe { config.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if !config_is_valid(config) {
        controller.algorithm_configured = false;
        controller.stop();
        return FocStatus::InvalidArgument;
    }

    controller.params = st_gbm2804_reference_parameters();
    controller.params.id_pi = to_pi_param(config.id_pi);
    controller.params.iq_pi = to_pi_param(config.iq_pi);
    controller.params.motor.nominal_bus_voltage_v = config.nominal_dc_bus_voltage;
    controller.runtime_config = default_st_runtime_config();
    controller.runtime_config.id_pi = config.id_pi;
    controller.runtime_config.iq_pi = config.iq_pi;
    controller.runtime_config.nominal_bus_voltage_v = config.nominal_dc_bus_voltage;
    controller.observer = ConfigurableObserver::new(
        ObserverBackend::SmoPll,
        controller.params.motor,
        1.0 / controller.params.pwm_frequency_hz as f32,
    );
    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.algorithm_configured = true;
    controller.state = FocState::Disabled;
    FocStatus::Ok
}

#[no_mangle]
/// Selects the parameters and cascaded-loop topology transcribed from the ST
/// MCSDK 6.4.1 reference project.
///
/// # Safety
/// `context` must have been initialized and be exclusively accessible.
pub unsafe extern "C" fn foc_rust_configure_st_reference(
    context: *mut FocRustContextStorage,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    configure_runtime(controller, default_st_runtime_config());
    FocStatus::Ok
}

#[no_mangle]
/// Enters the running state only when both Rust parameters and the C platform are ready.
///
/// # Safety
/// `context` must have been initialized by `foc_rust_init` and be exclusively
/// accessible for the duration of the call.
pub unsafe extern "C" fn foc_rust_request_start(
    context: *mut FocRustContextStorage,
    platform_ready: u32,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured || platform_ready == 0 {
        controller.state = FocState::Disabled;
        return FocStatus::NotConfigured;
    }

    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.state = FocState::Running;
    FocStatus::Ok
}

#[no_mangle]
/// Starts the ISR-owned startup/current-control runtime. The requested speed is
/// kept separate from the configured rev-up endpoint so both can be tuned.
///
/// # Safety
/// `context` must be initialized, exclusively accessible and not concurrently
/// used by the ADC ISR until this function returns.
pub unsafe extern "C" fn foc_rust_start_realtime(
    context: *mut FocRustContextStorage,
    platform_ready: u32,
    target_speed_rpm: f32,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured || platform_ready == 0 {
        controller.state = FocState::Disabled;
        return FocStatus::NotConfigured;
    }
    if !target_speed_rpm.is_finite()
        || target_speed_rpm < controller.runtime_config.startup_final_speed_rpm
        || target_speed_rpm > controller.params.motor.max_speed_rpm
    {
        return FocStatus::InvalidArgument;
    }

    controller.current_loop.reset();
    controller.speed_loop.reset();
    controller.observer.reset(0.0);
    controller.observer_counter = 0;
    controller.observer_feedback = RotorFeedback::default();
    controller.observer_reliable = false;
    controller.startup.reset();
    controller.previous_pwm = PwmCommand::default();
    controller.current_reference = CurrentCommand::default();
    controller.speed_current_command = CurrentCommand::default();
    controller.target_speed_rpm = target_speed_rpm;
    controller.speed_reference_rpm = 0.0;
    controller.speed_counter = 0;
    controller.observer_wait_elapsed_s = 0.0;
    controller.observer_loss_elapsed_s = 0.0;
    controller.closed_loop_initialized = false;
    controller.telemetry = FocTelemetry::default();
    controller.state = FocState::Alignment;
    FocStatus::Ok
}

#[no_mangle]
/// Executes observer, startup, current loop and optional speed loop exactly
/// once. The C ADC interrupt is the sole caller and therefore the sole owner
/// of mutable controller state while the power stage is armed.
///
/// # Safety
/// `context` must be initialized and exclusively owned by the caller;
/// `feedback` must be readable, `output` writable, and optional `telemetry`
/// writable. The pointed-to objects must not overlap.
pub unsafe extern "C" fn foc_rust_realtime_step(
    context: *mut FocRustContextStorage,
    feedback: *const FocFeedback,
    output: *mut FocOutput,
    telemetry: *mut FocTelemetry,
) -> FocStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(feedback) = (unsafe { feedback.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if !matches!(
        controller.state,
        FocState::Alignment
            | FocState::OpenLoopRamp
            | FocState::OpenLoopHold
            | FocState::ObserverTransition
            | FocState::ClosedLoop
    ) {
        return FocStatus::Disabled;
    }
    if !feedback_is_valid(feedback) {
        controller.fault_flags |= FOC_FAULT_INVALID_FEEDBACK;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    let mut snapshot = FeedbackSnapshot {
        currents: PhaseCurrents {
            a: feedback.phase_current_a,
            b: feedback.phase_current_b,
            c: feedback.phase_current_c,
        },
        dc_bus_voltage: feedback.dc_bus_voltage,
        rotor: RotorFeedback::default(),
    };
    let mut math = PlatformMath::default();
    let dt_s = 1.0 / controller.params.pwm_frequency_hz as f32;
    let observer_enabled = controller.runtime_config.observer_enable != 0;
    let observer_due = observer_enabled && controller.observer_counter == 0;
    if observer_due {
        #[cfg(not(target_os = "none"))]
        let observer_pwm = controller.host_dead_time_compensation.observer_pwm(
            controller.previous_pwm,
            snapshot.currents,
            dt_s,
        );
        #[cfg(target_os = "none")]
        let observer_pwm = controller.previous_pwm;
        controller.observer_feedback =
            controller
                .observer
                .update_with_math(&snapshot, observer_pwm, &mut math);
        controller.observer_reliable = controller.observer.is_reliable();
    }
    if observer_enabled {
        controller.observer_counter += 1;
        if controller.observer_counter >= controller.runtime_config.observer_update_divider {
            controller.observer_counter = 0;
        }
    }
    let observer_feedback = controller.observer_feedback;
    let observer_reliable = controller.observer_reliable;
    let handoff_ready = controller.runtime_config.closed_loop_enable != 0 && observer_reliable;
    let observer_iq_a = if handoff_ready
        && !matches!(
            controller.state,
            FocState::ObserverTransition | FocState::ClosedLoop
        ) {
        let current = clarke(Abc {
            a: feedback.phase_current_a,
            b: feedback.phase_current_b,
            c: feedback.phase_current_c,
        });
        let (sin, cos) = math.sin_cos(observer_feedback.electrical_angle_rad);
        -current.alpha * sin + current.beta * cos
    } else {
        controller.current_reference.iq_ref_a
    };
    let startup = controller.startup.update(
        dt_s,
        observer_feedback.electrical_angle_rad,
        handoff_ready,
        observer_iq_a,
    );
    let observer_controls = matches!(
        startup.phase,
        RevUpPhase::ObserverTransition | RevUpPhase::ClosedLoop
    );

    if controller.runtime_config.closed_loop_enable != 0
        && startup.phase == RevUpPhase::OpenLoopHold
    {
        controller.observer_wait_elapsed_s += dt_s;
        if controller.observer_wait_elapsed_s
            >= controller.runtime_config.observer_acquisition_timeout_s
        {
            controller.fault_flags |= FOC_FAULT_OBSERVER_STARTUP;
            controller.state = FocState::Fault;
            return FocStatus::HardwareFault;
        }
    } else {
        controller.observer_wait_elapsed_s = 0.0;
    }

    if observer_controls {
        if observer_reliable {
            controller.observer_loss_elapsed_s = 0.0;
        } else {
            controller.observer_loss_elapsed_s += dt_s;
            if controller.observer_loss_elapsed_s
                >= controller.runtime_config.observer_loss_timeout_s
            {
                controller.fault_flags |= FOC_FAULT_OBSERVER_LOST;
                controller.state = FocState::Fault;
                return FocStatus::HardwareFault;
            }
        }
    } else {
        controller.observer_loss_elapsed_s = 0.0;
    }

    snapshot.rotor = if observer_controls {
        RotorFeedback {
            // During ObserverTransition this is the shortest-path blend from
            // forced angle to observer angle. In ClosedLoop it equals the
            // observer angle. A transient reliability drop keeps this continuous
            // and is handled by the timed observer-loss fault above.
            electrical_angle_rad: startup.angle_for_control_rad,
            mechanical_speed_rad_s: observer_feedback.mechanical_speed_rad_s,
        }
    } else {
        RotorFeedback {
            electrical_angle_rad: startup.forced_electrical_angle_rad,
            mechanical_speed_rad_s: 0.0,
        }
    };

    controller.current_reference = if startup.phase == RevUpPhase::ClosedLoop {
        let divider =
            (controller.params.pwm_frequency_hz / controller.params.speed_loop_frequency_hz).max(1);
        if !controller.closed_loop_initialized {
            controller.speed_reference_rpm = observer_feedback.mechanical_speed_rad_s * 30.0 / PI;
            controller.speed_current_command = controller.speed_loop.preload(
                &controller.params,
                SpeedCommand {
                    target_rpm: controller.speed_reference_rpm,
                    id_ref_a: 0.0,
                },
                observer_feedback.mechanical_speed_rad_s,
                startup.current_reference.iq_ref_a
                    * controller.runtime_config.speed_pi_preload_ratio,
            );
            // Preserve the final switch-over current exactly on the first
            // closed-loop sample. The applied reference then slews toward the
            // speed PI output without a one-tick torque step.
            controller.current_reference = startup.current_reference;
            controller.speed_counter = 0;
            controller.closed_loop_initialized = true;
        } else if controller.speed_counter == 0 {
            let maximum_step = controller.runtime_config.closed_loop_speed_ramp_rpm_per_s
                / controller.params.speed_loop_frequency_hz as f32;
            controller.speed_reference_rpm = move_towards(
                controller.speed_reference_rpm,
                controller.target_speed_rpm,
                maximum_step,
            );
            controller.speed_current_command = controller.speed_loop.update(
                &controller.params,
                SpeedCommand {
                    target_rpm: controller.speed_reference_rpm,
                    id_ref_a: 0.0,
                },
                observer_feedback.mechanical_speed_rad_s,
            );
        }
        controller.speed_counter = (controller.speed_counter + 1) % divider;
        let maximum_current_step =
            controller.runtime_config.closed_loop_current_slew_a_per_s * dt_s;
        controller.current_reference.id_ref_a = move_towards(
            controller.current_reference.id_ref_a,
            controller.speed_current_command.id_ref_a,
            maximum_current_step,
        );
        controller.current_reference.iq_ref_a = move_towards(
            controller.current_reference.iq_ref_a,
            controller.speed_current_command.iq_ref_a,
            maximum_current_step,
        );
        controller.state = FocState::ClosedLoop;
        controller.current_reference
    } else {
        controller.state = match startup.phase {
            RevUpPhase::Alignment => FocState::Alignment,
            RevUpPhase::OpenLoopRamp => FocState::OpenLoopRamp,
            RevUpPhase::OpenLoopHold => FocState::OpenLoopHold,
            RevUpPhase::ObserverTransition => FocState::ObserverTransition,
            RevUpPhase::ClosedLoop => FocState::ClosedLoop,
        };
        startup.current_reference
    };

    // The observer owns one scheduled slot and the power stage holds the last
    // valid PWM command for that single carrier period. This optional
    // multi-rate path is retained for future higher-frequency board profiles;
    // divider=1 executes observer and current controller in the same slot.
    if observer_due && controller.runtime_config.observer_update_divider > 1 {
        let pwm = controller.previous_pwm;
        output.duty_a = pwm.duty_a;
        output.duty_b = pwm.duty_b;
        output.duty_c = pwm.duty_c;
        controller.telemetry.state = controller.state as u32;
        controller.telemetry.observer_backend = controller.observer.backend() as u32;
        controller.telemetry.observer_reliable = observer_reliable as u32;
        controller.telemetry.closed_loop_active = (controller.state == FocState::ClosedLoop) as u32;
        controller.telemetry.target_speed_rpm = controller.target_speed_rpm;
        controller.telemetry.measured_speed_rpm =
            observer_feedback.mechanical_speed_rad_s * 30.0 / PI;
        controller.telemetry.electrical_angle_rad = snapshot.rotor.electrical_angle_rad;
        controller.telemetry.forced_electrical_angle_rad = startup.forced_electrical_angle_rad;
        controller.telemetry.observer_electrical_angle_rad = observer_feedback.electrical_angle_rad;
        controller.telemetry.id_reference_a = controller.current_reference.id_ref_a;
        controller.telemetry.iq_reference_a = controller.current_reference.iq_ref_a;
        if let Some(telemetry) = unsafe { telemetry.as_mut() } {
            *telemetry = controller.telemetry;
        }
        return FocStatus::Ok;
    }

    let (pwm, control) = controller.current_loop.update_with_math(
        &controller.params,
        &snapshot,
        controller.current_reference,
        &mut math,
    );
    if !pwm.is_valid() {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    #[cfg(not(target_os = "none"))]
    let pwm = controller
        .host_dead_time_compensation
        .feedforward_pwm(pwm, snapshot.currents, dt_s);
    if !pwm.is_valid() {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    controller.previous_pwm = pwm;
    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    controller.telemetry = FocTelemetry {
        state: controller.state as u32,
        observer_backend: controller.observer.backend() as u32,
        observer_reliable: observer_reliable as u32,
        closed_loop_active: (controller.state == FocState::ClosedLoop) as u32,
        target_speed_rpm: controller.target_speed_rpm,
        measured_speed_rpm: observer_feedback.mechanical_speed_rad_s * 30.0 / PI,
        electrical_angle_rad: snapshot.rotor.electrical_angle_rad,
        id_reference_a: controller.current_reference.id_ref_a,
        iq_reference_a: controller.current_reference.iq_ref_a,
        id_measured_a: control.current_dq.d,
        iq_measured_a: control.current_dq.q,
        vd_command_v: control.voltage_dq.d,
        vq_command_v: control.voltage_dq.q,
        forced_electrical_angle_rad: startup.forced_electrical_angle_rad,
        observer_electrical_angle_rad: observer_feedback.electrical_angle_rad,
    };
    if let Some(telemetry) = unsafe { telemetry.as_mut() } {
        *telemetry = controller.telemetry;
    }
    FocStatus::Ok
}

#[no_mangle]
/// Copies the last realtime snapshot.
///
/// # Safety
/// `context` must be initialized and not concurrently mutated; `telemetry`
/// must be null or point to writable C ABI storage.
pub unsafe extern "C" fn foc_rust_get_telemetry(
    context: *mut FocRustContextStorage,
    telemetry: *mut FocTelemetry,
) -> FocStatus {
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    let Some(telemetry) = (unsafe { telemetry.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *telemetry = controller.telemetry;
    FocStatus::Ok
}

#[no_mangle]
/// Stops control and resets all algorithm state.
///
/// # Safety
/// `context` must be null or an initialized context with exclusive access.
pub unsafe extern "C" fn foc_rust_stop(context: *mut FocRustContextStorage) {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    if let Some(controller) = unsafe { controller_mut(context) } {
        controller.stop();
    }
}

#[no_mangle]
/// Executes one allocation-free current-control step.
///
/// # Safety
/// `context` must be initialized and exclusively accessible. `feedback` and
/// `reference` must be null or readable, and `output` must be null or writable.
/// No pointed-to object may alias another mutable object during the call.
pub unsafe extern "C" fn foc_rust_fast_step(
    context: *mut FocRustContextStorage,
    feedback: *const FocFeedback,
    reference: *const FocReference,
    output: *mut FocOutput,
) -> FocStatus {
    // SAFETY: output must point to writable C ABI storage.
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);

    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }

    // SAFETY: the caller promises readable C ABI inputs for this call.
    let Some(feedback) = (unsafe { feedback.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    // SAFETY: the caller promises readable C ABI inputs for this call.
    let Some(reference) = (unsafe { reference.as_ref() }) else {
        return FocStatus::InvalidArgument;
    };
    if !feedback_is_valid(feedback) || !reference_is_valid(reference) {
        return FocStatus::InvalidArgument;
    }

    let mut math = PlatformMath::default();
    let (pwm, _) = controller.current_loop.update_with_math(
        &controller.params,
        &FeedbackSnapshot {
            currents: PhaseCurrents {
                a: feedback.phase_current_a,
                b: feedback.phase_current_b,
                c: feedback.phase_current_c,
            },
            dc_bus_voltage: feedback.dc_bus_voltage,
            rotor: RotorFeedback {
                electrical_angle_rad: feedback.electrical_angle_rad,
                mechanical_speed_rad_s: 0.0,
            },
        },
        CurrentCommand {
            id_ref_a: reference.id_ref,
            iq_ref_a: reference.iq_ref,
        },
        &mut math,
    );

    if !pwm.duty_a.is_finite()
        || !pwm.duty_b.is_finite()
        || !pwm.duty_c.is_finite()
        || !(0.0..=1.0).contains(&pwm.duty_a)
        || !(0.0..=1.0).contains(&pwm.duty_b)
        || !(0.0..=1.0).contains(&pwm.duty_c)
    {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    FocStatus::Ok
}

#[no_mangle]
/// Generates one deliberately voltage-limited open-loop vector for board
/// bring-up. C owns arming, current trips, PWM registers and gate enables.
///
/// # Safety
/// Pointer and exclusive-access requirements are identical to
/// `foc_rust_fast_step`.
pub unsafe extern "C" fn foc_rust_open_loop_step(
    context: *mut FocRustContextStorage,
    electrical_speed_rad_s: f32,
    voltage_magnitude_v: f32,
    dc_bus_voltage_v: f32,
    sample_time_s: f32,
    output: *mut FocOutput,
) -> FocStatus {
    let Some(output) = (unsafe { output.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    zero_output(output);

    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }
    if !electrical_speed_rad_s.is_finite()
        || !voltage_magnitude_v.is_finite()
        || !dc_bus_voltage_v.is_finite()
        || !sample_time_s.is_finite()
        || voltage_magnitude_v < 0.0
        || dc_bus_voltage_v <= 0.0
        || voltage_magnitude_v > dc_bus_voltage_v * 0.08
        || !(0.0..=0.01).contains(&sample_time_s)
        || sample_time_s == 0.0
    {
        return FocStatus::InvalidArgument;
    }

    controller.trial_angle_rad =
        wrap_angle_0_to_2pi(controller.trial_angle_rad + electrical_speed_rad_s * sample_time_s);
    let mut math = PlatformMath::default();
    let (sin, cos) = math.sin_cos(controller.trial_angle_rad);
    let pwm = svpwm_update(
        AlphaBeta {
            alpha: voltage_magnitude_v * cos,
            beta: voltage_magnitude_v * sin,
        },
        &SvpwmParam {
            v_bus: dc_bus_voltage_v,
        },
    );
    if !pwm.duty_a.is_finite()
        || !pwm.duty_b.is_finite()
        || !pwm.duty_c.is_finite()
        || !(0.0..=1.0).contains(&pwm.duty_a)
        || !(0.0..=1.0).contains(&pwm.duty_b)
        || !(0.0..=1.0).contains(&pwm.duty_c)
    {
        controller.fault_flags |= FOC_FAULT_ALGORITHM_OUTPUT;
        controller.state = FocState::Fault;
        return FocStatus::HardwareFault;
    }

    output.duty_a = pwm.duty_a;
    output.duty_b = pwm.duty_b;
    output.duty_c = pwm.duty_c;
    FocStatus::Ok
}

#[no_mangle]
/// Executes the 1 kHz-equivalent speed PI and returns Id/Iq references for the
/// fast current loop. C decides when to call it and still owns ISR scheduling.
///
/// # Safety
/// `context` and `reference` follow the same ownership rules as the fast step.
pub unsafe extern "C" fn foc_rust_speed_step(
    context: *mut FocRustContextStorage,
    target_speed_rpm: f32,
    measured_speed_rpm: f32,
    reference: *mut FocReference,
) -> FocStatus {
    // SAFETY: reference must point to writable C ABI storage.
    let Some(reference) = (unsafe { reference.as_mut() }) else {
        return FocStatus::InvalidArgument;
    };
    *reference = FocReference::default();
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    if controller.state == FocState::Fault {
        return FocStatus::HardwareFault;
    }
    if !controller.algorithm_configured {
        return FocStatus::NotConfigured;
    }
    if controller.state != FocState::Running {
        return FocStatus::Disabled;
    }
    if !target_speed_rpm.is_finite() || !measured_speed_rpm.is_finite() {
        return FocStatus::InvalidArgument;
    }
    let current = controller.speed_loop.update(
        &controller.params,
        SpeedCommand {
            target_rpm: target_speed_rpm,
            id_ref_a: 0.0,
        },
        measured_speed_rpm * PI / 30.0,
    );
    reference.id_ref = current.id_ref_a;
    reference.iq_ref = current.iq_ref_a;
    FocStatus::Ok
}

#[no_mangle]
/// Latches fault bits and places the controller in the fault state.
///
/// # Safety
/// `context` must be initialized and exclusively accessible.
pub unsafe extern "C" fn foc_rust_latch_fault(
    context: *mut FocRustContextStorage,
    fault_flags: u32,
) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    controller.fault_flags |= fault_flags;
    controller.state = FocState::Fault;
    FocStatus::HardwareFault
}

#[no_mangle]
/// Clears logical fault state and returns to disabled state.
///
/// # Safety
/// `context` must be initialized and exclusively accessible. Hardware faults
/// must already have been checked and cleared by the C platform layer.
pub unsafe extern "C" fn foc_rust_clear_fault(context: *mut FocRustContextStorage) -> FocStatus {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    let Some(controller) = (unsafe { controller_mut(context) }) else {
        return FocStatus::InvalidArgument;
    };
    controller.fault_flags = 0;
    controller.stop();
    FocStatus::Ok
}

#[no_mangle]
/// Returns the current logical control state.
///
/// # Safety
/// `context` must be null or an initialized context without concurrent mutation.
pub unsafe extern "C" fn foc_rust_state(context: *mut FocRustContextStorage) -> FocState {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    unsafe { controller_mut(context) }
        .map(|controller| controller.state)
        .unwrap_or(FocState::Uninitialized)
}

#[no_mangle]
/// Returns the latched logical fault bits.
///
/// # Safety
/// `context` must be null or an initialized context without concurrent mutation.
pub unsafe extern "C" fn foc_rust_fault_flags(context: *mut FocRustContextStorage) -> u32 {
    // SAFETY: all pointer conversions stay inside this FFI boundary.
    unsafe { controller_mut(context) }
        .map(|controller| controller.fault_flags)
        .unwrap_or(0)
}

#[cfg(all(not(test), target_os = "none"))]
extern "C" {
    fn foc_platform_emergency_stop();
}

#[cfg(all(not(test), target_os = "none"))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    // SAFETY: this is the C platform's non-returning-safe hardware shutdown hook.
    // It is the only hardware-facing call allowed in the Rust bridge.
    unsafe { foc_platform_emergency_stop() };
    loop {
        core::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context() -> FocRustContextStorage {
        FocRustContextStorage {
            bytes: [0; FOC_RUST_CONTEXT_CAPACITY],
        }
    }

    fn config() -> FocBasicConfig {
        let pi = FocPiConfig {
            kp: 1.0,
            ki: 0.0,
            ts: 0.000_1,
            out_min: -12.0,
            out_max: 12.0,
            integrator_min: -6.0,
            integrator_max: 6.0,
        };
        FocBasicConfig {
            id_pi: pi,
            iq_pi: pi,
            nominal_dc_bus_voltage: 24.0,
        }
    }

    #[test]
    fn starts_safe_and_requires_both_gates() {
        let mut context = context();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_state(&mut context), FocState::Disabled);
            assert_eq!(
                foc_rust_request_start(&mut context, 1),
                FocStatus::NotConfigured
            );
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            assert_eq!(
                foc_rust_request_start(&mut context, 0),
                FocStatus::NotConfigured
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(foc_rust_state(&mut context), FocState::Running);
        }
    }

    #[test]
    fn executes_imported_foc_basic_algorithm() {
        let mut context = context();
        let feedback = FocFeedback {
            dc_bus_voltage: 24.0,
            ..FocFeedback::default()
        };
        let reference = FocReference {
            id_ref: 0.0,
            iq_ref: 2.0,
        };
        let mut output = FocOutput::default();

        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::Ok
            );
        }

        assert!((0.0..=1.0).contains(&output.duty_a));
        assert!((0.0..=1.0).contains(&output.duty_b));
        assert!((0.0..=1.0).contains(&output.duty_c));
        assert_ne!(output.duty_a, output.duty_b);
    }

    #[test]
    fn st_reference_config_exposes_speed_and_current_loops() {
        let mut context = context();
        let mut reference = FocReference::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_configure_st_reference(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_speed_step(&mut context, 524.0, 0.0, &mut reference),
                FocStatus::Ok
            );
        }
        assert_eq!(reference.id_ref, 0.0);
        assert!(reference.iq_ref > 0.0 && reference.iq_ref <= 0.8);
    }

    #[test]
    fn stop_and_fault_paths_always_zero_output() {
        let mut context = context();
        let feedback = FocFeedback {
            dc_bus_voltage: 24.0,
            ..FocFeedback::default()
        };
        let reference = FocReference::default();
        let mut output = FocOutput {
            duty_a: 1.0,
            duty_b: 1.0,
            duty_c: 1.0,
        };

        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_configure_basic(&mut context, &config()),
                FocStatus::Ok
            );
            foc_rust_stop(&mut context);
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::Disabled
            );
            assert_eq!(output.duty_a, 0.0);
            assert_eq!(
                foc_rust_latch_fault(&mut context, 0x10),
                FocStatus::HardwareFault
            );
            assert_eq!(
                foc_rust_fast_step(&mut context, &feedback, &reference, &mut output),
                FocStatus::HardwareFault
            );
            assert_eq!(output.duty_b, 0.0);
            assert_eq!(foc_rust_fault_flags(&mut context), 0x10);
        }
    }

    #[test]
    fn open_loop_trial_is_running_gated_and_voltage_limited() {
        let mut context = context();
        let mut output = FocOutput::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_configure_st_reference(&mut context), FocStatus::Ok);
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 1.0, 0.5, 13.0, 0.001, &mut output),
                FocStatus::Disabled
            );
            assert_eq!(foc_rust_request_start(&mut context, 1), FocStatus::Ok);
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 10.0, 0.5, 13.0, 0.001, &mut output),
                FocStatus::Ok
            );
            assert!((0.44..=0.56).contains(&output.duty_a));
            assert_eq!(
                foc_rust_open_loop_step(&mut context, 10.0, 2.0, 13.0, 0.001, &mut output),
                FocStatus::InvalidArgument
            );
            assert_eq!(output.duty_a, 0.0);
        }
    }

    #[test]
    fn realtime_path_uses_versioned_smo_configuration_and_startup_gate() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            assert_eq!(runtime.observer_backend, ObserverBackend::SmoPll as u32);
            assert_eq!(runtime.observer_enable, 1);
            assert_eq!(runtime.observer_update_divider, 1);
            assert_eq!(runtime.closed_loop_enable, 0);
            assert!((runtime.observer_smo_k_slide_v - 4.0).abs() < 1e-6);
            assert!((runtime.observer_smo_boundary_a - 0.16).abs() < 1e-6);
            assert!((runtime.observer_pll_kp - 80.0).abs() < 1e-6);
            assert!((runtime.observer_pll_ki - 1_000.0).abs() < 1e-6);
            assert!((runtime.observer_minimum_speed_rpm - 524.0).abs() < 1e-6);
            assert_eq!(runtime.observer_consecutive_samples, 2);
            assert_eq!(runtime.speed_pi_preload_ratio, 0.0);
            assert!((runtime.closed_loop_current_slew_a_per_s - 32.0).abs() < 1e-6);
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
            for _ in 0..300 {
                assert_eq!(
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry,),
                    FocStatus::Ok
                );
                assert!(output.duty_a.is_finite());
                assert!(output.duty_b.is_finite());
                assert!(output.duty_c.is_finite());
            }
        }
        assert_eq!(telemetry.state, FocState::Alignment as u32);
        assert_eq!(telemetry.observer_backend, ObserverBackend::SmoPll as u32);
        assert_eq!(telemetry.closed_loop_active, 0);
        assert!(
            (output.duty_a - 0.5).abs() > 1e-6
                || (output.duty_b - 0.5).abs() > 1e-6
                || (output.duty_c - 0.5).abs() > 1e-6
        );
    }

    #[test]
    fn closed_loop_request_faults_when_observer_never_converges() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.closed_loop_enable = 1;
            runtime.alignment_duration_s = 0.001;
            runtime.open_loop_ramp_duration_s = 0.001;
            runtime.observer_acquisition_timeout_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );
            let mut status = FocStatus::Ok;
            for _ in 0..100 {
                status =
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry);
                if status != FocStatus::Ok {
                    break;
                }
            }
            assert_eq!(status, FocStatus::HardwareFault);
            assert_eq!(foc_rust_state(&mut context), FocState::Fault);
            assert_ne!(
                foc_rust_fault_flags(&mut context) & FOC_FAULT_OBSERVER_STARTUP,
                0
            );
            assert_eq!(output.duty_a, 0.0);
        }
    }

    #[test]
    fn closed_loop_latches_fault_after_observer_loss_timeout() {
        let mut context = context();
        let mut runtime = FocRuntimeConfig::default();
        let feedback = FocFeedback {
            dc_bus_voltage: 13.0,
            ..FocFeedback::default()
        };
        let mut output = FocOutput::default();
        let mut telemetry = FocTelemetry::default();
        unsafe {
            assert_eq!(foc_rust_init(&mut context), FocStatus::Ok);
            assert_eq!(foc_rust_default_st_config(&mut runtime), FocStatus::Ok);
            runtime.closed_loop_enable = 1;
            runtime.observer_loss_timeout_s = 0.001;
            assert_eq!(foc_rust_configure(&mut context, &runtime), FocStatus::Ok);
            assert_eq!(
                foc_rust_start_realtime(&mut context, 1, runtime.startup_final_speed_rpm),
                FocStatus::Ok
            );

            let controller = controller_mut(&mut context).unwrap();
            let _ = controller.startup.update(2.05, 0.0, true, 0.2);
            let _ = controller.startup.update(0.026, 0.0, false, 0.0);
            controller.state = FocState::ClosedLoop;
            controller.observer_reliable = false;

            let mut status = FocStatus::Ok;
            for _ in 0..32 {
                status =
                    foc_rust_realtime_step(&mut context, &feedback, &mut output, &mut telemetry);
                if status != FocStatus::Ok {
                    break;
                }
            }
            assert_eq!(status, FocStatus::HardwareFault);
            assert_ne!(
                foc_rust_fault_flags(&mut context) & FOC_FAULT_OBSERVER_LOST,
                0
            );
            assert_eq!(output.duty_b, 0.0);
        }
    }
}
