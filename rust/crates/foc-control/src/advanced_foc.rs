//! Advanced-FOC operating-region supervisor.
//!
//! This module turns the existing leaf algorithms into one deterministic policy
//! owner.  It does not touch PWM, ADC, an RTOS, or global state.  The target ISR
//! and both simulators consume the same [`AdvancedFocOutput`].  Every feature is
//! opt-in; a disabled configuration is an exact pass-through for the basic FOC
//! current reference.

use core::mem::size_of;

use foc_algorithm::{
    AlphaBeta, DpwmMode, Dq, HighPassParam, HighPassState, MtpaParam, MtpaState, MtpvParam,
    MtpvState, RotatingHfParam, RotatingHfState, WeakeningParam, WeakeningState,
};

use crate::{CurrentCommand, CurrentLoopModulation, CurrentLoopPolicy, MotorParameters};

pub const ADVANCED_FOC_CONFIG_VERSION: u32 = 1;

pub const ADV_FOC_MTPA: u32 = 1 << 0;
pub const ADV_FOC_FIELD_WEAKENING: u32 = 1 << 1;
pub const ADV_FOC_MTPV: u32 = 1 << 2;
pub const ADV_FOC_DECOUPLING: u32 = 1 << 3;
pub const ADV_FOC_DPWM: u32 = 1 << 4;
pub const ADV_FOC_OVERMODULATION: u32 = 1 << 5;
pub const ADV_FOC_HFI: u32 = 1 << 6;
pub const ADV_FOC_FLYING_START: u32 = 1 << 7;
pub const ADV_FOC_KNOWN_MASK: u32 = ADV_FOC_MTPA
    | ADV_FOC_FIELD_WEAKENING
    | ADV_FOC_MTPV
    | ADV_FOC_DECOUPLING
    | ADV_FOC_DPWM
    | ADV_FOC_OVERMODULATION
    | ADV_FOC_HFI
    | ADV_FOC_FLYING_START;

const SQRT_3: f32 = 1.732_050_8;
const SVPWM_LINEAR_VBUS_RATIO: f32 = 1.0 / SQRT_3;
const SIX_STEP_FUNDAMENTAL_VBUS_RATIO: f32 = 0.636_619_75; // 2 / pi.

/// Mutually exclusive current-operating region selected by the supervisor.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AdvancedFocRegion {
    #[default]
    Basic = 0,
    Mtpa = 1,
    FieldWeakening = 2,
    Mtpv = 3,
}

/// Final modulation policy requested from the current-loop output stage.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AdvancedModulationMode {
    #[default]
    Svpwm = 0,
    DpwmClampMax = 1,
    DpwmClampMin = 2,
    Overmodulation = 3,
}

/// Passive flying-start scanner state.  It never energises the bridge itself.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum FlyingStartState {
    #[default]
    Idle = 0,
    Scanning = 1,
    Captured = 2,
    Failed = 3,
}

/// Versioned, fixed-layout advanced-FOC configuration.
///
/// `enabled_features == 0` is the production-safe default.  Parameters belonging
/// to disabled features may remain zero; parameters of every enabled feature are
/// validated as a group before the supervisor state is changed.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdvancedFocConfig {
    pub struct_size: u32,
    pub version: u32,
    pub enabled_features: u32,
    /// MTPA/FW/MTPV execute once per N current-loop samples and hold their output.
    pub region_update_divider: u32,
    pub current_limit_a: f32,
    pub mtpa_min_current_a: f32,
    pub mtpa_search_steps: u32,
    pub weakening_entry_utilization: f32,
    pub weakening_exit_utilization: f32,
    pub weakening_kp_a_per_v: f32,
    pub weakening_id_min_a: f32,
    pub weakening_slew_a_per_s: f32,
    pub mtpv_entry_electrical_speed_rad_s: f32,
    pub mtpv_exit_electrical_speed_rad_s: f32,
    pub mtpv_search_steps: u32,
    pub decoupling_gain: f32,
    pub dpwm_entry_modulation: f32,
    pub dpwm_exit_modulation: f32,
    /// Raw value of [`DpwmMode`], kept integer for the future C ABI.
    pub dpwm_mode: u32,
    /// Maximum phase-voltage magnitude divided by Vbus, in `(1/sqrt(3), 2/pi]`.
    pub overmodulation_max_voltage_ratio: f32,
    pub overmodulation_entry_modulation: f32,
    pub overmodulation_exit_modulation: f32,
    pub hfi_amplitude_v: f32,
    pub hfi_frequency_hz: f32,
    pub hfi_demod_alpha: f32,
    pub hfi_high_pass_alpha: f32,
    pub hfi_min_response_a: f32,
    pub hfi_max_electrical_speed_rad_s: f32,
    pub hfi_settling_samples: u32,
    pub flying_start_min_electrical_speed_rad_s: f32,
    pub flying_start_stable_samples: u32,
    pub flying_start_timeout_samples: u32,
}

impl Default for AdvancedFocConfig {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: ADVANCED_FOC_CONFIG_VERSION,
            enabled_features: 0,
            region_update_divider: 1,
            current_limit_a: 0.0,
            mtpa_min_current_a: 0.0,
            mtpa_search_steps: 0,
            weakening_entry_utilization: 0.0,
            weakening_exit_utilization: 0.0,
            weakening_kp_a_per_v: 0.0,
            weakening_id_min_a: 0.0,
            weakening_slew_a_per_s: 0.0,
            mtpv_entry_electrical_speed_rad_s: 0.0,
            mtpv_exit_electrical_speed_rad_s: 0.0,
            mtpv_search_steps: 0,
            decoupling_gain: 0.0,
            dpwm_entry_modulation: 0.0,
            dpwm_exit_modulation: 0.0,
            dpwm_mode: DpwmMode::ClampMax as u32,
            overmodulation_max_voltage_ratio: 0.0,
            overmodulation_entry_modulation: 0.0,
            overmodulation_exit_modulation: 0.0,
            hfi_amplitude_v: 0.0,
            hfi_frequency_hz: 0.0,
            hfi_demod_alpha: 0.0,
            hfi_high_pass_alpha: 0.0,
            hfi_min_response_a: 0.0,
            hfi_max_electrical_speed_rad_s: 0.0,
            hfi_settling_samples: 0,
            flying_start_min_electrical_speed_rad_s: 0.0,
            flying_start_stable_samples: 0,
            flying_start_timeout_samples: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdvancedFocError {
    NotConfigured,
    InvalidIdentity,
    UnknownFeature,
    InvalidCommonConfig,
    InvalidMtpaConfig,
    InvalidWeakeningConfig,
    InvalidMtpvConfig,
    InvalidDecouplingConfig,
    InvalidModulationConfig,
    InvalidHfiConfig,
    MotorHasInsufficientSaliency,
    InvalidFlyingStartConfig,
    InvalidInput,
}

/// Per-tick inputs.  HFI voltage and flying-start capture each require an
/// explicit request, so merely enabling the feature in configuration cannot
/// energise or take over anything.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdvancedFocInput {
    pub base_reference: CurrentCommand,
    pub measured_current_dq: Dq,
    pub current_alpha_beta: AlphaBeta,
    pub previous_voltage_dq: Dq,
    pub estimated_electrical_angle_rad: f32,
    pub estimated_electrical_speed_rad_s: f32,
    pub dc_bus_voltage_v: f32,
    pub linear_voltage_utilization: f32,
    pub closed_loop_active: bool,
    pub observer_reliable: bool,
    pub allow_voltage_injection: bool,
    pub request_flying_start: bool,
}

/// One deterministic decision shared by firmware and simulation.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdvancedFocOutput {
    pub current_reference: CurrentCommand,
    pub voltage_feedforward_dq: Dq,
    pub injection_voltage_alpha_beta: AlphaBeta,
    pub voltage_limit_v: f32,
    pub region: AdvancedFocRegion,
    pub modulation_mode: AdvancedModulationMode,
    pub active_features: u32,
    pub current_limited: bool,
    pub hfi_angle_candidate_rad: f32,
    pub hfi_angle_candidate_valid: bool,
    pub flying_start_state: FlyingStartState,
    pub flying_start_angle_rad: f32,
    pub flying_start_speed_rad_s: f32,
}

impl AdvancedFocOutput {
    /// Maps the supervisor decision into the current loop without exposing any
    /// hardware type.  Overmodulation uses the SVPWM mapper with a deliberately
    /// enlarged voltage circle; DPWM selects its dedicated zero-sequence policy.
    pub fn current_loop_policy(&self) -> CurrentLoopPolicy {
        CurrentLoopPolicy {
            voltage_feedforward_dq: self.voltage_feedforward_dq,
            injection_voltage_alpha_beta: self.injection_voltage_alpha_beta,
            voltage_limit_v: self.voltage_limit_v,
            modulation: match self.modulation_mode {
                AdvancedModulationMode::Svpwm | AdvancedModulationMode::Overmodulation => {
                    CurrentLoopModulation::Svpwm
                }
                AdvancedModulationMode::DpwmClampMax => CurrentLoopModulation::DpwmClampMax,
                AdvancedModulationMode::DpwmClampMin => CurrentLoopModulation::DpwmClampMin,
            },
            vector_anti_windup: self.active_features
                & (ADV_FOC_FIELD_WEAKENING
                    | ADV_FOC_MTPV
                    | ADV_FOC_DECOUPLING
                    | ADV_FOC_HFI
                    | ADV_FOC_OVERMODULATION)
                != 0,
        }
    }
}

/// Fixed-memory owner of all advanced-FOC states.
#[derive(Clone, Copy, Debug)]
pub struct AdvancedFocSupervisor {
    config: AdvancedFocConfig,
    configured: bool,
    sample_period_s: f32,
    region_counter: u32,
    held_reference: CurrentCommand,
    region: AdvancedFocRegion,
    weakening_active: bool,
    mtpv_active: bool,
    dpwm_active: bool,
    overmodulation_active: bool,
    mtpa: MtpaState,
    mtpv: MtpvState,
    weakening: WeakeningState,
    hfi: RotatingHfState,
    hfi_alpha_hpf: HighPassState,
    hfi_beta_hpf: HighPassState,
    hfi_samples: u32,
    flying_start_state: FlyingStartState,
    flying_start_stable: u32,
    flying_start_elapsed: u32,
    flying_start_angle_rad: f32,
    flying_start_speed_rad_s: f32,
}

impl Default for AdvancedFocSupervisor {
    fn default() -> Self {
        Self {
            config: AdvancedFocConfig::default(),
            configured: false,
            sample_period_s: 0.0,
            region_counter: 0,
            held_reference: CurrentCommand::default(),
            region: AdvancedFocRegion::Basic,
            weakening_active: false,
            mtpv_active: false,
            dpwm_active: false,
            overmodulation_active: false,
            mtpa: MtpaState::default(),
            mtpv: MtpvState::default(),
            weakening: WeakeningState::default(),
            hfi: RotatingHfState::default(),
            hfi_alpha_hpf: HighPassState::default(),
            hfi_beta_hpf: HighPassState::default(),
            hfi_samples: 0,
            flying_start_state: FlyingStartState::Idle,
            flying_start_stable: 0,
            flying_start_elapsed: 0,
            flying_start_angle_rad: 0.0,
            flying_start_speed_rad_s: 0.0,
        }
    }
}

impl AdvancedFocConfig {
    pub fn validate(
        &self,
        motor: &MotorParameters,
        control_frequency_hz: u32,
    ) -> Result<(), AdvancedFocError> {
        if self.struct_size != size_of::<Self>() as u32
            || self.version != ADVANCED_FOC_CONFIG_VERSION
        {
            return Err(AdvancedFocError::InvalidIdentity);
        }
        if self.enabled_features & !ADV_FOC_KNOWN_MASK != 0 {
            return Err(AdvancedFocError::UnknownFeature);
        }
        if control_frequency_hz == 0
            || self.region_update_divider == 0
            || self.region_update_divider > control_frequency_hz
            || !motor_is_valid(motor)
        {
            return Err(AdvancedFocError::InvalidCommonConfig);
        }
        if self.enabled_features == 0 {
            return Ok(());
        }
        if !finite_positive(self.current_limit_a) || self.current_limit_a > motor.rated_current_a {
            return Err(AdvancedFocError::InvalidCommonConfig);
        }
        if self.enabled_features & ADV_FOC_MTPA != 0
            && (!finite_nonnegative(self.mtpa_min_current_a)
                || self.mtpa_min_current_a > self.current_limit_a
                || !(8..=64).contains(&self.mtpa_search_steps))
        {
            return Err(AdvancedFocError::InvalidMtpaConfig);
        }
        if self.enabled_features & ADV_FOC_FIELD_WEAKENING != 0
            && (!(0.5..=1.0).contains(&self.weakening_entry_utilization)
                || !(0.0..self.weakening_entry_utilization)
                    .contains(&self.weakening_exit_utilization)
                || !finite_positive(self.weakening_kp_a_per_v)
                || !self.weakening_id_min_a.is_finite()
                || self.weakening_id_min_a >= 0.0
                || self.weakening_id_min_a < -self.current_limit_a
                || !finite_positive(self.weakening_slew_a_per_s))
        {
            return Err(AdvancedFocError::InvalidWeakeningConfig);
        }
        if self.enabled_features & ADV_FOC_MTPV != 0
            && (!finite_positive(self.mtpv_entry_electrical_speed_rad_s)
                || !finite_nonnegative(self.mtpv_exit_electrical_speed_rad_s)
                || self.mtpv_exit_electrical_speed_rad_s >= self.mtpv_entry_electrical_speed_rad_s
                || !(8..=64).contains(&self.mtpv_search_steps))
        {
            return Err(AdvancedFocError::InvalidMtpvConfig);
        }
        if self.enabled_features & ADV_FOC_DECOUPLING != 0
            && (!self.decoupling_gain.is_finite() || !(0.0..=1.5).contains(&self.decoupling_gain))
        {
            return Err(AdvancedFocError::InvalidDecouplingConfig);
        }
        if self.enabled_features & ADV_FOC_DPWM != 0
            && (!(0.0..=1.0).contains(&self.dpwm_entry_modulation)
                || !(0.0..self.dpwm_entry_modulation).contains(&self.dpwm_exit_modulation)
                || !matches!(
                    self.dpwm_mode,
                    value if value == DpwmMode::ClampMax as u32
                        || value == DpwmMode::ClampMin as u32
                ))
        {
            return Err(AdvancedFocError::InvalidModulationConfig);
        }
        if self.enabled_features & ADV_FOC_OVERMODULATION != 0
            && (!self.overmodulation_max_voltage_ratio.is_finite()
                || self.overmodulation_max_voltage_ratio <= SVPWM_LINEAR_VBUS_RATIO
                || self.overmodulation_max_voltage_ratio > SIX_STEP_FUNDAMENTAL_VBUS_RATIO
                || !(1.0..=1.15).contains(&self.overmodulation_entry_modulation)
                || !(0.9..self.overmodulation_entry_modulation)
                    .contains(&self.overmodulation_exit_modulation))
        {
            return Err(AdvancedFocError::InvalidModulationConfig);
        }
        if self.enabled_features & ADV_FOC_HFI != 0 {
            let nyquist_hz = control_frequency_hz as f32 * 0.5;
            if !finite_positive(self.hfi_amplitude_v)
                || !finite_positive(self.hfi_frequency_hz)
                || self.hfi_frequency_hz >= nyquist_hz
                || !(0.0..=1.0).contains(&self.hfi_demod_alpha)
                || self.hfi_demod_alpha == 0.0
                || !(0.0..=1.0).contains(&self.hfi_high_pass_alpha)
                || self.hfi_high_pass_alpha == 0.0
                || !finite_positive(self.hfi_min_response_a)
                || !finite_nonnegative(self.hfi_max_electrical_speed_rad_s)
                || self.hfi_settling_samples == 0
            {
                return Err(AdvancedFocError::InvalidHfiConfig);
            }
            let saliency_ratio = (motor.ld_h - motor.lq_h).abs() / motor.ld_h.max(motor.lq_h);
            if saliency_ratio < 0.02 {
                return Err(AdvancedFocError::MotorHasInsufficientSaliency);
            }
        }
        if self.enabled_features & ADV_FOC_FLYING_START != 0
            && (!finite_nonnegative(self.flying_start_min_electrical_speed_rad_s)
                || self.flying_start_stable_samples == 0
                || self.flying_start_timeout_samples < self.flying_start_stable_samples)
        {
            return Err(AdvancedFocError::InvalidFlyingStartConfig);
        }
        Ok(())
    }
}

impl AdvancedFocSupervisor {
    pub fn configure(
        &mut self,
        config: AdvancedFocConfig,
        motor: &MotorParameters,
        control_frequency_hz: u32,
    ) -> Result<(), AdvancedFocError> {
        config.validate(motor, control_frequency_hz)?;
        let next = Self {
            config,
            configured: true,
            sample_period_s: 1.0 / control_frequency_hz as f32,
            ..Self::default()
        };
        *self = next;
        Ok(())
    }

    pub fn reset(&mut self) {
        let config = self.config;
        let sample_period_s = self.sample_period_s;
        let configured = self.configured;
        *self = Self::default();
        self.config = config;
        self.sample_period_s = sample_period_s;
        self.configured = configured;
    }

    pub fn config(&self) -> AdvancedFocConfig {
        self.config
    }

    pub fn step(
        &mut self,
        motor: &MotorParameters,
        input: AdvancedFocInput,
    ) -> Result<AdvancedFocOutput, AdvancedFocError> {
        if !self.configured {
            return Err(AdvancedFocError::NotConfigured);
        }
        if !input_is_valid(&input) || !motor_is_valid(motor) {
            return Err(AdvancedFocError::InvalidInput);
        }

        let mut output = AdvancedFocOutput {
            current_reference: input.base_reference,
            voltage_limit_v: input.linear_voltage_utilization * input.dc_bus_voltage_v / SQRT_3,
            flying_start_state: self.flying_start_state,
            ..AdvancedFocOutput::default()
        };
        if self.config.enabled_features == 0 {
            return Ok(output);
        }

        if input.closed_loop_active {
            self.update_operating_region(motor, &input);
            output.current_reference = self.held_reference;
            output.region = self.region;
            output.active_features |= match self.region {
                AdvancedFocRegion::Basic => 0,
                AdvancedFocRegion::Mtpa => ADV_FOC_MTPA,
                AdvancedFocRegion::FieldWeakening => ADV_FOC_FIELD_WEAKENING,
                AdvancedFocRegion::Mtpv => ADV_FOC_MTPV,
            };
        } else {
            self.held_reference = input.base_reference;
            self.region = AdvancedFocRegion::Basic;
            self.weakening_active = false;
            self.mtpv_active = false;
            self.weakening.reset();
        }
        output.current_limited =
            current_magnitude(input.base_reference) > self.config.current_limit_a + f32::EPSILON;

        if self.config.enabled_features & ADV_FOC_DECOUPLING != 0 && input.closed_loop_active {
            let omega = input.estimated_electrical_speed_rad_s;
            let gain = self.config.decoupling_gain;
            output.voltage_feedforward_dq = Dq {
                d: -gain * omega * motor.lq_h * input.measured_current_dq.q,
                q: gain
                    * omega
                    * (motor.ld_h * input.measured_current_dq.d + motor.flux_linkage_wb),
            };
            output.active_features |= ADV_FOC_DECOUPLING;
        }

        let prior_modulation = voltage_magnitude(input.previous_voltage_dq)
            / (input.dc_bus_voltage_v / SQRT_3).max(f32::MIN_POSITIVE);
        if self.config.enabled_features & ADV_FOC_OVERMODULATION != 0 {
            if self.overmodulation_active {
                if prior_modulation <= self.config.overmodulation_exit_modulation {
                    self.overmodulation_active = false;
                }
            } else if prior_modulation >= self.config.overmodulation_entry_modulation {
                self.overmodulation_active = true;
            }
        } else {
            self.overmodulation_active = false;
        }
        if self.overmodulation_active {
            output.modulation_mode = AdvancedModulationMode::Overmodulation;
            output.voltage_limit_v =
                input.dc_bus_voltage_v * self.config.overmodulation_max_voltage_ratio;
            output.active_features |= ADV_FOC_OVERMODULATION;
        } else {
            if self.config.enabled_features & ADV_FOC_DPWM != 0 {
                if self.dpwm_active {
                    if prior_modulation <= self.config.dpwm_exit_modulation {
                        self.dpwm_active = false;
                    }
                } else if prior_modulation >= self.config.dpwm_entry_modulation {
                    self.dpwm_active = true;
                }
            } else {
                self.dpwm_active = false;
            }
        }
        if !self.overmodulation_active && self.dpwm_active {
            output.modulation_mode = if self.config.dpwm_mode == DpwmMode::ClampMin as u32 {
                AdvancedModulationMode::DpwmClampMin
            } else {
                AdvancedModulationMode::DpwmClampMax
            };
            output.active_features |= ADV_FOC_DPWM;
        }

        self.update_hfi(&input, &mut output);
        self.update_flying_start(&input, &mut output);
        Ok(output)
    }

    fn update_operating_region(&mut self, motor: &MotorParameters, input: &AdvancedFocInput) {
        let due = self.region_counter == 0;
        self.region_counter += 1;
        if self.region_counter >= self.config.region_update_divider {
            self.region_counter = 0;
        }
        if !due {
            return;
        }

        let mut reference = limit_current(input.base_reference, self.config.current_limit_a);
        let demand = current_magnitude(reference);
        self.region = AdvancedFocRegion::Basic;

        if self.config.enabled_features & ADV_FOC_MTPA != 0
            && demand >= self.config.mtpa_min_current_a
        {
            let dq = self.mtpa.update(
                &MtpaParam {
                    flux_pm: motor.flux_linkage_wb,
                    ld: motor.ld_h,
                    lq: motor.lq_h,
                    search_steps: self.config.mtpa_search_steps as i32,
                },
                demand.copysign(reference.iq_ref_a),
            );
            reference = CurrentCommand {
                id_ref_a: dq.d,
                iq_ref_a: dq.q,
            };
            self.region = AdvancedFocRegion::Mtpa;
        }

        let voltage_ratio = voltage_magnitude(input.previous_voltage_dq)
            / (input.dc_bus_voltage_v / SQRT_3).max(f32::MIN_POSITIVE);
        if self.config.enabled_features & ADV_FOC_FIELD_WEAKENING != 0 {
            if self.weakening_active {
                if voltage_ratio <= self.config.weakening_exit_utilization {
                    self.weakening_active = false;
                    self.weakening.reset();
                }
            } else if voltage_ratio >= self.config.weakening_entry_utilization {
                self.weakening_active = true;
            }
            if self.weakening_active {
                let target_id = self.weakening.update(
                    &WeakeningParam {
                        kp: self.config.weakening_kp_a_per_v,
                        voltage_limit: self.config.weakening_entry_utilization
                            * input.dc_bus_voltage_v
                            / SQRT_3,
                        id_min: self.config.weakening_id_min_a,
                        id_max: 0.0,
                    },
                    input.previous_voltage_dq,
                    reference.id_ref_a,
                );
                let maximum_step = self.config.weakening_slew_a_per_s
                    * self.sample_period_s
                    * self.config.region_update_divider as f32;
                reference.id_ref_a = move_towards(reference.id_ref_a, target_id, maximum_step);
                self.region = AdvancedFocRegion::FieldWeakening;
            }
        }

        if self.config.enabled_features & ADV_FOC_MTPV != 0 {
            let speed = input.estimated_electrical_speed_rad_s.abs();
            if self.mtpv_active {
                if speed <= self.config.mtpv_exit_electrical_speed_rad_s {
                    self.mtpv_active = false;
                    self.mtpv.reset();
                }
            } else if speed >= self.config.mtpv_entry_electrical_speed_rad_s {
                self.mtpv_active = true;
            }
        } else {
            self.mtpv_active = false;
        }
        if self.mtpv_active && demand > 0.0 {
            let dq = self.mtpv.update(
                &MtpvParam {
                    flux_pm: motor.flux_linkage_wb,
                    ld: motor.ld_h,
                    lq: motor.lq_h,
                    current_limit: demand.min(self.config.current_limit_a),
                    voltage_limit: input.linear_voltage_utilization * input.dc_bus_voltage_v
                        / SQRT_3,
                    search_steps: self.config.mtpv_search_steps as i32,
                },
                input.estimated_electrical_speed_rad_s,
                reference.iq_ref_a,
            );
            if self.mtpv.valid != 0 {
                reference = CurrentCommand {
                    id_ref_a: dq.d,
                    iq_ref_a: dq.q,
                };
                self.region = AdvancedFocRegion::Mtpv;
            }
        }
        self.held_reference = limit_current(reference, self.config.current_limit_a);
    }

    fn update_hfi(&mut self, input: &AdvancedFocInput, output: &mut AdvancedFocOutput) {
        let active = self.config.enabled_features & ADV_FOC_HFI != 0
            && input.allow_voltage_injection
            && input.estimated_electrical_speed_rad_s.abs()
                <= self.config.hfi_max_electrical_speed_rad_s;
        if !active {
            self.hfi.reset();
            self.hfi_alpha_hpf.reset(input.current_alpha_beta.alpha);
            self.hfi_beta_hpf.reset(input.current_alpha_beta.beta);
            self.hfi_samples = 0;
            return;
        }
        let high_pass = HighPassParam {
            alpha: self.config.hfi_high_pass_alpha,
        };
        let response = AlphaBeta {
            alpha: self
                .hfi_alpha_hpf
                .update(&high_pass, input.current_alpha_beta.alpha),
            beta: self
                .hfi_beta_hpf
                .update(&high_pass, input.current_alpha_beta.beta),
        };
        output.injection_voltage_alpha_beta = self.hfi.update(
            &RotatingHfParam {
                amplitude: self.config.hfi_amplitude_v,
                freq_hz: self.config.hfi_frequency_hz,
                ts: self.sample_period_s,
                demod_alpha: self.config.hfi_demod_alpha,
            },
            response,
        );
        self.hfi_samples = self.hfi_samples.saturating_add(1);
        output.hfi_angle_candidate_rad = self.hfi.theta_est_rad;
        output.hfi_angle_candidate_valid = self.hfi_samples >= self.config.hfi_settling_samples
            && alpha_beta_magnitude(self.hfi.demod) >= self.config.hfi_min_response_a;
        output.active_features |= ADV_FOC_HFI;
    }

    fn update_flying_start(&mut self, input: &AdvancedFocInput, output: &mut AdvancedFocOutput) {
        if self.config.enabled_features & ADV_FOC_FLYING_START == 0 || !input.request_flying_start {
            self.flying_start_state = FlyingStartState::Idle;
            self.flying_start_stable = 0;
            self.flying_start_elapsed = 0;
        } else if self.flying_start_state != FlyingStartState::Captured
            && self.flying_start_state != FlyingStartState::Failed
        {
            self.flying_start_state = FlyingStartState::Scanning;
            self.flying_start_elapsed = self.flying_start_elapsed.saturating_add(1);
            if input.observer_reliable
                && input.estimated_electrical_speed_rad_s.abs()
                    >= self.config.flying_start_min_electrical_speed_rad_s
            {
                self.flying_start_stable = self.flying_start_stable.saturating_add(1);
                self.flying_start_angle_rad = input.estimated_electrical_angle_rad;
                self.flying_start_speed_rad_s = input.estimated_electrical_speed_rad_s;
                if self.flying_start_stable >= self.config.flying_start_stable_samples {
                    self.flying_start_state = FlyingStartState::Captured;
                }
            } else {
                self.flying_start_stable = 0;
            }
            if self.flying_start_elapsed >= self.config.flying_start_timeout_samples
                && self.flying_start_state != FlyingStartState::Captured
            {
                self.flying_start_state = FlyingStartState::Failed;
            }
        }
        output.flying_start_state = self.flying_start_state;
        output.flying_start_angle_rad = self.flying_start_angle_rad;
        output.flying_start_speed_rad_s = self.flying_start_speed_rad_s;
        if self.flying_start_state != FlyingStartState::Idle {
            output.active_features |= ADV_FOC_FLYING_START;
        }
    }
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn motor_is_valid(motor: &MotorParameters) -> bool {
    motor.pole_pairs != 0
        && finite_positive(motor.stator_resistance_ohm)
        && finite_positive(motor.ld_h)
        && finite_positive(motor.lq_h)
        && finite_positive(motor.flux_linkage_wb)
        && finite_positive(motor.rated_current_a)
}

fn input_is_valid(input: &AdvancedFocInput) -> bool {
    input.base_reference.id_ref_a.is_finite()
        && input.base_reference.iq_ref_a.is_finite()
        && input.measured_current_dq.d.is_finite()
        && input.measured_current_dq.q.is_finite()
        && input.current_alpha_beta.alpha.is_finite()
        && input.current_alpha_beta.beta.is_finite()
        && input.previous_voltage_dq.d.is_finite()
        && input.previous_voltage_dq.q.is_finite()
        && input.estimated_electrical_angle_rad.is_finite()
        && input.estimated_electrical_speed_rad_s.is_finite()
        && finite_positive(input.dc_bus_voltage_v)
        && input.linear_voltage_utilization.is_finite()
        && (0.05..=1.0).contains(&input.linear_voltage_utilization)
}

fn voltage_magnitude(value: Dq) -> f32 {
    libm::sqrtf(value.d * value.d + value.q * value.q)
}

fn alpha_beta_magnitude(value: AlphaBeta) -> f32 {
    libm::sqrtf(value.alpha * value.alpha + value.beta * value.beta)
}

fn current_magnitude(value: CurrentCommand) -> f32 {
    libm::sqrtf(value.id_ref_a * value.id_ref_a + value.iq_ref_a * value.iq_ref_a)
}

fn limit_current(value: CurrentCommand, limit: f32) -> CurrentCommand {
    let magnitude = current_magnitude(value);
    if limit > 0.0 && magnitude > limit && magnitude > 0.0 {
        let scale = limit / magnitude;
        CurrentCommand {
            id_ref_a: value.id_ref_a * scale,
            iq_ref_a: value.iq_ref_a * scale,
        }
    } else {
        value
    }
}

fn move_towards(current: f32, target: f32, maximum_step: f32) -> f32 {
    if target > current {
        (current + maximum_step).min(target)
    } else {
        (current - maximum_step).max(target)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spmsm() -> MotorParameters {
        MotorParameters {
            pole_pairs: 7,
            stator_resistance_ohm: 1.0,
            ld_h: 0.001,
            lq_h: 0.001,
            flux_linkage_wb: 0.01,
            rated_current_a: 10.0,
            max_speed_rpm: 10_000.0,
            nominal_bus_voltage_v: 48.0,
            inertia_kg_m2: 1.0e-4,
            viscous_friction_nm_s: 1.0e-5,
        }
    }

    fn ipmsm() -> MotorParameters {
        MotorParameters {
            ld_h: 0.000_6,
            lq_h: 0.001_2,
            ..spmsm()
        }
    }

    fn input() -> AdvancedFocInput {
        AdvancedFocInput {
            base_reference: CurrentCommand {
                id_ref_a: 0.0,
                iq_ref_a: 4.0,
            },
            measured_current_dq: Dq { d: 0.2, q: 3.5 },
            current_alpha_beta: AlphaBeta {
                alpha: 0.2,
                beta: -0.1,
            },
            previous_voltage_dq: Dq { d: 2.0, q: 8.0 },
            estimated_electrical_angle_rad: 1.0,
            estimated_electrical_speed_rad_s: 200.0,
            dc_bus_voltage_v: 48.0,
            linear_voltage_utilization: 0.95,
            closed_loop_active: true,
            observer_reliable: true,
            allow_voltage_injection: false,
            request_flying_start: false,
        }
    }

    fn candidate(features: u32) -> AdvancedFocConfig {
        AdvancedFocConfig {
            enabled_features: features,
            region_update_divider: 12,
            current_limit_a: 8.0,
            mtpa_min_current_a: 0.1,
            mtpa_search_steps: 16,
            weakening_entry_utilization: 0.92,
            weakening_exit_utilization: 0.86,
            weakening_kp_a_per_v: 0.2,
            weakening_id_min_a: -5.0,
            weakening_slew_a_per_s: 1000.0,
            mtpv_entry_electrical_speed_rad_s: 1000.0,
            mtpv_exit_electrical_speed_rad_s: 900.0,
            mtpv_search_steps: 16,
            decoupling_gain: 1.0,
            dpwm_entry_modulation: 0.8,
            dpwm_exit_modulation: 0.7,
            dpwm_mode: DpwmMode::ClampMax as u32,
            overmodulation_max_voltage_ratio: 0.62,
            overmodulation_entry_modulation: 1.0,
            overmodulation_exit_modulation: 0.97,
            hfi_amplitude_v: 1.0,
            hfi_frequency_hz: 1000.0,
            hfi_demod_alpha: 0.1,
            hfi_high_pass_alpha: 0.9,
            hfi_min_response_a: 0.001,
            hfi_max_electrical_speed_rad_s: 100.0,
            hfi_settling_samples: 4,
            flying_start_min_electrical_speed_rad_s: 20.0,
            flying_start_stable_samples: 3,
            flying_start_timeout_samples: 10,
            ..AdvancedFocConfig::default()
        }
    }

    #[test]
    fn disabled_configuration_is_exact_reference_passthrough() {
        let motor = spmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        supervisor
            .configure(AdvancedFocConfig::default(), &motor, 12_000)
            .unwrap();
        let input = input();
        let output = supervisor.step(&motor, input).unwrap();
        assert_eq!(output.current_reference, input.base_reference);
        assert_eq!(output.active_features, 0);
        assert_eq!(output.region, AdvancedFocRegion::Basic);
        assert_eq!(output.modulation_mode, AdvancedModulationMode::Svpwm);
        assert_eq!(output.voltage_feedforward_dq, Dq::default());
        assert_eq!(output.injection_voltage_alpha_beta, AlphaBeta::default());
    }

    #[test]
    fn invalid_reconfiguration_is_transactional() {
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        let good = candidate(ADV_FOC_MTPA);
        supervisor.configure(good, &motor, 12_000).unwrap();
        let mut bad = good;
        bad.mtpa_search_steps = 1000;
        assert_eq!(
            supervisor.configure(bad, &motor, 12_000),
            Err(AdvancedFocError::InvalidMtpaConfig)
        );
        assert_eq!(supervisor.config(), good);
    }

    #[test]
    fn mtpa_uses_saliency_but_spmsm_stays_at_zero_id() {
        let config = candidate(ADV_FOC_MTPA);
        let mut spm = AdvancedFocSupervisor::default();
        spm.configure(config, &spmsm(), 12_000).unwrap();
        let spm_output = spm.step(&spmsm(), input()).unwrap();
        assert_eq!(spm_output.current_reference.id_ref_a, 0.0);
        assert_eq!(spm_output.region, AdvancedFocRegion::Mtpa);

        let mut ipm = AdvancedFocSupervisor::default();
        ipm.configure(config, &ipmsm(), 12_000).unwrap();
        let ipm_output = ipm.step(&ipmsm(), input()).unwrap();
        assert!(ipm_output.current_reference.id_ref_a < 0.0);
        assert!(current_magnitude(ipm_output.current_reference) <= 8.0 + 1e-5);
    }

    #[test]
    fn weakening_uses_hysteresis_slew_and_current_circle() {
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        let config = candidate(ADV_FOC_FIELD_WEAKENING);
        supervisor.configure(config, &motor, 12_000).unwrap();
        let mut high = input();
        high.previous_voltage_dq = Dq { d: 0.0, q: 27.0 };
        let first = supervisor.step(&motor, high).unwrap();
        assert_eq!(first.region, AdvancedFocRegion::FieldWeakening);
        assert!(first.current_reference.id_ref_a < 0.0);
        assert!(current_magnitude(first.current_reference) <= config.current_limit_a + 1e-5);

        let mut between = high;
        between.previous_voltage_dq = Dq { d: 0.0, q: 25.0 };
        for _ in 0..config.region_update_divider {
            let held = supervisor.step(&motor, between).unwrap();
            assert_eq!(held.region, AdvancedFocRegion::FieldWeakening);
        }

        let mut low = high;
        low.previous_voltage_dq = Dq { d: 0.0, q: 20.0 };
        for _ in 0..config.region_update_divider {
            let _ = supervisor.step(&motor, low).unwrap();
        }
        assert_eq!(
            supervisor.step(&motor, low).unwrap().region,
            AdvancedFocRegion::Basic
        );
    }

    #[test]
    fn mtpv_is_bounded_by_requested_current_not_only_motor_limit() {
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        let mut config = candidate(ADV_FOC_MTPV);
        config.region_update_divider = 1;
        supervisor.configure(config, &motor, 12_000).unwrap();
        let mut high_speed = input();
        high_speed.estimated_electrical_speed_rad_s = 2500.0;
        let output = supervisor.step(&motor, high_speed).unwrap();
        assert_eq!(output.region, AdvancedFocRegion::Mtpv);
        assert!(current_magnitude(output.current_reference) <= 4.0 + 1e-4);
        high_speed.estimated_electrical_speed_rad_s = 950.0;
        assert_eq!(
            supervisor.step(&motor, high_speed).unwrap().region,
            AdvancedFocRegion::Mtpv
        );
        high_speed.estimated_electrical_speed_rad_s = 850.0;
        assert_eq!(
            supervisor.step(&motor, high_speed).unwrap().region,
            AdvancedFocRegion::Basic
        );
    }

    #[test]
    fn decoupling_feedforward_has_expected_dq_signs() {
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        supervisor
            .configure(candidate(ADV_FOC_DECOUPLING), &motor, 12_000)
            .unwrap();
        let output = supervisor.step(&motor, input()).unwrap();
        assert!(output.voltage_feedforward_dq.d < 0.0);
        assert!(output.voltage_feedforward_dq.q > 0.0);
        assert_ne!(output.active_features & ADV_FOC_DECOUPLING, 0);
    }

    #[test]
    fn modulation_policy_selects_dpwm_then_bounded_overmodulation() {
        let motor = ipmsm();
        let features = ADV_FOC_DPWM | ADV_FOC_OVERMODULATION;
        let mut supervisor = AdvancedFocSupervisor::default();
        let config = candidate(features);
        supervisor.configure(config, &motor, 12_000).unwrap();
        let mut dpwm = input();
        dpwm.previous_voltage_dq = Dq { d: 0.0, q: 23.0 };
        assert_eq!(
            supervisor.step(&motor, dpwm).unwrap().modulation_mode,
            AdvancedModulationMode::DpwmClampMax
        );
        let mut overmod = dpwm;
        overmod.previous_voltage_dq.q = 29.0;
        let output = supervisor.step(&motor, overmod).unwrap();
        assert_eq!(
            output.modulation_mode,
            AdvancedModulationMode::Overmodulation
        );
        assert!((output.voltage_limit_v - 48.0 * 0.62).abs() < 1e-5);
        overmod.previous_voltage_dq.q = 27.5;
        assert_eq!(
            supervisor.step(&motor, overmod).unwrap().modulation_mode,
            AdvancedModulationMode::Overmodulation
        );
        overmod.previous_voltage_dq.q = 26.0;
        assert_eq!(
            supervisor.step(&motor, overmod).unwrap().modulation_mode,
            AdvancedModulationMode::DpwmClampMax
        );
    }

    #[test]
    fn hfi_is_rejected_for_non_salient_motor_and_explicitly_gated() {
        let config = candidate(ADV_FOC_HFI);
        assert_eq!(
            config.validate(&spmsm(), 12_000),
            Err(AdvancedFocError::MotorHasInsufficientSaliency)
        );
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        supervisor.configure(config, &motor, 12_000).unwrap();
        let no_request = supervisor.step(&motor, input()).unwrap();
        assert_eq!(
            no_request.injection_voltage_alpha_beta,
            AlphaBeta::default()
        );
        let mut requested = input();
        requested.estimated_electrical_speed_rad_s = 0.0;
        requested.allow_voltage_injection = true;
        let first = supervisor.step(&motor, requested).unwrap();
        assert_ne!(first.injection_voltage_alpha_beta, AlphaBeta::default());
        assert!(!first.hfi_angle_candidate_valid);
    }

    #[test]
    fn flying_start_requires_stable_reliable_passive_samples() {
        let motor = ipmsm();
        let mut supervisor = AdvancedFocSupervisor::default();
        supervisor
            .configure(candidate(ADV_FOC_FLYING_START), &motor, 12_000)
            .unwrap();
        let mut scan = input();
        scan.closed_loop_active = false;
        scan.request_flying_start = true;
        scan.estimated_electrical_speed_rad_s = 50.0;
        for _ in 0..2 {
            assert_eq!(
                supervisor.step(&motor, scan).unwrap().flying_start_state,
                FlyingStartState::Scanning
            );
        }
        let captured = supervisor.step(&motor, scan).unwrap();
        assert_eq!(captured.flying_start_state, FlyingStartState::Captured);
        assert_eq!(
            captured.flying_start_angle_rad,
            scan.estimated_electrical_angle_rad
        );
        assert_eq!(captured.flying_start_speed_rad_s, 50.0);
    }
}
