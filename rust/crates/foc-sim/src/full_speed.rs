//! Closed-loop composition for full-speed-range design simulation.
//!
//! This host-only owner connects the production current/speed controllers and
//! advanced supervisor to [`crate::dynamic_plant::DynamicPmsmPlant`] with ideal
//! rotor/current feedback.  It is a design and regression model, not evidence
//! that a sensorless observer, ADC chain or physical power stage is validated.

use foc_algorithm::{clarke, inverse_clarke, inverse_park, park, Abc, AlphaBeta, Dq};
use foc_control::{
    AdvancedFocConfig, AdvancedFocError, AdvancedFocInput, AdvancedFocOutput,
    AdvancedFocSupervisor, ControlAngleOffsets, ControlParameters, ControlTelemetry, CpuMath,
    CurrentCommand, CurrentLoop, FeedbackSnapshot, PhaseCurrents, PwmCommand, RotorFeedback,
    SpeedCommand, SpeedLoop,
};

use crate::dynamic_plant::{
    DynamicPmsmError, DynamicPmsmInput, DynamicPmsmOutput, DynamicPmsmPlant,
};

#[derive(Clone, Copy, Debug)]
pub struct FullSpeedClosedLoopConfig {
    pub control: ControlParameters,
    pub advanced: AdvancedFocConfig,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FullSpeedClosedLoopOutput {
    pub target_speed_rpm: f32,
    pub base_current_reference: CurrentCommand,
    pub advanced: AdvancedFocOutput,
    pub control: ControlTelemetry,
    pub pwm: PwmCommand,
    pub plant: DynamicPmsmOutput,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FullSpeedClosedLoopError {
    InvalidConfiguration(&'static str),
    Advanced(AdvancedFocError),
    Plant(DynamicPmsmError),
    InvalidOutput,
}

pub struct FullSpeedClosedLoop {
    config: FullSpeedClosedLoopConfig,
    speed: SpeedLoop,
    current: CurrentLoop,
    advanced: AdvancedFocSupervisor,
    speed_divider: u32,
    speed_counter: u32,
    base_reference: CurrentCommand,
    previous_voltage_dq: Dq,
}

impl FullSpeedClosedLoop {
    pub fn new(config: FullSpeedClosedLoopConfig) -> Result<Self, FullSpeedClosedLoopError> {
        let control = config.control;
        if control.pwm_frequency_hz == 0
            || control.speed_loop_frequency_hz == 0
            || !control
                .pwm_frequency_hz
                .is_multiple_of(control.speed_loop_frequency_hz)
            || !control.voltage_utilization.is_finite()
            || !(0.0..=1.0).contains(&control.voltage_utilization)
        {
            return Err(FullSpeedClosedLoopError::InvalidConfiguration(
                "control rates and voltage utilization must be physical",
            ));
        }
        let mut advanced = AdvancedFocSupervisor::default();
        advanced
            .configure(config.advanced, &control.motor, control.pwm_frequency_hz)
            .map_err(FullSpeedClosedLoopError::Advanced)?;
        Ok(Self {
            config,
            speed: SpeedLoop::default(),
            current: CurrentLoop::default(),
            advanced,
            speed_divider: control.pwm_frequency_hz / control.speed_loop_frequency_hz,
            speed_counter: 0,
            base_reference: CurrentCommand::default(),
            previous_voltage_dq: Dq::default(),
        })
    }

    pub fn reset(&mut self) {
        self.speed.reset();
        self.current.reset();
        self.advanced.reset();
        self.speed_counter = 0;
        self.base_reference = CurrentCommand::default();
        self.previous_voltage_dq = Dq::default();
    }

    pub fn step(
        &mut self,
        plant: &mut DynamicPmsmPlant,
        target_speed_rpm: f32,
    ) -> Result<FullSpeedClosedLoopOutput, FullSpeedClosedLoopError> {
        if !target_speed_rpm.is_finite()
            || target_speed_rpm.abs() > self.config.control.motor.max_speed_rpm
        {
            return Err(FullSpeedClosedLoopError::InvalidConfiguration(
                "target speed exceeds the declared motor envelope",
            ));
        }
        let expected_step = 1.0 / self.config.control.pwm_frequency_hz as f32;
        if (plant.config().step_s - expected_step).abs() > expected_step * 1.0e-4 {
            return Err(FullSpeedClosedLoopError::InvalidConfiguration(
                "plant and controller sample periods disagree",
            ));
        }

        let state = plant.state();
        let angle = plant.electrical_angle_rad();
        let currents = ideal_phase_currents(state.current_d_a, state.current_q_a, angle);
        let feedback = FeedbackSnapshot {
            currents,
            dc_bus_voltage: state.dc_bus_voltage_v,
            rotor: RotorFeedback {
                electrical_angle_rad: angle,
                mechanical_speed_rad_s: state.mechanical_speed_rad_s,
            },
        };
        if self.speed_counter == 0 {
            self.base_reference = self.speed.update(
                &self.config.control,
                SpeedCommand {
                    target_rpm: target_speed_rpm,
                    id_ref_a: 0.0,
                },
                state.mechanical_speed_rad_s,
            );
        }
        self.speed_counter += 1;
        if self.speed_counter >= self.speed_divider {
            self.speed_counter = 0;
        }

        let current_alpha_beta = clarke(Abc {
            a: currents.a,
            b: currents.b,
            c: currents.c,
        });
        let mut math = CpuMath;
        let frame = CurrentLoop::precompute_current_frame_with_math(
            &feedback,
            current_alpha_beta,
            0.0,
            &mut math,
        );
        let advanced = self
            .advanced
            .step(
                &self.config.control.motor,
                AdvancedFocInput {
                    base_reference: self.base_reference,
                    measured_current_dq: frame.current_dq,
                    current_alpha_beta,
                    previous_voltage_dq: self.previous_voltage_dq,
                    estimated_electrical_angle_rad: angle,
                    estimated_electrical_speed_rad_s: state.mechanical_speed_rad_s
                        * self.config.control.motor.pole_pairs as f32,
                    dc_bus_voltage_v: state.dc_bus_voltage_v,
                    linear_voltage_utilization: self.config.control.voltage_utilization,
                    closed_loop_active: true,
                    observer_reliable: true,
                    allow_voltage_injection: false,
                    request_flying_start: false,
                },
            )
            .map_err(FullSpeedClosedLoopError::Advanced)?;
        let (pwm, control) = self
            .current
            .update_from_precomputed_current_frame_with_policy_and_math(
                &self.config.control,
                &feedback,
                advanced.current_reference,
                ControlAngleOffsets::default(),
                frame,
                advanced.current_loop_policy(),
                &mut math,
            );
        if !pwm.is_valid() || !control.voltage_dq.d.is_finite() || !control.voltage_dq.q.is_finite()
        {
            self.reset();
            return Err(FullSpeedClosedLoopError::InvalidOutput);
        }
        self.previous_voltage_dq = control.voltage_dq;

        // Reconstruct the average applied voltage from duty, then use the true
        // plant angle to enter its d/q input. This preserves the production PWM
        // mapper while keeping the design plant independent from controller state.
        let applied_alpha_beta = pwm_to_alpha_beta(pwm, state.dc_bus_voltage_v);
        let applied_dq = park(applied_alpha_beta, angle);
        let plant_output = plant
            .step(DynamicPmsmInput {
                voltage_d_v: applied_dq.d,
                voltage_q_v: applied_dq.q,
            })
            .map_err(FullSpeedClosedLoopError::Plant)?;

        Ok(FullSpeedClosedLoopOutput {
            target_speed_rpm,
            base_current_reference: self.base_reference,
            advanced,
            control,
            pwm,
            plant: plant_output,
        })
    }
}

fn ideal_phase_currents(current_d_a: f32, current_q_a: f32, angle: f32) -> PhaseCurrents {
    let stationary = inverse_park(
        Dq {
            d: current_d_a,
            q: current_q_a,
        },
        angle,
    );
    let abc = inverse_clarke(stationary);
    PhaseCurrents {
        a: abc.a,
        b: abc.b,
        c: abc.c,
    }
}

fn pwm_to_alpha_beta(pwm: PwmCommand, dc_bus_voltage_v: f32) -> AlphaBeta {
    let common = (pwm.duty_a + pwm.duty_b + pwm.duty_c) / 3.0;
    clarke(Abc {
        a: (pwm.duty_a - common) * dc_bus_voltage_v,
        b: (pwm.duty_b - common) * dc_bus_voltage_v,
        c: (pwm.duty_c - common) * dc_bus_voltage_v,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamic_plant::{
        DynamicDcBus, DynamicInverterLimit, DynamicPmsmConfig, DynamicPmsmMotor, DynamicPmsmState,
        PmsmTopology,
    };
    use foc_control::{
        st_gbm2804_reference_parameters, ADVANCED_FOC_CONFIG_VERSION, ADV_FOC_DECOUPLING,
        ADV_FOC_FIELD_WEAKENING,
    };

    fn fixture() -> (FullSpeedClosedLoop, DynamicPmsmPlant) {
        let control = st_gbm2804_reference_parameters();
        let mut advanced = AdvancedFocConfig {
            struct_size: core::mem::size_of::<AdvancedFocConfig>() as u32,
            version: ADVANCED_FOC_CONFIG_VERSION,
            enabled_features: ADV_FOC_FIELD_WEAKENING | ADV_FOC_DECOUPLING,
            region_update_divider: 12,
            current_limit_a: 0.8,
            weakening_entry_utilization: 0.92,
            weakening_exit_utilization: 0.82,
            weakening_kp_a_per_v: 0.08,
            weakening_id_min_a: -0.6,
            weakening_slew_a_per_s: 20.0,
            decoupling_gain: 1.0,
            ..AdvancedFocConfig::default()
        };
        advanced.struct_size = core::mem::size_of::<AdvancedFocConfig>() as u32;
        let controller =
            FullSpeedClosedLoop::new(FullSpeedClosedLoopConfig { control, advanced }).unwrap();
        let plant = DynamicPmsmPlant::new(
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
                    source_voltage_v: 13.0,
                    source_resistance_ohm: 0.2,
                    capacitance_f: 0.002,
                    minimum_voltage_v: 6.0,
                    maximum_voltage_v: 20.0,
                    source_can_sink: false,
                },
            },
            DynamicPmsmState::at_rest(13.0),
        )
        .unwrap();
        (controller, plant)
    }

    #[test]
    fn closed_loop_accelerates_in_both_directions_with_finite_outputs() {
        for target in [600.0, -600.0] {
            let (mut controller, mut plant) = fixture();
            let mut last = None;
            for _ in 0..12_000 {
                last = Some(controller.step(&mut plant, target).unwrap());
            }
            let output = last.unwrap();
            assert!(output.plant.state.mechanical_speed_rad_s.signum() == target.signum());
            assert!(output.plant.state.dc_bus_voltage_v.is_finite());
            assert!(output.control.current_dq.d.is_finite());
            assert!(output.pwm.is_valid());
        }
    }

    #[test]
    fn invalid_target_and_timebase_fail_closed() {
        let (mut controller, mut plant) = fixture();
        assert!(matches!(
            controller.step(&mut plant, 2000.0),
            Err(FullSpeedClosedLoopError::InvalidConfiguration(_))
        ));
    }
}
