//! Persisted-product configuration mapping for the P4.2 motion runtime.
//!
//! This is the only place that derives planner/cascade parameters from a
//! [`ConfigBundle`].  The control cores therefore remain independent from the
//! persistence schema, while target firmware and PC simulation share exactly
//! the same unit conversions and limits.

use crate::{ConfigBundle, ConfigValidationError, MotionCascadeConfig, MotionPlannerConfig};

/// Validated, runtime-ready motion configuration in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionRuntimeConfig {
    pub config_revision: u32,
    pub planner: MotionPlannerConfig,
    pub cascade: MotionCascadeConfig,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionConfigError {
    InvalidBundle(ConfigValidationError),
    Disabled,
    InvalidDerivedValue,
}

impl MotionRuntimeConfig {
    /// Map one persisted configuration bundle into the motion core.
    ///
    /// The torque constant uses the base SPMSM relation
    /// `Kt = 1.5 * pole_pairs * flux_linkage`.  Reluctance-torque/MTPA mapping
    /// belongs to a later motor-model extension and must not be silently folded
    /// into this conversion.
    pub fn from_bundle(bundle: &ConfigBundle) -> Result<Self, MotionConfigError> {
        bundle
            .validate()
            .map_err(MotionConfigError::InvalidBundle)?;
        if bundle.axis.motion_control_enabled == 0 {
            return Err(MotionConfigError::Disabled);
        }

        let control_period_s = 1.0 / bundle.board.control_frequency_hz as f32;
        let maximum_current_a = bundle
            .motor
            .continuous_current_a
            .min(bundle.inverter.maximum_phase_current_a);
        let torque_constant_nm_per_a =
            1.5 * bundle.motor.pole_pairs as f32 * bundle.motor.flux_linkage_v_s;
        let maximum_torque_nm = maximum_current_a * torque_constant_nm_per_a;

        let runtime = Self {
            config_revision: bundle.bundle_revision,
            planner: MotionPlannerConfig {
                control_period_s,
                maximum_current_a,
                maximum_torque_nm,
                maximum_velocity_rad_s: bundle.motor.maximum_speed_rad_s,
                torque_ramp_rate_nm_s: bundle.axis.torque_ramp_rate_nm_s,
                velocity_ramp_rate_rad_s2: bundle.axis.velocity_ramp_rate_rad_s2,
                position_filter_bandwidth_rad_s: bundle.axis.position_filter_bandwidth_rad_s,
                trajectory_acceleration_rad_s2: bundle.axis.trajectory_acceleration_rad_s2,
                trajectory_deceleration_rad_s2: bundle.axis.trajectory_deceleration_rad_s2,
                soft_limit_min_rad: bundle.axis.soft_limit_min_rad,
                soft_limit_max_rad: bundle.axis.soft_limit_max_rad,
            },
            cascade: MotionCascadeConfig {
                control_period_s,
                position_kp_per_s: bundle.axis.position_kp,
                velocity_kp_nm_per_rad_s: bundle.axis.velocity_kp,
                velocity_ki_nm_per_rad: bundle.axis.velocity_ki,
                torque_constant_nm_per_a,
            },
        };
        runtime
            .planner
            .validate()
            .map_err(|_| MotionConfigError::InvalidDerivedValue)?;
        runtime
            .cascade
            .validate()
            .map_err(|_| MotionConfigError::InvalidDerivedValue)?;
        Ok(runtime)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        AppConfig, AxisConfig, BoardConfig, CalibrationData, CommandSourceConfig, ExternalIoConfig,
        FeedbackMode, InverterConfig, MotorConfig, BOARD_CAP_THREE_SHUNT_CURRENT,
        COMMAND_SOURCE_PERMISSION_KNOWN_MASK, CONFIG_GROUP_VERSION,
    };

    fn enabled_bundle() -> ConfigBundle {
        ConfigBundle {
            bundle_revision: 9,
            generated_by_version: 1,
            board: BoardConfig {
                version: CONFIG_GROUP_VERSION,
                board_id: 1,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.01,
            },
            motor: MotorConfig {
                version: CONFIG_GROUP_VERSION,
                motor_id: 2,
                pole_pairs: 7,
                phase_resistance_ohm: 5.0,
                d_inductance_h: 0.001,
                q_inductance_h: 0.001,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: InverterConfig {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 3,
                maximum_phase_current_a: 2.0,
                minimum_bus_voltage_v: 7.0,
                maximum_bus_voltage_v: 18.0,
                dead_time_s: 550.0e-9,
                transistor_drop_v: 0.35,
                brake_resistance_ohm: 0.0,
                maximum_duty: 0.95,
            },
            axis: AxisConfig {
                version: CONFIG_GROUP_VERSION,
                axis_id: 0,
                feedback_mode: FeedbackMode::Sensorless as u32,
                direction: 1,
                current_kp: 3.0,
                current_ki: 100.0,
                velocity_kp: 0.02,
                velocity_ki: 0.2,
                position_kp: 1.0,
                soft_limit_min_rad: -100.0,
                soft_limit_max_rad: 100.0,
                motion_control_enabled: 1,
                torque_ramp_rate_nm_s: 1.0,
                velocity_ramp_rate_rad_s2: 20.0,
                position_filter_bandwidth_rad_s: 40.0,
                trajectory_acceleration_rad_s2: 30.0,
                trajectory_deceleration_rad_s2: 35.0,
            },
            app: AppConfig {
                version: CONFIG_GROUP_VERSION,
                can_node_id: 1,
                uart_baud: 115_200,
                command_source_count: 1,
                command_sources: [
                    CommandSourceConfig {
                        source_id: 1,
                        priority: 10,
                        permissions: COMMAND_SOURCE_PERMISSION_KNOWN_MASK,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                ],
            },
            external_io: ExternalIoConfig::default(),
            calibration: CalibrationData {
                version: CONFIG_GROUP_VERSION,
                board_id: 1,
                motor_id: 2,
                valid_flags: 0,
                current_offset_counts: [0.0; 3],
                phase_voltage_gain: [0.0; 3],
                phase_voltage_offset_v: [0.0; 3],
                encoder_offset_rad: 0.0,
                encoder_counts_per_revolution: 0,
                hall_sequence_packed: 0,
            },
        }
    }

    #[test]
    fn maps_si_units_limits_and_spmsm_torque_constant() {
        let runtime = MotionRuntimeConfig::from_bundle(&enabled_bundle()).unwrap();
        assert_eq!(runtime.config_revision, 9);
        assert!((runtime.planner.control_period_s - 1.0 / 12_000.0).abs() < 1.0e-9);
        assert_eq!(runtime.planner.maximum_current_a, 0.8);
        assert!((runtime.cascade.torque_constant_nm_per_a - 0.2625).abs() < 1.0e-6);
        assert!((runtime.planner.maximum_torque_nm - 0.21).abs() < 1.0e-6);
        assert_eq!(runtime.planner.maximum_velocity_rad_s, 200.0);
        assert_eq!(runtime.cascade.position_kp_per_s, 1.0);
    }

    #[test]
    fn refuses_default_off_and_invalid_persisted_configuration() {
        let mut bundle = enabled_bundle();
        bundle.axis.motion_control_enabled = 0;
        bundle.axis.torque_ramp_rate_nm_s = 0.0;
        bundle.axis.velocity_ramp_rate_rad_s2 = 0.0;
        bundle.axis.position_filter_bandwidth_rad_s = 0.0;
        bundle.axis.trajectory_acceleration_rad_s2 = 0.0;
        bundle.axis.trajectory_deceleration_rad_s2 = 0.0;
        assert_eq!(
            MotionRuntimeConfig::from_bundle(&bundle),
            Err(MotionConfigError::Disabled)
        );

        let mut invalid = enabled_bundle();
        invalid.axis.velocity_ramp_rate_rad_s2 = f32::NAN;
        assert!(matches!(
            MotionRuntimeConfig::from_bundle(&invalid),
            Err(MotionConfigError::InvalidBundle(
                ConfigValidationError::Axis
            ))
        ));
    }
}
