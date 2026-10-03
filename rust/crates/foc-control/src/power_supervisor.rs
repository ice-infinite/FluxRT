//! Deterministic thermal, DC-bus and regenerative-energy supervisor.
//!
//! This module owns policy only. It never reads a sensor, switches a brake
//! resistor, writes PWM, or assumes a particular MCU. Positive DC current means
//! energy drawn from the source; negative current means regeneration into the
//! source or brake path. The caller remains responsible for immediate hardware
//! over-voltage and over-temperature shutdown.

/// Temperature feedback is missing or outside its declared measurement range.
pub const POWER_FAULT_TEMPERATURE_SENSOR: u32 = 1 << 0;
/// Temperature reached the configured trip point.
pub const POWER_FAULT_OVER_TEMPERATURE: u32 = 1 << 1;
/// DC-bus voltage reached or crossed the under-voltage trip point.
pub const POWER_FAULT_BUS_UNDERVOLTAGE: u32 = 1 << 2;
/// DC-bus voltage reached or crossed the over-voltage trip point.
pub const POWER_FAULT_BUS_OVERVOLTAGE: u32 = 1 << 3;
/// At least one floating-point input was non-finite, or Vbus was non-positive.
pub const POWER_FAULT_INVALID_INPUT: u32 = 1 << 4;
/// Positive power was requested while the configured source was unavailable.
pub const POWER_FAULT_SOURCE_UNAVAILABLE: u32 = 1 << 5;
pub const POWER_FAULT_KNOWN_MASK: u32 = POWER_FAULT_TEMPERATURE_SENSOR
    | POWER_FAULT_OVER_TEMPERATURE
    | POWER_FAULT_BUS_UNDERVOLTAGE
    | POWER_FAULT_BUS_OVERVOLTAGE
    | POWER_FAULT_INVALID_INPUT
    | POWER_FAULT_SOURCE_UNAVAILABLE;

/// Stopped-state configuration for [`PowerSupervisor`].
///
/// `enabled == false` is the safe default: no motoring, regeneration or brake
/// request is authorised. Regeneration additionally requires an explicit
/// `regeneration_allowed` and a non-zero sink or brake capability.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PowerSupervisorConfig {
    pub enabled: bool,
    pub temperature_sensor_required: bool,
    pub minimum_valid_temperature_c: f32,
    pub maximum_valid_temperature_c: f32,
    pub thermal_derating_release_c: f32,
    pub thermal_derating_start_c: f32,
    pub over_temperature_trip_c: f32,
    pub bus_undervoltage_trip_v: f32,
    pub bus_undervoltage_recovery_v: f32,
    pub bus_overvoltage_recovery_v: f32,
    pub bus_overvoltage_trip_v: f32,
    pub source_current_limit_a: f32,
    pub regeneration_allowed: bool,
    pub sink_current_limit_a: f32,
    pub brake_resistor_available: bool,
    pub brake_current_limit_a: f32,
    pub brake_release_voltage_v: f32,
    pub brake_engage_voltage_v: f32,
}

impl Default for PowerSupervisorConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            temperature_sensor_required: false,
            minimum_valid_temperature_c: -40.0,
            maximum_valid_temperature_c: 150.0,
            thermal_derating_release_c: 70.0,
            thermal_derating_start_c: 80.0,
            over_temperature_trip_c: 100.0,
            bus_undervoltage_trip_v: 7.0,
            bus_undervoltage_recovery_v: 8.0,
            bus_overvoltage_recovery_v: 16.0,
            bus_overvoltage_trip_v: 18.0,
            source_current_limit_a: 0.0,
            regeneration_allowed: false,
            sink_current_limit_a: 0.0,
            brake_resistor_available: false,
            brake_current_limit_a: 0.0,
            brake_release_voltage_v: 0.0,
            brake_engage_voltage_v: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PowerSupervisorConfigError {
    InvalidTemperatureWindow,
    InvalidBusWindow,
    InvalidSourceCapability,
    InvalidSinkCapability,
    InvalidBrakeCapability,
}

impl PowerSupervisorConfig {
    pub fn validate(&self) -> Result<(), PowerSupervisorConfigError> {
        if !all_finite(&[
            self.minimum_valid_temperature_c,
            self.maximum_valid_temperature_c,
            self.thermal_derating_release_c,
            self.thermal_derating_start_c,
            self.over_temperature_trip_c,
        ]) || self.minimum_valid_temperature_c >= self.thermal_derating_release_c
            || self.thermal_derating_release_c >= self.thermal_derating_start_c
            || self.thermal_derating_start_c >= self.over_temperature_trip_c
            || self.over_temperature_trip_c > self.maximum_valid_temperature_c
        {
            return Err(PowerSupervisorConfigError::InvalidTemperatureWindow);
        }
        if !all_finite(&[
            self.bus_undervoltage_trip_v,
            self.bus_undervoltage_recovery_v,
            self.bus_overvoltage_recovery_v,
            self.bus_overvoltage_trip_v,
        ]) || self.bus_undervoltage_trip_v <= 0.0
            || self.bus_undervoltage_trip_v >= self.bus_undervoltage_recovery_v
            || self.bus_undervoltage_recovery_v >= self.bus_overvoltage_recovery_v
            || self.bus_overvoltage_recovery_v >= self.bus_overvoltage_trip_v
        {
            return Err(PowerSupervisorConfigError::InvalidBusWindow);
        }
        if !finite_nonnegative(self.source_current_limit_a)
            || (self.enabled && self.source_current_limit_a <= 0.0)
        {
            return Err(PowerSupervisorConfigError::InvalidSourceCapability);
        }
        if !finite_nonnegative(self.sink_current_limit_a) {
            return Err(PowerSupervisorConfigError::InvalidSinkCapability);
        }
        if self.brake_resistor_available {
            if !finite_positive(self.brake_current_limit_a)
                || !all_finite(&[self.brake_release_voltage_v, self.brake_engage_voltage_v])
                || self.brake_release_voltage_v < self.bus_overvoltage_recovery_v
                || self.brake_release_voltage_v >= self.brake_engage_voltage_v
                || self.brake_engage_voltage_v >= self.bus_overvoltage_trip_v
            {
                return Err(PowerSupervisorConfigError::InvalidBrakeCapability);
            }
        } else if self.brake_current_limit_a != 0.0
            || self.brake_release_voltage_v != 0.0
            || self.brake_engage_voltage_v != 0.0
        {
            return Err(PowerSupervisorConfigError::InvalidBrakeCapability);
        }
        if self.regeneration_allowed
            && self.sink_current_limit_a <= 0.0
            && !self.brake_resistor_available
        {
            return Err(PowerSupervisorConfigError::InvalidSinkCapability);
        }
        if !self.enabled
            && (self.source_current_limit_a != 0.0
                || self.regeneration_allowed
                || self.sink_current_limit_a != 0.0
                || self.brake_resistor_available)
        {
            return Err(PowerSupervisorConfigError::InvalidSourceCapability);
        }
        Ok(())
    }
}

/// One coherent sample supplied by platform-independent orchestration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PowerSupervisorInput {
    pub temperature_valid: bool,
    pub temperature_c: f32,
    pub dc_bus_voltage_v: f32,
    /// Positive draws from the source; negative requests regeneration.
    pub requested_dc_current_a: f32,
    pub source_available: bool,
    pub sink_available: bool,
    pub brake_available: bool,
}

/// Deterministic policy result for one sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PowerSupervisorOutput {
    pub configured: bool,
    pub enabled: bool,
    pub drive_allowed: bool,
    pub shutdown_requested: bool,
    pub active_fault_flags: u32,
    pub latched_fault_flags: u32,
    pub temperature_valid: bool,
    pub thermal_derating_active: bool,
    pub thermal_current_scale: f32,
    pub maximum_source_current_a: f32,
    pub maximum_regeneration_current_a: f32,
    pub requested_dc_current_a: f32,
    pub limited_dc_current_a: f32,
    pub source_limited: bool,
    pub regeneration_limited: bool,
    pub brake_requested: bool,
    pub brake_current_limit_a: f32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PowerSupervisorResetError {
    NotConfigured,
    InvalidInput,
    UnsafeConditions,
}

/// Fixed-memory owner of thermal and DC-energy safety state.
#[derive(Clone, Copy, Debug, Default)]
pub struct PowerSupervisor {
    config: PowerSupervisorConfig,
    configured: bool,
    thermal_derating_active: bool,
    brake_active: bool,
    latched_fault_flags: u32,
}

impl PowerSupervisor {
    /// Applies a fully validated stopped-state configuration transactionally.
    /// Reconfiguration deliberately preserves latched faults; only
    /// [`Self::reset_faults`] may clear them.
    pub fn configure(
        &mut self,
        config: PowerSupervisorConfig,
    ) -> Result<(), PowerSupervisorConfigError> {
        config.validate()?;
        self.config = config;
        self.configured = true;
        self.thermal_derating_active = false;
        self.brake_active = false;
        Ok(())
    }

    pub const fn is_configured(&self) -> bool {
        self.configured
    }

    pub const fn fault_flags(&self) -> u32 {
        self.latched_fault_flags
    }

    /// Advances all policies once. No error path can leave a stale permissive
    /// output: invalid data is latched and this call returns a zero-current,
    /// shutdown-requested result in the same tick.
    pub fn step(&mut self, input: PowerSupervisorInput) -> PowerSupervisorOutput {
        if !self.configured {
            return self.safe_output(input, 0, false);
        }
        if !self.config.enabled {
            return self.safe_output(input, 0, false);
        }

        let active_fault_flags = self.active_faults(&input);
        self.latched_fault_flags |= active_fault_flags;
        self.update_hysteresis(&input);

        let brake_requested = self.brake_active;
        let brake_current_limit_a = if brake_requested {
            self.config.brake_current_limit_a
        } else {
            0.0
        };
        if self.latched_fault_flags != 0 {
            return self.safe_output_with_brake(
                input,
                active_fault_flags,
                brake_requested,
                brake_current_limit_a,
            );
        }

        let thermal_current_scale = self.thermal_scale(&input);
        let maximum_source_current_a = if input.source_available {
            self.config.source_current_limit_a * thermal_current_scale
        } else {
            0.0
        };
        let sink_limit_a = if self.config.regeneration_allowed && input.sink_available {
            self.config.sink_current_limit_a
        } else {
            0.0
        };
        let maximum_regeneration_current_a = if self.config.regeneration_allowed {
            (sink_limit_a + brake_current_limit_a) * thermal_current_scale
        } else {
            0.0
        };
        let requested_dc_current_a = input.requested_dc_current_a;
        let limited_dc_current_a = if requested_dc_current_a >= 0.0 {
            requested_dc_current_a.min(maximum_source_current_a)
        } else {
            -(-requested_dc_current_a).min(maximum_regeneration_current_a)
        };

        PowerSupervisorOutput {
            configured: true,
            enabled: true,
            drive_allowed: input.source_available,
            shutdown_requested: !input.source_available,
            active_fault_flags,
            latched_fault_flags: self.latched_fault_flags,
            temperature_valid: input.temperature_valid,
            thermal_derating_active: self.thermal_derating_active,
            thermal_current_scale,
            maximum_source_current_a,
            maximum_regeneration_current_a,
            requested_dc_current_a,
            limited_dc_current_a,
            source_limited: requested_dc_current_a > limited_dc_current_a,
            regeneration_limited: requested_dc_current_a < limited_dc_current_a,
            brake_requested,
            brake_current_limit_a,
        }
    }

    /// Clears every sticky fault only after an explicit zero-demand request and
    /// recovery-temperature/recovery-voltage checks. Failed resets do not mutate
    /// any state.
    pub fn reset_faults(
        &mut self,
        input: PowerSupervisorInput,
    ) -> Result<(), PowerSupervisorResetError> {
        if !self.configured {
            return Err(PowerSupervisorResetError::NotConfigured);
        }
        if !inputs_are_finite(&input) || input.dc_bus_voltage_v <= 0.0 {
            return Err(PowerSupervisorResetError::InvalidInput);
        }
        let temperature_safe = if input.temperature_valid {
            self.temperature_in_measurement_range(input.temperature_c)
                && input.temperature_c <= self.config.thermal_derating_release_c
        } else {
            !self.config.temperature_sensor_required
        };
        let source_safe = !self.config.enabled || input.source_available;
        if input.requested_dc_current_a != 0.0
            || !temperature_safe
            || !source_safe
            || input.dc_bus_voltage_v < self.config.bus_undervoltage_recovery_v
            || input.dc_bus_voltage_v > self.config.bus_overvoltage_recovery_v
        {
            return Err(PowerSupervisorResetError::UnsafeConditions);
        }

        self.latched_fault_flags = 0;
        self.thermal_derating_active = false;
        self.brake_active = false;
        Ok(())
    }

    fn active_faults(&self, input: &PowerSupervisorInput) -> u32 {
        let mut flags = 0;
        if !inputs_are_finite(input) || input.dc_bus_voltage_v <= 0.0 {
            flags |= POWER_FAULT_INVALID_INPUT;
        }
        if input.temperature_valid {
            if !input.temperature_c.is_finite()
                || !self.temperature_in_measurement_range(input.temperature_c)
            {
                flags |= POWER_FAULT_TEMPERATURE_SENSOR;
            }
            if input.temperature_c.is_finite()
                && input.temperature_c >= self.config.over_temperature_trip_c
            {
                flags |= POWER_FAULT_OVER_TEMPERATURE;
            }
        } else if self.config.temperature_sensor_required {
            flags |= POWER_FAULT_TEMPERATURE_SENSOR;
        }
        if input.dc_bus_voltage_v.is_finite() && input.dc_bus_voltage_v > 0.0 {
            if input.dc_bus_voltage_v <= self.config.bus_undervoltage_trip_v {
                flags |= POWER_FAULT_BUS_UNDERVOLTAGE;
            }
            if input.dc_bus_voltage_v >= self.config.bus_overvoltage_trip_v {
                flags |= POWER_FAULT_BUS_OVERVOLTAGE;
            }
        }
        if input.requested_dc_current_a.is_finite()
            && input.requested_dc_current_a > 0.0
            && !input.source_available
        {
            flags |= POWER_FAULT_SOURCE_UNAVAILABLE;
        }
        flags
    }

    fn update_hysteresis(&mut self, input: &PowerSupervisorInput) {
        if input.temperature_valid && self.temperature_in_measurement_range(input.temperature_c) {
            if self.thermal_derating_active {
                if input.temperature_c <= self.config.thermal_derating_release_c {
                    self.thermal_derating_active = false;
                }
            } else if input.temperature_c >= self.config.thermal_derating_start_c {
                self.thermal_derating_active = true;
            }
        }

        if self.config.brake_resistor_available
            && input.brake_available
            && input.dc_bus_voltage_v.is_finite()
            && input.dc_bus_voltage_v > 0.0
        {
            if self.brake_active {
                if input.dc_bus_voltage_v <= self.config.brake_release_voltage_v {
                    self.brake_active = false;
                }
            } else if input.dc_bus_voltage_v >= self.config.brake_engage_voltage_v {
                self.brake_active = true;
            }
        } else {
            self.brake_active = false;
        }
    }

    fn thermal_scale(&self, input: &PowerSupervisorInput) -> f32 {
        if !self.thermal_derating_active || !input.temperature_valid {
            return 1.0;
        }
        if input.temperature_c <= self.config.thermal_derating_start_c {
            return 1.0;
        }
        if input.temperature_c >= self.config.over_temperature_trip_c {
            return 0.0;
        }
        (self.config.over_temperature_trip_c - input.temperature_c)
            / (self.config.over_temperature_trip_c - self.config.thermal_derating_start_c)
    }

    fn temperature_in_measurement_range(&self, temperature_c: f32) -> bool {
        temperature_c.is_finite()
            && temperature_c >= self.config.minimum_valid_temperature_c
            && temperature_c <= self.config.maximum_valid_temperature_c
    }

    fn safe_output(
        &self,
        input: PowerSupervisorInput,
        active_fault_flags: u32,
        brake_requested: bool,
    ) -> PowerSupervisorOutput {
        self.safe_output_with_brake(input, active_fault_flags, brake_requested, 0.0)
    }

    fn safe_output_with_brake(
        &self,
        input: PowerSupervisorInput,
        active_fault_flags: u32,
        brake_requested: bool,
        brake_current_limit_a: f32,
    ) -> PowerSupervisorOutput {
        PowerSupervisorOutput {
            configured: self.configured,
            enabled: self.configured && self.config.enabled,
            drive_allowed: false,
            shutdown_requested: true,
            active_fault_flags,
            latched_fault_flags: self.latched_fault_flags,
            temperature_valid: input.temperature_valid,
            thermal_derating_active: self.thermal_derating_active,
            thermal_current_scale: 0.0,
            maximum_source_current_a: 0.0,
            maximum_regeneration_current_a: 0.0,
            requested_dc_current_a: if input.requested_dc_current_a.is_finite() {
                input.requested_dc_current_a
            } else {
                0.0
            },
            limited_dc_current_a: 0.0,
            source_limited: input.requested_dc_current_a.is_finite()
                && input.requested_dc_current_a > 0.0,
            regeneration_limited: input.requested_dc_current_a.is_finite()
                && input.requested_dc_current_a < 0.0,
            brake_requested,
            brake_current_limit_a,
        }
    }
}

fn inputs_are_finite(input: &PowerSupervisorInput) -> bool {
    input.temperature_c.is_finite()
        && input.dc_bus_voltage_v.is_finite()
        && input.requested_dc_current_a.is_finite()
}

fn all_finite(values: &[f32]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled_config() -> PowerSupervisorConfig {
        PowerSupervisorConfig {
            enabled: true,
            temperature_sensor_required: true,
            source_current_limit_a: 10.0,
            regeneration_allowed: false,
            ..PowerSupervisorConfig::default()
        }
    }

    fn nominal_input(requested_dc_current_a: f32) -> PowerSupervisorInput {
        PowerSupervisorInput {
            temperature_valid: true,
            temperature_c: 25.0,
            dc_bus_voltage_v: 12.0,
            requested_dc_current_a,
            source_available: true,
            sink_available: false,
            brake_available: false,
        }
    }

    #[test]
    fn default_is_safe_and_regeneration_is_not_authorised() {
        let mut supervisor = PowerSupervisor::default();
        let output = supervisor.step(nominal_input(-3.0));
        assert!(!output.configured);
        assert!(!output.enabled);
        assert!(!output.drive_allowed);
        assert!(output.shutdown_requested);
        assert_eq!(output.maximum_regeneration_current_a, 0.0);
        assert_eq!(output.limited_dc_current_a, 0.0);
        assert!(output.regeneration_limited);

        supervisor
            .configure(PowerSupervisorConfig::default())
            .unwrap();
        let output = supervisor.step(nominal_input(1.0));
        assert!(output.configured);
        assert!(!output.enabled);
        assert!(!output.drive_allowed);
        assert_eq!(output.limited_dc_current_a, 0.0);
    }

    #[test]
    fn configuration_validation_is_transactional_and_rejects_nonfinite_values() {
        let mut supervisor = PowerSupervisor::default();
        let mut config = enabled_config();
        config.bus_overvoltage_trip_v = f32::NAN;
        assert_eq!(
            supervisor.configure(config),
            Err(PowerSupervisorConfigError::InvalidBusWindow)
        );
        assert!(!supervisor.is_configured());

        let mut config = enabled_config();
        config.thermal_derating_release_c = config.thermal_derating_start_c;
        assert_eq!(
            config.validate(),
            Err(PowerSupervisorConfigError::InvalidTemperatureWindow)
        );
        let mut config = enabled_config();
        config.regeneration_allowed = true;
        assert_eq!(
            config.validate(),
            Err(PowerSupervisorConfigError::InvalidSinkCapability)
        );
    }

    #[test]
    fn source_limit_and_thermal_derating_are_deterministic_with_hysteresis() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();

        let output = supervisor.step(nominal_input(12.0));
        assert_eq!(output.maximum_source_current_a, 10.0);
        assert_eq!(output.limited_dc_current_a, 10.0);
        assert!(output.source_limited);

        let mut hot = nominal_input(8.0);
        hot.temperature_c = 90.0;
        let output = supervisor.step(hot);
        assert!(output.thermal_derating_active);
        assert_eq!(output.thermal_current_scale, 0.5);
        assert_eq!(output.maximum_source_current_a, 5.0);
        assert_eq!(output.limited_dc_current_a, 5.0);

        hot.temperature_c = 75.0;
        let output = supervisor.step(hot);
        assert!(output.thermal_derating_active);
        assert_eq!(output.thermal_current_scale, 1.0);
        hot.temperature_c = 70.0;
        assert!(!supervisor.step(hot).thermal_derating_active);
    }

    #[test]
    fn temperature_sensor_and_trip_faults_latch_until_explicit_safe_reset() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let mut input = nominal_input(2.0);
        input.temperature_valid = false;
        let output = supervisor.step(input);
        assert_eq!(output.latched_fault_flags, POWER_FAULT_TEMPERATURE_SENSOR);
        assert!(output.shutdown_requested);

        let recovered = nominal_input(0.0);
        assert_eq!(supervisor.step(recovered).active_fault_flags, 0);
        assert_eq!(supervisor.fault_flags(), POWER_FAULT_TEMPERATURE_SENSOR);
        supervisor.reset_faults(recovered).unwrap();
        assert_eq!(supervisor.fault_flags(), 0);

        let mut hot = nominal_input(0.0);
        hot.temperature_c = 100.0;
        assert_ne!(
            supervisor.step(hot).latched_fault_flags & POWER_FAULT_OVER_TEMPERATURE,
            0
        );
        hot.temperature_c = 75.0;
        assert_eq!(
            supervisor.reset_faults(hot),
            Err(PowerSupervisorResetError::UnsafeConditions)
        );
        hot.temperature_c = 70.0;
        supervisor.reset_faults(hot).unwrap();
    }

    #[test]
    fn sensor_range_is_checked_even_when_the_value_is_finite() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let mut input = nominal_input(1.0);
        input.temperature_c = -41.0;
        let output = supervisor.step(input);
        assert_ne!(
            output.latched_fault_flags & POWER_FAULT_TEMPERATURE_SENSOR,
            0
        );
        assert_eq!(output.limited_dc_current_a, 0.0);
    }

    #[test]
    fn bus_trips_are_inclusive_and_recovery_window_only_enables_explicit_reset() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let mut input = nominal_input(1.0);
        input.dc_bus_voltage_v = 7.0;
        assert_ne!(
            supervisor.step(input).latched_fault_flags & POWER_FAULT_BUS_UNDERVOLTAGE,
            0
        );
        input.requested_dc_current_a = 0.0;
        input.dc_bus_voltage_v = 7.9;
        assert_eq!(
            supervisor.reset_faults(input),
            Err(PowerSupervisorResetError::UnsafeConditions)
        );
        input.dc_bus_voltage_v = 8.0;
        supervisor.reset_faults(input).unwrap();

        input.dc_bus_voltage_v = 18.0;
        assert_ne!(
            supervisor.step(input).latched_fault_flags & POWER_FAULT_BUS_OVERVOLTAGE,
            0
        );
        input.dc_bus_voltage_v = 16.1;
        assert_eq!(
            supervisor.reset_faults(input),
            Err(PowerSupervisorResetError::UnsafeConditions)
        );
        input.dc_bus_voltage_v = 16.0;
        supervisor.reset_faults(input).unwrap();
    }

    #[test]
    fn unavailable_source_faults_only_when_positive_power_is_requested() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let mut input = nominal_input(0.0);
        input.source_available = false;
        let output = supervisor.step(input);
        assert_eq!(output.latched_fault_flags, 0);
        assert!(!output.drive_allowed);
        assert!(output.shutdown_requested);

        input.requested_dc_current_a = 1.0;
        let output = supervisor.step(input);
        assert_ne!(
            output.latched_fault_flags & POWER_FAULT_SOURCE_UNAVAILABLE,
            0
        );
    }

    #[test]
    fn regeneration_requires_opt_in_and_a_live_sink() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let output = supervisor.step(nominal_input(-4.0));
        assert_eq!(output.latched_fault_flags, 0);
        assert_eq!(output.maximum_regeneration_current_a, 0.0);
        assert_eq!(output.limited_dc_current_a, 0.0);
        assert!(output.regeneration_limited);

        let mut config = enabled_config();
        config.regeneration_allowed = true;
        config.sink_current_limit_a = 3.0;
        supervisor.configure(config).unwrap();
        let mut input = nominal_input(-4.0);
        input.sink_available = true;
        let output = supervisor.step(input);
        assert_eq!(output.maximum_regeneration_current_a, 3.0);
        assert_eq!(output.limited_dc_current_a, -3.0);
        assert!(output.regeneration_limited);
    }

    #[test]
    fn brake_request_has_voltage_hysteresis_and_bounded_regeneration_capacity() {
        let mut supervisor = PowerSupervisor::default();
        let mut config = enabled_config();
        config.regeneration_allowed = true;
        config.brake_resistor_available = true;
        config.brake_current_limit_a = 2.0;
        config.brake_release_voltage_v = 16.5;
        config.brake_engage_voltage_v = 17.0;
        supervisor.configure(config).unwrap();
        let mut input = nominal_input(-3.0);
        input.brake_available = true;

        input.dc_bus_voltage_v = 16.9;
        let output = supervisor.step(input);
        assert!(!output.brake_requested);
        assert_eq!(output.limited_dc_current_a, 0.0);
        input.dc_bus_voltage_v = 17.0;
        let output = supervisor.step(input);
        assert!(output.brake_requested);
        assert_eq!(output.maximum_regeneration_current_a, 2.0);
        assert_eq!(output.limited_dc_current_a, -2.0);
        input.dc_bus_voltage_v = 16.7;
        assert!(supervisor.step(input).brake_requested);
        input.dc_bus_voltage_v = 16.5;
        assert!(!supervisor.step(input).brake_requested);
    }

    #[test]
    fn overvoltage_fault_can_keep_a_protective_brake_request_but_never_regenerate() {
        let mut supervisor = PowerSupervisor::default();
        let mut config = enabled_config();
        config.regeneration_allowed = true;
        config.sink_current_limit_a = 1.0;
        config.brake_resistor_available = true;
        config.brake_current_limit_a = 2.0;
        config.brake_release_voltage_v = 16.5;
        config.brake_engage_voltage_v = 17.0;
        supervisor.configure(config).unwrap();
        let mut input = nominal_input(-3.0);
        input.sink_available = true;
        input.brake_available = true;
        input.dc_bus_voltage_v = 18.0;
        let output = supervisor.step(input);
        assert!(output.brake_requested);
        assert_eq!(output.brake_current_limit_a, 2.0);
        assert_eq!(output.maximum_regeneration_current_a, 0.0);
        assert_eq!(output.limited_dc_current_a, 0.0);
        assert!(output.shutdown_requested);
    }

    #[test]
    fn nonfinite_inputs_fail_closed_in_the_same_tick() {
        for index in 0..3 {
            let mut supervisor = PowerSupervisor::default();
            supervisor.configure(enabled_config()).unwrap();
            let mut input = nominal_input(1.0);
            match index {
                0 => input.temperature_c = f32::NAN,
                1 => input.dc_bus_voltage_v = f32::INFINITY,
                _ => input.requested_dc_current_a = f32::NEG_INFINITY,
            }
            let output = supervisor.step(input);
            assert_ne!(output.latched_fault_flags & POWER_FAULT_INVALID_INPUT, 0);
            assert!(!output.drive_allowed);
            assert!(output.shutdown_requested);
            assert_eq!(output.limited_dc_current_a, 0.0);
        }
    }

    #[test]
    fn failed_or_successful_reconfiguration_never_clears_a_fault() {
        let mut supervisor = PowerSupervisor::default();
        supervisor.configure(enabled_config()).unwrap();
        let mut input = nominal_input(0.0);
        input.dc_bus_voltage_v = 6.0;
        supervisor.step(input);
        let fault = supervisor.fault_flags();

        let mut invalid = enabled_config();
        invalid.source_current_limit_a = f32::NAN;
        assert!(supervisor.configure(invalid).is_err());
        assert_eq!(supervisor.fault_flags(), fault);
        supervisor.configure(enabled_config()).unwrap();
        assert_eq!(supervisor.fault_flags(), fault);
    }

    #[test]
    fn reset_requires_zero_demand_and_safe_source_temperature_and_bus() {
        let mut supervisor = PowerSupervisor::default();
        assert_eq!(
            supervisor.reset_faults(nominal_input(0.0)),
            Err(PowerSupervisorResetError::NotConfigured)
        );
        supervisor.configure(enabled_config()).unwrap();
        let mut bad = nominal_input(f32::NAN);
        assert_eq!(
            supervisor.reset_faults(bad),
            Err(PowerSupervisorResetError::InvalidInput)
        );
        bad = nominal_input(1.0);
        assert_eq!(
            supervisor.reset_faults(bad),
            Err(PowerSupervisorResetError::UnsafeConditions)
        );
        bad = nominal_input(0.0);
        bad.source_available = false;
        assert_eq!(
            supervisor.reset_faults(bad),
            Err(PowerSupervisorResetError::UnsafeConditions)
        );
    }

    #[test]
    fn known_fault_mask_covers_every_declared_bit() {
        assert_eq!(POWER_FAULT_KNOWN_MASK, 0x3f);
    }
}
