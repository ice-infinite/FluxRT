//! Fixed-step full-speed-range PMSM design plant.
//!
//! This module is a deterministic, file-system-free design model for software
//! sweeps. It is not a hardware truth model: switching ripple, magnetic
//! saturation, iron loss, temperature drift, sensor error and mechanical
//! resonance are intentionally absent.

use std::f32::consts::PI;

const TWO_PI: f32 = 2.0 * PI;
const SPMSM_RELATIVE_INDUCTANCE_TOLERANCE: f32 = 1.0e-5;

/// Rotor construction used to validate the d/q inductance model.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PmsmTopology {
    /// Surface magnets: `Ld` and `Lq` must be equal within a small tolerance.
    Surface,
    /// Interior magnets: saliency (`Ld != Lq`) is required.
    Interior,
}

/// Electrical and mechanical PMSM parameters, all in SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPmsmMotor {
    pub topology: PmsmTopology,
    pub pole_pairs: u32,
    pub stator_resistance_ohm: f32,
    pub inductance_d_h: f32,
    pub inductance_q_h: f32,
    pub permanent_magnet_flux_wb: f32,
    pub inertia_kg_m2: f32,
    pub viscous_friction_nm_s_rad: f32,
}

/// Averaged inverter voltage-vector limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicInverterLimit {
    /// When false, the requested d/q voltage is applied without a bus limit.
    pub enabled: bool,
    /// Maximum voltage-vector magnitude divided by instantaneous DC bus voltage.
    pub maximum_voltage_to_bus_ratio: f32,
    /// Fixed voltage reserve subtracted after applying the ratio `[V]`.
    pub reserve_voltage_v: f32,
}

impl Default for DynamicInverterLimit {
    fn default() -> Self {
        Self {
            enabled: true,
            maximum_voltage_to_bus_ratio: 1.0 / 3.0_f32.sqrt(),
            reserve_voltage_v: 0.0,
        }
    }
}

/// Capacitive DC-bus model fed by a Thevenin source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicDcBus {
    /// Source open-circuit voltage `[V]`; runtime steps use the plant setter.
    pub source_voltage_v: f32,
    /// Source/wiring resistance `[ohm]`.
    pub source_resistance_ohm: f32,
    /// DC-link capacitance `[F]`.
    pub capacitance_f: f32,
    /// Numerical/physical lower rail `[V]`, strictly positive.
    pub minimum_voltage_v: f32,
    /// Numerical/physical upper rail `[V]`.
    pub maximum_voltage_v: f32,
    /// Whether negative source current may return regenerated energy upstream.
    pub source_can_sink: bool,
}

/// Immutable plant configuration. `step_s` is the only integration time base.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPmsmConfig {
    pub step_s: f32,
    pub motor: DynamicPmsmMotor,
    pub inverter: DynamicInverterLimit,
    pub dc_bus: DynamicDcBus,
}

/// Integrated plant state in the rotor d/q reference frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DynamicPmsmState {
    pub current_d_a: f32,
    pub current_q_a: f32,
    pub mechanical_speed_rad_s: f32,
    pub mechanical_angle_rad: f32,
    pub dc_bus_voltage_v: f32,
}

impl DynamicPmsmState {
    /// Rest state with a caller-selected initial DC bus voltage.
    pub fn at_rest(dc_bus_voltage_v: f32) -> Self {
        Self {
            dc_bus_voltage_v,
            ..Self::default()
        }
    }
}

/// Zero-order-held d/q voltage request for one fixed plant step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DynamicPmsmInput {
    pub voltage_d_v: f32,
    pub voltage_q_v: f32,
}

/// Indicates whether the bus state hit a configured rail during a step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DcBusClamp {
    None,
    Minimum,
    Maximum,
}

/// Finite diagnostic snapshot returned after one committed fixed step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPmsmOutput {
    pub step_index: u64,
    pub state: DynamicPmsmState,
    pub requested_voltage_d_v: f32,
    pub requested_voltage_q_v: f32,
    pub applied_voltage_d_v: f32,
    pub applied_voltage_q_v: f32,
    pub voltage_limited: bool,
    pub electrical_angle_rad: f32,
    pub electrical_speed_rad_s: f32,
    pub electromagnetic_torque_nm: f32,
    pub load_torque_nm: f32,
    pub ac_electrical_power_w: f32,
    pub inverter_dc_current_a: f32,
    pub source_current_a: f32,
    pub bus_clamp: DcBusClamp,
}

/// Fail-closed validation and numerical errors.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DynamicPmsmError {
    InvalidConfig(&'static str),
    InvalidInitialState(&'static str),
    InvalidInput(&'static str),
    NumericalDivergence,
    FaultLatched,
}

/// One no-I/O operating point used by the generic sweep helper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPmsmOperatingPoint {
    pub initial_state: DynamicPmsmState,
    pub source_voltage_v: f32,
    pub load_torque_nm: f32,
    pub input: DynamicPmsmInput,
    pub step_count: u32,
}

/// Compact result of one in-memory speed/bus/load operating-point run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DynamicPmsmSweepResult {
    pub point: DynamicPmsmOperatingPoint,
    pub final_output: DynamicPmsmOutput,
    pub peak_current_magnitude_a: f32,
    pub minimum_bus_voltage_v: f32,
    pub maximum_bus_voltage_v: f32,
    pub voltage_limited_steps: u32,
}

#[derive(Clone, Copy, Debug)]
struct Derivative {
    current_d_a_s: f32,
    current_q_a_s: f32,
    mechanical_speed_rad_s2: f32,
    mechanical_angle_rad_s: f32,
    dc_bus_voltage_v_s: f32,
}

/// Deterministic RK4 plant with sticky fail-closed numerical state.
#[derive(Clone, Copy, Debug)]
pub struct DynamicPmsmPlant {
    config: DynamicPmsmConfig,
    state: DynamicPmsmState,
    source_voltage_v: f32,
    load_torque_nm: f32,
    step_index: u64,
    fault: Option<DynamicPmsmError>,
}

impl DynamicPmsmPlant {
    pub fn new(
        config: DynamicPmsmConfig,
        mut initial_state: DynamicPmsmState,
    ) -> Result<Self, DynamicPmsmError> {
        validate_config(config)?;
        validate_state(config.dc_bus, initial_state)?;
        initial_state.mechanical_angle_rad = initial_state.mechanical_angle_rad.rem_euclid(TWO_PI);
        Ok(Self {
            config,
            state: initial_state,
            source_voltage_v: config.dc_bus.source_voltage_v,
            load_torque_nm: 0.0,
            step_index: 0,
            fault: None,
        })
    }

    pub fn config(&self) -> DynamicPmsmConfig {
        self.config
    }

    pub fn state(&self) -> DynamicPmsmState {
        self.state
    }

    pub fn source_voltage_v(&self) -> f32 {
        self.source_voltage_v
    }

    pub fn load_torque_nm(&self) -> f32 {
        self.load_torque_nm
    }

    pub fn fault(&self) -> Option<DynamicPmsmError> {
        self.fault
    }

    pub fn electrical_angle_rad(&self) -> f32 {
        (self.state.mechanical_angle_rad * self.config.motor.pole_pairs as f32).rem_euclid(TWO_PI)
    }

    pub fn electromagnetic_torque_nm(&self) -> f32 {
        electromagnetic_torque(self.config.motor, self.state)
    }

    /// Apply a signed load torque step. Positive torque is subtracted from the
    /// motor torque; a reverse-direction resisting load is therefore negative.
    pub fn set_load_torque_nm(&mut self, load_torque_nm: f32) -> Result<(), DynamicPmsmError> {
        if self.fault.is_some() {
            return Err(DynamicPmsmError::FaultLatched);
        }
        if !load_torque_nm.is_finite() {
            return self.latch_invalid_input("load torque must be finite");
        }
        self.load_torque_nm = load_torque_nm;
        Ok(())
    }

    /// Apply a DC source-voltage step without touching the capacitor state.
    pub fn set_source_voltage_v(&mut self, source_voltage_v: f32) -> Result<(), DynamicPmsmError> {
        if self.fault.is_some() {
            return Err(DynamicPmsmError::FaultLatched);
        }
        if !source_voltage_v.is_finite()
            || !(self.config.dc_bus.minimum_voltage_v..=self.config.dc_bus.maximum_voltage_v)
                .contains(&source_voltage_v)
        {
            return self.latch_invalid_input("source voltage outside bus rails");
        }
        self.source_voltage_v = source_voltage_v;
        Ok(())
    }

    /// Advances exactly one configured fixed step. Inputs are held constant for
    /// the complete RK4 integration interval.
    pub fn step(&mut self, input: DynamicPmsmInput) -> Result<DynamicPmsmOutput, DynamicPmsmError> {
        if self.fault.is_some() {
            return Err(DynamicPmsmError::FaultLatched);
        }
        if !input.voltage_d_v.is_finite() || !input.voltage_q_v.is_finite() {
            return self.latch_invalid_input("d/q voltage request must be finite");
        }

        let (applied_d_v, applied_q_v, voltage_limited) =
            match apply_voltage_limit(self.config.inverter, self.state.dc_bus_voltage_v, input) {
                Ok(applied) => applied,
                Err(error) => {
                    self.fault = Some(error);
                    return Err(error);
                }
            };
        let dt = self.config.step_s;
        let k1 = self.derivative(self.state, applied_d_v, applied_q_v);
        let k2 = self.derivative(
            add_scaled(self.state, k1, 0.5 * dt),
            applied_d_v,
            applied_q_v,
        );
        let k3 = self.derivative(
            add_scaled(self.state, k2, 0.5 * dt),
            applied_d_v,
            applied_q_v,
        );
        let k4 = self.derivative(add_scaled(self.state, k3, dt), applied_d_v, applied_q_v);
        let mut next = combine_rk4(self.state, k1, k2, k3, k4, dt);
        let bus_clamp = if next.dc_bus_voltage_v < self.config.dc_bus.minimum_voltage_v {
            next.dc_bus_voltage_v = self.config.dc_bus.minimum_voltage_v;
            DcBusClamp::Minimum
        } else if next.dc_bus_voltage_v > self.config.dc_bus.maximum_voltage_v {
            next.dc_bus_voltage_v = self.config.dc_bus.maximum_voltage_v;
            DcBusClamp::Maximum
        } else {
            DcBusClamp::None
        };
        next.mechanical_angle_rad = next.mechanical_angle_rad.rem_euclid(TWO_PI);
        if !state_is_finite(next) {
            self.fault = Some(DynamicPmsmError::NumericalDivergence);
            return Err(DynamicPmsmError::NumericalDivergence);
        }

        let torque_nm = electromagnetic_torque(self.config.motor, next);
        let electrical_speed_rad_s =
            next.mechanical_speed_rad_s * self.config.motor.pole_pairs as f32;
        let electrical_angle_rad =
            (next.mechanical_angle_rad * self.config.motor.pole_pairs as f32).rem_euclid(TWO_PI);
        let ac_power_w = 1.5 * (applied_d_v * next.current_d_a + applied_q_v * next.current_q_a);
        let inverter_dc_current_a = ac_power_w / next.dc_bus_voltage_v;
        let source_current_a = source_current(
            self.config.dc_bus,
            self.source_voltage_v,
            next.dc_bus_voltage_v,
        );
        let output = DynamicPmsmOutput {
            step_index: self.step_index.wrapping_add(1),
            state: next,
            requested_voltage_d_v: input.voltage_d_v,
            requested_voltage_q_v: input.voltage_q_v,
            applied_voltage_d_v: applied_d_v,
            applied_voltage_q_v: applied_q_v,
            voltage_limited,
            electrical_angle_rad,
            electrical_speed_rad_s,
            electromagnetic_torque_nm: torque_nm,
            load_torque_nm: self.load_torque_nm,
            ac_electrical_power_w: ac_power_w,
            inverter_dc_current_a,
            source_current_a,
            bus_clamp,
        };
        if !output_is_finite(output) {
            self.fault = Some(DynamicPmsmError::NumericalDivergence);
            return Err(DynamicPmsmError::NumericalDivergence);
        }

        self.state = next;
        self.step_index = output.step_index;
        Ok(output)
    }

    fn derivative(
        &self,
        state: DynamicPmsmState,
        voltage_d_v: f32,
        voltage_q_v: f32,
    ) -> Derivative {
        let motor = self.config.motor;
        let electrical_speed_rad_s = state.mechanical_speed_rad_s * motor.pole_pairs as f32;
        let current_d_a_s = (voltage_d_v - motor.stator_resistance_ohm * state.current_d_a
            + electrical_speed_rad_s * motor.inductance_q_h * state.current_q_a)
            / motor.inductance_d_h;
        let current_q_a_s = (voltage_q_v
            - motor.stator_resistance_ohm * state.current_q_a
            - electrical_speed_rad_s
                * (motor.inductance_d_h * state.current_d_a + motor.permanent_magnet_flux_wb))
            / motor.inductance_q_h;
        let torque_nm = electromagnetic_torque(motor, state);
        let mechanical_speed_rad_s2 = (torque_nm
            - self.load_torque_nm
            - motor.viscous_friction_nm_s_rad * state.mechanical_speed_rad_s)
            / motor.inertia_kg_m2;
        let ac_power_w = 1.5 * (voltage_d_v * state.current_d_a + voltage_q_v * state.current_q_a);
        let inverter_dc_current_a = ac_power_w / state.dc_bus_voltage_v;
        let source_current_a = source_current(
            self.config.dc_bus,
            self.source_voltage_v,
            state.dc_bus_voltage_v,
        );
        Derivative {
            current_d_a_s,
            current_q_a_s,
            mechanical_speed_rad_s2,
            mechanical_angle_rad_s: state.mechanical_speed_rad_s,
            dc_bus_voltage_v_s: (source_current_a - inverter_dc_current_a)
                / self.config.dc_bus.capacitance_f,
        }
    }

    fn latch_invalid_input<T>(&mut self, detail: &'static str) -> Result<T, DynamicPmsmError> {
        let error = DynamicPmsmError::InvalidInput(detail);
        self.fault = Some(error);
        Err(error)
    }
}

/// Runs independent fixed-input operating points entirely in memory.
pub fn sweep_operating_points(
    config: DynamicPmsmConfig,
    points: &[DynamicPmsmOperatingPoint],
) -> Vec<Result<DynamicPmsmSweepResult, DynamicPmsmError>> {
    points
        .iter()
        .copied()
        .map(|point| run_operating_point(config, point))
        .collect()
}

/// Runs one fixed-input operating point without file or process I/O.
pub fn run_operating_point(
    mut config: DynamicPmsmConfig,
    point: DynamicPmsmOperatingPoint,
) -> Result<DynamicPmsmSweepResult, DynamicPmsmError> {
    if point.step_count == 0 {
        return Err(DynamicPmsmError::InvalidInput(
            "operating point step count must be non-zero",
        ));
    }
    config.dc_bus.source_voltage_v = point.source_voltage_v;
    let mut plant = DynamicPmsmPlant::new(config, point.initial_state)?;
    plant.set_load_torque_nm(point.load_torque_nm)?;
    let mut final_output = plant.step(point.input)?;
    let mut peak_current_magnitude_a = current_magnitude(
        final_output.state.current_d_a,
        final_output.state.current_q_a,
    );
    let mut minimum_bus_voltage_v = final_output.state.dc_bus_voltage_v;
    let mut maximum_bus_voltage_v = final_output.state.dc_bus_voltage_v;
    let mut voltage_limited_steps = u32::from(final_output.voltage_limited);
    for _ in 1..point.step_count {
        final_output = plant.step(point.input)?;
        peak_current_magnitude_a = peak_current_magnitude_a.max(current_magnitude(
            final_output.state.current_d_a,
            final_output.state.current_q_a,
        ));
        minimum_bus_voltage_v = minimum_bus_voltage_v.min(final_output.state.dc_bus_voltage_v);
        maximum_bus_voltage_v = maximum_bus_voltage_v.max(final_output.state.dc_bus_voltage_v);
        voltage_limited_steps += u32::from(final_output.voltage_limited);
    }
    Ok(DynamicPmsmSweepResult {
        point,
        final_output,
        peak_current_magnitude_a,
        minimum_bus_voltage_v,
        maximum_bus_voltage_v,
        voltage_limited_steps,
    })
}

fn validate_config(config: DynamicPmsmConfig) -> Result<(), DynamicPmsmError> {
    let motor = config.motor;
    let bus = config.dc_bus;
    let inverter = config.inverter;
    if !config.step_s.is_finite() || config.step_s <= 0.0 {
        return Err(DynamicPmsmError::InvalidConfig(
            "fixed step must be finite and positive",
        ));
    }
    if motor.pole_pairs == 0
        || !motor.stator_resistance_ohm.is_finite()
        || motor.stator_resistance_ohm <= 0.0
        || !motor.inductance_d_h.is_finite()
        || motor.inductance_d_h <= 0.0
        || !motor.inductance_q_h.is_finite()
        || motor.inductance_q_h <= 0.0
        || !motor.permanent_magnet_flux_wb.is_finite()
        || motor.permanent_magnet_flux_wb <= 0.0
        || !motor.inertia_kg_m2.is_finite()
        || motor.inertia_kg_m2 <= 0.0
        || !motor.viscous_friction_nm_s_rad.is_finite()
        || motor.viscous_friction_nm_s_rad < 0.0
    {
        return Err(DynamicPmsmError::InvalidConfig(
            "motor parameters must be finite and physical",
        ));
    }
    let inductance_scale = motor.inductance_d_h.max(motor.inductance_q_h);
    let relative_saliency = (motor.inductance_d_h - motor.inductance_q_h).abs() / inductance_scale;
    if (motor.topology == PmsmTopology::Surface
        && relative_saliency > SPMSM_RELATIVE_INDUCTANCE_TOLERANCE)
        || (motor.topology == PmsmTopology::Interior
            && relative_saliency <= SPMSM_RELATIVE_INDUCTANCE_TOLERANCE)
    {
        return Err(DynamicPmsmError::InvalidConfig(
            "topology and d/q inductances disagree",
        ));
    }
    if !bus.source_voltage_v.is_finite()
        || !bus.minimum_voltage_v.is_finite()
        || bus.minimum_voltage_v <= 0.0
        || !bus.maximum_voltage_v.is_finite()
        || bus.maximum_voltage_v <= bus.minimum_voltage_v
        || !(bus.minimum_voltage_v..=bus.maximum_voltage_v).contains(&bus.source_voltage_v)
        || !bus.source_resistance_ohm.is_finite()
        || bus.source_resistance_ohm <= 0.0
        || !bus.capacitance_f.is_finite()
        || bus.capacitance_f <= 0.0
    {
        return Err(DynamicPmsmError::InvalidConfig(
            "DC bus parameters must be finite and physical",
        ));
    }
    if !inverter.maximum_voltage_to_bus_ratio.is_finite()
        || inverter.maximum_voltage_to_bus_ratio <= 0.0
        || inverter.maximum_voltage_to_bus_ratio > 1.0
        || !inverter.reserve_voltage_v.is_finite()
        || inverter.reserve_voltage_v < 0.0
    {
        return Err(DynamicPmsmError::InvalidConfig(
            "inverter voltage limit must be finite and bounded",
        ));
    }
    Ok(())
}

fn validate_state(bus: DynamicDcBus, state: DynamicPmsmState) -> Result<(), DynamicPmsmError> {
    if !state_is_finite(state) {
        return Err(DynamicPmsmError::InvalidInitialState(
            "initial state must be finite",
        ));
    }
    if !(bus.minimum_voltage_v..=bus.maximum_voltage_v).contains(&state.dc_bus_voltage_v) {
        return Err(DynamicPmsmError::InvalidInitialState(
            "initial bus voltage outside configured rails",
        ));
    }
    Ok(())
}

fn apply_voltage_limit(
    inverter: DynamicInverterLimit,
    bus_voltage_v: f32,
    input: DynamicPmsmInput,
) -> Result<(f32, f32, bool), DynamicPmsmError> {
    if !inverter.enabled {
        return Ok((input.voltage_d_v, input.voltage_q_v, false));
    }
    let maximum_voltage_v = (bus_voltage_v * inverter.maximum_voltage_to_bus_ratio
        - inverter.reserve_voltage_v)
        .max(0.0);
    let requested_magnitude_v = current_magnitude(input.voltage_d_v, input.voltage_q_v);
    if !maximum_voltage_v.is_finite() || !requested_magnitude_v.is_finite() {
        return Err(DynamicPmsmError::NumericalDivergence);
    }
    if requested_magnitude_v <= maximum_voltage_v || requested_magnitude_v == 0.0 {
        Ok((input.voltage_d_v, input.voltage_q_v, false))
    } else {
        let scale = maximum_voltage_v / requested_magnitude_v;
        Ok((input.voltage_d_v * scale, input.voltage_q_v * scale, true))
    }
}

fn electromagnetic_torque(motor: DynamicPmsmMotor, state: DynamicPmsmState) -> f32 {
    1.5 * motor.pole_pairs as f32
        * (motor.permanent_magnet_flux_wb * state.current_q_a
            + (motor.inductance_d_h - motor.inductance_q_h) * state.current_d_a * state.current_q_a)
}

fn source_current(bus: DynamicDcBus, source_voltage_v: f32, bus_voltage_v: f32) -> f32 {
    let current_a = (source_voltage_v - bus_voltage_v) / bus.source_resistance_ohm;
    if bus.source_can_sink {
        current_a
    } else {
        current_a.max(0.0)
    }
}

fn add_scaled(state: DynamicPmsmState, derivative: Derivative, scale: f32) -> DynamicPmsmState {
    DynamicPmsmState {
        current_d_a: state.current_d_a + derivative.current_d_a_s * scale,
        current_q_a: state.current_q_a + derivative.current_q_a_s * scale,
        mechanical_speed_rad_s: state.mechanical_speed_rad_s
            + derivative.mechanical_speed_rad_s2 * scale,
        mechanical_angle_rad: state.mechanical_angle_rad
            + derivative.mechanical_angle_rad_s * scale,
        dc_bus_voltage_v: state.dc_bus_voltage_v + derivative.dc_bus_voltage_v_s * scale,
    }
}

fn combine_rk4(
    state: DynamicPmsmState,
    k1: Derivative,
    k2: Derivative,
    k3: Derivative,
    k4: Derivative,
    dt: f32,
) -> DynamicPmsmState {
    let scale = dt / 6.0;
    DynamicPmsmState {
        current_d_a: state.current_d_a
            + scale
                * (k1.current_d_a_s
                    + 2.0 * k2.current_d_a_s
                    + 2.0 * k3.current_d_a_s
                    + k4.current_d_a_s),
        current_q_a: state.current_q_a
            + scale
                * (k1.current_q_a_s
                    + 2.0 * k2.current_q_a_s
                    + 2.0 * k3.current_q_a_s
                    + k4.current_q_a_s),
        mechanical_speed_rad_s: state.mechanical_speed_rad_s
            + scale
                * (k1.mechanical_speed_rad_s2
                    + 2.0 * k2.mechanical_speed_rad_s2
                    + 2.0 * k3.mechanical_speed_rad_s2
                    + k4.mechanical_speed_rad_s2),
        mechanical_angle_rad: state.mechanical_angle_rad
            + scale
                * (k1.mechanical_angle_rad_s
                    + 2.0 * k2.mechanical_angle_rad_s
                    + 2.0 * k3.mechanical_angle_rad_s
                    + k4.mechanical_angle_rad_s),
        dc_bus_voltage_v: state.dc_bus_voltage_v
            + scale
                * (k1.dc_bus_voltage_v_s
                    + 2.0 * k2.dc_bus_voltage_v_s
                    + 2.0 * k3.dc_bus_voltage_v_s
                    + k4.dc_bus_voltage_v_s),
    }
}

fn state_is_finite(state: DynamicPmsmState) -> bool {
    state.current_d_a.is_finite()
        && state.current_q_a.is_finite()
        && state.mechanical_speed_rad_s.is_finite()
        && state.mechanical_angle_rad.is_finite()
        && state.dc_bus_voltage_v.is_finite()
}

fn output_is_finite(output: DynamicPmsmOutput) -> bool {
    state_is_finite(output.state)
        && output.requested_voltage_d_v.is_finite()
        && output.requested_voltage_q_v.is_finite()
        && output.applied_voltage_d_v.is_finite()
        && output.applied_voltage_q_v.is_finite()
        && output.electrical_angle_rad.is_finite()
        && output.electrical_speed_rad_s.is_finite()
        && output.electromagnetic_torque_nm.is_finite()
        && output.load_torque_nm.is_finite()
        && output.ac_electrical_power_w.is_finite()
        && output.inverter_dc_current_a.is_finite()
        && output.source_current_a.is_finite()
}

fn current_magnitude(d: f32, q: f32) -> f32 {
    d.hypot(q)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn motor(topology: PmsmTopology) -> DynamicPmsmMotor {
        DynamicPmsmMotor {
            topology,
            pole_pairs: 7,
            stator_resistance_ohm: 5.0,
            inductance_d_h: 1.0e-3,
            inductance_q_h: if topology == PmsmTopology::Surface {
                1.0e-3
            } else {
                1.8e-3
            },
            permanent_magnet_flux_wb: 0.025,
            inertia_kg_m2: 5.0e-5,
            viscous_friction_nm_s_rad: 1.0e-5,
        }
    }

    fn config(topology: PmsmTopology) -> DynamicPmsmConfig {
        DynamicPmsmConfig {
            step_s: 10.0e-6,
            motor: motor(topology),
            inverter: DynamicInverterLimit::default(),
            dc_bus: DynamicDcBus {
                source_voltage_v: 24.0,
                source_resistance_ohm: 0.2,
                capacitance_f: 0.02,
                minimum_voltage_v: 5.0,
                maximum_voltage_v: 40.0,
                source_can_sink: false,
            },
        }
    }

    #[test]
    fn topology_validation_accepts_spmsm_and_salient_ipmsm() {
        assert!(DynamicPmsmPlant::new(
            config(PmsmTopology::Surface),
            DynamicPmsmState::at_rest(24.0)
        )
        .is_ok());
        assert!(DynamicPmsmPlant::new(
            config(PmsmTopology::Interior),
            DynamicPmsmState::at_rest(24.0)
        )
        .is_ok());

        let mut invalid = config(PmsmTopology::Surface);
        invalid.motor.inductance_q_h *= 1.2;
        assert!(matches!(
            DynamicPmsmPlant::new(invalid, DynamicPmsmState::at_rest(24.0)),
            Err(DynamicPmsmError::InvalidConfig(_))
        ));
    }

    #[test]
    fn positive_and_negative_q_voltage_drive_opposite_directions() {
        let cfg = config(PmsmTopology::Surface);
        let mut forward = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        let mut reverse = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        for _ in 0..5_000 {
            forward
                .step(DynamicPmsmInput {
                    voltage_q_v: 4.0,
                    ..DynamicPmsmInput::default()
                })
                .unwrap();
            reverse
                .step(DynamicPmsmInput {
                    voltage_q_v: -4.0,
                    ..DynamicPmsmInput::default()
                })
                .unwrap();
        }
        assert!(forward.state().mechanical_speed_rad_s > 0.0);
        assert!(reverse.state().mechanical_speed_rad_s < 0.0);
        assert!(
            (forward.state().mechanical_speed_rad_s + reverse.state().mechanical_speed_rad_s).abs()
                < 1.0e-3
        );
        assert!((0.0..TWO_PI).contains(&forward.state().mechanical_angle_rad));
        assert!((0.0..TWO_PI).contains(&reverse.state().mechanical_angle_rad));
    }

    #[test]
    fn ipmsm_torque_contains_reluctance_term() {
        let cfg = config(PmsmTopology::Interior);
        let state = DynamicPmsmState {
            current_d_a: -2.0,
            current_q_a: 3.0,
            dc_bus_voltage_v: 24.0,
            ..DynamicPmsmState::default()
        };
        let plant = DynamicPmsmPlant::new(cfg, state).unwrap();
        let expected = 1.5
            * cfg.motor.pole_pairs as f32
            * (cfg.motor.permanent_magnet_flux_wb * state.current_q_a
                + (cfg.motor.inductance_d_h - cfg.motor.inductance_q_h)
                    * state.current_d_a
                    * state.current_q_a);
        assert!((plant.electromagnetic_torque_nm() - expected).abs() < 1.0e-6);
    }

    #[test]
    fn inverter_limit_scales_vector_to_instantaneous_bus_limit() {
        let cfg = config(PmsmTopology::Surface);
        let mut plant = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        let output = plant
            .step(DynamicPmsmInput {
                voltage_d_v: 30.0,
                voltage_q_v: 40.0,
            })
            .unwrap();
        let applied = current_magnitude(output.applied_voltage_d_v, output.applied_voltage_q_v);
        let expected = 24.0 * cfg.inverter.maximum_voltage_to_bus_ratio;
        assert!(output.voltage_limited);
        assert!((applied - expected).abs() < 1.0e-5);
        assert!((output.applied_voltage_d_v / output.applied_voltage_q_v - 0.75).abs() < 1.0e-6);
    }

    #[test]
    fn load_and_source_steps_change_mechanical_and_bus_states() {
        let cfg = config(PmsmTopology::Surface);
        let mut baseline = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        let mut disturbed = baseline;
        let drive = DynamicPmsmInput {
            voltage_q_v: 4.0,
            ..DynamicPmsmInput::default()
        };
        for _ in 0..1_000 {
            baseline.step(drive).unwrap();
            disturbed.step(drive).unwrap();
        }
        disturbed.set_load_torque_nm(0.08).unwrap();
        disturbed.set_source_voltage_v(18.0).unwrap();
        for _ in 0..2_000 {
            baseline.step(drive).unwrap();
            disturbed.step(drive).unwrap();
        }
        assert!(disturbed.state().mechanical_speed_rad_s < baseline.state().mechanical_speed_rad_s);
        assert!(disturbed.state().dc_bus_voltage_v < baseline.state().dc_bus_voltage_v);
    }

    #[test]
    fn regenerative_electrical_power_charges_a_non_sinking_bus() {
        let cfg = config(PmsmTopology::Surface);
        let initial = DynamicPmsmState {
            current_q_a: 1.0,
            dc_bus_voltage_v: 24.0,
            ..DynamicPmsmState::default()
        };
        let mut plant = DynamicPmsmPlant::new(cfg, initial).unwrap();
        let output = plant
            .step(DynamicPmsmInput {
                voltage_q_v: -1.0,
                ..DynamicPmsmInput::default()
            })
            .unwrap();
        assert!(output.ac_electrical_power_w < 0.0);
        assert!(output.inverter_dc_current_a < 0.0);
        assert!(output.state.dc_bus_voltage_v > initial.dc_bus_voltage_v);
    }

    #[test]
    fn invalid_or_divergent_input_latches_without_committing_state() {
        let cfg = config(PmsmTopology::Surface);
        let mut plant = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        let before = plant.state();
        assert!(matches!(
            plant.step(DynamicPmsmInput {
                voltage_d_v: f32::NAN,
                voltage_q_v: 0.0,
            }),
            Err(DynamicPmsmError::InvalidInput(_))
        ));
        assert_eq!(plant.state(), before);
        assert!(matches!(
            plant.step(DynamicPmsmInput::default()),
            Err(DynamicPmsmError::FaultLatched)
        ));

        let mut unlimited = cfg;
        unlimited.inverter.enabled = false;
        let mut plant = DynamicPmsmPlant::new(unlimited, DynamicPmsmState::at_rest(24.0)).unwrap();
        let before = plant.state();
        assert_eq!(
            plant.step(DynamicPmsmInput {
                voltage_d_v: f32::MAX,
                voltage_q_v: f32::MAX,
            }),
            Err(DynamicPmsmError::NumericalDivergence)
        );
        assert_eq!(plant.state(), before);
        assert_eq!(plant.fault(), Some(DynamicPmsmError::NumericalDivergence));

        let mut limited = DynamicPmsmPlant::new(cfg, DynamicPmsmState::at_rest(24.0)).unwrap();
        assert_eq!(
            limited.step(DynamicPmsmInput {
                voltage_d_v: f32::MAX,
                voltage_q_v: f32::MAX,
            }),
            Err(DynamicPmsmError::NumericalDivergence)
        );
        assert_eq!(limited.fault(), Some(DynamicPmsmError::NumericalDivergence));
    }

    #[test]
    fn in_memory_sweep_covers_speed_bus_and_load_axes() {
        let cfg = config(PmsmTopology::Interior);
        let points = [
            DynamicPmsmOperatingPoint {
                initial_state: DynamicPmsmState {
                    mechanical_speed_rad_s: 100.0,
                    dc_bus_voltage_v: 24.0,
                    ..DynamicPmsmState::default()
                },
                source_voltage_v: 24.0,
                load_torque_nm: 0.01,
                input: DynamicPmsmInput {
                    voltage_q_v: 8.0,
                    ..DynamicPmsmInput::default()
                },
                step_count: 200,
            },
            DynamicPmsmOperatingPoint {
                initial_state: DynamicPmsmState {
                    mechanical_speed_rad_s: -100.0,
                    dc_bus_voltage_v: 18.0,
                    ..DynamicPmsmState::default()
                },
                source_voltage_v: 18.0,
                load_torque_nm: -0.02,
                input: DynamicPmsmInput {
                    voltage_q_v: -8.0,
                    ..DynamicPmsmInput::default()
                },
                step_count: 200,
            },
        ];
        let results = sweep_operating_points(cfg, &points);
        assert_eq!(results.len(), points.len());
        for result in results {
            let result = result.unwrap();
            assert!(state_is_finite(result.final_output.state));
            assert!(result.peak_current_magnitude_a.is_finite());
            assert!(result.minimum_bus_voltage_v.is_finite());
            assert!(result.maximum_bus_voltage_v.is_finite());
        }
    }
}
