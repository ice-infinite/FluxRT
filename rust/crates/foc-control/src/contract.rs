//! Product-level Axis, command and telemetry contract shared by target and host.
//!
//! This module is deliberately independent from the current 12 kHz controller
//! state machine. It freezes the public vocabulary used by the later command
//! arbiter, configuration system, simulations and host tools without changing
//! the existing realtime path.

use core::mem::size_of;

pub const PRODUCT_CONTRACT_VERSION: u32 = 0x0001_0000;
pub const PRODUCT_COMMAND_VERSION: u32 = 1;
pub const PRODUCT_FEEDBACK_VERSION: u32 = 1;
pub const PRODUCT_TELEMETRY_VERSION: u32 = 1;
pub const PRODUCT_FAULT_SNAPSHOT_VERSION: u32 = 1;

pub const PRODUCT_COMMAND_FLAG_CURRENT_LIMIT: u32 = 1 << 0;
pub const PRODUCT_COMMAND_FLAG_TORQUE_LIMIT: u32 = 1 << 1;
pub const PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT: u32 = 1 << 2;
pub const PRODUCT_COMMAND_FLAG_KNOWN_MASK: u32 = PRODUCT_COMMAND_FLAG_CURRENT_LIMIT
    | PRODUCT_COMMAND_FLAG_TORQUE_LIMIT
    | PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;

pub const PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION: u32 = 1 << 0;
pub const PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION: u32 = 1 << 1;
pub const PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY: u32 = 1 << 2;
pub const PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE: u32 = 1 << 3;
pub const PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY: u32 = 1 << 4;
pub const PRODUCT_FEEDBACK_VALID_KNOWN_MASK: u32 = PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION
    | PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION
    | PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY
    | PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE
    | PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY;

pub const PRODUCT_FEEDBACK_QUALITY_CALIBRATED: u32 = 1 << 0;
pub const PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND: u32 = 1 << 1;
pub const PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID: u32 = 1 << 2;
pub const PRODUCT_FEEDBACK_QUALITY_STALE: u32 = 1 << 3;
pub const PRODUCT_FEEDBACK_QUALITY_DEGRADED: u32 = 1 << 4;
pub const PRODUCT_FEEDBACK_QUALITY_KNOWN_MASK: u32 = PRODUCT_FEEDBACK_QUALITY_CALIBRATED
    | PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND
    | PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID
    | PRODUCT_FEEDBACK_QUALITY_STALE
    | PRODUCT_FEEDBACK_QUALITY_DEGRADED;

pub const PRODUCT_TELEMETRY_VALID_POSITION_REFERENCE: u32 = 1 << 0;
pub const PRODUCT_TELEMETRY_VALID_POSITION_MEASURED: u32 = 1 << 1;
pub const PRODUCT_TELEMETRY_VALID_POSITION_ESTIMATED: u32 = 1 << 2;
pub const PRODUCT_TELEMETRY_VALID_VELOCITY_REFERENCE: u32 = 1 << 3;
pub const PRODUCT_TELEMETRY_VALID_VELOCITY_MEASURED: u32 = 1 << 4;
pub const PRODUCT_TELEMETRY_VALID_VELOCITY_ESTIMATED: u32 = 1 << 5;
pub const PRODUCT_TELEMETRY_VALID_TORQUE_REFERENCE: u32 = 1 << 6;
pub const PRODUCT_TELEMETRY_VALID_TORQUE_ESTIMATED: u32 = 1 << 7;
pub const PRODUCT_TELEMETRY_VALID_CURRENT_REFERENCE: u32 = 1 << 8;
pub const PRODUCT_TELEMETRY_VALID_CURRENT_MEASURED: u32 = 1 << 9;
pub const PRODUCT_TELEMETRY_VALID_VOLTAGE_COMMAND: u32 = 1 << 10;
pub const PRODUCT_TELEMETRY_VALID_DC_BUS_VOLTAGE: u32 = 1 << 11;
pub const PRODUCT_TELEMETRY_VALID_MOTOR_TEMPERATURE: u32 = 1 << 12;
pub const PRODUCT_TELEMETRY_VALID_KNOWN_MASK: u32 = PRODUCT_TELEMETRY_VALID_POSITION_REFERENCE
    | PRODUCT_TELEMETRY_VALID_POSITION_MEASURED
    | PRODUCT_TELEMETRY_VALID_POSITION_ESTIMATED
    | PRODUCT_TELEMETRY_VALID_VELOCITY_REFERENCE
    | PRODUCT_TELEMETRY_VALID_VELOCITY_MEASURED
    | PRODUCT_TELEMETRY_VALID_VELOCITY_ESTIMATED
    | PRODUCT_TELEMETRY_VALID_TORQUE_REFERENCE
    | PRODUCT_TELEMETRY_VALID_TORQUE_ESTIMATED
    | PRODUCT_TELEMETRY_VALID_CURRENT_REFERENCE
    | PRODUCT_TELEMETRY_VALID_CURRENT_MEASURED
    | PRODUCT_TELEMETRY_VALID_VOLTAGE_COMMAND
    | PRODUCT_TELEMETRY_VALID_DC_BUS_VOLTAGE
    | PRODUCT_TELEMETRY_VALID_MOTOR_TEMPERATURE;

pub const PRODUCT_FAULT_EMERGENCY_STOP: u32 = 1 << 0;
pub const PRODUCT_FAULT_DRIVER: u32 = 1 << 1;
pub const PRODUCT_FAULT_BREAK: u32 = 1 << 2;
pub const PRODUCT_FAULT_OVERCURRENT: u32 = 1 << 3;
pub const PRODUCT_FAULT_BUS_UNDERVOLTAGE: u32 = 1 << 4;
pub const PRODUCT_FAULT_BUS_OVERVOLTAGE: u32 = 1 << 5;
pub const PRODUCT_FAULT_ADC_SAMPLE: u32 = 1 << 6;
pub const PRODUCT_FAULT_DEADLINE: u32 = 1 << 7;
pub const PRODUCT_FAULT_INVALID_FEEDBACK: u32 = 1 << 8;
pub const PRODUCT_FAULT_OBSERVER_STARTUP: u32 = 1 << 9;
pub const PRODUCT_FAULT_OBSERVER_LOST: u32 = 1 << 10;
pub const PRODUCT_FAULT_OUTPUT_REJECTED: u32 = 1 << 11;
pub const PRODUCT_FAULT_OVERTEMPERATURE: u32 = 1 << 12;
pub const PRODUCT_FAULT_COMMAND_TIMEOUT: u32 = 1 << 13;
pub const PRODUCT_FAULT_CONFIG_INVALID: u32 = 1 << 14;
pub const PRODUCT_FAULT_KNOWN_MASK: u32 = PRODUCT_FAULT_EMERGENCY_STOP
    | PRODUCT_FAULT_DRIVER
    | PRODUCT_FAULT_BREAK
    | PRODUCT_FAULT_OVERCURRENT
    | PRODUCT_FAULT_BUS_UNDERVOLTAGE
    | PRODUCT_FAULT_BUS_OVERVOLTAGE
    | PRODUCT_FAULT_ADC_SAMPLE
    | PRODUCT_FAULT_DEADLINE
    | PRODUCT_FAULT_INVALID_FEEDBACK
    | PRODUCT_FAULT_OBSERVER_STARTUP
    | PRODUCT_FAULT_OBSERVER_LOST
    | PRODUCT_FAULT_OUTPUT_REJECTED
    | PRODUCT_FAULT_OVERTEMPERATURE
    | PRODUCT_FAULT_COMMAND_TIMEOUT
    | PRODUCT_FAULT_CONFIG_INVALID;

macro_rules! fixed_u32_enum {
    ($name:ident { $($variant:ident = $value:expr),+ $(,)? }) => {
        #[repr(u32)]
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        pub enum $name { $($variant = $value),+ }

        impl TryFrom<u32> for $name {
            type Error = ContractError;

            fn try_from(value: u32) -> Result<Self, Self::Error> {
                match value {
                    $($value => Ok(Self::$variant),)+
                    _ => Err(ContractError::UnknownEnum),
                }
            }
        }
    };
}

fixed_u32_enum!(AxisState {
    Uninitialized = 0,
    Disabled = 1,
    Calibration = 2,
    Startup = 3,
    ClosedLoop = 4,
    Stopping = 5,
    FaultLatched = 6,
});

fixed_u32_enum!(AxisRequest {
    None = 0,
    Disabled = 1,
    CurrentOffsetCalibration = 2,
    MotorIdentification = 3,
    EncoderIndexSearch = 4,
    EncoderOffsetCalibration = 5,
    HallCalibration = 6,
    Homing = 7,
    OpenLoopTest = 8,
    ClosedLoopControl = 9,
});

fixed_u32_enum!(ControlMode {
    Inactive = 0,
    Voltage = 1,
    Duty = 2,
    Current = 3,
    Torque = 4,
    Velocity = 5,
    Position = 6,
});

fixed_u32_enum!(InputMode {
    Inactive = 0,
    Passthrough = 1,
    TorqueRamp = 2,
    VelocityRamp = 3,
    PositionFilter = 4,
    TrapezoidalTrajectory = 5,
    ExternalSynchronized = 6,
});

fixed_u32_enum!(FeedbackMode {
    Sensorless = 0,
    Hall = 1,
    IncrementalEncoder = 2,
    AbsoluteEncoder = 3,
    Resolver = 4,
    Fused = 5,
});

fixed_u32_enum!(ProductCommandKind {
    Release = 0,
    AxisRequest = 1,
    Setpoint = 2,
    ClearFault = 3,
    EmergencyStop = 4,
});

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractError {
    Header,
    UnsupportedAxis,
    UnknownEnum,
    UnknownFlags,
    NonFinite,
    OutOfRange,
    UnsupportedCombination,
    NonCanonical,
}

/// Canonical command envelope. All physical setpoints use SI units.
/// Timestamps are wrapping monotonic milliseconds and are interpreted by P1.2.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProductCommand {
    pub struct_size: u32,
    pub version: u32,
    pub axis_id: u32,
    pub source_id: u32,
    pub sequence: u32,
    pub created_at_ms: u32,
    pub valid_until_ms: u32,
    pub command_kind: u32,
    pub axis_request: u32,
    pub control_mode: u32,
    pub input_mode: u32,
    pub feedback_mode: u32,
    pub flags: u32,
    pub duty_ref: f32,
    pub voltage_d_ref_v: f32,
    pub voltage_q_ref_v: f32,
    pub current_d_ref_a: f32,
    pub current_q_ref_a: f32,
    pub torque_ref_nm: f32,
    pub velocity_ref_rad_s: f32,
    pub position_ref_rad: f32,
    pub velocity_feedforward_rad_s: f32,
    pub torque_feedforward_nm: f32,
    pub current_limit_a: f32,
    pub torque_limit_nm: f32,
    pub velocity_limit_rad_s: f32,
}

impl Default for ProductCommand {
    fn default() -> Self {
        Self {
            struct_size: size_of::<Self>() as u32,
            version: PRODUCT_COMMAND_VERSION,
            axis_id: 0,
            source_id: 0,
            sequence: 0,
            created_at_ms: 0,
            valid_until_ms: 0,
            command_kind: ProductCommandKind::Release as u32,
            axis_request: AxisRequest::None as u32,
            control_mode: ControlMode::Inactive as u32,
            input_mode: InputMode::Inactive as u32,
            feedback_mode: FeedbackMode::Sensorless as u32,
            flags: 0,
            duty_ref: 0.0,
            voltage_d_ref_v: 0.0,
            voltage_q_ref_v: 0.0,
            current_d_ref_a: 0.0,
            current_q_ref_a: 0.0,
            torque_ref_nm: 0.0,
            velocity_ref_rad_s: 0.0,
            position_ref_rad: 0.0,
            velocity_feedforward_rad_s: 0.0,
            torque_feedforward_nm: 0.0,
            current_limit_a: 0.0,
            torque_limit_nm: 0.0,
            velocity_limit_rad_s: 0.0,
        }
    }
}

impl ProductCommand {
    pub fn validate(&self) -> Result<(), ContractError> {
        if self.struct_size != size_of::<Self>() as u32 || self.version != PRODUCT_COMMAND_VERSION {
            return Err(ContractError::Header);
        }
        if self.axis_id != 0 {
            return Err(ContractError::UnsupportedAxis);
        }
        if self.flags & !PRODUCT_COMMAND_FLAG_KNOWN_MASK != 0 {
            return Err(ContractError::UnknownFlags);
        }
        if !self.all_floats_finite() {
            return Err(ContractError::NonFinite);
        }

        let kind = ProductCommandKind::try_from(self.command_kind)?;
        let request = AxisRequest::try_from(self.axis_request)?;
        let control = ControlMode::try_from(self.control_mode)?;
        let input = InputMode::try_from(self.input_mode)?;
        let feedback = FeedbackMode::try_from(self.feedback_mode)?;
        self.validate_limits(control)?;

        match kind {
            ProductCommandKind::Release
            | ProductCommandKind::ClearFault
            | ProductCommandKind::EmergencyStop => {
                if request != AxisRequest::None
                    || control != ControlMode::Inactive
                    || input != InputMode::Inactive
                    || feedback != FeedbackMode::Sensorless
                    || self.flags != 0
                    || !self.setpoints_are_zero()
                {
                    return Err(ContractError::NonCanonical);
                }
            }
            ProductCommandKind::AxisRequest => {
                if request == AxisRequest::None
                    || control != ControlMode::Inactive
                    || input != InputMode::Inactive
                    || self.flags != 0
                    || !self.setpoints_are_zero()
                {
                    return Err(ContractError::UnsupportedCombination);
                }
            }
            ProductCommandKind::Setpoint => {
                if request != AxisRequest::None || control == ControlMode::Inactive {
                    return Err(ContractError::UnsupportedCombination);
                }
                if !input_is_compatible(control, input) {
                    return Err(ContractError::UnsupportedCombination);
                }
                self.validate_setpoint_fields(control)?;
            }
        }
        Ok(())
    }

    fn all_floats_finite(&self) -> bool {
        [
            self.duty_ref,
            self.voltage_d_ref_v,
            self.voltage_q_ref_v,
            self.current_d_ref_a,
            self.current_q_ref_a,
            self.torque_ref_nm,
            self.velocity_ref_rad_s,
            self.position_ref_rad,
            self.velocity_feedforward_rad_s,
            self.torque_feedforward_nm,
            self.current_limit_a,
            self.torque_limit_nm,
            self.velocity_limit_rad_s,
        ]
        .iter()
        .all(|value| value.is_finite())
    }

    fn setpoints_are_zero(&self) -> bool {
        self.duty_ref == 0.0
            && self.voltage_d_ref_v == 0.0
            && self.voltage_q_ref_v == 0.0
            && self.current_d_ref_a == 0.0
            && self.current_q_ref_a == 0.0
            && self.torque_ref_nm == 0.0
            && self.velocity_ref_rad_s == 0.0
            && self.position_ref_rad == 0.0
            && self.velocity_feedforward_rad_s == 0.0
            && self.torque_feedforward_nm == 0.0
            && self.current_limit_a == 0.0
            && self.torque_limit_nm == 0.0
            && self.velocity_limit_rad_s == 0.0
    }

    fn validate_limits(&self, control: ControlMode) -> Result<(), ContractError> {
        let current_enabled = self.flags & PRODUCT_COMMAND_FLAG_CURRENT_LIMIT != 0;
        let torque_enabled = self.flags & PRODUCT_COMMAND_FLAG_TORQUE_LIMIT != 0;
        let velocity_enabled = self.flags & PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT != 0;

        if (current_enabled && self.current_limit_a <= 0.0)
            || (!current_enabled && self.current_limit_a != 0.0)
            || (torque_enabled && self.torque_limit_nm <= 0.0)
            || (!torque_enabled && self.torque_limit_nm != 0.0)
            || (velocity_enabled && self.velocity_limit_rad_s <= 0.0)
            || (!velocity_enabled && self.velocity_limit_rad_s != 0.0)
        {
            return Err(ContractError::OutOfRange);
        }
        if (current_enabled
            && !matches!(
                control,
                ControlMode::Current
                    | ControlMode::Torque
                    | ControlMode::Velocity
                    | ControlMode::Position
            ))
            || (torque_enabled
                && !matches!(
                    control,
                    ControlMode::Torque | ControlMode::Velocity | ControlMode::Position
                ))
            || (velocity_enabled
                && !matches!(control, ControlMode::Velocity | ControlMode::Position))
        {
            return Err(ContractError::UnsupportedCombination);
        }
        Ok(())
    }

    fn validate_setpoint_fields(&self, control: ControlMode) -> Result<(), ContractError> {
        let valid = match control {
            ControlMode::Inactive => false,
            ControlMode::Voltage => {
                self.duty_ref == 0.0
                    && self.current_d_ref_a == 0.0
                    && self.current_q_ref_a == 0.0
                    && self.torque_ref_nm == 0.0
                    && self.velocity_ref_rad_s == 0.0
                    && self.position_ref_rad == 0.0
                    && self.velocity_feedforward_rad_s == 0.0
                    && self.torque_feedforward_nm == 0.0
                    && self.flags == 0
            }
            ControlMode::Duty => {
                (-1.0..=1.0).contains(&self.duty_ref)
                    && self.voltage_d_ref_v == 0.0
                    && self.voltage_q_ref_v == 0.0
                    && self.current_d_ref_a == 0.0
                    && self.current_q_ref_a == 0.0
                    && self.torque_ref_nm == 0.0
                    && self.velocity_ref_rad_s == 0.0
                    && self.position_ref_rad == 0.0
                    && self.velocity_feedforward_rad_s == 0.0
                    && self.torque_feedforward_nm == 0.0
                    && self.flags == 0
            }
            ControlMode::Current => {
                self.duty_ref == 0.0
                    && self.voltage_d_ref_v == 0.0
                    && self.voltage_q_ref_v == 0.0
                    && self.torque_ref_nm == 0.0
                    && self.velocity_ref_rad_s == 0.0
                    && self.position_ref_rad == 0.0
                    && self.velocity_feedforward_rad_s == 0.0
                    && self.torque_feedforward_nm == 0.0
            }
            ControlMode::Torque => {
                self.duty_ref == 0.0
                    && self.voltage_d_ref_v == 0.0
                    && self.voltage_q_ref_v == 0.0
                    && self.current_d_ref_a == 0.0
                    && self.current_q_ref_a == 0.0
                    && self.velocity_ref_rad_s == 0.0
                    && self.position_ref_rad == 0.0
                    && self.velocity_feedforward_rad_s == 0.0
                    && self.torque_feedforward_nm == 0.0
                    && self.velocity_limit_rad_s == 0.0
            }
            ControlMode::Velocity => {
                self.duty_ref == 0.0
                    && self.voltage_d_ref_v == 0.0
                    && self.voltage_q_ref_v == 0.0
                    && self.current_d_ref_a == 0.0
                    && self.current_q_ref_a == 0.0
                    && self.torque_ref_nm == 0.0
                    && self.position_ref_rad == 0.0
                    && self.velocity_feedforward_rad_s == 0.0
            }
            ControlMode::Position => {
                self.duty_ref == 0.0
                    && self.voltage_d_ref_v == 0.0
                    && self.voltage_q_ref_v == 0.0
                    && self.current_d_ref_a == 0.0
                    && self.current_q_ref_a == 0.0
                    && self.torque_ref_nm == 0.0
                    && self.velocity_ref_rad_s == 0.0
            }
        };
        if valid {
            Ok(())
        } else {
            Err(ContractError::NonCanonical)
        }
    }
}

pub fn input_is_compatible(control: ControlMode, input: InputMode) -> bool {
    match input {
        InputMode::Inactive => control == ControlMode::Inactive,
        InputMode::Passthrough | InputMode::ExternalSynchronized => {
            control != ControlMode::Inactive
        }
        InputMode::TorqueRamp => control == ControlMode::Torque,
        InputMode::VelocityRamp => control == ControlMode::Velocity,
        InputMode::PositionFilter | InputMode::TrapezoidalTrajectory => {
            control == ControlMode::Position
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProductFeedbackSnapshot {
    pub struct_size: u32,
    pub version: u32,
    pub axis_id: u32,
    pub sequence: u32,
    pub sampled_at_ms: u32,
    pub sample_age_us: u32,
    pub valid_flags: u32,
    pub quality_flags: u32,
    pub active_feedback_mode: u32,
    pub backup_feedback_mode: u32,
    pub direction: i32,
    pub pole_pair_revision: u32,
    pub mechanical_position_rad: f32,
    pub multi_turn_position_rad: f32,
    pub mechanical_velocity_rad_s: f32,
    pub electrical_angle_rad: f32,
    pub electrical_velocity_rad_s: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProductTelemetrySnapshot {
    pub struct_size: u32,
    pub version: u32,
    pub axis_id: u32,
    pub sequence: u32,
    pub captured_at_ms: u32,
    pub axis_state: u32,
    pub control_mode: u32,
    pub input_mode: u32,
    pub feedback_mode: u32,
    pub active_source_id: u32,
    pub fault_flags: u32,
    pub valid_flags: u32,
    pub position_reference_rad: f32,
    pub position_measured_rad: f32,
    pub position_estimated_rad: f32,
    pub position_used_rad: f32,
    pub velocity_reference_rad_s: f32,
    pub velocity_measured_rad_s: f32,
    pub velocity_estimated_rad_s: f32,
    pub velocity_used_rad_s: f32,
    pub torque_reference_nm: f32,
    pub torque_estimated_nm: f32,
    pub current_d_reference_a: f32,
    pub current_q_reference_a: f32,
    pub current_d_measured_a: f32,
    pub current_q_measured_a: f32,
    pub voltage_d_command_v: f32,
    pub voltage_q_command_v: f32,
    pub dc_bus_voltage_v: f32,
    pub motor_temperature_c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProductFaultSnapshot {
    pub struct_size: u32,
    pub version: u32,
    pub axis_id: u32,
    pub sequence: u32,
    pub first_occurred_at_ms: u32,
    pub last_occurred_at_ms: u32,
    pub occurrence_count: u32,
    pub latched_fault_flags: u32,
    pub active_fault_flags: u32,
    pub primary_fault: u32,
    pub axis_state_at_fault: u32,
    pub control_mode_at_fault: u32,
    pub input_mode_at_fault: u32,
    pub feedback_mode_at_fault: u32,
    pub dc_bus_voltage_v: f32,
    pub peak_phase_current_a: f32,
    pub mechanical_velocity_rad_s: f32,
    pub motor_temperature_c: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    fn setpoint(control: ControlMode, input: InputMode) -> ProductCommand {
        ProductCommand {
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: control as u32,
            input_mode: input as u32,
            ..ProductCommand::default()
        }
    }

    #[test]
    fn product_contract_layout_and_values_are_frozen() {
        assert_eq!(PRODUCT_CONTRACT_VERSION, 0x0001_0000);
        assert_eq!(AxisState::FaultLatched as u32, 6);
        assert_eq!(AxisRequest::ClosedLoopControl as u32, 9);
        assert_eq!(ControlMode::Velocity as u32, 5);
        assert_eq!(InputMode::TrapezoidalTrajectory as u32, 5);
        assert_eq!(FeedbackMode::Fused as u32, 5);
        assert_eq!(size_of::<ProductCommand>(), 104);
        assert_eq!(align_of::<ProductCommand>(), 4);
        assert_eq!(offset_of!(ProductCommand, flags), 48);
        assert_eq!(offset_of!(ProductCommand, duty_ref), 52);
        assert_eq!(offset_of!(ProductCommand, current_d_ref_a), 64);
        assert_eq!(offset_of!(ProductCommand, velocity_ref_rad_s), 76);
        assert_eq!(offset_of!(ProductCommand, velocity_limit_rad_s), 100);
        assert_eq!(size_of::<ProductFeedbackSnapshot>(), 68);
        assert_eq!(align_of::<ProductFeedbackSnapshot>(), 4);
        assert_eq!(
            offset_of!(ProductFeedbackSnapshot, active_feedback_mode),
            32
        );
        assert_eq!(
            offset_of!(ProductFeedbackSnapshot, mechanical_position_rad),
            48
        );
        assert_eq!(size_of::<ProductTelemetrySnapshot>(), 120);
        assert_eq!(align_of::<ProductTelemetrySnapshot>(), 4);
        assert_eq!(offset_of!(ProductTelemetrySnapshot, valid_flags), 44);
        assert_eq!(
            offset_of!(ProductTelemetrySnapshot, position_reference_rad),
            48
        );
        assert_eq!(offset_of!(ProductTelemetrySnapshot, dc_bus_voltage_v), 112);
        assert_eq!(size_of::<ProductFaultSnapshot>(), 72);
        assert_eq!(align_of::<ProductFaultSnapshot>(), 4);
        assert_eq!(offset_of!(ProductFaultSnapshot, latched_fault_flags), 28);
        assert_eq!(offset_of!(ProductFaultSnapshot, primary_fault), 36);
        assert_eq!(offset_of!(ProductFaultSnapshot, dc_bus_voltage_v), 56);
        assert_eq!(PRODUCT_TELEMETRY_VALID_KNOWN_MASK, 0x1fff);
    }

    #[test]
    fn supported_control_and_input_pairs_validate() {
        let mut voltage = setpoint(ControlMode::Voltage, InputMode::Passthrough);
        voltage.voltage_q_ref_v = 1.0;
        assert_eq!(voltage.validate(), Ok(()));

        let mut duty = setpoint(ControlMode::Duty, InputMode::ExternalSynchronized);
        duty.duty_ref = -0.25;
        assert_eq!(duty.validate(), Ok(()));

        let mut current = setpoint(ControlMode::Current, InputMode::Passthrough);
        current.current_q_ref_a = 0.2;
        current.flags = PRODUCT_COMMAND_FLAG_CURRENT_LIMIT;
        current.current_limit_a = 0.8;
        assert_eq!(current.validate(), Ok(()));

        let mut torque = setpoint(ControlMode::Torque, InputMode::TorqueRamp);
        torque.torque_ref_nm = 0.02;
        torque.flags = PRODUCT_COMMAND_FLAG_CURRENT_LIMIT | PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
        torque.current_limit_a = 0.8;
        torque.torque_limit_nm = 0.03;
        assert_eq!(torque.validate(), Ok(()));

        let mut velocity = setpoint(ControlMode::Velocity, InputMode::VelocityRamp);
        velocity.velocity_ref_rad_s = 54.873_15;
        velocity.torque_feedforward_nm = 0.001;
        velocity.flags = PRODUCT_COMMAND_FLAG_CURRENT_LIMIT | PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT;
        velocity.current_limit_a = 0.8;
        velocity.velocity_limit_rad_s = 100.0;
        assert_eq!(velocity.validate(), Ok(()));

        let mut position = setpoint(ControlMode::Position, InputMode::TrapezoidalTrajectory);
        position.position_ref_rad = 1.0;
        position.velocity_feedforward_rad_s = 0.1;
        assert_eq!(position.validate(), Ok(()));
    }

    #[test]
    fn unknown_values_and_illegal_combinations_fail_closed() {
        let mut command = ProductCommand {
            struct_size: 0,
            ..ProductCommand::default()
        };
        assert_eq!(command.validate(), Err(ContractError::Header));

        command = ProductCommand::default();
        command.axis_id = 1;
        assert_eq!(command.validate(), Err(ContractError::UnsupportedAxis));

        command = setpoint(ControlMode::Velocity, InputMode::TorqueRamp);
        assert_eq!(
            command.validate(),
            Err(ContractError::UnsupportedCombination)
        );

        command = setpoint(ControlMode::Duty, InputMode::Passthrough);
        command.duty_ref = 1.01;
        assert_eq!(command.validate(), Err(ContractError::NonCanonical));

        command = setpoint(ControlMode::Velocity, InputMode::Passthrough);
        command.position_ref_rad = 1.0;
        assert_eq!(command.validate(), Err(ContractError::NonCanonical));

        command = setpoint(ControlMode::Current, InputMode::Passthrough);
        command.current_q_ref_a = f32::NAN;
        assert_eq!(command.validate(), Err(ContractError::NonFinite));

        command = setpoint(ControlMode::Current, InputMode::Passthrough);
        command.current_limit_a = -0.1;
        assert_eq!(command.validate(), Err(ContractError::OutOfRange));

        command = setpoint(ControlMode::Position, InputMode::PositionFilter);
        command.velocity_ref_rad_s = 1.0;
        assert_eq!(command.validate(), Err(ContractError::NonCanonical));

        command = ProductCommand {
            command_kind: u32::MAX,
            ..ProductCommand::default()
        };
        assert_eq!(command.validate(), Err(ContractError::UnknownEnum));

        command = ProductCommand {
            flags: 1 << 31,
            ..ProductCommand::default()
        };
        assert_eq!(command.validate(), Err(ContractError::UnknownFlags));

        command = ProductCommand {
            command_kind: ProductCommandKind::ClearFault as u32,
            feedback_mode: FeedbackMode::Hall as u32,
            ..ProductCommand::default()
        };
        assert_eq!(command.validate(), Err(ContractError::NonCanonical));
    }

    #[test]
    fn lifecycle_commands_are_canonical_and_explicit() {
        assert_eq!(ProductCommand::default().validate(), Ok(()));

        let request = ProductCommand {
            command_kind: ProductCommandKind::AxisRequest as u32,
            axis_request: AxisRequest::ClosedLoopControl as u32,
            ..ProductCommand::default()
        };
        assert_eq!(request.validate(), Ok(()));

        let mut stale = request;
        stale.velocity_ref_rad_s = 1.0;
        assert_eq!(stale.validate(), Err(ContractError::UnsupportedCombination));

        let mut clear = ProductCommand {
            command_kind: ProductCommandKind::ClearFault as u32,
            ..ProductCommand::default()
        };
        assert_eq!(clear.validate(), Ok(()));
        clear.axis_request = AxisRequest::Disabled as u32;
        assert_eq!(clear.validate(), Err(ContractError::NonCanonical));
    }

    #[test]
    fn legacy_speed_start_is_expressible_in_si_without_changing_target() {
        let legacy_target_rpm = 524.0_f32;
        let target_rad_s = legacy_target_rpm * core::f32::consts::PI / 30.0;
        let setpoint = ProductCommand {
            command_kind: ProductCommandKind::Setpoint as u32,
            control_mode: ControlMode::Velocity as u32,
            input_mode: InputMode::Passthrough as u32,
            velocity_ref_rad_s: target_rad_s,
            ..ProductCommand::default()
        };
        let start = ProductCommand {
            command_kind: ProductCommandKind::AxisRequest as u32,
            axis_request: AxisRequest::ClosedLoopControl as u32,
            ..ProductCommand::default()
        };

        assert_eq!(setpoint.validate(), Ok(()));
        assert_eq!(start.validate(), Ok(()));
        assert!(
            (setpoint.velocity_ref_rad_s * 30.0 / core::f32::consts::PI - legacy_target_rpm).abs()
                < 1.0e-4
        );
    }
}
