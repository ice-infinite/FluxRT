//! Recoverable product configuration core.
//!
//! This module owns the board-independent configuration schema and the pure
//! two-slot persistence protocol. It intentionally does not erase/program MCU
//! Flash, mutate the active realtime controller, or expose a C ABI. A platform
//! adapter may execute [`PreparedConfigWrite`] only while the Axis is safe and
//! disabled; P3.2 will add the pending/active transaction above this layer.

use crate::{CommandPermissions, CommandSourcePolicy, FeedbackMode, ProductCommandKind};

pub const CONFIG_STORAGE_FORMAT_VERSION: u16 = 1;
pub const CONFIG_SCHEMA_VERSION: u16 = 1;
pub const CONFIG_LEGACY_SCHEMA_VERSION: u16 = 0;
pub const CONFIG_GROUP_VERSION: u32 = 1;
pub const CONFIG_SLOT_COUNT: usize = 2;
pub const CONFIG_SLOT_SIZE: usize = 512;
pub const CONFIG_MAX_COMMAND_SOURCES: usize = 4;

pub const COMMAND_SOURCE_PERMISSION_RELEASE: u32 = 1 << ProductCommandKind::Release as u32;
pub const COMMAND_SOURCE_PERMISSION_AXIS_REQUEST: u32 = 1 << ProductCommandKind::AxisRequest as u32;
pub const COMMAND_SOURCE_PERMISSION_SETPOINT: u32 = 1 << ProductCommandKind::Setpoint as u32;
pub const COMMAND_SOURCE_PERMISSION_CLEAR_FAULT: u32 = 1 << ProductCommandKind::ClearFault as u32;
pub const COMMAND_SOURCE_PERMISSION_EMERGENCY_STOP: u32 =
    1 << ProductCommandKind::EmergencyStop as u32;
pub const COMMAND_SOURCE_PERMISSION_KNOWN_MASK: u32 = COMMAND_SOURCE_PERMISSION_RELEASE
    | COMMAND_SOURCE_PERMISSION_AXIS_REQUEST
    | COMMAND_SOURCE_PERMISSION_SETPOINT
    | COMMAND_SOURCE_PERMISSION_CLEAR_FAULT
    | COMMAND_SOURCE_PERMISSION_EMERGENCY_STOP;

const CONFIG_RECORD_MAGIC: u32 = 0x4652_4346; // "FRCF" in little endian.
const CONFIG_COMMIT_MAGIC: u32 = 0x434D_4954; // "CMIT" in little endian.
const CONFIG_HEADER_SIZE: usize = 40;
const CONFIG_PAYLOAD_OFFSET: usize = CONFIG_HEADER_SIZE;
const CONFIG_COMMIT_OFFSET: usize = CONFIG_SLOT_SIZE - 4;
const CONFIG_MAX_PAYLOAD_SIZE: usize = CONFIG_COMMIT_OFFSET - CONFIG_PAYLOAD_OFFSET;
const CONFIG_PAYLOAD_V1_SIZE: usize = 316;
const CONFIG_PAYLOAD_V0_SIZE: usize = 284;
const WRAPPING_HALF_RANGE: u32 = 0x8000_0000;

const HEADER_MAGIC_OFFSET: usize = 0;
const HEADER_STORAGE_VERSION_OFFSET: usize = 4;
const HEADER_SCHEMA_VERSION_OFFSET: usize = 6;
const HEADER_SIZE_OFFSET: usize = 8;
const HEADER_PAYLOAD_SIZE_OFFSET: usize = 10;
const HEADER_SEQUENCE_OFFSET: usize = 12;
const HEADER_APPROVAL_OFFSET: usize = 16;
const HEADER_BOARD_ID_OFFSET: usize = 20;
const HEADER_MOTOR_ID_OFFSET: usize = 24;
const HEADER_PAYLOAD_CRC_OFFSET: usize = 28;
const HEADER_RECORD_CRC_OFFSET: usize = 32;
const HEADER_RESERVED_OFFSET: usize = 36;

pub const BOARD_CAP_THREE_SHUNT_CURRENT: u32 = 1 << 0;
pub const BOARD_CAP_PHASE_VOLTAGE_SENSE: u32 = 1 << 1;
pub const BOARD_CAP_BRAKE_RESISTOR: u32 = 1 << 2;
pub const BOARD_CAP_ENCODER: u32 = 1 << 3;
pub const BOARD_CAP_HALL: u32 = 1 << 4;
pub const BOARD_CAP_KNOWN_MASK: u32 = BOARD_CAP_THREE_SHUNT_CURRENT
    | BOARD_CAP_PHASE_VOLTAGE_SENSE
    | BOARD_CAP_BRAKE_RESISTOR
    | BOARD_CAP_ENCODER
    | BOARD_CAP_HALL;

pub const CALIBRATION_CURRENT_OFFSETS_VALID: u32 = 1 << 0;
pub const CALIBRATION_PHASE_VOLTAGE_VALID: u32 = 1 << 1;
pub const CALIBRATION_ENCODER_VALID: u32 = 1 << 2;
pub const CALIBRATION_HALL_VALID: u32 = 1 << 3;
pub const CALIBRATION_KNOWN_MASK: u32 = CALIBRATION_CURRENT_OFFSETS_VALID
    | CALIBRATION_PHASE_VOLTAGE_VALID
    | CALIBRATION_ENCODER_VALID
    | CALIBRATION_HALL_VALID;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoardConfig {
    pub version: u32,
    pub board_id: u32,
    pub pwm_frequency_hz: u32,
    pub control_frequency_hz: u32,
    pub capability_flags: u32,
    pub adc_reference_v: f32,
    pub current_gain_a_per_count: f32,
    pub bus_voltage_v_per_count: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorConfig {
    pub version: u32,
    pub motor_id: u32,
    pub pole_pairs: u32,
    pub phase_resistance_ohm: f32,
    pub d_inductance_h: f32,
    pub q_inductance_h: f32,
    pub flux_linkage_v_s: f32,
    pub continuous_current_a: f32,
    pub maximum_speed_rad_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InverterConfig {
    pub version: u32,
    pub inverter_id: u32,
    pub maximum_phase_current_a: f32,
    pub minimum_bus_voltage_v: f32,
    pub maximum_bus_voltage_v: f32,
    pub dead_time_s: f32,
    pub transistor_drop_v: f32,
    /// Zero means that no brake resistor is fitted.
    pub brake_resistance_ohm: f32,
    pub maximum_duty: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisConfig {
    pub version: u32,
    pub axis_id: u32,
    pub feedback_mode: u32,
    pub direction: i32,
    pub current_kp: f32,
    pub current_ki: f32,
    pub velocity_kp: f32,
    pub velocity_ki: f32,
    pub position_kp: f32,
    pub soft_limit_min_rad: f32,
    pub soft_limit_max_rad: f32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommandSourceConfig {
    pub source_id: u32,
    pub priority: u32,
    pub permissions: u32,
    pub lease_ms: u32,
    pub command_timeout_ms: u32,
}

impl CommandSourceConfig {
    pub fn policy(&self) -> Result<CommandSourcePolicy, ConfigValidationError> {
        self.validate()?;
        let permissions = permissions_from_bits(self.permissions)
            .ok_or(ConfigValidationError::CommandSourcePermissions)?;
        Ok(CommandSourcePolicy {
            source_id: self.source_id,
            priority: self.priority as u8,
            permissions,
            lease_ms: self.lease_ms,
            command_timeout_ms: self.command_timeout_ms,
        })
    }

    fn validate(&self) -> Result<(), ConfigValidationError> {
        if self.priority > u8::MAX.into() {
            return Err(ConfigValidationError::CommandSourcePriority);
        }
        if self.permissions == 0 || self.permissions & !COMMAND_SOURCE_PERMISSION_KNOWN_MASK != 0 {
            return Err(ConfigValidationError::CommandSourcePermissions);
        }
        if self.lease_ms == 0
            || self.command_timeout_ms == 0
            || self.lease_ms >= WRAPPING_HALF_RANGE
            || self.command_timeout_ms >= WRAPPING_HALF_RANGE
            || self.lease_ms > self.command_timeout_ms
        {
            return Err(ConfigValidationError::CommandSourceTime);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AppConfig {
    pub version: u32,
    pub can_node_id: u32,
    pub uart_baud: u32,
    pub command_source_count: u32,
    pub command_sources: [CommandSourceConfig; CONFIG_MAX_COMMAND_SOURCES],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalibrationData {
    pub version: u32,
    pub board_id: u32,
    pub motor_id: u32,
    pub valid_flags: u32,
    pub current_offset_counts: [f32; 3],
    pub phase_voltage_gain: [f32; 3],
    pub phase_voltage_offset_v: [f32; 3],
    pub encoder_offset_rad: f32,
    pub encoder_counts_per_revolution: u32,
    /// Six Hall sectors packed as six 4-bit values, least-significant sector first.
    pub hall_sequence_packed: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConfigBundle {
    pub bundle_revision: u32,
    pub generated_by_version: u32,
    pub board: BoardConfig,
    pub motor: MotorConfig,
    pub inverter: InverterConfig,
    pub axis: AxisConfig,
    pub app: AppConfig,
    pub calibration: CalibrationData,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigValidationError {
    GroupVersion,
    ZeroIdentity,
    Frequency,
    BoardCapabilities,
    BoardScaling,
    MotorParameters,
    InverterParameters,
    MotorExceedsInverter,
    Axis,
    App,
    CommandSourceCount,
    CommandSourceDuplicate,
    CommandSourcePriority,
    CommandSourcePermissions,
    CommandSourceTime,
    NonCanonicalUnusedSource,
    CalibrationFlags,
    CalibrationIdentity,
    CalibrationValue,
    FeedbackCalibrationMissing,
    HardwareIdentityMismatch,
    HardwareCapabilityMissing,
}

impl ConfigBundle {
    pub fn validate(&self) -> Result<(), ConfigValidationError> {
        if self.bundle_revision == 0
            || self.generated_by_version == 0
            || self.board.version != CONFIG_GROUP_VERSION
            || self.motor.version != CONFIG_GROUP_VERSION
            || self.inverter.version != CONFIG_GROUP_VERSION
            || self.axis.version != CONFIG_GROUP_VERSION
            || self.app.version != CONFIG_GROUP_VERSION
            || self.calibration.version != CONFIG_GROUP_VERSION
        {
            return Err(ConfigValidationError::GroupVersion);
        }
        if self.board.board_id == 0 || self.motor.motor_id == 0 || self.inverter.inverter_id == 0 {
            return Err(ConfigValidationError::ZeroIdentity);
        }
        if self.board.pwm_frequency_hz == 0
            || self.board.control_frequency_hz == 0
            || self.board.pwm_frequency_hz > 200_000
            || self.board.control_frequency_hz > self.board.pwm_frequency_hz
            || !self
                .board
                .pwm_frequency_hz
                .is_multiple_of(self.board.control_frequency_hz)
        {
            return Err(ConfigValidationError::Frequency);
        }
        if self.board.capability_flags & !BOARD_CAP_KNOWN_MASK != 0 {
            return Err(ConfigValidationError::BoardCapabilities);
        }
        if !finite_positive(self.board.adc_reference_v)
            || self.board.adc_reference_v > 6.0
            || !finite_positive(self.board.current_gain_a_per_count)
            || !finite_positive(self.board.bus_voltage_v_per_count)
        {
            return Err(ConfigValidationError::BoardScaling);
        }
        if !(1..=64).contains(&self.motor.pole_pairs)
            || !finite_positive(self.motor.phase_resistance_ohm)
            || !finite_positive(self.motor.d_inductance_h)
            || !finite_positive(self.motor.q_inductance_h)
            || !finite_positive(self.motor.flux_linkage_v_s)
            || !finite_positive(self.motor.continuous_current_a)
            || !finite_positive(self.motor.maximum_speed_rad_s)
        {
            return Err(ConfigValidationError::MotorParameters);
        }
        if !finite_positive(self.inverter.maximum_phase_current_a)
            || !finite_positive(self.inverter.minimum_bus_voltage_v)
            || !finite_positive(self.inverter.maximum_bus_voltage_v)
            || self.inverter.minimum_bus_voltage_v >= self.inverter.maximum_bus_voltage_v
            || !finite_nonnegative(self.inverter.dead_time_s)
            || self.inverter.dead_time_s > 20.0e-6
            || !finite_nonnegative(self.inverter.transistor_drop_v)
            || !finite_nonnegative(self.inverter.brake_resistance_ohm)
            || !self.inverter.maximum_duty.is_finite()
            || self.inverter.maximum_duty <= 0.0
            || self.inverter.maximum_duty > 1.0
        {
            return Err(ConfigValidationError::InverterParameters);
        }
        if self.motor.continuous_current_a > self.inverter.maximum_phase_current_a {
            return Err(ConfigValidationError::MotorExceedsInverter);
        }
        self.validate_axis()?;
        self.validate_app()?;
        self.validate_calibration()?;
        Ok(())
    }

    /// Final hardware/arm gate. Structurally valid records may still be saved as
    /// candidates, but cannot arm under the wrong board/motor identity or with
    /// missing feedback calibration.
    pub fn validate_for_arm(
        &self,
        expected_board_id: u32,
        expected_motor_id: u32,
    ) -> Result<(), ConfigValidationError> {
        self.validate()?;
        if self.board.board_id != expected_board_id || self.motor.motor_id != expected_motor_id {
            return Err(ConfigValidationError::HardwareIdentityMismatch);
        }
        let feedback = FeedbackMode::try_from(self.axis.feedback_mode)
            .map_err(|_| ConfigValidationError::Axis)?;
        let (required_flag, required_capability) = match feedback {
            FeedbackMode::Sensorless => return Ok(()),
            FeedbackMode::Hall => (CALIBRATION_HALL_VALID, BOARD_CAP_HALL),
            FeedbackMode::IncrementalEncoder
            | FeedbackMode::AbsoluteEncoder
            | FeedbackMode::Resolver
            | FeedbackMode::Fused => (CALIBRATION_ENCODER_VALID, BOARD_CAP_ENCODER),
        };
        if self.board.capability_flags & required_capability == 0 {
            return Err(ConfigValidationError::HardwareCapabilityMissing);
        }
        if self.calibration.valid_flags & required_flag == 0 {
            return Err(ConfigValidationError::FeedbackCalibrationMissing);
        }
        Ok(())
    }

    fn validate_axis(&self) -> Result<(), ConfigValidationError> {
        if self.axis.axis_id != 0
            || FeedbackMode::try_from(self.axis.feedback_mode).is_err()
            || !matches!(self.axis.direction, -1 | 1)
            || !finite_nonnegative(self.axis.current_kp)
            || !finite_nonnegative(self.axis.current_ki)
            || !finite_nonnegative(self.axis.velocity_kp)
            || !finite_nonnegative(self.axis.velocity_ki)
            || !finite_nonnegative(self.axis.position_kp)
            || !self.axis.soft_limit_min_rad.is_finite()
            || !self.axis.soft_limit_max_rad.is_finite()
            || self.axis.soft_limit_min_rad >= self.axis.soft_limit_max_rad
        {
            return Err(ConfigValidationError::Axis);
        }
        Ok(())
    }

    fn validate_app(&self) -> Result<(), ConfigValidationError> {
        if self.app.can_node_id > 127 || !(1_200..=12_000_000).contains(&self.app.uart_baud) {
            return Err(ConfigValidationError::App);
        }
        let source_count = self.app.command_source_count as usize;
        if source_count == 0 || source_count > CONFIG_MAX_COMMAND_SOURCES {
            return Err(ConfigValidationError::CommandSourceCount);
        }
        for index in 0..source_count {
            self.app.command_sources[index].validate()?;
            if self.app.command_sources[..index]
                .iter()
                .any(|source| source.source_id == self.app.command_sources[index].source_id)
            {
                return Err(ConfigValidationError::CommandSourceDuplicate);
            }
        }
        if self.app.command_sources[source_count..]
            .iter()
            .any(|source| *source != CommandSourceConfig::default())
        {
            return Err(ConfigValidationError::NonCanonicalUnusedSource);
        }
        Ok(())
    }

    fn validate_calibration(&self) -> Result<(), ConfigValidationError> {
        let calibration = &self.calibration;
        if calibration.valid_flags & !CALIBRATION_KNOWN_MASK != 0 {
            return Err(ConfigValidationError::CalibrationFlags);
        }
        if calibration.board_id != self.board.board_id
            || calibration.motor_id != self.motor.motor_id
        {
            return Err(ConfigValidationError::CalibrationIdentity);
        }
        if !all_finite(&calibration.current_offset_counts)
            || !all_finite(&calibration.phase_voltage_gain)
            || !all_finite(&calibration.phase_voltage_offset_v)
            || !calibration.encoder_offset_rad.is_finite()
        {
            return Err(ConfigValidationError::CalibrationValue);
        }
        if calibration.valid_flags & CALIBRATION_CURRENT_OFFSETS_VALID == 0
            && calibration.current_offset_counts != [0.0; 3]
        {
            return Err(ConfigValidationError::CalibrationValue);
        }
        if calibration.valid_flags & CALIBRATION_PHASE_VOLTAGE_VALID != 0 {
            if calibration
                .phase_voltage_gain
                .iter()
                .any(|gain| *gain <= 0.0)
            {
                return Err(ConfigValidationError::CalibrationValue);
            }
        } else if calibration.phase_voltage_gain != [0.0; 3]
            || calibration.phase_voltage_offset_v != [0.0; 3]
        {
            return Err(ConfigValidationError::CalibrationValue);
        }
        if calibration.valid_flags & CALIBRATION_ENCODER_VALID != 0 {
            if calibration.encoder_counts_per_revolution == 0 {
                return Err(ConfigValidationError::CalibrationValue);
            }
        } else if calibration.encoder_offset_rad != 0.0
            || calibration.encoder_counts_per_revolution != 0
        {
            return Err(ConfigValidationError::CalibrationValue);
        }
        if calibration.valid_flags & CALIBRATION_HALL_VALID != 0 {
            if !hall_sequence_is_valid(calibration.hall_sequence_packed) {
                return Err(ConfigValidationError::CalibrationValue);
            }
        } else if calibration.hall_sequence_packed != 0 {
            return Err(ConfigValidationError::CalibrationValue);
        }
        Ok(())
    }
}

#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigApproval {
    Candidate = 1,
    Approved = 2,
}

impl TryFrom<u32> for ConfigApproval {
    type Error = ConfigRecordError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Candidate),
            2 => Ok(Self::Approved),
            _ => Err(ConfigRecordError::InvalidApproval),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigRecordError {
    Erased,
    CommitMissing,
    Header,
    UnsupportedStorageVersion,
    UnsupportedSchemaVersion,
    PayloadLength,
    PayloadCrc,
    RecordCrc,
    InvalidApproval,
    IdentityMismatch,
    InvalidConfig(ConfigValidationError),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecodedConfigRecord {
    pub sequence: u32,
    pub approval: ConfigApproval,
    pub schema_version: u16,
    pub migrated: bool,
    pub config: ConfigBundle,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SlotState {
    Erased,
    Valid {
        sequence: u32,
        approval: ConfigApproval,
        migrated: bool,
    },
    Invalid(ConfigRecordError),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigSource {
    Factory,
    Slot0,
    Slot1,
}

impl ConfigSource {
    fn from_slot(index: usize) -> Self {
        if index == 0 {
            Self::Slot0
        } else {
            Self::Slot1
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SelectedConfig {
    pub source: ConfigSource,
    pub sequence: Option<u32>,
    pub migrated: bool,
    pub config: ConfigBundle,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryDecision {
    pub active: SelectedConfig,
    pub candidate: Option<SelectedConfig>,
    pub previous_approved: Option<SelectedConfig>,
    pub slots: [SlotState; CONFIG_SLOT_COUNT],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigRecoveryError {
    InvalidFactory(ConfigValidationError),
    AmbiguousSequence,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigWriteError {
    InvalidConfig(ConfigValidationError),
    AmbiguousSequence,
    RollbackUnavailable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigSlot {
    bytes: [u8; CONFIG_SLOT_SIZE],
}

impl ConfigSlot {
    pub const fn erased() -> Self {
        Self {
            bytes: [0xFF; CONFIG_SLOT_SIZE],
        }
    }

    pub const fn from_bytes(bytes: [u8; CONFIG_SLOT_SIZE]) -> Self {
        Self { bytes }
    }

    pub const fn as_bytes(&self) -> &[u8; CONFIG_SLOT_SIZE] {
        &self.bytes
    }

    pub fn is_erased(&self) -> bool {
        self.bytes.iter().all(|byte| *byte == 0xFF)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PreparedConfigWrite {
    pub target_slot: usize,
    pub sequence: u32,
    image: ConfigSlot,
    prefix_len: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ConfigWriteByte {
    pub offset: u16,
    pub value: u8,
}

impl PreparedConfigWrite {
    /// Number of bytes the platform writes after erasing the target slot.
    /// The four-byte commit marker is always the final operation.
    pub const fn write_step_count(&self) -> usize {
        self.prefix_len + 4
    }

    /// Returns the exact byte operation for one platform write step. The final
    /// four steps always write the commit marker at the end of the slot.
    pub fn write_byte(&self, step: usize) -> Option<ConfigWriteByte> {
        let offset = if step < self.prefix_len {
            step
        } else if step < self.write_step_count() {
            CONFIG_COMMIT_OFFSET + step - self.prefix_len
        } else {
            return None;
        };
        Some(ConfigWriteByte {
            offset: offset as u16,
            value: self.image.bytes[offset],
        })
    }

    /// Host/fake-Flash helper: erase the target and execute the first `steps`
    /// byte writes. A value smaller than `write_step_count()` models power loss.
    pub fn apply_prefix(&self, slots: &mut [ConfigSlot; CONFIG_SLOT_COUNT], steps: usize) {
        slots[self.target_slot] = ConfigSlot::erased();
        for step in 0..steps.min(self.write_step_count()) {
            if let Some(write) = self.write_byte(step) {
                slots[self.target_slot].bytes[usize::from(write.offset)] = write.value;
            }
        }
    }

    pub fn apply_complete(&self, slots: &mut [ConfigSlot; CONFIG_SLOT_COUNT]) {
        self.apply_prefix(slots, self.write_step_count());
    }
}

pub fn decode_config_slot(slot: &ConfigSlot) -> Result<DecodedConfigRecord, ConfigRecordError> {
    if slot.is_erased() {
        return Err(ConfigRecordError::Erased);
    }
    if read_u32(&slot.bytes, CONFIG_COMMIT_OFFSET) != CONFIG_COMMIT_MAGIC {
        return Err(ConfigRecordError::CommitMissing);
    }
    if read_u32(&slot.bytes, HEADER_MAGIC_OFFSET) != CONFIG_RECORD_MAGIC
        || read_u16(&slot.bytes, HEADER_SIZE_OFFSET) as usize != CONFIG_HEADER_SIZE
        || read_u32(&slot.bytes, HEADER_RESERVED_OFFSET) != 0
    {
        return Err(ConfigRecordError::Header);
    }
    if read_u16(&slot.bytes, HEADER_STORAGE_VERSION_OFFSET) != CONFIG_STORAGE_FORMAT_VERSION {
        return Err(ConfigRecordError::UnsupportedStorageVersion);
    }
    let schema_version = read_u16(&slot.bytes, HEADER_SCHEMA_VERSION_OFFSET);
    let expected_payload_size = match schema_version {
        CONFIG_SCHEMA_VERSION => CONFIG_PAYLOAD_V1_SIZE,
        CONFIG_LEGACY_SCHEMA_VERSION => CONFIG_PAYLOAD_V0_SIZE,
        _ => return Err(ConfigRecordError::UnsupportedSchemaVersion),
    };
    let payload_size = read_u16(&slot.bytes, HEADER_PAYLOAD_SIZE_OFFSET) as usize;
    if payload_size != expected_payload_size || payload_size > CONFIG_MAX_PAYLOAD_SIZE {
        return Err(ConfigRecordError::PayloadLength);
    }
    let payload = &slot.bytes[CONFIG_PAYLOAD_OFFSET..CONFIG_PAYLOAD_OFFSET + payload_size];
    if crc32(payload) != read_u32(&slot.bytes, HEADER_PAYLOAD_CRC_OFFSET) {
        return Err(ConfigRecordError::PayloadCrc);
    }
    if record_crc32(&slot.bytes, CONFIG_PAYLOAD_OFFSET + payload_size)
        != read_u32(&slot.bytes, HEADER_RECORD_CRC_OFFSET)
    {
        return Err(ConfigRecordError::RecordCrc);
    }
    let approval = ConfigApproval::try_from(read_u32(&slot.bytes, HEADER_APPROVAL_OFFSET))?;
    let config = decode_payload(payload, schema_version)?;
    if read_u32(&slot.bytes, HEADER_BOARD_ID_OFFSET) != config.board.board_id
        || read_u32(&slot.bytes, HEADER_MOTOR_ID_OFFSET) != config.motor.motor_id
    {
        return Err(ConfigRecordError::IdentityMismatch);
    }
    config
        .validate()
        .map_err(ConfigRecordError::InvalidConfig)?;
    Ok(DecodedConfigRecord {
        sequence: read_u32(&slot.bytes, HEADER_SEQUENCE_OFFSET),
        approval,
        schema_version,
        migrated: schema_version != CONFIG_SCHEMA_VERSION,
        config,
    })
}

pub fn recover_config(
    slots: &[ConfigSlot; CONFIG_SLOT_COUNT],
    factory: ConfigBundle,
) -> Result<RecoveryDecision, ConfigRecoveryError> {
    factory
        .validate()
        .map_err(ConfigRecoveryError::InvalidFactory)?;
    let decoded = [decode_config_slot(&slots[0]), decode_config_slot(&slots[1])];
    let states = [slot_state(&decoded[0]), slot_state(&decoded[1])];
    let approved = select_newest(&decoded, Some(ConfigApproval::Approved))?;
    let candidate = select_newest(&decoded, Some(ConfigApproval::Candidate))?;
    let previous_approved = if let Some((active_index, _)) = approved {
        select_other_approved(&decoded, active_index)?
    } else {
        None
    };

    let active = approved.map_or(
        SelectedConfig {
            source: ConfigSource::Factory,
            sequence: None,
            migrated: false,
            config: factory,
        },
        |(index, record)| selected(index, record),
    );

    Ok(RecoveryDecision {
        active,
        candidate: candidate.map(|(index, record)| selected(index, record)),
        previous_approved: previous_approved.map(|(index, record)| selected(index, record)),
        slots: states,
    })
}

pub fn prepare_config_save(
    slots: &[ConfigSlot; CONFIG_SLOT_COUNT],
    config: ConfigBundle,
    approval: ConfigApproval,
) -> Result<PreparedConfigWrite, ConfigWriteError> {
    config.validate().map_err(ConfigWriteError::InvalidConfig)?;
    let decoded = [decode_config_slot(&slots[0]), decode_config_slot(&slots[1])];
    let newest = select_newest(&decoded, None).map_err(map_recovery_write_error)?;
    let sequence = if let Some((_, record)) = newest {
        record.sequence.wrapping_add(1)
    } else {
        1
    };
    let target_slot = select_write_target(&decoded, approval, newest)?;
    encode_record(
        target_slot,
        sequence,
        approval,
        config,
        CONFIG_SCHEMA_VERSION,
    )
}

/// Create a new approved record containing the previous approved bundle. The
/// old active slot is overwritten, so the prior bundle becomes the newest only
/// after the commit marker is complete.
pub fn prepare_config_rollback(
    slots: &[ConfigSlot; CONFIG_SLOT_COUNT],
) -> Result<PreparedConfigWrite, ConfigWriteError> {
    let decoded = [decode_config_slot(&slots[0]), decode_config_slot(&slots[1])];
    let active = select_newest(&decoded, Some(ConfigApproval::Approved))
        .map_err(map_recovery_write_error)?
        .ok_or(ConfigWriteError::RollbackUnavailable)?;
    let previous = select_other_approved(&decoded, active.0)
        .map_err(map_recovery_write_error)?
        .ok_or(ConfigWriteError::RollbackUnavailable)?;
    let sequence = active.1.sequence.wrapping_add(1);
    encode_record(
        active.0,
        sequence,
        ConfigApproval::Approved,
        previous.1.config,
        CONFIG_SCHEMA_VERSION,
    )
}

fn select_write_target(
    decoded: &[Result<DecodedConfigRecord, ConfigRecordError>; CONFIG_SLOT_COUNT],
    approval: ConfigApproval,
    newest: Option<(usize, DecodedConfigRecord)>,
) -> Result<usize, ConfigWriteError> {
    if approval == ConfigApproval::Candidate {
        if let Some((index, _)) = select_newest(decoded, Some(ConfigApproval::Candidate))
            .map_err(map_recovery_write_error)?
        {
            return Ok(index);
        }
    }
    if approval == ConfigApproval::Approved {
        if let Some((active_index, _)) = select_newest(decoded, Some(ConfigApproval::Approved))
            .map_err(map_recovery_write_error)?
        {
            return Ok(1 - active_index);
        }
    }
    if decoded[0].is_err() {
        return Ok(0);
    }
    if decoded[1].is_err() {
        return Ok(1);
    }
    Ok(1 - newest.ok_or(ConfigWriteError::AmbiguousSequence)?.0)
}

fn encode_record(
    target_slot: usize,
    sequence: u32,
    approval: ConfigApproval,
    config: ConfigBundle,
    schema_version: u16,
) -> Result<PreparedConfigWrite, ConfigWriteError> {
    let mut image = ConfigSlot::erased();
    let payload_size = encode_payload(
        &config,
        schema_version,
        &mut image.bytes[CONFIG_PAYLOAD_OFFSET..CONFIG_COMMIT_OFFSET],
    );
    write_u32(&mut image.bytes, HEADER_MAGIC_OFFSET, CONFIG_RECORD_MAGIC);
    write_u16(
        &mut image.bytes,
        HEADER_STORAGE_VERSION_OFFSET,
        CONFIG_STORAGE_FORMAT_VERSION,
    );
    write_u16(
        &mut image.bytes,
        HEADER_SCHEMA_VERSION_OFFSET,
        schema_version,
    );
    write_u16(
        &mut image.bytes,
        HEADER_SIZE_OFFSET,
        CONFIG_HEADER_SIZE as u16,
    );
    write_u16(
        &mut image.bytes,
        HEADER_PAYLOAD_SIZE_OFFSET,
        payload_size as u16,
    );
    write_u32(&mut image.bytes, HEADER_SEQUENCE_OFFSET, sequence);
    write_u32(&mut image.bytes, HEADER_APPROVAL_OFFSET, approval as u32);
    write_u32(
        &mut image.bytes,
        HEADER_BOARD_ID_OFFSET,
        config.board.board_id,
    );
    write_u32(
        &mut image.bytes,
        HEADER_MOTOR_ID_OFFSET,
        config.motor.motor_id,
    );
    let payload_crc =
        crc32(&image.bytes[CONFIG_PAYLOAD_OFFSET..CONFIG_PAYLOAD_OFFSET + payload_size]);
    write_u32(&mut image.bytes, HEADER_PAYLOAD_CRC_OFFSET, payload_crc);
    write_u32(&mut image.bytes, HEADER_RECORD_CRC_OFFSET, 0);
    write_u32(&mut image.bytes, HEADER_RESERVED_OFFSET, 0);
    let record_crc = record_crc32(&image.bytes, CONFIG_PAYLOAD_OFFSET + payload_size);
    write_u32(&mut image.bytes, HEADER_RECORD_CRC_OFFSET, record_crc);
    write_u32(&mut image.bytes, CONFIG_COMMIT_OFFSET, CONFIG_COMMIT_MAGIC);
    Ok(PreparedConfigWrite {
        target_slot,
        sequence,
        image,
        prefix_len: CONFIG_PAYLOAD_OFFSET + payload_size,
    })
}

fn encode_payload(config: &ConfigBundle, schema: u16, output: &mut [u8]) -> usize {
    let mut writer = PayloadWriter::new(output);
    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.bundle_revision);
        writer.u32(config.generated_by_version);
        writer.u32(config.board.version);
    }
    writer.u32(config.board.board_id);
    writer.u32(config.board.pwm_frequency_hz);
    writer.u32(config.board.control_frequency_hz);
    writer.u32(config.board.capability_flags);
    writer.f32(config.board.adc_reference_v);
    writer.f32(config.board.current_gain_a_per_count);
    writer.f32(config.board.bus_voltage_v_per_count);

    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.motor.version);
    }
    writer.u32(config.motor.motor_id);
    writer.u32(config.motor.pole_pairs);
    writer.f32(config.motor.phase_resistance_ohm);
    writer.f32(config.motor.d_inductance_h);
    writer.f32(config.motor.q_inductance_h);
    writer.f32(config.motor.flux_linkage_v_s);
    writer.f32(config.motor.continuous_current_a);
    writer.f32(config.motor.maximum_speed_rad_s);

    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.inverter.version);
    }
    writer.u32(config.inverter.inverter_id);
    writer.f32(config.inverter.maximum_phase_current_a);
    writer.f32(config.inverter.minimum_bus_voltage_v);
    writer.f32(config.inverter.maximum_bus_voltage_v);
    writer.f32(config.inverter.dead_time_s);
    writer.f32(config.inverter.transistor_drop_v);
    writer.f32(config.inverter.brake_resistance_ohm);
    writer.f32(config.inverter.maximum_duty);

    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.axis.version);
    }
    writer.u32(config.axis.axis_id);
    writer.u32(config.axis.feedback_mode);
    writer.u32(config.axis.direction as u32);
    writer.f32(config.axis.current_kp);
    writer.f32(config.axis.current_ki);
    writer.f32(config.axis.velocity_kp);
    writer.f32(config.axis.velocity_ki);
    writer.f32(config.axis.position_kp);
    writer.f32(config.axis.soft_limit_min_rad);
    writer.f32(config.axis.soft_limit_max_rad);

    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.app.version);
    }
    writer.u32(config.app.can_node_id);
    writer.u32(config.app.uart_baud);
    writer.u32(config.app.command_source_count);
    for source in config.app.command_sources {
        writer.u32(source.source_id);
        writer.u32(source.priority);
        writer.u32(source.permissions);
        writer.u32(source.lease_ms);
        writer.u32(source.command_timeout_ms);
    }

    if schema == CONFIG_SCHEMA_VERSION {
        writer.u32(config.calibration.version);
    }
    writer.u32(config.calibration.board_id);
    writer.u32(config.calibration.motor_id);
    writer.u32(config.calibration.valid_flags);
    for value in config.calibration.current_offset_counts {
        writer.f32(value);
    }
    for value in config.calibration.phase_voltage_gain {
        writer.f32(value);
    }
    for value in config.calibration.phase_voltage_offset_v {
        writer.f32(value);
    }
    writer.f32(config.calibration.encoder_offset_rad);
    writer.u32(config.calibration.encoder_counts_per_revolution);
    writer.u32(config.calibration.hall_sequence_packed);
    writer.position
}

fn decode_payload(payload: &[u8], schema: u16) -> Result<ConfigBundle, ConfigRecordError> {
    let mut reader = PayloadReader::new(payload);
    let (bundle_revision, generated_by_version, board_version) = if schema == CONFIG_SCHEMA_VERSION
    {
        (reader.u32()?, reader.u32()?, reader.u32()?)
    } else {
        (1, 1, CONFIG_GROUP_VERSION)
    };
    let board = BoardConfig {
        version: board_version,
        board_id: reader.u32()?,
        pwm_frequency_hz: reader.u32()?,
        control_frequency_hz: reader.u32()?,
        capability_flags: reader.u32()?,
        adc_reference_v: reader.f32()?,
        current_gain_a_per_count: reader.f32()?,
        bus_voltage_v_per_count: reader.f32()?,
    };
    let motor = MotorConfig {
        version: if schema == CONFIG_SCHEMA_VERSION {
            reader.u32()?
        } else {
            CONFIG_GROUP_VERSION
        },
        motor_id: reader.u32()?,
        pole_pairs: reader.u32()?,
        phase_resistance_ohm: reader.f32()?,
        d_inductance_h: reader.f32()?,
        q_inductance_h: reader.f32()?,
        flux_linkage_v_s: reader.f32()?,
        continuous_current_a: reader.f32()?,
        maximum_speed_rad_s: reader.f32()?,
    };
    let inverter = InverterConfig {
        version: if schema == CONFIG_SCHEMA_VERSION {
            reader.u32()?
        } else {
            CONFIG_GROUP_VERSION
        },
        inverter_id: reader.u32()?,
        maximum_phase_current_a: reader.f32()?,
        minimum_bus_voltage_v: reader.f32()?,
        maximum_bus_voltage_v: reader.f32()?,
        dead_time_s: reader.f32()?,
        transistor_drop_v: reader.f32()?,
        brake_resistance_ohm: reader.f32()?,
        maximum_duty: reader.f32()?,
    };
    let axis = AxisConfig {
        version: if schema == CONFIG_SCHEMA_VERSION {
            reader.u32()?
        } else {
            CONFIG_GROUP_VERSION
        },
        axis_id: reader.u32()?,
        feedback_mode: reader.u32()?,
        direction: reader.u32()? as i32,
        current_kp: reader.f32()?,
        current_ki: reader.f32()?,
        velocity_kp: reader.f32()?,
        velocity_ki: reader.f32()?,
        position_kp: reader.f32()?,
        soft_limit_min_rad: reader.f32()?,
        soft_limit_max_rad: reader.f32()?,
    };
    let app_version = if schema == CONFIG_SCHEMA_VERSION {
        reader.u32()?
    } else {
        CONFIG_GROUP_VERSION
    };
    let can_node_id = reader.u32()?;
    let uart_baud = reader.u32()?;
    let command_source_count = reader.u32()?;
    let mut command_sources = [CommandSourceConfig::default(); CONFIG_MAX_COMMAND_SOURCES];
    for source in &mut command_sources {
        *source = CommandSourceConfig {
            source_id: reader.u32()?,
            priority: reader.u32()?,
            permissions: reader.u32()?,
            lease_ms: reader.u32()?,
            command_timeout_ms: reader.u32()?,
        };
    }
    let calibration_version = if schema == CONFIG_SCHEMA_VERSION {
        reader.u32()?
    } else {
        CONFIG_GROUP_VERSION
    };
    let calibration_board_id = reader.u32()?;
    let calibration_motor_id = reader.u32()?;
    let valid_flags = reader.u32()?;
    let mut current_offset_counts = [0.0; 3];
    let mut phase_voltage_gain = [0.0; 3];
    let mut phase_voltage_offset_v = [0.0; 3];
    reader.f32_array(&mut current_offset_counts)?;
    reader.f32_array(&mut phase_voltage_gain)?;
    reader.f32_array(&mut phase_voltage_offset_v)?;
    let config = ConfigBundle {
        bundle_revision,
        generated_by_version,
        board,
        motor,
        inverter,
        axis,
        app: AppConfig {
            version: app_version,
            can_node_id,
            uart_baud,
            command_source_count,
            command_sources,
        },
        calibration: CalibrationData {
            version: calibration_version,
            board_id: calibration_board_id,
            motor_id: calibration_motor_id,
            valid_flags,
            current_offset_counts,
            phase_voltage_gain,
            phase_voltage_offset_v,
            encoder_offset_rad: reader.f32()?,
            encoder_counts_per_revolution: reader.u32()?,
            hall_sequence_packed: reader.u32()?,
        },
    };
    if reader.position != payload.len() {
        return Err(ConfigRecordError::PayloadLength);
    }
    Ok(config)
}

fn select_newest(
    decoded: &[Result<DecodedConfigRecord, ConfigRecordError>; CONFIG_SLOT_COUNT],
    approval: Option<ConfigApproval>,
) -> Result<Option<(usize, DecodedConfigRecord)>, ConfigRecoveryError> {
    let left = decoded[0]
        .as_ref()
        .ok()
        .copied()
        .filter(|record| approval.is_none_or(|required| record.approval == required));
    let right = decoded[1]
        .as_ref()
        .ok()
        .copied()
        .filter(|record| approval.is_none_or(|required| record.approval == required));
    match (left, right) {
        (None, None) => Ok(None),
        (Some(record), None) => Ok(Some((0, record))),
        (None, Some(record)) => Ok(Some((1, record))),
        (Some(left), Some(right)) => match sequence_order(left.sequence, right.sequence)? {
            SequenceOrder::LeftNewer => Ok(Some((0, left))),
            SequenceOrder::RightNewer => Ok(Some((1, right))),
        },
    }
}

fn select_other_approved(
    decoded: &[Result<DecodedConfigRecord, ConfigRecordError>; CONFIG_SLOT_COUNT],
    active_index: usize,
) -> Result<Option<(usize, DecodedConfigRecord)>, ConfigRecoveryError> {
    let other_index = 1 - active_index;
    Ok(decoded[other_index]
        .as_ref()
        .ok()
        .copied()
        .filter(|record| record.approval == ConfigApproval::Approved)
        .map(|record| (other_index, record)))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SequenceOrder {
    LeftNewer,
    RightNewer,
}

fn sequence_order(left: u32, right: u32) -> Result<SequenceOrder, ConfigRecoveryError> {
    let distance = right.wrapping_sub(left);
    if distance == 0 || distance == WRAPPING_HALF_RANGE {
        return Err(ConfigRecoveryError::AmbiguousSequence);
    }
    if distance < WRAPPING_HALF_RANGE {
        Ok(SequenceOrder::RightNewer)
    } else {
        Ok(SequenceOrder::LeftNewer)
    }
}

fn selected(index: usize, record: DecodedConfigRecord) -> SelectedConfig {
    SelectedConfig {
        source: ConfigSource::from_slot(index),
        sequence: Some(record.sequence),
        migrated: record.migrated,
        config: record.config,
    }
}

fn slot_state(record: &Result<DecodedConfigRecord, ConfigRecordError>) -> SlotState {
    match record {
        Ok(record) => SlotState::Valid {
            sequence: record.sequence,
            approval: record.approval,
            migrated: record.migrated,
        },
        Err(ConfigRecordError::Erased) => SlotState::Erased,
        Err(error) => SlotState::Invalid(*error),
    }
}

fn map_recovery_write_error(error: ConfigRecoveryError) -> ConfigWriteError {
    match error {
        ConfigRecoveryError::AmbiguousSequence => ConfigWriteError::AmbiguousSequence,
        ConfigRecoveryError::InvalidFactory(error) => ConfigWriteError::InvalidConfig(error),
    }
}

fn finite_positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn finite_nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

fn all_finite(values: &[f32]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn hall_sequence_is_valid(packed: u32) -> bool {
    let mut seen = 0_u32;
    for index in 0..6 {
        let sector = (packed >> (index * 4)) & 0xF;
        if sector >= 6 || seen & (1 << sector) != 0 {
            return false;
        }
        seen |= 1 << sector;
    }
    packed >> 24 == 0 && seen == 0x3F
}

fn permissions_from_bits(bits: u32) -> Option<CommandPermissions> {
    if bits == 0 || bits & !COMMAND_SOURCE_PERMISSION_KNOWN_MASK != 0 {
        return None;
    }
    let ordered = [
        (
            COMMAND_SOURCE_PERMISSION_RELEASE,
            CommandPermissions::RELEASE,
        ),
        (
            COMMAND_SOURCE_PERMISSION_AXIS_REQUEST,
            CommandPermissions::AXIS_REQUEST,
        ),
        (
            COMMAND_SOURCE_PERMISSION_SETPOINT,
            CommandPermissions::SETPOINT,
        ),
        (
            COMMAND_SOURCE_PERMISSION_CLEAR_FAULT,
            CommandPermissions::CLEAR_FAULT,
        ),
        (
            COMMAND_SOURCE_PERMISSION_EMERGENCY_STOP,
            CommandPermissions::EMERGENCY_STOP,
        ),
    ];
    let mut result: Option<CommandPermissions> = None;
    for (mask, permission) in ordered {
        if bits & mask != 0 {
            result = Some(result.map_or(permission, |current| current.union(permission)));
        }
    }
    result
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn record_crc32(bytes: &[u8; CONFIG_SLOT_SIZE], payload_end: usize) -> u32 {
    let mut crc = 0xFFFF_FFFF_u32;
    for (index, byte) in bytes[..payload_end].iter().enumerate() {
        let value = if (HEADER_RECORD_CRC_OFFSET..HEADER_RECORD_CRC_OFFSET + 4).contains(&index) {
            0
        } else {
            *byte
        };
        crc ^= u32::from(value);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn read_u16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn read_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

fn write_u16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn write_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

struct PayloadWriter<'a> {
    bytes: &'a mut [u8],
    position: usize,
}

impl<'a> PayloadWriter<'a> {
    fn new(bytes: &'a mut [u8]) -> Self {
        Self { bytes, position: 0 }
    }

    fn u32(&mut self, value: u32) {
        write_u32(self.bytes, self.position, value);
        self.position += 4;
    }

    fn f32(&mut self, value: f32) {
        self.u32(value.to_bits());
    }
}

struct PayloadReader<'a> {
    bytes: &'a [u8],
    position: usize,
}

impl<'a> PayloadReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, position: 0 }
    }

    fn u32(&mut self) -> Result<u32, ConfigRecordError> {
        if self.position + 4 > self.bytes.len() {
            return Err(ConfigRecordError::PayloadLength);
        }
        let value = read_u32(self.bytes, self.position);
        self.position += 4;
        Ok(value)
    }

    fn f32(&mut self) -> Result<f32, ConfigRecordError> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn f32_array(&mut self, output: &mut [f32]) -> Result<(), ConfigRecordError> {
        for value in output {
            *value = self.f32()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ProductCommandKind;

    const BOARD_ID: u32 = 0x4311_6001;
    const MOTOR_ID: u32 = 0x2804_1007;

    fn bundle(revision: u32) -> ConfigBundle {
        let control_permissions = COMMAND_SOURCE_PERMISSION_KNOWN_MASK;
        ConfigBundle {
            bundle_revision: revision,
            generated_by_version: 1,
            board: BoardConfig {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                pwm_frequency_hz: 12_000,
                control_frequency_hz: 12_000,
                capability_flags: BOARD_CAP_THREE_SHUNT_CURRENT
                    | BOARD_CAP_PHASE_VOLTAGE_SENSE
                    | BOARD_CAP_ENCODER
                    | BOARD_CAP_HALL,
                adc_reference_v: 3.3,
                current_gain_a_per_count: 0.001,
                bus_voltage_v_per_count: 0.012,
            },
            motor: MotorConfig {
                version: CONFIG_GROUP_VERSION,
                motor_id: MOTOR_ID,
                pole_pairs: 7,
                phase_resistance_ohm: 4.9667,
                d_inductance_h: 0.0012,
                q_inductance_h: 0.0013,
                flux_linkage_v_s: 0.025,
                continuous_current_a: 0.8,
                maximum_speed_rad_s: 200.0,
            },
            inverter: InverterConfig {
                version: CONFIG_GROUP_VERSION,
                inverter_id: 0x1600_0001,
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
                        permissions: control_permissions,
                        lease_ms: 100,
                        command_timeout_ms: 250,
                    },
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                    CommandSourceConfig::default(),
                ],
            },
            calibration: CalibrationData {
                version: CONFIG_GROUP_VERSION,
                board_id: BOARD_ID,
                motor_id: MOTOR_ID,
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

    fn committed(sequence: u32, approval: ConfigApproval, config: ConfigBundle) -> ConfigSlot {
        encode_record(0, sequence, approval, config, CONFIG_SCHEMA_VERSION)
            .unwrap()
            .image
    }

    #[test]
    fn grouped_config_and_command_policy_validate() {
        let config = bundle(1);
        assert_eq!(config.validate(), Ok(()));
        assert_eq!(config.validate_for_arm(BOARD_ID, MOTOR_ID), Ok(()));
        let policy = config.app.command_sources[0].policy().unwrap();
        assert_eq!(policy.source_id, 1);
        assert!(policy.permissions.allows(ProductCommandKind::EmergencyStop));
    }

    #[test]
    fn arm_gate_rejects_identity_and_missing_feedback_calibration() {
        let mut config = bundle(1);
        assert_eq!(
            config.validate_for_arm(0xDEAD_BEEF, MOTOR_ID),
            Err(ConfigValidationError::HardwareIdentityMismatch)
        );
        config.axis.feedback_mode = FeedbackMode::IncrementalEncoder as u32;
        assert_eq!(
            config.validate_for_arm(BOARD_ID, MOTOR_ID),
            Err(ConfigValidationError::FeedbackCalibrationMissing)
        );
        config.calibration.valid_flags = CALIBRATION_ENCODER_VALID;
        config.calibration.encoder_counts_per_revolution = 4096;
        assert_eq!(config.validate_for_arm(BOARD_ID, MOTOR_ID), Ok(()));
    }

    #[test]
    fn invalid_groups_and_noncanonical_calibration_fail_closed() {
        let mut config = bundle(1);
        config.board.control_frequency_hz = 7_000;
        assert_eq!(config.validate(), Err(ConfigValidationError::Frequency));
        let mut config = bundle(1);
        config.app.command_sources[1].source_id = 9;
        assert_eq!(
            config.validate(),
            Err(ConfigValidationError::NonCanonicalUnusedSource)
        );
        let mut config = bundle(1);
        config.calibration.current_offset_counts[0] = 1.0;
        assert_eq!(
            config.validate(),
            Err(ConfigValidationError::CalibrationValue)
        );
    }

    #[test]
    fn record_round_trip_and_crc_corruption_are_deterministic() {
        let config = bundle(7);
        let slot = committed(42, ConfigApproval::Approved, config);
        let decoded = decode_config_slot(&slot).unwrap();
        assert_eq!(decoded.sequence, 42);
        assert_eq!(decoded.approval, ConfigApproval::Approved);
        assert_eq!(decoded.config, config);
        assert!(!decoded.migrated);

        let mut damaged = slot;
        damaged.bytes[CONFIG_PAYLOAD_OFFSET + 17] ^= 0x01;
        assert_eq!(
            decode_config_slot(&damaged),
            Err(ConfigRecordError::PayloadCrc)
        );
    }

    #[test]
    fn recovery_uses_factory_then_candidate_never_becomes_active() {
        let factory = bundle(1);
        let slots = [ConfigSlot::erased(), ConfigSlot::erased()];
        let recovered = recover_config(&slots, factory).unwrap();
        assert_eq!(recovered.active.source, ConfigSource::Factory);

        let candidate = bundle(2);
        let slots = [
            committed(1, ConfigApproval::Candidate, candidate),
            ConfigSlot::erased(),
        ];
        let recovered = recover_config(&slots, factory).unwrap();
        assert_eq!(recovered.active.source, ConfigSource::Factory);
        assert_eq!(recovered.candidate.unwrap().config, candidate);
    }

    #[test]
    fn recovery_selects_newest_approved_and_keeps_previous() {
        let old = bundle(1);
        let new = bundle(2);
        let slots = [
            committed(7, ConfigApproval::Approved, old),
            committed(8, ConfigApproval::Approved, new),
        ];
        let recovered = recover_config(&slots, bundle(99)).unwrap();
        assert_eq!(recovered.active.config, new);
        assert_eq!(recovered.active.source, ConfigSource::Slot1);
        assert_eq!(recovered.previous_approved.unwrap().config, old);
    }

    #[test]
    fn corrupt_newest_automatically_falls_back_to_last_known_good() {
        let old = bundle(1);
        let mut newest = committed(11, ConfigApproval::Approved, bundle(2));
        newest.bytes[CONFIG_PAYLOAD_OFFSET] ^= 0x80;
        let slots = [committed(10, ConfigApproval::Approved, old), newest];
        let recovered = recover_config(&slots, bundle(99)).unwrap();
        assert_eq!(recovered.active.config, old);
        assert_eq!(recovered.active.source, ConfigSource::Slot0);
        assert!(matches!(recovered.slots[1], SlotState::Invalid(_)));
    }

    #[test]
    fn all_program_power_loss_points_preserve_old_approved_record() {
        let factory = bundle(99);
        let old = bundle(1);
        let initial = [
            committed(100, ConfigApproval::Approved, old),
            ConfigSlot::erased(),
        ];
        let next = bundle(2);
        let plan = prepare_config_save(&initial, next, ConfigApproval::Approved).unwrap();
        assert_eq!(plan.target_slot, 1);
        for cut_after in 0..plan.write_step_count() {
            let mut interrupted = initial;
            plan.apply_prefix(&mut interrupted, cut_after);
            let recovered = recover_config(&interrupted, factory).unwrap();
            assert_eq!(recovered.active.config, old, "cut_after={cut_after}");
        }
        let mut complete = initial;
        plan.apply_complete(&mut complete);
        assert_eq!(
            recover_config(&complete, factory).unwrap().active.config,
            next
        );
    }

    #[test]
    fn interrupted_erase_or_arbitrary_target_damage_preserves_other_slot() {
        let factory = bundle(99);
        let old = bundle(1);
        let initial = [
            committed(100, ConfigApproval::Approved, old),
            ConfigSlot::erased(),
        ];
        for damaged_byte in 0..CONFIG_SLOT_SIZE {
            let mut interrupted = initial;
            interrupted[1].bytes[damaged_byte] = 0;
            let recovered = recover_config(&interrupted, factory).unwrap();
            assert_eq!(recovered.active.config, old, "byte={damaged_byte}");
        }
    }

    #[test]
    fn candidate_updates_overwrite_candidate_not_active_approved() {
        let active = bundle(1);
        let first_candidate = bundle(2);
        let slots = [
            committed(10, ConfigApproval::Approved, active),
            committed(11, ConfigApproval::Candidate, first_candidate),
        ];
        let second_candidate = bundle(3);
        let plan =
            prepare_config_save(&slots, second_candidate, ConfigApproval::Candidate).unwrap();
        assert_eq!(plan.target_slot, 1);
        let mut complete = slots;
        plan.apply_complete(&mut complete);
        let recovered = recover_config(&complete, bundle(99)).unwrap();
        assert_eq!(recovered.active.config, active);
        assert_eq!(recovered.candidate.unwrap().config, second_candidate);
    }

    #[test]
    fn approved_save_preserves_active_until_commit() {
        let active = bundle(1);
        let candidate = bundle(2);
        let slots = [
            committed(10, ConfigApproval::Approved, active),
            committed(11, ConfigApproval::Candidate, candidate),
        ];
        let plan = prepare_config_save(&slots, candidate, ConfigApproval::Approved).unwrap();
        assert_eq!(plan.target_slot, 1);
        let mut before_commit = slots;
        plan.apply_prefix(&mut before_commit, plan.write_step_count() - 1);
        assert_eq!(
            recover_config(&before_commit, bundle(99))
                .unwrap()
                .active
                .config,
            active
        );
        plan.apply_complete(&mut before_commit);
        assert_eq!(
            recover_config(&before_commit, bundle(99))
                .unwrap()
                .active
                .config,
            candidate
        );
    }

    #[test]
    fn write_plan_exposes_commit_marker_only_as_the_last_four_steps() {
        let slots = [ConfigSlot::erased(), ConfigSlot::erased()];
        let plan = prepare_config_save(&slots, bundle(1), ConfigApproval::Approved).unwrap();
        assert_eq!(plan.write_byte(plan.write_step_count()), None);
        for step in 0..plan.write_step_count() - 4 {
            assert!((plan.write_byte(step).unwrap().offset as usize) < CONFIG_COMMIT_OFFSET);
        }
        for step in plan.write_step_count() - 4..plan.write_step_count() {
            assert!((plan.write_byte(step).unwrap().offset as usize) >= CONFIG_COMMIT_OFFSET);
        }
    }

    #[test]
    fn explicit_rollback_creates_a_new_approved_generation() {
        let old = bundle(1);
        let new = bundle(2);
        let slots = [
            committed(20, ConfigApproval::Approved, old),
            committed(21, ConfigApproval::Approved, new),
        ];
        let plan = prepare_config_rollback(&slots).unwrap();
        assert_eq!(plan.target_slot, 1);
        assert_eq!(plan.sequence, 22);
        let mut rolled_back = slots;
        plan.apply_complete(&mut rolled_back);
        let recovered = recover_config(&rolled_back, bundle(99)).unwrap();
        assert_eq!(recovered.active.config, old);
        assert_eq!(recovered.active.sequence, Some(22));
    }

    #[test]
    fn legacy_v0_record_migrates_in_ram_and_requests_resave() {
        let config = bundle(7);
        let slot = encode_record(
            0,
            9,
            ConfigApproval::Approved,
            config,
            CONFIG_LEGACY_SCHEMA_VERSION,
        )
        .unwrap()
        .image;
        let decoded = decode_config_slot(&slot).unwrap();
        assert!(decoded.migrated);
        assert_eq!(decoded.schema_version, CONFIG_LEGACY_SCHEMA_VERSION);
        assert_eq!(decoded.config.bundle_revision, 1);
        assert_eq!(decoded.config.board.version, CONFIG_GROUP_VERSION);
        assert_eq!(decoded.config.board.board_id, config.board.board_id);
        assert_eq!(decoded.config.motor, config.motor);
    }

    #[test]
    fn unknown_schema_and_ambiguous_sequences_fail_closed() {
        let config = bundle(1);
        let mut unknown = committed(1, ConfigApproval::Approved, config);
        write_u16(&mut unknown.bytes, HEADER_SCHEMA_VERSION_OFFSET, 99);
        assert_eq!(
            decode_config_slot(&unknown),
            Err(ConfigRecordError::UnsupportedSchemaVersion)
        );

        let slots = [
            committed(1, ConfigApproval::Approved, config),
            committed(
                1_u32.wrapping_add(WRAPPING_HALF_RANGE),
                ConfigApproval::Approved,
                bundle(2),
            ),
        ];
        assert_eq!(
            recover_config(&slots, bundle(99)),
            Err(ConfigRecoveryError::AmbiguousSequence)
        );
    }

    #[test]
    fn sequence_wrap_selects_zero_as_newer_than_maximum() {
        let old = bundle(1);
        let new = bundle(2);
        let slots = [
            committed(u32::MAX, ConfigApproval::Approved, old),
            committed(0, ConfigApproval::Approved, new),
        ];
        assert_eq!(
            recover_config(&slots, bundle(99)).unwrap().active.config,
            new
        );
        let plan = prepare_config_save(&slots, bundle(3), ConfigApproval::Approved).unwrap();
        assert_eq!(plan.sequence, 1);
    }
}
