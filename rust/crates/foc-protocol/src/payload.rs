//! Fixed-width FluxRT Native V1 payload schemas.
//!
//! These functions deliberately encode every field. The fact that the current
//! C/Rust structs are little-endian and four-byte aligned is never used as a
//! wire-format shortcut.

use crate::{Error, MAX_PAYLOAD_SIZE};

pub const PAYLOAD_SCHEMA_VERSION: u32 = 1;
pub const IDENTITY_PAYLOAD_SIZE: usize = 48;
pub const CAPABILITIES_PAYLOAD_SIZE: usize = 64;
pub const STATUS_PAYLOAD_SIZE: usize = 64;
pub const COMMAND_RESULT_PAYLOAD_SIZE: usize = 32;
pub const PRODUCT_COMMAND_PAYLOAD_SIZE: usize = 104;
pub const PARAMETER_GET_REQUEST_PAYLOAD_SIZE: usize = 32;
pub const PARAMETER_GET_RESPONSE_PAYLOAD_SIZE: usize = 256;
pub const PARAMETER_SET_REQUEST_PAYLOAD_SIZE: usize = 256;
pub const PARAMETER_TRANSACTION_PAYLOAD_SIZE: usize = 32;
pub const PARAMETER_CHUNK_DATA_SIZE: usize = 224;

pub const PARAMETER_GROUP_KNOWN_MASK: u32 = 0x7f;
pub const PARAMETER_SCOPE_ACTIVE: u32 = 0;
pub const PARAMETER_SCOPE_PENDING: u32 = 1;
pub const PARAMETER_ACTION_BEGIN: u32 = 0;
pub const PARAMETER_ACTION_VALIDATE: u32 = 1;
pub const PARAMETER_ACTION_APPLY: u32 = 2;
pub const PARAMETER_ACTION_COMMIT: u32 = 3;
pub const PARAMETER_ACTION_ROLLBACK: u32 = 4;
pub const PARAMETER_ACTION_MAX: u32 = PARAMETER_ACTION_ROLLBACK;

pub const DEVICE_FLAG_READY: u32 = 1 << 0;
pub const DEVICE_FLAG_CONFIG_VALID: u32 = 1 << 1;
pub const DEVICE_FLAG_OUTPUTS_ARMED: u32 = 1 << 2;
pub const DEVICE_FLAG_TRANSPORT_HEALTHY: u32 = 1 << 3;
pub const DEVICE_FLAG_KNOWN_MASK: u32 = DEVICE_FLAG_READY
    | DEVICE_FLAG_CONFIG_VALID
    | DEVICE_FLAG_OUTPUTS_ARMED
    | DEVICE_FLAG_TRANSPORT_HEALTHY;

pub const AXIS_REQUEST_KNOWN_MASK: u32 = (1 << 10) - 1;
pub const CONTROL_MODE_KNOWN_MASK: u32 = (1 << 7) - 1;
pub const INPUT_MODE_KNOWN_MASK: u32 = (1 << 7) - 1;
pub const FEEDBACK_MODE_KNOWN_MASK: u32 = (1 << 6) - 1;
pub const COMMAND_KIND_KNOWN_MASK: u32 = (1 << 5) - 1;
pub const TRANSPORT_KNOWN_MASK: u32 = (1 << 5) - 1;
pub const PROTOCOL_KNOWN_MASK: u32 = (1 << 5) - 1;
pub const EXTERNAL_INPUT_KNOWN_MASK: u32 = (1 << 4) - 1;
pub const TELEMETRY_VALID_KNOWN_MASK: u32 = (1 << 13) - 1;
pub const PRODUCT_FAULT_KNOWN_MASK: u32 = (1 << 15) - 1;
pub const PRODUCT_COMMAND_FLAG_KNOWN_MASK: u32 = (1 << 3) - 1;

pub const TELEMETRY_VALID_VELOCITY_MEASURED: u32 = 1 << 4;
pub const TELEMETRY_VALID_VELOCITY_ESTIMATED: u32 = 1 << 5;
pub const TELEMETRY_VALID_CURRENT_MEASURED: u32 = 1 << 9;
pub const TELEMETRY_VALID_DC_BUS_VOLTAGE: u32 = 1 << 11;
pub const TELEMETRY_VALID_MOTOR_TEMPERATURE: u32 = 1 << 12;

pub const COMMAND_RESULT_ACCEPTED: u32 = 0;
pub const COMMAND_RESULT_MALFORMED: u32 = 1;
pub const COMMAND_RESULT_UNSUPPORTED: u32 = 2;
pub const COMMAND_RESULT_UNAUTHORIZED: u32 = 3;
pub const COMMAND_RESULT_EXPIRED: u32 = 4;
pub const COMMAND_RESULT_BUSY: u32 = 5;
pub const COMMAND_RESULT_REJECTED: u32 = 6;
pub const COMMAND_RESULT_INTERNAL: u32 = 7;
pub const COMMAND_RESULT_READ_ONLY: u32 = 8;
pub const COMMAND_RESULT_MAX: u32 = COMMAND_RESULT_READ_ONLY;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct IdentityPayload {
    pub schema_version: u32,
    pub product_contract_version: u32,
    pub firmware_abi_version: u32,
    pub config_abi_version: u32,
    pub board_id: u32,
    pub motor_id: u32,
    pub inverter_id: u32,
    pub profile_revision: u32,
    pub firmware_revision: u32,
    pub build_id_crc32c: u32,
    pub axis_count: u32,
    pub node_id: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CapabilitiesPayload {
    pub schema_version: u32,
    pub axis_count: u32,
    pub axis_request_mask: u32,
    pub control_mode_mask: u32,
    pub input_mode_mask: u32,
    pub feedback_mode_mask: u32,
    pub command_kind_mask: u32,
    pub compiled_transport_mask: u32,
    pub board_transport_mask: u32,
    pub compiled_protocol_mask: u32,
    pub compiled_input_mask: u32,
    pub board_input_mask: u32,
    pub telemetry_valid_mask: u32,
    pub maximum_payload_size: u32,
    pub minimum_status_period_ms: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StatusPayload {
    pub schema_version: u32,
    pub device_flags: u32,
    pub uptime_ms: u32,
    pub config_revision: u32,
    pub axis_id: u32,
    pub axis_state: u32,
    pub control_mode: u32,
    pub input_mode: u32,
    pub feedback_mode: u32,
    pub active_source_id: u32,
    pub fault_flags: u32,
    pub telemetry_valid_flags: u32,
    pub dc_bus_voltage_v: f32,
    pub mechanical_velocity_rad_s: f32,
    pub current_q_a: f32,
    pub motor_temperature_c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct CommandResultPayload {
    pub schema_version: u32,
    pub request_sequence: u32,
    pub source_id: u32,
    pub axis_id: u32,
    pub result: u32,
    pub detail: u32,
    pub accepted_sequence: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProductCommandPayload {
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

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ParameterGetRequestPayload {
    pub schema_version: u32,
    pub request_id: u32,
    pub scope: u32,
    pub transaction_token: u32,
    pub group_mask: u32,
    pub offset: u32,
    pub length: u32,
    pub reserved: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ParameterGetResponsePayload {
    pub schema_version: u32,
    pub request_id: u32,
    pub result: u32,
    pub detail: u32,
    pub revision: u32,
    pub total_length: u32,
    pub chunk_offset: u32,
    pub chunk_length: u32,
    pub data: [u8; PARAMETER_CHUNK_DATA_SIZE],
}

impl Default for ParameterGetResponsePayload {
    fn default() -> Self {
        Self {
            schema_version: 0,
            request_id: 0,
            result: 0,
            detail: 0,
            revision: 0,
            total_length: 0,
            chunk_offset: 0,
            chunk_length: 0,
            data: [0; PARAMETER_CHUNK_DATA_SIZE],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ParameterSetRequestPayload {
    pub schema_version: u32,
    pub request_id: u32,
    pub transaction_token: u32,
    pub group: u32,
    pub offset: u32,
    pub total_length: u32,
    pub chunk_length: u32,
    pub flags: u32,
    pub data: [u8; PARAMETER_CHUNK_DATA_SIZE],
}

impl Default for ParameterSetRequestPayload {
    fn default() -> Self {
        Self {
            schema_version: 0,
            request_id: 0,
            transaction_token: 0,
            group: 0,
            offset: 0,
            total_length: 0,
            chunk_length: 0,
            flags: 0,
            data: [0; PARAMETER_CHUNK_DATA_SIZE],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ParameterTransactionRequestPayload {
    pub schema_version: u32,
    pub request_id: u32,
    pub transaction_token: u32,
    pub action: u32,
    pub expected_revision: u32,
    pub flags: u32,
    pub reserved0: u32,
    pub reserved1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ParameterTransactionResponsePayload {
    pub schema_version: u32,
    pub request_id: u32,
    pub result: u32,
    pub detail: u32,
    pub transaction_token: u32,
    pub changed_groups: u32,
    pub transaction_state: u32,
    pub active_revision: u32,
}

const _: () = assert!(core::mem::size_of::<IdentityPayload>() == IDENTITY_PAYLOAD_SIZE);
const _: () = assert!(core::mem::size_of::<CapabilitiesPayload>() == CAPABILITIES_PAYLOAD_SIZE);
const _: () = assert!(core::mem::size_of::<StatusPayload>() == STATUS_PAYLOAD_SIZE);
const _: () = assert!(core::mem::size_of::<CommandResultPayload>() == COMMAND_RESULT_PAYLOAD_SIZE);
const _: () =
    assert!(core::mem::size_of::<ProductCommandPayload>() == PRODUCT_COMMAND_PAYLOAD_SIZE);
const _: () = assert!(core::mem::size_of::<ParameterGetRequestPayload>() == 32);
const _: () = assert!(core::mem::size_of::<ParameterGetResponsePayload>() == 256);
const _: () = assert!(core::mem::size_of::<ParameterSetRequestPayload>() == 256);
const _: () = assert!(core::mem::size_of::<ParameterTransactionRequestPayload>() == 32);
const _: () = assert!(core::mem::size_of::<ParameterTransactionResponsePayload>() == 32);
const _: () = assert!(MAX_PAYLOAD_SIZE >= PRODUCT_COMMAND_PAYLOAD_SIZE);

fn put_u32(output: &mut [u8], offset: usize, value: u32) {
    output[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn get_u32(input: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        input[offset],
        input[offset + 1],
        input[offset + 2],
        input[offset + 3],
    ])
}

fn put_f32(output: &mut [u8], offset: usize, value: f32) {
    put_u32(output, offset, value.to_bits());
}

fn get_f32(input: &[u8], offset: usize) -> f32 {
    f32::from_bits(get_u32(input, offset))
}

fn require_size(data: &[u8], expected: usize) -> Result<(), Error> {
    if data.len() == expected {
        Ok(())
    } else {
        Err(Error::BadLength)
    }
}

fn require_output(output: &[u8], expected: usize) -> Result<(), Error> {
    if output.len() >= expected {
        Ok(())
    } else {
        Err(Error::BufferTooSmall)
    }
}

impl IdentityPayload {
    pub fn validate(&self) -> Result<(), Error> {
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.product_contract_version == 0
            || self.firmware_abi_version == 0
            || self.config_abi_version == 0
            || self.board_id == 0
            || self.motor_id == 0
            || self.inverter_id == 0
            || self.axis_count == 0
            || self.axis_count > 256
            || self.node_id == 0
            || self.node_id >= 0xffff
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl CapabilitiesPayload {
    pub fn validate(&self) -> Result<(), Error> {
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.axis_count == 0
            || self.axis_count > 256
            || self.axis_request_mask & !AXIS_REQUEST_KNOWN_MASK != 0
            || self.control_mode_mask & !CONTROL_MODE_KNOWN_MASK != 0
            || self.input_mode_mask & !INPUT_MODE_KNOWN_MASK != 0
            || self.feedback_mode_mask & !FEEDBACK_MODE_KNOWN_MASK != 0
            || self.command_kind_mask & !COMMAND_KIND_KNOWN_MASK != 0
            || self.compiled_transport_mask & !TRANSPORT_KNOWN_MASK != 0
            || self.board_transport_mask & !TRANSPORT_KNOWN_MASK != 0
            || self.board_transport_mask & !self.compiled_transport_mask != 0
            || self.compiled_protocol_mask & !PROTOCOL_KNOWN_MASK != 0
            || self.compiled_input_mask & !EXTERNAL_INPUT_KNOWN_MASK != 0
            || self.board_input_mask & !EXTERNAL_INPUT_KNOWN_MASK != 0
            || self.board_input_mask & !self.compiled_input_mask != 0
            || self.telemetry_valid_mask & !TELEMETRY_VALID_KNOWN_MASK != 0
            || self.maximum_payload_size != MAX_PAYLOAD_SIZE as u32
            || self.minimum_status_period_ms == 0
            || self.reserved != 0
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl StatusPayload {
    pub fn validate(&self) -> Result<(), Error> {
        let velocity_valid = self.telemetry_valid_flags
            & (TELEMETRY_VALID_VELOCITY_MEASURED | TELEMETRY_VALID_VELOCITY_ESTIMATED)
            != 0;
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.device_flags & !DEVICE_FLAG_KNOWN_MASK != 0
            || self.axis_id > 255
            || self.axis_state > 6
            || self.control_mode > 6
            || self.input_mode > 6
            || self.feedback_mode > 5
            || self.fault_flags & !PRODUCT_FAULT_KNOWN_MASK != 0
            || self.telemetry_valid_flags & !TELEMETRY_VALID_KNOWN_MASK != 0
            || !self.dc_bus_voltage_v.is_finite()
            || !self.mechanical_velocity_rad_s.is_finite()
            || !self.current_q_a.is_finite()
            || !self.motor_temperature_c.is_finite()
            || (self.telemetry_valid_flags & TELEMETRY_VALID_DC_BUS_VOLTAGE == 0
                && self.dc_bus_voltage_v != 0.0)
            || (!velocity_valid && self.mechanical_velocity_rad_s != 0.0)
            || (self.telemetry_valid_flags & TELEMETRY_VALID_CURRENT_MEASURED == 0
                && self.current_q_a != 0.0)
            || (self.telemetry_valid_flags & TELEMETRY_VALID_MOTOR_TEMPERATURE == 0
                && self.motor_temperature_c != 0.0)
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl CommandResultPayload {
    pub fn validate(&self) -> Result<(), Error> {
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.axis_id > 255
            || self.result > COMMAND_RESULT_MAX
            || self.reserved != 0
            || (self.result != COMMAND_RESULT_ACCEPTED && self.accepted_sequence != 0)
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ProductCommandPayload {
    pub fn validate_wire(&self) -> Result<(), Error> {
        let floats = [
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
        ];
        if self.struct_size != PRODUCT_COMMAND_PAYLOAD_SIZE as u32
            || self.version != 1
            || self.axis_id > 255
            || self.command_kind > 4
            || self.axis_request > 9
            || self.control_mode > 6
            || self.input_mode > 6
            || self.feedback_mode > 5
            || self.flags & !PRODUCT_COMMAND_FLAG_KNOWN_MASK != 0
            || floats.iter().any(|value| !value.is_finite())
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ParameterGetRequestPayload {
    pub fn validate(&self) -> Result<(), Error> {
        let scope_valid = (self.scope == PARAMETER_SCOPE_ACTIVE && self.transaction_token == 0)
            || (self.scope == PARAMETER_SCOPE_PENDING && self.transaction_token != 0);
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.group_mask == 0
            || self.group_mask & !PARAMETER_GROUP_KNOWN_MASK != 0
            || self.length == 0
            || self.length as usize > PARAMETER_CHUNK_DATA_SIZE
            || self.reserved != 0
            || !scope_valid
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ParameterGetResponsePayload {
    pub fn validate(&self) -> Result<(), Error> {
        let success = self.result == COMMAND_RESULT_ACCEPTED;
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.result > COMMAND_RESULT_MAX
            || self.chunk_length as usize > PARAMETER_CHUNK_DATA_SIZE
            || self.chunk_offset.saturating_add(self.chunk_length) > self.total_length
            || (!success
                && (self.revision != 0
                    || self.total_length != 0
                    || self.chunk_offset != 0
                    || self.chunk_length != 0))
            || self.data[self.chunk_length as usize..]
                .iter()
                .any(|value| *value != 0)
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ParameterSetRequestPayload {
    pub fn validate(&self) -> Result<(), Error> {
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.transaction_token == 0
            || self.group == 0
            || self.group & !PARAMETER_GROUP_KNOWN_MASK != 0
            || self.group.count_ones() != 1
            || self.total_length == 0
            || self.chunk_length == 0
            || self.chunk_length as usize > PARAMETER_CHUNK_DATA_SIZE
            || self.offset.saturating_add(self.chunk_length) > self.total_length
            || self.flags != 0
            || self.data[self.chunk_length as usize..]
                .iter()
                .any(|value| *value != 0)
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ParameterTransactionRequestPayload {
    pub fn validate(&self) -> Result<(), Error> {
        let token_valid = if self.action == PARAMETER_ACTION_BEGIN {
            self.transaction_token == 0
        } else {
            self.transaction_token != 0
        };
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.action > PARAMETER_ACTION_MAX
            || !token_valid
            || self.flags != 0
            || self.reserved0 != 0
            || self.reserved1 != 0
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

impl ParameterTransactionResponsePayload {
    pub fn validate(&self) -> Result<(), Error> {
        if self.schema_version != PAYLOAD_SCHEMA_VERSION
            || self.result > COMMAND_RESULT_MAX
            || self.changed_groups & !PARAMETER_GROUP_KNOWN_MASK != 0
            || (self.result != COMMAND_RESULT_ACCEPTED
                && (self.transaction_token != 0
                    || self.changed_groups != 0
                    || self.transaction_state != 0
                    || self.active_revision != 0))
        {
            Err(Error::BadPayload)
        } else {
            Ok(())
        }
    }
}

macro_rules! payload_codec {
    ($encode:ident, $decode:ident, $ty:ident, $size:expr,
     [$($field:ident),+ $(,)?], [$($float:ident),* $(,)?]) => {
        pub fn $encode(value: &$ty, output: &mut [u8]) -> Result<usize, Error> {
            value.validate()?;
            require_output(output, $size)?;
            let mut offset = 0usize;
            $(put_u32(output, offset, value.$field); offset += 4;)+
            $(put_f32(output, offset, value.$float); offset += 4;)*
            debug_assert_eq!(offset, $size);
            Ok($size)
        }

        pub fn $decode(input: &[u8]) -> Result<$ty, Error> {
            require_size(input, $size)?;
            let mut offset = 0usize;
            let value = $ty {
                $($field: { let v = get_u32(input, offset); offset += 4; v },)+
                $($float: { let v = get_f32(input, offset); offset += 4; v },)*
            };
            debug_assert_eq!(offset, $size);
            value.validate()?;
            Ok(value)
        }
    };
}

payload_codec!(
    encode_identity,
    decode_identity,
    IdentityPayload,
    IDENTITY_PAYLOAD_SIZE,
    [
        schema_version,
        product_contract_version,
        firmware_abi_version,
        config_abi_version,
        board_id,
        motor_id,
        inverter_id,
        profile_revision,
        firmware_revision,
        build_id_crc32c,
        axis_count,
        node_id
    ],
    []
);

payload_codec!(
    encode_capabilities,
    decode_capabilities,
    CapabilitiesPayload,
    CAPABILITIES_PAYLOAD_SIZE,
    [
        schema_version,
        axis_count,
        axis_request_mask,
        control_mode_mask,
        input_mode_mask,
        feedback_mode_mask,
        command_kind_mask,
        compiled_transport_mask,
        board_transport_mask,
        compiled_protocol_mask,
        compiled_input_mask,
        board_input_mask,
        telemetry_valid_mask,
        maximum_payload_size,
        minimum_status_period_ms,
        reserved
    ],
    []
);

payload_codec!(
    encode_status,
    decode_status,
    StatusPayload,
    STATUS_PAYLOAD_SIZE,
    [
        schema_version,
        device_flags,
        uptime_ms,
        config_revision,
        axis_id,
        axis_state,
        control_mode,
        input_mode,
        feedback_mode,
        active_source_id,
        fault_flags,
        telemetry_valid_flags
    ],
    [
        dc_bus_voltage_v,
        mechanical_velocity_rad_s,
        current_q_a,
        motor_temperature_c
    ]
);

payload_codec!(
    encode_command_result,
    decode_command_result,
    CommandResultPayload,
    COMMAND_RESULT_PAYLOAD_SIZE,
    [
        schema_version,
        request_sequence,
        source_id,
        axis_id,
        result,
        detail,
        accepted_sequence,
        reserved
    ],
    []
);

pub fn encode_product_command(
    value: &ProductCommandPayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate_wire()?;
    require_output(output, PRODUCT_COMMAND_PAYLOAD_SIZE)?;
    let words = [
        value.struct_size,
        value.version,
        value.axis_id,
        value.source_id,
        value.sequence,
        value.created_at_ms,
        value.valid_until_ms,
        value.command_kind,
        value.axis_request,
        value.control_mode,
        value.input_mode,
        value.feedback_mode,
        value.flags,
    ];
    for (index, word) in words.iter().enumerate() {
        put_u32(output, index * 4, *word);
    }
    let floats = [
        value.duty_ref,
        value.voltage_d_ref_v,
        value.voltage_q_ref_v,
        value.current_d_ref_a,
        value.current_q_ref_a,
        value.torque_ref_nm,
        value.velocity_ref_rad_s,
        value.position_ref_rad,
        value.velocity_feedforward_rad_s,
        value.torque_feedforward_nm,
        value.current_limit_a,
        value.torque_limit_nm,
        value.velocity_limit_rad_s,
    ];
    for (index, value) in floats.iter().enumerate() {
        put_f32(output, 52 + index * 4, *value);
    }
    Ok(PRODUCT_COMMAND_PAYLOAD_SIZE)
}

pub fn decode_product_command(input: &[u8]) -> Result<ProductCommandPayload, Error> {
    require_size(input, PRODUCT_COMMAND_PAYLOAD_SIZE)?;
    let value = ProductCommandPayload {
        struct_size: get_u32(input, 0),
        version: get_u32(input, 4),
        axis_id: get_u32(input, 8),
        source_id: get_u32(input, 12),
        sequence: get_u32(input, 16),
        created_at_ms: get_u32(input, 20),
        valid_until_ms: get_u32(input, 24),
        command_kind: get_u32(input, 28),
        axis_request: get_u32(input, 32),
        control_mode: get_u32(input, 36),
        input_mode: get_u32(input, 40),
        feedback_mode: get_u32(input, 44),
        flags: get_u32(input, 48),
        duty_ref: get_f32(input, 52),
        voltage_d_ref_v: get_f32(input, 56),
        voltage_q_ref_v: get_f32(input, 60),
        current_d_ref_a: get_f32(input, 64),
        current_q_ref_a: get_f32(input, 68),
        torque_ref_nm: get_f32(input, 72),
        velocity_ref_rad_s: get_f32(input, 76),
        position_ref_rad: get_f32(input, 80),
        velocity_feedforward_rad_s: get_f32(input, 84),
        torque_feedforward_nm: get_f32(input, 88),
        current_limit_a: get_f32(input, 92),
        torque_limit_nm: get_f32(input, 96),
        velocity_limit_rad_s: get_f32(input, 100),
    };
    value.validate_wire()?;
    Ok(value)
}

pub fn encode_parameter_get_request(
    value: &ParameterGetRequestPayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate()?;
    require_output(output, PARAMETER_GET_REQUEST_PAYLOAD_SIZE)?;
    let words = [
        value.schema_version,
        value.request_id,
        value.scope,
        value.transaction_token,
        value.group_mask,
        value.offset,
        value.length,
        value.reserved,
    ];
    put_words(output, &words);
    Ok(PARAMETER_GET_REQUEST_PAYLOAD_SIZE)
}

pub fn decode_parameter_get_request(input: &[u8]) -> Result<ParameterGetRequestPayload, Error> {
    require_size(input, PARAMETER_GET_REQUEST_PAYLOAD_SIZE)?;
    let value = ParameterGetRequestPayload {
        schema_version: get_u32(input, 0),
        request_id: get_u32(input, 4),
        scope: get_u32(input, 8),
        transaction_token: get_u32(input, 12),
        group_mask: get_u32(input, 16),
        offset: get_u32(input, 20),
        length: get_u32(input, 24),
        reserved: get_u32(input, 28),
    };
    value.validate()?;
    Ok(value)
}

pub fn encode_parameter_get_response(
    value: &ParameterGetResponsePayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate()?;
    require_output(output, PARAMETER_GET_RESPONSE_PAYLOAD_SIZE)?;
    put_words(
        output,
        &[
            value.schema_version,
            value.request_id,
            value.result,
            value.detail,
            value.revision,
            value.total_length,
            value.chunk_offset,
            value.chunk_length,
        ],
    );
    output[32..256].copy_from_slice(&value.data);
    Ok(PARAMETER_GET_RESPONSE_PAYLOAD_SIZE)
}

pub fn decode_parameter_get_response(input: &[u8]) -> Result<ParameterGetResponsePayload, Error> {
    require_size(input, PARAMETER_GET_RESPONSE_PAYLOAD_SIZE)?;
    let mut value = ParameterGetResponsePayload {
        schema_version: get_u32(input, 0),
        request_id: get_u32(input, 4),
        result: get_u32(input, 8),
        detail: get_u32(input, 12),
        revision: get_u32(input, 16),
        total_length: get_u32(input, 20),
        chunk_offset: get_u32(input, 24),
        chunk_length: get_u32(input, 28),
        ..ParameterGetResponsePayload::default()
    };
    value.data.copy_from_slice(&input[32..256]);
    value.validate()?;
    Ok(value)
}

pub fn encode_parameter_set_request(
    value: &ParameterSetRequestPayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate()?;
    require_output(output, PARAMETER_SET_REQUEST_PAYLOAD_SIZE)?;
    put_words(
        output,
        &[
            value.schema_version,
            value.request_id,
            value.transaction_token,
            value.group,
            value.offset,
            value.total_length,
            value.chunk_length,
            value.flags,
        ],
    );
    output[32..256].copy_from_slice(&value.data);
    Ok(PARAMETER_SET_REQUEST_PAYLOAD_SIZE)
}

pub fn decode_parameter_set_request(input: &[u8]) -> Result<ParameterSetRequestPayload, Error> {
    require_size(input, PARAMETER_SET_REQUEST_PAYLOAD_SIZE)?;
    let mut value = ParameterSetRequestPayload {
        schema_version: get_u32(input, 0),
        request_id: get_u32(input, 4),
        transaction_token: get_u32(input, 8),
        group: get_u32(input, 12),
        offset: get_u32(input, 16),
        total_length: get_u32(input, 20),
        chunk_length: get_u32(input, 24),
        flags: get_u32(input, 28),
        ..ParameterSetRequestPayload::default()
    };
    value.data.copy_from_slice(&input[32..256]);
    value.validate()?;
    Ok(value)
}

pub fn encode_parameter_transaction_request(
    value: &ParameterTransactionRequestPayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate()?;
    require_output(output, PARAMETER_TRANSACTION_PAYLOAD_SIZE)?;
    put_words(
        output,
        &[
            value.schema_version,
            value.request_id,
            value.transaction_token,
            value.action,
            value.expected_revision,
            value.flags,
            value.reserved0,
            value.reserved1,
        ],
    );
    Ok(PARAMETER_TRANSACTION_PAYLOAD_SIZE)
}

pub fn decode_parameter_transaction_request(
    input: &[u8],
) -> Result<ParameterTransactionRequestPayload, Error> {
    require_size(input, PARAMETER_TRANSACTION_PAYLOAD_SIZE)?;
    let value = ParameterTransactionRequestPayload {
        schema_version: get_u32(input, 0),
        request_id: get_u32(input, 4),
        transaction_token: get_u32(input, 8),
        action: get_u32(input, 12),
        expected_revision: get_u32(input, 16),
        flags: get_u32(input, 20),
        reserved0: get_u32(input, 24),
        reserved1: get_u32(input, 28),
    };
    value.validate()?;
    Ok(value)
}

pub fn encode_parameter_transaction_response(
    value: &ParameterTransactionResponsePayload,
    output: &mut [u8],
) -> Result<usize, Error> {
    value.validate()?;
    require_output(output, PARAMETER_TRANSACTION_PAYLOAD_SIZE)?;
    put_words(
        output,
        &[
            value.schema_version,
            value.request_id,
            value.result,
            value.detail,
            value.transaction_token,
            value.changed_groups,
            value.transaction_state,
            value.active_revision,
        ],
    );
    Ok(PARAMETER_TRANSACTION_PAYLOAD_SIZE)
}

pub fn decode_parameter_transaction_response(
    input: &[u8],
) -> Result<ParameterTransactionResponsePayload, Error> {
    require_size(input, PARAMETER_TRANSACTION_PAYLOAD_SIZE)?;
    let value = ParameterTransactionResponsePayload {
        schema_version: get_u32(input, 0),
        request_id: get_u32(input, 4),
        result: get_u32(input, 8),
        detail: get_u32(input, 12),
        transaction_token: get_u32(input, 16),
        changed_groups: get_u32(input, 20),
        transaction_state: get_u32(input, 24),
        active_revision: get_u32(input, 28),
    };
    value.validate()?;
    Ok(value)
}

fn put_words(output: &mut [u8], words: &[u32]) {
    for (index, value) in words.iter().copied().enumerate() {
        put_u32(output, index * 4, value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity() -> IdentityPayload {
        IdentityPayload {
            schema_version: 1,
            product_contract_version: 0x0001_0000,
            firmware_abi_version: 0x0016_0000,
            config_abi_version: 0x0004_0000,
            board_id: 0x4311_6001,
            motor_id: 0x2804_1007,
            inverter_id: 0x0016_0001,
            profile_revision: 7,
            firmware_revision: 0x0002_0602,
            build_id_crc32c: 0x1234_5678,
            axis_count: 1,
            node_id: 1,
        }
    }

    #[test]
    fn fixed_payloads_round_trip_and_reject_noncanonical_values() {
        let mut bytes = [0_u8; MAX_PAYLOAD_SIZE];
        assert_eq!(encode_identity(&identity(), &mut bytes), Ok(48));
        assert_eq!(decode_identity(&bytes[..48]), Ok(identity()));

        let capabilities = CapabilitiesPayload {
            schema_version: 1,
            axis_count: 1,
            axis_request_mask: AXIS_REQUEST_KNOWN_MASK,
            control_mode_mask: CONTROL_MODE_KNOWN_MASK,
            input_mode_mask: INPUT_MODE_KNOWN_MASK,
            feedback_mode_mask: 1,
            command_kind_mask: COMMAND_KIND_KNOWN_MASK,
            compiled_protocol_mask: 1,
            telemetry_valid_mask: TELEMETRY_VALID_DC_BUS_VOLTAGE,
            maximum_payload_size: 256,
            minimum_status_period_ms: 10,
            ..CapabilitiesPayload::default()
        };
        assert_eq!(encode_capabilities(&capabilities, &mut bytes), Ok(64));
        assert_eq!(decode_capabilities(&bytes[..64]), Ok(capabilities));

        let status = StatusPayload {
            schema_version: 1,
            device_flags: DEVICE_FLAG_READY | DEVICE_FLAG_CONFIG_VALID,
            axis_state: 1,
            telemetry_valid_flags: TELEMETRY_VALID_DC_BUS_VOLTAGE,
            dc_bus_voltage_v: 12.3,
            ..StatusPayload::default()
        };
        assert_eq!(encode_status(&status, &mut bytes), Ok(64));
        assert_eq!(decode_status(&bytes[..64]), Ok(status));
        let mut stale = status;
        stale.telemetry_valid_flags = 0;
        assert_eq!(stale.validate(), Err(Error::BadPayload));

        let result = CommandResultPayload {
            schema_version: 1,
            request_sequence: 9,
            source_id: 42,
            result: COMMAND_RESULT_READ_ONLY,
            ..CommandResultPayload::default()
        };
        assert_eq!(encode_command_result(&result, &mut bytes), Ok(32));
        assert_eq!(decode_command_result(&bytes[..32]), Ok(result));
    }

    #[test]
    fn product_command_is_field_encoded_and_rejects_nan() {
        let command = ProductCommandPayload {
            struct_size: 104,
            version: 1,
            source_id: 42,
            sequence: 7,
            created_at_ms: 100,
            valid_until_ms: 120,
            ..ProductCommandPayload::default()
        };
        let mut bytes = [0_u8; PRODUCT_COMMAND_PAYLOAD_SIZE];
        assert_eq!(encode_product_command(&command, &mut bytes), Ok(104));
        assert_eq!(get_u32(&bytes, 12), 42);
        assert_eq!(decode_product_command(&bytes), Ok(command));
        let mut invalid = command;
        invalid.current_q_ref_a = f32::NAN;
        assert_eq!(invalid.validate_wire(), Err(Error::BadPayload));
    }

    #[test]
    fn parameter_chunks_and_transactions_are_fixed_width_and_canonical() {
        let mut bytes = [0_u8; MAX_PAYLOAD_SIZE];
        let get = ParameterGetRequestPayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            request_id: 7,
            scope: PARAMETER_SCOPE_ACTIVE,
            group_mask: 1,
            length: PARAMETER_CHUNK_DATA_SIZE as u32,
            ..ParameterGetRequestPayload::default()
        };
        assert_eq!(encode_parameter_get_request(&get, &mut bytes), Ok(32));
        assert_eq!(decode_parameter_get_request(&bytes[..32]), Ok(get));

        let mut set = ParameterSetRequestPayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            request_id: 8,
            transaction_token: 99,
            group: 1 << 1,
            total_length: 36,
            chunk_length: 36,
            ..ParameterSetRequestPayload::default()
        };
        for (index, value) in set.data[..36].iter_mut().enumerate() {
            *value = index as u8;
        }
        assert_eq!(encode_parameter_set_request(&set, &mut bytes), Ok(256));
        assert_eq!(decode_parameter_set_request(&bytes), Ok(set));
        set.data[200] = 1;
        assert_eq!(set.validate(), Err(Error::BadPayload));

        let begin = ParameterTransactionRequestPayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            request_id: 9,
            action: PARAMETER_ACTION_BEGIN,
            expected_revision: 4,
            ..ParameterTransactionRequestPayload::default()
        };
        assert_eq!(
            encode_parameter_transaction_request(&begin, &mut bytes),
            Ok(32)
        );
        assert_eq!(
            decode_parameter_transaction_request(&bytes[..32]),
            Ok(begin)
        );
        let read_only = ParameterTransactionResponsePayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            request_id: 9,
            result: COMMAND_RESULT_READ_ONLY,
            ..ParameterTransactionResponsePayload::default()
        };
        assert_eq!(
            encode_parameter_transaction_response(&read_only, &mut bytes),
            Ok(32)
        );
        assert_eq!(
            decode_parameter_transaction_response(&bytes[..32]),
            Ok(read_only)
        );
    }
}
