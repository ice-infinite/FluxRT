#![cfg_attr(not(test), no_std)]

//! Allocation-free, target-neutral FluxRT Native V1 framing.
//!
//! This crate owns wire framing and byte-stream resynchronization only. It has
//! no RTOS, transport, hardware, controller or Axis ownership dependency.

mod payload;
mod service;

pub use payload::*;
pub use service::*;

pub const PROTOCOL_VERSION: u32 = 0x0001_0000;
pub const MESSAGE_VERSION: u32 = 1;
pub const STREAM_VERSION: u32 = 1;
pub const WIRE_VERSION: u8 = 1;
pub const WIRE_HEADER_SIZE: usize = 20;
pub const WIRE_TRAILER_SIZE: usize = 4;
pub const MAX_PAYLOAD_SIZE: usize = 256;
pub const MAX_FRAME_SIZE: usize = WIRE_HEADER_SIZE + MAX_PAYLOAD_SIZE + WIRE_TRAILER_SIZE;

pub const NODE_BROADCAST: u32 = 0xffff;
pub const AXIS_DEVICE: u32 = 0xffff;
pub const AXIS_MAX: u32 = 255;

pub const FLAG_REQUEST: u32 = 1 << 0;
pub const FLAG_RESPONSE: u32 = 1 << 1;
pub const FLAG_ERROR: u32 = 1 << 2;
pub const FLAG_ACK_REQUIRED: u32 = 1 << 3;
pub const FLAG_KNOWN_MASK: u32 = FLAG_REQUEST | FLAG_RESPONSE | FLAG_ERROR | FLAG_ACK_REQUIRED;

pub const MESSAGE_DISCOVER_REQUEST: u32 = 0x0001;
pub const MESSAGE_DISCOVER_RESPONSE: u32 = 0x0002;
pub const MESSAGE_CAPABILITIES_REQUEST: u32 = 0x0011;
pub const MESSAGE_CAPABILITIES_RESPONSE: u32 = 0x0012;
pub const MESSAGE_STATUS_REQUEST: u32 = 0x0021;
pub const MESSAGE_STATUS_RESPONSE: u32 = 0x0022;
pub const MESSAGE_PRODUCT_COMMAND: u32 = 0x0031;
pub const MESSAGE_COMMAND_RESULT: u32 = 0x0032;
pub const MESSAGE_PARAMETER_GET_REQUEST: u32 = 0x0041;
pub const MESSAGE_PARAMETER_GET_RESPONSE: u32 = 0x0042;
pub const MESSAGE_PARAMETER_SET_REQUEST: u32 = 0x0043;
pub const MESSAGE_PARAMETER_SET_RESPONSE: u32 = 0x0044;
pub const MESSAGE_PARAMETER_COMMIT_REQUEST: u32 = 0x0045;
pub const MESSAGE_PARAMETER_COMMIT_RESPONSE: u32 = 0x0046;

const MAGIC_0: u8 = b'F';
const MAGIC_1: u8 = b'R';

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum Error {
    InvalidArgument = 3,
    BufferTooSmall = 4,
    BadMagic = 5,
    BadVersion = 6,
    BadHeader = 7,
    BadFlags = 8,
    BadMessageType = 9,
    BadAddress = 10,
    BadLength = 11,
    BadCrc = 12,
    BadPayload = 13,
    NotForNode = 14,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(C)]
pub struct Message {
    pub struct_size: u32,
    pub version: u32,
    pub flags: u32,
    pub message_type: u32,
    pub source_node: u32,
    pub destination_node: u32,
    pub axis_id: u32,
    pub sequence: u32,
    pub payload_length: u32,
    pub payload: [u8; MAX_PAYLOAD_SIZE],
}

impl Message {
    pub const fn new() -> Self {
        Self {
            struct_size: core::mem::size_of::<Self>() as u32,
            version: MESSAGE_VERSION,
            flags: 0,
            message_type: 0,
            source_node: 0,
            destination_node: 0,
            axis_id: AXIS_DEVICE,
            sequence: 0,
            payload_length: 0,
            payload: [0; MAX_PAYLOAD_SIZE],
        }
    }
}

impl Default for Message {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(C)]
pub struct StreamDecoder {
    pub struct_size: u32,
    pub version: u32,
    pub buffered_length: u32,
    pub expected_length: u32,
    pub frames_decoded: u32,
    pub dropped_bytes: u32,
    pub framing_errors: u32,
    pub crc_errors: u32,
    pub last_error: u32,
    pub buffer: [u8; MAX_FRAME_SIZE],
}

impl StreamDecoder {
    pub const fn new() -> Self {
        Self {
            struct_size: core::mem::size_of::<Self>() as u32,
            version: STREAM_VERSION,
            buffered_length: 0,
            expected_length: 0,
            frames_decoded: 0,
            dropped_bytes: 0,
            framing_errors: 0,
            crc_errors: 0,
            last_error: 0,
            buffer: [0; MAX_FRAME_SIZE],
        }
    }

    fn reset_frame(&mut self, possible_magic: u8) {
        self.buffered_length = 0;
        self.expected_length = 0;
        if possible_magic == MAGIC_0 {
            self.buffer[0] = possible_magic;
            self.buffered_length = 1;
        }
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<Message>, Error> {
        if self.struct_size != core::mem::size_of::<Self>() as u32 || self.version != STREAM_VERSION
        {
            return Err(Error::InvalidArgument);
        }
        if self.buffered_length == 0 {
            if byte != MAGIC_0 {
                self.dropped_bytes = self.dropped_bytes.wrapping_add(1);
                return Ok(None);
            }
            self.buffer[0] = byte;
            self.buffered_length = 1;
            return Ok(None);
        }
        if self.buffered_length == 1 {
            if byte != MAGIC_1 {
                self.dropped_bytes = self.dropped_bytes.wrapping_add(1);
                self.reset_frame(byte);
                return Ok(None);
            }
            self.buffer[1] = byte;
            self.buffered_length = 2;
            return Ok(None);
        }
        let index = self.buffered_length as usize;
        if index >= MAX_FRAME_SIZE {
            self.framing_errors = self.framing_errors.wrapping_add(1);
            self.last_error = Error::BadLength as u32;
            self.reset_frame(byte);
            return Err(Error::BadLength);
        }
        self.buffer[index] = byte;
        self.buffered_length += 1;

        if self.buffered_length as usize == WIRE_HEADER_SIZE {
            if self.buffer[2] != WIRE_VERSION {
                self.framing_errors = self.framing_errors.wrapping_add(1);
                self.last_error = Error::BadVersion as u32;
                self.reset_frame(byte);
                return Err(Error::BadVersion);
            }
            if self.buffer[3] as usize != WIRE_HEADER_SIZE {
                self.framing_errors = self.framing_errors.wrapping_add(1);
                self.last_error = Error::BadHeader as u32;
                self.reset_frame(byte);
                return Err(Error::BadHeader);
            }
            let payload_length = read_u16_le(&self.buffer[14..16]) as usize;
            let Some(expected) = frame_size(payload_length as u32) else {
                self.framing_errors = self.framing_errors.wrapping_add(1);
                self.last_error = Error::BadLength as u32;
                self.reset_frame(byte);
                return Err(Error::BadLength);
            };
            self.expected_length = expected as u32;
        }
        if self.expected_length == 0 || self.buffered_length < self.expected_length {
            return Ok(None);
        }
        let expected_length = self.expected_length as usize;
        match decode(&self.buffer[..expected_length]) {
            Ok(message) => {
                self.frames_decoded = self.frames_decoded.wrapping_add(1);
                self.last_error = 0;
                self.reset_frame(0);
                Ok(Some(message))
            }
            Err(error) => {
                if error == Error::BadCrc {
                    self.crc_errors = self.crc_errors.wrapping_add(1);
                } else {
                    self.framing_errors = self.framing_errors.wrapping_add(1);
                }
                self.last_error = error as u32;
                self.reset_frame(byte);
                Err(error)
            }
        }
    }
}

impl Default for StreamDecoder {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = assert!(core::mem::size_of::<Message>() == 292);
const _: () = assert!(core::mem::size_of::<StreamDecoder>() == 316);
const _: () = assert!(MAX_FRAME_SIZE == 280);

pub const fn frame_size(payload_length: u32) -> Option<usize> {
    if payload_length as usize > MAX_PAYLOAD_SIZE {
        None
    } else {
        Some(WIRE_HEADER_SIZE + payload_length as usize + WIRE_TRAILER_SIZE)
    }
}

pub fn crc32c(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for value in data {
        crc ^= u32::from(*value);
        for _ in 0..8 {
            let mask = 0_u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0x82f6_3b78 & mask);
        }
    }
    !crc
}

fn read_u16_le(data: &[u8]) -> u16 {
    u16::from_le_bytes([data[0], data[1]])
}

fn read_u32_le(data: &[u8]) -> u32 {
    u32::from_le_bytes([data[0], data[1], data[2], data[3]])
}

fn is_request(message_type: u32) -> bool {
    matches!(
        message_type,
        MESSAGE_DISCOVER_REQUEST
            | MESSAGE_CAPABILITIES_REQUEST
            | MESSAGE_STATUS_REQUEST
            | MESSAGE_PRODUCT_COMMAND
            | MESSAGE_PARAMETER_GET_REQUEST
            | MESSAGE_PARAMETER_SET_REQUEST
            | MESSAGE_PARAMETER_COMMIT_REQUEST
    )
}

fn is_response(message_type: u32) -> bool {
    matches!(
        message_type,
        MESSAGE_DISCOVER_RESPONSE
            | MESSAGE_CAPABILITIES_RESPONSE
            | MESSAGE_STATUS_RESPONSE
            | MESSAGE_COMMAND_RESULT
            | MESSAGE_PARAMETER_GET_RESPONSE
            | MESSAGE_PARAMETER_SET_RESPONSE
            | MESSAGE_PARAMETER_COMMIT_RESPONSE
    )
}

fn targets_device(message_type: u32) -> bool {
    matches!(
        message_type,
        MESSAGE_DISCOVER_REQUEST
            | MESSAGE_DISCOVER_RESPONSE
            | MESSAGE_CAPABILITIES_REQUEST
            | MESSAGE_CAPABILITIES_RESPONSE
            | MESSAGE_STATUS_REQUEST
            | MESSAGE_STATUS_RESPONSE
    )
}

pub fn validate(message: &Message) -> Result<(), Error> {
    if message.struct_size != core::mem::size_of::<Message>() as u32
        || message.version != MESSAGE_VERSION
    {
        return Err(Error::InvalidArgument);
    }
    if message.flags & !FLAG_KNOWN_MASK != 0 {
        return Err(Error::BadFlags);
    }
    let request = is_request(message.message_type);
    let response = is_response(message.message_type);
    if !request && !response {
        return Err(Error::BadMessageType);
    }
    if (request
        && (message.flags & FLAG_REQUEST == 0 || message.flags & (FLAG_RESPONSE | FLAG_ERROR) != 0))
        || (response
            && (message.flags & FLAG_RESPONSE == 0
                || message.flags & (FLAG_REQUEST | FLAG_ACK_REQUIRED) != 0))
    {
        return Err(Error::BadFlags);
    }
    if message.source_node == 0
        || message.source_node >= NODE_BROADCAST
        || message.destination_node == 0
        || message.destination_node > NODE_BROADCAST
        || (response && message.destination_node == NODE_BROADCAST)
    {
        return Err(Error::BadAddress);
    }
    if message.axis_id != AXIS_DEVICE && message.axis_id > AXIS_MAX {
        return Err(Error::BadAddress);
    }
    if targets_device(message.message_type) && message.axis_id != AXIS_DEVICE {
        return Err(Error::BadAddress);
    }
    if message.payload_length as usize > MAX_PAYLOAD_SIZE {
        return Err(Error::BadLength);
    }
    let expected_payload_length = match message.message_type {
        MESSAGE_DISCOVER_REQUEST | MESSAGE_CAPABILITIES_REQUEST | MESSAGE_STATUS_REQUEST => Some(0),
        MESSAGE_DISCOVER_RESPONSE => Some(IDENTITY_PAYLOAD_SIZE as u32),
        MESSAGE_CAPABILITIES_RESPONSE => Some(CAPABILITIES_PAYLOAD_SIZE as u32),
        MESSAGE_STATUS_RESPONSE => Some(STATUS_PAYLOAD_SIZE as u32),
        MESSAGE_PRODUCT_COMMAND => Some(PRODUCT_COMMAND_PAYLOAD_SIZE as u32),
        MESSAGE_COMMAND_RESULT => Some(COMMAND_RESULT_PAYLOAD_SIZE as u32),
        MESSAGE_PARAMETER_GET_REQUEST => Some(PARAMETER_GET_REQUEST_PAYLOAD_SIZE as u32),
        MESSAGE_PARAMETER_GET_RESPONSE => Some(PARAMETER_GET_RESPONSE_PAYLOAD_SIZE as u32),
        MESSAGE_PARAMETER_SET_REQUEST => Some(PARAMETER_SET_REQUEST_PAYLOAD_SIZE as u32),
        MESSAGE_PARAMETER_SET_RESPONSE
        | MESSAGE_PARAMETER_COMMIT_REQUEST
        | MESSAGE_PARAMETER_COMMIT_RESPONSE => Some(PARAMETER_TRANSACTION_PAYLOAD_SIZE as u32),
        _ => None,
    };
    if expected_payload_length.is_some_and(|length| message.payload_length != length) {
        return Err(Error::BadLength);
    }
    Ok(())
}

pub fn encode(message: &Message, frame: &mut [u8]) -> Result<usize, Error> {
    validate(message)?;
    let encoded_length = frame_size(message.payload_length).ok_or(Error::BadLength)?;
    if frame.len() < encoded_length {
        return Err(Error::BufferTooSmall);
    }
    frame[0] = MAGIC_0;
    frame[1] = MAGIC_1;
    frame[2] = WIRE_VERSION;
    frame[3] = WIRE_HEADER_SIZE as u8;
    frame[4..6].copy_from_slice(&(message.flags as u16).to_le_bytes());
    frame[6..8].copy_from_slice(&(message.message_type as u16).to_le_bytes());
    frame[8..10].copy_from_slice(&(message.source_node as u16).to_le_bytes());
    frame[10..12].copy_from_slice(&(message.destination_node as u16).to_le_bytes());
    frame[12..14].copy_from_slice(&(message.axis_id as u16).to_le_bytes());
    frame[14..16].copy_from_slice(&(message.payload_length as u16).to_le_bytes());
    frame[16..20].copy_from_slice(&message.sequence.to_le_bytes());
    let payload_end = WIRE_HEADER_SIZE + message.payload_length as usize;
    frame[WIRE_HEADER_SIZE..payload_end]
        .copy_from_slice(&message.payload[..message.payload_length as usize]);
    let checksum = crc32c(&frame[..payload_end]);
    frame[payload_end..encoded_length].copy_from_slice(&checksum.to_le_bytes());
    Ok(encoded_length)
}

pub fn decode(frame: &[u8]) -> Result<Message, Error> {
    if frame.len() < WIRE_HEADER_SIZE + WIRE_TRAILER_SIZE {
        return Err(Error::BadLength);
    }
    if frame[0] != MAGIC_0 || frame[1] != MAGIC_1 {
        return Err(Error::BadMagic);
    }
    if frame[2] != WIRE_VERSION {
        return Err(Error::BadVersion);
    }
    if frame[3] as usize != WIRE_HEADER_SIZE {
        return Err(Error::BadHeader);
    }
    let payload_length = u32::from(read_u16_le(&frame[14..16]));
    let expected_length = frame_size(payload_length).ok_or(Error::BadLength)?;
    if frame.len() != expected_length {
        return Err(Error::BadLength);
    }
    let payload_end = expected_length - WIRE_TRAILER_SIZE;
    if crc32c(&frame[..payload_end]) != read_u32_le(&frame[payload_end..expected_length]) {
        return Err(Error::BadCrc);
    }
    let mut message = Message::new();
    message.flags = u32::from(read_u16_le(&frame[4..6]));
    message.message_type = u32::from(read_u16_le(&frame[6..8]));
    message.source_node = u32::from(read_u16_le(&frame[8..10]));
    message.destination_node = u32::from(read_u16_le(&frame[10..12]));
    message.axis_id = u32::from(read_u16_le(&frame[12..14]));
    message.payload_length = payload_length;
    message.sequence = read_u32_le(&frame[16..20]);
    message.payload[..payload_length as usize]
        .copy_from_slice(&frame[WIRE_HEADER_SIZE..payload_end]);
    validate(&message)?;
    Ok(message)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DISCOVER_GOLDEN: [u8; 24] = [
        0x46, 0x52, 0x01, 0x14, 0x01, 0x00, 0x01, 0x00, 0x2a, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00,
        0x00, 0x12, 0x34, 0x56, 0x78, 0xe2, 0xd6, 0x3a, 0x11,
    ];

    fn discover() -> Message {
        Message {
            flags: FLAG_REQUEST,
            message_type: MESSAGE_DISCOVER_REQUEST,
            source_node: 42,
            destination_node: NODE_BROADCAST,
            sequence: 0x7856_3412,
            ..Message::new()
        }
    }

    #[test]
    fn crc_and_discovery_golden_are_frozen() {
        assert_eq!(crc32c(b"123456789"), 0xe306_9283);
        let mut frame = [0_u8; MAX_FRAME_SIZE];
        let length = encode(&discover(), &mut frame).unwrap();
        assert_eq!(&frame[..length], &DISCOVER_GOLDEN);
        assert_eq!(decode(&DISCOVER_GOLDEN).unwrap(), discover());
    }

    #[test]
    fn command_and_max_payload_round_trip() {
        let mut command = Message {
            flags: FLAG_REQUEST | FLAG_ACK_REQUIRED,
            message_type: MESSAGE_PRODUCT_COMMAND,
            source_node: 7,
            destination_node: 1,
            axis_id: 0,
            sequence: 9,
            payload_length: PRODUCT_COMMAND_PAYLOAD_SIZE as u32,
            ..Message::new()
        };
        for (index, value) in command.payload[..PRODUCT_COMMAND_PAYLOAD_SIZE]
            .iter_mut()
            .enumerate()
        {
            *value = (index as u8) ^ 0x5a;
        }
        let mut frame = [0_u8; MAX_FRAME_SIZE];
        let length = encode(&command, &mut frame).unwrap();
        assert_eq!(decode(&frame[..length]).unwrap(), command);

        let mut maximum = Message {
            flags: FLAG_RESPONSE,
            message_type: MESSAGE_PARAMETER_GET_RESPONSE,
            source_node: 1,
            destination_node: 7,
            payload_length: MAX_PAYLOAD_SIZE as u32,
            ..Message::new()
        };
        for (index, value) in maximum.payload.iter_mut().enumerate() {
            *value = index as u8;
        }
        let length = encode(&maximum, &mut frame).unwrap();
        assert_eq!(length, MAX_FRAME_SIZE);
        assert_eq!(decode(&frame).unwrap(), maximum);
    }

    #[test]
    fn structure_flags_address_and_lengths_fail_closed() {
        let mut message = discover();
        message.flags |= 1 << 31;
        assert_eq!(validate(&message), Err(Error::BadFlags));
        message = discover();
        message.flags = FLAG_RESPONSE;
        assert_eq!(validate(&message), Err(Error::BadFlags));
        message = discover();
        message.source_node = NODE_BROADCAST;
        assert_eq!(validate(&message), Err(Error::BadAddress));
        message = discover();
        message.axis_id = 0;
        assert_eq!(validate(&message), Err(Error::BadAddress));
        message = discover();
        message.payload_length = 1;
        assert_eq!(validate(&message), Err(Error::BadLength));
        let mut short = [0_u8; 10];
        assert_eq!(encode(&discover(), &mut short), Err(Error::BufferTooSmall));
    }

    #[test]
    fn every_single_bit_golden_mutation_is_rejected() {
        for byte_index in 0..DISCOVER_GOLDEN.len() {
            for bit in 0..8 {
                let mut mutated = DISCOVER_GOLDEN;
                mutated[byte_index] ^= 1 << bit;
                assert!(decode(&mutated).is_err());
            }
        }
    }

    #[test]
    fn stream_recovers_after_noise_and_crc_error() {
        let mut decoder = StreamDecoder::new();
        assert_eq!(decoder.push(0).unwrap(), None);
        let mut corrupt = DISCOVER_GOLDEN;
        corrupt[18] ^= 0x40;
        let mut error = None;
        for byte in corrupt {
            if let Err(value) = decoder.push(byte) {
                error = Some(value);
            }
        }
        assert_eq!(error, Some(Error::BadCrc));
        assert_eq!(decoder.crc_errors, 1);
        let mut result = None;
        for byte in DISCOVER_GOLDEN {
            if let Some(message) = decoder.push(byte).unwrap() {
                result = Some(message);
            }
        }
        assert_eq!(result, Some(discover()));
        assert_eq!(decoder.frames_decoded, 1);
        assert_eq!(decoder.buffered_length, 0);
    }
}
