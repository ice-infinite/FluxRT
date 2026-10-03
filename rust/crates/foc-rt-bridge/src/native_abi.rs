use core::{mem::size_of, ptr, slice};

use foc_control::ProductCommand as ControlProductCommand;
use foc_protocol::{
    self, CapabilitiesPayload, FakeService, IdentityPayload, Message, ProductCommandPayload,
    StatusPayload, StreamDecoder,
};

pub const FOC_NATIVE_ABI_VERSION: u32 = foc_protocol::PROTOCOL_VERSION;
pub const FOC_NATIVE_STATUS_OK: u32 = 0;
pub const FOC_NATIVE_STATUS_NEED_MORE: u32 = 1;
pub const FOC_NATIVE_STATUS_FRAME_READY: u32 = 2;

const _: () = assert!(size_of::<Message>() == 292);
const _: () = assert!(size_of::<StreamDecoder>() == 316);
const _: () = assert!(size_of::<IdentityPayload>() == 48);
const _: () = assert!(size_of::<CapabilitiesPayload>() == 64);
const _: () = assert!(size_of::<StatusPayload>() == 64);
const _: () = assert!(size_of::<FakeService>() == 196);

fn status(error: foc_protocol::Error) -> u32 {
    error as u32
}

#[no_mangle]
pub extern "C" fn foc_rust_native_protocol_version() -> u32 {
    FOC_NATIVE_ABI_VERSION
}

#[no_mangle]
/// Initialize one caller-owned Native message object.
///
/// # Safety
///
/// `message` must be null or point to one writable, naturally aligned complete
/// `Message` exclusively owned by the caller for this call.
pub unsafe extern "C" fn foc_rust_native_message_init(message: *mut Message) -> u32 {
    if message.is_null() {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: the C ABI requires a writable, aligned complete Message object.
    unsafe { ptr::write(message, Message::new()) };
    FOC_NATIVE_STATUS_OK
}

#[no_mangle]
pub extern "C" fn foc_rust_native_frame_size(payload_length: u32) -> usize {
    foc_protocol::frame_size(payload_length).unwrap_or(0)
}

#[no_mangle]
/// Calculate CRC-32C over a caller-provided byte slice.
///
/// # Safety
///
/// When `length` is nonzero, `data` must point to at least `length` readable
/// bytes and remain valid for this call. A null pointer is accepted only for
/// the zero-length slice.
pub unsafe extern "C" fn foc_rust_native_crc32c(data: *const u8, length: usize) -> u32 {
    if data.is_null() {
        return if length == 0 {
            foc_protocol::crc32c(&[])
        } else {
            0
        };
    }
    if length > isize::MAX as usize {
        return 0;
    }
    // SAFETY: caller promises `length` readable bytes at `data` for this call.
    let bytes = unsafe { slice::from_raw_parts(data, length) };
    foc_protocol::crc32c(bytes)
}

#[no_mangle]
/// Encode one validated Native message into a caller-owned output buffer.
///
/// # Safety
///
/// `message` must point to a readable, aligned complete `Message`; `frame` must
/// point to `frame_capacity` writable bytes; `frame_length` must point to one
/// writable `usize`. The three regions must not overlap in a way that violates
/// Rust aliasing rules for the duration of this call.
pub unsafe extern "C" fn foc_rust_native_encode(
    message: *const Message,
    frame: *mut u8,
    frame_capacity: usize,
    frame_length: *mut usize,
) -> u32 {
    if !frame_length.is_null() {
        // SAFETY: non-null output follows the C ABI writable size_t contract.
        unsafe { ptr::write(frame_length, 0) };
    }
    if message.is_null()
        || frame.is_null()
        || frame_length.is_null()
        || frame_capacity > isize::MAX as usize
    {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: pointers are non-null and the caller owns complete objects/buffers.
    let message = unsafe { &*message };
    // SAFETY: caller promises `frame_capacity` writable bytes at `frame`.
    let output = unsafe { slice::from_raw_parts_mut(frame, frame_capacity) };
    match foc_protocol::encode(message, output) {
        Ok(length) => {
            // SAFETY: validated non-null writable output.
            unsafe { ptr::write(frame_length, length) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[no_mangle]
/// Decode one complete Native frame into a caller-owned message.
///
/// # Safety
///
/// `frame` must point to `frame_length` readable bytes and `message` must point
/// to one writable, aligned complete `Message`. Input and output must not
/// overlap for this call.
pub unsafe extern "C" fn foc_rust_native_decode(
    frame: *const u8,
    frame_length: usize,
    message: *mut Message,
) -> u32 {
    if frame.is_null() || message.is_null() || frame_length > isize::MAX as usize {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller promises `frame_length` readable bytes at `frame`.
    let input = unsafe { slice::from_raw_parts(frame, frame_length) };
    match foc_protocol::decode(input) {
        Ok(decoded) => {
            // SAFETY: caller supplies a writable, aligned complete Message.
            unsafe { ptr::write(message, decoded) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[no_mangle]
/// Initialize one caller-owned bounded stream decoder.
///
/// # Safety
///
/// `decoder` must be null or point to one writable, naturally aligned complete
/// `StreamDecoder` exclusively owned by the caller for this call.
pub unsafe extern "C" fn foc_rust_native_stream_init(decoder: *mut StreamDecoder) -> u32 {
    if decoder.is_null() {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller supplies a writable, aligned complete StreamDecoder.
    unsafe { ptr::write(decoder, StreamDecoder::new()) };
    FOC_NATIVE_STATUS_OK
}

#[no_mangle]
/// Feed one byte into the bounded stream decoder.
///
/// # Safety
///
/// `decoder` must point to a valid, initialized, writable `StreamDecoder` with
/// unique ownership for this call. `message` must point to a writable, aligned
/// complete `Message`; the objects must not overlap.
pub unsafe extern "C" fn foc_rust_native_stream_push(
    decoder: *mut StreamDecoder,
    byte: u8,
    message: *mut Message,
) -> u32 {
    if decoder.is_null() || message.is_null() {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: decoder has unique mutable ownership for this call.
    let decoder = unsafe { &mut *decoder };
    match decoder.push(byte) {
        Ok(Some(decoded)) => {
            // SAFETY: caller supplies a writable, aligned complete Message.
            unsafe { ptr::write(message, decoded) };
            FOC_NATIVE_STATUS_FRAME_READY
        }
        Ok(None) => FOC_NATIVE_STATUS_NEED_MORE,
        Err(error) => status(error),
    }
}

fn wire_command(command: &ControlProductCommand) -> ProductCommandPayload {
    ProductCommandPayload {
        struct_size: command.struct_size,
        version: command.version,
        axis_id: command.axis_id,
        source_id: command.source_id,
        sequence: command.sequence,
        created_at_ms: command.created_at_ms,
        valid_until_ms: command.valid_until_ms,
        command_kind: command.command_kind,
        axis_request: command.axis_request,
        control_mode: command.control_mode,
        input_mode: command.input_mode,
        feedback_mode: command.feedback_mode,
        flags: command.flags,
        duty_ref: command.duty_ref,
        voltage_d_ref_v: command.voltage_d_ref_v,
        voltage_q_ref_v: command.voltage_q_ref_v,
        current_d_ref_a: command.current_d_ref_a,
        current_q_ref_a: command.current_q_ref_a,
        torque_ref_nm: command.torque_ref_nm,
        velocity_ref_rad_s: command.velocity_ref_rad_s,
        position_ref_rad: command.position_ref_rad,
        velocity_feedforward_rad_s: command.velocity_feedforward_rad_s,
        torque_feedforward_nm: command.torque_feedforward_nm,
        current_limit_a: command.current_limit_a,
        torque_limit_nm: command.torque_limit_nm,
        velocity_limit_rad_s: command.velocity_limit_rad_s,
    }
}

fn control_command(command: ProductCommandPayload) -> ControlProductCommand {
    ControlProductCommand {
        struct_size: command.struct_size,
        version: command.version,
        axis_id: command.axis_id,
        source_id: command.source_id,
        sequence: command.sequence,
        created_at_ms: command.created_at_ms,
        valid_until_ms: command.valid_until_ms,
        command_kind: command.command_kind,
        axis_request: command.axis_request,
        control_mode: command.control_mode,
        input_mode: command.input_mode,
        feedback_mode: command.feedback_mode,
        flags: command.flags,
        duty_ref: command.duty_ref,
        voltage_d_ref_v: command.voltage_d_ref_v,
        voltage_q_ref_v: command.voltage_q_ref_v,
        current_d_ref_a: command.current_d_ref_a,
        current_q_ref_a: command.current_q_ref_a,
        torque_ref_nm: command.torque_ref_nm,
        velocity_ref_rad_s: command.velocity_ref_rad_s,
        position_ref_rad: command.position_ref_rad,
        velocity_feedforward_rad_s: command.velocity_feedforward_rad_s,
        torque_feedforward_nm: command.torque_feedforward_nm,
        current_limit_a: command.current_limit_a,
        torque_limit_nm: command.torque_limit_nm,
        velocity_limit_rad_s: command.velocity_limit_rad_s,
    }
}

#[no_mangle]
/// Encode the stable ProductCommand ABI field-by-field into its V1 wire payload.
///
/// # Safety
///
/// `command` must point to one readable aligned command; `payload` must point to
/// `payload_capacity` writable bytes and `payload_length` to one writable
/// `usize`. The regions must not overlap in a way that violates aliasing.
pub unsafe extern "C" fn foc_rust_native_encode_product_command(
    command: *const ControlProductCommand,
    payload: *mut u8,
    payload_capacity: usize,
    payload_length: *mut usize,
) -> u32 {
    if !payload_length.is_null() {
        // SAFETY: non-null output follows the C ABI writable size_t contract.
        unsafe { ptr::write(payload_length, 0) };
    }
    if command.is_null()
        || payload.is_null()
        || payload_length.is_null()
        || payload_capacity > isize::MAX as usize
    {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller supplies a readable aligned complete command.
    let command = unsafe { &*command };
    if command.validate().is_err() {
        return foc_protocol::Error::BadPayload as u32;
    }
    // SAFETY: caller supplies `payload_capacity` writable bytes.
    let output = unsafe { slice::from_raw_parts_mut(payload, payload_capacity) };
    match foc_protocol::encode_product_command(&wire_command(command), output) {
        Ok(length) => {
            // SAFETY: validated writable output.
            unsafe { ptr::write(payload_length, length) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[no_mangle]
/// Decode and semantically validate a ProductCommand V1 wire payload.
///
/// # Safety
///
/// `payload` must point to `payload_length` readable bytes and `command` to one
/// writable aligned complete command. Input and output must not overlap.
pub unsafe extern "C" fn foc_rust_native_decode_product_command(
    payload: *const u8,
    payload_length: usize,
    command: *mut ControlProductCommand,
) -> u32 {
    if payload.is_null() || command.is_null() || payload_length > isize::MAX as usize {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller supplies `payload_length` readable bytes.
    let input = unsafe { slice::from_raw_parts(payload, payload_length) };
    let decoded = match foc_protocol::decode_product_command(input) {
        Ok(value) => control_command(value),
        Err(error) => return status(error),
    };
    if decoded.validate().is_err() {
        return foc_protocol::Error::BadPayload as u32;
    }
    // SAFETY: caller supplies one writable aligned complete command.
    unsafe { ptr::write(command, decoded) };
    FOC_NATIVE_STATUS_OK
}

#[no_mangle]
/// Initialize the read-only no-transport Native service.
///
/// # Safety
///
/// The three input pointers must address readable aligned complete payloads and
/// `service` one writable aligned complete `FakeService`. None may overlap.
pub unsafe extern "C" fn foc_rust_native_fake_service_init(
    service: *mut FakeService,
    identity: *const IdentityPayload,
    capabilities: *const CapabilitiesPayload,
    service_status: *const StatusPayload,
) -> u32 {
    if service.is_null() || identity.is_null() || capabilities.is_null() || service_status.is_null()
    {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: all inputs are non-null readable aligned complete objects.
    let created = FakeService::new(unsafe { *identity }, unsafe { *capabilities }, unsafe {
        *service_status
    });
    match created {
        Ok(value) => {
            // SAFETY: caller supplies one writable aligned complete service.
            unsafe { ptr::write(service, value) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[no_mangle]
/// Replace only the read-only status snapshot after validating it.
///
/// # Safety
///
/// `service` must point to an initialized uniquely owned writable service and
/// `service_status` to one readable aligned complete status payload.
pub unsafe extern "C" fn foc_rust_native_fake_service_set_status(
    service: *mut FakeService,
    service_status: *const StatusPayload,
) -> u32 {
    if service.is_null() || service_status.is_null() {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller provides unique mutable service ownership for this call.
    let service = unsafe { &mut *service };
    // SAFETY: caller supplies a readable aligned complete status payload.
    let service_status = unsafe { *service_status };
    if service.struct_size != size_of::<FakeService>() as u32
        || service.version != foc_protocol::FAKE_SERVICE_VERSION
        || service_status.axis_id >= service.identity.axis_count
    {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    if let Err(error) = service_status.validate() {
        return status(error);
    }
    service.status = service_status;
    FOC_NATIVE_STATUS_OK
}

#[no_mangle]
/// Handle one already-decoded request with the read-only Fake service.
///
/// # Safety
///
/// `service` must point to an initialized uniquely owned writable service,
/// `request` to one readable message and `response` to one writable message.
/// The three objects must not overlap.
pub unsafe extern "C" fn foc_rust_native_fake_service_handle(
    service: *mut FakeService,
    request: *const Message,
    response: *mut Message,
) -> u32 {
    if service.is_null() || request.is_null() || response.is_null() {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller provides unique mutable service ownership and readable request.
    let service = unsafe { &mut *service };
    let request = unsafe { &*request };
    match service.handle(request) {
        Ok(value) => {
            // SAFETY: caller supplies one writable aligned complete response.
            unsafe { ptr::write(response, value) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[no_mangle]
/// Run one complete request-frame to response-frame transaction in memory.
///
/// # Safety
///
/// `service` must point to an initialized uniquely owned writable service;
/// `request_frame` must address `request_length` readable bytes;
/// `response_frame` must address `response_capacity` writable bytes; and
/// `response_length` must point to one writable `usize`. Regions must not
/// overlap in a way that violates Rust aliasing.
pub unsafe extern "C" fn foc_rust_native_fake_service_handle_frame(
    service: *mut FakeService,
    request_frame: *const u8,
    request_length: usize,
    response_frame: *mut u8,
    response_capacity: usize,
    response_length: *mut usize,
) -> u32 {
    if !response_length.is_null() {
        // SAFETY: non-null output follows the C ABI writable size_t contract.
        unsafe { ptr::write(response_length, 0) };
    }
    if service.is_null()
        || request_frame.is_null()
        || response_frame.is_null()
        || response_length.is_null()
        || request_length > isize::MAX as usize
        || response_capacity > isize::MAX as usize
    {
        return foc_protocol::Error::InvalidArgument as u32;
    }
    // SAFETY: caller provides a uniquely owned service and the declared buffers.
    let service = unsafe { &mut *service };
    let request = unsafe { slice::from_raw_parts(request_frame, request_length) };
    let response = unsafe { slice::from_raw_parts_mut(response_frame, response_capacity) };
    match service.handle_frame(request, response) {
        Ok(length) => {
            // SAFETY: validated writable output.
            unsafe { ptr::write(response_length, length) };
            FOC_NATIVE_STATUS_OK
        }
        Err(error) => status(error),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use foc_protocol::{
        AXIS_REQUEST_KNOWN_MASK, COMMAND_KIND_KNOWN_MASK, CONTROL_MODE_KNOWN_MASK,
        FEEDBACK_MODE_KNOWN_MASK, INPUT_MODE_KNOWN_MASK, MAX_PAYLOAD_SIZE, MESSAGE_STATUS_REQUEST,
        PAYLOAD_SCHEMA_VERSION, TELEMETRY_VALID_DC_BUS_VOLTAGE,
    };

    #[test]
    fn c_abi_round_trip_and_layout_are_stable() {
        let mut message = Message::new();
        assert_eq!(unsafe { foc_rust_native_message_init(&mut message) }, 0);
        message.flags = foc_protocol::FLAG_REQUEST;
        message.message_type = foc_protocol::MESSAGE_DISCOVER_REQUEST;
        message.source_node = 42;
        message.destination_node = foc_protocol::NODE_BROADCAST;
        message.sequence = 9;
        let mut frame = [0_u8; foc_protocol::MAX_FRAME_SIZE];
        let mut length = 0_usize;
        assert_eq!(
            unsafe {
                foc_rust_native_encode(&message, frame.as_mut_ptr(), frame.len(), &mut length)
            },
            FOC_NATIVE_STATUS_OK
        );
        let mut decoded = Message::new();
        assert_eq!(
            unsafe { foc_rust_native_decode(frame.as_ptr(), length, &mut decoded) },
            FOC_NATIVE_STATUS_OK
        );
        assert_eq!(decoded, message);
    }

    #[test]
    fn product_command_adapter_is_semantic_and_field_encoded() {
        let command = ControlProductCommand::default();
        let mut payload = [0_u8; foc_protocol::PRODUCT_COMMAND_PAYLOAD_SIZE];
        let mut length = 0_usize;
        assert_eq!(
            unsafe {
                foc_rust_native_encode_product_command(
                    &command,
                    payload.as_mut_ptr(),
                    payload.len(),
                    &mut length,
                )
            },
            FOC_NATIVE_STATUS_OK
        );
        assert_eq!(length, 104);
        assert_eq!(&payload[..4], &104_u32.to_le_bytes());
        let mut decoded = ControlProductCommand::default();
        assert_eq!(
            unsafe {
                foc_rust_native_decode_product_command(payload.as_ptr(), length, &mut decoded)
            },
            FOC_NATIVE_STATUS_OK
        );
        assert_eq!(decoded, command);
        let mut invalid = command;
        invalid.current_q_ref_a = f32::NAN;
        assert_eq!(
            unsafe {
                foc_rust_native_encode_product_command(
                    &invalid,
                    payload.as_mut_ptr(),
                    payload.len(),
                    &mut length,
                )
            },
            foc_protocol::Error::BadPayload as u32
        );
    }

    #[test]
    fn fake_service_abi_answers_status_without_transport_or_command_sink() {
        let identity = IdentityPayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            product_contract_version: 0x0001_0000,
            firmware_abi_version: 0x0015_0000,
            config_abi_version: 0x0004_0000,
            board_id: 0x4311_6001,
            motor_id: 0x2804_1007,
            inverter_id: 0x0016_0001,
            profile_revision: 7,
            firmware_revision: 0x0002_0602,
            build_id_crc32c: 0x1234_5678,
            axis_count: 1,
            node_id: 1,
        };
        let capabilities = CapabilitiesPayload {
            schema_version: 1,
            axis_count: 1,
            axis_request_mask: AXIS_REQUEST_KNOWN_MASK,
            control_mode_mask: CONTROL_MODE_KNOWN_MASK,
            input_mode_mask: INPUT_MODE_KNOWN_MASK,
            feedback_mode_mask: FEEDBACK_MODE_KNOWN_MASK,
            command_kind_mask: COMMAND_KIND_KNOWN_MASK,
            compiled_protocol_mask: 1,
            telemetry_valid_mask: TELEMETRY_VALID_DC_BUS_VOLTAGE,
            maximum_payload_size: MAX_PAYLOAD_SIZE as u32,
            minimum_status_period_ms: 10,
            ..CapabilitiesPayload::default()
        };
        let service_status = StatusPayload {
            schema_version: 1,
            axis_state: 1,
            telemetry_valid_flags: TELEMETRY_VALID_DC_BUS_VOLTAGE,
            dc_bus_voltage_v: 12.3,
            ..StatusPayload::default()
        };
        let mut service = unsafe { core::mem::zeroed::<FakeService>() };
        assert_eq!(
            unsafe {
                foc_rust_native_fake_service_init(
                    &mut service,
                    &identity,
                    &capabilities,
                    &service_status,
                )
            },
            FOC_NATIVE_STATUS_OK
        );
        let request = Message {
            flags: foc_protocol::FLAG_REQUEST,
            message_type: MESSAGE_STATUS_REQUEST,
            source_node: 42,
            destination_node: 1,
            axis_id: foc_protocol::AXIS_DEVICE,
            sequence: 9,
            ..Message::new()
        };
        let mut response = Message::new();
        assert_eq!(
            unsafe { foc_rust_native_fake_service_handle(&mut service, &request, &mut response) },
            FOC_NATIVE_STATUS_OK
        );
        assert_eq!(response.message_type, foc_protocol::MESSAGE_STATUS_RESPONSE);
        assert_eq!(service.responses, 1);

        let mut request_frame = [0_u8; foc_protocol::MAX_FRAME_SIZE];
        let request_length = foc_protocol::encode(&request, &mut request_frame).unwrap();
        let mut response_frame = [0_u8; foc_protocol::MAX_FRAME_SIZE];
        let mut response_length = 0_usize;
        assert_eq!(
            unsafe {
                foc_rust_native_fake_service_handle_frame(
                    &mut service,
                    request_frame.as_ptr(),
                    request_length,
                    response_frame.as_mut_ptr(),
                    response_frame.len(),
                    &mut response_length,
                )
            },
            FOC_NATIVE_STATUS_OK
        );
        assert_eq!(
            foc_protocol::decode(&response_frame[..response_length])
                .unwrap()
                .message_type,
            foc_protocol::MESSAGE_STATUS_RESPONSE
        );
        assert_eq!(service.responses, 2);
    }
}
