//! Read-only fixed-memory service used before physical transports exist.
//!
//! It proves request/response semantics without owning an RTOS task, a driver,
//! a command arbiter or a motor. Product commands are decoded only far enough
//! to return an explicit read-only result and are never dispatched.

use crate::{
    decode, decode_parameter_get_request, decode_parameter_set_request,
    decode_parameter_transaction_request, decode_product_command, encode, encode_capabilities,
    encode_command_result, encode_identity, encode_parameter_get_response,
    encode_parameter_transaction_response, encode_status, validate, CapabilitiesPayload,
    CommandResultPayload, Error, IdentityPayload, Message, ParameterGetResponsePayload,
    ParameterTransactionResponsePayload, StatusPayload, AXIS_DEVICE, COMMAND_RESULT_MALFORMED,
    COMMAND_RESULT_READ_ONLY, FLAG_ERROR, FLAG_RESPONSE, MESSAGE_CAPABILITIES_REQUEST,
    MESSAGE_CAPABILITIES_RESPONSE, MESSAGE_COMMAND_RESULT, MESSAGE_DISCOVER_REQUEST,
    MESSAGE_DISCOVER_RESPONSE, MESSAGE_PARAMETER_COMMIT_REQUEST, MESSAGE_PARAMETER_COMMIT_RESPONSE,
    MESSAGE_PARAMETER_GET_REQUEST, MESSAGE_PARAMETER_GET_RESPONSE, MESSAGE_PARAMETER_SET_REQUEST,
    MESSAGE_PARAMETER_SET_RESPONSE, MESSAGE_PRODUCT_COMMAND, MESSAGE_STATUS_REQUEST,
    MESSAGE_STATUS_RESPONSE, NODE_BROADCAST, PAYLOAD_SCHEMA_VERSION,
};

pub const FAKE_SERVICE_VERSION: u32 = 1;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FakeService {
    pub struct_size: u32,
    pub version: u32,
    pub identity: IdentityPayload,
    pub capabilities: CapabilitiesPayload,
    pub status: StatusPayload,
    pub responses: u32,
    pub rejected: u32,
    pub last_error: u32,
}

const _: () = assert!(core::mem::size_of::<FakeService>() == 196);

impl FakeService {
    pub fn new(
        identity: IdentityPayload,
        capabilities: CapabilitiesPayload,
        status: StatusPayload,
    ) -> Result<Self, Error> {
        identity.validate()?;
        capabilities.validate()?;
        status.validate()?;
        if identity.axis_count != capabilities.axis_count || status.axis_id >= identity.axis_count {
            return Err(Error::BadPayload);
        }
        Ok(Self {
            struct_size: core::mem::size_of::<Self>() as u32,
            version: FAKE_SERVICE_VERSION,
            identity,
            capabilities,
            status,
            responses: 0,
            rejected: 0,
            last_error: 0,
        })
    }

    fn is_valid(&self) -> bool {
        self.struct_size == core::mem::size_of::<Self>() as u32
            && self.version == FAKE_SERVICE_VERSION
    }

    fn response_base(&self, request: &Message, message_type: u32) -> Message {
        Message {
            flags: FLAG_RESPONSE,
            message_type,
            source_node: self.identity.node_id,
            destination_node: request.source_node,
            axis_id: request.axis_id,
            sequence: request.sequence,
            ..Message::new()
        }
    }

    fn fail(&mut self, error: Error) -> Result<Message, Error> {
        self.rejected = self.rejected.wrapping_add(1);
        self.last_error = error as u32;
        Err(error)
    }

    pub fn handle(&mut self, request: &Message) -> Result<Message, Error> {
        if !self.is_valid() {
            return Err(Error::InvalidArgument);
        }
        if let Err(error) = validate(request) {
            return self.fail(error);
        }
        let addressed = request.destination_node == self.identity.node_id
            || (request.message_type == MESSAGE_DISCOVER_REQUEST
                && request.destination_node == NODE_BROADCAST);
        if !addressed {
            return self.fail(Error::NotForNode);
        }

        let response = match request.message_type {
            MESSAGE_DISCOVER_REQUEST => {
                let mut response = self.response_base(request, MESSAGE_DISCOVER_RESPONSE);
                response.payload_length =
                    encode_identity(&self.identity, &mut response.payload)? as u32;
                response
            }
            MESSAGE_CAPABILITIES_REQUEST => {
                let mut response = self.response_base(request, MESSAGE_CAPABILITIES_RESPONSE);
                response.payload_length =
                    encode_capabilities(&self.capabilities, &mut response.payload)? as u32;
                response
            }
            MESSAGE_STATUS_REQUEST => {
                let mut response = self.response_base(request, MESSAGE_STATUS_RESPONSE);
                response.payload_length =
                    encode_status(&self.status, &mut response.payload)? as u32;
                response
            }
            MESSAGE_PRODUCT_COMMAND => {
                let decoded =
                    decode_product_command(&request.payload[..request.payload_length as usize]);
                let (source_id, axis_id, result) = match decoded {
                    Ok(command) if command.axis_id == request.axis_id => {
                        (command.source_id, command.axis_id, COMMAND_RESULT_READ_ONLY)
                    }
                    Ok(command) => (command.source_id, request.axis_id, COMMAND_RESULT_MALFORMED),
                    Err(_) => (0, request.axis_id, COMMAND_RESULT_MALFORMED),
                };
                let result_payload = CommandResultPayload {
                    schema_version: PAYLOAD_SCHEMA_VERSION,
                    request_sequence: request.sequence,
                    source_id,
                    axis_id,
                    result,
                    detail: 0,
                    accepted_sequence: 0,
                    reserved: 0,
                };
                let mut response = self.response_base(request, MESSAGE_COMMAND_RESULT);
                response.flags |= FLAG_ERROR;
                response.payload_length =
                    encode_command_result(&result_payload, &mut response.payload)? as u32;
                self.rejected = self.rejected.wrapping_add(1);
                response
            }
            MESSAGE_PARAMETER_GET_REQUEST => {
                let request_payload = decode_parameter_get_request(
                    &request.payload[..request.payload_length as usize],
                );
                let (request_id, result) = match request_payload {
                    Ok(value) => (value.request_id, COMMAND_RESULT_READ_ONLY),
                    Err(_) => (0, COMMAND_RESULT_MALFORMED),
                };
                let payload = ParameterGetResponsePayload {
                    schema_version: PAYLOAD_SCHEMA_VERSION,
                    request_id,
                    result,
                    ..ParameterGetResponsePayload::default()
                };
                let mut response = self.response_base(request, MESSAGE_PARAMETER_GET_RESPONSE);
                response.flags |= FLAG_ERROR;
                response.payload_length =
                    encode_parameter_get_response(&payload, &mut response.payload)? as u32;
                self.rejected = self.rejected.wrapping_add(1);
                response
            }
            MESSAGE_PARAMETER_SET_REQUEST => {
                let request_payload = decode_parameter_set_request(
                    &request.payload[..request.payload_length as usize],
                );
                let (request_id, result) = match request_payload {
                    Ok(value) => (value.request_id, COMMAND_RESULT_READ_ONLY),
                    Err(_) => (0, COMMAND_RESULT_MALFORMED),
                };
                let payload = ParameterTransactionResponsePayload {
                    schema_version: PAYLOAD_SCHEMA_VERSION,
                    request_id,
                    result,
                    ..ParameterTransactionResponsePayload::default()
                };
                let mut response = self.response_base(request, MESSAGE_PARAMETER_SET_RESPONSE);
                response.flags |= FLAG_ERROR;
                response.payload_length =
                    encode_parameter_transaction_response(&payload, &mut response.payload)? as u32;
                self.rejected = self.rejected.wrapping_add(1);
                response
            }
            MESSAGE_PARAMETER_COMMIT_REQUEST => {
                let request_payload = decode_parameter_transaction_request(
                    &request.payload[..request.payload_length as usize],
                );
                let (request_id, result) = match request_payload {
                    Ok(value) => (value.request_id, COMMAND_RESULT_READ_ONLY),
                    Err(_) => (0, COMMAND_RESULT_MALFORMED),
                };
                let payload = ParameterTransactionResponsePayload {
                    schema_version: PAYLOAD_SCHEMA_VERSION,
                    request_id,
                    result,
                    ..ParameterTransactionResponsePayload::default()
                };
                let mut response = self.response_base(request, MESSAGE_PARAMETER_COMMIT_RESPONSE);
                response.flags |= FLAG_ERROR;
                response.payload_length =
                    encode_parameter_transaction_response(&payload, &mut response.payload)? as u32;
                self.rejected = self.rejected.wrapping_add(1);
                response
            }
            _ => return self.fail(Error::BadMessageType),
        };
        if response.axis_id == AXIS_DEVICE && response.message_type == MESSAGE_COMMAND_RESULT {
            return self.fail(Error::BadAddress);
        }
        validate(&response)?;
        self.responses = self.responses.wrapping_add(1);
        self.last_error = 0;
        Ok(response)
    }

    pub fn handle_frame(
        &mut self,
        request_frame: &[u8],
        response_frame: &mut [u8],
    ) -> Result<usize, Error> {
        let request = match decode(request_frame) {
            Ok(value) => value,
            Err(error) => return self.fail(error).map(|_| 0),
        };
        let response = self.handle(&request)?;
        encode(&response, response_frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        decode_command_result, decode_identity, decode_parameter_transaction_response,
        encode_parameter_transaction_request, CapabilitiesPayload,
        ParameterTransactionRequestPayload, ProductCommandPayload, AXIS_REQUEST_KNOWN_MASK,
        COMMAND_KIND_KNOWN_MASK, CONTROL_MODE_KNOWN_MASK, FEEDBACK_MODE_KNOWN_MASK,
        INPUT_MODE_KNOWN_MASK, MAX_PAYLOAD_SIZE, PARAMETER_ACTION_BEGIN,
        PRODUCT_COMMAND_PAYLOAD_SIZE, TELEMETRY_VALID_DC_BUS_VOLTAGE,
    };

    fn service() -> FakeService {
        FakeService::new(
            IdentityPayload {
                schema_version: 1,
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
            },
            CapabilitiesPayload {
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
            },
            StatusPayload {
                schema_version: 1,
                axis_state: 1,
                telemetry_valid_flags: TELEMETRY_VALID_DC_BUS_VOLTAGE,
                dc_bus_voltage_v: 12.3,
                ..StatusPayload::default()
            },
        )
        .unwrap()
    }

    fn request(message_type: u32, axis_id: u32) -> Message {
        Message {
            flags: crate::FLAG_REQUEST,
            message_type,
            source_node: 42,
            destination_node: 1,
            axis_id,
            sequence: 9,
            ..Message::new()
        }
    }

    #[test]
    fn fake_service_answers_read_only_requests_and_preserves_correlation() {
        let mut service = service();
        let mut discover = request(MESSAGE_DISCOVER_REQUEST, AXIS_DEVICE);
        discover.destination_node = NODE_BROADCAST;
        let response = service.handle(&discover).unwrap();
        assert_eq!(response.message_type, MESSAGE_DISCOVER_RESPONSE);
        assert_eq!(response.destination_node, 42);
        assert_eq!(response.sequence, 9);
        assert_eq!(
            decode_identity(&response.payload[..response.payload_length as usize]).unwrap(),
            service.identity
        );
        assert_eq!(
            service
                .handle(&request(MESSAGE_CAPABILITIES_REQUEST, AXIS_DEVICE))
                .unwrap()
                .message_type,
            MESSAGE_CAPABILITIES_RESPONSE
        );
        assert_eq!(
            service
                .handle(&request(MESSAGE_STATUS_REQUEST, AXIS_DEVICE))
                .unwrap()
                .message_type,
            MESSAGE_STATUS_RESPONSE
        );
        assert_eq!(service.responses, 3);
    }

    #[test]
    fn fake_link_runs_complete_frame_decode_service_encode_path() {
        let mut service = service();
        let request = request(MESSAGE_STATUS_REQUEST, AXIS_DEVICE);
        let mut request_frame = [0_u8; crate::MAX_FRAME_SIZE];
        let request_length = crate::encode(&request, &mut request_frame).unwrap();
        let mut response_frame = [0_u8; crate::MAX_FRAME_SIZE];
        let response_length = service
            .handle_frame(
                &request_frame[..request_length],
                response_frame.as_mut_slice(),
            )
            .unwrap();
        let response = crate::decode(&response_frame[..response_length]).unwrap();
        assert_eq!(response.message_type, MESSAGE_STATUS_RESPONSE);
        assert_eq!(response.sequence, request.sequence);
        assert_eq!(service.responses, 1);
    }

    #[test]
    fn fake_service_never_dispatches_product_commands() {
        let mut service = service();
        let command = ProductCommandPayload {
            struct_size: PRODUCT_COMMAND_PAYLOAD_SIZE as u32,
            version: 1,
            source_id: 55,
            sequence: 4,
            created_at_ms: 100,
            valid_until_ms: 120,
            ..ProductCommandPayload::default()
        };
        let mut request = request(MESSAGE_PRODUCT_COMMAND, 0);
        request.payload_length =
            crate::encode_product_command(&command, &mut request.payload).unwrap() as u32;
        let response = service.handle(&request).unwrap();
        assert_eq!(response.message_type, MESSAGE_COMMAND_RESULT);
        assert_ne!(response.flags & FLAG_ERROR, 0);
        let result =
            decode_command_result(&response.payload[..response.payload_length as usize]).unwrap();
        assert_eq!(result.result, COMMAND_RESULT_READ_ONLY);
        assert_eq!(result.source_id, 55);
        assert_eq!(service.rejected, 1);
    }

    #[test]
    fn fake_service_rejects_other_nodes_and_returns_explicit_read_only_parameter_result() {
        let mut service = service();
        let mut wrong_node = request(MESSAGE_STATUS_REQUEST, AXIS_DEVICE);
        wrong_node.destination_node = 2;
        assert_eq!(service.handle(&wrong_node), Err(Error::NotForNode));
        let transaction = ParameterTransactionRequestPayload {
            schema_version: PAYLOAD_SCHEMA_VERSION,
            request_id: 77,
            action: PARAMETER_ACTION_BEGIN,
            expected_revision: 4,
            ..ParameterTransactionRequestPayload::default()
        };
        let mut parameter = request(MESSAGE_PARAMETER_COMMIT_REQUEST, 0);
        parameter.payload_length =
            encode_parameter_transaction_request(&transaction, &mut parameter.payload).unwrap()
                as u32;
        let response = service.handle(&parameter).unwrap();
        assert_eq!(response.message_type, MESSAGE_PARAMETER_COMMIT_RESPONSE);
        assert_ne!(response.flags & FLAG_ERROR, 0);
        let decoded = decode_parameter_transaction_response(
            &response.payload[..response.payload_length as usize],
        )
        .unwrap();
        assert_eq!(decoded.request_id, 77);
        assert_eq!(decoded.result, COMMAND_RESULT_READ_ONLY);
        assert_eq!(service.rejected, 2);
    }
}
