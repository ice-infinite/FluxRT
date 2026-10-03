#ifndef FOC_NATIVE_BRIDGE_H
#define FOC_NATIVE_BRIDGE_H

/* P2.6A FluxRT Native V1 framing ABI.
 *
 * Rust owns validation, CRC-32C, framing and byte-stream resynchronization.
 * C transport drivers only move bytes and call this ABI. Decoding never
 * dispatches a command, acquires Axis ownership or arms the power stage.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_product_contract.h"

#define FOC_NATIVE_ABI_VERSION          (0x00010000UL)
#define FOC_NATIVE_MESSAGE_VERSION      (1UL)
#define FOC_NATIVE_STREAM_VERSION       (1UL)
#define FOC_NATIVE_WIRE_VERSION         (1UL)
#define FOC_NATIVE_WIRE_HEADER_SIZE     (20UL)
#define FOC_NATIVE_WIRE_TRAILER_SIZE    (4UL)
#define FOC_NATIVE_MAX_PAYLOAD_SIZE     (256UL)
#define FOC_NATIVE_MAX_FRAME_SIZE       (280UL)
#define FOC_NATIVE_NODE_BROADCAST       (0xFFFFUL)
#define FOC_NATIVE_AXIS_DEVICE          (0xFFFFUL)
#define FOC_NATIVE_PAYLOAD_SCHEMA_VERSION (1UL)
#define FOC_NATIVE_IDENTITY_PAYLOAD_SIZE  (48UL)
#define FOC_NATIVE_CAPABILITIES_PAYLOAD_SIZE (64UL)
#define FOC_NATIVE_STATUS_PAYLOAD_SIZE    (64UL)
#define FOC_NATIVE_COMMAND_RESULT_PAYLOAD_SIZE (32UL)
#define FOC_NATIVE_PRODUCT_COMMAND_PAYLOAD_SIZE (104UL)
#define FOC_NATIVE_PARAMETER_GET_REQUEST_PAYLOAD_SIZE (32UL)
#define FOC_NATIVE_PARAMETER_GET_RESPONSE_PAYLOAD_SIZE (256UL)
#define FOC_NATIVE_PARAMETER_SET_REQUEST_PAYLOAD_SIZE (256UL)
#define FOC_NATIVE_PARAMETER_TRANSACTION_PAYLOAD_SIZE (32UL)
#define FOC_NATIVE_PARAMETER_CHUNK_DATA_SIZE (224UL)
#define FOC_NATIVE_FAKE_SERVICE_VERSION   (1UL)

typedef uint32_t foc_native_status_t;
enum
{
    FOC_NATIVE_STATUS_OK = 0,
    FOC_NATIVE_STATUS_NEED_MORE = 1,
    FOC_NATIVE_STATUS_FRAME_READY = 2,
    FOC_NATIVE_STATUS_INVALID_ARGUMENT = 3,
    FOC_NATIVE_STATUS_BUFFER_TOO_SMALL = 4,
    FOC_NATIVE_STATUS_BAD_MAGIC = 5,
    FOC_NATIVE_STATUS_BAD_VERSION = 6,
    FOC_NATIVE_STATUS_BAD_HEADER = 7,
    FOC_NATIVE_STATUS_BAD_FLAGS = 8,
    FOC_NATIVE_STATUS_BAD_MESSAGE_TYPE = 9,
    FOC_NATIVE_STATUS_BAD_ADDRESS = 10,
    FOC_NATIVE_STATUS_BAD_LENGTH = 11,
    FOC_NATIVE_STATUS_BAD_CRC = 12,
    FOC_NATIVE_STATUS_BAD_PAYLOAD = 13,
    FOC_NATIVE_STATUS_NOT_FOR_NODE = 14,
};

enum
{
    FOC_NATIVE_FLAG_REQUEST = (1UL << 0),
    FOC_NATIVE_FLAG_RESPONSE = (1UL << 1),
    FOC_NATIVE_FLAG_ERROR = (1UL << 2),
    FOC_NATIVE_FLAG_ACK_REQUIRED = (1UL << 3),
    FOC_NATIVE_FLAG_KNOWN_MASK = 0x0FUL,
};

enum
{
    FOC_NATIVE_MESSAGE_DISCOVER_REQUEST = 0x0001,
    FOC_NATIVE_MESSAGE_DISCOVER_RESPONSE = 0x0002,
    FOC_NATIVE_MESSAGE_CAPABILITIES_REQUEST = 0x0011,
    FOC_NATIVE_MESSAGE_CAPABILITIES_RESPONSE = 0x0012,
    FOC_NATIVE_MESSAGE_STATUS_REQUEST = 0x0021,
    FOC_NATIVE_MESSAGE_STATUS_RESPONSE = 0x0022,
    FOC_NATIVE_MESSAGE_PRODUCT_COMMAND = 0x0031,
    FOC_NATIVE_MESSAGE_COMMAND_RESULT = 0x0032,
    FOC_NATIVE_MESSAGE_PARAMETER_GET_REQUEST = 0x0041,
    FOC_NATIVE_MESSAGE_PARAMETER_GET_RESPONSE = 0x0042,
    FOC_NATIVE_MESSAGE_PARAMETER_SET_REQUEST = 0x0043,
    FOC_NATIVE_MESSAGE_PARAMETER_SET_RESPONSE = 0x0044,
    FOC_NATIVE_MESSAGE_PARAMETER_COMMIT_REQUEST = 0x0045,
    FOC_NATIVE_MESSAGE_PARAMETER_COMMIT_RESPONSE = 0x0046,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t flags;
    uint32_t message_type;
    uint32_t source_node;
    uint32_t destination_node;
    uint32_t axis_id;
    uint32_t sequence;
    uint32_t payload_length;
    uint8_t payload[FOC_NATIVE_MAX_PAYLOAD_SIZE];
} foc_native_message_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t buffered_length;
    uint32_t expected_length;
    uint32_t frames_decoded;
    uint32_t dropped_bytes;
    uint32_t framing_errors;
    uint32_t crc_errors;
    foc_native_status_t last_error;
    uint8_t buffer[FOC_NATIVE_MAX_FRAME_SIZE];
} foc_native_stream_decoder_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t product_contract_version;
    uint32_t firmware_abi_version;
    uint32_t config_abi_version;
    uint32_t board_id;
    uint32_t motor_id;
    uint32_t inverter_id;
    uint32_t profile_revision;
    uint32_t firmware_revision;
    uint32_t build_id_crc32c;
    uint32_t axis_count;
    uint32_t node_id;
} foc_native_identity_payload_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t axis_count;
    uint32_t axis_request_mask;
    uint32_t control_mode_mask;
    uint32_t input_mode_mask;
    uint32_t feedback_mode_mask;
    uint32_t command_kind_mask;
    uint32_t compiled_transport_mask;
    uint32_t board_transport_mask;
    uint32_t compiled_protocol_mask;
    uint32_t compiled_input_mask;
    uint32_t board_input_mask;
    uint32_t telemetry_valid_mask;
    uint32_t maximum_payload_size;
    uint32_t minimum_status_period_ms;
    uint32_t reserved;
} foc_native_capabilities_payload_t;

enum
{
    FOC_NATIVE_DEVICE_FLAG_READY = (1UL << 0),
    FOC_NATIVE_DEVICE_FLAG_CONFIG_VALID = (1UL << 1),
    FOC_NATIVE_DEVICE_FLAG_OUTPUTS_ARMED = (1UL << 2),
    FOC_NATIVE_DEVICE_FLAG_TRANSPORT_HEALTHY = (1UL << 3),
    FOC_NATIVE_DEVICE_FLAG_KNOWN_MASK = 0x0FUL,
};

typedef struct
{
    uint32_t schema_version;
    uint32_t device_flags;
    uint32_t uptime_ms;
    uint32_t config_revision;
    uint32_t axis_id;
    uint32_t axis_state;
    uint32_t control_mode;
    uint32_t input_mode;
    uint32_t feedback_mode;
    uint32_t active_source_id;
    uint32_t fault_flags;
    uint32_t telemetry_valid_flags;
    float dc_bus_voltage_v;
    float mechanical_velocity_rad_s;
    float current_q_a;
    float motor_temperature_c;
} foc_native_status_payload_t;

typedef uint32_t foc_native_command_result_t;
enum
{
    FOC_NATIVE_COMMAND_ACCEPTED = 0,
    FOC_NATIVE_COMMAND_MALFORMED = 1,
    FOC_NATIVE_COMMAND_UNSUPPORTED = 2,
    FOC_NATIVE_COMMAND_UNAUTHORIZED = 3,
    FOC_NATIVE_COMMAND_EXPIRED = 4,
    FOC_NATIVE_COMMAND_BUSY = 5,
    FOC_NATIVE_COMMAND_REJECTED = 6,
    FOC_NATIVE_COMMAND_INTERNAL = 7,
    FOC_NATIVE_COMMAND_READ_ONLY = 8,
};

typedef struct
{
    uint32_t schema_version;
    uint32_t request_sequence;
    uint32_t source_id;
    uint32_t axis_id;
    foc_native_command_result_t result;
    uint32_t detail;
    uint32_t accepted_sequence;
    uint32_t reserved;
} foc_native_command_result_payload_t;

enum
{
    FOC_NATIVE_PARAMETER_SCOPE_ACTIVE = 0,
    FOC_NATIVE_PARAMETER_SCOPE_PENDING = 1,
    FOC_NATIVE_PARAMETER_ACTION_BEGIN = 0,
    FOC_NATIVE_PARAMETER_ACTION_VALIDATE = 1,
    FOC_NATIVE_PARAMETER_ACTION_APPLY = 2,
    FOC_NATIVE_PARAMETER_ACTION_COMMIT = 3,
    FOC_NATIVE_PARAMETER_ACTION_ROLLBACK = 4,
    FOC_NATIVE_PARAMETER_GROUP_KNOWN_MASK = 0x7FUL,
};

typedef struct
{
    uint32_t schema_version;
    uint32_t request_id;
    uint32_t scope;
    uint32_t transaction_token;
    uint32_t group_mask;
    uint32_t offset;
    uint32_t length;
    uint32_t reserved;
} foc_native_parameter_get_request_payload_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t request_id;
    foc_native_command_result_t result;
    uint32_t detail;
    uint32_t revision;
    uint32_t total_length;
    uint32_t chunk_offset;
    uint32_t chunk_length;
    uint8_t data[FOC_NATIVE_PARAMETER_CHUNK_DATA_SIZE];
} foc_native_parameter_get_response_payload_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t request_id;
    uint32_t transaction_token;
    uint32_t group;
    uint32_t offset;
    uint32_t total_length;
    uint32_t chunk_length;
    uint32_t flags;
    uint8_t data[FOC_NATIVE_PARAMETER_CHUNK_DATA_SIZE];
} foc_native_parameter_set_request_payload_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t request_id;
    uint32_t transaction_token;
    uint32_t action;
    uint32_t expected_revision;
    uint32_t flags;
    uint32_t reserved0;
    uint32_t reserved1;
} foc_native_parameter_transaction_request_payload_t;

typedef struct
{
    uint32_t schema_version;
    uint32_t request_id;
    foc_native_command_result_t result;
    uint32_t detail;
    uint32_t transaction_token;
    uint32_t changed_groups;
    uint32_t transaction_state;
    uint32_t active_revision;
} foc_native_parameter_transaction_response_payload_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_native_identity_payload_t identity;
    foc_native_capabilities_payload_t capabilities;
    foc_native_status_payload_t status;
    uint32_t responses;
    uint32_t rejected;
    foc_native_status_t last_error;
} foc_native_fake_service_t;

_Static_assert(sizeof(foc_native_message_t) == 292U,
               "Native message ABI layout changed");
_Static_assert(sizeof(foc_native_stream_decoder_t) == 316U,
               "Native stream ABI layout changed");
_Static_assert(offsetof(foc_native_message_t, payload) == 36U,
               "Native message payload offset changed");
_Static_assert(offsetof(foc_native_stream_decoder_t, buffer) == 36U,
               "Native stream buffer offset changed");
_Static_assert(sizeof(foc_native_identity_payload_t) == 48U,
               "Native identity payload layout changed");
_Static_assert(sizeof(foc_native_capabilities_payload_t) == 64U,
               "Native capabilities payload layout changed");
_Static_assert(sizeof(foc_native_status_payload_t) == 64U,
               "Native status payload layout changed");
_Static_assert(sizeof(foc_native_command_result_payload_t) == 32U,
               "Native command result payload layout changed");
_Static_assert(sizeof(foc_native_parameter_get_request_payload_t) == 32U,
               "Native parameter get request layout changed");
_Static_assert(sizeof(foc_native_parameter_get_response_payload_t) == 256U,
               "Native parameter get response layout changed");
_Static_assert(sizeof(foc_native_parameter_set_request_payload_t) == 256U,
               "Native parameter set request layout changed");
_Static_assert(sizeof(foc_native_parameter_transaction_request_payload_t) == 32U,
               "Native parameter transaction request layout changed");
_Static_assert(sizeof(foc_native_parameter_transaction_response_payload_t) == 32U,
               "Native parameter transaction response layout changed");
_Static_assert(sizeof(foc_native_fake_service_t) == 196U,
               "Native fake service layout changed");
_Static_assert(sizeof(foc_product_command_t) ==
                   FOC_NATIVE_PRODUCT_COMMAND_PAYLOAD_SIZE,
               "Product command payload size changed");

uint32_t foc_rust_native_protocol_version(void);
foc_native_status_t foc_rust_native_message_init(
    foc_native_message_t *message);
size_t foc_rust_native_frame_size(uint32_t payload_length);
uint32_t foc_rust_native_crc32c(const uint8_t *data, size_t length);
foc_native_status_t foc_rust_native_encode(
    const foc_native_message_t *message,
    uint8_t *frame,
    size_t frame_capacity,
    size_t *frame_length);
foc_native_status_t foc_rust_native_decode(
    const uint8_t *frame,
    size_t frame_length,
    foc_native_message_t *message);
foc_native_status_t foc_rust_native_stream_init(
    foc_native_stream_decoder_t *decoder);
foc_native_status_t foc_rust_native_stream_push(
    foc_native_stream_decoder_t *decoder,
    uint8_t byte,
    foc_native_message_t *message);
foc_native_status_t foc_rust_native_encode_product_command(
    const foc_product_command_t *command,
    uint8_t *payload,
    size_t payload_capacity,
    size_t *payload_length);
foc_native_status_t foc_rust_native_decode_product_command(
    const uint8_t *payload,
    size_t payload_length,
    foc_product_command_t *command);
foc_native_status_t foc_rust_native_fake_service_init(
    foc_native_fake_service_t *service,
    const foc_native_identity_payload_t *identity,
    const foc_native_capabilities_payload_t *capabilities,
    const foc_native_status_payload_t *status);
foc_native_status_t foc_rust_native_fake_service_set_status(
    foc_native_fake_service_t *service,
    const foc_native_status_payload_t *status);
foc_native_status_t foc_rust_native_fake_service_handle(
    foc_native_fake_service_t *service,
    const foc_native_message_t *request,
    foc_native_message_t *response);
foc_native_status_t foc_rust_native_fake_service_handle_frame(
    foc_native_fake_service_t *service,
    const uint8_t *request_frame,
    size_t request_length,
    uint8_t *response_frame,
    size_t response_capacity,
    size_t *response_length);

#endif /* FOC_NATIVE_BRIDGE_H */
