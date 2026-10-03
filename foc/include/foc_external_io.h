#ifndef FOC_EXTERNAL_IO_H
#define FOC_EXTERNAL_IO_H

/*
 * FluxRT external-control and communication contract V2.
 *
 * This is a device-side, allocation-free management contract.  It describes
 * what an image compiled, what the board can physically provide and what a
 * stopped-state configuration requests.  It does not parse a wire protocol,
 * acquire Axis ownership, arm the power stage or touch MCU registers.
 *
 * ABI rules match the product contract: fixed-width scalars only, explicit
 * size/version fields and fail-closed rejection of unknown bits/enums.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_product_contract.h"

#define FOC_EXTERNAL_IO_CONTRACT_VERSION       (0x00020000UL)
#define FOC_EXTERNAL_IO_CONFIG_VERSION         (2UL)
#define FOC_EXTERNAL_IO_CAPABILITIES_VERSION   (1UL)
#define FOC_EXTERNAL_IO_VALIDATION_VERSION     (1UL)
#define FOC_SIMPLE_INPUT_CONFIG_VERSION         (1UL)
#define FOC_EXTERNAL_IO_INPUT_COUNT            (4UL)
#define FOC_EXTERNAL_IO_LINK_COUNT             (4UL)
#define FOC_EXTERNAL_IO_MAX_COMMAND_SOURCES    (8UL)

typedef uint32_t foc_external_transport_mask_t;
enum
{
    FOC_EXTERNAL_TRANSPORT_UART = (1UL << 0),
    FOC_EXTERNAL_TRANSPORT_USB_CDC = (1UL << 1),
    FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC = (1UL << 2),
    FOC_EXTERNAL_TRANSPORT_CAN_FD = (1UL << 3),
    FOC_EXTERNAL_TRANSPORT_ETHERCAT = (1UL << 4),
    FOC_EXTERNAL_TRANSPORT_KNOWN_MASK =
        FOC_EXTERNAL_TRANSPORT_UART |
        FOC_EXTERNAL_TRANSPORT_USB_CDC |
        FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC |
        FOC_EXTERNAL_TRANSPORT_CAN_FD |
        FOC_EXTERNAL_TRANSPORT_ETHERCAT,
};

typedef uint32_t foc_external_protocol_t;
enum
{
    FOC_EXTERNAL_PROTOCOL_DISABLED = 0,
    FOC_EXTERNAL_PROTOCOL_FLUXRT_NATIVE = 1,
    FOC_EXTERNAL_PROTOCOL_DRONECAN = 2,
    FOC_EXTERNAL_PROTOCOL_VESC_CAN = 3,
    FOC_EXTERNAL_PROTOCOL_CANOPEN_CIA402 = 4,
    FOC_EXTERNAL_PROTOCOL_ETHERCAT_COE_CIA402 = 5,
};

enum
{
    FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE = (1UL << 0),
    FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN = (1UL << 1),
    FOC_EXTERNAL_PROTOCOL_MASK_VESC_CAN = (1UL << 2),
    FOC_EXTERNAL_PROTOCOL_MASK_CANOPEN_CIA402 = (1UL << 3),
    FOC_EXTERNAL_PROTOCOL_MASK_ETHERCAT_COE_CIA402 = (1UL << 4),
    FOC_EXTERNAL_PROTOCOL_KNOWN_MASK =
        FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE |
        FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN |
        FOC_EXTERNAL_PROTOCOL_MASK_VESC_CAN |
        FOC_EXTERNAL_PROTOCOL_MASK_CANOPEN_CIA402 |
        FOC_EXTERNAL_PROTOCOL_MASK_ETHERCAT_COE_CIA402,
};

typedef uint32_t foc_external_input_mask_t;
enum
{
    FOC_EXTERNAL_INPUT_PWM_PULSE = (1UL << 0),
    FOC_EXTERNAL_INPUT_DSHOT = (1UL << 1),
    FOC_EXTERNAL_INPUT_ANALOG = (1UL << 2),
    FOC_EXTERNAL_INPUT_STEP_DIR = (1UL << 3),
    FOC_EXTERNAL_INPUT_KNOWN_MASK =
        FOC_EXTERNAL_INPUT_PWM_PULSE |
        FOC_EXTERNAL_INPUT_DSHOT |
        FOC_EXTERNAL_INPUT_ANALOG |
        FOC_EXTERNAL_INPUT_STEP_DIR,
};

typedef uint32_t foc_external_failure_action_t;
enum
{
    FOC_EXTERNAL_FAILURE_RELEASE = 0,
    FOC_EXTERNAL_FAILURE_CONTROLLED_STOP = 1,
    FOC_EXTERNAL_FAILURE_HOLD = 2,
};

/* Resource bits are board-owned opaque identifiers.  The common layer only
 * compares masks; platform code documents the meaning of each bit. */
typedef uint32_t foc_external_resource_mask_t;

typedef struct
{
    uint32_t source_id;
    uint32_t priority;
    uint32_t permissions;
    uint32_t lease_ms;
    uint32_t command_timeout_ms;
} foc_external_source_policy_t;

typedef struct
{
    foc_external_protocol_t protocol;
    uint32_t node_id;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint32_t heartbeat_ms;
    foc_external_source_policy_t source;
} foc_external_link_config_t;

typedef struct
{
    foc_control_mode_t control_mode;
    foc_external_failure_action_t failure_action;
    foc_external_source_policy_t source;
} foc_external_input_config_t;

typedef struct
{
    int32_t raw_min;
    int32_t raw_neutral;
    int32_t raw_max;
    uint32_t deadband;
    float negative_limit_si;
    float positive_limit_si;
} foc_centered_input_calibration_config_t;

typedef struct
{
    uint32_t full_steps_per_revolution;
    uint32_t microsteps;
    uint32_t gear_numerator;
    uint32_t gear_denominator;
    int32_t direction;
    int32_t zero_count;
    float zero_position_rad;
    uint32_t maximum_step_rate_hz;
} foc_step_dir_calibration_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_centered_input_calibration_config_t pwm;
    foc_centered_input_calibration_config_t analog;
    foc_step_dir_calibration_config_t step_dir;
} foc_simple_input_config_t;

/* Links are fixed in the order UART, USB CDC, CAN, EtherCAT.  Inputs are fixed
 * in the order PWM pulse, DShot, Analog, Step/Dir. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t revision;
    foc_external_transport_mask_t transport_enable_mask;
    foc_external_input_mask_t input_enable_mask;
    uint32_t reserved;
    foc_external_link_config_t links[FOC_EXTERNAL_IO_LINK_COUNT];
    foc_external_input_config_t inputs[FOC_EXTERNAL_IO_INPUT_COUNT];
    foc_simple_input_config_t simple_inputs;
} foc_external_io_config_t;

/* Resource arrays use transport bit order UART/USB/CAN classic/CAN FD/EtherCAT
 * and input bit order PWM/DShot/Analog/StepDir. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_transport_mask_t compiled_transport_mask;
    uint32_t compiled_protocol_mask;
    foc_external_input_mask_t compiled_input_mask;
    foc_external_transport_mask_t board_transport_mask;
    foc_external_input_mask_t board_input_mask;
    foc_external_resource_mask_t reserved_resource_mask;
    foc_external_resource_mask_t transport_resource_masks[5];
    foc_external_resource_mask_t input_resource_masks[FOC_EXTERNAL_IO_INPUT_COUNT];
} foc_external_io_capabilities_t;

typedef uint32_t foc_external_io_status_t;
enum
{
    FOC_EXTERNAL_IO_STATUS_OK = 0,
    FOC_EXTERNAL_IO_STATUS_INVALID_ARGUMENT = 1,
    FOC_EXTERNAL_IO_STATUS_INVALID_LAYOUT = 2,
    FOC_EXTERNAL_IO_STATUS_UNKNOWN_CAPABILITY = 3,
    FOC_EXTERNAL_IO_STATUS_NOT_COMPILED = 4,
    FOC_EXTERNAL_IO_STATUS_BOARD_UNAVAILABLE = 5,
    FOC_EXTERNAL_IO_STATUS_INVALID_PROTOCOL = 6,
    FOC_EXTERNAL_IO_STATUS_INVALID_SOURCE = 7,
    FOC_EXTERNAL_IO_STATUS_DUPLICATE_SOURCE = 8,
    FOC_EXTERNAL_IO_STATUS_INVALID_TIMING = 9,
    FOC_EXTERNAL_IO_STATUS_INVALID_PERMISSION = 10,
    FOC_EXTERNAL_IO_STATUS_INVALID_MAPPING = 11,
    FOC_EXTERNAL_IO_STATUS_RESOURCE_CONFLICT = 12,
    FOC_EXTERNAL_IO_STATUS_INVALID_CALIBRATION = 13,
};

typedef uint32_t foc_external_io_field_t;
enum
{
    FOC_EXTERNAL_IO_FIELD_NONE = 0,
    FOC_EXTERNAL_IO_FIELD_CAPABILITIES = 1,
    FOC_EXTERNAL_IO_FIELD_TRANSPORT_MASK = 2,
    FOC_EXTERNAL_IO_FIELD_INPUT_MASK = 3,
    FOC_EXTERNAL_IO_FIELD_UART = 4,
    FOC_EXTERNAL_IO_FIELD_USB = 5,
    FOC_EXTERNAL_IO_FIELD_CAN = 6,
    FOC_EXTERNAL_IO_FIELD_ETHERCAT = 7,
    FOC_EXTERNAL_IO_FIELD_PWM = 8,
    FOC_EXTERNAL_IO_FIELD_DSHOT = 9,
    FOC_EXTERNAL_IO_FIELD_ANALOG = 10,
    FOC_EXTERNAL_IO_FIELD_STEP_DIR = 11,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_io_status_t status;
    foc_external_io_field_t field;
    uint32_t detail;
    foc_external_resource_mask_t conflict_resource_mask;
    foc_external_transport_mask_t configured_transport_mask;
    foc_external_input_mask_t configured_input_mask;
    uint32_t configured_source_count;
} foc_external_io_validation_t;

void foc_external_io_default_config(foc_external_io_config_t *config);
void foc_external_io_empty_capabilities(
    foc_external_io_capabilities_t *capabilities);

/* Build-derived masks remain zero until a real driver/adapter feature is
 * compiled.  Board code must still intersect them with physical capability. */
void foc_external_io_build_capabilities(
    foc_external_io_capabilities_t *capabilities);

foc_external_io_status_t foc_external_io_validate_config(
    const foc_external_io_capabilities_t *capabilities,
    const foc_external_io_config_t *config,
    foc_external_io_validation_t *validation);

uint32_t foc_external_io_config_is_default_off(
    const foc_external_io_config_t *config);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_external_source_policy_t) == 20U,
               "external source policy layout drifted");
_Static_assert(sizeof(foc_external_link_config_t) == 40U,
               "external link config layout drifted");
_Static_assert(sizeof(foc_external_input_config_t) == 28U,
               "external input config layout drifted");
_Static_assert(sizeof(foc_centered_input_calibration_config_t) == 24U,
               "centered input calibration layout drifted");
_Static_assert(sizeof(foc_step_dir_calibration_config_t) == 32U,
               "step/dir calibration layout drifted");
_Static_assert(sizeof(foc_simple_input_config_t) == 88U,
               "simple input config layout drifted");
_Static_assert(sizeof(foc_external_io_config_t) == 384U,
               "external IO config layout drifted");
_Static_assert(sizeof(foc_external_io_capabilities_t) == 68U,
               "external IO capabilities layout drifted");
_Static_assert(sizeof(foc_external_io_validation_t) == 36U,
               "external IO validation layout drifted");
#endif

#endif /* FOC_EXTERNAL_IO_H */
