#ifndef FOC_EXTERNAL_INPUT_OWNER_H
#define FOC_EXTERNAL_INPUT_OWNER_H

/*
 * Management-task owner for raw external-input capture.
 *
 * This layer is the only caller allowed to start/stop a raw input port. It
 * converts target-neutral raw samples into normalized simple-input candidates,
 * submits them through ExternalIoManagement and releases the command source on
 * timeout or invalid input. It never runs in the motor ISR and cannot arm an
 * Axis or write PWM registers.
 */

#include "foc_external_input_port.h"
#include "foc_external_io_management.h"

#define FOC_EXTERNAL_INPUT_OWNER_VERSION        (1UL)
#define FOC_EXTERNAL_INPUT_OWNER_STATUS_VERSION (1UL)

typedef uint32_t foc_external_input_owner_result_t;
enum
{
    FOC_EXTERNAL_INPUT_OWNER_OK = 0,
    FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT = 1,
    FOC_EXTERNAL_INPUT_OWNER_UNSAFE_STATE = 2,
    FOC_EXTERNAL_INPUT_OWNER_CONFIG_REJECTED = 3,
    FOC_EXTERNAL_INPUT_OWNER_PORT_FAILED = 4,
    FOC_EXTERNAL_INPUT_OWNER_NORMALIZATION_FAILED = 5,
    FOC_EXTERNAL_INPUT_OWNER_COMMAND_FAILED = 6,
    FOC_EXTERNAL_INPUT_OWNER_POLL_FAILED = 7,
};

typedef uint32_t foc_external_input_failure_reason_t;
enum
{
    FOC_EXTERNAL_INPUT_FAILURE_NONE = 0,
    FOC_EXTERNAL_INPUT_FAILURE_TIMEOUT = 1,
    FOC_EXTERNAL_INPUT_FAILURE_INVALID_RAW = 2,
    FOC_EXTERNAL_INPUT_FAILURE_OUT_OF_RANGE = 3,
    FOC_EXTERNAL_INPUT_FAILURE_PORT = 4,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    foc_external_input_mask_t managed_input_mask;
    foc_external_input_mask_t enabled_input_mask;
    foc_external_input_mask_t healthy_input_mask;
    foc_external_input_mask_t fault_input_mask;
    foc_external_input_mask_t degraded_input_mask;
    foc_external_input_port_result_t last_port_result;
    foc_input_normalization_status_t last_normalization_status;
    foc_external_input_failure_reason_t last_failure_reason;
    foc_external_failure_action_t requested_failure_action;
    foc_external_failure_action_t effective_failure_action;
    uint32_t raw_sample_count;
    uint32_t setpoint_count;
    uint32_t failure_count;
    uint32_t recovery_count;
    uint32_t release_count;
} foc_external_input_owner_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    foc_external_io_management_t *management;
    foc_external_input_port_t *port;
    foc_external_input_owner_status_t status;
    uint32_t activation_ms[FOC_EXTERNAL_IO_INPUT_COUNT];
    uint32_t last_success_ms[FOC_EXTERNAL_IO_INPUT_COUNT];
    uint32_t last_raw_sequence[FOC_EXTERNAL_IO_INPUT_COUNT];
    uint32_t raw_sequence_valid[FOC_EXTERNAL_IO_INPUT_COUNT];
    uint32_t command_sequence[FOC_EXTERNAL_IO_INPUT_COUNT];
} foc_external_input_owner_t;

foc_external_input_owner_result_t foc_external_input_owner_init(
    foc_external_input_owner_t *owner,
    foc_external_io_management_t *management,
    foc_external_input_port_t *port);

/* Configuration is accepted only under the same Disabled/no-drive/no-fault
 * guard as the common configuration transaction. Port start/stop and common
 * service reconfiguration are rolled back together on failure. */
foc_external_input_owner_result_t foc_external_input_owner_apply_config(
    foc_external_input_owner_t *owner,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    uint32_t now_ms);

/* Called by one management task. It is bounded by one latest sample per managed
 * input and never waits for hardware. */
foc_external_input_owner_result_t foc_external_input_owner_poll(
    foc_external_input_owner_t *owner,
    uint32_t now_ms,
    uint32_t fault_active);

foc_external_input_owner_result_t foc_external_input_owner_get_status(
    const foc_external_input_owner_t *owner,
    foc_external_input_owner_status_t *status);

#endif /* FOC_EXTERNAL_INPUT_OWNER_H */
