#ifndef FOC_EXTERNAL_IO_MANAGEMENT_H
#define FOC_EXTERNAL_IO_MANAGEMENT_H

/*
 * Default-off composition root for the external-I/O framework.
 *
 * It binds the C command gateway to the Rust arbiter and consumes an injected
 * immutable capability table. No transport/input driver or management thread
 * is started, and this layer never includes a target-platform header.
 */

#include "foc_command_rust_adapter.h"
#include "foc_external_command_owner.h"
#include "foc_input_service.h"
#include "foc_simple_input_adapter.h"

#define FOC_EXTERNAL_IO_MANAGEMENT_VERSION (1UL)

typedef uint32_t foc_external_io_management_result_t;
enum
{
    FOC_EXTERNAL_IO_MANAGEMENT_OK = 0,
    FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT = 1,
    FOC_EXTERNAL_IO_MANAGEMENT_RUST_ABI_FAILED = 2,
    FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_SERVICE_FAILED = 3,
    FOC_EXTERNAL_IO_MANAGEMENT_DEFAULT_CONFIG_FAILED = 4,
    FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_OWNER_FAILED = 5,
    FOC_EXTERNAL_IO_MANAGEMENT_CONFIG_APPLY_FAILED = 6,
    FOC_EXTERNAL_IO_MANAGEMENT_POLL_FAILED = 7,
    FOC_EXTERNAL_IO_MANAGEMENT_INPUT_ABI_FAILED = 8,
    FOC_EXTERNAL_IO_MANAGEMENT_INPUT_SERVICE_FAILED = 9,
    FOC_EXTERNAL_IO_MANAGEMENT_CANDIDATE_REJECTED = 10,
    FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_SUBMIT_FAILED = 11,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    foc_external_io_management_result_t last_result;
    foc_external_io_capabilities_t capabilities;
    foc_external_io_config_t active_config;
    foc_external_io_validation_t validation;
    foc_command_rust_adapter_t command_adapter;
    foc_command_service_t command_service;
    foc_input_service_t input_service;
    foc_external_command_owner_t command_owner;
    foc_external_input_mask_t last_expired_input_mask;
} foc_external_io_management_t;

/* Host/HIL and every target composition root inject an explicit immutable
 * capability table.  The management layer has no default board and therefore
 * cannot silently bind itself to one MCU or PCB. */
foc_external_io_management_result_t
foc_external_io_management_init_with_capabilities(
    foc_external_io_management_t *management,
    const foc_external_io_capabilities_t *capabilities);

foc_external_io_management_result_t foc_external_io_management_apply_config(
    foc_external_io_management_t *management,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard);

foc_external_io_management_result_t foc_external_io_management_poll(
    foc_external_io_management_t *management,
    uint32_t now_ms,
    uint32_t fault_active);

/* Accepts only a simple-input SETPOINT candidate. It publishes the snapshot to
 * InputService, then submits the command to the sole CommandService gateway.
 * It still cannot call MotorService, request an Axis state or arm outputs. */
foc_external_io_management_result_t
foc_external_io_management_submit_simple_input(
    foc_external_io_management_t *management,
    const foc_simple_input_candidate_t *candidate,
    uint32_t now_ms);

foc_external_io_management_result_t
foc_external_io_management_get_input_status(
    const foc_external_io_management_t *management,
    foc_input_service_status_t *output);

foc_external_io_management_result_t foc_external_io_management_get_event(
    const foc_external_io_management_t *management,
    foc_external_command_event_t *output);

#endif /* FOC_EXTERNAL_IO_MANAGEMENT_H */
