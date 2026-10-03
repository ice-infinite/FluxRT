#ifndef FOC_COMMAND_RUST_ADAPTER_H
#define FOC_COMMAND_RUST_ADAPTER_H

/* C ownership adapter between foc_command_service and the Rust CommandArbiter. */

#include "foc_command_arbiter_bridge.h"
#include "foc_command_service.h"

#define FOC_COMMAND_RUST_ADAPTER_VERSION (1UL)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t reserved;
    foc_command_context_storage_t storage;
} foc_command_rust_adapter_t;

foc_command_arbiter_status_t foc_command_rust_adapter_init(
    foc_command_rust_adapter_t *adapter);

foc_command_service_result_t foc_command_rust_adapter_make_ops(
    foc_command_rust_adapter_t *adapter,
    foc_command_service_ops_t *ops);

foc_command_arbiter_status_t foc_command_rust_adapter_poll(
    foc_command_rust_adapter_t *adapter,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output);

foc_command_arbiter_status_t foc_command_rust_adapter_get_status(
    foc_command_rust_adapter_t *adapter,
    foc_command_arbiter_snapshot_t *output);

#endif /* FOC_COMMAND_RUST_ADAPTER_H */
