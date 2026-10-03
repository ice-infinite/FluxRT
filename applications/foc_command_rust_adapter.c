#include "foc_command_rust_adapter.h"

#include <string.h>

static uint32_t foc_command_rust_adapter_is_valid(
    const foc_command_rust_adapter_t *adapter)
{
    return ((adapter != 0) &&
            (adapter->struct_size == sizeof(*adapter)) &&
            (adapter->version == FOC_COMMAND_RUST_ADAPTER_VERSION) &&
            (adapter->initialized != 0U)) ? 1U : 0U;
}

static uint32_t foc_command_rust_adapter_configure(
    void *context,
    const foc_command_service_source_t *sources,
    uint32_t source_count)
{
    foc_command_rust_adapter_t *adapter =
        (foc_command_rust_adapter_t *)context;
    if (foc_command_rust_adapter_is_valid(adapter) == 0U)
    {
        return FOC_COMMAND_ARBITER_NOT_INITIALIZED;
    }
    return foc_rust_command_configure(
        &adapter->storage,
        (const foc_command_source_policy_abi_t *)sources,
        source_count);
}

static uint32_t foc_command_rust_adapter_submit(
    void *context,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    foc_command_rust_adapter_t *adapter =
        (foc_command_rust_adapter_t *)context;
    if (foc_command_rust_adapter_is_valid(adapter) == 0U)
    {
        return FOC_COMMAND_ARBITER_NOT_INITIALIZED;
    }
    return foc_rust_command_submit(&adapter->storage, command, now_ms);
}

foc_command_arbiter_status_t foc_command_rust_adapter_init(
    foc_command_rust_adapter_t *adapter)
{
    foc_command_arbiter_status_t result;
    if ((adapter == 0) ||
        (foc_rust_command_abi_version() != FOC_COMMAND_ABI_VERSION) ||
        (foc_rust_command_context_required_size() >
            FOC_COMMAND_CONTEXT_CAPACITY) ||
        (foc_rust_command_context_required_align() > sizeof(uint64_t)))
    {
        return FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
    }
    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->struct_size = sizeof(*adapter);
    adapter->version = FOC_COMMAND_RUST_ADAPTER_VERSION;
    result = foc_rust_command_init(&adapter->storage);
    if (result != FOC_COMMAND_ARBITER_OK)
    {
        return result;
    }
    adapter->initialized = 1U;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_service_result_t foc_command_rust_adapter_make_ops(
    foc_command_rust_adapter_t *adapter,
    foc_command_service_ops_t *ops)
{
    if ((foc_command_rust_adapter_is_valid(adapter) == 0U) || (ops == 0))
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(ops, 0, sizeof(*ops));
    ops->struct_size = sizeof(*ops);
    ops->version = FOC_COMMAND_SERVICE_VERSION;
    ops->configure = foc_command_rust_adapter_configure;
    ops->submit = foc_command_rust_adapter_submit;
    ops->context = adapter;
    return FOC_COMMAND_SERVICE_OK;
}

foc_command_arbiter_status_t foc_command_rust_adapter_poll(
    foc_command_rust_adapter_t *adapter,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output)
{
    if (foc_command_rust_adapter_is_valid(adapter) == 0U)
    {
        return FOC_COMMAND_ARBITER_NOT_INITIALIZED;
    }
    return foc_rust_command_poll(
        &adapter->storage, now_ms, fault_active, output);
}

foc_command_arbiter_status_t foc_command_rust_adapter_get_status(
    foc_command_rust_adapter_t *adapter,
    foc_command_arbiter_snapshot_t *output)
{
    if (foc_command_rust_adapter_is_valid(adapter) == 0U)
    {
        return FOC_COMMAND_ARBITER_NOT_INITIALIZED;
    }
    return foc_rust_command_get_status(&adapter->storage, output);
}

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_command_service_source_t) ==
                   sizeof(foc_command_source_policy_abi_t),
               "command service/Rust policy layout drifted");
_Static_assert(offsetof(foc_command_service_source_t, source_id) ==
                   offsetof(foc_command_source_policy_abi_t, source_id),
               "command service/Rust source-id offset drifted");
_Static_assert(offsetof(foc_command_service_source_t, priority) ==
                   offsetof(foc_command_source_policy_abi_t, priority),
               "command service/Rust priority offset drifted");
_Static_assert(offsetof(foc_command_service_source_t, permissions) ==
                   offsetof(foc_command_source_policy_abi_t, permissions),
               "command service/Rust permissions offset drifted");
_Static_assert(offsetof(foc_command_service_source_t, lease_ms) ==
                   offsetof(foc_command_source_policy_abi_t, lease_ms),
               "command service/Rust lease offset drifted");
_Static_assert(offsetof(foc_command_service_source_t, command_timeout_ms) ==
                   offsetof(foc_command_source_policy_abi_t, command_timeout_ms),
               "command service/Rust policy offsets drifted");
#endif
