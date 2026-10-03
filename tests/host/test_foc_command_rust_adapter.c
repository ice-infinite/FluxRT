#include "foc_command_rust_adapter.h"

#include <assert.h>
#include <string.h>

static uint32_t g_configure_calls;
static uint32_t g_submit_calls;
static uint32_t g_last_source_count;
static uint32_t g_next_result;

uint32_t foc_rust_command_abi_version(void)
{
    return FOC_COMMAND_ABI_VERSION;
}

uint32_t foc_rust_command_context_required_size(void)
{
    return 1024U;
}

uint32_t foc_rust_command_context_required_align(void)
{
    return 8U;
}

foc_command_arbiter_status_t foc_rust_command_init(
    foc_command_context_storage_t *storage)
{
    return (storage != 0) ?
        FOC_COMMAND_ARBITER_OK : FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
}

foc_command_arbiter_status_t foc_rust_command_configure(
    foc_command_context_storage_t *storage,
    const foc_command_source_policy_abi_t *sources,
    uint32_t source_count)
{
    (void)storage;
    ++g_configure_calls;
    g_last_source_count = source_count;
    if (source_count != 0U)
    {
        assert(sources != 0);
        assert(sources[0].source_id == 41U);
    }
    return g_next_result;
}

foc_command_arbiter_status_t foc_rust_command_submit(
    foc_command_context_storage_t *storage,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    (void)storage;
    ++g_submit_calls;
    assert(command != 0);
    assert(command->source_id == 41U);
    assert(now_ms == 12U);
    return g_next_result;
}

foc_command_arbiter_status_t foc_rust_command_poll(
    foc_command_context_storage_t *storage,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output)
{
    (void)storage;
    assert(now_ms == 13U);
    assert(fault_active == 1U);
    assert(output != 0);
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_COMMAND_ABI_VERSION;
    output->safe_reason = FOC_COMMAND_SAFE_FAULT_ACTIVE;
    return g_next_result;
}

foc_command_arbiter_status_t foc_rust_command_get_status(
    foc_command_context_storage_t *storage,
    foc_command_arbiter_snapshot_t *output)
{
    (void)storage;
    assert(output != 0);
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_COMMAND_ABI_VERSION;
    output->configured_source_count = g_last_source_count;
    return g_next_result;
}

static void reset_fake(void)
{
    g_configure_calls = 0U;
    g_submit_calls = 0U;
    g_last_source_count = 0U;
    g_next_result = FOC_COMMAND_ARBITER_OK;
}

static void test_adapter_exports_service_sink_without_bypassing_arbiter(void)
{
    foc_command_rust_adapter_t adapter;
    foc_command_service_ops_t ops;
    foc_command_service_source_t source = {0};
    foc_product_command_t command = {0};
    foc_command_decision_t decision;
    foc_command_arbiter_snapshot_t status;

    reset_fake();
    assert(foc_command_rust_adapter_init(&adapter) ==
           FOC_COMMAND_ARBITER_OK);
    assert(adapter.initialized == 1U);
    assert(foc_command_rust_adapter_make_ops(&adapter, &ops) ==
           FOC_COMMAND_SERVICE_OK);

    source.source_id = 41U;
    source.priority = 7U;
    source.permissions = 5U;
    source.lease_ms = 20U;
    source.command_timeout_ms = 40U;
    assert(ops.configure(ops.context, &source, 1U) ==
           FOC_COMMAND_ARBITER_OK);
    assert(g_configure_calls == 1U);
    assert(g_last_source_count == 1U);

    command.source_id = 41U;
    assert(ops.submit(ops.context, &command, 12U) ==
           FOC_COMMAND_ARBITER_OK);
    assert(g_submit_calls == 1U);

    assert(foc_command_rust_adapter_poll(
               &adapter, 13U, 1U, &decision) == FOC_COMMAND_ARBITER_OK);
    assert(decision.safe_reason == FOC_COMMAND_SAFE_FAULT_ACTIVE);
    assert(foc_command_rust_adapter_get_status(&adapter, &status) ==
           FOC_COMMAND_ARBITER_OK);
    assert(status.configured_source_count == 1U);
}

static void test_sink_errors_are_preserved_and_uninitialized_is_rejected(void)
{
    foc_command_rust_adapter_t adapter;
    foc_command_service_ops_t ops;
    foc_command_service_source_t source = {0};

    reset_fake();
    (void)memset(&adapter, 0, sizeof(adapter));
    assert(foc_command_rust_adapter_make_ops(&adapter, &ops) ==
           FOC_COMMAND_SERVICE_INVALID_ARGUMENT);

    assert(foc_command_rust_adapter_init(&adapter) ==
           FOC_COMMAND_ARBITER_OK);
    assert(foc_command_rust_adapter_make_ops(&adapter, &ops) ==
           FOC_COMMAND_SERVICE_OK);
    source.source_id = 41U;
    g_next_result = FOC_COMMAND_ARBITER_INVALID_CONFIG;
    assert(ops.configure(ops.context, &source, 1U) ==
           FOC_COMMAND_ARBITER_INVALID_CONFIG);
}

int main(void)
{
    test_adapter_exports_service_sink_without_bypassing_arbiter();
    test_sink_errors_are_preserved_and_uninitialized_is_rejected();
    return 0;
}
