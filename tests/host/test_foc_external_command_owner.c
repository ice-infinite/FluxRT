#include "foc_external_command_owner.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static foc_command_arbiter_status_t g_poll_result = FOC_COMMAND_ARBITER_OK;
static foc_command_arbiter_status_t g_status_result = FOC_COMMAND_ARBITER_OK;
static foc_command_decision_kind_t g_decision = FOC_COMMAND_DECISION_SAFE;
static uint32_t g_polls;

uint32_t foc_rust_command_abi_version(void)
{
    return FOC_COMMAND_ABI_VERSION;
}

uint32_t foc_rust_command_context_required_size(void)
{
    return 64U;
}

uint32_t foc_rust_command_context_required_align(void)
{
    return 8U;
}

foc_command_arbiter_status_t foc_rust_command_init(
    foc_command_context_storage_t *storage)
{
    return (storage != 0) ? FOC_COMMAND_ARBITER_OK :
                            FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
}

foc_command_arbiter_status_t foc_rust_command_configure(
    foc_command_context_storage_t *storage,
    const foc_command_source_policy_abi_t *sources,
    uint32_t source_count)
{
    (void)storage;
    (void)sources;
    (void)source_count;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_submit(
    foc_command_context_storage_t *storage,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    (void)storage;
    (void)command;
    (void)now_ms;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_poll(
    foc_command_context_storage_t *storage,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output)
{
    (void)storage;
    (void)now_ms;
    (void)fault_active;
    ++g_polls;
    if ((g_poll_result == FOC_COMMAND_ARBITER_OK) && (output != 0))
    {
        (void)memset(output, 0, sizeof(*output));
        output->struct_size = sizeof(*output);
        output->abi_version = FOC_COMMAND_ABI_VERSION;
        output->decision = g_decision;
        output->source_id = (g_decision == FOC_COMMAND_DECISION_SELECTED) ? 7U : 0U;
    }
    return g_poll_result;
}

foc_command_arbiter_status_t foc_rust_command_get_status(
    foc_command_context_storage_t *storage,
    foc_command_arbiter_snapshot_t *output)
{
    (void)storage;
    if ((g_status_result == FOC_COMMAND_ARBITER_OK) && (output != 0))
    {
        (void)memset(output, 0, sizeof(*output));
        output->struct_size = sizeof(*output);
        output->abi_version = FOC_COMMAND_ABI_VERSION;
        output->polls = g_polls;
    }
    return g_status_result;
}

int main(void)
{
    foc_command_rust_adapter_t adapter;
    foc_external_command_owner_t owner;
    foc_external_command_event_t event;

    assert(foc_command_rust_adapter_init(&adapter) == FOC_COMMAND_ARBITER_OK);
    assert(foc_external_command_owner_init(&owner) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(foc_external_command_owner_tick(&owner, &adapter, 10U, 0U) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(foc_external_command_owner_get_event(&owner, &event) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(event.sequence == 1U);
    assert(event.timestamp_ms == 10U);
    assert(event.kind == FOC_EXTERNAL_COMMAND_EVENT_SAFE);
    assert(event.arbiter.polls == 1U);

    g_decision = FOC_COMMAND_DECISION_SELECTED;
    assert(foc_external_command_owner_tick(&owner, &adapter, 20U, 0U) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(foc_external_command_owner_get_event(&owner, &event) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(event.sequence == 2U);
    assert(event.kind == FOC_EXTERNAL_COMMAND_EVENT_SELECTED);
    assert(event.decision.source_id == 7U);

    assert(foc_external_command_owner_tick(&owner, &adapter, 30U, 2U) ==
           FOC_EXTERNAL_COMMAND_OWNER_INVALID_ARGUMENT);
    assert(g_polls == 2U);

    g_poll_result = FOC_COMMAND_ARBITER_INVALID_TIME;
    assert(foc_external_command_owner_tick(&owner, &adapter, 40U, 0U) ==
           FOC_EXTERNAL_COMMAND_OWNER_POLL_FAILED);
    assert(foc_external_command_owner_get_event(&owner, &event) ==
           FOC_EXTERNAL_COMMAND_OWNER_OK);
    assert(event.sequence == 3U);
    assert(event.kind == FOC_EXTERNAL_COMMAND_EVENT_POLL_FAILED);
    assert(event.poll_status == FOC_COMMAND_ARBITER_INVALID_TIME);

    puts("foc external command owner tests passed");
    return 0;
}
