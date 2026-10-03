#include "foc_external_command_owner.h"

#include <string.h>

static uint32_t foc_external_command_owner_is_valid(
    const foc_external_command_owner_t *owner)
{
    return ((owner != 0) &&
            (owner->struct_size == sizeof(*owner)) &&
            (owner->version == FOC_EXTERNAL_COMMAND_OWNER_VERSION) &&
            (owner->initialized != 0U)) ? 1U : 0U;
}

static uint32_t foc_external_command_next_sequence(uint32_t current)
{
    const uint32_t next = current + 1U;
    return (next == 0U) ? 1U : next;
}

foc_external_command_owner_result_t foc_external_command_owner_init(
    foc_external_command_owner_t *owner)
{
    if (owner == 0)
    {
        return FOC_EXTERNAL_COMMAND_OWNER_INVALID_ARGUMENT;
    }
    (void)memset(owner, 0, sizeof(*owner));
    owner->struct_size = sizeof(*owner);
    owner->version = FOC_EXTERNAL_COMMAND_OWNER_VERSION;
    owner->initialized = 1U;
    owner->last_event.struct_size = sizeof(owner->last_event);
    owner->last_event.version = FOC_EXTERNAL_COMMAND_EVENT_VERSION;
    return FOC_EXTERNAL_COMMAND_OWNER_OK;
}

foc_external_command_owner_result_t foc_external_command_owner_tick(
    foc_external_command_owner_t *owner,
    foc_command_rust_adapter_t *adapter,
    uint32_t now_ms,
    uint32_t fault_active)
{
    foc_external_command_event_t event;
    foc_command_arbiter_status_t result;

    if ((foc_external_command_owner_is_valid(owner) == 0U) ||
        (adapter == 0) || (fault_active > 1U))
    {
        return (owner == 0 || owner->initialized == 0U) ?
            FOC_EXTERNAL_COMMAND_OWNER_NOT_INITIALIZED :
            FOC_EXTERNAL_COMMAND_OWNER_INVALID_ARGUMENT;
    }

    (void)memset(&event, 0, sizeof(event));
    event.struct_size = sizeof(event);
    event.version = FOC_EXTERNAL_COMMAND_EVENT_VERSION;
    owner->next_sequence =
        foc_external_command_next_sequence(owner->next_sequence);
    event.sequence = owner->next_sequence;
    event.timestamp_ms = now_ms;

    result = foc_command_rust_adapter_poll(
        adapter, now_ms, fault_active, &event.decision);
    event.poll_status = result;
    if (result != FOC_COMMAND_ARBITER_OK)
    {
        event.kind = FOC_EXTERNAL_COMMAND_EVENT_POLL_FAILED;
        owner->last_event = event;
        return FOC_EXTERNAL_COMMAND_OWNER_POLL_FAILED;
    }

    result = foc_command_rust_adapter_get_status(adapter, &event.arbiter);
    if (result != FOC_COMMAND_ARBITER_OK)
    {
        event.poll_status = result;
        event.kind = FOC_EXTERNAL_COMMAND_EVENT_POLL_FAILED;
        owner->last_event = event;
        return FOC_EXTERNAL_COMMAND_OWNER_STATUS_FAILED;
    }

    event.kind = (event.decision.decision == FOC_COMMAND_DECISION_SELECTED) ?
        FOC_EXTERNAL_COMMAND_EVENT_SELECTED :
        FOC_EXTERNAL_COMMAND_EVENT_SAFE;
    owner->last_event = event;
    return FOC_EXTERNAL_COMMAND_OWNER_OK;
}

foc_external_command_owner_result_t foc_external_command_owner_get_event(
    const foc_external_command_owner_t *owner,
    foc_external_command_event_t *output)
{
    if ((foc_external_command_owner_is_valid(owner) == 0U) || (output == 0))
    {
        return (owner == 0 || owner->initialized == 0U) ?
            FOC_EXTERNAL_COMMAND_OWNER_NOT_INITIALIZED :
            FOC_EXTERNAL_COMMAND_OWNER_INVALID_ARGUMENT;
    }
    *output = owner->last_event;
    return FOC_EXTERNAL_COMMAND_OWNER_OK;
}
