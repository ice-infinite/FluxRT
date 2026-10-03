#include "foc_input_service.h"

#include <math.h>
#include <string.h>

static const uint32_t g_foc_input_service_bits[FOC_EXTERNAL_IO_INPUT_COUNT] = {
    FOC_EXTERNAL_INPUT_PWM_PULSE,
    FOC_EXTERNAL_INPUT_DSHOT,
    FOC_EXTERNAL_INPUT_ANALOG,
    FOC_EXTERNAL_INPUT_STEP_DIR,
};

static uint32_t foc_input_service_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_input_service_is_valid(const foc_input_service_t *service)
{
    return ((service != 0) &&
            (service->struct_size == sizeof(*service)) &&
            (service->version == FOC_INPUT_SERVICE_VERSION) &&
            (service->initialized != 0U)) ? 1U : 0U;
}

static int32_t foc_input_service_index(uint32_t input)
{
    uint32_t index;
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        if (g_foc_input_service_bits[index] == input)
        {
            return (int32_t)index;
        }
    }
    return -1;
}

static uint32_t foc_input_service_sequence_is_forward(
    uint32_t previous,
    uint32_t next)
{
    uint32_t delta = (uint32_t)(next - previous);
    return ((delta != 0U) && (delta < 0x80000000UL)) ? 1U : 0U;
}

foc_input_service_result_t foc_input_service_init(
    foc_input_service_t *service,
    const foc_external_io_capabilities_t *capabilities)
{
    if ((service == 0) || (capabilities == 0) ||
        (capabilities->struct_size != sizeof(*capabilities)) ||
        (capabilities->version != FOC_EXTERNAL_IO_CAPABILITIES_VERSION))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(service, 0, sizeof(*service));
    service->struct_size = sizeof(*service);
    service->version = FOC_INPUT_SERVICE_VERSION;
    service->capabilities = *capabilities;
    service->initialized = 1U;
    return FOC_INPUT_SERVICE_OK;
}

foc_input_service_result_t foc_input_service_apply_config(
    foc_input_service_t *service,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    foc_external_io_validation_t *validation)
{
    uint32_t index;
    foc_external_io_status_t result;
    if ((foc_input_service_is_valid(service) == 0U) ||
        (config == 0) || (validation == 0))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    if (foc_input_service_guard_is_safe(guard) == 0U)
    {
        return FOC_INPUT_SERVICE_UNSAFE_STATE;
    }
    result = foc_external_io_validate_config(
        &service->capabilities, config, validation);
    if (result != FOC_EXTERNAL_IO_STATUS_OK)
    {
        return FOC_INPUT_SERVICE_INVALID_CONFIG;
    }
    service->configured_input_mask = config->input_enable_mask;
    service->healthy_input_mask = 0U;
    service->expired_input_mask = 0U;
    (void)memset(service->slots, 0, sizeof(service->slots));
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        if ((config->input_enable_mask & g_foc_input_service_bits[index]) != 0U)
        {
            service->slots[index].source_id = config->inputs[index].source.source_id;
            service->slots[index].timeout_ms =
                config->inputs[index].source.command_timeout_ms;
        }
    }
    return FOC_INPUT_SERVICE_OK;
}

foc_input_service_result_t foc_input_service_publish(
    foc_input_service_t *service,
    const foc_input_sample_t *sample,
    uint32_t now_ms)
{
    int32_t index;
    foc_input_service_slot_t *slot;
    if ((foc_input_service_is_valid(service) == 0U) || (sample == 0) ||
        (sample->struct_size != sizeof(*sample)) ||
        (sample->version != FOC_INPUT_SAMPLE_VERSION) ||
        (sample->instance_id != 0U) ||
        (sample->valid_flags == 0U) ||
        ((sample->valid_flags & ~((uint32_t)FOC_INPUT_SAMPLE_VALID_KNOWN_MASK)) != 0U) ||
        ((sample->quality_flags & ~((uint32_t)FOC_INPUT_SAMPLE_QUALITY_KNOWN_MASK)) != 0U) ||
        (((sample->valid_flags & FOC_INPUT_SAMPLE_VALID_NORMALIZED) != 0U) &&
         !isfinite(sample->normalized_value)))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    index = foc_input_service_index(sample->input);
    if (index < 0)
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    if ((service->configured_input_mask & sample->input) == 0U)
    {
        return FOC_INPUT_SERVICE_DISABLED;
    }
    slot = &service->slots[(uint32_t)index];
    if (sample->source_id != slot->source_id)
    {
        return FOC_INPUT_SERVICE_WRONG_SOURCE;
    }
    if ((slot->sequence_valid != 0U) &&
        (foc_input_service_sequence_is_forward(
             slot->last_sequence, sample->sequence) == 0U))
    {
        return FOC_INPUT_SERVICE_STALE_SEQUENCE;
    }
    if ((service->expired_input_mask & sample->input) != 0U)
    {
        ++slot->recovery_count;
    }
    slot->latest = *sample;
    slot->last_update_ms = now_ms;
    slot->last_sequence = sample->sequence;
    slot->sequence_valid = 1U;
    ++slot->sample_count;
    service->healthy_input_mask |= sample->input;
    service->expired_input_mask &= ~sample->input;
    return FOC_INPUT_SERVICE_OK;
}

foc_input_service_result_t foc_input_service_mark_unhealthy(
    foc_input_service_t *service,
    foc_external_input_mask_t input,
    uint32_t quality_flags)
{
    int32_t index;
    foc_input_service_slot_t *slot;
    if ((foc_input_service_is_valid(service) == 0U) ||
        (quality_flags == 0U) ||
        ((quality_flags &
          ~((uint32_t)FOC_INPUT_SAMPLE_QUALITY_KNOWN_MASK)) != 0U))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    index = foc_input_service_index(input);
    if (index < 0)
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    if ((service->configured_input_mask & input) == 0U)
    {
        return FOC_INPUT_SERVICE_DISABLED;
    }
    slot = &service->slots[(uint32_t)index];
    if ((service->expired_input_mask & input) == 0U)
    {
        ++slot->failure_count;
    }
    service->healthy_input_mask &= ~input;
    service->expired_input_mask |= input;
    slot->latest.quality_flags |= quality_flags;
    return FOC_INPUT_SERVICE_OK;
}

foc_input_service_result_t foc_input_service_poll(
    foc_input_service_t *service,
    uint32_t now_ms,
    foc_external_input_mask_t *expired_mask_out)
{
    uint32_t index;
    uint32_t newly_expired = 0U;
    if ((foc_input_service_is_valid(service) == 0U) ||
        (expired_mask_out == 0))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        uint32_t bit = g_foc_input_service_bits[index];
        foc_input_service_slot_t *slot = &service->slots[index];
        if (((service->configured_input_mask & bit) == 0U) ||
            (slot->sequence_valid == 0U) ||
            ((service->healthy_input_mask & bit) == 0U))
        {
            continue;
        }
        if ((uint32_t)(now_ms - slot->last_update_ms) >= slot->timeout_ms)
        {
            service->healthy_input_mask &= ~bit;
            service->expired_input_mask |= bit;
            newly_expired |= bit;
            ++slot->timeout_count;
            slot->latest.quality_flags |= FOC_INPUT_SAMPLE_QUALITY_STALE;
        }
    }
    *expired_mask_out = newly_expired;
    return FOC_INPUT_SERVICE_OK;
}

foc_input_service_result_t foc_input_service_get_status(
    const foc_input_service_t *service,
    foc_input_service_status_t *status)
{
    if ((foc_input_service_is_valid(service) == 0U) || (status == 0))
    {
        return FOC_INPUT_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_INPUT_SERVICE_VERSION;
    status->configured_input_mask = service->configured_input_mask;
    status->healthy_input_mask = service->healthy_input_mask;
    status->expired_input_mask = service->expired_input_mask;
    (void)memcpy(status->slots, service->slots, sizeof(status->slots));
    return FOC_INPUT_SERVICE_OK;
}
