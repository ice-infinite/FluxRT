#include "foc_command_service.h"

#include <string.h>

static uint32_t foc_command_service_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_command_service_is_valid(
    const foc_command_service_t *service)
{
    return ((service != 0) &&
            (service->struct_size == sizeof(*service)) &&
            (service->version == FOC_COMMAND_SERVICE_VERSION) &&
            (service->initialized != 0U) &&
            (service->ops.struct_size == sizeof(service->ops)) &&
            (service->ops.version == FOC_COMMAND_SERVICE_VERSION) &&
            (service->ops.configure != 0) &&
            (service->ops.submit != 0)) ? 1U : 0U;
}

static foc_command_service_result_t foc_command_service_record(
    foc_command_service_t *service,
    foc_command_service_result_t result,
    uint32_t source_id,
    uint32_t detail)
{
    if (service != 0)
    {
        service->last_result = result;
        service->last_source_id = source_id;
        service->last_detail = detail;
    }
    return result;
}

static uint32_t foc_command_service_time_is_valid(
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    uint32_t window = (uint32_t)(
        command->valid_until_ms - command->created_at_ms);
    uint32_t age = (uint32_t)(now_ms - command->created_at_ms);
    return ((window != 0U) &&
            (window < 0x80000000UL) &&
            (age < 0x80000000UL) &&
            (age < window)) ? 1U : 0U;
}

static const foc_command_service_source_t *foc_command_service_find_source(
    const foc_command_service_t *service,
    uint32_t source_id)
{
    uint32_t index;
    for (index = 0U; index < service->source_count; ++index)
    {
        if (service->sources[index].source_id == source_id)
        {
            return &service->sources[index];
        }
    }
    return 0;
}

static void foc_command_service_add_source(
    foc_command_service_t *service,
    const foc_external_source_policy_t *policy)
{
    foc_command_service_source_t *target;
    if ((policy->source_id == 0U) ||
        (service->source_count >= FOC_EXTERNAL_IO_MAX_COMMAND_SOURCES))
    {
        return;
    }
    target = &service->sources[service->source_count++];
    target->source_id = policy->source_id;
    target->priority = policy->priority;
    target->permissions = policy->permissions;
    target->lease_ms = policy->lease_ms;
    target->command_timeout_ms = policy->command_timeout_ms;
}

foc_command_service_result_t foc_command_service_init(
    foc_command_service_t *service,
    const foc_external_io_capabilities_t *capabilities,
    const foc_command_service_ops_t *ops)
{
    if ((service == 0) || (capabilities == 0) || (ops == 0) ||
        (capabilities->struct_size != sizeof(*capabilities)) ||
        (capabilities->version != FOC_EXTERNAL_IO_CAPABILITIES_VERSION) ||
        (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_COMMAND_SERVICE_VERSION) ||
        (ops->configure == 0) ||
        (ops->submit == 0))
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(service, 0, sizeof(*service));
    service->struct_size = sizeof(*service);
    service->version = FOC_COMMAND_SERVICE_VERSION;
    service->capabilities = *capabilities;
    service->ops = *ops;
    service->initialized = 1U;
    service->last_result = FOC_COMMAND_SERVICE_DISABLED;
    if (service->ops.configure(service->ops.context, 0, 0U) != 0U)
    {
        service->initialized = 0U;
        return FOC_COMMAND_SERVICE_SINK_REJECTED;
    }
    return FOC_COMMAND_SERVICE_OK;
}

foc_command_service_result_t foc_command_service_apply_config(
    foc_command_service_t *service,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    foc_external_io_validation_t *validation)
{
    uint32_t index;
    foc_external_io_status_t result;
    if ((foc_command_service_is_valid(service) == 0U) ||
        (config == 0) || (validation == 0))
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    if (foc_command_service_guard_is_safe(guard) == 0U)
    {
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_UNSAFE_STATE, 0U, 0U);
    }
    result = foc_external_io_validate_config(
        &service->capabilities, config, validation);
    if (result != FOC_EXTERNAL_IO_STATUS_OK)
    {
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_INVALID_CONFIG, 0U, result);
    }
    service->enabled = 0U;
    service->source_count = 0U;
    (void)memset(service->sources, 0, sizeof(service->sources));
    for (index = 0U; index < FOC_EXTERNAL_IO_LINK_COUNT; ++index)
    {
        uint32_t enabled = 0U;
        if (index == 0U)
        {
            enabled = config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_UART;
        }
        else if (index == 1U)
        {
            enabled = config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_USB_CDC;
        }
        else if (index == 2U)
        {
            enabled = config->transport_enable_mask &
                (FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC | FOC_EXTERNAL_TRANSPORT_CAN_FD);
        }
        else
        {
            enabled = config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_ETHERCAT;
        }
        if (enabled != 0U)
        {
            foc_command_service_add_source(service, &config->links[index].source);
        }
    }
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        if ((config->input_enable_mask & (1UL << index)) != 0U)
        {
            foc_command_service_add_source(service, &config->inputs[index].source);
        }
    }
    result = service->ops.configure(
        service->ops.context, service->sources, service->source_count);
    if (result != 0U)
    {
        service->enabled = 0U;
        service->source_count = 0U;
        (void)memset(service->sources, 0, sizeof(service->sources));
        ++service->rejected;
        ++service->sink_rejected;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_SINK_REJECTED, 0U, result);
    }
    service->enabled = (service->source_count != 0U) ? 1U : 0U;
    return foc_command_service_record(
        service,
        (service->enabled != 0U) ?
            FOC_COMMAND_SERVICE_OK : FOC_COMMAND_SERVICE_DISABLED,
        0U, 0U);
}

foc_command_service_result_t foc_command_service_disable(
    foc_command_service_t *service)
{
    uint32_t sink_result;
    if (foc_command_service_is_valid(service) == 0U)
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    service->enabled = 0U;
    sink_result = service->ops.configure(service->ops.context, 0, 0U);
    service->source_count = 0U;
    (void)memset(service->sources, 0, sizeof(service->sources));
    if (sink_result != 0U)
    {
        ++service->rejected;
        ++service->sink_rejected;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_SINK_REJECTED, 0U, sink_result);
    }
    return foc_command_service_record(
        service, FOC_COMMAND_SERVICE_DISABLED, 0U, 0U);
}

foc_command_service_result_t foc_command_service_submit(
    foc_command_service_t *service,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    const foc_command_service_source_t *source;
    uint32_t sink_result;
    uint32_t permission;
    if ((foc_command_service_is_valid(service) == 0U) ||
        (command == 0) ||
        (command->struct_size != sizeof(*command)) ||
        (command->version != FOC_PRODUCT_COMMAND_VERSION) ||
        (command->command_kind > FOC_PRODUCT_COMMAND_EMERGENCY_STOP) ||
        ((command->flags & ~((uint32_t)FOC_PRODUCT_COMMAND_FLAG_KNOWN_MASK)) != 0U))
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    ++service->submitted;
    if (service->enabled == 0U)
    {
        ++service->rejected;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_DISABLED,
            command->source_id, 0U);
    }
    source = foc_command_service_find_source(service, command->source_id);
    if (source == 0)
    {
        ++service->rejected;
        ++service->unknown_source;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_UNKNOWN_SOURCE,
            command->source_id, 0U);
    }
    permission = 1UL << command->command_kind;
    if ((source->permissions & permission) == 0U)
    {
        ++service->rejected;
        ++service->unauthorized;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_UNAUTHORIZED,
            command->source_id, command->command_kind);
    }
    if (foc_command_service_time_is_valid(command, now_ms) == 0U)
    {
        ++service->rejected;
        ++service->expired;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_EXPIRED,
            command->source_id, command->valid_until_ms);
    }
    sink_result = service->ops.submit(
        service->ops.context, command, now_ms);
    if (sink_result != 0U)
    {
        ++service->rejected;
        ++service->sink_rejected;
        return foc_command_service_record(
            service, FOC_COMMAND_SERVICE_SINK_REJECTED,
            command->source_id, sink_result);
    }
    ++service->accepted;
    return foc_command_service_record(
        service, FOC_COMMAND_SERVICE_OK, command->source_id, 0U);
}

foc_command_service_result_t foc_command_service_get_status(
    const foc_command_service_t *service,
    foc_command_service_status_t *status)
{
    if ((foc_command_service_is_valid(service) == 0U) || (status == 0))
    {
        return FOC_COMMAND_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_COMMAND_SERVICE_VERSION;
    status->enabled = service->enabled;
    status->source_count = service->source_count;
    status->submitted = service->submitted;
    status->accepted = service->accepted;
    status->rejected = service->rejected;
    status->unknown_source = service->unknown_source;
    status->unauthorized = service->unauthorized;
    status->expired = service->expired;
    status->sink_rejected = service->sink_rejected;
    status->last_source_id = service->last_source_id;
    status->last_result = service->last_result;
    status->last_detail = service->last_detail;
    return FOC_COMMAND_SERVICE_OK;
}
