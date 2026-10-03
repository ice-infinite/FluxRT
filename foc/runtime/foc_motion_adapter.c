#include "foc_motion_adapter.h"

#include <stddef.h>
#include <string.h>

static void foc_motion_adapter_safe_output(foc_motion_output_t *output)
{
    if (output != NULL)
    {
        (void)memset(output, 0, sizeof(*output));
        output->struct_size = (uint32_t)sizeof(*output);
        output->version = FOC_MOTION_OUTPUT_VERSION;
        output->control_mode = FOC_CONTROL_MODE_INACTIVE;
        output->input_mode = FOC_INPUT_MODE_INACTIVE;
    }
}

#if defined(FLUXRT_DIAGNOSTIC_BUILD)

static bool foc_motion_adapter_guard_is_safe(const foc_config_apply_guard_t *guard)
{
    return (guard != NULL) &&
           (guard->struct_size == (uint32_t)sizeof(*guard)) &&
           (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
           (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
           (guard->drive_active == 0U) &&
           (guard->active_fault_flags == 0U);
}

foc_motion_status_t foc_motion_adapter_init(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config)
{
    foc_motion_status_t status;
    if ((adapter == NULL) || (config == NULL))
    {
        return FOC_MOTION_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(adapter, 0, sizeof(*adapter));
    if ((foc_rust_motion_abi_version() != FOC_MOTION_ABI_VERSION) ||
        (foc_rust_motion_context_required_size() > FOC_MOTION_CONTEXT_CAPACITY) ||
        (foc_rust_motion_context_required_align() > _Alignof(foc_motion_context_t)))
    {
        adapter->last_status = FOC_MOTION_STATUS_INVALID_ARGUMENT;
        return adapter->last_status;
    }
    status = foc_rust_motion_init(&adapter->rust_context);
    if (status != FOC_MOTION_STATUS_OK)
    {
        adapter->last_status = status;
        return status;
    }
    adapter->initialized = 1U;
    status = foc_rust_motion_configure(&adapter->rust_context, config);
    adapter->last_status = status;
    if (status == FOC_MOTION_STATUS_OK)
    {
        adapter->configured = 1U;
    }
    /* Configuration never enables actuation; enable requires a later guard. */
    adapter->enabled = 0U;
    return status;
}

foc_motion_status_t foc_motion_adapter_apply_config(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;
    if ((adapter == NULL) || (config == NULL))
    {
        return FOC_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if ((adapter->initialized == 0U) || (adapter->enabled != 0U) ||
        !foc_motion_adapter_guard_is_safe(guard))
    {
        adapter->last_status = FOC_MOTION_STATUS_INVALID_STATE;
        return adapter->last_status;
    }
    status = foc_rust_motion_configure(&adapter->rust_context, config);
    adapter->last_status = status;
    adapter->configured = (status == FOC_MOTION_STATUS_OK) ? 1U : 0U;
    return status;
}

foc_motion_status_t foc_motion_adapter_enable(
    foc_motion_adapter_t *adapter,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;
    if (adapter == NULL)
    {
        return FOC_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if ((adapter->initialized == 0U) || (adapter->configured == 0U) ||
        !foc_motion_adapter_guard_is_safe(guard))
    {
        adapter->last_status = FOC_MOTION_STATUS_INVALID_STATE;
        return adapter->last_status;
    }
    status = foc_rust_motion_enable(&adapter->rust_context);
    adapter->last_status = status;
    adapter->enabled = (status == FOC_MOTION_STATUS_OK) ? 1U : 0U;
    return status;
}

foc_motion_status_t foc_motion_adapter_disable(foc_motion_adapter_t *adapter)
{
    foc_motion_status_t status;
    if ((adapter == NULL) || (adapter->initialized == 0U))
    {
        return FOC_MOTION_STATUS_INVALID_ARGUMENT;
    }
    status = foc_rust_motion_disable(&adapter->rust_context);
    adapter->last_status = status;
    if (status == FOC_MOTION_STATUS_OK)
    {
        adapter->enabled = 0U;
    }
    return status;
}

foc_motion_status_t foc_motion_adapter_step(
    foc_motion_adapter_t *adapter,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output)
{
    foc_motion_status_t status;
    uint32_t config_revision;
    foc_motion_adapter_safe_output(output);
    if ((adapter == NULL) || (command == NULL) || (feedback == NULL) ||
        (output == NULL))
    {
        return FOC_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if ((adapter->initialized == 0U) || (adapter->configured == 0U) ||
        (adapter->enabled == 0U))
    {
        adapter->last_status = FOC_MOTION_STATUS_DISABLED;
        return adapter->last_status;
    }
    status = foc_rust_motion_step(&adapter->rust_context, command, feedback, output);
    adapter->last_status = status;
    if (status != FOC_MOTION_STATUS_OK)
    {
        config_revision = output->config_revision;
        /* Rust also returns a safe output; repeat here to keep the C boundary
         * fail-safe even when linked against a malformed/mock implementation. */
        foc_motion_adapter_safe_output(output);
        output->config_revision = config_revision;
        output->source_sequence = command->sequence;
    }
    return status;
}

#else

/* Non-Diagnostic profiles deliberately contain no Rust motion references even
 * if a stale generated rtconfig.h still carries the candidate Kconfig symbol. */
foc_motion_status_t foc_motion_adapter_init(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config)
{
    (void)config;
    if (adapter != NULL)
    {
        (void)memset(adapter, 0, sizeof(*adapter));
        adapter->last_status = FOC_MOTION_STATUS_DISABLED;
    }
    return FOC_MOTION_STATUS_DISABLED;
}

foc_motion_status_t foc_motion_adapter_apply_config(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard)
{
    (void)adapter;
    (void)config;
    (void)guard;
    return FOC_MOTION_STATUS_DISABLED;
}

foc_motion_status_t foc_motion_adapter_enable(
    foc_motion_adapter_t *adapter,
    const foc_config_apply_guard_t *guard)
{
    (void)adapter;
    (void)guard;
    return FOC_MOTION_STATUS_DISABLED;
}

foc_motion_status_t foc_motion_adapter_disable(foc_motion_adapter_t *adapter)
{
    (void)adapter;
    return FOC_MOTION_STATUS_DISABLED;
}

foc_motion_status_t foc_motion_adapter_step(
    foc_motion_adapter_t *adapter,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output)
{
    (void)adapter;
    (void)command;
    (void)feedback;
    foc_motion_adapter_safe_output(output);
    return FOC_MOTION_STATUS_DISABLED;
}

#endif
