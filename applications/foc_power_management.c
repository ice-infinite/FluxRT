#include "foc_power_management.h"

#include <string.h>

static uint32_t foc_power_management_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_power_management_is_valid(
    const foc_power_management_t *management)
{
    return ((management != 0) &&
            (management->struct_size == sizeof(*management)) &&
            (management->version == FOC_POWER_MANAGEMENT_VERSION) &&
            (management->initialized != 0U) &&
            (management->ops.struct_size == sizeof(management->ops)) &&
            (management->ops.version == FOC_POWER_MANAGEMENT_VERSION) &&
            (management->ops.force_safe != 0)) ? 1U : 0U;
}

static void foc_power_management_force_safe(foc_power_management_t *management)
{
    if ((management != 0) && (management->ops.force_safe != 0))
    {
        management->ops.force_safe(management->ops.context);
    }
}

foc_power_management_result_t foc_power_management_init(
    foc_power_management_t *management,
    const foc_power_management_ops_t *ops)
{
    foc_power_status_t status;

    if ((management == 0) || (ops == 0) ||
        (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_POWER_MANAGEMENT_VERSION) ||
        (ops->force_safe == 0) ||
        (foc_rust_power_abi_version() != FOC_POWER_ABI_VERSION))
    {
        if ((ops != 0) && (ops->force_safe != 0))
        {
            ops->force_safe(ops->context);
        }
        return FOC_POWER_MANAGEMENT_INVALID_ARGUMENT;
    }

    memset(management, 0, sizeof(*management));
    management->struct_size = sizeof(*management);
    management->version = FOC_POWER_MANAGEMENT_VERSION;
    management->ops = *ops;
    foc_power_management_force_safe(management);

    status = foc_rust_power_init(&management->storage);
    if (status == FOC_POWER_STATUS_OK)
    {
        status = foc_rust_power_default_config(&management->active_config);
    }
    if ((status == FOC_POWER_STATUS_OK) &&
        ((management->active_config.struct_size !=
          sizeof(management->active_config)) ||
         (management->active_config.abi_version != FOC_POWER_ABI_VERSION) ||
         (management->active_config.enabled != 0U) ||
         (management->active_config.regeneration_allowed != 0U) ||
         (management->active_config.brake_resistor_available != 0U)))
    {
        status = FOC_POWER_STATUS_INVALID_ARGUMENT;
    }
    if (status == FOC_POWER_STATUS_OK)
    {
        status = foc_rust_power_configure(
            &management->storage, &management->active_config);
    }
    management->last_power_status = status;
    if (status != FOC_POWER_STATUS_OK)
    {
        foc_power_management_force_safe(management);
        return FOC_POWER_MANAGEMENT_ABI_ERROR;
    }
    management->initialized = 1U;
    management->configured = 1U;
    return FOC_POWER_MANAGEMENT_OK;
}

foc_power_management_result_t foc_power_management_configure(
    foc_power_management_t *management,
    const foc_power_runtime_config_t *config,
    const foc_config_apply_guard_t *guard)
{
    foc_power_status_t status;

    if ((foc_power_management_is_valid(management) == 0U) || (config == 0))
    {
        return FOC_POWER_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (foc_power_management_guard_is_safe(guard) == 0U)
    {
        foc_power_management_force_safe(management);
        return FOC_POWER_MANAGEMENT_UNSAFE_STATE;
    }
    status = foc_rust_power_configure(&management->storage, config);
    management->last_power_status = status;
    if (status != FOC_POWER_STATUS_OK)
    {
        return FOC_POWER_MANAGEMENT_POLICY_ERROR;
    }
    management->active_config = *config;
    management->configured = 1U;
    return FOC_POWER_MANAGEMENT_OK;
}

foc_power_management_result_t foc_power_management_step(
    foc_power_management_t *management,
    const foc_power_input_t *input,
    foc_power_output_t *output)
{
    foc_power_status_t status;

    if ((foc_power_management_is_valid(management) == 0U) ||
        (input == 0) || (output == 0))
    {
        return FOC_POWER_MANAGEMENT_INVALID_ARGUMENT;
    }
    memset(output, 0, sizeof(*output));
    status = foc_rust_power_step(&management->storage, input, output);
    management->last_power_status = status;
    if (status != FOC_POWER_STATUS_OK)
    {
        foc_power_management_force_safe(management);
        return FOC_POWER_MANAGEMENT_POLICY_ERROR;
    }
    management->last_output = *output;
    if ((output->shutdown_requested != 0U) ||
        (output->latched_fault_flags != 0U) ||
        (output->drive_allowed == 0U))
    {
        foc_power_management_force_safe(management);
        return FOC_POWER_MANAGEMENT_SHUTDOWN;
    }
    return FOC_POWER_MANAGEMENT_OK;
}

foc_power_management_result_t foc_power_management_reset_faults(
    foc_power_management_t *management,
    const foc_power_input_t *input,
    const foc_config_apply_guard_t *guard)
{
    foc_power_status_t status;

    if ((foc_power_management_is_valid(management) == 0U) || (input == 0))
    {
        return FOC_POWER_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (foc_power_management_guard_is_safe(guard) == 0U)
    {
        foc_power_management_force_safe(management);
        return FOC_POWER_MANAGEMENT_UNSAFE_STATE;
    }
    status = foc_rust_power_reset_faults(&management->storage, input);
    management->last_power_status = status;
    return (status == FOC_POWER_STATUS_OK) ?
        FOC_POWER_MANAGEMENT_OK : FOC_POWER_MANAGEMENT_POLICY_ERROR;
}
