#include "foc_external_input_owner.h"

#include "foc_simple_input_adapter.h"

#include <string.h>

static const foc_external_input_mask_t g_owner_input_bits[
    FOC_EXTERNAL_IO_INPUT_COUNT] =
{
    FOC_EXTERNAL_INPUT_PWM_PULSE,
    FOC_EXTERNAL_INPUT_DSHOT,
    FOC_EXTERNAL_INPUT_ANALOG,
    FOC_EXTERNAL_INPUT_STEP_DIR,
};

static uint32_t foc_external_input_owner_is_valid(
    const foc_external_input_owner_t *owner)
{
    return ((owner != 0) &&
            (owner->struct_size == sizeof(*owner)) &&
            (owner->version == FOC_EXTERNAL_INPUT_OWNER_VERSION) &&
            (owner->initialized != 0U) &&
            (owner->management != 0) &&
            (owner->port != 0)) ? 1U : 0U;
}

static uint32_t foc_external_input_owner_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static int32_t foc_external_input_owner_index(
    foc_external_input_mask_t input)
{
    uint32_t index;
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        if (g_owner_input_bits[index] == input)
        {
            return (int32_t)index;
        }
    }
    return -1;
}

static foc_external_input_owner_result_t
foc_external_input_owner_submit_release(
    foc_external_input_owner_t *owner,
    uint32_t index,
    uint32_t now_ms)
{
    foc_product_command_t command;
    const foc_external_input_config_t *config =
        &owner->management->active_config.inputs[index];
    foc_command_service_result_t result;

    (void)memset(&command, 0, sizeof(command));
    command.struct_size = sizeof(command);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.axis_id = 0U;
    command.source_id = config->source.source_id;
    command.sequence = ++owner->command_sequence[index];
    command.created_at_ms = now_ms;
    command.valid_until_ms = now_ms + config->source.command_timeout_ms;
    command.command_kind = FOC_PRODUCT_COMMAND_RELEASE;
    command.axis_request = FOC_AXIS_REQUEST_NONE;
    command.control_mode = FOC_CONTROL_MODE_INACTIVE;
    command.input_mode = FOC_INPUT_MODE_INACTIVE;
    command.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;

    result = foc_command_service_submit(
        &owner->management->command_service, &command, now_ms);
    if (result != FOC_COMMAND_SERVICE_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_COMMAND_FAILED;
    }
    ++owner->status.release_count;
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}

static foc_external_input_owner_result_t
foc_external_input_owner_fail(
    foc_external_input_owner_t *owner,
    foc_external_input_mask_t input,
    foc_external_input_failure_reason_t reason,
    uint32_t now_ms)
{
    int32_t signed_index = foc_external_input_owner_index(input);
    uint32_t index;
    uint32_t quality = FOC_INPUT_SAMPLE_QUALITY_DEGRADED;
    foc_input_service_result_t service_result;
    foc_external_input_owner_result_t release_result;

    if (signed_index < 0)
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    index = (uint32_t)signed_index;
    if (reason == FOC_EXTERNAL_INPUT_FAILURE_TIMEOUT)
    {
        quality |= FOC_INPUT_SAMPLE_QUALITY_STALE;
    }
    service_result = foc_input_service_mark_unhealthy(
        &owner->management->input_service, input, quality);
    if ((service_result != FOC_INPUT_SERVICE_OK) &&
        (service_result != FOC_INPUT_SERVICE_DISABLED))
    {
        return FOC_EXTERNAL_INPUT_OWNER_POLL_FAILED;
    }
    owner->status.last_failure_reason = reason;
    owner->status.requested_failure_action =
        owner->management->active_config.inputs[index].failure_action;
    /* Simple inputs are intentionally authorized only for SETPOINT and RELEASE.
     * Until MotorService owns a controlled-stop request, RELEASE is the safe
     * executable fallback for both configured Release and ControlledStop. */
    owner->status.effective_failure_action = FOC_EXTERNAL_FAILURE_RELEASE;
    owner->status.healthy_input_mask &= ~input;
    owner->status.degraded_input_mask |= input;
    if ((owner->status.fault_input_mask & input) != 0U)
    {
        return FOC_EXTERNAL_INPUT_OWNER_OK;
    }
    owner->status.fault_input_mask |= input;
    ++owner->status.failure_count;
    release_result = foc_external_input_owner_submit_release(
        owner, index, now_ms);
    return release_result;
}

#if defined(FLUXRT_INPUT_ANALOG)
static foc_external_input_owner_result_t
foc_external_input_owner_process_analog(
    foc_external_input_owner_t *owner,
    const foc_external_input_raw_sample_t *raw,
    uint32_t now_ms)
{
    const uint32_t index = 2U;
    foc_centered_input_config_t config;
    foc_input_normalization_output_t normalized;
    foc_simple_input_candidate_t candidate;
    foc_simple_input_adapter_result_t adapter_result;
    foc_external_io_management_result_t management_result;
    uint32_t was_faulted;

    if (((raw->valid_flags &
          (FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
           FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP)) !=
         (FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
          FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP)) ||
        (raw->instance_id != 0U))
    {
        return foc_external_input_owner_fail(
            owner, FOC_EXTERNAL_INPUT_ANALOG,
            FOC_EXTERNAL_INPUT_FAILURE_INVALID_RAW, now_ms);
    }
    adapter_result = foc_simple_input_adapter_map_centered_config(
        &owner->management->active_config,
        FOC_EXTERNAL_INPUT_ANALOG, &config);
    if (adapter_result != FOC_SIMPLE_INPUT_ADAPTER_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_CONFIG_REJECTED;
    }
    (void)memset(&normalized, 0, sizeof(normalized));
    owner->status.last_normalization_status =
        foc_rust_input_normalize_analog(&config, raw->raw_value, &normalized);
    if (owner->status.last_normalization_status != FOC_INPUT_STATUS_OK)
    {
        return foc_external_input_owner_fail(
            owner, FOC_EXTERNAL_INPUT_ANALOG,
            (owner->status.last_normalization_status ==
             FOC_INPUT_STATUS_OUT_OF_RANGE) ?
                FOC_EXTERNAL_INPUT_FAILURE_OUT_OF_RANGE :
                FOC_EXTERNAL_INPUT_FAILURE_INVALID_RAW,
            now_ms);
    }
    adapter_result = foc_simple_input_adapter_build_candidate(
        FOC_EXTERNAL_INPUT_ANALOG,
        owner->management->active_config.inputs[index].source.source_id,
        ++owner->command_sequence[index], raw->sampled_at_us, now_ms,
        owner->management->active_config.inputs[index].source.command_timeout_ms,
        owner->management->active_config.inputs[index].control_mode,
        FOC_FEEDBACK_MODE_SENSORLESS, &normalized, &candidate);
    if (adapter_result != FOC_SIMPLE_INPUT_ADAPTER_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_NORMALIZATION_FAILED;
    }
    if ((raw->quality_flags &
         (FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS |
          FOC_EXTERNAL_INPUT_RAW_QUALITY_OVERRUN)) != 0U)
    {
        candidate.sample.quality_flags |=
            FOC_INPUT_SAMPLE_QUALITY_DEGRADED;
        owner->status.degraded_input_mask |= FOC_EXTERNAL_INPUT_ANALOG;
    }
    else
    {
        owner->status.degraded_input_mask &=
            ~(uint32_t)FOC_EXTERNAL_INPUT_ANALOG;
    }
    was_faulted = owner->status.fault_input_mask & FOC_EXTERNAL_INPUT_ANALOG;
    management_result = foc_external_io_management_submit_simple_input(
        owner->management, &candidate, now_ms);
    if (management_result != FOC_EXTERNAL_IO_MANAGEMENT_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_COMMAND_FAILED;
    }
    owner->last_success_ms[index] = now_ms;
    owner->status.healthy_input_mask |= FOC_EXTERNAL_INPUT_ANALOG;
    owner->status.fault_input_mask &=
        ~(uint32_t)FOC_EXTERNAL_INPUT_ANALOG;
    owner->status.last_failure_reason = FOC_EXTERNAL_INPUT_FAILURE_NONE;
    ++owner->status.setpoint_count;
    if (was_faulted != 0U)
    {
        ++owner->status.recovery_count;
    }
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}
#endif

static foc_external_input_owner_result_t
foc_external_input_owner_process_latest(
    foc_external_input_owner_t *owner,
    foc_external_input_mask_t input,
    uint32_t now_ms)
{
    foc_external_input_raw_sample_t raw;
    foc_external_input_port_result_t port_result;
    int32_t signed_index = foc_external_input_owner_index(input);
    uint32_t index;

    if (signed_index < 0)
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    index = (uint32_t)signed_index;
    port_result = foc_external_input_port_read_latest(
        owner->port, input, &raw);
    owner->status.last_port_result = port_result;
    if ((port_result == FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE) ||
        (port_result == FOC_EXTERNAL_INPUT_PORT_BUSY))
    {
        return FOC_EXTERNAL_INPUT_OWNER_OK;
    }
    if (port_result != FOC_EXTERNAL_INPUT_PORT_OK)
    {
        return foc_external_input_owner_fail(
            owner, input, FOC_EXTERNAL_INPUT_FAILURE_PORT, now_ms);
    }
    if ((owner->raw_sequence_valid[index] != 0U) &&
        (owner->last_raw_sequence[index] == raw.sequence))
    {
        return FOC_EXTERNAL_INPUT_OWNER_OK;
    }
    owner->last_raw_sequence[index] = raw.sequence;
    owner->raw_sequence_valid[index] = 1U;
    ++owner->status.raw_sample_count;
#if defined(FLUXRT_INPUT_ANALOG)
    if (input == FOC_EXTERNAL_INPUT_ANALOG)
    {
        return foc_external_input_owner_process_analog(owner, &raw, now_ms);
    }
#endif
    return foc_external_input_owner_fail(
        owner, input, FOC_EXTERNAL_INPUT_FAILURE_INVALID_RAW, now_ms);
}

foc_external_input_owner_result_t foc_external_input_owner_init(
    foc_external_input_owner_t *owner,
    foc_external_io_management_t *management,
    foc_external_input_port_t *port)
{
    if ((owner == 0) || (management == 0) || (port == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U) ||
        (port->struct_size != sizeof(*port)) ||
        (port->version != FOC_EXTERNAL_INPUT_PORT_VERSION) ||
        (port->initialized == 0U) ||
        ((port->supported_input_mask &
          ~management->capabilities.compiled_input_mask) != 0U) ||
        (port->enabled_input_mask != 0U))
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    (void)memset(owner, 0, sizeof(*owner));
    owner->struct_size = sizeof(*owner);
    owner->version = FOC_EXTERNAL_INPUT_OWNER_VERSION;
    owner->management = management;
    owner->port = port;
    owner->status.struct_size = sizeof(owner->status);
    owner->status.version = FOC_EXTERNAL_INPUT_OWNER_STATUS_VERSION;
    owner->status.initialized = 1U;
    owner->status.managed_input_mask = port->supported_input_mask;
    owner->status.last_port_result = FOC_EXTERNAL_INPUT_PORT_DISABLED;
    owner->status.last_normalization_status = FOC_INPUT_STATUS_INVALID_ARGUMENT;
    owner->initialized = 1U;
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}

foc_external_input_owner_result_t foc_external_input_owner_apply_config(
    foc_external_input_owner_t *owner,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    uint32_t now_ms)
{
    foc_external_io_validation_t validation;
    foc_external_input_mask_t desired_mask;
    foc_external_input_mask_t removed_mask;
    foc_external_input_mask_t added_mask;
    foc_external_input_port_result_t port_result;
    foc_external_io_management_result_t management_result;
    uint32_t index;

    if ((foc_external_input_owner_is_valid(owner) == 0U) ||
        (config == 0) || (guard == 0))
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    if (foc_external_input_owner_guard_is_safe(guard) == 0U)
    {
        return FOC_EXTERNAL_INPUT_OWNER_UNSAFE_STATE;
    }
    if (foc_external_io_validate_config(
            &owner->management->capabilities,
            config, &validation) != FOC_EXTERNAL_IO_STATUS_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_CONFIG_REJECTED;
    }
    desired_mask = config->input_enable_mask &
        owner->status.managed_input_mask;
    removed_mask = owner->status.enabled_input_mask & ~desired_mask;
    added_mask = desired_mask & ~owner->status.enabled_input_mask;

    if (removed_mask != 0U)
    {
        port_result = foc_external_input_port_stop(owner->port, removed_mask);
        owner->status.last_port_result = port_result;
        if (port_result != FOC_EXTERNAL_INPUT_PORT_OK)
        {
            return FOC_EXTERNAL_INPUT_OWNER_PORT_FAILED;
        }
    }
    if (added_mask != 0U)
    {
        port_result = foc_external_input_port_start(owner->port, added_mask);
        owner->status.last_port_result = port_result;
        if (port_result != FOC_EXTERNAL_INPUT_PORT_OK)
        {
            if (removed_mask != 0U)
            {
                (void)foc_external_input_port_start(owner->port, removed_mask);
            }
            return FOC_EXTERNAL_INPUT_OWNER_PORT_FAILED;
        }
    }
    management_result = foc_external_io_management_apply_config(
        owner->management, config, guard);
    if (management_result != FOC_EXTERNAL_IO_MANAGEMENT_OK)
    {
        if (added_mask != 0U)
        {
            (void)foc_external_input_port_stop(owner->port, added_mask);
        }
        if (removed_mask != 0U)
        {
            (void)foc_external_input_port_start(owner->port, removed_mask);
        }
        return FOC_EXTERNAL_INPUT_OWNER_CONFIG_REJECTED;
    }
    owner->status.enabled_input_mask = desired_mask;
    owner->status.healthy_input_mask &= desired_mask;
    owner->status.fault_input_mask &= desired_mask;
    owner->status.degraded_input_mask &= desired_mask;
    for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
    {
        foc_external_input_mask_t bit = g_owner_input_bits[index];
        if ((added_mask & bit) != 0U)
        {
            owner->activation_ms[index] = now_ms;
            owner->last_success_ms[index] = now_ms;
            owner->raw_sequence_valid[index] = 0U;
        }
        if ((removed_mask & bit) != 0U)
        {
            owner->activation_ms[index] = 0U;
            owner->last_success_ms[index] = 0U;
            owner->raw_sequence_valid[index] = 0U;
            owner->command_sequence[index] = 0U;
        }
    }
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}

foc_external_input_owner_result_t foc_external_input_owner_poll(
    foc_external_input_owner_t *owner,
    uint32_t now_ms,
    uint32_t fault_active)
{
    foc_external_input_raw_health_t health;
    foc_external_input_owner_result_t result;
    foc_external_io_management_result_t management_result;
    uint32_t index;

    if (foc_external_input_owner_is_valid(owner) == 0U)
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    if (owner->status.enabled_input_mask != 0U)
    {
        owner->status.last_port_result =
            foc_external_input_port_get_health(owner->port, &health);
        if (owner->status.last_port_result != FOC_EXTERNAL_INPUT_PORT_OK)
        {
            return FOC_EXTERNAL_INPUT_OWNER_PORT_FAILED;
        }
        for (index = 0U; index < FOC_EXTERNAL_IO_INPUT_COUNT; ++index)
        {
            foc_external_input_mask_t bit = g_owner_input_bits[index];
            uint32_t timeout_ms;
            uint32_t reference_ms;
            uint32_t previous_sequence;
            uint32_t previous_sequence_valid;
            uint32_t processed_new_sample;
            if ((owner->status.enabled_input_mask & bit) == 0U)
            {
                continue;
            }
            if ((health.fault_input_mask & bit) != 0U)
            {
                result = foc_external_input_owner_fail(
                    owner, bit, FOC_EXTERNAL_INPUT_FAILURE_PORT, now_ms);
                if (result != FOC_EXTERNAL_INPUT_OWNER_OK)
                {
                    return result;
                }
            }
            previous_sequence = owner->last_raw_sequence[index];
            previous_sequence_valid = owner->raw_sequence_valid[index];
            result = foc_external_input_owner_process_latest(
                owner, bit, now_ms);
            if (result != FOC_EXTERNAL_INPUT_OWNER_OK)
            {
                return result;
            }
            processed_new_sample =
                ((owner->raw_sequence_valid[index] != 0U) &&
                 ((previous_sequence_valid == 0U) ||
                  (owner->last_raw_sequence[index] != previous_sequence))) ?
                    1U : 0U;
            /* Preserve the precise failure reason of a newly consumed sample.
             * Timeout is a no-sample condition and must not overwrite an
             * invalid/over-range sample in the same management tick. */
            if ((processed_new_sample != 0U) &&
                ((owner->status.fault_input_mask & bit) != 0U))
            {
                continue;
            }
            timeout_ms = owner->management->active_config.inputs[index]
                .source.command_timeout_ms;
            reference_ms =
                ((owner->status.healthy_input_mask & bit) != 0U) ?
                    owner->last_success_ms[index] : owner->activation_ms[index];
            if ((timeout_ms != 0U) &&
                ((uint32_t)(now_ms - reference_ms) >= timeout_ms))
            {
                result = foc_external_input_owner_fail(
                    owner, bit, FOC_EXTERNAL_INPUT_FAILURE_TIMEOUT, now_ms);
                if (result != FOC_EXTERNAL_INPUT_OWNER_OK)
                {
                    return result;
                }
            }
        }
    }
    management_result = foc_external_io_management_poll(
        owner->management, now_ms, fault_active);
    if (management_result != FOC_EXTERNAL_IO_MANAGEMENT_OK)
    {
        return FOC_EXTERNAL_INPUT_OWNER_POLL_FAILED;
    }
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}

foc_external_input_owner_result_t foc_external_input_owner_get_status(
    const foc_external_input_owner_t *owner,
    foc_external_input_owner_status_t *status)
{
    if ((foc_external_input_owner_is_valid(owner) == 0U) || (status == 0))
    {
        return FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    }
    *status = owner->status;
    return FOC_EXTERNAL_INPUT_OWNER_OK;
}
