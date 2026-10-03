/* FluxRT - default-off, allocation-free motion management owner. */

#include "foc_motion_management.h"

#include <string.h>

enum
{
    FOC_MOTION_MANAGEMENT_DETAIL_INIT = 1U,
    FOC_MOTION_MANAGEMENT_DETAIL_DISABLE = 2U,
    FOC_MOTION_MANAGEMENT_DETAIL_ENABLE = 3U,
    FOC_MOTION_MANAGEMENT_DETAIL_CONFIG = 4U,
    FOC_MOTION_MANAGEMENT_DETAIL_STEP = 5U,
    FOC_MOTION_MANAGEMENT_DETAIL_TIMEOUT = 6U,
    FOC_MOTION_MANAGEMENT_DETAIL_COMMAND_EXPIRED = 7U,
    FOC_MOTION_MANAGEMENT_DETAIL_FAULT = 8U,
    FOC_MOTION_MANAGEMENT_DETAIL_RECOVER = 9U,
    FOC_MOTION_MANAGEMENT_DETAIL_GUARD = 10U,
    FOC_MOTION_MANAGEMENT_DETAIL_COMMAND = 11U,
    FOC_MOTION_MANAGEMENT_DETAIL_SOURCE = 12U,
    FOC_MOTION_MANAGEMENT_DETAIL_SEQUENCE = 13U,
    FOC_MOTION_MANAGEMENT_DETAIL_OWNER = 14U,
};

static void foc_motion_management_safe_output(foc_motion_output_t *output)
{
    if (output != 0)
    {
        (void)memset(output, 0, sizeof(*output));
        output->struct_size = sizeof(*output);
        output->version = FOC_MOTION_OUTPUT_VERSION;
        output->control_mode = FOC_CONTROL_MODE_INACTIVE;
        output->input_mode = FOC_INPUT_MODE_INACTIVE;
    }
}

static uint32_t foc_motion_management_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_motion_management_is_valid(
    const foc_motion_management_t *management)
{
    return ((management != 0) &&
            (management->struct_size == sizeof(*management)) &&
            (management->version == FOC_MOTION_MANAGEMENT_VERSION) &&
            (management->initialized != 0U) &&
            (management->adapter != 0) &&
            (management->ops.struct_size == sizeof(management->ops)) &&
            (management->ops.version == FOC_MOTION_MANAGEMENT_VERSION) &&
            (management->ops.force_safe != 0)) ? 1U : 0U;
}

static void foc_motion_management_force_safe(
    foc_motion_management_t *management)
{
    foc_motion_management_safe_output(
        (management != 0) ? &management->output : 0);
    if ((management != 0) && (management->ops.force_safe != 0))
    {
        management->ops.force_safe(management->ops.context);
    }
}

static foc_motion_management_result_t foc_motion_management_record(
    foc_motion_management_t *management,
    foc_motion_management_result_t result,
    uint32_t detail)
{
    if (management != 0)
    {
        management->last_result = result;
        management->last_detail = detail;
    }
    return result;
}

static uint32_t foc_motion_management_adapter_detail(
    uint32_t operation,
    foc_motion_status_t status)
{
    return (operation << 24) | (status & 0xFFFFU);
}

static void foc_motion_management_clear_session(
    foc_motion_management_t *management)
{
    management->owner_id = 0U;
    management->source_id = 0U;
    management->started_at_ms = 0U;
    management->last_activity_ms = 0U;
    management->timeout_ms = 0U;
    management->last_sequence = 0U;
    management->sequence_valid = 0U;
}

static foc_motion_management_result_t foc_motion_management_fail(
    foc_motion_management_t *management,
    foc_motion_management_result_t result,
    uint32_t detail)
{
    foc_motion_management_force_safe(management);
    if (management != 0)
    {
        (void)foc_motion_adapter_disable(management->adapter);
        if (management->state != FOC_MOTION_MANAGEMENT_STATE_FAILED)
        {
            ++management->failure_count;
        }
        management->state = FOC_MOTION_MANAGEMENT_STATE_FAILED;
        management->shutdown_required = 1U;
    }
    return foc_motion_management_record(management, result, detail);
}

static uint32_t foc_motion_management_session_expired(
    const foc_motion_management_t *management,
    uint32_t now_ms)
{
    return ((management->timeout_ms != 0U) &&
            ((uint32_t)(now_ms - management->last_activity_ms) >=
             management->timeout_ms)) ? 1U : 0U;
}

/* P1.2 command windows are exclusive and limited to one u32 half-range. */
static uint32_t foc_motion_management_command_expired(
    uint32_t now_ms,
    uint32_t created_at_ms,
    uint32_t valid_until_ms)
{
    uint32_t window = (uint32_t)(valid_until_ms - created_at_ms);
    uint32_t age = (uint32_t)(now_ms - created_at_ms);

    return ((window == 0U) || (window >= 0x80000000UL) ||
            (age >= 0x80000000UL) || (age >= window)) ? 1U : 0U;
}

static uint32_t foc_motion_management_sequence_is_forward(
    uint32_t previous,
    uint32_t next)
{
    uint32_t delta = (uint32_t)(next - previous);
    return ((delta != 0U) && (delta < 0x80000000UL)) ? 1U : 0U;
}

foc_motion_management_result_t foc_motion_management_init(
    foc_motion_management_t *management,
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_motion_management_ops_t *ops)
{
    foc_motion_status_t status;
    foc_motion_status_t disable_status;

    if ((management == 0) || (adapter == 0) || (config == 0) ||
        (ops == 0) || (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_MOTION_MANAGEMENT_VERSION) ||
        (ops->force_safe == 0))
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }

    (void)memset(management, 0, sizeof(*management));
    management->struct_size = sizeof(*management);
    management->version = FOC_MOTION_MANAGEMENT_VERSION;
    management->state = FOC_MOTION_MANAGEMENT_STATE_DISABLED;
    management->adapter = adapter;
    management->ops = *ops;
    management->initialized = 1U;
    foc_motion_management_force_safe(management);

    status = foc_motion_adapter_init(adapter, config);
    if ((status != FOC_MOTION_STATUS_OK) &&
        (status != FOC_MOTION_STATUS_DISABLED))
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_INIT, status));
    }

    disable_status = foc_motion_adapter_disable(adapter);
    if ((disable_status != FOC_MOTION_STATUS_OK) &&
        (disable_status != FOC_MOTION_STATUS_DISABLED))
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_DISABLE, disable_status));
    }
    management->enabled = 0U;
    management->shutdown_required = 0U;
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_set_enabled(
    foc_motion_management_t *management,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;
    uint32_t already_disabled;

    if ((foc_motion_management_is_valid(management) == 0U) ||
        (enabled > 1U))
    {
        if (management != 0)
        {
            (void)foc_motion_management_record(
                management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT, 0U);
        }
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }

    if (enabled == 0U)
    {
        already_disabled =
            ((management->enabled == 0U) &&
             (management->state == FOC_MOTION_MANAGEMENT_STATE_DISABLED));
        foc_motion_management_force_safe(management);
        status = foc_motion_adapter_disable(management->adapter);
        management->enabled = 0U;
        if ((status != FOC_MOTION_STATUS_OK) &&
            (status != FOC_MOTION_STATUS_DISABLED))
        {
            return foc_motion_management_fail(
                management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
                foc_motion_management_adapter_detail(
                    FOC_MOTION_MANAGEMENT_DETAIL_DISABLE, status));
        }
        if ((management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED) ||
            (management->shutdown_required != 0U))
        {
            return foc_motion_management_record(
                management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
                management->last_detail);
        }
        management->state = FOC_MOTION_MANAGEMENT_STATE_DISABLED;
        management->shutdown_required = 0U;
        foc_motion_management_clear_session(management);
        return foc_motion_management_record(
            management,
            (already_disabled != 0U) ?
                FOC_MOTION_MANAGEMENT_ALREADY_SAFE :
                FOC_MOTION_MANAGEMENT_OK,
            0U);
    }

    if (foc_motion_management_guard_is_safe(guard) == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_UNSAFE_STATE,
            FOC_MOTION_MANAGEMENT_DETAIL_GUARD);
    }
    if ((management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED) ||
        (management->shutdown_required != 0U))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_ACTIVE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_BUSY, 0U);
    }
    management->enabled = 1U;
    management->state = FOC_MOTION_MANAGEMENT_STATE_IDLE;
    management->shutdown_required = 0U;
    foc_motion_management_clear_session(management);
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_apply_config(
    foc_motion_management_t *management,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;

    if ((foc_motion_management_is_valid(management) == 0U) || (config == 0))
    {
        if (management != 0)
        {
            (void)foc_motion_management_record(
                management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT, 0U);
        }
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (foc_motion_management_guard_is_safe(guard) == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_UNSAFE_STATE,
            FOC_MOTION_MANAGEMENT_DETAIL_GUARD);
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->enabled == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_DISABLED, 0U);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_IDLE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_BUSY, 0U);
    }
    status = foc_motion_adapter_apply_config(
        management->adapter, config, guard);
    if (status != FOC_MOTION_STATUS_OK)
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_CONFIG, status));
    }
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_get_status(
    foc_motion_management_t *management,
    foc_motion_management_status_t *status)
{
    if ((foc_motion_management_is_valid(management) == 0U) || (status == 0))
    {
        if (management != 0)
        {
            (void)foc_motion_management_record(
                management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT, 0U);
        }
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }

    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_MOTION_MANAGEMENT_VERSION;
    status->enabled = management->enabled;
    status->state = management->state;
    status->owner_id = management->owner_id;
    status->source_id = management->source_id;
    status->started_at_ms = management->started_at_ms;
    status->last_activity_ms = management->last_activity_ms;
    status->timeout_ms = management->timeout_ms;
    status->last_sequence = management->last_sequence;
    status->sequence_valid = management->sequence_valid;
    status->shutdown_required = management->shutdown_required;
    status->begin_count = management->begin_count;
    status->stop_count = management->stop_count;
    status->timeout_count = management->timeout_count;
    status->failure_count = management->failure_count;
    status->step_count = management->step_count;
    status->last_result = management->last_result;
    status->last_detail = management->last_detail;
    status->motion_status = management->adapter->last_status;
    status->output = management->output;
    return FOC_MOTION_MANAGEMENT_OK;
}

foc_motion_management_result_t foc_motion_management_begin(
    foc_motion_management_t *management,
    uint32_t owner_id,
    uint32_t source_id,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;

    if (foc_motion_management_is_valid(management) == 0U)
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if ((owner_id == 0U) || (source_id == 0U) ||
        (timeout_ms < FOC_MOTION_MANAGEMENT_MIN_TIMEOUT_MS) ||
        (timeout_ms > FOC_MOTION_MANAGEMENT_MAX_TIMEOUT_MS))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT, 0U);
    }
    if (foc_motion_management_guard_is_safe(guard) == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_UNSAFE_STATE,
            FOC_MOTION_MANAGEMENT_DETAIL_GUARD);
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->enabled == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_DISABLED, 0U);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_IDLE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_BUSY, 0U);
    }

    status = foc_motion_adapter_enable(management->adapter, guard);
    if (status != FOC_MOTION_STATUS_OK)
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_ENABLE, status));
    }
    foc_motion_management_safe_output(&management->output);
    management->owner_id = owner_id;
    management->source_id = source_id;
    management->started_at_ms = now_ms;
    management->last_activity_ms = now_ms;
    management->timeout_ms = timeout_ms;
    management->last_sequence = 0U;
    management->sequence_valid = 0U;
    management->shutdown_required = 0U;
    management->state = FOC_MOTION_MANAGEMENT_STATE_ACTIVE;
    ++management->begin_count;
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_step(
    foc_motion_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output)
{
    foc_motion_status_t status;

    foc_motion_management_safe_output(output);
    if ((foc_motion_management_is_valid(management) == 0U) ||
        (command == 0) || (feedback == 0) || (output == 0))
    {
        if (management != 0)
        {
            (void)foc_motion_management_record(
                management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT, 0U);
        }
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->enabled == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_DISABLED, 0U);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_ACTIVE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_NOT_READY, 0U);
    }
    if ((owner_id == 0U) || (owner_id != management->owner_id))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_NOT_OWNER,
            FOC_MOTION_MANAGEMENT_DETAIL_OWNER);
    }
    if (foc_motion_management_session_expired(management, now_ms) != 0U)
    {
        ++management->timeout_count;
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_EXPIRED,
            FOC_MOTION_MANAGEMENT_DETAIL_TIMEOUT);
    }
    if ((command->struct_size != sizeof(*command)) ||
        (command->version != FOC_PRODUCT_COMMAND_VERSION))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT,
            FOC_MOTION_MANAGEMENT_DETAIL_COMMAND);
    }
    if (command->source_id != management->source_id)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_WRONG_SOURCE,
            FOC_MOTION_MANAGEMENT_DETAIL_SOURCE);
    }
    if (foc_motion_management_command_expired(
            now_ms, command->created_at_ms, command->valid_until_ms) != 0U)
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_EXPIRED,
            FOC_MOTION_MANAGEMENT_DETAIL_COMMAND_EXPIRED);
    }
    if ((management->sequence_valid != 0U) &&
        (foc_motion_management_sequence_is_forward(
            management->last_sequence, command->sequence) == 0U))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_STALE_SEQUENCE,
            FOC_MOTION_MANAGEMENT_DETAIL_SEQUENCE);
    }

    status = foc_motion_adapter_step(
        management->adapter, command, feedback, output);
    if (status != FOC_MOTION_STATUS_OK)
    {
        foc_motion_management_safe_output(output);
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_STEP, status));
    }
    management->output = *output;
    management->last_sequence = command->sequence;
    management->sequence_valid = 1U;
    management->last_activity_ms = now_ms;
    ++management->step_count;
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_stop(
    foc_motion_management_t *management,
    uint32_t owner_id)
{
    foc_motion_status_t status;

    if (foc_motion_management_is_valid(management) == 0U)
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_ACTIVE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_ALREADY_SAFE, 0U);
    }
    if ((owner_id == 0U) || (owner_id != management->owner_id))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_NOT_OWNER,
            FOC_MOTION_MANAGEMENT_DETAIL_OWNER);
    }

    foc_motion_management_force_safe(management);
    status = foc_motion_adapter_disable(management->adapter);
    if ((status != FOC_MOTION_STATUS_OK) &&
        (status != FOC_MOTION_STATUS_DISABLED))
    {
        return foc_motion_management_fail(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_DISABLE, status));
    }
    ++management->stop_count;
    foc_motion_management_clear_session(management);
    management->shutdown_required = 0U;
    management->state = (management->enabled != 0U) ?
        FOC_MOTION_MANAGEMENT_STATE_IDLE :
        FOC_MOTION_MANAGEMENT_STATE_DISABLED;
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}

foc_motion_management_result_t foc_motion_management_latch_fault(
    foc_motion_management_t *management,
    uint32_t detail)
{
    if (foc_motion_management_is_valid(management) == 0U)
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    return foc_motion_management_fail(
        management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
        (detail != 0U) ? detail : FOC_MOTION_MANAGEMENT_DETAIL_FAULT);
}

foc_motion_management_result_t foc_motion_management_poll(
    foc_motion_management_t *management,
    uint32_t now_ms)
{
    if (foc_motion_management_is_valid(management) == 0U)
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        foc_motion_management_force_safe(management);
        (void)foc_motion_adapter_disable(management->adapter);
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_ACTIVE)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_ALREADY_SAFE, 0U);
    }
    if (foc_motion_management_session_expired(management, now_ms) == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_OK, 0U);
    }
    ++management->timeout_count;
    return foc_motion_management_fail(
        management, FOC_MOTION_MANAGEMENT_EXPIRED,
        FOC_MOTION_MANAGEMENT_DETAIL_TIMEOUT);
}

foc_motion_management_result_t foc_motion_management_recover(
    foc_motion_management_t *management,
    const foc_config_apply_guard_t *guard)
{
    foc_motion_status_t status;

    if (foc_motion_management_is_valid(management) == 0U)
    {
        return FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (foc_motion_management_guard_is_safe(guard) == 0U)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_UNSAFE_STATE,
            FOC_MOTION_MANAGEMENT_DETAIL_GUARD);
    }
    if (management->state != FOC_MOTION_MANAGEMENT_STATE_FAILED)
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_ALREADY_SAFE, 0U);
    }

    foc_motion_management_force_safe(management);
    status = foc_motion_adapter_disable(management->adapter);
    if ((status != FOC_MOTION_STATUS_OK) &&
        (status != FOC_MOTION_STATUS_DISABLED))
    {
        return foc_motion_management_record(
            management, FOC_MOTION_MANAGEMENT_MOTION_ERROR,
            foc_motion_management_adapter_detail(
                FOC_MOTION_MANAGEMENT_DETAIL_RECOVER, status));
    }
    foc_motion_management_clear_session(management);
    management->shutdown_required = 0U;
    management->state = (management->enabled != 0U) ?
        FOC_MOTION_MANAGEMENT_STATE_IDLE :
        FOC_MOTION_MANAGEMENT_STATE_DISABLED;
    return foc_motion_management_record(
        management, FOC_MOTION_MANAGEMENT_OK, 0U);
}
