/* FluxRT - default-off, Host-testable feedback calibration management service. */

#include "foc_feedback_management.h"

#include <string.h>

enum
{
    FOC_FEEDBACK_DETAIL_ABI = 1U,
    FOC_FEEDBACK_DETAIL_FEEDBACK_STATUS = 2U,
    FOC_FEEDBACK_DETAIL_CONFIG_STATUS = 3U,
    FOC_FEEDBACK_DETAIL_BEGIN = 10U,
    FOC_FEEDBACK_DETAIL_EVIDENCE = 20U,
    FOC_FEEDBACK_DETAIL_APPROVE = 30U,
    FOC_FEEDBACK_DETAIL_CONFIG_BEGIN = 31U,
    FOC_FEEDBACK_DETAIL_CONFIG_STAGE = 32U,
    FOC_FEEDBACK_DETAIL_CONFIG_VALIDATE = 33U,
    FOC_FEEDBACK_DETAIL_CONFIG_ROLLBACK = 34U,
    FOC_FEEDBACK_DETAIL_FINISH = 40U,
    FOC_FEEDBACK_DETAIL_CANCEL = 50U,
    FOC_FEEDBACK_DETAIL_TIMEOUT = 60U,
    FOC_FEEDBACK_DETAIL_RECOVER = 70U,
};

static uint32_t foc_feedback_management_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_feedback_management_is_valid(
    const foc_feedback_management_t *management)
{
    return ((management != 0) &&
            (management->struct_size == sizeof(*management)) &&
            (management->version == FOC_FEEDBACK_MANAGEMENT_VERSION) &&
            (management->initialized != 0U) &&
            (management->feedback_storage != 0) &&
            (management->config_storage != 0) &&
            (management->ops.struct_size == sizeof(management->ops)) &&
            (management->ops.version == FOC_FEEDBACK_MANAGEMENT_VERSION) &&
            (management->ops.force_safe != 0)) ? 1U : 0U;
}

static void foc_feedback_management_force_safe(
    foc_feedback_management_t *management)
{
    if ((management != 0) && (management->ops.force_safe != 0))
    {
        management->ops.force_safe(management->ops.context);
    }
}

static foc_feedback_management_result_t foc_feedback_management_record(
    foc_feedback_management_t *management,
    foc_feedback_management_result_t result,
    uint32_t detail)
{
    if (management != 0)
    {
        management->last_result = result;
        management->last_detail = detail;
    }
    return result;
}

static foc_feedback_management_result_t foc_feedback_management_fail(
    foc_feedback_management_t *management,
    foc_feedback_management_result_t result,
    uint32_t detail)
{
    foc_feedback_management_force_safe(management);
    if (management != 0)
    {
        if (management->state != FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
        {
            ++management->failure_count;
        }
        management->state = FOC_FEEDBACK_MANAGEMENT_STATE_FAILED;
        management->shutdown_required = 1U;
    }
    return foc_feedback_management_record(management, result, detail);
}

static void foc_feedback_management_clear_session(
    foc_feedback_management_t *management)
{
    management->owner_id = 0U;
    management->feedback_token = 0U;
    management->config_token = 0U;
    management->started_at_ms = 0U;
    management->last_activity_ms = 0U;
    management->timeout_ms = 0U;
    management->expected_config_revision = 0U;
}

static uint32_t foc_feedback_management_session_active(
    const foc_feedback_management_t *management)
{
    return ((management->state == FOC_FEEDBACK_MANAGEMENT_STATE_COLLECTING) ||
            (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW) ||
            (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_STAGED)) ? 1U : 0U;
}

static uint32_t foc_feedback_management_expired(
    const foc_feedback_management_t *management,
    uint32_t now_ms)
{
    return ((management->timeout_ms != 0U) &&
            ((uint32_t)(now_ms - management->last_activity_ms) >=
             management->timeout_ms)) ? 1U : 0U;
}

static foc_feedback_management_result_t foc_feedback_management_check_owner(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms)
{
    if (foc_feedback_management_is_valid(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED, management->last_detail);
    }
    if (management->enabled == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_DISABLED, 0U);
    }
    if (foc_feedback_management_session_active(management) == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY, 0U);
    }
    if ((owner_id == 0U) || (owner_id != management->owner_id))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_OWNER, 0U);
    }
    if (foc_feedback_management_expired(management, now_ms) != 0U)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_EXPIRED,
            FOC_FEEDBACK_DETAIL_TIMEOUT);
    }
    return FOC_FEEDBACK_MANAGEMENT_OK;
}

static foc_feedback_management_result_t foc_feedback_management_refresh_state(
    foc_feedback_management_t *management)
{
    foc_feedback_status_snapshot_t feedback;
    foc_feedback_status_t feedback_result;

    memset(&feedback, 0, sizeof(feedback));
    feedback_result = foc_rust_feedback_get_status(
        management->feedback_storage, &feedback);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_FEEDBACK_STATUS << 16) | feedback_result);
    }
    if (feedback.calibration_state == FOC_FEEDBACK_CALIBRATION_COLLECTING)
    {
        management->state = FOC_FEEDBACK_MANAGEMENT_STATE_COLLECTING;
    }
    else if (feedback.calibration_state ==
             FOC_FEEDBACK_CALIBRATION_READY_FOR_REVIEW)
    {
        management->state = FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW;
    }
    else if (feedback.calibration_state == FOC_FEEDBACK_CALIBRATION_FAILED)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_EVIDENCE << 16) | feedback.last_result);
    }
    return FOC_FEEDBACK_MANAGEMENT_OK;
}

static foc_feedback_management_result_t foc_feedback_management_rollback_config(
    foc_feedback_management_t *management,
    const foc_config_apply_guard_t *guard)
{
    foc_config_transaction_status_t status;
    foc_config_status_t result;

    if (management->config_token == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_OK;
    }
    memset(&status, 0, sizeof(status));
    result = foc_rust_config_get_status(management->config_storage, &status);
    if (result != FOC_CONFIG_STATUS_OK)
    {
        return FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR;
    }
    if (status.state == FOC_CONFIG_TRANSACTION_APPLY_PREPARED)
    {
        result = foc_rust_config_cancel_apply(
            management->config_storage, management->config_token);
        if (result != FOC_CONFIG_STATUS_OK)
        {
            return FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR;
        }
    }
    if ((status.state == FOC_CONFIG_TRANSACTION_EDITING) ||
        (status.state == FOC_CONFIG_TRANSACTION_VALIDATED) ||
        (status.state == FOC_CONFIG_TRANSACTION_APPLY_PREPARED) ||
        (status.state == FOC_CONFIG_TRANSACTION_VOLATILE_APPLIED))
    {
        result = foc_rust_config_rollback(
            management->config_storage, management->config_token, guard);
        return (result == FOC_CONFIG_STATUS_OK) ?
            FOC_FEEDBACK_MANAGEMENT_OK : FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR;
    }
    return FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR;
}

static foc_feedback_management_result_t foc_feedback_management_cancel_internal(
    foc_feedback_management_t *management,
    const foc_config_apply_guard_t *guard,
    uint32_t timeout)
{
    foc_feedback_status_t feedback_result;

    foc_feedback_management_force_safe(management);
    if ((management->config_token != 0U) &&
        (foc_feedback_management_rollback_config(management, guard) !=
         FOC_FEEDBACK_MANAGEMENT_OK))
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            FOC_FEEDBACK_DETAIL_CONFIG_ROLLBACK);
    }
    feedback_result = foc_rust_feedback_calibration_cancel(
        management->feedback_storage, management->feedback_token);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_CANCEL << 16) | feedback_result);
    }
    ++management->cancel_count;
    foc_feedback_management_clear_session(management);
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_IDLE;
    if (timeout != 0U)
    {
        ++management->timeout_count;
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_EXPIRED,
            FOC_FEEDBACK_DETAIL_TIMEOUT);
    }
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_init(
    foc_feedback_management_t *management,
    foc_feedback_context_storage_t *feedback_storage,
    foc_config_context_storage_t *config_storage,
    const foc_feedback_management_ops_t *ops)
{
    foc_feedback_status_snapshot_t feedback;
    foc_config_transaction_status_t config;

    if ((management == 0) || (feedback_storage == 0) ||
        (config_storage == 0) || (ops == 0) ||
        (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_FEEDBACK_MANAGEMENT_VERSION) ||
        (ops->force_safe == 0))
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    memset(management, 0, sizeof(*management));
    management->struct_size = sizeof(*management);
    management->version = FOC_FEEDBACK_MANAGEMENT_VERSION;
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_DISABLED;
    management->feedback_storage = feedback_storage;
    management->config_storage = config_storage;
    management->ops = *ops;
    foc_feedback_management_force_safe(management);
    if ((foc_rust_feedback_abi_version() != FOC_FEEDBACK_ABI_VERSION) ||
        (foc_rust_feedback_context_required_size() >
         FOC_FEEDBACK_CONTEXT_CAPACITY) ||
        (foc_rust_feedback_context_required_align() >
         sizeof(uint64_t)) ||
        (foc_rust_config_abi_version() != FOC_CONFIG_ABI_VERSION) ||
        (foc_rust_config_context_required_size() > FOC_CONFIG_CONTEXT_CAPACITY) ||
        (foc_rust_config_context_required_align() > sizeof(uint64_t)))
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY,
            FOC_FEEDBACK_DETAIL_ABI);
    }
    memset(&feedback, 0, sizeof(feedback));
    memset(&config, 0, sizeof(config));
    if ((foc_rust_feedback_get_status(feedback_storage, &feedback) !=
         FOC_FEEDBACK_STATUS_OK) ||
        (foc_rust_config_get_status(config_storage, &config) !=
         FOC_CONFIG_STATUS_OK))
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY,
            FOC_FEEDBACK_DETAIL_FEEDBACK_STATUS);
    }
    management->initialized = 1U;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_set_enabled(
    foc_feedback_management_t *management,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard)
{
    foc_feedback_status_snapshot_t feedback;
    foc_config_transaction_status_t config;

    if ((foc_feedback_management_is_valid(management) == 0U) ||
        (enabled > 1U))
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    foc_feedback_management_force_safe(management);
    if (enabled == 0U)
    {
        if (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
        {
            management->enabled = 0U;
            return foc_feedback_management_record(
                management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED,
                management->last_detail);
        }
        if (foc_feedback_management_session_active(management) != 0U)
        {
            foc_feedback_management_result_t result =
                foc_feedback_management_cancel_internal(management, guard, 0U);
            if (result != FOC_FEEDBACK_MANAGEMENT_OK)
            {
                management->enabled = 0U;
                return result;
            }
        }
        management->enabled = 0U;
        management->state = FOC_FEEDBACK_MANAGEMENT_STATE_DISABLED;
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
    }
    if ((management->shutdown_required != 0U) ||
        (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE, 0U);
    }
    memset(&feedback, 0, sizeof(feedback));
    memset(&config, 0, sizeof(config));
    if ((foc_rust_feedback_get_status(management->feedback_storage, &feedback) !=
         FOC_FEEDBACK_STATUS_OK) ||
        (foc_rust_config_get_status(management->config_storage, &config) !=
         FOC_CONFIG_STATUS_OK) ||
        (feedback.calibration_state != FOC_FEEDBACK_CALIBRATION_IDLE) ||
        (config.state != FOC_CONFIG_TRANSACTION_IDLE))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY,
            FOC_FEEDBACK_DETAIL_CONFIG_STATUS);
    }
    management->enabled = 1U;
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_IDLE;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_get_status(
    foc_feedback_management_t *management,
    foc_feedback_management_status_t *status)
{
    foc_feedback_status_t feedback_result;
    foc_config_status_t config_result;

    if ((foc_feedback_management_is_valid(management) == 0U) || (status == 0))
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_FEEDBACK_MANAGEMENT_VERSION;
    status->enabled = management->enabled;
    status->state = management->state;
    status->owner_id = management->owner_id;
    status->feedback_token = management->feedback_token;
    status->config_token = management->config_token;
    status->started_at_ms = management->started_at_ms;
    status->last_activity_ms = management->last_activity_ms;
    status->timeout_ms = management->timeout_ms;
    status->expected_config_revision = management->expected_config_revision;
    status->shutdown_required = management->shutdown_required;
    status->begin_count = management->begin_count;
    status->cancel_count = management->cancel_count;
    status->timeout_count = management->timeout_count;
    status->failure_count = management->failure_count;
    status->last_result = management->last_result;
    status->last_detail = management->last_detail;
    feedback_result = foc_rust_feedback_get_status(
        management->feedback_storage, &status->feedback);
    config_result = foc_rust_config_get_status(
        management->config_storage, &status->config);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_FEEDBACK_STATUS << 16) | feedback_result);
    }
    if (config_result != FOC_CONFIG_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            (FOC_FEEDBACK_DETAIL_CONFIG_STATUS << 16) | config_result);
    }
    return FOC_FEEDBACK_MANAGEMENT_OK;
}

foc_feedback_management_result_t foc_feedback_management_begin(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    foc_feedback_mode_t feedback_mode,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const foc_config_apply_guard_t *guard)
{
    foc_feedback_status_t result;
    uint32_t token = 0U;

    if (foc_feedback_management_is_valid(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->enabled == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_DISABLED, 0U);
    }
    if (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (management->state != FOC_FEEDBACK_MANAGEMENT_STATE_IDLE)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_BUSY, 0U);
    }
    if ((owner_id == 0U) ||
        ((feedback_mode != FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER) &&
         (feedback_mode != FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER) &&
         (feedback_mode != FOC_FEEDBACK_MODE_HALL)) ||
        (timeout_ms < FOC_FEEDBACK_MANAGEMENT_MIN_TIMEOUT_MS) ||
        (timeout_ms > FOC_FEEDBACK_MANAGEMENT_MAX_TIMEOUT_MS))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT, 0U);
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE, 0U);
    }
    foc_feedback_management_force_safe(management);
    result = foc_rust_feedback_calibration_begin(
        management->feedback_storage, feedback_mode, guard, &token);
    if ((result != FOC_FEEDBACK_STATUS_OK) || (token == 0U))
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_BEGIN << 16) | result);
    }
    management->owner_id = owner_id;
    management->feedback_token = token;
    management->config_token = 0U;
    management->started_at_ms = now_ms;
    management->last_activity_ms = now_ms;
    management->timeout_ms = timeout_ms;
    management->expected_config_revision = 0U;
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_COLLECTING;
    ++management->begin_count;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_submit_evidence(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_feedback_management_evidence_t *evidence)
{
    foc_feedback_management_result_t owner_result;
    foc_feedback_status_t result = FOC_FEEDBACK_STATUS_OK;

    owner_result = foc_feedback_management_check_owner(
        management, owner_id, now_ms);
    if (owner_result != FOC_FEEDBACK_MANAGEMENT_OK)
    {
        return owner_result;
    }
    if ((management->state != FOC_FEEDBACK_MANAGEMENT_STATE_COLLECTING) &&
        (management->state != FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY, 0U);
    }
    if ((evidence == 0) ||
        (evidence->struct_size != sizeof(*evidence)) ||
        (evidence->version != FOC_FEEDBACK_MANAGEMENT_VERSION) ||
        (evidence->evidence_flags == 0U) ||
        ((evidence->evidence_flags &
          ~(uint32_t)FOC_FEEDBACK_EVIDENCE_KNOWN_MASK) != 0U))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT, 0U);
    }
    if ((evidence->evidence_flags & FOC_FEEDBACK_EVIDENCE_DIRECTION) != 0U)
    {
        result = foc_rust_feedback_calibration_record_direction(
            management->feedback_storage, management->feedback_token,
            evidence->direction, evidence->direction_samples);
    }
    if ((result == FOC_FEEDBACK_STATUS_OK) &&
        ((evidence->evidence_flags & FOC_FEEDBACK_EVIDENCE_ENCODER_INDEX) != 0U))
    {
        result = foc_rust_feedback_calibration_record_encoder_index(
            management->feedback_storage, management->feedback_token,
            evidence->encoder_index_samples);
    }
    if ((result == FOC_FEEDBACK_STATUS_OK) &&
        ((evidence->evidence_flags & FOC_FEEDBACK_EVIDENCE_ENCODER_OFFSET) != 0U))
    {
        result = foc_rust_feedback_calibration_record_encoder_offset(
            management->feedback_storage, management->feedback_token,
            evidence->encoder_offset_rad, evidence->encoder_offset_samples);
    }
    if ((result == FOC_FEEDBACK_STATUS_OK) &&
        ((evidence->evidence_flags & FOC_FEEDBACK_EVIDENCE_HALL_SEQUENCE) != 0U))
    {
        result = foc_rust_feedback_calibration_record_hall_sequence(
            management->feedback_storage, management->feedback_token,
            evidence->hall_sequence_packed, evidence->hall_sequence_samples);
    }
    if (result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_EVIDENCE << 16) | result);
    }
    management->last_activity_ms = now_ms;
    return foc_feedback_management_refresh_state(management);
}

foc_feedback_management_result_t foc_feedback_management_approve_and_stage(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard)
{
    foc_feedback_management_result_t owner_result;
    foc_feedback_calibration_update_t update;
    foc_config_transaction_status_t config;
    foc_feedback_status_t feedback_result;
    foc_config_status_t config_result;
    uint32_t config_token = 0U;

    owner_result = foc_feedback_management_check_owner(
        management, owner_id, now_ms);
    if (owner_result != FOC_FEEDBACK_MANAGEMENT_OK)
    {
        return owner_result;
    }
    if ((management->state != FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW) ||
        (foc_feedback_management_guard_is_safe(guard) == 0U))
    {
        return foc_feedback_management_record(
            management,
            (foc_feedback_management_guard_is_safe(guard) != 0U) ?
                FOC_FEEDBACK_MANAGEMENT_NOT_READY :
                FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE,
            0U);
    }
    foc_feedback_management_force_safe(management);
    memset(&update, 0, sizeof(update));
    feedback_result = foc_rust_feedback_calibration_approve(
        management->feedback_storage, management->feedback_token,
        guard, &update);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_APPROVE << 16) | feedback_result);
    }
    config_result = foc_rust_config_begin(
        management->config_storage, &config_token);
    if ((config_result != FOC_CONFIG_STATUS_OK) || (config_token == 0U))
    {
        (void)foc_rust_feedback_calibration_cancel(
            management->feedback_storage, management->feedback_token);
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            (FOC_FEEDBACK_DETAIL_CONFIG_BEGIN << 16) | config_result);
    }
    management->config_token = config_token;
    feedback_result = foc_rust_feedback_stage_config_update(
        management->feedback_storage, management->feedback_token,
        management->config_storage, config_token);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        (void)foc_rust_config_rollback(
            management->config_storage, config_token, guard);
        (void)foc_rust_feedback_calibration_cancel(
            management->feedback_storage, management->feedback_token);
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_CONFIG_STAGE << 16) | feedback_result);
    }
    config_result = foc_rust_config_validate(
        management->config_storage, config_token);
    if (config_result != FOC_CONFIG_STATUS_OK)
    {
        (void)foc_rust_config_rollback(
            management->config_storage, config_token, guard);
        (void)foc_rust_feedback_calibration_cancel(
            management->feedback_storage, management->feedback_token);
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            (FOC_FEEDBACK_DETAIL_CONFIG_VALIDATE << 16) | config_result);
    }
    memset(&config, 0, sizeof(config));
    config_result = foc_rust_config_get_status(
        management->config_storage, &config);
    if ((config_result != FOC_CONFIG_STATUS_OK) ||
        (config.state != FOC_CONFIG_TRANSACTION_VALIDATED) ||
        (config.token != config_token) ||
        (config.pending_revision == 0U))
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            FOC_FEEDBACK_DETAIL_CONFIG_STATUS);
    }
    management->expected_config_revision = config.pending_revision;
    management->last_activity_ms = now_ms;
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_STAGED;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_finish_committed(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard)
{
    foc_feedback_management_result_t owner_result;
    foc_config_transaction_status_t config;
    foc_config_status_t config_result;
    foc_feedback_status_t feedback_result;

    owner_result = foc_feedback_management_check_owner(
        management, owner_id, now_ms);
    if (owner_result != FOC_FEEDBACK_MANAGEMENT_OK)
    {
        return owner_result;
    }
    if (management->state != FOC_FEEDBACK_MANAGEMENT_STATE_STAGED)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY, 0U);
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE,
            FOC_FEEDBACK_DETAIL_FINISH);
    }
    foc_feedback_management_force_safe(management);
    memset(&config, 0, sizeof(config));
    config_result = foc_rust_config_get_status(
        management->config_storage, &config);
    if ((config_result != FOC_CONFIG_STATUS_OK) ||
        (config.state != FOC_CONFIG_TRANSACTION_IDLE) ||
        (config.active_revision != management->expected_config_revision))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_READY,
            FOC_FEEDBACK_DETAIL_CONFIG_STATUS);
    }
    feedback_result = foc_rust_feedback_calibration_finish_applied(
        management->feedback_storage, management->feedback_token, guard);
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_FINISH << 16) | feedback_result);
    }
    foc_feedback_management_clear_session(management);
    management->state = FOC_FEEDBACK_MANAGEMENT_STATE_IDLE;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}

foc_feedback_management_result_t foc_feedback_management_cancel(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    const foc_config_apply_guard_t *guard)
{
    if (foc_feedback_management_is_valid(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (foc_feedback_management_session_active(management) == 0U)
    {
        foc_feedback_management_force_safe(management);
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_ALREADY_SAFE, 0U);
    }
    if ((owner_id == 0U) || (owner_id != management->owner_id))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_NOT_OWNER, 0U);
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE,
            FOC_FEEDBACK_DETAIL_CANCEL);
    }
    return foc_feedback_management_cancel_internal(management, guard, 0U);
}

foc_feedback_management_result_t foc_feedback_management_poll(
    foc_feedback_management_t *management,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard)
{
    if (foc_feedback_management_is_valid(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
    {
        foc_feedback_management_force_safe(management);
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED,
            management->last_detail);
    }
    if (foc_feedback_management_session_active(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_ALREADY_SAFE;
    }
    if (foc_feedback_management_expired(management, now_ms) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_OK;
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_fail(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE,
            FOC_FEEDBACK_DETAIL_TIMEOUT);
    }
    return foc_feedback_management_cancel_internal(management, guard, 1U);
}

foc_feedback_management_result_t foc_feedback_management_recover(
    foc_feedback_management_t *management,
    const foc_config_apply_guard_t *guard)
{
    foc_feedback_status_snapshot_t feedback;
    foc_config_transaction_status_t config;
    foc_feedback_status_t feedback_result = FOC_FEEDBACK_STATUS_OK;
    foc_config_status_t config_result;

    if (foc_feedback_management_is_valid(management) == 0U)
    {
        return FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT;
    }
    if (management->state != FOC_FEEDBACK_MANAGEMENT_STATE_FAILED)
    {
        return FOC_FEEDBACK_MANAGEMENT_ALREADY_SAFE;
    }
    if (foc_feedback_management_guard_is_safe(guard) == 0U)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE,
            FOC_FEEDBACK_DETAIL_RECOVER);
    }
    foc_feedback_management_force_safe(management);
    memset(&config, 0, sizeof(config));
    config_result = foc_rust_config_get_status(
        management->config_storage, &config);
    if (config_result != FOC_CONFIG_STATUS_OK)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            FOC_FEEDBACK_DETAIL_RECOVER);
    }
    if ((config.state != FOC_CONFIG_TRANSACTION_IDLE) &&
        (management->config_token != 0U))
    {
        if (foc_feedback_management_rollback_config(management, guard) !=
            FOC_FEEDBACK_MANAGEMENT_OK)
        {
            return foc_feedback_management_record(
                management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
                FOC_FEEDBACK_DETAIL_RECOVER);
        }
        memset(&config, 0, sizeof(config));
        config_result = foc_rust_config_get_status(
            management->config_storage, &config);
    }
    if ((config_result != FOC_CONFIG_STATUS_OK) ||
        (config.state != FOC_CONFIG_TRANSACTION_IDLE))
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR,
            FOC_FEEDBACK_DETAIL_RECOVER);
    }
    memset(&feedback, 0, sizeof(feedback));
    if (foc_rust_feedback_get_status(
            management->feedback_storage, &feedback) !=
        FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            FOC_FEEDBACK_DETAIL_RECOVER);
    }
    if (feedback.calibration_state == FOC_FEEDBACK_CALIBRATION_FAILED)
    {
        feedback_result = foc_rust_feedback_calibration_reset_failed(
            management->feedback_storage, guard);
    }
    else if (feedback.calibration_state != FOC_FEEDBACK_CALIBRATION_IDLE)
    {
        feedback_result = foc_rust_feedback_calibration_cancel(
            management->feedback_storage, management->feedback_token);
    }
    if (feedback_result != FOC_FEEDBACK_STATUS_OK)
    {
        return foc_feedback_management_record(
            management, FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR,
            (FOC_FEEDBACK_DETAIL_RECOVER << 16) | feedback_result);
    }
    foc_feedback_management_clear_session(management);
    management->shutdown_required = 0U;
    management->state = (management->enabled != 0U) ?
        FOC_FEEDBACK_MANAGEMENT_STATE_IDLE :
        FOC_FEEDBACK_MANAGEMENT_STATE_DISABLED;
    return foc_feedback_management_record(
        management, FOC_FEEDBACK_MANAGEMENT_OK, 0U);
}
