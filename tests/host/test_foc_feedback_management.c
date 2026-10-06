#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_feedback_management.h"

typedef struct
{
    uint32_t safe_count;
    foc_feedback_calibration_state_t feedback_state;
    foc_feedback_mode_t feedback_mode;
    uint32_t feedback_token;
    uint32_t completed_steps;
    foc_config_transaction_state_t config_state;
    uint32_t config_token;
    uint32_t active_revision;
    uint32_t pending_revision;
    uint32_t fail_stage;
} fake_state_t;

static fake_state_t g_fake;

static void fake_force_safe(void *context)
{
    fake_state_t *state = (fake_state_t *)context;
    ++state->safe_count;
}

static void fake_reset(void)
{
    memset(&g_fake, 0, sizeof(g_fake));
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_IDLE;
    g_fake.config_state = FOC_CONFIG_TRANSACTION_IDLE;
    g_fake.active_revision = 1U;
}

uint32_t foc_rust_feedback_abi_version(void)
{
    return FOC_FEEDBACK_ABI_VERSION;
}

uint32_t foc_rust_feedback_context_required_size(void)
{
    return 128U;
}

uint32_t foc_rust_feedback_context_required_align(void)
{
    return 8U;
}

uint32_t foc_rust_config_abi_version(void)
{
    return FOC_CONFIG_ABI_VERSION;
}

uint32_t foc_rust_config_context_required_size(void)
{
    return 512U;
}

uint32_t foc_rust_config_context_required_align(void)
{
    return 8U;
}

foc_feedback_status_t foc_rust_feedback_get_status(
    foc_feedback_context_storage_t *storage,
    foc_feedback_status_snapshot_t *output)
{
    (void)storage;
    if (output == 0)
    {
        return FOC_FEEDBACK_STATUS_INVALID_ARGUMENT;
    }
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_FEEDBACK_ABI_VERSION;
    output->calibration_state = g_fake.feedback_state;
    output->calibration_token = g_fake.feedback_token;
    output->completed_steps = g_fake.completed_steps;
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_begin(
    foc_feedback_context_storage_t *storage,
    foc_feedback_mode_t feedback_mode,
    const foc_config_apply_guard_t *guard,
    uint32_t *token_out)
{
    (void)storage;
    (void)guard;
    if ((g_fake.feedback_state != FOC_FEEDBACK_CALIBRATION_IDLE) ||
        (token_out == 0))
    {
        return FOC_FEEDBACK_STATUS_BUSY;
    }
    g_fake.feedback_token = 41U;
    g_fake.feedback_mode = feedback_mode;
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_COLLECTING;
    g_fake.completed_steps = 0U;
    *token_out = g_fake.feedback_token;
    return FOC_FEEDBACK_STATUS_OK;
}

static void fake_update_ready(void)
{
    uint32_t expected = FOC_FEEDBACK_CALIBRATION_STEP_DIRECTION;
    if (g_fake.feedback_mode == FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER)
    {
        expected |= FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX |
                    FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET;
    }
    else if (g_fake.feedback_mode == FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER)
    {
        expected |= FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET;
    }
    else
    {
        expected |= FOC_FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE;
    }
    if ((g_fake.completed_steps & expected) == expected)
    {
        g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_READY_FOR_REVIEW;
    }
}

foc_feedback_status_t foc_rust_feedback_calibration_record_direction(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    int32_t direction,
    uint32_t sample_count)
{
    (void)storage;
    if ((token != g_fake.feedback_token) || (sample_count == 0U))
    {
        return FOC_FEEDBACK_STATUS_STALE_TOKEN;
    }
    if ((direction != -1) && (direction != 1))
    {
        g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_FAILED;
        return FOC_FEEDBACK_STATUS_INVALID_EVIDENCE;
    }
    g_fake.completed_steps |= FOC_FEEDBACK_CALIBRATION_STEP_DIRECTION;
    fake_update_ready();
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_record_encoder_index(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    uint32_t sample_count)
{
    (void)storage;
    if ((token != g_fake.feedback_token) || (sample_count == 0U))
    {
        return FOC_FEEDBACK_STATUS_INVALID_EVIDENCE;
    }
    g_fake.completed_steps |= FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX;
    fake_update_ready();
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_record_encoder_offset(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    float offset_rad,
    uint32_t sample_count)
{
    (void)storage;
    if ((token != g_fake.feedback_token) || (sample_count == 0U) ||
        (!isfinite(offset_rad)))
    {
        return FOC_FEEDBACK_STATUS_INVALID_EVIDENCE;
    }
    g_fake.completed_steps |= FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET;
    fake_update_ready();
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_record_hall_sequence(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    uint32_t sequence_packed,
    uint32_t sample_count)
{
    (void)storage;
    (void)sequence_packed;
    if ((token != g_fake.feedback_token) || (sample_count == 0U))
    {
        return FOC_FEEDBACK_STATUS_INVALID_EVIDENCE;
    }
    g_fake.completed_steps |= FOC_FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE;
    fake_update_ready();
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_approve(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    const foc_config_apply_guard_t *guard,
    foc_feedback_calibration_update_t *output)
{
    (void)storage;
    (void)guard;
    if ((token != g_fake.feedback_token) ||
        (g_fake.feedback_state != FOC_FEEDBACK_CALIBRATION_READY_FOR_REVIEW) ||
        (output == 0))
    {
        return FOC_FEEDBACK_STATUS_INCOMPLETE_EVIDENCE;
    }
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_FEEDBACK_ABI_VERSION;
    output->feedback_mode = g_fake.feedback_mode;
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_APPROVED;
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_stage_config_update(
    foc_feedback_context_storage_t *storage,
    uint32_t calibration_token,
    foc_config_context_storage_t *config_storage,
    uint32_t config_token)
{
    (void)storage;
    (void)config_storage;
    if (g_fake.fail_stage != 0U)
    {
        return FOC_FEEDBACK_STATUS_CONFIG_TRANSACTION_FAILED;
    }
    return ((calibration_token == g_fake.feedback_token) &&
            (config_token == g_fake.config_token) &&
            (g_fake.feedback_state == FOC_FEEDBACK_CALIBRATION_APPROVED)) ?
        FOC_FEEDBACK_STATUS_OK : FOC_FEEDBACK_STATUS_STALE_TOKEN;
}

foc_feedback_status_t foc_rust_feedback_calibration_cancel(
    foc_feedback_context_storage_t *storage,
    uint32_t token)
{
    (void)storage;
    if ((token == 0U) || (token != g_fake.feedback_token))
    {
        return FOC_FEEDBACK_STATUS_STALE_TOKEN;
    }
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_IDLE;
    g_fake.feedback_token = 0U;
    g_fake.completed_steps = 0U;
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_reset_failed(
    foc_feedback_context_storage_t *storage,
    const foc_config_apply_guard_t *guard)
{
    (void)storage;
    (void)guard;
    if (g_fake.feedback_state != FOC_FEEDBACK_CALIBRATION_FAILED)
    {
        return FOC_FEEDBACK_STATUS_INVALID_STATE;
    }
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_IDLE;
    g_fake.feedback_token = 0U;
    g_fake.completed_steps = 0U;
    return FOC_FEEDBACK_STATUS_OK;
}

foc_feedback_status_t foc_rust_feedback_calibration_finish_applied(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    const foc_config_apply_guard_t *guard)
{
    (void)storage;
    (void)guard;
    if ((token != g_fake.feedback_token) ||
        (g_fake.feedback_state != FOC_FEEDBACK_CALIBRATION_APPROVED))
    {
        return FOC_FEEDBACK_STATUS_INVALID_STATE;
    }
    g_fake.feedback_state = FOC_FEEDBACK_CALIBRATION_IDLE;
    g_fake.feedback_token = 0U;
    g_fake.completed_steps = 0U;
    return FOC_FEEDBACK_STATUS_OK;
}

foc_config_status_t foc_rust_config_get_status(
    foc_config_context_storage_t *storage,
    foc_config_transaction_status_t *output)
{
    (void)storage;
    if (output == 0)
    {
        return FOC_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_CONFIG_ABI_VERSION;
    output->state = g_fake.config_state;
    output->token = g_fake.config_token;
    output->active_revision = g_fake.active_revision;
    output->pending_revision = g_fake.pending_revision;
    return FOC_CONFIG_STATUS_OK;
}

foc_config_status_t foc_rust_config_begin(
    foc_config_context_storage_t *storage,
    uint32_t *token_out)
{
    (void)storage;
    if ((g_fake.config_state != FOC_CONFIG_TRANSACTION_IDLE) ||
        (token_out == 0))
    {
        return FOC_CONFIG_STATUS_BUSY;
    }
    g_fake.config_token = 73U;
    g_fake.config_state = FOC_CONFIG_TRANSACTION_EDITING;
    g_fake.pending_revision = g_fake.active_revision + 1U;
    *token_out = g_fake.config_token;
    return FOC_CONFIG_STATUS_OK;
}

foc_config_status_t foc_rust_config_validate(
    foc_config_context_storage_t *storage,
    uint32_t token)
{
    (void)storage;
    if ((token != g_fake.config_token) ||
        (g_fake.config_state != FOC_CONFIG_TRANSACTION_EDITING))
    {
        return FOC_CONFIG_STATUS_INVALID_STATE;
    }
    g_fake.config_state = FOC_CONFIG_TRANSACTION_VALIDATED;
    return FOC_CONFIG_STATUS_OK;
}

foc_config_status_t foc_rust_config_cancel_apply(
    foc_config_context_storage_t *storage,
    uint32_t token)
{
    (void)storage;
    if ((token != g_fake.config_token) ||
        (g_fake.config_state != FOC_CONFIG_TRANSACTION_APPLY_PREPARED))
    {
        return FOC_CONFIG_STATUS_INVALID_STATE;
    }
    g_fake.config_state = FOC_CONFIG_TRANSACTION_VALIDATED;
    return FOC_CONFIG_STATUS_OK;
}

foc_config_status_t foc_rust_config_rollback(
    foc_config_context_storage_t *storage,
    uint32_t token,
    const foc_config_apply_guard_t *guard)
{
    (void)storage;
    (void)guard;
    if ((token != g_fake.config_token) ||
        ((g_fake.config_state != FOC_CONFIG_TRANSACTION_EDITING) &&
         (g_fake.config_state != FOC_CONFIG_TRANSACTION_VALIDATED) &&
         (g_fake.config_state != FOC_CONFIG_TRANSACTION_VOLATILE_APPLIED)))
    {
        return FOC_CONFIG_STATUS_INVALID_STATE;
    }
    g_fake.config_state = FOC_CONFIG_TRANSACTION_IDLE;
    g_fake.config_token = 0U;
    g_fake.pending_revision = 0U;
    return FOC_CONFIG_STATUS_OK;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t guard;
    memset(&guard, 0, sizeof(guard));
    guard.struct_size = sizeof(guard);
    guard.abi_version = FOC_CONFIG_ABI_VERSION;
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    return guard;
}

static foc_feedback_management_evidence_t encoder_evidence(void)
{
    foc_feedback_management_evidence_t evidence;
    memset(&evidence, 0, sizeof(evidence));
    evidence.struct_size = sizeof(evidence);
    evidence.version = FOC_FEEDBACK_MANAGEMENT_VERSION;
    evidence.evidence_flags =
        FOC_FEEDBACK_EVIDENCE_DIRECTION |
        FOC_FEEDBACK_EVIDENCE_ENCODER_INDEX |
        FOC_FEEDBACK_EVIDENCE_ENCODER_OFFSET;
    evidence.direction = 1;
    evidence.direction_samples = 4U;
    evidence.encoder_index_samples = 4U;
    evidence.encoder_offset_rad = 0.2f;
    evidence.encoder_offset_samples = 4U;
    return evidence;
}

static foc_feedback_management_evidence_t absolute_encoder_evidence(void)
{
    foc_feedback_management_evidence_t evidence = encoder_evidence();
    evidence.evidence_flags &=
        ~(uint32_t)FOC_FEEDBACK_EVIDENCE_ENCODER_INDEX;
    evidence.encoder_index_samples = 0U;
    return evidence;
}

static void setup(
    foc_feedback_management_t *management,
    foc_feedback_context_storage_t *feedback,
    foc_config_context_storage_t *config)
{
    foc_feedback_management_ops_t ops;
    fake_reset();
    memset(feedback, 0, sizeof(*feedback));
    memset(config, 0, sizeof(*config));
    memset(&ops, 0, sizeof(ops));
    ops.struct_size = sizeof(ops);
    ops.version = FOC_FEEDBACK_MANAGEMENT_VERSION;
    ops.force_safe = fake_force_safe;
    ops.context = &g_fake;
    assert(foc_feedback_management_init(
               management, feedback, config, &ops) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
}

static void enable(foc_feedback_management_t *management)
{
    foc_config_apply_guard_t guard = safe_guard();
    assert(foc_feedback_management_set_enabled(management, 1U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
}

static void begin_encoder(
    foc_feedback_management_t *management,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    foc_config_apply_guard_t guard = safe_guard();
    assert(foc_feedback_management_begin(
               management, 7U, FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER,
               now_ms, timeout_ms, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
}

static void make_ready(foc_feedback_management_t *management, uint32_t now_ms)
{
    foc_feedback_management_evidence_t evidence = encoder_evidence();
    assert(foc_feedback_management_submit_evidence(
               management, 7U, now_ms, &evidence) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management->state ==
           FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW);
}

static void stage(foc_feedback_management_t *management, uint32_t now_ms)
{
    foc_config_apply_guard_t guard = safe_guard();
    assert(foc_feedback_management_approve_and_stage(
               management, 7U, now_ms, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management->state == FOC_FEEDBACK_MANAGEMENT_STATE_STAGED);
    assert(g_fake.config_state == FOC_CONFIG_TRANSACTION_VALIDATED);
}

static void test_default_off_and_safe_enable(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_config_apply_guard_t guard = safe_guard();

    setup(&management, &feedback, &config);
    assert(management.enabled == 0U);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_DISABLED);
    assert(g_fake.safe_count == 1U);
    assert(foc_feedback_management_begin(
               &management, 7U, FOC_FEEDBACK_MODE_HALL,
               0U, 100U, &guard) == FOC_FEEDBACK_MANAGEMENT_DISABLED);
    guard.drive_active = 1U;
    assert(foc_feedback_management_set_enabled(&management, 1U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE);
    guard.drive_active = 0U;
    assert(foc_feedback_management_set_enabled(&management, 1U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
}

static void test_owner_and_committed_finish(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_feedback_management_evidence_t evidence = encoder_evidence();
    foc_config_apply_guard_t guard = safe_guard();

    setup(&management, &feedback, &config);
    enable(&management);
    begin_encoder(&management, 10U, 100U);
    assert(foc_feedback_management_begin(
               &management, 8U, FOC_FEEDBACK_MODE_HALL,
               11U, 100U, &guard) == FOC_FEEDBACK_MANAGEMENT_BUSY);
    assert(foc_feedback_management_submit_evidence(
               &management, 8U, 12U, &evidence) ==
           FOC_FEEDBACK_MANAGEMENT_NOT_OWNER);
    make_ready(&management, 13U);
    stage(&management, 14U);
    assert(foc_feedback_management_finish_committed(
               &management, 7U, 15U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_NOT_READY);
    g_fake.config_state = FOC_CONFIG_TRANSACTION_IDLE;
    g_fake.config_token = 0U;
    g_fake.active_revision = management.expected_config_revision;
    g_fake.pending_revision = 0U;
    assert(foc_feedback_management_finish_committed(
               &management, 7U, 16U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_IDLE);
    assert(management.owner_id == 0U);
    assert(g_fake.feedback_state == FOC_FEEDBACK_CALIBRATION_IDLE);
}

static void test_absolute_encoder_does_not_require_index_evidence(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_config_apply_guard_t guard = safe_guard();
    foc_feedback_management_evidence_t evidence = absolute_encoder_evidence();

    setup(&management, &feedback, &config);
    enable(&management);
    assert(foc_feedback_management_begin(
               &management, 7U, FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER,
               10U, 100U, &guard) == FOC_FEEDBACK_MANAGEMENT_OK);
    assert(foc_feedback_management_submit_evidence(
               &management, 7U, 11U, &evidence) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management.state ==
           FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW);
    assert((g_fake.completed_steps &
            FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX) == 0U);
}

static void test_cancel_rolls_back_staged_transaction(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_config_apply_guard_t guard = safe_guard();

    setup(&management, &feedback, &config);
    enable(&management);
    begin_encoder(&management, 100U, 100U);
    make_ready(&management, 101U);
    stage(&management, 102U);
    assert(foc_feedback_management_cancel(&management, 8U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_NOT_OWNER);
    assert(foc_feedback_management_cancel(&management, 7U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(g_fake.config_state == FOC_CONFIG_TRANSACTION_IDLE);
    assert(g_fake.feedback_state == FOC_FEEDBACK_CALIBRATION_IDLE);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_IDLE);
}

static void test_timeout_wrap_latches_safe_failure(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_config_apply_guard_t guard = safe_guard();
    uint32_t safe_before;

    setup(&management, &feedback, &config);
    enable(&management);
    begin_encoder(&management, UINT32_MAX - 5U, 10U);
    assert(foc_feedback_management_poll(&management, 3U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    safe_before = g_fake.safe_count;
    assert(foc_feedback_management_poll(&management, 4U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_EXPIRED);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);
    assert(management.timeout_count == 1U);
    assert(g_fake.safe_count > safe_before);
    assert(foc_feedback_management_recover(&management, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_IDLE);
    assert(management.shutdown_required == 0U);
}

static void test_bad_evidence_requires_recovery(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_feedback_management_evidence_t evidence = encoder_evidence();
    foc_config_apply_guard_t guard = safe_guard();

    setup(&management, &feedback, &config);
    enable(&management);
    begin_encoder(&management, 1U, 100U);
    evidence.direction = 0;
    assert(foc_feedback_management_submit_evidence(
               &management, 7U, 2U, &evidence) ==
           FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED);
    assert(foc_feedback_management_begin(
               &management, 7U, FOC_FEEDBACK_MODE_HALL,
               3U, 100U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED);
    assert(foc_feedback_management_recover(&management, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_OK);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_IDLE);
}

static void test_unrecoverable_commit_prepared_stays_locked(void)
{
    foc_feedback_management_t management;
    foc_feedback_context_storage_t feedback;
    foc_config_context_storage_t config;
    foc_config_apply_guard_t guard = safe_guard();

    setup(&management, &feedback, &config);
    enable(&management);
    begin_encoder(&management, 1U, 100U);
    make_ready(&management, 2U);
    stage(&management, 3U);
    g_fake.config_state = FOC_CONFIG_TRANSACTION_COMMIT_PREPARED;
    assert(foc_feedback_management_cancel(&management, 7U, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);
    assert(foc_feedback_management_recover(&management, &guard) ==
           FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR);
    assert(management.state == FOC_FEEDBACK_MANAGEMENT_STATE_FAILED);
}

int main(void)
{
    test_default_off_and_safe_enable();
    test_owner_and_committed_finish();
    test_absolute_encoder_does_not_require_index_evidence();
    test_cancel_rolls_back_staged_transaction();
    test_timeout_wrap_latches_safe_failure();
    test_bad_evidence_requires_recovery();
    test_unrecoverable_commit_prepared_stays_locked();
    puts("foc feedback management tests passed");
    return 0;
}
