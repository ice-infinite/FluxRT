#include "foc_motion_management.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
 * These fakes exercise the documented single-caller lifecycle only. They do
 * not claim safety for concurrent ISR/task access; target integration must
 * keep ownership and serialization outside this management object.
 */

static foc_motion_status_t g_init_result;
static foc_motion_status_t g_apply_result;
static foc_motion_status_t g_enable_result;
static foc_motion_status_t g_disable_result;
static foc_motion_status_t g_step_result;
static uint32_t g_init_calls;
static uint32_t g_apply_calls;
static uint32_t g_enable_calls;
static uint32_t g_disable_calls;
static uint32_t g_step_calls;
static uint32_t g_force_safe_calls;

static void fake_reset(void)
{
    g_init_result = FOC_MOTION_STATUS_OK;
    g_apply_result = FOC_MOTION_STATUS_OK;
    g_enable_result = FOC_MOTION_STATUS_OK;
    g_disable_result = FOC_MOTION_STATUS_OK;
    g_step_result = FOC_MOTION_STATUS_OK;
    g_init_calls = 0U;
    g_apply_calls = 0U;
    g_enable_calls = 0U;
    g_disable_calls = 0U;
    g_step_calls = 0U;
    g_force_safe_calls = 0U;
}

foc_motion_status_t foc_motion_adapter_init(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config)
{
    assert(adapter != NULL);
    assert(config != NULL);
    ++g_init_calls;
    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->initialized = 1U;
    adapter->configured = (g_init_result == FOC_MOTION_STATUS_OK) ? 1U : 0U;
    adapter->last_status = g_init_result;
    return g_init_result;
}

foc_motion_status_t foc_motion_adapter_apply_config(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard)
{
    assert(adapter != NULL);
    assert(config != NULL);
    assert(guard != NULL);
    ++g_apply_calls;
    adapter->last_status = g_apply_result;
    return g_apply_result;
}

foc_motion_status_t foc_motion_adapter_enable(
    foc_motion_adapter_t *adapter,
    const foc_config_apply_guard_t *guard)
{
    assert(adapter != NULL);
    assert(guard != NULL);
    ++g_enable_calls;
    if (g_enable_result == FOC_MOTION_STATUS_OK)
    {
        adapter->enabled = 1U;
    }
    adapter->last_status = g_enable_result;
    return g_enable_result;
}

foc_motion_status_t foc_motion_adapter_disable(foc_motion_adapter_t *adapter)
{
    assert(adapter != NULL);
    ++g_disable_calls;
    adapter->enabled = 0U;
    adapter->last_status = g_disable_result;
    return g_disable_result;
}

foc_motion_status_t foc_motion_adapter_step(
    foc_motion_adapter_t *adapter,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output)
{
    assert(adapter != NULL);
    assert(command != NULL);
    assert(feedback != NULL);
    assert(output != NULL);
    ++g_step_calls;
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_MOTION_OUTPUT_VERSION;
    output->config_revision = 19U;
    output->source_sequence = command->sequence;
    output->control_mode = FOC_CONTROL_MODE_VELOCITY;
    output->input_mode = FOC_INPUT_MODE_VELOCITY_RAMP;
    output->current_q_reference_a = 0.25f;
    adapter->last_status = g_step_result;
    return g_step_result;
}

static void fake_force_safe(void *context)
{
    uint32_t *const observed = (uint32_t *)context;
    ++g_force_safe_calls;
    if (observed != NULL)
    {
        ++(*observed);
    }
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t guard;
    (void)memset(&guard, 0, sizeof(guard));
    guard.struct_size = sizeof(guard);
    guard.abi_version = FOC_CONFIG_ABI_VERSION;
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    return guard;
}

static foc_product_command_t valid_command(
    uint32_t source_id,
    uint32_t sequence,
    uint32_t created_at_ms,
    uint32_t valid_until_ms)
{
    foc_product_command_t command;
    (void)memset(&command, 0, sizeof(command));
    command.struct_size = sizeof(command);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.source_id = source_id;
    command.sequence = sequence;
    command.created_at_ms = created_at_ms;
    command.valid_until_ms = valid_until_ms;
    command.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    command.control_mode = FOC_CONTROL_MODE_VELOCITY;
    command.input_mode = FOC_INPUT_MODE_VELOCITY_RAMP;
    command.feedback_mode = FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER;
    command.velocity_ref_rad_s = 1.0f;
    return command;
}

static foc_motion_feedback_t valid_feedback(uint32_t sequence)
{
    foc_motion_feedback_t feedback;
    (void)memset(&feedback, 0, sizeof(feedback));
    feedback.struct_size = sizeof(feedback);
    feedback.version = FOC_MOTION_FEEDBACK_VERSION;
    feedback.sequence = sequence;
    feedback.valid_flags = FOC_MOTION_FEEDBACK_VALID_POSITION |
                           FOC_MOTION_FEEDBACK_VALID_VELOCITY |
                           FOC_MOTION_FEEDBACK_VALID_CURRENT_Q;
    return feedback;
}

static uint32_t output_is_safe_zero(const foc_motion_output_t *output)
{
    foc_motion_output_t expected;
    (void)memset(&expected, 0, sizeof(expected));
    expected.struct_size = sizeof(expected);
    expected.version = FOC_MOTION_OUTPUT_VERSION;
    expected.control_mode = FOC_CONTROL_MODE_INACTIVE;
    expected.input_mode = FOC_INPUT_MODE_INACTIVE;
    return (memcmp(output, &expected, sizeof(expected)) == 0) ? 1U : 0U;
}

static void poison_output(foc_motion_output_t *output)
{
    (void)memset(output, 0xA5, sizeof(*output));
}

static void init_management(
    foc_motion_management_t *management,
    foc_motion_adapter_t *adapter,
    foc_config_bundle_t *config,
    uint32_t *safe_observed)
{
    foc_motion_management_ops_t ops;
    (void)memset(&ops, 0, sizeof(ops));
    ops.struct_size = sizeof(ops);
    ops.version = FOC_MOTION_MANAGEMENT_VERSION;
    ops.force_safe = fake_force_safe;
    ops.context = safe_observed;
    (void)memset(config, 0, sizeof(*config));
    assert(foc_motion_management_init(management, adapter, config, &ops) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(g_init_calls == 1U);
    assert(g_force_safe_calls == 1U);
    assert(*safe_observed == 1U);
    assert(management->initialized == 1U);
    assert(management->enabled == 0U);
    assert(management->state == FOC_MOTION_MANAGEMENT_STATE_DISABLED);
}

static void enable_management(
    foc_motion_management_t *management,
    const foc_config_apply_guard_t *guard)
{
    assert(foc_motion_management_set_enabled(management, 1U, guard) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(management->enabled == 1U);
    assert(management->state == FOC_MOTION_MANAGEMENT_STATE_IDLE);
}

static void begin_management(
    foc_motion_management_t *management,
    const foc_config_apply_guard_t *guard,
    uint32_t owner_id,
    uint32_t source_id,
    uint32_t now_ms,
    uint32_t timeout_ms)
{
    assert(foc_motion_management_begin(
               management, owner_id, source_id, now_ms, timeout_ms, guard) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(management->state == FOC_MOTION_MANAGEMENT_STATE_ACTIVE);
    assert(management->owner_id == owner_id);
    assert(management->source_id == source_id);
    assert(management->started_at_ms == now_ms);
    assert(management->last_activity_ms == now_ms);
    assert(management->timeout_ms == timeout_ms);
    assert(management->begin_count == 1U);
}

static void test_init_default_off_and_unsafe_enable(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    foc_config_apply_guard_t guard = safe_guard();
    foc_motion_management_status_t status;
    uint32_t safe_observed = 0U;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    (void)memset(&status, 0, sizeof(status));
    assert(foc_motion_management_get_status(&management, &status) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(status.state == FOC_MOTION_MANAGEMENT_STATE_DISABLED);
    assert(status.enabled == 0U);

    guard.drive_active = 1U;
    assert(foc_motion_management_set_enabled(&management, 1U, &guard) ==
           FOC_MOTION_MANAGEMENT_UNSAFE_STATE);
    assert(g_enable_calls == 0U);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_DISABLED);
    assert(management.enabled == 0U);
}

static void test_begin_and_rejected_steps_do_not_advance_core(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command;
    foc_motion_feedback_t feedback;
    foc_motion_output_t output;
    uint32_t safe_observed = 0U;
    uint32_t accepted_step_calls;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);

    assert(foc_motion_management_begin(
               &management, 0U, 12U, 100U, 20U, &guard) ==
           FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT);
    assert(foc_motion_management_begin(
               &management, 7U, 0U, 100U, 20U, &guard) ==
           FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT);
    assert(foc_motion_management_begin(
               &management, 7U, 12U, 100U, 0U, &guard) ==
           FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT);
    assert(foc_motion_management_begin(
               &management, 7U, 12U, 100U,
               FOC_MOTION_MANAGEMENT_MAX_TIMEOUT_MS + 1U, &guard) ==
           FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT);

    begin_management(&management, &guard, 7U, 12U, 100U, 20U);
    command = valid_command(12U, 10U, 100U, 119U);
    feedback = valid_feedback(10U);
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 101U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(g_step_calls == 1U);
    assert(output.control_mode == FOC_CONTROL_MODE_VELOCITY);
    assert(output.current_q_reference_a == 0.25f);
    assert(management.last_sequence == 10U);
    assert(management.sequence_valid == 1U);
    assert(management.last_activity_ms == 101U);
    assert(management.step_count == 1U);
    accepted_step_calls = g_step_calls;

    command.sequence = 11U;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 8U, 102U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_NOT_OWNER);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == accepted_step_calls);

    command.source_id = 13U;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 102U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_WRONG_SOURCE);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == accepted_step_calls);

    command.source_id = 12U;
    command.sequence = 10U;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 102U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_STALE_SEQUENCE);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == accepted_step_calls);
    assert(management.last_sequence == 10U);
    assert(management.step_count == 1U);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_ACTIVE);
}

static void test_expired_command_fails_closed(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command;
    foc_motion_feedback_t feedback = valid_feedback(1U);
    foc_motion_output_t output;
    uint32_t safe_observed = 0U;
    uint32_t safe_before;
    uint32_t disable_before;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 7U, 12U, 100U, 50U);
    command = valid_command(12U, 1U, 100U, 110U);
    safe_before = g_force_safe_calls;
    disable_before = g_disable_calls;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 110U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_EXPIRED);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == 0U);
    assert(g_force_safe_calls == (safe_before + 1U));
    assert(g_disable_calls == (disable_before + 1U));
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);
    assert(management.failure_count == 1U);
}

static void test_poll_wrap_timeout_fails_closed(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    uint32_t safe_observed = 0U;
    uint32_t safe_before;
    uint32_t disable_before;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 3U, 4U, UINT32_MAX - 5U, 10U);
    assert(foc_motion_management_poll(&management, 3U) ==
           FOC_MOTION_MANAGEMENT_OK);
    safe_before = g_force_safe_calls;
    disable_before = g_disable_calls;
    assert(foc_motion_management_poll(&management, 4U) ==
           FOC_MOTION_MANAGEMENT_EXPIRED);
    assert(g_force_safe_calls == (safe_before + 1U));
    assert(g_disable_calls == (disable_before + 1U));
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert(management.timeout_count == 1U);
}

static void test_motion_error_fails_closed(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command;
    foc_motion_feedback_t feedback = valid_feedback(1U);
    foc_motion_output_t output;
    uint32_t safe_observed = 0U;
    uint32_t safe_before;
    uint32_t disable_before;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 7U, 12U, 100U, 50U);
    command = valid_command(12U, 1U, 100U, 120U);
    g_step_result = FOC_MOTION_STATUS_CONTROL_FAILURE;
    safe_before = g_force_safe_calls;
    disable_before = g_disable_calls;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 101U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_MOTION_ERROR);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == 1U);
    assert(g_force_safe_calls == (safe_before + 1U));
    assert(g_disable_calls == (disable_before + 1U));
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert((management.last_detail & 0xFFFFU) ==
           FOC_MOTION_STATUS_CONTROL_FAILURE);
}

static void assert_invalid_time_is_rejected(
    uint32_t created_at_ms,
    uint32_t valid_until_ms,
    uint32_t now_ms)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command;
    foc_motion_feedback_t feedback = valid_feedback(1U);
    foc_motion_output_t output;
    uint32_t safe_observed = 0U;
    uint32_t safe_before;
    uint32_t disable_before;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 7U, 12U, now_ms, 50U);
    command = valid_command(
        12U, 1U, created_at_ms, valid_until_ms);
    safe_before = g_force_safe_calls;
    disable_before = g_disable_calls;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, now_ms, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_EXPIRED);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == 0U);
    assert(g_force_safe_calls == (safe_before + 1U));
    assert(g_disable_calls == (disable_before + 1U));
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);
}

static void test_command_time_half_range_contract(void)
{
    /* A zero-width and an exactly-half-range window are both ambiguous. */
    assert_invalid_time_is_rejected(100U, 100U, 100U);
    assert_invalid_time_is_rejected(
        100U, 100U + 0x80000000UL, 100U);

    /* now-created >= half range means the command came from the future. */
    assert_invalid_time_is_rejected(200U, 220U, 199U);
}

static void test_sequence_wrap_and_half_range_rejection(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    const foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command;
    foc_motion_feedback_t feedback = valid_feedback(1U);
    foc_motion_output_t output;
    uint32_t safe_observed = 0U;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 7U, 12U, 100U, 50U);

    command = valid_command(12U, UINT32_MAX, 100U, 130U);
    assert(foc_motion_management_step(
               &management, 7U, 101U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(management.last_sequence == UINT32_MAX);

    command.sequence = 0U;
    assert(foc_motion_management_step(
               &management, 7U, 102U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(management.last_sequence == 0U);
    assert(g_step_calls == 2U);

    command.sequence = 0x80000000UL;
    poison_output(&output);
    assert(foc_motion_management_step(
               &management, 7U, 103U, &command, &feedback, &output) ==
           FOC_MOTION_MANAGEMENT_STALE_SEQUENCE);
    assert(output_is_safe_zero(&output) != 0U);
    assert(g_step_calls == 2U);
    assert(management.last_sequence == 0U);
}

static void test_stop_owner_latch_and_safe_recover(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    foc_config_apply_guard_t guard = safe_guard();
    uint32_t safe_observed = 0U;
    uint32_t disable_before;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    enable_management(&management, &guard);
    begin_management(&management, &guard, 7U, 12U, 100U, 50U);
    disable_before = g_disable_calls;
    assert(foc_motion_management_stop(&management, 8U) ==
           FOC_MOTION_MANAGEMENT_NOT_OWNER);
    assert(g_disable_calls == disable_before);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_ACTIVE);
    assert(foc_motion_management_stop(&management, 7U) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(g_disable_calls == (disable_before + 1U));
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_IDLE);
    assert(management.owner_id == 0U);
    assert(management.stop_count == 1U);

    assert(foc_motion_management_latch_fault(&management, 0x1234U) ==
           FOC_MOTION_MANAGEMENT_MOTION_ERROR);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);
    assert(management.last_detail == 0x1234U);

    /* Disabling always drives the adapter safe, but must not erase a latched
     * failure or bypass the guarded recovery path. */
    assert(foc_motion_management_set_enabled(&management, 0U, NULL) ==
           FOC_MOTION_MANAGEMENT_FAILED_LOCKED);
    assert(management.enabled == 0U);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    assert(management.shutdown_required == 1U);

    guard.active_fault_flags = 1U;
    assert(foc_motion_management_recover(&management, &guard) ==
           FOC_MOTION_MANAGEMENT_UNSAFE_STATE);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_FAILED);
    guard.active_fault_flags = 0U;
    assert(foc_motion_management_recover(&management, &guard) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(management.state == FOC_MOTION_MANAGEMENT_STATE_DISABLED);
    assert(management.shutdown_required == 0U);
}

static void test_apply_config_only_in_safe_idle(void)
{
    foc_motion_management_t management;
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config;
    foc_config_apply_guard_t guard = safe_guard();
    uint32_t safe_observed = 0U;

    fake_reset();
    init_management(&management, &adapter, &config, &safe_observed);
    assert(foc_motion_management_apply_config(
               &management, &config, &guard) ==
           FOC_MOTION_MANAGEMENT_DISABLED);
    assert(g_apply_calls == 0U);
    enable_management(&management, &guard);

    guard.axis_state = FOC_AXIS_STATE_CLOSED_LOOP;
    assert(foc_motion_management_apply_config(
               &management, &config, &guard) ==
           FOC_MOTION_MANAGEMENT_UNSAFE_STATE);
    assert(g_apply_calls == 0U);
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    assert(foc_motion_management_apply_config(
               &management, &config, &guard) ==
           FOC_MOTION_MANAGEMENT_OK);
    assert(g_apply_calls == 1U);

    begin_management(&management, &guard, 9U, 2U, 100U, 20U);
    assert(foc_motion_management_apply_config(
               &management, &config, &guard) ==
           FOC_MOTION_MANAGEMENT_BUSY);
    assert(g_apply_calls == 1U);
}

int main(void)
{
    test_init_default_off_and_unsafe_enable();
    test_begin_and_rejected_steps_do_not_advance_core();
    test_expired_command_fails_closed();
    test_poll_wrap_timeout_fails_closed();
    test_motion_error_fails_closed();
    test_command_time_half_range_contract();
    test_sequence_wrap_and_half_range_rejection();
    test_stop_owner_latch_and_safe_recover();
    test_apply_config_only_in_safe_idle();
    puts("foc motion management tests passed");
    return 0;
}
