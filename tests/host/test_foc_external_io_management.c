#include "foc_external_io_management.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t g_configured_count;
static uint32_t g_polls;
static uint32_t g_submits;
static foc_product_command_t g_last_command;

uint32_t foc_rust_command_abi_version(void)
{
    return FOC_COMMAND_ABI_VERSION;
}

uint32_t foc_rust_command_context_required_size(void)
{
    return 64U;
}

uint32_t foc_rust_command_context_required_align(void)
{
    return 8U;
}

foc_command_arbiter_status_t foc_rust_command_init(
    foc_command_context_storage_t *storage)
{
    return (storage != 0) ? FOC_COMMAND_ARBITER_OK :
                            FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
}

foc_command_arbiter_status_t foc_rust_command_configure(
    foc_command_context_storage_t *storage,
    const foc_command_source_policy_abi_t *sources,
    uint32_t source_count)
{
    (void)storage;
    (void)sources;
    g_configured_count = source_count;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_submit(
    foc_command_context_storage_t *storage,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    (void)storage;
    (void)now_ms;
    ++g_submits;
    g_last_command = *command;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_poll(
    foc_command_context_storage_t *storage,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output)
{
    (void)storage;
    (void)now_ms;
    (void)fault_active;
    ++g_polls;
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_COMMAND_ABI_VERSION;
    output->decision = FOC_COMMAND_DECISION_SAFE;
    output->safe_reason = FOC_COMMAND_SAFE_NO_COMMAND;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_get_status(
    foc_command_context_storage_t *storage,
    foc_command_arbiter_snapshot_t *output)
{
    (void)storage;
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_COMMAND_ABI_VERSION;
    output->configured_source_count = g_configured_count;
    output->polls = g_polls;
    return FOC_COMMAND_ARBITER_OK;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.axis_state = FOC_AXIS_STATE_DISABLED;
    return value;
}

static foc_external_io_capabilities_t pwm_capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_input_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.board_input_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    return value;
}

static foc_external_io_config_t pwm_config(void)
{
    foc_external_io_config_t value;
    foc_external_io_default_config(&value);
    value.input_enable_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.inputs[0].control_mode = FOC_CONTROL_MODE_VELOCITY;
    value.inputs[0].failure_action = FOC_EXTERNAL_FAILURE_RELEASE;
    value.inputs[0].source.source_id = 10U;
    value.inputs[0].source.priority = 5U;
    value.inputs[0].source.permissions =
        (1UL << FOC_PRODUCT_COMMAND_RELEASE) |
        (1UL << FOC_PRODUCT_COMMAND_SETPOINT);
    value.inputs[0].source.lease_ms = 10U;
    value.inputs[0].source.command_timeout_ms = 20U;
    value.simple_inputs.pwm.raw_min = 1000;
    value.simple_inputs.pwm.raw_neutral = 1500;
    value.simple_inputs.pwm.raw_max = 2000;
    value.simple_inputs.pwm.deadband = 20U;
    value.simple_inputs.pwm.negative_limit_si = 100.0f;
    value.simple_inputs.pwm.positive_limit_si = 100.0f;
    return value;
}

static foc_simple_input_candidate_t pwm_candidate(void)
{
    foc_simple_input_candidate_t value = {0};
    value.struct_size = sizeof(value);
    value.version = FOC_SIMPLE_INPUT_CANDIDATE_VERSION;
    value.sample.struct_size = sizeof(value.sample);
    value.sample.version = FOC_INPUT_SAMPLE_VERSION;
    value.sample.input = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.sample.source_id = 10U;
    value.sample.sequence = 1U;
    value.sample.valid_flags =
        FOC_INPUT_SAMPLE_VALID_RAW | FOC_INPUT_SAMPLE_VALID_NORMALIZED;
    value.sample.quality_flags = FOC_INPUT_SAMPLE_QUALITY_CALIBRATED;
    value.sample.raw_value = 1600;
    value.sample.normalized_value = 0.2f;
    value.command.struct_size = sizeof(value.command);
    value.command.version = FOC_PRODUCT_COMMAND_VERSION;
    value.command.source_id = 10U;
    value.command.sequence = 1U;
    value.command.created_at_ms = 100U;
    value.command.valid_until_ms = 120U;
    value.command.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    value.command.axis_request = FOC_AXIS_REQUEST_NONE;
    value.command.control_mode = FOC_CONTROL_MODE_VELOCITY;
    value.command.input_mode = FOC_INPUT_MODE_PASSTHROUGH;
    value.command.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    value.command.velocity_ref_rad_s = 20.0f;
    return value;
}

static void test_injected_capability_input_path(void)
{
    foc_external_io_management_t management;
    foc_external_io_capabilities_t caps = pwm_capabilities();
    foc_external_io_config_t config = pwm_config();
    foc_config_apply_guard_t guard = safe_guard();
    foc_simple_input_candidate_t candidate = pwm_candidate();
    foc_input_service_status_t status;

    g_submits = 0U;
    (void)memset(&g_last_command, 0, sizeof(g_last_command));
    assert(foc_external_io_management_init_with_capabilities(
               &management, &caps) == FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(foc_external_io_management_apply_config(
               &management, &config, &guard) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(g_configured_count == 1U);
    assert(foc_external_io_management_submit_simple_input(
               &management, &candidate, 100U) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(g_submits == 1U);
    assert(g_last_command.velocity_ref_rad_s == 20.0f);
    assert(foc_external_io_management_get_input_status(
               &management, &status) == FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(status.healthy_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);

    candidate.command.axis_request = FOC_AXIS_REQUEST_CLOSED_LOOP_CONTROL;
    assert(foc_external_io_management_submit_simple_input(
               &management, &candidate, 101U) ==
           FOC_EXTERNAL_IO_MANAGEMENT_CANDIDATE_REJECTED);
    assert(g_submits == 1U);

    assert(foc_external_io_management_poll(&management, 119U, 0U) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(management.last_expired_input_mask == 0U);
    assert(foc_external_io_management_poll(&management, 120U, 0U) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(management.last_expired_input_mask ==
           FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(foc_external_io_management_get_input_status(
               &management, &status) == FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(status.healthy_input_mask == 0U);
    assert(status.expired_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);
}

int main(void)
{
    foc_external_io_management_t management;
    foc_external_io_capabilities_t capabilities;
    foc_external_io_config_t config;
    foc_external_io_config_t before;
    foc_external_command_event_t event;
    foc_config_apply_guard_t guard = safe_guard();

    foc_external_io_empty_capabilities(&capabilities);
    assert(foc_external_io_management_init_with_capabilities(
               &management, &capabilities) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(management.initialized == 1U);
    assert(g_configured_count == 0U);
    assert(foc_external_io_config_is_default_off(
               &management.active_config) == 1U);

    foc_external_io_default_config(&config);
    config.revision = 2U;
    assert(foc_external_io_management_apply_config(
               &management, &config, &guard) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(management.active_config.revision == 2U);

    before = management.active_config;
    config.transport_enable_mask = FOC_EXTERNAL_TRANSPORT_UART;
    guard.drive_active = 1U;
    assert(foc_external_io_management_apply_config(
               &management, &config, &guard) ==
           FOC_EXTERNAL_IO_MANAGEMENT_CONFIG_APPLY_FAILED);
    assert(memcmp(&before, &management.active_config, sizeof(before)) == 0);

    assert(foc_external_io_management_poll(&management, 10U, 0U) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(foc_external_io_management_get_event(&management, &event) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(event.kind == FOC_EXTERNAL_COMMAND_EVENT_SAFE);
    assert(event.sequence == 1U);
    assert(event.arbiter.configured_source_count == 0U);

    test_injected_capability_input_path();

    puts("foc external io management tests passed");
    return 0;
}
