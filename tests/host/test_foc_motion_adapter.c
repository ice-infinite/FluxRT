#include "foc_motion_adapter.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static foc_motion_status_t g_configure_status = FOC_MOTION_STATUS_OK;
static foc_motion_status_t g_step_status = FOC_MOTION_STATUS_OK;
static uint32_t g_enable_calls;
static uint32_t g_configure_calls;

uint32_t foc_rust_motion_abi_version(void)
{
    return FOC_MOTION_ABI_VERSION;
}

uint32_t foc_rust_motion_context_required_size(void)
{
    return 128U;
}

uint32_t foc_rust_motion_context_required_align(void)
{
    return 8U;
}

foc_motion_status_t foc_rust_motion_init(foc_motion_context_t *storage)
{
    return (storage != NULL) ? FOC_MOTION_STATUS_OK :
                               FOC_MOTION_STATUS_INVALID_ARGUMENT;
}

foc_motion_status_t foc_rust_motion_configure(
    foc_motion_context_t *storage,
    const foc_config_bundle_t *config)
{
    assert(storage != NULL);
    assert(config != NULL);
    g_configure_calls++;
    return g_configure_status;
}

foc_motion_status_t foc_rust_motion_enable(foc_motion_context_t *storage)
{
    assert(storage != NULL);
    g_enable_calls++;
    return FOC_MOTION_STATUS_OK;
}

foc_motion_status_t foc_rust_motion_disable(foc_motion_context_t *storage)
{
    assert(storage != NULL);
    return FOC_MOTION_STATUS_OK;
}

foc_motion_status_t foc_rust_motion_step(
    foc_motion_context_t *storage,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output)
{
    assert(storage != NULL);
    assert(command != NULL);
    assert(feedback != NULL);
    assert(output != NULL);
    (void)memset(output, 0xA5, sizeof(*output));
    output->config_revision = 77U;
    output->source_sequence = command->sequence;
    if (g_step_status == FOC_MOTION_STATUS_OK)
    {
        output->struct_size = sizeof(*output);
        output->version = FOC_MOTION_OUTPUT_VERSION;
        output->control_mode = FOC_CONTROL_MODE_VELOCITY;
        output->input_mode = FOC_INPUT_MODE_VELOCITY_RAMP;
        output->current_q_reference_a = 0.25f;
    }
    return g_step_status;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t guard = {0};
    guard.struct_size = sizeof(guard);
    guard.abi_version = FOC_CONFIG_ABI_VERSION;
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    return guard;
}

static void test_default_off_guard_and_fail_safe_step(void)
{
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config = {0};
    foc_config_apply_guard_t guard = safe_guard();
    foc_product_command_t command = {0};
    foc_motion_feedback_t feedback = {0};
    foc_motion_output_t output;

    assert(foc_motion_adapter_init(&adapter, &config) == FOC_MOTION_STATUS_OK);
    assert(g_configure_calls == 1U);
    assert(adapter.initialized == 1U);
    assert(adapter.configured == 1U);
    assert(adapter.enabled == 0U);

    (void)memset(&output, 0xA5, sizeof(output));
    assert(foc_motion_adapter_step(&adapter, &command, &feedback, &output) ==
           FOC_MOTION_STATUS_DISABLED);
    assert(output.control_mode == FOC_CONTROL_MODE_INACTIVE);
    assert(output.current_q_reference_a == 0.0f);

    guard.drive_active = 1U;
    assert(foc_motion_adapter_enable(&adapter, &guard) ==
           FOC_MOTION_STATUS_INVALID_STATE);
    assert(g_enable_calls == 0U);
    guard.drive_active = 0U;
    assert(foc_motion_adapter_enable(&adapter, &guard) == FOC_MOTION_STATUS_OK);
    assert(g_enable_calls == 1U);
    assert(adapter.enabled == 1U);

    assert(foc_motion_adapter_apply_config(&adapter, &config, &guard) ==
           FOC_MOTION_STATUS_INVALID_STATE);
    command.sequence = 42U;
    assert(foc_motion_adapter_step(&adapter, &command, &feedback, &output) ==
           FOC_MOTION_STATUS_OK);
    assert(output.control_mode == FOC_CONTROL_MODE_VELOCITY);
    assert(output.current_q_reference_a == 0.25f);

    g_step_status = FOC_MOTION_STATUS_INVALID_FEEDBACK;
    assert(foc_motion_adapter_step(&adapter, &command, &feedback, &output) ==
           FOC_MOTION_STATUS_INVALID_FEEDBACK);
    assert(output.config_revision == 77U);
    assert(output.source_sequence == 42U);
    assert(output.control_mode == FOC_CONTROL_MODE_INACTIVE);
    assert(output.current_q_reference_a == 0.0f);

    assert(foc_motion_adapter_disable(&adapter) == FOC_MOTION_STATUS_OK);
    assert(adapter.enabled == 0U);
}

static void test_disabled_configuration_stays_unconfigured(void)
{
    foc_motion_adapter_t adapter;
    foc_config_bundle_t config = {0};
    g_configure_status = FOC_MOTION_STATUS_DISABLED;
    assert(foc_motion_adapter_init(&adapter, &config) ==
           FOC_MOTION_STATUS_DISABLED);
    assert(adapter.initialized == 1U);
    assert(adapter.configured == 0U);
    assert(adapter.enabled == 0U);
}

int main(void)
{
    test_default_off_guard_and_fail_safe_step();
    test_disabled_configuration_stays_unconfigured();
    puts("foc motion adapter tests passed");
    return 0;
}
