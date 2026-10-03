#include "foc_external_input_owner.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    foc_external_input_mask_t enabled_mask;
    foc_external_input_raw_sample_t latest;
    foc_external_input_raw_health_t health;
    uint32_t has_sample;
    uint32_t start_count;
    uint32_t stop_count;
    uint32_t fail_start;
} fake_port_context_t;

static uint32_t g_submit_count;
static uint32_t g_setpoint_count;
static uint32_t g_release_count;
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
    (void)source_count;
    return FOC_COMMAND_ARBITER_OK;
}

foc_command_arbiter_status_t foc_rust_command_submit(
    foc_command_context_storage_t *storage,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    (void)storage;
    (void)now_ms;
    if (command == 0)
    {
        return FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
    }
    ++g_submit_count;
    g_last_command = *command;
    if (command->command_kind == FOC_PRODUCT_COMMAND_SETPOINT)
    {
        ++g_setpoint_count;
    }
    if (command->command_kind == FOC_PRODUCT_COMMAND_RELEASE)
    {
        ++g_release_count;
    }
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
    if (output == 0)
    {
        return FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
    }
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
    if (output == 0)
    {
        return FOC_COMMAND_ARBITER_INVALID_ARGUMENT;
    }
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_COMMAND_ABI_VERSION;
    return FOC_COMMAND_ARBITER_OK;
}

uint32_t foc_rust_input_normalize_pwm(
    const foc_centered_input_config_t *config,
    uint32_t pulse_width_us,
    foc_input_normalization_output_t *output)
{
    (void)pulse_width_us;
    if ((config == 0) || (output == 0))
    {
        return FOC_INPUT_STATUS_INVALID_ARGUMENT;
    }
    return FOC_INPUT_STATUS_INVALID_CONFIG;
}

uint32_t foc_rust_input_normalize_analog(
    const foc_centered_input_config_t *config,
    int32_t adc_counts,
    foc_input_normalization_output_t *output)
{
    float normalized;
    if ((config == 0) || (output == 0))
    {
        return FOC_INPUT_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_INPUT_OUTPUT_VERSION;
    output->raw_value = adc_counts;
    if ((adc_counts < config->raw_min) || (adc_counts > config->raw_max))
    {
        output->status = FOC_INPUT_STATUS_OUT_OF_RANGE;
        return output->status;
    }
    if (adc_counts >= config->raw_neutral)
    {
        normalized = (float)(adc_counts - config->raw_neutral) /
            (float)(config->raw_max - config->raw_neutral);
        output->setpoint_si = normalized * config->positive_limit_si;
    }
    else
    {
        normalized = (float)(adc_counts - config->raw_neutral) /
            (float)(config->raw_neutral - config->raw_min);
        output->setpoint_si = normalized * config->negative_limit_si;
    }
    output->status = FOC_INPUT_STATUS_OK;
    output->quality_flags =
        FOC_INPUT_QUALITY_CALIBRATED | FOC_INPUT_QUALITY_CENTERED;
    output->normalized_value = normalized;
    return output->status;
}

uint32_t foc_rust_input_convert_step_dir(
    const foc_step_dir_input_config_t *config,
    int32_t accumulated_count,
    foc_input_normalization_output_t *output)
{
    (void)accumulated_count;
    if ((config == 0) || (output == 0))
    {
        return FOC_INPUT_STATUS_INVALID_ARGUMENT;
    }
    return FOC_INPUT_STATUS_INVALID_CONFIG;
}

static foc_external_input_port_result_t fake_start(
    void *context,
    foc_external_input_mask_t input_mask)
{
    fake_port_context_t *fake = (fake_port_context_t *)context;
    ++fake->start_count;
    if (fake->fail_start != 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_HARDWARE_ERROR;
    }
    fake->enabled_mask |= input_mask;
    fake->health.enabled_input_mask = fake->enabled_mask;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_stop(
    void *context,
    foc_external_input_mask_t input_mask)
{
    fake_port_context_t *fake = (fake_port_context_t *)context;
    ++fake->stop_count;
    fake->enabled_mask &= ~input_mask;
    fake->health.enabled_input_mask = fake->enabled_mask;
    fake->health.healthy_input_mask &= fake->enabled_mask;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_read_latest(
    void *context,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample)
{
    fake_port_context_t *fake = (fake_port_context_t *)context;
    if ((fake->enabled_mask & input) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_DISABLED;
    }
    if (fake->has_sample == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE;
    }
    *sample = fake->latest;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t fake_get_health(
    void *context,
    foc_external_input_raw_health_t *health)
{
    fake_port_context_t *fake = (fake_port_context_t *)context;
    *health = fake->health;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static const foc_external_input_port_ops_t g_fake_ops =
{
    sizeof(foc_external_input_port_ops_t),
    FOC_EXTERNAL_INPUT_PORT_OPS_VERSION,
    fake_start,
    fake_stop,
    fake_read_latest,
    fake_get_health,
};

static foc_external_io_capabilities_t analog_capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_input_mask = FOC_EXTERNAL_INPUT_ANALOG;
    value.board_input_mask = FOC_EXTERNAL_INPUT_ANALOG;
    return value;
}

static foc_external_io_config_t analog_config(uint32_t revision)
{
    foc_external_io_config_t value;
    foc_external_io_default_config(&value);
    value.revision = revision;
    value.input_enable_mask = FOC_EXTERNAL_INPUT_ANALOG;
    value.inputs[2].control_mode = FOC_CONTROL_MODE_VELOCITY;
    value.inputs[2].failure_action = FOC_EXTERNAL_FAILURE_CONTROLLED_STOP;
    value.inputs[2].source.source_id = 30U;
    value.inputs[2].source.priority = 5U;
    value.inputs[2].source.permissions =
        (1UL << FOC_PRODUCT_COMMAND_RELEASE) |
        (1UL << FOC_PRODUCT_COMMAND_SETPOINT);
    value.inputs[2].source.lease_ms = 10U;
    value.inputs[2].source.command_timeout_ms = 20U;
    value.simple_inputs.analog.raw_min = 0;
    value.simple_inputs.analog.raw_neutral = 2048;
    value.simple_inputs.analog.raw_max = 4095;
    value.simple_inputs.analog.deadband = 10U;
    value.simple_inputs.analog.negative_limit_si = 100.0f;
    value.simple_inputs.analog.positive_limit_si = 100.0f;
    return value;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.axis_state = FOC_AXIS_STATE_DISABLED;
    return value;
}

static void fake_init(fake_port_context_t *fake)
{
    (void)memset(fake, 0, sizeof(*fake));
    fake->health.struct_size = sizeof(fake->health);
    fake->health.version = FOC_EXTERNAL_INPUT_RAW_HEALTH_VERSION;
    fake->health.supported_input_mask = FOC_EXTERNAL_INPUT_ANALOG;
    fake->health.last_error = FOC_EXTERNAL_INPUT_PORT_OK;
}

static void fake_publish(
    fake_port_context_t *fake,
    uint32_t sequence,
    int32_t raw_value,
    uint32_t quality_flags)
{
    (void)memset(&fake->latest, 0, sizeof(fake->latest));
    fake->latest.struct_size = sizeof(fake->latest);
    fake->latest.version = FOC_EXTERNAL_INPUT_RAW_SAMPLE_VERSION;
    fake->latest.input = FOC_EXTERNAL_INPUT_ANALOG;
    fake->latest.sequence = sequence;
    fake->latest.sampled_at_us = sequence * 1000U;
    fake->latest.valid_flags =
        FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
        FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP;
    fake->latest.quality_flags = quality_flags;
    fake->latest.raw_value = raw_value;
    fake->has_sample = 1U;
    fake->health.healthy_input_mask = FOC_EXTERNAL_INPUT_ANALOG;
    fake->health.sample_count = sequence;
}

int main(void)
{
    foc_external_io_capabilities_t capabilities = analog_capabilities();
    foc_external_io_management_t management;
    foc_external_input_port_t port;
    foc_external_input_owner_t owner;
    foc_external_input_owner_status_t status;
    fake_port_context_t fake;
    foc_external_io_config_t config = analog_config(2U);
    foc_external_io_config_t disabled;
    foc_config_apply_guard_t guard = safe_guard();

    fake_init(&fake);
    g_submit_count = 0U;
    g_setpoint_count = 0U;
    g_release_count = 0U;
    assert(foc_external_io_management_init_with_capabilities(
               &management, &capabilities) ==
           FOC_EXTERNAL_IO_MANAGEMENT_OK);
    assert(foc_external_input_port_bind(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &fake, &g_fake_ops) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_owner_init(&owner, &management, &port) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(foc_external_input_owner_apply_config(
               &owner, &config, &guard, 100U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(fake.start_count == 1U);
    assert(owner.status.enabled_input_mask == FOC_EXTERNAL_INPUT_ANALOG);

    assert(foc_external_input_owner_poll(&owner, 119U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_submit_count == 0U);
    assert(foc_external_input_owner_poll(&owner, 120U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_release_count == 1U);
    assert(owner.status.requested_failure_action ==
           FOC_EXTERNAL_FAILURE_CONTROLLED_STOP);
    assert(owner.status.effective_failure_action ==
           FOC_EXTERNAL_FAILURE_RELEASE);

    fake_publish(&fake, 1U, 3072, FOC_EXTERNAL_INPUT_RAW_QUALITY_FIRST);
    assert(foc_external_input_owner_poll(&owner, 121U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_setpoint_count == 1U);
    assert(g_last_command.command_kind == FOC_PRODUCT_COMMAND_SETPOINT);
    assert(owner.status.recovery_count == 1U);
    assert(foc_external_input_owner_poll(&owner, 122U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_setpoint_count == 1U);

    fake_publish(&fake, 2U, 5000, 0U);
    assert(foc_external_input_owner_poll(&owner, 123U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_release_count == 2U);
    assert(owner.status.last_failure_reason ==
           FOC_EXTERNAL_INPUT_FAILURE_OUT_OF_RANGE);

    fake_publish(
        &fake, 3U, 2200,
        FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS);
    assert(foc_external_input_owner_poll(&owner, 124U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_setpoint_count == 2U);
    assert(owner.status.recovery_count == 2U);
    assert((owner.status.degraded_input_mask &
            FOC_EXTERNAL_INPUT_ANALOG) != 0U);
    assert(foc_external_input_owner_poll(&owner, 143U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_release_count == 2U);
    assert(foc_external_input_owner_poll(&owner, 144U, 0U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(g_release_count == 3U);

    assert(foc_external_input_owner_get_status(&owner, &status) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(status.raw_sample_count == 3U);
    assert(status.setpoint_count == 2U);
    assert(status.failure_count == 3U);
    assert(status.release_count == 3U);

    foc_external_io_default_config(&disabled);
    disabled.revision = 3U;
    assert(foc_external_input_owner_apply_config(
               &owner, &disabled, &guard, 145U) ==
           FOC_EXTERNAL_INPUT_OWNER_OK);
    assert(fake.stop_count == 1U);
    assert(owner.status.enabled_input_mask == 0U);

    fake.fail_start = 1U;
    config.revision = 4U;
    assert(foc_external_input_owner_apply_config(
               &owner, &config, &guard, 146U) ==
           FOC_EXTERNAL_INPUT_OWNER_PORT_FAILED);
    assert(owner.status.enabled_input_mask == 0U);
    assert(management.active_config.input_enable_mask == 0U);

    fake.fail_start = 0U;
    guard.drive_active = 1U;
    assert(foc_external_input_owner_apply_config(
               &owner, &config, &guard, 147U) ==
           FOC_EXTERNAL_INPUT_OWNER_UNSAFE_STATE);

    puts("foc external input owner tests passed");
    return 0;
}
