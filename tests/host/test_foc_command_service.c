#include "foc_command_service.h"

#include <assert.h>
#include <stdio.h>

static uint32_t g_sink_calls;
static uint32_t g_sink_result;
static uint32_t g_configure_calls;
static uint32_t g_configure_result;

static uint32_t fake_configure(
    void *context,
    const foc_command_service_source_t *sources,
    uint32_t source_count)
{
    uint32_t *last_source = (uint32_t *)context;
    ++g_configure_calls;
    if ((source_count != 0U) && (sources != NULL))
    {
        *last_source = sources[0].source_id;
    }
    return g_configure_result;
}

static uint32_t fake_sink(
    void *context,
    const foc_product_command_t *command,
    uint32_t now_ms)
{
    uint32_t *last_source = (uint32_t *)context;
    ++g_sink_calls;
    *last_source = command->source_id + now_ms;
    return g_sink_result;
}

static foc_external_io_capabilities_t capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_input_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.board_input_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    return value;
}

static foc_external_io_config_t config(void)
{
    foc_external_io_config_t value;
    foc_external_io_default_config(&value);
    value.input_enable_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.inputs[0].control_mode = FOC_CONTROL_MODE_TORQUE;
    value.inputs[0].failure_action = FOC_EXTERNAL_FAILURE_RELEASE;
    value.inputs[0].source.source_id = 10U;
    value.inputs[0].source.priority = 1U;
    value.inputs[0].source.permissions =
        (1UL << FOC_PRODUCT_COMMAND_RELEASE) |
        (1UL << FOC_PRODUCT_COMMAND_SETPOINT);
    value.inputs[0].source.lease_ms = 10U;
    value.inputs[0].source.command_timeout_ms = 20U;
    value.simple_inputs.pwm.raw_min = 1000;
    value.simple_inputs.pwm.raw_neutral = 1500;
    value.simple_inputs.pwm.raw_max = 2000;
    value.simple_inputs.pwm.deadband = 20U;
    value.simple_inputs.pwm.negative_limit_si = 1.0f;
    value.simple_inputs.pwm.positive_limit_si = 1.0f;
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

static foc_product_command_t command(void)
{
    foc_product_command_t value = {0};
    value.struct_size = sizeof(value);
    value.version = FOC_PRODUCT_COMMAND_VERSION;
    value.source_id = 10U;
    value.sequence = 1U;
    value.created_at_ms = 100U;
    value.valid_until_ms = 120U;
    value.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    value.control_mode = FOC_CONTROL_MODE_TORQUE;
    value.input_mode = FOC_INPUT_MODE_PASSTHROUGH;
    value.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    value.torque_ref_nm = 0.01f;
    return value;
}

static void test_single_gateway_permissions_time_and_sink(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t value = config();
    foc_config_apply_guard_t guard = safe_guard();
    foc_external_io_validation_t validation;
    foc_command_service_t service;
    foc_command_service_status_t status;
    foc_command_service_ops_t ops = {0};
    foc_product_command_t product = command();
    uint32_t sink_context = 0U;

    g_sink_calls = 0U;
    g_sink_result = 0U;
    g_configure_calls = 0U;
    g_configure_result = 0U;
    ops.struct_size = sizeof(ops);
    ops.version = FOC_COMMAND_SERVICE_VERSION;
    ops.configure = fake_configure;
    ops.submit = fake_sink;
    ops.context = &sink_context;
    assert(foc_command_service_init(&service, &caps, &ops) ==
           FOC_COMMAND_SERVICE_OK);
    assert(foc_command_service_submit(&service, &product, 101U) ==
           FOC_COMMAND_SERVICE_DISABLED);
    assert(foc_command_service_apply_config(
               &service, &value, &guard, &validation) ==
           FOC_COMMAND_SERVICE_OK);
    assert(g_configure_calls == 2U);
    assert(sink_context == 10U);
    assert(foc_command_service_submit(&service, &product, 101U) ==
           FOC_COMMAND_SERVICE_OK);
    assert(g_sink_calls == 1U);
    assert(sink_context == 111U);

    product.command_kind = FOC_PRODUCT_COMMAND_AXIS_REQUEST;
    assert(foc_command_service_submit(&service, &product, 102U) ==
           FOC_COMMAND_SERVICE_UNAUTHORIZED);
    product = command();
    product.source_id = 11U;
    assert(foc_command_service_submit(&service, &product, 102U) ==
           FOC_COMMAND_SERVICE_UNKNOWN_SOURCE);
    product = command();
    assert(foc_command_service_submit(&service, &product, 120U) ==
           FOC_COMMAND_SERVICE_EXPIRED);
    g_sink_result = 0x55U;
    assert(foc_command_service_submit(&service, &product, 110U) ==
           FOC_COMMAND_SERVICE_SINK_REJECTED);
    assert(foc_command_service_get_status(&service, &status) ==
           FOC_COMMAND_SERVICE_OK);
    assert(status.submitted == 6U);
    assert(status.accepted == 1U);
    assert(status.rejected == 5U);
    assert(status.unknown_source == 1U);
    assert(status.unauthorized == 1U);
    assert(status.expired == 1U);
    assert(status.sink_rejected == 1U);
    assert(status.last_detail == 0x55U);
}

static void test_apply_requires_safe_guard(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t value = config();
    foc_config_apply_guard_t guard = safe_guard();
    foc_external_io_validation_t validation;
    foc_command_service_t service;
    foc_command_service_ops_t ops = {0};
    uint32_t context = 0U;

    ops.struct_size = sizeof(ops);
    ops.version = FOC_COMMAND_SERVICE_VERSION;
    ops.configure = fake_configure;
    ops.submit = fake_sink;
    ops.context = &context;
    assert(foc_command_service_init(&service, &caps, &ops) ==
           FOC_COMMAND_SERVICE_OK);
    guard.drive_active = 1U;
    assert(foc_command_service_apply_config(
               &service, &value, &guard, &validation) ==
           FOC_COMMAND_SERVICE_UNSAFE_STATE);
    assert(foc_command_service_disable(&service) ==
           FOC_COMMAND_SERVICE_DISABLED);
}

int main(void)
{
    test_single_gateway_permissions_time_and_sink();
    test_apply_requires_safe_guard();
    puts("foc command service tests passed");
    return 0;
}
