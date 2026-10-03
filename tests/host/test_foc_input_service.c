#include "foc_input_service.h"

#include <assert.h>
#include <stdio.h>

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

static foc_input_sample_t sample(uint32_t sequence)
{
    foc_input_sample_t value = {0};
    value.struct_size = sizeof(value);
    value.version = FOC_INPUT_SAMPLE_VERSION;
    value.input = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.source_id = 10U;
    value.sequence = sequence;
    value.valid_flags =
        FOC_INPUT_SAMPLE_VALID_RAW | FOC_INPUT_SAMPLE_VALID_NORMALIZED;
    value.raw_value = 1500;
    value.normalized_value = 0.25f;
    return value;
}

static void test_apply_publish_timeout_and_recovery(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t value = config();
    foc_config_apply_guard_t guard = safe_guard();
    foc_external_io_validation_t validation;
    foc_input_service_t service;
    foc_input_service_status_t status;
    foc_input_sample_t input = sample(1U);
    uint32_t expired;

    assert(foc_input_service_init(&service, &caps) == FOC_INPUT_SERVICE_OK);
    guard.axis_state = FOC_AXIS_STATE_CLOSED_LOOP;
    assert(foc_input_service_apply_config(
               &service, &value, &guard, &validation) ==
           FOC_INPUT_SERVICE_UNSAFE_STATE);
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    assert(foc_input_service_apply_config(
               &service, &value, &guard, &validation) ==
           FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_publish(&service, &input, 100U) ==
           FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_publish(&service, &input, 101U) ==
           FOC_INPUT_SERVICE_STALE_SEQUENCE);
    assert(foc_input_service_poll(&service, 119U, &expired) ==
           FOC_INPUT_SERVICE_OK);
    assert(expired == 0U);
    assert(foc_input_service_poll(&service, 120U, &expired) ==
           FOC_INPUT_SERVICE_OK);
    assert(expired == FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(foc_input_service_get_status(&service, &status) ==
           FOC_INPUT_SERVICE_OK);
    assert(status.healthy_input_mask == 0U);
    assert(status.expired_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(status.slots[0].timeout_count == 1U);

    input = sample(2U);
    assert(foc_input_service_publish(&service, &input, 121U) ==
           FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_get_status(&service, &status) ==
           FOC_INPUT_SERVICE_OK);
    assert(status.healthy_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(status.expired_input_mask == 0U);
    assert(status.slots[0].recovery_count == 1U);

    assert(foc_input_service_mark_unhealthy(
               &service,
               FOC_EXTERNAL_INPUT_PWM_PULSE,
               FOC_INPUT_SAMPLE_QUALITY_DEGRADED) ==
           FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_get_status(&service, &status) ==
           FOC_INPUT_SERVICE_OK);
    assert(status.healthy_input_mask == 0U);
    assert(status.expired_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(status.slots[0].failure_count == 1U);

    input = sample(3U);
    assert(foc_input_service_publish(&service, &input, 122U) ==
           FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_get_status(&service, &status) ==
           FOC_INPUT_SERVICE_OK);
    assert(status.healthy_input_mask == FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(status.expired_input_mask == 0U);
    assert(status.slots[0].recovery_count == 2U);
    assert(status.slots[0].failure_count == 1U);
}

static void test_wrong_source_and_disabled_input(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t value = config();
    foc_config_apply_guard_t guard = safe_guard();
    foc_external_io_validation_t validation;
    foc_input_service_t service;
    foc_input_sample_t input = sample(1U);

    assert(foc_input_service_init(&service, &caps) == FOC_INPUT_SERVICE_OK);
    assert(foc_input_service_publish(&service, &input, 0U) ==
           FOC_INPUT_SERVICE_DISABLED);
    assert(foc_input_service_apply_config(
               &service, &value, &guard, &validation) ==
           FOC_INPUT_SERVICE_OK);
    input.source_id = 11U;
    assert(foc_input_service_publish(&service, &input, 0U) ==
           FOC_INPUT_SERVICE_WRONG_SOURCE);
}

int main(void)
{
    test_apply_publish_timeout_and_recovery();
    test_wrong_source_and_disabled_input();
    puts("foc input service tests passed");
    return 0;
}
