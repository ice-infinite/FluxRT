#include "foc_lsi_actuation_executor.h"
#include "foc_lsi_identification.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert((uint32_t)FOC_LSI_DRIVE_OFF == FOC_LSI_DRIVE_ABI_OFF,
               "OFF mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_BIAS == FOC_LSI_DRIVE_ABI_BIAS,
               "BIAS mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_PULSE_POSITIVE ==
                   FOC_LSI_DRIVE_ABI_PULSE_POSITIVE,
               "positive mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_PULSE_NEGATIVE ==
                   FOC_LSI_DRIVE_ABI_PULSE_NEGATIVE,
               "negative mapping drifted");

/* Host fake: Rust owns the real formula tests; this executable tests C glue. */
foc_status_t foc_rust_lsi_plan(
    const foc_lsi_actuation_config_t *config,
    const foc_lsi_actuation_input_t *input,
    foc_lsi_actuation_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_LSI_ACTUATION_OUTPUT_VERSION;
    output->safe_output_required = 1U;
    if ((config == 0) || (input == 0) ||
        (config->version != FOC_LSI_ACTUATION_CONFIG_VERSION) ||
        (input->version != FOC_LSI_ACTUATION_INPUT_VERSION))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    output->source_control_tick = input->control_tick;
    output->expected_active_control_tick = input->control_tick;
    if (input->drive_request == FOC_LSI_DRIVE_ABI_OFF)
    {
        return (input->force_safe_output != 0U) ?
            FOC_STATUS_OK : FOC_STATUS_INVALID_ARGUMENT;
    }
    if (input->capture_ready == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    if ((input->capture_full != 0U) || (input->hardware_fault != 0U) ||
        (input->software_trip != 0U))
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }
    output->safe_output_required = 0U;
    output->drive_active = 1U;
    output->expected_active_control_tick = input->control_tick + 1U;
    output->duty_u = (input->requested_perturbation_voltage_v == 0.123f) ?
        1.2f : 0.60f;
    output->duty_v = 0.45f;
    output->duty_w = 0.45f;
    return FOC_STATUS_OK;
}

static foc_lsi_actuation_config_t actuation_config(void)
{
    foc_lsi_actuation_config_t config = {0};
    config.struct_size = sizeof(config);
    config.version = FOC_LSI_ACTUATION_CONFIG_VERSION;
    config.sample_rate_hz = 12000U;
    config.actuation_delay_control_ticks = 1U;
    config.stator_resistance_ohm = 4.9666667f;
    config.maximum_bias_current_a = 0.2f;
    config.maximum_perturbation_voltage_v = 0.4f;
    config.current_trip_a = 1.15f;
    config.minimum_bus_voltage_v = 7.0f;
    config.maximum_bus_voltage_v = 18.0f;
    config.minimum_duty = 0.03f;
    config.maximum_duty = 0.97f;
    return config;
}

static foc_lsi_executor_config_t platform_config(void)
{
    foc_lsi_executor_config_t config = {0};
    config.struct_size = sizeof(config);
    config.version = FOC_LSI_EXECUTOR_CONFIG_VERSION;
    config.sample_rate_hz = 12000U;
    config.pwm_period_ticks = 7083U;
    config.adc_max_code = 4095U;
    config.actuation_delay_control_ticks = 1U;
    config.current_u_offset_raw = 2048U;
    config.current_v_offset_raw = 2040U;
    config.current_counts_per_amp = 626.5f;
    config.bus_volts_per_count = 3.3f / (4095.0f * 0.0625f);
    config.minimum_duty = 0.03f;
    config.maximum_duty = 0.97f;
    return config;
}

static foc_lsi_executor_command_t command(uint32_t request)
{
    foc_lsi_executor_command_t value = {0};
    value.struct_size = sizeof(value);
    value.version = FOC_LSI_EXECUTOR_COMMAND_VERSION;
    value.drive_request = request;
    value.capture_ready = 1U;
    value.requested_bias_current_a = 0.2f;
    if (request == FOC_LSI_DRIVE_ABI_PULSE_POSITIVE)
    {
        value.requested_perturbation_voltage_v = 0.4f;
    }
    else if (request == FOC_LSI_DRIVE_ABI_PULSE_NEGATIVE)
    {
        value.requested_perturbation_voltage_v = -0.4f;
    }
    return value;
}

static foc_lsi_raw_sample_t raw_sample(uint32_t tick)
{
    foc_lsi_raw_sample_t raw = {0};
    raw.control_tick = tick;
    raw.current_u_raw = 1985U;
    raw.current_v_raw = 2040U;
    raw.bus_voltage_raw = 954U;
    raw.pwm_period_ticks = 7083U;
    raw.flags = FOC_LSI_RAW_FLAG_ADC_VALID;
    return raw;
}

static void assert_preload(const foc_lsi_executor_output_t *output)
{
    assert(output->action == FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD);
    assert(output->result == FOC_LSI_EXECUTOR_RESULT_PRELOAD_READY);
    assert(output->compare_u == 4250U);
    assert(output->compare_v == 3187U);
    assert(output->compare_w == 3187U);
}

static void test_conversion_quantisation_and_ledger(void)
{
    foc_lsi_executor_t executor;
    foc_lsi_actuation_config_t actuation = actuation_config();
    foc_lsi_executor_config_t platform = platform_config();
    foc_lsi_executor_command_t request = command(FOC_LSI_DRIVE_ABI_BIAS);
    foc_lsi_executor_output_t output;
    foc_lsi_raw_sample_t raw = raw_sample(100U);

    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_OK);
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_OK);
    assert_preload(&output);
    assert(fabsf(output.phase_u_current_a - (63.0f / 626.5f)) < 1.0e-6f);
    assert(fabsf(output.bus_voltage_v -
                 (954.0f * platform.bus_volts_per_count)) < 1.0e-6f);
    assert(output.expected_active_control_tick == 101U);

    raw = raw_sample(101U);
    raw.compare_u = output.compare_u;
    raw.compare_v = output.compare_v;
    raw.compare_w = output.compare_w;
    raw.flags |= FOC_LSI_RAW_FLAG_DRIVE_ACTIVE;
    request = command(FOC_LSI_DRIVE_ABI_OFF);
    request.force_safe_output = 1U;
    request.capture_ready = 0U;
    request.requested_bias_current_a = 0.0f;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_OK);
    assert(output.action == FOC_LSI_EXECUTOR_ACTION_SAFE);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_SAFE_REQUESTED);
    assert(output.ledger_checked == 1U);
    assert(output.ledger_matched == 1U);
    assert(executor.pending_ledger_valid == 0U);
}

static void test_ledger_polarity_and_capture_completion_fail_closed(void)
{
    foc_lsi_executor_t executor;
    foc_lsi_actuation_config_t actuation = actuation_config();
    foc_lsi_executor_config_t platform = platform_config();
    foc_lsi_executor_command_t request =
        command(FOC_LSI_DRIVE_ABI_PULSE_POSITIVE);
    foc_lsi_executor_output_t output;
    foc_lsi_raw_sample_t raw = raw_sample(5U);

    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_OK);
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_OK);
    raw = raw_sample(6U);
    raw.compare_u = output.compare_u;
    raw.compare_v = output.compare_v;
    raw.compare_w = output.compare_w;
    raw.flags |= FOC_LSI_RAW_FLAG_DRIVE_ACTIVE; /* Missing positive flag. */
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_HARDWARE_FAULT);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_LEDGER_MISMATCH);
    assert(output.action == FOC_LSI_EXECUTOR_ACTION_SAFE);

    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_OK);
    raw = raw_sample(20U);
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_OK);
    raw = raw_sample(21U);
    raw.compare_u = output.compare_u;
    raw.compare_v = output.compare_v;
    raw.compare_w = output.compare_w;
    raw.flags |= FOC_LSI_RAW_FLAG_DRIVE_ACTIVE |
                 FOC_LSI_RAW_FLAG_PULSE_POSITIVE;
    request.capture_full = 1U;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_DISABLED);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_CAPTURE_COMPLETE);
    assert(output.ledger_matched == 1U);
    assert(output.action == FOC_LSI_EXECUTOR_ACTION_SAFE);
}

static void test_all_rejections_clear_pending_output(void)
{
    foc_lsi_executor_t executor;
    foc_lsi_actuation_config_t actuation = actuation_config();
    foc_lsi_executor_config_t platform = platform_config();
    foc_lsi_executor_command_t request = command(FOC_LSI_DRIVE_ABI_BIAS);
    foc_lsi_executor_output_t output;
    foc_lsi_raw_sample_t raw = raw_sample(1U);

    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_OK);
    request.capture_ready = 0U;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_NOT_CONFIGURED);
    assert(output.action == FOC_LSI_EXECUTOR_ACTION_SAFE);

    request.capture_ready = 1U;
    request.capture_error = 1U;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_HARDWARE_FAULT);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_CAPTURE_ERROR);

    request.capture_error = 0U;
    raw.flags |= FOC_LSI_RAW_FLAG_HARDWARE_FAULT;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_HARDWARE_FAULT);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_FAULT);

    raw = raw_sample(2U);
    raw.compare_u = 7084U;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_HARDWARE_FAULT);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_RAW_SAMPLE_INVALID);

    raw = raw_sample(3U);
    request.requested_perturbation_voltage_v = 0.123f;
    assert(foc_lsi_executor_step(&executor, &request, &raw, &output) ==
           FOC_STATUS_INVALID_ARGUMENT);
    assert(output.result == FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED);
    assert(output.action == FOC_LSI_EXECUTOR_ACTION_SAFE);
    assert(executor.pending_ledger_valid == 0U);

    platform.sample_rate_hz = 16000U;
    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_INVALID_ARGUMENT);
    platform = platform_config();
    actuation.maximum_bias_current_a = 0.21f;
    assert(foc_lsi_executor_init(&executor, &platform, &actuation) ==
           FOC_STATUS_INVALID_ARGUMENT);
}

int main(void)
{
    test_conversion_quantisation_and_ledger();
    test_ledger_polarity_and_capture_completion_fail_closed();
    test_all_rejections_clear_pending_output();
    puts("FOC LSI ACTUATION EXECUTOR: PASS");
    return 0;
}
