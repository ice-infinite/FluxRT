/* Host-only safety tests for the EXP-B3 Ls(I) sequencer. */

#include "foc_lsi_identification.h"
#include "foc_rust_bridge.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 应用状态机和 Rust ABI 的数值映射必须逐项相同；两侧只能追加，不能重排。 */
_Static_assert((uint32_t)FOC_LSI_DRIVE_OFF ==
                   (uint32_t)FOC_LSI_DRIVE_ABI_OFF,
               "LSI OFF mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_BIAS ==
                   (uint32_t)FOC_LSI_DRIVE_ABI_BIAS,
               "LSI BIAS mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_PULSE_POSITIVE ==
                   (uint32_t)FOC_LSI_DRIVE_ABI_PULSE_POSITIVE,
               "LSI positive pulse mapping drifted");
_Static_assert((uint32_t)FOC_LSI_DRIVE_PULSE_NEGATIVE ==
                   (uint32_t)FOC_LSI_DRIVE_ABI_PULSE_NEGATIVE,
               "LSI negative pulse mapping drifted");

static foc_lsi_config_t test_config(void)
{
    foc_lsi_config_t config = {0};
    config.struct_size = sizeof(config);
    config.version = FOC_LSI_IDENTIFICATION_VERSION;
    config.sample_rate_hz = FOC_LSI_SAMPLE_RATE_HZ;
    config.offset_sample_count = 2U;
    config.bias_settle_ticks = 2U;
    config.pulse_ticks_per_polarity = 2U;
    config.pulse_pair_count = 2U;
    config.cooldown_zero_ticks = 2U;
    config.max_active_ticks = 20U;
    config.total_timeout_ticks = 40U;
    config.bias_current_a = 0.2f;
    config.perturbation_voltage_v = 0.4f;
    config.current_trip_a = 1.15f;
    config.cooldown_current_threshold_a = 0.05f;
    config.bus_voltage_min_v = 7.0f;
    config.bus_voltage_max_v = 18.0f;
    return config;
}

static foc_lsi_input_t safe_input(void)
{
    foc_lsi_input_t input = {0};
    input.struct_size = sizeof(input);
    input.version = FOC_LSI_INPUT_VERSION;
    input.identification_build_authorized = 1U;
    input.power_stage_idle = 1U;
    input.motor_stopped = 1U;
    input.offset_sample_valid = 1U;
    input.bus_voltage_v = 12.3f;
    input.abs_phase_current_a = 0.0f;
    return input;
}

static void assert_safe(const foc_lsi_output_t *output)
{
    assert(output->force_safe_output == 1U);
    assert(output->drive_request == FOC_LSI_DRIVE_OFF);
    assert(output->requested_bias_current_a == 0.0f);
    assert(output->requested_perturbation_voltage_v == 0.0f);
}

static void start_and_enter_offset(foc_lsi_context_t *context,
                                   foc_lsi_output_t *output,
                                   foc_lsi_input_t *input)
{
    foc_lsi_config_t config = test_config();
    assert(foc_lsi_init(context, &config) == 1U);
    assert(foc_lsi_request_start(context, FOC_LSI_START_CONFIRMATION) == 1U);
    assert(foc_lsi_step(context, input, output) == FOC_LSI_STATE_OFFSET_CAL);
    assert_safe(output);
    assert(output->capture_offset_sample == 1U);
}

static void advance_to(foc_lsi_context_t *context,
                       foc_lsi_output_t *output,
                       foc_lsi_input_t *input,
                       foc_lsi_state_t target)
{
    uint32_t guard = 0U;
    while ((context->state != target) && (guard < 64U))
    {
        (void)foc_lsi_step(context, input, output);
        ++guard;
    }
    assert(context->state == target);
}

static void test_configuration_gate(void)
{
    foc_lsi_config_t config = test_config();
    foc_lsi_context_t context;
    foc_lsi_output_t output;

    assert(foc_lsi_config_is_valid(&config) == 1U);
    config.sample_rate_hz = 16000U;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.bias_current_a = 0.61f;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.perturbation_voltage_v = 0.41f;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.current_trip_a = 1.16f;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.max_active_ticks = 9U;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.total_timeout_ticks = 6001U;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.bus_voltage_min_v = 6.9f;
    assert(foc_lsi_config_is_valid(&config) == 0U);
    config = test_config();
    config.bias_current_a = NAN;
    assert(foc_lsi_init(&context, &config) == 0U);
    assert(context.state == FOC_LSI_STATE_IDLE);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 0U);
    foc_lsi_get_output(&context, &output);
    assert_safe(&output);
}

static void test_complete_sequence_and_manual_rearm(void)
{
    foc_lsi_config_t config = test_config();
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    foc_lsi_input_t input = safe_input();

    assert(foc_lsi_init(&context, &config) == 1U);
    foc_lsi_get_output(&context, &output);
    assert(output.state == FOC_LSI_STATE_IDLE);
    assert_safe(&output);
    assert(foc_lsi_request_start(&context, 0U) == 0U);
    assert(context.state == FOC_LSI_STATE_IDLE);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
    foc_lsi_get_output(&context, &output);
    assert(output.state == FOC_LSI_STATE_PREFLIGHT);
    assert_safe(&output);

    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_OFFSET_CAL);
    assert_safe(&output);
    assert(output.capture_offset_sample == 1U);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_OFFSET_CAL);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_BIAS_SETTLE);
    assert(output.drive_request == FOC_LSI_DRIVE_BIAS);
    assert(output.force_safe_output == 0U);
    assert(output.capture_raw_sample == 1U);
    assert(output.requested_bias_current_a == 0.2f);

    input.abs_phase_current_a = 0.2f;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_BIAS_SETTLE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_POSITIVE);
    assert(output.drive_request == FOC_LSI_DRIVE_PULSE_POSITIVE);
    assert(output.requested_perturbation_voltage_v == 0.4f);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_POSITIVE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_NEGATIVE);
    assert(output.drive_request == FOC_LSI_DRIVE_PULSE_NEGATIVE);
    assert(output.requested_perturbation_voltage_v == -0.4f);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_NEGATIVE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_POSITIVE);
    assert(output.pulse_pair_index == 1U);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_POSITIVE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_NEGATIVE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_PULSE_NEGATIVE);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_COOLDOWN);
    assert(output.pulse_pair_index == 2U);
    assert_safe(&output);
    assert(output.capture_raw_sample == 1U);

    input.abs_phase_current_a = 0.04f;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_COOLDOWN);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_COMPLETE);
    assert_safe(&output);
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_COMPLETE);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 0U);
    assert(foc_lsi_reset(&context, &output) == 1U);
    assert(output.state == FOC_LSI_STATE_IDLE);
    assert_safe(&output);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
}

static void test_preflight_refusals(void)
{
    foc_lsi_config_t config = test_config();
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    foc_lsi_input_t input = safe_input();

    assert(foc_lsi_init(&context, &config) == 1U);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
    input.identification_build_authorized = 0U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_UNAUTHORIZED_BUILD);
    assert_safe(&output);

    assert(foc_lsi_reset(&context, &output) == 1U);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
    input.identification_build_authorized = 1U;
    input.power_stage_idle = 0U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_POWER_STAGE_NOT_IDLE);
    assert_safe(&output);

    assert(foc_lsi_reset(&context, &output) == 1U);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
    input.power_stage_idle = 1U;
    input.motor_stopped = 0U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_MOTOR_NOT_STOPPED);
    assert_safe(&output);
}

static void test_abort_paths_are_fail_closed(void)
{
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    foc_lsi_input_t input = safe_input();

    start_and_enter_offset(&context, &output, &input);
    input.hardware_fault = 1U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_HARDWARE_FAULT);
    assert_safe(&output);

    input = safe_input();
    start_and_enter_offset(&context, &output, &input);
    advance_to(&context, &output, &input, FOC_LSI_STATE_BIAS_SETTLE);
    input.software_trip = 1U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_SOFTWARE_TRIP);
    assert_safe(&output);

    input = safe_input();
    start_and_enter_offset(&context, &output, &input);
    advance_to(&context, &output, &input, FOC_LSI_STATE_PULSE_POSITIVE);
    input.abs_phase_current_a = 1.15f;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_CURRENT_LIMIT);
    assert_safe(&output);

    input = safe_input();
    start_and_enter_offset(&context, &output, &input);
    advance_to(&context, &output, &input, FOC_LSI_STATE_PULSE_NEGATIVE);
    input.abort_requested = 1U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_REQUESTED);
    assert_safe(&output);

    input = safe_input();
    start_and_enter_offset(&context, &output, &input);
    advance_to(&context, &output, &input, FOC_LSI_STATE_COOLDOWN);
    input.bus_voltage_v = 18.1f;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_BUS_VOLTAGE);
    assert_safe(&output);

    input = safe_input();
    start_and_enter_offset(&context, &output, &input);
    input.offset_sample_valid = 2U;
    assert(foc_lsi_step(&context, &input, &output) == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_INVALID_INPUT);
    assert_safe(&output);
}

static void test_operator_abort_from_every_stage(void)
{
    static const foc_lsi_state_t stages[] =
    {
        FOC_LSI_STATE_PREFLIGHT,
        FOC_LSI_STATE_OFFSET_CAL,
        FOC_LSI_STATE_BIAS_SETTLE,
        FOC_LSI_STATE_PULSE_POSITIVE,
        FOC_LSI_STATE_PULSE_NEGATIVE,
        FOC_LSI_STATE_COOLDOWN,
    };
    foc_lsi_config_t config = test_config();
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    foc_lsi_input_t input;
    uint32_t index;

    for (index = 0U; index < (sizeof(stages) / sizeof(stages[0])); ++index)
    {
        input = safe_input();
        assert(foc_lsi_init(&context, &config) == 1U);
        assert(foc_lsi_request_start(
                   &context, FOC_LSI_START_CONFIRMATION) == 1U);
        advance_to(&context, &output, &input, stages[index]);
        input.abort_requested = 1U;
        assert(foc_lsi_step(&context, &input, &output) ==
               FOC_LSI_STATE_ABORTED);
        assert(output.abort_reason == FOC_LSI_ABORT_REQUESTED);
        assert_safe(&output);
    }
}

static void test_total_timeout_is_fail_closed(void)
{
    foc_lsi_config_t config = test_config();
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    foc_lsi_input_t input = safe_input();
    uint32_t guard = 0U;

    config.total_timeout_ticks = 16U;
    assert(foc_lsi_init(&context, &config) == 1U);
    assert(foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION) == 1U);
    advance_to(&context, &output, &input, FOC_LSI_STATE_COOLDOWN);
    input.abs_phase_current_a = 0.2f;
    while ((context.state != FOC_LSI_STATE_ABORTED) && (guard < 32U))
    {
        (void)foc_lsi_step(&context, &input, &output);
        ++guard;
    }
    assert(context.state == FOC_LSI_STATE_ABORTED);
    assert(output.abort_reason == FOC_LSI_ABORT_TOTAL_TIMEOUT);
    assert_safe(&output);
    assert(foc_lsi_reset(&context, &output) == 1U);
    assert_safe(&output);
    assert(foc_lsi_reset(&context, &output) == 0U);
}

int main(void)
{
    test_configuration_gate();
    test_complete_sequence_and_manual_rearm();
    test_preflight_refusals();
    test_abort_paths_are_fail_closed();
    test_operator_abort_from_every_stage();
    test_total_timeout_is_fail_closed();
    puts("FOC LSI IDENTIFICATION STATE MACHINE: PASS");
    return 0;
}
