#include "foc_phase_voltage_platform_adapter.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static foc_phase_voltage_model_t nominal_model(void)
{
    foc_phase_voltage_model_t model = {0};
    model.struct_size = sizeof(model);
    model.version = FOC_PHASE_VOLTAGE_MODEL_VERSION;
    model.source = FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL;
    model.flags = FOC_PHASE_VOLTAGE_MODEL_FLAG_NOMINAL_COMPONENTS |
                  FOC_PHASE_VOLTAGE_MODEL_FLAG_DIAGNOSTIC_ONLY;
    model.adc_reference_mv = 3300U;
    model.adc_max_code = 4095U;
    model.divider_upper_ohm = 10000U;
    model.divider_lower_ohm = 2200U;
    model.phase_full_scale_mv = 18300U;
    model.volts_per_count_uv = 4469U;
    return model;
}

static foc_phase_voltage_quality_config_t quality_config(void)
{
    foc_phase_voltage_quality_config_t config = {0};
    config.struct_size = sizeof(config);
    config.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    config.adc_max_code = 4095U;
    config.low_saturation_enter_code = 8U;
    config.low_saturation_release_code = 16U;
    config.high_saturation_release_code = 4079U;
    config.high_saturation_enter_code = 4087U;
    config.max_sample_age_ticks = 2U;
    config.open_stuck_enter_delta_codes = 1U;
    config.open_release_delta_codes = 4U;
    config.open_confirm_samples = 2U;
    config.inconsistency_release_mv = 300U;
    config.inconsistency_enter_mv = 600U;
    config.inconsistency_confirm_samples = 2U;
    config.recovery_confirm_samples = 3U;
    return config;
}

static foc_phase_voltage_platform_input_t valid_input(
    uint32_t sequence,
    foc_phase_voltage_request_t request)
{
    foc_phase_voltage_platform_input_t input = {0};
    input.struct_size = sizeof(input);
    input.version = FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION;
    input.flags = FOC_PHASE_VOLTAGE_ADAPTER_INPUT_KNOWN_MASK;
    input.divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_ENABLED;
    input.requested_mode = (uint32_t)request;
    input.command_u_mv = 4469U;
    input.command_v_mv = 8938U;
    input.command_w_mv = 13407U;
    input.raw_sample.sequence = sequence;
    input.raw_sample.phase_u_raw = 1000U;
    input.raw_sample.phase_v_raw = 2000U;
    input.raw_sample.phase_w_raw = 3000U;
    return input;
}

static void assert_nominal_is_locked_out(
    const foc_phase_voltage_platform_output_t *output,
    foc_phase_voltage_request_t request)
{
    assert(output->observer_eligible == 0U);
    assert((output->flags &
            FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT) != 0U);
    assert(output->quality.state ==
           FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED);
    assert((output->quality.reason_mask &
            FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED) != 0U);
    assert(output->quality.measured_eligible == 0U);
    if (request == FOC_PHASE_VOLTAGE_REQUEST_MEASURED)
    {
        assert(output->quality.selected_source ==
               FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
        assert(output->quality.unavailable_active == 1U);
    }
    else if (request == FOC_PHASE_VOLTAGE_REQUEST_HYBRID)
    {
        assert(output->quality.selected_source ==
               FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
        assert(output->quality.fallback_active == 1U);
    }
    else
    {
        assert(output->quality.selected_source ==
               FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
        assert(output->quality.fallback_active == 0U);
        assert(output->quality.unavailable_active == 0U);
    }
}

static void assert_quality_result_matches_status(
    const foc_phase_voltage_platform_adapter_t *adapter,
    const foc_phase_voltage_platform_output_t *output)
{
    foc_phase_voltage_quality_status_t status;

    foc_phase_voltage_quality_get_status(&adapter->quality, &status);
    assert(output->quality.state == status.state);
    assert(output->quality.reason_mask == status.reason_mask);
    assert(output->quality.selected_source == status.selected_source);
    assert(output->quality.fallback_active == status.fallback_active);
    assert(output->quality.unavailable_active == status.unavailable_active);
    assert(output->quality.fallback_sample_count ==
           status.fallback_sample_count);
    assert(output->quality.fallback_event_count ==
           status.fallback_event_count);
    assert(output->quality.unavailable_sample_count ==
           status.unavailable_sample_count);
    assert(output->quality.unavailable_event_count ==
           status.unavailable_event_count);
}

static void test_init_and_empty_inputs_fail_closed(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input =
        valid_input(0U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);

    foc_phase_voltage_platform_adapter_init(&adapter);
    assert(adapter.struct_size == sizeof(adapter));
    assert(adapter.version == FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION);
    assert(adapter.quality.last_state ==
           FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);

    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, &config, &model, 0, &output) == 1U);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED) != 0U);
    assert(output.observer_eligible == 0U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);

    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, &config, 0, &input, &output) == 1U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(output.quality.fallback_active == 1U);
    assert(output.observer_eligible == 0U);

    input = valid_input(1U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, 0, &model, &input, &output) == 1U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(output.observer_eligible == 0U);

    assert(foc_phase_voltage_platform_adapter_evaluate(
               0, &config, &model, &input, &output) == 0U);
    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, &config, &model, &input, 0) == 0U);
}

static void test_nominal_conversion_is_diagnostic_only(void)
{
    foc_phase_voltage_platform_adapter_t adapter = {0};
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input =
        valid_input(0U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);

    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, &config, &model, &input, &output) == 1U);
    assert(adapter.struct_size == sizeof(adapter));
    assert(adapter.version == FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID) != 0U);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID) != 0U);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY) != 0U);
    assert(output.model_source ==
           FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL);
    assert(output.phase_u_mv == 4469U);
    assert(output.phase_v_mv == 8938U);
    assert(output.phase_w_mv == 13407U);
    assert_nominal_is_locked_out(&output,
                                 FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    assert(output.quality.unavailable_event_count == 1U);
    assert_quality_result_matches_status(&adapter, &output);
}

static void test_hybrid_never_recovers_to_measured(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input;
    uint32_t sequence;

    foc_phase_voltage_platform_adapter_init(&adapter);
    for (sequence = 0U; sequence < 8U; ++sequence)
    {
        input = valid_input(sequence, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
        assert(foc_phase_voltage_platform_adapter_evaluate(
                   &adapter, &config, &model, &input, &output) == 1U);
        assert_nominal_is_locked_out(&output,
                                     FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    }
    assert(output.quality.fallback_sample_count == 8U);
    assert(output.quality.fallback_event_count == 1U);
    assert(output.quality.consecutive_healthy_samples == 0U);

    input = valid_input(8U, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
    assert(foc_phase_voltage_platform_adapter_evaluate(
               &adapter, &config, &model, &input, &output) == 1U);
    assert_nominal_is_locked_out(&output,
                                 FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL);
}

static void test_sequence_wrap_drop_and_age(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input;

    foc_phase_voltage_platform_adapter_init(&adapter);
    input = valid_input(UINT32_MAX, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    input = valid_input(0U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED) == 0U);
    assert(output.stale_sample_count == 0U);

    input = valid_input(2U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED) != 0U);
    assert((output.quality.reason_mask &
            FOC_PHASE_VOLTAGE_REASON_STALE) == 0U);
    assert(output.quality.state ==
           FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED);
    assert(output.stale_sample_count == 1U);
    assert(adapter.quality.reason_counts[3] == 0U);
    assert_quality_result_matches_status(&adapter, &output);

    input = valid_input(3U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    input.age_ticks = config.max_sample_age_ticks + 1U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED) != 0U);
    assert((output.quality.reason_mask &
            FOC_PHASE_VOLTAGE_REASON_STALE) == 0U);
    assert(output.stale_sample_count == 2U);
    assert(adapter.quality.reason_counts[3] == 0U);
    assert_quality_result_matches_status(&adapter, &output);

    input = valid_input(4U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    input.age_ticks = config.max_sample_age_ticks;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED) == 0U);
    assert_nominal_is_locked_out(&output,
                                 FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
}

static void test_invalid_model_conversion_and_config_are_rejected(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input =
        valid_input(0U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);

    foc_phase_voltage_platform_adapter_init(&adapter);
    model.flags |= FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID) == 0U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
    assert(output.observer_eligible == 0U);

    model = nominal_model();
    model.source = FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED;
    model.flags = FOC_PHASE_VOLTAGE_MODEL_FLAG_BOARD_CALIBRATED |
                  FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE;
    input = valid_input(1U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID) == 0U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
    assert(output.observer_eligible == 0U);

    model = nominal_model();
    input = valid_input(2U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    input.divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_DISABLED;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID) == 0U);
    assert(output.conversion_failure_count == 1U);
    assert_nominal_is_locked_out(&output,
                                 FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    assert_quality_result_matches_status(&adapter, &output);

    input = valid_input(3U, FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    input.raw_sample.phase_u_raw = 4096U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID) == 0U);
    assert(output.conversion_failure_count == 2U);
    assert_nominal_is_locked_out(&output,
                                 FOC_PHASE_VOLTAGE_REQUEST_MEASURED);
    assert_quality_result_matches_status(&adapter, &output);

    input = valid_input(4U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    config.adc_max_code = 4094U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(output.observer_eligible == 0U);
}

static void test_atomic_input_flag_contract(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input;
    const uint32_t invalid_flags[] = {
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_RAW_VALID,
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_COMMAND_REFERENCE_VALID,
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_KNOWN_MASK | (1UL << 8),
        0U,
    };
    uint32_t index;

    foc_phase_voltage_platform_adapter_init(&adapter);
    for (index = 0U;
         index < (uint32_t)(sizeof(invalid_flags) / sizeof(invalid_flags[0]));
         ++index)
    {
        input = valid_input(index, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
        input.flags = invalid_flags[index];
        (void)foc_phase_voltage_platform_adapter_evaluate(
            &adapter, &config, &model, &input, &output);
        assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED) != 0U);
        assert(output.observer_eligible == 0U);
        assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
        assert(output.quality.selected_source ==
               FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
        assert_quality_result_matches_status(&adapter, &output);
    }
    assert(output.input_reject_count == 4U);
}

static void test_adapter_counters_saturate(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input =
        valid_input(2U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);

    foc_phase_voltage_platform_adapter_init(&adapter);
    adapter.evaluated_sample_count = UINT32_MAX;
    adapter.stale_sample_count = UINT32_MAX;
    adapter.conversion_failure_count = UINT32_MAX;
    adapter.input_reject_count = UINT32_MAX;
    adapter.previous_sequence = 0U;
    adapter.previous_sequence_valid = 1U;
    input.divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_DISABLED;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert(adapter.evaluated_sample_count == UINT32_MAX);
    assert(output.stale_sample_count == UINT32_MAX);
    assert(output.conversion_failure_count == UINT32_MAX);

    input = valid_input(3U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    input.flags = 0U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert(output.input_reject_count == UINT32_MAX);
}

static void test_malformed_and_unknown_mode_are_rejected(void)
{
    foc_phase_voltage_platform_adapter_t adapter;
    foc_phase_voltage_platform_output_t output;
    foc_phase_voltage_quality_config_t config = quality_config();
    foc_phase_voltage_model_t model = nominal_model();
    foc_phase_voltage_platform_input_t input =
        valid_input(0U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);

    foc_phase_voltage_platform_adapter_init(&adapter);
    input.flags = FOC_PHASE_VOLTAGE_ADAPTER_INPUT_RAW_VALID;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert((output.flags & FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED) != 0U);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(output.quality.fallback_event_count == 1U);

    input = valid_input(1U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    input.version += 1U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(output.observer_eligible == 0U);

    input = valid_input(2U, FOC_PHASE_VOLTAGE_REQUEST_HYBRID);
    input.requested_mode = 99U;
    (void)foc_phase_voltage_platform_adapter_evaluate(
        &adapter, &config, &model, &input, &output);
    assert(output.quality.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert((output.quality.reason_mask &
            FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE) != 0U);
    assert(output.quality.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
    assert(output.input_reject_count == 3U);
}

int main(void)
{
    test_init_and_empty_inputs_fail_closed();
    test_nominal_conversion_is_diagnostic_only();
    test_hybrid_never_recovers_to_measured();
    test_sequence_wrap_drop_and_age();
    test_invalid_model_conversion_and_config_are_rejected();
    test_malformed_and_unknown_mode_are_rejected();
    test_atomic_input_flag_contract();
    test_adapter_counters_saturate();
    puts("FOC PHASE VOLTAGE PLATFORM ADAPTER: PASS");
    return 0;
}
