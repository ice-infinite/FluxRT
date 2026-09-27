#include "foc_phase_voltage_quality.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

static foc_phase_voltage_quality_config_t valid_config(void)
{
    foc_phase_voltage_quality_config_t config = {0};
    config.struct_size = sizeof(config);
    config.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    config.adc_max_code = 4095U;
    config.low_saturation_enter_code = 50U;
    config.low_saturation_release_code = 100U;
    config.high_saturation_release_code = 3900U;
    config.high_saturation_enter_code = 4000U;
    config.max_sample_age_ticks = 2U;
    config.open_stuck_enter_delta_codes = 1U;
    config.open_release_delta_codes = 5U;
    config.open_confirm_samples = 2U;
    config.inconsistency_release_mv = 100U;
    config.inconsistency_enter_mv = 200U;
    config.inconsistency_confirm_samples = 2U;
    config.recovery_confirm_samples = 3U;
    return config;
}

static foc_phase_voltage_quality_sample_t valid_sample(uint32_t sequence)
{
    foc_phase_voltage_quality_sample_t sample = {0};
    sample.struct_size = sizeof(sample);
    sample.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    sample.sequence = sequence;
    sample.flags = FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED |
                   FOC_PHASE_VOLTAGE_SAMPLE_CALIBRATED |
                   FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID |
                   FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID;
    sample.phase_u_raw = 1000U;
    sample.phase_v_raw = 2000U;
    sample.phase_w_raw = 3000U;
    sample.phase_u_mv = 4000U;
    sample.phase_v_mv = 8000U;
    sample.phase_w_mv = 12000U;
    sample.command_u_mv = sample.phase_u_mv;
    sample.command_v_mv = sample.phase_v_mv;
    sample.command_w_mv = sample.phase_w_mv;
    return sample;
}

static void qualify_source(foc_phase_voltage_quality_t *quality,
                           const foc_phase_voltage_quality_config_t *config,
                           foc_phase_voltage_request_t source,
                           uint32_t first_sequence)
{
    foc_phase_voltage_quality_result_t result;
    foc_phase_voltage_quality_sample_t sample;
    uint32_t index;

    for (index = 0U; index < config->recovery_confirm_samples; ++index)
    {
        sample = valid_sample(first_sequence + index);
        assert(foc_phase_voltage_quality_evaluate(
                   quality, config, &sample, source, &result) == 1U);
    }
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.measured_eligible == 1U);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);
}

static void test_config_boundaries(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();

    assert(foc_phase_voltage_quality_config_is_valid(&config) == 1U);
    assert(foc_phase_voltage_quality_config_is_valid(0) == 0U);
    config.low_saturation_release_code =
        config.low_saturation_enter_code;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.high_saturation_release_code =
        config.high_saturation_enter_code;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.high_saturation_enter_code = config.adc_max_code + 1U;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.open_release_delta_codes =
        config.open_stuck_enter_delta_codes;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.open_confirm_samples = 0U;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.open_release_delta_codes = config.adc_max_code + 1U;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.inconsistency_enter_mv = config.inconsistency_release_mv;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
    config = valid_config();
    config.recovery_confirm_samples = 0U;
    assert(foc_phase_voltage_quality_config_is_valid(&config) == 0U);
}

static void test_fail_closed_states_and_counts(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample = valid_sample(0U);
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;
    foc_phase_voltage_quality_status_t status;

    foc_phase_voltage_quality_init(&quality);
    foc_phase_voltage_quality_get_status(&quality, &status);
    assert(status.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(status.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);

    assert(foc_phase_voltage_quality_evaluate(
               &quality, 0, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(result.fallback_active == 1U);
    assert(result.fallback_event_count == 1U);

    sample.flags &= ~((uint32_t)FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED);

    sample = valid_sample(0U);
    sample.flags &= ~((uint32_t)FOC_PHASE_VOLTAGE_SAMPLE_CALIBRATED);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED);
    assert(result.reason_mask == FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED);
    assert(result.fallback_sample_count == 3U);
    assert(result.fallback_event_count == 1U);

    sample = valid_sample(0U);
    sample.flags &=
        ~((uint32_t)FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
    assert(result.unavailable_active == 1U);
    assert(result.unavailable_sample_count == 1U);
    assert(result.unavailable_event_count == 1U);

    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, 0, FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(result.fallback_active == 0U);
    assert(result.unavailable_active == 0U);

    foc_phase_voltage_quality_get_status(&quality, &status);
    assert(status.evaluated_sample_count == 5U);
    assert(status.reason_counts[0] == 2U);
    assert(status.reason_counts[1] == 1U);
    assert(status.reason_counts[2] == 2U);
}

static void test_recovery_and_stale_hysteresis(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample;
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;

    foc_phase_voltage_quality_init(&quality);
    sample = valid_sample(0U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.reason_mask ==
           FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    sample = valid_sample(1U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    sample = valid_sample(2U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.reason_mask == 0U);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);
    assert(result.recovery_count == 1U);

    sample = valid_sample(3U);
    sample.age_ticks = config.max_sample_age_ticks;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);
    sample = valid_sample(4U);
    sample.age_ticks = config.max_sample_age_ticks + 1U;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_STALE);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    assert(result.fallback_event_count == 2U);

    sample = valid_sample(5U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.reason_mask ==
           FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
    sample = valid_sample(5U); /* duplicate sequence is stale */
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_STALE);
}

static void test_saturation_priority_and_release(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample;
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;

    foc_phase_voltage_quality_init(&quality);
    qualify_source(&quality, &config, FOC_PHASE_VOLTAGE_REQUEST_MEASURED, 0U);

    sample = valid_sample(3U);
    sample.phase_u_raw = (uint16_t)config.low_saturation_enter_code;
    sample.phase_v_raw = (uint16_t)config.high_saturation_enter_code;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_LOW_SATURATION);
    assert((result.reason_mask & FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION) != 0U);
    assert((result.reason_mask & FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION) != 0U);
    assert(result.affected_phase_mask ==
           (FOC_PHASE_VOLTAGE_PHASE_U | FOC_PHASE_VOLTAGE_PHASE_V));
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);

    sample = valid_sample(4U);
    sample.phase_u_raw = 75U;
    sample.phase_v_raw = 3950U;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    assert((result.reason_mask & FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION) != 0U);
    assert((result.reason_mask & FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION) != 0U);

    sample = valid_sample(5U);
    sample.phase_u_raw = (uint16_t)config.low_saturation_release_code;
    sample.phase_v_raw = (uint16_t)config.high_saturation_release_code;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.reason_mask ==
           FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS);
    sample = valid_sample(6U);
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
        &result);
    sample = valid_sample(7U);
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
        &result);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);
    assert(result.recovery_count == 2U);
}

static void test_open_suspect_confirmation_and_release(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample;
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;
    uint32_t sequence;

    foc_phase_voltage_quality_init(&quality);
    for (sequence = 0U; sequence < 3U; ++sequence)
    {
        sample = valid_sample(sequence);
        sample.expected_change_mask = FOC_PHASE_VOLTAGE_PHASE_U;
        sample.phase_u_raw = (uint16_t)(1000U + (10U * sequence));
        assert(foc_phase_voltage_quality_evaluate(
                   &quality, &config, &sample,
                   FOC_PHASE_VOLTAGE_REQUEST_HYBRID, &result) == 1U);
    }
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);

    sample = valid_sample(3U);
    sample.expected_change_mask = FOC_PHASE_VOLTAGE_PHASE_U;
    sample.phase_u_raw = 1020U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    sample.sequence = 4U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT);
    assert(result.affected_phase_mask == FOC_PHASE_VOLTAGE_PHASE_U);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);

    sample.sequence = 5U;
    sample.phase_u_raw = 1023U; /* between enter and release: hold latch */
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT);
    sample.sequence = 6U;
    sample.phase_u_raw = 1028U; /* release boundary */
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);
}

static void test_inconsistency_confirmation_and_release(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample;
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;

    foc_phase_voltage_quality_init(&quality);
    qualify_source(&quality, &config, FOC_PHASE_VOLTAGE_REQUEST_HYBRID, 0U);

    sample = valid_sample(3U);
    sample.phase_u_mv += 500U;
    sample.phase_v_mv += 500U;
    sample.phase_w_mv += 500U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);
    sample.sequence = 4U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);

    sample = valid_sample(5U);
    sample.phase_v_mv += config.inconsistency_enter_mv;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    sample.sequence = 6U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state ==
           FOC_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT);
    assert(result.affected_phase_mask == FOC_PHASE_VOLTAGE_PHASE_ALL);
    assert(result.selected_source ==
           FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL);

    sample.sequence = 7U;
    sample.phase_v_mv = sample.command_v_mv + 150U;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state ==
           FOC_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT);
    sample.sequence = 8U;
    sample.phase_v_mv = sample.command_v_mv +
                        config.inconsistency_release_mv;
    (void)foc_phase_voltage_quality_evaluate(
        &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
        &result);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.reason_mask ==
           FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS);
}

static void test_wrap_unknown_source_and_saturating_counters(void)
{
    foc_phase_voltage_quality_config_t config = valid_config();
    foc_phase_voltage_quality_sample_t sample = valid_sample(UINT32_MAX);
    foc_phase_voltage_quality_t quality;
    foc_phase_voltage_quality_result_t result;

    config.recovery_confirm_samples = 1U;
    foc_phase_voltage_quality_init(&quality);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    sample = valid_sample(0U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_MEASURED,
               &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_VALID);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED);

    sample = valid_sample(1U);
    quality.evaluated_sample_count = UINT32_MAX;
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample,
               (foc_phase_voltage_request_t)99U, &result) == 1U);
    assert(result.state == FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE);
    assert(result.selected_source == FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE);
    assert(result.evaluated_sample_count == UINT32_MAX);
    assert(foc_phase_voltage_quality_evaluate(
               0, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               &result) == 0U);
    assert(foc_phase_voltage_quality_evaluate(
               &quality, &config, &sample, FOC_PHASE_VOLTAGE_REQUEST_HYBRID,
               0) == 0U);
}

int main(void)
{
    test_config_boundaries();
    test_fail_closed_states_and_counts();
    test_recovery_and_stale_hysteresis();
    test_saturation_priority_and_release();
    test_open_suspect_confirmation_and_release();
    test_inconsistency_confirmation_and_release();
    test_wrap_unknown_source_and_saturating_counters();
    puts("FOC PHASE VOLTAGE QUALITY: PASS");
    return 0;
}
