#include "foc_phase_voltage_platform_adapter.h"

#include <limits.h>
#include <string.h>

static void foc_phase_voltage_adapter_increment(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

static uint32_t foc_phase_voltage_adapter_request_valid(uint32_t request)
{
    return (request <= FOC_PHASE_VOLTAGE_REQUEST_HYBRID) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_adapter_input_valid(
    const foc_phase_voltage_platform_input_t *input)
{
    if ((input == 0) ||
        (input->struct_size != sizeof(*input)) ||
        (input->version != FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION) ||
        (input->flags != FOC_PHASE_VOLTAGE_ADAPTER_INPUT_KNOWN_MASK) ||
        ((input->divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) &&
         (input->divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_ENABLED)) ||
        ((input->expected_change_mask &
          ~((uint32_t)FOC_PHASE_VOLTAGE_PHASE_ALL)) != 0U) ||
        (foc_phase_voltage_adapter_request_valid(input->requested_mode) == 0U))
    {
        return 0U;
    }
    return 1U;
}

static void foc_phase_voltage_adapter_init_output(
    foc_phase_voltage_platform_output_t *output)
{
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION;
    output->flags = FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT;
    output->model_source = FOC_PHASE_VOLTAGE_MODEL_NONE;
    output->observer_eligible = 0U;
}

void foc_phase_voltage_platform_adapter_init(
    foc_phase_voltage_platform_adapter_t *adapter)
{
    if (adapter == 0)
    {
        return;
    }
    memset(adapter, 0, sizeof(*adapter));
    adapter->struct_size = sizeof(*adapter);
    adapter->version = FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION;
    foc_phase_voltage_quality_init(&adapter->quality);
}

uint32_t foc_phase_voltage_platform_adapter_evaluate(
    foc_phase_voltage_platform_adapter_t *adapter,
    const foc_phase_voltage_quality_config_t *quality_config,
    const foc_phase_voltage_model_t *model,
    const foc_phase_voltage_platform_input_t *input,
    foc_phase_voltage_platform_output_t *output)
{
    foc_phase_voltage_quality_sample_t quality_sample = {0};
    const foc_phase_voltage_quality_sample_t *quality_sample_ptr = 0;
    /* A missing/unknown request is rejected by the adapter.  MEASURED is the
     * conservative quality-core default because a null sample then resolves
     * to UNAVAILABLE without relying on an out-of-range enum conversion. */
    foc_phase_voltage_request_t request =
        FOC_PHASE_VOLTAGE_REQUEST_MEASURED;
    uint32_t input_valid;
    uint32_t model_valid;
    uint32_t quality_config_valid;
    uint32_t stale_observed = 0U;
    uint32_t conversion_valid = 0U;

    if ((adapter == 0) || (output == 0))
    {
        return 0U;
    }
    if ((adapter->struct_size != sizeof(*adapter)) ||
        (adapter->version != FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION))
    {
        foc_phase_voltage_platform_adapter_init(adapter);
    }
    foc_phase_voltage_adapter_init_output(output);
    foc_phase_voltage_adapter_increment(&adapter->evaluated_sample_count);

    input_valid = foc_phase_voltage_adapter_input_valid(input);
    if (input_valid == 0U)
    {
        output->flags |= FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED;
        foc_phase_voltage_adapter_increment(&adapter->input_reject_count);
        if ((input != 0) &&
            (foc_phase_voltage_adapter_request_valid(
                input->requested_mode) != 0U))
        {
            request = (foc_phase_voltage_request_t)input->requested_mode;
        }
        (void)foc_phase_voltage_quality_evaluate(
            &adapter->quality, quality_config, 0, request, &output->quality);
        output->stale_sample_count = adapter->stale_sample_count;
        output->conversion_failure_count = adapter->conversion_failure_count;
        output->input_reject_count = adapter->input_reject_count;
        return 1U;
    }

    request = (foc_phase_voltage_request_t)input->requested_mode;
    output->phase_sequence = input->raw_sample.sequence;
    quality_config_valid =
        foc_phase_voltage_quality_config_is_valid(quality_config);
    if ((adapter->previous_sequence_valid != 0U) &&
        (input->raw_sample.sequence != (adapter->previous_sequence + 1U)))
    {
        stale_observed = 1U;
    }
    if ((quality_config_valid != 0U) &&
        (input->age_ticks > quality_config->max_sample_age_ticks))
    {
        stale_observed = 1U;
    }
    adapter->previous_sequence = input->raw_sample.sequence;
    adapter->previous_sequence_valid = 1U;
    if (stale_observed != 0U)
    {
        output->flags |= FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED;
        foc_phase_voltage_adapter_increment(&adapter->stale_sample_count);
    }

    quality_sample.struct_size = sizeof(quality_sample);
    quality_sample.version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    quality_sample.sequence = input->raw_sample.sequence;
    quality_sample.age_ticks = input->age_ticks;
    quality_sample.expected_change_mask = input->expected_change_mask;
    quality_sample.phase_u_raw = input->raw_sample.phase_u_raw;
    quality_sample.phase_v_raw = input->raw_sample.phase_v_raw;
    quality_sample.phase_w_raw = input->raw_sample.phase_w_raw;
    quality_sample.command_u_mv = input->command_u_mv;
    quality_sample.command_v_mv = input->command_v_mv;
    quality_sample.command_w_mv = input->command_w_mv;
    quality_sample.flags =
        FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID;

    model_valid = foc_phase_voltage_model_validate(model);
    if (model_valid != 0U)
    {
        output->flags |= FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID |
                         FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY;
        output->model_source = model->source;
        if ((quality_config_valid != 0U) &&
            (quality_config->adc_max_code == model->adc_max_code))
        {
            quality_sample.flags |= FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED;
            conversion_valid =
                foc_phase_voltage_raw_to_mv(
                    model,
                    (foc_phase_voltage_divider_mode_t)input->divider_mode,
                    input->raw_sample.phase_u_raw,
                    &output->phase_u_mv) &&
                foc_phase_voltage_raw_to_mv(
                    model,
                    (foc_phase_voltage_divider_mode_t)input->divider_mode,
                    input->raw_sample.phase_v_raw,
                    &output->phase_v_mv) &&
                foc_phase_voltage_raw_to_mv(
                    model,
                    (foc_phase_voltage_divider_mode_t)input->divider_mode,
                    input->raw_sample.phase_w_raw,
                    &output->phase_w_mv);
            if (conversion_valid != 0U)
            {
                output->flags |=
                    FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID;
                quality_sample.flags |=
                    FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID;
                quality_sample.phase_u_mv = output->phase_u_mv;
                quality_sample.phase_v_mv = output->phase_v_mv;
                quality_sample.phase_w_mv = output->phase_w_mv;
            }
            else
            {
                output->phase_u_mv = 0U;
                output->phase_v_mv = 0U;
                output->phase_w_mv = 0U;
                foc_phase_voltage_adapter_increment(
                    &adapter->conversion_failure_count);
            }
        }
    }
    quality_sample_ptr = &quality_sample;
    (void)foc_phase_voltage_quality_evaluate(
        &adapter->quality, quality_config, quality_sample_ptr, request,
        &output->quality);

    /* Adapter v1 has no per-board calibration representation.  Its model
     * validator accepts only ST nominal/diagnostic models and the quality
     * sample above never asserts CALIBRATED.  Therefore A22 itself must
     * produce an uncalibrated, non-measured result.  Do not repair the result
     * here: post-evaluation mutation would desynchronise A22 status/counters. */
    output->observer_eligible = 0U;
    output->stale_sample_count = adapter->stale_sample_count;
    output->conversion_failure_count = adapter->conversion_failure_count;
    output->input_reject_count = adapter->input_reject_count;
    return 1U;
}
