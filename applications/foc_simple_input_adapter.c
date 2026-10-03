#include "foc_simple_input_adapter.h"

#include <math.h>
#include <string.h>

foc_simple_input_adapter_result_t foc_simple_input_adapter_abi_self_check(void)
{
    if ((foc_rust_input_normalize_pwm(0, 0U, 0) !=
         FOC_INPUT_STATUS_INVALID_ARGUMENT) ||
        (foc_rust_input_normalize_analog(0, 0, 0) !=
         FOC_INPUT_STATUS_INVALID_ARGUMENT) ||
        (foc_rust_input_convert_step_dir(0, 0, 0) !=
         FOC_INPUT_STATUS_INVALID_ARGUMENT))
    {
        return FOC_SIMPLE_INPUT_ADAPTER_ABI_MISMATCH;
    }
    return FOC_SIMPLE_INPUT_ADAPTER_OK;
}

static uint32_t foc_simple_input_is_known(
    foc_external_input_mask_t input)
{
    return ((input == FOC_EXTERNAL_INPUT_PWM_PULSE) ||
            (input == FOC_EXTERNAL_INPUT_ANALOG) ||
            (input == FOC_EXTERNAL_INPUT_STEP_DIR)) ? 1U : 0U;
}

static uint32_t foc_simple_input_mode_is_valid(
    foc_external_input_mask_t input,
    foc_control_mode_t control_mode)
{
    if (input == FOC_EXTERNAL_INPUT_STEP_DIR)
    {
        return (control_mode == FOC_CONTROL_MODE_POSITION) ? 1U : 0U;
    }
    return ((control_mode == FOC_CONTROL_MODE_TORQUE) ||
            (control_mode == FOC_CONTROL_MODE_VELOCITY) ||
            (control_mode == FOC_CONTROL_MODE_POSITION)) ? 1U : 0U;
}

static uint32_t foc_simple_input_config_layout_is_valid(
    const foc_external_io_config_t *config)
{
    return ((config != 0) &&
            (config->struct_size == sizeof(*config)) &&
            (config->version == FOC_EXTERNAL_IO_CONFIG_VERSION) &&
            (config->simple_inputs.struct_size ==
             sizeof(config->simple_inputs)) &&
            (config->simple_inputs.version ==
             FOC_SIMPLE_INPUT_CONFIG_VERSION)) ? 1U : 0U;
}

foc_simple_input_adapter_result_t foc_simple_input_adapter_map_centered_config(
    const foc_external_io_config_t *config,
    foc_external_input_mask_t input,
    foc_centered_input_config_t *output)
{
    const foc_centered_input_calibration_config_t *source;
    uint32_t input_index;

    if (output == 0)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_ARGUMENT;
    }
    (void)memset(output, 0, sizeof(*output));
    if (foc_simple_input_config_layout_is_valid(config) == 0U)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_CONFIG;
    }
    if (input == FOC_EXTERNAL_INPUT_PWM_PULSE)
    {
        input_index = 0U;
        source = &config->simple_inputs.pwm;
    }
    else if (input == FOC_EXTERNAL_INPUT_ANALOG)
    {
        input_index = 2U;
        source = &config->simple_inputs.analog;
    }
    else
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_MAPPING;
    }
    if ((config->input_enable_mask & input) == 0U)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_CONFIG;
    }

    output->struct_size = sizeof(*output);
    output->version = FOC_INPUT_CONFIG_VERSION;
    output->raw_min = source->raw_min;
    output->raw_neutral = source->raw_neutral;
    output->raw_max = source->raw_max;
    output->deadband = source->deadband;
    output->control_mode = config->inputs[input_index].control_mode;
    output->negative_limit_si = source->negative_limit_si;
    output->positive_limit_si = source->positive_limit_si;
    return FOC_SIMPLE_INPUT_ADAPTER_OK;
}

foc_simple_input_adapter_result_t foc_simple_input_adapter_map_step_dir_config(
    const foc_external_io_config_t *config,
    foc_step_dir_input_config_t *output)
{
    const foc_step_dir_calibration_config_t *source;

    if (output == 0)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_ARGUMENT;
    }
    (void)memset(output, 0, sizeof(*output));
    if ((foc_simple_input_config_layout_is_valid(config) == 0U) ||
        ((config->input_enable_mask & FOC_EXTERNAL_INPUT_STEP_DIR) == 0U))
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_CONFIG;
    }
    if (config->inputs[3].control_mode != FOC_CONTROL_MODE_POSITION)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_MAPPING;
    }

    source = &config->simple_inputs.step_dir;
    output->struct_size = sizeof(*output);
    output->version = FOC_INPUT_CONFIG_VERSION;
    output->full_steps_per_revolution = source->full_steps_per_revolution;
    output->microsteps = source->microsteps;
    output->gear_numerator = source->gear_numerator;
    output->gear_denominator = source->gear_denominator;
    output->direction = source->direction;
    output->zero_count = source->zero_count;
    output->zero_position_rad = source->zero_position_rad;
    return FOC_SIMPLE_INPUT_ADAPTER_OK;
}

foc_simple_input_adapter_result_t foc_simple_input_adapter_build_candidate(
    foc_external_input_mask_t input,
    uint32_t source_id,
    uint32_t sequence,
    uint32_t sampled_at_us,
    uint32_t now_ms,
    uint32_t timeout_ms,
    foc_control_mode_t control_mode,
    foc_feedback_mode_t feedback_mode,
    const foc_input_normalization_output_t *normalized,
    foc_simple_input_candidate_t *candidate)
{
    if (candidate == 0)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_ARGUMENT;
    }
    (void)memset(candidate, 0, sizeof(*candidate));
    if ((normalized == 0) ||
        (normalized->struct_size != sizeof(*normalized)) ||
        (normalized->version != FOC_INPUT_OUTPUT_VERSION) ||
        ((normalized->quality_flags &
          ~((uint32_t)FOC_INPUT_QUALITY_KNOWN_MASK)) != 0U) ||
        !isfinite(normalized->normalized_value) ||
        !isfinite(normalized->setpoint_si) ||
        (source_id == 0U) || (timeout_ms == 0U) ||
        (timeout_ms >= 0x80000000UL) ||
        (feedback_mode > FOC_FEEDBACK_MODE_FUSED) ||
        (foc_simple_input_is_known(input) == 0U))
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_ARGUMENT;
    }
    if (normalized->status != FOC_INPUT_STATUS_OK)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_NORMALIZATION_FAILED;
    }
    if (foc_simple_input_mode_is_valid(input, control_mode) == 0U)
    {
        return FOC_SIMPLE_INPUT_ADAPTER_INVALID_MAPPING;
    }

    candidate->struct_size = sizeof(*candidate);
    candidate->version = FOC_SIMPLE_INPUT_CANDIDATE_VERSION;
    candidate->sample.struct_size = sizeof(candidate->sample);
    candidate->sample.version = FOC_INPUT_SAMPLE_VERSION;
    candidate->sample.input = input;
    candidate->sample.instance_id = 0U;
    candidate->sample.source_id = source_id;
    candidate->sample.sequence = sequence;
    candidate->sample.sampled_at_us = sampled_at_us;
    candidate->sample.valid_flags =
        FOC_INPUT_SAMPLE_VALID_RAW | FOC_INPUT_SAMPLE_VALID_NORMALIZED;
    candidate->sample.quality_flags = FOC_INPUT_SAMPLE_QUALITY_CALIBRATED;
    candidate->sample.raw_value = normalized->raw_value;
    candidate->sample.normalized_value = normalized->normalized_value;

    candidate->command.struct_size = sizeof(candidate->command);
    candidate->command.version = FOC_PRODUCT_COMMAND_VERSION;
    candidate->command.axis_id = 0U;
    candidate->command.source_id = source_id;
    candidate->command.sequence = sequence;
    candidate->command.created_at_ms = now_ms;
    candidate->command.valid_until_ms = now_ms + timeout_ms;
    candidate->command.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    candidate->command.axis_request = FOC_AXIS_REQUEST_NONE;
    candidate->command.control_mode = control_mode;
    candidate->command.input_mode = FOC_INPUT_MODE_PASSTHROUGH;
    candidate->command.feedback_mode = feedback_mode;
    if (control_mode == FOC_CONTROL_MODE_TORQUE)
    {
        candidate->command.torque_ref_nm = normalized->setpoint_si;
    }
    else if (control_mode == FOC_CONTROL_MODE_VELOCITY)
    {
        candidate->command.velocity_ref_rad_s = normalized->setpoint_si;
    }
    else
    {
        candidate->command.position_ref_rad = normalized->setpoint_si;
    }
    return FOC_SIMPLE_INPUT_ADAPTER_OK;
}
