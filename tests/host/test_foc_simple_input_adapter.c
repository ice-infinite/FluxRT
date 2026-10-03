#include "foc_simple_input_adapter.h"

#include <assert.h>
#include <stdio.h>

uint32_t foc_rust_input_normalize_pwm(
    const foc_centered_input_config_t *config,
    uint32_t pulse_width_us,
    foc_input_normalization_output_t *output)
{
    (void)pulse_width_us;
    return ((config == 0) && (output == 0)) ?
        FOC_INPUT_STATUS_INVALID_ARGUMENT : FOC_INPUT_STATUS_OK;
}

uint32_t foc_rust_input_normalize_analog(
    const foc_centered_input_config_t *config,
    int32_t adc_counts,
    foc_input_normalization_output_t *output)
{
    (void)adc_counts;
    return ((config == 0) && (output == 0)) ?
        FOC_INPUT_STATUS_INVALID_ARGUMENT : FOC_INPUT_STATUS_OK;
}

uint32_t foc_rust_input_convert_step_dir(
    const foc_step_dir_input_config_t *config,
    int32_t accumulated_count,
    foc_input_normalization_output_t *output)
{
    (void)accumulated_count;
    return ((config == 0) && (output == 0)) ?
        FOC_INPUT_STATUS_INVALID_ARGUMENT : FOC_INPUT_STATUS_OK;
}

static foc_input_normalization_output_t normalized(float setpoint)
{
    foc_input_normalization_output_t value = {0};
    value.struct_size = sizeof(value);
    value.version = FOC_INPUT_OUTPUT_VERSION;
    value.status = FOC_INPUT_STATUS_OK;
    value.quality_flags = FOC_INPUT_QUALITY_CALIBRATED;
    value.raw_value = 1500;
    value.normalized_value = 0.5f;
    value.setpoint_si = setpoint;
    return value;
}

static void test_persistent_config_mapping_is_explicit(void)
{
    foc_external_io_config_t config;
    foc_centered_input_config_t centered;
    foc_step_dir_input_config_t step_dir;

    foc_external_io_default_config(&config);
    config.input_enable_mask =
        FOC_EXTERNAL_INPUT_PWM_PULSE | FOC_EXTERNAL_INPUT_STEP_DIR;
    config.inputs[0].control_mode = FOC_CONTROL_MODE_VELOCITY;
    config.inputs[3].control_mode = FOC_CONTROL_MODE_POSITION;
    config.simple_inputs.pwm.raw_min = 1000;
    config.simple_inputs.pwm.raw_neutral = 1500;
    config.simple_inputs.pwm.raw_max = 2000;
    config.simple_inputs.pwm.deadband = 20U;
    config.simple_inputs.pwm.negative_limit_si = 80.0f;
    config.simple_inputs.pwm.positive_limit_si = 100.0f;
    config.simple_inputs.step_dir.full_steps_per_revolution = 200U;
    config.simple_inputs.step_dir.microsteps = 16U;
    config.simple_inputs.step_dir.gear_numerator = 2U;
    config.simple_inputs.step_dir.gear_denominator = 1U;
    config.simple_inputs.step_dir.direction = -1;
    config.simple_inputs.step_dir.zero_count = 12;
    config.simple_inputs.step_dir.zero_position_rad = 0.25f;

    assert(foc_simple_input_adapter_map_centered_config(
               &config, FOC_EXTERNAL_INPUT_PWM_PULSE, &centered) ==
           FOC_SIMPLE_INPUT_ADAPTER_OK);
    assert(centered.control_mode == FOC_CONTROL_MODE_VELOCITY);
    assert(centered.raw_neutral == 1500);
    assert(centered.negative_limit_si == 80.0f);
    assert(centered.reserved == 0U);
    assert(foc_simple_input_adapter_map_centered_config(
               &config, FOC_EXTERNAL_INPUT_ANALOG, &centered) ==
           FOC_SIMPLE_INPUT_ADAPTER_INVALID_CONFIG);
    assert(centered.struct_size == 0U);

    assert(foc_simple_input_adapter_map_step_dir_config(
               &config, &step_dir) == FOC_SIMPLE_INPUT_ADAPTER_OK);
    assert(step_dir.full_steps_per_revolution == 200U);
    assert(step_dir.microsteps == 16U);
    assert(step_dir.direction == -1);
    assert(step_dir.reserved == 0U);
}

static void test_setpoint_only_and_wrapping_deadline(void)
{
    foc_input_normalization_output_t output = normalized(12.5f);
    foc_simple_input_candidate_t candidate;
    assert(foc_simple_input_adapter_build_candidate(
               FOC_EXTERNAL_INPUT_PWM_PULSE, 7U, 9U, 1234U,
               0xFFFFFFF0UL, 32U, FOC_CONTROL_MODE_VELOCITY,
               FOC_FEEDBACK_MODE_SENSORLESS, &output, &candidate) ==
           FOC_SIMPLE_INPUT_ADAPTER_OK);
    assert(candidate.sample.source_id == 7U);
    assert(candidate.sample.sequence == 9U);
    assert(candidate.command.command_kind == FOC_PRODUCT_COMMAND_SETPOINT);
    assert(candidate.command.axis_request == FOC_AXIS_REQUEST_NONE);
    assert(candidate.command.input_mode == FOC_INPUT_MODE_PASSTHROUGH);
    assert(candidate.command.velocity_ref_rad_s == 12.5f);
    assert(candidate.command.valid_until_ms == 0x00000010UL);
}

static void test_step_dir_is_position_only_and_bad_output_fails(void)
{
    foc_input_normalization_output_t output = normalized(3.0f);
    foc_simple_input_candidate_t candidate;
    assert(foc_simple_input_adapter_build_candidate(
               FOC_EXTERNAL_INPUT_STEP_DIR, 8U, 1U, 0U, 0U, 10U,
               FOC_CONTROL_MODE_VELOCITY, FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER,
               &output, &candidate) ==
           FOC_SIMPLE_INPUT_ADAPTER_INVALID_MAPPING);
    assert(foc_simple_input_adapter_build_candidate(
               FOC_EXTERNAL_INPUT_STEP_DIR, 8U, 1U, 0U, 0U, 10U,
               FOC_CONTROL_MODE_POSITION, FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER,
               &output, &candidate) ==
           FOC_SIMPLE_INPUT_ADAPTER_OK);
    assert(candidate.command.position_ref_rad == 3.0f);
    output.status = FOC_INPUT_STATUS_OUT_OF_RANGE;
    assert(foc_simple_input_adapter_build_candidate(
               FOC_EXTERNAL_INPUT_ANALOG, 8U, 2U, 0U, 0U, 10U,
               FOC_CONTROL_MODE_TORQUE, FOC_FEEDBACK_MODE_SENSORLESS,
               &output, &candidate) ==
           FOC_SIMPLE_INPUT_ADAPTER_NORMALIZATION_FAILED);
    assert(candidate.struct_size == 0U);
    assert(candidate.command.command_kind == 0U);
}

int main(void)
{
    assert(foc_simple_input_adapter_abi_self_check() ==
           FOC_SIMPLE_INPUT_ADAPTER_OK);
    test_persistent_config_mapping_is_explicit();
    test_setpoint_only_and_wrapping_deadline();
    test_step_dir_is_position_only_and_bad_output_fails();
    puts("foc simple input adapter tests passed");
    return 0;
}
