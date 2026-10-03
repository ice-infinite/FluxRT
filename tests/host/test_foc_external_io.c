#include "foc_external_io.h"

#include <assert.h>
#include <stdio.h>

static foc_external_io_capabilities_t capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_transport_mask =
        FOC_EXTERNAL_TRANSPORT_UART |
        FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC |
        FOC_EXTERNAL_TRANSPORT_CAN_FD;
    value.board_transport_mask = value.compiled_transport_mask;
    value.compiled_protocol_mask =
        FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE |
        FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN;
    value.compiled_input_mask =
        FOC_EXTERNAL_INPUT_PWM_PULSE | FOC_EXTERNAL_INPUT_ANALOG;
    value.board_input_mask = value.compiled_input_mask;
    value.transport_resource_masks[0] = 1UL << 0;
    value.transport_resource_masks[2] = 1UL << 1;
    value.transport_resource_masks[3] = 1UL << 1;
    value.input_resource_masks[0] = 1UL << 2;
    value.input_resource_masks[2] = 1UL << 3;
    return value;
}

static foc_external_source_policy_t source(uint32_t id)
{
    foc_external_source_policy_t value = {0};
    value.source_id = id;
    value.priority = 10U;
    value.permissions =
        (1UL << FOC_PRODUCT_COMMAND_RELEASE) |
        (1UL << FOC_PRODUCT_COMMAND_SETPOINT);
    value.lease_ms = 50U;
    value.command_timeout_ms = 100U;
    return value;
}

static foc_centered_input_calibration_config_t centered_calibration(void)
{
    foc_centered_input_calibration_config_t value = {0};
    value.raw_min = 1000;
    value.raw_neutral = 1500;
    value.raw_max = 2000;
    value.deadband = 20U;
    value.negative_limit_si = 1.0f;
    value.positive_limit_si = 1.0f;
    return value;
}

static foc_external_io_config_t valid_config(void)
{
    foc_external_io_config_t value;
    foc_external_io_default_config(&value);
    value.transport_enable_mask =
        FOC_EXTERNAL_TRANSPORT_UART | FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC;
    value.links[0].protocol = FOC_EXTERNAL_PROTOCOL_FLUXRT_NATIVE;
    value.links[0].nominal_bitrate = 115200U;
    value.links[2].protocol = FOC_EXTERNAL_PROTOCOL_DRONECAN;
    value.links[2].node_id = 42U;
    value.links[2].nominal_bitrate = 500000U;
    value.links[2].heartbeat_ms = 100U;
    value.links[2].source = source(20U);
    value.input_enable_mask = FOC_EXTERNAL_INPUT_PWM_PULSE;
    value.inputs[0].control_mode = FOC_CONTROL_MODE_TORQUE;
    value.inputs[0].failure_action = FOC_EXTERNAL_FAILURE_RELEASE;
    value.inputs[0].source = source(10U);
    value.simple_inputs.pwm = centered_calibration();
    return value;
}

static void test_default_is_off_and_valid(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t config;
    foc_external_io_validation_t result;
    foc_external_io_default_config(&config);
    assert(foc_external_io_config_is_default_off(&config) == 1U);
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_OK);
    assert(result.configured_source_count == 0U);
}

static void test_valid_multi_source_config(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t config = valid_config();
    foc_external_io_validation_t result;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_OK);
    assert(result.configured_source_count == 2U);
    assert(result.configured_transport_mask == config.transport_enable_mask);
    assert(result.configured_input_mask == config.input_enable_mask);
}

static void test_compile_board_and_can_mode_gates(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t config = valid_config();
    foc_external_io_validation_t result;

    caps.compiled_input_mask &=
        ~((uint32_t)FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_NOT_COMPILED);

    caps = capabilities();
    caps.board_input_mask &=
        ~((uint32_t)FOC_EXTERNAL_INPUT_PWM_PULSE);
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_BOARD_UNAVAILABLE);

    caps = capabilities();
    config.transport_enable_mask |= FOC_EXTERNAL_TRANSPORT_CAN_FD;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_INVALID_MAPPING);
    assert(result.field == FOC_EXTERNAL_IO_FIELD_CAN);
}

static void test_source_permission_and_resource_conflicts(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t config = valid_config();
    foc_external_io_validation_t result;

    config.inputs[0].source.source_id = 20U;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_DUPLICATE_SOURCE);

    config = valid_config();
    config.inputs[0].source.permissions |=
        1UL << FOC_PRODUCT_COMMAND_AXIS_REQUEST;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_INVALID_PERMISSION);

    config = valid_config();
    config.input_enable_mask |= FOC_EXTERNAL_INPUT_ANALOG;
    config.inputs[2].control_mode = FOC_CONTROL_MODE_VELOCITY;
    config.inputs[2].failure_action = FOC_EXTERNAL_FAILURE_CONTROLLED_STOP;
    config.inputs[2].source = source(30U);
    config.simple_inputs.analog = centered_calibration();
    caps.input_resource_masks[2] = caps.input_resource_masks[0];
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_RESOURCE_CONFLICT);
    assert(result.field == FOC_EXTERNAL_IO_FIELD_ANALOG);
    assert(result.conflict_resource_mask == (1UL << 2));
}

static void test_invalid_calibration_and_step_dir_mapping_fail_closed(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_external_io_config_t config = valid_config();
    foc_external_io_validation_t result;

    config.simple_inputs.pwm.raw_neutral = config.simple_inputs.pwm.raw_min;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_INVALID_CALIBRATION);
    assert(result.field == FOC_EXTERNAL_IO_FIELD_PWM);

    caps.compiled_input_mask |= FOC_EXTERNAL_INPUT_STEP_DIR;
    caps.board_input_mask |= FOC_EXTERNAL_INPUT_STEP_DIR;
    config = valid_config();
    config.input_enable_mask = FOC_EXTERNAL_INPUT_STEP_DIR;
    config.inputs[3].control_mode = FOC_CONTROL_MODE_VELOCITY;
    config.inputs[3].failure_action = FOC_EXTERNAL_FAILURE_HOLD;
    config.inputs[3].source = source(31U);
    config.simple_inputs.step_dir.full_steps_per_revolution = 200U;
    config.simple_inputs.step_dir.microsteps = 16U;
    config.simple_inputs.step_dir.gear_numerator = 1U;
    config.simple_inputs.step_dir.gear_denominator = 1U;
    config.simple_inputs.step_dir.direction = 1;
    config.simple_inputs.step_dir.maximum_step_rate_hz = 100000U;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_INVALID_MAPPING);
    config.inputs[3].control_mode = FOC_CONTROL_MODE_POSITION;
    assert(foc_external_io_validate_config(&caps, &config, &result) ==
           FOC_EXTERNAL_IO_STATUS_OK);
}

static void test_build_capabilities_do_not_invent_drivers(void)
{
    foc_external_io_capabilities_t caps;
    foc_external_io_build_capabilities(&caps);
    assert(caps.struct_size == sizeof(caps));
    assert(caps.version == FOC_EXTERNAL_IO_CAPABILITIES_VERSION);
    assert(caps.compiled_transport_mask == 0U);
    assert(caps.compiled_protocol_mask == 0U);
    assert(caps.compiled_input_mask == 0U);
}

int main(void)
{
    test_default_is_off_and_valid();
    test_valid_multi_source_config();
    test_compile_board_and_can_mode_gates();
    test_source_permission_and_resource_conflicts();
    test_invalid_calibration_and_step_dir_mapping_fail_closed();
    test_build_capabilities_do_not_invent_drivers();
    puts("foc external IO contract tests passed");
    return 0;
}
