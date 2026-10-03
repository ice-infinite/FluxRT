#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_product_contract.h"
#include "foc_advanced_bridge.h"
#include "foc_power_bridge.h"
#include "foc_rust_bridge.h"

static foc_product_command_t legacy_speed_setpoint(float target_rpm)
{
    foc_product_command_t command;
    const float pi = 3.14159265358979323846f;

    memset(&command, 0, sizeof(command));
    command.struct_size = (uint32_t)sizeof(command);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.command_kind = (uint32_t)FOC_PRODUCT_COMMAND_SETPOINT;
    command.control_mode = (uint32_t)FOC_CONTROL_MODE_VELOCITY;
    command.input_mode = (uint32_t)FOC_INPUT_MODE_PASSTHROUGH;
    command.feedback_mode = (uint32_t)FOC_FEEDBACK_MODE_SENSORLESS;
    command.velocity_ref_rad_s = target_rpm * pi / 30.0f;
    return command;
}

int main(void)
{
    assert(FOC_ADVANCED_ABI_VERSION == 0x00010000UL);
    assert(sizeof(foc_advanced_algorithm_config_t) == 128U);
    assert(sizeof(foc_advanced_runtime_config_t) == 144U);
    assert(sizeof(foc_advanced_telemetry_t) == 68U);
    assert(FOC_POWER_ABI_VERSION == 0x00010000UL);
    assert(sizeof(foc_power_runtime_config_t) == 80U);
    assert(sizeof(foc_power_input_t) == 36U);
    assert(sizeof(foc_power_output_t) == 76U);
    foc_product_command_t command = legacy_speed_setpoint(524.0f);
    foc_product_command_t start;
    float recovered_rpm;

    assert(FOC_RUST_ABI_VERSION == 0x00150000UL);
    assert(FOC_PRODUCT_CONTRACT_VERSION == 0x00010000UL);
    assert(FOC_AXIS_STATE_FAULT_LATCHED == 6);
    assert(FOC_AXIS_REQUEST_CLOSED_LOOP_CONTROL == 9);
    assert(FOC_CONTROL_MODE_CURRENT == 3);
    assert(FOC_CONTROL_MODE_POSITION == 6);
    assert(FOC_INPUT_MODE_TRAPEZOIDAL_TRAJECTORY == 5);
    assert(FOC_FEEDBACK_MODE_FUSED == 5);
    assert(FOC_PRODUCT_TELEMETRY_VALID_KNOWN_MASK == 0x1FFFUL);
    assert(sizeof(command) == 104U);
    assert(offsetof(foc_product_command_t, flags) == 48U);
    assert(offsetof(foc_product_command_t, velocity_ref_rad_s) == 76U);
    assert(sizeof(foc_product_feedback_snapshot_t) == 68U);
    assert(offsetof(foc_product_feedback_snapshot_t, mechanical_position_rad) == 48U);
    assert(sizeof(foc_product_telemetry_snapshot_t) == 120U);
    assert(offsetof(foc_product_telemetry_snapshot_t, dc_bus_voltage_v) == 112U);
    assert(sizeof(foc_product_fault_snapshot_t) == 72U);
    assert(offsetof(foc_product_fault_snapshot_t, dc_bus_voltage_v) == 56U);

    recovered_rpm = command.velocity_ref_rad_s * 30.0f /
                    3.14159265358979323846f;
    assert(fabsf(recovered_rpm - 524.0f) < 1.0e-4f);

    memset(&start, 0, sizeof(start));
    start.struct_size = (uint32_t)sizeof(start);
    start.version = FOC_PRODUCT_COMMAND_VERSION;
    start.command_kind = (uint32_t)FOC_PRODUCT_COMMAND_AXIS_REQUEST;
    start.axis_request = (uint32_t)FOC_AXIS_REQUEST_CLOSED_LOOP_CONTROL;
    start.feedback_mode = (uint32_t)FOC_FEEDBACK_MODE_SENSORLESS;
    assert(start.control_mode == (uint32_t)FOC_CONTROL_MODE_INACTIVE);
    assert(start.input_mode == (uint32_t)FOC_INPUT_MODE_INACTIVE);

    puts("foc product contract host tests passed");
    return 0;
}
