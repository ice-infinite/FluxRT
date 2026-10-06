#include <assert.h>
#include <stdint.h>

#include "foc_sensorless_bridge.h"

int main(void)
{
    assert(FOC_SENSORLESS_REQUIRED_CAPABILITIES == 0x3FU);
    assert(FOC_SENSORLESS_INPUT_KNOWN_MASK == 0x0FU);
    assert(sizeof(foc_sensorless_runtime_config_t) == 144U);
    assert(sizeof(foc_sensorless_realtime_input_t) == 48U);
    assert(sizeof(foc_sensorless_realtime_output_t) == 84U);
    assert(sizeof(foc_sensorless_voltage_input_t) == 48U);
    assert(sizeof(foc_sensorless_voltage_output_t) == 44U);
    assert(sizeof(foc_sensorless_composite_input_t) == 44U);
    assert(sizeof(foc_sensorless_composite_output_t) == 24U);
    assert(offsetof(foc_sensorless_realtime_input_t,
                    applied_injection_alpha_v) == 32U);
    assert(offsetof(foc_sensorless_realtime_output_t,
                    electrical_angle_rad) == 56U);
    return 0;
}
