#include <assert.h>
#include <stdint.h>

#include "foc_sensorless_bridge.h"
#include "foc_sensorless_platform.h"

int main(void)
{
    assert(FOC_SENSORLESS_REQUIRED_CAPABILITIES == 0x3FU);
    assert(FOC_SENSORLESS_INPUT_KNOWN_MASK == 0x0FU);
    assert(sizeof(foc_sensorless_runtime_config_t) == 144U);
    assert(sizeof(foc_sensorless_realtime_input_t) == 48U);
    assert(sizeof(foc_sensorless_realtime_output_t) == 132U);
    /* V3 appended chain_cycles; the earlier offsets are unchanged. */
    assert(FOC_SENSORLESS_OUTPUT_VERSION == 3U);
    assert(offsetof(foc_sensorless_realtime_output_t, chain_cycles) == 84U);
    assert(offsetof(foc_sensorless_realtime_output_t, chain_hfi_cycles) == 88U);
    assert(offsetof(foc_sensorless_realtime_output_t, chain_fusion_cycles) == 92U);
    assert(offsetof(foc_sensorless_realtime_output_t, chain_separator_cycles) == 96U);
    assert(offsetof(foc_sensorless_realtime_output_t, abi_prepare_cycles) == 100U);
    assert(offsetof(foc_sensorless_realtime_output_t, abi_checks_cycles) == 104U);
    assert(offsetof(foc_sensorless_realtime_output_t, abi_publish_cycles) == 108U);
    assert(offsetof(foc_sensorless_realtime_output_t, abi_tail_cycles) == 112U);
    /* V3 core attribution, appended after the chain split. */
    assert(offsetof(foc_sensorless_realtime_output_t, core_gate_cycles) == 116U);
    assert(offsetof(foc_sensorless_realtime_output_t, core_observer_cycles) == 120U);
    assert(offsetof(foc_sensorless_realtime_output_t, core_startup_cycles) == 124U);
    assert(offsetof(foc_sensorless_realtime_output_t,
                    core_current_loop_cycles) == 128U);
    assert(sizeof(foc_probe_decomp_tick_t) == 28U);
    assert(sizeof(foc_sensorless_voltage_input_t) == 48U);
    assert(sizeof(foc_sensorless_voltage_output_t) == 44U);
    assert(sizeof(foc_sensorless_composite_input_t) == 44U);
    /* V2 appended chain_status; the earlier offsets are unchanged. */
    assert(FOC_SENSORLESS_COMPOSITE_OUTPUT_VERSION == 2U);
    assert(sizeof(foc_sensorless_composite_output_t) == 28U);
    assert(offsetof(foc_sensorless_composite_output_t, chain_status) == 24U);
    assert(offsetof(foc_sensorless_composite_output_t,
                    applied_injection_beta_v) == 20U);
    assert(offsetof(foc_sensorless_realtime_input_t,
                    applied_injection_alpha_v) == 32U);
    assert(offsetof(foc_sensorless_realtime_output_t,
                    electrical_angle_rad) == 56U);
    return 0;
}
