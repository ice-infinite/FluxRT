#include "foc_input_bridge.h"
#include "foc_product_contract.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    foc_centered_input_config_t centered = {0};
    foc_step_dir_input_config_t step_dir = {0};
    foc_input_normalization_output_t output = {0};

    centered.struct_size = sizeof(centered);
    centered.version = FOC_INPUT_CONFIG_VERSION;
    centered.control_mode = FOC_CONTROL_MODE_TORQUE;
    step_dir.struct_size = sizeof(step_dir);
    step_dir.version = FOC_INPUT_CONFIG_VERSION;
    output.struct_size = sizeof(output);
    output.version = FOC_INPUT_OUTPUT_VERSION;

    assert(FOC_INPUT_ABI_VERSION == 0x00010000UL);
    assert(centered.struct_size == 40U);
    assert(step_dir.struct_size == 40U);
    assert(output.struct_size == 28U);
    assert(offsetof(foc_centered_input_config_t, negative_limit_si) == 32U);
    assert(offsetof(foc_input_normalization_output_t, setpoint_si) == 24U);
    assert(centered.control_mode == FOC_CONTROL_MODE_TORQUE);
    puts("foc input bridge layout tests passed");
    return 0;
}
