#include "foc_external_io_board.h"
#include "foc_external_io_platform.h"

#include <assert.h>

int main(void)
{
    foc_external_io_capabilities_t capabilities;
    foc_external_io_platform_capabilities(&capabilities);

    assert(capabilities.struct_size == sizeof(capabilities));
    assert(capabilities.version == FOC_EXTERNAL_IO_CAPABILITIES_VERSION);
    assert(capabilities.board_transport_mask == 0U);
    assert(capabilities.board_input_mask == 0U);
    assert((capabilities.reserved_resource_mask &
            FOC_G431_RESOURCE_TIM1_POWER_PWM) != 0U);
    assert((capabilities.reserved_resource_mask &
            FOC_G431_RESOURCE_ADC12_CURRENT_BUS) != 0U);
    assert(capabilities.transport_resource_masks[0] ==
           FOC_G431_RESOURCE_LPUART1_PA2_PA3);
    assert(capabilities.input_resource_masks[2] ==
           FOC_G431_RESOURCE_ADC1_REGULAR_PC2);
    assert((capabilities.reserved_resource_mask &
            FOC_G431_RESOURCE_ADC1_REGULAR_PC2) == 0U);
    assert((capabilities.reserved_resource_mask &
            FOC_G431_RESOURCE_IHM16_CURRENT_REF_PB4) != 0U);
    assert((capabilities.input_resource_masks[0] &
            FOC_G431_RESOURCE_IHM16_CURRENT_REF_PB4) != 0U);
    return 0;
}
