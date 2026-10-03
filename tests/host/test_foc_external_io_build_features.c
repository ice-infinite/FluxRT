#include "foc_external_io.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    foc_external_io_capabilities_t caps;
    foc_external_io_build_capabilities(&caps);
    assert(caps.compiled_transport_mask ==
           (FOC_EXTERNAL_TRANSPORT_UART |
            FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC));
    assert(caps.compiled_protocol_mask ==
           (FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE |
            FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN));
    assert(caps.compiled_input_mask ==
           (FOC_EXTERNAL_INPUT_PWM_PULSE |
            FOC_EXTERNAL_INPUT_ANALOG |
            FOC_EXTERNAL_INPUT_STEP_DIR));
    /* Build features never invent physical board availability. */
    assert(caps.board_transport_mask == 0U);
    assert(caps.board_input_mask == 0U);
    puts("foc external IO build feature tests passed");
    return 0;
}
