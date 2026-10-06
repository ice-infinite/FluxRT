#include "foc_encoder_realtime_bridge.h"

#include <assert.h>
#include <stddef.h>

int main(void)
{
    assert(FOC_ENCODER_REALTIME_ABI_VERSION == 0x00010000UL);
    assert(FOC_ENCODER_REALTIME_INPUT_VERSION == 1UL);
    assert(sizeof(foc_encoder_realtime_input_t) == 48U);
    assert(offsetof(foc_encoder_realtime_input_t, actual_dt_s) == 16U);
    assert(offsetof(foc_encoder_realtime_input_t,
                    mechanical_speed_rad_s) == 40U);
    return 0;
}
