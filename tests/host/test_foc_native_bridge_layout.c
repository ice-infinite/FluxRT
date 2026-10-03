#include "foc_native_bridge.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(FOC_NATIVE_ABI_VERSION == 0x00010000UL);
    assert(FOC_NATIVE_MAX_FRAME_SIZE == 280UL);
    assert(FOC_NATIVE_MESSAGE_PRODUCT_COMMAND == 0x0031);
    assert(sizeof(foc_native_message_t) == 292U);
    assert(sizeof(foc_native_stream_decoder_t) == 316U);
    assert(sizeof(foc_native_identity_payload_t) == 48U);
    assert(sizeof(foc_native_capabilities_payload_t) == 64U);
    assert(sizeof(foc_native_status_payload_t) == 64U);
    assert(sizeof(foc_native_command_result_payload_t) == 32U);
    assert(sizeof(foc_native_fake_service_t) == 196U);
    assert(sizeof(foc_product_command_t) == 104U);
    assert(offsetof(foc_native_status_payload_t, dc_bus_voltage_v) == 48U);
    assert(offsetof(foc_native_fake_service_t, identity) == 8U);
    assert(offsetof(foc_native_fake_service_t, status) == 120U);
    puts("FluxRT Native C ABI layout tests passed");
    return 0;
}
