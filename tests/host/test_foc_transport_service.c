#include "foc_transport_service.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static foc_external_io_capabilities_t capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_transport_mask = FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC;
    value.board_transport_mask = FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC;
    return value;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.axis_state = FOC_AXIS_STATE_DISABLED;
    return value;
}

static foc_transport_packet_t packet(uint32_t sequence)
{
    foc_transport_packet_t value;
    (void)memset(&value, 0, sizeof(value));
    value.struct_size = sizeof(value);
    value.version = FOC_TRANSPORT_PACKET_VERSION;
    value.transport = FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC;
    value.instance_id = 0U;
    value.sequence = sequence;
    value.length = 8U;
    value.payload[0] = (uint8_t)sequence;
    return value;
}

static void test_default_off_safe_enable_and_fifo(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_transport_service_t service;
    foc_transport_service_status_t status;
    foc_config_apply_guard_t guard = safe_guard();
    foc_transport_packet_t value;
    uint32_t index;

    assert(foc_transport_service_init(
               &service, &caps, FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC, 0U) ==
           FOC_TRANSPORT_SERVICE_OK);
    value = packet(1U);
    assert(foc_transport_service_receive_from_driver(&service, &value) ==
           FOC_TRANSPORT_SERVICE_DISABLED);
    guard.drive_active = 1U;
    assert(foc_transport_service_set_enabled(&service, 1U, &guard) ==
           FOC_TRANSPORT_SERVICE_UNSAFE_STATE);
    guard.drive_active = 0U;
    assert(foc_transport_service_set_enabled(&service, 1U, &guard) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_receive_from_driver(&service, &value) ==
           FOC_TRANSPORT_SERVICE_UNHEALTHY);
    assert(foc_transport_service_mark_healthy(&service, 1U) ==
           FOC_TRANSPORT_SERVICE_OK);

    for (index = 0U; index < FOC_TRANSPORT_QUEUE_CAPACITY; ++index)
    {
        value = packet(index + 1U);
        assert(foc_transport_service_receive_from_driver(&service, &value) ==
               FOC_TRANSPORT_SERVICE_OK);
    }
    value = packet(99U);
    assert(foc_transport_service_receive_from_driver(&service, &value) ==
           FOC_TRANSPORT_SERVICE_QUEUE_FULL);
    assert(foc_transport_service_get_status(&service, &status) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(status.rx_depth == FOC_TRANSPORT_QUEUE_CAPACITY);
    assert(status.rx_peak_depth == FOC_TRANSPORT_QUEUE_CAPACITY);
    assert(status.rx_frames == FOC_TRANSPORT_QUEUE_CAPACITY);
    assert(status.dropped == 1U);
    assert(status.overflow == 1U);

    for (index = 0U; index < FOC_TRANSPORT_QUEUE_CAPACITY; ++index)
    {
        assert(foc_transport_service_take_received(&service, &value) ==
               FOC_TRANSPORT_SERVICE_OK);
        assert(value.sequence == (index + 1U));
    }
    assert(foc_transport_service_take_received(&service, &value) ==
           FOC_TRANSPORT_SERVICE_EMPTY);
}

static void test_tx_and_fault_health(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_transport_service_t service;
    foc_transport_service_status_t status;
    foc_config_apply_guard_t guard = safe_guard();
    foc_transport_packet_t value = packet(7U);

    assert(foc_transport_service_init(
               &service, &caps, FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC, 0U) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_set_enabled(&service, 1U, &guard) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_mark_healthy(&service, 1U) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_send(&service, &value) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_take_for_driver(&service, &value) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_report_error(
               &service, FOC_TRANSPORT_ERROR_BUS_OFF) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_send(&service, &value) ==
           FOC_TRANSPORT_SERVICE_UNHEALTHY);
    assert(foc_transport_service_get_status(&service, &status) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(status.healthy == 0U);
    assert(status.bus_off == 1U);
    assert(status.last_error == FOC_TRANSPORT_ERROR_BUS_OFF);
    assert(foc_transport_service_set_enabled(&service, 0U, NULL) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_mark_healthy(&service, 1U) ==
           FOC_TRANSPORT_SERVICE_DISABLED);
}

int main(void)
{
    test_default_off_safe_enable_and_fifo();
    test_tx_and_fault_health();
    puts("foc transport service tests passed");
    return 0;
}
