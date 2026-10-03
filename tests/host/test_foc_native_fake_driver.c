#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_native_fake_driver.h"

static uint32_t g_snapshot_calls;

uint32_t foc_rust_native_protocol_version(void) { return FOC_NATIVE_ABI_VERSION; }

foc_native_status_t foc_rust_native_stream_init(
    foc_native_stream_decoder_t *decoder)
{
    memset(decoder, 0, sizeof(*decoder));
    decoder->struct_size = sizeof(*decoder);
    decoder->version = FOC_NATIVE_STREAM_VERSION;
    return FOC_NATIVE_STATUS_OK;
}

foc_native_status_t foc_rust_native_stream_push(
    foc_native_stream_decoder_t *decoder,
    uint8_t byte,
    foc_native_message_t *message)
{
    if ((decoder == 0) || (message == 0)) return FOC_NATIVE_STATUS_INVALID_ARGUMENT;
    if (byte != 0xAAU) return FOC_NATIVE_STATUS_NEED_MORE;
    memset(message, 0, sizeof(*message));
    message->struct_size = sizeof(*message);
    message->version = FOC_NATIVE_MESSAGE_VERSION;
    message->source_node = 42U;
    message->sequence = 7U;
    return FOC_NATIVE_STATUS_FRAME_READY;
}

foc_native_status_t foc_rust_native_fake_service_init(
    foc_native_fake_service_t *service,
    const foc_native_identity_payload_t *identity,
    const foc_native_capabilities_payload_t *capabilities,
    const foc_native_status_payload_t *status)
{
    memset(service, 0, sizeof(*service));
    service->struct_size = sizeof(*service);
    service->version = FOC_NATIVE_FAKE_SERVICE_VERSION;
    service->identity = *identity;
    service->capabilities = *capabilities;
    service->status = *status;
    return FOC_NATIVE_STATUS_OK;
}

foc_native_status_t foc_rust_native_fake_service_set_status(
    foc_native_fake_service_t *service,
    const foc_native_status_payload_t *status)
{
    service->status = *status;
    return FOC_NATIVE_STATUS_OK;
}

foc_native_status_t foc_rust_native_fake_service_handle(
    foc_native_fake_service_t *service,
    const foc_native_message_t *request,
    foc_native_message_t *response)
{
    (void)service;
    memset(response, 0, sizeof(*response));
    response->struct_size = sizeof(*response);
    response->version = FOC_NATIVE_MESSAGE_VERSION;
    response->destination_node = request->source_node;
    response->sequence = request->sequence;
    return FOC_NATIVE_STATUS_OK;
}

foc_native_status_t foc_rust_native_encode(
    const foc_native_message_t *message,
    uint8_t *frame,
    size_t frame_capacity,
    size_t *frame_length)
{
    uint32_t index;
    (void)message;
    if (frame_capacity < 130U) return FOC_NATIVE_STATUS_BUFFER_TOO_SMALL;
    for (index = 0U; index < 130U; ++index) frame[index] = (uint8_t)index;
    *frame_length = 130U;
    return FOC_NATIVE_STATUS_OK;
}

static uint32_t snapshot(void *context, foc_native_status_payload_t *status)
{
    (void)context;
    memset(status, 0, sizeof(*status));
    status->schema_version = FOC_NATIVE_PAYLOAD_SCHEMA_VERSION;
    ++g_snapshot_calls;
    return 1U;
}

static foc_external_io_capabilities_t capabilities(void)
{
    foc_external_io_capabilities_t value;
    foc_external_io_empty_capabilities(&value);
    value.compiled_transport_mask = FOC_EXTERNAL_TRANSPORT_UART;
    value.board_transport_mask = FOC_EXTERNAL_TRANSPORT_UART;
    return value;
}

static foc_config_apply_guard_t guard(void)
{
    foc_config_apply_guard_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.axis_state = FOC_AXIS_STATE_DISABLED;
    return value;
}

static foc_transport_packet_t packet(uint8_t byte, uint32_t sequence)
{
    foc_transport_packet_t value;
    memset(&value, 0, sizeof(value));
    value.struct_size = sizeof(value);
    value.version = FOC_TRANSPORT_PACKET_VERSION;
    value.transport = FOC_EXTERNAL_TRANSPORT_UART;
    value.instance_id = 0U;
    value.sequence = sequence;
    value.length = 1U;
    value.payload[0] = byte;
    return value;
}

int main(void)
{
    foc_external_io_capabilities_t caps = capabilities();
    foc_config_apply_guard_t safe = guard();
    foc_transport_service_t transport;
    foc_native_fake_driver_t driver;
    foc_native_identity_payload_t identity = {0};
    foc_native_capabilities_payload_t native_caps = {0};
    foc_native_status_payload_t initial = {0};
    const foc_native_fake_driver_ops_t ops = {
        sizeof(foc_native_fake_driver_ops_t), FOC_NATIVE_FAKE_DRIVER_VERSION,
        snapshot, 0,
    };
    foc_transport_packet_t value;
    uint32_t count = 0U;

    assert(foc_transport_service_init(
               &transport, &caps, FOC_EXTERNAL_TRANSPORT_UART, 0U) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_native_fake_driver_init(
               &driver, &transport, &identity, &native_caps, &initial, &ops) ==
           FOC_NATIVE_FAKE_DRIVER_OK);
    assert(foc_native_fake_driver_poll(&driver, 8U) ==
           FOC_NATIVE_FAKE_DRIVER_DISABLED);
    assert(driver.resets == 1U);
    assert(foc_transport_service_set_enabled(&transport, 1U, &safe) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_transport_service_mark_healthy(&transport, 1U) ==
           FOC_TRANSPORT_SERVICE_OK);
    value = packet(0xAAU, 1U);
    assert(foc_transport_service_receive_from_driver(&transport, &value) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_native_fake_driver_poll(&driver, 8U) ==
           FOC_NATIVE_FAKE_DRIVER_OK);
    assert(driver.processed_frames == 1U);
    assert(g_snapshot_calls == 1U);
    while (foc_transport_service_take_for_driver(&transport, &value) ==
           FOC_TRANSPORT_SERVICE_OK)
    {
        ++count;
    }
    assert(count == 3U);
    assert(driver.tx_length == 0U);

    /* Fill TX first; the complete response must remain pending, never dropped. */
    for (count = 0U; count < FOC_TRANSPORT_QUEUE_CAPACITY; ++count)
    {
        value = packet((uint8_t)count, count);
        assert(foc_transport_service_send(&transport, &value) ==
               FOC_TRANSPORT_SERVICE_OK);
    }
    value = packet(0xAAU, 2U);
    assert(foc_transport_service_receive_from_driver(&transport, &value) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_native_fake_driver_poll(&driver, 8U) ==
           FOC_NATIVE_FAKE_DRIVER_BACKPRESSURE);
    assert(driver.tx_length == 130U && driver.tx_offset == 0U);
    while (foc_transport_service_take_for_driver(&transport, &value) ==
           FOC_TRANSPORT_SERVICE_OK) {}
    assert(foc_native_fake_driver_poll(&driver, 8U) ==
           FOC_NATIVE_FAKE_DRIVER_IDLE);
    assert(driver.tx_length == 0U);

    assert(foc_transport_service_mark_healthy(&transport, 0U) ==
           FOC_TRANSPORT_SERVICE_OK);
    assert(foc_native_fake_driver_poll(&driver, 8U) ==
           FOC_NATIVE_FAKE_DRIVER_UNHEALTHY);
    assert(driver.resets >= 2U);
    puts("foc Native fake driver host tests passed");
    return 0;
}
