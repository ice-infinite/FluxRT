#include "foc_native_fake_driver.h"

#include <string.h>

static uint32_t foc_native_fake_driver_is_valid(
    const foc_native_fake_driver_t *driver)
{
    return ((driver != 0) &&
            (driver->struct_size == sizeof(*driver)) &&
            (driver->version == FOC_NATIVE_FAKE_DRIVER_VERSION) &&
            (driver->initialized != 0U) &&
            (driver->transport != 0) &&
            (driver->ops.struct_size == sizeof(driver->ops)) &&
            (driver->ops.version == FOC_NATIVE_FAKE_DRIVER_VERSION) &&
            (driver->ops.snapshot_status != 0)) ? 1U : 0U;
}

static foc_native_fake_driver_result_t foc_native_fake_driver_transport_state(
    foc_native_fake_driver_t *driver)
{
    foc_transport_service_status_t status;
    if (foc_transport_service_get_status(driver->transport, &status) !=
        FOC_TRANSPORT_SERVICE_OK)
    {
        return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
    }
    if (status.enabled == 0U)
    {
        return FOC_NATIVE_FAKE_DRIVER_DISABLED;
    }
    if (status.healthy == 0U)
    {
        return FOC_NATIVE_FAKE_DRIVER_UNHEALTHY;
    }
    return FOC_NATIVE_FAKE_DRIVER_OK;
}

static foc_native_fake_driver_result_t foc_native_fake_driver_flush_tx(
    foc_native_fake_driver_t *driver)
{
    while (driver->tx_offset < driver->tx_length)
    {
        foc_transport_packet_t packet;
        uint32_t remaining = driver->tx_length - driver->tx_offset;
        uint32_t length = remaining;
        foc_transport_service_result_t result;
        if (length > FOC_TRANSPORT_PACKET_PAYLOAD_SIZE)
        {
            length = FOC_TRANSPORT_PACKET_PAYLOAD_SIZE;
        }
        (void)memset(&packet, 0, sizeof(packet));
        packet.struct_size = sizeof(packet);
        packet.version = FOC_TRANSPORT_PACKET_VERSION;
        packet.transport = driver->transport->transport;
        packet.instance_id = driver->transport->instance_id;
        packet.sequence = driver->tx_packet_sequence;
        packet.length = length;
        (void)memcpy(packet.payload, &driver->tx_frame[driver->tx_offset], length);
        result = foc_transport_service_send(driver->transport, &packet);
        if (result == FOC_TRANSPORT_SERVICE_QUEUE_FULL)
        {
            ++driver->backpressure_events;
            return FOC_NATIVE_FAKE_DRIVER_BACKPRESSURE;
        }
        if (result == FOC_TRANSPORT_SERVICE_DISABLED)
        {
            return FOC_NATIVE_FAKE_DRIVER_DISABLED;
        }
        if (result == FOC_TRANSPORT_SERVICE_UNHEALTHY)
        {
            return FOC_NATIVE_FAKE_DRIVER_UNHEALTHY;
        }
        if (result != FOC_TRANSPORT_SERVICE_OK)
        {
            return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
        }
        driver->tx_offset += length;
        ++driver->tx_packet_sequence;
    }
    driver->tx_length = 0U;
    driver->tx_offset = 0U;
    return FOC_NATIVE_FAKE_DRIVER_OK;
}

static foc_native_fake_driver_result_t foc_native_fake_driver_respond(
    foc_native_fake_driver_t *driver,
    const foc_native_message_t *request)
{
    foc_native_status_payload_t status;
    foc_native_message_t response;
    size_t frame_length = 0U;
    foc_native_status_t native_status;

    (void)memset(&status, 0, sizeof(status));
    if (driver->ops.snapshot_status(driver->ops.context, &status) == 0U)
    {
        ++driver->snapshot_errors;
        return FOC_NATIVE_FAKE_DRIVER_SNAPSHOT_ERROR;
    }
    native_status = foc_rust_native_fake_service_set_status(
        &driver->service, &status);
    if (native_status != FOC_NATIVE_STATUS_OK)
    {
        ++driver->snapshot_errors;
        return FOC_NATIVE_FAKE_DRIVER_SNAPSHOT_ERROR;
    }
    native_status = foc_rust_native_fake_service_handle(
        &driver->service, request, &response);
    if (native_status != FOC_NATIVE_STATUS_OK)
    {
        ++driver->rejected_frames;
        (void)foc_transport_service_report_error(
            driver->transport, FOC_TRANSPORT_ERROR_PARSE);
        return FOC_NATIVE_FAKE_DRIVER_PROTOCOL_ERROR;
    }
    native_status = foc_rust_native_encode(
        &response, driver->tx_frame, sizeof(driver->tx_frame), &frame_length);
    if ((native_status != FOC_NATIVE_STATUS_OK) ||
        (frame_length == 0U) || (frame_length > sizeof(driver->tx_frame)))
    {
        ++driver->rejected_frames;
        return FOC_NATIVE_FAKE_DRIVER_PROTOCOL_ERROR;
    }
    driver->tx_length = (uint32_t)frame_length;
    driver->tx_offset = 0U;
    ++driver->processed_frames;
    return foc_native_fake_driver_flush_tx(driver);
}

foc_native_fake_driver_result_t foc_native_fake_driver_init(
    foc_native_fake_driver_t *driver,
    foc_transport_service_t *transport,
    const foc_native_identity_payload_t *identity,
    const foc_native_capabilities_payload_t *capabilities,
    const foc_native_status_payload_t *initial_status,
    const foc_native_fake_driver_ops_t *ops)
{
    foc_native_status_t status;
    if ((driver == 0) || (transport == 0) || (identity == 0) ||
        (capabilities == 0) || (initial_status == 0) || (ops == 0) ||
        (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_NATIVE_FAKE_DRIVER_VERSION) ||
        (ops->snapshot_status == 0))
    {
        return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
    }
    (void)memset(driver, 0, sizeof(*driver));
    driver->struct_size = sizeof(*driver);
    driver->version = FOC_NATIVE_FAKE_DRIVER_VERSION;
    driver->transport = transport;
    driver->ops = *ops;
    status = foc_rust_native_stream_init(&driver->decoder);
    if (status == FOC_NATIVE_STATUS_OK)
    {
        status = foc_rust_native_fake_service_init(
            &driver->service, identity, capabilities, initial_status);
    }
    if (status != FOC_NATIVE_STATUS_OK)
    {
        return FOC_NATIVE_FAKE_DRIVER_PROTOCOL_ERROR;
    }
    driver->initialized = 1U;
    return FOC_NATIVE_FAKE_DRIVER_OK;
}

foc_native_fake_driver_result_t foc_native_fake_driver_reset(
    foc_native_fake_driver_t *driver)
{
    if (foc_native_fake_driver_is_valid(driver) == 0U)
    {
        return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
    }
    if (foc_rust_native_stream_init(&driver->decoder) != FOC_NATIVE_STATUS_OK)
    {
        return FOC_NATIVE_FAKE_DRIVER_PROTOCOL_ERROR;
    }
    driver->rx_pending = 0U;
    driver->rx_offset = 0U;
    driver->tx_length = 0U;
    driver->tx_offset = 0U;
    ++driver->resets;
    return FOC_NATIVE_FAKE_DRIVER_OK;
}

foc_native_fake_driver_result_t foc_native_fake_driver_poll(
    foc_native_fake_driver_t *driver,
    uint32_t byte_budget)
{
    uint32_t processed_bytes = 0U;
    foc_native_fake_driver_result_t result;
    if ((foc_native_fake_driver_is_valid(driver) == 0U) || (byte_budget == 0U))
    {
        return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
    }
    result = foc_native_fake_driver_transport_state(driver);
    if (result != FOC_NATIVE_FAKE_DRIVER_OK)
    {
        (void)foc_native_fake_driver_reset(driver);
        return result;
    }
    result = foc_native_fake_driver_flush_tx(driver);
    if (result != FOC_NATIVE_FAKE_DRIVER_OK)
    {
        return result;
    }

    while (processed_bytes < byte_budget)
    {
        if (driver->rx_pending == 0U)
        {
            foc_transport_service_result_t take =
                foc_transport_service_take_received(
                    driver->transport, &driver->rx_packet);
            if (take == FOC_TRANSPORT_SERVICE_EMPTY)
            {
                return (processed_bytes == 0U) ?
                    FOC_NATIVE_FAKE_DRIVER_IDLE : FOC_NATIVE_FAKE_DRIVER_OK;
            }
            if (take != FOC_TRANSPORT_SERVICE_OK)
            {
                return FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT;
            }
            driver->rx_pending = 1U;
            driver->rx_offset = 0U;
        }
        while ((driver->rx_offset < driver->rx_packet.length) &&
               (processed_bytes < byte_budget))
        {
            foc_native_message_t request;
            foc_native_status_t native_status = foc_rust_native_stream_push(
                &driver->decoder,
                driver->rx_packet.payload[driver->rx_offset],
                &request);
            ++driver->rx_offset;
            ++processed_bytes;
            if (native_status == FOC_NATIVE_STATUS_FRAME_READY)
            {
                result = foc_native_fake_driver_respond(driver, &request);
                if (result != FOC_NATIVE_FAKE_DRIVER_OK)
                {
                    return result;
                }
            }
            else if (native_status != FOC_NATIVE_STATUS_NEED_MORE)
            {
                ++driver->rejected_frames;
                (void)foc_transport_service_report_error(
                    driver->transport,
                    (native_status == FOC_NATIVE_STATUS_BAD_CRC) ?
                        FOC_TRANSPORT_ERROR_CRC : FOC_TRANSPORT_ERROR_FRAMING);
            }
        }
        if (driver->rx_offset >= driver->rx_packet.length)
        {
            driver->rx_pending = 0U;
            driver->rx_offset = 0U;
        }
    }
    return FOC_NATIVE_FAKE_DRIVER_OK;
}
