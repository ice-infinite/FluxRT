#include "foc_transport_service.h"

#include <string.h>

static uint32_t foc_transport_service_guard_is_safe(
    const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

static uint32_t foc_transport_service_is_single_transport(uint32_t transport)
{
    return ((transport != 0U) &&
            ((transport & (transport - 1U)) == 0U) &&
            ((transport & ~((uint32_t)FOC_EXTERNAL_TRANSPORT_KNOWN_MASK)) == 0U)) ? 1U : 0U;
}

static uint32_t foc_transport_service_is_valid(
    const foc_transport_service_t *service)
{
    return ((service != 0) &&
            (service->struct_size == sizeof(*service)) &&
            (service->version == FOC_TRANSPORT_SERVICE_VERSION) &&
            (service->initialized != 0U)) ? 1U : 0U;
}

static void foc_transport_queue_init(foc_transport_packet_queue_t *queue)
{
    atomic_init(&queue->write_sequence, 0U);
    atomic_init(&queue->read_sequence, 0U);
    (void)memset(queue->packets, 0, sizeof(queue->packets));
}

static uint32_t foc_transport_queue_depth(
    const foc_transport_packet_queue_t *queue)
{
    uint32_t write_sequence = atomic_load_explicit(
        &queue->write_sequence, memory_order_acquire);
    uint32_t read_sequence = atomic_load_explicit(
        &queue->read_sequence, memory_order_acquire);
    return (uint32_t)(write_sequence - read_sequence);
}

static void foc_transport_update_peak(
    atomic_uint_least32_t *peak,
    uint32_t depth)
{
    uint_least32_t observed = atomic_load_explicit(peak, memory_order_relaxed);
    while ((observed < depth) &&
           !atomic_compare_exchange_weak_explicit(
               peak, &observed, depth,
               memory_order_relaxed, memory_order_relaxed))
    {
    }
}

static foc_transport_service_result_t foc_transport_queue_push(
    foc_transport_packet_queue_t *queue,
    const foc_transport_packet_t *packet,
    atomic_uint_least32_t *peak)
{
    uint32_t write_sequence = atomic_load_explicit(
        &queue->write_sequence, memory_order_relaxed);
    uint32_t read_sequence = atomic_load_explicit(
        &queue->read_sequence, memory_order_acquire);
    uint32_t depth = (uint32_t)(write_sequence - read_sequence);
    if (depth >= FOC_TRANSPORT_QUEUE_CAPACITY)
    {
        return FOC_TRANSPORT_SERVICE_QUEUE_FULL;
    }
    queue->packets[write_sequence % FOC_TRANSPORT_QUEUE_CAPACITY] = *packet;
    atomic_store_explicit(
        &queue->write_sequence, write_sequence + 1U, memory_order_release);
    foc_transport_update_peak(peak, depth + 1U);
    return FOC_TRANSPORT_SERVICE_OK;
}

static foc_transport_service_result_t foc_transport_queue_pop(
    foc_transport_packet_queue_t *queue,
    foc_transport_packet_t *packet)
{
    uint32_t read_sequence = atomic_load_explicit(
        &queue->read_sequence, memory_order_relaxed);
    uint32_t write_sequence = atomic_load_explicit(
        &queue->write_sequence, memory_order_acquire);
    if (read_sequence == write_sequence)
    {
        return FOC_TRANSPORT_SERVICE_EMPTY;
    }
    *packet = queue->packets[read_sequence % FOC_TRANSPORT_QUEUE_CAPACITY];
    atomic_store_explicit(
        &queue->read_sequence, read_sequence + 1U, memory_order_release);
    return FOC_TRANSPORT_SERVICE_OK;
}

static uint32_t foc_transport_packet_is_valid(
    const foc_transport_service_t *service,
    const foc_transport_packet_t *packet)
{
    return ((packet != 0) &&
            (packet->struct_size == sizeof(*packet)) &&
            (packet->version == FOC_TRANSPORT_PACKET_VERSION) &&
            (packet->transport == service->transport) &&
            (packet->instance_id == service->instance_id) &&
            (packet->length > 0U) &&
            (packet->length <= FOC_TRANSPORT_PACKET_PAYLOAD_SIZE)) ? 1U : 0U;
}

static void foc_transport_service_flush(foc_transport_service_t *service)
{
    foc_transport_queue_init(&service->rx);
    foc_transport_queue_init(&service->tx);
    atomic_store_explicit(&service->rx_peak_depth, 0U, memory_order_relaxed);
    atomic_store_explicit(&service->tx_peak_depth, 0U, memory_order_relaxed);
}

foc_transport_service_result_t foc_transport_service_init(
    foc_transport_service_t *service,
    const foc_external_io_capabilities_t *capabilities,
    foc_external_transport_mask_t transport,
    uint32_t instance_id)
{
    if ((service == 0) || (capabilities == 0) ||
        (capabilities->struct_size != sizeof(*capabilities)) ||
        (capabilities->version != FOC_EXTERNAL_IO_CAPABILITIES_VERSION) ||
        (foc_transport_service_is_single_transport(transport) == 0U) ||
        ((capabilities->compiled_transport_mask & transport) == 0U) ||
        ((capabilities->board_transport_mask & transport) == 0U))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(service, 0, sizeof(*service));
    service->struct_size = sizeof(*service);
    service->version = FOC_TRANSPORT_SERVICE_VERSION;
    service->transport = transport;
    service->instance_id = instance_id;
    service->initialized = 1U;
    atomic_init(&service->enabled, 0U);
    atomic_init(&service->healthy, 0U);
    atomic_init(&service->rx_peak_depth, 0U);
    atomic_init(&service->tx_peak_depth, 0U);
    atomic_init(&service->rx_frames, 0U);
    atomic_init(&service->tx_frames, 0U);
    atomic_init(&service->crc_errors, 0U);
    atomic_init(&service->parse_errors, 0U);
    atomic_init(&service->framing_errors, 0U);
    atomic_init(&service->dropped, 0U);
    atomic_init(&service->overflow, 0U);
    atomic_init(&service->bus_off, 0U);
    atomic_init(&service->watchdog, 0U);
    atomic_init(&service->driver_errors, 0U);
    atomic_init(&service->last_error, 0U);
    foc_transport_service_flush(service);
    return FOC_TRANSPORT_SERVICE_OK;
}

foc_transport_service_result_t foc_transport_service_set_enabled(
    foc_transport_service_t *service,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard)
{
    if ((foc_transport_service_is_valid(service) == 0U) || (enabled > 1U))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    if (enabled == 0U)
    {
        atomic_store_explicit(&service->healthy, 0U, memory_order_release);
        atomic_store_explicit(&service->enabled, 0U, memory_order_release);
        foc_transport_service_flush(service);
        return FOC_TRANSPORT_SERVICE_OK;
    }
    if (foc_transport_service_guard_is_safe(guard) == 0U)
    {
        return FOC_TRANSPORT_SERVICE_UNSAFE_STATE;
    }
    foc_transport_service_flush(service);
    atomic_store_explicit(&service->healthy, 0U, memory_order_release);
    atomic_store_explicit(&service->enabled, 1U, memory_order_release);
    return FOC_TRANSPORT_SERVICE_OK;
}

foc_transport_service_result_t foc_transport_service_mark_healthy(
    foc_transport_service_t *service,
    uint32_t healthy)
{
    if ((foc_transport_service_is_valid(service) == 0U) || (healthy > 1U))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    if (atomic_load_explicit(&service->enabled, memory_order_acquire) == 0U)
    {
        return FOC_TRANSPORT_SERVICE_DISABLED;
    }
    atomic_store_explicit(&service->healthy, healthy, memory_order_release);
    return FOC_TRANSPORT_SERVICE_OK;
}

static foc_transport_service_result_t foc_transport_service_push(
    foc_transport_service_t *service,
    foc_transport_packet_queue_t *queue,
    atomic_uint_least32_t *peak,
    atomic_uint_least32_t *frames,
    const foc_transport_packet_t *packet)
{
    foc_transport_service_result_t result;
    if ((foc_transport_service_is_valid(service) == 0U) ||
        (foc_transport_packet_is_valid(service, packet) == 0U))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    if (atomic_load_explicit(&service->enabled, memory_order_acquire) == 0U)
    {
        return FOC_TRANSPORT_SERVICE_DISABLED;
    }
    if (atomic_load_explicit(&service->healthy, memory_order_acquire) == 0U)
    {
        return FOC_TRANSPORT_SERVICE_UNHEALTHY;
    }
    result = foc_transport_queue_push(queue, packet, peak);
    if (result == FOC_TRANSPORT_SERVICE_QUEUE_FULL)
    {
        (void)atomic_fetch_add_explicit(
            &service->dropped, 1U, memory_order_relaxed);
        (void)atomic_fetch_add_explicit(
            &service->overflow, 1U, memory_order_relaxed);
        return result;
    }
    (void)atomic_fetch_add_explicit(frames, 1U, memory_order_relaxed);
    return FOC_TRANSPORT_SERVICE_OK;
}

foc_transport_service_result_t foc_transport_service_receive_from_driver(
    foc_transport_service_t *service,
    const foc_transport_packet_t *packet)
{
    if (service == 0)
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    return foc_transport_service_push(
        service, &service->rx, &service->rx_peak_depth,
        &service->rx_frames, packet);
}

foc_transport_service_result_t foc_transport_service_take_received(
    foc_transport_service_t *service,
    foc_transport_packet_t *packet)
{
    if ((foc_transport_service_is_valid(service) == 0U) || (packet == 0))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    return foc_transport_queue_pop(&service->rx, packet);
}

foc_transport_service_result_t foc_transport_service_send(
    foc_transport_service_t *service,
    const foc_transport_packet_t *packet)
{
    if (service == 0)
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    return foc_transport_service_push(
        service, &service->tx, &service->tx_peak_depth,
        &service->tx_frames, packet);
}

foc_transport_service_result_t foc_transport_service_take_for_driver(
    foc_transport_service_t *service,
    foc_transport_packet_t *packet)
{
    if ((foc_transport_service_is_valid(service) == 0U) || (packet == 0))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    return foc_transport_queue_pop(&service->tx, packet);
}

foc_transport_service_result_t foc_transport_service_report_error(
    foc_transport_service_t *service,
    foc_transport_error_t error)
{
    atomic_uint_least32_t *counter = 0;
    uint32_t unhealthy = 0U;
    if (foc_transport_service_is_valid(service) == 0U)
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    switch (error)
    {
    case FOC_TRANSPORT_ERROR_CRC:
        counter = &service->crc_errors;
        break;
    case FOC_TRANSPORT_ERROR_PARSE:
        counter = &service->parse_errors;
        break;
    case FOC_TRANSPORT_ERROR_FRAMING:
        counter = &service->framing_errors;
        break;
    case FOC_TRANSPORT_ERROR_BUS_OFF:
        counter = &service->bus_off;
        unhealthy = 1U;
        break;
    case FOC_TRANSPORT_ERROR_WATCHDOG:
        counter = &service->watchdog;
        unhealthy = 1U;
        break;
    case FOC_TRANSPORT_ERROR_DRIVER:
        counter = &service->driver_errors;
        unhealthy = 1U;
        break;
    default:
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    (void)atomic_fetch_add_explicit(counter, 1U, memory_order_relaxed);
    atomic_store_explicit(&service->last_error, error, memory_order_relaxed);
    if (unhealthy != 0U)
    {
        atomic_store_explicit(&service->healthy, 0U, memory_order_release);
    }
    return FOC_TRANSPORT_SERVICE_OK;
}

foc_transport_service_result_t foc_transport_service_get_status(
    const foc_transport_service_t *service,
    foc_transport_service_status_t *status)
{
    if ((foc_transport_service_is_valid(service) == 0U) || (status == 0))
    {
        return FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT;
    }
    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_TRANSPORT_SERVICE_VERSION;
    status->enabled = atomic_load_explicit(&service->enabled, memory_order_acquire);
    status->healthy = atomic_load_explicit(&service->healthy, memory_order_acquire);
    status->transport = service->transport;
    status->instance_id = service->instance_id;
    status->rx_depth = foc_transport_queue_depth(&service->rx);
    status->tx_depth = foc_transport_queue_depth(&service->tx);
    status->rx_peak_depth = atomic_load_explicit(
        &service->rx_peak_depth, memory_order_relaxed);
    status->tx_peak_depth = atomic_load_explicit(
        &service->tx_peak_depth, memory_order_relaxed);
    status->rx_frames = atomic_load_explicit(&service->rx_frames, memory_order_relaxed);
    status->tx_frames = atomic_load_explicit(&service->tx_frames, memory_order_relaxed);
    status->crc_errors = atomic_load_explicit(&service->crc_errors, memory_order_relaxed);
    status->parse_errors = atomic_load_explicit(&service->parse_errors, memory_order_relaxed);
    status->framing_errors = atomic_load_explicit(
        &service->framing_errors, memory_order_relaxed);
    status->dropped = atomic_load_explicit(&service->dropped, memory_order_relaxed);
    status->overflow = atomic_load_explicit(&service->overflow, memory_order_relaxed);
    status->bus_off = atomic_load_explicit(&service->bus_off, memory_order_relaxed);
    status->watchdog = atomic_load_explicit(&service->watchdog, memory_order_relaxed);
    status->driver_errors = atomic_load_explicit(
        &service->driver_errors, memory_order_relaxed);
    status->last_error = atomic_load_explicit(
        &service->last_error, memory_order_relaxed);
    return FOC_TRANSPORT_SERVICE_OK;
}
