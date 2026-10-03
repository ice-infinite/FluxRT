#ifndef FOC_TRANSPORT_SERVICE_H
#define FOC_TRANSPORT_SERVICE_H

/*
 * Allocation-free transport queue used between one driver producer and one
 * service consumer.  Create one instance per physical transport instance.
 * RX and TX are independent SPSC queues; adding a second producer requires a
 * higher-level serializer rather than calling the same queue concurrently.
 */

#include <stdatomic.h>
#include <stdint.h>

#include "foc_config_bridge.h"
#include "foc_external_io.h"

#define FOC_TRANSPORT_SERVICE_VERSION       (1UL)
#define FOC_TRANSPORT_PACKET_VERSION        (1UL)
#define FOC_TRANSPORT_QUEUE_CAPACITY        (8UL)
#define FOC_TRANSPORT_PACKET_PAYLOAD_SIZE   (64UL)

typedef uint32_t foc_transport_service_result_t;
enum
{
    FOC_TRANSPORT_SERVICE_OK = 0,
    FOC_TRANSPORT_SERVICE_EMPTY = 1,
    FOC_TRANSPORT_SERVICE_DISABLED = 2,
    FOC_TRANSPORT_SERVICE_UNHEALTHY = 3,
    FOC_TRANSPORT_SERVICE_QUEUE_FULL = 4,
    FOC_TRANSPORT_SERVICE_INVALID_ARGUMENT = 5,
    FOC_TRANSPORT_SERVICE_UNSAFE_STATE = 6,
};

typedef uint32_t foc_transport_error_t;
enum
{
    FOC_TRANSPORT_ERROR_CRC = 1,
    FOC_TRANSPORT_ERROR_PARSE = 2,
    FOC_TRANSPORT_ERROR_FRAMING = 3,
    FOC_TRANSPORT_ERROR_BUS_OFF = 4,
    FOC_TRANSPORT_ERROR_WATCHDOG = 5,
    FOC_TRANSPORT_ERROR_DRIVER = 6,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_transport_mask_t transport;
    uint32_t instance_id;
    uint32_t timestamp_us;
    uint32_t sequence;
    uint32_t length;
    uint8_t payload[FOC_TRANSPORT_PACKET_PAYLOAD_SIZE];
} foc_transport_packet_t;

typedef struct
{
    atomic_uint_least32_t write_sequence;
    atomic_uint_least32_t read_sequence;
    foc_transport_packet_t packets[FOC_TRANSPORT_QUEUE_CAPACITY];
} foc_transport_packet_queue_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t enabled;
    uint32_t healthy;
    foc_external_transport_mask_t transport;
    uint32_t instance_id;
    uint32_t rx_depth;
    uint32_t tx_depth;
    uint32_t rx_peak_depth;
    uint32_t tx_peak_depth;
    uint32_t rx_frames;
    uint32_t tx_frames;
    uint32_t crc_errors;
    uint32_t parse_errors;
    uint32_t framing_errors;
    uint32_t dropped;
    uint32_t overflow;
    uint32_t bus_off;
    uint32_t watchdog;
    uint32_t driver_errors;
    foc_transport_error_t last_error;
} foc_transport_service_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    atomic_uint_least32_t enabled;
    atomic_uint_least32_t healthy;
    foc_external_transport_mask_t transport;
    uint32_t instance_id;
    foc_transport_packet_queue_t rx;
    foc_transport_packet_queue_t tx;
    atomic_uint_least32_t rx_peak_depth;
    atomic_uint_least32_t tx_peak_depth;
    atomic_uint_least32_t rx_frames;
    atomic_uint_least32_t tx_frames;
    atomic_uint_least32_t crc_errors;
    atomic_uint_least32_t parse_errors;
    atomic_uint_least32_t framing_errors;
    atomic_uint_least32_t dropped;
    atomic_uint_least32_t overflow;
    atomic_uint_least32_t bus_off;
    atomic_uint_least32_t watchdog;
    atomic_uint_least32_t driver_errors;
    atomic_uint_least32_t last_error;
} foc_transport_service_t;

foc_transport_service_result_t foc_transport_service_init(
    foc_transport_service_t *service,
    const foc_external_io_capabilities_t *capabilities,
    foc_external_transport_mask_t transport,
    uint32_t instance_id);

/* Enabling requires a fully Disabled/fault-free guard.  The driver must be
 * quiesced before disabling; disable flushes both queues and health. */
foc_transport_service_result_t foc_transport_service_set_enabled(
    foc_transport_service_t *service,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard);

foc_transport_service_result_t foc_transport_service_mark_healthy(
    foc_transport_service_t *service,
    uint32_t healthy);

foc_transport_service_result_t foc_transport_service_receive_from_driver(
    foc_transport_service_t *service,
    const foc_transport_packet_t *packet);
foc_transport_service_result_t foc_transport_service_take_received(
    foc_transport_service_t *service,
    foc_transport_packet_t *packet);

foc_transport_service_result_t foc_transport_service_send(
    foc_transport_service_t *service,
    const foc_transport_packet_t *packet);
foc_transport_service_result_t foc_transport_service_take_for_driver(
    foc_transport_service_t *service,
    foc_transport_packet_t *packet);

foc_transport_service_result_t foc_transport_service_report_error(
    foc_transport_service_t *service,
    foc_transport_error_t error);
foc_transport_service_result_t foc_transport_service_get_status(
    const foc_transport_service_t *service,
    foc_transport_service_status_t *status);

#endif /* FOC_TRANSPORT_SERVICE_H */
