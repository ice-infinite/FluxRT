#ifndef FOC_NATIVE_FAKE_DRIVER_H
#define FOC_NATIVE_FAKE_DRIVER_H

/*
 * Allocation-free P2.6C bridge between the generic packet queues and the
 * read-only FluxRT Native service.  It is a host/target Fake driver: no UART,
 * USB or CAN registers, no MotorService dispatch and no power-stage ownership.
 */

#include <stdint.h>

#include "foc_native_bridge.h"
#include "foc_transport_service.h"

#define FOC_NATIVE_FAKE_DRIVER_VERSION (1UL)

typedef uint32_t foc_native_fake_driver_result_t;
enum
{
    FOC_NATIVE_FAKE_DRIVER_OK = 0,
    FOC_NATIVE_FAKE_DRIVER_IDLE = 1,
    FOC_NATIVE_FAKE_DRIVER_BACKPRESSURE = 2,
    FOC_NATIVE_FAKE_DRIVER_DISABLED = 3,
    FOC_NATIVE_FAKE_DRIVER_UNHEALTHY = 4,
    FOC_NATIVE_FAKE_DRIVER_INVALID_ARGUMENT = 5,
    FOC_NATIVE_FAKE_DRIVER_SNAPSHOT_ERROR = 6,
    FOC_NATIVE_FAKE_DRIVER_PROTOCOL_ERROR = 7,
};

typedef uint32_t (*foc_native_status_snapshot_fn)(
    void *context,
    foc_native_status_payload_t *status);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_native_status_snapshot_fn snapshot_status;
    void *context;
} foc_native_fake_driver_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t processed_frames;
    uint32_t rejected_frames;
    uint32_t snapshot_errors;
    uint32_t backpressure_events;
    uint32_t resets;
    uint32_t tx_packet_sequence;
    uint32_t rx_pending;
    uint32_t rx_offset;
    uint32_t tx_length;
    uint32_t tx_offset;
    foc_transport_service_t *transport;
    foc_native_fake_driver_ops_t ops;
    foc_native_fake_service_t service;
    foc_native_stream_decoder_t decoder;
    foc_transport_packet_t rx_packet;
    uint8_t tx_frame[FOC_NATIVE_MAX_FRAME_SIZE];
} foc_native_fake_driver_t;

foc_native_fake_driver_result_t foc_native_fake_driver_init(
    foc_native_fake_driver_t *driver,
    foc_transport_service_t *transport,
    const foc_native_identity_payload_t *identity,
    const foc_native_capabilities_payload_t *capabilities,
    const foc_native_status_payload_t *initial_status,
    const foc_native_fake_driver_ops_t *ops);

/* Drops partial protocol state only; transport queues remain owned by service. */
foc_native_fake_driver_result_t foc_native_fake_driver_reset(
    foc_native_fake_driver_t *driver);

/* Processes at most byte_budget RX bytes and never blocks on a full TX queue. */
foc_native_fake_driver_result_t foc_native_fake_driver_poll(
    foc_native_fake_driver_t *driver,
    uint32_t byte_budget);

#endif /* FOC_NATIVE_FAKE_DRIVER_H */
