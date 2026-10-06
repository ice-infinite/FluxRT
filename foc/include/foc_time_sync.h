#ifndef FOC_TIME_SYNC_H
#define FOC_TIME_SYNC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Target-neutral H3 time-anchor contract. It intentionally carries no GPIO,
 * timer-register or RTOS type. A platform may submit an edge only after a
 * separate mechanism has verified its session, sequence and tag identity. */
#define FOC_TIME_SYNC_ABI_VERSION      (0x00010000UL)
#define FOC_TIME_SYNC_EVENT_VERSION    (1UL)
#define FOC_TIME_SYNC_IDENTITY_VERSION (1UL)
#define FOC_TIME_SYNC_CAPTURE_CAPACITY (16UL)
#define FOC_TIME_SYNC_NO_CONTROL_TICK  (0xFFFFFFFFUL)

typedef enum
{
    FOC_TIME_SYNC_SOURCE_INVALID = 0,
    FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE = 1,
    FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR = 2,
} foc_time_sync_source_t;

typedef enum
{
    FOC_TIME_SYNC_CAPTURE_IDLE = 0,
    FOC_TIME_SYNC_CAPTURE_ARMED = 1,
    FOC_TIME_SYNC_CAPTURE_COMPLETE = 2,
    FOC_TIME_SYNC_CAPTURE_FAILED = 3,
} foc_time_sync_capture_state_t;

typedef enum
{
    FOC_TIME_SYNC_FAILURE_NONE = 0,
    FOC_TIME_SYNC_FAILURE_BAD_EVENT = 1,
    FOC_TIME_SYNC_FAILURE_SESSION_MISMATCH = 2,
    FOC_TIME_SYNC_FAILURE_SEQUENCE_MISMATCH = 3,
    FOC_TIME_SYNC_FAILURE_SOURCE_MISMATCH = 4,
    FOC_TIME_SYNC_FAILURE_QUEUE_FULL = 5,
    FOC_TIME_SYNC_FAILURE_LOCAL_TICK = 6,
    FOC_TIME_SYNC_FAILURE_CONTROL_TICK = 7,
} foc_time_sync_failure_t;

#define FOC_TIME_SYNC_FLAG_RISING_EDGE       (1UL << 0)
#define FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED (1UL << 1)
#define FOC_TIME_SYNC_FLAG_ALLOWED_MASK                                      \
    (FOC_TIME_SYNC_FLAG_RISING_EDGE | FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t session_id;
    uint32_t edge_sequence;
    uint32_t edge_tag;
    uint32_t flags;
} foc_time_sync_edge_identity_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t session_id;
    uint32_t edge_sequence;
    uint32_t edge_tag;
    uint32_t source;
    uint32_t local_tick_low;
    uint32_t local_tick_high;
    uint32_t control_tick;
    uint32_t flags;
} foc_time_sync_event_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t failure_reason;
    uint32_t source;
    uint32_t session_id;
    uint32_t next_edge_sequence;
    uint32_t recorded_count;
    uint32_t unread_count;
    uint32_t overflow_count;
    uint32_t rejected_count;
    uint32_t port_error_count;
} foc_time_sync_status_t;

/* One ISR producer and one thread/serializer consumer. Monotonic counters and
 * release/acquire publication prevent a consumer from observing a partial
 * event. Storage is fixed; full queues fail closed and never overwrite data. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    volatile uint32_t state;
    volatile uint32_t failure_reason;
    uint32_t source;
    uint32_t session_id;
    volatile uint32_t next_edge_sequence;
    volatile uint32_t write_count;
    volatile uint32_t read_count;
    volatile uint32_t overflow_count;
    volatile uint32_t rejected_count;
    volatile uint32_t port_error_count;
    foc_time_sync_event_t events[FOC_TIME_SYNC_CAPTURE_CAPACITY];
} foc_time_sync_capture_t;

typedef uint32_t (*foc_time_sync_read_local_tick_isr_fn)(
    void *context,
    uint32_t *tick_low,
    uint32_t *tick_high);
typedef uint32_t (*foc_time_sync_read_control_tick_isr_fn)(
    void *context,
    uint32_t *control_tick);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t source;
    foc_time_sync_capture_t *capture;
    void *port_context;
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr;
    foc_time_sync_read_control_tick_isr_fn read_control_tick_isr;
} foc_time_sync_adapter_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_time_sync_edge_identity_t) == 24U,
               "time-sync identity ABI size mismatch");
_Static_assert(sizeof(foc_time_sync_event_t) == 40U,
               "time-sync event ABI size mismatch");
_Static_assert(offsetof(foc_time_sync_event_t, local_tick_low) == 24U,
               "time-sync local tick ABI offset mismatch");
_Static_assert(offsetof(foc_time_sync_event_t, control_tick) == 32U,
               "time-sync control tick ABI offset mismatch");
_Static_assert(sizeof(foc_time_sync_status_t) == 48U,
               "time-sync status ABI size mismatch");
#endif

void foc_time_sync_capture_init(foc_time_sync_capture_t *capture);
uint32_t foc_time_sync_capture_arm(foc_time_sync_capture_t *capture,
                                   uint32_t source,
                                   uint32_t session_id,
                                   uint32_t first_edge_sequence);
uint32_t foc_time_sync_capture_record_isr(
    foc_time_sync_capture_t *capture,
    const foc_time_sync_event_t *event);
void foc_time_sync_capture_fail_isr(foc_time_sync_capture_t *capture,
                                    uint32_t failure_reason);
void foc_time_sync_capture_stop(foc_time_sync_capture_t *capture);
uint32_t foc_time_sync_capture_pop(foc_time_sync_capture_t *capture,
                                   foc_time_sync_event_t *event);
void foc_time_sync_capture_get_status(const foc_time_sync_capture_t *capture,
                                      foc_time_sync_status_t *status);

/* Resolve a lower-priority edge IRQ against the latest higher-priority
 * control-sample timestamp. All values are modulo-u32; caller guarantees the
 * IRQ delay is less than half a wrap. */
uint32_t foc_time_sync_control_tick_at_edge(
    uint32_t edge_cycle_tick,
    uint32_t latest_sample_cycle_tick,
    uint32_t latest_control_tick);

uint32_t foc_time_sync_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    uint32_t source,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr,
    foc_time_sync_read_control_tick_isr_fn read_control_tick_isr);
uint32_t foc_time_sync_adapter_record_verified_edge_isr(
    foc_time_sync_adapter_t *adapter,
    const foc_time_sync_edge_identity_t *identity);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TIME_SYNC_H */
