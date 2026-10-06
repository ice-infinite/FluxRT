#include "foc_time_sync.h"

#include <string.h>

#if defined(__GNUC__) || defined(__clang__)
#define FOC_ATOMIC_LOAD_ACQUIRE(pointer)                                     \
    __atomic_load_n((pointer), __ATOMIC_ACQUIRE)
#define FOC_ATOMIC_LOAD_RELAXED(pointer)                                     \
    __atomic_load_n((pointer), __ATOMIC_RELAXED)
#define FOC_ATOMIC_STORE_RELEASE(pointer, value)                             \
    __atomic_store_n((pointer), (value), __ATOMIC_RELEASE)
#define FOC_ATOMIC_STORE_RELAXED(pointer, value)                             \
    __atomic_store_n((pointer), (value), __ATOMIC_RELAXED)
#else
/* Host MSVC fallback. FluxRT target compilers use GCC-compatible atomics. */
#define FOC_ATOMIC_LOAD_ACQUIRE(pointer) (*(pointer))
#define FOC_ATOMIC_LOAD_RELAXED(pointer) (*(pointer))
#define FOC_ATOMIC_STORE_RELEASE(pointer, value) (*(pointer) = (value))
#define FOC_ATOMIC_STORE_RELAXED(pointer, value) (*(pointer) = (value))
#endif

static uint32_t foc_time_sync_source_valid(uint32_t source)
{
    return (source == (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE) ||
           (source == (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR);
}

static uint32_t foc_time_sync_identity_valid(
    const foc_time_sync_edge_identity_t *identity)
{
    if ((identity == NULL) ||
        (identity->struct_size != sizeof(foc_time_sync_edge_identity_t)) ||
        (identity->version != FOC_TIME_SYNC_IDENTITY_VERSION) ||
        (identity->session_id == 0U) || (identity->edge_tag == 0U))
    {
        return 0U;
    }
    if ((identity->flags & FOC_TIME_SYNC_FLAG_ALLOWED_MASK) !=
        FOC_TIME_SYNC_FLAG_ALLOWED_MASK)
    {
        return 0U;
    }
    return ((identity->flags & ~FOC_TIME_SYNC_FLAG_ALLOWED_MASK) == 0U) ? 1U
                                                                        : 0U;
}

static uint32_t foc_time_sync_event_valid(const foc_time_sync_event_t *event)
{
    if ((event == NULL) ||
        (event->struct_size != sizeof(foc_time_sync_event_t)) ||
        (event->version != FOC_TIME_SYNC_EVENT_VERSION) ||
        (event->session_id == 0U) || (event->edge_tag == 0U) ||
        (foc_time_sync_source_valid(event->source) == 0U))
    {
        return 0U;
    }
    if ((event->flags & FOC_TIME_SYNC_FLAG_ALLOWED_MASK) !=
        FOC_TIME_SYNC_FLAG_ALLOWED_MASK)
    {
        return 0U;
    }
    if ((event->flags & ~FOC_TIME_SYNC_FLAG_ALLOWED_MASK) != 0U)
    {
        return 0U;
    }
    if ((event->source == (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR) &&
        (event->control_tick != FOC_TIME_SYNC_NO_CONTROL_TICK))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t foc_time_sync_saturating_increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : (value + 1U);
}

uint32_t foc_time_sync_control_tick_at_edge(
    uint32_t edge_cycle_tick,
    uint32_t latest_sample_cycle_tick,
    uint32_t latest_control_tick)
{
    if (((int32_t)(edge_cycle_tick - latest_sample_cycle_tick) < 0) &&
        (latest_control_tick != 0U))
    {
        return latest_control_tick - 1U;
    }
    return latest_control_tick;
}

static void foc_time_sync_capture_reject_isr(
    foc_time_sync_capture_t *capture,
    uint32_t failure_reason)
{
    uint32_t rejected;

    rejected = FOC_ATOMIC_LOAD_RELAXED(&capture->rejected_count);
    FOC_ATOMIC_STORE_RELAXED(&capture->rejected_count,
                             foc_time_sync_saturating_increment(rejected));
    FOC_ATOMIC_STORE_RELAXED(&capture->failure_reason, failure_reason);
    FOC_ATOMIC_STORE_RELEASE(&capture->state,
                             (uint32_t)FOC_TIME_SYNC_CAPTURE_FAILED);
}

void foc_time_sync_capture_init(foc_time_sync_capture_t *capture)
{
    if (capture == NULL)
    {
        return;
    }
    (void)memset(capture, 0, sizeof(*capture));
    capture->struct_size = sizeof(*capture);
    capture->version = FOC_TIME_SYNC_ABI_VERSION;
    capture->state = (uint32_t)FOC_TIME_SYNC_CAPTURE_IDLE;
}

uint32_t foc_time_sync_capture_arm(foc_time_sync_capture_t *capture,
                                   uint32_t source,
                                   uint32_t session_id,
                                   uint32_t first_edge_sequence)
{
    if ((capture == NULL) || (capture->struct_size != sizeof(*capture)) ||
        (capture->version != FOC_TIME_SYNC_ABI_VERSION) ||
        (foc_time_sync_source_valid(source) == 0U) || (session_id == 0U))
    {
        return 0U;
    }
    if (FOC_ATOMIC_LOAD_ACQUIRE(&capture->state) ==
        (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED)
    {
        return 0U;
    }

    capture->source = source;
    capture->session_id = session_id;
    FOC_ATOMIC_STORE_RELAXED(&capture->next_edge_sequence,
                             first_edge_sequence);
    FOC_ATOMIC_STORE_RELAXED(&capture->write_count, 0U);
    FOC_ATOMIC_STORE_RELAXED(&capture->read_count, 0U);
    FOC_ATOMIC_STORE_RELAXED(&capture->overflow_count, 0U);
    FOC_ATOMIC_STORE_RELAXED(&capture->rejected_count, 0U);
    FOC_ATOMIC_STORE_RELAXED(&capture->port_error_count, 0U);
    FOC_ATOMIC_STORE_RELAXED(&capture->failure_reason,
                             (uint32_t)FOC_TIME_SYNC_FAILURE_NONE);
    FOC_ATOMIC_STORE_RELEASE(&capture->state,
                             (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED);
    return 1U;
}

uint32_t foc_time_sync_capture_record_isr(
    foc_time_sync_capture_t *capture,
    const foc_time_sync_event_t *event)
{
    uint32_t write_count;
    uint32_t read_count;
    uint32_t expected_sequence;
    uint32_t overflow_count;

    if ((capture == NULL) ||
        (FOC_ATOMIC_LOAD_ACQUIRE(&capture->state) !=
         (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED))
    {
        return 0U;
    }
    if (foc_time_sync_event_valid(event) == 0U)
    {
        foc_time_sync_capture_reject_isr(
            capture, (uint32_t)FOC_TIME_SYNC_FAILURE_BAD_EVENT);
        return 0U;
    }
    if (event->session_id != capture->session_id)
    {
        foc_time_sync_capture_reject_isr(
            capture, (uint32_t)FOC_TIME_SYNC_FAILURE_SESSION_MISMATCH);
        return 0U;
    }
    if (event->source != capture->source)
    {
        foc_time_sync_capture_reject_isr(
            capture, (uint32_t)FOC_TIME_SYNC_FAILURE_SOURCE_MISMATCH);
        return 0U;
    }
    expected_sequence = FOC_ATOMIC_LOAD_RELAXED(
        &capture->next_edge_sequence);
    if (event->edge_sequence != expected_sequence)
    {
        foc_time_sync_capture_reject_isr(
            capture, (uint32_t)FOC_TIME_SYNC_FAILURE_SEQUENCE_MISMATCH);
        return 0U;
    }

    write_count = FOC_ATOMIC_LOAD_RELAXED(&capture->write_count);
    read_count = FOC_ATOMIC_LOAD_ACQUIRE(&capture->read_count);
    if ((write_count - read_count) >= FOC_TIME_SYNC_CAPTURE_CAPACITY)
    {
        overflow_count = FOC_ATOMIC_LOAD_RELAXED(&capture->overflow_count);
        FOC_ATOMIC_STORE_RELAXED(
            &capture->overflow_count,
            foc_time_sync_saturating_increment(overflow_count));
        foc_time_sync_capture_reject_isr(
            capture, (uint32_t)FOC_TIME_SYNC_FAILURE_QUEUE_FULL);
        return 0U;
    }

    capture->events[write_count % FOC_TIME_SYNC_CAPTURE_CAPACITY] = *event;
    FOC_ATOMIC_STORE_RELAXED(&capture->next_edge_sequence,
                             expected_sequence + 1U);
    FOC_ATOMIC_STORE_RELEASE(&capture->write_count, write_count + 1U);
    return 1U;
}

void foc_time_sync_capture_fail_isr(foc_time_sync_capture_t *capture,
                                    uint32_t failure_reason)
{
    uint32_t port_errors;

    if ((capture == NULL) ||
        (FOC_ATOMIC_LOAD_ACQUIRE(&capture->state) !=
         (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED))
    {
        return;
    }
    if ((failure_reason == (uint32_t)FOC_TIME_SYNC_FAILURE_LOCAL_TICK) ||
        (failure_reason == (uint32_t)FOC_TIME_SYNC_FAILURE_CONTROL_TICK))
    {
        port_errors = FOC_ATOMIC_LOAD_RELAXED(&capture->port_error_count);
        FOC_ATOMIC_STORE_RELAXED(
            &capture->port_error_count,
            foc_time_sync_saturating_increment(port_errors));
    }
    foc_time_sync_capture_reject_isr(capture, failure_reason);
}

void foc_time_sync_capture_stop(foc_time_sync_capture_t *capture)
{
    if ((capture != NULL) &&
        (FOC_ATOMIC_LOAD_ACQUIRE(&capture->state) ==
         (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED))
    {
        FOC_ATOMIC_STORE_RELEASE(&capture->state,
                                 (uint32_t)FOC_TIME_SYNC_CAPTURE_COMPLETE);
    }
}

uint32_t foc_time_sync_capture_pop(foc_time_sync_capture_t *capture,
                                   foc_time_sync_event_t *event)
{
    uint32_t read_count;
    uint32_t write_count;

    if ((capture == NULL) || (event == NULL))
    {
        return 0U;
    }
    read_count = FOC_ATOMIC_LOAD_RELAXED(&capture->read_count);
    write_count = FOC_ATOMIC_LOAD_ACQUIRE(&capture->write_count);
    if (read_count == write_count)
    {
        return 0U;
    }
    *event = capture->events[read_count % FOC_TIME_SYNC_CAPTURE_CAPACITY];
    FOC_ATOMIC_STORE_RELEASE(&capture->read_count, read_count + 1U);
    return 1U;
}

void foc_time_sync_capture_get_status(const foc_time_sync_capture_t *capture,
                                      foc_time_sync_status_t *status)
{
    uint32_t write_count;
    uint32_t read_count;
    uint32_t unread_count;

    if (status == NULL)
    {
        return;
    }
    (void)memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_TIME_SYNC_ABI_VERSION;
    if (capture == NULL)
    {
        return;
    }
    write_count = FOC_ATOMIC_LOAD_ACQUIRE(&capture->write_count);
    read_count = FOC_ATOMIC_LOAD_ACQUIRE(&capture->read_count);
    unread_count = write_count - read_count;
    if (unread_count > FOC_TIME_SYNC_CAPTURE_CAPACITY)
    {
        unread_count = FOC_TIME_SYNC_CAPTURE_CAPACITY;
    }
    status->state = FOC_ATOMIC_LOAD_ACQUIRE(&capture->state);
    status->failure_reason = FOC_ATOMIC_LOAD_ACQUIRE(
        &capture->failure_reason);
    status->source = capture->source;
    status->session_id = capture->session_id;
    status->next_edge_sequence = FOC_ATOMIC_LOAD_ACQUIRE(
        &capture->next_edge_sequence);
    status->recorded_count = write_count;
    status->unread_count = unread_count;
    status->overflow_count = FOC_ATOMIC_LOAD_ACQUIRE(
        &capture->overflow_count);
    status->rejected_count = FOC_ATOMIC_LOAD_ACQUIRE(
        &capture->rejected_count);
    status->port_error_count = FOC_ATOMIC_LOAD_ACQUIRE(
        &capture->port_error_count);
}

uint32_t foc_time_sync_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    uint32_t source,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr,
    foc_time_sync_read_control_tick_isr_fn read_control_tick_isr)
{
    if ((adapter == NULL) || (capture == NULL) ||
        (capture->struct_size != sizeof(*capture)) ||
        (capture->version != FOC_TIME_SYNC_ABI_VERSION) ||
        (foc_time_sync_source_valid(source) == 0U) ||
        (read_local_tick_isr == NULL))
    {
        return 0U;
    }
    if ((source == (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE) &&
        (read_control_tick_isr == NULL))
    {
        return 0U;
    }
    (void)memset(adapter, 0, sizeof(*adapter));
    adapter->struct_size = sizeof(*adapter);
    adapter->version = FOC_TIME_SYNC_ABI_VERSION;
    adapter->source = source;
    adapter->capture = capture;
    adapter->port_context = port_context;
    adapter->read_local_tick_isr = read_local_tick_isr;
    adapter->read_control_tick_isr = read_control_tick_isr;
    return 1U;
}

uint32_t foc_time_sync_adapter_record_verified_edge_isr(
    foc_time_sync_adapter_t *adapter,
    const foc_time_sync_edge_identity_t *identity)
{
    foc_time_sync_event_t event;

    if ((adapter == NULL) ||
        (adapter->struct_size != sizeof(*adapter)) ||
        (adapter->version != FOC_TIME_SYNC_ABI_VERSION) ||
        (adapter->capture == NULL) ||
        (foc_time_sync_identity_valid(identity) == 0U))
    {
        if ((adapter != NULL) && (adapter->capture != NULL))
        {
            foc_time_sync_capture_fail_isr(
                adapter->capture,
                (uint32_t)FOC_TIME_SYNC_FAILURE_BAD_EVENT);
        }
        return 0U;
    }

    (void)memset(&event, 0, sizeof(event));
    event.struct_size = sizeof(event);
    event.version = FOC_TIME_SYNC_EVENT_VERSION;
    event.session_id = identity->session_id;
    event.edge_sequence = identity->edge_sequence;
    event.edge_tag = identity->edge_tag;
    event.source = adapter->source;
    event.control_tick = FOC_TIME_SYNC_NO_CONTROL_TICK;
    event.flags = identity->flags;

    if (adapter->read_local_tick_isr(adapter->port_context,
                                     &event.local_tick_low,
                                     &event.local_tick_high) == 0U)
    {
        foc_time_sync_capture_fail_isr(
            adapter->capture,
            (uint32_t)FOC_TIME_SYNC_FAILURE_LOCAL_TICK);
        return 0U;
    }
    if ((adapter->source ==
         (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE) &&
        (adapter->read_control_tick_isr(
             adapter->port_context, &event.control_tick) == 0U))
    {
        foc_time_sync_capture_fail_isr(
            adapter->capture,
            (uint32_t)FOC_TIME_SYNC_FAILURE_CONTROL_TICK);
        return 0U;
    }
    return foc_time_sync_capture_record_isr(adapter->capture, &event);
}
