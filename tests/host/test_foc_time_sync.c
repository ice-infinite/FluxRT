#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "foc_time_sync.h"
#include "foc_time_sync_dengfoc.h"
#include "foc_time_sync_stm32g431.h"

typedef struct
{
    uint32_t tick_low;
    uint32_t tick_high;
    uint32_t control_tick;
    uint32_t local_ok;
    uint32_t control_ok;
} fake_clock_t;

static uint32_t fake_read_local_tick(void *context,
                                     uint32_t *tick_low,
                                     uint32_t *tick_high)
{
    fake_clock_t *clock = (fake_clock_t *)context;

    if ((clock == NULL) || (tick_low == NULL) || (tick_high == NULL) ||
        (clock->local_ok == 0U))
    {
        return 0U;
    }
    *tick_low = clock->tick_low;
    *tick_high = clock->tick_high;
    return 1U;
}

static uint32_t fake_read_control_tick(void *context, uint32_t *control_tick)
{
    fake_clock_t *clock = (fake_clock_t *)context;

    if ((clock == NULL) || (control_tick == NULL) ||
        (clock->control_ok == 0U))
    {
        return 0U;
    }
    *control_tick = clock->control_tick;
    return 1U;
}

static foc_time_sync_edge_identity_t make_identity(uint32_t session_id,
                                                   uint32_t sequence,
                                                   uint32_t tag)
{
    foc_time_sync_edge_identity_t identity;

    (void)memset(&identity, 0, sizeof(identity));
    identity.struct_size = sizeof(identity);
    identity.version = FOC_TIME_SYNC_IDENTITY_VERSION;
    identity.session_id = session_id;
    identity.edge_sequence = sequence;
    identity.edge_tag = tag;
    identity.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                     FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
    return identity;
}

static void test_g431_adapter_records_control_anchor(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_adapter_t adapter;
    foc_time_sync_edge_identity_t identity;
    foc_time_sync_event_t event;
    foc_time_sync_status_t status;
    fake_clock_t clock = {0x89ABCDEFU, 0x01234567U, 4242U, 1U, 1U};

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
               19U,
               7U) == 1U);
    assert(foc_time_sync_stm32g431_adapter_init(
               &adapter,
               &capture,
               &clock,
               fake_read_local_tick,
               fake_read_control_tick) == 1U);

    identity = make_identity(19U, 7U, 0xA5010007U);
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 1U);
    assert(foc_time_sync_capture_pop(&capture, &event) == 1U);
    assert(event.source ==
           (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE);
    assert(event.local_tick_low == clock.tick_low);
    assert(event.local_tick_high == clock.tick_high);
    assert(event.control_tick == clock.control_tick);
    assert(event.edge_tag == identity.edge_tag);
    assert(foc_time_sync_capture_pop(&capture, &event) == 0U);

    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED);
    assert(status.recorded_count == 1U);
    assert(status.unread_count == 0U);
    assert(status.next_edge_sequence == 8U);
}

static void test_dengfoc_adapter_has_no_control_tick(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_adapter_t adapter;
    foc_time_sync_edge_identity_t identity;
    foc_time_sync_event_t event;
    fake_clock_t clock = {1234U, 2U, 777U, 1U, 0U};

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               21U,
               0U) == 1U);
    assert(foc_time_sync_dengfoc_adapter_init(
               &adapter, &capture, &clock, fake_read_local_tick) == 1U);
    identity = make_identity(21U, 0U, 0xB6020001U);
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 1U);
    assert(foc_time_sync_capture_pop(&capture, &event) == 1U);
    assert(event.source == (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR);
    assert(event.control_tick == FOC_TIME_SYNC_NO_CONTROL_TICK);
}

static void test_unverified_identity_fails_closed(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_adapter_t adapter;
    foc_time_sync_edge_identity_t identity;
    foc_time_sync_status_t status;
    fake_clock_t clock = {1U, 0U, 2U, 1U, 1U};

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
               30U,
               0U) == 1U);
    assert(foc_time_sync_stm32g431_adapter_init(
               &adapter,
               &capture,
               &clock,
               fake_read_local_tick,
               fake_read_control_tick) == 1U);
    identity = make_identity(30U, 0U, 0x101U);
    identity.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE;
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_CAPTURE_FAILED);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_FAILURE_BAD_EVENT);
    assert(status.rejected_count == 1U);
    assert(status.port_error_count == 0U);
}

static void test_sequence_and_session_are_fail_closed(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_adapter_t adapter;
    foc_time_sync_edge_identity_t identity;
    foc_time_sync_status_t status;
    fake_clock_t clock = {1U, 0U, 2U, 1U, 1U};

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
               40U,
               5U) == 1U);
    assert(foc_time_sync_stm32g431_adapter_init(
               &adapter,
               &capture,
               &clock,
               fake_read_local_tick,
               fake_read_control_tick) == 1U);
    identity = make_identity(40U, 6U, 0x202U);
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_FAILURE_SEQUENCE_MISMATCH);

    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
               40U,
               5U) == 1U);
    identity = make_identity(41U, 5U, 0x203U);
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_FAILURE_SESSION_MISMATCH);
}

static void test_fixed_queue_never_overwrites(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_event_t event;
    foc_time_sync_status_t status;
    uint32_t index;

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               50U,
               100U) == 1U);
    for (index = 0U; index < FOC_TIME_SYNC_CAPTURE_CAPACITY; ++index)
    {
        (void)memset(&event, 0, sizeof(event));
        event.struct_size = sizeof(event);
        event.version = FOC_TIME_SYNC_EVENT_VERSION;
        event.session_id = 50U;
        event.edge_sequence = 100U + index;
        event.edge_tag = 0x300U + index;
        event.source = (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR;
        event.control_tick = FOC_TIME_SYNC_NO_CONTROL_TICK;
        event.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                      FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
        assert(foc_time_sync_capture_record_isr(&capture, &event) == 1U);
    }
    event.edge_sequence = 100U + FOC_TIME_SYNC_CAPTURE_CAPACITY;
    event.edge_tag = 0x400U;
    assert(foc_time_sync_capture_record_isr(&capture, &event) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_CAPTURE_FAILED);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_FAILURE_QUEUE_FULL);
    assert(status.recorded_count == FOC_TIME_SYNC_CAPTURE_CAPACITY);
    assert(status.unread_count == FOC_TIME_SYNC_CAPTURE_CAPACITY);
    assert(status.overflow_count == 1U);
    assert(status.rejected_count == 1U);

    for (index = 0U; index < FOC_TIME_SYNC_CAPTURE_CAPACITY; ++index)
    {
        assert(foc_time_sync_capture_pop(&capture, &event) == 1U);
        assert(event.edge_sequence == (100U + index));
    }
    assert(foc_time_sync_capture_pop(&capture, &event) == 0U);
}

static void test_spsc_reuses_slots_and_sequence_wraps(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_event_t input;
    foc_time_sync_event_t output;

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               60U,
               UINT32_MAX) == 1U);
    (void)memset(&input, 0, sizeof(input));
    input.struct_size = sizeof(input);
    input.version = FOC_TIME_SYNC_EVENT_VERSION;
    input.session_id = 60U;
    input.edge_sequence = UINT32_MAX;
    input.edge_tag = 0x501U;
    input.source = (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR;
    input.control_tick = FOC_TIME_SYNC_NO_CONTROL_TICK;
    input.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                  FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
    assert(foc_time_sync_capture_record_isr(&capture, &input) == 1U);
    assert(foc_time_sync_capture_pop(&capture, &output) == 1U);
    assert(output.edge_sequence == UINT32_MAX);

    input.edge_sequence = 0U;
    input.edge_tag = 0x502U;
    assert(foc_time_sync_capture_record_isr(&capture, &input) == 1U);
    assert(foc_time_sync_capture_pop(&capture, &output) == 1U);
    assert(output.edge_sequence == 0U);
}

static void test_tick_port_failure_is_latched(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_adapter_t adapter;
    foc_time_sync_edge_identity_t identity;
    foc_time_sync_status_t status;
    fake_clock_t clock = {0U, 0U, 0U, 0U, 1U};

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
               70U,
               0U) == 1U);
    assert(foc_time_sync_stm32g431_adapter_init(
               &adapter,
               &capture,
               &clock,
               fake_read_local_tick,
               fake_read_control_tick) == 1U);
    identity = make_identity(70U, 0U, 0x601U);
    assert(foc_time_sync_adapter_record_verified_edge_isr(
               &adapter, &identity) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.failure_reason ==
           (uint32_t)FOC_TIME_SYNC_FAILURE_LOCAL_TICK);
    assert(status.port_error_count == 1U);
    assert(status.recorded_count == 0U);
}

static void test_armed_capture_cannot_be_silently_rearmed(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_status_t status;

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               80U,
               12U) == 1U);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               81U,
               0U) == 0U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED);
    assert(status.session_id == 80U);
    assert(status.next_edge_sequence == 12U);
}

static void test_drained_capture_can_cross_an_explicit_session_boundary(void)
{
    foc_time_sync_capture_t capture;
    foc_time_sync_event_t event;
    foc_time_sync_status_t status;

    foc_time_sync_capture_init(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               80U,
               12U) == 1U);
    (void)memset(&event, 0, sizeof(event));
    event.struct_size = sizeof(event);
    event.version = FOC_TIME_SYNC_EVENT_VERSION;
    event.session_id = 80U;
    event.edge_sequence = 12U;
    event.edge_tag = 0x701U;
    event.source = (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR;
    event.control_tick = FOC_TIME_SYNC_NO_CONTROL_TICK;
    event.flags = FOC_TIME_SYNC_FLAG_RISING_EDGE |
                  FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED;
    assert(foc_time_sync_capture_record_isr(&capture, &event) == 1U);
    assert(foc_time_sync_capture_pop(&capture, &event) == 1U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.unread_count == 0U);

    foc_time_sync_capture_stop(&capture);
    assert(foc_time_sync_capture_arm(
               &capture,
               (uint32_t)FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
               81U,
               20U) == 1U);
    foc_time_sync_capture_get_status(&capture, &status);
    assert(status.state == (uint32_t)FOC_TIME_SYNC_CAPTURE_ARMED);
    assert(status.session_id == 81U);
    assert(status.next_edge_sequence == 20U);
}

static void test_edge_tick_resolves_preemption_and_u32_wrap(void)
{
    assert(foc_time_sync_control_tick_at_edge(1000U, 900U, 42U) == 42U);
    assert(foc_time_sync_control_tick_at_edge(1000U, 1000U, 42U) == 42U);
    assert(foc_time_sync_control_tick_at_edge(900U, 1000U, 42U) == 41U);
    assert(foc_time_sync_control_tick_at_edge(900U, 1000U, 0U) == 0U);

    /* Edge just before a DWT wrap, latest ADC sample just after it: the ADC
     * pre-empted the pending lower-priority edge IRQ, so use the prior tick. */
    assert(foc_time_sync_control_tick_at_edge(
               0xFFFFFFF0U, 0x00000010U, 77U) == 76U);
    /* Edge just after wrap while the latest sample is just before wrap. */
    assert(foc_time_sync_control_tick_at_edge(
               0x00000020U, 0xFFFFFFF0U, 77U) == 77U);
}

int main(void)
{
    test_g431_adapter_records_control_anchor();
    test_dengfoc_adapter_has_no_control_tick();
    test_unverified_identity_fails_closed();
    test_sequence_and_session_are_fail_closed();
    test_fixed_queue_never_overwrites();
    test_spsc_reuses_slots_and_sequence_wraps();
    test_tick_port_failure_is_latched();
    test_armed_capture_cannot_be_silently_rearmed();
    test_drained_capture_can_cross_an_explicit_session_boundary();
    test_edge_tick_resolves_preemption_and_u32_wrap();
    return 0;
}
