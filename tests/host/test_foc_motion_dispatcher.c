#include "foc_motion_dispatcher.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    foc_motion_dispatcher_t *dispatcher;
    uint32_t enter_count;
    uint32_t exit_count;
    uint32_t last_exit_key;
    uint32_t consume_during_next_enter;
    foc_motion_dispatcher_result_t nested_result;
    foc_motion_realtime_request_t nested_request;
} fake_critical_t;

static uint32_t fake_enter_critical(void *context)
{
    fake_critical_t *fake = (fake_critical_t *)context;
    uint32_t key = UINT32_C(0xC0010000) | fake->enter_count;
    fake->enter_count++;
    if (fake->consume_during_next_enter != 0U)
    {
        fake->consume_during_next_enter = 0U;
        fake->nested_result = foc_motion_dispatcher_consume_isr(
            fake->dispatcher, UINT32_C(0x11223344), &fake->nested_request);
    }
    return key;
}

static void fake_exit_critical(void *context, uint32_t key)
{
    fake_critical_t *fake = (fake_critical_t *)context;
    fake->exit_count++;
    fake->last_exit_key = key;
}

static foc_product_command_t make_command(uint32_t sequence, float marker)
{
    foc_product_command_t command;
    (void)memset(&command, 0, sizeof(command));
    command.struct_size = (uint32_t)sizeof(command);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.sequence = sequence;
    command.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    command.control_mode = FOC_CONTROL_MODE_VELOCITY;
    command.input_mode = FOC_INPUT_MODE_VELOCITY_RAMP;
    command.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    command.current_q_ref_a = marker;
    command.velocity_ref_rad_s = marker * 10.0f;
    return command;
}

static void init_dispatcher(foc_motion_dispatcher_t *dispatcher,
                            fake_critical_t *fake)
{
    foc_motion_dispatcher_ops_t ops;
    (void)memset(fake, 0, sizeof(*fake));
    fake->dispatcher = dispatcher;
    (void)memset(&ops, 0, sizeof(ops));
    ops.struct_size = (uint32_t)sizeof(ops);
    ops.version = FOC_MOTION_DISPATCHER_VERSION;
    ops.enter_critical = fake_enter_critical;
    ops.exit_critical = fake_exit_critical;
    ops.context = fake;
    assert(foc_motion_dispatcher_init(dispatcher, &ops) ==
           FOC_MOTION_DISPATCHER_OK);
}

static void test_layout_finite_and_position_validation(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t command = make_command(1U, 0.5f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    command.struct_size--;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    command.struct_size = (uint32_t)sizeof(command);
    command.version++;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.torque_ref_nm = NAN;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    command.torque_ref_nm = 0.0f;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 2U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 1U, INFINITY, 4U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);

    /* An absent optional position is sanitized instead of leaking a NaN. */
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, NAN, UINT32_MAX) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 5U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_POSITION_VALID) == 0U);
    assert(request.mechanical_position_rad == 0.0f);
    assert(request.position_sampled_at_ms == 0U);
}

static void test_publish_is_not_torn_when_isr_preempts_before_commit(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t old_command = make_command(10U, 1.0f);
    foc_product_command_t new_command = make_command(20U, 2.0f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    assert(foc_motion_dispatcher_publish(&dispatcher, &old_command, 1U, 0.25f, 10U) ==
           FOC_MOTION_DISPATCHER_OK);
    fake.consume_during_next_enter = 1U;
    assert(foc_motion_dispatcher_publish(&dispatcher, &new_command, 1U, 0.75f, 20U) ==
           FOC_MOTION_DISPATCHER_OK);

    assert(fake.nested_result == FOC_MOTION_DISPATCHER_OK);
    assert(fake.nested_request.command.sequence == old_command.sequence);
    assert(fake.nested_request.command.current_q_ref_a == 1.0f);
    assert(fake.nested_request.mechanical_position_rad == 0.25f);
    assert(fake.nested_request.position_sampled_at_ms == 10U);

    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 100U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.command.sequence == new_command.sequence);
    assert(request.command.current_q_ref_a == 2.0f);
    assert(request.mechanical_position_rad == 0.75f);
    assert(request.position_sampled_at_ms == 20U);
    assert(request.consumer_now_ms == 100U);
    assert(request.struct_size == (uint32_t)sizeof(request));
    assert(request.version == FOC_MOTION_REALTIME_REQUEST_VERSION);
    assert(fake.enter_count == fake.exit_count);
}

static void test_urgent_priority_and_one_shot_consumption(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t command = make_command(30U, 3.0f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_request_stop(&dispatcher) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_request_fault(&dispatcher, UINT32_C(0x00000004)) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_request_fault(&dispatcher, UINT32_C(0x00000020)) ==
           FOC_MOTION_DISPATCHER_OK);

    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 7U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_FAULT) != 0U);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_STOP) == 0U);
    assert(request.fault_detail == UINT32_C(0x00000024));

    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 8U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags &
            (FOC_MOTION_REALTIME_REQUEST_STOP |
             FOC_MOTION_REALTIME_REQUEST_FAULT)) == 0U);
    assert(request.fault_detail == 0U);
    assert(request.command.sequence == command.sequence);
    assert(dispatcher.consume_count == 2U);
}

static void test_sequence_wrap_and_repeated_consume(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t command = make_command(UINT32_MAX, 4.0f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    dispatcher.next_publication_sequence = UINT32_MAX;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, UINT32_MAX, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == UINT32_MAX);
    assert(request.consumer_now_ms == UINT32_MAX);
    assert(request.command.sequence == UINT32_MAX);

    command.sequence++;
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 0U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == 0U);
    assert(request.command.sequence == command.sequence);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 1U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == 0U);
    assert(request.consumer_now_ms == 1U);
}

static void test_invalid_publish_is_transactional(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t old_command = make_command(60U, 6.0f);
    foc_product_command_t rejected_command = make_command(61U, 7.0f);
    foc_product_command_t new_command = make_command(62U, 8.0f);
    foc_motion_realtime_request_t request;
    uint32_t active_before;
    uint32_t next_sequence_before;

    init_dispatcher(&dispatcher, &fake);
    assert(foc_motion_dispatcher_publish(
               &dispatcher, &old_command, 1U, 0.6f, UINT32_MAX) ==
           FOC_MOTION_DISPATCHER_OK);
    active_before = dispatcher.active_slot;
    next_sequence_before = dispatcher.next_publication_sequence;

    rejected_command.velocity_feedforward_rad_s = INFINITY;
    assert(foc_motion_dispatcher_publish(
               &dispatcher, &rejected_command, 1U, 0.7f, 0U) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    assert(dispatcher.active_slot == active_before);
    assert(dispatcher.next_publication_sequence == next_sequence_before);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 0U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == 1U);
    assert(request.command.sequence == old_command.sequence);
    assert(request.command.current_q_ref_a == 6.0f);
    assert(request.mechanical_position_rad == 0.6f);
    assert(request.position_sampled_at_ms == UINT32_MAX);

    assert(foc_motion_dispatcher_publish(
               &dispatcher, &new_command, 1U, 0.8f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 1U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == 2U);
    assert(request.command.sequence == new_command.sequence);
}

static void test_urgent_latch_survives_isr_at_critical_entry(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t command = make_command(70U, 7.0f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    assert(foc_motion_dispatcher_publish(
               &dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);

    /* Model the ADC IRQ winning immediately before enter_critical masks it.
     * That consume may observe the old state, but the task-side request must
     * remain latched for the next consume after the critical section exits. */
    fake.consume_during_next_enter = 1U;
    assert(foc_motion_dispatcher_request_stop(&dispatcher) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(fake.nested_result == FOC_MOTION_DISPATCHER_OK);
    assert((fake.nested_request.request_flags &
            FOC_MOTION_REALTIME_REQUEST_STOP) == 0U);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, UINT32_MAX, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_STOP) != 0U);
    assert(request.consumer_now_ms == UINT32_MAX);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 0U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_STOP) == 0U);

    fake.consume_during_next_enter = 1U;
    assert(foc_motion_dispatcher_request_fault(&dispatcher, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((fake.nested_request.request_flags &
            FOC_MOTION_REALTIME_REQUEST_FAULT) == 0U);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 1U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_FAULT) != 0U);
    assert(request.fault_detail == 0U);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 2U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert((request.request_flags & FOC_MOTION_REALTIME_REQUEST_FAULT) == 0U);
    assert(fake.enter_count == fake.exit_count);
}

static void test_no_command_urgent_only_and_stopped_reset(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_product_command_t command = make_command(50U, 5.0f);
    foc_motion_realtime_request_t request;

    init_dispatcher(&dispatcher, &fake);
    (void)memset(&request, 0xA5, sizeof(request));
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 1U, &request) ==
           FOC_MOTION_DISPATCHER_NO_COMMAND);
    assert(request.struct_size == (uint32_t)sizeof(request));
    assert(request.version == FOC_MOTION_REALTIME_REQUEST_VERSION);
    assert(request.request_flags == 0U);

    assert(foc_motion_dispatcher_request_stop(&dispatcher) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 2U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.request_flags == FOC_MOTION_REALTIME_REQUEST_STOP);

    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 1U, 0.5f, 2U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_request_fault(&dispatcher, 9U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_reset_stopped(&dispatcher) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(dispatcher.initialized == 1U);
    assert(dispatcher.next_publication_sequence == 1U);
    assert(dispatcher.consume_count == 0U);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 3U, &request) ==
           FOC_MOTION_DISPATCHER_NO_COMMAND);
    assert(request.request_flags == 0U);
    assert(foc_motion_dispatcher_publish(&dispatcher, &command, 0U, 0.0f, 0U) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 4U, &request) ==
           FOC_MOTION_DISPATCHER_OK);
    assert(request.publication_sequence == 1U);
}

static void test_init_and_output_argument_defense(void)
{
    foc_motion_dispatcher_t dispatcher;
    fake_critical_t fake;
    foc_motion_dispatcher_ops_t bad_ops;
    foc_motion_realtime_request_t request;

    (void)memset(&bad_ops, 0, sizeof(bad_ops));
    assert(foc_motion_dispatcher_init(&dispatcher, &bad_ops) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    init_dispatcher(&dispatcher, &fake);
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 0U, NULL) ==
           FOC_MOTION_DISPATCHER_INVALID_ARGUMENT);
    dispatcher.version++;
    assert(foc_motion_dispatcher_consume_isr(&dispatcher, 0U, &request) ==
           FOC_MOTION_DISPATCHER_NOT_INITIALIZED);
    assert(request.struct_size == (uint32_t)sizeof(request));
    assert(request.version == FOC_MOTION_REALTIME_REQUEST_VERSION);
}

int main(void)
{
    test_layout_finite_and_position_validation();
    test_publish_is_not_torn_when_isr_preempts_before_commit();
    test_urgent_priority_and_one_shot_consumption();
    test_sequence_wrap_and_repeated_consume();
    test_invalid_publish_is_transactional();
    test_urgent_latch_survives_isr_at_critical_entry();
    test_no_command_urgent_only_and_stopped_reset();
    test_init_and_output_argument_defense();
    puts("foc motion dispatcher tests passed");
    return 0;
}
