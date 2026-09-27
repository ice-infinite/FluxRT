#include "foc_lsi_preload_sink_internal.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    volatile uint32_t compare_u;
    volatile uint32_t compare_v;
    volatile uint32_t compare_w;
    volatile uint32_t period;
} fake_tim1_t;

static foc_lsi_executor_output_t valid_output(uint32_t tick)
{
    foc_lsi_executor_output_t output;

    memset(&output, 0, sizeof(output));
    output.struct_size = sizeof(output);
    output.version = FOC_LSI_EXECUTOR_OUTPUT_VERSION;
    output.action = FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD;
    output.result = FOC_LSI_EXECUTOR_RESULT_PRELOAD_READY;
    output.planner_status = FOC_STATUS_OK;
    output.source_control_tick = tick;
    output.expected_active_control_tick = tick + 1U;
    output.compare_u = 4250U;
    output.compare_v = 3187U;
    output.compare_w = 3187U;
    output.pwm_period_ticks = 7083U;
    output.plan.struct_size = sizeof(output.plan);
    output.plan.version = FOC_LSI_ACTUATION_OUTPUT_VERSION;
    output.plan.drive_active = 1U;
    output.plan.source_control_tick = tick;
    output.plan.expected_active_control_tick = tick + 1U;
    output.plan.duty_u = 0.60f;
    output.plan.duty_v = 0.45f;
    output.plan.duty_w = 0.45f;
    return output;
}

static foc_lsi_preload_safety_t safe_hardware(void)
{
    foc_lsi_preload_safety_t safety;

    memset(&safety, 0, sizeof(safety));
    safety.timer_clock_enabled = 1U;
    safety.timer_configured = 1U;
    safety.compare_preload_enabled = 1U;
    return safety;
}

static foc_lsi_preload_safety_t active_hardware(void)
{
    foc_lsi_preload_safety_t safety = safe_hardware();
    safety.gate_enabled = 1U;
    safety.main_output_enabled = 1U;
    safety.channel_outputs_enabled = 1U;
    return safety;
}

static foc_lsi_preload_registers_t bind_fake(fake_tim1_t *tim1)
{
    foc_lsi_preload_registers_t registers = {
        &tim1->compare_u,
        &tim1->compare_v,
        &tim1->compare_w,
        &tim1->period,
    };
    return registers;
}

static void init_session(foc_lsi_preload_session_t *session)
{
    foc_lsi_preload_session_init(session, 7083U, 1U, 213U, 6871U);
    assert(session->initialized == 1U);
}

static void test_one_shot_write_readback_and_replay_rejection(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t permit;
    foc_lsi_executor_output_t output = valid_output(100U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    fake_tim1_t tim1 = {0U, 0U, 0U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_validation(&session, 0U) ==
           FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(tim1.compare_u == output.compare_u);
    assert(tim1.compare_v == output.compare_v);
    assert(tim1.compare_w == output.compare_w);
    assert(session.validation_open == 0U);
    assert(session.permit_consumed == 1U);

    /* A consumed permit cannot be replayed; rejection clears stale compares. */
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED);
    assert((tim1.compare_u == 0U) &&
           (tim1.compare_v == 0U) &&
           (tim1.compare_w == 0U));
}

static void test_stale_or_mutated_permit_fails_closed(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t stale;
    foc_lsi_preload_permit_t current;
    foc_lsi_executor_output_t output = valid_output(200U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    fake_tim1_t tim1 = {1U, 2U, 3U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &stale) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    foc_lsi_preload_session_abort(&session);

    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &current) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(current.generation != stale.generation);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &stale,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_PERMIT_REJECTED);
    assert((tim1.compare_u == 0U) &&
           (tim1.compare_v == 0U) &&
           (tim1.compare_w == 0U));

    init_session(&session);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &current) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    output.compare_u += 1U;
    assert(foc_lsi_preload_sink_apply(&session,
                                      &current,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_PERMIT_REJECTED);
}

static void test_hardware_and_output_gates(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t permit;
    foc_lsi_executor_output_t output = valid_output(300U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    fake_tim1_t tim1 = {11U, 12U, 13U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    safety.main_output_enabled = 1U;
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE);
    assert((tim1.compare_u == 0U) &&
           (tim1.compare_v == 0U) &&
           (tim1.compare_w == 0U));

    init_session(&session);
    output = valid_output(301U);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    output.compare_u = 7000U;
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OUTPUT_REJECTED);
    assert(session.validation_open == 0U);

    init_session(&session);
    output = valid_output(302U);
    output.plan.duty_u = NAN;
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OUTPUT_REJECTED);

    init_session(&session);
    output = valid_output(UINT32_MAX);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OUTPUT_REJECTED);

    init_session(&session);
    output = valid_output(303U);
    safety = safe_hardware();
    tim1.period = 7000U;
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE);
}

static void test_register_alias_and_invalid_configuration(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t permit;
    foc_lsi_executor_output_t output = valid_output(400U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    volatile uint32_t aliased_compare = 0U;
    volatile uint32_t period = 7083U;
    foc_lsi_preload_registers_t registers = {
        &aliased_compare,
        &aliased_compare,
        &aliased_compare,
        &period,
    };

    foc_lsi_preload_session_init(&session, 0U, 1U, 0U, 0U);
    assert(session.initialized == 0U);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT);

    init_session(&session);
    assert(foc_lsi_preload_session_open_validation(
               &session, FOC_LSI_PRELOAD_CONFIRMATION) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_REGISTER_MISMATCH);
    assert(aliased_compare == 0U);
}

static void test_bounded_active_session(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t first;
    foc_lsi_preload_permit_t second;
    foc_lsi_executor_output_t output = valid_output(500U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    fake_tim1_t tim1 = {0U, 0U, 0U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_active(
               &session, 0U, 2U, 4U, 500U) ==
           FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 0U, 4U, 500U) ==
           FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 2U, 4U, 500U) ==
           FOC_LSI_PRELOAD_RESULT_OK);

    /* First preload must happen while every physical output is still off. */
    assert(foc_lsi_preload_session_issue(&session, &output, &first) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(first.session_mode == FOC_LSI_PRELOAD_MODE_ACTIVE);
    assert(first.permit_sequence == 1U);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &first,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(session.active_open == 1U);
    assert(session.active_write_count == 1U);

    output = valid_output(501U);
    assert(foc_lsi_preload_session_issue(&session, &output, &second) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(second.permit_sequence == 2U);
    safety = active_hardware();
    assert(foc_lsi_preload_sink_apply(&session,
                                      &second,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(session.active_write_count == 2U);

    /* Replaying an already-consumed permit fails closed and clears compares. */
    assert(foc_lsi_preload_sink_apply(&session,
                                      &second,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED);
    assert((tim1.compare_u == 0U) &&
           (tim1.compare_v == 0U) &&
           (tim1.compare_w == 0U));
}

static void test_active_sequence_and_budget_gates(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t permit;
    foc_lsi_executor_output_t output = valid_output(600U);
    foc_lsi_preload_safety_t safety = safe_hardware();
    fake_tim1_t tim1 = {0U, 0U, 0U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 2U, 3U, 600U) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);

    output = valid_output(602U);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_SEQUENCE_ERROR);
    assert(session.active_open == 0U);

    init_session(&session);
    output = valid_output(700U);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 1U, 2U, 700U) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    safety = safe_hardware();
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    output = valid_output(701U);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_BUDGET_EXHAUSTED);
    assert(session.active_open == 0U);

    init_session(&session);
    output = valid_output(800U);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 2U, 2U, 800U) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    output = valid_output(802U);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_BUDGET_EXHAUSTED);
}

static void test_active_hardware_phase_gates(void)
{
    foc_lsi_preload_session_t session;
    foc_lsi_preload_permit_t permit;
    foc_lsi_executor_output_t output = valid_output(900U);
    foc_lsi_preload_safety_t safety = active_hardware();
    fake_tim1_t tim1 = {0U, 0U, 0U, 7083U};
    foc_lsi_preload_registers_t registers = bind_fake(&tim1);

    init_session(&session);
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 2U, 4U, 900U) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    /* First write must precede gate/MOE/CCER enable. */
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE);
    assert(session.active_open == 0U);

    init_session(&session);
    output = valid_output(910U);
    safety = safe_hardware();
    assert(foc_lsi_preload_session_open_active(
               &session, FOC_LSI_ACTIVE_CONFIRMATION, 2U, 4U, 910U) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    output = valid_output(911U);
    assert(foc_lsi_preload_session_issue(&session, &output, &permit) ==
           FOC_LSI_PRELOAD_RESULT_OK);
    /* Every later write must see all three physical enable facts asserted. */
    assert(foc_lsi_preload_sink_apply(&session,
                                      &permit,
                                      &output,
                                      &registers,
                                      &safety) ==
           FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE);
    assert(session.active_open == 0U);
    assert((tim1.compare_u == 0U) &&
           (tim1.compare_v == 0U) &&
           (tim1.compare_w == 0U));
}

int main(void)
{
    test_one_shot_write_readback_and_replay_rejection();
    test_stale_or_mutated_permit_fails_closed();
    test_hardware_and_output_gates();
    test_register_alias_and_invalid_configuration();
    test_bounded_active_session();
    test_active_sequence_and_budget_gates();
    test_active_hardware_phase_gates();
    puts("FOC LSI PRELOAD SINK: PASS");
    return 0;
}
