#include "foc_lsi_preload_sink_internal.h"

#include <math.h>
#include <string.h>

#define FOC_LSI_PRELOAD_TAG_SEED (0xA54C5349UL)

static uint32_t foc_lsi_preload_bool_is_valid(uint32_t value)
{
    return (value <= 1U) ? 1U : 0U;
}

static uint32_t foc_lsi_preload_tag(
    const foc_lsi_preload_session_t *session,
    const foc_lsi_executor_output_t *output)
{
    uint32_t tag = FOC_LSI_PRELOAD_TAG_SEED ^ session->generation;

    tag = (tag << 5U) | (tag >> 27U);
    tag ^= output->source_control_tick;
    tag = (tag << 7U) | (tag >> 25U);
    tag ^= output->expected_active_control_tick;
    tag ^= ((uint32_t)output->compare_u << 16U) | output->compare_v;
    tag = (tag << 11U) | (tag >> 21U);
    tag ^= ((uint32_t)output->compare_w << 16U) |
           output->pwm_period_ticks;
    tag ^= session->next_permit_sequence;
    tag ^= (session->active_open != 0U) ?
        FOC_LSI_PRELOAD_MODE_ACTIVE : FOC_LSI_PRELOAD_MODE_VALIDATION;
    return tag;
}

static uint32_t foc_lsi_preload_session_is_valid(
    const foc_lsi_preload_session_t *session)
{
    return ((session != 0) &&
            (session->struct_size == sizeof(*session)) &&
            (session->version == FOC_LSI_PRELOAD_SESSION_VERSION) &&
            (session->initialized != 0U) &&
            (session->pwm_period_ticks > 0U) &&
            (session->pwm_period_ticks <= 65535U) &&
            (session->actuation_delay_control_ticks == 1U) &&
            (session->minimum_compare <= session->maximum_compare) &&
            ((uint32_t)session->maximum_compare <=
             session->pwm_period_ticks)) ? 1U : 0U;
}

static uint32_t foc_lsi_preload_output_is_valid(
    const foc_lsi_preload_session_t *session,
    const foc_lsi_executor_output_t *output)
{
    return ((output != 0) &&
            (output->struct_size == sizeof(*output)) &&
            (output->version == FOC_LSI_EXECUTOR_OUTPUT_VERSION) &&
            (output->action == FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD) &&
            (output->result == FOC_LSI_EXECUTOR_RESULT_PRELOAD_READY) &&
            (output->planner_status == FOC_STATUS_OK) &&
            (output->plan.struct_size == sizeof(output->plan)) &&
            (output->plan.version == FOC_LSI_ACTUATION_OUTPUT_VERSION) &&
            (output->plan.safe_output_required == 0U) &&
            (output->plan.drive_active == 1U) &&
            (output->ledger_checked == output->ledger_matched) &&
            (output->source_control_tick != UINT32_MAX) &&
            (output->source_control_tick ==
             output->plan.source_control_tick) &&
            (output->expected_active_control_tick ==
             output->plan.expected_active_control_tick) &&
            (output->expected_active_control_tick ==
             output->source_control_tick +
                 session->actuation_delay_control_ticks) &&
            ((uint32_t)output->pwm_period_ticks ==
             session->pwm_period_ticks) &&
            isfinite(output->plan.duty_u) &&
            isfinite(output->plan.duty_v) &&
            isfinite(output->plan.duty_w) &&
            (output->plan.duty_u >= 0.0f) &&
            (output->plan.duty_u <= 1.0f) &&
            (output->plan.duty_v >= 0.0f) &&
            (output->plan.duty_v <= 1.0f) &&
            (output->plan.duty_w >= 0.0f) &&
            (output->plan.duty_w <= 1.0f) &&
            ((uint16_t)((output->plan.duty_u *
                         (float)session->pwm_period_ticks) + 0.5f) ==
             output->compare_u) &&
            ((uint16_t)((output->plan.duty_v *
                         (float)session->pwm_period_ticks) + 0.5f) ==
             output->compare_v) &&
            ((uint16_t)((output->plan.duty_w *
                         (float)session->pwm_period_ticks) + 0.5f) ==
             output->compare_w) &&
            (output->compare_u >= session->minimum_compare) &&
            (output->compare_u <= session->maximum_compare) &&
            (output->compare_v >= session->minimum_compare) &&
            (output->compare_v <= session->maximum_compare) &&
            (output->compare_w >= session->minimum_compare) &&
            (output->compare_w <= session->maximum_compare)) ? 1U : 0U;
}

static void foc_lsi_preload_registers_safe(
    const foc_lsi_preload_registers_t *registers)
{
    if (registers == 0)
    {
        return;
    }
    if (registers->compare_u != 0)
    {
        *registers->compare_u = 0U;
    }
    if (registers->compare_v != 0)
    {
        *registers->compare_v = 0U;
    }
    if (registers->compare_w != 0)
    {
        *registers->compare_w = 0U;
    }
}

void foc_lsi_preload_session_init(
    foc_lsi_preload_session_t *session,
    uint32_t pwm_period_ticks,
    uint32_t actuation_delay_control_ticks,
    uint16_t minimum_compare,
    uint16_t maximum_compare)
{
    if (session == 0)
    {
        return;
    }
    memset(session, 0, sizeof(*session));
    session->struct_size = sizeof(*session);
    session->version = FOC_LSI_PRELOAD_SESSION_VERSION;
    session->pwm_period_ticks = pwm_period_ticks;
    session->actuation_delay_control_ticks = actuation_delay_control_ticks;
    session->minimum_compare = minimum_compare;
    session->maximum_compare = maximum_compare;
    if ((pwm_period_ticks > 0U) &&
        (pwm_period_ticks <= 65535U) &&
        (actuation_delay_control_ticks == 1U) &&
        (minimum_compare <= maximum_compare) &&
        ((uint32_t)maximum_compare <= pwm_period_ticks))
    {
        session->initialized = 1U;
    }
}

foc_lsi_preload_result_t foc_lsi_preload_session_open_validation(
    foc_lsi_preload_session_t *session,
    uint32_t confirmation)
{
    if ((foc_lsi_preload_session_is_valid(session) == 0U) ||
        (confirmation != FOC_LSI_PRELOAD_CONFIRMATION))
    {
        return FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT;
    }
    if ((session->validation_open != 0U) || (session->active_open != 0U))
    {
        session->validation_open = 0U;
        session->active_open = 0U;
        return FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED;
    }
    ++session->generation;
    if (session->generation == 0U)
    {
        ++session->generation;
    }
    session->permit_issued = 0U;
    session->permit_consumed = 0U;
    session->next_permit_sequence = 1U;
    session->last_consumed_sequence = 0U;
    session->validation_open = 1U;
    return FOC_LSI_PRELOAD_RESULT_OK;
}

foc_lsi_preload_result_t foc_lsi_preload_session_open_active(
    foc_lsi_preload_session_t *session,
    uint32_t confirmation,
    uint32_t active_write_limit,
    uint32_t total_tick_limit,
    uint32_t start_control_tick)
{
    if ((foc_lsi_preload_session_is_valid(session) == 0U) ||
        (confirmation != FOC_LSI_ACTIVE_CONFIRMATION) ||
        (active_write_limit == 0U) ||
        (active_write_limit > FOC_LSI_ACTIVE_WRITE_LIMIT_MAX) ||
        (total_tick_limit < active_write_limit) ||
        (total_tick_limit > FOC_LSI_ACTIVE_TOTAL_TICKS_MAX))
    {
        return FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT;
    }
    if ((session->validation_open != 0U) || (session->active_open != 0U))
    {
        session->validation_open = 0U;
        session->active_open = 0U;
        return FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED;
    }
    ++session->generation;
    if (session->generation == 0U)
    {
        ++session->generation;
    }
    session->permit_issued = 0U;
    session->permit_consumed = 0U;
    session->active_write_limit = active_write_limit;
    session->active_write_count = 0U;
    session->active_total_tick_limit = total_tick_limit;
    session->active_start_control_tick = start_control_tick;
    session->next_permit_sequence = 1U;
    session->last_consumed_sequence = 0U;
    session->last_source_control_tick = 0U;
    session->has_last_source_control_tick = 0U;
    session->active_open = 1U;
    return FOC_LSI_PRELOAD_RESULT_OK;
}

foc_lsi_preload_result_t foc_lsi_preload_session_issue(
    foc_lsi_preload_session_t *session,
    const foc_lsi_executor_output_t *output,
    foc_lsi_preload_permit_t *permit)
{
    if (permit != 0)
    {
        memset(permit, 0, sizeof(*permit));
    }
    if ((foc_lsi_preload_session_is_valid(session) == 0U) ||
        (permit == 0))
    {
        if (foc_lsi_preload_session_is_valid(session) != 0U)
        {
            session->validation_open = 0U;
            session->active_open = 0U;
        }
        return FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT;
    }
    if (((session->validation_open == 0U) &&
         (session->active_open == 0U)) ||
        (session->permit_issued != 0U) ||
        (session->permit_consumed != 0U))
    {
        return FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED;
    }
    if (session->active_open != 0U)
    {
        if ((session->active_write_count >= session->active_write_limit) ||
            ((output != 0) &&
             ((output->source_control_tick -
               session->active_start_control_tick) >=
              session->active_total_tick_limit)))
        {
            session->active_open = 0U;
            return FOC_LSI_PRELOAD_RESULT_BUDGET_EXHAUSTED;
        }
        if ((session->has_last_source_control_tick != 0U) &&
            ((output == 0) ||
             (output->source_control_tick !=
              session->last_source_control_tick + 1U)))
        {
            session->active_open = 0U;
            return FOC_LSI_PRELOAD_RESULT_SEQUENCE_ERROR;
        }
    }
    if (foc_lsi_preload_output_is_valid(session, output) == 0U)
    {
        session->validation_open = 0U;
        session->active_open = 0U;
        return FOC_LSI_PRELOAD_RESULT_OUTPUT_REJECTED;
    }

    permit->struct_size = sizeof(*permit);
    permit->version = FOC_LSI_PRELOAD_PERMIT_VERSION;
    permit->generation = session->generation;
    permit->source_control_tick = output->source_control_tick;
    permit->expected_active_control_tick =
        output->expected_active_control_tick;
    permit->session_mode = (session->active_open != 0U) ?
        FOC_LSI_PRELOAD_MODE_ACTIVE : FOC_LSI_PRELOAD_MODE_VALIDATION;
    permit->permit_sequence = session->next_permit_sequence;
    permit->compare_u = output->compare_u;
    permit->compare_v = output->compare_v;
    permit->compare_w = output->compare_w;
    permit->pwm_period_ticks = output->pwm_period_ticks;
    permit->authorization_tag = foc_lsi_preload_tag(session, output);
    session->permit_issued = 1U;
    return FOC_LSI_PRELOAD_RESULT_OK;
}

foc_lsi_preload_result_t foc_lsi_preload_sink_apply(
    foc_lsi_preload_session_t *session,
    const foc_lsi_preload_permit_t *permit,
    const foc_lsi_executor_output_t *output,
    const foc_lsi_preload_registers_t *registers,
    const foc_lsi_preload_safety_t *safety)
{
    foc_lsi_preload_result_t result = FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT;

    if ((foc_lsi_preload_session_is_valid(session) == 0U) ||
        (permit == 0) ||
        (registers == 0) ||
        (safety == 0) ||
        (registers->compare_u == 0) ||
        (registers->compare_v == 0) ||
        (registers->compare_w == 0) ||
        (registers->period == 0))
    {
        foc_lsi_preload_registers_safe(registers);
        if (foc_lsi_preload_session_is_valid(session) != 0U)
        {
            session->validation_open = 0U;
            session->active_open = 0U;
        }
        return result;
    }
    if (((session->validation_open == 0U) &&
         (session->active_open == 0U)) ||
        (session->permit_issued == 0U) ||
        (session->permit_consumed != 0U))
    {
        result = FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED;
        goto reject;
    }
    if ((foc_lsi_preload_output_is_valid(session, output) == 0U) ||
        (permit->struct_size != sizeof(*permit)) ||
        (permit->version != FOC_LSI_PRELOAD_PERMIT_VERSION) ||
        (permit->generation != session->generation) ||
        (permit->session_mode != ((session->active_open != 0U) ?
             FOC_LSI_PRELOAD_MODE_ACTIVE :
             FOC_LSI_PRELOAD_MODE_VALIDATION)) ||
        (permit->permit_sequence != session->next_permit_sequence) ||
        (permit->source_control_tick != output->source_control_tick) ||
        (permit->expected_active_control_tick !=
         output->expected_active_control_tick) ||
        (permit->compare_u != output->compare_u) ||
        (permit->compare_v != output->compare_v) ||
        (permit->compare_w != output->compare_w) ||
        (permit->pwm_period_ticks != output->pwm_period_ticks) ||
        (permit->authorization_tag != foc_lsi_preload_tag(session, output)))
    {
        result = FOC_LSI_PRELOAD_RESULT_PERMIT_REJECTED;
        goto reject;
    }
    if ((foc_lsi_preload_bool_is_valid(safety->timer_clock_enabled) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->timer_configured) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->compare_preload_enabled) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->gate_enabled) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->main_output_enabled) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->channel_outputs_enabled) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->hardware_fault) == 0U) ||
        (foc_lsi_preload_bool_is_valid(safety->software_trip) == 0U) ||
        (safety->timer_clock_enabled == 0U) ||
        (safety->timer_configured == 0U) ||
        (safety->compare_preload_enabled == 0U) ||
        (safety->hardware_fault != 0U) ||
        (safety->software_trip != 0U) ||
        (*registers->period != session->pwm_period_ticks))
    {
        result = FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE;
        goto reject;
    }
    if (((session->validation_open != 0U) &&
         ((safety->gate_enabled != 0U) ||
          (safety->main_output_enabled != 0U) ||
          (safety->channel_outputs_enabled != 0U))) ||
        ((session->active_open != 0U) &&
         (((session->active_write_count == 0U) &&
           ((safety->gate_enabled != 0U) ||
            (safety->main_output_enabled != 0U) ||
            (safety->channel_outputs_enabled != 0U))) ||
          ((session->active_write_count != 0U) &&
           ((safety->gate_enabled == 0U) ||
            (safety->main_output_enabled == 0U) ||
            (safety->channel_outputs_enabled == 0U))))))
    {
        result = FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE;
        goto reject;
    }

    *registers->compare_u = permit->compare_u;
    *registers->compare_v = permit->compare_v;
    *registers->compare_w = permit->compare_w;
    if ((*registers->compare_u != permit->compare_u) ||
        (*registers->compare_v != permit->compare_v) ||
        (*registers->compare_w != permit->compare_w))
    {
        result = FOC_LSI_PRELOAD_RESULT_REGISTER_MISMATCH;
        goto reject;
    }
    session->permit_consumed = 1U;
    if (session->validation_open != 0U)
    {
        session->validation_open = 0U;
    }
    else
    {
        ++session->active_write_count;
        session->last_source_control_tick = output->source_control_tick;
        session->has_last_source_control_tick = 1U;
        session->last_consumed_sequence = permit->permit_sequence;
        ++session->next_permit_sequence;
        if (session->next_permit_sequence == 0U)
        {
            session->next_permit_sequence = 1U;
        }
        session->permit_issued = 0U;
        session->permit_consumed = 0U;
    }
    return FOC_LSI_PRELOAD_RESULT_OK;

reject:
    foc_lsi_preload_registers_safe(registers);
    session->validation_open = 0U;
    session->active_open = 0U;
    return result;
}

void foc_lsi_preload_session_abort(foc_lsi_preload_session_t *session)
{
    if (session == 0)
    {
        return;
    }
    session->validation_open = 0U;
    session->active_open = 0U;
    session->permit_issued = 0U;
    session->permit_consumed = 0U;
}
