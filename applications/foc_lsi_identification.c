/*
 * FluxRT - pure Host-testable application Ls(I) experiment state machine.
 *
 * No allocation, logging, locks, RTOS calls, HAL calls, or register access are
 * permitted here. An output is only a request to the future platform adapter;
 * FORCE_SAFE remains the fail-closed default for every non-driving state.
 */

#include "foc_lsi_identification.h"

#include <math.h>
#include <string.h>

#define FOC_LSI_MAX_OFFSET_SAMPLES (512UL)
#define FOC_LSI_MAX_PULSE_PAIRS    (32UL)
#define FOC_LSI_MAX_BIAS_CURRENT_A (0.6f)
#define FOC_LSI_MAX_PERTURBATION_V (0.4f)
#define FOC_LSI_MAX_TRIP_CURRENT_A (1.15f)
#define FOC_LSI_MIN_BUS_VOLTAGE_V  (7.0f)
#define FOC_LSI_MAX_BUS_VOLTAGE_V  (18.0f)

static uint32_t foc_lsi_bool_is_valid(uint32_t value)
{
    return (value <= 1U) ? 1U : 0U;
}

static uint32_t foc_lsi_state_is_active(foc_lsi_state_t state)
{
    return ((state == FOC_LSI_STATE_BIAS_SETTLE) ||
            (state == FOC_LSI_STATE_PULSE_POSITIVE) ||
            (state == FOC_LSI_STATE_PULSE_NEGATIVE)) ? 1U : 0U;
}

static void foc_lsi_enter_state(foc_lsi_context_t *context,
                                foc_lsi_state_t state)
{
    context->state = state;
    context->state_ticks = 0U;
}

static void foc_lsi_abort(foc_lsi_context_t *context,
                          foc_lsi_abort_reason_t reason)
{
    context->abort_reason = reason;
    foc_lsi_enter_state(context, FOC_LSI_STATE_ABORTED);
}

uint32_t foc_lsi_config_is_valid(const foc_lsi_config_t *config)
{
    uint64_t active_ticks;
    uint64_t minimum_total_ticks;

    if ((config == 0) ||
        (config->struct_size != sizeof(*config)) ||
        (config->version != FOC_LSI_IDENTIFICATION_VERSION) ||
        (config->sample_rate_hz != FOC_LSI_SAMPLE_RATE_HZ) ||
        (config->offset_sample_count == 0U) ||
        (config->offset_sample_count > FOC_LSI_MAX_OFFSET_SAMPLES) ||
        (config->bias_settle_ticks == 0U) ||
        (config->pulse_ticks_per_polarity == 0U) ||
        (config->pulse_pair_count == 0U) ||
        (config->pulse_pair_count > FOC_LSI_MAX_PULSE_PAIRS) ||
        (config->cooldown_zero_ticks == 0U) ||
        !isfinite(config->bias_current_a) ||
        !isfinite(config->perturbation_voltage_v) ||
        !isfinite(config->current_trip_a) ||
        !isfinite(config->cooldown_current_threshold_a) ||
        !isfinite(config->bus_voltage_min_v) ||
        !isfinite(config->bus_voltage_max_v))
    {
        return 0U;
    }

    active_ticks = (uint64_t)config->bias_settle_ticks +
                   (2ULL * (uint64_t)config->pulse_ticks_per_polarity *
                    (uint64_t)config->pulse_pair_count);
    minimum_total_ticks = 1ULL + (uint64_t)config->offset_sample_count +
                          active_ticks +
                          (uint64_t)config->cooldown_zero_ticks;

    if ((config->max_active_ticks < active_ticks) ||
        (config->max_active_ticks > (FOC_LSI_SAMPLE_RATE_HZ / 20U)) ||
        ((uint64_t)config->total_timeout_ticks <= minimum_total_ticks) ||
        (config->total_timeout_ticks > (FOC_LSI_SAMPLE_RATE_HZ / 2U)) ||
        (config->bias_current_a <= 0.0f) ||
        (config->bias_current_a > FOC_LSI_MAX_BIAS_CURRENT_A) ||
        (config->perturbation_voltage_v <= 0.0f) ||
        (config->perturbation_voltage_v > FOC_LSI_MAX_PERTURBATION_V) ||
        (config->current_trip_a <= config->bias_current_a) ||
        (config->current_trip_a > FOC_LSI_MAX_TRIP_CURRENT_A) ||
        (config->cooldown_current_threshold_a < 0.0f) ||
        (config->cooldown_current_threshold_a >= config->bias_current_a) ||
        (config->bus_voltage_min_v < FOC_LSI_MIN_BUS_VOLTAGE_V) ||
        (config->bus_voltage_max_v > FOC_LSI_MAX_BUS_VOLTAGE_V) ||
        (config->bus_voltage_min_v >= config->bus_voltage_max_v))
    {
        return 0U;
    }
    return 1U;
}

uint32_t foc_lsi_build_is_authorized(void)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    return 1U;
#else
    return 0U;
#endif
}

void foc_lsi_get_output(const foc_lsi_context_t *context,
                        foc_lsi_output_t *output)
{
    if (output == 0)
    {
        return;
    }
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->version = FOC_LSI_OUTPUT_VERSION;
    output->force_safe_output = 1U;
    if (context == 0)
    {
        output->state = FOC_LSI_STATE_ABORTED;
        output->abort_reason = FOC_LSI_ABORT_INVALID_INPUT;
        return;
    }

    output->state = context->state;
    output->abort_reason = context->abort_reason;
    output->pulse_pair_index = context->pulse_pair_index;

    switch (context->state)
    {
    case FOC_LSI_STATE_OFFSET_CAL:
        output->capture_offset_sample = 1U;
        break;
    case FOC_LSI_STATE_BIAS_SETTLE:
        output->drive_request = FOC_LSI_DRIVE_BIAS;
        output->force_safe_output = 0U;
        output->capture_raw_sample = 1U;
        output->requested_bias_current_a = context->config.bias_current_a;
        break;
    case FOC_LSI_STATE_PULSE_POSITIVE:
        output->drive_request = FOC_LSI_DRIVE_PULSE_POSITIVE;
        output->force_safe_output = 0U;
        output->capture_raw_sample = 1U;
        output->requested_bias_current_a = context->config.bias_current_a;
        output->requested_perturbation_voltage_v =
            context->config.perturbation_voltage_v;
        break;
    case FOC_LSI_STATE_PULSE_NEGATIVE:
        output->drive_request = FOC_LSI_DRIVE_PULSE_NEGATIVE;
        output->force_safe_output = 0U;
        output->capture_raw_sample = 1U;
        output->requested_bias_current_a = context->config.bias_current_a;
        output->requested_perturbation_voltage_v =
            -context->config.perturbation_voltage_v;
        break;
    case FOC_LSI_STATE_COOLDOWN:
        output->capture_raw_sample = 1U;
        break;
    default:
        break;
    }
}

uint32_t foc_lsi_init(foc_lsi_context_t *context,
                      const foc_lsi_config_t *config)
{
    if (context == 0)
    {
        return 0U;
    }
    memset(context, 0, sizeof(*context));
    context->struct_size = sizeof(*context);
    context->version = FOC_LSI_IDENTIFICATION_VERSION;
    context->state = FOC_LSI_STATE_IDLE;
    if (foc_lsi_config_is_valid(config) == 0U)
    {
        return 0U;
    }
    context->config = *config;
    context->config_valid = 1U;
    return 1U;
}

uint32_t foc_lsi_request_start(foc_lsi_context_t *context,
                               uint32_t confirmation)
{
    if ((context == 0) ||
        (context->struct_size != sizeof(*context)) ||
        (context->version != FOC_LSI_IDENTIFICATION_VERSION) ||
        (context->config_valid == 0U) ||
        (context->state != FOC_LSI_STATE_IDLE) ||
        (foc_lsi_build_is_authorized() == 0U) ||
        (confirmation != FOC_LSI_START_CONFIRMATION))
    {
        return 0U;
    }
    context->abort_reason = FOC_LSI_ABORT_NONE;
    context->elapsed_ticks = 0U;
    context->active_ticks = 0U;
    context->offset_samples = 0U;
    context->pulse_pair_index = 0U;
    context->cooldown_zero_count = 0U;
    foc_lsi_enter_state(context, FOC_LSI_STATE_PREFLIGHT);
    return 1U;
}

static uint32_t foc_lsi_input_is_valid(const foc_lsi_input_t *input)
{
    return ((input != 0) &&
            (input->struct_size == sizeof(*input)) &&
            (input->version == FOC_LSI_INPUT_VERSION) &&
            (foc_lsi_bool_is_valid(input->identification_build_authorized) != 0U) &&
            (foc_lsi_bool_is_valid(input->power_stage_idle) != 0U) &&
            (foc_lsi_bool_is_valid(input->motor_stopped) != 0U) &&
            (foc_lsi_bool_is_valid(input->hardware_fault) != 0U) &&
            (foc_lsi_bool_is_valid(input->software_trip) != 0U) &&
            (foc_lsi_bool_is_valid(input->abort_requested) != 0U) &&
            (foc_lsi_bool_is_valid(input->offset_sample_valid) != 0U) &&
            isfinite(input->bus_voltage_v) &&
            isfinite(input->abs_phase_current_a) &&
            (input->abs_phase_current_a >= 0.0f)) ? 1U : 0U;
}

foc_lsi_state_t foc_lsi_step(foc_lsi_context_t *context,
                             const foc_lsi_input_t *input,
                             foc_lsi_output_t *output)
{
    if ((context == 0) ||
        (context->struct_size != sizeof(*context)) ||
        (context->version != FOC_LSI_IDENTIFICATION_VERSION) ||
        (context->config_valid == 0U))
    {
        foc_lsi_get_output(0, output);
        return FOC_LSI_STATE_ABORTED;
    }
    if ((context->state == FOC_LSI_STATE_IDLE) ||
        (context->state == FOC_LSI_STATE_COMPLETE) ||
        (context->state == FOC_LSI_STATE_ABORTED))
    {
        foc_lsi_get_output(context, output);
        return context->state;
    }
    if (foc_lsi_input_is_valid(input) == 0U)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_INVALID_INPUT);
        foc_lsi_get_output(context, output);
        return context->state;
    }
    if (input->abort_requested != 0U)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_REQUESTED);
    }
    else if (input->hardware_fault != 0U)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_HARDWARE_FAULT);
    }
    else if (input->software_trip != 0U)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_SOFTWARE_TRIP);
    }
    else if (input->abs_phase_current_a >= context->config.current_trip_a)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_CURRENT_LIMIT);
    }
    else if ((input->bus_voltage_v < context->config.bus_voltage_min_v) ||
             (input->bus_voltage_v > context->config.bus_voltage_max_v))
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_BUS_VOLTAGE);
    }
    if (context->state == FOC_LSI_STATE_ABORTED)
    {
        foc_lsi_get_output(context, output);
        return context->state;
    }

    if (context->elapsed_ticks >= context->config.total_timeout_ticks)
    {
        foc_lsi_abort(context, FOC_LSI_ABORT_TOTAL_TIMEOUT);
        foc_lsi_get_output(context, output);
        return context->state;
    }
    ++context->elapsed_ticks;

    if (foc_lsi_state_is_active(context->state) != 0U)
    {
        if (context->active_ticks >= context->config.max_active_ticks)
        {
            foc_lsi_abort(context, FOC_LSI_ABORT_ACTIVE_TIMEOUT);
            foc_lsi_get_output(context, output);
            return context->state;
        }
        ++context->active_ticks;
    }

    switch (context->state)
    {
    case FOC_LSI_STATE_PREFLIGHT:
        if (input->identification_build_authorized == 0U)
        {
            foc_lsi_abort(context, FOC_LSI_ABORT_UNAUTHORIZED_BUILD);
        }
        else if (input->power_stage_idle == 0U)
        {
            foc_lsi_abort(context, FOC_LSI_ABORT_POWER_STAGE_NOT_IDLE);
        }
        else if (input->motor_stopped == 0U)
        {
            foc_lsi_abort(context, FOC_LSI_ABORT_MOTOR_NOT_STOPPED);
        }
        else
        {
            foc_lsi_enter_state(context, FOC_LSI_STATE_OFFSET_CAL);
        }
        break;
    case FOC_LSI_STATE_OFFSET_CAL:
        if (input->offset_sample_valid != 0U)
        {
            ++context->offset_samples;
        }
        if (context->offset_samples >= context->config.offset_sample_count)
        {
            foc_lsi_enter_state(context, FOC_LSI_STATE_BIAS_SETTLE);
        }
        break;
    case FOC_LSI_STATE_BIAS_SETTLE:
        ++context->state_ticks;
        if (context->state_ticks >= context->config.bias_settle_ticks)
        {
            foc_lsi_enter_state(context, FOC_LSI_STATE_PULSE_POSITIVE);
        }
        break;
    case FOC_LSI_STATE_PULSE_POSITIVE:
        ++context->state_ticks;
        if (context->state_ticks >= context->config.pulse_ticks_per_polarity)
        {
            foc_lsi_enter_state(context, FOC_LSI_STATE_PULSE_NEGATIVE);
        }
        break;
    case FOC_LSI_STATE_PULSE_NEGATIVE:
        ++context->state_ticks;
        if (context->state_ticks >= context->config.pulse_ticks_per_polarity)
        {
            ++context->pulse_pair_index;
            if (context->pulse_pair_index >= context->config.pulse_pair_count)
            {
                context->cooldown_zero_count = 0U;
                foc_lsi_enter_state(context, FOC_LSI_STATE_COOLDOWN);
            }
            else
            {
                foc_lsi_enter_state(context, FOC_LSI_STATE_PULSE_POSITIVE);
            }
        }
        break;
    case FOC_LSI_STATE_COOLDOWN:
        if (input->abs_phase_current_a <=
            context->config.cooldown_current_threshold_a)
        {
            ++context->cooldown_zero_count;
        }
        else
        {
            context->cooldown_zero_count = 0U;
        }
        if (context->cooldown_zero_count >=
            context->config.cooldown_zero_ticks)
        {
            foc_lsi_enter_state(context, FOC_LSI_STATE_COMPLETE);
        }
        break;
    default:
        foc_lsi_abort(context, FOC_LSI_ABORT_INVALID_INPUT);
        break;
    }

    foc_lsi_get_output(context, output);
    return context->state;
}

uint32_t foc_lsi_reset(foc_lsi_context_t *context,
                       foc_lsi_output_t *output)
{
    if ((context == 0) ||
        ((context->state != FOC_LSI_STATE_COMPLETE) &&
         (context->state != FOC_LSI_STATE_ABORTED)))
    {
        foc_lsi_get_output(context, output);
        return 0U;
    }
    context->abort_reason = FOC_LSI_ABORT_NONE;
    context->elapsed_ticks = 0U;
    context->active_ticks = 0U;
    context->state_ticks = 0U;
    context->offset_samples = 0U;
    context->pulse_pair_index = 0U;
    context->cooldown_zero_count = 0U;
    context->state = FOC_LSI_STATE_IDLE;
    foc_lsi_get_output(context, output);
    return 1U;
}
