/* FluxRT - pure, Host-testable EXP-B3 management layer. */

#include "foc_lsi_management.h"

#include <string.h>

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)

static foc_lsi_management_t g_foc_lsi_shared_management;
static uint32_t g_foc_lsi_shared_initialized;

static uint32_t foc_lsi_management_is_valid(
    const foc_lsi_management_t *management)
{
    return ((management != 0) &&
            (management->struct_size == sizeof(*management)) &&
            (management->version == FOC_LSI_MANAGEMENT_VERSION) &&
            (management->context.struct_size == sizeof(management->context)) &&
            (management->context.version == FOC_LSI_IDENTIFICATION_VERSION) &&
            (management->context.config_valid != 0U)) ? 1U : 0U;
}

void foc_lsi_management_default_config(foc_lsi_config_t *config)
{
    if (config == 0)
    {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->struct_size = sizeof(*config);
    config->version = FOC_LSI_IDENTIFICATION_VERSION;
    config->sample_rate_hz = FOC_LSI_SAMPLE_RATE_HZ;
    config->offset_sample_count = 256U;
    /* S5.2: the first powered window measured up to 1.76 ADC-count noise.
     * Keep six pulse pairs, but spend the bounded active window on longer
     * six-tick polarities instead of a ten-millisecond bias hold. */
    config->bias_settle_ticks = 24U;        /* 2 ms at 12 kHz. */
    config->pulse_ticks_per_polarity = 6U;
    config->pulse_pair_count = 6U;          /* 72 pulse samples. */
    config->cooldown_zero_ticks = 24U;      /* 2 ms below threshold. */
    config->max_active_ticks = 600U;        /* 50 ms hard active cap. */
    config->total_timeout_ticks = 6000U;    /* 500 ms total cap. */
    config->bias_current_a = 0.2f;
    config->perturbation_voltage_v = 0.4f;
    config->current_trip_a = 1.15f;
    config->cooldown_current_threshold_a = 0.05f;
    config->bus_voltage_min_v = 7.0f;
    config->bus_voltage_max_v = 18.0f;
}

uint32_t foc_lsi_management_init(foc_lsi_management_t *management)
{
    foc_lsi_config_t config;

    if (management == 0)
    {
        return 0U;
    }
    memset(management, 0, sizeof(*management));
    management->struct_size = sizeof(*management);
    management->version = FOC_LSI_MANAGEMENT_VERSION;
    foc_lsi_management_default_config(&config);
    return foc_lsi_init(&management->context, &config);
}

uint32_t foc_lsi_management_get_status(
    const foc_lsi_management_t *management,
    foc_lsi_management_status_t *status)
{
    if (status == 0)
    {
        return 0U;
    }
    memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_LSI_MANAGEMENT_VERSION;
    status->build_authorized = foc_lsi_build_is_authorized();
    /* S4.9 exposes one frozen start command and the dedicated PWM/ADC adapter.
     * These fields never mean that a session is currently armed. */
    status->identification_start_command_exposed = 1U;
    status->pwm_adapter_present = 1U;
    status->adc_adapter_present = 1U;
    if (foc_lsi_management_is_valid(management) == 0U)
    {
        foc_lsi_get_output(0, &status->output);
        return 0U;
    }
    status->abort_command_count = management->abort_command_count;
    status->reset_command_count = management->reset_command_count;
    status->start_command_count = management->start_command_count;
    status->start_consumed = management->start_consumed;
    foc_lsi_get_output(&management->context, &status->output);
    return 1U;
}

foc_lsi_management_result_t foc_lsi_management_request_start(
    foc_lsi_management_t *management,
    uint32_t confirmation,
    foc_lsi_management_status_t *status)
{
    if (foc_lsi_management_is_valid(management) == 0U)
    {
        (void)foc_lsi_management_get_status(management, status);
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    ++management->start_command_count;
    if ((management->start_consumed != 0U) ||
        (foc_lsi_request_start(&management->context, confirmation) == 0U))
    {
        (void)foc_lsi_management_get_status(management, status);
        return FOC_LSI_MANAGEMENT_REFUSED;
    }
    management->start_consumed = 1U;
    (void)foc_lsi_management_get_status(management, status);
    return FOC_LSI_MANAGEMENT_OK;
}

foc_lsi_state_t foc_lsi_management_step(
    foc_lsi_management_t *management,
    const foc_lsi_input_t *input,
    foc_lsi_output_t *output)
{
    if (foc_lsi_management_is_valid(management) == 0U)
    {
        foc_lsi_get_output(0, output);
        return FOC_LSI_STATE_ABORTED;
    }
    return foc_lsi_step(&management->context, input, output);
}

foc_lsi_management_result_t foc_lsi_management_abort(
    foc_lsi_management_t *management,
    foc_lsi_management_status_t *status)
{
    foc_lsi_input_t input;
    foc_lsi_state_t prior_state;

    if (foc_lsi_management_is_valid(management) == 0U)
    {
        (void)foc_lsi_management_get_status(management, status);
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    prior_state = management->context.state;
    ++management->abort_command_count;
    if ((prior_state != FOC_LSI_STATE_IDLE) &&
        (prior_state != FOC_LSI_STATE_COMPLETE) &&
        (prior_state != FOC_LSI_STATE_ABORTED))
    {
        memset(&input, 0, sizeof(input));
        input.struct_size = sizeof(input);
        input.version = FOC_LSI_INPUT_VERSION;
        input.identification_build_authorized = 1U;
        input.power_stage_idle = 1U;
        input.motor_stopped = 1U;
        input.abort_requested = 1U;
        input.bus_voltage_v = management->context.config.bus_voltage_min_v;
        (void)foc_lsi_step(&management->context, &input, 0);
    }
    (void)foc_lsi_management_get_status(management, status);
    return ((prior_state == FOC_LSI_STATE_IDLE) ||
            (prior_state == FOC_LSI_STATE_COMPLETE) ||
            (prior_state == FOC_LSI_STATE_ABORTED)) ?
        FOC_LSI_MANAGEMENT_ALREADY_SAFE : FOC_LSI_MANAGEMENT_OK;
}

foc_lsi_management_result_t foc_lsi_management_reset(
    foc_lsi_management_t *management,
    foc_lsi_management_status_t *status)
{
    foc_lsi_state_t prior_state;

    if (foc_lsi_management_is_valid(management) == 0U)
    {
        (void)foc_lsi_management_get_status(management, status);
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    prior_state = management->context.state;
    ++management->reset_command_count;
    if ((prior_state != FOC_LSI_STATE_IDLE) &&
        (prior_state != FOC_LSI_STATE_COMPLETE) &&
        (prior_state != FOC_LSI_STATE_ABORTED))
    {
        (void)foc_lsi_management_abort(management, 0);
    }
    if ((management->context.state == FOC_LSI_STATE_COMPLETE) ||
        (management->context.state == FOC_LSI_STATE_ABORTED))
    {
        (void)foc_lsi_reset(&management->context, 0);
    }
    /* Re-initialise IDLE too, so reset is a deterministic recovery command. */
    if (management->context.state == FOC_LSI_STATE_IDLE)
    {
        uint32_t abort_count = management->abort_command_count;
        uint32_t reset_count = management->reset_command_count;
        uint32_t start_count = management->start_command_count;
        uint32_t start_consumed = management->start_consumed;
        foc_lsi_config_t config = management->context.config;
        (void)foc_lsi_init(&management->context, &config);
        management->abort_command_count = abort_count;
        management->reset_command_count = reset_count;
        management->start_command_count = start_count;
        management->start_consumed = start_consumed;
    }
    (void)foc_lsi_management_get_status(management, status);
    return (prior_state == FOC_LSI_STATE_IDLE) ?
        FOC_LSI_MANAGEMENT_ALREADY_SAFE : FOC_LSI_MANAGEMENT_OK;
}

uint32_t foc_lsi_management_shared_init(void)
{
    if (g_foc_lsi_shared_initialized != 0U)
    {
        return 1U;
    }
    g_foc_lsi_shared_initialized =
        foc_lsi_management_init(&g_foc_lsi_shared_management);
    return g_foc_lsi_shared_initialized;
}

uint32_t foc_lsi_management_shared_get_status(
    foc_lsi_management_status_t *status)
{
    return (foc_lsi_management_shared_init() != 0U) ?
        foc_lsi_management_get_status(&g_foc_lsi_shared_management, status) :
        0U;
}

uint32_t foc_lsi_management_shared_get_config(foc_lsi_config_t *config)
{
    if ((config == 0) || (foc_lsi_management_shared_init() == 0U))
    {
        return 0U;
    }
    *config = g_foc_lsi_shared_management.context.config;
    return 1U;
}

foc_lsi_management_result_t foc_lsi_management_shared_request_start(
    uint32_t confirmation,
    foc_lsi_management_status_t *status)
{
    if (foc_lsi_management_shared_init() == 0U)
    {
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    return foc_lsi_management_request_start(&g_foc_lsi_shared_management,
                                            confirmation,
                                            status);
}

foc_lsi_state_t foc_lsi_management_shared_step(
    const foc_lsi_input_t *input,
    foc_lsi_output_t *output)
{
    if (foc_lsi_management_shared_init() == 0U)
    {
        foc_lsi_get_output(0, output);
        return FOC_LSI_STATE_ABORTED;
    }
    return foc_lsi_management_step(&g_foc_lsi_shared_management,
                                   input,
                                   output);
}

foc_lsi_management_result_t foc_lsi_management_shared_abort(
    foc_lsi_management_status_t *status)
{
    if (foc_lsi_management_shared_init() == 0U)
    {
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    return foc_lsi_management_abort(&g_foc_lsi_shared_management, status);
}

foc_lsi_management_result_t foc_lsi_management_shared_reset(
    foc_lsi_management_status_t *status)
{
    if (foc_lsi_management_shared_init() == 0U)
    {
        return FOC_LSI_MANAGEMENT_INVALID;
    }
    return foc_lsi_management_reset(&g_foc_lsi_shared_management, status);
}

void foc_lsi_management_shared_abort_isr(void)
{
    foc_lsi_input_t input;

    if ((g_foc_lsi_shared_initialized == 0U) ||
        (g_foc_lsi_shared_management.context.state == FOC_LSI_STATE_IDLE) ||
        (g_foc_lsi_shared_management.context.state == FOC_LSI_STATE_COMPLETE) ||
        (g_foc_lsi_shared_management.context.state == FOC_LSI_STATE_ABORTED))
    {
        return;
    }
    memset(&input, 0, sizeof(input));
    input.struct_size = sizeof(input);
    input.version = FOC_LSI_INPUT_VERSION;
    input.identification_build_authorized = 1U;
    input.power_stage_idle = 1U;
    input.motor_stopped = 1U;
    input.abort_requested = 1U;
    input.bus_voltage_v =
        g_foc_lsi_shared_management.context.config.bus_voltage_min_v;
    (void)foc_lsi_step(&g_foc_lsi_shared_management.context, &input, 0);
}

#endif
