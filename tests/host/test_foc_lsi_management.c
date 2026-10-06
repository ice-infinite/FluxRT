#include <stdint.h>
#include <string.h>

#include "foc_lsi_management.h"

static int status_is_safe_idle(const foc_lsi_management_status_t *status)
{
    return ((status->build_authorized == 1U) &&
            (status->identification_start_command_exposed == 1U) &&
            (status->pwm_adapter_present == 1U) &&
            (status->adc_adapter_present == 1U) &&
            (status->output.state == FOC_LSI_STATE_IDLE) &&
            (status->output.abort_reason == FOC_LSI_ABORT_NONE) &&
            (status->output.force_safe_output == 1U) &&
            (status->output.drive_request == FOC_LSI_DRIVE_OFF));
}

int main(void)
{
    foc_lsi_management_t management;
    foc_lsi_management_status_t status;
    foc_lsi_config_t config;
    foc_lsi_management_result_t result;

    foc_lsi_management_default_config(&config);
    if ((foc_lsi_config_is_valid(&config) == 0U) ||
        (config.sample_rate_hz != 12000U) ||
        (config.offset_sample_count != 256U) ||
        (config.bias_settle_ticks != 24U) ||
        (config.pulse_ticks_per_polarity != 6U) ||
        (config.pulse_pair_count != 6U) ||
        (config.cooldown_zero_ticks != 24U) ||
        ((config.bias_settle_ticks +
          (2U * config.pulse_ticks_per_polarity * config.pulse_pair_count) +
          config.cooldown_zero_ticks) != 120U) ||
        ((1U + config.offset_sample_count + config.bias_settle_ticks +
          (2U * config.pulse_ticks_per_polarity * config.pulse_pair_count) +
          config.cooldown_zero_ticks) != 377U) ||
        (config.bias_current_a != FOC_LSI_H2_FIRST_MAX_BIAS_CURRENT_A) ||
        (config.perturbation_voltage_v !=
         FOC_LSI_H2_FIRST_MAX_PERTURBATION_V) ||
        (config.current_trip_a != FOC_LSI_H2_FIRST_CURRENT_TRIP_A) ||
        (config.max_active_ticks != 600U) ||
        (config.total_timeout_ticks != 6000U))
    {
        return 1;
    }
    {
        foc_lsi_actuation_config_t actuation = {0};
        foc_lsi_actuation_config_t original;
        foc_lsi_config_t widened = config;

        actuation.struct_size = sizeof(actuation);
        actuation.version = FOC_LSI_ACTUATION_CONFIG_VERSION;
        actuation.maximum_bias_current_a = 0.2f;
        actuation.maximum_perturbation_voltage_v = 0.4f;
        actuation.current_trip_a = 1.15f;
        actuation.minimum_bus_voltage_v = 7.0f;
        actuation.maximum_bus_voltage_v = 18.0f;
        if ((foc_lsi_management_apply_h2_actuation_envelope(
                 &config, &actuation) == 0U) ||
            (actuation.maximum_bias_current_a !=
             FOC_LSI_H2_FIRST_MAX_BIAS_CURRENT_A) ||
            (actuation.maximum_perturbation_voltage_v !=
             FOC_LSI_H2_FIRST_MAX_PERTURBATION_V) ||
            (actuation.current_trip_a != FOC_LSI_H2_FIRST_CURRENT_TRIP_A))
        {
            return 15;
        }

        widened.perturbation_voltage_v = 0.4f;
        widened.current_trip_a = 1.15f;
        original = actuation;
        if ((foc_lsi_management_apply_h2_actuation_envelope(
                 &widened, &actuation) != 0U) ||
            (memcmp(&actuation, &original, sizeof(actuation)) != 0))
        {
            return 16;
        }
    }
    if ((foc_lsi_management_init(&management) == 0U) ||
        (foc_lsi_management_get_status(&management, &status) == 0U) ||
        !status_is_safe_idle(&status))
    {
        return 2;
    }

    result = foc_lsi_management_abort(&management, &status);
    if ((result != FOC_LSI_MANAGEMENT_ALREADY_SAFE) ||
        !status_is_safe_idle(&status) ||
        (status.abort_command_count != 1U))
    {
        return 3;
    }

    result = foc_lsi_management_request_start(&management,
                                              0U,
                                              &status);
    if ((result != FOC_LSI_MANAGEMENT_REFUSED) ||
        (status.start_command_count != 1U) ||
        (status.start_consumed != 0U) ||
        !status_is_safe_idle(&status))
    {
        return 4;
    }
    result = foc_lsi_management_request_start(&management,
                                              FOC_LSI_START_CONFIRMATION,
                                              &status);
    if ((result != FOC_LSI_MANAGEMENT_OK) ||
        (management.context.state != FOC_LSI_STATE_PREFLIGHT))
    {
        return 5;
    }
    result = foc_lsi_management_reset(&management, &status);
    if ((result != FOC_LSI_MANAGEMENT_OK) ||
        !status_is_safe_idle(&status) ||
        (status.abort_command_count != 2U) ||
        (status.reset_command_count != 1U) ||
        (status.start_command_count != 2U) ||
        (status.start_consumed != 1U))
    {
        return 6;
    }

    result = foc_lsi_management_request_start(&management,
                                              FOC_LSI_START_CONFIRMATION,
                                              &status);
    if ((result != FOC_LSI_MANAGEMENT_REFUSED) ||
        (status.start_command_count != 3U) ||
        (status.start_consumed != 1U) ||
        !status_is_safe_idle(&status))
    {
        return 7;
    }

    /* Simulated reboot: all command counters and state must return to defaults. */
    if ((foc_lsi_management_init(&management) == 0U) ||
        (foc_lsi_management_get_status(&management, &status) == 0U) ||
        !status_is_safe_idle(&status) ||
        (status.abort_command_count != 0U) ||
        (status.reset_command_count != 0U) ||
        (status.start_command_count != 0U) ||
        (status.start_consumed != 0U))
    {
        return 8;
    }

    if ((foc_lsi_management_shared_init() == 0U) ||
        (foc_lsi_management_shared_get_status(&status) == 0U) ||
        !status_is_safe_idle(&status))
    {
        return 9;
    }
    result = foc_lsi_management_shared_request_start(0U, &status);
    if ((result != FOC_LSI_MANAGEMENT_REFUSED) ||
        (status.start_command_count != 1U) ||
        (status.start_consumed != 0U))
    {
        return 10;
    }
    result = foc_lsi_management_shared_request_start(
        FOC_LSI_START_CONFIRMATION, &status);
    if ((result != FOC_LSI_MANAGEMENT_OK) ||
        (status.output.state != FOC_LSI_STATE_PREFLIGHT) ||
        (status.start_command_count != 2U) ||
        (status.start_consumed != 1U))
    {
        return 11;
    }
    foc_lsi_management_shared_abort_isr();
    if ((foc_lsi_management_shared_get_status(&status) == 0U) ||
        (status.output.state != FOC_LSI_STATE_ABORTED) ||
        (status.output.abort_reason != FOC_LSI_ABORT_REQUESTED) ||
        (status.abort_command_count != 0U))
    {
        return 12;
    }
    result = foc_lsi_management_shared_reset(&status);
    if ((result != FOC_LSI_MANAGEMENT_OK) ||
        !status_is_safe_idle(&status) ||
        (status.reset_command_count != 1U) ||
        (status.start_consumed != 1U))
    {
        return 13;
    }
    result = foc_lsi_management_shared_request_start(
        FOC_LSI_START_CONFIRMATION, &status);
    if ((result != FOC_LSI_MANAGEMENT_REFUSED) ||
        (status.start_command_count != 3U) ||
        (status.start_consumed != 1U))
    {
        return 14;
    }
    return 0;
}
