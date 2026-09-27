#include "foc_lsi_sample_contract.h"

uint32_t foc_lsi_sample_contract_is_valid(
    const foc_lsi_sample_contract_t *contract)
{
    return ((contract != 0) &&
            (contract->struct_size == sizeof(*contract)) &&
            (contract->version == FOC_LSI_SAMPLE_CONTRACT_VERSION) &&
            (contract->adc_max_code > 0U) &&
            (contract->adc_max_code <= 65535U) &&
            (contract->expected_pwm_period_ticks > 0U) &&
            (contract->expected_pwm_period_ticks <= 65535U)) ? 1U : 0U;
}

foc_lsi_sample_result_t foc_lsi_sample_validate(
    const foc_lsi_sample_contract_t *contract,
    const foc_lsi_raw_sample_t *previous,
    const foc_lsi_raw_sample_t *sample)
{
    uint32_t pulse_flags;

    if ((foc_lsi_sample_contract_is_valid(contract) == 0U) ||
        (sample == 0))
    {
        return FOC_LSI_SAMPLE_INVALID_ARGUMENT;
    }
    if (((previous == 0) && (sample->sequence != 0U)) ||
        ((previous != 0) &&
         (sample->sequence != (previous->sequence + 1U))))
    {
        return FOC_LSI_SAMPLE_SEQUENCE_ERROR;
    }
    if ((previous != 0) &&
        (sample->control_tick != (previous->control_tick + 1U)))
    {
        return FOC_LSI_SAMPLE_CONTROL_TICK_ERROR;
    }
    if (((uint32_t)sample->current_u_raw > contract->adc_max_code) ||
        ((uint32_t)sample->current_v_raw > contract->adc_max_code) ||
        ((uint32_t)sample->bus_voltage_raw > contract->adc_max_code))
    {
        return FOC_LSI_SAMPLE_ADC_RANGE_ERROR;
    }
    if ((uint32_t)sample->pwm_period_ticks !=
        contract->expected_pwm_period_ticks)
    {
        return FOC_LSI_SAMPLE_PWM_PERIOD_ERROR;
    }
    if (((uint32_t)sample->compare_u > sample->pwm_period_ticks) ||
        ((uint32_t)sample->compare_v > sample->pwm_period_ticks) ||
        ((uint32_t)sample->compare_w > sample->pwm_period_ticks))
    {
        return FOC_LSI_SAMPLE_COMPARE_RANGE_ERROR;
    }
    if ((sample->flags & FOC_LSI_RAW_FLAG_ADC_VALID) == 0U)
    {
        return FOC_LSI_SAMPLE_ADC_INVALID;
    }
    if ((sample->flags & (FOC_LSI_RAW_FLAG_HARDWARE_FAULT |
                          FOC_LSI_RAW_FLAG_SOFTWARE_TRIP)) != 0U)
    {
        return FOC_LSI_SAMPLE_FAULT_FLAGGED;
    }
    pulse_flags = sample->flags & (FOC_LSI_RAW_FLAG_PULSE_POSITIVE |
                                   FOC_LSI_RAW_FLAG_PULSE_NEGATIVE);
    if ((pulse_flags == (FOC_LSI_RAW_FLAG_PULSE_POSITIVE |
                         FOC_LSI_RAW_FLAG_PULSE_NEGATIVE)) ||
        ((pulse_flags != 0U) &&
         ((sample->flags & FOC_LSI_RAW_FLAG_DRIVE_ACTIVE) == 0U)))
    {
        return FOC_LSI_SAMPLE_POLARITY_ERROR;
    }
    return FOC_LSI_SAMPLE_OK;
}
