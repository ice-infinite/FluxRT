#include "foc_phase_voltage_model.h"

#include <stdint.h>

uint32_t foc_phase_voltage_model_validate(
    const foc_phase_voltage_model_t *model)
{
    uint64_t expected_full_scale_numerator;
    uint32_t expected_full_scale_mv;
    uint32_t expected_uv_per_count;

    if ((model == 0) ||
        (model->struct_size != sizeof(*model)) ||
        (model->version != FOC_PHASE_VOLTAGE_MODEL_VERSION) ||
        (model->adc_reference_mv == 0U) ||
        (model->adc_max_code == 0U) ||
        (model->divider_upper_ohm == 0U) ||
        (model->divider_lower_ohm == 0U))
    {
        return 0U;
    }

    /* v1 只有共用的理论分压关系，没有逐相 slope/intercept。接受 calibrated 会让
     * 三相误差和偏置被静默丢掉，因此必须等结构升版后再支持。 */
    if ((model->source != FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL) ||
        (model->flags !=
         (FOC_PHASE_VOLTAGE_MODEL_FLAG_NOMINAL_COMPONENTS |
          FOC_PHASE_VOLTAGE_MODEL_FLAG_DIAGNOSTIC_ONLY)))
    {
        return 0U;
    }

    expected_full_scale_numerator =
        (uint64_t)model->adc_reference_mv *
        ((uint64_t)model->divider_upper_ohm +
         (uint64_t)model->divider_lower_ohm);
    expected_full_scale_mv = (uint32_t)((expected_full_scale_numerator +
        ((uint64_t)model->divider_lower_ohm / 2ULL)) /
        (uint64_t)model->divider_lower_ohm);
    expected_uv_per_count = (uint32_t)((
        ((uint64_t)expected_full_scale_mv * 1000ULL) +
        ((uint64_t)model->adc_max_code / 2ULL)) /
        (uint64_t)model->adc_max_code);

    return ((model->phase_full_scale_mv == expected_full_scale_mv) &&
            (model->volts_per_count_uv == expected_uv_per_count)) ? 1U : 0U;
}

uint32_t foc_phase_voltage_raw_to_mv(
    const foc_phase_voltage_model_t *model,
    foc_phase_voltage_divider_mode_t divider_mode,
    uint16_t raw,
    uint32_t *phase_voltage_mv)
{
    uint64_t numerator;

    if (phase_voltage_mv == 0)
    {
        return 0U;
    }
    *phase_voltage_mv = 0U;
    if ((foc_phase_voltage_model_validate(model) == 0U) ||
        (divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_ENABLED) ||
        ((uint32_t)raw > model->adc_max_code))
    {
        return 0U;
    }

    numerator = (uint64_t)raw * (uint64_t)model->phase_full_scale_mv;
    *phase_voltage_mv = (uint32_t)((numerator +
        ((uint64_t)model->adc_max_code / 2ULL)) /
        (uint64_t)model->adc_max_code);
    return 1U;
}
