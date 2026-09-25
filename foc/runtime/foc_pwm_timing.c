#include "foc_pwm_timing.h"

uint32_t foc_pwm_timing_ratio(const foc_pwm_timing_plan_t *plan)
{
    if ((plan == 0) || (plan->pwm_frequency_hz == 0U) ||
        (plan->control_frequency_hz == 0U) ||
        ((plan->pwm_frequency_hz % plan->control_frequency_hz) != 0U))
    {
        return 0U;
    }
    return plan->pwm_frequency_hz / plan->control_frequency_hz;
}

uint32_t foc_pwm_timing_plan_is_valid(const foc_pwm_timing_plan_t *plan)
{
    uint32_t ratio;
    uint32_t expected_period;

    if ((plan == 0) || (plan->timer_clock_hz == 0U) ||
        (plan->pwm_frequency_hz == 0U))
    {
        return 0U;
    }
    ratio = foc_pwm_timing_ratio(plan);
    if ((ratio == 0U) || (plan->pwm_frequency_hz > (plan->timer_clock_hz / 2U)))
    {
        return 0U;
    }
    expected_period = plan->timer_clock_hz / (2U * plan->pwm_frequency_hz);
    if ((expected_period < 2U) || (plan->pwm_period_ticks != expected_period) ||
        ((plan->repetition_counter + 1U) != (2U * ratio)) ||
        (plan->actuation_delay_pwm_ticks != ratio))
    {
        return 0U;
    }

    if (plan->adc_trigger == FOC_PWM_ADC_TRIGGER_OC4REF_RISING)
    {
        return (ratio == 1U) ? 1U : 0U;
    }
    if (plan->adc_trigger == FOC_PWM_ADC_TRIGGER_DIVIDED_OC4REF)
    {
        return (ratio > 1U) ? 1U : 0U;
    }
    /* TIM1 Update 的频率可以正确，但相位不等于有效的三分流采样窗口。 */
    return 0U;
}
