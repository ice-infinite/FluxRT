#include "foc_monitor_sampling.h"

#include <assert.h>

int main(void)
{
    foc_monitor_sampling_plan_t plan;

    plan = foc_monitor_sampling_plan(0U, 0U);
    assert((plan & FOC_MONITOR_SAMPLE_PHASE_CURRENTS) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC1) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC2) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1) != 0U);

    plan = foc_monitor_sampling_plan(1U, 0U);
    assert((plan & FOC_MONITOR_SAMPLE_PHASE_CURRENTS) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC1) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC2) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1) != 0U);

    plan = foc_monitor_sampling_plan(0U, 1U);
    assert((plan & FOC_MONITOR_SAMPLE_PHASE_CURRENTS) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC1) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC2) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1) == 0U);

    plan = foc_monitor_sampling_plan(1U, 1U);
    assert((plan & FOC_MONITOR_SAMPLE_PHASE_CURRENTS) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC1) == 0U);
    assert((plan & FOC_MONITOR_SAMPLE_VBUS_ADC2) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2) != 0U);
    assert((plan & FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1) == 0U);
    return 0;
}
