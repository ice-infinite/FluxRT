#include "foc_monitor_sampling.h"

foc_monitor_sampling_plan_t foc_monitor_sampling_plan(
    uint32_t sync_running,
    uint32_t adc1_regular_owned)
{
    foc_monitor_sampling_plan_t plan =
        FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2;

    if (adc1_regular_owned != 0U)
    {
        return plan | FOC_MONITOR_SAMPLE_VBUS_ADC2;
    }

    plan |= FOC_MONITOR_SAMPLE_VBUS_ADC1 |
            FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1;
    if (sync_running == 0U)
    {
        plan |= FOC_MONITOR_SAMPLE_PHASE_CURRENTS;
    }
    return plan;
}
