#ifndef FOC_MONITOR_SAMPLING_H
#define FOC_MONITOR_SAMPLING_H

/* Pure routing policy for stopped-state monitor conversions. Keeping this
 * decision independent from STM32 HAL makes the ADC ownership matrix directly
 * host-testable. */

#include <stdint.h>

typedef uint32_t foc_monitor_sampling_plan_t;

enum
{
    FOC_MONITOR_SAMPLE_PHASE_CURRENTS = (1UL << 0),
    FOC_MONITOR_SAMPLE_VBUS_ADC1 = (1UL << 1),
    FOC_MONITOR_SAMPLE_VBUS_ADC2 = (1UL << 2),
    FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2 = (1UL << 3),
    FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1 = (1UL << 4)
};

/* ADC1 regular conversions are never included while the external-input port
 * owns them. Vbus remains fresh through the same PA0/ADC12_IN1 signal routed
 * to ADC2. Phase currents are polled only while the synchronous injected
 * monitor is stopped and ADC1 is available. */
foc_monitor_sampling_plan_t foc_monitor_sampling_plan(
    uint32_t sync_running,
    uint32_t adc1_regular_owned);

#endif /* FOC_MONITOR_SAMPLING_H */
