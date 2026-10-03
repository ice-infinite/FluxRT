#ifndef FOC_EXTERNAL_INPUT_PLATFORM_H
#define FOC_EXTERNAL_INPUT_PLATFORM_H

/* Build-selected platform provider for the target-neutral raw input port. */

#include "foc_external_input_port.h"

/* Called after the motor-control platform initialized ADC/timers.  Binding
 * never starts a capture and therefore remains safe in the default-off image. */
foc_external_input_port_result_t foc_external_input_platform_bind(
    foc_external_input_port_t *port);

/* Hardware-IRQ hook.  The selected platform implementation must make this a
 * constant-time no-op while no input is enabled. */
void foc_external_input_platform_control_tick(void);

/* True only while the selected platform raw-input driver owns the ADC regular
 * conversion group. Slow monitor code must not reconfigure/start that group
 * until this returns false. */
uint32_t foc_external_input_platform_regular_adc_owned(void);

#endif /* FOC_EXTERNAL_INPUT_PLATFORM_H */
