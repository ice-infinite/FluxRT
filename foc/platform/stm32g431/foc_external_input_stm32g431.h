#ifndef FOC_EXTERNAL_INPUT_STM32G431_H
#define FOC_EXTERNAL_INPUT_STM32G431_H

/* Private STM32G431 raw-capture driver.  Register pointers are explicit so the
 * exact ADC scheduler can be exercised with fake registers on the host. */

#include <stdint.h>

#include "foc_external_input_port.h"

#define FOC_STM32G431_EXTERNAL_INPUT_DRIVER_VERSION (1UL)

typedef struct
{
    volatile uint32_t *isr;
    volatile uint32_t *cr;
    volatile uint32_t *sqr1;
    volatile uint32_t *smpr1;
    volatile uint32_t *dr;
} foc_stm32g431_external_input_adc_registers_t;

typedef struct
{
    uint32_t eoc_mask;
    uint32_t start_mask;
    uint32_t rank1_mask;
    uint32_t rank1_shift;
    uint32_t channel_index;
    uint32_t sample_mask;
    uint32_t sample_shift;
    uint32_t sample_code;
} foc_stm32g431_external_input_adc_bits_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    volatile uint32_t initialized;
    volatile uint32_t publish_guard;
    foc_external_input_mask_t supported_input_mask;
    volatile foc_external_input_mask_t enabled_input_mask;
    volatile uint32_t conversion_pending;
    uint32_t pending_timestamp_us;
    uint32_t missed_since_sample;
    foc_stm32g431_external_input_adc_registers_t adc;
    foc_stm32g431_external_input_adc_bits_t bits;
    foc_external_input_raw_sample_t latest;
    foc_external_input_raw_health_t health;
} foc_stm32g431_external_input_driver_t;

foc_external_input_port_result_t
foc_stm32g431_external_input_driver_init(
    foc_stm32g431_external_input_driver_t *driver,
    foc_external_input_mask_t supported_input_mask,
    const foc_stm32g431_external_input_adc_registers_t *adc,
    const foc_stm32g431_external_input_adc_bits_t *bits);

foc_external_input_port_result_t
foc_stm32g431_external_input_driver_bind_port(
    foc_stm32g431_external_input_driver_t *driver,
    foc_external_input_port_t *port);

/* Called by the ADC control IRQ after the injected sample was captured.  It
 * reads the previous non-blocking regular conversion and starts the next one.
 * No polling, allocation, logging or command dispatch is permitted here. */
foc_external_input_port_result_t
foc_stm32g431_external_input_driver_control_tick(
    foc_stm32g431_external_input_driver_t *driver,
    uint32_t sampled_at_us);

#endif /* FOC_EXTERNAL_INPUT_STM32G431_H */
