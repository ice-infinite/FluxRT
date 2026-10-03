#include "foc_external_input_stm32g431.h"

#include <assert.h>
#include <string.h>

int main(void)
{
    volatile uint32_t adc_isr = 0U;
    volatile uint32_t adc_cr = 0U;
    volatile uint32_t adc_sqr1 = 0xA5A50000UL;
    volatile uint32_t adc_smpr1 = 0x55000000UL;
    volatile uint32_t adc_dr = 0U;
    const foc_stm32g431_external_input_adc_registers_t registers =
    {
        &adc_isr, &adc_cr, &adc_sqr1, &adc_smpr1, &adc_dr,
    };
    const foc_stm32g431_external_input_adc_bits_t bits =
    {
        1UL << 2,
        1UL << 3,
        0x1FUL << 6,
        6U,
        8U,
        0x7UL << 24,
        24U,
        4U,
    };
    foc_stm32g431_external_input_driver_t driver;
    foc_external_input_port_t port;
    foc_external_input_raw_sample_t sample;
    foc_external_input_raw_health_t health;

    assert(foc_stm32g431_external_input_driver_init(
               &driver, FOC_EXTERNAL_INPUT_ANALOG, &registers, &bits) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_stm32g431_external_input_driver_bind_port(&driver, &port) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_port_start(
               &port, FOC_EXTERNAL_INPUT_ANALOG) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_port_read_latest(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &sample) ==
           FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE);

    /* First control tick only schedules the non-blocking regular conversion. */
    assert(foc_stm32g431_external_input_driver_control_tick(
               &driver, 100U) == FOC_EXTERNAL_INPUT_PORT_OK);
    assert((adc_cr & bits.start_mask) != 0U);
    assert((adc_sqr1 & bits.rank1_mask) == (8UL << 6));
    assert((adc_smpr1 & bits.sample_mask) == (4UL << 24));

    /* The next tick consumes the prior result and records its start timestamp. */
    adc_dr = 2048U;
    adc_isr = bits.eoc_mask;
    assert(foc_stm32g431_external_input_driver_control_tick(
               &driver, 183U) == FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_port_read_latest(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &sample) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(sample.sequence == 1U);
    assert(sample.sampled_at_us == 100U);
    assert(sample.raw_value == 2048);
    assert(sample.quality_flags == FOC_EXTERNAL_INPUT_RAW_QUALITY_FIRST);

    /* A late regular conversion is counted without polling or publishing a
     * fabricated value; recovery marks the next real sample as degraded. */
    adc_isr = 0U;
    assert(foc_stm32g431_external_input_driver_control_tick(
               &driver, 266U) == FOC_EXTERNAL_INPUT_PORT_BUSY);
    adc_dr = 3072U;
    adc_isr = bits.eoc_mask;
    assert(foc_stm32g431_external_input_driver_control_tick(
               &driver, 349U) == FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_external_input_port_read_latest(
               &port, FOC_EXTERNAL_INPUT_ANALOG, &sample) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(sample.sequence == 2U);
    assert(sample.sampled_at_us == 183U);
    assert(sample.raw_value == 3072);
    assert((sample.quality_flags &
            FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS) != 0U);
    assert(foc_external_input_port_get_health(&port, &health) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(health.sample_count == 2U);
    assert(health.missed_capture_count == 1U);
    assert(health.healthy_input_mask == FOC_EXTERNAL_INPUT_ANALOG);

    /* Ownership must not be transferred while ADC1 still has an active
     * regular conversion.  The management owner retries after ADSTART clears. */
    assert(foc_external_input_port_stop(
               &port, FOC_EXTERNAL_INPUT_ANALOG) ==
           FOC_EXTERNAL_INPUT_PORT_BUSY);
    assert(port.enabled_input_mask == FOC_EXTERNAL_INPUT_ANALOG);
    adc_cr &= ~bits.start_mask;
    assert(foc_external_input_port_stop(
               &port, FOC_EXTERNAL_INPUT_ANALOG) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(foc_stm32g431_external_input_driver_control_tick(
               &driver, 432U) == FOC_EXTERNAL_INPUT_PORT_DISABLED);
    assert(foc_external_input_port_get_health(&port, &health) ==
           FOC_EXTERNAL_INPUT_PORT_OK);
    assert(health.enabled_input_mask == 0U);
    assert(health.healthy_input_mask == 0U);
    return 0;
}
