#include "foc_external_input_stm32g431.h"
#include "foc_external_input_platform.h"
#include "foc_build_profile.h"

#include <stdatomic.h>
#include <string.h>

#if defined(FOC_TARGET_STM32G431)
#include "stm32g4xx_hal.h"
#endif

#define FOC_STM32G431_ANALOG_INPUT_MASK FOC_EXTERNAL_INPUT_ANALOG
#define FOC_STM32G431_ANALOG_INSTANCE_ID (0UL)
#define FOC_STM32G431_CONTROL_FREQUENCY_HZ (12000UL)
#define FOC_STM32G431_US_PER_SECOND (1000000UL)

static foc_stm32g431_external_input_driver_t g_external_input_driver;
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_INPUT_ANALOG)
static uint32_t g_external_input_timestamp_us;
static uint32_t g_external_input_timestamp_remainder;
#endif

static uint32_t foc_stm32g431_external_input_driver_is_valid(
    const foc_stm32g431_external_input_driver_t *driver)
{
    return ((driver != 0) &&
            (driver->struct_size == sizeof(*driver)) &&
            (driver->version ==
             FOC_STM32G431_EXTERNAL_INPUT_DRIVER_VERSION) &&
            (driver->initialized != 0U) &&
            ((driver->supported_input_mask &
              ~(uint32_t)FOC_STM32G431_ANALOG_INPUT_MASK) == 0U) &&
            ((driver->enabled_input_mask &
              ~driver->supported_input_mask) == 0U)) ? 1U : 0U;
}

static uint32_t foc_stm32g431_external_input_enter_critical(void)
{
#if defined(FOC_TARGET_STM32G431)
    uint32_t was_enabled =
        (NVIC_GetEnableIRQ(ADC1_2_IRQn) != 0U) ? 1U : 0U;
    NVIC_DisableIRQ(ADC1_2_IRQn);
    __DSB();
    __ISB();
    return was_enabled;
#else
    return 0U;
#endif
}

static void foc_stm32g431_external_input_exit_critical(
    uint32_t was_enabled)
{
#if defined(FOC_TARGET_STM32G431)
    if (was_enabled != 0U)
    {
        NVIC_EnableIRQ(ADC1_2_IRQn);
    }
#else
    (void)was_enabled;
#endif
}

static void foc_stm32g431_external_input_publish_begin(
    foc_stm32g431_external_input_driver_t *driver)
{
    ++driver->publish_guard;
    atomic_signal_fence(memory_order_seq_cst);
}

static void foc_stm32g431_external_input_publish_end(
    foc_stm32g431_external_input_driver_t *driver)
{
    atomic_signal_fence(memory_order_seq_cst);
    ++driver->publish_guard;
}

static foc_external_input_port_result_t
foc_stm32g431_external_input_start(
    void *context,
    foc_external_input_mask_t input_mask)
{
    foc_stm32g431_external_input_driver_t *driver =
        (foc_stm32g431_external_input_driver_t *)context;
    uint32_t was_enabled;
    uint32_t discard;

    if ((foc_stm32g431_external_input_driver_is_valid(driver) == 0U) ||
        (input_mask == 0U) ||
        ((input_mask &
          ~(uint32_t)FOC_STM32G431_ANALOG_INPUT_MASK) != 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((input_mask & ~driver->supported_input_mask) != 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE;
    }
    was_enabled = foc_stm32g431_external_input_enter_critical();
    if ((*driver->adc.cr & driver->bits.start_mask) != 0U)
    {
        foc_stm32g431_external_input_exit_critical(was_enabled);
        return FOC_EXTERNAL_INPUT_PORT_BUSY;
    }
    /* Drain a result left by the stopped-state monitor before scheduling the
     * first PC2 conversion; otherwise the first published sample could belong
     * to Vbus/temperature instead of the requested analog input. */
    if ((*driver->adc.isr & driver->bits.eoc_mask) != 0U)
    {
        discard = *driver->adc.dr;
        (void)discard;
    }
    foc_stm32g431_external_input_publish_begin(driver);
    driver->enabled_input_mask |= input_mask;
    driver->conversion_pending = 0U;
    driver->missed_since_sample = 0U;
    driver->health.enabled_input_mask = driver->enabled_input_mask;
    driver->health.healthy_input_mask &= ~input_mask;
    driver->health.fault_input_mask &= ~input_mask;
    driver->health.last_error = FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE;
    foc_stm32g431_external_input_publish_end(driver);
    foc_stm32g431_external_input_exit_critical(was_enabled);
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t
foc_stm32g431_external_input_stop(
    void *context,
    foc_external_input_mask_t input_mask)
{
    foc_stm32g431_external_input_driver_t *driver =
        (foc_stm32g431_external_input_driver_t *)context;
    uint32_t was_enabled;

    if ((foc_stm32g431_external_input_driver_is_valid(driver) == 0U) ||
        (input_mask == 0U) ||
        ((input_mask &
          ~(uint32_t)FOC_STM32G431_ANALOG_INPUT_MASK) != 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((input_mask & ~driver->supported_input_mask) != 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE;
    }
    was_enabled = foc_stm32g431_external_input_enter_critical();
    /* Do not hand ADC1 regular ownership back while hardware still converts.
     * The management transaction can retry on its next bounded poll. */
    if ((*driver->adc.cr & driver->bits.start_mask) != 0U)
    {
        foc_stm32g431_external_input_exit_critical(was_enabled);
        return FOC_EXTERNAL_INPUT_PORT_BUSY;
    }
    if ((*driver->adc.isr & driver->bits.eoc_mask) != 0U)
    {
        uint32_t discard = *driver->adc.dr;
        (void)discard;
    }
    foc_stm32g431_external_input_publish_begin(driver);
    driver->enabled_input_mask &= ~input_mask;
    driver->conversion_pending = 0U;
    driver->missed_since_sample = 0U;
    driver->health.enabled_input_mask = driver->enabled_input_mask;
    driver->health.healthy_input_mask &= ~input_mask;
    driver->health.fault_input_mask &= ~input_mask;
    driver->health.last_error = FOC_EXTERNAL_INPUT_PORT_DISABLED;
    foc_stm32g431_external_input_publish_end(driver);
    foc_stm32g431_external_input_exit_critical(was_enabled);
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

static foc_external_input_port_result_t
foc_stm32g431_external_input_read_latest(
    void *context,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample)
{
    foc_stm32g431_external_input_driver_t *driver =
        (foc_stm32g431_external_input_driver_t *)context;
    uint32_t attempt;

    if ((foc_stm32g431_external_input_driver_is_valid(driver) == 0U) ||
        (sample == 0) || (input != FOC_EXTERNAL_INPUT_ANALOG))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((driver->enabled_input_mask & input) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_DISABLED;
    }
    for (attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t before = driver->publish_guard;
        uint32_t after;

        if ((before & 1U) != 0U)
        {
            continue;
        }
        atomic_signal_fence(memory_order_seq_cst);
        *sample = driver->latest;
        atomic_signal_fence(memory_order_seq_cst);
        after = driver->publish_guard;
        if (before == after)
        {
            return (sample->sequence != 0U) ?
                FOC_EXTERNAL_INPUT_PORT_OK :
                FOC_EXTERNAL_INPUT_PORT_NO_SAMPLE;
        }
    }
    return FOC_EXTERNAL_INPUT_PORT_BUSY;
}

static foc_external_input_port_result_t
foc_stm32g431_external_input_get_health(
    void *context,
    foc_external_input_raw_health_t *health)
{
    foc_stm32g431_external_input_driver_t *driver =
        (foc_stm32g431_external_input_driver_t *)context;
    uint32_t attempt;

    if ((foc_stm32g431_external_input_driver_is_valid(driver) == 0U) ||
        (health == 0))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    for (attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t before = driver->publish_guard;
        uint32_t after;

        if ((before & 1U) != 0U)
        {
            continue;
        }
        atomic_signal_fence(memory_order_seq_cst);
        *health = driver->health;
        atomic_signal_fence(memory_order_seq_cst);
        after = driver->publish_guard;
        if (before == after)
        {
            return FOC_EXTERNAL_INPUT_PORT_OK;
        }
    }
    return FOC_EXTERNAL_INPUT_PORT_BUSY;
}

static const foc_external_input_port_ops_t g_external_input_ops =
{
    sizeof(foc_external_input_port_ops_t),
    FOC_EXTERNAL_INPUT_PORT_OPS_VERSION,
    foc_stm32g431_external_input_start,
    foc_stm32g431_external_input_stop,
    foc_stm32g431_external_input_read_latest,
    foc_stm32g431_external_input_get_health,
};

foc_external_input_port_result_t
foc_stm32g431_external_input_driver_init(
    foc_stm32g431_external_input_driver_t *driver,
    foc_external_input_mask_t supported_input_mask,
    const foc_stm32g431_external_input_adc_registers_t *adc,
    const foc_stm32g431_external_input_adc_bits_t *bits)
{
    if (driver == 0)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    (void)memset(driver, 0, sizeof(*driver));
    if (((supported_input_mask &
          ~(uint32_t)FOC_STM32G431_ANALOG_INPUT_MASK) != 0U) ||
        ((supported_input_mask != 0U) &&
         ((adc == 0) || (bits == 0) ||
          (adc->isr == 0) || (adc->cr == 0) ||
          (adc->sqr1 == 0) || (adc->smpr1 == 0) || (adc->dr == 0) ||
          (bits->eoc_mask == 0U) || (bits->start_mask == 0U) ||
          (bits->rank1_mask == 0U) || (bits->sample_mask == 0U) ||
          (bits->channel_index > 31U) || (bits->sample_code > 7U))))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    driver->struct_size = sizeof(*driver);
    driver->version = FOC_STM32G431_EXTERNAL_INPUT_DRIVER_VERSION;
    driver->supported_input_mask = supported_input_mask;
    if (supported_input_mask != 0U)
    {
        driver->adc = *adc;
        driver->bits = *bits;
    }
    driver->latest.struct_size = sizeof(driver->latest);
    driver->latest.version = FOC_EXTERNAL_INPUT_RAW_SAMPLE_VERSION;
    driver->latest.input = FOC_EXTERNAL_INPUT_ANALOG;
    driver->latest.instance_id = FOC_STM32G431_ANALOG_INSTANCE_ID;
    driver->health.struct_size = sizeof(driver->health);
    driver->health.version = FOC_EXTERNAL_INPUT_RAW_HEALTH_VERSION;
    driver->health.supported_input_mask = supported_input_mask;
    driver->health.last_error = FOC_EXTERNAL_INPUT_PORT_DISABLED;
    /* Publish initialized last: the ADC ISR can already be running while the
     * application binds this optional port after platform init. */
    atomic_signal_fence(memory_order_seq_cst);
    driver->initialized = 1U;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

foc_external_input_port_result_t
foc_stm32g431_external_input_driver_bind_port(
    foc_stm32g431_external_input_driver_t *driver,
    foc_external_input_port_t *port)
{
    if (foc_stm32g431_external_input_driver_is_valid(driver) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    return foc_external_input_port_bind(
        port, driver->supported_input_mask, driver, &g_external_input_ops);
}

foc_external_input_port_result_t
foc_stm32g431_external_input_driver_control_tick(
    foc_stm32g431_external_input_driver_t *driver,
    uint32_t sampled_at_us)
{
    uint32_t had_miss;

    if (foc_stm32g431_external_input_driver_is_valid(driver) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((driver->enabled_input_mask & FOC_EXTERNAL_INPUT_ANALOG) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_DISABLED;
    }

    if (driver->conversion_pending != 0U)
    {
        if ((*driver->adc.isr & driver->bits.eoc_mask) == 0U)
        {
            foc_stm32g431_external_input_publish_begin(driver);
            ++driver->health.missed_capture_count;
            ++driver->missed_since_sample;
            driver->health.last_error = FOC_EXTERNAL_INPUT_PORT_BUSY;
            foc_stm32g431_external_input_publish_end(driver);
            return FOC_EXTERNAL_INPUT_PORT_BUSY;
        }

        had_miss = driver->missed_since_sample;
        foc_stm32g431_external_input_publish_begin(driver);
        ++driver->latest.sequence;
        driver->latest.sampled_at_us = driver->pending_timestamp_us;
        driver->latest.valid_flags =
            FOC_EXTERNAL_INPUT_RAW_VALID_VALUE |
            FOC_EXTERNAL_INPUT_RAW_VALID_TIMESTAMP;
        driver->latest.quality_flags =
            (driver->latest.sequence == 1U) ?
                FOC_EXTERNAL_INPUT_RAW_QUALITY_FIRST : 0U;
        if (had_miss != 0U)
        {
            driver->latest.quality_flags |=
                FOC_EXTERNAL_INPUT_RAW_QUALITY_MISSED_PREVIOUS;
        }
        driver->latest.raw_value =
            (int32_t)(*driver->adc.dr & 0x0000FFFFUL);
        driver->latest.auxiliary_value = 0U;
        driver->health.healthy_input_mask |= FOC_EXTERNAL_INPUT_ANALOG;
        driver->health.fault_input_mask &=
            ~(uint32_t)FOC_EXTERNAL_INPUT_ANALOG;
        ++driver->health.sample_count;
        driver->health.last_error = FOC_EXTERNAL_INPUT_PORT_OK;
        driver->health.last_sampled_at_us = driver->pending_timestamp_us;
        driver->missed_since_sample = 0U;
        foc_stm32g431_external_input_publish_end(driver);
    }

    *driver->adc.sqr1 =
        (*driver->adc.sqr1 & ~driver->bits.rank1_mask) |
        ((driver->bits.channel_index << driver->bits.rank1_shift) &
         driver->bits.rank1_mask);
    *driver->adc.smpr1 =
        (*driver->adc.smpr1 & ~driver->bits.sample_mask) |
        ((driver->bits.sample_code << driver->bits.sample_shift) &
         driver->bits.sample_mask);
    driver->pending_timestamp_us = sampled_at_us;
    driver->conversion_pending = 1U;
    *driver->adc.cr |= driver->bits.start_mask;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

foc_external_input_port_result_t foc_external_input_platform_bind(
    foc_external_input_port_t *port)
{
    foc_external_input_port_result_t result;
    foc_external_input_mask_t supported_input_mask = 0U;

#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_INPUT_ANALOG)
    const foc_stm32g431_external_input_adc_registers_t adc =
    {
        &ADC1->ISR,
        &ADC1->CR,
        &ADC1->SQR1,
        &ADC1->SMPR1,
        &ADC1->DR,
    };
    const foc_stm32g431_external_input_adc_bits_t bits =
    {
        ADC_ISR_EOC,
        ADC_CR_ADSTART,
        ADC_SQR1_SQ1_Msk,
        ADC_SQR1_SQ1_Pos,
        8U,
        ADC_SMPR1_SMP8_Msk,
        ADC_SMPR1_SMP8_Pos,
        ADC_SAMPLETIME_47CYCLES_5,
    };
    supported_input_mask = FOC_EXTERNAL_INPUT_ANALOG;
    result = foc_stm32g431_external_input_driver_init(
        &g_external_input_driver, supported_input_mask, &adc, &bits);
#else
    result = foc_stm32g431_external_input_driver_init(
        &g_external_input_driver, supported_input_mask, 0, 0);
#endif
    if (result != FOC_EXTERNAL_INPUT_PORT_OK)
    {
        return result;
    }
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_INPUT_ANALOG)
    g_external_input_timestamp_us = 0U;
    g_external_input_timestamp_remainder = 0U;
#endif
    return foc_stm32g431_external_input_driver_bind_port(
        &g_external_input_driver, port);
}

void foc_external_input_platform_control_tick(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_INPUT_ANALOG)
    if ((g_external_input_driver.initialized != 0U) &&
        ((g_external_input_driver.enabled_input_mask &
          FOC_EXTERNAL_INPUT_ANALOG) != 0U))
    {
        (void)foc_stm32g431_external_input_driver_control_tick(
            &g_external_input_driver, g_external_input_timestamp_us);
        g_external_input_timestamp_us +=
            FOC_STM32G431_US_PER_SECOND /
            FOC_STM32G431_CONTROL_FREQUENCY_HZ;
        g_external_input_timestamp_remainder +=
            FOC_STM32G431_US_PER_SECOND %
            FOC_STM32G431_CONTROL_FREQUENCY_HZ;
        if (g_external_input_timestamp_remainder >=
            FOC_STM32G431_CONTROL_FREQUENCY_HZ)
        {
            ++g_external_input_timestamp_us;
            g_external_input_timestamp_remainder -=
                FOC_STM32G431_CONTROL_FREQUENCY_HZ;
        }
    }
#endif
}

uint32_t foc_external_input_platform_regular_adc_owned(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_INPUT_ANALOG)
    return ((g_external_input_driver.initialized != 0U) &&
            ((g_external_input_driver.enabled_input_mask &
              FOC_EXTERNAL_INPUT_ANALOG) != 0U)) ? 1U : 0U;
#else
    return 0U;
#endif
}
