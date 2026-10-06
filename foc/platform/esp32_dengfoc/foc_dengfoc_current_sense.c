#include "foc_dengfoc_current_sense.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    uint32_t minimum;
    uint32_t maximum;
    float mean;
    float m2;
} running_stat_t;

static void clear_output(
    foc_dengfoc_current_zero_stat_t
        output[FOC_DENGFOC_CURRENT_CHANNEL_COUNT])
{
    if (output != NULL)
    {
        (void)memset(output, 0,
                     sizeof(*output) * FOC_DENGFOC_CURRENT_CHANNEL_COUNT);
    }
}

static void update_stat(running_stat_t *stat, uint32_t raw, uint32_t count)
{
    const float raw_value = (float)raw;
    const float delta = raw_value - stat->mean;
    if (raw < stat->minimum)
    {
        stat->minimum = raw;
    }
    if (raw > stat->maximum)
    {
        stat->maximum = raw;
    }
    stat->mean += delta / (float)count;
    stat->m2 += delta * (raw_value - stat->mean);
}

bool foc_dengfoc_current_capture_zero(
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    const foc_dengfoc_adc_ops_t *ops,
    uint32_t sample_count,
    foc_dengfoc_current_zero_stat_t
        output[FOC_DENGFOC_CURRENT_CHANNEL_COUNT])
{
    uint32_t sample;
    uint32_t channel;
    running_stat_t stat[FOC_DENGFOC_CURRENT_CHANNEL_COUNT];
    uint32_t gpio[FOC_DENGFOC_CURRENT_CHANNEL_COUNT];
    float volts_per_count;
    float amperes_per_count;

    clear_output(output);
    if ((profile == NULL) ||
        !foc_dengfoc_v04_profile_valid(profile) ||
        (axis >= FOC_DENGFOC_V04_AXIS_COUNT) ||
        (ops == NULL) ||
        (ops->read_raw == NULL) ||
        (sample_count == 0U) ||
        (output == NULL))
    {
        return false;
    }

    gpio[0] = profile->axis[axis].current_a_adc_gpio;
    gpio[1] = profile->axis[axis].current_b_adc_gpio;
    volts_per_count =
        profile->adc_reference_v / (float)profile->adc_full_scale_count;
    amperes_per_count =
        volts_per_count /
        (profile->shunt_resistance_ohm * profile->current_amplifier_gain);
    amperes_per_count *= (float)profile->current_polarity;

    for (channel = 0U; channel < FOC_DENGFOC_CURRENT_CHANNEL_COUNT; ++channel)
    {
        stat[channel].minimum = profile->adc_full_scale_count;
        stat[channel].maximum = 0U;
        stat[channel].mean = 0.0F;
        stat[channel].m2 = 0.0F;
    }

    for (sample = 0U; sample < sample_count; ++sample)
    {
        for (channel = 0U; channel < FOC_DENGFOC_CURRENT_CHANNEL_COUNT;
             ++channel)
        {
            uint32_t raw = 0U;
            if (!ops->read_raw(ops->context, gpio[channel], &raw) ||
                (raw > profile->adc_full_scale_count))
            {
                clear_output(output);
                return false;
            }
            update_stat(&stat[channel], raw, sample + 1U);
        }
    }

    for (channel = 0U; channel < FOC_DENGFOC_CURRENT_CHANNEL_COUNT; ++channel)
    {
        const float standard_deviation =
            sqrtf(stat[channel].m2 / (float)sample_count);
        output[channel].sample_count = sample_count;
        output[channel].minimum_raw_count = stat[channel].minimum;
        output[channel].maximum_raw_count = stat[channel].maximum;
        output[channel].mean_raw_count = stat[channel].mean;
        output[channel].standard_deviation_raw_count = standard_deviation;
        output[channel].offset_voltage_v = stat[channel].mean * volts_per_count;
        output[channel].amperes_per_count = amperes_per_count;
        output[channel].noise_rms_a =
            standard_deviation * fabsf(amperes_per_count);
    }
    return true;
}
