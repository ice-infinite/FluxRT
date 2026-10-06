#ifndef FOC_DENGFOC_CURRENT_SENSE_H
#define FOC_DENGFOC_CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_board_dengfoc_v04.h"

#define FOC_DENGFOC_CURRENT_CHANNEL_COUNT (2U)

typedef bool (*foc_dengfoc_adc_read_raw_fn)(
    void *context,
    uint32_t gpio,
    uint32_t *raw_count);

typedef struct
{
    void *context;
    foc_dengfoc_adc_read_raw_fn read_raw;
} foc_dengfoc_adc_ops_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t minimum_raw_count;
    uint32_t maximum_raw_count;
    float mean_raw_count;
    float standard_deviation_raw_count;
    float offset_voltage_v;
    float amperes_per_count;
    float noise_rms_a;
} foc_dengfoc_current_zero_stat_t;

bool foc_dengfoc_current_capture_zero(
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    const foc_dengfoc_adc_ops_t *ops,
    uint32_t sample_count,
    foc_dengfoc_current_zero_stat_t
        output[FOC_DENGFOC_CURRENT_CHANNEL_COUNT]);

#endif /* FOC_DENGFOC_CURRENT_SENSE_H */
