#include "foc_as5600.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define FOC_AS5600_TWO_PI        (6.28318530717958647692F)
#define FOC_AS5600_HALF_RANGE    (0x80000000UL)
#define FOC_AS5600_HALF_COUNTS   (2048)

static bool foc_as5600_direction_valid(int32_t direction)
{
    return (direction == -1) || (direction == 1);
}

static float foc_as5600_wrap_two_pi(float value)
{
    float wrapped = fmodf(value, FOC_AS5600_TWO_PI);
    if (wrapped < 0.0F)
    {
        wrapped += FOC_AS5600_TWO_PI;
    }
    return wrapped;
}

static uint32_t foc_as5600_next_sequence(uint32_t current)
{
    uint32_t next = current + 1U;
    if (next == 0U)
    {
        next = 1U;
    }
    return next;
}

static bool foc_as5600_elapsed(
    uint32_t now_us,
    uint32_t then_us,
    uint32_t *elapsed_us)
{
    uint32_t elapsed;
    if (elapsed_us == NULL)
    {
        return false;
    }
    elapsed = now_us - then_us;
    if ((elapsed == 0U) || (elapsed >= FOC_AS5600_HALF_RANGE))
    {
        return false;
    }
    *elapsed_us = elapsed;
    return true;
}

bool foc_as5600_decode_raw_angle(uint8_t msb, uint8_t lsb, uint16_t *raw_count)
{
    if (raw_count == NULL)
    {
        return false;
    }
    *raw_count = (uint16_t)((((uint16_t)msb & 0x000FU) << 8U) |
                            (uint16_t)lsb);
    return true;
}

bool foc_as5600_tracker_init(
    foc_as5600_tracker_t *tracker,
    const foc_as5600_config_t *config)
{
    if ((tracker == NULL) || (config == NULL) ||
        !foc_as5600_direction_valid(config->direction) ||
        (config->pole_pairs == 0U) ||
        (config->pole_pair_revision == 0U) ||
        !isfinite(config->electrical_offset_rad) ||
        (config->calibrated > 1U) ||
        (config->maximum_sample_period_us == 0U) ||
        (config->maximum_sample_period_us >= FOC_AS5600_HALF_RANGE))
    {
        return false;
    }
    (void)memset(tracker, 0, sizeof(*tracker));
    tracker->config = *config;
    return true;
}

bool foc_as5600_tracker_update(
    foc_as5600_tracker_t *tracker,
    uint16_t raw_count,
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_absolute_encoder_feedback_port_t *output)
{
    float count_to_rad;
    float raw_angle_rad;
    float directed_angle_rad;
    uint32_t elapsed_us = 0U;
    int32_t delta_counts;

    if ((tracker == NULL) || (output == NULL) ||
        ((uint32_t)raw_count >= FOC_AS5600_COUNTS_PER_REVOLUTION))
    {
        return false;
    }
    (void)memset(output, 0, sizeof(*output));
    tracker->sequence = foc_as5600_next_sequence(tracker->sequence);
    output->common.axis_id = tracker->config.axis_id;
    output->common.sequence = tracker->sequence;
    output->common.sampled_at_ms = sampled_at_ms;
    output->common.sampled_at_us = sampled_at_us;
    output->common.direction_valid = 1U;
    output->common.direction = tracker->config.direction;
    output->common.pole_pair_revision = tracker->config.pole_pair_revision;
    output->calibrated = tracker->config.calibrated;

    count_to_rad = FOC_AS5600_TWO_PI /
                   (float)FOC_AS5600_COUNTS_PER_REVOLUTION;
    raw_angle_rad = (float)raw_count * count_to_rad;
    directed_angle_rad = foc_as5600_wrap_two_pi(
        (float)tracker->config.direction * raw_angle_rad);

    if (tracker->initialized == 0U)
    {
        tracker->initialized = 1U;
        tracker->previous_raw_count = raw_count;
        tracker->previous_sampled_at_us = sampled_at_us;
        tracker->multi_turn_position_rad = directed_angle_rad;
        return true;
    }

    delta_counts = (int32_t)raw_count - (int32_t)tracker->previous_raw_count;
    if (delta_counts > FOC_AS5600_HALF_COUNTS)
    {
        delta_counts -= (int32_t)FOC_AS5600_COUNTS_PER_REVOLUTION;
    }
    else if (delta_counts < -FOC_AS5600_HALF_COUNTS)
    {
        delta_counts += (int32_t)FOC_AS5600_COUNTS_PER_REVOLUTION;
    }
    delta_counts *= tracker->config.direction;

    if (!foc_as5600_elapsed(sampled_at_us,
                            tracker->previous_sampled_at_us,
                            &elapsed_us) ||
        (elapsed_us > tracker->config.maximum_sample_period_us))
    {
        tracker->previous_raw_count = raw_count;
        tracker->previous_sampled_at_us = sampled_at_us;
        tracker->multi_turn_position_rad = directed_angle_rad;
        return true;
    }

    tracker->multi_turn_position_rad += (float)delta_counts * count_to_rad;
    tracker->previous_raw_count = raw_count;
    tracker->previous_sampled_at_us = sampled_at_us;

    output->common.available = 1U;
    output->mechanical_position_rad = directed_angle_rad;
    output->multi_turn_position_rad = tracker->multi_turn_position_rad;
    output->mechanical_velocity_rad_s =
        ((float)delta_counts * count_to_rad) /
        ((float)elapsed_us * 1.0e-6F);
    output->electrical_angle_rad = foc_as5600_wrap_two_pi(
        (float)tracker->config.pole_pairs * directed_angle_rad -
        tracker->config.electrical_offset_rad);
    output->electrical_velocity_rad_s =
        output->mechanical_velocity_rad_s * (float)tracker->config.pole_pairs;
    return true;
}
