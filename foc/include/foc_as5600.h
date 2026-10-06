#ifndef FOC_AS5600_H
#define FOC_AS5600_H

/*
 * Hardware-neutral AS5600 sample normalizer.
 *
 * A platform driver owns I2C transactions and timestamps. This module only
 * decodes the 12-bit angle, unwraps consecutive samples, derives velocity and
 * produces FluxRT's absolute-encoder feedback port. It performs no bus access,
 * allocation, blocking wait, PWM write or motor actuation.
 */

#include <stdbool.h>
#include <stdint.h>

#include "foc_feedback_adapter.h"

#define FOC_AS5600_I2C_ADDRESS       (0x36U)
#define FOC_AS5600_RAW_ANGLE_REGISTER (0x0CU)
#define FOC_AS5600_COUNTS_PER_REVOLUTION (4096U)
#define FOC_AS5600_RAW_MASK          (0x0FFFU)

typedef struct
{
    uint32_t axis_id;
    int32_t direction;
    uint32_t pole_pairs;
    uint32_t pole_pair_revision;
    float electrical_offset_rad;
    uint32_t calibrated;
    uint32_t maximum_sample_period_us;
} foc_as5600_config_t;

typedef struct
{
    foc_as5600_config_t config;
    uint32_t initialized;
    uint32_t sequence;
    uint16_t previous_raw_count;
    uint16_t reserved;
    uint32_t previous_sampled_at_us;
    float multi_turn_position_rad;
} foc_as5600_tracker_t;

bool foc_as5600_decode_raw_angle(uint8_t msb, uint8_t lsb, uint16_t *raw_count);
bool foc_as5600_tracker_init(
    foc_as5600_tracker_t *tracker,
    const foc_as5600_config_t *config);
bool foc_as5600_tracker_update(
    foc_as5600_tracker_t *tracker,
    uint16_t raw_count,
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_absolute_encoder_feedback_port_t *output);

#endif /* FOC_AS5600_H */
