#ifndef FOC_FEEDBACK_ADAPTER_H
#define FOC_FEEDBACK_ADAPTER_H

/* Hardware-neutral C normalization helpers. No HAL or RTOS dependency. */

#include <stdbool.h>
#include <stdint.h>

#include "foc_feedback_bridge.h"

typedef struct
{
    uint32_t axis_id;
    uint32_t sequence;
    uint32_t sampled_at_ms;
    uint32_t sampled_at_us;
    uint32_t available;
    uint32_t direction_valid;
    int32_t direction;
    uint32_t pole_pair_revision;
} foc_feedback_adapter_common_t;

typedef struct
{
    foc_feedback_adapter_common_t common;
    uint32_t reliable;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float electrical_velocity_rad_s;
} foc_sensorless_feedback_port_t;

typedef struct
{
    foc_feedback_adapter_common_t common;
    uint32_t calibrated;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float electrical_velocity_rad_s;
} foc_hall_feedback_port_t;

typedef struct
{
    foc_feedback_adapter_common_t common;
    uint32_t calibrated;
    uint32_t index_found;
    float mechanical_position_rad;
    float multi_turn_position_rad;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float electrical_velocity_rad_s;
} foc_incremental_encoder_feedback_port_t;

bool foc_feedback_adapt_sensorless(
    const foc_sensorless_feedback_port_t *input,
    foc_feedback_source_sample_t *output);
bool foc_feedback_adapt_hall(
    const foc_hall_feedback_port_t *input,
    foc_feedback_source_sample_t *output);
bool foc_feedback_adapt_incremental_encoder(
    const foc_incremental_encoder_feedback_port_t *input,
    foc_feedback_source_sample_t *output);

#endif /* FOC_FEEDBACK_ADAPTER_H */
