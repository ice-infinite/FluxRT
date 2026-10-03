#include "foc_feedback_adapter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool foc_feedback_adapter_common_valid(
    const foc_feedback_adapter_common_t *common)
{
    if ((common == NULL) || (common->sequence == 0U) ||
        (common->pole_pair_revision == 0U) || (common->available > 1U) ||
        (common->direction_valid > 1U))
    {
        return false;
    }
    if (common->direction_valid != 0U)
    {
        return (common->direction == -1) || (common->direction == 1);
    }
    return common->direction == 0;
}

static bool foc_feedback_adapter_begin(
    const foc_feedback_adapter_common_t *common,
    foc_feedback_mode_t mode,
    foc_feedback_source_sample_t *output)
{
    if (output == NULL)
    {
        return false;
    }
    (void)memset(output, 0, sizeof(*output));
    if (!foc_feedback_adapter_common_valid(common))
    {
        return false;
    }
    output->struct_size = (uint32_t)sizeof(*output);
    output->abi_version = FOC_FEEDBACK_ABI_VERSION;
    output->axis_id = common->axis_id;
    output->mode = mode;
    output->sequence = common->sequence;
    output->sampled_at_ms = common->sampled_at_ms;
    output->sampled_at_us = common->sampled_at_us;
    output->pole_pair_revision = common->pole_pair_revision;
    if (common->direction_valid != 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID;
        output->direction = common->direction;
    }
    if (common->available == 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_STALE;
    }
    return true;
}

static bool foc_feedback_three_values_finite(
    float first,
    float second,
    float third)
{
    return isfinite(first) && isfinite(second) && isfinite(third);
}

bool foc_feedback_adapt_sensorless(
    const foc_sensorless_feedback_port_t *input,
    foc_feedback_source_sample_t *output)
{
    if ((input == NULL) || (input->reliable > 1U) ||
        !foc_feedback_adapter_begin(&input->common, FOC_FEEDBACK_MODE_SENSORLESS, output))
    {
        return false;
    }
    if (input->common.available == 0U)
    {
        return true;
    }
    if (!foc_feedback_three_values_finite(input->mechanical_velocity_rad_s,
                                          input->electrical_angle_rad,
                                          input->electrical_velocity_rad_s))
    {
        (void)memset(output, 0, sizeof(*output));
        return false;
    }
    output->valid_flags = FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY |
                          FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE |
                          FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY;
    output->mechanical_velocity_rad_s = input->mechanical_velocity_rad_s;
    output->electrical_angle_rad = input->electrical_angle_rad;
    output->electrical_velocity_rad_s = input->electrical_velocity_rad_s;
    if (input->reliable == 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED;
    }
    return true;
}

bool foc_feedback_adapt_hall(
    const foc_hall_feedback_port_t *input,
    foc_feedback_source_sample_t *output)
{
    if ((input == NULL) || (input->calibrated > 1U) ||
        !foc_feedback_adapter_begin(&input->common, FOC_FEEDBACK_MODE_HALL, output))
    {
        return false;
    }
    if (input->common.available == 0U)
    {
        return true;
    }
    if (!foc_feedback_three_values_finite(input->mechanical_velocity_rad_s,
                                          input->electrical_angle_rad,
                                          input->electrical_velocity_rad_s))
    {
        (void)memset(output, 0, sizeof(*output));
        return false;
    }
    output->valid_flags = FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY |
                          FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE |
                          FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY;
    output->mechanical_velocity_rad_s = input->mechanical_velocity_rad_s;
    output->electrical_angle_rad = input->electrical_angle_rad;
    output->electrical_velocity_rad_s = input->electrical_velocity_rad_s;
    if (input->calibrated != 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED;
    }
    else
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED;
    }
    return true;
}

bool foc_feedback_adapt_incremental_encoder(
    const foc_incremental_encoder_feedback_port_t *input,
    foc_feedback_source_sample_t *output)
{
    if ((input == NULL) || (input->calibrated > 1U) ||
        (input->index_found > 1U) ||
        !foc_feedback_adapter_begin(&input->common,
                                    FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER,
                                    output))
    {
        return false;
    }
    if (input->common.available == 0U)
    {
        return true;
    }
    if (!isfinite(input->mechanical_position_rad) ||
        !isfinite(input->multi_turn_position_rad) ||
        !foc_feedback_three_values_finite(input->mechanical_velocity_rad_s,
                                          input->electrical_angle_rad,
                                          input->electrical_velocity_rad_s))
    {
        (void)memset(output, 0, sizeof(*output));
        return false;
    }
    output->valid_flags = FOC_PRODUCT_FEEDBACK_VALID_KNOWN_MASK;
    output->mechanical_position_rad = input->mechanical_position_rad;
    output->multi_turn_position_rad = input->multi_turn_position_rad;
    output->mechanical_velocity_rad_s = input->mechanical_velocity_rad_s;
    output->electrical_angle_rad = input->electrical_angle_rad;
    output->electrical_velocity_rad_s = input->electrical_velocity_rad_s;
    if (input->calibrated != 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED;
    }
    else
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED;
    }
    if (input->index_found != 0U)
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND;
    }
    else
    {
        output->quality_flags |= FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED;
    }
    return true;
}
