#include "foc_feedback_adapter.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static foc_feedback_adapter_common_t common_input(void)
{
    foc_feedback_adapter_common_t value;
    (void)memset(&value, 0, sizeof(value));
    value.axis_id = 0U;
    value.sequence = 1U;
    value.sampled_at_ms = 42U;
    value.sampled_at_us = 42000U;
    value.available = 1U;
    value.direction_valid = 1U;
    value.direction = 1;
    value.pole_pair_revision = 7U;
    return value;
}

static void sensorless_maps_reliable_and_unavailable_states(void)
{
    foc_sensorless_feedback_port_t input;
    foc_feedback_source_sample_t output;
    (void)memset(&input, 0, sizeof(input));
    input.common = common_input();
    input.reliable = 1U;
    input.mechanical_velocity_rad_s = 10.0F;
    input.electrical_angle_rad = 1.0F;
    input.electrical_velocity_rad_s = 70.0F;
    assert(foc_feedback_adapt_sensorless(&input, &output));
    assert(output.struct_size == sizeof(output));
    assert(output.abi_version == FOC_FEEDBACK_ABI_VERSION);
    assert(output.mode == FOC_FEEDBACK_MODE_SENSORLESS);
    assert(output.valid_flags ==
           (FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY |
            FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE |
            FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY));
    assert(output.quality_flags == FOC_PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID);

    input.reliable = 0U;
    assert(foc_feedback_adapt_sensorless(&input, &output));
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED) != 0U);

    input.common.available = 0U;
    assert(foc_feedback_adapt_sensorless(&input, &output));
    assert(output.valid_flags == 0U);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_STALE) != 0U);
    assert(output.mechanical_velocity_rad_s == 0.0F);
}

static void encoder_requires_calibration_and_index_quality(void)
{
    foc_incremental_encoder_feedback_port_t input;
    foc_feedback_source_sample_t output;
    (void)memset(&input, 0, sizeof(input));
    input.common = common_input();
    input.calibrated = 1U;
    input.index_found = 1U;
    input.mechanical_position_rad = 0.25F;
    input.multi_turn_position_rad = 12.5F;
    input.mechanical_velocity_rad_s = 10.0F;
    input.electrical_angle_rad = 1.75F;
    input.electrical_velocity_rad_s = 70.0F;
    assert(foc_feedback_adapt_incremental_encoder(&input, &output));
    assert(output.mode == FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER);
    assert(output.valid_flags == FOC_PRODUCT_FEEDBACK_VALID_KNOWN_MASK);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED) != 0U);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND) != 0U);

    input.index_found = 0U;
    assert(foc_feedback_adapt_incremental_encoder(&input, &output));
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND) == 0U);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED) != 0U);
}

static void hall_and_invalid_inputs_are_canonical(void)
{
    foc_hall_feedback_port_t input;
    foc_feedback_source_sample_t output;
    (void)memset(&input, 0, sizeof(input));
    input.common = common_input();
    input.calibrated = 0U;
    input.mechanical_velocity_rad_s = 5.0F;
    input.electrical_angle_rad = 2.0F;
    input.electrical_velocity_rad_s = 35.0F;
    assert(foc_feedback_adapt_hall(&input, &output));
    assert(output.mode == FOC_FEEDBACK_MODE_HALL);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED) == 0U);
    assert((output.quality_flags & FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED) != 0U);

    input.common.direction_valid = 0U;
    input.common.direction = 1;
    (void)memset(&output, 0xA5, sizeof(output));
    assert(!foc_feedback_adapt_hall(&input, &output));
    assert(output.struct_size == 0U);

    input.common.direction_valid = 1U;
    input.common.direction = 1;
    input.electrical_angle_rad = NAN;
    assert(!foc_feedback_adapt_hall(&input, &output));
    assert(output.struct_size == 0U);
    assert(!foc_feedback_adapt_hall(NULL, &output));
    assert(!foc_feedback_adapt_hall(&input, NULL));
}

int main(void)
{
    sensorless_maps_reliable_and_unavailable_states();
    encoder_requires_calibration_and_index_quality();
    hall_and_invalid_inputs_are_canonical();
    return 0;
}
