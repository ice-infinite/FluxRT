#include "foc_phase_voltage_quality.h"

#include <limits.h>
#include <string.h>

#define FOC_PHASE_VOLTAGE_SAMPLE_KNOWN_FLAGS                            \
    (FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED |                              \
     FOC_PHASE_VOLTAGE_SAMPLE_CALIBRATED |                              \
     FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID |                        \
     FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID)

static void foc_phase_voltage_increment(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

static uint32_t foc_phase_voltage_abs_diff(uint32_t left, uint32_t right)
{
    return (left >= right) ? (left - right) : (right - left);
}

static uint32_t foc_phase_voltage_request_is_valid(uint32_t request)
{
    return (request <= FOC_PHASE_VOLTAGE_REQUEST_HYBRID) ? 1U : 0U;
}

static void foc_phase_voltage_clear_detectors(
    foc_phase_voltage_quality_t *quality)
{
    quality->previous_valid = 0U;
    quality->low_saturation_phase_mask = 0U;
    quality->high_saturation_phase_mask = 0U;
    quality->open_suspect_phase_mask = 0U;
    quality->open_streak[0] = 0U;
    quality->open_streak[1] = 0U;
    quality->open_streak[2] = 0U;
    quality->inconsistency_streak = 0U;
    quality->inconsistency_latched = 0U;
}

static void foc_phase_voltage_update_saturation(
    foc_phase_voltage_quality_t *quality,
    const foc_phase_voltage_quality_config_t *config,
    const uint32_t raw[3])
{
    uint32_t index;

    for (index = 0U; index < 3U; ++index)
    {
        uint32_t phase_bit = 1UL << index;
        if (raw[index] <= config->low_saturation_enter_code)
        {
            quality->low_saturation_phase_mask |= phase_bit;
        }
        else if (raw[index] >= config->low_saturation_release_code)
        {
            quality->low_saturation_phase_mask &= ~phase_bit;
        }

        if (raw[index] >= config->high_saturation_enter_code)
        {
            quality->high_saturation_phase_mask |= phase_bit;
        }
        else if (raw[index] <= config->high_saturation_release_code)
        {
            quality->high_saturation_phase_mask &= ~phase_bit;
        }
    }
}

static void foc_phase_voltage_update_open_detector(
    foc_phase_voltage_quality_t *quality,
    const foc_phase_voltage_quality_config_t *config,
    const foc_phase_voltage_quality_sample_t *sample,
    const uint32_t raw[3])
{
    uint32_t index;

    if (quality->previous_valid == 0U)
    {
        return;
    }
    for (index = 0U; index < 3U; ++index)
    {
        uint32_t phase_bit = 1UL << index;
        uint32_t delta;

        if ((sample->expected_change_mask & phase_bit) == 0U)
        {
            continue;
        }
        delta = foc_phase_voltage_abs_diff(raw[index],
                                           quality->previous_raw[index]);
        if (delta <= config->open_stuck_enter_delta_codes)
        {
            foc_phase_voltage_increment(&quality->open_streak[index]);
            if (quality->open_streak[index] >= config->open_confirm_samples)
            {
                quality->open_suspect_phase_mask |= phase_bit;
            }
        }
        else if (delta >= config->open_release_delta_codes)
        {
            quality->open_streak[index] = 0U;
            quality->open_suspect_phase_mask &= ~phase_bit;
        }
    }
}

static void foc_phase_voltage_update_inconsistency(
    foc_phase_voltage_quality_t *quality,
    const foc_phase_voltage_quality_config_t *config,
    const foc_phase_voltage_quality_sample_t *sample)
{
    int64_t measured_line;
    int64_t command_line;
    uint64_t maximum_residual = 0ULL;
    uint64_t residual;
    const uint32_t measured[3] = {sample->phase_u_mv, sample->phase_v_mv,
                                  sample->phase_w_mv};
    const uint32_t command[3] = {sample->command_u_mv, sample->command_v_mv,
                                 sample->command_w_mv};
    const uint32_t left[3] = {0U, 1U, 2U};
    const uint32_t right[3] = {1U, 2U, 0U};
    uint32_t index;

    /* Compare U-V, V-W and W-U.  A common-mode offset therefore cancels and
     * cannot masquerade as a three-phase inconsistency. */
    for (index = 0U; index < 3U; ++index)
    {
        measured_line = (int64_t)measured[left[index]] -
                        (int64_t)measured[right[index]];
        command_line = (int64_t)command[left[index]] -
                       (int64_t)command[right[index]];
        residual = (measured_line >= command_line) ?
            (uint64_t)(measured_line - command_line) :
            (uint64_t)(command_line - measured_line);
        if (residual > maximum_residual)
        {
            maximum_residual = residual;
        }
    }

    if (maximum_residual >= config->inconsistency_enter_mv)
    {
        foc_phase_voltage_increment(&quality->inconsistency_streak);
        if (quality->inconsistency_streak >=
            config->inconsistency_confirm_samples)
        {
            quality->inconsistency_latched = 1U;
        }
    }
    else if (maximum_residual <= config->inconsistency_release_mv)
    {
        quality->inconsistency_streak = 0U;
        quality->inconsistency_latched = 0U;
    }
}

static foc_phase_voltage_quality_state_t foc_phase_voltage_primary_state(
    uint32_t reason_mask)
{
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_STALE) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_STALE;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_LOW_SATURATION;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION;
    }
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_OPEN_SUSPECT) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT;
    }
    if ((reason_mask &
         FOC_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT) != 0U)
    {
        return FOC_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT;
    }
    return FOC_PHASE_VOLTAGE_QUALITY_VALID;
}

static void foc_phase_voltage_count_reasons(
    foc_phase_voltage_quality_t *quality,
    uint32_t reason_mask)
{
    uint32_t index;

    for (index = 0U; index < FOC_PHASE_VOLTAGE_QUALITY_REASON_COUNT; ++index)
    {
        if ((reason_mask & (1UL << index)) != 0U)
        {
            foc_phase_voltage_increment(&quality->reason_counts[index]);
        }
    }
}

uint32_t foc_phase_voltage_quality_config_is_valid(
    const foc_phase_voltage_quality_config_t *config)
{
    return ((config != 0) &&
            (config->struct_size == sizeof(*config)) &&
            (config->version == FOC_PHASE_VOLTAGE_QUALITY_VERSION) &&
            (config->adc_max_code > 0U) &&
            (config->adc_max_code <= 65535U) &&
            (config->low_saturation_enter_code <
             config->low_saturation_release_code) &&
            (config->low_saturation_release_code <
             config->high_saturation_release_code) &&
            (config->high_saturation_release_code <
             config->high_saturation_enter_code) &&
            (config->high_saturation_enter_code <= config->adc_max_code) &&
            (config->open_stuck_enter_delta_codes <
             config->open_release_delta_codes) &&
            (config->open_release_delta_codes <= config->adc_max_code) &&
            (config->open_confirm_samples > 0U) &&
            (config->inconsistency_release_mv <
             config->inconsistency_enter_mv) &&
            (config->inconsistency_confirm_samples > 0U) &&
            (config->recovery_confirm_samples > 0U)) ? 1U : 0U;
}

void foc_phase_voltage_quality_init(foc_phase_voltage_quality_t *quality)
{
    if (quality == 0)
    {
        return;
    }
    memset(quality, 0, sizeof(*quality));
    quality->struct_size = sizeof(*quality);
    quality->version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    quality->last_state = FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED;
    quality->last_reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED;
    quality->last_selected_source =
        FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
}

uint32_t foc_phase_voltage_quality_evaluate(
    foc_phase_voltage_quality_t *quality,
    const foc_phase_voltage_quality_config_t *config,
    const foc_phase_voltage_quality_sample_t *sample,
    foc_phase_voltage_request_t requested_mode,
    foc_phase_voltage_quality_result_t *result)
{
    uint32_t reason_mask = 0U;
    uint32_t affected_phase_mask = 0U;
    uint32_t raw[3] = {0U, 0U, 0U};
    uint32_t sequence_contiguous = 0U;
    uint32_t recovery_confirm_samples = UINT32_MAX;
    uint32_t measured_eligible;
    uint32_t selected_source;
    uint32_t fallback_required;
    uint32_t unavailable_required;
    uint32_t was_fallback_active;
    uint32_t was_unavailable_active;
    foc_phase_voltage_quality_state_t state;

    if ((quality == 0) || (result == 0))
    {
        return 0U;
    }
    if ((quality->struct_size != sizeof(*quality)) ||
        (quality->version != FOC_PHASE_VOLTAGE_QUALITY_VERSION))
    {
        foc_phase_voltage_quality_init(quality);
    }

    memset(result, 0, sizeof(*result));
    result->struct_size = sizeof(*result);
    result->version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    result->state = FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED;
    result->reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED;
    result->requested_mode = (uint32_t)requested_mode;
    result->selected_source = FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    result->sample_sequence = (sample != 0) ? sample->sequence : 0U;

    if (foc_phase_voltage_quality_config_is_valid(config) == 0U)
    {
        reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED;
        foc_phase_voltage_clear_detectors(quality);
    }
    else if ((sample == 0) ||
             (sample->struct_size != sizeof(*sample)) ||
             (sample->version != FOC_PHASE_VOLTAGE_QUALITY_VERSION) ||
             (sample->reserved != 0U) ||
             ((sample->flags &
               ~((uint32_t)FOC_PHASE_VOLTAGE_SAMPLE_KNOWN_FLAGS)) != 0U) ||
             ((sample->expected_change_mask &
               ~((uint32_t)FOC_PHASE_VOLTAGE_PHASE_ALL)) != 0U))
    {
        reason_mask = FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE;
        foc_phase_voltage_clear_detectors(quality);
    }
    else if ((sample->flags & FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED) == 0U)
    {
        reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED;
        foc_phase_voltage_clear_detectors(quality);
    }
    else if ((sample->flags & FOC_PHASE_VOLTAGE_SAMPLE_CALIBRATED) == 0U)
    {
        reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED;
        foc_phase_voltage_clear_detectors(quality);
    }
    else if ((sample->flags &
              (FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID |
               FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID)) !=
             (FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID |
              FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID))
    {
        reason_mask = FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE;
        foc_phase_voltage_clear_detectors(quality);
    }
    else
    {
        recovery_confirm_samples = config->recovery_confirm_samples;
        raw[0] = sample->phase_u_raw;
        raw[1] = sample->phase_v_raw;
        raw[2] = sample->phase_w_raw;
        sequence_contiguous = ((quality->previous_valid == 0U) ||
                               (sample->sequence ==
                                (quality->previous_sequence + 1U))) ? 1U : 0U;
        if ((sample->age_ticks > config->max_sample_age_ticks) ||
            (sequence_contiguous == 0U))
        {
            reason_mask |= FOC_PHASE_VOLTAGE_REASON_STALE;
        }

        foc_phase_voltage_update_saturation(quality, config, raw);
        if (quality->low_saturation_phase_mask != 0U)
        {
            reason_mask |= FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION;
            affected_phase_mask |= quality->low_saturation_phase_mask;
        }
        if (quality->high_saturation_phase_mask != 0U)
        {
            reason_mask |= FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION;
            affected_phase_mask |= quality->high_saturation_phase_mask;
        }

        if ((sequence_contiguous != 0U) &&
            (sample->age_ticks <= config->max_sample_age_ticks) &&
            (quality->low_saturation_phase_mask == 0U) &&
            (quality->high_saturation_phase_mask == 0U))
        {
            foc_phase_voltage_update_open_detector(quality, config,
                                                   sample, raw);
            foc_phase_voltage_update_inconsistency(quality, config, sample);
        }
        if (quality->open_suspect_phase_mask != 0U)
        {
            reason_mask |= FOC_PHASE_VOLTAGE_REASON_OPEN_SUSPECT;
            affected_phase_mask |= quality->open_suspect_phase_mask;
        }
        if (quality->inconsistency_latched != 0U)
        {
            reason_mask |=
                FOC_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT;
            affected_phase_mask |= FOC_PHASE_VOLTAGE_PHASE_ALL;
        }

        quality->previous_sequence = sample->sequence;
        quality->previous_raw[0] = raw[0];
        quality->previous_raw[1] = raw[1];
        quality->previous_raw[2] = raw[2];
        quality->previous_valid = 1U;
    }

    if (foc_phase_voltage_request_is_valid((uint32_t)requested_mode) == 0U)
    {
        reason_mask |= FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE;
    }

    state = foc_phase_voltage_primary_state(reason_mask);
    if (state == FOC_PHASE_VOLTAGE_QUALITY_VALID)
    {
        foc_phase_voltage_increment(&quality->healthy_streak);
        foc_phase_voltage_increment(&quality->valid_sample_count);
    }
    else
    {
        quality->healthy_streak = 0U;
    }
    measured_eligible = ((state == FOC_PHASE_VOLTAGE_QUALITY_VALID) &&
                         (quality->healthy_streak >=
                          recovery_confirm_samples)) ? 1U : 0U;
    if ((state == FOC_PHASE_VOLTAGE_QUALITY_VALID) &&
        (measured_eligible == 0U))
    {
        reason_mask |= FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS;
    }

    selected_source = FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE;
    if (requested_mode == FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL)
    {
        selected_source = FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    }
    else if ((requested_mode == FOC_PHASE_VOLTAGE_REQUEST_MEASURED) &&
             (measured_eligible != 0U))
    {
        selected_source = FOC_PHASE_VOLTAGE_SELECTION_MEASURED;
    }
    else if (requested_mode == FOC_PHASE_VOLTAGE_REQUEST_HYBRID)
    {
        selected_source = (measured_eligible != 0U) ?
            FOC_PHASE_VOLTAGE_SELECTION_MEASURED :
            FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    }
    fallback_required = ((requested_mode ==
                          FOC_PHASE_VOLTAGE_REQUEST_HYBRID) &&
                         (selected_source ==
                          FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL)) ? 1U : 0U;
    unavailable_required = (selected_source ==
                            FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE) ? 1U : 0U;
    was_fallback_active = quality->fallback_active;
    was_unavailable_active = quality->unavailable_active;
    if (fallback_required != 0U)
    {
        foc_phase_voltage_increment(&quality->fallback_sample_count);
        if (was_fallback_active == 0U)
        {
            foc_phase_voltage_increment(&quality->fallback_event_count);
        }
    }
    if (unavailable_required != 0U)
    {
        foc_phase_voltage_increment(&quality->unavailable_sample_count);
        if (was_unavailable_active == 0U)
        {
            foc_phase_voltage_increment(&quality->unavailable_event_count);
        }
    }
    if (((was_fallback_active != 0U) ||
         (was_unavailable_active != 0U)) &&
        (selected_source == FOC_PHASE_VOLTAGE_SELECTION_MEASURED))
    {
        foc_phase_voltage_increment(&quality->recovery_count);
    }
    quality->fallback_active = fallback_required;
    quality->unavailable_active = unavailable_required;
    foc_phase_voltage_increment(&quality->evaluated_sample_count);
    foc_phase_voltage_count_reasons(quality, reason_mask);

    quality->last_state = (uint32_t)state;
    quality->last_reason_mask = reason_mask;
    quality->last_selected_source = selected_source;

    result->state = (uint32_t)state;
    result->reason_mask = reason_mask;
    result->selected_source = selected_source;
    result->measured_eligible = measured_eligible;
    result->fallback_active = fallback_required;
    result->unavailable_active = unavailable_required;
    result->affected_phase_mask = affected_phase_mask;
    result->consecutive_healthy_samples = quality->healthy_streak;
    result->fallback_sample_count = quality->fallback_sample_count;
    result->fallback_event_count = quality->fallback_event_count;
    result->unavailable_sample_count = quality->unavailable_sample_count;
    result->unavailable_event_count = quality->unavailable_event_count;
    result->recovery_count = quality->recovery_count;
    result->evaluated_sample_count = quality->evaluated_sample_count;
    return 1U;
}

void foc_phase_voltage_quality_get_status(
    const foc_phase_voltage_quality_t *quality,
    foc_phase_voltage_quality_status_t *status)
{
    uint32_t index;

    if (status == 0)
    {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_PHASE_VOLTAGE_QUALITY_VERSION;
    status->state = FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED;
    status->reason_mask = FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED;
    status->selected_source = FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL;
    if ((quality == 0) ||
        (quality->struct_size != sizeof(*quality)) ||
        (quality->version != FOC_PHASE_VOLTAGE_QUALITY_VERSION))
    {
        return;
    }
    status->state = quality->last_state;
    status->reason_mask = quality->last_reason_mask;
    status->selected_source = quality->last_selected_source;
    status->fallback_active = quality->fallback_active;
    status->unavailable_active = quality->unavailable_active;
    status->low_saturation_phase_mask =
        quality->low_saturation_phase_mask;
    status->high_saturation_phase_mask =
        quality->high_saturation_phase_mask;
    status->open_suspect_phase_mask = quality->open_suspect_phase_mask;
    status->consecutive_healthy_samples = quality->healthy_streak;
    status->evaluated_sample_count = quality->evaluated_sample_count;
    status->valid_sample_count = quality->valid_sample_count;
    status->fallback_sample_count = quality->fallback_sample_count;
    status->fallback_event_count = quality->fallback_event_count;
    status->unavailable_sample_count = quality->unavailable_sample_count;
    status->unavailable_event_count = quality->unavailable_event_count;
    status->recovery_count = quality->recovery_count;
    for (index = 0U; index < FOC_PHASE_VOLTAGE_QUALITY_REASON_COUNT; ++index)
    {
        status->reason_counts[index] = quality->reason_counts[index];
    }
}
