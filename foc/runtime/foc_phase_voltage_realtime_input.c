#include "foc_phase_voltage_realtime_input.h"

#include <math.h>
#include <string.h>

#include "foc_rust_bridge.h"

enum
{
    FOC_PHASE_VOLTAGE_ADAPTER_OUTPUT_KNOWN_MASK =
        FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID |
        FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID |
        FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY |
        FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT |
        FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED |
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED,
};

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert((uint32_t)FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED ==
                   (uint32_t)FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCONFIGURED,
               "A22/V19 quality values diverged");
_Static_assert((uint32_t)FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE ==
                   (uint32_t)FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE,
               "A22/V19 quality values diverged");
_Static_assert((uint32_t)FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL ==
                   (uint32_t)FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL,
               "A22/V19 selection values diverged");
_Static_assert((uint32_t)FOC_PHASE_VOLTAGE_SELECTION_MEASURED ==
                   (uint32_t)FOC_REALTIME_OBSERVER_VOLTAGE_MEASURED,
               "A22/V19 selection values diverged");
_Static_assert((uint32_t)FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE ==
                   (uint32_t)FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE,
               "A22/V19 selection values diverged");
#endif

static uint32_t foc_phase_voltage_realtime_float_is_finite(float value)
{
    return isfinite(value) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_request_mode_is_valid(
    uint32_t mode)
{
    return (mode <= FOC_PHASE_VOLTAGE_REQUEST_HYBRID) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_request_is_valid(
    const foc_phase_voltage_realtime_input_request_t *request)
{
    const foc_feedback_t *feedback;

    if ((request == 0) ||
        (request->struct_size != sizeof(*request)) ||
        (request->version !=
         FOC_PHASE_VOLTAGE_REALTIME_INPUT_ASSEMBLER_VERSION) ||
        ((request->hardware_fault_flags &
          ~((uint32_t)FOC_REALTIME_HW_FAULT_KNOWN_MASK)) != 0U) ||
        (foc_phase_voltage_realtime_request_mode_is_valid(
             request->requested_mode) == 0U) ||
        (foc_phase_voltage_realtime_float_is_finite(request->actual_dt_s) ==
         0U) ||
        (request->actual_dt_s <= 0.0f))
    {
        return 0U;
    }

    feedback = &request->legacy_feedback;
    if ((foc_phase_voltage_realtime_float_is_finite(
             feedback->phase_current_a) == 0U) ||
        (foc_phase_voltage_realtime_float_is_finite(
             feedback->phase_current_b) == 0U) ||
        (foc_phase_voltage_realtime_float_is_finite(
             feedback->phase_current_c) == 0U) ||
        (foc_phase_voltage_realtime_float_is_finite(
             feedback->dc_bus_voltage) == 0U) ||
        (feedback->dc_bus_voltage <= 0.0f) ||
        (foc_phase_voltage_realtime_float_is_finite(
             feedback->electrical_angle_rad) == 0U))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t foc_phase_voltage_realtime_quality_state_is_valid(
    uint32_t state)
{
    return (state <= FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_selection_is_valid(
    uint32_t selection)
{
    return (selection <= FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_bool_is_valid(uint32_t value)
{
    return (value <= 1U) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_quality_is_valid(
    const foc_phase_voltage_quality_result_t *quality)
{
    if ((quality->struct_size != sizeof(*quality)) ||
        (quality->version != FOC_PHASE_VOLTAGE_QUALITY_VERSION) ||
        (foc_phase_voltage_realtime_quality_state_is_valid(quality->state) ==
         0U) ||
        ((quality->reason_mask &
          ~((uint32_t)FOC_REALTIME_PHASE_VOLTAGE_REASON_KNOWN_MASK)) != 0U) ||
        (foc_phase_voltage_realtime_request_mode_is_valid(
             quality->requested_mode) == 0U) ||
        (foc_phase_voltage_realtime_selection_is_valid(
             quality->selected_source) == 0U) ||
        (foc_phase_voltage_realtime_bool_is_valid(
             quality->measured_eligible) == 0U) ||
        (foc_phase_voltage_realtime_bool_is_valid(
             quality->fallback_active) == 0U) ||
        (foc_phase_voltage_realtime_bool_is_valid(
             quality->unavailable_active) == 0U) ||
        ((quality->affected_phase_mask &
          ~((uint32_t)FOC_PHASE_VOLTAGE_PHASE_ALL)) != 0U))
    {
        return 0U;
    }

    if ((quality->measured_eligible != 0U) &&
        ((quality->state != FOC_PHASE_VOLTAGE_QUALITY_VALID) ||
         (quality->reason_mask != 0U)))
    {
        return 0U;
    }

    switch (quality->requested_mode)
    {
        case FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL:
            return ((quality->selected_source ==
                     FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL) &&
                    (quality->fallback_active == 0U) &&
                    (quality->unavailable_active == 0U)) ? 1U : 0U;

        case FOC_PHASE_VOLTAGE_REQUEST_MEASURED:
            if (quality->selected_source ==
                FOC_PHASE_VOLTAGE_SELECTION_MEASURED)
            {
                return ((quality->measured_eligible != 0U) &&
                        (quality->fallback_active == 0U) &&
                        (quality->unavailable_active == 0U)) ? 1U : 0U;
            }
            return ((quality->selected_source ==
                     FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE) &&
                    (quality->measured_eligible == 0U) &&
                    (quality->fallback_active == 0U) &&
                    (quality->unavailable_active != 0U)) ? 1U : 0U;

        case FOC_PHASE_VOLTAGE_REQUEST_HYBRID:
            if (quality->selected_source ==
                FOC_PHASE_VOLTAGE_SELECTION_MEASURED)
            {
                return ((quality->measured_eligible != 0U) &&
                        (quality->fallback_active == 0U) &&
                        (quality->unavailable_active == 0U)) ? 1U : 0U;
            }
            return ((quality->selected_source ==
                     FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL) &&
                    (quality->measured_eligible == 0U) &&
                    (quality->fallback_active != 0U) &&
                    (quality->unavailable_active == 0U)) ? 1U : 0U;

        default:
            return 0U;
    }
}

static uint32_t foc_phase_voltage_realtime_model_is_valid(
    const foc_phase_voltage_platform_output_t *adapter_output)
{
    const uint32_t flags = adapter_output->flags;

    if ((flags & ~((uint32_t)
                   FOC_PHASE_VOLTAGE_ADAPTER_OUTPUT_KNOWN_MASK)) != 0U)
    {
        return 0U;
    }
    if ((foc_phase_voltage_realtime_bool_is_valid(
             adapter_output->observer_eligible) == 0U) ||
        ((flags & FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT) == 0U))
    {
        return 0U;
    }
    if (((flags & FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID) != 0U) &&
        ((flags & FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID) == 0U))
    {
        return 0U;
    }
    if (((flags & FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED) != 0U) &&
        ((flags & (FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID |
                   FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID |
                   FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY)) != 0U))
    {
        return 0U;
    }

    switch (adapter_output->model_source)
    {
        case FOC_PHASE_VOLTAGE_MODEL_NONE:
            if ((flags & (FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID |
                          FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID |
                          FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY)) != 0U)
            {
                return 0U;
            }
            break;

        case FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL:
            if (((flags & FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID) == 0U) ||
                ((flags & FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY) == 0U) ||
                ((flags & FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT) ==
                 0U) ||
                (adapter_output->observer_eligible != 0U))
            {
                return 0U;
            }
            break;

        case FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED:
            /* Adapter output version 1 cannot carry a board-calibrated model.
             * A caller-crafted value is not evidence and must not pre-open the
             * future Measured path. */
            return 0U;

        default:
            return 0U;
    }

    return (adapter_output->observer_eligible == 0U) ? 1U : 0U;
}

static uint32_t foc_phase_voltage_realtime_quality_matches_model_v1(
    const foc_phase_voltage_platform_output_t *adapter_output)
{
    const foc_phase_voltage_quality_result_t *quality =
        &adapter_output->quality;

    switch (adapter_output->model_source)
    {
        case FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL:
            /* Adapter v1 never asserts CALIBRATED.  A nominal model can only
             * be waiting for configuration or explicitly uncalibrated; in
             * particular it can never claim VALID/recovery-hysteresis. */
            return (((quality->state ==
                      FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED) &&
                     (quality->reason_mask ==
                      FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED)) ||
                    ((quality->state ==
                      FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED) &&
                     (quality->reason_mask ==
                      FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED))) ? 1U : 0U;

        case FOC_PHASE_VOLTAGE_MODEL_NONE:
            /* Without a model the adapter can only report an unconfigured
             * sample or a structurally invalid one. */
            return (((quality->state ==
                      FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED) &&
                     (quality->reason_mask ==
                      FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED)) ||
                    ((quality->state ==
                      FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE) &&
                     (quality->reason_mask ==
                      FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE))) ? 1U : 0U;

        default:
            return 0U;
    }
}

static uint32_t foc_phase_voltage_realtime_adapter_is_valid(
    const foc_phase_voltage_realtime_input_request_t *request,
    const foc_phase_voltage_platform_output_t *adapter_output)
{
    if ((adapter_output == 0) ||
        (adapter_output->struct_size != sizeof(*adapter_output)) ||
        (adapter_output->version !=
         FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION) ||
        (foc_phase_voltage_realtime_model_is_valid(adapter_output) == 0U) ||
        (foc_phase_voltage_realtime_quality_is_valid(
             &adapter_output->quality) == 0U) ||
        (foc_phase_voltage_realtime_quality_matches_model_v1(
             adapter_output) == 0U) ||
        (adapter_output->quality.requested_mode !=
         request->requested_mode) ||
        (adapter_output->quality.sample_sequence !=
         adapter_output->phase_sequence) ||
        ((request->control_sequence - adapter_output->phase_sequence) !=
         request->phase_voltage_age_ticks))
    {
        return 0U;
    }

    if ((adapter_output->quality.selected_source ==
         FOC_PHASE_VOLTAGE_SELECTION_MEASURED) ||
        (adapter_output->quality.measured_eligible != 0U) ||
        (adapter_output->observer_eligible != 0U))
    {
        /* A22.2 output version 1 is permanently uncalibrated.  A future
         * calibrated adapter must introduce and review a new contract version
         * before this assembly layer can emit Measured. */
        return 0U;
    }
    return 1U;
}

static uint32_t foc_phase_voltage_realtime_provenance(
    uint32_t model_source)
{
    switch (model_source)
    {
        case FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL:
            return FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_ST_NOMINAL;
        default:
            return FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_NONE;
    }
}

static uint32_t foc_phase_voltage_realtime_map_quality_state(
    uint32_t state,
    uint32_t *mapped)
{
    if (mapped == 0)
    {
        return 0U;
    }
    switch (state)
    {
        case FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCONFIGURED;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCALIBRATED;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_VALID:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_STALE:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_STALE;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_LOW_SATURATION:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_LOW_SATURATION;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT:
            *mapped =
                FOC_REALTIME_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT;
            break;
        case FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE:
            *mapped = FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE;
            break;
        default:
            return 0U;
    }
    return 1U;
}

static uint32_t foc_phase_voltage_realtime_map_reason_mask(
    uint32_t reason_mask)
{
    uint32_t mapped = 0U;

    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCONFIGURED;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCALIBRATED;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_STALE) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_LOW_SATURATION;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_HIGH_SATURATION;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_OPEN_SUSPECT) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_OPEN_SUSPECT;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT;
    if ((reason_mask & FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS) != 0U)
        mapped |= FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS;
    return mapped;
}

static uint32_t foc_phase_voltage_realtime_map_selection(
    uint32_t selection,
    uint32_t *mapped)
{
    if (mapped == 0)
    {
        return 0U;
    }
    switch (selection)
    {
        case FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL:
            *mapped = FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL;
            break;
        case FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE:
            *mapped = FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE;
            break;
        case FOC_PHASE_VOLTAGE_SELECTION_MEASURED:
        default:
            return 0U;
    }
    return 1U;
}

uint32_t foc_phase_voltage_realtime_input_assemble(
    const foc_phase_voltage_realtime_input_request_t *request,
    const foc_phase_voltage_platform_output_t *adapter_output,
    foc_realtime_input_t *output)
{
    uint32_t mapped_quality_state;
    uint32_t mapped_selection;

    if (output == 0)
    {
        return 0U;
    }
    memset(output, 0, sizeof(*output));

    if ((foc_phase_voltage_realtime_request_is_valid(request) == 0U) ||
        (foc_phase_voltage_realtime_adapter_is_valid(request,
                                                     adapter_output) == 0U) ||
        (foc_phase_voltage_realtime_map_quality_state(
             adapter_output->quality.state, &mapped_quality_state) == 0U) ||
        (foc_phase_voltage_realtime_map_selection(
             adapter_output->quality.selected_source,
             &mapped_selection) == 0U))
    {
        return 0U;
    }

    foc_realtime_input_from_legacy(&request->legacy_feedback,
                                   request->control_sequence,
                                   request->actual_dt_s,
                                   output);
    output->hardware_fault_flags = request->hardware_fault_flags;
    output->phase_voltage_sequence = adapter_output->phase_sequence;
    output->phase_voltage_age_ticks = request->phase_voltage_age_ticks;
    output->phase_voltage_provenance =
        foc_phase_voltage_realtime_provenance(adapter_output->model_source);
    output->phase_voltage_quality_state = mapped_quality_state;
    output->phase_voltage_reason_mask =
        foc_phase_voltage_realtime_map_reason_mask(
            adapter_output->quality.reason_mask);
    output->observer_voltage_selection = mapped_selection;
    output->phase_voltage_fallback_event_count =
        adapter_output->quality.fallback_event_count;

    /* Adapter v1 millivolts remain C-side diagnostics.  No phase-voltage
     * validity bit or diagnostic voltage value crosses V19. */
    return 1U;
}
