#ifndef FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_H
#define FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_H

/*
 * FluxRT - STM32G431 phase-voltage nominal-model adapter.
 *
 * The adapter has no register, HAL, RTOS, or ISR dependency.  It converts a
 * captured raw sample with the existing ST nominal model and feeds the A22
 * quality core.  Version 1 deliberately never asserts CALIBRATED: converted
 * millivolts are diagnostic-only and can never become observer-eligible.
 */

#include <stdint.h>

#include "foc_phase_voltage_capture.h"
#include "foc_phase_voltage_model.h"
#include "foc_phase_voltage_quality.h"

#define FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_VERSION (1UL)

enum
{
    FOC_PHASE_VOLTAGE_ADAPTER_INPUT_RAW_VALID = (1UL << 0),
    FOC_PHASE_VOLTAGE_ADAPTER_INPUT_COMMAND_REFERENCE_VALID = (1UL << 1),
    FOC_PHASE_VOLTAGE_ADAPTER_INPUT_KNOWN_MASK =
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_RAW_VALID |
        FOC_PHASE_VOLTAGE_ADAPTER_INPUT_COMMAND_REFERENCE_VALID,
};

/* Adapter v1 is an atomic diagnostic snapshot: RAW_VALID and
 * COMMAND_REFERENCE_VALID must both be present and no unknown flag is
 * accepted.  Partial snapshots are rejected instead of being reinterpreted.
 */

enum
{
    FOC_PHASE_VOLTAGE_ADAPTER_MODEL_VALID = (1UL << 0),
    FOC_PHASE_VOLTAGE_ADAPTER_CONVERSION_VALID = (1UL << 1),
    FOC_PHASE_VOLTAGE_ADAPTER_NOMINAL_ONLY = (1UL << 2),
    FOC_PHASE_VOLTAGE_ADAPTER_OBSERVER_LOCKED_OUT = (1UL << 3),
    FOC_PHASE_VOLTAGE_ADAPTER_STALE_OBSERVED = (1UL << 4),
    FOC_PHASE_VOLTAGE_ADAPTER_INPUT_REJECTED = (1UL << 5),
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t flags;
    uint32_t divider_mode;
    uint32_t age_ticks;
    uint32_t expected_change_mask;
    uint32_t requested_mode;
    uint32_t command_u_mv;
    uint32_t command_v_mv;
    uint32_t command_w_mv;
    foc_phase_voltage_sample_t raw_sample;
} foc_phase_voltage_platform_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_phase_voltage_quality_t quality;
    uint32_t previous_sequence;
    uint32_t previous_sequence_valid;
    uint32_t evaluated_sample_count;
    uint32_t stale_sample_count;
    uint32_t conversion_failure_count;
    uint32_t input_reject_count;
} foc_phase_voltage_platform_adapter_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t flags;
    uint32_t model_source;
    uint32_t observer_eligible;
    uint32_t phase_u_mv;
    uint32_t phase_v_mv;
    uint32_t phase_w_mv;
    uint32_t phase_sequence;
    uint32_t stale_sample_count;
    uint32_t conversion_failure_count;
    uint32_t input_reject_count;
    foc_phase_voltage_quality_result_t quality;
} foc_phase_voltage_platform_output_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_phase_voltage_platform_input_t) == 56U,
               "phase-voltage platform input ABI changed");
_Static_assert(sizeof(foc_phase_voltage_platform_output_t) == 120U,
               "phase-voltage platform output ABI changed");
#endif

void foc_phase_voltage_platform_adapter_init(
    foc_phase_voltage_platform_adapter_t *adapter);

/*
 * Returns zero only when adapter/output cannot be written.  Invalid or absent
 * model/input/config still produces a deterministic fail-closed quality result.
 * observer_eligible is always zero in adapter version 1.  STALE_OBSERVED and
 * stale_sample_count are adapter diagnostics: because v1 is deliberately
 * uncalibrated, they are not injected into the A22 quality reason counters.
 * Full V19 sequence/age coherence additionally requires the assembly layer to
 * verify control_sequence - phase_sequence == age_ticks with wrapping maths.
 */
uint32_t foc_phase_voltage_platform_adapter_evaluate(
    foc_phase_voltage_platform_adapter_t *adapter,
    const foc_phase_voltage_quality_config_t *quality_config,
    const foc_phase_voltage_model_t *model,
    const foc_phase_voltage_platform_input_t *input,
    foc_phase_voltage_platform_output_t *output);

#endif /* FOC_PHASE_VOLTAGE_PLATFORM_ADAPTER_H */
