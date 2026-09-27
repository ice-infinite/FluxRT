#ifndef FOC_PHASE_VOLTAGE_REALTIME_INPUT_H
#define FOC_PHASE_VOLTAGE_REALTIME_INPUT_H

/*
 * FluxRT - hardware-neutral A22 adapter-output to V19 input assembly.
 *
 * This layer owns no ADC, HAL, RTOS, ISR, or observer state.  It validates one
 * complete platform-adapter snapshot, proves its sequence/age relationship,
 * and then materialises the stable foc_realtime_input_t ABI.  Version 1 keeps
 * ST nominal data diagnostic-only: a Measured request becomes UNAVAILABLE and
 * Hybrid becomes CommandModel fallback.
 */

#include <stdint.h>

#include "foc_phase_voltage_platform_adapter.h"
#include "foc_types.h"

#define FOC_PHASE_VOLTAGE_REALTIME_INPUT_ASSEMBLER_VERSION (1UL)

/*
 * Values that do not belong to the phase-voltage adapter are explicit inputs
 * to the assembler.  phase_voltage_age_ticks must be in the same wrapping
 * sequence domain as control_sequence and adapter_output.phase_sequence.
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t control_sequence;
    uint32_t phase_voltage_age_ticks;
    uint32_t hardware_fault_flags;
    uint32_t requested_mode;
    float actual_dt_s;
    foc_feedback_t legacy_feedback;
} foc_phase_voltage_realtime_input_request_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_phase_voltage_realtime_input_request_t) == 48U,
               "phase-voltage realtime input request ABI changed");
#endif

/*
 * Returns 1 only after a complete V19 snapshot has been assembled.  On every
 * failure, output is all-zero when output is non-null.  In particular, an
 * unknown field, a non-finite legacy value, a sequence/age mismatch, or any
 * v1 Measured/board-calibrated claim is rejected rather than silently
 * downgraded.  A later calibrated path requires an explicit version bump.
 */
uint32_t foc_phase_voltage_realtime_input_assemble(
    const foc_phase_voltage_realtime_input_request_t *request,
    const foc_phase_voltage_platform_output_t *adapter_output,
    foc_realtime_input_t *output);

#endif /* FOC_PHASE_VOLTAGE_REALTIME_INPUT_H */
