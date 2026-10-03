#ifndef FOC_ADVANCED_PROBE_H
#define FOC_ADVANCED_PROBE_H

/* Hardware-neutral guard for the P5.3 Advanced-Lab no-power target probe. */

#include <stdint.h>

#include "foc_power_safety.h"
#include "foc_rust_bridge.h"

#define FOC_ADVANCED_PROBE_VERSION    (1UL)
#define FOC_ADVANCED_PROBE_MIN_TICKS  (1UL)
#define FOC_ADVANCED_PROBE_MAX_TICKS  (120000UL)

typedef uint32_t foc_advanced_probe_state_t;
enum
{
    FOC_ADVANCED_PROBE_IDLE = 0,
    FOC_ADVANCED_PROBE_RUNNING = 1,
    FOC_ADVANCED_PROBE_COMPLETE = 2,
    FOC_ADVANCED_PROBE_FAILED = 3,
};

typedef uint32_t foc_advanced_probe_result_t;
enum
{
    FOC_ADVANCED_PROBE_RESULT_OK = 0,
    FOC_ADVANCED_PROBE_RESULT_COMPLETE = 1,
    FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT = 2,
    FOC_ADVANCED_PROBE_RESULT_INVALID_STATE = 3,
    FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE = 4,
    FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED = 5,
    FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE = 6,
};

typedef struct
{
    uint32_t control_armed;
    uint32_t power_safety_state;
    uint32_t fault_epoch;
    uint32_t break_latched;
    uint32_t driver_faulted;
    uint32_t gate_is_low;
    uint32_t moe_enabled;
    uint32_t phase_channels_enabled;
} foc_advanced_probe_register_snapshot_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_advanced_probe_state_t state;
    foc_advanced_probe_result_t last_result;
    uint32_t requested_ticks;
    uint32_t executed_ticks;
    uint32_t no_power_commit_count;
    uint32_t rejected_tick_count;
    uint32_t expected_fault_epoch;
    uint32_t observed_fault_epoch;
    foc_status_t last_control_status;
    uint32_t decision_signature;
    uint32_t decision_samples;
} foc_advanced_probe_status_t;

typedef foc_advanced_probe_status_t foc_advanced_probe_t;

foc_advanced_probe_result_t foc_advanced_probe_init(
    foc_advanced_probe_t *probe);
foc_advanced_probe_result_t foc_advanced_probe_start(
    foc_advanced_probe_t *probe,
    uint32_t requested_ticks,
    uint32_t expected_fault_epoch,
    const foc_advanced_probe_register_snapshot_t *registers);
foc_advanced_probe_result_t foc_advanced_probe_begin_tick(
    foc_advanced_probe_t *probe,
    const foc_advanced_probe_register_snapshot_t *registers);
foc_advanced_probe_result_t foc_advanced_probe_complete_control(
    foc_advanced_probe_t *probe,
    foc_status_t control_status,
    const foc_advanced_probe_register_snapshot_t *registers);
foc_advanced_probe_result_t foc_advanced_probe_complete_commit(
    foc_advanced_probe_t *probe,
    const foc_advanced_probe_register_snapshot_t *before,
    const foc_advanced_probe_register_snapshot_t *after);
foc_advanced_probe_result_t foc_advanced_probe_fail(
    foc_advanced_probe_t *probe,
    foc_advanced_probe_result_t reason,
    uint32_t observed_fault_epoch);
foc_advanced_probe_result_t foc_advanced_probe_get_status(
    const foc_advanced_probe_t *probe,
    foc_advanced_probe_status_t *status);
uint32_t foc_advanced_probe_registers_are_safe_off(
    const foc_advanced_probe_register_snapshot_t *registers);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_advanced_probe_register_snapshot_t) == 32U,
               "advanced probe register snapshot layout drifted");
_Static_assert(sizeof(foc_advanced_probe_status_t) == 52U,
               "advanced probe status layout drifted");
#endif

#endif /* FOC_ADVANCED_PROBE_H */
