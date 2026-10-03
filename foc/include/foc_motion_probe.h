#ifndef FOC_MOTION_PROBE_H
#define FOC_MOTION_PROBE_H

/*
 * FluxRT P4.2E1B no-power motion timing guard.
 *
 * This module is hardware-neutral.  The STM32 platform snapshots the physical
 * output registers before/after a dry-run CCR preload and feeds those facts to
 * this state machine.  A dry run is permitted only while the power stage is
 * provably off; any fault-epoch or register change latches FAILED.
 */

#include <stdint.h>

#include "foc_power_safety.h"
#include "foc_rust_bridge.h"

#define FOC_MOTION_PROBE_VERSION             (1UL)
#define FOC_MOTION_PROBE_MIN_TICKS           (1UL)
#define FOC_MOTION_PROBE_MAX_TICKS           (120000UL)

typedef uint32_t foc_motion_probe_state_t;
enum
{
    FOC_MOTION_PROBE_IDLE = 0,
    FOC_MOTION_PROBE_CONFIGURED = 1,
    FOC_MOTION_PROBE_RUNNING = 2,
    FOC_MOTION_PROBE_COMPLETE = 3,
    FOC_MOTION_PROBE_FAILED = 4,
};

typedef uint32_t foc_motion_probe_result_t;
enum
{
    FOC_MOTION_PROBE_RESULT_OK = 0,
    FOC_MOTION_PROBE_RESULT_COMPLETE = 1,
    FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT = 2,
    FOC_MOTION_PROBE_RESULT_INVALID_STATE = 3,
    FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE = 4,
    FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED = 5,
    FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE = 6,
};

typedef uint32_t foc_motion_probe_injection_point_t;
enum
{
    FOC_MOTION_PROBE_INJECT_NONE = 0,
    FOC_MOTION_PROBE_INJECT_CONFIGURE_BEFORE_ROUTE = 1,
    FOC_MOTION_PROBE_INJECT_CONFIGURE_AFTER_ROUTE = 2,
    FOC_MOTION_PROBE_INJECT_BEFORE_COMBINED = 3,
    /* Models a priority-0 Break that preempted the Rust call.  The platform
     * latches the fault immediately after the call returns and before commit. */
    FOC_MOTION_PROBE_INJECT_DURING_COMBINED = 4,
    FOC_MOTION_PROBE_INJECT_BEFORE_COMMIT = 5,
    FOC_MOTION_PROBE_INJECT_AFTER_COMMIT = 6,
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
} foc_motion_probe_register_snapshot_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_motion_probe_state_t state;
    foc_motion_probe_result_t last_result;
    uint32_t requested_ticks;
    uint32_t executed_ticks;
    uint32_t no_power_commit_count;
    uint32_t rejected_tick_count;
    uint32_t expected_fault_epoch;
    uint32_t observed_fault_epoch;
    foc_status_t last_control_status;
    foc_motion_probe_injection_point_t pending_injection;
    uint32_t injection_count;
} foc_motion_probe_status_t;

typedef foc_motion_probe_status_t foc_motion_probe_t;

foc_motion_probe_result_t foc_motion_probe_init(foc_motion_probe_t *probe);
foc_motion_probe_result_t foc_motion_probe_configure(
    foc_motion_probe_t *probe,
    uint32_t expected_fault_epoch);
foc_motion_probe_result_t foc_motion_probe_start(
    foc_motion_probe_t *probe,
    uint32_t requested_ticks,
    const foc_motion_probe_register_snapshot_t *registers);
foc_motion_probe_result_t foc_motion_probe_begin_tick(
    foc_motion_probe_t *probe,
    const foc_motion_probe_register_snapshot_t *registers);
foc_motion_probe_result_t foc_motion_probe_complete_combined(
    foc_motion_probe_t *probe,
    foc_status_t control_status,
    const foc_motion_probe_register_snapshot_t *registers);
foc_motion_probe_result_t foc_motion_probe_complete_commit(
    foc_motion_probe_t *probe,
    const foc_motion_probe_register_snapshot_t *before,
    const foc_motion_probe_register_snapshot_t *after);
foc_motion_probe_result_t foc_motion_probe_fail(
    foc_motion_probe_t *probe,
    foc_motion_probe_result_t reason,
    uint32_t observed_fault_epoch);
foc_motion_probe_result_t foc_motion_probe_set_injection(
    foc_motion_probe_t *probe,
    foc_motion_probe_injection_point_t point);
uint32_t foc_motion_probe_take_injection(
    foc_motion_probe_t *probe,
    foc_motion_probe_injection_point_t point);
foc_motion_probe_result_t foc_motion_probe_get_status(
    const foc_motion_probe_t *probe,
    foc_motion_probe_status_t *status);
uint32_t foc_motion_probe_registers_are_safe_off(
    const foc_motion_probe_register_snapshot_t *registers);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_motion_probe_register_snapshot_t) == 32U,
               "motion probe register snapshot layout drifted");
_Static_assert(sizeof(foc_motion_probe_status_t) == 52U,
               "motion probe status layout drifted");
#endif

#endif /* FOC_MOTION_PROBE_H */
