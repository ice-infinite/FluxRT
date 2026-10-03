#ifndef FOC_MOTION_MANAGEMENT_H
#define FOC_MOTION_MANAGEMENT_H

/*
 * FluxRT default-off motion management owner.
 *
 * This application-layer service owns one foc_motion_adapter_t lifecycle. It
 * has no RT-Thread, HAL, register or Flash dependency and is not an ISR hook.
 * Every API is single-execution-context only: step/poll and lifecycle calls
 * must never concurrently mutate one instance from an ISR and a task. A later
 * target dispatcher must give the realtime context one writer and exchange
 * fixed snapshots/atomic requests with the management task.
 *
 * The caller supplies already-arbitrated ProductCommand and physical feedback.
 * Timeout, source/owner mismatch and motion errors fail closed through the
 * force_safe callback and a latched FAILED state.
 */

#include <stdint.h>

#include "foc_motion_adapter.h"

#define FOC_MOTION_MANAGEMENT_VERSION          (1UL)
#define FOC_MOTION_MANAGEMENT_MIN_TIMEOUT_MS   (1UL)
#define FOC_MOTION_MANAGEMENT_MAX_TIMEOUT_MS   (60000UL)

typedef uint32_t foc_motion_management_result_t;
enum
{
    FOC_MOTION_MANAGEMENT_OK = 0,
    FOC_MOTION_MANAGEMENT_ALREADY_SAFE = 1,
    FOC_MOTION_MANAGEMENT_DISABLED = 2,
    FOC_MOTION_MANAGEMENT_INVALID_ARGUMENT = 3,
    FOC_MOTION_MANAGEMENT_NOT_READY = 4,
    FOC_MOTION_MANAGEMENT_BUSY = 5,
    FOC_MOTION_MANAGEMENT_NOT_OWNER = 6,
    FOC_MOTION_MANAGEMENT_EXPIRED = 7,
    FOC_MOTION_MANAGEMENT_MOTION_ERROR = 8,
    FOC_MOTION_MANAGEMENT_UNSAFE_STATE = 9,
    FOC_MOTION_MANAGEMENT_FAILED_LOCKED = 10,
    FOC_MOTION_MANAGEMENT_WRONG_SOURCE = 11,
    FOC_MOTION_MANAGEMENT_STALE_SEQUENCE = 12,
};

typedef uint32_t foc_motion_management_state_t;
enum
{
    FOC_MOTION_MANAGEMENT_STATE_DISABLED = 0,
    FOC_MOTION_MANAGEMENT_STATE_IDLE = 1,
    FOC_MOTION_MANAGEMENT_STATE_ACTIVE = 2,
    FOC_MOTION_MANAGEMENT_STATE_FAILED = 3,
};

typedef void (*foc_motion_management_force_safe_fn)(void *context);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_motion_management_force_safe_fn force_safe;
    void *context;
} foc_motion_management_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t enabled;
    foc_motion_management_state_t state;
    uint32_t owner_id;
    uint32_t source_id;
    uint32_t started_at_ms;
    uint32_t last_activity_ms;
    uint32_t timeout_ms;
    uint32_t last_sequence;
    uint32_t sequence_valid;
    uint32_t shutdown_required;
    uint32_t begin_count;
    uint32_t stop_count;
    uint32_t timeout_count;
    uint32_t failure_count;
    uint32_t step_count;
    foc_motion_management_result_t last_result;
    uint32_t last_detail;
    foc_motion_status_t motion_status;
    foc_motion_output_t output;
} foc_motion_management_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t enabled;
    foc_motion_management_state_t state;
    uint32_t owner_id;
    uint32_t source_id;
    uint32_t started_at_ms;
    uint32_t last_activity_ms;
    uint32_t timeout_ms;
    uint32_t last_sequence;
    uint32_t sequence_valid;
    uint32_t shutdown_required;
    uint32_t begin_count;
    uint32_t stop_count;
    uint32_t timeout_count;
    uint32_t failure_count;
    uint32_t step_count;
    foc_motion_management_result_t last_result;
    uint32_t last_detail;
    foc_motion_adapter_t *adapter;
    foc_motion_management_ops_t ops;
    foc_motion_output_t output;
} foc_motion_management_t;

/* Initialization configures the adapter but leaves both layers disabled. */
foc_motion_management_result_t foc_motion_management_init(
    foc_motion_management_t *management,
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_motion_management_ops_t *ops);

/* Feature gate. Disable is unconditional and always forces a safe output. */
foc_motion_management_result_t foc_motion_management_set_enabled(
    foc_motion_management_t *management,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard);

/* Reconfiguration is accepted only while the owner is idle and hardware-safe. */
foc_motion_management_result_t foc_motion_management_apply_config(
    foc_motion_management_t *management,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard);

foc_motion_management_result_t foc_motion_management_get_status(
    foc_motion_management_t *management,
    foc_motion_management_status_t *status);

/* Begin one source-owned session while the Axis is Disabled and output is off. */
foc_motion_management_result_t foc_motion_management_begin(
    foc_motion_management_t *management,
    uint32_t owner_id,
    uint32_t source_id,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const foc_config_apply_guard_t *guard);

/*
 * Execute one already-arbitrated command/feedback tick.
 * Any rejected tick returns a zeroed output. Motion-core errors latch FAILED;
 * caller identity/source/sequence mistakes are rejected without advancing state.
 */
foc_motion_management_result_t foc_motion_management_step(
    foc_motion_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output);

/* Owner-requested normal stop. Wrong-owner requests cannot consume the session. */
foc_motion_management_result_t foc_motion_management_stop(
    foc_motion_management_t *management,
    uint32_t owner_id);

/* Unconditional external safety path for hardware faults and emergency stop. */
foc_motion_management_result_t foc_motion_management_latch_fault(
    foc_motion_management_t *management,
    uint32_t detail);

/* Management-thread timeout poll; unsigned subtraction supports u32 wrap. */
foc_motion_management_result_t foc_motion_management_poll(
    foc_motion_management_t *management,
    uint32_t now_ms);

/* Clear FAILED only after the Axis/output/fault guard proves a safe state. */
foc_motion_management_result_t foc_motion_management_recover(
    foc_motion_management_t *management,
    const foc_config_apply_guard_t *guard);

#endif /* FOC_MOTION_MANAGEMENT_H */
