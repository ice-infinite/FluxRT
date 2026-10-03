#ifndef FOC_FEEDBACK_MANAGEMENT_H
#define FOC_FEEDBACK_MANAGEMENT_H

/*
 * FluxRT feedback-calibration management coordinator.
 *
 * This is an application/management-plane service. It never runs from the
 * control ISR, never touches sensor registers and never writes Flash. The
 * caller owns and serializes the Rust feedback/config contexts. A successful
 * stage operation leaves a validated configuration transaction for the
 * parameter service; only an externally completed commit/readback may finish
 * the calibration session.
 */

#include <stdint.h>

#include "foc_feedback_bridge.h"

#define FOC_FEEDBACK_MANAGEMENT_VERSION          (1UL)
#define FOC_FEEDBACK_MANAGEMENT_MIN_TIMEOUT_MS   (1UL)
#define FOC_FEEDBACK_MANAGEMENT_MAX_TIMEOUT_MS   (600000UL)

typedef uint32_t foc_feedback_management_result_t;
enum
{
    FOC_FEEDBACK_MANAGEMENT_OK = 0,
    FOC_FEEDBACK_MANAGEMENT_ALREADY_SAFE = 1,
    FOC_FEEDBACK_MANAGEMENT_DISABLED = 2,
    FOC_FEEDBACK_MANAGEMENT_INVALID_ARGUMENT = 3,
    FOC_FEEDBACK_MANAGEMENT_NOT_READY = 4,
    FOC_FEEDBACK_MANAGEMENT_BUSY = 5,
    FOC_FEEDBACK_MANAGEMENT_NOT_OWNER = 6,
    FOC_FEEDBACK_MANAGEMENT_EXPIRED = 7,
    FOC_FEEDBACK_MANAGEMENT_FEEDBACK_ERROR = 8,
    FOC_FEEDBACK_MANAGEMENT_CONFIG_ERROR = 9,
    FOC_FEEDBACK_MANAGEMENT_UNSAFE_STATE = 10,
    FOC_FEEDBACK_MANAGEMENT_FAILED_LOCKED = 11,
};

typedef uint32_t foc_feedback_management_state_t;
enum
{
    FOC_FEEDBACK_MANAGEMENT_STATE_DISABLED = 0,
    FOC_FEEDBACK_MANAGEMENT_STATE_IDLE = 1,
    FOC_FEEDBACK_MANAGEMENT_STATE_COLLECTING = 2,
    FOC_FEEDBACK_MANAGEMENT_STATE_READY_FOR_REVIEW = 3,
    FOC_FEEDBACK_MANAGEMENT_STATE_STAGED = 4,
    FOC_FEEDBACK_MANAGEMENT_STATE_FAILED = 5,
};

enum
{
    FOC_FEEDBACK_EVIDENCE_DIRECTION = (1UL << 0),
    FOC_FEEDBACK_EVIDENCE_ENCODER_INDEX = (1UL << 1),
    FOC_FEEDBACK_EVIDENCE_ENCODER_OFFSET = (1UL << 2),
    FOC_FEEDBACK_EVIDENCE_HALL_SEQUENCE = (1UL << 3),
    FOC_FEEDBACK_EVIDENCE_KNOWN_MASK =
        FOC_FEEDBACK_EVIDENCE_DIRECTION |
        FOC_FEEDBACK_EVIDENCE_ENCODER_INDEX |
        FOC_FEEDBACK_EVIDENCE_ENCODER_OFFSET |
        FOC_FEEDBACK_EVIDENCE_HALL_SEQUENCE,
};

typedef void (*foc_feedback_management_force_safe_fn)(void *context);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_feedback_management_force_safe_fn force_safe;
    void *context;
} foc_feedback_management_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t evidence_flags;
    int32_t direction;
    uint32_t direction_samples;
    uint32_t encoder_index_samples;
    float encoder_offset_rad;
    uint32_t encoder_offset_samples;
    uint32_t hall_sequence_packed;
    uint32_t hall_sequence_samples;
} foc_feedback_management_evidence_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t enabled;
    foc_feedback_management_state_t state;
    uint32_t owner_id;
    uint32_t feedback_token;
    uint32_t config_token;
    uint32_t started_at_ms;
    uint32_t last_activity_ms;
    uint32_t timeout_ms;
    uint32_t expected_config_revision;
    uint32_t shutdown_required;
    uint32_t begin_count;
    uint32_t cancel_count;
    uint32_t timeout_count;
    uint32_t failure_count;
    foc_feedback_management_result_t last_result;
    uint32_t last_detail;
    foc_feedback_status_snapshot_t feedback;
    foc_config_transaction_status_t config;
} foc_feedback_management_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t enabled;
    foc_feedback_management_state_t state;
    uint32_t owner_id;
    uint32_t feedback_token;
    uint32_t config_token;
    uint32_t started_at_ms;
    uint32_t last_activity_ms;
    uint32_t timeout_ms;
    uint32_t expected_config_revision;
    uint32_t shutdown_required;
    uint32_t begin_count;
    uint32_t cancel_count;
    uint32_t timeout_count;
    uint32_t failure_count;
    foc_feedback_management_result_t last_result;
    uint32_t last_detail;
    foc_feedback_context_storage_t *feedback_storage;
    foc_config_context_storage_t *config_storage;
    foc_feedback_management_ops_t ops;
} foc_feedback_management_t;

/* Initialization is fail-safe and leaves the feature disabled. */
foc_feedback_management_result_t foc_feedback_management_init(
    foc_feedback_management_t *management,
    foc_feedback_context_storage_t *feedback_storage,
    foc_config_context_storage_t *config_storage,
    const foc_feedback_management_ops_t *ops);

/* Explicit feature gate. Enabling is accepted only from a safe, clean state. */
foc_feedback_management_result_t foc_feedback_management_set_enabled(
    foc_feedback_management_t *management,
    uint32_t enabled,
    const foc_config_apply_guard_t *guard);

foc_feedback_management_result_t foc_feedback_management_get_status(
    foc_feedback_management_t *management,
    foc_feedback_management_status_t *status);

foc_feedback_management_result_t foc_feedback_management_begin(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    foc_feedback_mode_t feedback_mode,
    uint32_t now_ms,
    uint32_t timeout_ms,
    const foc_config_apply_guard_t *guard);

foc_feedback_management_result_t foc_feedback_management_submit_evidence(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_feedback_management_evidence_t *evidence);

/* Approve evidence, begin one config transaction, stage Axis/Calibration and validate it. */
foc_feedback_management_result_t foc_feedback_management_approve_and_stage(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard);

/* Finish only after an external parameter service has committed and read back the transaction. */
foc_feedback_management_result_t foc_feedback_management_finish_committed(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard);

foc_feedback_management_result_t foc_feedback_management_cancel(
    foc_feedback_management_t *management,
    uint32_t owner_id,
    const foc_config_apply_guard_t *guard);

/* Call from one RT-Thread management owner; unsigned elapsed time supports u32 wrap. */
foc_feedback_management_result_t foc_feedback_management_poll(
    foc_feedback_management_t *management,
    uint32_t now_ms,
    const foc_config_apply_guard_t *guard);

/* Clear a latched manager/Rust calibration failure only while both outputs and config are safe. */
foc_feedback_management_result_t foc_feedback_management_recover(
    foc_feedback_management_t *management,
    const foc_config_apply_guard_t *guard);

#endif /* FOC_FEEDBACK_MANAGEMENT_H */
