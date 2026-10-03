#ifndef FOC_FEEDBACK_BRIDGE_H
#define FOC_FEEDBACK_BRIDGE_H

/*
 * FluxRT feedback routing and calibration management sub-ABI V1.
 *
 * This is not the 12 kHz realtime ABI. One management task owns the opaque
 * context and serializes all calls. Platform code converts hardware-specific
 * observer/ABZ/Hall data into foc_feedback_source_sample_t before ingestion.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_config_bridge.h"

#define FOC_FEEDBACK_ABI_VERSION      (0x00010000UL)
#define FOC_FEEDBACK_CONTEXT_CAPACITY (512UL)

typedef uint32_t foc_feedback_status_t;
enum
{
    FOC_FEEDBACK_STATUS_OK = 0,
    FOC_FEEDBACK_STATUS_INVALID_ARGUMENT = 1,
    FOC_FEEDBACK_STATUS_NOT_INITIALIZED = 2,
    FOC_FEEDBACK_STATUS_INVALID_CONFIG = 3,
    FOC_FEEDBACK_STATUS_INVALID_SAMPLE = 4,
    FOC_FEEDBACK_STATUS_UNSUPPORTED_MODE = 5,
    FOC_FEEDBACK_STATUS_STALE_SEQUENCE = 6,
    FOC_FEEDBACK_STATUS_STALE_CYCLE = 7,
    FOC_FEEDBACK_STATUS_UNSAFE_STATE = 8,
    FOC_FEEDBACK_STATUS_BUSY = 9,
    FOC_FEEDBACK_STATUS_NO_SESSION = 10,
    FOC_FEEDBACK_STATUS_STALE_TOKEN = 11,
    FOC_FEEDBACK_STATUS_INVALID_STATE = 12,
    FOC_FEEDBACK_STATUS_INVALID_EVIDENCE = 13,
    FOC_FEEDBACK_STATUS_INCOMPLETE_EVIDENCE = 14,
    FOC_FEEDBACK_STATUS_CONFIG_TRANSACTION_FAILED = 15,
};

typedef uint32_t foc_feedback_route_state_t;
enum
{
    FOC_FEEDBACK_ROUTE_ACQUIRING = 0,
    FOC_FEEDBACK_ROUTE_PRIMARY = 1,
    FOC_FEEDBACK_ROUTE_FALLBACK = 2,
    FOC_FEEDBACK_ROUTE_LOST = 3,
};

typedef uint32_t foc_feedback_calibration_state_t;
enum
{
    FOC_FEEDBACK_CALIBRATION_IDLE = 0,
    FOC_FEEDBACK_CALIBRATION_COLLECTING = 1,
    FOC_FEEDBACK_CALIBRATION_READY_FOR_REVIEW = 2,
    FOC_FEEDBACK_CALIBRATION_APPROVED = 3,
    FOC_FEEDBACK_CALIBRATION_FAILED = 4,
};

enum
{
    FOC_FEEDBACK_CALIBRATION_STEP_DIRECTION = (1UL << 0),
    FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_INDEX = (1UL << 1),
    FOC_FEEDBACK_CALIBRATION_STEP_ENCODER_OFFSET = (1UL << 2),
    FOC_FEEDBACK_CALIBRATION_STEP_HALL_SEQUENCE = (1UL << 3),
};

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t axis_id;
    foc_feedback_mode_t primary_mode;
    foc_feedback_mode_t backup_mode;
    uint32_t fallback_enabled;
    uint32_t maximum_age_us;
    uint32_t acquire_good_samples;
    uint32_t loss_bad_samples;
    uint32_t recovery_good_samples;
} foc_feedback_router_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t axis_id;
    foc_feedback_mode_t mode;
    uint32_t sequence;
    uint32_t sampled_at_ms;
    uint32_t sampled_at_us;
    uint32_t valid_flags;
    uint32_t quality_flags;
    int32_t direction;
    uint32_t pole_pair_revision;
    float mechanical_position_rad;
    float multi_turn_position_rad;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float electrical_velocity_rad_s;
} foc_feedback_source_sample_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t minimum_direction_samples;
    uint32_t minimum_index_samples;
    uint32_t minimum_offset_samples;
    uint32_t minimum_hall_samples;
    uint32_t encoder_counts_per_revolution;
} foc_feedback_calibration_policy_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    foc_feedback_route_state_t route_state;
    uint32_t reserved;
    foc_product_feedback_snapshot_t snapshot;
} foc_feedback_route_decision_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    foc_feedback_mode_t feedback_mode;
    int32_t axis_direction;
    uint32_t calibration_flags_to_set;
    float encoder_offset_rad;
    uint32_t encoder_counts_per_revolution;
    uint32_t hall_sequence_packed;
    uint32_t evidence_steps;
    uint32_t reserved;
} foc_feedback_calibration_update_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    foc_feedback_route_state_t route_state;
    foc_feedback_calibration_state_t calibration_state;
    uint32_t calibration_token;
    uint32_t completed_steps;
    foc_feedback_status_t last_result;
    uint32_t last_detail;
    uint32_t reserved0;
    uint32_t reserved1;
} foc_feedback_status_snapshot_t;

typedef union
{
    uint64_t alignment;
    uint8_t bytes[FOC_FEEDBACK_CONTEXT_CAPACITY];
} foc_feedback_context_storage_t;

uint32_t foc_rust_feedback_abi_version(void);
uint32_t foc_rust_feedback_context_required_size(void);
uint32_t foc_rust_feedback_context_required_align(void);
foc_feedback_status_t foc_rust_feedback_init(
    foc_feedback_context_storage_t *storage,
    const foc_feedback_router_config_t *router_config,
    const foc_feedback_calibration_policy_t *calibration_policy);
foc_feedback_status_t foc_rust_feedback_get_status(
    foc_feedback_context_storage_t *storage,
    foc_feedback_status_snapshot_t *output);
foc_feedback_status_t foc_rust_feedback_ingest(
    foc_feedback_context_storage_t *storage,
    const foc_feedback_source_sample_t *sample);
foc_feedback_status_t foc_rust_feedback_route(
    foc_feedback_context_storage_t *storage,
    uint32_t cycle_sequence,
    uint32_t now_us,
    foc_feedback_route_decision_t *output);
foc_feedback_status_t foc_rust_feedback_calibration_begin(
    foc_feedback_context_storage_t *storage,
    foc_feedback_mode_t feedback_mode,
    const foc_config_apply_guard_t *guard,
    uint32_t *token_out);
foc_feedback_status_t foc_rust_feedback_calibration_record_direction(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    int32_t direction,
    uint32_t sample_count);
foc_feedback_status_t foc_rust_feedback_calibration_record_encoder_index(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    uint32_t sample_count);
foc_feedback_status_t foc_rust_feedback_calibration_record_encoder_offset(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    float offset_rad,
    uint32_t sample_count);
foc_feedback_status_t foc_rust_feedback_calibration_record_hall_sequence(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    uint32_t sequence_packed,
    uint32_t sample_count);
foc_feedback_status_t foc_rust_feedback_calibration_approve(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    const foc_config_apply_guard_t *guard,
    foc_feedback_calibration_update_t *output);
foc_feedback_status_t foc_rust_feedback_stage_config_update(
    foc_feedback_context_storage_t *storage,
    uint32_t calibration_token,
    foc_config_context_storage_t *config_storage,
    uint32_t config_token);
foc_feedback_status_t foc_rust_feedback_calibration_cancel(
    foc_feedback_context_storage_t *storage,
    uint32_t token);
foc_feedback_status_t foc_rust_feedback_calibration_reset_failed(
    foc_feedback_context_storage_t *storage,
    const foc_config_apply_guard_t *guard);
foc_feedback_status_t foc_rust_feedback_calibration_finish_applied(
    foc_feedback_context_storage_t *storage,
    uint32_t token,
    const foc_config_apply_guard_t *guard);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_feedback_router_config_t) == 40U,
               "feedback router config ABI drifted");
_Static_assert(sizeof(foc_feedback_source_sample_t) == 64U,
               "feedback sample ABI drifted");
_Static_assert(sizeof(foc_feedback_calibration_policy_t) == 28U,
               "feedback calibration policy ABI drifted");
_Static_assert(sizeof(foc_feedback_route_decision_t) == 84U,
               "feedback route decision ABI drifted");
_Static_assert(sizeof(foc_feedback_calibration_update_t) == 40U,
               "feedback calibration update ABI drifted");
_Static_assert(sizeof(foc_feedback_status_snapshot_t) == 40U,
               "feedback status ABI drifted");
_Static_assert(sizeof(foc_feedback_context_storage_t) == FOC_FEEDBACK_CONTEXT_CAPACITY,
               "feedback context capacity drifted");
#endif

#endif /* FOC_FEEDBACK_BRIDGE_H */
