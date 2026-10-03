#ifndef FOC_INPUT_SERVICE_H
#define FOC_INPUT_SERVICE_H

/*
 * Management-task owner for normalized external input snapshots.
 *
 * Drivers and future P2.5 adapters publish here only after raw capture,
 * validation and SI normalization.  This object never runs in the FOC ISR,
 * never generates PWM and never arms the Axis.  Timeout is reported as an
 * expired bit; the command service/arbiter owns the resulting Release/Stop.
 */

#include <stdint.h>

#include "foc_config_bridge.h"
#include "foc_external_io.h"

#define FOC_INPUT_SERVICE_VERSION       (1UL)
#define FOC_INPUT_SAMPLE_VERSION        (1UL)

enum
{
    FOC_INPUT_SAMPLE_VALID_RAW = (1UL << 0),
    FOC_INPUT_SAMPLE_VALID_NORMALIZED = (1UL << 1),
    FOC_INPUT_SAMPLE_VALID_KNOWN_MASK =
        FOC_INPUT_SAMPLE_VALID_RAW | FOC_INPUT_SAMPLE_VALID_NORMALIZED,
};

enum
{
    FOC_INPUT_SAMPLE_QUALITY_CALIBRATED = (1UL << 0),
    FOC_INPUT_SAMPLE_QUALITY_DEGRADED = (1UL << 1),
    FOC_INPUT_SAMPLE_QUALITY_STALE = (1UL << 2),
    FOC_INPUT_SAMPLE_QUALITY_KNOWN_MASK =
        FOC_INPUT_SAMPLE_QUALITY_CALIBRATED |
        FOC_INPUT_SAMPLE_QUALITY_DEGRADED |
        FOC_INPUT_SAMPLE_QUALITY_STALE,
};

typedef uint32_t foc_input_service_result_t;
enum
{
    FOC_INPUT_SERVICE_OK = 0,
    FOC_INPUT_SERVICE_DISABLED = 1,
    FOC_INPUT_SERVICE_INVALID_ARGUMENT = 2,
    FOC_INPUT_SERVICE_UNSAFE_STATE = 3,
    FOC_INPUT_SERVICE_INVALID_CONFIG = 4,
    FOC_INPUT_SERVICE_WRONG_SOURCE = 5,
    FOC_INPUT_SERVICE_STALE_SEQUENCE = 6,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_input_mask_t input;
    uint32_t instance_id;
    uint32_t source_id;
    uint32_t sequence;
    uint32_t sampled_at_us;
    uint32_t valid_flags;
    uint32_t quality_flags;
    int32_t raw_value;
    float normalized_value;
} foc_input_sample_t;

typedef struct
{
    uint32_t source_id;
    uint32_t timeout_ms;
    uint32_t last_update_ms;
    uint32_t last_sequence;
    uint32_t sequence_valid;
    uint32_t sample_count;
    uint32_t timeout_count;
    uint32_t failure_count;
    uint32_t recovery_count;
    foc_input_sample_t latest;
} foc_input_service_slot_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_external_input_mask_t configured_input_mask;
    foc_external_input_mask_t healthy_input_mask;
    foc_external_input_mask_t expired_input_mask;
    foc_input_service_slot_t slots[FOC_EXTERNAL_IO_INPUT_COUNT];
} foc_input_service_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    foc_external_io_capabilities_t capabilities;
    foc_external_input_mask_t configured_input_mask;
    foc_external_input_mask_t healthy_input_mask;
    foc_external_input_mask_t expired_input_mask;
    foc_input_service_slot_t slots[FOC_EXTERNAL_IO_INPUT_COUNT];
} foc_input_service_t;

foc_input_service_result_t foc_input_service_init(
    foc_input_service_t *service,
    const foc_external_io_capabilities_t *capabilities);

foc_input_service_result_t foc_input_service_apply_config(
    foc_input_service_t *service,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard,
    foc_external_io_validation_t *validation);

foc_input_service_result_t foc_input_service_publish(
    foc_input_service_t *service,
    const foc_input_sample_t *sample,
    uint32_t now_ms);

/* Marks a configured input unhealthy immediately after raw-capture or
 * normalization failure. The command owner separately performs Release/Stop;
 * this function only updates observable input health. */
foc_input_service_result_t foc_input_service_mark_unhealthy(
    foc_input_service_t *service,
    foc_external_input_mask_t input,
    uint32_t quality_flags);

/* Returns OK even when one or more inputs expire; inspect expired_mask_out. */
foc_input_service_result_t foc_input_service_poll(
    foc_input_service_t *service,
    uint32_t now_ms,
    foc_external_input_mask_t *expired_mask_out);

foc_input_service_result_t foc_input_service_get_status(
    const foc_input_service_t *service,
    foc_input_service_status_t *status);

#endif /* FOC_INPUT_SERVICE_H */
