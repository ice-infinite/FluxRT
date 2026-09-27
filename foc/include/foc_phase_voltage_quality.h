#ifndef FOC_PHASE_VOLTAGE_QUALITY_H
#define FOC_PHASE_VOLTAGE_QUALITY_H

/*
 * FluxRT - deterministic phase-voltage quality and fallback contract.
 *
 * This module is pure C.  It owns neither ADC conversion nor the observer.  A
 * platform adapter supplies one calibrated sample plus the matching
 * CommandModel reference; the gate decides whether the requested measured or
 * hybrid policy is eligible.  Hybrid falls back to CommandModel; pure Measured
 * reports UNAVAILABLE so its caller must fail closed rather than silently
 * changing policy.
 */

#include <stdint.h>

#define FOC_PHASE_VOLTAGE_QUALITY_VERSION      (1UL)
#define FOC_PHASE_VOLTAGE_QUALITY_REASON_COUNT (9UL)

typedef enum
{
    FOC_PHASE_VOLTAGE_QUALITY_UNCONFIGURED = 0,
    FOC_PHASE_VOLTAGE_QUALITY_UNCALIBRATED = 1,
    FOC_PHASE_VOLTAGE_QUALITY_VALID = 2,
    FOC_PHASE_VOLTAGE_QUALITY_STALE = 3,
    FOC_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT = 4,
    FOC_PHASE_VOLTAGE_QUALITY_LOW_SATURATION = 5,
    FOC_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION = 6,
    FOC_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT = 7,
    FOC_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE = 8,
} foc_phase_voltage_quality_state_t;

/* Bit positions are stable: status.reason_counts[index] uses the same index. */
typedef enum
{
    FOC_PHASE_VOLTAGE_REASON_UNCONFIGURED = (1UL << 0),
    FOC_PHASE_VOLTAGE_REASON_UNCALIBRATED = (1UL << 1),
    FOC_PHASE_VOLTAGE_REASON_INVALID_SAMPLE = (1UL << 2),
    FOC_PHASE_VOLTAGE_REASON_STALE = (1UL << 3),
    FOC_PHASE_VOLTAGE_REASON_LOW_SATURATION = (1UL << 4),
    FOC_PHASE_VOLTAGE_REASON_HIGH_SATURATION = (1UL << 5),
    FOC_PHASE_VOLTAGE_REASON_OPEN_SUSPECT = (1UL << 6),
    FOC_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT = (1UL << 7),
    FOC_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS = (1UL << 8),
} foc_phase_voltage_quality_reason_t;

/* Requested policy.  These are modes, not the selected source reported below. */
typedef enum
{
    FOC_PHASE_VOLTAGE_REQUEST_COMMAND_MODEL = 0,
    FOC_PHASE_VOLTAGE_REQUEST_MEASURED = 1,
    FOC_PHASE_VOLTAGE_REQUEST_HYBRID = 2,
} foc_phase_voltage_request_t;

/* Actual selection.  Hybrid is never an actual source: when healthy it selects
 * measured voltage, otherwise it falls back to CommandModel.  Pure Measured
 * mode is fail-closed and reports UNAVAILABLE instead of silently changing
 * policy. */
typedef enum
{
    FOC_PHASE_VOLTAGE_SELECTION_COMMAND_MODEL = 0,
    FOC_PHASE_VOLTAGE_SELECTION_MEASURED = 1,
    FOC_PHASE_VOLTAGE_SELECTION_UNAVAILABLE = 2,
} foc_phase_voltage_selection_t;

enum
{
    FOC_PHASE_VOLTAGE_PHASE_U = (1UL << 0),
    FOC_PHASE_VOLTAGE_PHASE_V = (1UL << 1),
    FOC_PHASE_VOLTAGE_PHASE_W = (1UL << 2),
    FOC_PHASE_VOLTAGE_PHASE_ALL = FOC_PHASE_VOLTAGE_PHASE_U |
                                  FOC_PHASE_VOLTAGE_PHASE_V |
                                  FOC_PHASE_VOLTAGE_PHASE_W,
};

enum
{
    FOC_PHASE_VOLTAGE_SAMPLE_CONFIGURED = (1UL << 0),
    FOC_PHASE_VOLTAGE_SAMPLE_CALIBRATED = (1UL << 1),
    FOC_PHASE_VOLTAGE_SAMPLE_CONVERSION_VALID = (1UL << 2),
    FOC_PHASE_VOLTAGE_SAMPLE_COMMAND_REFERENCE_VALID = (1UL << 3),
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t adc_max_code;
    uint32_t low_saturation_enter_code;
    uint32_t low_saturation_release_code;
    uint32_t high_saturation_release_code;
    uint32_t high_saturation_enter_code;
    uint32_t max_sample_age_ticks;
    uint32_t open_stuck_enter_delta_codes;
    uint32_t open_release_delta_codes;
    uint32_t open_confirm_samples;
    uint32_t inconsistency_release_mv;
    uint32_t inconsistency_enter_mv;
    uint32_t inconsistency_confirm_samples;
    uint32_t recovery_confirm_samples;
} foc_phase_voltage_quality_config_t;

/*
 * expected_change_mask is supplied from the PWM/command side.  A phase is
 * suspected open only when it was expected to move but its raw code remains
 * within open_stuck_enter_delta_codes for open_confirm_samples consecutive
 * observations.  Zero means that no open-circuit evidence is available on
 * this tick; it does not by itself fail the sample.
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sequence;
    uint32_t age_ticks;
    uint32_t flags;
    uint32_t expected_change_mask;
    uint16_t phase_u_raw;
    uint16_t phase_v_raw;
    uint16_t phase_w_raw;
    uint16_t reserved;
    uint32_t phase_u_mv;
    uint32_t phase_v_mv;
    uint32_t phase_w_mv;
    uint32_t command_u_mv;
    uint32_t command_v_mv;
    uint32_t command_w_mv;
} foc_phase_voltage_quality_sample_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t reason_mask;
    uint32_t requested_mode;
    uint32_t selected_source;
    uint32_t measured_eligible;
    uint32_t fallback_active;
    uint32_t unavailable_active;
    uint32_t affected_phase_mask;
    uint32_t consecutive_healthy_samples;
    uint32_t sample_sequence;
    uint32_t fallback_sample_count;
    uint32_t fallback_event_count;
    uint32_t unavailable_sample_count;
    uint32_t unavailable_event_count;
    uint32_t recovery_count;
    uint32_t evaluated_sample_count;
} foc_phase_voltage_quality_result_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t previous_sequence;
    uint32_t previous_raw[3];
    uint32_t previous_valid;
    uint32_t low_saturation_phase_mask;
    uint32_t high_saturation_phase_mask;
    uint32_t open_suspect_phase_mask;
    uint32_t open_streak[3];
    uint32_t inconsistency_streak;
    uint32_t inconsistency_latched;
    uint32_t healthy_streak;
    uint32_t last_state;
    uint32_t last_reason_mask;
    uint32_t last_selected_source;
    uint32_t fallback_active;
    uint32_t unavailable_active;
    uint32_t evaluated_sample_count;
    uint32_t valid_sample_count;
    uint32_t fallback_sample_count;
    uint32_t fallback_event_count;
    uint32_t unavailable_sample_count;
    uint32_t unavailable_event_count;
    uint32_t recovery_count;
    uint32_t reason_counts[FOC_PHASE_VOLTAGE_QUALITY_REASON_COUNT];
} foc_phase_voltage_quality_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t reason_mask;
    uint32_t selected_source;
    uint32_t fallback_active;
    uint32_t unavailable_active;
    uint32_t low_saturation_phase_mask;
    uint32_t high_saturation_phase_mask;
    uint32_t open_suspect_phase_mask;
    uint32_t consecutive_healthy_samples;
    uint32_t evaluated_sample_count;
    uint32_t valid_sample_count;
    uint32_t fallback_sample_count;
    uint32_t fallback_event_count;
    uint32_t unavailable_sample_count;
    uint32_t unavailable_event_count;
    uint32_t recovery_count;
    uint32_t reason_counts[FOC_PHASE_VOLTAGE_QUALITY_REASON_COUNT];
} foc_phase_voltage_quality_status_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_phase_voltage_quality_config_t) == 60U,
               "phase-voltage quality config ABI changed");
_Static_assert(sizeof(foc_phase_voltage_quality_sample_t) == 56U,
               "phase-voltage quality sample ABI changed");
_Static_assert(sizeof(foc_phase_voltage_quality_result_t) == 72U,
               "phase-voltage quality result ABI changed");
#endif

uint32_t foc_phase_voltage_quality_config_is_valid(
    const foc_phase_voltage_quality_config_t *config);

void foc_phase_voltage_quality_init(foc_phase_voltage_quality_t *quality);

/*
 * Returns zero only when quality/result cannot be written.  Invalid config,
 * sample, or request is still a processed fail-closed result.  See
 * foc_phase_voltage_selection_t for the policy-specific outcome.
 */
uint32_t foc_phase_voltage_quality_evaluate(
    foc_phase_voltage_quality_t *quality,
    const foc_phase_voltage_quality_config_t *config,
    const foc_phase_voltage_quality_sample_t *sample,
    foc_phase_voltage_request_t requested_mode,
    foc_phase_voltage_quality_result_t *result);

void foc_phase_voltage_quality_get_status(
    const foc_phase_voltage_quality_t *quality,
    foc_phase_voltage_quality_status_t *status);

#endif /* FOC_PHASE_VOLTAGE_QUALITY_H */
