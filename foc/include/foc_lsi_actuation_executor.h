#ifndef FOC_LSI_ACTUATION_EXECUTOR_H
#define FOC_LSI_ACTUATION_EXECUTOR_H

/*
 * FluxRT - hardware-neutral C execution contract for EXP-B3 Ls(I).
 *
 * This layer joins one synchronised raw ADC/PWM snapshot to the Rust duty
 * planner.  It owns raw-to-SI conversion, duty-to-CCR quantisation and the
 * one-control-tick preload ledger, but it never touches a register and never
 * enables a gate.  The MCU platform remains the sole hardware owner.
 */

#include <stdint.h>

#include "foc_lsi_raw_sample.h"
#include "foc_rust_bridge.h"

#define FOC_LSI_EXECUTOR_CONFIG_VERSION  (1UL)
#define FOC_LSI_EXECUTOR_COMMAND_VERSION (1UL)
#define FOC_LSI_EXECUTOR_OUTPUT_VERSION  (1UL)
#define FOC_LSI_EXECUTOR_VERSION         (1UL)

typedef uint32_t foc_lsi_executor_action_t;
enum
{
    /* Caller must keep/put the power stage in its hardware-safe state. */
    FOC_LSI_EXECUTOR_ACTION_SAFE = 0,
    /* Valid compare values for the caller's next preload write. */
    FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD = 1,
};

typedef uint32_t foc_lsi_executor_result_t;
enum
{
    FOC_LSI_EXECUTOR_RESULT_SAFE_REQUESTED = 0,
    FOC_LSI_EXECUTOR_RESULT_PRELOAD_READY = 1,
    FOC_LSI_EXECUTOR_RESULT_INVALID_ARGUMENT = 2,
    FOC_LSI_EXECUTOR_RESULT_CAPTURE_NOT_READY = 3,
    FOC_LSI_EXECUTOR_RESULT_CAPTURE_COMPLETE = 4,
    FOC_LSI_EXECUTOR_RESULT_CAPTURE_ERROR = 5,
    FOC_LSI_EXECUTOR_RESULT_FAULT = 6,
    FOC_LSI_EXECUTOR_RESULT_RAW_SAMPLE_INVALID = 7,
    FOC_LSI_EXECUTOR_RESULT_LEDGER_MISMATCH = 8,
    FOC_LSI_EXECUTOR_RESULT_PLANNER_REJECTED = 9,
};

/* Board calibration and timer facts. No algorithm parameter belongs here. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sample_rate_hz;
    uint32_t pwm_period_ticks;
    uint32_t adc_max_code;
    uint32_t actuation_delay_control_ticks;
    uint16_t current_u_offset_raw;
    uint16_t current_v_offset_raw;
    uint16_t reserved0;
    uint16_t reserved1;
    float current_counts_per_amp;
    float bus_volts_per_count;
    float minimum_duty;
    float maximum_duty;
} foc_lsi_executor_config_t;

/* One state-machine request plus capture/hardware gates for the same tick. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_drive_request_abi_t drive_request;
    uint32_t force_safe_output;
    uint32_t capture_ready;
    uint32_t capture_full;
    uint32_t capture_error;
    uint32_t hardware_fault;
    uint32_t software_trip;
    float requested_bias_current_a;
    float requested_perturbation_voltage_v;
} foc_lsi_executor_command_t;

/* Persistent state is explicit so Host tests can inspect every ledger edge. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t pending_ledger_valid;
    uint32_t pending_source_control_tick;
    uint32_t pending_expected_active_control_tick;
    foc_lsi_drive_request_abi_t pending_drive_request;
    uint16_t pending_compare_u;
    uint16_t pending_compare_v;
    uint16_t pending_compare_w;
    uint16_t reserved;
    foc_lsi_executor_config_t platform;
    foc_lsi_actuation_config_t actuation;
} foc_lsi_executor_t;

/* Result is a decision record, not proof that hardware applied the compares. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_executor_action_t action;
    foc_lsi_executor_result_t result;
    foc_status_t planner_status;
    uint32_t ledger_checked;
    uint32_t ledger_matched;
    uint32_t source_control_tick;
    uint32_t expected_active_control_tick;
    uint16_t compare_u;
    uint16_t compare_v;
    uint16_t compare_w;
    uint16_t pwm_period_ticks;
    float phase_u_current_a;
    float bus_voltage_v;
    foc_lsi_actuation_output_t plan;
} foc_lsi_executor_output_t;

/* Identification-only; other build profiles return FOC_STATUS_DISABLED. */
foc_status_t foc_lsi_executor_init(
    foc_lsi_executor_t *executor,
    const foc_lsi_executor_config_t *platform,
    const foc_lsi_actuation_config_t *actuation);

/*
 * Evaluates exactly one control tick. A WRITE_PRELOAD result is still only a
 * validated request; the platform must re-check hardware before writing CCRs.
 */
foc_status_t foc_lsi_executor_step(
    foc_lsi_executor_t *executor,
    const foc_lsi_executor_command_t *command,
    const foc_lsi_raw_sample_t *raw,
    foc_lsi_executor_output_t *output);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_lsi_executor_config_t) == 48U,
               "LSI executor config layout changed");
_Static_assert(sizeof(foc_lsi_executor_command_t) == 44U,
               "LSI executor command layout changed");
#endif

#endif /* FOC_LSI_ACTUATION_EXECUTOR_H */
