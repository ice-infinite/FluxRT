#ifndef FOC_LSI_IDENTIFICATION_H
#define FOC_LSI_IDENTIFICATION_H

/*
 * FluxRT - application Ls(I) sequencing contract (pure C, no hardware).
 *
 * This module owns only the deterministic experiment sequence and its safety
 * invariants. It never writes PWM/ADC registers and never arms a power stage.
 * The application must explicitly authorize one run, while the platform is
 * the sole owner that may translate a drive request into hardware output.
 */

#include <stdint.h>

#include "foc_build_profile.h"

#define FOC_LSI_IDENTIFICATION_VERSION (1UL)
#define FOC_LSI_INPUT_VERSION          (1UL)
#define FOC_LSI_OUTPUT_VERSION         (1UL)
#define FOC_LSI_SAMPLE_RATE_HZ         (12000UL)
#define FOC_LSI_START_CONFIRMATION     (0x4C534931UL) /* "LSI1" */

typedef enum
{
    FOC_LSI_STATE_IDLE = 0,
    FOC_LSI_STATE_PREFLIGHT = 1,
    FOC_LSI_STATE_OFFSET_CAL = 2,
    FOC_LSI_STATE_BIAS_SETTLE = 3,
    FOC_LSI_STATE_PULSE_POSITIVE = 4,
    FOC_LSI_STATE_PULSE_NEGATIVE = 5,
    FOC_LSI_STATE_COOLDOWN = 6,
    FOC_LSI_STATE_COMPLETE = 7,
    FOC_LSI_STATE_ABORTED = 8,
} foc_lsi_state_t;

typedef enum
{
    FOC_LSI_ABORT_NONE = 0,
    FOC_LSI_ABORT_REQUESTED = 1,
    FOC_LSI_ABORT_UNAUTHORIZED_BUILD = 2,
    FOC_LSI_ABORT_POWER_STAGE_NOT_IDLE = 3,
    FOC_LSI_ABORT_MOTOR_NOT_STOPPED = 4,
    FOC_LSI_ABORT_HARDWARE_FAULT = 5,
    FOC_LSI_ABORT_SOFTWARE_TRIP = 6,
    FOC_LSI_ABORT_CURRENT_LIMIT = 7,
    FOC_LSI_ABORT_BUS_VOLTAGE = 8,
    FOC_LSI_ABORT_TOTAL_TIMEOUT = 9,
    FOC_LSI_ABORT_ACTIVE_TIMEOUT = 10,
    FOC_LSI_ABORT_INVALID_INPUT = 11,
} foc_lsi_abort_reason_t;

/* A request is not proof that the platform applied any PWM output. */
typedef enum
{
    FOC_LSI_DRIVE_OFF = 0,
    FOC_LSI_DRIVE_BIAS = 1,
    FOC_LSI_DRIVE_PULSE_POSITIVE = 2,
    FOC_LSI_DRIVE_PULSE_NEGATIVE = 3,
} foc_lsi_drive_request_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sample_rate_hz;
    uint32_t offset_sample_count;
    uint32_t bias_settle_ticks;
    uint32_t pulse_ticks_per_polarity;
    uint32_t pulse_pair_count;
    uint32_t cooldown_zero_ticks;
    uint32_t max_active_ticks;
    uint32_t total_timeout_ticks;
    float bias_current_a;
    float perturbation_voltage_v;
    float current_trip_a;
    float cooldown_current_threshold_a;
    float bus_voltage_min_v;
    float bus_voltage_max_v;
} foc_lsi_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t identification_build_authorized;
    uint32_t power_stage_idle;
    uint32_t motor_stopped;
    uint32_t hardware_fault;
    uint32_t software_trip;
    uint32_t abort_requested;
    uint32_t offset_sample_valid;
    float bus_voltage_v;
    float abs_phase_current_a;
} foc_lsi_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_state_t state;
    foc_lsi_abort_reason_t abort_reason;
    foc_lsi_drive_request_t drive_request;
    uint32_t force_safe_output;
    uint32_t capture_offset_sample;
    uint32_t capture_raw_sample;
    uint32_t pulse_pair_index;
    float requested_bias_current_a;
    float requested_perturbation_voltage_v;
} foc_lsi_output_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_config_t config;
    foc_lsi_state_t state;
    foc_lsi_abort_reason_t abort_reason;
    uint32_t config_valid;
    uint32_t elapsed_ticks;
    uint32_t active_ticks;
    uint32_t state_ticks;
    uint32_t offset_samples;
    uint32_t pulse_pair_index;
    uint32_t cooldown_zero_count;
} foc_lsi_context_t;

/* Returns 1 only for a configuration inside the frozen EXP-B3 envelope. */
uint32_t foc_lsi_config_is_valid(const foc_lsi_config_t *config);

/* Returns 1 only in the dedicated Identification firmware profile. */
uint32_t foc_lsi_build_is_authorized(void);

/* Invalid configuration leaves a deterministic IDLE, non-startable context. */
uint32_t foc_lsi_init(foc_lsi_context_t *context,
                      const foc_lsi_config_t *config);

/* The exact confirmation token is required; a rejected request stays IDLE. */
uint32_t foc_lsi_request_start(foc_lsi_context_t *context,
                               uint32_t confirmation);

/* Advances one 12 kHz interval and returns the resulting state. */
foc_lsi_state_t foc_lsi_step(foc_lsi_context_t *context,
                             const foc_lsi_input_t *input,
                             foc_lsi_output_t *output);

/* COMPLETE/ABORTED may be reset to IDLE; active states cannot be reset. */
uint32_t foc_lsi_reset(foc_lsi_context_t *context,
                       foc_lsi_output_t *output);

/* Reads the current requested action without advancing the sequence. */
void foc_lsi_get_output(const foc_lsi_context_t *context,
                        foc_lsi_output_t *output);

#endif
