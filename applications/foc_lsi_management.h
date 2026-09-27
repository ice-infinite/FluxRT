#ifndef FOC_LSI_MANAGEMENT_H
#define FOC_LSI_MANAGEMENT_H

/*
 * FluxRT - non-realtime management facade for EXP-B3.
 *
 * This layer owns the frozen candidate sequence and the one-run-per-boot
 * application token.  It still never touches hardware: the platform performs
 * all physical preflight checks and is the only caller allowed to request the
 * shared runtime start.
 */

#include <stdint.h>

#include "foc_lsi_identification.h"

#define FOC_LSI_MANAGEMENT_VERSION (2UL)

typedef enum
{
    FOC_LSI_MANAGEMENT_OK = 0,
    FOC_LSI_MANAGEMENT_ALREADY_SAFE = 1,
    FOC_LSI_MANAGEMENT_INVALID = 2,
    FOC_LSI_MANAGEMENT_REFUSED = 3,
} foc_lsi_management_result_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_context_t context;
    uint32_t abort_command_count;
    uint32_t reset_command_count;
    uint32_t start_command_count;
    uint32_t start_consumed;
} foc_lsi_management_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t build_authorized;
    uint32_t identification_start_command_exposed;
    uint32_t pwm_adapter_present;
    uint32_t adc_adapter_present;
    uint32_t abort_command_count;
    uint32_t reset_command_count;
    uint32_t start_command_count;
    uint32_t start_consumed;
    foc_lsi_output_t output;
} foc_lsi_management_status_t;

/* Returns the read-only 0.2 A S5 candidate; it does not grant hardware use. */
void foc_lsi_management_default_config(foc_lsi_config_t *config);

/* Boot/reboot initialization always returns the manager to a safe IDLE state. */
uint32_t foc_lsi_management_init(foc_lsi_management_t *management);

/* Read-only snapshot of the S4.9 application/session coordinator. */
uint32_t foc_lsi_management_get_status(
    const foc_lsi_management_t *management,
    foc_lsi_management_status_t *status);

/* The confirmation may be consumed only once per manager initialisation. */
foc_lsi_management_result_t foc_lsi_management_request_start(
    foc_lsi_management_t *management,
    uint32_t confirmation,
    foc_lsi_management_status_t *status);

/* Realtime-safe pure state-machine step; no lock, HAL or allocation. */
foc_lsi_state_t foc_lsi_management_step(
    foc_lsi_management_t *management,
    const foc_lsi_input_t *input,
    foc_lsi_output_t *output);

/* Idempotent: active sequencing becomes ABORTED; safe terminal states stay safe. */
foc_lsi_management_result_t foc_lsi_management_abort(
    foc_lsi_management_t *management,
    foc_lsi_management_status_t *status);

/* Forces a safe abort first when needed, then returns to a clean IDLE state. */
foc_lsi_management_result_t foc_lsi_management_reset(
    foc_lsi_management_t *management,
    foc_lsi_management_status_t *status);

/*
 * Single shared target instance. Callers provide their own short interrupt
 * critical section when mixing Shell/thread and ADC ISR access.
 */
uint32_t foc_lsi_management_shared_init(void);
uint32_t foc_lsi_management_shared_get_status(
    foc_lsi_management_status_t *status);
uint32_t foc_lsi_management_shared_get_config(foc_lsi_config_t *config);
foc_lsi_management_result_t foc_lsi_management_shared_request_start(
    uint32_t confirmation,
    foc_lsi_management_status_t *status);
foc_lsi_state_t foc_lsi_management_shared_step(
    const foc_lsi_input_t *input,
    foc_lsi_output_t *output);
foc_lsi_management_result_t foc_lsi_management_shared_abort(
    foc_lsi_management_status_t *status);
foc_lsi_management_result_t foc_lsi_management_shared_reset(
    foc_lsi_management_status_t *status);
/* Automatic fail-close: does not increment the operator abort counter. */
void foc_lsi_management_shared_abort_isr(void);

#endif
