#ifndef FOC_LSI_PRELOAD_SINK_INTERNAL_H
#define FOC_LSI_PRELOAD_SINK_INTERNAL_H

/*
 * FluxRT - STM32G431-private EXP-B3 preload capability.
 *
 * This header is intentionally kept beside the platform implementation and is
 * not exported through foc/include.  It authorises the S4.6 one-shot validation
 * preload and, since S4.8, bounded sequential permits for an active caller.
 * It cannot enable CCER, MOE or a gate and it is not a motor-start interface;
 * S4.9's ADC ISR may consume permits only inside the separately gated,
 * one-run-per-boot platform session.
 */

#include <stdint.h>

#include "foc_lsi_actuation_executor.h"

#define FOC_LSI_PRELOAD_SESSION_VERSION (2UL)
#define FOC_LSI_PRELOAD_PERMIT_VERSION  (2UL)
#define FOC_LSI_PRELOAD_CONFIRMATION    (0x4C534956UL) /* "LSIV" */
#define FOC_LSI_ACTIVE_CONFIRMATION     (0x4C534941UL) /* "LSIA" */
#define FOC_LSI_ACTIVE_WRITE_LIMIT_MAX  (600UL)
#define FOC_LSI_ACTIVE_TOTAL_TICKS_MAX  (6000UL)

#define FOC_LSI_PRELOAD_MODE_VALIDATION (1UL)
#define FOC_LSI_PRELOAD_MODE_ACTIVE     (2UL)

typedef uint32_t foc_lsi_preload_result_t;
enum
{
    FOC_LSI_PRELOAD_RESULT_OK = 0,
    FOC_LSI_PRELOAD_RESULT_INVALID_ARGUMENT = 1,
    FOC_LSI_PRELOAD_RESULT_SESSION_CLOSED = 2,
    FOC_LSI_PRELOAD_RESULT_PERMIT_REJECTED = 3,
    FOC_LSI_PRELOAD_RESULT_OUTPUT_REJECTED = 4,
    FOC_LSI_PRELOAD_RESULT_HARDWARE_UNSAFE = 5,
    FOC_LSI_PRELOAD_RESULT_REGISTER_MISMATCH = 6,
    FOC_LSI_PRELOAD_RESULT_BUDGET_EXHAUSTED = 7,
    FOC_LSI_PRELOAD_RESULT_SEQUENCE_ERROR = 8,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t validation_open;
    uint32_t generation;
    uint32_t permit_issued;
    uint32_t permit_consumed;
    uint32_t pwm_period_ticks;
    uint32_t actuation_delay_control_ticks;
    uint16_t minimum_compare;
    uint16_t maximum_compare;
    uint16_t reserved0;
    uint16_t reserved1;
    uint32_t active_open;
    uint32_t active_write_limit;
    uint32_t active_write_count;
    uint32_t active_total_tick_limit;
    uint32_t active_start_control_tick;
    uint32_t next_permit_sequence;
    uint32_t last_consumed_sequence;
    uint32_t last_source_control_tick;
    uint32_t has_last_source_control_tick;
} foc_lsi_preload_session_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t generation;
    uint32_t source_control_tick;
    uint32_t expected_active_control_tick;
    uint32_t authorization_tag;
    uint32_t session_mode;
    uint32_t permit_sequence;
    uint16_t compare_u;
    uint16_t compare_v;
    uint16_t compare_w;
    uint16_t pwm_period_ticks;
} foc_lsi_preload_permit_t;

/* Host tests bind these pointers to fake words; the target binds TIM1 registers. */
typedef struct
{
    volatile uint32_t *compare_u;
    volatile uint32_t *compare_v;
    volatile uint32_t *compare_w;
    volatile uint32_t *period;
} foc_lsi_preload_registers_t;

/* Facts sampled by the STM32 wrapper immediately before the write. */
typedef struct
{
    uint32_t timer_clock_enabled;
    uint32_t timer_configured;
    uint32_t compare_preload_enabled;
    uint32_t gate_enabled;
    uint32_t main_output_enabled;
    uint32_t channel_outputs_enabled;
    uint32_t hardware_fault;
    uint32_t software_trip;
} foc_lsi_preload_safety_t;

void foc_lsi_preload_session_init(
    foc_lsi_preload_session_t *session,
    uint32_t pwm_period_ticks,
    uint32_t actuation_delay_control_ticks,
    uint16_t minimum_compare,
    uint16_t maximum_compare);

/* Opens exactly one safe, outputs-disabled validation write. */
foc_lsi_preload_result_t foc_lsi_preload_session_open_validation(
    foc_lsi_preload_session_t *session,
    uint32_t confirmation);

/*
 * Opens a bounded multi-write session, but does not enable any output.  The
 * first write is accepted only while gate/MOE/CCER are all off; every later
 * write requires all three hardware enable facts to be on.  The target has no
 * caller for this mode until the separate ISR/start gate is completed.
 */
foc_lsi_preload_result_t foc_lsi_preload_session_open_active(
    foc_lsi_preload_session_t *session,
    uint32_t confirmation,
    uint32_t active_write_limit,
    uint32_t total_tick_limit,
    uint32_t start_control_tick);

foc_lsi_preload_result_t foc_lsi_preload_session_issue(
    foc_lsi_preload_session_t *session,
    const foc_lsi_executor_output_t *output,
    foc_lsi_preload_permit_t *permit);

/*
 * Consumes the permit and writes all three compare registers, then reads them
 * back.  Success closes the validation session.  Failure never opens outputs.
 */
foc_lsi_preload_result_t foc_lsi_preload_sink_apply(
    foc_lsi_preload_session_t *session,
    const foc_lsi_preload_permit_t *permit,
    const foc_lsi_executor_output_t *output,
    const foc_lsi_preload_registers_t *registers,
    const foc_lsi_preload_safety_t *safety);

void foc_lsi_preload_session_abort(foc_lsi_preload_session_t *session);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_lsi_preload_session_t) == 80U,
               "LSI preload session layout changed");
_Static_assert(sizeof(foc_lsi_preload_permit_t) == 40U,
               "LSI preload permit layout changed");
#endif

#endif /* FOC_LSI_PRELOAD_SINK_INTERNAL_H */
