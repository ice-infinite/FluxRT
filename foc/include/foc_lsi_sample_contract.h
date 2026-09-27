#ifndef FOC_LSI_SAMPLE_CONTRACT_H
#define FOC_LSI_SAMPLE_CONTRACT_H

/* Pure C validation gate between the STM32 snapshot and stored evidence. */

#include <stdint.h>

#include "foc_lsi_raw_sample.h"

#define FOC_LSI_SAMPLE_CONTRACT_VERSION (1UL)

typedef enum
{
    FOC_LSI_SAMPLE_OK = 0,
    FOC_LSI_SAMPLE_INVALID_ARGUMENT = 1,
    FOC_LSI_SAMPLE_SEQUENCE_ERROR = 2,
    FOC_LSI_SAMPLE_CONTROL_TICK_ERROR = 3,
    FOC_LSI_SAMPLE_ADC_RANGE_ERROR = 4,
    FOC_LSI_SAMPLE_PWM_PERIOD_ERROR = 5,
    FOC_LSI_SAMPLE_COMPARE_RANGE_ERROR = 6,
    FOC_LSI_SAMPLE_ADC_INVALID = 7,
    FOC_LSI_SAMPLE_FAULT_FLAGGED = 8,
    FOC_LSI_SAMPLE_POLARITY_ERROR = 9,
} foc_lsi_sample_result_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t adc_max_code;
    uint32_t expected_pwm_period_ticks;
} foc_lsi_sample_contract_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_lsi_sample_contract_t) == 16U,
               "LSI sample contract ABI changed");
#endif

uint32_t foc_lsi_sample_contract_is_valid(
    const foc_lsi_sample_contract_t *contract);

/*
 * `previous` is NULL only for sequence zero.  Every accepted later sample must
 * advance both sequence and control_tick by exactly one.  The adapter must
 * force safe output on any non-OK result; this function never owns hardware.
 */
foc_lsi_sample_result_t foc_lsi_sample_validate(
    const foc_lsi_sample_contract_t *contract,
    const foc_lsi_raw_sample_t *previous,
    const foc_lsi_raw_sample_t *sample);

#endif /* FOC_LSI_SAMPLE_CONTRACT_H */
