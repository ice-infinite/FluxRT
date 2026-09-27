#ifndef FOC_LSI_RAW_CAPTURE_H
#define FOC_LSI_RAW_CAPTURE_H

/* Fixed, allocation-free raw evidence window for EXP-B3 Ls(I). */

#include <stdint.h>

#include "foc_lsi_raw_sample.h"

#define FOC_LSI_RAW_CAPTURE_VERSION  (1UL)
#define FOC_LSI_RAW_CAPTURE_CAPACITY (256UL)

typedef enum
{
    FOC_LSI_RAW_CAPTURE_IDLE = 0,
    FOC_LSI_RAW_CAPTURE_ARMED = 1,
    FOC_LSI_RAW_CAPTURE_COMPLETE = 2,
    FOC_LSI_RAW_CAPTURE_OVERFLOW = 3,
} foc_lsi_raw_capture_state_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t state;
    uint32_t sample_rate_hz;
    uint32_t capacity;
    uint32_t sample_count;
    uint32_t unread_count;
    uint32_t overflow_count;
} foc_lsi_raw_capture_status_t;

typedef struct
{
    volatile uint32_t state;
    volatile uint32_t write_index;
    uint32_t read_index;
    uint32_t sample_rate_hz;
    volatile uint32_t overflow_count;
    foc_lsi_raw_sample_t samples[FOC_LSI_RAW_CAPTURE_CAPACITY];
} foc_lsi_raw_capture_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(FOC_LSI_RAW_CAPTURE_CAPACITY == 256U,
               "LSI capture capacity is part of the evidence contract");
#endif

void foc_lsi_raw_capture_init(foc_lsi_raw_capture_t *capture,
                              uint32_t sample_rate_hz);
uint32_t foc_lsi_raw_capture_arm(foc_lsi_raw_capture_t *capture);
uint32_t foc_lsi_raw_capture_record_isr(
    foc_lsi_raw_capture_t *capture,
    const foc_lsi_raw_sample_t *sample);
void foc_lsi_raw_capture_stop(foc_lsi_raw_capture_t *capture);
uint32_t foc_lsi_raw_capture_pop(foc_lsi_raw_capture_t *capture,
                                foc_lsi_raw_sample_t *sample);
void foc_lsi_raw_capture_get_status(
    const foc_lsi_raw_capture_t *capture,
    foc_lsi_raw_capture_status_t *status);

#endif /* FOC_LSI_RAW_CAPTURE_H */
