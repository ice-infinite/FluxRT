#ifndef FOC_LSI_CAPTURE_SERVICE_H
#define FOC_LSI_CAPTURE_SERVICE_H

/* Hardware-neutral singleton joining the ISR producer to the Shell consumer. */

#include <stdint.h>

#include "foc_lsi_raw_capture.h"
#include "foc_lsi_sample_contract.h"

#define FOC_LSI_CAPTURE_SERVICE_VERSION (1UL)

typedef enum
{
    FOC_LSI_CAPTURE_RECORD_IGNORED = 0,
    FOC_LSI_CAPTURE_RECORD_ACCEPTED = 1,
    FOC_LSI_CAPTURE_RECORD_COMPLETE = 2,
    FOC_LSI_CAPTURE_RECORD_CONTRACT_ERROR = 3,
    FOC_LSI_CAPTURE_RECORD_STORAGE_ERROR = 4,
} foc_lsi_capture_record_result_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_lsi_raw_capture_status_t capture;
    uint32_t last_contract_result;
    uint32_t contract_error_count;
    uint32_t storage_error_count;
} foc_lsi_capture_service_status_t;

void foc_lsi_capture_service_init(uint32_t sample_rate_hz,
                                  uint32_t adc_max_code,
                                  uint32_t pwm_period_ticks);
uint32_t foc_lsi_capture_service_arm(void);
uint32_t foc_lsi_capture_service_is_armed(void);
foc_lsi_capture_record_result_t foc_lsi_capture_service_record_isr(
    const foc_lsi_raw_sample_t *sample);
void foc_lsi_capture_service_stop(void);
uint32_t foc_lsi_capture_service_pop(foc_lsi_raw_sample_t *sample);
void foc_lsi_capture_service_get_status(
    foc_lsi_capture_service_status_t *status);

#endif /* FOC_LSI_CAPTURE_SERVICE_H */
