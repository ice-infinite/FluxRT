#include "foc_build_profile.h"
#include "foc_lsi_capture_service.h"

#include <string.h>

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
static foc_lsi_raw_capture_t g_foc_lsi_capture;
static foc_lsi_sample_contract_t g_foc_lsi_contract;
static foc_lsi_raw_sample_t g_foc_lsi_previous_sample;
static uint32_t g_foc_lsi_has_previous;
static volatile uint32_t g_foc_lsi_last_contract_result;
static volatile uint32_t g_foc_lsi_contract_error_count;
static volatile uint32_t g_foc_lsi_storage_error_count;
#endif

void foc_lsi_capture_service_init(uint32_t sample_rate_hz,
                                  uint32_t adc_max_code,
                                  uint32_t pwm_period_ticks)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_raw_capture_init(&g_foc_lsi_capture, sample_rate_hz);
    g_foc_lsi_contract.struct_size = sizeof(g_foc_lsi_contract);
    g_foc_lsi_contract.version = FOC_LSI_SAMPLE_CONTRACT_VERSION;
    g_foc_lsi_contract.adc_max_code = adc_max_code;
    g_foc_lsi_contract.expected_pwm_period_ticks = pwm_period_ticks;
    memset(&g_foc_lsi_previous_sample, 0, sizeof(g_foc_lsi_previous_sample));
    g_foc_lsi_has_previous = 0U;
    g_foc_lsi_last_contract_result = FOC_LSI_SAMPLE_OK;
    g_foc_lsi_contract_error_count = 0U;
    g_foc_lsi_storage_error_count = 0U;
#else
    (void)sample_rate_hz;
    (void)adc_max_code;
    (void)pwm_period_ticks;
#endif
}

uint32_t foc_lsi_capture_service_arm(void)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if (g_foc_lsi_capture.state == FOC_LSI_RAW_CAPTURE_ARMED)
    {
        return 0U;
    }
    g_foc_lsi_has_previous = 0U;
    g_foc_lsi_last_contract_result = FOC_LSI_SAMPLE_OK;
    g_foc_lsi_contract_error_count = 0U;
    g_foc_lsi_storage_error_count = 0U;
    return foc_lsi_raw_capture_arm(&g_foc_lsi_capture);
#else
    return 0U;
#endif
}

uint32_t foc_lsi_capture_service_is_armed(void)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    return (g_foc_lsi_capture.state == FOC_LSI_RAW_CAPTURE_ARMED) ? 1U : 0U;
#else
    return 0U;
#endif
}

foc_lsi_capture_record_result_t foc_lsi_capture_service_record_isr(
    const foc_lsi_raw_sample_t *sample)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_raw_sample_t accepted_sample;
    foc_lsi_sample_result_t contract_result;

    if ((sample == 0) ||
        (g_foc_lsi_capture.state != FOC_LSI_RAW_CAPTURE_ARMED))
    {
        return FOC_LSI_CAPTURE_RECORD_IGNORED;
    }
    accepted_sample = *sample;
    accepted_sample.sequence = g_foc_lsi_capture.write_index;
    contract_result = foc_lsi_sample_validate(
        &g_foc_lsi_contract,
        (g_foc_lsi_has_previous != 0U) ? &g_foc_lsi_previous_sample : 0,
        &accepted_sample);
    g_foc_lsi_last_contract_result = (uint32_t)contract_result;
    if (contract_result != FOC_LSI_SAMPLE_OK)
    {
        ++g_foc_lsi_contract_error_count;
        foc_lsi_raw_capture_stop(&g_foc_lsi_capture);
        return FOC_LSI_CAPTURE_RECORD_CONTRACT_ERROR;
    }
    if (foc_lsi_raw_capture_record_isr(&g_foc_lsi_capture,
                                       &accepted_sample) == 0U)
    {
        ++g_foc_lsi_storage_error_count;
        return FOC_LSI_CAPTURE_RECORD_STORAGE_ERROR;
    }
    g_foc_lsi_previous_sample = accepted_sample;
    g_foc_lsi_previous_sample.sequence =
        g_foc_lsi_capture.write_index - 1U;
    g_foc_lsi_has_previous = 1U;
    if (g_foc_lsi_capture.write_index >= FOC_LSI_RAW_CAPTURE_CAPACITY)
    {
        foc_lsi_raw_capture_stop(&g_foc_lsi_capture);
        return FOC_LSI_CAPTURE_RECORD_COMPLETE;
    }
    return FOC_LSI_CAPTURE_RECORD_ACCEPTED;
#else
    (void)sample;
    return FOC_LSI_CAPTURE_RECORD_IGNORED;
#endif
}

void foc_lsi_capture_service_stop(void)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_raw_capture_stop(&g_foc_lsi_capture);
#endif
}

uint32_t foc_lsi_capture_service_pop(foc_lsi_raw_sample_t *sample)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    return foc_lsi_raw_capture_pop(&g_foc_lsi_capture, sample);
#else
    (void)sample;
    return 0U;
#endif
}

void foc_lsi_capture_service_get_status(
    foc_lsi_capture_service_status_t *status)
{
    if (status == 0)
    {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_LSI_CAPTURE_SERVICE_VERSION;
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_raw_capture_get_status(&g_foc_lsi_capture, &status->capture);
    status->last_contract_result = g_foc_lsi_last_contract_result;
    status->contract_error_count = g_foc_lsi_contract_error_count;
    status->storage_error_count = g_foc_lsi_storage_error_count;
#else
    status->capture.struct_size = sizeof(status->capture);
    status->capture.version = FOC_LSI_RAW_CAPTURE_VERSION;
    status->capture.state = FOC_LSI_RAW_CAPTURE_IDLE;
    status->capture.capacity = FOC_LSI_RAW_CAPTURE_CAPACITY;
#endif
}
