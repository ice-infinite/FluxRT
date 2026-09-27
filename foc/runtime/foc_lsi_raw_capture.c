#include "foc_lsi_raw_capture.h"

#include <string.h>

void foc_lsi_raw_capture_init(foc_lsi_raw_capture_t *capture,
                              uint32_t sample_rate_hz)
{
    if (capture == 0)
    {
        return;
    }
    capture->state = FOC_LSI_RAW_CAPTURE_IDLE;
    capture->write_index = 0U;
    capture->read_index = 0U;
    capture->sample_rate_hz = sample_rate_hz;
    capture->overflow_count = 0U;
}

uint32_t foc_lsi_raw_capture_arm(foc_lsi_raw_capture_t *capture)
{
    if ((capture == 0) || (capture->sample_rate_hz == 0U) ||
        (capture->state == FOC_LSI_RAW_CAPTURE_ARMED))
    {
        return 0U;
    }
    capture->write_index = 0U;
    capture->read_index = 0U;
    capture->overflow_count = 0U;
    capture->state = FOC_LSI_RAW_CAPTURE_ARMED;
    return 1U;
}

uint32_t foc_lsi_raw_capture_record_isr(
    foc_lsi_raw_capture_t *capture,
    const foc_lsi_raw_sample_t *sample)
{
    uint32_t index;

    if ((capture == 0) || (sample == 0) ||
        (capture->state != FOC_LSI_RAW_CAPTURE_ARMED))
    {
        return 0U;
    }
    index = capture->write_index;
    if (index >= FOC_LSI_RAW_CAPTURE_CAPACITY)
    {
        ++capture->overflow_count;
        capture->state = FOC_LSI_RAW_CAPTURE_OVERFLOW;
        return 0U;
    }

    memcpy(&capture->samples[index], sample, sizeof(*sample));
    capture->samples[index].sequence = index;
    capture->write_index = index + 1U;
    return 1U;
}

void foc_lsi_raw_capture_stop(foc_lsi_raw_capture_t *capture)
{
    if ((capture != 0) &&
        (capture->state == FOC_LSI_RAW_CAPTURE_ARMED))
    {
        capture->state = FOC_LSI_RAW_CAPTURE_COMPLETE;
    }
}

uint32_t foc_lsi_raw_capture_pop(foc_lsi_raw_capture_t *capture,
                                foc_lsi_raw_sample_t *sample)
{
    uint32_t index;

    if ((capture == 0) || (sample == 0) ||
        (capture->state == FOC_LSI_RAW_CAPTURE_ARMED) ||
        (capture->state == FOC_LSI_RAW_CAPTURE_IDLE))
    {
        return 0U;
    }
    index = capture->read_index;
    if (index >= capture->write_index)
    {
        return 0U;
    }
    *sample = capture->samples[index];
    capture->read_index = index + 1U;
    return 1U;
}

void foc_lsi_raw_capture_get_status(
    const foc_lsi_raw_capture_t *capture,
    foc_lsi_raw_capture_status_t *status)
{
    if (status == 0)
    {
        return;
    }
    memset(status, 0, sizeof(*status));
    status->struct_size = sizeof(*status);
    status->version = FOC_LSI_RAW_CAPTURE_VERSION;
    status->state = FOC_LSI_RAW_CAPTURE_IDLE;
    status->capacity = FOC_LSI_RAW_CAPTURE_CAPACITY;
    if (capture == 0)
    {
        return;
    }
    status->state = capture->state;
    status->sample_rate_hz = capture->sample_rate_hz;
    status->sample_count = capture->write_index;
    status->unread_count = capture->write_index - capture->read_index;
    status->overflow_count = capture->overflow_count;
}
