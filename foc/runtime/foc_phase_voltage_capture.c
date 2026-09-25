#include "foc_phase_voltage_capture.h"

void foc_phase_voltage_capture_init(foc_phase_voltage_capture_t *capture,
                                    uint32_t sample_rate_hz)
{
    if (capture == 0)
    {
        return;
    }
    capture->state = FOC_PHASE_VOLTAGE_CAPTURE_IDLE;
    capture->divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_DISABLED;
    capture->write_index = 0U;
    capture->read_index = 0U;
    capture->sample_rate_hz = sample_rate_hz;
}

uint32_t foc_phase_voltage_capture_arm(
    foc_phase_voltage_capture_t *capture,
    foc_phase_voltage_divider_mode_t divider_mode)
{
    if ((capture == 0) || (capture->sample_rate_hz == 0U) ||
        (capture->state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED) ||
        ((divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) &&
         (divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_ENABLED)))
    {
        return 0U;
    }
    capture->write_index = 0U;
    capture->read_index = 0U;
    capture->divider_mode = (uint32_t)divider_mode;
    capture->state = FOC_PHASE_VOLTAGE_CAPTURE_ARMED;
    return 1U;
}

uint32_t foc_phase_voltage_capture_record_isr(
    foc_phase_voltage_capture_t *capture,
    const foc_phase_voltage_sample_t *sample)
{
    uint32_t index;

    if ((capture == 0) || (sample == 0) ||
        (capture->state != FOC_PHASE_VOLTAGE_CAPTURE_ARMED))
    {
        return 0U;
    }
    index = capture->write_index;
    if (index >= FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY)
    {
        capture->state = FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE;
        return 0U;
    }
    capture->samples[index] = *sample;
    capture->samples[index].sequence = index;
    ++index;
    capture->write_index = index;
    if (index == FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY)
    {
        capture->state = FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE;
    }
    return 1U;
}

void foc_phase_voltage_capture_stop(foc_phase_voltage_capture_t *capture)
{
    if ((capture != 0) &&
        (capture->state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED))
    {
        capture->state = FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE;
    }
}

uint32_t foc_phase_voltage_capture_pop(foc_phase_voltage_capture_t *capture,
                                      foc_phase_voltage_sample_t *sample)
{
    uint32_t index;

    if ((capture == 0) || (sample == 0) ||
        (capture->state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED))
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

void foc_phase_voltage_capture_get_status(
    const foc_phase_voltage_capture_t *capture,
    foc_phase_voltage_capture_status_t *status)
{
    if (status == 0)
    {
        return;
    }
    status->struct_size = sizeof(*status);
    status->version = FOC_PHASE_VOLTAGE_CAPTURE_VERSION;
    status->state = FOC_PHASE_VOLTAGE_CAPTURE_IDLE;
    status->divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_DISABLED;
    status->sample_rate_hz = 0U;
    status->capacity = FOC_PHASE_VOLTAGE_CAPTURE_CAPACITY;
    status->sample_count = 0U;
    status->unread_count = 0U;
    if (capture == 0)
    {
        return;
    }
    status->state = capture->state;
    status->divider_mode = capture->divider_mode;
    status->sample_rate_hz = capture->sample_rate_hz;
    status->sample_count = capture->write_index;
    status->unread_count = capture->write_index - capture->read_index;
}
