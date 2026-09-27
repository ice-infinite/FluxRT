/* Host-only contract tests for the fixed EXP-B3 raw capture window. */

#include "foc_lsi_raw_capture.h"

#include <assert.h>
#include <stdio.h>

static foc_lsi_raw_sample_t sample_for(uint32_t tick)
{
    foc_lsi_raw_sample_t sample = {0};
    sample.sequence = 0xFFFFFFFFUL;
    sample.control_tick = tick;
    sample.current_u_raw = (uint16_t)(1000U + tick);
    sample.current_v_raw = (uint16_t)(1100U + tick);
    sample.bus_voltage_raw = 2300U;
    sample.compare_u = 900U;
    sample.compare_v = 800U;
    sample.compare_w = 700U;
    sample.pwm_period_ticks = 7083U;
    sample.flags = FOC_LSI_RAW_FLAG_ADC_VALID |
                   FOC_LSI_RAW_FLAG_DRIVE_ACTIVE |
                   FOC_LSI_RAW_FLAG_PULSE_POSITIVE | 4U;
    return sample;
}

static void test_null_and_invalid_state_are_rejected(void)
{
    foc_lsi_raw_capture_t capture;
    foc_lsi_raw_capture_status_t status;
    foc_lsi_raw_sample_t sample = sample_for(1U);

    foc_lsi_raw_capture_init(&capture, 0U);
    assert(foc_lsi_raw_capture_arm(&capture) == 0U);
    assert(foc_lsi_raw_capture_record_isr(&capture, &sample) == 0U);
    assert(foc_lsi_raw_capture_pop(&capture, &sample) == 0U);
    foc_lsi_raw_capture_get_status(0, &status);
    assert(status.state == FOC_LSI_RAW_CAPTURE_IDLE);
    assert(status.capacity == FOC_LSI_RAW_CAPTURE_CAPACITY);
}

static void test_partial_window_is_atomic_and_ordered(void)
{
    foc_lsi_raw_capture_t capture;
    foc_lsi_raw_capture_status_t status;
    foc_lsi_raw_sample_t input;
    foc_lsi_raw_sample_t output;
    uint32_t index;

    foc_lsi_raw_capture_init(&capture, 12000U);
    assert(foc_lsi_raw_capture_arm(&capture) == 1U);
    assert(foc_lsi_raw_capture_arm(&capture) == 0U);
    for (index = 0U; index < 48U; ++index)
    {
        input = sample_for(100U + index);
        assert(foc_lsi_raw_capture_record_isr(&capture, &input) == 1U);
    }
    assert(foc_lsi_raw_capture_pop(&capture, &output) == 0U);
    foc_lsi_raw_capture_stop(&capture);
    foc_lsi_raw_capture_get_status(&capture, &status);
    assert(status.state == FOC_LSI_RAW_CAPTURE_COMPLETE);
    assert(status.sample_count == 48U);
    assert(status.unread_count == 48U);
    assert(status.overflow_count == 0U);
    for (index = 0U; index < 48U; ++index)
    {
        assert(foc_lsi_raw_capture_pop(&capture, &output) == 1U);
        assert(output.sequence == index);
        assert(output.control_tick == 100U + index);
        assert(output.current_u_raw == (uint16_t)(1100U + index));
        assert(output.pwm_period_ticks == 7083U);
    }
    assert(foc_lsi_raw_capture_pop(&capture, &output) == 0U);
}

static void test_full_window_latches_without_overwrite(void)
{
    foc_lsi_raw_capture_t capture;
    foc_lsi_raw_capture_status_t status;
    foc_lsi_raw_sample_t sample;
    foc_lsi_raw_sample_t output;
    uint32_t index;

    foc_lsi_raw_capture_init(&capture, 12000U);
    assert(foc_lsi_raw_capture_arm(&capture) == 1U);
    for (index = 0U; index < FOC_LSI_RAW_CAPTURE_CAPACITY; ++index)
    {
        sample = sample_for(index);
        assert(foc_lsi_raw_capture_record_isr(&capture, &sample) == 1U);
    }
    sample = sample_for(9999U);
    assert(foc_lsi_raw_capture_record_isr(&capture, &sample) == 0U);
    foc_lsi_raw_capture_get_status(&capture, &status);
    assert(status.state == FOC_LSI_RAW_CAPTURE_OVERFLOW);
    assert(status.sample_count == FOC_LSI_RAW_CAPTURE_CAPACITY);
    assert(status.overflow_count == 1U);
    assert(foc_lsi_raw_capture_pop(&capture, &output) == 1U);
    assert(output.sequence == 0U);
    assert(output.control_tick == 0U);

    /* Re-arm is explicit and resets both evidence and the overflow latch. */
    assert(foc_lsi_raw_capture_arm(&capture) == 1U);
    foc_lsi_raw_capture_get_status(&capture, &status);
    assert(status.state == FOC_LSI_RAW_CAPTURE_ARMED);
    assert(status.sample_count == 0U);
    assert(status.overflow_count == 0U);
}

int main(void)
{
    assert(sizeof(foc_lsi_raw_sample_t) == 24U);
    test_null_and_invalid_state_are_rejected();
    test_partial_window_is_atomic_and_ordered();
    test_full_window_latches_without_overwrite();
    puts("FOC LSI RAW CAPTURE: PASS");
    return 0;
}
