#include "foc_lsi_capture_service.h"

#include <assert.h>
#include <stdio.h>

static foc_lsi_raw_sample_t valid_sample(uint32_t tick)
{
    foc_lsi_raw_sample_t sample = {0};

    sample.control_tick = tick;
    sample.current_u_raw = 1950U;
    sample.current_v_raw = 1930U;
    sample.bus_voltage_raw = 2U;
    sample.compare_u = 0U;
    sample.compare_v = 0U;
    sample.compare_w = 0U;
    sample.pwm_period_ticks = 7083U;
    sample.flags = FOC_LSI_RAW_FLAG_ADC_VALID;
    return sample;
}

static void test_fixed_window_completes_without_overflow(void)
{
    foc_lsi_capture_service_status_t status;
    foc_lsi_raw_sample_t sample;
    uint32_t index;

    foc_lsi_capture_service_init(12000U, 4095U, 7083U);
    assert(foc_lsi_capture_service_arm() == 1U);
    for (index = 0U; index < FOC_LSI_RAW_CAPTURE_CAPACITY; ++index)
    {
        sample = valid_sample(1000U + index);
        assert(foc_lsi_capture_service_record_isr(&sample) ==
               ((index + 1U == FOC_LSI_RAW_CAPTURE_CAPACITY) ?
                    FOC_LSI_CAPTURE_RECORD_COMPLETE :
                    FOC_LSI_CAPTURE_RECORD_ACCEPTED));
        if (index == 0U)
        {
            /* A rejected re-arm must not reset the previous-sample contract. */
            assert(foc_lsi_capture_service_arm() == 0U);
        }
    }
    assert(foc_lsi_capture_service_is_armed() == 0U);
    foc_lsi_capture_service_get_status(&status);
    assert(status.capture.state == FOC_LSI_RAW_CAPTURE_COMPLETE);
    assert(status.capture.sample_count == FOC_LSI_RAW_CAPTURE_CAPACITY);
    assert(status.capture.overflow_count == 0U);
    assert(status.contract_error_count == 0U);
    for (index = 0U; index < FOC_LSI_RAW_CAPTURE_CAPACITY; ++index)
    {
        assert(foc_lsi_capture_service_pop(&sample) == 1U);
        assert(sample.sequence == index);
        assert(sample.control_tick == 1000U + index);
        assert(sample.pwm_period_ticks == 7083U);
    }
}

static void test_contract_failure_stops_capture(void)
{
    foc_lsi_capture_service_status_t status;
    foc_lsi_raw_sample_t sample;

    foc_lsi_capture_service_init(12000U, 4095U, 7083U);
    assert(foc_lsi_capture_service_arm() == 1U);
    sample = valid_sample(4U);
    assert(foc_lsi_capture_service_record_isr(&sample) ==
           FOC_LSI_CAPTURE_RECORD_ACCEPTED);
    sample = valid_sample(6U);
    assert(foc_lsi_capture_service_record_isr(&sample) ==
           FOC_LSI_CAPTURE_RECORD_CONTRACT_ERROR);
    foc_lsi_capture_service_get_status(&status);
    assert(status.capture.state == FOC_LSI_RAW_CAPTURE_COMPLETE);
    assert(status.last_contract_result == FOC_LSI_SAMPLE_CONTROL_TICK_ERROR);
    assert(status.contract_error_count == 1U);
}

int main(void)
{
    test_fixed_window_completes_without_overflow();
    test_contract_failure_stops_capture();
    puts("FOC LSI CAPTURE SERVICE: PASS");
    return 0;
}
