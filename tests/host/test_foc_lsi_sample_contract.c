/* Host fake-platform test for EXP-B3 raw snapshot validation. */

#include "foc_lsi_raw_capture.h"
#include "foc_lsi_sample_contract.h"

#include <assert.h>
#include <stdio.h>

static foc_lsi_sample_contract_t contract_12khz(void)
{
    foc_lsi_sample_contract_t contract = {0};
    contract.struct_size = sizeof(contract);
    contract.version = FOC_LSI_SAMPLE_CONTRACT_VERSION;
    contract.adc_max_code = 4095U;
    contract.expected_pwm_period_ticks = 7083U;
    return contract;
}

static foc_lsi_raw_sample_t valid_sample(uint32_t sequence,
                                         uint32_t control_tick)
{
    foc_lsi_raw_sample_t sample = {0};
    sample.sequence = sequence;
    sample.control_tick = control_tick;
    sample.current_u_raw = 2040U;
    sample.current_v_raw = 2050U;
    sample.bus_voltage_raw = 2300U;
    sample.compare_u = 3600U;
    sample.compare_v = 3500U;
    sample.compare_w = 3500U;
    sample.pwm_period_ticks = 7083U;
    sample.flags = FOC_LSI_RAW_FLAG_ADC_VALID |
                   FOC_LSI_RAW_FLAG_DRIVE_ACTIVE |
                   FOC_LSI_RAW_FLAG_PULSE_POSITIVE | 4U;
    return sample;
}

static void test_fake_platform_window_is_contiguous(void)
{
    foc_lsi_sample_contract_t contract = contract_12khz();
    foc_lsi_raw_capture_t capture;
    foc_lsi_raw_sample_t input;
    foc_lsi_raw_sample_t current;
    foc_lsi_raw_sample_t previous;
    const foc_lsi_raw_sample_t *previous_ptr = 0;
    uint32_t index;

    foc_lsi_raw_capture_init(&capture, 12000U);
    assert(foc_lsi_raw_capture_arm(&capture) == 1U);
    for (index = 0U; index < 192U; ++index)
    {
        input = valid_sample(index, 1000U + index);
        assert(foc_lsi_sample_validate(&contract, previous_ptr, &input) ==
               FOC_LSI_SAMPLE_OK);
        assert(foc_lsi_raw_capture_record_isr(&capture, &input) == 1U);
        previous = input;
        previous_ptr = &previous;
    }
    foc_lsi_raw_capture_stop(&capture);

    previous_ptr = 0;
    for (index = 0U; index < 192U; ++index)
    {
        assert(foc_lsi_raw_capture_pop(&capture, &current) == 1U);
        assert(foc_lsi_sample_validate(&contract, previous_ptr, &current) ==
               FOC_LSI_SAMPLE_OK);
        previous = current;
        previous_ptr = &previous;
    }
}

static void test_every_rejection_is_deterministic(void)
{
    foc_lsi_sample_contract_t contract = contract_12khz();
    foc_lsi_raw_sample_t previous = valid_sample(0U, 100U);
    foc_lsi_raw_sample_t sample = valid_sample(1U, 101U);

    assert(foc_lsi_sample_contract_is_valid(&contract) == 1U);
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_OK);

    sample.sequence = 2U;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_SEQUENCE_ERROR);
    sample = valid_sample(1U, 102U);
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_CONTROL_TICK_ERROR);
    sample = valid_sample(1U, 101U);
    sample.bus_voltage_raw = 4096U;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_ADC_RANGE_ERROR);
    sample = valid_sample(1U, 101U);
    sample.pwm_period_ticks = 7082U;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_PWM_PERIOD_ERROR);
    sample = valid_sample(1U, 101U);
    sample.compare_w = 7084U;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_COMPARE_RANGE_ERROR);
    sample = valid_sample(1U, 101U);
    sample.flags &= (uint16_t)~FOC_LSI_RAW_FLAG_ADC_VALID;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_ADC_INVALID);
    sample = valid_sample(1U, 101U);
    sample.flags |= FOC_LSI_RAW_FLAG_HARDWARE_FAULT;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_FAULT_FLAGGED);
    sample = valid_sample(1U, 101U);
    sample.flags |= FOC_LSI_RAW_FLAG_PULSE_NEGATIVE;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_POLARITY_ERROR);
    sample = valid_sample(1U, 101U);
    sample.flags &= (uint16_t)~FOC_LSI_RAW_FLAG_DRIVE_ACTIVE;
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_POLARITY_ERROR);

    contract.expected_pwm_period_ticks = 0U;
    assert(foc_lsi_sample_contract_is_valid(&contract) == 0U);
    assert(foc_lsi_sample_validate(&contract, &previous, &sample) ==
           FOC_LSI_SAMPLE_INVALID_ARGUMENT);
}

int main(void)
{
    test_fake_platform_window_is_contiguous();
    test_every_rejection_is_deterministic();
    puts("FOC LSI SAMPLE CONTRACT: PASS");
    return 0;
}
