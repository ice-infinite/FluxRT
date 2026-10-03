#include "foc_motion_probe.h"

#include <assert.h>
#include <stdio.h>

static foc_motion_probe_register_snapshot_t safe_snapshot(uint32_t epoch)
{
    foc_motion_probe_register_snapshot_t value = {0};
    value.power_safety_state = FOC_POWER_SAFETY_DISABLED;
    value.fault_epoch = epoch;
    value.gate_is_low = 1U;
    return value;
}

static void test_safe_run_and_exact_completion(void)
{
    foc_motion_probe_t probe;
    foc_motion_probe_status_t status;
    /* Epoch zero is the valid clean-boot baseline, not an uninitialized value. */
    foc_motion_probe_register_snapshot_t before = safe_snapshot(0U);
    foc_motion_probe_register_snapshot_t after = before;

    assert(foc_motion_probe_init(&probe) == FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_configure(&probe, 0U) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_start(&probe, 2U, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_begin_tick(&probe, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_combined(
               &probe, FOC_STATUS_OK, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_commit(&probe, &before, &after) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_begin_tick(&probe, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_combined(
               &probe, FOC_STATUS_OK, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_commit(&probe, &before, &after) ==
           FOC_MOTION_PROBE_RESULT_COMPLETE);
    assert(foc_motion_probe_get_status(&probe, &status) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(status.state == FOC_MOTION_PROBE_COMPLETE);
    assert(status.executed_ticks == 2U);
    assert(status.no_power_commit_count == 2U);
    assert(status.rejected_tick_count == 0U);
}

static void test_every_output_gate_is_fail_closed(void)
{
    uint32_t field;
    for (field = 0U; field < 7U; ++field)
    {
        foc_motion_probe_t probe;
        foc_motion_probe_register_snapshot_t unsafe = safe_snapshot(3U);

        switch (field)
        {
        case 0U: unsafe.control_armed = 1U; break;
        case 1U: unsafe.power_safety_state = FOC_POWER_SAFETY_ARMED; break;
        case 2U: unsafe.break_latched = 1U; break;
        case 3U: unsafe.driver_faulted = 1U; break;
        case 4U: unsafe.gate_is_low = 0U; break;
        case 5U: unsafe.moe_enabled = 1U; break;
        default: unsafe.phase_channels_enabled = 1U; break;
        }
        assert(foc_motion_probe_init(&probe) == FOC_MOTION_PROBE_RESULT_OK);
        assert(foc_motion_probe_configure(&probe, 3U) ==
               FOC_MOTION_PROBE_RESULT_OK);
        assert(foc_motion_probe_start(&probe, 1U, &unsafe) ==
               FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE);
        assert(probe.state == FOC_MOTION_PROBE_FAILED);
    }
}

static void test_fault_epoch_and_control_failures_latch(void)
{
    foc_motion_probe_t probe;
    foc_motion_probe_register_snapshot_t before = safe_snapshot(11U);
    foc_motion_probe_register_snapshot_t after = before;

    assert(foc_motion_probe_init(&probe) == FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_configure(&probe, 11U) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_start(&probe, 1U, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    after.fault_epoch = 12U;
    assert(foc_motion_probe_complete_combined(
               &probe, FOC_STATUS_OK, &after) ==
           FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED);
    assert(probe.state == FOC_MOTION_PROBE_FAILED);
    assert(probe.no_power_commit_count == 0U);

    assert(foc_motion_probe_init(&probe) == FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_configure(&probe, 11U) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_start(&probe, 1U, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_combined(
               &probe, FOC_STATUS_HARDWARE_FAULT, &before) ==
           FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE);
    assert(probe.state == FOC_MOTION_PROBE_FAILED);
}

static void test_commit_postcheck_and_injection_are_one_shot(void)
{
    foc_motion_probe_t probe;
    foc_motion_probe_register_snapshot_t before = safe_snapshot(21U);
    foc_motion_probe_register_snapshot_t after = before;

    assert(foc_motion_probe_init(&probe) == FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_set_injection(
               &probe, FOC_MOTION_PROBE_INJECT_BEFORE_COMBINED) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_take_injection(
               &probe, FOC_MOTION_PROBE_INJECT_BEFORE_COMBINED) == 1U);
    assert(foc_motion_probe_take_injection(
               &probe, FOC_MOTION_PROBE_INJECT_BEFORE_COMBINED) == 0U);
    assert(probe.injection_count == 1U);

    assert(foc_motion_probe_configure(&probe, 21U) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_start(&probe, 1U, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_begin_tick(&probe, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    assert(foc_motion_probe_complete_combined(
               &probe, FOC_STATUS_OK, &before) ==
           FOC_MOTION_PROBE_RESULT_OK);
    after.moe_enabled = 1U;
    assert(foc_motion_probe_complete_commit(&probe, &before, &after) ==
           FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE);
    assert(probe.state == FOC_MOTION_PROBE_FAILED);
    assert(probe.executed_ticks == 0U);
}

int main(void)
{
    assert(sizeof(foc_motion_probe_register_snapshot_t) == 32U);
    assert(sizeof(foc_motion_probe_status_t) == 52U);
    test_safe_run_and_exact_completion();
    test_every_output_gate_is_fail_closed();
    test_fault_epoch_and_control_failures_latch();
    test_commit_postcheck_and_injection_are_one_shot();
    puts("foc motion no-power probe guard tests passed");
    return 0;
}
