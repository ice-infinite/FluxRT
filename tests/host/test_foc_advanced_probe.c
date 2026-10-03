#include "foc_advanced_probe.h"

#include <assert.h>
#include <stdio.h>

static foc_advanced_probe_register_snapshot_t safe_snapshot(uint32_t epoch)
{
    foc_advanced_probe_register_snapshot_t value = {0};
    value.power_safety_state = FOC_POWER_SAFETY_DISABLED;
    value.fault_epoch = epoch;
    value.gate_is_low = 1U;
    return value;
}

static void test_exact_completion(void)
{
    foc_advanced_probe_t probe;
    foc_advanced_probe_status_t status;
    foc_advanced_probe_register_snapshot_t before = safe_snapshot(4U);
    foc_advanced_probe_register_snapshot_t after = before;

    assert(foc_advanced_probe_init(&probe) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(probe.decision_signature == 2166136261UL);
    assert(foc_advanced_probe_start(&probe, 2U, 4U, &before) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_begin_tick(&probe, &before) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_complete_control(
               &probe, FOC_STATUS_OK, &before) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_complete_commit(&probe, &before, &after) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_begin_tick(&probe, &before) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_complete_control(
               &probe, FOC_STATUS_OK, &before) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_complete_commit(&probe, &before, &after) ==
           FOC_ADVANCED_PROBE_RESULT_COMPLETE);
    assert(foc_advanced_probe_get_status(&probe, &status) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(status.state == FOC_ADVANCED_PROBE_COMPLETE);
    assert(status.executed_ticks == 2U);
    assert(status.no_power_commit_count == 2U);
    assert(status.rejected_tick_count == 0U);
}

static void test_safety_and_fault_epoch_fail_closed(void)
{
    uint32_t field;

    for (field = 0U; field < 7U; ++field)
    {
        foc_advanced_probe_t probe;
        foc_advanced_probe_register_snapshot_t unsafe = safe_snapshot(9U);

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
        assert(foc_advanced_probe_init(&probe) ==
               FOC_ADVANCED_PROBE_RESULT_OK);
        assert(foc_advanced_probe_start(&probe, 1U, 9U, &unsafe) ==
               FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE);
        assert(probe.state == FOC_ADVANCED_PROBE_FAILED);
    }

    {
        foc_advanced_probe_t probe;
        foc_advanced_probe_register_snapshot_t before = safe_snapshot(11U);
        foc_advanced_probe_register_snapshot_t after = before;

        assert(foc_advanced_probe_init(&probe) ==
               FOC_ADVANCED_PROBE_RESULT_OK);
        assert(foc_advanced_probe_start(&probe, 1U, 11U, &before) ==
               FOC_ADVANCED_PROBE_RESULT_OK);
        assert(foc_advanced_probe_complete_control(
                   &probe, FOC_STATUS_OK, &before) ==
               FOC_ADVANCED_PROBE_RESULT_OK);
        after.fault_epoch = 12U;
        assert(foc_advanced_probe_complete_commit(
                   &probe, &before, &after) ==
               FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED);
        assert(probe.state == FOC_ADVANCED_PROBE_FAILED);
        assert(probe.executed_ticks == 0U);
    }
}

static void test_control_failure_latches(void)
{
    foc_advanced_probe_t probe;
    foc_advanced_probe_register_snapshot_t snapshot = safe_snapshot(15U);

    assert(foc_advanced_probe_init(&probe) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_start(&probe, 1U, 15U, &snapshot) ==
           FOC_ADVANCED_PROBE_RESULT_OK);
    assert(foc_advanced_probe_complete_control(
               &probe, FOC_STATUS_HARDWARE_FAULT, &snapshot) ==
           FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE);
    assert(probe.state == FOC_ADVANCED_PROBE_FAILED);
}

int main(void)
{
    assert(sizeof(foc_advanced_probe_register_snapshot_t) == 32U);
    assert(sizeof(foc_advanced_probe_status_t) == 52U);
    test_exact_completion();
    test_safety_and_fault_epoch_fail_closed();
    test_control_failure_latches();
    puts("foc advanced no-power probe guard tests passed");
    return 0;
}
