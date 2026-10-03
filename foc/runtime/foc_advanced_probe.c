#include "foc_advanced_probe.h"

#include <string.h>

static uint32_t foc_advanced_probe_valid(const foc_advanced_probe_t *probe)
{
    return (probe != 0) &&
           (probe->struct_size == (uint32_t)sizeof(*probe)) &&
           (probe->version == FOC_ADVANCED_PROBE_VERSION) &&
           (probe->state <= FOC_ADVANCED_PROBE_FAILED);
}

static foc_advanced_probe_result_t foc_advanced_probe_record(
    foc_advanced_probe_t *probe,
    foc_advanced_probe_result_t result)
{
    probe->last_result = result;
    return result;
}

uint32_t foc_advanced_probe_registers_are_safe_off(
    const foc_advanced_probe_register_snapshot_t *registers)
{
    return (registers != 0) &&
           (registers->control_armed == 0U) &&
           (registers->power_safety_state == FOC_POWER_SAFETY_DISABLED) &&
           (registers->break_latched == 0U) &&
           (registers->driver_faulted == 0U) &&
           (registers->gate_is_low != 0U) &&
           (registers->moe_enabled == 0U) &&
           (registers->phase_channels_enabled == 0U);
}

foc_advanced_probe_result_t foc_advanced_probe_init(
    foc_advanced_probe_t *probe)
{
    if (probe == 0)
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    (void)memset(probe, 0, sizeof(*probe));
    probe->struct_size = (uint32_t)sizeof(*probe);
    probe->version = FOC_ADVANCED_PROBE_VERSION;
    probe->state = FOC_ADVANCED_PROBE_IDLE;
    probe->last_control_status = FOC_STATUS_DISABLED;
    probe->decision_signature = 2166136261UL;
    return FOC_ADVANCED_PROBE_RESULT_OK;
}

foc_advanced_probe_result_t foc_advanced_probe_start(
    foc_advanced_probe_t *probe,
    uint32_t requested_ticks,
    uint32_t expected_fault_epoch,
    const foc_advanced_probe_register_snapshot_t *registers)
{
    if ((foc_advanced_probe_valid(probe) == 0U) || (registers == 0) ||
        (requested_ticks < FOC_ADVANCED_PROBE_MIN_TICKS) ||
        (requested_ticks > FOC_ADVANCED_PROBE_MAX_TICKS))
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if (probe->state != FOC_ADVANCED_PROBE_IDLE)
    {
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_INVALID_STATE);
    }
    if (foc_advanced_probe_registers_are_safe_off(registers) == 0U)
    {
        probe->state = FOC_ADVANCED_PROBE_FAILED;
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE);
    }
    if (registers->fault_epoch != expected_fault_epoch)
    {
        probe->state = FOC_ADVANCED_PROBE_FAILED;
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED);
    }
    probe->requested_ticks = requested_ticks;
    probe->expected_fault_epoch = expected_fault_epoch;
    probe->observed_fault_epoch = registers->fault_epoch;
    probe->state = FOC_ADVANCED_PROBE_RUNNING;
    return foc_advanced_probe_record(probe, FOC_ADVANCED_PROBE_RESULT_OK);
}

foc_advanced_probe_result_t foc_advanced_probe_begin_tick(
    foc_advanced_probe_t *probe,
    const foc_advanced_probe_register_snapshot_t *registers)
{
    if ((foc_advanced_probe_valid(probe) == 0U) || (registers == 0))
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if (probe->state != FOC_ADVANCED_PROBE_RUNNING)
    {
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_INVALID_STATE);
    }
    if (foc_advanced_probe_registers_are_safe_off(registers) == 0U)
    {
        ++probe->rejected_tick_count;
        return foc_advanced_probe_fail(
            probe, FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE,
            registers->fault_epoch);
    }
    if (registers->fault_epoch != probe->expected_fault_epoch)
    {
        ++probe->rejected_tick_count;
        return foc_advanced_probe_fail(
            probe, FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            registers->fault_epoch);
    }
    return foc_advanced_probe_record(probe, FOC_ADVANCED_PROBE_RESULT_OK);
}

foc_advanced_probe_result_t foc_advanced_probe_complete_control(
    foc_advanced_probe_t *probe,
    foc_status_t control_status,
    const foc_advanced_probe_register_snapshot_t *registers)
{
    if ((foc_advanced_probe_valid(probe) == 0U) || (registers == 0))
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    probe->last_control_status = control_status;
    if ((probe->state != FOC_ADVANCED_PROBE_RUNNING) ||
        (control_status != FOC_STATUS_OK))
    {
        ++probe->rejected_tick_count;
        return foc_advanced_probe_fail(
            probe, FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
            registers->fault_epoch);
    }
    return foc_advanced_probe_begin_tick(probe, registers);
}

foc_advanced_probe_result_t foc_advanced_probe_complete_commit(
    foc_advanced_probe_t *probe,
    const foc_advanced_probe_register_snapshot_t *before,
    const foc_advanced_probe_register_snapshot_t *after)
{
    if ((foc_advanced_probe_valid(probe) == 0U) ||
        (before == 0) || (after == 0))
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if ((probe->state != FOC_ADVANCED_PROBE_RUNNING) ||
        (foc_advanced_probe_registers_are_safe_off(before) == 0U) ||
        (foc_advanced_probe_registers_are_safe_off(after) == 0U))
    {
        ++probe->rejected_tick_count;
        return foc_advanced_probe_fail(
            probe, FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE,
            after->fault_epoch);
    }
    if ((before->fault_epoch != probe->expected_fault_epoch) ||
        (after->fault_epoch != probe->expected_fault_epoch))
    {
        ++probe->rejected_tick_count;
        return foc_advanced_probe_fail(
            probe, FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            after->fault_epoch);
    }
    ++probe->executed_ticks;
    ++probe->no_power_commit_count;
    probe->observed_fault_epoch = after->fault_epoch;
    if (probe->executed_ticks >= probe->requested_ticks)
    {
        probe->state = FOC_ADVANCED_PROBE_COMPLETE;
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_COMPLETE);
    }
    return foc_advanced_probe_record(probe, FOC_ADVANCED_PROBE_RESULT_OK);
}

foc_advanced_probe_result_t foc_advanced_probe_fail(
    foc_advanced_probe_t *probe,
    foc_advanced_probe_result_t reason,
    uint32_t observed_fault_epoch)
{
    if (foc_advanced_probe_valid(probe) == 0U)
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if ((reason != FOC_ADVANCED_PROBE_RESULT_OUTPUT_NOT_SAFE) &&
        (reason != FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED) &&
        (reason != FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE))
    {
        return foc_advanced_probe_record(
            probe, FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT);
    }
    probe->state = FOC_ADVANCED_PROBE_FAILED;
    probe->observed_fault_epoch = observed_fault_epoch;
    return foc_advanced_probe_record(probe, reason);
}

foc_advanced_probe_result_t foc_advanced_probe_get_status(
    const foc_advanced_probe_t *probe,
    foc_advanced_probe_status_t *status)
{
    if ((foc_advanced_probe_valid(probe) == 0U) || (status == 0))
    {
        return FOC_ADVANCED_PROBE_RESULT_INVALID_ARGUMENT;
    }
    *status = *probe;
    return FOC_ADVANCED_PROBE_RESULT_OK;
}
