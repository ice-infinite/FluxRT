#include "foc_motion_probe.h"

#include <string.h>

static uint32_t foc_motion_probe_valid(const foc_motion_probe_t *probe)
{
    return (probe != 0) &&
           (probe->struct_size == (uint32_t)sizeof(*probe)) &&
           (probe->version == FOC_MOTION_PROBE_VERSION) &&
           (probe->state <= FOC_MOTION_PROBE_FAILED);
}

static foc_motion_probe_result_t foc_motion_probe_record(
    foc_motion_probe_t *probe,
    foc_motion_probe_result_t result)
{
    probe->last_result = result;
    return result;
}

uint32_t foc_motion_probe_registers_are_safe_off(
    const foc_motion_probe_register_snapshot_t *registers)
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

foc_motion_probe_result_t foc_motion_probe_init(foc_motion_probe_t *probe)
{
    if (probe == 0)
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    (void)memset(probe, 0, sizeof(*probe));
    probe->struct_size = (uint32_t)sizeof(*probe);
    probe->version = FOC_MOTION_PROBE_VERSION;
    probe->state = FOC_MOTION_PROBE_IDLE;
    probe->last_control_status = FOC_STATUS_DISABLED;
    return FOC_MOTION_PROBE_RESULT_OK;
}

foc_motion_probe_result_t foc_motion_probe_configure(
    foc_motion_probe_t *probe,
    uint32_t expected_fault_epoch)
{
    if ((foc_motion_probe_valid(probe) == 0U) ||
        ((probe->state != FOC_MOTION_PROBE_IDLE) &&
         (probe->state != FOC_MOTION_PROBE_CONFIGURED)))
    {
        return (probe == 0) ? FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT :
                              FOC_MOTION_PROBE_RESULT_INVALID_STATE;
    }
    probe->state = FOC_MOTION_PROBE_CONFIGURED;
    probe->requested_ticks = 0U;
    probe->executed_ticks = 0U;
    probe->no_power_commit_count = 0U;
    probe->rejected_tick_count = 0U;
    probe->expected_fault_epoch = expected_fault_epoch;
    probe->observed_fault_epoch = expected_fault_epoch;
    probe->last_control_status = FOC_STATUS_DISABLED;
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

foc_motion_probe_result_t foc_motion_probe_start(
    foc_motion_probe_t *probe,
    uint32_t requested_ticks,
    const foc_motion_probe_register_snapshot_t *registers)
{
    if ((foc_motion_probe_valid(probe) == 0U) || (registers == 0) ||
        (requested_ticks < FOC_MOTION_PROBE_MIN_TICKS) ||
        (requested_ticks > FOC_MOTION_PROBE_MAX_TICKS))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if (probe->state != FOC_MOTION_PROBE_CONFIGURED)
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_STATE);
    }
    if (foc_motion_probe_registers_are_safe_off(registers) == 0U)
    {
        probe->state = FOC_MOTION_PROBE_FAILED;
        probe->observed_fault_epoch = registers->fault_epoch;
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE);
    }
    if (registers->fault_epoch != probe->expected_fault_epoch)
    {
        probe->state = FOC_MOTION_PROBE_FAILED;
        probe->observed_fault_epoch = registers->fault_epoch;
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED);
    }
    probe->requested_ticks = requested_ticks;
    probe->state = FOC_MOTION_PROBE_RUNNING;
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

foc_motion_probe_result_t foc_motion_probe_begin_tick(
    foc_motion_probe_t *probe,
    const foc_motion_probe_register_snapshot_t *registers)
{
    if ((foc_motion_probe_valid(probe) == 0U) || (registers == 0))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if (probe->state != FOC_MOTION_PROBE_RUNNING)
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_STATE);
    }
    if (foc_motion_probe_registers_are_safe_off(registers) == 0U)
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe, FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE,
            registers->fault_epoch);
    }
    if (registers->fault_epoch != probe->expected_fault_epoch)
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe, FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            registers->fault_epoch);
    }
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

foc_motion_probe_result_t foc_motion_probe_complete_combined(
    foc_motion_probe_t *probe,
    foc_status_t control_status,
    const foc_motion_probe_register_snapshot_t *registers)
{
    if ((foc_motion_probe_valid(probe) == 0U) || (registers == 0))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    probe->last_control_status = control_status;
    if (probe->state != FOC_MOTION_PROBE_RUNNING)
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_STATE);
    }
    if ((control_status != FOC_STATUS_OK) ||
        (foc_motion_probe_registers_are_safe_off(registers) == 0U))
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe,
            (control_status == FOC_STATUS_OK) ?
                FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE :
                FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE,
            registers->fault_epoch);
    }
    if (registers->fault_epoch != probe->expected_fault_epoch)
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe, FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            registers->fault_epoch);
    }
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

foc_motion_probe_result_t foc_motion_probe_complete_commit(
    foc_motion_probe_t *probe,
    const foc_motion_probe_register_snapshot_t *before,
    const foc_motion_probe_register_snapshot_t *after)
{
    if ((foc_motion_probe_valid(probe) == 0U) ||
        (before == 0) || (after == 0))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if (probe->state != FOC_MOTION_PROBE_RUNNING)
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_STATE);
    }
    if ((foc_motion_probe_registers_are_safe_off(before) == 0U) ||
        (foc_motion_probe_registers_are_safe_off(after) == 0U))
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe, FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE,
            after->fault_epoch);
    }
    if ((before->fault_epoch != probe->expected_fault_epoch) ||
        (after->fault_epoch != probe->expected_fault_epoch))
    {
        ++probe->rejected_tick_count;
        return foc_motion_probe_fail(
            probe, FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            after->fault_epoch);
    }

    ++probe->executed_ticks;
    ++probe->no_power_commit_count;
    probe->observed_fault_epoch = after->fault_epoch;
    if (probe->executed_ticks >= probe->requested_ticks)
    {
        probe->state = FOC_MOTION_PROBE_COMPLETE;
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_COMPLETE);
    }
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

foc_motion_probe_result_t foc_motion_probe_fail(
    foc_motion_probe_t *probe,
    foc_motion_probe_result_t reason,
    uint32_t observed_fault_epoch)
{
    if (foc_motion_probe_valid(probe) == 0U)
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if ((reason != FOC_MOTION_PROBE_RESULT_OUTPUT_NOT_SAFE) &&
        (reason != FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED) &&
        (reason != FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE))
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT);
    }
    probe->state = FOC_MOTION_PROBE_FAILED;
    probe->observed_fault_epoch = observed_fault_epoch;
    return foc_motion_probe_record(probe, reason);
}

foc_motion_probe_result_t foc_motion_probe_set_injection(
    foc_motion_probe_t *probe,
    foc_motion_probe_injection_point_t point)
{
    if ((foc_motion_probe_valid(probe) == 0U) ||
        (point > FOC_MOTION_PROBE_INJECT_AFTER_COMMIT))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    if ((probe->state == FOC_MOTION_PROBE_RUNNING) ||
        (probe->pending_injection != FOC_MOTION_PROBE_INJECT_NONE))
    {
        return foc_motion_probe_record(
            probe, FOC_MOTION_PROBE_RESULT_INVALID_STATE);
    }
    probe->pending_injection = point;
    return foc_motion_probe_record(probe, FOC_MOTION_PROBE_RESULT_OK);
}

uint32_t foc_motion_probe_take_injection(
    foc_motion_probe_t *probe,
    foc_motion_probe_injection_point_t point)
{
    if ((foc_motion_probe_valid(probe) == 0U) ||
        (point == FOC_MOTION_PROBE_INJECT_NONE) ||
        (probe->pending_injection != point))
    {
        return 0U;
    }
    probe->pending_injection = FOC_MOTION_PROBE_INJECT_NONE;
    ++probe->injection_count;
    return 1U;
}

foc_motion_probe_result_t foc_motion_probe_get_status(
    const foc_motion_probe_t *probe,
    foc_motion_probe_status_t *status)
{
    if ((foc_motion_probe_valid(probe) == 0U) || (status == 0))
    {
        return FOC_MOTION_PROBE_RESULT_INVALID_ARGUMENT;
    }
    *status = *probe;
    return FOC_MOTION_PROBE_RESULT_OK;
}
