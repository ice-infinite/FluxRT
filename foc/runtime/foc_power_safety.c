#include "foc_power_safety.h"

void foc_power_safety_init(foc_power_safety_t *safety)
{
    if (safety == 0)
    {
        return;
    }
    safety->state = FOC_POWER_SAFETY_DISABLED;
    safety->fault_epoch = 0U;
    safety->latched_faults = 0U;
}

uint32_t foc_power_safety_begin_arm(foc_power_safety_t *safety,
                                    foc_power_arm_token_t *token)
{
    if ((safety == 0) || (token == 0) ||
        (safety->state != FOC_POWER_SAFETY_DISABLED) ||
        (safety->latched_faults != 0U))
    {
        return 0U;
    }
    token->fault_epoch = safety->fault_epoch;
    safety->state = FOC_POWER_SAFETY_ARMING;
    return 1U;
}

uint32_t foc_power_safety_commit_arm(foc_power_safety_t *safety,
                                     const foc_power_arm_token_t *token)
{
    if ((safety == 0) || (token == 0) ||
        (safety->state != FOC_POWER_SAFETY_ARMING) ||
        (safety->latched_faults != 0U) ||
        (safety->fault_epoch != token->fault_epoch))
    {
        return 0U;
    }
    safety->state = FOC_POWER_SAFETY_ARMED;
    return 1U;
}

void foc_power_safety_abort_arm(foc_power_safety_t *safety)
{
    if ((safety != 0) && (safety->state == FOC_POWER_SAFETY_ARMING))
    {
        safety->state = FOC_POWER_SAFETY_DISABLED;
    }
}

void foc_power_safety_stop(foc_power_safety_t *safety)
{
    if ((safety != 0) &&
        (safety->state != FOC_POWER_SAFETY_FAULT_LATCHED))
    {
        safety->state = FOC_POWER_SAFETY_DISABLED;
    }
}

void foc_power_safety_latch_fault(foc_power_safety_t *safety,
                                  uint32_t fault_mask)
{
    if (safety == 0)
    {
        return;
    }
    safety->latched_faults |= (fault_mask != 0U) ?
        fault_mask : FOC_POWER_FAULT_PLATFORM;
    /* 故障纪元只能单调不减，饱和比回绕更安全；锁存位仍会使
     * 饱和后的任何 arm 提交失败。
     * The fault epoch is monotonic and saturates instead of wrapping. The
     * latched mask still rejects every arm commit after saturation. */
    if (safety->fault_epoch != UINT32_MAX)
    {
        ++safety->fault_epoch;
    }
    safety->state = FOC_POWER_SAFETY_FAULT_LATCHED;
}

uint32_t foc_power_safety_output_permitted(
    const foc_power_safety_t *safety,
    uint32_t expected_fault_epoch)
{
    return foc_power_safety_output_permitted_inline(
        safety, expected_fault_epoch);
}

uint32_t foc_power_safety_clear_faults(foc_power_safety_t *safety,
                                       uint32_t recovery_preflight_passed)
{
    if ((safety == 0) || (recovery_preflight_passed == 0U) ||
        (safety->state != FOC_POWER_SAFETY_FAULT_LATCHED))
    {
        return 0U;
    }
    safety->latched_faults = 0U;
    safety->state = FOC_POWER_SAFETY_DISABLED;
    return 1U;
}
