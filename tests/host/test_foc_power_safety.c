#include "foc_power_safety.h"

#include <assert.h>

int main(void)
{
    foc_power_safety_t safety;
    foc_power_arm_token_t token;
    uint32_t armed_epoch;

    foc_power_safety_init(&safety);
    assert(safety.state == FOC_POWER_SAFETY_DISABLED);
    assert(foc_power_safety_begin_arm(&safety, &token) == 1U);
    assert(foc_power_safety_commit_arm(&safety, &token) == 1U);
    armed_epoch = safety.fault_epoch;
    assert(foc_power_safety_output_permitted(&safety, armed_epoch) == 1U);

    /* A fault pre-empting a control step invalidates its captured epoch. */
    foc_power_safety_latch_fault(&safety, FOC_POWER_FAULT_BREAK);
    assert(safety.state == FOC_POWER_SAFETY_FAULT_LATCHED);
    assert((safety.latched_faults & FOC_POWER_FAULT_BREAK) != 0U);
    assert(foc_power_safety_output_permitted(&safety, armed_epoch) == 0U);
    assert(foc_power_safety_begin_arm(&safety, &token) == 0U);
    assert(foc_power_safety_clear_faults(&safety, 0U) == 0U);
    assert(foc_power_safety_clear_faults(&safety, 1U) == 1U);

    /* A fault between begin and commit makes the arm token stale. */
    assert(foc_power_safety_begin_arm(&safety, &token) == 1U);
    foc_power_safety_latch_fault(&safety, FOC_POWER_FAULT_DRIVER);
    assert(foc_power_safety_commit_arm(&safety, &token) == 0U);
    assert(foc_power_safety_clear_faults(&safety, 1U) == 1U);

    assert(foc_power_safety_begin_arm(&safety, &token) == 1U);
    foc_power_safety_abort_arm(&safety);
    assert(safety.state == FOC_POWER_SAFETY_DISABLED);
    assert(foc_power_safety_commit_arm(&safety, &token) == 0U);

    assert(foc_power_safety_begin_arm(&safety, &token) == 1U);
    assert(foc_power_safety_commit_arm(&safety, &token) == 1U);
    foc_power_safety_stop(&safety);
    assert(safety.state == FOC_POWER_SAFETY_DISABLED);
    return 0;
}
