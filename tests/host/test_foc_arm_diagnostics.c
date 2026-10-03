#include <assert.h>
#include <stdint.h>

#include "foc_arm_diagnostics.h"

static void pre_enable_facts_distinguish_break_sources(void)
{
    const uint32_t bif = (1UL << 7);
    const uint32_t b2if = (1UL << 8);
    uint16_t facts;

    facts = foc_arm_diagnostics_pre_facts(0U, bif, b2if, 0U, 1U, 1U, 1U);
    assert(facts == 0U);

    facts = foc_arm_diagnostics_pre_facts(
        bif | b2if, bif, b2if, 1U, 0U, 0U, 0U);
    assert((facts & FOC_ARM_REJECT_FACT_BIF) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_B2IF) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_DRIVER) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_SAFETY_STATE) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_FAULT_EPOCH) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_BUS_VOLTAGE) != 0U);
    assert((facts & (FOC_ARM_REJECT_FACT_CHANNELS |
                     FOC_ARM_REJECT_FACT_MOE |
                     FOC_ARM_REJECT_FACT_GATE |
                     FOC_ARM_REJECT_FACT_COMMIT)) == 0U);
}

static void post_enable_facts_cover_output_commit(void)
{
    const uint32_t bif = (1UL << 3);
    const uint32_t b2if = (1UL << 5);
    uint16_t facts;

    facts = foc_arm_diagnostics_post_facts(
        0U, bif, b2if, 0U, 1U, 1U, 1U, 1U, 1U, 1U, 1U);
    assert(facts == 0U);

    facts = foc_arm_diagnostics_post_facts(
        b2if, bif, b2if, 1U, 1U, 1U, 1U, 0U, 0U, 0U, 0U);
    assert((facts & FOC_ARM_REJECT_FACT_BIF) == 0U);
    assert((facts & FOC_ARM_REJECT_FACT_B2IF) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_DRIVER) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_CHANNELS) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_MOE) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_GATE) != 0U);
    assert((facts & FOC_ARM_REJECT_FACT_COMMIT) != 0U);
}

int main(void)
{
    pre_enable_facts_distinguish_break_sources();
    post_enable_facts_cover_output_commit();
    return 0;
}
