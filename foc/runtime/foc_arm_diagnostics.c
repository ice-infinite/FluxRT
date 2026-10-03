#include "foc_arm_diagnostics.h"

static uint16_t foc_arm_diagnostics_common_facts(
    uint32_t timer_status,
    uint32_t break1_mask,
    uint32_t break2_mask,
    uint32_t driver_faulted,
    uint32_t safety_state_ok,
    uint32_t fault_epoch_ok,
    uint32_t bus_voltage_ok)
{
    uint16_t facts = 0U;

    if ((timer_status & break1_mask) != 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_BIF;
    }
    if ((timer_status & break2_mask) != 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_B2IF;
    }
    if (driver_faulted != 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_DRIVER;
    }
    if (safety_state_ok == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_SAFETY_STATE;
    }
    if (fault_epoch_ok == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_FAULT_EPOCH;
    }
    if (bus_voltage_ok == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_BUS_VOLTAGE;
    }
    return facts;
}

uint16_t foc_arm_diagnostics_pre_facts(
    uint32_t timer_status,
    uint32_t break1_mask,
    uint32_t break2_mask,
    uint32_t driver_faulted,
    uint32_t safety_state_ok,
    uint32_t fault_epoch_ok,
    uint32_t bus_voltage_ok)
{
    return foc_arm_diagnostics_common_facts(
        timer_status,
        break1_mask,
        break2_mask,
        driver_faulted,
        safety_state_ok,
        fault_epoch_ok,
        bus_voltage_ok);
}

uint16_t foc_arm_diagnostics_post_facts(
    uint32_t timer_status,
    uint32_t break1_mask,
    uint32_t break2_mask,
    uint32_t driver_faulted,
    uint32_t safety_state_ok,
    uint32_t fault_epoch_ok,
    uint32_t bus_voltage_ok,
    uint32_t phase_channels_enabled,
    uint32_t moe_enabled,
    uint32_t gate_enabled,
    uint32_t commit_ok)
{
    uint16_t facts = foc_arm_diagnostics_common_facts(
        timer_status,
        break1_mask,
        break2_mask,
        driver_faulted,
        safety_state_ok,
        fault_epoch_ok,
        bus_voltage_ok);

    if (phase_channels_enabled == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_CHANNELS;
    }
    if (moe_enabled == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_MOE;
    }
    if (gate_enabled == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_GATE;
    }
    if (commit_ok == 0U)
    {
        facts |= FOC_ARM_REJECT_FACT_COMMIT;
    }
    return facts;
}
