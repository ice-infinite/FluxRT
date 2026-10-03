#ifndef FOC_ARM_DIAGNOSTICS_H
#define FOC_ARM_DIAGNOSTICS_H

/*
 * Arm transaction evidence.
 *
 * This module is deliberately register-agnostic: the STM32 platform captures
 * one coherent register fact set and this helper encodes it into a stable
 * 16-bit diagnostic mask.  It owns no hardware and allocates no state.
 */

#include <stdint.h>

typedef uint16_t foc_arm_reject_stage_t;
enum
{
    FOC_ARM_REJECT_STAGE_NONE = 0U,
    FOC_ARM_REJECT_STAGE_BEGIN = 1U,
    FOC_ARM_REJECT_STAGE_PRE_ENABLE = 2U,
    FOC_ARM_REJECT_STAGE_POST_ENABLE = 3U,
};

enum
{
    FOC_ARM_REJECT_FACT_BIF = (1U << 0),
    FOC_ARM_REJECT_FACT_B2IF = (1U << 1),
    FOC_ARM_REJECT_FACT_DRIVER = (1U << 2),
    FOC_ARM_REJECT_FACT_SAFETY_STATE = (1U << 3),
    FOC_ARM_REJECT_FACT_FAULT_EPOCH = (1U << 4),
    FOC_ARM_REJECT_FACT_BUS_VOLTAGE = (1U << 5),
    FOC_ARM_REJECT_FACT_CHANNELS = (1U << 6),
    FOC_ARM_REJECT_FACT_MOE = (1U << 7),
    FOC_ARM_REJECT_FACT_GATE = (1U << 8),
    FOC_ARM_REJECT_FACT_COMMIT = (1U << 9),
    FOC_ARM_REJECT_FACT_BREAK_IRQ = (1U << 10),
    /* A stale B2IF was cleared only after every output/live-input guard passed.
     * This is evidence, not a fault bit. */
    FOC_ARM_REJECT_FACT_B2IF_REARMED = (1U << 11),
};

/* Return blockers for clearing a historical B2IF while every output remains
 * closed. B2IF itself is intentionally not a blocker; BIF is never cleared by
 * this recovery path. */
uint16_t foc_arm_diagnostics_rearm_blockers(
    uint32_t timer_status,
    uint32_t break1_mask,
    uint32_t driver_faulted,
    uint32_t safety_disabled,
    uint32_t phase_channels_disabled,
    uint32_t moe_disabled,
    uint32_t gate_low,
    uint32_t break_irq_disabled);

uint16_t foc_arm_diagnostics_pre_facts(
    uint32_t timer_status,
    uint32_t break1_mask,
    uint32_t break2_mask,
    uint32_t driver_faulted,
    uint32_t safety_state_ok,
    uint32_t fault_epoch_ok,
    uint32_t bus_voltage_ok);

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
    uint32_t commit_ok);

#endif /* FOC_ARM_DIAGNOSTICS_H */
