#ifndef FOC_POWER_SAFETY_H
#define FOC_POWER_SAFETY_H

#include <stdint.h>

/*
 * Power-stage safety transaction owned by the C platform layer.
 *
 * All fields are 32-bit and naturally aligned so Cortex-M4 thread/ISR accesses
 * are indivisible.  Register ordering and interrupt masking remain the board
 * adapter's responsibility; this module supplies the state/epoch contract that
 * makes an interrupted arm or output commit fail closed.
 */
typedef enum
{
    FOC_POWER_SAFETY_DISABLED = 0U,
    FOC_POWER_SAFETY_ARMING = 1U,
    FOC_POWER_SAFETY_ARMED = 2U,
    FOC_POWER_SAFETY_FAULT_LATCHED = 3U,
} foc_power_safety_state_t;

enum
{
    FOC_POWER_FAULT_BREAK = (1UL << 0),
    FOC_POWER_FAULT_DRIVER = (1UL << 1),
    FOC_POWER_FAULT_CURRENT = (1UL << 2),
    FOC_POWER_FAULT_BUS_VOLTAGE = (1UL << 3),
    FOC_POWER_FAULT_DEADLINE = (1UL << 4),
    FOC_POWER_FAULT_CONTROL = (1UL << 5),
    FOC_POWER_FAULT_OUTPUT = (1UL << 6),
    FOC_POWER_FAULT_PLATFORM = (1UL << 7),
};

typedef struct
{
    volatile uint32_t state;
    volatile uint32_t fault_epoch;
    volatile uint32_t latched_faults;
} foc_power_safety_t;

typedef struct
{
    uint32_t fault_epoch;
} foc_power_arm_token_t;

/*
 * Fast-path form of the output-commit predicate.  Keep the predicate in this
 * header so the target ISR can inline the three volatile reads while the
 * exported wrapper below remains available to host tests and non-hot callers.
 * Both forms deliberately require ARMED, no sticky fault, and the exact epoch
 * captured before the control calculation.
 */
static inline uint32_t foc_power_safety_output_permitted_inline(
    const foc_power_safety_t *safety,
    uint32_t expected_fault_epoch)
{
    if ((safety == 0) ||
        (safety->state != FOC_POWER_SAFETY_ARMED) ||
        (safety->latched_faults != 0U) ||
        (safety->fault_epoch != expected_fault_epoch))
    {
        return 0U;
    }
    return 1U;
}

void foc_power_safety_init(foc_power_safety_t *safety);
uint32_t foc_power_safety_begin_arm(foc_power_safety_t *safety,
                                    foc_power_arm_token_t *token);
uint32_t foc_power_safety_commit_arm(foc_power_safety_t *safety,
                                     const foc_power_arm_token_t *token);
void foc_power_safety_abort_arm(foc_power_safety_t *safety);
void foc_power_safety_stop(foc_power_safety_t *safety);
void foc_power_safety_latch_fault(foc_power_safety_t *safety,
                                  uint32_t fault_mask);
uint32_t foc_power_safety_output_permitted(
    const foc_power_safety_t *safety,
    uint32_t expected_fault_epoch);
uint32_t foc_power_safety_clear_faults(foc_power_safety_t *safety,
                                       uint32_t recovery_preflight_passed);

#endif /* FOC_POWER_SAFETY_H */
