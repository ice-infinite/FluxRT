#ifndef FOC_ADVANCED_POWER_TRIAL_H
#define FOC_ADVANCED_POWER_TRIAL_H

/*
 * P5.4A bounded Advanced-FOC powered-trial coordinator.
 *
 * This is a hardware-neutral, allocation-free state machine.  It never arms
 * or touches a peripheral.  The platform remains the sole owner of
 * Gate/MOE/CCER and calls this module at the ADC-ISR boundary.
 */

#include <stdint.h>

#include "foc_advanced_bridge.h"

#define FOC_ADVANCED_POWER_TRIAL_VERSION          (1UL)
#define FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS     (1200UL)
#define FOC_ADVANCED_POWER_TRIAL_MAX_TOTAL_TICKS  (168000UL)
#define FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM (582.0f)

typedef uint32_t foc_advanced_power_trial_mode_t;
enum
{
    FOC_ADVANCED_POWER_TRIAL_MODE_BASIC = 0,
    FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING = 1,
};

typedef uint32_t foc_advanced_power_trial_state_t;
enum
{
    FOC_ADVANCED_POWER_TRIAL_IDLE = 0,
    FOC_ADVANCED_POWER_TRIAL_READY = 1,
    FOC_ADVANCED_POWER_TRIAL_STARTUP = 2,
    FOC_ADVANCED_POWER_TRIAL_ACTIVE = 3,
    FOC_ADVANCED_POWER_TRIAL_COMPLETE = 4,
    FOC_ADVANCED_POWER_TRIAL_FAILED = 5,
    FOC_ADVANCED_POWER_TRIAL_ABORTED = 6,
};

typedef uint32_t foc_advanced_power_trial_result_t;
enum
{
    FOC_ADVANCED_POWER_TRIAL_RESULT_NONE = 0,
    FOC_ADVANCED_POWER_TRIAL_RESULT_OK = 1,
    FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ARGUMENT = 2,
    FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE = 3,
    FOC_ADVANCED_POWER_TRIAL_RESULT_TOTAL_TIMEOUT = 4,
    FOC_ADVANCED_POWER_TRIAL_RESULT_FAULT_EPOCH_CHANGED = 5,
    FOC_ADVANCED_POWER_TRIAL_RESULT_DEADLINE_MISSED = 6,
    FOC_ADVANCED_POWER_TRIAL_RESULT_OBSERVER_FALLBACK = 7,
    FOC_ADVANCED_POWER_TRIAL_RESULT_ADVANCED_FAULT = 8,
    FOC_ADVANCED_POWER_TRIAL_RESULT_PLATFORM_FAULT = 9,
    FOC_ADVANCED_POWER_TRIAL_RESULT_ABORTED = 10,
};

typedef uint32_t foc_advanced_power_trial_action_t;
enum
{
    FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE = 0,
    FOC_ADVANCED_POWER_TRIAL_ACTION_COMPLETE_STOP = 1,
    FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP = 2,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_advanced_power_trial_state_t state;
    foc_advanced_power_trial_result_t result;
    foc_advanced_power_trial_mode_t mode;
    uint32_t total_ticks;
    uint32_t active_ticks;
    uint32_t first_active_tick;
    uint32_t expected_fault_epoch;
    uint32_t observed_fault_epoch;
    uint32_t initial_deadline_miss_count;
    uint32_t observed_deadline_miss_count;
    uint32_t last_advanced_status_flags;
    uint32_t last_active_features;
} foc_advanced_power_trial_t;

typedef foc_advanced_power_trial_t foc_advanced_power_trial_status_t;

foc_advanced_power_trial_result_t foc_advanced_power_trial_init(
    foc_advanced_power_trial_t *trial);
foc_advanced_power_trial_result_t foc_advanced_power_trial_prepare(
    foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_mode_t mode,
    const foc_runtime_config_t *runtime_config,
    const foc_advanced_runtime_config_t *advanced_config,
    uint32_t expected_fault_epoch,
    uint32_t initial_deadline_miss_count);
foc_advanced_power_trial_result_t foc_advanced_power_trial_mark_armed(
    foc_advanced_power_trial_t *trial);
foc_advanced_power_trial_action_t foc_advanced_power_trial_begin_tick(
    foc_advanced_power_trial_t *trial,
    uint32_t fault_epoch,
    uint32_t deadline_miss_count);
/* Validate the compact coherent controller/Advanced snapshot before PWM
 * commit.  This may enter
 * ACTIVE but deliberately does not count a tick.  The platform must call
 * record_commit only after the physical output transaction is accepted. */
foc_advanced_power_trial_action_t foc_advanced_power_trial_validate_control(
    foc_advanced_power_trial_t *trial,
    const foc_advanced_power_trial_snapshot_t *snapshot);
foc_advanced_power_trial_action_t foc_advanced_power_trial_record_commit(
    foc_advanced_power_trial_t *trial);
void foc_advanced_power_trial_fail(
    foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_result_t result,
    uint32_t fault_epoch,
    uint32_t deadline_miss_count);
void foc_advanced_power_trial_abort(foc_advanced_power_trial_t *trial);
foc_advanced_power_trial_result_t foc_advanced_power_trial_get_status(
    const foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_status_t *status);

#endif /* FOC_ADVANCED_POWER_TRIAL_H */
