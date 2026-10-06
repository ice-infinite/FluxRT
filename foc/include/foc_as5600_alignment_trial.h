#ifndef FOC_AS5600_ALIGNMENT_TRIAL_H
#define FOC_AS5600_ALIGNMENT_TRIAL_H

/*
 * Bounded electrical-alignment trial owner.
 *
 * This state machine is hardware-independent. It validates one exact runtime
 * envelope and tells the STM32 platform when the accepted alignment window has
 * ended. It never touches registers and never reads the AS5600; Gate/MOE/CCER
 * remain owned by the platform and I2C remains in task context.
 */

#include <stdint.h>

#include "foc_rust_bridge.h"

#define FOC_AS5600_ALIGNMENT_TRIAL_VERSION              (1UL)
#define FOC_AS5600_ALIGNMENT_TRIAL_CONTROL_HZ           (12000UL)
#define FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS         (12000UL)
#define FOC_AS5600_ALIGNMENT_TRIAL_MAX_TOTAL_TICKS      (12600UL)
#define FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A            (0.20f)
#define FOC_AS5600_ALIGNMENT_TRIAL_ALIGNMENT_DURATION_S (1.25f)
#define FOC_AS5600_ALIGNMENT_TRIAL_SOFTWARE_TRIP_A      (0.45f)

typedef uint32_t foc_as5600_alignment_trial_state_t;
enum
{
    FOC_AS5600_ALIGNMENT_TRIAL_IDLE = 0,
    FOC_AS5600_ALIGNMENT_TRIAL_READY = 1,
    FOC_AS5600_ALIGNMENT_TRIAL_ARMED = 2,
    FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE = 3,
    FOC_AS5600_ALIGNMENT_TRIAL_COMPLETE = 4,
    FOC_AS5600_ALIGNMENT_TRIAL_FAILED = 5,
    FOC_AS5600_ALIGNMENT_TRIAL_ABORTED = 6,
};

typedef uint32_t foc_as5600_alignment_trial_result_t;
enum
{
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_NONE = 0,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK = 1,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT = 2,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ENVELOPE = 3,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_WRONG_CONTROL_STATE = 4,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_TOTAL_TIMEOUT = 5,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_PLATFORM_FAULT = 6,
    FOC_AS5600_ALIGNMENT_TRIAL_RESULT_ABORTED = 7,
};

typedef uint32_t foc_as5600_alignment_trial_action_t;
enum
{
    FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE = 0,
    FOC_AS5600_ALIGNMENT_TRIAL_ACTION_COMPLETE_STOP = 1,
    FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP = 2,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_as5600_alignment_trial_state_t state;
    foc_as5600_alignment_trial_result_t result;
    uint32_t total_ticks;
    uint32_t committed_alignment_ticks;
    uint32_t fault_epoch;
} foc_as5600_alignment_trial_t;

typedef foc_as5600_alignment_trial_t foc_as5600_alignment_trial_status_t;

foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_init(
    foc_as5600_alignment_trial_t *trial);
foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_prepare(
    foc_as5600_alignment_trial_t *trial,
    const foc_runtime_config_t *runtime_config);
foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_mark_armed(
    foc_as5600_alignment_trial_t *trial);
foc_as5600_alignment_trial_action_t foc_as5600_alignment_trial_begin_tick(
    foc_as5600_alignment_trial_t *trial);
foc_as5600_alignment_trial_action_t foc_as5600_alignment_trial_record_commit(
    foc_as5600_alignment_trial_t *trial,
    foc_state_t controller_state);
void foc_as5600_alignment_trial_fail(
    foc_as5600_alignment_trial_t *trial,
    foc_as5600_alignment_trial_result_t result,
    uint32_t fault_epoch);
void foc_as5600_alignment_trial_abort(foc_as5600_alignment_trial_t *trial);
foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_get_status(
    const foc_as5600_alignment_trial_t *trial,
    foc_as5600_alignment_trial_status_t *status);

#endif /* FOC_AS5600_ALIGNMENT_TRIAL_H */
