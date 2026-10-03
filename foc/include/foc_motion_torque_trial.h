#ifndef FOC_MOTION_TORQUE_TRIAL_H
#define FOC_MOTION_TORQUE_TRIAL_H

/*
 * P4.2E2 bounded Torque hardware-trial coordinator.
 *
 * This module is pure C state-machine logic.  It owns no peripheral and never
 * arms power.  The STM32 platform calls it at the ADC-ISR boundary and remains
 * the sole owner of Gate/MOE/CCER and physical shutdown.
 */

#include <stdint.h>

#include "foc_motion_bridge.h"

#define FOC_MOTION_TORQUE_TRIAL_VERSION             (1UL)
#define FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS        (1200UL)
#define FOC_MOTION_TORQUE_TRIAL_MAX_TOTAL_TICKS     (168000UL)
#define FOC_MOTION_TORQUE_TRIAL_COMMAND_WINDOW_MS   (15000UL)
#define FOC_MOTION_TORQUE_TRIAL_TORQUE_NM           (0.002f)
#define FOC_MOTION_TORQUE_TRIAL_TORQUE_LIMIT_NM     (0.003f)
#define FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A     (0.08f)
#define FOC_MOTION_TORQUE_TRIAL_STARTUP_CURRENT_A   (0.8f)

typedef uint32_t foc_motion_torque_trial_state_t;
enum
{
    FOC_MOTION_TORQUE_TRIAL_IDLE = 0,
    FOC_MOTION_TORQUE_TRIAL_READY = 1,
    FOC_MOTION_TORQUE_TRIAL_STARTUP = 2,
    FOC_MOTION_TORQUE_TRIAL_ACTIVE = 3,
    FOC_MOTION_TORQUE_TRIAL_COMPLETE = 4,
    FOC_MOTION_TORQUE_TRIAL_FAILED = 5,
    FOC_MOTION_TORQUE_TRIAL_ABORTED = 6,
};

typedef uint32_t foc_motion_torque_trial_result_t;
enum
{
    FOC_MOTION_TORQUE_TRIAL_RESULT_NONE = 0,
    FOC_MOTION_TORQUE_TRIAL_RESULT_OK = 1,
    FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ARGUMENT = 2,
    FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE = 3,
    FOC_MOTION_TORQUE_TRIAL_RESULT_TOTAL_TIMEOUT = 4,
    FOC_MOTION_TORQUE_TRIAL_RESULT_PLATFORM_FAULT = 5,
    FOC_MOTION_TORQUE_TRIAL_RESULT_ABORTED = 6,
};

typedef uint32_t foc_motion_torque_trial_action_t;
enum
{
    FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE = 0,
    FOC_MOTION_TORQUE_TRIAL_ACTION_COMPLETE_STOP = 1,
    FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP = 2,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_motion_torque_trial_state_t state;
    foc_motion_torque_trial_result_t result;
    uint32_t total_ticks;
    uint32_t active_ticks;
    uint32_t first_active_tick;
    uint32_t fault_epoch;
    uint32_t source_sequence;
} foc_motion_torque_trial_t;

typedef foc_motion_torque_trial_t foc_motion_torque_trial_status_t;

foc_motion_torque_trial_result_t foc_motion_torque_trial_init(
    foc_motion_torque_trial_t *trial);
foc_motion_torque_trial_result_t foc_motion_torque_trial_prepare(
    foc_motion_torque_trial_t *trial,
    const foc_product_command_t *command,
    const foc_runtime_config_t *runtime_config);
foc_motion_torque_trial_result_t foc_motion_torque_trial_mark_armed(
    foc_motion_torque_trial_t *trial);
foc_motion_torque_trial_action_t foc_motion_torque_trial_begin_tick(
    foc_motion_torque_trial_t *trial);
foc_motion_torque_trial_action_t foc_motion_torque_trial_record_commit(
    foc_motion_torque_trial_t *trial,
    const foc_motion_output_t *motion_output);
void foc_motion_torque_trial_fail(foc_motion_torque_trial_t *trial,
                                  foc_motion_torque_trial_result_t result,
                                  uint32_t fault_epoch);
void foc_motion_torque_trial_abort(foc_motion_torque_trial_t *trial);
foc_motion_torque_trial_result_t foc_motion_torque_trial_get_status(
    const foc_motion_torque_trial_t *trial,
    foc_motion_torque_trial_status_t *status);

#endif /* FOC_MOTION_TORQUE_TRIAL_H */
