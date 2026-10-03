#ifndef FOC_CONFIG_APPLY_EXECUTOR_H
#define FOC_CONFIG_APPLY_EXECUTOR_H

/*
 * Hardware-neutral, non-realtime executor for one Rust configuration plan.
 *
 * The executor deliberately contains no callbacks and touches no hardware. A
 * management task asks for the next action, performs it through the owning
 * adapter, and reports success/failure. Only COMPLETE permits
 * foc_rust_config_confirm_apply(); FAILED requires an explicit safe reset and
 * rollback/recovery decision by the caller.
 */

#include <stdint.h>

#include "foc_config_bridge.h"

#define FOC_CONFIG_APPLY_EXECUTOR_VERSION (1UL)

typedef uint32_t foc_config_apply_executor_status_t;
enum
{
    FOC_CONFIG_EXECUTOR_STATUS_OK = 0,
    FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT = 1,
    FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE = 2,
    FOC_CONFIG_EXECUTOR_STATUS_ACTION_FAILED = 3,
};

typedef uint32_t foc_config_apply_executor_state_t;
enum
{
    FOC_CONFIG_EXECUTOR_IDLE = 0,
    FOC_CONFIG_EXECUTOR_RUNNING = 1,
    FOC_CONFIG_EXECUTOR_COMPLETE = 2,
    FOC_CONFIG_EXECUTOR_FAILED = 3,
};

typedef uint32_t foc_config_apply_action_t;
enum
{
    FOC_CONFIG_APPLY_ACTION_NONE = 0,
    FOC_CONFIG_APPLY_ACTION_PLATFORM = 1,
    FOC_CONFIG_APPLY_ACTION_AXIS = 2,
    FOC_CONFIG_APPLY_ACTION_MANAGEMENT = 3,
};

enum
{
    FOC_CONFIG_APPLY_ACTION_MASK_PLATFORM = (1UL << 0),
    FOC_CONFIG_APPLY_ACTION_MASK_AXIS = (1UL << 1),
    FOC_CONFIG_APPLY_ACTION_MASK_MANAGEMENT = (1UL << 2),
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_config_apply_executor_state_t state;
    uint32_t token;
    uint32_t changed_groups;
    uint32_t pending_actions;
    uint32_t completed_actions;
    foc_config_apply_action_t current_action;
    foc_config_apply_action_t failure_action;
    uint32_t failure_detail;
} foc_config_apply_executor_t;

foc_config_apply_executor_status_t foc_config_apply_executor_init(
    foc_config_apply_executor_t *executor);
foc_config_apply_executor_status_t foc_config_apply_executor_begin(
    foc_config_apply_executor_t *executor,
    const foc_config_apply_plan_t *plan);
foc_config_apply_executor_status_t foc_config_apply_executor_next(
    foc_config_apply_executor_t *executor,
    foc_config_apply_action_t *action_out);
foc_config_apply_executor_status_t foc_config_apply_executor_complete(
    foc_config_apply_executor_t *executor,
    foc_config_apply_action_t action,
    uint32_t success,
    uint32_t failure_detail);
foc_config_apply_executor_status_t foc_config_apply_executor_reset(
    foc_config_apply_executor_t *executor,
    const foc_config_apply_guard_t *guard);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_config_apply_executor_t) == 40U,
               "config apply executor layout drifted");
#endif

#endif /* FOC_CONFIG_APPLY_EXECUTOR_H */
