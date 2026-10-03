#include "foc_config_apply_executor.h"

#include <string.h>

#define FOC_CONFIG_PLATFORM_GROUPS \
    (FOC_CONFIG_GROUP_BOARD | FOC_CONFIG_GROUP_INVERTER | \
     FOC_CONFIG_GROUP_CALIBRATION)
#define FOC_CONFIG_AXIS_GROUPS \
    (FOC_CONFIG_GROUP_MOTOR | FOC_CONFIG_GROUP_AXIS)

static uint32_t foc_config_expected_class(uint32_t groups)
{
    if ((groups & FOC_CONFIG_PLATFORM_GROUPS) != 0U)
    {
        return FOC_CONFIG_APPLY_PLATFORM_RESTART;
    }
    if ((groups & FOC_CONFIG_AXIS_GROUPS) != 0U)
    {
        return FOC_CONFIG_APPLY_AXIS_RESTART;
    }
    return FOC_CONFIG_APPLY_MANAGEMENT_ONLY;
}

static uint32_t foc_config_actions(uint32_t groups)
{
    uint32_t actions = 0U;
    if ((groups & FOC_CONFIG_PLATFORM_GROUPS) != 0U)
    {
        actions |= FOC_CONFIG_APPLY_ACTION_MASK_PLATFORM |
                   FOC_CONFIG_APPLY_ACTION_MASK_AXIS;
    }
    else if ((groups & FOC_CONFIG_AXIS_GROUPS) != 0U)
    {
        actions |= FOC_CONFIG_APPLY_ACTION_MASK_AXIS;
    }
    if ((groups & (FOC_CONFIG_GROUP_APP |
                   FOC_CONFIG_GROUP_EXTERNAL_IO)) != 0U)
    {
        actions |= FOC_CONFIG_APPLY_ACTION_MASK_MANAGEMENT;
    }
    return actions;
}

static uint32_t foc_config_action_mask(foc_config_apply_action_t action)
{
    switch (action)
    {
    case FOC_CONFIG_APPLY_ACTION_PLATFORM:
        return FOC_CONFIG_APPLY_ACTION_MASK_PLATFORM;
    case FOC_CONFIG_APPLY_ACTION_AXIS:
        return FOC_CONFIG_APPLY_ACTION_MASK_AXIS;
    case FOC_CONFIG_APPLY_ACTION_MANAGEMENT:
        return FOC_CONFIG_APPLY_ACTION_MASK_MANAGEMENT;
    default:
        return 0U;
    }
}

static uint32_t foc_config_guard_is_safe(const foc_config_apply_guard_t *guard)
{
    return ((guard != 0) &&
            (guard->struct_size == sizeof(*guard)) &&
            (guard->abi_version == FOC_CONFIG_ABI_VERSION) &&
            (guard->axis_state == FOC_AXIS_STATE_DISABLED) &&
            (guard->drive_active == 0U) &&
            (guard->active_fault_flags == 0U)) ? 1U : 0U;
}

foc_config_apply_executor_status_t foc_config_apply_executor_init(
    foc_config_apply_executor_t *executor)
{
    if (executor == 0)
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    memset(executor, 0, sizeof(*executor));
    executor->struct_size = sizeof(*executor);
    executor->version = FOC_CONFIG_APPLY_EXECUTOR_VERSION;
    executor->state = FOC_CONFIG_EXECUTOR_IDLE;
    return FOC_CONFIG_EXECUTOR_STATUS_OK;
}

foc_config_apply_executor_status_t foc_config_apply_executor_begin(
    foc_config_apply_executor_t *executor,
    const foc_config_apply_plan_t *plan)
{
    uint32_t actions;
    if ((executor == 0) ||
        (executor->struct_size != sizeof(*executor)) ||
        (executor->version != FOC_CONFIG_APPLY_EXECUTOR_VERSION) ||
        (executor->state != FOC_CONFIG_EXECUTOR_IDLE) ||
        (plan == 0) ||
        (plan->struct_size != sizeof(*plan)) ||
        (plan->abi_version != FOC_CONFIG_ABI_VERSION) ||
        (plan->token == 0U) ||
        (plan->from_revision == 0U) ||
        (plan->to_revision == 0U) ||
        (plan->from_revision == plan->to_revision) ||
        (plan->changed_groups == 0U) ||
        ((plan->changed_groups & ~(uint32_t)FOC_CONFIG_GROUP_KNOWN_MASK) != 0U) ||
        (plan->apply_class != foc_config_expected_class(plan->changed_groups)) ||
        (plan->reserved != 0U))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    actions = foc_config_actions(plan->changed_groups);
    if (actions == 0U)
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    executor->state = FOC_CONFIG_EXECUTOR_RUNNING;
    executor->token = plan->token;
    executor->changed_groups = plan->changed_groups;
    executor->pending_actions = actions;
    executor->completed_actions = 0U;
    executor->current_action = FOC_CONFIG_APPLY_ACTION_NONE;
    executor->failure_action = FOC_CONFIG_APPLY_ACTION_NONE;
    executor->failure_detail = 0U;
    return FOC_CONFIG_EXECUTOR_STATUS_OK;
}

foc_config_apply_executor_status_t foc_config_apply_executor_next(
    foc_config_apply_executor_t *executor,
    foc_config_apply_action_t *action_out)
{
    foc_config_apply_action_t action = FOC_CONFIG_APPLY_ACTION_NONE;
    if ((executor == 0) || (action_out == 0) ||
        (executor->struct_size != sizeof(*executor)) ||
        (executor->version != FOC_CONFIG_APPLY_EXECUTOR_VERSION))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    *action_out = FOC_CONFIG_APPLY_ACTION_NONE;
    if ((executor->state != FOC_CONFIG_EXECUTOR_RUNNING) ||
        (executor->current_action != FOC_CONFIG_APPLY_ACTION_NONE))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE;
    }
    if ((executor->pending_actions & FOC_CONFIG_APPLY_ACTION_MASK_PLATFORM) != 0U)
    {
        action = FOC_CONFIG_APPLY_ACTION_PLATFORM;
    }
    else if ((executor->pending_actions & FOC_CONFIG_APPLY_ACTION_MASK_AXIS) != 0U)
    {
        action = FOC_CONFIG_APPLY_ACTION_AXIS;
    }
    else if ((executor->pending_actions & FOC_CONFIG_APPLY_ACTION_MASK_MANAGEMENT) != 0U)
    {
        action = FOC_CONFIG_APPLY_ACTION_MANAGEMENT;
    }
    else
    {
        executor->state = FOC_CONFIG_EXECUTOR_COMPLETE;
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE;
    }
    executor->current_action = action;
    *action_out = action;
    return FOC_CONFIG_EXECUTOR_STATUS_OK;
}

foc_config_apply_executor_status_t foc_config_apply_executor_complete(
    foc_config_apply_executor_t *executor,
    foc_config_apply_action_t action,
    uint32_t success,
    uint32_t failure_detail)
{
    uint32_t mask;
    if ((executor == 0) ||
        (executor->struct_size != sizeof(*executor)) ||
        (executor->version != FOC_CONFIG_APPLY_EXECUTOR_VERSION) ||
        (success > 1U))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    mask = foc_config_action_mask(action);
    if ((executor->state != FOC_CONFIG_EXECUTOR_RUNNING) ||
        (mask == 0U) ||
        (executor->current_action != action) ||
        ((executor->pending_actions & mask) == 0U))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE;
    }
    if (success == 0U)
    {
        executor->state = FOC_CONFIG_EXECUTOR_FAILED;
        executor->failure_action = action;
        executor->failure_detail = failure_detail;
        executor->current_action = FOC_CONFIG_APPLY_ACTION_NONE;
        return FOC_CONFIG_EXECUTOR_STATUS_ACTION_FAILED;
    }
    executor->pending_actions &= ~mask;
    executor->completed_actions |= mask;
    executor->current_action = FOC_CONFIG_APPLY_ACTION_NONE;
    if (executor->pending_actions == 0U)
    {
        executor->state = FOC_CONFIG_EXECUTOR_COMPLETE;
    }
    return FOC_CONFIG_EXECUTOR_STATUS_OK;
}

foc_config_apply_executor_status_t foc_config_apply_executor_reset(
    foc_config_apply_executor_t *executor,
    const foc_config_apply_guard_t *guard)
{
    if ((executor == 0) ||
        (executor->struct_size != sizeof(*executor)) ||
        (executor->version != FOC_CONFIG_APPLY_EXECUTOR_VERSION) ||
        (foc_config_guard_is_safe(guard) == 0U))
    {
        return FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT;
    }
    return foc_config_apply_executor_init(executor);
}
