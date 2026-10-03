#include "foc_config_apply_executor.h"

#include <assert.h>
#include <stdio.h>

static foc_config_apply_plan_t plan(uint32_t groups, uint32_t apply_class)
{
    foc_config_apply_plan_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.token = 7U;
    value.from_revision = 10U;
    value.to_revision = 11U;
    value.changed_groups = groups;
    value.apply_class = apply_class;
    return value;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t value = {0};
    value.struct_size = sizeof(value);
    value.abi_version = FOC_CONFIG_ABI_VERSION;
    value.axis_state = FOC_AXIS_STATE_DISABLED;
    return value;
}

static void expect_action(foc_config_apply_executor_t *executor,
                          foc_config_apply_action_t expected)
{
    foc_config_apply_action_t action = FOC_CONFIG_APPLY_ACTION_NONE;
    assert(foc_config_apply_executor_next(executor, &action) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(action == expected);
    assert(foc_config_apply_executor_complete(executor, action, 1U, 0U) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
}

static void test_management_only(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_APP, FOC_CONFIG_APPLY_MANAGEMENT_ONLY);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_MANAGEMENT);
    assert(executor.state == FOC_CONFIG_EXECUTOR_COMPLETE);
}

static void test_external_io_is_management_only(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_EXTERNAL_IO, FOC_CONFIG_APPLY_MANAGEMENT_ONLY);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_MANAGEMENT);
    assert(executor.state == FOC_CONFIG_EXECUTOR_COMPLETE);
}

static void test_axis_restart(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_MOTOR | FOC_CONFIG_GROUP_AXIS,
             FOC_CONFIG_APPLY_AXIS_RESTART);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_AXIS);
    assert(executor.state == FOC_CONFIG_EXECUTOR_COMPLETE);
}

static void test_platform_axis_management_order(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_BOARD | FOC_CONFIG_GROUP_MOTOR |
                 FOC_CONFIG_GROUP_APP | FOC_CONFIG_GROUP_CALIBRATION,
             FOC_CONFIG_APPLY_PLATFORM_RESTART);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_PLATFORM);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_AXIS);
    expect_action(&executor, FOC_CONFIG_APPLY_ACTION_MANAGEMENT);
    assert(executor.state == FOC_CONFIG_EXECUTOR_COMPLETE);
    assert(executor.completed_actions ==
           (FOC_CONFIG_APPLY_ACTION_MASK_PLATFORM |
            FOC_CONFIG_APPLY_ACTION_MASK_AXIS |
            FOC_CONFIG_APPLY_ACTION_MASK_MANAGEMENT));
}

static void test_bad_plan_and_out_of_order_completion(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_action_t action;
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_MOTOR, FOC_CONFIG_APPLY_MANAGEMENT_ONLY);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT);
    value = plan(FOC_CONFIG_GROUP_KNOWN_MASK | (1UL << 20),
                 FOC_CONFIG_APPLY_PLATFORM_RESTART);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT);
    value = plan(FOC_CONFIG_GROUP_INVERTER, FOC_CONFIG_APPLY_PLATFORM_RESTART);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_next(&executor, &action) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(action == FOC_CONFIG_APPLY_ACTION_PLATFORM);
    assert(foc_config_apply_executor_complete(
               &executor, FOC_CONFIG_APPLY_ACTION_AXIS, 1U, 0U) ==
           FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE);
    assert(executor.current_action == FOC_CONFIG_APPLY_ACTION_PLATFORM);
}

static void test_failure_latches_until_safe_reset(void)
{
    foc_config_apply_executor_t executor;
    foc_config_apply_action_t action;
    foc_config_apply_guard_t guard = safe_guard();
    foc_config_apply_plan_t value =
        plan(FOC_CONFIG_GROUP_AXIS, FOC_CONFIG_APPLY_AXIS_RESTART);
    assert(foc_config_apply_executor_init(&executor) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_begin(&executor, &value) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_next(&executor, &action) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(foc_config_apply_executor_complete(&executor, action, 0U, 0x42U) ==
           FOC_CONFIG_EXECUTOR_STATUS_ACTION_FAILED);
    assert(executor.state == FOC_CONFIG_EXECUTOR_FAILED);
    assert(executor.failure_action == FOC_CONFIG_APPLY_ACTION_AXIS);
    assert(executor.failure_detail == 0x42U);
    assert(foc_config_apply_executor_next(&executor, &action) ==
           FOC_CONFIG_EXECUTOR_STATUS_INVALID_STATE);
    guard.drive_active = 1U;
    assert(foc_config_apply_executor_reset(&executor, &guard) ==
           FOC_CONFIG_EXECUTOR_STATUS_INVALID_ARGUMENT);
    guard.drive_active = 0U;
    assert(foc_config_apply_executor_reset(&executor, &guard) ==
           FOC_CONFIG_EXECUTOR_STATUS_OK);
    assert(executor.state == FOC_CONFIG_EXECUTOR_IDLE);
}

int main(void)
{
    test_management_only();
    test_external_io_is_management_only();
    test_axis_restart();
    test_platform_axis_management_order();
    test_bad_plan_and_out_of_order_completion();
    test_failure_latches_until_safe_reset();
    puts("foc config apply executor tests passed");
    return 0;
}
