#include "foc_advanced_power_trial.h"

#include <string.h>

static uint32_t foc_advanced_power_trial_layout_valid(
    const foc_advanced_power_trial_t *trial)
{
    return ((trial != 0) &&
            (trial->struct_size == sizeof(*trial)) &&
            (trial->version == FOC_ADVANCED_POWER_TRIAL_VERSION)) ? 1U : 0U;
}

static uint32_t foc_advanced_power_trial_runtime_valid(
    const foc_runtime_config_t *config)
{
    return ((config != 0) &&
            (config->struct_size == sizeof(*config)) &&
            (config->config_version == FOC_RUST_CONFIG_VERSION) &&
            (config->closed_loop_enable == 1U) &&
            (config->observer_enable == 1U) &&
            (config->observer_update_divider == 1U) &&
            (config->pwm_frequency_hz == 12000U) &&
            (config->rated_current_a > 0.0f) &&
            (config->rated_current_a <= 0.8f) &&
            (config->startup_alignment_current_a > 0.0f) &&
            (config->startup_alignment_current_a <= 0.8f) &&
            (config->startup_current_a > 0.0f) &&
            (config->startup_current_a <= 0.8f) &&
            (config->startup_final_speed_rpm ==
             FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM)) ? 1U : 0U;
}

static uint32_t foc_advanced_power_trial_config_valid(
    foc_advanced_power_trial_mode_t mode,
    const foc_runtime_config_t *runtime_config,
    const foc_advanced_runtime_config_t *config)
{
    uint32_t expected_features;

    if ((config == 0) || (runtime_config == 0) ||
        (config->struct_size != sizeof(*config)) ||
        (config->abi_version != FOC_ADVANCED_ABI_VERSION) ||
        (config->algorithm.struct_size != sizeof(config->algorithm)) ||
        (config->algorithm.version != FOC_ADVANCED_CONFIG_VERSION) ||
        (config->platform_capabilities != 0U) ||
        (config->minimum_duty != 0.03f) ||
        (config->maximum_duty != 0.97f) ||
        (config->algorithm.current_limit_a !=
         runtime_config->rated_current_a))
    {
        return 0U;
    }
    if (mode == FOC_ADVANCED_POWER_TRIAL_MODE_BASIC)
    {
        expected_features = 0U;
    }
    else if (mode == FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING)
    {
        expected_features = FOC_ADVANCED_FEATURE_DECOUPLING;
    }
    else
    {
        return 0U;
    }
    return (config->algorithm.enabled_features == expected_features) ? 1U : 0U;
}

static foc_advanced_power_trial_action_t foc_advanced_power_trial_stop(
    foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_result_t result)
{
    trial->state = FOC_ADVANCED_POWER_TRIAL_FAILED;
    trial->result = result;
    return FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP;
}

foc_advanced_power_trial_result_t foc_advanced_power_trial_init(
    foc_advanced_power_trial_t *trial)
{
    if (trial == 0)
    {
        return FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    (void)memset(trial, 0, sizeof(*trial));
    trial->struct_size = sizeof(*trial);
    trial->version = FOC_ADVANCED_POWER_TRIAL_VERSION;
    trial->state = FOC_ADVANCED_POWER_TRIAL_IDLE;
    return FOC_ADVANCED_POWER_TRIAL_RESULT_OK;
}

foc_advanced_power_trial_result_t foc_advanced_power_trial_prepare(
    foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_mode_t mode,
    const foc_runtime_config_t *runtime_config,
    const foc_advanced_runtime_config_t *advanced_config,
    uint32_t expected_fault_epoch,
    uint32_t initial_deadline_miss_count)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_ADVANCED_POWER_TRIAL_IDLE))
    {
        return FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    if ((foc_advanced_power_trial_runtime_valid(runtime_config) == 0U) ||
        (foc_advanced_power_trial_config_valid(mode,
                                               runtime_config,
                                               advanced_config) == 0U))
    {
        trial->state = FOC_ADVANCED_POWER_TRIAL_FAILED;
        trial->result = FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE;
        return trial->result;
    }
    trial->state = FOC_ADVANCED_POWER_TRIAL_READY;
    trial->result = FOC_ADVANCED_POWER_TRIAL_RESULT_NONE;
    trial->mode = mode;
    trial->expected_fault_epoch = expected_fault_epoch;
    trial->observed_fault_epoch = expected_fault_epoch;
    trial->initial_deadline_miss_count = initial_deadline_miss_count;
    trial->observed_deadline_miss_count = initial_deadline_miss_count;
    return FOC_ADVANCED_POWER_TRIAL_RESULT_OK;
}

foc_advanced_power_trial_result_t foc_advanced_power_trial_mark_armed(
    foc_advanced_power_trial_t *trial)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_ADVANCED_POWER_TRIAL_READY))
    {
        return FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    trial->state = FOC_ADVANCED_POWER_TRIAL_STARTUP;
    return FOC_ADVANCED_POWER_TRIAL_RESULT_OK;
}

foc_advanced_power_trial_action_t foc_advanced_power_trial_begin_tick(
    foc_advanced_power_trial_t *trial,
    uint32_t fault_epoch,
    uint32_t deadline_miss_count)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        ((trial->state != FOC_ADVANCED_POWER_TRIAL_STARTUP) &&
         (trial->state != FOC_ADVANCED_POWER_TRIAL_ACTIVE)))
    {
        return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
    }
    trial->observed_fault_epoch = fault_epoch;
    trial->observed_deadline_miss_count = deadline_miss_count;
    if (fault_epoch != trial->expected_fault_epoch)
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_FAULT_EPOCH_CHANGED);
    }
    if (deadline_miss_count != trial->initial_deadline_miss_count)
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_DEADLINE_MISSED);
    }
    if (trial->total_ticks >= FOC_ADVANCED_POWER_TRIAL_MAX_TOTAL_TICKS)
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_TOTAL_TIMEOUT);
    }
    ++trial->total_ticks;
    return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
}

foc_advanced_power_trial_action_t foc_advanced_power_trial_validate_control(
    foc_advanced_power_trial_t *trial,
    const foc_advanced_power_trial_snapshot_t *snapshot)
{
    const uint32_t forbidden_status = FOC_ADVANCED_STATUS_FAULTED |
                                      FOC_ADVANCED_STATUS_CONFIG_REJECTED;
    const uint32_t expected_features =
        (trial != 0 &&
         trial->mode == FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING) ?
            FOC_ADVANCED_FEATURE_DECOUPLING : 0U;

    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        ((trial->state != FOC_ADVANCED_POWER_TRIAL_STARTUP) &&
         (trial->state != FOC_ADVANCED_POWER_TRIAL_ACTIVE)))
    {
        return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
    }
    if ((snapshot == 0) ||
        (snapshot->struct_size != sizeof(*snapshot)) ||
        (snapshot->version != FOC_ADVANCED_POWER_TRIAL_SNAPSHOT_VERSION))
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE);
    }
    trial->last_advanced_status_flags = snapshot->status_flags;
    trial->last_active_features = snapshot->active_features;
    if ((snapshot->state == FOC_STATE_FAULT) ||
        ((snapshot->status_flags & forbidden_status) != 0U))
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_ADVANCED_FAULT);
    }
    if ((snapshot->closed_loop_active == 0U) ||
        (snapshot->observer_reliable == 0U))
    {
        if (trial->state == FOC_ADVANCED_POWER_TRIAL_ACTIVE)
        {
            return foc_advanced_power_trial_stop(
                trial, FOC_ADVANCED_POWER_TRIAL_RESULT_OBSERVER_FALLBACK);
        }
        return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
    }
    if (expected_features == 0U)
    {
        if ((snapshot->active_features != 0U) ||
            (snapshot->status_flags != 0U))
        {
            return foc_advanced_power_trial_stop(
                trial, FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE);
        }
    }
    else if ((snapshot->active_features != expected_features) ||
             ((snapshot->status_flags &
               (FOC_ADVANCED_STATUS_CONFIGURED |
                FOC_ADVANCED_STATUS_BASIC_FALLBACK)) !=
              FOC_ADVANCED_STATUS_CONFIGURED))
    {
        return foc_advanced_power_trial_stop(
            trial, FOC_ADVANCED_POWER_TRIAL_RESULT_OBSERVER_FALLBACK);
    }

    if (trial->state == FOC_ADVANCED_POWER_TRIAL_STARTUP)
    {
        trial->state = FOC_ADVANCED_POWER_TRIAL_ACTIVE;
        trial->first_active_tick = trial->total_ticks;
    }
    return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
}

foc_advanced_power_trial_action_t foc_advanced_power_trial_record_commit(
    foc_advanced_power_trial_t *trial)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_ADVANCED_POWER_TRIAL_ACTIVE))
    {
        return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
    }
    ++trial->active_ticks;
    if (trial->active_ticks >= FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS)
    {
        trial->state = FOC_ADVANCED_POWER_TRIAL_COMPLETE;
        trial->result = FOC_ADVANCED_POWER_TRIAL_RESULT_OK;
        return FOC_ADVANCED_POWER_TRIAL_ACTION_COMPLETE_STOP;
    }
    return FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
}

void foc_advanced_power_trial_fail(
    foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_result_t result,
    uint32_t fault_epoch,
    uint32_t deadline_miss_count)
{
    if (foc_advanced_power_trial_layout_valid(trial) == 0U)
    {
        return;
    }
    if ((trial->state != FOC_ADVANCED_POWER_TRIAL_READY) &&
        (trial->state != FOC_ADVANCED_POWER_TRIAL_STARTUP) &&
        (trial->state != FOC_ADVANCED_POWER_TRIAL_ACTIVE))
    {
        return;
    }
    trial->state = FOC_ADVANCED_POWER_TRIAL_FAILED;
    trial->result = (result == FOC_ADVANCED_POWER_TRIAL_RESULT_NONE) ?
        FOC_ADVANCED_POWER_TRIAL_RESULT_PLATFORM_FAULT : result;
    trial->observed_fault_epoch = fault_epoch;
    trial->observed_deadline_miss_count = deadline_miss_count;
}

void foc_advanced_power_trial_abort(foc_advanced_power_trial_t *trial)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) ||
        (trial->state == FOC_ADVANCED_POWER_TRIAL_IDLE) ||
        (trial->state == FOC_ADVANCED_POWER_TRIAL_COMPLETE) ||
        (trial->state == FOC_ADVANCED_POWER_TRIAL_FAILED))
    {
        return;
    }
    trial->state = FOC_ADVANCED_POWER_TRIAL_ABORTED;
    trial->result = FOC_ADVANCED_POWER_TRIAL_RESULT_ABORTED;
}

foc_advanced_power_trial_result_t foc_advanced_power_trial_get_status(
    const foc_advanced_power_trial_t *trial,
    foc_advanced_power_trial_status_t *status)
{
    if ((foc_advanced_power_trial_layout_valid(trial) == 0U) || (status == 0))
    {
        return FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    *status = *trial;
    return FOC_ADVANCED_POWER_TRIAL_RESULT_OK;
}
