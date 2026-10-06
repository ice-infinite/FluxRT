#include "foc_as5600_alignment_trial.h"

#include <string.h>

static uint32_t foc_as5600_alignment_trial_layout_valid(
    const foc_as5600_alignment_trial_t *trial)
{
    return ((trial != 0) &&
            (trial->struct_size == sizeof(*trial)) &&
            (trial->version == FOC_AS5600_ALIGNMENT_TRIAL_VERSION)) ? 1U : 0U;
}

static uint32_t foc_as5600_alignment_trial_runtime_valid(
    const foc_runtime_config_t *config)
{
    return ((config != 0) &&
            (config->struct_size == sizeof(*config)) &&
            (config->config_version == FOC_RUST_CONFIG_VERSION) &&
            (config->pwm_frequency_hz ==
             FOC_AS5600_ALIGNMENT_TRIAL_CONTROL_HZ) &&
            (config->observer_enable == 0U) &&
            (config->closed_loop_enable == 0U) &&
            (config->startup_alignment_current_a ==
             FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A) &&
            (config->startup_current_a ==
             FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A) &&
            (config->alignment_duration_s ==
             FOC_AS5600_ALIGNMENT_TRIAL_ALIGNMENT_DURATION_S) &&
            (config->rated_current_a >=
             FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A)) ? 1U : 0U;
}

foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_init(
    foc_as5600_alignment_trial_t *trial)
{
    if (trial == 0)
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    (void)memset(trial, 0, sizeof(*trial));
    trial->struct_size = sizeof(*trial);
    trial->version = FOC_AS5600_ALIGNMENT_TRIAL_VERSION;
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_IDLE;
    return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK;
}

foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_prepare(
    foc_as5600_alignment_trial_t *trial,
    const foc_runtime_config_t *runtime_config)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_AS5600_ALIGNMENT_TRIAL_IDLE))
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    if (foc_as5600_alignment_trial_runtime_valid(runtime_config) == 0U)
    {
        trial->state = FOC_AS5600_ALIGNMENT_TRIAL_FAILED;
        trial->result = FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ENVELOPE;
        return trial->result;
    }
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_READY;
    trial->result = FOC_AS5600_ALIGNMENT_TRIAL_RESULT_NONE;
    return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK;
}

foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_mark_armed(
    foc_as5600_alignment_trial_t *trial)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_AS5600_ALIGNMENT_TRIAL_READY))
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_ARMED;
    return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK;
}

foc_as5600_alignment_trial_action_t foc_as5600_alignment_trial_begin_tick(
    foc_as5600_alignment_trial_t *trial)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        ((trial->state != FOC_AS5600_ALIGNMENT_TRIAL_ARMED) &&
         (trial->state != FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE)))
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE;
    }
    if (trial->total_ticks >= FOC_AS5600_ALIGNMENT_TRIAL_MAX_TOTAL_TICKS)
    {
        trial->state = FOC_AS5600_ALIGNMENT_TRIAL_FAILED;
        trial->result = FOC_AS5600_ALIGNMENT_TRIAL_RESULT_TOTAL_TIMEOUT;
        return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP;
    }
    ++trial->total_ticks;
    return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE;
}

foc_as5600_alignment_trial_action_t foc_as5600_alignment_trial_record_commit(
    foc_as5600_alignment_trial_t *trial,
    foc_state_t controller_state)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        ((trial->state != FOC_AS5600_ALIGNMENT_TRIAL_ARMED) &&
         (trial->state != FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE)))
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE;
    }
    if (controller_state != FOC_STATE_ALIGNMENT)
    {
        trial->state = FOC_AS5600_ALIGNMENT_TRIAL_FAILED;
        trial->result =
            FOC_AS5600_ALIGNMENT_TRIAL_RESULT_WRONG_CONTROL_STATE;
        return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP;
    }
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE;
    ++trial->committed_alignment_ticks;
    if (trial->committed_alignment_ticks >=
        FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS)
    {
        trial->state = FOC_AS5600_ALIGNMENT_TRIAL_COMPLETE;
        trial->result = FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK;
        return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_COMPLETE_STOP;
    }
    return FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE;
}

void foc_as5600_alignment_trial_fail(
    foc_as5600_alignment_trial_t *trial,
    foc_as5600_alignment_trial_result_t result,
    uint32_t fault_epoch)
{
    if (foc_as5600_alignment_trial_layout_valid(trial) == 0U)
    {
        return;
    }
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_FAILED;
    trial->result = (result == FOC_AS5600_ALIGNMENT_TRIAL_RESULT_NONE) ?
        FOC_AS5600_ALIGNMENT_TRIAL_RESULT_PLATFORM_FAULT : result;
    trial->fault_epoch = fault_epoch;
}

void foc_as5600_alignment_trial_abort(foc_as5600_alignment_trial_t *trial)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        (trial->state == FOC_AS5600_ALIGNMENT_TRIAL_IDLE) ||
        (trial->state == FOC_AS5600_ALIGNMENT_TRIAL_COMPLETE) ||
        (trial->state == FOC_AS5600_ALIGNMENT_TRIAL_FAILED))
    {
        return;
    }
    trial->state = FOC_AS5600_ALIGNMENT_TRIAL_ABORTED;
    trial->result = FOC_AS5600_ALIGNMENT_TRIAL_RESULT_ABORTED;
}

foc_as5600_alignment_trial_result_t foc_as5600_alignment_trial_get_status(
    const foc_as5600_alignment_trial_t *trial,
    foc_as5600_alignment_trial_status_t *status)
{
    if ((foc_as5600_alignment_trial_layout_valid(trial) == 0U) ||
        (status == 0))
    {
        return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    *status = *trial;
    return FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK;
}
