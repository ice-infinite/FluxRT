#include "foc_motion_torque_trial.h"

#include <string.h>

static uint32_t foc_motion_torque_trial_layout_valid(
    const foc_motion_torque_trial_t *trial)
{
    return ((trial != 0) &&
            (trial->struct_size == sizeof(*trial)) &&
            (trial->version == FOC_MOTION_TORQUE_TRIAL_VERSION)) ? 1U : 0U;
}

static uint32_t foc_motion_torque_trial_command_valid(
    const foc_product_command_t *command)
{
    const uint32_t required_flags = FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT |
                                    FOC_PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
    uint32_t window_ms;

    if ((command == 0) ||
        (command->struct_size != sizeof(*command)) ||
        (command->version != FOC_PRODUCT_COMMAND_VERSION) ||
        (command->axis_id != 0U) ||
        (command->command_kind != FOC_PRODUCT_COMMAND_SETPOINT) ||
        (command->control_mode != FOC_CONTROL_MODE_TORQUE) ||
        (command->input_mode != FOC_INPUT_MODE_TORQUE_RAMP) ||
        (command->feedback_mode != FOC_FEEDBACK_MODE_SENSORLESS) ||
        (command->flags != required_flags) ||
        (command->torque_ref_nm != FOC_MOTION_TORQUE_TRIAL_TORQUE_NM) ||
        (command->current_limit_a !=
         FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A) ||
        (command->torque_limit_nm !=
         FOC_MOTION_TORQUE_TRIAL_TORQUE_LIMIT_NM) ||
        (command->duty_ref != 0.0f) ||
        (command->voltage_d_ref_v != 0.0f) ||
        (command->voltage_q_ref_v != 0.0f) ||
        (command->current_d_ref_a != 0.0f) ||
        (command->current_q_ref_a != 0.0f) ||
        (command->velocity_ref_rad_s != 0.0f) ||
        (command->position_ref_rad != 0.0f) ||
        (command->velocity_feedforward_rad_s != 0.0f) ||
        (command->torque_feedforward_nm != 0.0f) ||
        (command->velocity_limit_rad_s != 0.0f))
    {
        return 0U;
    }
    window_ms = command->valid_until_ms - command->created_at_ms;
    return ((window_ms == FOC_MOTION_TORQUE_TRIAL_COMMAND_WINDOW_MS) &&
            (window_ms < 0x80000000UL)) ? 1U : 0U;
}

static uint32_t foc_motion_torque_trial_runtime_valid(
    const foc_runtime_config_t *config)
{
    if ((config == 0) ||
        (config->struct_size != sizeof(*config)) ||
        (config->config_version != FOC_RUST_CONFIG_VERSION) ||
        (config->closed_loop_enable != 1U) ||
        (config->observer_enable != 1U) ||
        (config->observer_update_divider != 1U) ||
        (config->pwm_frequency_hz != 12000U) ||
        !(config->startup_alignment_current_a > 0.0f) ||
        !(config->startup_current_a > 0.0f) ||
        (config->startup_alignment_current_a >
         FOC_MOTION_TORQUE_TRIAL_STARTUP_CURRENT_A) ||
        (config->startup_current_a >
         FOC_MOTION_TORQUE_TRIAL_STARTUP_CURRENT_A))
    {
        return 0U;
    }
    return 1U;
}

foc_motion_torque_trial_result_t foc_motion_torque_trial_init(
    foc_motion_torque_trial_t *trial)
{
    if (trial == 0)
    {
        return FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    (void)memset(trial, 0, sizeof(*trial));
    trial->struct_size = sizeof(*trial);
    trial->version = FOC_MOTION_TORQUE_TRIAL_VERSION;
    trial->state = FOC_MOTION_TORQUE_TRIAL_IDLE;
    return FOC_MOTION_TORQUE_TRIAL_RESULT_OK;
}

foc_motion_torque_trial_result_t foc_motion_torque_trial_prepare(
    foc_motion_torque_trial_t *trial,
    const foc_product_command_t *command,
    const foc_runtime_config_t *runtime_config)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_MOTION_TORQUE_TRIAL_IDLE))
    {
        return FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    if ((foc_motion_torque_trial_command_valid(command) == 0U) ||
        (foc_motion_torque_trial_runtime_valid(runtime_config) == 0U))
    {
        trial->state = FOC_MOTION_TORQUE_TRIAL_FAILED;
        trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE;
        return trial->result;
    }
    trial->state = FOC_MOTION_TORQUE_TRIAL_READY;
    trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_NONE;
    trial->source_sequence = command->sequence;
    return FOC_MOTION_TORQUE_TRIAL_RESULT_OK;
}

foc_motion_torque_trial_result_t foc_motion_torque_trial_mark_armed(
    foc_motion_torque_trial_t *trial)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) ||
        (trial->state != FOC_MOTION_TORQUE_TRIAL_READY))
    {
        return FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    trial->state = FOC_MOTION_TORQUE_TRIAL_STARTUP;
    return FOC_MOTION_TORQUE_TRIAL_RESULT_OK;
}

foc_motion_torque_trial_action_t foc_motion_torque_trial_begin_tick(
    foc_motion_torque_trial_t *trial)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) ||
        ((trial->state != FOC_MOTION_TORQUE_TRIAL_STARTUP) &&
         (trial->state != FOC_MOTION_TORQUE_TRIAL_ACTIVE)))
    {
        return FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
    }
    if (trial->total_ticks >= FOC_MOTION_TORQUE_TRIAL_MAX_TOTAL_TICKS)
    {
        trial->state = FOC_MOTION_TORQUE_TRIAL_FAILED;
        trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_TOTAL_TIMEOUT;
        return FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP;
    }
    ++trial->total_ticks;
    return FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
}

foc_motion_torque_trial_action_t foc_motion_torque_trial_record_commit(
    foc_motion_torque_trial_t *trial,
    const foc_motion_output_t *output)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) ||
        (output == 0) ||
        ((trial->state != FOC_MOTION_TORQUE_TRIAL_STARTUP) &&
         (trial->state != FOC_MOTION_TORQUE_TRIAL_ACTIVE)))
    {
        return FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
    }

    if (output->control_mode == FOC_CONTROL_MODE_INACTIVE)
    {
        if (trial->state == FOC_MOTION_TORQUE_TRIAL_ACTIVE)
        {
            trial->state = FOC_MOTION_TORQUE_TRIAL_FAILED;
            trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE;
            return FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP;
        }
        return FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
    }
    if ((output->control_mode != FOC_CONTROL_MODE_TORQUE) ||
        (output->input_mode != FOC_INPUT_MODE_TORQUE_RAMP) ||
        (output->source_sequence != trial->source_sequence) ||
        !(output->torque_reference_nm >= 0.0f) ||
        (output->torque_reference_nm >
         FOC_MOTION_TORQUE_TRIAL_TORQUE_LIMIT_NM) ||
        !(output->current_q_reference_a >=
          -FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A) ||
        !(output->current_q_reference_a <=
          FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A))
    {
        trial->state = FOC_MOTION_TORQUE_TRIAL_FAILED;
        trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE;
        return FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP;
    }

    if (trial->state == FOC_MOTION_TORQUE_TRIAL_STARTUP)
    {
        trial->state = FOC_MOTION_TORQUE_TRIAL_ACTIVE;
        trial->first_active_tick = trial->total_ticks;
    }
    ++trial->active_ticks;
    if (trial->active_ticks >= FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS)
    {
        trial->state = FOC_MOTION_TORQUE_TRIAL_COMPLETE;
        trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_OK;
        return FOC_MOTION_TORQUE_TRIAL_ACTION_COMPLETE_STOP;
    }
    return FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
}

void foc_motion_torque_trial_fail(foc_motion_torque_trial_t *trial,
                                  foc_motion_torque_trial_result_t result,
                                  uint32_t fault_epoch)
{
    if (foc_motion_torque_trial_layout_valid(trial) == 0U)
    {
        return;
    }
    trial->state = FOC_MOTION_TORQUE_TRIAL_FAILED;
    trial->result = (result == FOC_MOTION_TORQUE_TRIAL_RESULT_NONE) ?
        FOC_MOTION_TORQUE_TRIAL_RESULT_PLATFORM_FAULT : result;
    trial->fault_epoch = fault_epoch;
}

void foc_motion_torque_trial_abort(foc_motion_torque_trial_t *trial)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) ||
        (trial->state == FOC_MOTION_TORQUE_TRIAL_IDLE) ||
        (trial->state == FOC_MOTION_TORQUE_TRIAL_COMPLETE) ||
        (trial->state == FOC_MOTION_TORQUE_TRIAL_FAILED))
    {
        return;
    }
    trial->state = FOC_MOTION_TORQUE_TRIAL_ABORTED;
    trial->result = FOC_MOTION_TORQUE_TRIAL_RESULT_ABORTED;
}

foc_motion_torque_trial_result_t foc_motion_torque_trial_get_status(
    const foc_motion_torque_trial_t *trial,
    foc_motion_torque_trial_status_t *status)
{
    if ((foc_motion_torque_trial_layout_valid(trial) == 0U) || (status == 0))
    {
        return FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ARGUMENT;
    }
    *status = *trial;
    return FOC_MOTION_TORQUE_TRIAL_RESULT_OK;
}
