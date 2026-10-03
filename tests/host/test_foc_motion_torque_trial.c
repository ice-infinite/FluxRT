#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "foc_motion_torque_trial.h"

static foc_runtime_config_t valid_runtime(void)
{
    foc_runtime_config_t config;
    (void)memset(&config, 0, sizeof(config));
    config.struct_size = sizeof(config);
    config.config_version = FOC_RUST_CONFIG_VERSION;
    config.closed_loop_enable = 1U;
    config.observer_enable = 1U;
    config.observer_update_divider = 1U;
    config.pwm_frequency_hz = 12000U;
    config.startup_alignment_current_a = 0.8f;
    config.startup_current_a = 0.8f;
    return config;
}

static foc_product_command_t valid_command(void)
{
    foc_product_command_t command;
    (void)memset(&command, 0, sizeof(command));
    command.struct_size = sizeof(command);
    command.version = FOC_PRODUCT_COMMAND_VERSION;
    command.source_id = 1U;
    command.sequence = 7U;
    command.created_at_ms = 100U;
    command.valid_until_ms =
        command.created_at_ms + FOC_MOTION_TORQUE_TRIAL_COMMAND_WINDOW_MS;
    command.command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    command.control_mode = FOC_CONTROL_MODE_TORQUE;
    command.input_mode = FOC_INPUT_MODE_TORQUE_RAMP;
    command.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    command.flags = FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT |
                    FOC_PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
    command.torque_ref_nm = FOC_MOTION_TORQUE_TRIAL_TORQUE_NM;
    command.current_limit_a = FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A;
    command.torque_limit_nm = FOC_MOTION_TORQUE_TRIAL_TORQUE_LIMIT_NM;
    return command;
}

static foc_motion_output_t startup_output(void)
{
    foc_motion_output_t output;
    (void)memset(&output, 0, sizeof(output));
    output.struct_size = sizeof(output);
    output.version = FOC_MOTION_OUTPUT_VERSION;
    output.control_mode = FOC_CONTROL_MODE_INACTIVE;
    output.input_mode = FOC_INPUT_MODE_INACTIVE;
    return output;
}

static foc_motion_output_t torque_output(void)
{
    foc_motion_output_t output = startup_output();
    output.source_sequence = 7U;
    output.control_mode = FOC_CONTROL_MODE_TORQUE;
    output.input_mode = FOC_INPUT_MODE_TORQUE_RAMP;
    output.torque_reference_nm = FOC_MOTION_TORQUE_TRIAL_TORQUE_NM;
    output.current_q_reference_a = 0.0345f;
    return output;
}

static void test_exact_envelope_and_hard_stop(void)
{
    foc_motion_torque_trial_t trial;
    foc_motion_torque_trial_status_t status;
    foc_runtime_config_t config = valid_runtime();
    foc_product_command_t command = valid_command();
    foc_motion_output_t output = startup_output();
    uint32_t index;

    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_mark_armed(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    for (index = 0U; index < 1234U; ++index)
    {
        assert(foc_motion_torque_trial_begin_tick(&trial) ==
               FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
        assert(foc_motion_torque_trial_record_commit(&trial, &output) ==
               FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
    }
    output = torque_output();
    for (index = 0U; index < FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS; ++index)
    {
        assert(foc_motion_torque_trial_begin_tick(&trial) ==
               FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
        if ((index + 1U) == FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS)
        {
            assert(foc_motion_torque_trial_record_commit(&trial, &output) ==
                   FOC_MOTION_TORQUE_TRIAL_ACTION_COMPLETE_STOP);
        }
        else
        {
            assert(foc_motion_torque_trial_record_commit(&trial, &output) ==
                   FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
        }
    }
    assert(foc_motion_torque_trial_get_status(&trial, &status) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(status.state == FOC_MOTION_TORQUE_TRIAL_COMPLETE);
    assert(status.result == FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(status.active_ticks == FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS);
    assert(status.first_active_tick == 1235U);
    assert(status.total_ticks == 1234U + FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS);
}

static void test_invalid_envelopes_fail_closed(void)
{
    foc_motion_torque_trial_t trial;
    foc_runtime_config_t config = valid_runtime();
    foc_product_command_t command = valid_command();
    foc_motion_output_t output;

    config.closed_loop_enable = 0U;
    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE);
    assert(trial.state == FOC_MOTION_TORQUE_TRIAL_FAILED);

    config = valid_runtime();
    command.current_limit_a = 0.081f;
    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE);

    command = valid_command();
    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_mark_armed(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_begin_tick(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
    output = torque_output();
    output.current_q_reference_a = 0.081f;
    assert(foc_motion_torque_trial_record_commit(&trial, &output) ==
           FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result == FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE);
}

static void test_total_timeout_fault_and_abort(void)
{
    foc_motion_torque_trial_t trial;
    foc_runtime_config_t config = valid_runtime();
    foc_product_command_t command = valid_command();
    uint32_t index;

    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_mark_armed(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    for (index = 0U; index < FOC_MOTION_TORQUE_TRIAL_MAX_TOTAL_TICKS; ++index)
    {
        assert(foc_motion_torque_trial_begin_tick(&trial) ==
               FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE);
    }
    assert(foc_motion_torque_trial_begin_tick(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP);
    assert(trial.state == FOC_MOTION_TORQUE_TRIAL_FAILED);
    assert(trial.result == FOC_MOTION_TORQUE_TRIAL_RESULT_TOTAL_TIMEOUT);

    assert(foc_motion_torque_trial_init(&trial) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    assert(foc_motion_torque_trial_prepare(&trial, &command, &config) ==
           FOC_MOTION_TORQUE_TRIAL_RESULT_OK);
    foc_motion_torque_trial_abort(&trial);
    assert(trial.state == FOC_MOTION_TORQUE_TRIAL_ABORTED);
    assert(trial.result == FOC_MOTION_TORQUE_TRIAL_RESULT_ABORTED);
}

int main(void)
{
    test_exact_envelope_and_hard_stop();
    test_invalid_envelopes_fail_closed();
    test_total_timeout_fault_and_abort();
    puts("foc_motion_torque_trial_tests: PASS");
    return 0;
}
