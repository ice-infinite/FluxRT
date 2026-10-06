#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "foc_as5600_alignment_trial.h"

static foc_runtime_config_t valid_runtime(void)
{
    foc_runtime_config_t config;
    (void)memset(&config, 0, sizeof(config));
    config.struct_size = sizeof(config);
    config.config_version = FOC_RUST_CONFIG_VERSION;
    config.pwm_frequency_hz = FOC_AS5600_ALIGNMENT_TRIAL_CONTROL_HZ;
    config.observer_enable = 0U;
    config.closed_loop_enable = 0U;
    config.startup_alignment_current_a =
        FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A;
    config.startup_current_a = FOC_AS5600_ALIGNMENT_TRIAL_CURRENT_A;
    config.alignment_duration_s =
        FOC_AS5600_ALIGNMENT_TRIAL_ALIGNMENT_DURATION_S;
    config.rated_current_a = 0.8f;
    return config;
}

static void test_exact_envelope_and_hard_stop(void)
{
    foc_as5600_alignment_trial_t trial;
    foc_as5600_alignment_trial_status_t status;
    foc_runtime_config_t config = valid_runtime();
    uint32_t index;

    assert(foc_as5600_alignment_trial_init(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_mark_armed(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    for (index = 0U;
         index < FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS;
         ++index)
    {
        assert(foc_as5600_alignment_trial_begin_tick(&trial) ==
               FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE);
        assert(foc_as5600_alignment_trial_record_commit(
                   &trial, FOC_STATE_ALIGNMENT) ==
               (((index + 1U) == FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS) ?
                    FOC_AS5600_ALIGNMENT_TRIAL_ACTION_COMPLETE_STOP :
                    FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE));
    }
    assert(foc_as5600_alignment_trial_get_status(&trial, &status) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(status.state == FOC_AS5600_ALIGNMENT_TRIAL_COMPLETE);
    assert(status.result == FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(status.total_ticks == FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS);
    assert(status.committed_alignment_ticks ==
           FOC_AS5600_ALIGNMENT_TRIAL_ACTIVE_TICKS);
}

static void test_wrong_state_and_relaxed_envelope_fail(void)
{
    foc_as5600_alignment_trial_t trial;
    foc_runtime_config_t config = valid_runtime();

    config.startup_alignment_current_a = 0.201f;
    assert(foc_as5600_alignment_trial_init(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ENVELOPE);

    config = valid_runtime();
    assert(foc_as5600_alignment_trial_init(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_mark_armed(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_begin_tick(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE);
    assert(foc_as5600_alignment_trial_record_commit(
               &trial, FOC_STATE_OPEN_LOOP_RAMP) ==
           FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_WRONG_CONTROL_STATE);
}

static void test_timeout_abort_and_one_shot_state(void)
{
    foc_as5600_alignment_trial_t trial;
    foc_runtime_config_t config = valid_runtime();
    uint32_t index;

    assert(foc_as5600_alignment_trial_init(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_mark_armed(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    for (index = 0U;
         index < FOC_AS5600_ALIGNMENT_TRIAL_MAX_TOTAL_TICKS;
         ++index)
    {
        assert(foc_as5600_alignment_trial_begin_tick(&trial) ==
               FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE);
    }
    assert(foc_as5600_alignment_trial_begin_tick(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_TOTAL_TIMEOUT);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_INVALID_ARGUMENT);

    assert(foc_as5600_alignment_trial_init(&trial) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    assert(foc_as5600_alignment_trial_prepare(&trial, &config) ==
           FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK);
    foc_as5600_alignment_trial_abort(&trial);
    assert(trial.state == FOC_AS5600_ALIGNMENT_TRIAL_ABORTED);
    assert(trial.result == FOC_AS5600_ALIGNMENT_TRIAL_RESULT_ABORTED);
}

int main(void)
{
    test_exact_envelope_and_hard_stop();
    test_wrong_state_and_relaxed_envelope_fail();
    test_timeout_abort_and_one_shot_state();
    puts("foc_as5600_alignment_trial_tests: PASS");
    return 0;
}
