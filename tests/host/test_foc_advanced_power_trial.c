#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "foc_advanced_power_trial.h"

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
    config.rated_current_a = 0.8f;
    config.startup_alignment_current_a = 0.8f;
    config.startup_current_a = 0.8f;
    config.startup_final_speed_rpm =
        FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM;
    config.default_target_speed_rpm =
        FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM;
    return config;
}

static foc_advanced_runtime_config_t valid_advanced(uint32_t features)
{
    foc_advanced_runtime_config_t config;
    (void)memset(&config, 0, sizeof(config));
    config.struct_size = sizeof(config);
    config.abi_version = FOC_ADVANCED_ABI_VERSION;
    config.minimum_duty = 0.03f;
    config.maximum_duty = 0.97f;
    config.algorithm.struct_size = sizeof(config.algorithm);
    config.algorithm.version = FOC_ADVANCED_CONFIG_VERSION;
    config.algorithm.enabled_features = features;
    config.algorithm.current_limit_a = 0.8f;
    return config;
}

static foc_telemetry_t startup_telemetry(void)
{
    foc_telemetry_t telemetry;
    (void)memset(&telemetry, 0, sizeof(telemetry));
    telemetry.state = FOC_STATE_ALIGNMENT;
    return telemetry;
}

static foc_telemetry_t closed_loop_telemetry(void)
{
    foc_telemetry_t telemetry = startup_telemetry();
    telemetry.state = FOC_STATE_CLOSED_LOOP;
    telemetry.closed_loop_active = 1U;
    telemetry.observer_reliable = 1U;
    return telemetry;
}

static foc_advanced_telemetry_t advanced_telemetry(
    uint32_t features,
    uint32_t flags)
{
    foc_advanced_telemetry_t telemetry;
    (void)memset(&telemetry, 0, sizeof(telemetry));
    telemetry.struct_size = sizeof(telemetry);
    telemetry.abi_version = FOC_ADVANCED_ABI_VERSION;
    telemetry.active_features = features;
    telemetry.status_flags = flags;
    return telemetry;
}

static void prepare_and_arm(foc_advanced_power_trial_t *trial,
                            foc_advanced_power_trial_mode_t mode)
{
    foc_runtime_config_t runtime = valid_runtime();
    const uint32_t features =
        (mode == FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING) ?
            FOC_ADVANCED_FEATURE_DECOUPLING : 0U;
    foc_advanced_runtime_config_t advanced = valid_advanced(features);

    assert(foc_advanced_power_trial_init(trial) ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OK);
    assert(foc_advanced_power_trial_prepare(trial,
                                            mode,
                                            &runtime,
                                            &advanced,
                                            7U,
                                            2U) ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OK);
    assert(foc_advanced_power_trial_mark_armed(trial) ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OK);
}

static void test_basic_and_decoupling_complete_at_exact_window(void)
{
    foc_advanced_power_trial_t trial;
    foc_telemetry_t startup = startup_telemetry();
    foc_telemetry_t closed = closed_loop_telemetry();
    foc_advanced_telemetry_t basic = advanced_telemetry(0U, 0U);
    foc_advanced_telemetry_t decoupling = advanced_telemetry(
        FOC_ADVANCED_FEATURE_DECOUPLING,
        FOC_ADVANCED_STATUS_CONFIGURED);
    uint32_t index;

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_BASIC);
    for (index = 0U; index < 123U; ++index)
    {
        assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
               FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
        assert(foc_advanced_power_trial_validate_control(
                   &trial, &startup, &basic) ==
               FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    }
    for (index = 0U; index < FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS; ++index)
    {
        assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
               FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
        assert(foc_advanced_power_trial_validate_control(
                   &trial, &closed, &basic) ==
               FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
        assert(foc_advanced_power_trial_record_commit(&trial) ==
               (((index + 1U) == FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS) ?
                    FOC_ADVANCED_POWER_TRIAL_ACTION_COMPLETE_STOP :
                    FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE));
    }
    assert(trial.state == FOC_ADVANCED_POWER_TRIAL_COMPLETE);
    assert(trial.active_ticks == FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS);
    assert(trial.first_active_tick == 124U);

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING);
    assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(foc_advanced_power_trial_validate_control(
               &trial, &closed, &decoupling) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(trial.state == FOC_ADVANCED_POWER_TRIAL_ACTIVE);
    assert(trial.active_ticks == 0U);
    assert(foc_advanced_power_trial_record_commit(&trial) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(trial.active_ticks == 1U);
}

static void test_invalid_configuration_is_rejected(void)
{
    foc_advanced_power_trial_t trial;
    foc_runtime_config_t runtime = valid_runtime();
    foc_advanced_runtime_config_t advanced = valid_advanced(0U);

    assert(foc_advanced_power_trial_init(&trial) ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OK);
    runtime.closed_loop_enable = 0U;
    assert(foc_advanced_power_trial_prepare(
               &trial,
               FOC_ADVANCED_POWER_TRIAL_MODE_BASIC,
               &runtime,
               &advanced,
               0U,
               0U) == FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE);

    runtime = valid_runtime();
    advanced = valid_advanced(FOC_ADVANCED_FEATURE_FIELD_WEAKENING);
    assert(foc_advanced_power_trial_init(&trial) ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OK);
    assert(foc_advanced_power_trial_prepare(
               &trial,
               FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING,
               &runtime,
               &advanced,
               0U,
               0U) == FOC_ADVANCED_POWER_TRIAL_RESULT_INVALID_ENVELOPE);
}

static void test_fault_injections_fail_closed(void)
{
    foc_advanced_power_trial_t trial;
    foc_telemetry_t closed = closed_loop_telemetry();
    foc_advanced_telemetry_t decoupling = advanced_telemetry(
        FOC_ADVANCED_FEATURE_DECOUPLING,
        FOC_ADVANCED_STATUS_CONFIGURED);

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_BASIC);
    assert(foc_advanced_power_trial_begin_tick(&trial, 8U, 2U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_FAULT_EPOCH_CHANGED);

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_BASIC);
    assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 3U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result == FOC_ADVANCED_POWER_TRIAL_RESULT_DEADLINE_MISSED);

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING);
    assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(foc_advanced_power_trial_validate_control(
               &trial, &closed, &decoupling) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(trial.active_ticks == 0U);
    assert(foc_advanced_power_trial_record_commit(&trial) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    closed.observer_reliable = 0U;
    decoupling.status_flags |= FOC_ADVANCED_STATUS_BASIC_FALLBACK |
                                 FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE;
    assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(foc_advanced_power_trial_validate_control(
               &trial, &closed, &decoupling) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result ==
           FOC_ADVANCED_POWER_TRIAL_RESULT_OBSERVER_FALLBACK);

    prepare_and_arm(&trial, FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING);
    decoupling = advanced_telemetry(
        FOC_ADVANCED_FEATURE_DECOUPLING,
        FOC_ADVANCED_STATUS_CONFIGURED | FOC_ADVANCED_STATUS_FAULTED);
    closed = closed_loop_telemetry();
    assert(foc_advanced_power_trial_begin_tick(&trial, 7U, 2U) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE);
    assert(foc_advanced_power_trial_validate_control(
               &trial, &closed, &decoupling) ==
           FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP);
    assert(trial.result == FOC_ADVANCED_POWER_TRIAL_RESULT_ADVANCED_FAULT);
}

int main(void)
{
    test_basic_and_decoupling_complete_at_exact_window();
    test_invalid_configuration_is_rejected();
    test_fault_injections_fail_closed();
    puts("foc_advanced_power_trial_tests: PASS");
    return 0;
}
