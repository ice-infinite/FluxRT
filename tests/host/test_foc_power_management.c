#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "foc_power_management.h"

static uint32_t g_force_safe_count;
static uint32_t g_shutdown_next;
static foc_power_runtime_config_t g_config;

static void force_safe(void *context)
{
    (void)context;
    ++g_force_safe_count;
}

uint32_t foc_rust_power_abi_version(void)
{
    return FOC_POWER_ABI_VERSION;
}

foc_power_status_t foc_rust_power_init(foc_power_context_storage_t *storage)
{
    if (storage == 0) return FOC_POWER_STATUS_INVALID_ARGUMENT;
    memset(storage, 0x5A, sizeof(*storage));
    return FOC_POWER_STATUS_OK;
}

foc_power_status_t foc_rust_power_default_config(
    foc_power_runtime_config_t *output)
{
    if (output == 0) return FOC_POWER_STATUS_INVALID_ARGUMENT;
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_POWER_ABI_VERSION;
    output->minimum_valid_temperature_c = -40.0f;
    output->maximum_valid_temperature_c = 150.0f;
    output->thermal_derating_release_c = 70.0f;
    output->thermal_derating_start_c = 80.0f;
    output->over_temperature_trip_c = 100.0f;
    output->bus_undervoltage_trip_v = 7.0f;
    output->bus_undervoltage_recovery_v = 8.0f;
    output->bus_overvoltage_recovery_v = 16.0f;
    output->bus_overvoltage_trip_v = 18.0f;
    return FOC_POWER_STATUS_OK;
}

foc_power_status_t foc_rust_power_configure(
    foc_power_context_storage_t *storage,
    const foc_power_runtime_config_t *config)
{
    if ((storage == 0) || (config == 0) ||
        (config->struct_size != sizeof(*config)) ||
        (config->abi_version != FOC_POWER_ABI_VERSION))
    {
        return FOC_POWER_STATUS_INVALID_ARGUMENT;
    }
    if ((config->enabled != 0U) && (config->source_current_limit_a <= 0.0f))
    {
        return FOC_POWER_STATUS_INVALID_SOURCE_CAPABILITY;
    }
    g_config = *config;
    return FOC_POWER_STATUS_OK;
}

foc_power_status_t foc_rust_power_step(
    foc_power_context_storage_t *storage,
    const foc_power_input_t *input,
    foc_power_output_t *output)
{
    if ((storage == 0) || (input == 0) || (output == 0))
        return FOC_POWER_STATUS_INVALID_ARGUMENT;
    memset(output, 0, sizeof(*output));
    output->struct_size = sizeof(*output);
    output->abi_version = FOC_POWER_ABI_VERSION;
    output->configured = 1U;
    output->enabled = g_config.enabled;
    output->drive_allowed = (g_config.enabled != 0U) && (g_shutdown_next == 0U);
    output->shutdown_requested = (output->drive_allowed == 0U);
    output->thermal_current_scale = output->drive_allowed ? 1.0f : 0.0f;
    output->limited_dc_current_a = output->drive_allowed ?
        input->requested_dc_current_a : 0.0f;
    g_shutdown_next = 0U;
    return FOC_POWER_STATUS_OK;
}

foc_power_status_t foc_rust_power_reset_faults(
    foc_power_context_storage_t *storage,
    const foc_power_input_t *input)
{
    if ((storage == 0) || (input == 0))
        return FOC_POWER_STATUS_INVALID_ARGUMENT;
    return (input->requested_dc_current_a == 0.0f) ?
        FOC_POWER_STATUS_OK : FOC_POWER_STATUS_UNSAFE_RESET;
}

static foc_config_apply_guard_t safe_guard(void)
{
    foc_config_apply_guard_t guard;
    memset(&guard, 0, sizeof(guard));
    guard.struct_size = sizeof(guard);
    guard.abi_version = FOC_CONFIG_ABI_VERSION;
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    return guard;
}

int main(void)
{
    foc_power_management_t manager;
    const foc_power_management_ops_t ops = {
        sizeof(foc_power_management_ops_t),
        FOC_POWER_MANAGEMENT_VERSION,
        force_safe,
        0,
    };
    foc_power_runtime_config_t config;
    foc_power_input_t input;
    foc_power_output_t output;
    foc_config_apply_guard_t guard = safe_guard();

    memset(&manager, 0, sizeof(manager));
    assert(foc_power_management_init(&manager, &ops) == FOC_POWER_MANAGEMENT_OK);
    assert(g_force_safe_count == 1U);
    assert(manager.active_config.enabled == 0U);
    assert(manager.active_config.regeneration_allowed == 0U);

    config = manager.active_config;
    config.enabled = 1U;
    config.temperature_sensor_required = 1U;
    config.source_current_limit_a = 2.0f;
    guard.drive_active = 1U;
    assert(foc_power_management_configure(&manager, &config, &guard) ==
           FOC_POWER_MANAGEMENT_UNSAFE_STATE);
    assert(g_force_safe_count == 2U);
    guard.drive_active = 0U;
    assert(foc_power_management_configure(&manager, &config, &guard) ==
           FOC_POWER_MANAGEMENT_OK);

    memset(&input, 0, sizeof(input));
    input.struct_size = sizeof(input);
    input.abi_version = FOC_POWER_ABI_VERSION;
    input.temperature_valid = 1U;
    input.source_available = 1U;
    input.temperature_c = 25.0f;
    input.dc_bus_voltage_v = 12.0f;
    input.requested_dc_current_a = 1.0f;
    assert(foc_power_management_step(&manager, &input, &output) ==
           FOC_POWER_MANAGEMENT_OK);
    assert(output.drive_allowed == 1U);
    g_shutdown_next = 1U;
    assert(foc_power_management_step(&manager, &input, &output) ==
           FOC_POWER_MANAGEMENT_SHUTDOWN);
    assert(g_force_safe_count == 3U);

    input.requested_dc_current_a = 1.0f;
    assert(foc_power_management_reset_faults(&manager, &input, &guard) ==
           FOC_POWER_MANAGEMENT_POLICY_ERROR);
    input.requested_dc_current_a = 0.0f;
    assert(foc_power_management_reset_faults(&manager, &input, &guard) ==
           FOC_POWER_MANAGEMENT_OK);
    puts("foc power management host tests passed");
    return 0;
}
