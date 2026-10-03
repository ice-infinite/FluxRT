#include "foc_external_io_management.h"

#include "foc_build_profile.h"

#if defined(FLUXRT_INPUT_PWM_PULSE) || defined(FLUXRT_INPUT_ANALOG) || \
    defined(FLUXRT_INPUT_STEP_DIR)
#include "foc_simple_input_adapter.h"
#endif

#include <string.h>

static int32_t foc_external_io_management_input_index(
    foc_external_input_mask_t input)
{
    switch (input)
    {
    case FOC_EXTERNAL_INPUT_PWM_PULSE:
        return 0;
    case FOC_EXTERNAL_INPUT_ANALOG:
        return 2;
    case FOC_EXTERNAL_INPUT_STEP_DIR:
        return 3;
    default:
        return -1;
    }
}

foc_external_io_management_result_t
foc_external_io_management_init_with_capabilities(
    foc_external_io_management_t *management,
    const foc_external_io_capabilities_t *capabilities)
{
    foc_command_service_ops_t ops;
    foc_config_apply_guard_t guard;
    foc_command_service_result_t command_result;
    foc_command_arbiter_status_t rust_result;
    foc_external_command_owner_result_t owner_result;
    foc_input_service_result_t input_result;

    if ((management == 0) || (capabilities == 0) ||
        (capabilities->struct_size != sizeof(*capabilities)) ||
        (capabilities->version != FOC_EXTERNAL_IO_CAPABILITIES_VERSION))
    {
        return FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT;
    }
    (void)memset(management, 0, sizeof(*management));
    management->struct_size = sizeof(*management);
    management->version = FOC_EXTERNAL_IO_MANAGEMENT_VERSION;
    management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_RUST_ABI_FAILED;

#if defined(FLUXRT_INPUT_PWM_PULSE) || defined(FLUXRT_INPUT_ANALOG) || \
    defined(FLUXRT_INPUT_STEP_DIR)
    if (foc_simple_input_adapter_abi_self_check() !=
        FOC_SIMPLE_INPUT_ADAPTER_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_INPUT_ABI_FAILED;
        return management->last_result;
    }
#endif

    management->capabilities = *capabilities;
    foc_external_io_default_config(&management->active_config);

    rust_result = foc_command_rust_adapter_init(
        &management->command_adapter);
    if (rust_result != FOC_COMMAND_ARBITER_OK)
    {
        return management->last_result;
    }
    command_result = foc_command_rust_adapter_make_ops(
        &management->command_adapter, &ops);
    if (command_result != FOC_COMMAND_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_SERVICE_FAILED;
        return management->last_result;
    }
    owner_result = foc_external_command_owner_init(
        &management->command_owner);
    if (owner_result != FOC_EXTERNAL_COMMAND_OWNER_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_OWNER_FAILED;
        return management->last_result;
    }
    command_result = foc_command_service_init(
        &management->command_service,
        &management->capabilities,
        &ops);
    if (command_result != FOC_COMMAND_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_SERVICE_FAILED;
        return management->last_result;
    }
    input_result = foc_input_service_init(
        &management->input_service,
        &management->capabilities);
    if (input_result != FOC_INPUT_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_INPUT_SERVICE_FAILED;
        return management->last_result;
    }

    (void)memset(&guard, 0, sizeof(guard));
    guard.struct_size = sizeof(guard);
    guard.abi_version = FOC_CONFIG_ABI_VERSION;
    guard.axis_state = FOC_AXIS_STATE_DISABLED;
    command_result = foc_command_service_apply_config(
        &management->command_service,
        &management->active_config,
        &guard,
        &management->validation);
    if (command_result != FOC_COMMAND_SERVICE_DISABLED)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_DEFAULT_CONFIG_FAILED;
        return management->last_result;
    }
    input_result = foc_input_service_apply_config(
        &management->input_service,
        &management->active_config,
        &guard,
        &management->validation);
    if (input_result != FOC_INPUT_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_DEFAULT_CONFIG_FAILED;
        return management->last_result;
    }

    management->initialized = 1U;
    management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_OK;
    return management->last_result;
}

foc_external_io_management_result_t foc_external_io_management_apply_config(
    foc_external_io_management_t *management,
    const foc_external_io_config_t *config,
    const foc_config_apply_guard_t *guard)
{
    foc_external_io_config_t candidate;
    foc_external_io_validation_t validation;
    foc_input_service_t input_candidate;
    foc_command_service_result_t result;
    foc_input_service_result_t input_result;

    if ((management == 0) || (config == 0) || (guard == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U))
    {
        return FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT;
    }
    candidate = *config;
    input_candidate = management->input_service;
    input_result = foc_input_service_apply_config(
        &input_candidate,
        &candidate,
        guard,
        &validation);
    if (input_result != FOC_INPUT_SERVICE_OK)
    {
        management->validation = validation;
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_CONFIG_APPLY_FAILED;
        return management->last_result;
    }
    result = foc_command_service_apply_config(
        &management->command_service,
        &candidate,
        guard,
        &validation);
    if ((result != FOC_COMMAND_SERVICE_OK) &&
        (result != FOC_COMMAND_SERVICE_DISABLED))
    {
        management->validation = validation;
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_CONFIG_APPLY_FAILED;
        return management->last_result;
    }
    management->input_service = input_candidate;
    management->active_config = candidate;
    management->validation = validation;
    management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_OK;
    return management->last_result;
}

foc_external_io_management_result_t foc_external_io_management_poll(
    foc_external_io_management_t *management,
    uint32_t now_ms,
    uint32_t fault_active)
{
    foc_external_command_owner_result_t result;
    foc_input_service_result_t input_result;
    foc_external_input_mask_t expired_mask;
    if ((management == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U))
    {
        return FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT;
    }
    input_result = foc_input_service_poll(
        &management->input_service,
        now_ms,
        &expired_mask);
    if (input_result != FOC_INPUT_SERVICE_OK)
    {
        management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_POLL_FAILED;
        return management->last_result;
    }
    management->last_expired_input_mask = expired_mask;
    result = foc_external_command_owner_tick(
        &management->command_owner,
        &management->command_adapter,
        now_ms,
        fault_active);
    if (result != FOC_EXTERNAL_COMMAND_OWNER_OK)
    {
        management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_POLL_FAILED;
        return management->last_result;
    }
    management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_OK;
    return management->last_result;
}

foc_external_io_management_result_t
foc_external_io_management_submit_simple_input(
    foc_external_io_management_t *management,
    const foc_simple_input_candidate_t *candidate,
    uint32_t now_ms)
{
    foc_input_service_result_t input_result;
    foc_command_service_result_t command_result;
    int32_t input_index = -1;

    if (candidate != 0)
    {
        input_index = foc_external_io_management_input_index(
            candidate->sample.input);
    }

    if ((management == 0) || (candidate == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U) ||
        (candidate->struct_size != sizeof(*candidate)) ||
        (candidate->version != FOC_SIMPLE_INPUT_CANDIDATE_VERSION) ||
        (candidate->sample.source_id != candidate->command.source_id) ||
        (candidate->sample.sequence != candidate->command.sequence) ||
        (input_index < 0) ||
        ((management->active_config.input_enable_mask &
          candidate->sample.input) == 0U) ||
        (candidate->command.control_mode !=
         management->active_config.inputs[(uint32_t)input_index].control_mode) ||
        (candidate->command.command_kind != FOC_PRODUCT_COMMAND_SETPOINT) ||
        (candidate->command.axis_request != FOC_AXIS_REQUEST_NONE) ||
        (candidate->command.input_mode != FOC_INPUT_MODE_PASSTHROUGH))
    {
        if (management != 0)
        {
            management->last_result =
                FOC_EXTERNAL_IO_MANAGEMENT_CANDIDATE_REJECTED;
        }
        return FOC_EXTERNAL_IO_MANAGEMENT_CANDIDATE_REJECTED;
    }

    input_result = foc_input_service_publish(
        &management->input_service,
        &candidate->sample,
        now_ms);
    if (input_result != FOC_INPUT_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_INPUT_SERVICE_FAILED;
        return management->last_result;
    }
    command_result = foc_command_service_submit(
        &management->command_service,
        &candidate->command,
        now_ms);
    if (command_result != FOC_COMMAND_SERVICE_OK)
    {
        management->last_result =
            FOC_EXTERNAL_IO_MANAGEMENT_COMMAND_SUBMIT_FAILED;
        return management->last_result;
    }
    management->last_result = FOC_EXTERNAL_IO_MANAGEMENT_OK;
    return management->last_result;
}

foc_external_io_management_result_t
foc_external_io_management_get_input_status(
    const foc_external_io_management_t *management,
    foc_input_service_status_t *output)
{
    if ((management == 0) || (output == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U) ||
        (foc_input_service_get_status(
            &management->input_service, output) != FOC_INPUT_SERVICE_OK))
    {
        return FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT;
    }
    return FOC_EXTERNAL_IO_MANAGEMENT_OK;
}

foc_external_io_management_result_t foc_external_io_management_get_event(
    const foc_external_io_management_t *management,
    foc_external_command_event_t *output)
{
    if ((management == 0) || (output == 0) ||
        (management->struct_size != sizeof(*management)) ||
        (management->version != FOC_EXTERNAL_IO_MANAGEMENT_VERSION) ||
        (management->initialized == 0U) ||
        (foc_external_command_owner_get_event(
            &management->command_owner, output) !=
         FOC_EXTERNAL_COMMAND_OWNER_OK))
    {
        return FOC_EXTERNAL_IO_MANAGEMENT_INVALID_ARGUMENT;
    }
    return FOC_EXTERNAL_IO_MANAGEMENT_OK;
}
