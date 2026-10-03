#include "foc_external_io.h"

#include "foc_build_profile.h"

#include <math.h>
#include <string.h>

#define FOC_EXTERNAL_IO_HALF_RANGE (0x80000000UL)
#define FOC_EXTERNAL_IO_PERMISSION_KNOWN_MASK \
    ((1UL << FOC_PRODUCT_COMMAND_RELEASE) | \
     (1UL << FOC_PRODUCT_COMMAND_AXIS_REQUEST) | \
     (1UL << FOC_PRODUCT_COMMAND_SETPOINT) | \
     (1UL << FOC_PRODUCT_COMMAND_CLEAR_FAULT) | \
     (1UL << FOC_PRODUCT_COMMAND_EMERGENCY_STOP))
#define FOC_EXTERNAL_IO_SIMPLE_INPUT_PERMISSIONS \
    ((1UL << FOC_PRODUCT_COMMAND_RELEASE) | \
     (1UL << FOC_PRODUCT_COMMAND_SETPOINT))

static const uint32_t g_input_bits[FOC_EXTERNAL_IO_INPUT_COUNT] = {
    FOC_EXTERNAL_INPUT_PWM_PULSE,
    FOC_EXTERNAL_INPUT_DSHOT,
    FOC_EXTERNAL_INPUT_ANALOG,
    FOC_EXTERNAL_INPUT_STEP_DIR,
};

static const foc_external_io_field_t g_link_fields[FOC_EXTERNAL_IO_LINK_COUNT] = {
    FOC_EXTERNAL_IO_FIELD_UART,
    FOC_EXTERNAL_IO_FIELD_USB,
    FOC_EXTERNAL_IO_FIELD_CAN,
    FOC_EXTERNAL_IO_FIELD_ETHERCAT,
};

static const foc_external_io_field_t g_input_fields[FOC_EXTERNAL_IO_INPUT_COUNT] = {
    FOC_EXTERNAL_IO_FIELD_PWM,
    FOC_EXTERNAL_IO_FIELD_DSHOT,
    FOC_EXTERNAL_IO_FIELD_ANALOG,
    FOC_EXTERNAL_IO_FIELD_STEP_DIR,
};

static foc_external_io_status_t foc_external_io_fail(
    foc_external_io_validation_t *validation,
    foc_external_io_status_t status,
    foc_external_io_field_t field,
    uint32_t detail,
    uint32_t conflict)
{
    if (validation != 0)
    {
        validation->status = status;
        validation->field = field;
        validation->detail = detail;
        validation->conflict_resource_mask = conflict;
    }
    return status;
}

static uint32_t foc_external_protocol_mask(foc_external_protocol_t protocol)
{
    switch (protocol)
    {
    case FOC_EXTERNAL_PROTOCOL_FLUXRT_NATIVE:
        return FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE;
    case FOC_EXTERNAL_PROTOCOL_DRONECAN:
        return FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN;
    case FOC_EXTERNAL_PROTOCOL_VESC_CAN:
        return FOC_EXTERNAL_PROTOCOL_MASK_VESC_CAN;
    case FOC_EXTERNAL_PROTOCOL_CANOPEN_CIA402:
        return FOC_EXTERNAL_PROTOCOL_MASK_CANOPEN_CIA402;
    case FOC_EXTERNAL_PROTOCOL_ETHERCAT_COE_CIA402:
        return FOC_EXTERNAL_PROTOCOL_MASK_ETHERCAT_COE_CIA402;
    default:
        return 0U;
    }
}

static uint32_t foc_external_protocol_allowed(
    uint32_t link_index,
    foc_external_protocol_t protocol)
{
    if ((link_index == 0U) || (link_index == 1U))
    {
        return (protocol == FOC_EXTERNAL_PROTOCOL_FLUXRT_NATIVE) ? 1U : 0U;
    }
    if (link_index == 2U)
    {
        return ((protocol == FOC_EXTERNAL_PROTOCOL_FLUXRT_NATIVE) ||
                (protocol == FOC_EXTERNAL_PROTOCOL_DRONECAN) ||
                (protocol == FOC_EXTERNAL_PROTOCOL_VESC_CAN) ||
                (protocol == FOC_EXTERNAL_PROTOCOL_CANOPEN_CIA402)) ? 1U : 0U;
    }
    return (protocol == FOC_EXTERNAL_PROTOCOL_ETHERCAT_COE_CIA402) ? 1U : 0U;
}

static uint32_t foc_external_link_is_enabled(
    uint32_t link_index,
    uint32_t transport_mask)
{
    switch (link_index)
    {
    case 0U:
        return ((transport_mask & FOC_EXTERNAL_TRANSPORT_UART) != 0U) ? 1U : 0U;
    case 1U:
        return ((transport_mask & FOC_EXTERNAL_TRANSPORT_USB_CDC) != 0U) ? 1U : 0U;
    case 2U:
        return ((transport_mask & (FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC |
                                   FOC_EXTERNAL_TRANSPORT_CAN_FD)) != 0U) ? 1U : 0U;
    case 3U:
        return ((transport_mask & FOC_EXTERNAL_TRANSPORT_ETHERCAT) != 0U) ? 1U : 0U;
    default:
        return 0U;
    }
}

static uint32_t foc_external_policy_valid(
    const foc_external_source_policy_t *policy,
    uint32_t allowed_permissions,
    uint32_t optional)
{
    if (policy->source_id == 0U)
    {
        return ((optional != 0U) &&
                (policy->priority == 0U) &&
                (policy->permissions == 0U) &&
                (policy->lease_ms == 0U) &&
                (policy->command_timeout_ms == 0U)) ? 1U : 0U;
    }
    if ((policy->priority > 255U) ||
        (policy->permissions == 0U) ||
        ((policy->permissions & ~allowed_permissions) != 0U) ||
        (policy->lease_ms == 0U) ||
        (policy->command_timeout_ms == 0U) ||
        (policy->lease_ms > policy->command_timeout_ms) ||
        (policy->command_timeout_ms >= FOC_EXTERNAL_IO_HALF_RANGE))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t foc_external_source_is_duplicate(
    uint32_t source_id,
    const uint32_t *sources,
    uint32_t source_count)
{
    uint32_t index;
    for (index = 0U; index < source_count; ++index)
    {
        if (sources[index] == source_id)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint32_t foc_external_input_mapping_valid(
    uint32_t input_index,
    const foc_external_input_config_t *input)
{
    if ((input->control_mode != FOC_CONTROL_MODE_TORQUE) &&
        (input->control_mode != FOC_CONTROL_MODE_VELOCITY) &&
        (input->control_mode != FOC_CONTROL_MODE_POSITION))
    {
        return 0U;
    }
    if ((input->failure_action != FOC_EXTERNAL_FAILURE_RELEASE) &&
        (input->failure_action != FOC_EXTERNAL_FAILURE_CONTROLLED_STOP) &&
        !((input_index == 3U) &&
          (input->failure_action == FOC_EXTERNAL_FAILURE_HOLD)))
    {
        return 0U;
    }
    if ((input_index == 3U) &&
        (input->control_mode != FOC_CONTROL_MODE_POSITION))
    {
        return 0U;
    }
    return 1U;
}

static uint32_t foc_centered_input_calibration_valid(
    const foc_centered_input_calibration_config_t *config)
{
    int64_t negative_span;
    int64_t positive_span;

    negative_span = (int64_t)config->raw_neutral - (int64_t)config->raw_min;
    positive_span = (int64_t)config->raw_max - (int64_t)config->raw_neutral;
    return ((negative_span > 0) &&
            (positive_span > 0) &&
            ((int64_t)config->deadband < negative_span) &&
            ((int64_t)config->deadband < positive_span) &&
            isfinite(config->negative_limit_si) &&
            (config->negative_limit_si > 0.0f) &&
            isfinite(config->positive_limit_si) &&
            (config->positive_limit_si > 0.0f)) ? 1U : 0U;
}

static uint32_t foc_step_dir_calibration_valid(
    const foc_step_dir_calibration_config_t *config)
{
    return ((config->full_steps_per_revolution != 0U) &&
            (config->microsteps != 0U) &&
            (config->gear_numerator != 0U) &&
            (config->gear_denominator != 0U) &&
            ((config->direction == -1) || (config->direction == 1)) &&
            isfinite(config->zero_position_rad) &&
            (config->maximum_step_rate_hz != 0U)) ? 1U : 0U;
}

static uint32_t foc_simple_input_calibration_valid(
    uint32_t input_index,
    const foc_simple_input_config_t *config)
{
    if (input_index == 0U)
    {
        return foc_centered_input_calibration_valid(&config->pwm);
    }
    if (input_index == 2U)
    {
        return foc_centered_input_calibration_valid(&config->analog);
    }
    if (input_index == 3U)
    {
        return foc_step_dir_calibration_valid(&config->step_dir);
    }
    return 1U;
}

void foc_external_io_default_config(foc_external_io_config_t *config)
{
    if (config == 0)
    {
        return;
    }
    (void)memset(config, 0, sizeof(*config));
    config->struct_size = sizeof(*config);
    config->version = FOC_EXTERNAL_IO_CONFIG_VERSION;
    config->revision = 1U;
    config->simple_inputs.struct_size = sizeof(config->simple_inputs);
    config->simple_inputs.version = FOC_SIMPLE_INPUT_CONFIG_VERSION;
}

void foc_external_io_empty_capabilities(
    foc_external_io_capabilities_t *capabilities)
{
    if (capabilities == 0)
    {
        return;
    }
    (void)memset(capabilities, 0, sizeof(*capabilities));
    capabilities->struct_size = sizeof(*capabilities);
    capabilities->version = FOC_EXTERNAL_IO_CAPABILITIES_VERSION;
}

void foc_external_io_build_capabilities(
    foc_external_io_capabilities_t *capabilities)
{
    foc_external_io_empty_capabilities(capabilities);
    if (capabilities == 0)
    {
        return;
    }
#if defined(FLUXRT_TRANSPORT_UART)
    capabilities->compiled_transport_mask |= FOC_EXTERNAL_TRANSPORT_UART;
#endif
#if defined(FLUXRT_TRANSPORT_USB_CDC)
    capabilities->compiled_transport_mask |= FOC_EXTERNAL_TRANSPORT_USB_CDC;
#endif
#if defined(FLUXRT_TRANSPORT_CAN_CLASSIC)
    capabilities->compiled_transport_mask |= FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC;
#endif
#if defined(FLUXRT_TRANSPORT_CAN_FD)
    capabilities->compiled_transport_mask |= FOC_EXTERNAL_TRANSPORT_CAN_FD;
#endif
#if defined(FLUXRT_TRANSPORT_ETHERCAT)
    capabilities->compiled_transport_mask |= FOC_EXTERNAL_TRANSPORT_ETHERCAT;
#endif
#if defined(FLUXRT_PROTOCOL_NATIVE)
    capabilities->compiled_protocol_mask |= FOC_EXTERNAL_PROTOCOL_MASK_FLUXRT_NATIVE;
#endif
#if defined(FLUXRT_PROTOCOL_DRONECAN)
    capabilities->compiled_protocol_mask |= FOC_EXTERNAL_PROTOCOL_MASK_DRONECAN;
#endif
#if defined(FLUXRT_PROTOCOL_VESC_CAN)
    capabilities->compiled_protocol_mask |= FOC_EXTERNAL_PROTOCOL_MASK_VESC_CAN;
#endif
#if defined(FLUXRT_PROTOCOL_CANOPEN_CIA402)
    capabilities->compiled_protocol_mask |= FOC_EXTERNAL_PROTOCOL_MASK_CANOPEN_CIA402;
#endif
#if defined(FLUXRT_PROTOCOL_ETHERCAT_COE_CIA402)
    capabilities->compiled_protocol_mask |= FOC_EXTERNAL_PROTOCOL_MASK_ETHERCAT_COE_CIA402;
#endif
#if defined(FLUXRT_INPUT_PWM_PULSE)
    capabilities->compiled_input_mask |= FOC_EXTERNAL_INPUT_PWM_PULSE;
#endif
#if defined(FLUXRT_INPUT_DSHOT)
    capabilities->compiled_input_mask |= FOC_EXTERNAL_INPUT_DSHOT;
#endif
#if defined(FLUXRT_INPUT_ANALOG)
    capabilities->compiled_input_mask |= FOC_EXTERNAL_INPUT_ANALOG;
#endif
#if defined(FLUXRT_INPUT_STEP_DIR)
    capabilities->compiled_input_mask |= FOC_EXTERNAL_INPUT_STEP_DIR;
#endif
}

foc_external_io_status_t foc_external_io_validate_config(
    const foc_external_io_capabilities_t *capabilities,
    const foc_external_io_config_t *config,
    foc_external_io_validation_t *validation)
{
    uint32_t link_index;
    uint32_t input_index;
    uint32_t source_count = 0U;
    uint32_t sources[FOC_EXTERNAL_IO_MAX_COMMAND_SOURCES] = {0U};
    uint32_t resource_mask;

    if (validation != 0)
    {
        (void)memset(validation, 0, sizeof(*validation));
        validation->struct_size = sizeof(*validation);
        validation->version = FOC_EXTERNAL_IO_VALIDATION_VERSION;
        validation->status = FOC_EXTERNAL_IO_STATUS_INVALID_ARGUMENT;
    }
    if ((capabilities == 0) || (config == 0) || (validation == 0))
    {
        return FOC_EXTERNAL_IO_STATUS_INVALID_ARGUMENT;
    }
    if ((capabilities->struct_size != sizeof(*capabilities)) ||
        (capabilities->version != FOC_EXTERNAL_IO_CAPABILITIES_VERSION) ||
        (config->struct_size != sizeof(*config)) ||
        (config->version != FOC_EXTERNAL_IO_CONFIG_VERSION) ||
        (config->revision == 0U) ||
        (config->reserved != 0U) ||
        (config->simple_inputs.struct_size != sizeof(config->simple_inputs)) ||
        (config->simple_inputs.version != FOC_SIMPLE_INPUT_CONFIG_VERSION))
    {
        return foc_external_io_fail(
            validation, FOC_EXTERNAL_IO_STATUS_INVALID_LAYOUT,
            FOC_EXTERNAL_IO_FIELD_CAPABILITIES, 0U, 0U);
    }
    if (((capabilities->compiled_transport_mask |
          capabilities->board_transport_mask) &
         ~((uint32_t)FOC_EXTERNAL_TRANSPORT_KNOWN_MASK)) != 0U ||
        ((capabilities->compiled_input_mask |
          capabilities->board_input_mask) &
         ~((uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK)) != 0U ||
        (capabilities->compiled_protocol_mask &
         ~((uint32_t)FOC_EXTERNAL_PROTOCOL_KNOWN_MASK)) != 0U ||
        (config->transport_enable_mask &
         ~((uint32_t)FOC_EXTERNAL_TRANSPORT_KNOWN_MASK)) != 0U ||
        (config->input_enable_mask &
         ~((uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK)) != 0U)
    {
        return foc_external_io_fail(
            validation, FOC_EXTERNAL_IO_STATUS_UNKNOWN_CAPABILITY,
            FOC_EXTERNAL_IO_FIELD_CAPABILITIES, 0U, 0U);
    }
    validation->configured_transport_mask = config->transport_enable_mask;
    validation->configured_input_mask = config->input_enable_mask;
    if ((config->transport_enable_mask &
         ~capabilities->compiled_transport_mask) != 0U ||
        (config->input_enable_mask &
         ~capabilities->compiled_input_mask) != 0U)
    {
        return foc_external_io_fail(
            validation, FOC_EXTERNAL_IO_STATUS_NOT_COMPILED,
            FOC_EXTERNAL_IO_FIELD_CAPABILITIES, 0U, 0U);
    }
    if ((config->transport_enable_mask &
         ~capabilities->board_transport_mask) != 0U ||
        (config->input_enable_mask &
         ~capabilities->board_input_mask) != 0U)
    {
        return foc_external_io_fail(
            validation, FOC_EXTERNAL_IO_STATUS_BOARD_UNAVAILABLE,
            FOC_EXTERNAL_IO_FIELD_CAPABILITIES, 0U, 0U);
    }
    if ((config->transport_enable_mask &
         (FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC |
          FOC_EXTERNAL_TRANSPORT_CAN_FD)) ==
        (FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC |
         FOC_EXTERNAL_TRANSPORT_CAN_FD))
    {
        return foc_external_io_fail(
            validation, FOC_EXTERNAL_IO_STATUS_INVALID_MAPPING,
            FOC_EXTERNAL_IO_FIELD_CAN, config->transport_enable_mask, 0U);
    }

    resource_mask = capabilities->reserved_resource_mask;
    for (link_index = 0U; link_index < FOC_EXTERNAL_IO_LINK_COUNT; ++link_index)
    {
        const foc_external_link_config_t *link = &config->links[link_index];
        uint32_t protocol_mask;
        uint32_t claim = 0U;
        uint32_t transport_index = link_index;

        if (foc_external_link_is_enabled(
                link_index, config->transport_enable_mask) == 0U)
        {
            continue;
        }
        protocol_mask = foc_external_protocol_mask(link->protocol);
        if ((protocol_mask == 0U) ||
            (foc_external_protocol_allowed(link_index, link->protocol) == 0U) ||
            ((protocol_mask & capabilities->compiled_protocol_mask) == 0U))
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_INVALID_PROTOCOL,
                g_link_fields[link_index], link->protocol, 0U);
        }
        if (link_index == 0U)
        {
            if (link->nominal_bitrate == 0U)
            {
                return foc_external_io_fail(
                    validation, FOC_EXTERNAL_IO_STATUS_INVALID_TIMING,
                    g_link_fields[link_index], 0U, 0U);
            }
        }
        else if (link_index == 2U)
        {
            transport_index =
                ((config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_CAN_FD) != 0U) ?
                    3U : 2U;
            if ((link->nominal_bitrate == 0U) ||
                (((config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_CAN_FD) != 0U) &&
                 (link->data_bitrate == 0U)) ||
                (((config->transport_enable_mask & FOC_EXTERNAL_TRANSPORT_CAN_CLASSIC) != 0U) &&
                 (link->data_bitrate != 0U)))
            {
                return foc_external_io_fail(
                    validation, FOC_EXTERNAL_IO_STATUS_INVALID_TIMING,
                    g_link_fields[link_index], 0U, 0U);
            }
        }
        else if ((link_index == 3U) && (link->heartbeat_ms == 0U))
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_INVALID_TIMING,
                g_link_fields[link_index], 0U, 0U);
        }
        if (foc_external_policy_valid(
                &link->source, FOC_EXTERNAL_IO_PERMISSION_KNOWN_MASK, 1U) == 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_INVALID_SOURCE,
                g_link_fields[link_index], link->source.source_id, 0U);
        }
        if (link->source.source_id != 0U)
        {
            if (foc_external_source_is_duplicate(
                    link->source.source_id, sources, source_count) != 0U)
            {
                return foc_external_io_fail(
                    validation, FOC_EXTERNAL_IO_STATUS_DUPLICATE_SOURCE,
                    g_link_fields[link_index], link->source.source_id, 0U);
            }
            sources[source_count++] = link->source.source_id;
        }
        claim = capabilities->transport_resource_masks[transport_index];
        if ((claim & resource_mask) != 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_RESOURCE_CONFLICT,
                g_link_fields[link_index], transport_index,
                claim & resource_mask);
        }
        resource_mask |= claim;
    }

    for (input_index = 0U; input_index < FOC_EXTERNAL_IO_INPUT_COUNT; ++input_index)
    {
        const foc_external_input_config_t *input = &config->inputs[input_index];
        uint32_t claim;
        if ((config->input_enable_mask & g_input_bits[input_index]) == 0U)
        {
            continue;
        }
        if (foc_external_policy_valid(
                &input->source, FOC_EXTERNAL_IO_SIMPLE_INPUT_PERMISSIONS, 0U) == 0U)
        {
            return foc_external_io_fail(
                validation,
                ((input->source.permissions &
                  ~((uint32_t)FOC_EXTERNAL_IO_SIMPLE_INPUT_PERMISSIONS)) != 0U) ?
                    FOC_EXTERNAL_IO_STATUS_INVALID_PERMISSION :
                    FOC_EXTERNAL_IO_STATUS_INVALID_SOURCE,
                g_input_fields[input_index], input->source.source_id, 0U);
        }
        if (foc_external_input_mapping_valid(input_index, input) == 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_INVALID_MAPPING,
                g_input_fields[input_index], input->control_mode, 0U);
        }
        if (foc_simple_input_calibration_valid(
                input_index, &config->simple_inputs) == 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_INVALID_CALIBRATION,
                g_input_fields[input_index], 0U, 0U);
        }
        if (foc_external_source_is_duplicate(
                input->source.source_id, sources, source_count) != 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_DUPLICATE_SOURCE,
                g_input_fields[input_index], input->source.source_id, 0U);
        }
        sources[source_count++] = input->source.source_id;
        claim = capabilities->input_resource_masks[input_index];
        if ((claim & resource_mask) != 0U)
        {
            return foc_external_io_fail(
                validation, FOC_EXTERNAL_IO_STATUS_RESOURCE_CONFLICT,
                g_input_fields[input_index], input_index,
                claim & resource_mask);
        }
        resource_mask |= claim;
    }
    validation->configured_source_count = source_count;
    validation->status = FOC_EXTERNAL_IO_STATUS_OK;
    validation->field = FOC_EXTERNAL_IO_FIELD_NONE;
    return FOC_EXTERNAL_IO_STATUS_OK;
}

uint32_t foc_external_io_config_is_default_off(
    const foc_external_io_config_t *config)
{
    return ((config != 0) &&
            (config->struct_size == sizeof(*config)) &&
            (config->version == FOC_EXTERNAL_IO_CONFIG_VERSION) &&
            (config->transport_enable_mask == 0U) &&
            (config->input_enable_mask == 0U)) ? 1U : 0U;
}
