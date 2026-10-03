#include "foc_external_input_port.h"

#include <string.h>

static uint32_t foc_external_input_is_single_known(
    foc_external_input_mask_t input)
{
    return ((input != 0U) &&
            ((input & (input - 1U)) == 0U) &&
            ((input & ~(uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK) == 0U)) ? 1U : 0U;
}

static uint32_t foc_external_input_port_is_valid(
    const foc_external_input_port_t *port)
{
    return ((port != 0) &&
            (port->struct_size == sizeof(*port)) &&
            (port->version == FOC_EXTERNAL_INPUT_PORT_VERSION) &&
            (port->initialized != 0U) &&
            (port->ops != 0) &&
            (port->ops->struct_size == sizeof(*port->ops)) &&
            (port->ops->version == FOC_EXTERNAL_INPUT_PORT_OPS_VERSION) &&
            (port->ops->start != 0) &&
            (port->ops->stop != 0) &&
            (port->ops->read_latest != 0) &&
            (port->ops->get_health != 0) &&
            ((port->supported_input_mask &
              ~(uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK) == 0U) &&
            ((port->enabled_input_mask &
              ~port->supported_input_mask) == 0U)) ? 1U : 0U;
}

foc_external_input_port_result_t foc_external_input_port_bind(
    foc_external_input_port_t *port,
    foc_external_input_mask_t supported_input_mask,
    void *context,
    const foc_external_input_port_ops_t *ops)
{
    if (port == 0)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    (void)memset(port, 0, sizeof(*port));
    if ((context == 0) || (ops == 0) ||
        (ops->struct_size != sizeof(*ops)) ||
        (ops->version != FOC_EXTERNAL_INPUT_PORT_OPS_VERSION) ||
        (ops->start == 0) || (ops->stop == 0) ||
        (ops->read_latest == 0) || (ops->get_health == 0) ||
        ((supported_input_mask &
          ~(uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK) != 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    port->struct_size = sizeof(*port);
    port->version = FOC_EXTERNAL_INPUT_PORT_VERSION;
    port->initialized = 1U;
    port->supported_input_mask = supported_input_mask;
    port->context = context;
    port->ops = ops;
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

foc_external_input_port_result_t foc_external_input_port_start(
    foc_external_input_port_t *port,
    foc_external_input_mask_t input_mask)
{
    foc_external_input_port_result_t result;

    if ((foc_external_input_port_is_valid(port) == 0U) ||
        (input_mask == 0U) ||
        ((input_mask &
          ~(uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK) != 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((input_mask & ~port->supported_input_mask) != 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE;
    }
    result = port->ops->start(port->context, input_mask);
    if (result == FOC_EXTERNAL_INPUT_PORT_OK)
    {
        port->enabled_input_mask |= input_mask;
    }
    return result;
}

foc_external_input_port_result_t foc_external_input_port_stop(
    foc_external_input_port_t *port,
    foc_external_input_mask_t input_mask)
{
    foc_external_input_port_result_t result;

    if ((foc_external_input_port_is_valid(port) == 0U) ||
        (input_mask == 0U) ||
        ((input_mask &
          ~(uint32_t)FOC_EXTERNAL_INPUT_KNOWN_MASK) != 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((input_mask & ~port->supported_input_mask) != 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE;
    }
    result = port->ops->stop(port->context, input_mask);
    if (result == FOC_EXTERNAL_INPUT_PORT_OK)
    {
        port->enabled_input_mask &= ~input_mask;
    }
    return result;
}

foc_external_input_port_result_t foc_external_input_port_read_latest(
    const foc_external_input_port_t *port,
    foc_external_input_mask_t input,
    foc_external_input_raw_sample_t *sample)
{
    foc_external_input_port_result_t result;

    if (sample == 0)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    (void)memset(sample, 0, sizeof(*sample));
    if ((foc_external_input_port_is_valid(port) == 0U) ||
        (foc_external_input_is_single_known(input) == 0U))
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    if ((input & port->supported_input_mask) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_NOT_AVAILABLE;
    }
    result = port->ops->read_latest(port->context, input, sample);
    if (result != FOC_EXTERNAL_INPUT_PORT_OK)
    {
        (void)memset(sample, 0, sizeof(*sample));
        return result;
    }
    if ((sample->struct_size != sizeof(*sample)) ||
        (sample->version != FOC_EXTERNAL_INPUT_RAW_SAMPLE_VERSION) ||
        (sample->input != input) ||
        ((sample->valid_flags &
          ~(uint32_t)FOC_EXTERNAL_INPUT_RAW_VALID_KNOWN_MASK) != 0U) ||
        ((sample->quality_flags &
          ~(uint32_t)FOC_EXTERNAL_INPUT_RAW_QUALITY_KNOWN_MASK) != 0U))
    {
        (void)memset(sample, 0, sizeof(*sample));
        return FOC_EXTERNAL_INPUT_PORT_INVALID_LAYOUT;
    }
    return FOC_EXTERNAL_INPUT_PORT_OK;
}

foc_external_input_port_result_t foc_external_input_port_get_health(
    const foc_external_input_port_t *port,
    foc_external_input_raw_health_t *health)
{
    foc_external_input_port_result_t result;

    if (health == 0)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    (void)memset(health, 0, sizeof(*health));
    if (foc_external_input_port_is_valid(port) == 0U)
    {
        return FOC_EXTERNAL_INPUT_PORT_INVALID_ARGUMENT;
    }
    result = port->ops->get_health(port->context, health);
    if (result != FOC_EXTERNAL_INPUT_PORT_OK)
    {
        (void)memset(health, 0, sizeof(*health));
        return result;
    }
    if ((health->struct_size != sizeof(*health)) ||
        (health->version != FOC_EXTERNAL_INPUT_RAW_HEALTH_VERSION) ||
        ((health->supported_input_mask &
          ~port->supported_input_mask) != 0U) ||
        ((health->enabled_input_mask &
          ~health->supported_input_mask) != 0U) ||
        ((health->healthy_input_mask &
          ~health->enabled_input_mask) != 0U) ||
        ((health->fault_input_mask &
          ~health->supported_input_mask) != 0U))
    {
        (void)memset(health, 0, sizeof(*health));
        return FOC_EXTERNAL_INPUT_PORT_INVALID_LAYOUT;
    }
    return FOC_EXTERNAL_INPUT_PORT_OK;
}
