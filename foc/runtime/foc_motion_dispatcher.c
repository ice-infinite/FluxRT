#include "foc_motion_dispatcher.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static int foc_motion_dispatcher_ops_valid(
    const foc_motion_dispatcher_ops_t *ops)
{
    return (ops != NULL) &&
           (ops->struct_size == (uint32_t)sizeof(*ops)) &&
           (ops->version == FOC_MOTION_DISPATCHER_VERSION) &&
           (ops->enter_critical != NULL) &&
           (ops->exit_critical != NULL);
}

static int foc_motion_dispatcher_ready(
    const foc_motion_dispatcher_t *dispatcher)
{
    return (dispatcher != NULL) &&
           (dispatcher->struct_size == (uint32_t)sizeof(*dispatcher)) &&
           (dispatcher->version == FOC_MOTION_DISPATCHER_VERSION) &&
           (dispatcher->initialized != 0U) &&
           foc_motion_dispatcher_ops_valid(&dispatcher->ops) &&
           (dispatcher->active_slot < 2U);
}

static int foc_motion_command_layout_and_values_valid(
    const foc_product_command_t *command)
{
    if ((command == NULL) ||
        (command->struct_size != (uint32_t)sizeof(*command)) ||
        (command->version != FOC_PRODUCT_COMMAND_VERSION))
    {
        return 0;
    }

    return isfinite(command->duty_ref) &&
           isfinite(command->voltage_d_ref_v) &&
           isfinite(command->voltage_q_ref_v) &&
           isfinite(command->current_d_ref_a) &&
           isfinite(command->current_q_ref_a) &&
           isfinite(command->torque_ref_nm) &&
           isfinite(command->velocity_ref_rad_s) &&
           isfinite(command->position_ref_rad) &&
           isfinite(command->velocity_feedforward_rad_s) &&
           isfinite(command->torque_feedforward_nm) &&
           isfinite(command->current_limit_a) &&
           isfinite(command->torque_limit_nm) &&
           isfinite(command->velocity_limit_rad_s);
}

static void foc_motion_dispatcher_empty_request(
    foc_motion_realtime_request_t *request)
{
    (void)memset(request, 0, sizeof(*request));
    request->struct_size = (uint32_t)sizeof(*request);
    request->version = FOC_MOTION_REALTIME_REQUEST_VERSION;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_init(
    foc_motion_dispatcher_t *dispatcher,
    const foc_motion_dispatcher_ops_t *ops)
{
    foc_motion_dispatcher_ops_t ops_copy;

    if ((dispatcher == NULL) || !foc_motion_dispatcher_ops_valid(ops))
    {
        return FOC_MOTION_DISPATCHER_INVALID_ARGUMENT;
    }

    /* Permit callers to pass &dispatcher->ops without losing the callbacks. */
    ops_copy = *ops;
    (void)memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->struct_size = (uint32_t)sizeof(*dispatcher);
    dispatcher->version = FOC_MOTION_DISPATCHER_VERSION;
    dispatcher->initialized = 1U;
    dispatcher->active_slot = 0U;
    dispatcher->next_publication_sequence = 1U;
    dispatcher->ops = ops_copy;
    foc_motion_dispatcher_empty_request(&dispatcher->slots[0]);
    foc_motion_dispatcher_empty_request(&dispatcher->slots[1]);
    return FOC_MOTION_DISPATCHER_OK;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_publish(
    foc_motion_dispatcher_t *dispatcher,
    const foc_product_command_t *command,
    uint32_t position_valid,
    float mechanical_position_rad,
    uint32_t position_sampled_at_ms)
{
    uint32_t active_slot;
    uint32_t inactive_slot;
    uint32_t key;
    foc_motion_realtime_request_t *slot;

    if (!foc_motion_dispatcher_ready(dispatcher))
    {
        return (dispatcher == NULL) ? FOC_MOTION_DISPATCHER_INVALID_ARGUMENT :
                                      FOC_MOTION_DISPATCHER_NOT_INITIALIZED;
    }
    if (!foc_motion_command_layout_and_values_valid(command) ||
        (position_valid > 1U) ||
        ((position_valid != 0U) && !isfinite(mechanical_position_rad)))
    {
        return FOC_MOTION_DISPATCHER_INVALID_ARGUMENT;
    }

    active_slot = dispatcher->active_slot;
    inactive_slot = active_slot ^ 1U;
    slot = &dispatcher->slots[inactive_slot];
    foc_motion_dispatcher_empty_request(slot);
    slot->publication_sequence = dispatcher->next_publication_sequence;
    dispatcher->next_publication_sequence++;
    slot->request_flags = FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID;
    if (position_valid != 0U)
    {
        slot->request_flags |= FOC_MOTION_REALTIME_REQUEST_POSITION_VALID;
        slot->mechanical_position_rad = mechanical_position_rad;
        slot->position_sampled_at_ms = position_sampled_at_ms;
    }
    slot->command = *command;

    /* The callbacks mask the consuming ADC IRQ and provide the publication
     * barrier. Only the one-word active index is committed while masked. */
    key = dispatcher->ops.enter_critical(dispatcher->ops.context);
    dispatcher->active_slot = inactive_slot;
    dispatcher->ops.exit_critical(dispatcher->ops.context, key);
    return FOC_MOTION_DISPATCHER_OK;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_request_stop(
    foc_motion_dispatcher_t *dispatcher)
{
    uint32_t key;
    if (!foc_motion_dispatcher_ready(dispatcher))
    {
        return (dispatcher == NULL) ? FOC_MOTION_DISPATCHER_INVALID_ARGUMENT :
                                      FOC_MOTION_DISPATCHER_NOT_INITIALIZED;
    }

    key = dispatcher->ops.enter_critical(dispatcher->ops.context);
    dispatcher->urgent_flags |= FOC_MOTION_REALTIME_REQUEST_STOP;
    dispatcher->ops.exit_critical(dispatcher->ops.context, key);
    return FOC_MOTION_DISPATCHER_OK;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_request_fault(
    foc_motion_dispatcher_t *dispatcher,
    uint32_t fault_detail)
{
    uint32_t key;
    if (!foc_motion_dispatcher_ready(dispatcher))
    {
        return (dispatcher == NULL) ? FOC_MOTION_DISPATCHER_INVALID_ARGUMENT :
                                      FOC_MOTION_DISPATCHER_NOT_INITIALIZED;
    }

    key = dispatcher->ops.enter_critical(dispatcher->ops.context);
    dispatcher->urgent_fault_detail |= fault_detail;
    dispatcher->urgent_flags |= FOC_MOTION_REALTIME_REQUEST_FAULT;
    dispatcher->ops.exit_critical(dispatcher->ops.context, key);
    return FOC_MOTION_DISPATCHER_OK;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_consume_isr(
    foc_motion_dispatcher_t *dispatcher,
    uint32_t now_ms,
    foc_motion_realtime_request_t *request)
{
    uint32_t active_slot;
    uint32_t urgent_flags;
    uint32_t urgent_fault_detail;

    if (request == NULL)
    {
        return FOC_MOTION_DISPATCHER_INVALID_ARGUMENT;
    }
    if (!foc_motion_dispatcher_ready(dispatcher))
    {
        foc_motion_dispatcher_empty_request(request);
        return (dispatcher == NULL) ? FOC_MOTION_DISPATCHER_INVALID_ARGUMENT :
                                      FOC_MOTION_DISPATCHER_NOT_INITIALIZED;
    }

    active_slot = dispatcher->active_slot;
    *request = dispatcher->slots[active_slot];
    urgent_flags = dispatcher->urgent_flags &
                   (FOC_MOTION_REALTIME_REQUEST_STOP |
                    FOC_MOTION_REALTIME_REQUEST_FAULT);
    urgent_fault_detail = dispatcher->urgent_fault_detail;
    dispatcher->urgent_flags = 0U;
    dispatcher->urgent_fault_detail = 0U;

    request->consumer_now_ms = now_ms;
    if ((urgent_flags & FOC_MOTION_REALTIME_REQUEST_FAULT) != 0U)
    {
        /* Fault is strictly stronger than a normal stop request. */
        request->request_flags &=
            ~((uint32_t)FOC_MOTION_REALTIME_REQUEST_STOP);
        request->request_flags |= FOC_MOTION_REALTIME_REQUEST_FAULT;
        request->fault_detail = urgent_fault_detail;
    }
    else if ((urgent_flags & FOC_MOTION_REALTIME_REQUEST_STOP) != 0U)
    {
        request->request_flags |= FOC_MOTION_REALTIME_REQUEST_STOP;
    }

    if ((request->request_flags &
         (FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID |
          FOC_MOTION_REALTIME_REQUEST_STOP |
          FOC_MOTION_REALTIME_REQUEST_FAULT)) == 0U)
    {
        return FOC_MOTION_DISPATCHER_NO_COMMAND;
    }

    dispatcher->consume_count++;
    return FOC_MOTION_DISPATCHER_OK;
}

foc_motion_dispatcher_result_t foc_motion_dispatcher_reset_stopped(
    foc_motion_dispatcher_t *dispatcher)
{
    uint32_t key;
    if (!foc_motion_dispatcher_ready(dispatcher))
    {
        return (dispatcher == NULL) ? FOC_MOTION_DISPATCHER_INVALID_ARGUMENT :
                                      FOC_MOTION_DISPATCHER_NOT_INITIALIZED;
    }

    key = dispatcher->ops.enter_critical(dispatcher->ops.context);
    dispatcher->active_slot = 0U;
    dispatcher->next_publication_sequence = 1U;
    dispatcher->urgent_flags = 0U;
    dispatcher->urgent_fault_detail = 0U;
    dispatcher->consume_count = 0U;
    foc_motion_dispatcher_empty_request(&dispatcher->slots[0]);
    foc_motion_dispatcher_empty_request(&dispatcher->slots[1]);
    dispatcher->ops.exit_critical(dispatcher->ops.context, key);
    return FOC_MOTION_DISPATCHER_OK;
}
