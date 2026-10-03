#ifndef FOC_MOTION_DISPATCHER_H
#define FOC_MOTION_DISPATCHER_H

/*
 * FluxRT P4.2E1 management-task -> ADC-ISR dispatcher.
 *
 * Concurrency contract:
 *   - one management-task publisher, one ADC-ISR consumer;
 *   - the task writes only the inactive slot, then swaps one u32 index inside
 *     the supplied critical section;
 *   - the ISR is the only consumer and the only writer of Rust realtime state;
 *   - enter/exit_critical MUST mask the consuming ADC IRQ and provide compiler
 *     plus hardware memory ordering. They may not allocate, block or log;
 *   - hardware Break/driver faults still use the existing direct safety ISR and
 *     do not wait for this mailbox.
 *
 * This module owns no RTOS, HAL, peripheral or Rust context.
 *
 * Pointer contract: `command` passed to publish and `request` passed to consume
 * must not overlap the dispatcher or either internal slot.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_motion_bridge.h"

#define FOC_MOTION_DISPATCHER_VERSION (1UL)

typedef uint32_t foc_motion_dispatcher_result_t;
enum
{
    FOC_MOTION_DISPATCHER_OK = 0,
    FOC_MOTION_DISPATCHER_INVALID_ARGUMENT = 1,
    FOC_MOTION_DISPATCHER_NOT_INITIALIZED = 2,
    FOC_MOTION_DISPATCHER_NO_COMMAND = 3,
};

typedef uint32_t (*foc_motion_dispatcher_enter_critical_fn)(void *context);
typedef void (*foc_motion_dispatcher_exit_critical_fn)(void *context,
                                                       uint32_t key);

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    foc_motion_dispatcher_enter_critical_fn enter_critical;
    foc_motion_dispatcher_exit_critical_fn exit_critical;
    void *context;
} foc_motion_dispatcher_ops_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    volatile uint32_t active_slot;
    uint32_t next_publication_sequence;
    volatile uint32_t urgent_flags;
    volatile uint32_t urgent_fault_detail;
    uint32_t consume_count;
    foc_motion_dispatcher_ops_t ops;
    foc_motion_realtime_request_t slots[2];
} foc_motion_dispatcher_t;

foc_motion_dispatcher_result_t foc_motion_dispatcher_init(
    foc_motion_dispatcher_t *dispatcher,
    const foc_motion_dispatcher_ops_t *ops);

/* Publish one complete command/optional external position snapshot. */
foc_motion_dispatcher_result_t foc_motion_dispatcher_publish(
    foc_motion_dispatcher_t *dispatcher,
    const foc_product_command_t *command,
    uint32_t position_valid,
    float mechanical_position_rad,
    uint32_t position_sampled_at_ms);

/* Task-side normal-stop and fault requests. They are latched until one consume. */
foc_motion_dispatcher_result_t foc_motion_dispatcher_request_stop(
    foc_motion_dispatcher_t *dispatcher);
foc_motion_dispatcher_result_t foc_motion_dispatcher_request_fault(
    foc_motion_dispatcher_t *dispatcher,
    uint32_t fault_detail);

/*
 * ISR-only: copy one stable slot and atomically consume urgent request bits.
 * Once this returns OK, the ISR must immediately call the combined Rust entry;
 * any failure after consumption must fail closed because urgent bits are one-shot.
 */
foc_motion_dispatcher_result_t foc_motion_dispatcher_consume_isr(
    foc_motion_dispatcher_t *dispatcher,
    uint32_t now_ms,
    foc_motion_realtime_request_t *request);

/*
 * Stopped-state only: discard slots and pending requests, preserving callbacks.
 * Pair this with motion-context disable/re-init so both sequence histories reset.
 */
foc_motion_dispatcher_result_t foc_motion_dispatcher_reset_stopped(
    foc_motion_dispatcher_t *dispatcher);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_motion_realtime_request_t) == 136U,
               "motion realtime request layout drifted");
#endif

#endif /* FOC_MOTION_DISPATCHER_H */
