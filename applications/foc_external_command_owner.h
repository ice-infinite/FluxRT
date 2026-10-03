#ifndef FOC_EXTERNAL_COMMAND_OWNER_H
#define FOC_EXTERNAL_COMMAND_OWNER_H

/*
 * Single low-frequency owner of CommandArbiter polling.
 *
 * The owner publishes a read-only event snapshot. It deliberately has no
 * MotorService callback and cannot arm, start realtime control, or touch PWM.
 */

#include "foc_command_rust_adapter.h"

#define FOC_EXTERNAL_COMMAND_OWNER_VERSION (1UL)
#define FOC_EXTERNAL_COMMAND_EVENT_VERSION (1UL)

typedef uint32_t foc_external_command_owner_result_t;
enum
{
    FOC_EXTERNAL_COMMAND_OWNER_OK = 0,
    FOC_EXTERNAL_COMMAND_OWNER_INVALID_ARGUMENT = 1,
    FOC_EXTERNAL_COMMAND_OWNER_NOT_INITIALIZED = 2,
    FOC_EXTERNAL_COMMAND_OWNER_POLL_FAILED = 3,
    FOC_EXTERNAL_COMMAND_OWNER_STATUS_FAILED = 4,
};

typedef uint32_t foc_external_command_event_kind_t;
enum
{
    FOC_EXTERNAL_COMMAND_EVENT_NONE = 0,
    FOC_EXTERNAL_COMMAND_EVENT_SAFE = 1,
    FOC_EXTERNAL_COMMAND_EVENT_SELECTED = 2,
    FOC_EXTERNAL_COMMAND_EVENT_POLL_FAILED = 3,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sequence;
    uint32_t timestamp_ms;
    foc_external_command_event_kind_t kind;
    foc_command_arbiter_status_t poll_status;
    uint32_t reserved0;
    uint32_t reserved1;
    foc_command_decision_t decision;
    foc_command_arbiter_snapshot_t arbiter;
} foc_external_command_event_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t initialized;
    uint32_t next_sequence;
    foc_external_command_event_t last_event;
} foc_external_command_owner_t;

foc_external_command_owner_result_t foc_external_command_owner_init(
    foc_external_command_owner_t *owner);

foc_external_command_owner_result_t foc_external_command_owner_tick(
    foc_external_command_owner_t *owner,
    foc_command_rust_adapter_t *adapter,
    uint32_t now_ms,
    uint32_t fault_active);

foc_external_command_owner_result_t foc_external_command_owner_get_event(
    const foc_external_command_owner_t *owner,
    foc_external_command_event_t *output);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_external_command_event_t) == 308U,
               "external command event layout drifted");
#endif

#endif /* FOC_EXTERNAL_COMMAND_OWNER_H */
