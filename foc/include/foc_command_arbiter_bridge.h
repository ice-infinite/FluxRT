#ifndef FOC_COMMAND_ARBITER_BRIDGE_H
#define FOC_COMMAND_ARBITER_BRIDGE_H

/*
 * FluxRT device-side CommandArbiter management ABI V1.
 *
 * One management task owns the opaque context. Protocol/input adapters submit
 * ProductCommand values through foc_command_service; this ABI performs the
 * trusted Rust source-policy, lease, replay and arbitration logic. It never
 * touches PWM, the realtime controller or MCU registers.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_product_contract.h"

#define FOC_COMMAND_ABI_VERSION       (0x00010000UL)
#define FOC_COMMAND_CONTEXT_CAPACITY  (4096UL)
#define FOC_COMMAND_MAX_SOURCES       (8UL)

typedef uint32_t foc_command_arbiter_status_t;
enum
{
    FOC_COMMAND_ARBITER_OK = 0,
    FOC_COMMAND_ARBITER_INVALID_ARGUMENT = 1,
    FOC_COMMAND_ARBITER_NOT_INITIALIZED = 2,
    FOC_COMMAND_ARBITER_NOT_CONFIGURED = 3,
    FOC_COMMAND_ARBITER_INVALID_CONFIG = 4,
    FOC_COMMAND_ARBITER_INVALID_COMMAND = 5,
    FOC_COMMAND_ARBITER_UNKNOWN_SOURCE = 6,
    FOC_COMMAND_ARBITER_UNAUTHORIZED = 7,
    FOC_COMMAND_ARBITER_INVALID_TIME = 8,
    FOC_COMMAND_ARBITER_DUPLICATE_SEQUENCE = 9,
    FOC_COMMAND_ARBITER_OUT_OF_ORDER_SEQUENCE = 10,
};

typedef uint32_t foc_command_decision_kind_t;
enum
{
    FOC_COMMAND_DECISION_SAFE = 0,
    FOC_COMMAND_DECISION_SELECTED = 1,
};

typedef uint32_t foc_command_safe_reason_t;
enum
{
    FOC_COMMAND_SAFE_NO_COMMAND = 0,
    FOC_COMMAND_SAFE_FAULT_ACTIVE = 1,
    FOC_COMMAND_SAFE_EMERGENCY_STOP_LATCHED = 2,
    FOC_COMMAND_SAFE_COMMAND_TIMEOUT = 3,
};

typedef struct
{
    uint32_t source_id;
    uint32_t priority;
    uint32_t permissions;
    uint32_t lease_ms;
    uint32_t command_timeout_ms;
} foc_command_source_policy_abi_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    foc_command_decision_kind_t decision;
    foc_command_safe_reason_t safe_reason;
    uint32_t source_id;
    uint32_t has_setpoint;
    uint32_t has_action;
    uint32_t reserved;
    foc_product_command_t setpoint;
    foc_product_command_t action;
} foc_command_decision_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t configured_source_count;
    uint32_t submitted;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t polls;
    foc_command_arbiter_status_t last_result;
    uint32_t last_detail;
} foc_command_arbiter_snapshot_t;

/* The uint64_t member supplies the alignment required by Rust. */
typedef union
{
    uint64_t alignment;
    uint8_t bytes[FOC_COMMAND_CONTEXT_CAPACITY];
} foc_command_context_storage_t;

uint32_t foc_rust_command_abi_version(void);
uint32_t foc_rust_command_context_required_size(void);
uint32_t foc_rust_command_context_required_align(void);
foc_command_arbiter_status_t foc_rust_command_init(
    foc_command_context_storage_t *storage);
foc_command_arbiter_status_t foc_rust_command_configure(
    foc_command_context_storage_t *storage,
    const foc_command_source_policy_abi_t *sources,
    uint32_t source_count);
foc_command_arbiter_status_t foc_rust_command_submit(
    foc_command_context_storage_t *storage,
    const foc_product_command_t *command,
    uint32_t now_ms);
foc_command_arbiter_status_t foc_rust_command_poll(
    foc_command_context_storage_t *storage,
    uint32_t now_ms,
    uint32_t fault_active,
    foc_command_decision_t *output);
foc_command_arbiter_status_t foc_rust_command_get_status(
    foc_command_context_storage_t *storage,
    foc_command_arbiter_snapshot_t *output);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_command_source_policy_abi_t) == 20U,
               "command source ABI drifted");
_Static_assert(sizeof(foc_command_decision_t) == 240U,
               "command decision ABI drifted");
_Static_assert(sizeof(foc_command_arbiter_snapshot_t) == 36U,
               "command snapshot ABI drifted");
_Static_assert(sizeof(foc_command_context_storage_t) == FOC_COMMAND_CONTEXT_CAPACITY,
               "command context capacity drifted");
#endif

#endif /* FOC_COMMAND_ARBITER_BRIDGE_H */
