#ifndef FOC_MOTION_BRIDGE_H
#define FOC_MOTION_BRIDGE_H

/*
 * FluxRT P4.2 motion management/realtime sub-ABI V2.
 *
 * This ABI is independent from the 12 kHz FOC_RUST_ABI_VERSION. Configuration
 * and enable/disable belong to the management plane; foc_rust_motion_step() is
 * allocation-free and advances planner + cascade as one transaction.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_config_bridge.h"
#include "foc_product_contract.h"
#include "foc_rust_bridge.h"

#define FOC_MOTION_ABI_VERSION              (0x00020000UL)
#define FOC_MOTION_FEEDBACK_VERSION         (1UL)
#define FOC_MOTION_OUTPUT_VERSION           (1UL)
#define FOC_MOTION_REALTIME_REQUEST_VERSION (1UL)
#define FOC_MOTION_CONTEXT_CAPACITY         (512UL)

typedef uint32_t foc_motion_status_t;
enum
{
    FOC_MOTION_STATUS_OK = 0,
    FOC_MOTION_STATUS_INVALID_ARGUMENT = 1,
    FOC_MOTION_STATUS_NOT_INITIALIZED = 2,
    FOC_MOTION_STATUS_NOT_CONFIGURED = 3,
    FOC_MOTION_STATUS_DISABLED = 4,
    FOC_MOTION_STATUS_INVALID_CONFIG = 5,
    FOC_MOTION_STATUS_INVALID_COMMAND = 6,
    FOC_MOTION_STATUS_INVALID_FEEDBACK = 7,
    FOC_MOTION_STATUS_TRANSITION_REQUIRED = 8,
    FOC_MOTION_STATUS_CONTROL_FAILURE = 9,
    FOC_MOTION_STATUS_INVALID_STATE = 10,
};

enum
{
    FOC_MOTION_FEEDBACK_VALID_POSITION = (1UL << 0),
    FOC_MOTION_FEEDBACK_VALID_VELOCITY = (1UL << 1),
    FOC_MOTION_FEEDBACK_VALID_CURRENT_Q = (1UL << 2),
    FOC_MOTION_FEEDBACK_VALID_KNOWN_MASK =
        FOC_MOTION_FEEDBACK_VALID_POSITION |
        FOC_MOTION_FEEDBACK_VALID_VELOCITY |
        FOC_MOTION_FEEDBACK_VALID_CURRENT_Q,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sequence;
    uint32_t valid_flags;
    float mechanical_position_rad;
    float mechanical_velocity_rad_s;
    float current_q_a;
    uint32_t reserved;
} foc_motion_feedback_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t config_revision;
    uint32_t source_sequence;
    foc_control_mode_t control_mode;
    foc_input_mode_t input_mode;
    uint32_t reference_flags;
    uint32_t cascade_flags;
    uint32_t detail;
    float position_error_rad;
    float velocity_reference_rad_s;
    float velocity_error_rad_s;
    float torque_reference_nm;
    float current_d_reference_a;
    float current_q_reference_a;
    uint32_t reserved;
} foc_motion_output_t;

typedef union
{
    uint8_t bytes[FOC_MOTION_CONTEXT_CAPACITY];
    uint64_t force_alignment;
} foc_motion_context_t;

/*
 * Fixed management -> ISR request consumed by the combined Rust realtime
 * entry. The publisher never mutates a Rust context. `consumer_now_ms` is
 * filled by the ISR dispatcher after copying the active slot; it is therefore
 * the time used for the P1.2 wrapping command-window check, not the publication
 * time of the management task.
 */
enum
{
    FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID = (1UL << 0),
    FOC_MOTION_REALTIME_REQUEST_POSITION_VALID = (1UL << 1),
    FOC_MOTION_REALTIME_REQUEST_STOP = (1UL << 2),
    FOC_MOTION_REALTIME_REQUEST_FAULT = (1UL << 3),
    FOC_MOTION_REALTIME_REQUEST_KNOWN_MASK =
        FOC_MOTION_REALTIME_REQUEST_COMMAND_VALID |
        FOC_MOTION_REALTIME_REQUEST_POSITION_VALID |
        FOC_MOTION_REALTIME_REQUEST_STOP |
        FOC_MOTION_REALTIME_REQUEST_FAULT,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t publication_sequence;
    uint32_t request_flags;
    uint32_t consumer_now_ms;
    uint32_t fault_detail;
    float mechanical_position_rad;
    /* Same wrapping millisecond domain as command timestamps; zero when invalid. */
    uint32_t position_sampled_at_ms;
    foc_product_command_t command;
} foc_motion_realtime_request_t;

uint32_t foc_rust_motion_abi_version(void);
uint32_t foc_rust_motion_context_required_size(void);
uint32_t foc_rust_motion_context_required_align(void);
foc_motion_status_t foc_rust_motion_init(foc_motion_context_t *storage);
foc_motion_status_t foc_rust_motion_configure(
    foc_motion_context_t *storage,
    const foc_config_bundle_t *config);
/*
 * Configure the ISR-owned motion context for a divided outer loop. The bundle
 * carries the fast-loop frequency; outer_loop_divider changes planner/cascade
 * dt to `divider / control_frequency_hz` and must divide the fast loop exactly.
 * Call only while both controller and ADC ISR are stopped.
 */
foc_motion_status_t foc_rust_motion_configure_realtime(
    foc_motion_context_t *storage,
    const foc_config_bundle_t *config,
    uint32_t outer_loop_divider);
foc_motion_status_t foc_rust_motion_enable(foc_motion_context_t *storage);
foc_motion_status_t foc_rust_motion_disable(foc_motion_context_t *storage);
foc_motion_status_t foc_rust_motion_step(
    foc_motion_context_t *storage,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output);

/*
 * Single-FFI combined path. It updates observer/startup, executes the divided
 * motion planner/cascade from same-tick feedback when due, then feeds only the
 * resulting Id/Iq reference into the existing current PI. It never writes CCR,
 * MOE or any peripheral register. Both contexts are exclusively ISR-owned while
 * armed; the management task may publish only through foc_motion_dispatcher.
 */
foc_status_t foc_rust_realtime_step_with_motion(
    foc_rust_context_t *context,
    foc_motion_context_t *motion_context,
    const foc_realtime_input_t *input,
    const foc_motion_realtime_request_t *request,
    foc_output_t *output,
    foc_telemetry_t *telemetry,
    foc_motion_output_t *motion_output);

/* Diagnostic-only no-power timing path.  It forces the motion reference path
 * to execute before the normal startup sequencer reaches closed loop.  The C
 * platform must prove Gate low, MOE/phase channels off and Safety=Disabled
 * before and after every call; its duty result is never an active output. */
foc_status_t foc_rust_realtime_step_with_motion_no_power(
    foc_rust_context_t *context,
    foc_motion_context_t *motion_context,
    const foc_realtime_input_t *input,
    const foc_motion_realtime_request_t *request,
    foc_output_t *output,
    foc_telemetry_t *telemetry,
    foc_motion_output_t *motion_output);

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_motion_feedback_t) == 32U, "motion feedback ABI drifted");
_Static_assert(sizeof(foc_motion_output_t) == 64U, "motion output ABI drifted");
_Static_assert(sizeof(foc_motion_realtime_request_t) == 136U,
               "motion realtime request ABI drifted");
_Static_assert(offsetof(foc_motion_realtime_request_t, command) == 32U,
               "motion realtime command offset drifted");
_Static_assert(sizeof(foc_motion_context_t) == FOC_MOTION_CONTEXT_CAPACITY,
               "motion context capacity drifted");
_Static_assert(_Alignof(foc_motion_context_t) >= 8U, "motion context alignment drifted");
#endif

#endif /* FOC_MOTION_BRIDGE_H */
