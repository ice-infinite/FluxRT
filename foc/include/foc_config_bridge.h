#ifndef FOC_CONFIG_BRIDGE_H
#define FOC_CONFIG_BRIDGE_H

/*
 * FluxRT recoverable configuration management sub-ABI V4.
 *
 * This contract is independent from FOC_RUST_ABI_VERSION: configuration is a
 * non-realtime management-plane service and must never be called from the ADC
 * ISR. All objects are fixed-size, pointer-free records. The C caller owns the
 * opaque context storage and serializes every access from one management task.
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_external_io.h"
#include "foc_product_contract.h"

#define FOC_CONFIG_ABI_VERSION       (0x00040000UL)
#define FOC_CONFIG_CONTEXT_CAPACITY  (4096UL)
#define FOC_CONFIG_SLOT_COUNT        (2UL)
#define FOC_CONFIG_SLOT_SIZE         (768UL)
#define FOC_CONFIG_MAX_COMMAND_SOURCES (4UL)
#define FOC_CONFIG_GROUP_VERSION       (1UL)

enum
{
    FOC_CONFIG_BOARD_CAP_THREE_SHUNT_CURRENT = (1UL << 0),
    FOC_CONFIG_COMMAND_PERMISSION_RELEASE = (1UL << 0),
    FOC_CONFIG_COMMAND_PERMISSION_AXIS_REQUEST = (1UL << 1),
    FOC_CONFIG_COMMAND_PERMISSION_SETPOINT = (1UL << 2),
    FOC_CONFIG_COMMAND_PERMISSION_CLEAR_FAULT = (1UL << 3),
    FOC_CONFIG_COMMAND_PERMISSION_EMERGENCY_STOP = (1UL << 4),
};

typedef uint32_t foc_config_status_t;
enum
{
    FOC_CONFIG_STATUS_OK = 0,
    FOC_CONFIG_STATUS_INVALID_ARGUMENT = 1,
    FOC_CONFIG_STATUS_NOT_INITIALIZED = 2,
    FOC_CONFIG_STATUS_BUSY = 3,
    FOC_CONFIG_STATUS_NO_TRANSACTION = 4,
    FOC_CONFIG_STATUS_STALE_TOKEN = 5,
    FOC_CONFIG_STATUS_INVALID_STATE = 6,
    FOC_CONFIG_STATUS_EMPTY_TRANSACTION = 7,
    FOC_CONFIG_STATUS_UNSAFE_TO_APPLY = 8,
    FOC_CONFIG_STATUS_VALIDATION_FAILED = 9,
    FOC_CONFIG_STATUS_STORAGE_PLAN_FAILED = 10,
    FOC_CONFIG_STATUS_STORAGE_RECORD_FAILED = 11,
    FOC_CONFIG_STATUS_STORAGE_VERIFICATION_FAILED = 12,
};

typedef uint32_t foc_config_transaction_state_t;
enum
{
    FOC_CONFIG_TRANSACTION_IDLE = 0,
    FOC_CONFIG_TRANSACTION_EDITING = 1,
    FOC_CONFIG_TRANSACTION_VALIDATED = 2,
    FOC_CONFIG_TRANSACTION_APPLY_PREPARED = 3,
    FOC_CONFIG_TRANSACTION_VOLATILE_APPLIED = 4,
    FOC_CONFIG_TRANSACTION_COMMIT_PREPARED = 5,
    FOC_CONFIG_TRANSACTION_STORAGE_FAULT = 6,
};

typedef uint32_t foc_config_apply_class_t;
enum
{
    FOC_CONFIG_APPLY_MANAGEMENT_ONLY = 0,
    FOC_CONFIG_APPLY_AXIS_RESTART = 1,
    FOC_CONFIG_APPLY_PLATFORM_RESTART = 2,
};

enum
{
    FOC_CONFIG_GROUP_BOARD = (1UL << 0),
    FOC_CONFIG_GROUP_MOTOR = (1UL << 1),
    FOC_CONFIG_GROUP_INVERTER = (1UL << 2),
    FOC_CONFIG_GROUP_AXIS = (1UL << 3),
    FOC_CONFIG_GROUP_APP = (1UL << 4),
    FOC_CONFIG_GROUP_CALIBRATION = (1UL << 5),
    FOC_CONFIG_GROUP_EXTERNAL_IO = (1UL << 6),
    FOC_CONFIG_GROUP_KNOWN_MASK =
        FOC_CONFIG_GROUP_BOARD | FOC_CONFIG_GROUP_MOTOR |
        FOC_CONFIG_GROUP_INVERTER | FOC_CONFIG_GROUP_AXIS |
        FOC_CONFIG_GROUP_APP | FOC_CONFIG_GROUP_CALIBRATION |
        FOC_CONFIG_GROUP_EXTERNAL_IO,
};

typedef struct
{
    uint32_t version;
    uint32_t board_id;
    uint32_t pwm_frequency_hz;
    uint32_t control_frequency_hz;
    uint32_t capability_flags;
    float adc_reference_v;
    float current_gain_a_per_count;
    float bus_voltage_v_per_count;
} foc_config_board_t;

typedef struct
{
    uint32_t version;
    uint32_t motor_id;
    uint32_t pole_pairs;
    float phase_resistance_ohm;
    float d_inductance_h;
    float q_inductance_h;
    float flux_linkage_v_s;
    float continuous_current_a;
    float maximum_speed_rad_s;
} foc_config_motor_t;

typedef struct
{
    uint32_t version;
    uint32_t inverter_id;
    float maximum_phase_current_a;
    float minimum_bus_voltage_v;
    float maximum_bus_voltage_v;
    float dead_time_s;
    float transistor_drop_v;
    float brake_resistance_ohm;
    float maximum_duty;
} foc_config_inverter_t;

typedef struct
{
    uint32_t version;
    uint32_t axis_id;
    uint32_t feedback_mode;
    int32_t direction;
    float current_kp;
    float current_ki;
    float velocity_kp;
    float velocity_ki;
    float position_kp;
    float soft_limit_min_rad;
    float soft_limit_max_rad;
    uint32_t motion_control_enabled;
    float torque_ramp_rate_nm_s;
    float velocity_ramp_rate_rad_s2;
    float position_filter_bandwidth_rad_s;
    float trajectory_acceleration_rad_s2;
    float trajectory_deceleration_rad_s2;
} foc_config_axis_t;

typedef struct
{
    uint32_t source_id;
    uint32_t priority;
    uint32_t permissions;
    uint32_t lease_ms;
    uint32_t command_timeout_ms;
} foc_config_command_source_t;

typedef struct
{
    uint32_t version;
    uint32_t can_node_id;
    uint32_t uart_baud;
    uint32_t command_source_count;
    foc_config_command_source_t command_sources[FOC_CONFIG_MAX_COMMAND_SOURCES];
} foc_config_app_t;

typedef struct
{
    uint32_t version;
    uint32_t board_id;
    uint32_t motor_id;
    uint32_t valid_flags;
    float current_offset_counts[3];
    float phase_voltage_gain[3];
    float phase_voltage_offset_v[3];
    float encoder_offset_rad;
    uint32_t encoder_counts_per_revolution;
    uint32_t hall_sequence_packed;
} foc_config_calibration_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t bundle_revision;
    uint32_t generated_by_version;
    foc_config_board_t board;
    foc_config_motor_t motor;
    foc_config_inverter_t inverter;
    foc_config_axis_t axis;
    foc_config_app_t app;
    foc_external_io_config_t external_io;
    foc_config_calibration_t calibration;
} foc_config_bundle_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t axis_state;
    uint32_t drive_active;
    uint32_t active_fault_flags;
} foc_config_apply_guard_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t token;
    uint32_t from_revision;
    uint32_t to_revision;
    uint32_t changed_groups;
    foc_config_apply_class_t apply_class;
    uint32_t reserved;
} foc_config_apply_plan_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    foc_config_transaction_state_t state;
    uint32_t token;
    uint32_t active_revision;
    uint32_t pending_revision;
    uint32_t changed_groups;
    foc_config_status_t last_result;
    uint32_t last_detail;
} foc_config_transaction_status_t;

typedef struct
{
    uint8_t bytes[FOC_CONFIG_SLOT_COUNT][FOC_CONFIG_SLOT_SIZE];
} foc_config_slots_t;

typedef struct
{
    uint8_t bytes[FOC_CONFIG_SLOT_SIZE];
} foc_config_slot_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t token;
    uint32_t target_slot;
    uint32_t sequence;
    uint32_t write_step_count;
    uint32_t reserved0;
    uint32_t reserved1;
} foc_config_commit_plan_t;

typedef struct
{
    uint32_t offset;
    uint32_t value;
} foc_config_write_byte_t;

/* The uint64_t member supplies the 8-byte alignment required by Rust. */
typedef union
{
    uint64_t alignment;
    uint8_t bytes[FOC_CONFIG_CONTEXT_CAPACITY];
} foc_config_context_storage_t;

uint32_t foc_rust_config_abi_version(void);
uint32_t foc_rust_config_context_required_size(void);
uint32_t foc_rust_config_context_required_align(void);
foc_config_status_t foc_rust_config_init(
    foc_config_context_storage_t *storage,
    const foc_config_bundle_t *active,
    uint32_t expected_board_id,
    uint32_t expected_motor_id);
foc_config_status_t foc_rust_config_get_status(
    foc_config_context_storage_t *storage,
    foc_config_transaction_status_t *output);
foc_config_status_t foc_rust_config_get_active(
    foc_config_context_storage_t *storage,
    foc_config_bundle_t *output);
foc_config_status_t foc_rust_config_get_pending(
    foc_config_context_storage_t *storage,
    uint32_t token,
    foc_config_bundle_t *output);
foc_config_status_t foc_rust_config_begin(
    foc_config_context_storage_t *storage,
    uint32_t *token_out);
foc_config_status_t foc_rust_config_set_board(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_board_t *value);
foc_config_status_t foc_rust_config_set_motor(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_motor_t *value);
foc_config_status_t foc_rust_config_set_inverter(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_inverter_t *value);
foc_config_status_t foc_rust_config_set_axis(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_axis_t *value);
foc_config_status_t foc_rust_config_set_app(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_app_t *value);
foc_config_status_t foc_rust_config_set_external_io(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_external_io_config_t *value);
foc_config_status_t foc_rust_config_set_calibration(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_calibration_t *value);
foc_config_status_t foc_rust_config_validate(
    foc_config_context_storage_t *storage, uint32_t token);
foc_config_status_t foc_rust_config_prepare_apply(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_apply_guard_t *guard, foc_config_apply_plan_t *output);
foc_config_status_t foc_rust_config_confirm_apply(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_apply_guard_t *guard, const foc_config_apply_plan_t *plan);
foc_config_status_t foc_rust_config_cancel_apply(
    foc_config_context_storage_t *storage, uint32_t token);
foc_config_status_t foc_rust_config_rollback(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_apply_guard_t *guard);
foc_config_status_t foc_rust_config_prepare_commit(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_apply_guard_t *guard, const foc_config_slots_t *slots,
    foc_config_commit_plan_t *output);
foc_config_status_t foc_rust_config_commit_write_byte(
    foc_config_context_storage_t *storage, uint32_t step,
    foc_config_write_byte_t *output);
foc_config_status_t foc_rust_config_confirm_commit(
    foc_config_context_storage_t *storage, uint32_t token,
    const foc_config_apply_guard_t *guard, uint32_t stored_slot_index,
    const foc_config_slot_t *stored_slot);
foc_config_status_t foc_rust_config_validate_active_for_arm(
    foc_config_context_storage_t *storage);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_config_board_t) == 32U, "config board ABI drifted");
_Static_assert(sizeof(foc_config_motor_t) == 36U, "config motor ABI drifted");
_Static_assert(sizeof(foc_config_inverter_t) == 36U, "config inverter ABI drifted");
_Static_assert(sizeof(foc_config_axis_t) == 68U, "config axis ABI drifted");
_Static_assert(sizeof(foc_config_command_source_t) == 20U, "config source ABI drifted");
_Static_assert(sizeof(foc_config_app_t) == 96U, "config app ABI drifted");
_Static_assert(sizeof(foc_config_calibration_t) == 64U, "config calibration ABI drifted");
_Static_assert(sizeof(foc_config_bundle_t) == 732U, "config bundle ABI drifted");
_Static_assert(sizeof(foc_config_apply_guard_t) == 20U, "config guard ABI drifted");
_Static_assert(sizeof(foc_config_apply_plan_t) == 32U, "config plan ABI drifted");
_Static_assert(sizeof(foc_config_transaction_status_t) == 36U, "config status ABI drifted");
_Static_assert(sizeof(foc_config_commit_plan_t) == 32U, "config commit ABI drifted");
_Static_assert(sizeof(foc_config_write_byte_t) == 8U, "config write ABI drifted");
_Static_assert(sizeof(foc_config_context_storage_t) == FOC_CONFIG_CONTEXT_CAPACITY,
               "config context capacity drifted");
#endif

#endif /* FOC_CONFIG_BRIDGE_H */
