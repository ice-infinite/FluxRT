#ifndef FOC_PRODUCT_CONTRACT_H
#define FOC_PRODUCT_CONTRACT_H

/*
 * FluxRT product contract V1.
 *
 * This is the stable product-layer vocabulary shared by firmware, PC
 * simulation, MATLAB/Simulink and future host tools. It is intentionally
 * separate from foc_state_t and foc_realtime_input_t: those remain the current
 * controller/ISR contract, while this header describes Device/Axis behaviour.
 *
 * ABI rules:
 *   - all enum values cross the boundary as uint32_t;
 *   - structs contain only fixed-width scalars, never pointers or C bool;
 *   - physical fields use SI units named in their suffix;
 *   - unknown versions, enum values, flags and combinations are rejected;
 *   - values may only be appended under a new product contract version.
 */

#include <stddef.h>
#include <stdint.h>

#define FOC_PRODUCT_CONTRACT_VERSION        (0x00010000UL)
#define FOC_PRODUCT_COMMAND_VERSION         (1UL)
#define FOC_PRODUCT_FEEDBACK_VERSION        (1UL)
#define FOC_PRODUCT_TELEMETRY_VERSION       (1UL)
#define FOC_PRODUCT_FAULT_SNAPSHOT_VERSION  (1UL)

typedef uint32_t foc_axis_state_t;
enum
{
    FOC_AXIS_STATE_UNINITIALIZED = 0,
    FOC_AXIS_STATE_DISABLED = 1,
    FOC_AXIS_STATE_CALIBRATION = 2,
    FOC_AXIS_STATE_STARTUP = 3,
    FOC_AXIS_STATE_CLOSED_LOOP = 4,
    FOC_AXIS_STATE_STOPPING = 5,
    FOC_AXIS_STATE_FAULT_LATCHED = 6,
};

typedef uint32_t foc_axis_request_t;
enum
{
    FOC_AXIS_REQUEST_NONE = 0,
    FOC_AXIS_REQUEST_DISABLED = 1,
    FOC_AXIS_REQUEST_CURRENT_OFFSET_CALIBRATION = 2,
    FOC_AXIS_REQUEST_MOTOR_IDENTIFICATION = 3,
    FOC_AXIS_REQUEST_ENCODER_INDEX_SEARCH = 4,
    FOC_AXIS_REQUEST_ENCODER_OFFSET_CALIBRATION = 5,
    FOC_AXIS_REQUEST_HALL_CALIBRATION = 6,
    FOC_AXIS_REQUEST_HOMING = 7,
    FOC_AXIS_REQUEST_OPEN_LOOP_TEST = 8,
    FOC_AXIS_REQUEST_CLOSED_LOOP_CONTROL = 9,
};

typedef uint32_t foc_control_mode_t;
enum
{
    FOC_CONTROL_MODE_INACTIVE = 0,
    FOC_CONTROL_MODE_VOLTAGE = 1,
    FOC_CONTROL_MODE_DUTY = 2,
    FOC_CONTROL_MODE_CURRENT = 3,
    FOC_CONTROL_MODE_TORQUE = 4,
    FOC_CONTROL_MODE_VELOCITY = 5,
    FOC_CONTROL_MODE_POSITION = 6,
};

typedef uint32_t foc_input_mode_t;
enum
{
    FOC_INPUT_MODE_INACTIVE = 0,
    FOC_INPUT_MODE_PASSTHROUGH = 1,
    FOC_INPUT_MODE_TORQUE_RAMP = 2,
    FOC_INPUT_MODE_VELOCITY_RAMP = 3,
    FOC_INPUT_MODE_POSITION_FILTER = 4,
    FOC_INPUT_MODE_TRAPEZOIDAL_TRAJECTORY = 5,
    FOC_INPUT_MODE_EXTERNAL_SYNCHRONIZED = 6,
};

typedef uint32_t foc_feedback_mode_t;
enum
{
    FOC_FEEDBACK_MODE_SENSORLESS = 0,
    FOC_FEEDBACK_MODE_HALL = 1,
    FOC_FEEDBACK_MODE_INCREMENTAL_ENCODER = 2,
    FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER = 3,
    FOC_FEEDBACK_MODE_RESOLVER = 4,
    FOC_FEEDBACK_MODE_FUSED = 5,
};

typedef uint32_t foc_product_command_kind_t;
enum
{
    FOC_PRODUCT_COMMAND_RELEASE = 0,
    FOC_PRODUCT_COMMAND_AXIS_REQUEST = 1,
    FOC_PRODUCT_COMMAND_SETPOINT = 2,
    FOC_PRODUCT_COMMAND_CLEAR_FAULT = 3,
    FOC_PRODUCT_COMMAND_EMERGENCY_STOP = 4,
};

enum
{
    FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT = (1UL << 0),
    FOC_PRODUCT_COMMAND_FLAG_TORQUE_LIMIT = (1UL << 1),
    FOC_PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT = (1UL << 2),
    FOC_PRODUCT_COMMAND_FLAG_KNOWN_MASK =
        FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT |
        FOC_PRODUCT_COMMAND_FLAG_TORQUE_LIMIT |
        FOC_PRODUCT_COMMAND_FLAG_VELOCITY_LIMIT,
};

/*
 * Canonical command envelope. Physical setpoints are SI:
 * [V], [A], [N*m], [rad/s], [rad]. duty_ref is signed [-1, 1].
 * created_at_ms/valid_until_ms are wrapping monotonic milliseconds; P1.2 owns
 * lease comparison and source arbitration.
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t axis_id;
    uint32_t source_id;
    uint32_t sequence;
    uint32_t created_at_ms;
    uint32_t valid_until_ms;
    foc_product_command_kind_t command_kind;
    foc_axis_request_t axis_request;
    foc_control_mode_t control_mode;
    foc_input_mode_t input_mode;
    foc_feedback_mode_t feedback_mode;
    uint32_t flags;
    float duty_ref;
    float voltage_d_ref_v;
    float voltage_q_ref_v;
    float current_d_ref_a;
    float current_q_ref_a;
    float torque_ref_nm;
    float velocity_ref_rad_s;
    float position_ref_rad;
    float velocity_feedforward_rad_s;
    float torque_feedforward_nm;
    float current_limit_a;
    float torque_limit_nm;
    float velocity_limit_rad_s;
} foc_product_command_t;

enum
{
    FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION = (1UL << 0),
    FOC_PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION = (1UL << 1),
    FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY = (1UL << 2),
    FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE = (1UL << 3),
    FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY = (1UL << 4),
    FOC_PRODUCT_FEEDBACK_VALID_KNOWN_MASK =
        FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_POSITION |
        FOC_PRODUCT_FEEDBACK_VALID_MULTI_TURN_POSITION |
        FOC_PRODUCT_FEEDBACK_VALID_MECHANICAL_VELOCITY |
        FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_ANGLE |
        FOC_PRODUCT_FEEDBACK_VALID_ELECTRICAL_VELOCITY,
};

enum
{
    FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED = (1UL << 0),
    FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND = (1UL << 1),
    FOC_PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID = (1UL << 2),
    FOC_PRODUCT_FEEDBACK_QUALITY_STALE = (1UL << 3),
    FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED = (1UL << 4),
    FOC_PRODUCT_FEEDBACK_QUALITY_KNOWN_MASK =
        FOC_PRODUCT_FEEDBACK_QUALITY_CALIBRATED |
        FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND |
        FOC_PRODUCT_FEEDBACK_QUALITY_DIRECTION_VALID |
        FOC_PRODUCT_FEEDBACK_QUALITY_STALE |
        FOC_PRODUCT_FEEDBACK_QUALITY_DEGRADED,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t axis_id;
    uint32_t sequence;
    uint32_t sampled_at_ms;
    uint32_t sample_age_us;
    uint32_t valid_flags;
    uint32_t quality_flags;
    foc_feedback_mode_t active_feedback_mode;
    foc_feedback_mode_t backup_feedback_mode;
    int32_t direction;
    uint32_t pole_pair_revision;
    float mechanical_position_rad;
    float multi_turn_position_rad;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    float electrical_velocity_rad_s;
} foc_product_feedback_snapshot_t;

enum
{
    FOC_PRODUCT_TELEMETRY_VALID_POSITION_REFERENCE = (1UL << 0),
    FOC_PRODUCT_TELEMETRY_VALID_POSITION_MEASURED = (1UL << 1),
    FOC_PRODUCT_TELEMETRY_VALID_POSITION_ESTIMATED = (1UL << 2),
    FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_REFERENCE = (1UL << 3),
    FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_MEASURED = (1UL << 4),
    FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_ESTIMATED = (1UL << 5),
    FOC_PRODUCT_TELEMETRY_VALID_TORQUE_REFERENCE = (1UL << 6),
    FOC_PRODUCT_TELEMETRY_VALID_TORQUE_ESTIMATED = (1UL << 7),
    FOC_PRODUCT_TELEMETRY_VALID_CURRENT_REFERENCE = (1UL << 8),
    FOC_PRODUCT_TELEMETRY_VALID_CURRENT_MEASURED = (1UL << 9),
    FOC_PRODUCT_TELEMETRY_VALID_VOLTAGE_COMMAND = (1UL << 10),
    FOC_PRODUCT_TELEMETRY_VALID_DC_BUS_VOLTAGE = (1UL << 11),
    FOC_PRODUCT_TELEMETRY_VALID_MOTOR_TEMPERATURE = (1UL << 12),
    FOC_PRODUCT_TELEMETRY_VALID_KNOWN_MASK =
        FOC_PRODUCT_TELEMETRY_VALID_POSITION_REFERENCE |
        FOC_PRODUCT_TELEMETRY_VALID_POSITION_MEASURED |
        FOC_PRODUCT_TELEMETRY_VALID_POSITION_ESTIMATED |
        FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_REFERENCE |
        FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_MEASURED |
        FOC_PRODUCT_TELEMETRY_VALID_VELOCITY_ESTIMATED |
        FOC_PRODUCT_TELEMETRY_VALID_TORQUE_REFERENCE |
        FOC_PRODUCT_TELEMETRY_VALID_TORQUE_ESTIMATED |
        FOC_PRODUCT_TELEMETRY_VALID_CURRENT_REFERENCE |
        FOC_PRODUCT_TELEMETRY_VALID_CURRENT_MEASURED |
        FOC_PRODUCT_TELEMETRY_VALID_VOLTAGE_COMMAND |
        FOC_PRODUCT_TELEMETRY_VALID_DC_BUS_VOLTAGE |
        FOC_PRODUCT_TELEMETRY_VALID_MOTOR_TEMPERATURE,
};

/*
 * Reference, measured, estimated and controller-selected values are separate.
 * A consumer must never relabel an estimate as an independent measurement.
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t axis_id;
    uint32_t sequence;
    uint32_t captured_at_ms;
    foc_axis_state_t axis_state;
    foc_control_mode_t control_mode;
    foc_input_mode_t input_mode;
    foc_feedback_mode_t feedback_mode;
    uint32_t active_source_id;
    uint32_t fault_flags;
    uint32_t valid_flags;
    float position_reference_rad;
    float position_measured_rad;
    float position_estimated_rad;
    float position_used_rad;
    float velocity_reference_rad_s;
    float velocity_measured_rad_s;
    float velocity_estimated_rad_s;
    float velocity_used_rad_s;
    float torque_reference_nm;
    float torque_estimated_nm;
    float current_d_reference_a;
    float current_q_reference_a;
    float current_d_measured_a;
    float current_q_measured_a;
    float voltage_d_command_v;
    float voltage_q_command_v;
    float dc_bus_voltage_v;
    float motor_temperature_c;
} foc_product_telemetry_snapshot_t;

enum
{
    FOC_PRODUCT_FAULT_EMERGENCY_STOP = (1UL << 0),
    FOC_PRODUCT_FAULT_DRIVER = (1UL << 1),
    FOC_PRODUCT_FAULT_BREAK = (1UL << 2),
    FOC_PRODUCT_FAULT_OVERCURRENT = (1UL << 3),
    FOC_PRODUCT_FAULT_BUS_UNDERVOLTAGE = (1UL << 4),
    FOC_PRODUCT_FAULT_BUS_OVERVOLTAGE = (1UL << 5),
    FOC_PRODUCT_FAULT_ADC_SAMPLE = (1UL << 6),
    FOC_PRODUCT_FAULT_DEADLINE = (1UL << 7),
    FOC_PRODUCT_FAULT_INVALID_FEEDBACK = (1UL << 8),
    FOC_PRODUCT_FAULT_OBSERVER_STARTUP = (1UL << 9),
    FOC_PRODUCT_FAULT_OBSERVER_LOST = (1UL << 10),
    FOC_PRODUCT_FAULT_OUTPUT_REJECTED = (1UL << 11),
    FOC_PRODUCT_FAULT_OVERTEMPERATURE = (1UL << 12),
    FOC_PRODUCT_FAULT_COMMAND_TIMEOUT = (1UL << 13),
    FOC_PRODUCT_FAULT_CONFIG_INVALID = (1UL << 14),
    FOC_PRODUCT_FAULT_KNOWN_MASK =
        FOC_PRODUCT_FAULT_EMERGENCY_STOP |
        FOC_PRODUCT_FAULT_DRIVER |
        FOC_PRODUCT_FAULT_BREAK |
        FOC_PRODUCT_FAULT_OVERCURRENT |
        FOC_PRODUCT_FAULT_BUS_UNDERVOLTAGE |
        FOC_PRODUCT_FAULT_BUS_OVERVOLTAGE |
        FOC_PRODUCT_FAULT_ADC_SAMPLE |
        FOC_PRODUCT_FAULT_DEADLINE |
        FOC_PRODUCT_FAULT_INVALID_FEEDBACK |
        FOC_PRODUCT_FAULT_OBSERVER_STARTUP |
        FOC_PRODUCT_FAULT_OBSERVER_LOST |
        FOC_PRODUCT_FAULT_OUTPUT_REJECTED |
        FOC_PRODUCT_FAULT_OVERTEMPERATURE |
        FOC_PRODUCT_FAULT_COMMAND_TIMEOUT |
        FOC_PRODUCT_FAULT_CONFIG_INVALID,
};

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t axis_id;
    uint32_t sequence;
    uint32_t first_occurred_at_ms;
    uint32_t last_occurred_at_ms;
    uint32_t occurrence_count;
    uint32_t latched_fault_flags;
    uint32_t active_fault_flags;
    uint32_t primary_fault;
    foc_axis_state_t axis_state_at_fault;
    foc_control_mode_t control_mode_at_fault;
    foc_input_mode_t input_mode_at_fault;
    foc_feedback_mode_t feedback_mode_at_fault;
    float dc_bus_voltage_v;
    float peak_phase_current_a;
    float mechanical_velocity_rad_s;
    float motor_temperature_c;
} foc_product_fault_snapshot_t;

#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_axis_state_t) == 4U, "axis state ABI changed");
_Static_assert(sizeof(foc_control_mode_t) == 4U, "control mode ABI changed");
_Static_assert(sizeof(foc_product_command_t) == 104U, "product command ABI changed");
_Static_assert(_Alignof(foc_product_command_t) == 4U, "product command alignment changed");
_Static_assert(offsetof(foc_product_command_t, flags) == 48U, "product command flags offset changed");
_Static_assert(offsetof(foc_product_command_t, duty_ref) == 52U, "product command duty offset changed");
_Static_assert(offsetof(foc_product_command_t, current_d_ref_a) == 64U,
               "product command current offset changed");
_Static_assert(offsetof(foc_product_command_t, velocity_ref_rad_s) == 76U,
               "product command velocity offset changed");
_Static_assert(offsetof(foc_product_command_t, velocity_limit_rad_s) == 100U,
               "product command limit offset changed");
_Static_assert(sizeof(foc_product_feedback_snapshot_t) == 68U,
               "product feedback ABI changed");
_Static_assert(_Alignof(foc_product_feedback_snapshot_t) == 4U,
               "product feedback alignment changed");
_Static_assert(offsetof(foc_product_feedback_snapshot_t, active_feedback_mode) == 32U,
               "product feedback mode offset changed");
_Static_assert(offsetof(foc_product_feedback_snapshot_t, mechanical_position_rad) == 48U,
               "product feedback value offset changed");
_Static_assert(sizeof(foc_product_telemetry_snapshot_t) == 120U,
               "product telemetry ABI changed");
_Static_assert(_Alignof(foc_product_telemetry_snapshot_t) == 4U,
               "product telemetry alignment changed");
_Static_assert(offsetof(foc_product_telemetry_snapshot_t, valid_flags) == 44U,
               "product telemetry flags offset changed");
_Static_assert(offsetof(foc_product_telemetry_snapshot_t, position_reference_rad) == 48U,
               "product telemetry value offset changed");
_Static_assert(offsetof(foc_product_telemetry_snapshot_t, dc_bus_voltage_v) == 112U,
               "product telemetry bus offset changed");
_Static_assert(sizeof(foc_product_fault_snapshot_t) == 72U,
               "product fault ABI changed");
_Static_assert(_Alignof(foc_product_fault_snapshot_t) == 4U,
               "product fault alignment changed");
_Static_assert(offsetof(foc_product_fault_snapshot_t, latched_fault_flags) == 28U,
               "product fault flags offset changed");
_Static_assert(offsetof(foc_product_fault_snapshot_t, primary_fault) == 36U,
               "product primary fault offset changed");
_Static_assert(offsetof(foc_product_fault_snapshot_t, dc_bus_voltage_v) == 56U,
               "product fault evidence offset changed");
#endif

#endif
