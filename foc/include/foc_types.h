#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>

typedef uint32_t foc_status_t;
enum
{
    FOC_STATUS_OK = 0,
    FOC_STATUS_DISABLED,
    FOC_STATUS_NOT_CONFIGURED,
    FOC_STATUS_INVALID_ARGUMENT,
    FOC_STATUS_HARDWARE_FAULT,
};

typedef uint32_t foc_state_t;
enum
{
    FOC_STATE_UNINITIALIZED = 0,
    FOC_STATE_DISABLED,
    FOC_STATE_RUNNING,
    FOC_STATE_ALIGNMENT,
    FOC_STATE_OPEN_LOOP_RAMP,
    FOC_STATE_OPEN_LOOP_HOLD,
    FOC_STATE_OBSERVER_TRANSITION,
    FOC_STATE_CLOSED_LOOP,
    FOC_STATE_FAULT,
};

typedef struct
{
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float dc_bus_voltage;
    float electrical_angle_rad;
} foc_feedback_t;

typedef struct
{
    float id_ref;
    float iq_ref;
} foc_reference_t;

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} foc_output_t;

#endif
