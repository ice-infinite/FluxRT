#ifndef DENGFOC_MCPWM_POWER_PORT_H
#define DENGFOC_MCPWM_POWER_PORT_H

#include <driver/mcpwm_prelude.h>

extern "C" {
#include "foc_dengfoc_power_stage.h"
}

typedef struct
{
    const foc_dengfoc_board_profile_t *profile;
    uint32_t axis;
    uint32_t timer_resolution_hz;
    uint32_t period_ticks;
    uint32_t peak_ticks;
    uint32_t actual_frequency_hz;
    uint32_t maximum_duty_count;
    uint32_t pwm_configured;
    uint32_t runtime_actuation_permitted;
    uint32_t outputs_released;
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT];
    uint32_t coherent_update_count;
    uint32_t last_update_step;
    int32_t last_update_error;
    volatile uint32_t full_event_count;
    volatile uint32_t last_full_cpu_cycle;
    void (*cycle_event_callback)(void *context,
                                 uint32_t event_count,
                                 uint32_t cpu_cycle);
    void *cycle_event_context;
    void (*divided_cycle_event_callback)(void *context,
                                         uint32_t event_count,
                                         uint32_t cpu_cycle);
    void *divided_cycle_event_context;
    uint32_t divided_cycle_event_divider;
    volatile uint32_t divided_cycle_event_phase;
    volatile uint32_t divided_cycle_event_sequence;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operators[FOC_DENGFOC_PHASE_COUNT];
    mcpwm_cmpr_handle_t comparators[FOC_DENGFOC_PHASE_COUNT];
    mcpwm_gen_handle_t generators[FOC_DENGFOC_PHASE_COUNT];
} dengfoc_mcpwm_power_port_t;

typedef struct
{
    uint32_t pwm_configured;
    uint32_t driver_disabled;
    uint32_t frequency_hz[FOC_DENGFOC_PHASE_COUNT];
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT];
    uint32_t capabilities;
    uint32_t period_ticks;
    uint32_t peak_ticks;
    uint32_t full_event_count;
    uint32_t coherent_update_count;
    uint32_t last_update_step;
    int32_t last_update_error;
} dengfoc_mcpwm_power_snapshot_t;

bool dengfoc_mcpwm_power_port_init(
    dengfoc_mcpwm_power_port_t *port,
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    bool runtime_actuationPermitted,
    foc_dengfoc_power_ops_t *ops);

bool dengfoc_mcpwm_power_port_force_disabled(
    dengfoc_mcpwm_power_port_t *port);

bool dengfoc_mcpwm_power_port_is_disabled(
    dengfoc_mcpwm_power_port_t *port);

bool dengfoc_mcpwm_power_port_snapshot(
    dengfoc_mcpwm_power_port_t *port,
    dengfoc_mcpwm_power_snapshot_t *snapshot);

void dengfoc_mcpwm_power_port_set_cycle_event_callback(
    dengfoc_mcpwm_power_port_t *port,
    void (*callback)(void *context,
                     uint32_t event_count,
                     uint32_t cpu_cycle),
    void *context);

void dengfoc_mcpwm_power_port_set_divided_cycle_event_callback(
    dengfoc_mcpwm_power_port_t *port,
    uint32_t divider,
    void (*callback)(void *context,
                     uint32_t event_count,
                     uint32_t cpu_cycle),
    void *context);

#endif /* DENGFOC_MCPWM_POWER_PORT_H */
