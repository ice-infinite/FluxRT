#ifndef DENGFOC_ARDUINO_POWER_PORT_H
#define DENGFOC_ARDUINO_POWER_PORT_H

extern "C" {
#include "foc_dengfoc_power_stage.h"
}

typedef struct
{
    const foc_dengfoc_board_profile_t *profile;
    uint32_t axis;
    uint32_t pwm_channel[FOC_DENGFOC_PHASE_COUNT];
    uint32_t pwm_configured;
    uint32_t runtime_actuation_permitted;
} dengfoc_arduino_power_port_t;

typedef struct
{
    uint32_t pwm_configured;
    uint32_t driver_disabled;
    uint32_t frequency_hz[FOC_DENGFOC_PHASE_COUNT];
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT];
} dengfoc_arduino_power_snapshot_t;

bool dengfoc_arduino_power_port_init(
    dengfoc_arduino_power_port_t *port,
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    bool runtime_actuation_permitted,
    foc_dengfoc_power_ops_t *ops);

bool dengfoc_arduino_power_port_force_disabled(
    dengfoc_arduino_power_port_t *port);

bool dengfoc_arduino_power_port_is_disabled(
    dengfoc_arduino_power_port_t *port);

bool dengfoc_arduino_power_port_snapshot(
    dengfoc_arduino_power_port_t *port,
    dengfoc_arduino_power_snapshot_t *snapshot);

#endif /* DENGFOC_ARDUINO_POWER_PORT_H */
