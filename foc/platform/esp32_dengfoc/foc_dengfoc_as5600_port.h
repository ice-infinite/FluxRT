#ifndef FOC_DENGFOC_AS5600_PORT_H
#define FOC_DENGFOC_AS5600_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "foc_as5600.h"
#include "foc_board_dengfoc_v04.h"

/* The ESP32 Arduino/ESP-IDF binding implements only these two operations.
 * All angle semantics, freshness, unwrapping and FluxRT ABI normalization stay
 * below that binding and remain host-testable. */
typedef struct
{
    bool (*begin)(
        void *context,
        uint32_t controller,
        uint32_t sda_gpio,
        uint32_t scl_gpio,
        uint32_t frequency_hz);
    bool (*read_register)(
        void *context,
        uint32_t controller,
        uint8_t address,
        uint8_t register_address,
        uint8_t *data,
        size_t length);
} foc_dengfoc_i2c_ops_t;

typedef struct
{
    foc_dengfoc_i2c_ops_t i2c;
    void *i2c_context;
    foc_dengfoc_axis_profile_t axis_profile;
    foc_as5600_tracker_t tracker;
    uint32_t initialized;
} foc_dengfoc_as5600_port_t;

bool foc_dengfoc_as5600_port_init(
    foc_dengfoc_as5600_port_t *port,
    const foc_dengfoc_board_profile_t *board,
    uint32_t axis_index,
    const foc_dengfoc_i2c_ops_t *i2c,
    void *i2c_context,
    int32_t direction,
    uint32_t pole_pairs,
    uint32_t pole_pair_revision,
    float electrical_offset_rad,
    uint32_t calibrated,
    uint32_t maximum_sample_period_us);

bool foc_dengfoc_as5600_port_poll(
    foc_dengfoc_as5600_port_t *port,
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_feedback_source_sample_t *output);

#endif /* FOC_DENGFOC_AS5600_PORT_H */
