#include "foc_dengfoc_as5600_port.h"

#include <stddef.h>
#include <string.h>

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
    uint32_t maximum_sample_period_us)
{
    foc_as5600_config_t sensor_config;
    const foc_dengfoc_axis_profile_t *axis;
    if ((port == NULL) || (board == NULL) || (i2c == NULL) ||
        (i2c->begin == NULL) || (i2c->read_register == NULL) ||
        (axis_index >= FOC_DENGFOC_V04_AXIS_COUNT) ||
        !foc_dengfoc_v04_profile_valid(board))
    {
        return false;
    }
    axis = &board->axis[axis_index];
    (void)memset(port, 0, sizeof(*port));
    (void)memset(&sensor_config, 0, sizeof(sensor_config));
    sensor_config.axis_id = axis_index;
    sensor_config.direction = direction;
    sensor_config.pole_pairs = pole_pairs;
    sensor_config.pole_pair_revision = pole_pair_revision;
    sensor_config.electrical_offset_rad = electrical_offset_rad;
    sensor_config.calibrated = calibrated;
    sensor_config.maximum_sample_period_us = maximum_sample_period_us;
    if (!foc_as5600_tracker_init(&port->tracker, &sensor_config))
    {
        return false;
    }
    port->i2c = *i2c;
    port->i2c_context = i2c_context;
    port->axis_profile = *axis;
    if (!port->i2c.begin(port->i2c_context,
                         axis->i2c_controller,
                         axis->i2c_sda_gpio,
                         axis->i2c_scl_gpio,
                         board->encoder_i2c_frequency_hz))
    {
        (void)memset(port, 0, sizeof(*port));
        return false;
    }
    port->initialized = 1U;
    return true;
}

bool foc_dengfoc_as5600_port_poll(
    foc_dengfoc_as5600_port_t *port,
    uint32_t sampled_at_ms,
    uint32_t sampled_at_us,
    foc_feedback_source_sample_t *output)
{
    uint8_t bytes[2] = {0U, 0U};
    uint16_t raw_count = 0U;
    foc_absolute_encoder_feedback_port_t encoder;
    if (output == NULL)
    {
        return false;
    }
    (void)memset(output, 0, sizeof(*output));
    if ((port == NULL) || (port->initialized == 0U) ||
        !port->i2c.read_register(
            port->i2c_context,
            port->axis_profile.i2c_controller,
            (uint8_t)port->axis_profile.as5600_i2c_address,
            FOC_AS5600_RAW_ANGLE_REGISTER,
            bytes,
            sizeof(bytes)) ||
        !foc_as5600_decode_raw_angle(bytes[0], bytes[1], &raw_count) ||
        !foc_as5600_tracker_update(&port->tracker,
                                   raw_count,
                                   sampled_at_ms,
                                   sampled_at_us,
                                   &encoder) ||
        !foc_feedback_adapt_absolute_encoder(&encoder, output))
    {
        (void)memset(output, 0, sizeof(*output));
        return false;
    }
    return true;
}
