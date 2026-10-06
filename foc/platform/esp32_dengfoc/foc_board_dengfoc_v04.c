#include "foc_board_dengfoc_v04.h"

#include <math.h>
#include <stddef.h>

#include "foc_as5600.h"

const foc_dengfoc_board_profile_t foc_dengfoc_v04_reference_profile = {
    .version = FOC_DENGFOC_V04_PROFILE_VERSION,
    .esp32_cpu_frequency_hz = 240000000U,
    .pwm_frequency_hz = 30000U,
    .pwm_resolution_bits = 8U,
    .reference_control_frequency_hz = 2000U,
    .driver_enable_gpio = 12U,
    .driver_enable_active_high = 1U,
    .default_enabled_axis_mask = FOC_DENGFOC_V04_DEFAULT_AXIS_MASK,
    .encoder_counts_per_revolution = FOC_AS5600_COUNTS_PER_REVOLUTION,
    .encoder_i2c_frequency_hz = 400000U,
    .nominal_bus_voltage_v = 12.0F,
    .adc_reference_v = 3.3F,
    .adc_full_scale_count = 4095U,
    .shunt_resistance_ohm = 0.01F,
    .current_amplifier_gain = 50.0F,
    .current_polarity = 1,
    .axis = {
        {
            .pwm_a_gpio = 32U,
            .pwm_b_gpio = 33U,
            .pwm_c_gpio = 25U,
            .current_a_adc_gpio = 39U,
            .current_b_adc_gpio = 36U,
            .i2c_controller = 0U,
            .i2c_sda_gpio = 19U,
            .i2c_scl_gpio = 18U,
            .as5600_i2c_address = FOC_AS5600_I2C_ADDRESS,
        },
        {
            .pwm_a_gpio = 26U,
            .pwm_b_gpio = 27U,
            .pwm_c_gpio = 14U,
            .current_a_adc_gpio = 35U,
            .current_b_adc_gpio = 34U,
            .i2c_controller = 1U,
            .i2c_sda_gpio = 23U,
            .i2c_scl_gpio = 5U,
            .as5600_i2c_address = FOC_AS5600_I2C_ADDRESS,
        },
    },
};

bool foc_dengfoc_v04_profile_valid(
    const foc_dengfoc_board_profile_t *profile)
{
    uint32_t axis_index;
    if ((profile == NULL) ||
        (profile->version != FOC_DENGFOC_V04_PROFILE_VERSION) ||
        (profile->esp32_cpu_frequency_hz == 0U) ||
        (profile->pwm_frequency_hz == 0U) ||
        (profile->pwm_resolution_bits == 0U) ||
        (profile->reference_control_frequency_hz == 0U) ||
        (profile->driver_enable_active_high > 1U) ||
        ((profile->default_enabled_axis_mask & ~0x03U) != 0U) ||
        (profile->encoder_counts_per_revolution !=
         FOC_AS5600_COUNTS_PER_REVOLUTION) ||
        (profile->encoder_i2c_frequency_hz == 0U) ||
        !isfinite(profile->nominal_bus_voltage_v) ||
        !(profile->nominal_bus_voltage_v > 0.0F) ||
        !isfinite(profile->adc_reference_v) ||
        !(profile->adc_reference_v > 0.0F) ||
        (profile->adc_full_scale_count == 0U) ||
        !isfinite(profile->shunt_resistance_ohm) ||
        !(profile->shunt_resistance_ohm > 0.0F) ||
        !isfinite(profile->current_amplifier_gain) ||
        !(profile->current_amplifier_gain > 0.0F) ||
        ((profile->current_polarity != -1) &&
         (profile->current_polarity != 1)))
    {
        return false;
    }
    for (axis_index = 0U; axis_index < FOC_DENGFOC_V04_AXIS_COUNT;
         ++axis_index)
    {
        const foc_dengfoc_axis_profile_t *axis = &profile->axis[axis_index];
        if ((axis->pwm_a_gpio == axis->pwm_b_gpio) ||
            (axis->pwm_a_gpio == axis->pwm_c_gpio) ||
            (axis->pwm_b_gpio == axis->pwm_c_gpio) ||
            (axis->current_a_adc_gpio == axis->current_b_adc_gpio) ||
            (axis->i2c_sda_gpio == axis->i2c_scl_gpio) ||
            (axis->as5600_i2c_address != FOC_AS5600_I2C_ADDRESS))
        {
            return false;
        }
    }
    return true;
}
