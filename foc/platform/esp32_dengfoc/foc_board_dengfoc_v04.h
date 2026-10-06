#ifndef FOC_BOARD_DENGFOC_V04_H
#define FOC_BOARD_DENGFOC_V04_H

/*
 * DengFOC V0.4 + ESP32 board contract copied from the live reference project
 * at E:/File/PlatformIO/DengFOC. This file contains configuration only; ESP32
 * GPIO/I2C/LEDC/ADC access belongs in a separate platform implementation.
 */

#include <stdbool.h>
#include <stdint.h>

#define FOC_DENGFOC_V04_PROFILE_VERSION (1U)
#define FOC_DENGFOC_V04_AXIS_COUNT      (2U)
#define FOC_DENGFOC_V04_DEFAULT_AXIS_MASK (0x01U)

typedef struct
{
    uint32_t pwm_a_gpio;
    uint32_t pwm_b_gpio;
    uint32_t pwm_c_gpio;
    uint32_t current_a_adc_gpio;
    uint32_t current_b_adc_gpio;
    uint32_t i2c_controller;
    uint32_t i2c_sda_gpio;
    uint32_t i2c_scl_gpio;
    uint32_t as5600_i2c_address;
} foc_dengfoc_axis_profile_t;

typedef struct
{
    uint32_t version;
    uint32_t esp32_cpu_frequency_hz;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_resolution_bits;
    uint32_t reference_control_frequency_hz;
    uint32_t driver_enable_gpio;
    uint32_t driver_enable_active_high;
    uint32_t default_enabled_axis_mask;
    uint32_t encoder_counts_per_revolution;
    uint32_t encoder_i2c_frequency_hz;
    float nominal_bus_voltage_v;
    float adc_reference_v;
    uint32_t adc_full_scale_count;
    float shunt_resistance_ohm;
    float current_amplifier_gain;
    int32_t current_polarity;
    foc_dengfoc_axis_profile_t axis[FOC_DENGFOC_V04_AXIS_COUNT];
} foc_dengfoc_board_profile_t;

extern const foc_dengfoc_board_profile_t foc_dengfoc_v04_reference_profile;

bool foc_dengfoc_v04_profile_valid(
    const foc_dengfoc_board_profile_t *profile);

#endif /* FOC_BOARD_DENGFOC_V04_H */
