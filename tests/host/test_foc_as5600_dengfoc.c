#include "foc_as5600.h"
#include "foc_board_dengfoc_v04.h"
#include "foc_dengfoc_as5600_port.h"
#include "foc_dengfoc_current_sense.h"
#include "foc_time_sync_dengfoc.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_TWO_PI (6.28318530717958647692F)

typedef struct
{
    uint32_t begin_calls;
    uint32_t read_calls;
    uint32_t controller;
    uint32_t sda_gpio;
    uint32_t scl_gpio;
    uint32_t frequency_hz;
    uint8_t address;
    uint8_t register_address;
    uint16_t raw_count;
    bool read_ok;
} fake_i2c_t;

typedef struct
{
    uint32_t sample;
    uint32_t calls;
    uint32_t fail_after_calls;
} fake_adc_t;

static bool fake_adc_read(
    void *context,
    uint32_t gpio,
    uint32_t *raw_count)
{
    static const uint32_t a_values[4] = {2047U, 2049U, 2048U, 2048U};
    fake_adc_t *fake = (fake_adc_t *)context;
    if ((raw_count == NULL) ||
        ((fake->fail_after_calls != 0U) &&
         (fake->calls >= fake->fail_after_calls)))
    {
        return false;
    }
    if (gpio == 39U)
    {
        *raw_count = a_values[fake->sample];
    }
    else if (gpio == 36U)
    {
        *raw_count = 2050U;
        fake->sample += 1U;
    }
    else
    {
        return false;
    }
    fake->calls += 1U;
    return true;
}

static bool fake_i2c_begin(
    void *context,
    uint32_t controller,
    uint32_t sda_gpio,
    uint32_t scl_gpio,
    uint32_t frequency_hz)
{
    fake_i2c_t *fake = (fake_i2c_t *)context;
    fake->begin_calls += 1U;
    fake->controller = controller;
    fake->sda_gpio = sda_gpio;
    fake->scl_gpio = scl_gpio;
    fake->frequency_hz = frequency_hz;
    return true;
}

static bool fake_i2c_read(
    void *context,
    uint32_t controller,
    uint8_t address,
    uint8_t register_address,
    uint8_t *data,
    size_t length)
{
    fake_i2c_t *fake = (fake_i2c_t *)context;
    fake->read_calls += 1U;
    fake->controller = controller;
    fake->address = address;
    fake->register_address = register_address;
    if (!fake->read_ok || (data == NULL) || (length != 2U))
    {
        return false;
    }
    data[0] = (uint8_t)((fake->raw_count >> 8U) & 0x0FU);
    data[1] = (uint8_t)(fake->raw_count & 0x00FFU);
    return true;
}

static void board_profile_matches_live_reference(void)
{
    const foc_dengfoc_board_profile_t *profile =
        &foc_dengfoc_v04_reference_profile;
    assert(foc_dengfoc_v04_profile_valid(profile));
    assert(profile->pwm_frequency_hz == 30000U);
    assert(profile->pwm_resolution_bits == 8U);
    assert(profile->reference_control_frequency_hz == 2000U);
    assert(profile->driver_enable_gpio == 12U);
    assert(profile->default_enabled_axis_mask == 0x01U);
    assert(profile->axis[0].pwm_a_gpio == 32U);
    assert(profile->axis[0].pwm_b_gpio == 33U);
    assert(profile->axis[0].pwm_c_gpio == 25U);
    assert(profile->axis[0].current_a_adc_gpio == 39U);
    assert(profile->axis[0].current_b_adc_gpio == 36U);
    assert(profile->axis[0].i2c_sda_gpio == 19U);
    assert(profile->axis[0].i2c_scl_gpio == 18U);
    assert(profile->axis[1].pwm_a_gpio == 26U);
    assert(profile->axis[1].pwm_b_gpio == 27U);
    assert(profile->axis[1].pwm_c_gpio == 14U);
    assert(profile->axis[1].current_a_adc_gpio == 35U);
    assert(profile->axis[1].current_b_adc_gpio == 34U);
    assert(profile->axis[1].i2c_sda_gpio == 23U);
    assert(profile->axis[1].i2c_scl_gpio == 5U);
    assert(profile->axis[0].as5600_i2c_address == 0x36U);
    assert(profile->encoder_counts_per_revolution == 4096U);
    assert(profile->encoder_i2c_frequency_hz == 400000U);
    assert(fabsf(profile->shunt_resistance_ohm - 0.01F) < 1.0e-6F);
    assert(fabsf(profile->current_amplifier_gain - 50.0F) < 1.0e-6F);
    assert(profile->current_polarity == 1);
}

static void time_sync_resource_requires_a_disabled_axis_connector(void)
{
    const foc_dengfoc_board_profile_t *profile =
        &foc_dengfoc_v04_reference_profile;

    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x01U, 23U) == 1U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x03U, 23U) == 0U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x01U, 19U) == 0U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x01U, 12U) == 0U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x01U, 26U) == 0U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile, 0x00U, 23U) == 0U);
    assert(foc_time_sync_dengfoc_input_resource_valid(
               profile,
               0x01U,
               FOC_TIME_SYNC_DENGFOC_GPIO_UNCONFIGURED) == 0U);
}

static foc_as5600_tracker_t tracker(void)
{
    foc_as5600_tracker_t value;
    const foc_as5600_config_t config = {
        .axis_id = 0U,
        .direction = 1,
        .pole_pairs = 7U,
        .pole_pair_revision = 1U,
        .electrical_offset_rad = 0.0F,
        .calibrated = 1U,
        .maximum_sample_period_us = 5000U,
    };
    assert(foc_as5600_tracker_init(&value, &config));
    return value;
}

static void raw_decode_and_wrap_are_correct(void)
{
    uint16_t raw = 0U;
    foc_as5600_tracker_t value = tracker();
    foc_absolute_encoder_feedback_port_t sample;
    foc_feedback_source_sample_t normalized;
    assert(foc_as5600_decode_raw_angle(0x0FU, 0xFFU, &raw));
    assert(raw == 4095U);
    assert(foc_as5600_tracker_update(&value, 4090U, 1U, 1000U, &sample));
    assert(sample.common.available == 0U);
    assert(foc_as5600_tracker_update(&value, 5U, 2U, 2000U, &sample));
    assert(sample.common.available == 1U);
    assert(sample.mechanical_velocity_rad_s > 0.0F);
    assert(sample.multi_turn_position_rad > TEST_TWO_PI);
    assert(sample.electrical_angle_rad >= 0.0F);
    assert(sample.electrical_angle_rad < TEST_TWO_PI);
    assert(foc_feedback_adapt_absolute_encoder(&sample, &normalized));
    assert(normalized.mode == FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER);
    assert(normalized.valid_flags == FOC_PRODUCT_FEEDBACK_VALID_KNOWN_MASK);
    assert((normalized.quality_flags &
            FOC_PRODUCT_FEEDBACK_QUALITY_INDEX_FOUND) == 0U);
}

static void direction_and_timestamp_fail_closed(void)
{
    foc_as5600_tracker_t value;
    foc_as5600_config_t config;
    foc_absolute_encoder_feedback_port_t sample;
    (void)memset(&config, 0, sizeof(config));
    config.direction = 0;
    config.pole_pairs = 7U;
    config.pole_pair_revision = 1U;
    config.maximum_sample_period_us = 5000U;
    assert(!foc_as5600_tracker_init(&value, &config));

    config.direction = -1;
    assert(foc_as5600_tracker_init(&value, &config));
    assert(foc_as5600_tracker_update(&value, 100U, 1U, 1000U, &sample));
    assert(foc_as5600_tracker_update(&value, 120U, 20U, 20000U, &sample));
    assert(sample.common.available == 0U);
    assert(!foc_as5600_tracker_update(&value, 4096U, 21U, 21000U, &sample));
}

static void board_port_uses_axis_specific_i2c_and_fails_zero(void)
{
    fake_i2c_t fake;
    const foc_dengfoc_i2c_ops_t ops = {
        .begin = fake_i2c_begin,
        .read_register = fake_i2c_read,
    };
    foc_dengfoc_as5600_port_t port;
    foc_feedback_source_sample_t sample;
    (void)memset(&fake, 0, sizeof(fake));
    fake.read_ok = true;
    assert(foc_dengfoc_as5600_port_init(
        &port,
        &foc_dengfoc_v04_reference_profile,
        1U,
        &ops,
        &fake,
        1,
        7U,
        1U,
        0.0F,
        1U,
        5000U));
    assert(fake.begin_calls == 1U);
    assert(fake.controller == 1U);
    assert(fake.sda_gpio == 23U);
    assert(fake.scl_gpio == 5U);
    assert(fake.frequency_hz == 400000U);

    fake.raw_count = 100U;
    assert(foc_dengfoc_as5600_port_poll(&port, 1U, 1000U, &sample));
    assert(sample.mode == FOC_FEEDBACK_MODE_ABSOLUTE_ENCODER);
    assert(sample.valid_flags == 0U);
    fake.raw_count = 110U;
    assert(foc_dengfoc_as5600_port_poll(&port, 2U, 2000U, &sample));
    assert(fake.address == 0x36U);
    assert(fake.register_address == 0x0CU);
    assert(sample.axis_id == 1U);
    assert(sample.valid_flags == FOC_PRODUCT_FEEDBACK_VALID_KNOWN_MASK);

    fake.read_ok = false;
    (void)memset(&sample, 0xA5, sizeof(sample));
    assert(!foc_dengfoc_as5600_port_poll(&port, 3U, 3000U, &sample));
    assert(sample.struct_size == 0U);
    assert(sample.valid_flags == 0U);
}

static void current_zero_capture_is_bounded_and_fails_zero(void)
{
    fake_adc_t fake;
    const foc_dengfoc_adc_ops_t ops = {
        .context = &fake,
        .read_raw = fake_adc_read,
    };
    foc_dengfoc_current_zero_stat_t
        stat[FOC_DENGFOC_CURRENT_CHANNEL_COUNT];
    (void)memset(&fake, 0, sizeof(fake));
    assert(foc_dengfoc_current_capture_zero(
        &foc_dengfoc_v04_reference_profile, 0U, &ops, 4U, stat));
    assert(stat[0].sample_count == 4U);
    assert(stat[0].minimum_raw_count == 2047U);
    assert(stat[0].maximum_raw_count == 2049U);
    assert(fabsf(stat[0].mean_raw_count - 2048.0F) < 1.0e-4F);
    assert(fabsf(stat[0].standard_deviation_raw_count - 0.70710677F) <
           1.0e-4F);
    assert(stat[1].minimum_raw_count == 2050U);
    assert(stat[1].maximum_raw_count == 2050U);
    assert(stat[1].standard_deviation_raw_count == 0.0F);
    assert(stat[0].amperes_per_count > 0.0F);

    (void)memset(&fake, 0, sizeof(fake));
    fake.fail_after_calls = 3U;
    (void)memset(stat, 0xA5, sizeof(stat));
    assert(!foc_dengfoc_current_capture_zero(
        &foc_dengfoc_v04_reference_profile, 0U, &ops, 4U, stat));
    assert(stat[0].sample_count == 0U);
    assert(stat[1].sample_count == 0U);
    assert(!foc_dengfoc_current_capture_zero(
        &foc_dengfoc_v04_reference_profile, 0U, &ops, 0U, stat));
}

int main(void)
{
    board_profile_matches_live_reference();
    time_sync_resource_requires_a_disabled_axis_connector();
    raw_decode_and_wrap_are_correct();
    direction_and_timestamp_fail_closed();
    board_port_uses_axis_specific_i2c_and_fails_zero();
    current_zero_capture_is_bounded_and_fails_zero();
    return 0;
}
