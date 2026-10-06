#include "dengfoc_arduino_power_port.h"

#include <Arduino.h>
#include <driver/ledc.h>
#include <hal/ledc_ll.h>
#include <soc/soc_caps.h>

namespace {

bool forceDriverDisabled(void *opaque) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr)) {
        return false;
    }
    const auto pin = static_cast<uint8_t>(port->profile->driver_enable_gpio);
    const int disabled = port->profile->driver_enable_active_high != 0U
                             ? LOW
                             : HIGH;
    digitalWrite(pin, disabled);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, disabled);
    if (port->pwm_configured == 0U) {
        const auto &axis = port->profile->axis[port->axis];
        pinMode(static_cast<uint8_t>(axis.pwm_a_gpio), INPUT);
        pinMode(static_cast<uint8_t>(axis.pwm_b_gpio), INPUT);
        pinMode(static_cast<uint8_t>(axis.pwm_c_gpio), INPUT);
    }
    return digitalRead(pin) == disabled;
}

bool driverIsDisabled(void *opaque, bool *disabled) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) ||
        (disabled == nullptr)) {
        return false;
    }
    const auto pin = static_cast<uint8_t>(port->profile->driver_enable_gpio);
    const int disabledLevel = port->profile->driver_enable_active_high != 0U
                                  ? LOW
                                  : HIGH;
    *disabled = digitalRead(pin) == disabledLevel;
    return true;
}

bool configureCurrentAdc(void *opaque,
                         uint32_t axis,
                         uint32_t currentAGpio,
                         uint32_t currentBGpio) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) ||
        (axis != port->axis) ||
        (currentAGpio != port->profile->axis[axis].current_a_adc_gpio) ||
        (currentBGpio != port->profile->axis[axis].current_b_adc_gpio) ||
        !dengfoc_arduino_power_port_force_disabled(port)) {
        return false;
    }
    pinMode(static_cast<uint8_t>(currentAGpio), INPUT);
    pinMode(static_cast<uint8_t>(currentBGpio), INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(static_cast<uint8_t>(currentAGpio), ADC_11db);
    analogSetPinAttenuation(static_cast<uint8_t>(currentBGpio), ADC_11db);
    return true;
}

bool configurePwm(void *opaque,
                  uint32_t axis,
                  const uint32_t gpio[FOC_DENGFOC_PHASE_COUNT],
                  uint32_t frequencyHz,
                  uint32_t resolutionBits) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) || (gpio == nullptr) ||
        (axis != port->axis) ||
        (frequencyHz != port->profile->pwm_frequency_hz) ||
        (resolutionBits != port->profile->pwm_resolution_bits) ||
        !dengfoc_arduino_power_port_force_disabled(port)) {
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        const auto channel = static_cast<uint8_t>(port->pwm_channel[phase]);
        if (!ledcAttachChannel(static_cast<uint8_t>(gpio[phase]),
                               frequencyHz,
                               static_cast<uint8_t>(resolutionBits),
                               channel) ||
            !ledcWriteChannel(channel, 0U)) {
            (void)dengfoc_arduino_power_port_force_disabled(port);
            return false;
        }
    }
    port->pwm_configured = 1U;
    return true;
}

bool writePwm(void *opaque,
              uint32_t axis,
              const uint32_t duty[FOC_DENGFOC_PHASE_COUNT]) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (duty == nullptr) || (axis != port->axis) ||
        (port->pwm_configured == 0U)) {
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        if (!ledcWriteChannel(
                static_cast<uint8_t>(port->pwm_channel[phase]), duty[phase])) {
            (void)dengfoc_arduino_power_port_force_disabled(port);
            return false;
        }
    }
    return true;
}

bool readPwm(void *opaque,
             uint32_t axis,
             uint32_t duty[FOC_DENGFOC_PHASE_COUNT]) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) || (duty == nullptr) ||
        (axis != port->axis) || (port->pwm_configured == 0U)) {
        return false;
    }
    const auto &profile = port->profile->axis[axis];
    duty[0] = ledcRead(static_cast<uint8_t>(profile.pwm_a_gpio));
    duty[1] = ledcRead(static_cast<uint8_t>(profile.pwm_b_gpio));
    duty[2] = ledcRead(static_cast<uint8_t>(profile.pwm_c_gpio));
    return true;
}

bool setDriverEnabled(void *opaque, bool enabled) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr)) {
        return false;
    }
    if (!enabled) {
        return dengfoc_arduino_power_port_force_disabled(port);
    }
    if ((port->runtime_actuation_permitted == 0U) ||
        (port->pwm_configured == 0U)) {
        (void)dengfoc_arduino_power_port_force_disabled(port);
        return false;
    }
    const auto pin = static_cast<uint8_t>(port->profile->driver_enable_gpio);
    const int enabledLevel = port->profile->driver_enable_active_high != 0U
                                 ? HIGH
                                 : LOW;
    digitalWrite(pin, enabledLevel);
    return digitalRead(pin) == enabledLevel;
}

bool readCapabilities(void *opaque, uint32_t *capabilities) {
    auto *port = static_cast<dengfoc_arduino_power_port_t *>(opaque);
    if ((port == nullptr) || (capabilities == nullptr)) {
        return false;
    }
    *capabilities = port->pwm_configured != 0U
                        ? FOC_DENGFOC_CAP_PWM_DUTY_READBACK
                        : 0U;
    return true;
}

}  // namespace

bool dengfoc_arduino_power_port_init(
    dengfoc_arduino_power_port_t *port,
    const foc_dengfoc_board_profile_t *profile,
    uint32_t axis,
    bool runtimeActuationPermitted,
    foc_dengfoc_power_ops_t *ops) {
    if ((port == nullptr) || (profile == nullptr) || (ops == nullptr) ||
        !foc_dengfoc_v04_profile_valid(profile) ||
        (axis >= FOC_DENGFOC_V04_AXIS_COUNT)) {
        return false;
    }
    *port = {};
    port->profile = profile;
    port->axis = axis;
    port->pwm_channel[0] = axis * FOC_DENGFOC_PHASE_COUNT;
    port->pwm_channel[1] = port->pwm_channel[0] + 1U;
    port->pwm_channel[2] = port->pwm_channel[0] + 2U;
    port->runtime_actuation_permitted = runtimeActuationPermitted ? 1U : 0U;
    *ops = {};
    ops->context = port;
    ops->force_driver_disabled = forceDriverDisabled;
    ops->driver_is_disabled = driverIsDisabled;
    ops->configure_current_adc = configureCurrentAdc;
    ops->configure_pwm = configurePwm;
    ops->write_pwm = writePwm;
    ops->read_pwm = readPwm;
    ops->set_driver_enabled = setDriverEnabled;
    ops->read_capabilities = readCapabilities;
    return dengfoc_arduino_power_port_force_disabled(port);
}

bool dengfoc_arduino_power_port_force_disabled(
    dengfoc_arduino_power_port_t *port) {
    return forceDriverDisabled(port);
}

bool dengfoc_arduino_power_port_is_disabled(
    dengfoc_arduino_power_port_t *port) {
    bool disabled = false;
    return driverIsDisabled(port, &disabled) && disabled;
}

bool dengfoc_arduino_power_port_snapshot(
    dengfoc_arduino_power_port_t *port,
    dengfoc_arduino_power_snapshot_t *snapshot) {
    if ((port == nullptr) || (port->profile == nullptr) ||
        (snapshot == nullptr)) {
        return false;
    }
    *snapshot = {};
    snapshot->pwm_configured = port->pwm_configured;
    snapshot->driver_disabled =
        dengfoc_arduino_power_port_is_disabled(port) ? 1U : 0U;
    if (port->pwm_configured == 0U) {
        return true;
    }
    const auto &axis = port->profile->axis[port->axis];
    const uint32_t gpio[FOC_DENGFOC_PHASE_COUNT] = {
        axis.pwm_a_gpio,
        axis.pwm_b_gpio,
        axis.pwm_c_gpio,
    };
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        // Arduino-ESP32 ledcReadFreq(pin) returns zero when duty is zero.
        // This diagnostic deliberately holds zero duty, so read the timer
        // selected by the already assigned channel without changing output.
        const auto channel = static_cast<uint8_t>(port->pwm_channel[phase]);
        const auto group = static_cast<uint8_t>(
            channel / SOC_LEDC_CHANNEL_NUM);
        ledc_timer_t timer = LEDC_TIMER_0;
        ledc_ll_get_channel_timer(
            LEDC_LL_GET_HW(),
            static_cast<ledc_mode_t>(group),
            static_cast<ledc_channel_t>(
                channel % SOC_LEDC_CHANNEL_NUM),
            &timer);
        snapshot->frequency_hz[phase] = ledc_get_freq(
            static_cast<ledc_mode_t>(group), timer);
        snapshot->duty_count[phase] =
            ledcRead(static_cast<uint8_t>(gpio[phase]));
    }
    return true;
}
