#include "dengfoc_mcpwm_power_port.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_cpu.h>
#include <hal/mcpwm_ll.h>

namespace {

constexpr uint32_t kTimerResolutionHz = 40000000U;

bool stageCoherentUpdate(
    dengfoc_mcpwm_power_port_t *port,
    const uint32_t duty[FOC_DENGFOC_PHASE_COUNT]);

bool forceDriverDisabled(void *opaque) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
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
    port->outputs_released = 0U;
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        if ((port->generators[phase] != nullptr) &&
            (mcpwm_generator_set_force_level(
                 port->generators[phase], 0, true) != ESP_OK)) {
            return false;
        }
        port->duty_count[phase] = 0U;
    }
    if (port->pwm_configured != 0U) {
        const uint32_t zero[FOC_DENGFOC_PHASE_COUNT] = {0U, 0U, 0U};
        if (!stageCoherentUpdate(port, zero)) {
            return false;
        }
    }
    return digitalRead(pin) == disabled;
}

bool driverIsDisabled(void *opaque, bool *disabled) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
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
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) ||
        (axis != port->axis) ||
        (currentAGpio != port->profile->axis[axis].current_a_adc_gpio) ||
        (currentBGpio != port->profile->axis[axis].current_b_adc_gpio) ||
        !dengfoc_mcpwm_power_port_force_disabled(port)) {
        return false;
    }
    pinMode(static_cast<uint8_t>(currentAGpio), INPUT);
    pinMode(static_cast<uint8_t>(currentBGpio), INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(static_cast<uint8_t>(currentAGpio), ADC_11db);
    analogSetPinAttenuation(static_cast<uint8_t>(currentBGpio), ADC_11db);
    return true;
}

bool IRAM_ATTR onTimerFull(mcpwm_timer_handle_t,
                           const mcpwm_timer_event_data_t *,
                           void *opaque) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if (port != nullptr) {
        port->full_event_count += 1U;
        port->last_full_cpu_cycle = esp_cpu_get_cycle_count();
        if (port->cycle_event_callback != nullptr) {
            port->cycle_event_callback(
                port->cycle_event_context,
                port->full_event_count,
                port->last_full_cpu_cycle);
        }
        if ((port->divided_cycle_event_callback != nullptr) &&
            (port->divided_cycle_event_divider != 0U)) {
            port->divided_cycle_event_phase += 1U;
            if (port->divided_cycle_event_phase >=
                port->divided_cycle_event_divider) {
                port->divided_cycle_event_phase = 0U;
                port->divided_cycle_event_sequence += 1U;
                port->divided_cycle_event_callback(
                    port->divided_cycle_event_context,
                    port->divided_cycle_event_sequence,
                    port->last_full_cpu_cycle);
            }
        }
    }
    return false;
}

bool stageCoherentUpdate(
    dengfoc_mcpwm_power_port_t *port,
    const uint32_t duty[FOC_DENGFOC_PHASE_COUNT]) {
    if ((port == nullptr) || (duty == nullptr) ||
        (port->peak_ticks == 0U)) {
        return false;
    }
    port->last_update_step = 1U;
    port->last_update_error = ESP_ERR_INVALID_STATE;
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        if ((port->comparators[phase] == nullptr) ||
            (duty[phase] > port->maximum_duty_count)) {
            return false;
        }
        const uint32_t compareTicks = static_cast<uint32_t>(
            (static_cast<uint64_t>(duty[phase]) * port->peak_ticks) /
            port->maximum_duty_count);
        const esp_err_t compareError = mcpwm_comparator_set_compare_value(
            port->comparators[phase], compareTicks);
        if (compareError != ESP_OK) {
            port->last_update_step = phase + 1U;
            port->last_update_error = compareError;
            return false;
        }
    }
    /* All three comparator values above are held in shadow registers because
     * update_cmp_on_sync is enabled. The classic ESP32 MCPWM block provides a
     * group-wide shadow flush, which transfers all operators to active state
     * with one hardware update event and does not reset the timer phase. */
    mcpwm_ll_group_flush_shadow(MCPWM_LL_GET_HW(0));
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        port->duty_count[phase] = duty[phase];
    }
    port->coherent_update_count += 1U;
    port->last_update_step = 0U;
    port->last_update_error = ESP_OK;
    return true;
}

bool configurePwm(void *opaque,
                  uint32_t axis,
                  const uint32_t gpio[FOC_DENGFOC_PHASE_COUNT],
                  uint32_t frequencyHz,
                  uint32_t resolutionBits) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if ((port == nullptr) || (port->profile == nullptr) || (gpio == nullptr) ||
        (axis != port->axis) || (port->pwm_configured != 0U) ||
        (frequencyHz != port->profile->pwm_frequency_hz) ||
        (resolutionBits != port->profile->pwm_resolution_bits) ||
        (resolutionBits == 0U) || (resolutionBits >= 31U) ||
        (frequencyHz == 0U) ||
        !dengfoc_mcpwm_power_port_force_disabled(port)) {
        return false;
    }

    const uint32_t peakTicks =
        (kTimerResolutionHz + frequencyHz) / (2U * frequencyHz);
    if (peakTicks < 2U) {
        return false;
    }
    /* ESP-IDF defines period_ticks as the complete 0 -> peak -> 0 interval
     * in up-down mode. The hardware peak and comparator range are therefore
     * half of period_ticks. Keep the full period even so the peak is exact. */
    const uint32_t periodTicks = 2U * peakTicks;
    const mcpwm_timer_config_t timerConfig = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = kTimerResolutionHz,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = periodTicks,
        .intr_priority = 0,
        .flags = {},
    };
    if (mcpwm_new_timer(&timerConfig, &port->timer) != ESP_OK) {
        return false;
    }
    mcpwm_ll_group_enable_shadow_mode(MCPWM_LL_GET_HW(0));
    const mcpwm_timer_event_callbacks_t timerCallbacks = {
        .on_full = onTimerFull,
        .on_empty = nullptr,
        .on_stop = nullptr,
    };
    if (mcpwm_timer_register_event_callbacks(
            port->timer, &timerCallbacks, port) != ESP_OK) {
        return false;
    }

    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        const mcpwm_operator_config_t operatorConfig = {
            .group_id = 0,
            .intr_priority = 0,
            .flags = {
                .update_gen_action_on_tez = 1,
                .update_gen_action_on_tep = 0,
                .update_gen_action_on_sync = 0,
                .update_dead_time_on_tez = 0,
                .update_dead_time_on_tep = 0,
                .update_dead_time_on_sync = 0,
            },
        };
        if ((mcpwm_new_operator(
                 &operatorConfig, &port->operators[phase]) != ESP_OK) ||
            (mcpwm_operator_connect_timer(
                 port->operators[phase], port->timer) != ESP_OK)) {
            return false;
        }
        const mcpwm_comparator_config_t comparatorConfig = {
            .intr_priority = 0,
            .flags = {
                .update_cmp_on_tez = 0,
                .update_cmp_on_tep = 0,
                .update_cmp_on_sync = 1,
            },
        };
        if ((mcpwm_new_comparator(port->operators[phase],
                                  &comparatorConfig,
                                  &port->comparators[phase]) != ESP_OK) ||
            (mcpwm_comparator_set_compare_value(
                 port->comparators[phase], peakTicks / 2U) != ESP_OK)) {
            return false;
        }
        const mcpwm_generator_config_t generatorConfig = {
            .gen_gpio_num = static_cast<int>(gpio[phase]),
            .flags = {},
        };
        if ((mcpwm_new_generator(port->operators[phase],
                                 &generatorConfig,
                                 &port->generators[phase]) != ESP_OK) ||
            (mcpwm_generator_set_action_on_timer_event(
                 port->generators[phase],
                 MCPWM_GEN_TIMER_EVENT_ACTION(
                     MCPWM_TIMER_DIRECTION_UP,
                     MCPWM_TIMER_EVENT_EMPTY,
                     MCPWM_GEN_ACTION_HIGH)) != ESP_OK) ||
            (mcpwm_generator_set_action_on_compare_event(
                 port->generators[phase],
                 MCPWM_GEN_COMPARE_EVENT_ACTION(
                     MCPWM_TIMER_DIRECTION_UP,
                     port->comparators[phase],
                     MCPWM_GEN_ACTION_LOW)) != ESP_OK) ||
            (mcpwm_generator_set_action_on_compare_event(
                 port->generators[phase],
                 MCPWM_GEN_COMPARE_EVENT_ACTION(
                     MCPWM_TIMER_DIRECTION_DOWN,
                     port->comparators[phase],
                     MCPWM_GEN_ACTION_HIGH)) != ESP_OK) ||
            (mcpwm_generator_set_force_level(
                 port->generators[phase], 0, true) != ESP_OK)) {
            return false;
        }
    }

    if ((mcpwm_timer_enable(port->timer) != ESP_OK) ||
        (mcpwm_timer_start_stop(
             port->timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK)) {
        return false;
    }
    port->timer_resolution_hz = kTimerResolutionHz;
    port->period_ticks = periodTicks;
    port->peak_ticks = peakTicks;
    port->maximum_duty_count = (1UL << resolutionBits) - 1UL;
    port->actual_frequency_hz =
        kTimerResolutionHz / periodTicks;
    port->pwm_configured = 1U;
    return true;
}

bool writePwm(void *opaque,
              uint32_t axis,
              const uint32_t duty[FOC_DENGFOC_PHASE_COUNT]) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if ((port == nullptr) || (duty == nullptr) || (axis != port->axis) ||
        (port->pwm_configured == 0U)) {
        return false;
    }
    if (port->runtime_actuation_permitted == 0U) {
        for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
            if (duty[phase] != 0U) {
                (void)dengfoc_mcpwm_power_port_force_disabled(port);
                return false;
            }
        }
        return dengfoc_mcpwm_power_port_force_disabled(port) &&
               stageCoherentUpdate(port, duty);
    }
    if (port->outputs_released == 0U) {
        for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
            if (duty[phase] != 0U) {
                (void)dengfoc_mcpwm_power_port_force_disabled(port);
                return false;
            }
        }
        return stageCoherentUpdate(port, duty);
    }
    return stageCoherentUpdate(port, duty);
}

bool readPwm(void *opaque,
             uint32_t axis,
             uint32_t duty[FOC_DENGFOC_PHASE_COUNT]) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if ((port == nullptr) || (duty == nullptr) || (axis != port->axis) ||
        (port->pwm_configured == 0U)) {
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        duty[phase] = port->duty_count[phase];
    }
    return true;
}

bool setDriverEnabled(void *opaque, bool enabled) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if (port == nullptr) {
        return false;
    }
    if (!enabled) {
        return dengfoc_mcpwm_power_port_force_disabled(port);
    }
    if ((port->runtime_actuation_permitted == 0U) ||
        (port->pwm_configured == 0U) ||
        !dengfoc_mcpwm_power_port_is_disabled(port)) {
        (void)dengfoc_mcpwm_power_port_force_disabled(port);
        return false;
    }
    const uint32_t zero[FOC_DENGFOC_PHASE_COUNT] = {0U, 0U, 0U};
    if (!stageCoherentUpdate(port, zero)) {
        (void)dengfoc_mcpwm_power_port_force_disabled(port);
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        if (mcpwm_generator_set_force_level(
                port->generators[phase], -1, true) != ESP_OK) {
            (void)dengfoc_mcpwm_power_port_force_disabled(port);
            return false;
        }
    }
    const auto pin = static_cast<uint8_t>(port->profile->driver_enable_gpio);
    const int enabledLevel = port->profile->driver_enable_active_high != 0U
                                 ? HIGH
                                 : LOW;
    digitalWrite(pin, enabledLevel);
    port->outputs_released = 1U;
    if (digitalRead(pin) != enabledLevel) {
        (void)dengfoc_mcpwm_power_port_force_disabled(port);
        return false;
    }
    return true;
}

bool readCapabilities(void *opaque, uint32_t *capabilities) {
    auto *port = static_cast<dengfoc_mcpwm_power_port_t *>(opaque);
    if ((port == nullptr) || (capabilities == nullptr)) {
        return false;
    }
    *capabilities = port->pwm_configured != 0U
                        ? FOC_DENGFOC_CAP_PWM_DUTY_READBACK |
                              FOC_DENGFOC_CAP_CENTER_ALIGNED_PWM |
                              (port->coherent_update_count != 0U
                                   ? FOC_DENGFOC_CAP_COHERENT_THREE_PHASE_UPDATE
                                   : 0U) |
                              FOC_DENGFOC_CAP_PWM_CYCLE_ISR
                        : 0U;
    return true;
}

}  // namespace

bool dengfoc_mcpwm_power_port_init(
    dengfoc_mcpwm_power_port_t *port,
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
    port->runtime_actuation_permitted =
        runtimeActuationPermitted ? 1U : 0U;
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
    return dengfoc_mcpwm_power_port_force_disabled(port);
}

bool dengfoc_mcpwm_power_port_force_disabled(
    dengfoc_mcpwm_power_port_t *port) {
    return forceDriverDisabled(port);
}

bool dengfoc_mcpwm_power_port_is_disabled(
    dengfoc_mcpwm_power_port_t *port) {
    bool disabled = false;
    return driverIsDisabled(port, &disabled) && disabled;
}

bool dengfoc_mcpwm_power_port_snapshot(
    dengfoc_mcpwm_power_port_t *port,
    dengfoc_mcpwm_power_snapshot_t *snapshot) {
    if ((port == nullptr) || (snapshot == nullptr)) {
        return false;
    }
    *snapshot = {};
    snapshot->pwm_configured = port->pwm_configured;
    snapshot->driver_disabled =
        dengfoc_mcpwm_power_port_is_disabled(port) ? 1U : 0U;
    snapshot->period_ticks = port->period_ticks;
    snapshot->peak_ticks = port->peak_ticks;
    snapshot->full_event_count = port->full_event_count;
    snapshot->coherent_update_count = port->coherent_update_count;
    snapshot->last_update_step = port->last_update_step;
    snapshot->last_update_error = port->last_update_error;
    if (!readCapabilities(port, &snapshot->capabilities)) {
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        snapshot->frequency_hz[phase] = port->actual_frequency_hz;
        snapshot->duty_count[phase] = port->duty_count[phase];
    }
    return true;
}

void dengfoc_mcpwm_power_port_set_cycle_event_callback(
    dengfoc_mcpwm_power_port_t *port,
    void (*callback)(void *context,
                     uint32_t event_count,
                     uint32_t cpu_cycle),
    void *context) {
    if (port == nullptr) {
        return;
    }
    port->cycle_event_context = context;
    port->cycle_event_callback = callback;
}

void dengfoc_mcpwm_power_port_set_divided_cycle_event_callback(
    dengfoc_mcpwm_power_port_t *port,
    uint32_t divider,
    void (*callback)(void *context,
                     uint32_t event_count,
                     uint32_t cpu_cycle),
    void *context) {
    if (port == nullptr) {
        return;
    }
    port->divided_cycle_event_context = context;
    port->divided_cycle_event_divider = divider;
    port->divided_cycle_event_phase = 0U;
    port->divided_cycle_event_sequence = 0U;
    port->divided_cycle_event_callback = callback;
}
