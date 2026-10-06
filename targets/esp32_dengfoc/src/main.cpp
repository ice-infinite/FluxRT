#include <Arduino.h>
#include <Wire.h>
#include <esp_cpu.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "dengfoc_target_power_port.h"
#if FLUXRT_DENGFOC_RUST_PROBE
#include "dengfoc_adc_stream.h"
#endif

extern "C" {
#include "foc_board_dengfoc_v04.h"
#include "foc_dengfoc_as5600_port.h"
#include "foc_dengfoc_current_sense.h"
#if FLUXRT_DENGFOC_RUST_PROBE
#include "foc_encoder_realtime_bridge.h"
#include "foc_rust_bridge.h"
#endif
}

#if !defined(FLUXRT_DENGFOC_DIAGNOSTIC_ONLY) || \
    (FLUXRT_DENGFOC_DIAGNOSTIC_ONLY != 1)
#error "This target is intentionally restricted to non-actuating diagnostics"
#endif

#ifndef FLUXRT_DENGFOC_ENABLE_AXIS1
#define FLUXRT_DENGFOC_ENABLE_AXIS1 0
#endif

#ifndef FLUXRT_DENGFOC_PWM_PREPARE_ONLY
#define FLUXRT_DENGFOC_PWM_PREPARE_ONLY 0
#endif

#ifndef FLUXRT_DENGFOC_RUST_PROBE
#define FLUXRT_DENGFOC_RUST_PROBE 0
#endif

#if (FLUXRT_DENGFOC_PWM_PREPARE_ONLY != 0) && \
    (FLUXRT_DENGFOC_PWM_PREPARE_ONLY != 1)
#error "FLUXRT_DENGFOC_PWM_PREPARE_ONLY must be 0 or 1"
#endif

namespace {

constexpr uint32_t kSerialBaud = 921600U;
constexpr uint32_t kPollPeriodUs = 5000U;
constexpr uint32_t kTelemetryPeriodMs = 50U;
constexpr uint32_t kMaximumSamplePeriodUs = 20000U;
constexpr uint32_t kPolePairs = 7U;
constexpr uint32_t kCurrentZeroSamples = 1000U;

struct ArduinoI2cContext {
    TwoWire *wire[FOC_DENGFOC_V04_AXIS_COUNT];
};

TwoWire gWire0(0);
TwoWire gWire1(1);
ArduinoI2cContext gI2cContext{{&gWire0, &gWire1}};
foc_dengfoc_as5600_port_t gEncoder[FOC_DENGFOC_V04_AXIS_COUNT];
foc_feedback_source_sample_t gLastSample[FOC_DENGFOC_V04_AXIS_COUNT];
portMUX_TYPE gFeedbackMux = portMUX_INITIALIZER_UNLOCKED;
dengfoc_target_power_port_t gPowerPort{};
foc_dengfoc_power_stage_t gPowerStage{};
uint32_t gReadFailures[FOC_DENGFOC_V04_AXIS_COUNT] = {0U, 0U};
uint32_t gLastPollUs = 0U;
uint32_t gLastTelemetryMs = 0U;
#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
uint32_t gLastPowerCheckMs = 0U;
#if FLUXRT_DENGFOC_MCPWM_TIMING_ONLY
uint32_t gLastMcpwmFullCount = 0U;
#endif
#endif
volatile bool gFatal = false;
#if FLUXRT_DENGFOC_RUST_PROBE
foc_rust_context_t gRustController{};
dengfoc_adc_stream_t gAdcStream{};
foc_dengfoc_current_zero_stat_t
    gCurrentZeroStat[FOC_DENGFOC_V04_AXIS_COUNT]
                    [FOC_DENGFOC_CURRENT_CHANNEL_COUNT]{};
uint32_t gLastAdcSampleSequence = 0U;
uint32_t gLastAdcPoolOverflowCount = 0U;
uint32_t gLastAdcReadErrorCount = 0U;
bool gAdcHealthBaselineReady = false;
constexpr uint32_t kAdcProbeSampleFrequencyHz = 60000U;
constexpr uint32_t kShadowControlFrequencyHz = 2000U;
constexpr uint32_t kShadowPeriodUs =
    1000000U / kShadowControlFrequencyHz;
constexpr uint32_t kShadowPwmDivider = 15U;
TaskHandle_t gRustShadowTask = nullptr;
volatile uint32_t gRustShadowTriggerCpuCycle = 0U;

struct RustShadowStats {
    volatile uint32_t callback_count;
    volatile uint32_t step_count;
    volatile uint32_t stale_adc_count;
    volatile uint32_t invalid_feedback_count;
    volatile uint32_t invalid_angle_count;
    volatile uint32_t invalid_zero_count;
    volatile uint32_t step_error_count;
    volatile uint32_t deadline_miss_count;
    volatile uint32_t minimum_period_us;
    volatile uint32_t maximum_period_us;
    volatile uint32_t maximum_compute_cycles;
    volatile uint32_t maximum_wake_cycles;
    volatile uint32_t last_status;
    volatile float last_duty_a;
    volatile float last_duty_b;
    volatile float last_duty_c;
    volatile float maximum_duty_deviation;
};

RustShadowStats gRustShadowStats{};
uint32_t gRustShadowControlSequence = 0U;
uint32_t gRustShadowLastAdcSequence = 0U;
uint32_t gRustShadowLastCallbackUs = 0U;
#endif

TwoWire *wireFor(ArduinoI2cContext *context, uint32_t controller) {
    if ((context == nullptr) || (controller >= FOC_DENGFOC_V04_AXIS_COUNT)) {
        return nullptr;
    }
    return context->wire[controller];
}

bool beginI2c(void *opaque,
              uint32_t controller,
              uint32_t sdaGpio,
              uint32_t sclGpio,
              uint32_t frequencyHz) {
    auto *context = static_cast<ArduinoI2cContext *>(opaque);
    TwoWire *wire = wireFor(context, controller);
    if (wire == nullptr) {
        return false;
    }
    return wire->begin(static_cast<int>(sdaGpio),
                       static_cast<int>(sclGpio),
                       frequencyHz);
}

bool readI2cRegister(void *opaque,
                     uint32_t controller,
                     uint8_t address,
                     uint8_t registerAddress,
                     uint8_t *data,
                     size_t length) {
    auto *context = static_cast<ArduinoI2cContext *>(opaque);
    TwoWire *wire = wireFor(context, controller);
    if ((wire == nullptr) || (data == nullptr) || (length == 0U) ||
        (length > 255U)) {
        return false;
    }

    wire->beginTransmission(address);
    if (wire->write(registerAddress) != 1U || wire->endTransmission(false) != 0U) {
        return false;
    }
    const size_t received = wire->requestFrom(
        static_cast<uint16_t>(address),
        static_cast<uint8_t>(length),
        true);
    if (received != length) {
        while (wire->available() > 0) {
            (void)wire->read();
        }
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (wire->available() <= 0) {
            return false;
        }
        data[index] = static_cast<uint8_t>(wire->read());
    }
    return true;
}

const foc_dengfoc_i2c_ops_t kI2cOps = {
    .begin = beginI2c,
    .read_register = readI2cRegister,
};

bool readAdcRaw(void *, uint32_t gpio, uint32_t *rawCount) {
    if (rawCount == nullptr) {
        return false;
    }
    const auto raw = analogRead(static_cast<uint8_t>(gpio));
    *rawCount = static_cast<uint32_t>(raw);
    delayMicroseconds(500U);
    return true;
}

const foc_dengfoc_adc_ops_t kAdcOps = {
    .context = nullptr,
    .read_raw = readAdcRaw,
};

void forcePowerStageDisabled() {
    (void)dengfoc_target_power_port_force_disabled(&gPowerPort);
}

bool powerStageIsDisabled() {
    return dengfoc_target_power_port_is_disabled(&gPowerPort);
}

bool initializeAxis(uint32_t axis) {
    return foc_dengfoc_as5600_port_init(
        &gEncoder[axis],
        &foc_dengfoc_v04_reference_profile,
        axis,
        &kI2cOps,
        &gI2cContext,
        1,
        kPolePairs,
        1U,
        0.0F,
        0U,
        kMaximumSamplePeriodUs);
}

void pollAxis(uint32_t axis, uint32_t nowMs, uint32_t nowUs) {
    foc_feedback_source_sample_t sample{};
    if (foc_dengfoc_as5600_port_poll(
            &gEncoder[axis], nowMs, nowUs, &sample)) {
        portENTER_CRITICAL(&gFeedbackMux);
        gLastSample[axis] = sample;
        portEXIT_CRITICAL(&gFeedbackMux);
    } else {
        ++gReadFailures[axis];
        portENTER_CRITICAL(&gFeedbackMux);
        gLastSample[axis] = {};
        portEXIT_CRITICAL(&gFeedbackMux);
    }
}

void printAxis(uint32_t axis) {
    const auto &sample = gLastSample[axis];
    Serial.printf(
        "axis=%lu,valid=%u,seq=%lu,mech=%.6f,multi=%.6f,vel=%.6f,elec=%.6f,fail=%lu\n",
        static_cast<unsigned long>(axis),
        sample.valid_flags != 0U ? 1U : 0U,
        static_cast<unsigned long>(sample.sequence),
        static_cast<double>(sample.mechanical_position_rad),
        static_cast<double>(sample.multi_turn_position_rad),
        static_cast<double>(sample.mechanical_velocity_rad_s),
        static_cast<double>(sample.electrical_angle_rad),
        static_cast<unsigned long>(gReadFailures[axis]));
}

bool captureAndPrintCurrentZero(uint32_t axis) {
    const auto &board = foc_dengfoc_v04_reference_profile;
    const auto &axisProfile = board.axis[axis];
    foc_dengfoc_current_zero_stat_t stat[FOC_DENGFOC_CURRENT_CHANNEL_COUNT]{};
    pinMode(static_cast<uint8_t>(axisProfile.current_a_adc_gpio), INPUT);
    pinMode(static_cast<uint8_t>(axisProfile.current_b_adc_gpio), INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(
        static_cast<uint8_t>(axisProfile.current_a_adc_gpio), ADC_11db);
    analogSetPinAttenuation(
        static_cast<uint8_t>(axisProfile.current_b_adc_gpio), ADC_11db);
    forcePowerStageDisabled();
    const bool valid = foc_dengfoc_current_capture_zero(
        &board, axis, &kAdcOps, kCurrentZeroSamples, stat);
    forcePowerStageDisabled();
    if (!valid || !powerStageIsDisabled()) {
        Serial.printf("adc_zero_axis=%lu,valid=0\n",
                      static_cast<unsigned long>(axis));
        return false;
    }
#if FLUXRT_DENGFOC_RUST_PROBE
    for (uint32_t channel = 0U;
         channel < FOC_DENGFOC_CURRENT_CHANNEL_COUNT;
         ++channel) {
        gCurrentZeroStat[axis][channel] = stat[channel];
    }
#endif
    Serial.printf(
        "adc_zero_axis=%lu,valid=1,samples=%lu,"
        "a_mean=%.3f,a_min=%lu,a_max=%lu,a_std=%.3f,a_v=%.6f,a_noise=%.6f,"
        "b_mean=%.3f,b_min=%lu,b_max=%lu,b_std=%.3f,b_v=%.6f,b_noise=%.6f\n",
        static_cast<unsigned long>(axis),
        static_cast<unsigned long>(stat[0].sample_count),
        static_cast<double>(stat[0].mean_raw_count),
        static_cast<unsigned long>(stat[0].minimum_raw_count),
        static_cast<unsigned long>(stat[0].maximum_raw_count),
        static_cast<double>(stat[0].standard_deviation_raw_count),
        static_cast<double>(stat[0].offset_voltage_v),
        static_cast<double>(stat[0].noise_rms_a),
        static_cast<double>(stat[1].mean_raw_count),
        static_cast<unsigned long>(stat[1].minimum_raw_count),
        static_cast<unsigned long>(stat[1].maximum_raw_count),
        static_cast<double>(stat[1].standard_deviation_raw_count),
        static_cast<double>(stat[1].offset_voltage_v),
        static_cast<double>(stat[1].noise_rms_a));
    return true;
}

#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
bool pwmFrequencyIsWithinOnePercent(uint32_t actualHz,
                                    uint32_t targetHz) {
    if (targetHz == 0U) {
        return false;
    }
    const uint32_t difference = actualHz > targetHz
                                    ? actualHz - targetHz
                                    : targetHz - actualHz;
    return static_cast<uint64_t>(difference) * 100U <= targetHz;
}

bool pwmSnapshotIsSafe(dengfoc_target_power_snapshot_t *snapshot) {
    if ((snapshot == nullptr) ||
        !dengfoc_target_power_port_snapshot(&gPowerPort, snapshot) ||
        (snapshot->driver_disabled == 0U) ||
        (snapshot->pwm_configured == 0U)) {
        return false;
    }
    for (uint32_t phase = 0U; phase < FOC_DENGFOC_PHASE_COUNT; ++phase) {
        if (!pwmFrequencyIsWithinOnePercent(
                snapshot->frequency_hz[phase],
                foc_dengfoc_v04_reference_profile.pwm_frequency_hz) ||
            (snapshot->duty_count[phase] != 0U)) {
            return false;
        }
    }
    return true;
}
#endif

#if FLUXRT_DENGFOC_RUST_PROBE
bool runRustControlProbe() {
    foc_runtime_config_t config{};
    foc_feedback_t feedback{};
    foc_reference_t reference{};
    foc_output_t output{};
    constexpr uint32_t kControlFrequencyHz = 2000U;
    constexpr uint32_t kSpeedFrequencyHz = 200U;

    if ((foc_rust_abi_version() != FOC_RUST_ABI_VERSION) ||
        (foc_rust_context_required_size() > sizeof(gRustController)) ||
        (foc_rust_context_required_align() > alignof(foc_rust_context_t)) ||
        (foc_rust_init(&gRustController) != FOC_STATUS_OK) ||
        (foc_rust_default_st_config(&config) != FOC_STATUS_OK)) {
        return false;
    }
    config.pwm_frequency_hz = kControlFrequencyHz;
    config.speed_loop_frequency_hz = kSpeedFrequencyHz;
    config.id_pi.ts = 1.0F / static_cast<float>(kControlFrequencyHz);
    config.iq_pi.ts = config.id_pi.ts;
    config.speed_pi.ts = 1.0F / static_cast<float>(kSpeedFrequencyHz);
    config.closed_loop_enable = 0U;
    config.observer_enable = 0U;
    if ((foc_rust_configure(&gRustController, &config) != FOC_STATUS_OK) ||
        (foc_rust_request_start(&gRustController, 1U) != FOC_STATUS_OK)) {
        foc_rust_stop(&gRustController);
        return false;
    }
    feedback.dc_bus_voltage =
        foc_dengfoc_v04_reference_profile.nominal_bus_voltage_v;
    const bool legacyOk =
        (foc_rust_fast_step(
             &gRustController, &feedback, &reference, &output) ==
         FOC_STATUS_OK) &&
        isfinite(output.duty_a) && isfinite(output.duty_b) &&
        isfinite(output.duty_c) &&
        (output.duty_a >= 0.0F) && (output.duty_a <= 1.0F) &&
        (output.duty_b >= 0.0F) && (output.duty_b <= 1.0F) &&
        (output.duty_c >= 0.0F) && (output.duty_c <= 1.0F);
    foc_rust_stop(&gRustController);

    foc_encoder_realtime_input_t encoderInput{};
    foc_telemetry_t telemetry{};
    encoderInput.struct_size = sizeof(encoderInput);
    encoderInput.version = FOC_ENCODER_REALTIME_INPUT_VERSION;
    encoderInput.control_sequence = 0U;
    encoderInput.actual_dt_s = 1.0F / static_cast<float>(kControlFrequencyHz);
    encoderInput.dc_bus_voltage_v =
        foc_dengfoc_v04_reference_profile.nominal_bus_voltage_v;
    output = {};
    const bool encoderOk =
        (foc_rust_encoder_realtime_abi_version() ==
         FOC_ENCODER_REALTIME_ABI_VERSION) &&
        (foc_rust_start_encoder_realtime(&gRustController, 1U, 0.0F) ==
         FOC_STATUS_OK) &&
        (foc_rust_encoder_realtime_step(
             &gRustController, &encoderInput, &output, &telemetry) ==
         FOC_STATUS_OK) &&
        isfinite(output.duty_a) && isfinite(output.duty_b) &&
        isfinite(output.duty_c) &&
        (fabsf(output.duty_a - 0.5F) < 0.000001F) &&
        (fabsf(output.duty_b - 0.5F) < 0.000001F) &&
        (fabsf(output.duty_c - 0.5F) < 0.000001F) &&
        (telemetry.closed_loop_active == 1U) &&
        (telemetry.observer_reliable == 0U);
    foc_rust_stop(&gRustController);
    const bool liveStartOk =
        (foc_rust_start_encoder_realtime(
             &gRustController, 1U, 0.0F) == FOC_STATUS_OK);
    const bool ok = legacyOk && encoderOk && liveStartOk;
    Serial.printf(
        "rust_probe=%u,legacy=%u,encoder=%u,shadow_start=%u,abi=0x%08lx,encoder_abi=0x%08lx,"
        "ctx=%lu,align=%lu,duty=%.6f/%.6f/%.6f\n",
        ok ? 1U : 0U,
        legacyOk ? 1U : 0U,
        encoderOk ? 1U : 0U,
        liveStartOk ? 1U : 0U,
        static_cast<unsigned long>(foc_rust_abi_version()),
        static_cast<unsigned long>(foc_rust_encoder_realtime_abi_version()),
        static_cast<unsigned long>(foc_rust_context_required_size()),
        static_cast<unsigned long>(foc_rust_context_required_align()),
        static_cast<double>(output.duty_a),
        static_cast<double>(output.duty_b),
        static_cast<double>(output.duty_c));
    return ok;
}

void runRustShadowStep() {
    RustShadowStats &stats = gRustShadowStats;
    dengfoc_adc_stream_snapshot_t adcSnapshot{};
    if (!dengfoc_adc_stream_snapshot(&gAdcStream, &adcSnapshot) ||
        (adcSnapshot.started == 0U) ||
        (adcSnapshot.sample_sequence == gRustShadowLastAdcSequence)) {
        stats.stale_adc_count += 1U;
        return;
    }
    gRustShadowLastAdcSequence = adcSnapshot.sample_sequence;

    foc_feedback_source_sample_t feedback{};
    portENTER_CRITICAL(&gFeedbackMux);
    feedback = gLastSample[0U];
    portEXIT_CRITICAL(&gFeedbackMux);
    if (feedback.valid_flags == 0U) {
        stats.invalid_feedback_count += 1U;
        stats.invalid_angle_count += 1U;
        return;
    }
    if ((gCurrentZeroStat[0U][0U].sample_count == 0U) ||
        (gCurrentZeroStat[0U][1U].sample_count == 0U)) {
        stats.invalid_feedback_count += 1U;
        stats.invalid_zero_count += 1U;
        return;
    }

    foc_encoder_realtime_input_t input{};
    input.struct_size = sizeof(input);
    input.version = FOC_ENCODER_REALTIME_INPUT_VERSION;
    input.control_sequence = gRustShadowControlSequence;
    input.actual_dt_s =
        1.0F / static_cast<float>(kShadowControlFrequencyHz);
    input.phase_current_a =
        (static_cast<float>(adcSnapshot.raw[0U]) -
         gCurrentZeroStat[0U][0U].mean_raw_count) *
        gCurrentZeroStat[0U][0U].amperes_per_count;
    input.phase_current_b =
        (static_cast<float>(adcSnapshot.raw[1U]) -
         gCurrentZeroStat[0U][1U].mean_raw_count) *
        gCurrentZeroStat[0U][1U].amperes_per_count;
    input.phase_current_c =
        -input.phase_current_a - input.phase_current_b;
    input.dc_bus_voltage_v =
        foc_dengfoc_v04_reference_profile.nominal_bus_voltage_v;
    input.electrical_angle_rad = feedback.electrical_angle_rad;
    input.mechanical_speed_rad_s = feedback.mechanical_velocity_rad_s;

    foc_output_t output{};
    foc_telemetry_t telemetry{};
    const uint32_t startedAt = esp_cpu_get_cycle_count();
    const foc_status_t status = foc_rust_encoder_realtime_step(
        &gRustController, &input, &output, &telemetry);
    const uint32_t computeCycles = esp_cpu_get_cycle_count() - startedAt;
    if (computeCycles > stats.maximum_compute_cycles) {
        stats.maximum_compute_cycles = computeCycles;
    }
    stats.last_status = static_cast<uint32_t>(status);
    if ((status != FOC_STATUS_OK) || !isfinite(output.duty_a) ||
        !isfinite(output.duty_b) || !isfinite(output.duty_c) ||
        (output.duty_a < 0.0F) || (output.duty_a > 1.0F) ||
        (output.duty_b < 0.0F) || (output.duty_b > 1.0F) ||
        (output.duty_c < 0.0F) || (output.duty_c > 1.0F)) {
        stats.step_error_count += 1U;
        return;
    }
    gRustShadowControlSequence += 1U;
    stats.step_count += 1U;
    stats.last_duty_a = output.duty_a;
    stats.last_duty_b = output.duty_b;
    stats.last_duty_c = output.duty_c;
    float deviation = fabsf(output.duty_a - 0.5F);
    deviation = max(deviation, fabsf(output.duty_b - 0.5F));
    deviation = max(deviation, fabsf(output.duty_c - 0.5F));
    if (deviation > stats.maximum_duty_deviation) {
        stats.maximum_duty_deviation = deviation;
    }
}

void IRAM_ATTR rustShadowCycleEventCallback(void *,
                                            uint32_t,
                                            uint32_t cpuCycle) {
    if (gRustShadowTask == nullptr) {
        return;
    }
    gRustShadowTriggerCpuCycle = cpuCycle;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(gRustShadowTask, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void rustShadowTask(void *) {
    while (true) {
        const uint32_t notifications =
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notifications == 0U) {
            continue;
        }
        RustShadowStats &stats = gRustShadowStats;
        stats.callback_count += notifications;
        if (notifications > 1U) {
            stats.deadline_miss_count += notifications - 1U;
        }
        const uint32_t startedAt = esp_cpu_get_cycle_count();
        const uint32_t wakeCycles =
            startedAt - gRustShadowTriggerCpuCycle;
        if (wakeCycles > stats.maximum_wake_cycles) {
            stats.maximum_wake_cycles = wakeCycles;
        }
        const uint32_t nowUs = micros();
        if (gRustShadowLastCallbackUs != 0U) {
            const uint32_t periodUs = nowUs - gRustShadowLastCallbackUs;
            if ((stats.minimum_period_us == 0U) ||
                (periodUs < stats.minimum_period_us)) {
                stats.minimum_period_us = periodUs;
            }
            if (periodUs > stats.maximum_period_us) {
                stats.maximum_period_us = periodUs;
            }
            if ((notifications == 1U) &&
                (periodUs > (kShadowPeriodUs * 3U) / 2U)) {
                stats.deadline_miss_count += 1U;
            }
        }
        gRustShadowLastCallbackUs = nowUs;
        if (!gFatal) {
            runRustShadowStep();
        }
    }
}

bool startRustShadowOwner() {
    gRustShadowStats = {};
    gRustShadowControlSequence = 0U;
    gRustShadowLastAdcSequence = 0U;
    gRustShadowLastCallbackUs = 0U;
    gRustShadowTriggerCpuCycle = 0U;
    if (xTaskCreatePinnedToCore(
            rustShadowTask,
            "fluxrt_shadow",
            8192U,
            nullptr,
            configMAX_PRIORITIES - 3,
            &gRustShadowTask,
            xPortGetCoreID()) != pdPASS) {
        gRustShadowTask = nullptr;
        return false;
    }
    dengfoc_mcpwm_power_port_set_divided_cycle_event_callback(
        &gPowerPort,
        kShadowPwmDivider,
        rustShadowCycleEventCallback,
        nullptr);
    return true;
}
#endif

}  // namespace

#if FLUXRT_DENGFOC_RUST_PROBE
extern "C" void foc_platform_emergency_stop(void) {
    forcePowerStageDisabled();
}
#endif

void setup() {
    foc_dengfoc_power_ops_t powerOps{};
    const foc_dengfoc_power_policy_t powerPolicy = {
        .allow_actuation = 0U,
        .operation = FOC_DENGFOC_OPERATION_DISABLED,
        .required_capabilities = 0U,
        .maximum_active_duty_deviation_count = 0U,
    };
    const bool powerPortReady = dengfoc_target_power_port_init(
        &gPowerPort,
        &foc_dengfoc_v04_reference_profile,
        0U,
        false,
        &powerOps);
    const bool powerStageReady = powerPortReady && foc_dengfoc_power_stage_init(
        &gPowerStage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &powerPolicy,
        &powerOps);
    forcePowerStageDisabled();

    Serial.begin(kSerialBaud);
    delay(200U);

#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
#if FLUXRT_DENGFOC_MCPWM_TIMING_ONLY
#if FLUXRT_DENGFOC_RUST_PROBE
    Serial.println("FluxRT DengFOC USB-only Rust control probe");
#else
    Serial.println("FluxRT DengFOC USB-only MCPWM timing diagnostic");
#endif
#else
    Serial.println("FluxRT DengFOC USB-only PWM prepare diagnostic");
#endif
#else
    Serial.println("FluxRT DengFOC AS5600 USB-only diagnostic");
#endif
    Serial.printf("driver_enable_gpio=%lu,disabled=%u\n",
                  static_cast<unsigned long>(
                      foc_dengfoc_v04_reference_profile.driver_enable_gpio),
                  powerStageIsDisabled() ? 1U : 0U);
    Serial.printf("power_stage_init=%u,state=%u,allow_actuation=%lu,pwm_configured=%lu\n",
                  powerStageReady ? 1U : 0U,
                  static_cast<unsigned>(gPowerStage.state),
                  static_cast<unsigned long>(gPowerStage.allow_actuation),
                  static_cast<unsigned long>(gPowerStage.pwm_configured));
    if (!powerStageReady || !powerStageIsDisabled() ||
        !foc_dengfoc_v04_profile_valid(&foc_dengfoc_v04_reference_profile)) {
        forcePowerStageDisabled();
        gFatal = true;
        Serial.println("FATAL: board profile or driver-disable gate failed");
        return;
    }

    const bool axis0Ready = initializeAxis(0U);
    Serial.printf("axis0_init=%u\n", axis0Ready ? 1U : 0U);
    const bool axis0AdcZero = captureAndPrintCurrentZero(0U);
    Serial.printf("axis0_adc_zero=%u\n", axis0AdcZero ? 1U : 0U);
#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
    dengfoc_target_power_snapshot_t snapshot{};
    const bool prepared = foc_dengfoc_power_stage_prepare(&gPowerStage);
    const bool armRefused =
        !foc_dengfoc_power_stage_arm(&gPowerStage, gPowerStage.fault_epoch);
    const bool snapshotSafe = pwmSnapshotIsSafe(&snapshot);
    Serial.printf(
        "pwm_prepare=%u,state=%u,disabled=%lu,configured=%lu,"
        "freq=%lu/%lu/%lu,duty=%lu/%lu/%lu,arm_refused=%u,"
        "caps=0x%08lx,missing_policy=0x%08lx\n",
        (prepared && snapshotSafe && armRefused) ? 1U : 0U,
        static_cast<unsigned>(gPowerStage.state),
        static_cast<unsigned long>(snapshot.driver_disabled),
        static_cast<unsigned long>(snapshot.pwm_configured),
        static_cast<unsigned long>(snapshot.frequency_hz[0]),
        static_cast<unsigned long>(snapshot.frequency_hz[1]),
        static_cast<unsigned long>(snapshot.frequency_hz[2]),
        static_cast<unsigned long>(snapshot.duty_count[0]),
        static_cast<unsigned long>(snapshot.duty_count[1]),
        static_cast<unsigned long>(snapshot.duty_count[2]),
        armRefused ? 1U : 0U,
        static_cast<unsigned long>(gPowerStage.platform_capabilities),
        static_cast<unsigned long>(gPowerStage.missing_capabilities));
#if FLUXRT_DENGFOC_MCPWM_TIMING_ONLY
    Serial.printf(
        "mcpwm_timing=1,caps=0x%08lx,missing_runtime=0x%08lx,"
        "period=%lu,peak=%lu,full=%lu,coherent=%lu,update_step=%lu,update_err=%ld\n",
        static_cast<unsigned long>(snapshot.capabilities),
        static_cast<unsigned long>(
            FOC_DENGFOC_CURRENT_LOOP_REQUIRED_CAPABILITIES &
            ~snapshot.capabilities),
        static_cast<unsigned long>(snapshot.period_ticks),
        static_cast<unsigned long>(snapshot.peak_ticks),
        static_cast<unsigned long>(snapshot.full_event_count),
        static_cast<unsigned long>(snapshot.coherent_update_count),
        static_cast<unsigned long>(snapshot.last_update_step),
        static_cast<long>(snapshot.last_update_error));
    gLastMcpwmFullCount = snapshot.full_event_count;
#endif
    if (!axis0Ready || !axis0AdcZero || !prepared || !snapshotSafe || !armRefused ||
        (gPowerStage.state != FOC_DENGFOC_POWER_PWM_READY)) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        Serial.println("FATAL: USB-only PWM prepare gate failed");
    }
#if FLUXRT_DENGFOC_RUST_PROBE
    const auto &axisProfile =
        foc_dengfoc_v04_reference_profile.axis[0U];
    const bool adcStreamReady = dengfoc_adc_stream_init(
        &gAdcStream,
        axisProfile.current_a_adc_gpio,
        axisProfile.current_b_adc_gpio,
        kAdcProbeSampleFrequencyHz);
    if (adcStreamReady) {
        dengfoc_mcpwm_power_port_set_cycle_event_callback(
            &gPowerPort,
            dengfoc_adc_stream_mark_pwm_event,
            &gAdcStream);
    }
    Serial.printf("adc_stream_init=%u,sample_hz=%lu\n",
                  adcStreamReady ? 1U : 0U,
                  static_cast<unsigned long>(kAdcProbeSampleFrequencyHz));
    if (!adcStreamReady) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        Serial.println("FATAL: ADC continuous timing probe init failed");
    }
    if (!gFatal && !runRustControlProbe()) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        Serial.println("FATAL: Rust XTensa control probe failed");
    }
    const bool shadowOwnerReady = !gFatal && startRustShadowOwner();
    Serial.printf(
        "rust_shadow_owner=%u,frequency_hz=%lu,pwm_divider=%lu,output_commit=0\n",
        shadowOwnerReady ? 1U : 0U,
        static_cast<unsigned long>(kShadowControlFrequencyHz),
        static_cast<unsigned long>(kShadowPwmDivider));
    if (!shadowOwnerReady) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        Serial.println("FATAL: Rust shadow owner start failed");
    }
#endif
#endif
#if FLUXRT_DENGFOC_ENABLE_AXIS1
    Serial.printf("axis1_init=%u\n", initializeAxis(1U) ? 1U : 0U);
#endif
}

void loop() {
#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
    forcePowerStageDisabled();
#else
    (void)foc_dengfoc_power_stage_shutdown(&gPowerStage);
#endif
    if (!powerStageIsDisabled()) {
        Serial.println("FATAL: driver enable escaped disabled state");
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        delay(1000U);
        return;
    }
    if (gFatal) {
        delay(1000U);
        return;
    }

#if FLUXRT_DENGFOC_RUST_PROBE
    if (!dengfoc_adc_stream_service(&gAdcStream)) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gFatal = true;
        Serial.println("FATAL: ADC continuous service failed");
        return;
    }
#endif

    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - gLastPollUs) >= kPollPeriodUs) {
        gLastPollUs = nowUs;
        const uint32_t nowMs = millis();
        pollAxis(0U, nowMs, nowUs);
#if FLUXRT_DENGFOC_ENABLE_AXIS1
        pollAxis(1U, nowMs, nowUs);
#endif
    }

    const uint32_t nowMs = millis();
#if FLUXRT_DENGFOC_PWM_PREPARE_ONLY
    if (static_cast<uint32_t>(nowMs - gLastPowerCheckMs) >= 100U) {
        dengfoc_target_power_snapshot_t snapshot{};
        bool timingAlive = true;
        gLastPowerCheckMs = nowMs;
#if FLUXRT_DENGFOC_MCPWM_TIMING_ONLY
        timingAlive = dengfoc_target_power_port_snapshot(
                          &gPowerPort, &snapshot) &&
                      (snapshot.full_event_count !=
                       gLastMcpwmFullCount) &&
                      (snapshot.coherent_update_count != 0U);
        gLastMcpwmFullCount = snapshot.full_event_count;
#endif
#if FLUXRT_DENGFOC_RUST_PROBE
        dengfoc_adc_stream_snapshot_t adcSnapshot{};
        const bool adcSnapshotValid = dengfoc_adc_stream_snapshot(
            &gAdcStream, &adcSnapshot);
        const bool noNewAdcErrors =
            !gAdcHealthBaselineReady ||
            ((adcSnapshot.pool_overflow_count ==
              gLastAdcPoolOverflowCount) &&
             (adcSnapshot.read_error_count == gLastAdcReadErrorCount));
        const bool adcAlive = adcSnapshotValid &&
                              (adcSnapshot.started != 0U) &&
                              (adcSnapshot.sample_sequence !=
                               gLastAdcSampleSequence) &&
                              noNewAdcErrors;
        gLastAdcSampleSequence = adcSnapshot.sample_sequence;
        gLastAdcPoolOverflowCount = adcSnapshot.pool_overflow_count;
        gLastAdcReadErrorCount = adcSnapshot.read_error_count;
        gAdcHealthBaselineReady = true;
        Serial.printf(
            "adc_stream=1,alive=%u,pwm=%lu,sample=%lu,sample_pwm=%lu,"
            "raw=%lu/%lu,delay_cycles=%lu,overflow=%lu,read_err=%lu,miss=%lu\n",
            adcAlive ? 1U : 0U,
            static_cast<unsigned long>(adcSnapshot.pwm_event_sequence),
            static_cast<unsigned long>(adcSnapshot.sample_sequence),
            static_cast<unsigned long>(
                adcSnapshot.sampled_pwm_event_sequence),
            static_cast<unsigned long>(adcSnapshot.raw[0]),
            static_cast<unsigned long>(adcSnapshot.raw[1]),
            static_cast<unsigned long>(adcSnapshot.sample_delay_cycles),
            static_cast<unsigned long>(adcSnapshot.pool_overflow_count),
            static_cast<unsigned long>(adcSnapshot.read_error_count),
            static_cast<unsigned long>(
                adcSnapshot.missed_pwm_window_count));
        Serial.printf(
            "rust_shadow=1,callbacks=%lu,steps=%lu,stale_adc=%lu,"
            "invalid_feedback=%lu,invalid_angle=%lu,invalid_zero=%lu,"
            "step_error=%lu,deadline_miss=%lu,"
            "period_us=%lu/%lu,max_cycles=%lu,max_wake_cycles=%lu,status=%lu,"
            "duty=%.6f/%.6f/%.6f,max_dev=%.6f,output_commit=0\n",
            static_cast<unsigned long>(
                gRustShadowStats.callback_count),
            static_cast<unsigned long>(gRustShadowStats.step_count),
            static_cast<unsigned long>(gRustShadowStats.stale_adc_count),
            static_cast<unsigned long>(
                gRustShadowStats.invalid_feedback_count),
            static_cast<unsigned long>(
                gRustShadowStats.invalid_angle_count),
            static_cast<unsigned long>(
                gRustShadowStats.invalid_zero_count),
            static_cast<unsigned long>(gRustShadowStats.step_error_count),
            static_cast<unsigned long>(
                gRustShadowStats.deadline_miss_count),
            static_cast<unsigned long>(
                gRustShadowStats.minimum_period_us),
            static_cast<unsigned long>(
                gRustShadowStats.maximum_period_us),
            static_cast<unsigned long>(
                gRustShadowStats.maximum_compute_cycles),
            static_cast<unsigned long>(
                gRustShadowStats.maximum_wake_cycles),
            static_cast<unsigned long>(gRustShadowStats.last_status),
            static_cast<double>(gRustShadowStats.last_duty_a),
            static_cast<double>(gRustShadowStats.last_duty_b),
            static_cast<double>(gRustShadowStats.last_duty_c),
            static_cast<double>(
                gRustShadowStats.maximum_duty_deviation));
        timingAlive = timingAlive && adcAlive;
#endif
        if ((gPowerStage.state != FOC_DENGFOC_POWER_PWM_READY) ||
            !timingAlive || !pwmSnapshotIsSafe(&snapshot)) {
            foc_dengfoc_power_stage_latch_fault(&gPowerStage);
            gFatal = true;
            Serial.println("FATAL: PWM prepare invariant escaped");
            return;
        }
    }
#endif
    if (static_cast<uint32_t>(nowMs - gLastTelemetryMs) >=
        kTelemetryPeriodMs) {
        gLastTelemetryMs = nowMs;
        printAxis(0U);
#if FLUXRT_DENGFOC_ENABLE_AXIS1
        printAxis(1U);
#endif
    }
    delay(1U);
}
