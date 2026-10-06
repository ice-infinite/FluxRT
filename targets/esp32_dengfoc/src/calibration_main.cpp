#include <Arduino.h>
#include <Wire.h>
#include <esp_random.h>

#include <cmath>
#include <cstring>

#include "dengfoc_adc_stream.h"
#include "dengfoc_commissioning_candidate.h"
#include "dengfoc_mcpwm_power_port.h"

extern "C" {
#include "foc_board_dengfoc_v04.h"
#include "foc_dengfoc_as5600_port.h"
#include "foc_dengfoc_current_sense.h"
}

#if !defined(FLUXRT_DENGFOC_ALIGNMENT_CALIBRATION) || \
    (FLUXRT_DENGFOC_ALIGNMENT_CALIBRATION != 1)
#error "calibration_main.cpp is only for the explicit alignment target"
#endif

namespace {

constexpr uint32_t kSerialBaud = 921600U;
constexpr uint32_t kEncoderPollPeriodUs = 5000U;
constexpr uint32_t kEncoderMaximumPeriodUs = 20000U;
constexpr uint32_t kAdcSampleFrequencyHz = 60000U;
constexpr uint32_t kCurrentZeroSamples = 1000U;
constexpr uint32_t kPrepareWindowMs = 15000U;
constexpr uint32_t kAlignmentRampMs = 300U;
constexpr uint32_t kAlignmentTotalMs = 1000U;
constexpr uint32_t kAlignmentSampleStartMs = 700U;
constexpr uint32_t kControlPeriodUs = 500U;
constexpr float kAlignmentStartVoltageV = 0.15F;
constexpr float kAlignmentMaximumVoltageV = 0.60F;
constexpr float kSoftwareCurrentTripA = 0.25F;
constexpr float kMinimumEnergizedCurrentA = 0.015F;
constexpr uint32_t kMaximumDutyDeviationCount = 16U;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kSqrtThreeOverTwo = 0.86602540378443864676F;

enum class PreparedOperation : uint32_t {
    None = 0U,
    Alignment = 1U,
    DirectionPositive = 2U,
    DirectionNegative = 3U,
};

struct ArduinoI2cContext {
    TwoWire *wire;
};

TwoWire gWire(0);
ArduinoI2cContext gI2cContext{&gWire};
foc_dengfoc_as5600_port_t gEncoder{};
foc_feedback_source_sample_t gFeedback{};
dengfoc_mcpwm_power_port_t gPowerPort{};
foc_dengfoc_power_stage_t gPowerStage{};
dengfoc_adc_stream_t gAdcStream{};
foc_dengfoc_current_zero_stat_t
    gCurrentZero[FOC_DENGFOC_CURRENT_CHANNEL_COUNT]{};
bool gReady = false;
uint32_t gPwmSequence = 0U;
uint32_t gPrepareToken = 0U;
uint32_t gPrepareDeadlineMs = 0U;
PreparedOperation gPreparedOperation = PreparedOperation::None;
uint32_t gLastEncoderPollUs = 0U;
char gCommand[64]{};
size_t gCommandLength = 0U;

struct DirectionPeakDiagnostic {
    bool valid;
    uint32_t elapsed_ms;
    uint32_t sample_sequence;
    uint32_t pwm_event_sequence;
    uint32_t sampled_pwm_event_sequence;
    uint32_t sample_delay_cycles;
    uint32_t missed_pwm_window_count;
    uint32_t raw[2];
    float ia;
    float ib;
    float ic;
    float current;
    float commanded_q_voltage_v;
    float electrical_angle_rad;
    uint32_t duty_count[FOC_DENGFOC_PHASE_COUNT];
    uint32_t command_count;
};

bool beginI2c(void *opaque,
              uint32_t controller,
              uint32_t sdaGpio,
              uint32_t sclGpio,
              uint32_t frequencyHz) {
    auto *context = static_cast<ArduinoI2cContext *>(opaque);
    return (context != nullptr) && (context->wire != nullptr) &&
           (controller == 0U) &&
           context->wire->begin(static_cast<int>(sdaGpio),
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
    if ((context == nullptr) || (context->wire == nullptr) ||
        (controller != 0U) || (data == nullptr) || (length == 0U) ||
        (length > 255U)) {
        return false;
    }
    TwoWire &wire = *context->wire;
    wire.beginTransmission(address);
    if ((wire.write(registerAddress) != 1U) ||
        (wire.endTransmission(false) != 0U)) {
        return false;
    }
    if (wire.requestFrom(static_cast<uint16_t>(address),
                         static_cast<uint8_t>(length), true) != length) {
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (wire.available() <= 0) {
            return false;
        }
        data[index] = static_cast<uint8_t>(wire.read());
    }
    return true;
}

bool readAdcRaw(void *, uint32_t gpio, uint32_t *rawCount) {
    if (rawCount == nullptr) {
        return false;
    }
    *rawCount = static_cast<uint32_t>(
        analogRead(static_cast<uint8_t>(gpio)));
    delayMicroseconds(500U);
    return true;
}

const foc_dengfoc_i2c_ops_t kI2cOps = {
    .begin = beginI2c,
    .read_register = readI2cRegister,
};

const foc_dengfoc_adc_ops_t kAdcOps = {
    .context = nullptr,
    .read_raw = readAdcRaw,
};

void forceDisabled() {
    (void)dengfoc_mcpwm_power_port_force_disabled(&gPowerPort);
}

bool pollEncoder(uint32_t nowUs) {
    if ((gLastEncoderPollUs != 0U) &&
        (static_cast<uint32_t>(nowUs - gLastEncoderPollUs) <
         kEncoderPollPeriodUs)) {
        return gFeedback.valid_flags != 0U;
    }
    gLastEncoderPollUs = nowUs;
    foc_feedback_source_sample_t sample{};
    if (!foc_dengfoc_as5600_port_poll(
            &gEncoder, millis(), nowUs, &sample)) {
        gFeedback = {};
        return false;
    }
    gFeedback = sample;
    return sample.valid_flags != 0U;
}

bool captureCurrentZero() {
    const auto &board = foc_dengfoc_v04_reference_profile;
    const auto &axis = board.axis[0U];
    pinMode(static_cast<uint8_t>(axis.current_a_adc_gpio), INPUT);
    pinMode(static_cast<uint8_t>(axis.current_b_adc_gpio), INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(
        static_cast<uint8_t>(axis.current_a_adc_gpio), ADC_11db);
    analogSetPinAttenuation(
        static_cast<uint8_t>(axis.current_b_adc_gpio), ADC_11db);
    forceDisabled();
    return foc_dengfoc_current_capture_zero(
        &board, 0U, &kAdcOps, kCurrentZeroSamples, gCurrentZero);
}

float currentFromRaw(uint32_t channel, uint32_t raw) {
    return (static_cast<float>(raw) -
            gCurrentZero[channel].mean_raw_count) *
           gCurrentZero[channel].amperes_per_count;
}

uint32_t dutyFromPhaseVoltage(float phaseVoltageV) {
    const float bus = foc_dengfoc_v04_reference_profile.nominal_bus_voltage_v;
    const float duty = 0.5F + phaseVoltageV / bus;
    const float scaled = duty * 255.0F;
    return static_cast<uint32_t>(lroundf(scaled));
}

bool serialAbortRequested() {
    bool abort = false;
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if ((value == 'X') || (value == 'x') || (value == 0x03)) {
            abort = true;
        }
    }
    return abort;
}

bool commitAlignmentVoltage(float voltageV) {
    /* Voltage-only d-axis alignment at electrical field angle zero.  This is
     * commissioning math, not the production current-control composition. */
    const float phaseA = voltageV;
    const float phaseB = -0.5F * voltageV;
    const float phaseC = -0.5F * voltageV;
    const foc_dengfoc_pwm_command_t command = {
        .sequence = ++gPwmSequence,
        .duty_count = {
            dutyFromPhaseVoltage(phaseA),
            dutyFromPhaseVoltage(phaseB),
            dutyFromPhaseVoltage(phaseC),
        },
    };
    return foc_dengfoc_power_stage_commit(
        &gPowerStage, gPowerStage.fault_epoch, &command);
}

bool commitEncoderCommutatedVoltage(float qVoltageV,
                                    float electricalAngleRad) {
    if (!isfinite(qVoltageV) || !isfinite(electricalAngleRad)) {
        return false;
    }
    const float sine = sinf(electricalAngleRad);
    const float cosine = cosf(electricalAngleRad);
    const float alpha = -qVoltageV * sine;
    const float beta = qVoltageV * cosine;
    const float phaseA = alpha;
    const float phaseB = -0.5F * alpha + kSqrtThreeOverTwo * beta;
    const float phaseC = -0.5F * alpha - kSqrtThreeOverTwo * beta;
    const foc_dengfoc_pwm_command_t command = {
        .sequence = ++gPwmSequence,
        .duty_count = {
            dutyFromPhaseVoltage(phaseA),
            dutyFromPhaseVoltage(phaseB),
            dutyFromPhaseVoltage(phaseC),
        },
    };
    return foc_dengfoc_power_stage_commit(
        &gPowerStage, gPowerStage.fault_epoch, &command);
}

void printStatus() {
    dengfoc_mcpwm_power_snapshot_t power{};
    dengfoc_adc_stream_snapshot_t adc{};
    const bool powerOk = dengfoc_mcpwm_power_port_snapshot(
        &gPowerPort, &power);
    const bool adcOk = dengfoc_adc_stream_snapshot(&gAdcStream, &adc);
    Serial.printf(
        "CAL_STATUS,ready=%u,state=%u,disabled=%u,caps=0x%08lx,"
        "adc=%u,overflow=%lu,read_err=%lu,encoder=%u,token_live=%u,"
        "prepared_operation=%lu\n",
        gReady ? 1U : 0U,
        static_cast<unsigned>(gPowerStage.state),
        (powerOk && power.driver_disabled != 0U) ? 1U : 0U,
        static_cast<unsigned long>(power.capabilities),
        adcOk && adc.started != 0U ? 1U : 0U,
        static_cast<unsigned long>(adc.pool_overflow_count),
        static_cast<unsigned long>(adc.read_error_count),
        gFeedback.valid_flags != 0U ? 1U : 0U,
        (gPrepareToken != 0U &&
         static_cast<int32_t>(gPrepareDeadlineMs - millis()) > 0)
            ? 1U
            : 0U,
        static_cast<unsigned long>(gPreparedOperation));
    const float ia = adcOk ? currentFromRaw(0U, adc.raw[0U]) : 0.0F;
    const float ib = adcOk ? currentFromRaw(1U, adc.raw[1U]) : 0.0F;
    const float ic = -ia - ib;
    Serial.printf(
        "CAL_ADC,valid=%u,sample_seq=%lu,pwm_event_seq=%lu,"
        "sampled_pwm_event_seq=%lu,sample_delay_cycles=%lu,"
        "missed_pwm_window=%lu,raw_a=%lu,raw_b=%lu,"
        "ia=%.6f,ib=%.6f,ic=%.6f\n",
        adcOk ? 1U : 0U,
        static_cast<unsigned long>(adc.sample_sequence),
        static_cast<unsigned long>(adc.pwm_event_sequence),
        static_cast<unsigned long>(adc.sampled_pwm_event_sequence),
        static_cast<unsigned long>(adc.sample_delay_cycles),
        static_cast<unsigned long>(adc.missed_pwm_window_count),
        static_cast<unsigned long>(adc.raw[0U]),
        static_cast<unsigned long>(adc.raw[1U]),
        static_cast<double>(ia),
        static_cast<double>(ib),
        static_cast<double>(ic));
}

void runAlignment() {
    dengfoc_adc_stream_snapshot_t initialAdc{};
    if (!gReady || !dengfoc_adc_stream_snapshot(&gAdcStream, &initialAdc) ||
        !pollEncoder(micros()) ||
        !foc_dengfoc_power_stage_arm(
            &gPowerStage, gPowerStage.fault_epoch)) {
        forceDisabled();
        Serial.println("CAL_RESULT,ok=0,reason=preflight");
        return;
    }

    const float initialMechanicalPositionRad =
        gFeedback.mechanical_position_rad;
    const float initialMultiTurnPositionRad =
        gFeedback.multi_turn_position_rad;
    const uint32_t startedMs = millis();
    uint32_t nextControlUs = micros();
    uint32_t encoderSamples = 0U;
    uint32_t lastAveragedEncoderSequence = 0U;
    float sineSum = 0.0F;
    float cosineSum = 0.0F;
    float mechanicalSineSum = 0.0F;
    float mechanicalCosineSum = 0.0F;
    float multiTurnPositionSum = 0.0F;
    float minimumSettledMultiTurnPositionRad = 0.0F;
    float maximumSettledMultiTurnPositionRad = 0.0F;
    float maximumSettledSpeedRadS = 0.0F;
    float maximumCurrentA = 0.0F;
    const char *failure = nullptr;

    while (static_cast<uint32_t>(millis() - startedMs) <
           kAlignmentTotalMs) {
        if (serialAbortRequested()) {
            failure = "operator_abort";
            break;
        }
        if (!dengfoc_adc_stream_service(&gAdcStream)) {
            failure = "adc_service";
            break;
        }
        dengfoc_adc_stream_snapshot_t adc{};
        if (!dengfoc_adc_stream_snapshot(&gAdcStream, &adc) ||
            (adc.pool_overflow_count != initialAdc.pool_overflow_count) ||
            (adc.read_error_count != initialAdc.read_error_count)) {
            failure = "adc_health";
            break;
        }
        const float ia = currentFromRaw(0U, adc.raw[0U]);
        const float ib = currentFromRaw(1U, adc.raw[1U]);
        const float ic = -ia - ib;
        const float current = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
        maximumCurrentA = fmaxf(maximumCurrentA, current);
        if (!isfinite(current) || (current > kSoftwareCurrentTripA)) {
            failure = "software_current_trip";
            break;
        }

        const uint32_t nowUs = micros();
        if (!pollEncoder(nowUs)) {
            failure = "encoder";
            break;
        }
        const uint32_t elapsedMs = millis() - startedMs;
        if ((elapsedMs >= kAlignmentSampleStartMs) &&
            (gFeedback.sequence != lastAveragedEncoderSequence)) {
            const float rawElectrical = fmodf(
                static_cast<float>(
                    kDengfocM0CommissioningCandidate.pole_pairs) *
                    gFeedback.mechanical_position_rad,
                kTwoPi);
            sineSum += sinf(rawElectrical);
            cosineSum += cosf(rawElectrical);
            mechanicalSineSum += sinf(gFeedback.mechanical_position_rad);
            mechanicalCosineSum += cosf(gFeedback.mechanical_position_rad);
            multiTurnPositionSum += gFeedback.multi_turn_position_rad;
            maximumSettledSpeedRadS = fmaxf(
                maximumSettledSpeedRadS,
                fabsf(gFeedback.mechanical_velocity_rad_s));
            if (encoderSamples == 0U) {
                minimumSettledMultiTurnPositionRad =
                    gFeedback.multi_turn_position_rad;
                maximumSettledMultiTurnPositionRad =
                    gFeedback.multi_turn_position_rad;
            } else {
                minimumSettledMultiTurnPositionRad = fminf(
                    minimumSettledMultiTurnPositionRad,
                    gFeedback.multi_turn_position_rad);
                maximumSettledMultiTurnPositionRad = fmaxf(
                    maximumSettledMultiTurnPositionRad,
                    gFeedback.multi_turn_position_rad);
            }
            encoderSamples += 1U;
            lastAveragedEncoderSequence = gFeedback.sequence;
        }

        if (static_cast<int32_t>(nowUs - nextControlUs) >= 0) {
            float voltage = kAlignmentMaximumVoltageV;
            if (elapsedMs < kAlignmentRampMs) {
                const float fraction =
                    static_cast<float>(elapsedMs) /
                    static_cast<float>(kAlignmentRampMs);
                voltage = kAlignmentStartVoltageV +
                          fraction * (kAlignmentMaximumVoltageV -
                                      kAlignmentStartVoltageV);
            }
            if (!commitAlignmentVoltage(voltage)) {
                failure = "pwm_commit";
                break;
            }
            nextControlUs += kControlPeriodUs;
        }
        delayMicroseconds(50U);
    }

    const bool shutdownOk = foc_dengfoc_power_stage_shutdown(&gPowerStage);
    forceDisabled();
    gPwmSequence = 0U;
    if ((failure == nullptr) &&
        ((maximumCurrentA < kMinimumEnergizedCurrentA) ||
         (encoderSamples < 10U))) {
        failure = maximumCurrentA < kMinimumEnergizedCurrentA
                      ? "not_energized"
                      : "insufficient_encoder_samples";
    }
    if ((failure != nullptr) || !shutdownOk) {
        Serial.printf("CAL_RESULT,ok=0,reason=%s,max_current_a=%.6f,samples=%lu\n",
                      failure != nullptr ? failure : "shutdown",
                      static_cast<double>(maximumCurrentA),
                      static_cast<unsigned long>(encoderSamples));
        return;
    }

    float offset = atan2f(sineSum, cosineSum);
    if (offset < 0.0F) {
        offset += kTwoPi;
    }
    float finalMechanicalPositionRad =
        atan2f(mechanicalSineSum, mechanicalCosineSum);
    if (finalMechanicalPositionRad < 0.0F) {
        finalMechanicalPositionRad += kTwoPi;
    }
    const float finalMultiTurnPositionRad =
        multiTurnPositionSum / static_cast<float>(encoderSamples);
    const float mechanicalDisplacementRad =
        finalMultiTurnPositionRad - initialMultiTurnPositionRad;
    const float settledMechanicalSpanRad =
        maximumSettledMultiTurnPositionRad -
        minimumSettledMultiTurnPositionRad;
    Serial.printf(
        "CAL_RESULT,ok=1,electrical_offset_rad=%.9f,max_current_a=%.6f,"
        "samples=%lu,start_mech_rad=%.9f,final_mech_rad=%.9f,"
        "delta_mech_rad=%.9f,settle_span_mech_rad=%.9f,"
        "max_settle_speed_rad_s=%.6f,persisted=0\n",
        static_cast<double>(offset),
        static_cast<double>(maximumCurrentA),
        static_cast<unsigned long>(encoderSamples),
        static_cast<double>(initialMechanicalPositionRad),
        static_cast<double>(finalMechanicalPositionRad),
        static_cast<double>(mechanicalDisplacementRad),
        static_cast<double>(settledMechanicalSpanRad),
        static_cast<double>(maximumSettledSpeedRadS));
}

void runDirectionGate(int32_t requestedDirection) {
    dengfoc_adc_stream_snapshot_t initialAdc{};
    if (((requestedDirection != 1) && (requestedDirection != -1)) ||
        !gReady || !dengfoc_adc_stream_snapshot(&gAdcStream, &initialAdc) ||
        !pollEncoder(micros()) ||
        !foc_dengfoc_power_stage_arm(
            &gPowerStage, gPowerStage.fault_epoch)) {
        forceDisabled();
        Serial.println("DIR_RESULT,ok=0,reason=preflight");
        return;
    }

    const float initialMechanicalPositionRad =
        gFeedback.mechanical_position_rad;
    const float initialMultiTurnPositionRad =
        gFeedback.multi_turn_position_rad;
    float previousMultiTurnPositionRad = initialMultiTurnPositionRad;
    uint32_t lastEncoderSequence = gFeedback.sequence;
    uint32_t encoderSamples = 0U;
    uint32_t directionEvidenceSamples = 0U;
    float maximumCurrentA = 0.0F;
    float maximumSpeedRadS = 0.0F;
    float displacementRad = 0.0F;
    float lastCommandedQVoltageV = 0.0F;
    float lastElectricalAngleRad = 0.0F;
    uint32_t commandCount = 0U;
    DirectionPeakDiagnostic peakDiagnostic{};
    bool directionProven = false;
    const uint32_t startedMs = millis();
    uint32_t nextControlUs = micros();
    const char *failure = nullptr;

    while (static_cast<uint32_t>(millis() - startedMs) <
           kDengfocM0CommissioningCandidate.direction_total_ms) {
        if (serialAbortRequested()) {
            failure = "operator_abort";
            break;
        }
        if (!dengfoc_adc_stream_service(&gAdcStream)) {
            failure = "adc_service";
            break;
        }
        dengfoc_adc_stream_snapshot_t adc{};
        if (!dengfoc_adc_stream_snapshot(&gAdcStream, &adc) ||
            (adc.pool_overflow_count != initialAdc.pool_overflow_count) ||
            (adc.read_error_count != initialAdc.read_error_count)) {
            failure = "adc_health";
            break;
        }
        const float ia = currentFromRaw(0U, adc.raw[0U]);
        const float ib = currentFromRaw(1U, adc.raw[1U]);
        const float ic = -ia - ib;
        const float current = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
        /* Evidence only: capture the peak sample before the unchanged
         * immediate software-current trip below. Diagnostics must never
         * debounce, filter or otherwise weaken the commissioning guard. */
        if (isfinite(current) && (current > maximumCurrentA)) {
            maximumCurrentA = current;
            peakDiagnostic.valid = true;
            peakDiagnostic.elapsed_ms = millis() - startedMs;
            peakDiagnostic.sample_sequence = adc.sample_sequence;
            peakDiagnostic.pwm_event_sequence = adc.pwm_event_sequence;
            peakDiagnostic.sampled_pwm_event_sequence =
                adc.sampled_pwm_event_sequence;
            peakDiagnostic.sample_delay_cycles = adc.sample_delay_cycles;
            peakDiagnostic.missed_pwm_window_count =
                adc.missed_pwm_window_count;
            peakDiagnostic.raw[0U] = adc.raw[0U];
            peakDiagnostic.raw[1U] = adc.raw[1U];
            peakDiagnostic.ia = ia;
            peakDiagnostic.ib = ib;
            peakDiagnostic.ic = ic;
            peakDiagnostic.current = current;
            peakDiagnostic.commanded_q_voltage_v = lastCommandedQVoltageV;
            peakDiagnostic.electrical_angle_rad = lastElectricalAngleRad;
            peakDiagnostic.command_count = commandCount;
            dengfoc_mcpwm_power_snapshot_t power{};
            if (dengfoc_mcpwm_power_port_snapshot(&gPowerPort, &power)) {
                for (uint32_t phase = 0U;
                     phase < FOC_DENGFOC_PHASE_COUNT;
                     ++phase) {
                    peakDiagnostic.duty_count[phase] =
                        power.duty_count[phase];
                }
            }
        }
        if (!isfinite(current) || (current > kSoftwareCurrentTripA)) {
            failure = "software_current_trip";
            break;
        }

        const uint32_t nowUs = micros();
        if (!pollEncoder(nowUs)) {
            failure = "encoder";
            break;
        }
        if (gFeedback.sequence != lastEncoderSequence) {
            lastEncoderSequence = gFeedback.sequence;
            encoderSamples += 1U;
            const float incrementalMotionRad =
                gFeedback.multi_turn_position_rad -
                previousMultiTurnPositionRad;
            previousMultiTurnPositionRad =
                gFeedback.multi_turn_position_rad;
            displacementRad =
                gFeedback.multi_turn_position_rad -
                initialMultiTurnPositionRad;
            maximumSpeedRadS = fmaxf(
                maximumSpeedRadS,
                fabsf(gFeedback.mechanical_velocity_rad_s));
            if (!isfinite(displacementRad) ||
                !isfinite(incrementalMotionRad) ||
                !isfinite(gFeedback.mechanical_velocity_rad_s)) {
                failure = "encoder_nonfinite";
                break;
            }
            if ((static_cast<float>(requestedDirection) *
                 incrementalMotionRad) >=
                kDengfocM0CommissioningCandidate
                    .direction_minimum_increment_rad) {
                directionEvidenceSamples += 1U;
            }
            if (maximumSpeedRadS >
                kDengfocM0CommissioningCandidate
                    .direction_maximum_speed_rad_s) {
                failure = "speed_limit";
                break;
            }
            if (fabsf(displacementRad) >=
                kDengfocM0CommissioningCandidate
                    .direction_maximum_movement_rad) {
                failure = "movement_limit";
                break;
            }
            if ((static_cast<float>(requestedDirection) * displacementRad) <
                -kDengfocM0CommissioningCandidate
                     .direction_minimum_movement_rad) {
                failure = "direction_mismatch";
                break;
            }
            if (((static_cast<float>(requestedDirection) * displacementRad) >=
                 kDengfocM0CommissioningCandidate
                     .direction_minimum_movement_rad) &&
                (directionEvidenceSamples >=
                 kDengfocM0CommissioningCandidate
                     .direction_minimum_evidence_samples)) {
                directionProven = true;
                break;
            }
        }

        if (static_cast<int32_t>(nowUs - nextControlUs) >= 0) {
            const uint32_t elapsedMs = millis() - startedMs;
            float voltage = kDengfocM0CommissioningCandidate
                                .direction_maximum_voltage_v;
            if (elapsedMs <
                kDengfocM0CommissioningCandidate.direction_ramp_ms) {
                const float fraction =
                    static_cast<float>(elapsedMs) /
                    static_cast<float>(
                        kDengfocM0CommissioningCandidate.direction_ramp_ms);
                voltage =
                    kDengfocM0CommissioningCandidate
                        .direction_start_voltage_v +
                    fraction *
                        (kDengfocM0CommissioningCandidate
                             .direction_maximum_voltage_v -
                         kDengfocM0CommissioningCandidate
                             .direction_start_voltage_v);
            }
            float electricalAngle = fmodf(
                static_cast<float>(
                    kDengfocM0CommissioningCandidate.pole_pairs) *
                        gFeedback.mechanical_position_rad -
                    kDengfocM0CommissioningCandidate
                        .electrical_offset_rad,
                kTwoPi);
            if (electricalAngle < 0.0F) {
                electricalAngle += kTwoPi;
            }
            const float commandedQVoltage =
                static_cast<float>(
                    requestedDirection *
                    kDengfocM0CommissioningCandidate
                        .commutation_direction) *
                voltage;
            if (!commitEncoderCommutatedVoltage(
                    commandedQVoltage, electricalAngle)) {
                failure = "pwm_commit";
                break;
            }
            lastCommandedQVoltageV = commandedQVoltage;
            lastElectricalAngleRad = electricalAngle;
            commandCount += 1U;
            nextControlUs += kControlPeriodUs;
        }
        delayMicroseconds(50U);
    }

    const bool shutdownOk = foc_dengfoc_power_stage_shutdown(&gPowerStage);
    forceDisabled();
    gPwmSequence = 0U;
    if (failure == nullptr) {
        if (maximumCurrentA < kMinimumEnergizedCurrentA) {
            failure = "not_energized";
        } else if (directionEvidenceSamples <
                   kDengfocM0CommissioningCandidate
                       .direction_minimum_evidence_samples) {
            failure = "insufficient_direction_evidence";
        } else if (!directionProven) {
            failure = "insufficient_motion";
        }
    }
    Serial.printf(
        "DIR_RESULT,ok=%u,reason=%s,requested=%ld,commutation_direction=%ld,"
        "offset_rad=%.9f,start_mech_rad=%.9f,final_mech_rad=%.9f,"
        "delta_mech_rad=%.9f,max_speed_rad_s=%.6f,max_current_a=%.6f,"
        "samples=%lu,direction_evidence_samples=%lu,"
        "direction_proven=%u,persisted=0\n",
        ((failure == nullptr) && shutdownOk) ? 1U : 0U,
        failure != nullptr ? failure : (shutdownOk ? "passed" : "shutdown"),
        static_cast<long>(requestedDirection),
        static_cast<long>(
            kDengfocM0CommissioningCandidate.commutation_direction),
        static_cast<double>(
            kDengfocM0CommissioningCandidate.electrical_offset_rad),
        static_cast<double>(initialMechanicalPositionRad),
        static_cast<double>(gFeedback.mechanical_position_rad),
        static_cast<double>(displacementRad),
        static_cast<double>(maximumSpeedRadS),
        static_cast<double>(maximumCurrentA),
        static_cast<unsigned long>(encoderSamples),
        static_cast<unsigned long>(directionEvidenceSamples),
        directionProven ? 1U : 0U);
    Serial.printf(
        "DIR_DIAG,revision=1,peak_valid=%u,peak_elapsed_ms=%lu,"
        "sample_seq=%lu,pwm_event_seq=%lu,sampled_pwm_event_seq=%lu,"
        "sample_delay_cycles=%lu,missed_pwm_window=%lu,"
        "raw_a=%lu,raw_b=%lu,ia=%.6f,ib=%.6f,ic=%.6f,"
        "peak_current_a=%.6f,commanded_q_voltage_v=%.6f,"
        "electrical_angle_rad=%.9f,duty=%lu/%lu/%lu,command_count=%lu,"
        "zero_a=%.3f,zero_b=%.3f,zero_noise_a=%.6f,zero_noise_b=%.6f\n",
        peakDiagnostic.valid ? 1U : 0U,
        static_cast<unsigned long>(peakDiagnostic.elapsed_ms),
        static_cast<unsigned long>(peakDiagnostic.sample_sequence),
        static_cast<unsigned long>(peakDiagnostic.pwm_event_sequence),
        static_cast<unsigned long>(
            peakDiagnostic.sampled_pwm_event_sequence),
        static_cast<unsigned long>(peakDiagnostic.sample_delay_cycles),
        static_cast<unsigned long>(peakDiagnostic.missed_pwm_window_count),
        static_cast<unsigned long>(peakDiagnostic.raw[0U]),
        static_cast<unsigned long>(peakDiagnostic.raw[1U]),
        static_cast<double>(peakDiagnostic.ia),
        static_cast<double>(peakDiagnostic.ib),
        static_cast<double>(peakDiagnostic.ic),
        static_cast<double>(peakDiagnostic.current),
        static_cast<double>(peakDiagnostic.commanded_q_voltage_v),
        static_cast<double>(peakDiagnostic.electrical_angle_rad),
        static_cast<unsigned long>(peakDiagnostic.duty_count[0U]),
        static_cast<unsigned long>(peakDiagnostic.duty_count[1U]),
        static_cast<unsigned long>(peakDiagnostic.duty_count[2U]),
        static_cast<unsigned long>(peakDiagnostic.command_count),
        static_cast<double>(gCurrentZero[0U].mean_raw_count),
        static_cast<double>(gCurrentZero[1U].mean_raw_count),
        static_cast<double>(gCurrentZero[0U].noise_rms_a),
        static_cast<double>(gCurrentZero[1U].noise_rms_a));
}

void handleCommand(const char *command) {
    if (strcmp(command, "STATUS") == 0) {
        printStatus();
        return;
    }
    if (strcmp(command, "PREPARE") == 0) {
        dengfoc_adc_stream_snapshot_t adc{};
        if (!gReady || (gPowerStage.state != FOC_DENGFOC_POWER_PWM_READY) ||
            !pollEncoder(micros()) ||
            !dengfoc_adc_stream_snapshot(&gAdcStream, &adc) ||
            (adc.sample_sequence == 0U) ||
            (adc.pool_overflow_count != 0U) ||
            (adc.read_error_count != 0U)) {
            forceDisabled();
            Serial.println("CAL_PREPARE,ok=0");
            return;
        }
        gPrepareToken = esp_random();
        if (gPrepareToken == 0U) {
            gPrepareToken = 1U;
        }
        gPrepareDeadlineMs = millis() + kPrepareWindowMs;
        gPreparedOperation = PreparedOperation::Alignment;
        Serial.printf("CAL_PREPARE,ok=1,token=%08lX,expires_ms=%lu\n",
                      static_cast<unsigned long>(gPrepareToken),
                      static_cast<unsigned long>(kPrepareWindowMs));
        return;
    }
    int32_t requestedDirection = 0;
    if (strcmp(command, "PREPARE_DIRECTION POSITIVE") == 0) {
        requestedDirection = 1;
    } else if (strcmp(command, "PREPARE_DIRECTION NEGATIVE") == 0) {
        requestedDirection = -1;
    }
    if (requestedDirection != 0) {
        dengfoc_adc_stream_snapshot_t adc{};
        if (!gReady || (gPowerStage.state != FOC_DENGFOC_POWER_PWM_READY) ||
            !pollEncoder(micros()) ||
            !dengfoc_adc_stream_snapshot(&gAdcStream, &adc) ||
            (adc.sample_sequence == 0U) ||
            (adc.pool_overflow_count != 0U) ||
            (adc.read_error_count != 0U)) {
            forceDisabled();
            Serial.println("DIR_PREPARE,ok=0");
            return;
        }
        gPrepareToken = esp_random();
        if (gPrepareToken == 0U) {
            gPrepareToken = 1U;
        }
        gPrepareDeadlineMs = millis() + kPrepareWindowMs;
        gPreparedOperation = requestedDirection > 0
                                 ? PreparedOperation::DirectionPositive
                                 : PreparedOperation::DirectionNegative;
        Serial.printf(
            "DIR_PREPARE,ok=1,requested=%ld,token=%08lX,expires_ms=%lu\n",
            static_cast<long>(requestedDirection),
            static_cast<unsigned long>(gPrepareToken),
            static_cast<unsigned long>(kPrepareWindowMs));
        return;
    }

    unsigned long suppliedToken = 0UL;
    if ((sscanf(command, "ARM %lx", &suppliedToken) == 1) &&
        (gPrepareToken != 0U) &&
        (static_cast<uint32_t>(suppliedToken) == gPrepareToken) &&
        (static_cast<int32_t>(gPrepareDeadlineMs - millis()) > 0)) {
        const PreparedOperation operation = gPreparedOperation;
        gPrepareToken = 0U;
        gPrepareDeadlineMs = 0U;
        gPreparedOperation = PreparedOperation::None;
        if (operation == PreparedOperation::Alignment) {
            runAlignment();
        } else if (operation == PreparedOperation::DirectionPositive) {
            runDirectionGate(1);
        } else if (operation == PreparedOperation::DirectionNegative) {
            runDirectionGate(-1);
        } else {
            forceDisabled();
            Serial.println("CAL_COMMAND,ok=0,reason=no_prepared_operation");
        }
        return;
    }
    forceDisabled();
    gPrepareToken = 0U;
    gPrepareDeadlineMs = 0U;
    gPreparedOperation = PreparedOperation::None;
    Serial.println(
        "CAL_COMMAND,ok=0,help=STATUS|PREPARE|PREPARE_DIRECTION-space-"
        "POSITIVE-or-NEGATIVE|ARM-space-token|X");
}

void serviceSerial() {
    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            return;
        }
        const char byte = static_cast<char>(value);
        if ((byte == '\r') || (byte == '\n')) {
            if (gCommandLength != 0U) {
                gCommand[gCommandLength] = '\0';
                handleCommand(gCommand);
                gCommandLength = 0U;
            }
        } else if (gCommandLength + 1U < sizeof(gCommand)) {
            gCommand[gCommandLength++] = byte;
        } else {
            gCommandLength = 0U;
            forceDisabled();
        }
    }
}

}  // namespace

void setup() {
    foc_dengfoc_power_ops_t powerOps{};
    const foc_dengfoc_power_policy_t policy = {
        .allow_actuation = 1U,
        .operation = FOC_DENGFOC_OPERATION_VOLTAGE_COMMISSIONING,
        .required_capabilities =
            FOC_DENGFOC_VOLTAGE_COMMISSIONING_REQUIRED_CAPABILITIES,
        .maximum_active_duty_deviation_count =
            kMaximumDutyDeviationCount,
    };

    const bool portReady = dengfoc_mcpwm_power_port_init(
        &gPowerPort,
        &foc_dengfoc_v04_reference_profile,
        0U,
        true,
        &powerOps);
    const bool stageReady = portReady && foc_dengfoc_power_stage_init(
        &gPowerStage,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &policy,
        &powerOps);
    forceDisabled();

    Serial.begin(kSerialBaud);
    delay(200U);
    Serial.println("FluxRT DengFOC bounded voltage commissioning");
    Serial.println("CAL_SAFE_DEFAULT,driver_disabled=1,auto_arm=0,persisted=0");

    const bool encoderReady = foc_dengfoc_as5600_port_init(
        &gEncoder,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &kI2cOps,
        &gI2cContext,
        1,
        kDengfocM0CommissioningCandidate.pole_pairs,
        1U,
        0.0F,
        0U,
        kEncoderMaximumPeriodUs);
    const bool zeroReady = stageReady && captureCurrentZero();
    const bool prepared = zeroReady &&
        foc_dengfoc_power_stage_prepare(&gPowerStage);
    const auto &axis = foc_dengfoc_v04_reference_profile.axis[0U];
    const bool adcReady = prepared && dengfoc_adc_stream_init(
        &gAdcStream,
        axis.current_a_adc_gpio,
        axis.current_b_adc_gpio,
        kAdcSampleFrequencyHz);
    if (adcReady) {
        dengfoc_mcpwm_power_port_set_cycle_event_callback(
            &gPowerPort, dengfoc_adc_stream_mark_pwm_event, &gAdcStream);
    }
    gReady = encoderReady && zeroReady && prepared && adcReady &&
             (gPowerStage.state == FOC_DENGFOC_POWER_PWM_READY) &&
             dengfoc_mcpwm_power_port_is_disabled(&gPowerPort);
    if (!gReady) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        forceDisabled();
    }
    Serial.printf(
        "CAL_BOOT,ready=%u,encoder=%u,zero=%u,pwm=%u,adc=%u,state=%u,"
        "align_max_voltage_v=%.3f,direction_max_voltage_v=%.3f,"
        "candidate_revision=%lu,candidate_offset_rad=%.9f,"
        "commutation_direction=%ld,direction_success_stop=1,"
        "direction_min_evidence=%lu,direction_min_increment_rad=%.9f,"
        "current_trip_a=%.3f,duty_deviation=%lu\n",
        gReady ? 1U : 0U,
        encoderReady ? 1U : 0U,
        zeroReady ? 1U : 0U,
        prepared ? 1U : 0U,
        adcReady ? 1U : 0U,
        static_cast<unsigned>(gPowerStage.state),
        static_cast<double>(kAlignmentMaximumVoltageV),
        static_cast<double>(
            kDengfocM0CommissioningCandidate.direction_maximum_voltage_v),
        static_cast<unsigned long>(
            kDengfocM0CommissioningCandidate.revision),
        static_cast<double>(
            kDengfocM0CommissioningCandidate.electrical_offset_rad),
        static_cast<long>(
            kDengfocM0CommissioningCandidate.commutation_direction),
        static_cast<unsigned long>(
            kDengfocM0CommissioningCandidate
                .direction_minimum_evidence_samples),
        static_cast<double>(
            kDengfocM0CommissioningCandidate
                .direction_minimum_increment_rad),
        static_cast<double>(kSoftwareCurrentTripA),
        static_cast<unsigned long>(kMaximumDutyDeviationCount));
    Serial.printf(
        "CAL_ZERO,diag_revision=1,a_mean=%.3f,a_min=%lu,a_max=%lu,"
        "a_noise_rms=%.6f,b_mean=%.3f,b_min=%lu,b_max=%lu,"
        "b_noise_rms=%.6f,amperes_per_count=%.9f\n",
        static_cast<double>(gCurrentZero[0U].mean_raw_count),
        static_cast<unsigned long>(gCurrentZero[0U].minimum_raw_count),
        static_cast<unsigned long>(gCurrentZero[0U].maximum_raw_count),
        static_cast<double>(gCurrentZero[0U].noise_rms_a),
        static_cast<double>(gCurrentZero[1U].mean_raw_count),
        static_cast<unsigned long>(gCurrentZero[1U].minimum_raw_count),
        static_cast<unsigned long>(gCurrentZero[1U].maximum_raw_count),
        static_cast<double>(gCurrentZero[1U].noise_rms_a),
        static_cast<double>(gCurrentZero[0U].amperes_per_count));
    Serial.println(
        "CAL_HELP,commands=STATUS|PREPARE|PREPARE_DIRECTION-space-"
        "POSITIVE-or-NEGATIVE|ARM-space-token,abort=X");
}

void loop() {
    if (!gReady) {
        forceDisabled();
        delay(100U);
        return;
    }
    if (!dengfoc_adc_stream_service(&gAdcStream)) {
        foc_dengfoc_power_stage_latch_fault(&gPowerStage);
        gReady = false;
        Serial.println("CAL_FATAL,reason=adc_service");
        return;
    }
    (void)pollEncoder(micros());
    serviceSerial();
    if ((gPrepareToken != 0U) &&
        (static_cast<int32_t>(millis() - gPrepareDeadlineMs) >= 0)) {
        const PreparedOperation expiredOperation = gPreparedOperation;
        gPrepareToken = 0U;
        gPrepareDeadlineMs = 0U;
        gPreparedOperation = PreparedOperation::None;
        forceDisabled();
        Serial.printf("CAL_PREPARE_EXPIRED,operation=%lu,driver_disabled=1\n",
                      static_cast<unsigned long>(expiredOperation));
    }
    if (gPowerStage.state != FOC_DENGFOC_POWER_PWM_READY) {
        forceDisabled();
    }
    delay(1U);
}
