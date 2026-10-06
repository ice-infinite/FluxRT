#include <Arduino.h>
#include <Wire.h>

#include <driver/rmt_rx.h>
#include <esp_timer.h>
#include <stdio.h>

extern "C" {
#include "foc_board_dengfoc_v04.h"
#include "foc_dengfoc_as5600_port.h"
#include "foc_time_sync.h"
#include "foc_time_sync_dengfoc.h"
#include "foc_time_sync_wire.h"
}

#if !defined(FLUXRT_DENGFOC_TIME_SYNC_RX) || \
    (FLUXRT_DENGFOC_TIME_SYNC_RX != 1)
#error "time_sync_main.cpp is only for the explicit time-sync RX target"
#endif

#if !defined(FLUXRT_DENGFOC_SYNC_INPUT_GPIO)
#error "The time-sync RX target requires an explicit input GPIO"
#endif

namespace {

constexpr uint32_t kSerialBaud = 921600U;
constexpr uint32_t kEnabledAxisMask = 0x01U;
constexpr uint32_t kSyncInputGpio = FLUXRT_DENGFOC_SYNC_INPUT_GPIO;
constexpr uint32_t kRmtResolutionHz = 1000000U;
constexpr size_t kRmtBufferSymbols = 192U;
constexpr uint16_t kGuardHighTicks = 10U;
/* Classic ESP32 applies the RX glitch filter on its 80 MHz source clock.
 * 1 us stays below the hardware threshold limit and remains well below the
 * shortest valid 10 us guard pulse used by this wire format. */
constexpr uint32_t kRmtGlitchFilterNs = 1000U;
/* One complete truth line is about 120 bytes.  At the former effective 500 Hz
 * cadence the CH340 link carried roughly 60 kB/s plus status/anchor traffic,
 * and one powered capture proved a mid-line byte loss.  A fixed 250 Hz truth
 * cadence still gives five independent AS5600 samples per 50 Hz FTR query,
 * while halving USB-UART load and keeping a single missed sample inside the
 * existing 10 ms formal quality gate. */
constexpr uint32_t kTruthSamplePeriodUs = 4000U;
constexpr uint32_t kStatusPeriodMs = 1000U;
constexpr size_t kTruthLineCapacity = 192U;

/* One physical guard pulse follows the 161 codec symbols.  It creates the
 * rising edge that closes the final data-low interval; its trailing low is
 * allowed to extend until the RMT idle threshold. */
static_assert(kRmtBufferSymbols > FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
              "RMT buffer must also hold the physical end guard");

struct ArduinoI2cContext {
    TwoWire *wire[FOC_DENGFOC_V04_AXIS_COUNT];
};

struct SyncTickContext {
    uint32_t low;
    uint32_t high;
};

ArduinoI2cContext gI2c{{&Wire, &Wire1}};
foc_dengfoc_as5600_port_t gEncoder{};
foc_time_sync_capture_t gCapture{};
foc_time_sync_adapter_t gAdapter{};
foc_time_sync_wire_decoder_t gDecoder{};
SyncTickContext gSyncTick{};
rmt_channel_handle_t gRmtChannel = nullptr;
rmt_symbol_word_t gRmtSymbols[kRmtBufferSymbols]{};
rmt_receive_config_t gReceiveConfig{};
portMUX_TYPE gSyncMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool gReceiveArmed = false;
volatile bool gSofLatched = false;
volatile bool gFrameReady = false;
volatile bool gFrameLastPart = false;
volatile size_t gReceivedSymbols = 0U;
uint32_t gTruthSequence = 0U;
uint32_t gTruthReadFailures = 0U;
uint32_t gTruthTransportFailures = 0U;
uint32_t gFrameCount = 0U;
uint32_t gFrameFailureCount = 0U;
uint32_t gCaptureRecoveryCount = 0U;
uint32_t gSessionRolloverCount = 0U;
uint32_t gIdleCompletionCount = 0U;
uint32_t gLastDecodeStage = 0U;
uint32_t gLastDecodeIndex = 0U;
uint32_t gLastTruthUs = 0U;
uint32_t gLastStatusMs = 0U;
bool gReady = false;

bool i2cBegin(void *opaque,
              uint32_t controller,
              uint32_t sdaGpio,
              uint32_t sclGpio,
              uint32_t frequencyHz) {
    auto *context = static_cast<ArduinoI2cContext *>(opaque);
    if ((context == nullptr) ||
        (controller >= FOC_DENGFOC_V04_AXIS_COUNT) ||
        (context->wire[controller] == nullptr)) {
        return false;
    }
    return context->wire[controller]->begin(
        static_cast<int>(sdaGpio),
        static_cast<int>(sclGpio),
        frequencyHz);
}

bool i2cReadRegister(void *opaque,
                     uint32_t controller,
                     uint8_t address,
                     uint8_t registerAddress,
                     uint8_t *data,
                     size_t length) {
    auto *context = static_cast<ArduinoI2cContext *>(opaque);
    if ((context == nullptr) || (data == nullptr) || (length == 0U) ||
        (controller >= FOC_DENGFOC_V04_AXIS_COUNT) ||
        (context->wire[controller] == nullptr)) {
        return false;
    }
    TwoWire *wire = context->wire[controller];
    wire->beginTransmission(address);
    if (wire->write(registerAddress) != 1U ||
        wire->endTransmission(false) != 0U ||
        wire->requestFrom(static_cast<int>(address),
                          static_cast<int>(length),
                          static_cast<int>(true)) !=
            static_cast<int>(length)) {
        return false;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (!wire->available()) {
            return false;
        }
        data[index] = static_cast<uint8_t>(wire->read());
    }
    return true;
}

uint32_t readLatchedLocalTick(void *opaque,
                              uint32_t *tickLow,
                              uint32_t *tickHigh) {
    auto *context = static_cast<SyncTickContext *>(opaque);
    if ((context == nullptr) || (tickLow == nullptr) || (tickHigh == nullptr)) {
        return 0U;
    }
    *tickLow = context->low;
    *tickHigh = context->high;
    return 1U;
}

void IRAM_ATTR onSyncRisingEdge(void *) {
    const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
    portENTER_CRITICAL_ISR(&gSyncMux);
    if (gReceiveArmed && !gSofLatched) {
        gSyncTick.low = static_cast<uint32_t>(nowUs);
        gSyncTick.high = static_cast<uint32_t>(nowUs >> 32U);
        gSofLatched = true;
    }
    portEXIT_CRITICAL_ISR(&gSyncMux);
}

bool IRAM_ATTR onRmtReceiveDone(rmt_channel_handle_t,
                                const rmt_rx_done_event_data_t *event,
                                void *) {
    portENTER_CRITICAL_ISR(&gSyncMux);
    if (event != nullptr) {
        gReceivedSymbols = event->num_symbols;
        gFrameLastPart = event->flags.is_last != 0U;
    } else {
        gReceivedSymbols = 0U;
        gFrameLastPart = false;
    }
    gReceiveArmed = false;
    gFrameReady = true;
    portEXIT_CRITICAL_ISR(&gSyncMux);
    return false;
}

bool armRmtReceiver() {
    portENTER_CRITICAL(&gSyncMux);
    gSofLatched = false;
    gFrameReady = false;
    gFrameLastPart = false;
    gReceivedSymbols = 0U;
    gReceiveArmed = true;
    portEXIT_CRITICAL(&gSyncMux);
    const esp_err_t receiveResult = rmt_receive(gRmtChannel,
                                                gRmtSymbols,
                                                sizeof(gRmtSymbols),
                                                &gReceiveConfig);
    if (receiveResult != ESP_OK) {
        Serial.printf("SYNC_RMT_ERROR,stage=receive,code=%ld,name=%s\n",
                      static_cast<long>(receiveResult),
                      esp_err_to_name(receiveResult));
        portENTER_CRITICAL(&gSyncMux);
        gReceiveArmed = false;
        portEXIT_CRITICAL(&gSyncMux);
        return false;
    }
    return true;
}

bool rmtSymbolToWire(const rmt_symbol_word_t &input,
                     foc_time_sync_wire_symbol_t *output) {
    if ((output == nullptr) || (input.level0 != 1U) ||
        (input.level1 != 0U) || (input.duration0 > UINT16_MAX) ||
        (input.duration1 > UINT16_MAX)) {
        return false;
    }
    output->high_ticks = static_cast<uint16_t>(input.duration0);
    output->low_ticks = static_cast<uint16_t>(input.duration1);
    return true;
}

bool endGuardValid(const rmt_symbol_word_t &guard) {
    /* On classic ESP32 the final low interval is written as duration1 == 0
     * when that level itself trips the configured RX idle threshold.  This
     * is the hardware end marker for a line that stayed low, not a zero-width
     * pulse.  Keep accepting an explicitly measured long low as well so the
     * same wire contract remains usable on newer RMT implementations. */
    return (guard.level0 == 1U) && (guard.level1 == 0U) &&
           (guard.duration0 >= (kGuardHighTicks - 3U)) &&
           (guard.duration0 <= (kGuardHighTicks + 3U)) &&
           ((guard.duration1 == 0U) || (guard.duration1 >= 20U));
}

void reportCaptureEvent() {
    foc_time_sync_event_t event{};
    while (foc_time_sync_capture_pop(&gCapture, &event) != 0U) {
        const uint64_t tick =
            (static_cast<uint64_t>(event.local_tick_high) << 32U) |
            static_cast<uint64_t>(event.local_tick_low);
        Serial.printf(
            "SYNC_ANCHOR_RX,session=%lu,edge_sequence=%lu,edge_tag=%lu,"
            "truth_tick_us=",
            static_cast<unsigned long>(event.session_id),
            static_cast<unsigned long>(event.edge_sequence),
            static_cast<unsigned long>(event.edge_tag));
        Serial.print(static_cast<unsigned long long>(tick));
        Serial.printf(
            ",source=%lu,flags=0x%08lx\n",
            static_cast<unsigned long>(event.source),
            static_cast<unsigned long>(event.flags));
    }
}

bool decodePendingFrame() {
    bool sofLatched;
    bool lastPart;
    size_t receivedSymbols;
    foc_time_sync_edge_identity_t identity{};
    foc_time_sync_status_t captureStatus{};

    portENTER_CRITICAL(&gSyncMux);
    sofLatched = gSofLatched;
    lastPart = gFrameLastPart;
    receivedSymbols = gReceivedSymbols;
    portEXIT_CRITICAL(&gSyncMux);

    gLastDecodeStage = 0U;
    gLastDecodeIndex = 0U;
    if (!sofLatched || !lastPart ||
        (receivedSymbols != (FOC_TIME_SYNC_WIRE_SYMBOL_COUNT + 1U))) {
        gLastDecodeStage = 1U;
        return false;
    }
    if (!endGuardValid(gRmtSymbols[FOC_TIME_SYNC_WIRE_SYMBOL_COUNT])) {
        gLastDecodeStage = 2U;
        return false;
    }
    foc_time_sync_wire_decoder_reset(&gDecoder);
    for (size_t index = 0U; index < FOC_TIME_SYNC_WIRE_SYMBOL_COUNT;
         ++index) {
        foc_time_sync_wire_symbol_t symbol{};
        if (!rmtSymbolToWire(gRmtSymbols[index], &symbol)) {
            gLastDecodeStage = 3U;
            gLastDecodeIndex = static_cast<uint32_t>(index);
            return false;
        }
        if (foc_time_sync_wire_decoder_feed(&gDecoder, &symbol) == 0U) {
            gLastDecodeStage = 4U;
            gLastDecodeIndex = static_cast<uint32_t>(index);
            return false;
        }
    }
    if (foc_time_sync_wire_decoder_finish(&gDecoder) == 0U) {
        gLastDecodeStage = 5U;
        return false;
    }
    if (foc_time_sync_wire_decoder_get_identity(&gDecoder, &identity) == 0U) {
        gLastDecodeStage = 6U;
        return false;
    }

    foc_time_sync_capture_get_status(&gCapture, &captureStatus);
    if (captureStatus.state == FOC_TIME_SYNC_CAPTURE_IDLE) {
        if (foc_time_sync_capture_arm(
                &gCapture,
                FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
                identity.session_id,
                identity.edge_sequence) == 0U) {
            gLastDecodeStage = 7U;
            return false;
        }
    } else if ((captureStatus.state == FOC_TIME_SYNC_CAPTURE_ARMED) &&
               (captureStatus.session_id != identity.session_id)) {
        /* One RMT frame is one independently CRC-verified wire transaction.
         * The target-neutral capture object deliberately remains ARMED after
         * its queue is drained so it can enforce monotonic sequences inside a
         * session.  A later Host run legitimately starts a new session,
         * however.  Close the old transaction explicitly only after its queue
         * has been fully consumed, then arm the new identity.  This keeps
         * same-session gaps/reordering fail-closed and avoids sacrificing the
         * first frame merely to recover from a stale completed session. */
        if (captureStatus.unread_count != 0U) {
            gLastDecodeStage = 7U;
            return false;
        }
        foc_time_sync_capture_stop(&gCapture);
        if (foc_time_sync_capture_arm(
                &gCapture,
                FOC_TIME_SYNC_SOURCE_TRUTH_SENSOR,
                identity.session_id,
                identity.edge_sequence) == 0U) {
            gLastDecodeStage = 7U;
            return false;
        }
        ++gSessionRolloverCount;
    }
    if (foc_time_sync_adapter_record_verified_edge_isr(
            &gAdapter, &identity) == 0U) {
        gLastDecodeStage = 8U;
        return false;
    }
    reportCaptureEvent();
    return true;
}

void reportRejectedFrame() {
    bool sofLatched;
    bool lastPart;
    size_t receivedSymbols;

    portENTER_CRITICAL(&gSyncMux);
    sofLatched = gSofLatched;
    lastPart = gFrameLastPart;
    receivedSymbols = gReceivedSymbols;
    portEXIT_CRITICAL(&gSyncMux);

    const size_t lastIndex =
        receivedSymbols > 0U && receivedSymbols <= kRmtBufferSymbols
            ? receivedSymbols - 1U
            : 0U;
    const size_t previousIndex =
        receivedSymbols > 1U && receivedSymbols <= kRmtBufferSymbols
            ? receivedSymbols - 2U
            : lastIndex;
    const rmt_symbol_word_t first = gRmtSymbols[0U];
    const rmt_symbol_word_t previous = gRmtSymbols[previousIndex];
    const rmt_symbol_word_t last = gRmtSymbols[lastIndex];
    Serial.printf(
        "SYNC_FRAME_REJECT,stage=%lu,index=%lu,symbols=%u,sof=%u,last=%u,"
        "first=%u:%u/%u:%u,prev=%u:%u/%u:%u,tail=%u:%u/%u:%u\n",
        static_cast<unsigned long>(gLastDecodeStage),
        static_cast<unsigned long>(gLastDecodeIndex),
        static_cast<unsigned int>(receivedSymbols),
        sofLatched ? 1U : 0U,
        lastPart ? 1U : 0U,
        static_cast<unsigned int>(first.level0),
        static_cast<unsigned int>(first.duration0),
        static_cast<unsigned int>(first.level1),
        static_cast<unsigned int>(first.duration1),
        static_cast<unsigned int>(previous.level0),
        static_cast<unsigned int>(previous.duration0),
        static_cast<unsigned int>(previous.level1),
        static_cast<unsigned int>(previous.duration1),
        static_cast<unsigned int>(last.level0),
        static_cast<unsigned int>(last.duration0),
        static_cast<unsigned int>(last.level1),
        static_cast<unsigned int>(last.duration1));
}

bool initializeRmt() {
    rmt_rx_channel_config_t channelConfig{};
    rmt_rx_event_callbacks_t callbacks{};
    channelConfig.gpio_num = static_cast<gpio_num_t>(kSyncInputGpio);
    channelConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    channelConfig.resolution_hz = kRmtResolutionHz;
    channelConfig.mem_block_symbols = kRmtBufferSymbols;
    channelConfig.intr_priority = 0;
    channelConfig.flags.invert_in = 0U;
    channelConfig.flags.with_dma = 0U;
    channelConfig.flags.io_loop_back = 0U;
    const esp_err_t createResult =
        rmt_new_rx_channel(&channelConfig, &gRmtChannel);
    if (createResult != ESP_OK) {
        Serial.printf("SYNC_RMT_ERROR,stage=create,code=%ld,name=%s\n",
                      static_cast<long>(createResult),
                      esp_err_to_name(createResult));
        return false;
    }
    callbacks.on_recv_done = onRmtReceiveDone;
    const esp_err_t callbackResult =
        rmt_rx_register_event_callbacks(gRmtChannel, &callbacks, nullptr);
    if (callbackResult != ESP_OK) {
        Serial.printf("SYNC_RMT_ERROR,stage=callbacks,code=%ld,name=%s\n",
                      static_cast<long>(callbackResult),
                      esp_err_to_name(callbackResult));
        return false;
    }
    const esp_err_t enableResult = rmt_enable(gRmtChannel);
    if (enableResult != ESP_OK) {
        Serial.printf("SYNC_RMT_ERROR,stage=enable,code=%ld,name=%s\n",
                      static_cast<long>(enableResult),
                      esp_err_to_name(enableResult));
        return false;
    }
    gReceiveConfig.signal_range_min_ns = kRmtGlitchFilterNs;
    gReceiveConfig.signal_range_max_ns = 500000U;
    gReceiveConfig.flags.en_partial_rx = 0U;
    attachInterruptArg(static_cast<uint8_t>(kSyncInputGpio),
                       onSyncRisingEdge,
                       nullptr,
                       RISING);
    return armRmtReceiver();
}

void configureSafePins() {
    const auto &board = foc_dengfoc_v04_reference_profile;
    digitalWrite(static_cast<uint8_t>(board.driver_enable_gpio), LOW);
    pinMode(static_cast<uint8_t>(board.driver_enable_gpio), OUTPUT);
    for (uint32_t axis = 0U; axis < FOC_DENGFOC_V04_AXIS_COUNT; ++axis) {
        pinMode(static_cast<uint8_t>(board.axis[axis].pwm_a_gpio), INPUT);
        pinMode(static_cast<uint8_t>(board.axis[axis].pwm_b_gpio), INPUT);
        pinMode(static_cast<uint8_t>(board.axis[axis].pwm_c_gpio), INPUT);
    }
    pinMode(static_cast<uint8_t>(kSyncInputGpio), INPUT);
}

bool appendUnsigned64Decimal(char *line,
                             size_t capacity,
                             size_t *length,
                             uint64_t value) {
    char reversed[20]{};
    size_t digitCount = 0U;
    do {
        reversed[digitCount++] = static_cast<char>('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (digitCount < sizeof(reversed)));
    if ((*length + digitCount) >= capacity) {
        return false;
    }
    while (digitCount > 0U) {
        line[(*length)++] = reversed[--digitCount];
    }
    line[*length] = '\0';
    return true;
}

void pollTruth() {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - gLastTruthUs) < kTruthSamplePeriodUs) {
        return;
    }
    gLastTruthUs = nowUs;
    foc_feedback_source_sample_t sample{};
    const uint64_t truthTickUs =
        static_cast<uint64_t>(esp_timer_get_time());
    if (!foc_dengfoc_as5600_port_poll(
            &gEncoder,
            millis(),
            nowUs,
            &sample)) {
        ++gTruthReadFailures;
        return;
    }
    ++gTruthSequence;
    char line[kTruthLineCapacity]{};
    const int prefixLength = snprintf(
        line,
        sizeof(line),
        "SYNC_TRUTH,sequence=%lu,truth_tick_us=",
        static_cast<unsigned long>(gTruthSequence));
    size_t length = prefixLength > 0 ? static_cast<size_t>(prefixLength) : 0U;
    const bool prefixValid =
        (prefixLength > 0) && (length < sizeof(line));
    const bool tickValid = prefixValid && appendUnsigned64Decimal(
        line, sizeof(line), &length, truthTickUs);
    const int suffixLength = tickValid
        ? snprintf(
              line + length,
              sizeof(line) - length,
              ",mechanical_angle_rad=%.9f,valid=%u,read_failures=%lu\n",
              static_cast<double>(sample.mechanical_position_rad),
              sample.valid_flags != 0U ? 1U : 0U,
              static_cast<unsigned long>(gTruthReadFailures))
        : -1;
    const bool suffixValid =
        (suffixLength > 0) &&
        (static_cast<size_t>(suffixLength) < (sizeof(line) - length));
    if (suffixValid) {
        length += static_cast<size_t>(suffixLength);
    }
    if (!suffixValid ||
        (Serial.write(
             reinterpret_cast<const uint8_t *>(line),
             length) != length)) {
        ++gTruthTransportFailures;
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    delay(200U);
    configureSafePins();
    Serial.println("FluxRT DengFOC USB-only H3 time-sync RX diagnostic");
    Serial.printf("sync_gpio=%lu,axis_mask=0x%02lx,driver_disabled=1\n",
                  static_cast<unsigned long>(kSyncInputGpio),
                  static_cast<unsigned long>(kEnabledAxisMask));

    const foc_dengfoc_i2c_ops_t i2cOps = {
        .begin = i2cBegin,
        .read_register = i2cReadRegister,
    };
    const foc_time_sync_wire_timing_t timing = {
        .struct_size = sizeof(foc_time_sync_wire_timing_t),
        .version = FOC_TIME_SYNC_WIRE_ABI_VERSION,
        .short_high_ticks = 20U,
        .long_high_ticks = 40U,
        .data_low_ticks = 20U,
        .sof_high_ticks = 80U,
        .sof_low_ticks = 40U,
        .tolerance_ticks = 5U,
    };

    const bool resourceReady =
        foc_time_sync_dengfoc_input_resource_valid(
            &foc_dengfoc_v04_reference_profile,
            kEnabledAxisMask,
            kSyncInputGpio) != 0U;
    const bool encoderReady = foc_dengfoc_as5600_port_init(
        &gEncoder,
        &foc_dengfoc_v04_reference_profile,
        0U,
        &i2cOps,
        &gI2c,
        1,
        7U,
        1U,
        0.0F,
        1U,
        5000U);
    foc_time_sync_capture_init(&gCapture);
    const bool adapterReady = foc_time_sync_dengfoc_adapter_init(
                                  &gAdapter,
                                  &gCapture,
                                  &gSyncTick,
                                  readLatchedLocalTick) != 0U;
    const bool decoderReady =
        foc_time_sync_wire_decoder_init(&gDecoder, &timing) != 0U;
    const bool rmtReady = resourceReady && initializeRmt();
    gReady = resourceReady && encoderReady && adapterReady && decoderReady &&
             rmtReady;
    Serial.printf(
        "SYNC_READY,resource=%u,encoder=%u,adapter=%u,decoder=%u,rmt=%u,"
        "ready=%u\n",
        resourceReady ? 1U : 0U,
        encoderReady ? 1U : 0U,
        adapterReady ? 1U : 0U,
        decoderReady ? 1U : 0U,
        rmtReady ? 1U : 0U,
        gReady ? 1U : 0U);
}

void loop() {
    digitalWrite(
        static_cast<uint8_t>(
            foc_dengfoc_v04_reference_profile.driver_enable_gpio),
        LOW);
    if (!gReady) {
        delay(10U);
        return;
    }
    pollTruth();

    bool frameReady;
    bool sofLatched;
    portENTER_CRITICAL(&gSyncMux);
    frameReady = gFrameReady;
    sofLatched = gSofLatched;
    portEXIT_CRITICAL(&gSyncMux);
    if (frameReady) {
        /* Classic ESP32 RMT may publish one zero-duration idle completion when
         * the channel is started or a host opens/resets the USB-UART. Without
         * a GPIO rising-edge SOF this is not a candidate wire frame: count it
         * explicitly and re-arm, but never poison an already valid identity
         * sequence. Any candidate with SOF still follows the strict decoder
         * and fail-closed capture path below. */
        if (!sofLatched) {
            ++gIdleCompletionCount;
        } else if (decodePendingFrame()) {
            ++gFrameCount;
        } else {
            ++gFrameFailureCount;
            reportRejectedFrame();
            foc_time_sync_capture_fail_isr(
                &gCapture, FOC_TIME_SYNC_FAILURE_BAD_EVENT);
            /* A malformed frame is permanently accounted by frame_fail and
             * never becomes an anchor.  Reset only the per-frame capture
             * transaction so one historic rejection cannot poison every
             * later independently CRC/identity-verified frame until reboot. */
            foc_time_sync_capture_init(&gCapture);
            ++gCaptureRecoveryCount;
        }
        if (!armRmtReceiver()) {
            gReady = false;
        }
    }

    const uint32_t nowMs = millis();
    if (static_cast<uint32_t>(nowMs - gLastStatusMs) >= kStatusPeriodMs) {
        foc_time_sync_status_t status{};
        gLastStatusMs = nowMs;
        foc_time_sync_capture_get_status(&gCapture, &status);
        Serial.printf(
            "SYNC_STATUS,frames=%lu,frame_fail=%lu,truth=%lu,truth_fail=%lu,"
            "truth_tx_fail=%lu,"
            "idle=%lu,capture_state=%lu,capture_failure=%lu,rejected=%lu,overflow=%lu,"
            "recovered=%lu,session_rollovers=%lu,"
            "sync_level=%u,driver_disabled=1\n",
            static_cast<unsigned long>(gFrameCount),
            static_cast<unsigned long>(gFrameFailureCount),
            static_cast<unsigned long>(gTruthSequence),
            static_cast<unsigned long>(gTruthReadFailures),
            static_cast<unsigned long>(gTruthTransportFailures),
            static_cast<unsigned long>(gIdleCompletionCount),
            static_cast<unsigned long>(status.state),
            static_cast<unsigned long>(status.failure_reason),
            static_cast<unsigned long>(status.rejected_count),
            static_cast<unsigned long>(status.overflow_count),
            static_cast<unsigned long>(gCaptureRecoveryCount),
            static_cast<unsigned long>(gSessionRolloverCount),
            digitalRead(static_cast<uint8_t>(kSyncInputGpio)) == HIGH ? 1U : 0U);
    }
    delay(1U);
}
