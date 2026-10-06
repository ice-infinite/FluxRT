#include "dengfoc_adc_stream.h"

#include <esp_attr.h>
#include <esp_cpu.h>
#include <hal/adc_types.h>
#include <soc/soc_caps.h>

#include <cstring>

namespace {

constexpr uint32_t kChannelCount = 2U;
constexpr uint32_t kFrameBytes =
    SOC_ADC_DIGI_RESULT_BYTES * kChannelCount;
constexpr uint32_t kPoolBytes = 8192U;
constexpr uint32_t kServiceReadBytes = 1024U;

bool IRAM_ATTR onConversionDone(adc_continuous_handle_t,
                                const adc_continuous_evt_data_t *event,
                                void *opaque) {
    auto *stream = static_cast<dengfoc_adc_stream_t *>(opaque);
    if ((stream == nullptr) || (event == nullptr) ||
        (event->conv_frame_buffer == nullptr) ||
        (event->size < kFrameBytes)) {
        return false;
    }

    uint32_t raw[kChannelCount] = {0U, 0U};
    uint32_t seen = 0U;
    for (uint32_t offset = 0U;
         offset + SOC_ADC_DIGI_RESULT_BYTES <= event->size;
         offset += SOC_ADC_DIGI_RESULT_BYTES) {
        const auto *result = reinterpret_cast<const adc_digi_output_data_t *>(
            event->conv_frame_buffer + offset);
        for (uint32_t channel = 0U; channel < kChannelCount; ++channel) {
            if (result->type1.channel == stream->channel[channel]) {
                raw[channel] = result->type1.data;
                seen |= 1UL << channel;
            }
        }
    }
    if (seen != ((1UL << kChannelCount) - 1UL)) {
        stream->read_error_count += 1U;
        return false;
    }

    const uint32_t pwmSequence = stream->pwm_event_sequence;
    if (pwmSequence == stream->sampled_pwm_event_sequence) {
        return false;
    }
    if ((stream->sampled_pwm_event_sequence != 0U) &&
        (pwmSequence - stream->sampled_pwm_event_sequence > 1U)) {
        stream->missed_pwm_window_count +=
            pwmSequence - stream->sampled_pwm_event_sequence - 1U;
    }
    const uint32_t now = esp_cpu_get_cycle_count();
    stream->raw[0] = raw[0];
    stream->raw[1] = raw[1];
    stream->sample_cpu_cycle = now;
    stream->sample_delay_cycles = now - stream->pwm_event_cpu_cycle;
    stream->sampled_pwm_event_sequence = pwmSequence;
    stream->sample_sequence += 1U;
    return false;
}

bool IRAM_ATTR onPoolOverflow(adc_continuous_handle_t,
                              const adc_continuous_evt_data_t *,
                              void *opaque) {
    auto *stream = static_cast<dengfoc_adc_stream_t *>(opaque);
    if (stream != nullptr) {
        stream->pool_overflow_count += 1U;
    }
    return false;
}

}  // namespace

bool dengfoc_adc_stream_init(dengfoc_adc_stream_t *stream,
                             uint32_t currentAGpio,
                             uint32_t currentBGpio,
                             uint32_t sampleFrequencyHz) {
    if ((stream == nullptr) || (sampleFrequencyHz == 0U)) {
        return false;
    }
    *stream = {};

    adc_unit_t unit[kChannelCount]{};
    const int gpio[kChannelCount] = {
        static_cast<int>(currentAGpio),
        static_cast<int>(currentBGpio),
    };
    for (uint32_t index = 0U; index < kChannelCount; ++index) {
        if ((adc_continuous_io_to_channel(
                 gpio[index], &unit[index], &stream->channel[index]) != ESP_OK) ||
            (unit[index] != ADC_UNIT_1)) {
            return false;
        }
    }

    const adc_continuous_handle_cfg_t handleConfig = {
        .max_store_buf_size = kPoolBytes,
        .conv_frame_size = kFrameBytes,
        .flags = {.flush_pool = 1U},
    };
    if (adc_continuous_new_handle(&handleConfig, &stream->handle) != ESP_OK) {
        return false;
    }

    adc_digi_pattern_config_t pattern[kChannelCount]{};
    for (uint32_t index = 0U; index < kChannelCount; ++index) {
        pattern[index].atten = ADC_ATTEN_DB_12;
        pattern[index].channel = stream->channel[index];
        pattern[index].unit = ADC_UNIT_1;
        pattern[index].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    const adc_continuous_config_t adcConfig = {
        .pattern_num = kChannelCount,
        .adc_pattern = pattern,
        .sample_freq_hz = sampleFrequencyHz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    const adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = onConversionDone,
        .on_pool_ovf = onPoolOverflow,
    };
    if ((adc_continuous_config(stream->handle, &adcConfig) != ESP_OK) ||
        (adc_continuous_register_event_callbacks(
             stream->handle, &callbacks, stream) != ESP_OK) ||
        (adc_continuous_start(stream->handle) != ESP_OK)) {
        return false;
    }
    stream->started = 1U;
    return true;
}

void dengfoc_adc_stream_mark_pwm_event(void *context,
                                      uint32_t eventCount,
                                      uint32_t cpuCycle) {
    auto *stream = static_cast<dengfoc_adc_stream_t *>(context);
    if (stream == nullptr) {
        return;
    }
    stream->pwm_event_cpu_cycle = cpuCycle;
    stream->pwm_event_sequence = eventCount;
}

bool dengfoc_adc_stream_service(dengfoc_adc_stream_t *stream) {
    if ((stream == nullptr) || (stream->handle == nullptr) ||
        (stream->started == 0U)) {
        return false;
    }
    uint8_t buffer[kServiceReadBytes];
    while (true) {
        uint32_t received = 0U;
        const esp_err_t result = adc_continuous_read(
            stream->handle, buffer, sizeof(buffer), &received, 0U);
        if (result == ESP_ERR_TIMEOUT) {
            return true;
        }
        if (result != ESP_OK) {
            stream->read_error_count += 1U;
            return false;
        }
    }
}

bool dengfoc_adc_stream_snapshot(
    const dengfoc_adc_stream_t *stream,
    dengfoc_adc_stream_snapshot_t *snapshot) {
    if ((stream == nullptr) || (snapshot == nullptr)) {
        return false;
    }
    *snapshot = {};
    snapshot->pwm_event_sequence = stream->pwm_event_sequence;
    snapshot->sample_sequence = stream->sample_sequence;
    snapshot->sampled_pwm_event_sequence = stream->sampled_pwm_event_sequence;
    snapshot->raw[0] = stream->raw[0];
    snapshot->raw[1] = stream->raw[1];
    snapshot->sample_delay_cycles = stream->sample_delay_cycles;
    snapshot->pool_overflow_count = stream->pool_overflow_count;
    snapshot->read_error_count = stream->read_error_count;
    snapshot->missed_pwm_window_count = stream->missed_pwm_window_count;
    snapshot->started = stream->started;
    return true;
}
