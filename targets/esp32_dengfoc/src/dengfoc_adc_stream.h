#ifndef DENGFOC_ADC_STREAM_H
#define DENGFOC_ADC_STREAM_H

#include <esp_adc/adc_continuous.h>
#include <stdint.h>

typedef struct
{
    adc_continuous_handle_t handle;
    adc_channel_t channel[2];
    volatile uint32_t pwm_event_sequence;
    volatile uint32_t pwm_event_cpu_cycle;
    volatile uint32_t sample_sequence;
    volatile uint32_t sampled_pwm_event_sequence;
    volatile uint32_t raw[2];
    volatile uint32_t sample_cpu_cycle;
    volatile uint32_t sample_delay_cycles;
    volatile uint32_t pool_overflow_count;
    volatile uint32_t read_error_count;
    volatile uint32_t missed_pwm_window_count;
    uint32_t started;
} dengfoc_adc_stream_t;

typedef struct
{
    uint32_t pwm_event_sequence;
    uint32_t sample_sequence;
    uint32_t sampled_pwm_event_sequence;
    uint32_t raw[2];
    uint32_t sample_delay_cycles;
    uint32_t pool_overflow_count;
    uint32_t read_error_count;
    uint32_t missed_pwm_window_count;
    uint32_t started;
} dengfoc_adc_stream_snapshot_t;

bool dengfoc_adc_stream_init(dengfoc_adc_stream_t *stream,
                             uint32_t current_a_gpio,
                             uint32_t current_b_gpio,
                             uint32_t sample_frequency_hz);

void IRAM_ATTR dengfoc_adc_stream_mark_pwm_event(void *context,
                                                 uint32_t event_count,
                                                 uint32_t cpu_cycle);

bool dengfoc_adc_stream_service(dengfoc_adc_stream_t *stream);

bool dengfoc_adc_stream_snapshot(
    const dengfoc_adc_stream_t *stream,
    dengfoc_adc_stream_snapshot_t *snapshot);

#endif /* DENGFOC_ADC_STREAM_H */
