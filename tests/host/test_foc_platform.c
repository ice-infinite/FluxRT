#include <assert.h>
#include <stdio.h>

#include "foc_platform.h"
#include "foc_math_accel.h"
#include "foc_rust_bridge.h"

static void test_correlated_realtime_timing(void)
{
    foc_realtime_timing_stats_t stats;
    foc_realtime_timing_sample_t sample = {1U, 100U, 20U, 60U, 20U, 0U, 0U};

    foc_realtime_timing_reset(&stats);
    assert(stats.struct_size == sizeof(stats));
    assert(stats.version == FOC_REALTIME_TIMING_VERSION);
    assert(foc_realtime_timing_record(&stats, &sample) == 1U);
    assert(stats.sample_count == 1U);
    assert(stats.wcet.step == 1U);
    assert(stats.wcet.total_cycles == 100U);
    assert(stats.wcet.precontrol_cycles == 20U);
    assert(stats.wcet.control_cycles == 60U);
    assert(stats.wcet.postcontrol_cycles == 20U);

    sample.step = 2U;
    sample.total_cycles = 90U;
    sample.precontrol_cycles = 30U;
    sample.control_cycles = 50U;
    sample.postcontrol_cycles = 10U;
    sample.trace_enabled = 1U;
    sample.trace_sampled = 1U;
    assert(foc_realtime_timing_record(&stats, &sample) == 0U);
    assert(stats.wcet.step == 1U);
    assert(stats.peak_precontrol_cycles == 30U);
    assert(stats.peak_control_cycles == 60U);
    assert(stats.peak_postcontrol_cycles == 20U);

    sample.step = 3U;
    sample.total_cycles = 120U;
    sample.precontrol_cycles = 25U;
    sample.control_cycles = 70U;
    sample.postcontrol_cycles = 25U;
    sample.trace_enabled = 1U;
    sample.trace_sampled = 0U;
    assert(foc_realtime_timing_record(&stats, &sample) == 1U);
    assert(stats.wcet.step == 3U);
    assert(stats.wcet.trace_enabled == 1U);
    assert(stats.wcet.trace_sampled == 0U);

    sample.postcontrol_cycles = 24U;
    assert(foc_realtime_timing_record(&stats, &sample) == 0U);
    assert(stats.invalid_sample_count == 1U);
    assert(stats.sample_count == 3U);
}

int main(void)
{
    foc_feedback_t feedback = {0};
    foc_output_t output = {1.0f, 1.0f, 1.0f};
    foc_platform_diagnostics_t diagnostics = {0};
    foc_realtime_timing_stats_t timing = {0};
    float sin_value = 1.0f;
    float cos_value = 1.0f;
    float magnitude = 1.0f;

    test_correlated_realtime_timing();

    assert(FOC_RUST_ABI_VERSION == 0x00070000UL);
    assert(sizeof(foc_rust_context_t) == FOC_RUST_CONTEXT_CAPACITY);
    assert(sizeof(foc_runtime_config_t) == 228U);
    assert(sizeof(foc_telemetry_t) == 60U);
    assert(foc_math_accel_backend() == FOC_MATH_BACKEND_CPU);
    assert(foc_math_accel_sin_cos(0.0f, &sin_value, &cos_value) == 0U);
    assert(sin_value == 0.0f && cos_value == 0.0f);
    assert(foc_math_accel_magnitude(3.0f, 4.0f, &magnitude) == 0U);
    assert(magnitude == 0.0f);

    foc_platform_emergency_stop();
    assert(foc_platform_init() == FOC_STATUS_NOT_CONFIGURED);
    assert(foc_platform_read_feedback(&feedback) == FOC_STATUS_NOT_CONFIGURED);
    assert(feedback.phase_current_a == 0.0f);
    assert(feedback.phase_current_b == 0.0f);
    assert(feedback.phase_current_c == 0.0f);
    assert(feedback.dc_bus_voltage == 0.0f);
    assert(foc_platform_trial_arm() == FOC_STATUS_NOT_CONFIGURED);
    assert(foc_platform_apply_output(&output) == FOC_STATUS_NOT_CONFIGURED);
    foc_platform_trial_disarm();
    assert(foc_platform_get_diagnostics(&diagnostics) == FOC_STATUS_OK);
    assert(diagnostics.flags == 0U);
    assert(diagnostics.pwm_frequency_hz == 0U);
    assert(foc_platform_get_timing(&timing) == FOC_STATUS_OK);
    assert(timing.struct_size == sizeof(timing));
    assert(timing.version == FOC_REALTIME_TIMING_VERSION);
    assert(timing.sample_count == 0U);

    puts("FOC C PLATFORM SAFE EMPTY STATE: PASS");
    return 0;
}
