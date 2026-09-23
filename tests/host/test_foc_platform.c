#include <assert.h>
#include <stdio.h>

#include "foc_platform.h"
#include "foc_math_accel.h"
#include "foc_rust_bridge.h"

int main(void)
{
    foc_feedback_t feedback = {0};
    foc_output_t output = {1.0f, 1.0f, 1.0f};
    foc_platform_diagnostics_t diagnostics = {0};
    float sin_value = 1.0f;
    float cos_value = 1.0f;
    float magnitude = 1.0f;

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

    puts("FOC C PLATFORM SAFE EMPTY STATE: PASS");
    return 0;
}
