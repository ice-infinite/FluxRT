#include <stdint.h>

#include "foc_build_profile.h"
#include "foc_lsi_identification.h"

#if defined(TEST_EXPECT_DIAGNOSTIC)
#if !defined(FLUXRT_DIAGNOSTIC_BUILD) || \
    !defined(FLUXRT_RUNTIME_TUNING_BUILD) || \
    !defined(FLUXRT_TRACE_BUILD) || \
    !defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    !defined(FLUXRT_MATH_DIAGNOSTICS_BUILD)
#error "Diagnostic capability contract changed"
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD) || \
    defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
#error "Diagnostic must not enable identification or disable normal arm"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (15UL)
#define TEST_EXPECTED_LSI_AUTHORIZED (0UL)
#elif defined(TEST_EXPECT_CALIBRATION)
#if !defined(FLUXRT_CALIBRATION_BUILD) || \
    !defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD) || \
    !defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    !defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
#error "Calibration capture-only contract changed"
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD) || defined(FLUXRT_TRACE_BUILD) || \
    defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) || \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
#error "Calibration must not inherit Diagnostic capabilities"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (4UL)
#define TEST_EXPECTED_LSI_AUTHORIZED (0UL)
#elif defined(TEST_EXPECT_IDENTIFICATION)
#if !defined(FLUXRT_IDENTIFICATION_BUILD) || \
    !defined(FLUXRT_LSI_IDENTIFICATION_BUILD) || \
    !defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
#error "Identification capability contract changed"
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD) || defined(FLUXRT_TRACE_BUILD) || \
    defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) || \
    defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD)
#error "Identification must contain only its dedicated capability"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (16UL)
#define TEST_EXPECTED_LSI_AUTHORIZED (1UL)
#elif defined(TEST_EXPECT_PRODUCTION)
#if !defined(FLUXRT_PRODUCTION_BUILD)
#error "Production primary profile missing"
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD) || defined(FLUXRT_TRACE_BUILD) || \
    defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) || \
    defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD) || \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD) || \
    defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
#error "Production must contain no diagnostic or calibration capability"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (0UL)
#define TEST_EXPECTED_LSI_AUTHORIZED (0UL)
#else
#error "Test expectation is missing"
#endif

_Static_assert(FLUXRT_BUILD_CAPABILITY_MASK == TEST_EXPECTED_CAPABILITY_MASK,
               "build capability mask mismatch");

int main(void)
{
    foc_lsi_config_t config = {0};
    foc_lsi_context_t context;
    foc_lsi_output_t output;
    uint32_t started;

    if ((FLUXRT_BUILD_CAPABILITY_MASK != TEST_EXPECTED_CAPABILITY_MASK) ||
        (foc_lsi_build_is_authorized() != TEST_EXPECTED_LSI_AUTHORIZED))
    {
        return 1;
    }
    config.struct_size = sizeof(config);
    config.version = FOC_LSI_IDENTIFICATION_VERSION;
    config.sample_rate_hz = FOC_LSI_SAMPLE_RATE_HZ;
    config.offset_sample_count = 2U;
    config.bias_settle_ticks = 2U;
    config.pulse_ticks_per_polarity = 2U;
    config.pulse_pair_count = 2U;
    config.cooldown_zero_ticks = 2U;
    config.max_active_ticks = 20U;
    config.total_timeout_ticks = 40U;
    config.bias_current_a = 0.2f;
    config.perturbation_voltage_v = 0.4f;
    config.current_trip_a = 1.15f;
    config.cooldown_current_threshold_a = 0.05f;
    config.bus_voltage_min_v = 7.0f;
    config.bus_voltage_max_v = 18.0f;
    if (foc_lsi_init(&context, &config) == 0U)
    {
        return 2;
    }
    started = foc_lsi_request_start(&context, FOC_LSI_START_CONFIRMATION);
    if (started != TEST_EXPECTED_LSI_AUTHORIZED)
    {
        return 3;
    }
    foc_lsi_get_output(&context, &output);
    if (TEST_EXPECTED_LSI_AUTHORIZED != 0U)
    {
        return ((output.state == FOC_LSI_STATE_PREFLIGHT) &&
                (output.force_safe_output != 0U) &&
                (output.drive_request == FOC_LSI_DRIVE_OFF)) ? 0 : 4;
    }
    return ((output.state == FOC_LSI_STATE_IDLE) &&
            (output.force_safe_output != 0U) &&
            (output.drive_request == FOC_LSI_DRIVE_OFF)) ? 0 : 5;
}
