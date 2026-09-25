#include <stdint.h>

#include "foc_build_profile.h"

#if defined(TEST_EXPECT_DIAGNOSTIC)
#if !defined(FLUXRT_DIAGNOSTIC_BUILD) || \
    !defined(FLUXRT_RUNTIME_TUNING_BUILD) || \
    !defined(FLUXRT_TRACE_BUILD) || \
    !defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    !defined(FLUXRT_MATH_DIAGNOSTICS_BUILD)
#error "Diagnostic capability contract changed"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (15UL)
#elif defined(TEST_EXPECT_CALIBRATION)
#if !defined(FLUXRT_CALIBRATION_BUILD) || \
    !defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD) || \
    !defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
#error "Calibration capture-only contract changed"
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD) || defined(FLUXRT_TRACE_BUILD) || \
    defined(FLUXRT_MATH_DIAGNOSTICS_BUILD)
#error "Calibration must not inherit Diagnostic capabilities"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (4UL)
#elif defined(TEST_EXPECT_PRODUCTION)
#if !defined(FLUXRT_PRODUCTION_BUILD)
#error "Production primary profile missing"
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD) || defined(FLUXRT_TRACE_BUILD) || \
    defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD) || \
    defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) || \
    defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD)
#error "Production must contain no diagnostic or calibration capability"
#endif
#define TEST_EXPECTED_CAPABILITY_MASK (0UL)
#else
#error "Test expectation is missing"
#endif

_Static_assert(FLUXRT_BUILD_CAPABILITY_MASK == TEST_EXPECTED_CAPABILITY_MASK,
               "build capability mask mismatch");

int main(void)
{
    return (FLUXRT_BUILD_CAPABILITY_MASK == TEST_EXPECTED_CAPABILITY_MASK) ? 0 : 1;
}
