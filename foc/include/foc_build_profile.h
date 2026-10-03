#ifndef FOC_BUILD_PROFILE_H
#define FOC_BUILD_PROFILE_H

/* RT-Thread Kconfig symbols live in rtconfig.h, while the primary profile is
 * injected by CMake/SCons.  Pull the generated configuration in before deriving
 * capabilities so every target translation unit sees the same candidate
 * subprofile.  Host tests intentionally omit __RTTHREAD__ and inject only the
 * symbols under test. */
#if defined(__RTTHREAD__)
#include <rtconfig.h>
#endif

/*
 * FluxRT 构建档与能力契约。
 *
 * CMake 构建必须且只能定义一个主档位宏。直接 SCons/Host 编译若没有传入主档位，
 * 为兼容既有开发流程而默认使用 Diagnostic；一旦显式传入多个档位则硬失败。
 * 业务代码只检查下面派生的能力宏，不再用“非 Production”等价于 Diagnostic，
 * 从而保证 Calibration 不会意外带入在线调参、trace 或数学基准。
 */

#if (defined(FLUXRT_DIAGNOSTIC_BUILD) + \
     defined(FLUXRT_CALIBRATION_BUILD) + \
     defined(FLUXRT_IDENTIFICATION_BUILD) + \
     defined(FLUXRT_PRODUCTION_BUILD)) > 1
#error "FluxRT requires exactly one primary build profile"
#endif

#if !defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    !defined(FLUXRT_CALIBRATION_BUILD) && \
    !defined(FLUXRT_IDENTIFICATION_BUILD) && \
    !defined(FLUXRT_PRODUCTION_BUILD)
#define FLUXRT_DIAGNOSTIC_BUILD 1
#define FLUXRT_BUILD_PROFILE_IMPLICIT_DIAGNOSTIC 1
#endif

#define FLUXRT_BUILD_CAP_RUNTIME_TUNING       (1UL << 0)
#define FLUXRT_BUILD_CAP_REALTIME_TRACE        (1UL << 1)
#define FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE (1UL << 2)
#define FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS      (1UL << 3)
#define FLUXRT_BUILD_CAP_LSI_IDENTIFICATION     (1UL << 4)
#define FLUXRT_BUILD_CAP_MOTION_CANDIDATE       (1UL << 5)
#define FLUXRT_BUILD_CAP_ADVANCED_FOC_CANDIDATE (1UL << 6)
#define FLUXRT_BUILD_CAP_POWER_MANAGEMENT_CANDIDATE (1UL << 7)

#if defined(FLUXRT_DIAGNOSTIC_BUILD)
#if defined(FOC_ADVANCED_CONTROL_CANDIDATE)
/* Advanced FOC is compiled only into an explicitly selected Diagnostic
 * candidate.  Runtime features still default to zero and must pass the
 * stopped-state advanced configuration transaction before they can run. */
#define FLUXRT_ADVANCED_CANDIDATE_BUILD 1
#define FLUXRT_ADVANCED_CANDIDATE_CAPABILITY \
    FLUXRT_BUILD_CAP_ADVANCED_FOC_CANDIDATE
#else
#define FLUXRT_ADVANCED_CANDIDATE_CAPABILITY (0UL)
#endif
#if defined(FOC_POWER_MANAGEMENT_CANDIDATE)
/* This adds only a default-off management policy. Sensor/brake capabilities
 * are separate board evidence and are never inferred from this build bit. */
#define FLUXRT_POWER_CANDIDATE_BUILD 1
#define FLUXRT_POWER_CANDIDATE_CAPABILITY \
    FLUXRT_BUILD_CAP_POWER_MANAGEMENT_CANDIDATE
#else
#define FLUXRT_POWER_CANDIDATE_CAPABILITY (0UL)
#endif
#if defined(FOC_MOTION_CONTROL_CANDIDATE)
/* The target motion image is a dedicated, capacity-bounded Diagnostic
 * subprofile. It retains status/stop and the fixed candidate commands, but
 * removes generic tuning, trace, phase capture and math-benchmark surfaces. */
#define FLUXRT_MOTION_CANDIDATE_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_MOTION_CANDIDATE | \
     FLUXRT_ADVANCED_CANDIDATE_CAPABILITY | \
     FLUXRT_POWER_CANDIDATE_CAPABILITY)
#else
#define FLUXRT_RUNTIME_TUNING_BUILD 1
#define FLUXRT_TRACE_BUILD 1
#define FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD 1
#define FLUXRT_MATH_DIAGNOSTICS_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_RUNTIME_TUNING | FLUXRT_BUILD_CAP_REALTIME_TRACE | \
     FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE | FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS | \
     FLUXRT_ADVANCED_CANDIDATE_CAPABILITY | \
     FLUXRT_POWER_CANDIDATE_CAPABILITY)
#endif
#elif defined(FLUXRT_CALIBRATION_BUILD)
/* Calibration 是只采集档：允许停机相电压固定窗，但编译期禁止电机 arm。 */
#define FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD 1
#define FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD 1
#define FLUXRT_MOTOR_ARM_DISABLED_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE
#elif defined(FLUXRT_IDENTIFICATION_BUILD)
/* Identification 只开放 EXP-B3 状态机；普通 foc_start/平台 arm 仍编译期拒绝。 */
#define FLUXRT_LSI_IDENTIFICATION_BUILD 1
#define FLUXRT_MOTOR_ARM_DISABLED_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK FLUXRT_BUILD_CAP_LSI_IDENTIFICATION
#else
#define FLUXRT_BUILD_CAPABILITY_MASK (0UL)
#endif

#endif /* FOC_BUILD_PROFILE_H */
