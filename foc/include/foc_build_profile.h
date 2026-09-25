#ifndef FOC_BUILD_PROFILE_H
#define FOC_BUILD_PROFILE_H

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
     defined(FLUXRT_PRODUCTION_BUILD)) > 1
#error "FluxRT requires exactly one primary build profile"
#endif

#if !defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    !defined(FLUXRT_CALIBRATION_BUILD) && \
    !defined(FLUXRT_PRODUCTION_BUILD)
#define FLUXRT_DIAGNOSTIC_BUILD 1
#define FLUXRT_BUILD_PROFILE_IMPLICIT_DIAGNOSTIC 1
#endif

#define FLUXRT_BUILD_CAP_RUNTIME_TUNING       (1UL << 0)
#define FLUXRT_BUILD_CAP_REALTIME_TRACE        (1UL << 1)
#define FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE (1UL << 2)
#define FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS      (1UL << 3)

#if defined(FLUXRT_DIAGNOSTIC_BUILD)
#define FLUXRT_RUNTIME_TUNING_BUILD 1
#define FLUXRT_TRACE_BUILD 1
#define FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD 1
#define FLUXRT_MATH_DIAGNOSTICS_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_RUNTIME_TUNING | FLUXRT_BUILD_CAP_REALTIME_TRACE | \
     FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE | FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS)
#elif defined(FLUXRT_CALIBRATION_BUILD)
/* Calibration 是只采集档：允许停机相电压固定窗，但编译期禁止电机 arm。 */
#define FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD 1
#define FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE
#else
#define FLUXRT_BUILD_CAPABILITY_MASK (0UL)
#endif

#endif /* FOC_BUILD_PROFILE_H */
