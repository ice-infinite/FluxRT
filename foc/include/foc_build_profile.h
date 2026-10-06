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

/* Validate the independent G431RB Flash-composition envelope before deriving
 * optional build capabilities from Kconfig candidates. */
#include "foc_product_profile.h"

#define FLUXRT_BUILD_CAP_RUNTIME_TUNING       (1UL << 0)
#define FLUXRT_BUILD_CAP_REALTIME_TRACE        (1UL << 1)
#define FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE (1UL << 2)
#define FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS      (1UL << 3)
#define FLUXRT_BUILD_CAP_LSI_IDENTIFICATION     (1UL << 4)
#define FLUXRT_BUILD_CAP_MOTION_CANDIDATE       (1UL << 5)
#define FLUXRT_BUILD_CAP_ADVANCED_FOC_CANDIDATE (1UL << 6)
#define FLUXRT_BUILD_CAP_POWER_MANAGEMENT_CANDIDATE (1UL << 7)
#define FLUXRT_BUILD_CAP_SENSORLESS_FOC_CANDIDATE (1UL << 8)
#define FLUXRT_BUILD_CAP_H3_DYNAMIC_QUERY       (1UL << 9)
#define FLUXRT_BUILD_CAP_AS5600_TRUTH_DIAGNOSTIC (1UL << 10)
#define FLUXRT_BUILD_CAP_AS5600_ALIGNMENT_CANDIDATE (1UL << 11)

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
#if defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
/* The full-speed sensorless target image is a separate no-power timing lab.
 * It cannot coexist with the larger advanced supervisor, has no generic motor
 * arm authority, and the real board capability provider remains zero. */
#define FLUXRT_SENSORLESS_CANDIDATE_BUILD 1
#define FLUXRT_SENSORLESS_CANDIDATE_CAPABILITY \
    FLUXRT_BUILD_CAP_SENSORLESS_FOC_CANDIDATE
#else
#define FLUXRT_SENSORLESS_CANDIDATE_CAPABILITY (0UL)
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
#elif defined(FOC_AS5600_TRUTH_DIAGNOSTIC)
/* Direct encoder-truth bring-up keeps normal motor arm compiled out. The
 * optional alignment subprofile grants only its dedicated fixed-envelope
 * authority; I2C still runs exclusively from bounded Shell commands. */
#define FLUXRT_AS5600_TRUTH_BUILD 1
#define FLUXRT_MOTOR_ARM_DISABLED_BUILD 1
#if defined(FOC_AS5600_ALIGNMENT_CANDIDATE)
#define FLUXRT_AS5600_ALIGNMENT_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_AS5600_TRUTH_DIAGNOSTIC | \
     FLUXRT_BUILD_CAP_AS5600_ALIGNMENT_CANDIDATE)
#else
#define FLUXRT_BUILD_CAPABILITY_MASK \
    FLUXRT_BUILD_CAP_AS5600_TRUTH_DIAGNOSTIC
#endif
#elif defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
/* S11 measures only the sensorless ABI in the real ADC IRQ with Gate/MOE/CCER
 * closed. Remove generic tuning/trace surfaces and compile-time disable every
 * motor-arm path so the measurement image cannot become a drive image. */
#define FLUXRT_MOTOR_ARM_DISABLED_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    FLUXRT_SENSORLESS_CANDIDATE_CAPABILITY
#elif defined(FOC_H3_DYNAMIC_QUERY_CANDIDATE)
/* The H3 powered query reuses the already bounded Advanced trial owner.  Keep
 * only realtime trace and the verified PB6/PB7 synchronisation path so the
 * evidence image fits the G431RB Flash envelope. */
#define FLUXRT_H3_TIME_SYNC_BUILD 1
#define FLUXRT_H3_DYNAMIC_QUERY_BUILD 1
#define FLUXRT_H3_EDGE_CONTROL_TICK_BUILD 1
#define FLUXRT_TRACE_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_REALTIME_TRACE | \
     FLUXRT_ADVANCED_CANDIDATE_CAPABILITY | \
     FLUXRT_BUILD_CAP_H3_DYNAMIC_QUERY)
#else
#define FLUXRT_RUNTIME_TUNING_BUILD 1
#define FLUXRT_TRACE_BUILD 1
#define FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD 1
#define FLUXRT_MATH_DIAGNOSTICS_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK \
    (FLUXRT_BUILD_CAP_RUNTIME_TUNING | FLUXRT_BUILD_CAP_REALTIME_TRACE | \
     FLUXRT_BUILD_CAP_PHASE_VOLTAGE_CAPTURE | FLUXRT_BUILD_CAP_MATH_DIAGNOSTICS | \
     FLUXRT_ADVANCED_CANDIDATE_CAPABILITY | \
     FLUXRT_SENSORLESS_CANDIDATE_CAPABILITY | \
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
#define FLUXRT_H3_TIME_SYNC_BUILD 1
#define FLUXRT_MOTOR_ARM_DISABLED_BUILD 1
#define FLUXRT_BUILD_CAPABILITY_MASK FLUXRT_BUILD_CAP_LSI_IDENTIFICATION
#else
#define FLUXRT_BUILD_CAPABILITY_MASK (0UL)
#endif

#endif /* FOC_BUILD_PROFILE_H */
