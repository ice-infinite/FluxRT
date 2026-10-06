#ifndef FOC_PRODUCT_PROFILE_H
#define FOC_PRODUCT_PROFILE_H

/*
 * STM32G431RB product-composition contract.
 *
 * The primary build profile (Diagnostic/Calibration/Identification/Production)
 * controls operator and test capabilities.  This orthogonal profile controls
 * which large optional product feature may consume the G431RB's 128 KiB Flash.
 * A profile is an allow-list, not a runtime enable or motor-arm permission.
 */

#define FLUXRT_G431_PRODUCT_PROFILE_BASIC_DRIVE_ID (1UL)
#define FLUXRT_G431_PRODUCT_PROFILE_ADVANCED_LAB_ID (2UL)
#define FLUXRT_G431_PRODUCT_PROFILE_MOTION_LAB_ID (3UL)
#define FLUXRT_G431_PRODUCT_PROFILE_POWER_LAB_ID (4UL)
#define FLUXRT_G431_PRODUCT_PROFILE_CONNECTED_LAB_ID (5UL)

#if defined(SOC_STM32G431RB) || \
    defined(FLUXRT_G431_PROFILE_BASIC_DRIVE) || \
    defined(FLUXRT_G431_PROFILE_ADVANCED_LAB) || \
    defined(FLUXRT_G431_PROFILE_MOTION_LAB) || \
    defined(FLUXRT_G431_PROFILE_POWER_LAB) || \
    defined(FLUXRT_G431_PROFILE_CONNECTED_LAB)
#define FLUXRT_G431_PRODUCT_PROFILE_ENFORCED 1
#endif

#if (defined(FLUXRT_G431_PROFILE_BASIC_DRIVE) + \
     defined(FLUXRT_G431_PROFILE_ADVANCED_LAB) + \
     defined(FLUXRT_G431_PROFILE_MOTION_LAB) + \
     defined(FLUXRT_G431_PROFILE_POWER_LAB) + \
     defined(FLUXRT_G431_PROFILE_CONNECTED_LAB)) > 1
#error "STM32G431RB requires exactly one FluxRT product profile"
#endif

#if !defined(FLUXRT_G431_PROFILE_BASIC_DRIVE) && \
    !defined(FLUXRT_G431_PROFILE_ADVANCED_LAB) && \
    !defined(FLUXRT_G431_PROFILE_MOTION_LAB) && \
    !defined(FLUXRT_G431_PROFILE_POWER_LAB) && \
    !defined(FLUXRT_G431_PROFILE_CONNECTED_LAB)
#if defined(SOC_STM32G431RB)
#error "STM32G431RB product profile missing; regenerate rtconfig.h from Kconfig"
#else
/* Portable Host users that do not model this MCU retain the smallest envelope. */
#define FLUXRT_G431_PROFILE_BASIC_DRIVE 1
#define FLUXRT_G431_PRODUCT_PROFILE_IMPLICIT_BASIC 1
#endif
#endif

#if defined(FLUXRT_G431_PROFILE_BASIC_DRIVE)
#define FLUXRT_G431_PRODUCT_PROFILE_ID \
    FLUXRT_G431_PRODUCT_PROFILE_BASIC_DRIVE_ID
#define FLUXRT_G431_PRODUCT_PROFILE_NAME "basic-drive"
#elif defined(FLUXRT_G431_PROFILE_ADVANCED_LAB)
#define FLUXRT_G431_PRODUCT_PROFILE_ID \
    FLUXRT_G431_PRODUCT_PROFILE_ADVANCED_LAB_ID
#define FLUXRT_G431_PRODUCT_PROFILE_NAME "advanced-lab"
#elif defined(FLUXRT_G431_PROFILE_MOTION_LAB)
#define FLUXRT_G431_PRODUCT_PROFILE_ID \
    FLUXRT_G431_PRODUCT_PROFILE_MOTION_LAB_ID
#define FLUXRT_G431_PRODUCT_PROFILE_NAME "motion-lab"
#elif defined(FLUXRT_G431_PROFILE_POWER_LAB)
#define FLUXRT_G431_PRODUCT_PROFILE_ID \
    FLUXRT_G431_PRODUCT_PROFILE_POWER_LAB_ID
#define FLUXRT_G431_PRODUCT_PROFILE_NAME "power-lab"
#else
#define FLUXRT_G431_PRODUCT_PROFILE_ID \
    FLUXRT_G431_PRODUCT_PROFILE_CONNECTED_LAB_ID
#define FLUXRT_G431_PRODUCT_PROFILE_NAME "connected-lab"
#endif

/* Generic Host feature-mask tests do not model the G431's physical capacity.
 * Enforce the following rules only for that SoC or an explicit profile test. */
#if defined(FLUXRT_G431_PRODUCT_PROFILE_ENFORCED)

/* Every non-basic composition is a measurement image, never a release image. */
#if !defined(FLUXRT_G431_PROFILE_BASIC_DRIVE) && \
    !defined(FLUXRT_DIAGNOSTIC_BUILD)
#error "STM32G431RB lab product profiles require the Diagnostic build profile"
#endif

#if defined(FOC_ADVANCED_CONTROL_CANDIDATE) && \
    !defined(FLUXRT_G431_PROFILE_ADVANCED_LAB)
#error "Advanced FOC candidate requires the STM32G431RB Advanced Lab profile"
#endif

#if defined(FOC_SENSORLESS_CONTROL_CANDIDATE) && \
    !defined(FLUXRT_G431_PROFILE_ADVANCED_LAB)
#error "Sensorless FOC candidate requires the STM32G431RB Advanced Lab profile"
#endif

#if defined(FOC_H3_DYNAMIC_QUERY_CANDIDATE) && \
    !defined(FLUXRT_G431_PROFILE_ADVANCED_LAB)
#error "H3 dynamic query requires the STM32G431RB Advanced Lab profile"
#endif

#if defined(FOC_H3_DYNAMIC_QUERY_CANDIDATE) && \
    !defined(FOC_ADVANCED_CONTROL_CANDIDATE)
#error "H3 dynamic query requires the Advanced FOC candidate"
#endif

#if defined(FOC_AS5600_TRUTH_DIAGNOSTIC) && \
    !defined(FLUXRT_G431_PROFILE_BASIC_DRIVE)
#error "AS5600 truth diagnostic requires the STM32G431RB Basic Drive profile"
#endif

#if defined(FOC_AS5600_TRUTH_DIAGNOSTIC) && \
    !defined(FLUXRT_DIAGNOSTIC_BUILD)
#error "AS5600 truth diagnostic requires the Diagnostic build profile"
#endif

#if defined(FOC_AS5600_ALIGNMENT_CANDIDATE) && \
    !defined(FOC_AS5600_TRUTH_DIAGNOSTIC)
#error "AS5600 alignment candidate requires the AS5600 truth diagnostic"
#endif

#if defined(FOC_ADVANCED_CONTROL_CANDIDATE) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
#error "STM32G431RB Advanced Lab permits one large FOC candidate at a time"
#endif

#if defined(FOC_MOTION_CONTROL_CANDIDATE) && \
    !defined(FLUXRT_G431_PROFILE_MOTION_LAB)
#error "Motion candidate requires the STM32G431RB Motion Lab profile"
#endif

#if defined(FOC_POWER_MANAGEMENT_CANDIDATE) && \
    !defined(FLUXRT_G431_PROFILE_POWER_LAB)
#error "Power-management candidate requires the STM32G431RB Power Lab profile"
#endif

#if defined(FOC_EXTERNAL_IO_FRAMEWORK) && \
    !defined(FLUXRT_G431_PROFILE_CONNECTED_LAB)
#error "External I/O requires the STM32G431RB Connected Lab profile"
#endif

#if (defined(FLUXRT_PROTOCOL_NATIVE) || \
     defined(FLUXRT_INPUT_PWM_PULSE) || \
     defined(FLUXRT_INPUT_ANALOG) || \
     defined(FLUXRT_INPUT_STEP_DIR)) && \
    !defined(FOC_EXTERNAL_IO_FRAMEWORK)
#error "Protocol and input candidates require FOC_EXTERNAL_IO_FRAMEWORK"
#endif

/* Native may coexist with one input. Multiple physical-input candidates need
 * a separate capacity/resource proof and therefore fail closed on G431RB. */
#if (defined(FLUXRT_INPUT_PWM_PULSE) + \
     defined(FLUXRT_INPUT_ANALOG) + \
     defined(FLUXRT_INPUT_STEP_DIR)) > 1
#error "STM32G431RB Connected Lab permits at most one simple-input candidate"
#endif

#endif /* FLUXRT_G431_PRODUCT_PROFILE_ENFORCED */

#endif /* FOC_PRODUCT_PROFILE_H */
