include_guard(GLOBAL)

# Validate one G431RB Flash-composition envelope independently of Kconfig's UI.
# custom.cmake passes symbols read from the generated rtconfig.h; the standalone
# function is also exercised by tests/profile so stale/manual configurations
# fail before Cargo or the linker can produce a misleading image.
function(fluxrt_validate_g431_product_profile)
    cmake_parse_arguments(
        ARG
        ""
        "PROFILE;BUILD_PROFILE;ADVANCED;SENSORLESS;H3_DYNAMIC;AS5600_TRUTH;AS5600_ALIGNMENT;MOTION;POWER;EXTERNAL_IO;NATIVE;PWM_PULSE;ANALOG;STEP_DIR"
        ""
        ${ARGN})

    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "Unknown G431RB product-profile arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    set(valid_profiles
        basic-drive advanced-lab motion-lab power-lab connected-lab)
    list(FIND valid_profiles "${ARG_PROFILE}" profile_index)
    if(profile_index EQUAL -1)
        message(FATAL_ERROR "Unknown STM32G431RB product profile: ${ARG_PROFILE}")
    endif()

    if(NOT ARG_PROFILE STREQUAL "basic-drive" AND
       NOT ARG_BUILD_PROFILE STREQUAL "diagnostic")
        message(FATAL_ERROR
            "STM32G431RB ${ARG_PROFILE} requires FLUXRT_BUILD_PROFILE=diagnostic")
    endif()

    if(ARG_ADVANCED AND NOT ARG_PROFILE STREQUAL "advanced-lab")
        message(FATAL_ERROR "Advanced FOC candidate requires advanced-lab")
    endif()
    if(ARG_SENSORLESS AND NOT ARG_PROFILE STREQUAL "advanced-lab")
        message(FATAL_ERROR "Sensorless FOC candidate requires advanced-lab")
    endif()
    if(ARG_ADVANCED AND ARG_SENSORLESS)
        message(FATAL_ERROR
            "STM32G431RB Advanced Lab permits one large FOC candidate at a time")
    endif()
    if(ARG_H3_DYNAMIC AND NOT ARG_PROFILE STREQUAL "advanced-lab")
        message(FATAL_ERROR "H3 dynamic query requires advanced-lab")
    endif()
    if(ARG_H3_DYNAMIC AND NOT ARG_ADVANCED)
        message(FATAL_ERROR "H3 dynamic query requires the Advanced FOC candidate")
    endif()
    if(ARG_AS5600_TRUTH AND NOT ARG_PROFILE STREQUAL "basic-drive")
        message(FATAL_ERROR "AS5600 truth diagnostic requires basic-drive")
    endif()
    if(ARG_AS5600_TRUTH AND NOT ARG_BUILD_PROFILE STREQUAL "diagnostic")
        message(FATAL_ERROR "AS5600 truth diagnostic is Diagnostic-only")
    endif()
    if(ARG_AS5600_ALIGNMENT AND NOT ARG_AS5600_TRUTH)
        message(FATAL_ERROR
            "AS5600 alignment candidate requires the AS5600 truth diagnostic")
    endif()
    if(ARG_MOTION AND NOT ARG_PROFILE STREQUAL "motion-lab")
        message(FATAL_ERROR "Motion candidate requires motion-lab")
    endif()
    if(ARG_POWER AND NOT ARG_PROFILE STREQUAL "power-lab")
        message(FATAL_ERROR "Power-management candidate requires power-lab")
    endif()
    if(ARG_EXTERNAL_IO AND NOT ARG_PROFILE STREQUAL "connected-lab")
        message(FATAL_ERROR "External I/O requires connected-lab")
    endif()

    if((ARG_NATIVE OR ARG_PWM_PULSE OR ARG_ANALOG OR ARG_STEP_DIR) AND
       NOT ARG_EXTERNAL_IO)
        message(FATAL_ERROR
            "Native and simple-input candidates require FOC_EXTERNAL_IO_FRAMEWORK")
    endif()

    set(input_count 0)
    foreach(input_enabled IN ITEMS ARG_PWM_PULSE ARG_ANALOG ARG_STEP_DIR)
        if(${input_enabled})
            math(EXPR input_count "${input_count} + 1")
        endif()
    endforeach()
    if(input_count GREATER 1)
        message(FATAL_ERROR
            "STM32G431RB connected-lab permits at most one simple-input candidate")
    endif()

    set(FLUXRT_G431_VALIDATED_PRODUCT_PROFILE "${ARG_PROFILE}" PARENT_SCOPE)
endfunction()
