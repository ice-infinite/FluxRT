cmake_minimum_required(VERSION 3.20)
project(fluxrt_host_tests C)

enable_testing()

add_executable(foc_power_safety_tests
    test_foc_power_safety.c
    ../../foc/runtime/foc_power_safety.c
)
target_include_directories(foc_power_safety_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_power_safety_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_power_safety_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_power_safety_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_power_safety_transaction COMMAND foc_power_safety_tests)

add_executable(foc_product_contract_tests
    test_foc_product_contract.c
)
target_include_directories(foc_product_contract_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_product_contract_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_product_contract_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_product_contract_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_product_contract_tests PRIVATE m)
endif()
add_test(NAME foc_product_contract_layout COMMAND foc_product_contract_tests)

add_executable(foc_platform_tests
    test_foc_platform.c
    ../../applications/foc_production_profile.c
    ../../foc/runtime/foc_phase_voltage_capture.c
    ../../foc/runtime/foc_phase_voltage_model.c
    ../../foc/runtime/foc_pwm_timing.c
    ../../foc/runtime/foc_realtime_timing.c
    ../../foc/platform/stm32g431/foc_platform_stm32g431.c
    ../../foc/platform/stm32g431/foc_math_accel_stm32g431.c
)
target_include_directories(foc_platform_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_platform_tests PRIVATE c_std_11)
target_compile_definitions(foc_platform_tests PRIVATE
    FOC_MATH_DIAGNOSTIC_BENCHMARK=1
    FOC_MATH_DIAGNOSTIC_HEALTH=1
)

if(MSVC)
    target_compile_options(foc_platform_tests PRIVATE /W4)
else()
    target_compile_options(foc_platform_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME foc_platform_safe_empty_state COMMAND foc_platform_tests)

add_executable(foc_phase_voltage_quality_tests
    test_foc_phase_voltage_quality.c
    ../../foc/runtime/foc_phase_voltage_quality.c
)
target_include_directories(foc_phase_voltage_quality_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_phase_voltage_quality_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_phase_voltage_quality_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_phase_voltage_quality_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_phase_voltage_quality_fallback
    COMMAND foc_phase_voltage_quality_tests)

add_executable(foc_phase_voltage_platform_adapter_tests
    test_foc_phase_voltage_platform_adapter.c
    ../../foc/platform/stm32g431/foc_phase_voltage_platform_adapter.c
    ../../foc/runtime/foc_phase_voltage_quality.c
    ../../foc/runtime/foc_phase_voltage_model.c
)
target_include_directories(foc_phase_voltage_platform_adapter_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_phase_voltage_platform_adapter_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_phase_voltage_platform_adapter_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_phase_voltage_platform_adapter_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_phase_voltage_platform_uncalibrated_lockout
    COMMAND foc_phase_voltage_platform_adapter_tests)

add_executable(foc_phase_voltage_realtime_input_tests
    test_foc_phase_voltage_realtime_input.c
    ../../foc/runtime/foc_phase_voltage_realtime_input.c
)
target_include_directories(foc_phase_voltage_realtime_input_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_phase_voltage_realtime_input_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_phase_voltage_realtime_input_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_phase_voltage_realtime_input_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_phase_voltage_v19_assembly_lockout
    COMMAND foc_phase_voltage_realtime_input_tests)

add_executable(foc_shell_parse_tests
    test_foc_shell_parse.c
    ../../applications/foc_shell_parse.c
)
target_include_directories(foc_shell_parse_tests PRIVATE
    ../../applications
)
target_compile_features(foc_shell_parse_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_shell_parse_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_shell_parse_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_shell_strict_decimal_parser
    COMMAND foc_shell_parse_tests)

add_executable(foc_lsi_identification_tests
    test_foc_lsi_identification.c
    ../../applications/foc_lsi_identification.c
)
target_include_directories(foc_lsi_identification_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_lsi_identification_tests PRIVATE c_std_11)
target_compile_definitions(foc_lsi_identification_tests PRIVATE
    FLUXRT_IDENTIFICATION_BUILD=1)
if(MSVC)
    target_compile_options(foc_lsi_identification_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_identification_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_identification_state_machine
    COMMAND foc_lsi_identification_tests)

add_executable(foc_lsi_management_tests
    test_foc_lsi_management.c
    ../../applications/foc_lsi_management.c
    ../../applications/foc_lsi_identification.c
)
target_include_directories(foc_lsi_management_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_lsi_management_tests PRIVATE c_std_11)
target_compile_definitions(foc_lsi_management_tests PRIVATE
    FLUXRT_IDENTIFICATION_BUILD=1)
if(MSVC)
    target_compile_options(foc_lsi_management_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_management_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_management_safe_recovery
    COMMAND foc_lsi_management_tests)

add_executable(foc_lsi_raw_capture_tests
    test_foc_lsi_raw_capture.c
    ../../foc/runtime/foc_lsi_raw_capture.c
)
target_include_directories(foc_lsi_raw_capture_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_lsi_raw_capture_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_lsi_raw_capture_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_raw_capture_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_raw_capture_fixed_window
    COMMAND foc_lsi_raw_capture_tests)

add_executable(foc_lsi_sample_contract_tests
    test_foc_lsi_sample_contract.c
    ../../foc/runtime/foc_lsi_raw_capture.c
    ../../foc/runtime/foc_lsi_sample_contract.c
)
target_include_directories(foc_lsi_sample_contract_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_lsi_sample_contract_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_lsi_sample_contract_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_sample_contract_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_sample_contract_fake_platform
    COMMAND foc_lsi_sample_contract_tests)

add_executable(foc_lsi_capture_service_tests
    test_foc_lsi_capture_service.c
    ../../foc/runtime/foc_lsi_capture_service.c
    ../../foc/runtime/foc_lsi_raw_capture.c
    ../../foc/runtime/foc_lsi_sample_contract.c
)
target_include_directories(foc_lsi_capture_service_tests PRIVATE
    ../../foc/include)
target_compile_features(foc_lsi_capture_service_tests PRIVATE c_std_11)
target_compile_definitions(foc_lsi_capture_service_tests PRIVATE
    FLUXRT_IDENTIFICATION_BUILD=1)
if(MSVC)
    target_compile_options(foc_lsi_capture_service_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_capture_service_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_capture_service_fixed_window
    COMMAND foc_lsi_capture_service_tests)

add_executable(foc_lsi_actuation_executor_tests
    test_foc_lsi_actuation_executor.c
    ../../foc/runtime/foc_lsi_actuation_executor.c
)
target_include_directories(foc_lsi_actuation_executor_tests PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_lsi_actuation_executor_tests PRIVATE c_std_11)
target_compile_definitions(foc_lsi_actuation_executor_tests PRIVATE
    FLUXRT_IDENTIFICATION_BUILD=1)
if(MSVC)
    target_compile_options(foc_lsi_actuation_executor_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_actuation_executor_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_actuation_execution_contract
    COMMAND foc_lsi_actuation_executor_tests)

add_executable(foc_lsi_preload_sink_tests
    test_foc_lsi_preload_sink.c
    ../../foc/platform/stm32g431/foc_lsi_preload_sink.c
)
target_include_directories(foc_lsi_preload_sink_tests PRIVATE
    ../../applications
    ../../foc/include
    ../../foc/platform/stm32g431)
target_compile_features(foc_lsi_preload_sink_tests PRIVATE c_std_11)
target_compile_definitions(foc_lsi_preload_sink_tests PRIVATE
    FLUXRT_IDENTIFICATION_BUILD=1)
if(MSVC)
    target_compile_options(foc_lsi_preload_sink_tests PRIVATE /W4)
else()
    target_compile_options(foc_lsi_preload_sink_tests PRIVATE
        -Wall -Wextra -Wpedantic)
endif()
add_test(NAME foc_lsi_private_preload_sink
    COMMAND foc_lsi_preload_sink_tests)

foreach(profile_name IN ITEMS calibration identification)
    add_executable(foc_motor_arm_${profile_name}_gate_test
        test_foc_motor_arm_gate.c
        ../../foc/runtime/foc_phase_voltage_capture.c
        ../../foc/runtime/foc_phase_voltage_model.c
        ../../foc/runtime/foc_pwm_timing.c
        ../../foc/runtime/foc_realtime_timing.c
        ../../foc/platform/stm32g431/foc_platform_stm32g431.c
        ../../foc/platform/stm32g431/foc_math_accel_stm32g431.c)
    target_include_directories(foc_motor_arm_${profile_name}_gate_test PRIVATE
        ../../foc/include)
    target_compile_features(foc_motor_arm_${profile_name}_gate_test PRIVATE c_std_11)
    string(TOUPPER "${profile_name}" profile_upper)
    target_compile_definitions(foc_motor_arm_${profile_name}_gate_test PRIVATE
        FLUXRT_${profile_upper}_BUILD=1)
    if(MSVC)
        target_compile_options(foc_motor_arm_${profile_name}_gate_test PRIVATE /W4)
    else()
        target_compile_options(foc_motor_arm_${profile_name}_gate_test PRIVATE
            -Wall -Wextra -Wpedantic)
    endif()
    add_test(NAME foc_motor_arm_${profile_name}_refused
        COMMAND foc_motor_arm_${profile_name}_gate_test)
endforeach()

foreach(profile_name IN ITEMS diagnostic calibration identification production)
    add_executable(foc_build_profile_${profile_name}_test
        test_foc_build_profile.c
        ../../applications/foc_lsi_identification.c)
    target_include_directories(foc_build_profile_${profile_name}_test PRIVATE
        ../../applications
        ../../foc/include)
    target_compile_features(foc_build_profile_${profile_name}_test PRIVATE c_std_11)
    string(TOUPPER "${profile_name}" profile_upper)
    target_compile_definitions(foc_build_profile_${profile_name}_test PRIVATE
        FLUXRT_${profile_upper}_BUILD=1
        TEST_EXPECT_${profile_upper}=1)
    add_test(NAME foc_build_profile_${profile_name}
        COMMAND foc_build_profile_${profile_name}_test)
endforeach()
