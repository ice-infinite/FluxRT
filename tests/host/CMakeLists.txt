cmake_minimum_required(VERSION 3.20)
project(fluxrt_host_tests C)

enable_testing()

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

foreach(profile_name IN ITEMS diagnostic calibration production)
    add_executable(foc_build_profile_${profile_name}_test
        test_foc_build_profile.c)
    target_include_directories(foc_build_profile_${profile_name}_test PRIVATE
        ../../foc/include)
    target_compile_features(foc_build_profile_${profile_name}_test PRIVATE c_std_11)
    string(TOUPPER "${profile_name}" profile_upper)
    target_compile_definitions(foc_build_profile_${profile_name}_test PRIVATE
        FLUXRT_${profile_upper}_BUILD=1
        TEST_EXPECT_${profile_upper}=1)
    add_test(NAME foc_build_profile_${profile_name}
        COMMAND foc_build_profile_${profile_name}_test)
endforeach()
