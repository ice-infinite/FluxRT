cmake_minimum_required(VERSION 3.20)
project(fluxrt_host_tests C)

enable_testing()

add_executable(foc_platform_tests
    test_foc_platform.c
    ../../foc/platform/stm32g431/foc_platform_stm32g431.c
    ../../foc/platform/stm32g431/foc_math_accel_stm32g431.c
)
target_include_directories(foc_platform_tests PRIVATE ../../foc/include)
target_compile_features(foc_platform_tests PRIVATE c_std_11)

if(MSVC)
    target_compile_options(foc_platform_tests PRIVATE /W4)
else()
    target_compile_options(foc_platform_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME foc_platform_safe_empty_state COMMAND foc_platform_tests)
