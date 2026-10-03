cmake_minimum_required(VERSION 3.20)
project(fluxrt_host_tests C)

enable_testing()

add_executable(foc_monitor_sampling_tests
    test_foc_monitor_sampling.c
    ../../foc/runtime/foc_monitor_sampling.c
)
target_include_directories(foc_monitor_sampling_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_monitor_sampling_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_monitor_sampling_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_monitor_sampling_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_monitor_sampling_adc1_ownership_matrix
    COMMAND foc_monitor_sampling_tests)

add_executable(foc_external_io_tests
    test_foc_external_io.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_external_io_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_external_io_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_io_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_io_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_io_capability_config_and_conflicts
    COMMAND foc_external_io_tests)

add_executable(foc_external_io_build_features_tests
    test_foc_external_io_build_features.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_external_io_build_features_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_external_io_build_features_tests PRIVATE c_std_11)
target_compile_definitions(foc_external_io_build_features_tests PRIVATE
    FLUXRT_TRANSPORT_UART=1
    FLUXRT_TRANSPORT_CAN_CLASSIC=1
    FLUXRT_PROTOCOL_NATIVE=1
    FLUXRT_PROTOCOL_DRONECAN=1
    FLUXRT_INPUT_PWM_PULSE=1
    FLUXRT_INPUT_ANALOG=1
    FLUXRT_INPUT_STEP_DIR=1
)
if(MSVC)
    target_compile_options(foc_external_io_build_features_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_io_build_features_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_io_compile_feature_masks
    COMMAND foc_external_io_build_features_tests)

add_executable(foc_input_bridge_layout_tests
    test_foc_input_bridge_layout.c
)
target_include_directories(foc_input_bridge_layout_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_input_bridge_layout_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_input_bridge_layout_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_input_bridge_layout_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_input_bridge_fixed_layout
    COMMAND foc_input_bridge_layout_tests)

add_executable(foc_simple_input_adapter_tests
    test_foc_simple_input_adapter.c
    ../../applications/foc_simple_input_adapter.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_simple_input_adapter_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_simple_input_adapter_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_simple_input_adapter_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_simple_input_adapter_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_simple_input_adapter_tests PRIVATE m)
endif()
add_test(NAME foc_simple_input_setpoint_only_adapter
    COMMAND foc_simple_input_adapter_tests)

add_executable(foc_transport_service_tests
    test_foc_transport_service.c
    ../../applications/foc_transport_service.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_transport_service_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_transport_service_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_transport_service_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_transport_service_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_transport_service_bounded_spsc_and_health
    COMMAND foc_transport_service_tests)

add_executable(foc_native_bridge_layout_tests
    test_foc_native_bridge_layout.c
)
target_include_directories(foc_native_bridge_layout_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_native_bridge_layout_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_native_bridge_layout_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_native_bridge_layout_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_native_protocol_c_abi_layout
    COMMAND foc_native_bridge_layout_tests)

add_executable(foc_input_service_tests
    test_foc_input_service.c
    ../../applications/foc_input_service.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_input_service_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_input_service_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_input_service_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_input_service_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_input_service_tests PRIVATE m)
endif()
add_test(NAME foc_input_service_timeout_and_source_gate
    COMMAND foc_input_service_tests)

add_executable(foc_command_service_tests
    test_foc_command_service.c
    ../../applications/foc_command_service.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_command_service_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_command_service_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_command_service_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_command_service_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_command_service_single_gateway
    COMMAND foc_command_service_tests)

add_executable(foc_command_rust_adapter_tests
    test_foc_command_rust_adapter.c
    ../../applications/foc_command_rust_adapter.c
)
target_include_directories(foc_command_rust_adapter_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_command_rust_adapter_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_command_rust_adapter_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_command_rust_adapter_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_command_rust_adapter_bridge
    COMMAND foc_command_rust_adapter_tests)

add_executable(foc_external_command_owner_tests
    test_foc_external_command_owner.c
    ../../applications/foc_external_command_owner.c
    ../../applications/foc_command_rust_adapter.c
)
target_include_directories(foc_external_command_owner_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_external_command_owner_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_command_owner_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_command_owner_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_command_owner_read_only_poll
    COMMAND foc_external_command_owner_tests)

add_executable(foc_external_io_management_tests
    test_foc_external_io_management.c
    ../../applications/foc_external_io_management.c
    ../../applications/foc_external_command_owner.c
    ../../applications/foc_command_rust_adapter.c
    ../../applications/foc_command_service.c
    ../../applications/foc_input_service.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_external_io_management_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_external_io_management_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_io_management_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_io_management_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_io_management_default_off_and_poll
    COMMAND foc_external_io_management_tests)

add_executable(foc_external_io_management_header_isolation_tests
    test_foc_external_io_management_header_isolation.c
)
target_include_directories(foc_external_io_management_header_isolation_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_external_io_management_header_isolation_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_io_management_header_isolation_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_io_management_header_isolation_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_io_management_header_has_no_platform_dependency
    COMMAND foc_external_io_management_header_isolation_tests)

add_executable(foc_external_io_board_tests
    test_foc_external_io_board.c
    ../../foc/platform/stm32g431/foc_external_io_board.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_external_io_board_tests PRIVATE
    ../../foc/include
    ../../foc/platform/stm32g431
)
target_compile_features(foc_external_io_board_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_io_board_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_io_board_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_io_stm32g431_resource_table
    COMMAND foc_external_io_board_tests)

add_executable(foc_external_input_port_tests
    test_foc_external_input_port.c
    ../../foc/runtime/foc_external_input_port.c
)
target_include_directories(foc_external_input_port_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_external_input_port_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_external_input_port_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_input_port_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_input_target_neutral_raw_port
    COMMAND foc_external_input_port_tests)

add_executable(foc_external_input_stm32g431_tests
    test_foc_external_input_stm32g431.c
    ../../foc/runtime/foc_external_input_port.c
    ../../foc/platform/stm32g431/foc_external_input_stm32g431.c
)
target_include_directories(foc_external_input_stm32g431_tests PRIVATE
    ../../foc/include
    ../../foc/platform/stm32g431
)
target_compile_features(foc_external_input_stm32g431_tests PRIVATE c_std_11)
target_compile_definitions(foc_external_input_stm32g431_tests PRIVATE
    FLUXRT_INPUT_ANALOG=1)
if(MSVC)
    target_compile_options(foc_external_input_stm32g431_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_input_stm32g431_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_external_input_stm32g431_fake_register_pipeline
    COMMAND foc_external_input_stm32g431_tests)

add_executable(foc_external_input_owner_tests
    test_foc_external_input_owner.c
    ../../applications/foc_external_input_owner.c
    ../../applications/foc_external_io_management.c
    ../../applications/foc_external_command_owner.c
    ../../applications/foc_command_rust_adapter.c
    ../../applications/foc_command_service.c
    ../../applications/foc_input_service.c
    ../../applications/foc_simple_input_adapter.c
    ../../foc/runtime/foc_external_input_port.c
    ../../foc/runtime/foc_external_io.c
)
target_include_directories(foc_external_input_owner_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_external_input_owner_tests PRIVATE c_std_11)
target_compile_definitions(foc_external_input_owner_tests PRIVATE
    FLUXRT_INPUT_ANALOG=1)
if(MSVC)
    target_compile_options(foc_external_input_owner_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_external_input_owner_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_external_input_owner_tests PRIVATE m)
endif()
add_test(NAME foc_external_input_owner_lifecycle_and_fail_closed
    COMMAND foc_external_input_owner_tests)

add_executable(foc_feedback_management_tests
    test_foc_feedback_management.c
    ../../applications/foc_feedback_management.c
)
target_include_directories(foc_feedback_management_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_feedback_management_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_feedback_management_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_feedback_management_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_feedback_management_tests PRIVATE m)
endif()
add_test(NAME foc_feedback_management_session_and_timeout
    COMMAND foc_feedback_management_tests)

add_executable(foc_feedback_adapter_tests
    test_foc_feedback_adapter.c
    ../../foc/runtime/foc_feedback_adapter.c
)
target_include_directories(foc_feedback_adapter_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_feedback_adapter_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_feedback_adapter_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_feedback_adapter_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_feedback_adapter_tests PRIVATE m)
endif()
add_test(NAME foc_feedback_adapter_normalization
    COMMAND foc_feedback_adapter_tests)

add_executable(foc_config_apply_executor_tests
    test_foc_config_apply_executor.c
    ../../foc/runtime/foc_config_apply_executor.c
)
target_include_directories(foc_config_apply_executor_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_config_apply_executor_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_config_apply_executor_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_config_apply_executor_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_config_apply_order_and_fail_closed
    COMMAND foc_config_apply_executor_tests)

add_executable(foc_motion_adapter_tests
    test_foc_motion_adapter.c
    ../../foc/runtime/foc_motion_adapter.c
)
target_include_directories(foc_motion_adapter_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_motion_adapter_tests PRIVATE c_std_11)
target_compile_definitions(foc_motion_adapter_tests PRIVATE
    FLUXRT_DIAGNOSTIC_BUILD=1)
if(MSVC)
    target_compile_options(foc_motion_adapter_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_motion_adapter_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_motion_adapter_default_off_and_fail_safe
    COMMAND foc_motion_adapter_tests)

add_executable(foc_motion_management_tests
    test_foc_motion_management.c
    ../../applications/foc_motion_management.c
)
target_include_directories(foc_motion_management_tests PRIVATE
    ../../applications
    ../../foc/include
)
target_compile_features(foc_motion_management_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_motion_management_tests PRIVATE /W4 /WX)
else()
    target_compile_options(foc_motion_management_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_motion_management_owner_timeout_and_fail_closed
    COMMAND foc_motion_management_tests)

add_executable(foc_motion_dispatcher_tests
    test_foc_motion_dispatcher.c
    ../../foc/runtime/foc_motion_dispatcher.c
)
target_include_directories(foc_motion_dispatcher_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_motion_dispatcher_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_motion_dispatcher_tests PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(foc_motion_dispatcher_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
if(NOT MSVC)
    target_link_libraries(foc_motion_dispatcher_tests PRIVATE m)
endif()
add_test(NAME foc_motion_dispatcher_atomic_publication_and_urgent_requests
    COMMAND foc_motion_dispatcher_tests)

add_executable(foc_motion_probe_tests
    test_foc_motion_probe.c
    ../../foc/runtime/foc_motion_probe.c
)
target_include_directories(foc_motion_probe_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_motion_probe_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_motion_probe_tests PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(foc_motion_probe_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_motion_probe_no_power_and_fault_epoch_guard
    COMMAND foc_motion_probe_tests)

add_executable(foc_motion_torque_trial_tests
    test_foc_motion_torque_trial.c
    ../../foc/runtime/foc_motion_torque_trial.c
)
target_include_directories(foc_motion_torque_trial_tests PRIVATE
    ../../foc/include
)
target_compile_features(foc_motion_torque_trial_tests PRIVATE c_std_11)
if(MSVC)
    target_compile_options(foc_motion_torque_trial_tests PRIVATE /W4 /WX /utf-8)
else()
    target_compile_options(foc_motion_torque_trial_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Werror)
endif()
add_test(NAME foc_motion_torque_trial_bounded_fail_closed
    COMMAND foc_motion_torque_trial_tests)

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

add_executable(foc_build_profile_motion_candidate_test
    test_foc_build_profile.c
    ../../applications/foc_lsi_identification.c)
target_include_directories(foc_build_profile_motion_candidate_test PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_build_profile_motion_candidate_test PRIVATE c_std_11)
target_compile_definitions(foc_build_profile_motion_candidate_test PRIVATE
    FLUXRT_DIAGNOSTIC_BUILD=1
    FOC_MOTION_CONTROL_CANDIDATE=1
    TEST_EXPECT_MOTION_CANDIDATE=1)
add_test(NAME foc_build_profile_motion_candidate
    COMMAND foc_build_profile_motion_candidate_test)

add_executable(foc_build_profile_advanced_candidate_test
    test_foc_build_profile.c
    ../../applications/foc_lsi_identification.c)
target_include_directories(foc_build_profile_advanced_candidate_test PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_build_profile_advanced_candidate_test PRIVATE c_std_11)
target_compile_definitions(foc_build_profile_advanced_candidate_test PRIVATE
    FLUXRT_DIAGNOSTIC_BUILD=1
    FOC_ADVANCED_CONTROL_CANDIDATE=1
    TEST_EXPECT_ADVANCED_CANDIDATE=1)
add_test(NAME foc_build_profile_advanced_candidate
    COMMAND foc_build_profile_advanced_candidate_test)

add_executable(foc_build_profile_power_candidate_test
    test_foc_build_profile.c
    ../../applications/foc_lsi_identification.c)
target_include_directories(foc_build_profile_power_candidate_test PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_build_profile_power_candidate_test PRIVATE c_std_11)
target_compile_definitions(foc_build_profile_power_candidate_test PRIVATE
    FLUXRT_DIAGNOSTIC_BUILD=1
    FOC_POWER_MANAGEMENT_CANDIDATE=1
    TEST_EXPECT_POWER_CANDIDATE=1)
add_test(NAME foc_build_profile_power_candidate
    COMMAND foc_build_profile_power_candidate_test)

add_executable(foc_power_management_test
    test_foc_power_management.c
    ../../applications/foc_power_management.c)
target_include_directories(foc_power_management_test PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_power_management_test PRIVATE c_std_11)
add_test(NAME foc_power_management COMMAND foc_power_management_test)

add_executable(foc_native_fake_driver_test
    test_foc_native_fake_driver.c
    ../../applications/foc_native_fake_driver.c
    ../../applications/foc_transport_service.c
    ../../foc/runtime/foc_external_io.c)
target_include_directories(foc_native_fake_driver_test PRIVATE
    ../../applications
    ../../foc/include)
target_compile_features(foc_native_fake_driver_test PRIVATE c_std_11)
add_test(NAME foc_native_fake_driver COMMAND foc_native_fake_driver_test)
