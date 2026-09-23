# This file is intentionally outside generated CMakeLists.txt. RT-Thread's
# generator includes it automatically, so regeneration does not erase Rust.

find_program(CARGO_EXECUTABLE cargo REQUIRED)

set(FLUXRT_BUILD_PROFILE "diagnostic" CACHE STRING
    "FluxRT firmware profile: diagnostic or production")
set_property(CACHE FLUXRT_BUILD_PROFILE PROPERTY STRINGS diagnostic production)
set(FLUXRT_RUST_OPT_LEVEL "s" CACHE STRING
    "Rust release optimization level: 3, s, or z")
set_property(CACHE FLUXRT_RUST_OPT_LEVEL PROPERTY STRINGS 3 s z)

if(NOT FLUXRT_BUILD_PROFILE MATCHES "^(diagnostic|production)$")
    message(FATAL_ERROR "FLUXRT_BUILD_PROFILE must be diagnostic or production")
endif()
if(NOT FLUXRT_RUST_OPT_LEVEL MATCHES "^(3|s|z)$")
    message(FATAL_ERROR "FLUXRT_RUST_OPT_LEVEL must be 3, s, or z")
endif()

set(FOC_RUST_TARGET "thumbv7em-none-eabihf")
set(FOC_RUST_DIR "${CMAKE_SOURCE_DIR}/rust")
set(FOC_RUST_TARGET_DIR
    "${CMAKE_SOURCE_DIR}/build/rust-target-${FLUXRT_RUST_OPT_LEVEL}")
set(FOC_RUST_ARCHIVE
    "${FOC_RUST_TARGET_DIR}/${FOC_RUST_TARGET}/release/libfoc_rt_bridge.a")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/rtconfig.h")

file(STRINGS "${CMAKE_SOURCE_DIR}/rtconfig.h" FOC_CORDIC_CONFIG_LINE
    REGEX "^#define FOC_MATH_BACKEND_STM32G4_CORDIC$")
set(FOC_RUST_FEATURE_ARGS)
if(FOC_CORDIC_CONFIG_LINE)
    list(APPEND FOC_RUST_FEATURE_ARGS --features stm32g4-cordic)
    message(STATUS "FOC math backend: STM32G4 CORDIC with automatic CPU fallback")
else()
    message(STATUS "FOC math backend: portable CPU")
endif()

file(GLOB_RECURSE FOC_RUST_SOURCES CONFIGURE_DEPENDS
    "${FOC_RUST_DIR}/crates/*.rs"
    "${FOC_RUST_DIR}/crates/*/Cargo.toml")

add_custom_command(
    OUTPUT "${FOC_RUST_ARCHIVE}"
    COMMAND "${CMAKE_COMMAND}" -E env
            "CARGO_TARGET_DIR=${FOC_RUST_TARGET_DIR}"
            "CARGO_PROFILE_RELEASE_OPT_LEVEL=${FLUXRT_RUST_OPT_LEVEL}"
            "${CARGO_EXECUTABLE}" build
            --manifest-path "${FOC_RUST_DIR}/Cargo.toml"
            --package foc-rt-bridge
            --release
            --target "${FOC_RUST_TARGET}"
            --locked
            ${FOC_RUST_FEATURE_ARGS}
    DEPENDS
        ${FOC_RUST_SOURCES}
        "${FOC_RUST_DIR}/Cargo.toml"
        "${FOC_RUST_DIR}/Cargo.lock"
        "${FOC_RUST_DIR}/rust-toolchain.toml"
        "${CMAKE_SOURCE_DIR}/rtconfig.h"
    WORKING_DIRECTORY "${FOC_RUST_DIR}"
    COMMENT "Building Rust FOC static library for ${FOC_RUST_TARGET}"
    VERBATIM
    USES_TERMINAL)

add_custom_target(foc_rust_bridge_build DEPENDS "${FOC_RUST_ARCHIVE}")
add_dependencies(${CMAKE_PROJECT_NAME}.elf foc_rust_bridge_build)
# The RT-Thread generator copied the SCons library path before this file was
# included. Replace that generated path so each optimization variant links the
# archive it just built instead of a stale build/rust-target archive.
set_property(TARGET rtt_FOC PROPERTY INTERFACE_LINK_DIRECTORIES
    "${FOC_RUST_TARGET_DIR}/${FOC_RUST_TARGET}/release")
# The generated project links with `-lfoc_rt_bridge`, which establishes link
# order but does not make Ninja watch the archive path. This explicit dependency
# guarantees that a Rust-only source change relinks the final ELF in the same run.
set_property(TARGET ${CMAKE_PROJECT_NAME}.elf APPEND PROPERTY
    LINK_DEPENDS "${FOC_RUST_ARCHIVE}")

# The ADC ISR has about 14167 core cycles at 12 kHz. Keep debug symbols, but compile
# the realtime hardware adapter with release optimization even when the rest of
# the RT-Thread image uses the developer-friendly project default.
target_compile_options(rtt_FOC PRIVATE -O3)

target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
    FLUXRT_BUILD_PROFILE_NAME="${FLUXRT_BUILD_PROFILE}"
    FLUXRT_RUST_OPT_LEVEL_NAME="${FLUXRT_RUST_OPT_LEVEL}")

if(FLUXRT_BUILD_PROFILE STREQUAL "production")
    target_compile_definitions(${CMAKE_PROJECT_NAME}.elf PRIVATE
        FLUXRT_PRODUCTION_BUILD=1)
    target_compile_definitions(rtt_FOC PRIVATE FLUXRT_PRODUCTION_BUILD=1)
endif()

message(STATUS "FluxRT build profile: ${FLUXRT_BUILD_PROFILE}")
message(STATUS "FluxRT Rust opt-level: ${FLUXRT_RUST_OPT_LEVEL}")
