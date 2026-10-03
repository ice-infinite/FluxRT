#ifndef FOC_POWER_BRIDGE_H
#define FOC_POWER_BRIDGE_H

/*
 * Versioned management-plane ABI for thermal, DC-bus and regenerative-energy
 * policy.  It owns no ADC, PWM, brake GPIO or RTOS object.  The caller supplies
 * calibrated SI samples and must route shutdown requests through the existing
 * immediate C-side safety path.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_POWER_ABI_VERSION       (0x00010000UL)
#define FOC_POWER_CONTEXT_CAPACITY  (128UL)

typedef uint32_t foc_power_status_t;
enum
{
    FOC_POWER_STATUS_OK = 0,
    FOC_POWER_STATUS_INVALID_ARGUMENT = 1,
    FOC_POWER_STATUS_NOT_INITIALIZED = 2,
    FOC_POWER_STATUS_INVALID_TEMPERATURE_WINDOW = 3,
    FOC_POWER_STATUS_INVALID_BUS_WINDOW = 4,
    FOC_POWER_STATUS_INVALID_SOURCE_CAPABILITY = 5,
    FOC_POWER_STATUS_INVALID_SINK_CAPABILITY = 6,
    FOC_POWER_STATUS_INVALID_BRAKE_CAPABILITY = 7,
    FOC_POWER_STATUS_UNSAFE_RESET = 8,
};

#define FOC_POWER_FAULT_TEMPERATURE_SENSOR  (1UL << 0)
#define FOC_POWER_FAULT_OVER_TEMPERATURE    (1UL << 1)
#define FOC_POWER_FAULT_BUS_UNDERVOLTAGE    (1UL << 2)
#define FOC_POWER_FAULT_BUS_OVERVOLTAGE     (1UL << 3)
#define FOC_POWER_FAULT_INVALID_INPUT       (1UL << 4)
#define FOC_POWER_FAULT_SOURCE_UNAVAILABLE  (1UL << 5)
#define FOC_POWER_FAULT_KNOWN_MASK          (0x3FUL)

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t enabled;
    uint32_t temperature_sensor_required;
    float minimum_valid_temperature_c;
    float maximum_valid_temperature_c;
    float thermal_derating_release_c;
    float thermal_derating_start_c;
    float over_temperature_trip_c;
    float bus_undervoltage_trip_v;
    float bus_undervoltage_recovery_v;
    float bus_overvoltage_recovery_v;
    float bus_overvoltage_trip_v;
    float source_current_limit_a;
    uint32_t regeneration_allowed;
    float sink_current_limit_a;
    uint32_t brake_resistor_available;
    float brake_current_limit_a;
    float brake_release_voltage_v;
    float brake_engage_voltage_v;
} foc_power_runtime_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t temperature_valid;
    uint32_t source_available;
    uint32_t sink_available;
    uint32_t brake_available;
    float temperature_c;
    float dc_bus_voltage_v;
    float requested_dc_current_a;
} foc_power_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t configured;
    uint32_t enabled;
    uint32_t drive_allowed;
    uint32_t shutdown_requested;
    uint32_t active_fault_flags;
    uint32_t latched_fault_flags;
    uint32_t temperature_valid;
    uint32_t thermal_derating_active;
    float thermal_current_scale;
    float maximum_source_current_a;
    float maximum_regeneration_current_a;
    float requested_dc_current_a;
    float limited_dc_current_a;
    uint32_t source_limited;
    uint32_t regeneration_limited;
    uint32_t brake_requested;
    float brake_current_limit_a;
} foc_power_output_t;

#if defined(__GNUC__) || defined(__clang__)
typedef struct __attribute__((aligned(8)))
#else
typedef struct
#endif
{
    uint8_t bytes[FOC_POWER_CONTEXT_CAPACITY];
} foc_power_context_storage_t;

uint32_t foc_rust_power_abi_version(void);
foc_power_status_t foc_rust_power_init(foc_power_context_storage_t *storage);
foc_power_status_t foc_rust_power_default_config(
    foc_power_runtime_config_t *output);
foc_power_status_t foc_rust_power_configure(
    foc_power_context_storage_t *storage,
    const foc_power_runtime_config_t *config);
foc_power_status_t foc_rust_power_step(
    foc_power_context_storage_t *storage,
    const foc_power_input_t *input,
    foc_power_output_t *output);
foc_power_status_t foc_rust_power_reset_faults(
    foc_power_context_storage_t *storage,
    const foc_power_input_t *input);

_Static_assert(sizeof(foc_power_runtime_config_t) == 80U,
               "power runtime config ABI size mismatch");
_Static_assert(sizeof(foc_power_input_t) == 36U,
               "power input ABI size mismatch");
_Static_assert(sizeof(foc_power_output_t) == 76U,
               "power output ABI size mismatch");
_Static_assert(sizeof(foc_power_context_storage_t) ==
               FOC_POWER_CONTEXT_CAPACITY,
               "power context ABI size mismatch");

#ifdef __cplusplus
}
#endif

#endif /* FOC_POWER_BRIDGE_H */
