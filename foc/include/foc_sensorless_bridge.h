#ifndef FOC_SENSORLESS_BRIDGE_H
#define FOC_SENSORLESS_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "foc_rust_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* P5.5 full-speed sensorless ABI. Compiling this header or Rust module does
 * not grant a board permission to inject voltage. The platform provider must
 * supply every required capability at the stopped-state transaction. */
#define FOC_SENSORLESS_ABI_VERSION              (0x00010000UL)
#define FOC_SENSORLESS_CONFIG_VERSION           (1UL)
#define FOC_SENSORLESS_GUARD_VERSION            (1UL)
#define FOC_SENSORLESS_INPUT_VERSION            (1UL)
#define FOC_SENSORLESS_OUTPUT_VERSION           (2UL)
#define FOC_SENSORLESS_VOLTAGE_INPUT_VERSION    (1UL)
#define FOC_SENSORLESS_VOLTAGE_OUTPUT_VERSION   (1UL)
#define FOC_SENSORLESS_COMPOSITE_INPUT_VERSION  (1UL)
#define FOC_SENSORLESS_COMPOSITE_OUTPUT_VERSION (1UL)
#define FOC_SENSORLESS_CONTEXT_CAPACITY         (512UL)

#define FOC_SENSORLESS_CAP_SYNCHRONIZED_CURRENT_SAMPLE (1UL << 0)
#define FOC_SENSORLESS_CAP_CALIBRATED_ALPHA_BETA_CURRENT (1UL << 1)
#define FOC_SENSORLESS_CAP_NEXT_PWM_VOLTAGE_INJECTION  (1UL << 2)
#define FOC_SENSORLESS_CAP_FINAL_VECTOR_LIMIT          (1UL << 3)
#define FOC_SENSORLESS_CAP_HARDWARE_FAST_SHUTDOWN      (1UL << 4)
#define FOC_SENSORLESS_CAP_BEMF_CHANNEL                (1UL << 5)
#define FOC_SENSORLESS_CAP_KNOWN_MASK                  (0x3FUL)
#define FOC_SENSORLESS_REQUIRED_CAPABILITIES           \
    FOC_SENSORLESS_CAP_KNOWN_MASK

#define FOC_SENSORLESS_INPUT_INJECTION_PERMITTED (1UL << 0)
#define FOC_SENSORLESS_INPUT_BEMF_VALID          (1UL << 1)
#define FOC_SENSORLESS_INPUT_RESET               (1UL << 2)
#define FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED (1UL << 3)
#define FOC_SENSORLESS_INPUT_KNOWN_MASK           (0x0FUL)

#define FOC_SENSORLESS_OUTPUT_CONFIGURED          (1UL << 0)
#define FOC_SENSORLESS_OUTPUT_ENABLED             (1UL << 1)
#define FOC_SENSORLESS_OUTPUT_INJECTION_REQUESTED (1UL << 2)
#define FOC_SENSORLESS_OUTPUT_ANGLE_RELIABLE      (1UL << 3)
#define FOC_SENSORLESS_OUTPUT_FALLBACK_REQUIRED   (1UL << 4)
#define FOC_SENSORLESS_OUTPUT_FAULT_LATCHED       (1UL << 5)

#define FOC_SENSORLESS_VOLTAGE_OUTPUT_LIMITED     (1UL << 0)
#define FOC_SENSORLESS_COMPOSITE_OUTPUT_INJECTION_LIMITED (1UL << 0)

#define FOC_SENSORLESS_STAGE_AXIS_ACQUISITION      (0UL)
#define FOC_SENSORLESS_STAGE_POLARITY              (1UL)
#define FOC_SENSORLESS_STAGE_TRACKING              (2UL)
#define FOC_SENSORLESS_STAGE_FAILED                (3UL)

#define FOC_SENSORLESS_ANGLE_SOURCE_UNAVAILABLE    (0UL)
#define FOC_SENSORLESS_ANGLE_SOURCE_HFI            (1UL)
#define FOC_SENSORLESS_ANGLE_SOURCE_BLEND          (2UL)
#define FOC_SENSORLESS_ANGLE_SOURCE_BEMF           (3UL)

typedef enum
{
    FOC_SENSORLESS_STATUS_OK = 0,
    FOC_SENSORLESS_STATUS_INVALID_ARGUMENT = 1,
    FOC_SENSORLESS_STATUS_NOT_INITIALIZED = 2,
    FOC_SENSORLESS_STATUS_NOT_CONFIGURED = 3,
    FOC_SENSORLESS_STATUS_UNSAFE_CONFIGURATION_STATE = 4,
    FOC_SENSORLESS_STATUS_CAPABILITY_MISSING = 5,
    FOC_SENSORLESS_STATUS_INVALID_CONFIGURATION = 6,
    FOC_SENSORLESS_STATUS_SEQUENCE_MISMATCH = 7,
    FOC_SENSORLESS_STATUS_APPLIED_REQUEST_MISMATCH = 8,
    FOC_SENSORLESS_STATUS_CHAIN_FAILURE = 9,
    FOC_SENSORLESS_STATUS_FAULT_LATCHED = 10
} foc_sensorless_status_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    float amplitude_v;
    float frequency_hz;
    float sample_period_s;
    float demod_alpha;
    float phase_offset_rad;
    float minimum_response_a;
    float maximum_electrical_speed_rad_s;
    uint32_t axis_stable_samples;
    float current_low_pass_alpha;
} foc_sensorless_hfi_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    float pulse_voltage_v;
    uint32_t pulse_ticks;
    uint32_t minimum_off_ticks;
    uint32_t settle_timeout_ticks;
    float maximum_current_a;
    float maximum_residual_current_a;
    float minimum_response_delta_a;
    uint32_t maximum_pulse_pairs;
} foc_sensorless_polarity_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    float blend_enter_speed_rad_s;
    float hfi_reenter_speed_rad_s;
    float bemf_enter_speed_rad_s;
    float bemf_exit_speed_rad_s;
    float maximum_angle_disagreement_rad;
    float weight_slew_per_sample;
    uint32_t stable_samples;
    uint32_t invalid_timeout_samples;
} foc_sensorless_fusion_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t config_version;
    uint32_t enabled;
    uint32_t reserved;
    foc_sensorless_hfi_config_t hfi;
    foc_sensorless_polarity_config_t polarity;
    foc_sensorless_fusion_config_t fusion;
} foc_sensorless_runtime_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t controller_stopped;
    uint32_t outputs_disabled;
    uint32_t no_faults;
    uint32_t platform_capabilities;
    uint32_t reserved;
} foc_sensorless_configure_guard_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sample_sequence;
    uint32_t applied_request_sequence;
    uint32_t platform_capabilities;
    uint32_t input_flags;
    float measured_current_alpha_a;
    float measured_current_beta_a;
    float applied_injection_alpha_v;
    float applied_injection_beta_v;
    float bemf_angle_rad;
    float bemf_electrical_speed_rad_s;
} foc_sensorless_realtime_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sample_sequence;
    uint32_t request_apply_sequence;
    uint32_t status_flags;
    uint32_t stage;
    uint32_t failure;
    uint32_t polarity_failure;
    uint32_t angle_source;
    uint32_t angle_reliable;
    uint32_t fallback_required;
    float electrical_speed_rad_s;
    float injection_alpha_v;
    float injection_beta_v;
    float electrical_angle_rad;
    float hfi_angle_mod_pi_rad;
    float hfi_response_a;
    float bemf_weight;
    float angle_disagreement_rad;
    float high_frequency_current_alpha_a;
    float high_frequency_current_beta_a;
} foc_sensorless_realtime_output_t;

/* Deterministic N+1 voltage-composition transaction.  The caller supplies the
 * base stationary-frame voltage produced by the basic current loop and the
 * sensorless request for the same future PWM sequence.  Rust owns vector
 * composition, final circle limiting and SVPWM; C remains the sole CCR owner.
 * This ABI is usable by a gate-off commissioning image without granting the
 * real platform any sensorless capability bit. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t request_apply_sequence;
    uint32_t pwm_sequence;
    float base_voltage_alpha_v;
    float base_voltage_beta_v;
    float injection_alpha_v;
    float injection_beta_v;
    float dc_bus_voltage_v;
    float voltage_limit_v;
    float minimum_duty;
    float maximum_duty;
} foc_sensorless_voltage_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t pwm_sequence;
    uint32_t status_flags;
    float final_voltage_alpha_v;
    float final_voltage_beta_v;
    float applied_injection_alpha_v;
    float applied_injection_beta_v;
    float duty_a;
    float duty_b;
    float duty_c;
} foc_sensorless_voltage_output_t;

/* Formal combined fast-loop extension.  Currents, bus voltage, hardware fault
 * mirror and sample sequence come exclusively from foc_realtime_input_t, so a
 * caller cannot accidentally give HFI and the current PI two different ADC
 * samples.  The applied injection is the prior composite output. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t platform_capabilities;
    uint32_t input_flags;
    uint32_t applied_request_sequence;
    uint32_t reserved;
    float applied_injection_alpha_v;
    float applied_injection_beta_v;
    float voltage_limit_v;
    float minimum_duty;
    float maximum_duty;
} foc_sensorless_composite_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t pwm_sequence;
    uint32_t status_flags;
    float applied_injection_alpha_v;
    float applied_injection_beta_v;
} foc_sensorless_composite_output_t;

#if defined(_MSC_VER)
#define FOC_SENSORLESS_ALIGN8 __declspec(align(8))
#else
#define FOC_SENSORLESS_ALIGN8 __attribute__((aligned(8)))
#endif

typedef struct FOC_SENSORLESS_ALIGN8
{
    uint8_t bytes[FOC_SENSORLESS_CONTEXT_CAPACITY];
} foc_sensorless_context_t;

uint32_t foc_rust_sensorless_abi_version(void);
uint32_t foc_rust_sensorless_required_capabilities(void);
foc_sensorless_status_t foc_rust_sensorless_init(
    foc_sensorless_context_t *storage);
foc_sensorless_status_t foc_rust_sensorless_default_config(
    foc_sensorless_runtime_config_t *output);
foc_sensorless_status_t foc_rust_sensorless_configure(
    foc_sensorless_context_t *storage,
    const foc_sensorless_runtime_config_t *config,
    const foc_sensorless_configure_guard_t *guard);
foc_sensorless_status_t foc_rust_sensorless_step(
    foc_sensorless_context_t *storage,
    const foc_sensorless_realtime_input_t *input,
    foc_sensorless_realtime_output_t *output);
foc_sensorless_status_t foc_rust_sensorless_compose_voltage(
    const foc_sensorless_voltage_input_t *input,
    foc_sensorless_voltage_output_t *output);
foc_status_t foc_rust_realtime_step_sensorless(
    foc_rust_context_t *context,
    foc_sensorless_context_t *sensorless_context,
    const foc_realtime_input_t *input,
    const foc_sensorless_composite_input_t *composite_input,
    foc_output_t *output,
    foc_telemetry_t *telemetry,
    foc_sensorless_realtime_output_t *sensorless_output,
    foc_sensorless_composite_output_t *composite_output);

_Static_assert(sizeof(foc_sensorless_hfi_config_t) == 44U,
               "sensorless HFI config ABI mismatch");
_Static_assert(sizeof(foc_sensorless_polarity_config_t) == 40U,
               "sensorless polarity config ABI mismatch");
_Static_assert(sizeof(foc_sensorless_fusion_config_t) == 40U,
               "sensorless fusion config ABI mismatch");
_Static_assert(sizeof(foc_sensorless_runtime_config_t) == 144U,
               "sensorless runtime config ABI mismatch");
_Static_assert(sizeof(foc_sensorless_configure_guard_t) == 28U,
               "sensorless configure guard ABI mismatch");
_Static_assert(sizeof(foc_sensorless_realtime_input_t) == 48U,
               "sensorless realtime input ABI mismatch");
_Static_assert(sizeof(foc_sensorless_realtime_output_t) == 84U,
               "sensorless realtime output ABI mismatch");
_Static_assert(sizeof(foc_sensorless_voltage_input_t) == 48U,
               "sensorless voltage input ABI mismatch");
_Static_assert(sizeof(foc_sensorless_voltage_output_t) == 44U,
               "sensorless voltage output ABI mismatch");
_Static_assert(sizeof(foc_sensorless_composite_input_t) == 44U,
               "sensorless composite input ABI mismatch");
_Static_assert(sizeof(foc_sensorless_composite_output_t) == 24U,
               "sensorless composite output ABI mismatch");
_Static_assert(sizeof(foc_sensorless_context_t) == FOC_SENSORLESS_CONTEXT_CAPACITY,
               "sensorless context capacity mismatch");
_Static_assert(offsetof(foc_sensorless_realtime_input_t, measured_current_alpha_a) == 24U,
               "sensorless input field offset mismatch");
_Static_assert(offsetof(foc_sensorless_realtime_output_t, injection_alpha_v) == 48U,
               "sensorless output field offset mismatch");

#ifdef __cplusplus
}
#endif

#endif
