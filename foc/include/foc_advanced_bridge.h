#ifndef FOC_ADVANCED_BRIDGE_H
#define FOC_ADVANCED_BRIDGE_H

/* Optional advanced-FOC configuration/telemetry ABI.
 *
 * This contract is independent from foc_runtime_config_t.  All features are
 * disabled by default and configuration is accepted only while the base
 * controller is stopped, configured and fault-free.  DPWM and overmodulation
 * additionally require platform capability evidence; HFI and flying start are
 * intentionally simulator-only until the target gains a per-tick request path.
 */

#include <stdint.h>

#include "foc_rust_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_ADVANCED_ABI_VERSION       (0x00010000UL)
#define FOC_ADVANCED_CONFIG_VERSION    (1UL)
#define FOC_ADVANCED_PROBE_INPUT_VERSION (1UL)

#define FOC_ADVANCED_FEATURE_MTPA             (1UL << 0)
#define FOC_ADVANCED_FEATURE_FIELD_WEAKENING  (1UL << 1)
#define FOC_ADVANCED_FEATURE_MTPV             (1UL << 2)
#define FOC_ADVANCED_FEATURE_DECOUPLING       (1UL << 3)
#define FOC_ADVANCED_FEATURE_DPWM             (1UL << 4)
#define FOC_ADVANCED_FEATURE_OVERMODULATION   (1UL << 5)
#define FOC_ADVANCED_FEATURE_HFI              (1UL << 6)
#define FOC_ADVANCED_FEATURE_FLYING_START     (1UL << 7)

#define FOC_ADVANCED_CAP_DPWM_CURRENT_RECONSTRUCTION (1UL << 0)
#define FOC_ADVANCED_CAP_OVERMOD_MIN_PULSE           (1UL << 1)
#define FOC_ADVANCED_CAP_HFI_INJECTION                (1UL << 2)
#define FOC_ADVANCED_CAP_PASSIVE_FLYING_START         (1UL << 3)

#define FOC_ADVANCED_STATUS_CONFIGURED       (1UL << 0)
#define FOC_ADVANCED_STATUS_CURRENT_LIMITED  (1UL << 1)
#define FOC_ADVANCED_STATUS_HFI_ANGLE_VALID  (1UL << 2)
#define FOC_ADVANCED_STATUS_BASIC_FALLBACK   (1UL << 3)
#define FOC_ADVANCED_STATUS_FAULTED           (1UL << 4)
#define FOC_ADVANCED_STATUS_CONFIG_REJECTED   (1UL << 5)

/* Stable P5.3 safety-reason bits carried in telemetry.status_flags.  Keeping
 * them in the existing word preserves the V1/68-byte ABI for older readers. */
#define FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE  (1UL << 8)
#define FOC_ADVANCED_REASON_INVALID_RUNTIME_INPUT (1UL << 9)
#define FOC_ADVANCED_REASON_SUPERVISOR_STATE      (1UL << 10)
#define FOC_ADVANCED_REASON_OUTPUT_INVALID        (1UL << 11)
#define FOC_ADVANCED_REASON_CONFIG_IDENTITY       (1UL << 12)
#define FOC_ADVANCED_REASON_CONFIG_INVALID        (1UL << 13)
#define FOC_ADVANCED_REASON_CAPABILITY_MISSING    (1UL << 14)
#define FOC_ADVANCED_REASON_TARGET_UNSUPPORTED    (1UL << 15)
#define FOC_ADVANCED_REASON_CONTROLLER_STATE      (1UL << 16)
#define FOC_ADVANCED_REASON_KNOWN_MASK             \
    (FOC_ADVANCED_REASON_OBSERVER_UNRELIABLE |     \
     FOC_ADVANCED_REASON_INVALID_RUNTIME_INPUT |  \
     FOC_ADVANCED_REASON_SUPERVISOR_STATE |       \
     FOC_ADVANCED_REASON_OUTPUT_INVALID |         \
     FOC_ADVANCED_REASON_CONFIG_IDENTITY |        \
     FOC_ADVANCED_REASON_CONFIG_INVALID |         \
     FOC_ADVANCED_REASON_CAPABILITY_MISSING |     \
     FOC_ADVANCED_REASON_TARGET_UNSUPPORTED |     \
     FOC_ADVANCED_REASON_CONTROLLER_STATE)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t enabled_features;
    uint32_t region_update_divider;
    float current_limit_a;
    float mtpa_min_current_a;
    uint32_t mtpa_search_steps;
    float weakening_entry_utilization;
    float weakening_exit_utilization;
    float weakening_kp_a_per_v;
    float weakening_id_min_a;
    float weakening_slew_a_per_s;
    float mtpv_entry_electrical_speed_rad_s;
    float mtpv_exit_electrical_speed_rad_s;
    uint32_t mtpv_search_steps;
    float decoupling_gain;
    float dpwm_entry_modulation;
    float dpwm_exit_modulation;
    uint32_t dpwm_mode;
    float overmodulation_max_voltage_ratio;
    float overmodulation_entry_modulation;
    float overmodulation_exit_modulation;
    float hfi_amplitude_v;
    float hfi_frequency_hz;
    float hfi_demod_alpha;
    float hfi_high_pass_alpha;
    float hfi_min_response_a;
    float hfi_max_electrical_speed_rad_s;
    uint32_t hfi_settling_samples;
    float flying_start_min_electrical_speed_rad_s;
    uint32_t flying_start_stable_samples;
    uint32_t flying_start_timeout_samples;
} foc_advanced_algorithm_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t platform_capabilities;
    uint32_t reserved;
    foc_advanced_algorithm_config_t algorithm;
} foc_advanced_runtime_config_t;

/* Diagnostic-only synthetic observer/control frame for the no-power target
 * timing path.  It is never consumed by the powered realtime entry. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    float base_id_reference_a;
    float base_iq_reference_a;
    float electrical_angle_rad;
    float mechanical_speed_rad_s;
    float previous_vd_command_v;
    float previous_vq_command_v;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
} foc_advanced_probe_input_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t active_features;
    uint32_t region;
    uint32_t modulation_mode;
    uint32_t status_flags;
    uint32_t flying_start_state;
    float id_reference_a;
    float iq_reference_a;
    float vd_feedforward_v;
    float vq_feedforward_v;
    float injection_alpha_v;
    float injection_beta_v;
    float voltage_limit_v;
    float hfi_angle_candidate_rad;
    float flying_start_angle_rad;
    float flying_start_speed_rad_s;
} foc_advanced_telemetry_t;

uint32_t foc_rust_advanced_abi_version(void);
foc_status_t foc_rust_default_advanced_config(
    foc_rust_context_t *context,
    foc_advanced_runtime_config_t *config);
foc_status_t foc_rust_configure_advanced(
    foc_rust_context_t *context,
    const foc_advanced_runtime_config_t *config);
foc_status_t foc_rust_get_advanced_telemetry(
    foc_rust_context_t *context,
    foc_advanced_telemetry_t *telemetry);
/* Executes the normal observer/startup body and the exact advanced/current-loop
 * composition in one call, but substitutes the trusted control frame above.
 * The caller must keep Gate, MOE and all phase channels physically off and must
 * discard the returned duty from every active output path. */
foc_status_t foc_rust_realtime_step_advanced_no_power(
    foc_rust_context_t *context,
    const foc_realtime_input_t *input,
    const foc_advanced_probe_input_t *probe_input,
    foc_output_t *output,
    foc_telemetry_t *telemetry,
    foc_advanced_telemetry_t *advanced_telemetry);

_Static_assert(sizeof(foc_advanced_algorithm_config_t) == 128U,
               "advanced algorithm config ABI size mismatch");
_Static_assert(sizeof(foc_advanced_runtime_config_t) == 144U,
               "advanced runtime config ABI size mismatch");
_Static_assert(sizeof(foc_advanced_probe_input_t) == 44U,
               "advanced probe input ABI size mismatch");
_Static_assert(sizeof(foc_advanced_telemetry_t) == 68U,
               "advanced telemetry ABI size mismatch");

#ifdef __cplusplus
}
#endif

#endif /* FOC_ADVANCED_BRIDGE_H */
