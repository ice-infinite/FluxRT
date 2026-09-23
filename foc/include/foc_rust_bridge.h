#ifndef FOC_RUST_BRIDGE_H
#define FOC_RUST_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_RUST_ABI_VERSION        (0x00070000UL)
#define FOC_RUST_CONFIG_VERSION     (4UL)
#define FOC_RUST_CONTEXT_CAPACITY   (2048U)

#define FOC_RUST_FAULT_ALGORITHM_OUTPUT  (1UL << 0)
#define FOC_RUST_FAULT_INVALID_FEEDBACK  (1UL << 1)
#define FOC_RUST_FAULT_OBSERVER_STARTUP  (1UL << 2)
#define FOC_RUST_FAULT_OBSERVER_LOST     (1UL << 3)

typedef uint32_t foc_observer_backend_t;
enum
{
    FOC_OBSERVER_SMO_PLL = 0,
    FOC_OBSERVER_BEMF_PLL = 1,
    FOC_OBSERVER_ST_STO_PLL = 2,
};

typedef union
{
    uint64_t alignment;
    uint8_t bytes[FOC_RUST_CONTEXT_CAPACITY];
} foc_rust_context_t;

typedef struct
{
    float kp;
    float ki;
    float ts;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
} foc_pi_config_t;

typedef struct
{
    foc_pi_config_t id_pi;
    foc_pi_config_t iq_pi;
    float nominal_dc_bus_voltage;
} foc_basic_config_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t config_version;
    foc_observer_backend_t observer_backend;
    uint32_t observer_enable;
    uint32_t closed_loop_enable;
    uint32_t observer_update_divider;
    foc_pi_config_t id_pi;
    foc_pi_config_t iq_pi;
    foc_pi_config_t speed_pi;
    uint32_t pole_pairs;
    uint32_t pwm_frequency_hz;
    uint32_t speed_loop_frequency_hz;
    float stator_resistance_ohm;
    float stator_inductance_h;
    float flux_linkage_wb;
    float rated_current_a;
    float max_speed_rpm;
    float nominal_bus_voltage_v;
    float voltage_utilization;
    float default_target_speed_rpm;
    float alignment_duration_s;
    float open_loop_ramp_duration_s;
    float observer_transition_duration_s;
    float startup_final_speed_rpm;
    float startup_current_a;
    float observer_smo_k_slide_v;
    float observer_smo_boundary_a;
    float observer_emf_filter_alpha;
    float observer_pll_kp;
    float observer_pll_ki;
    float observer_minimum_speed_rpm;
    float observer_minimum_bemf_v;
    float observer_speed_variance_ratio;
    uint32_t observer_consecutive_samples;
    float observer_acquisition_timeout_s;
    float observer_loss_timeout_s;
    float closed_loop_speed_ramp_rpm_per_s;
    float speed_pi_preload_ratio;
    float closed_loop_current_slew_a_per_s;
} foc_runtime_config_t;

typedef struct
{
    foc_state_t state;
    foc_observer_backend_t observer_backend;
    uint32_t observer_reliable;
    uint32_t closed_loop_active;
    float target_speed_rpm;
    float measured_speed_rpm;
    float electrical_angle_rad;
    float id_reference_a;
    float iq_reference_a;
    float id_measured_a;
    float iq_measured_a;
    float vd_command_v;
    float vq_command_v;
    float forced_electrical_angle_rad;
    float observer_electrical_angle_rad;
} foc_telemetry_t;

uint32_t foc_rust_abi_version(void);
uint32_t foc_rust_context_required_size(void);
uint32_t foc_rust_context_required_align(void);

foc_status_t foc_rust_init(foc_rust_context_t *context);
foc_status_t foc_rust_default_st_config(foc_runtime_config_t *config);
foc_status_t foc_rust_configure(foc_rust_context_t *context,
                                const foc_runtime_config_t *config);
foc_status_t foc_rust_configure_basic(foc_rust_context_t *context,
                                      const foc_basic_config_t *config);
foc_status_t foc_rust_configure_st_reference(foc_rust_context_t *context);
foc_status_t foc_rust_request_start(foc_rust_context_t *context,
                                    uint32_t platform_ready);
foc_status_t foc_rust_start_realtime(foc_rust_context_t *context,
                                     uint32_t platform_ready,
                                     float target_speed_rpm);
foc_status_t foc_rust_realtime_step(foc_rust_context_t *context,
                                    const foc_feedback_t *feedback,
                                    foc_output_t *output,
                                    foc_telemetry_t *telemetry);
foc_status_t foc_rust_get_telemetry(foc_rust_context_t *context,
                                    foc_telemetry_t *telemetry);
void foc_rust_stop(foc_rust_context_t *context);
foc_status_t foc_rust_fast_step(foc_rust_context_t *context,
                                const foc_feedback_t *feedback,
                                const foc_reference_t *reference,
                                foc_output_t *output);
foc_status_t foc_rust_open_loop_step(foc_rust_context_t *context,
                                     float electrical_speed_rad_s,
                                     float voltage_magnitude_v,
                                     float dc_bus_voltage_v,
                                     float sample_time_s,
                                     foc_output_t *output);
foc_status_t foc_rust_speed_step(foc_rust_context_t *context,
                                 float target_speed_rpm,
                                 float measured_speed_rpm,
                                 foc_reference_t *reference);
foc_status_t foc_rust_latch_fault(foc_rust_context_t *context,
                                  uint32_t fault_flags);
foc_status_t foc_rust_clear_fault(foc_rust_context_t *context);
foc_state_t foc_rust_state(foc_rust_context_t *context);
uint32_t foc_rust_fault_flags(foc_rust_context_t *context);

#ifdef __cplusplus
}
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_status_t) == sizeof(uint32_t), "FOC status ABI changed");
_Static_assert(sizeof(foc_state_t) == sizeof(uint32_t), "FOC state ABI changed");
_Static_assert(sizeof(foc_feedback_t) == 20U, "FOC feedback ABI changed");
_Static_assert(sizeof(foc_reference_t) == 8U, "FOC reference ABI changed");
_Static_assert(sizeof(foc_output_t) == 12U, "FOC output ABI changed");
_Static_assert(sizeof(foc_basic_config_t) == 60U, "FOC config ABI changed");
_Static_assert(sizeof(foc_runtime_config_t) == 228U, "FOC runtime config ABI changed");
_Static_assert(sizeof(foc_telemetry_t) == 60U, "FOC telemetry ABI changed");
_Static_assert(sizeof(foc_rust_context_t) == FOC_RUST_CONTEXT_CAPACITY,
               "Rust context storage ABI changed");
#endif

#endif
