#ifndef FOC_PLATFORM_H
#define FOC_PLATFORM_H

#include "foc_rust_bridge.h"
#include "foc_realtime_timing.h"
#include "foc_types.h"

#define FOC_PLATFORM_CONFIG_VERSION (1UL)
#define FOC_DEFAULT_ISR_DEADLINE_CYCLES (12500UL)

enum
{
    FOC_PLATFORM_DIAG_GATE_SAFE = (1UL << 0),
    FOC_PLATFORM_DIAG_TIM1_CONFIGURED = (1UL << 1),
    FOC_PLATFORM_DIAG_ADC_CONFIGURED = (1UL << 2),
    FOC_PLATFORM_DIAG_ADC_CALIBRATED = (1UL << 3),
    FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID = (1UL << 4),
    FOC_PLATFORM_DIAG_DRIVER_FAULT = (1UL << 5),
    FOC_PLATFORM_DIAG_OUTPUT_ACTIVE = (1UL << 6),
    FOC_PLATFORM_DIAG_ADC_READ_ERROR = (1UL << 7),
    FOC_PLATFORM_DIAG_SYNC_RUNNING = (1UL << 8),
    FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID = (1UL << 9),
    FOC_PLATFORM_DIAG_BREAK_LATCHED = (1UL << 10),
    FOC_PLATFORM_DIAG_CURRENT_TRIP = (1UL << 11),
    FOC_PLATFORM_DIAG_TRIAL_ARMED = (1UL << 12),
    FOC_PLATFORM_DIAG_CONTROLLER_BOUND = (1UL << 13),
    FOC_PLATFORM_DIAG_REALTIME_ARMED = (1UL << 14),
    FOC_PLATFORM_DIAG_DEADLINE_MISSED = (1UL << 15),
    FOC_PLATFORM_DIAG_CONTROL_ERROR = (1UL << 16),
    FOC_PLATFORM_DIAG_OUTPUT_REJECTED = (1UL << 17),
};

typedef struct
{
    uint32_t struct_size;
    uint32_t config_version;
    float minimum_bus_voltage_v;
    float maximum_bus_voltage_v;
    float software_current_trip_a;
    float minimum_duty;
    float maximum_duty;
    uint32_t isr_deadline_cycles;
} foc_platform_config_t;

typedef struct
{
    uint32_t flags;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_period_ticks;
    uint32_t sync_sample_count;
    uint32_t break_fault_count;
    uint32_t trial_apply_count;
    uint32_t realtime_step_count;
    uint32_t realtime_error_count;
    uint32_t maximum_isr_cycles;
    uint32_t maximum_precontrol_cycles;
    uint32_t maximum_control_cycles;
    uint32_t maximum_postcontrol_cycles;
    uint32_t deadline_miss_count;
    uint32_t last_control_status;
    uint32_t control_fault_flags;
    uint16_t last_duty_a_per_mille;
    uint16_t last_duty_b_per_mille;
    uint16_t last_duty_c_per_mille;
    uint16_t reserved1;
    uint16_t peak_current_delta_counts;
    uint16_t reserved0;
    uint16_t phase_u_raw;
    uint16_t phase_v_raw;
    uint16_t phase_w_raw;
    uint16_t phase_u_offset;
    uint16_t phase_v_offset;
    uint16_t phase_w_offset;
    uint16_t bus_voltage_raw;
    uint16_t temperature_raw;
    uint16_t potentiometer_raw;
    uint16_t reserved;
} foc_platform_diagnostics_t;

/* Fixed-point trace sample shared by the ISR producer and shell consumer. */
typedef struct
{
    uint32_t step;
    uint32_t flags;
    uint16_t state;
    uint16_t observer_reliable;
    int16_t phase_a_ma;
    int16_t phase_b_ma;
    int16_t phase_c_ma;
    int16_t id_reference_ma;
    int16_t iq_reference_ma;
    int16_t id_measured_ma;
    int16_t iq_measured_ma;
    int16_t vd_command_mv;
    int16_t vq_command_mv;
    uint16_t duty_a_per_mille;
    uint16_t duty_b_per_mille;
    uint16_t duty_c_per_mille;
    uint16_t bus_voltage_mv;
    int16_t control_angle_mrad;
    int16_t forced_angle_mrad;
    int16_t observer_angle_mrad;
    int16_t observer_speed_rpm;
} foc_trace_sample_t;

/*
 * Chip/board port contract. A future MCU adapter must implement these symbols.
 * The STM32G431 adapter boots with every IHM16M1 output disabled. Its normal
 * closed-loop path remains unavailable; an explicit, bounded bring-up trial
 * may arm the power stage through foc_platform_trial_arm().
 */
foc_status_t foc_platform_init(void);
foc_status_t foc_platform_configure(const foc_platform_config_t *config);
foc_status_t foc_platform_bind_controller(foc_rust_context_t *context);
foc_status_t foc_platform_control_start(float target_speed_rpm);
void foc_platform_control_stop(void);
void foc_platform_emergency_stop(void);
foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback);
foc_status_t foc_platform_apply_output(const foc_output_t *output);
foc_status_t foc_platform_get_diagnostics(foc_platform_diagnostics_t *diagnostics);
foc_status_t foc_platform_get_telemetry(foc_telemetry_t *telemetry);
foc_status_t foc_platform_get_timing(foc_realtime_timing_stats_t *timing);
foc_status_t foc_platform_trace_start(uint32_t sample_divider);
void foc_platform_trace_stop(void);
uint32_t foc_platform_trace_is_enabled(void);
uint32_t foc_platform_trace_pop(foc_trace_sample_t *sample);
uint32_t foc_platform_trace_dropped(void);
foc_status_t foc_platform_trial_arm(void);
void foc_platform_trial_disarm(void);

#endif
