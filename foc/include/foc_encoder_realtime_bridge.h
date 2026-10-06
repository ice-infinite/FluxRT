#ifndef FOC_ENCODER_REALTIME_BRIDGE_H
#define FOC_ENCODER_REALTIME_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#include "foc_rust_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Independent from FOC_RUST_ABI_VERSION. This contract adds a sensored
 * realtime owner without changing the frozen sensorless ABI. */
#define FOC_ENCODER_REALTIME_ABI_VERSION   (0x00010000UL)
#define FOC_ENCODER_REALTIME_INPUT_VERSION (1UL)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t control_sequence;
    uint32_t hardware_fault_flags;
    float actual_dt_s;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float dc_bus_voltage_v;
    float electrical_angle_rad;
    float mechanical_speed_rad_s;
    uint32_t reserved;
} foc_encoder_realtime_input_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_encoder_realtime_input_t) == 48U,
               "foc_encoder_realtime_input_t ABI size mismatch");
_Static_assert(offsetof(foc_encoder_realtime_input_t, actual_dt_s) == 16U,
               "foc_encoder_realtime_input_t actual_dt_s offset mismatch");
_Static_assert(offsetof(foc_encoder_realtime_input_t, mechanical_speed_rad_s) == 40U,
               "foc_encoder_realtime_input_t speed offset mismatch");
#endif

uint32_t foc_rust_encoder_realtime_abi_version(void);

/* Starts directly in sensored closed loop. The caller must provide a calibrated
 * electrical angle, mechanical speed, coherent phase currents and protection. */
foc_status_t foc_rust_start_encoder_realtime(foc_rust_context_t *context,
                                             uint32_t platform_ready,
                                             float target_speed_rpm);

/* Sole realtime owner for the sensored path. Any physical/scheduler/hardware
 * fault clears output and latches the controller fault state. */
foc_status_t foc_rust_encoder_realtime_step(
    foc_rust_context_t *context,
    const foc_encoder_realtime_input_t *input,
    foc_output_t *output,
    foc_telemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* FOC_ENCODER_REALTIME_BRIDGE_H */
