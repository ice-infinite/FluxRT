#ifndef FOC_MOTION_ADAPTER_H
#define FOC_MOTION_ADAPTER_H

/* Default-off C owner for the Rust motion sub-ABI. No HAL/RTOS dependency. */

#include <stdbool.h>
#include <stdint.h>

#include "foc_motion_bridge.h"

typedef struct
{
    uint32_t initialized;
    uint32_t configured;
    uint32_t enabled;
    foc_motion_status_t last_status;
    foc_motion_context_t rust_context;
} foc_motion_adapter_t;

foc_motion_status_t foc_motion_adapter_init(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config);
foc_motion_status_t foc_motion_adapter_apply_config(
    foc_motion_adapter_t *adapter,
    const foc_config_bundle_t *config,
    const foc_config_apply_guard_t *guard);
foc_motion_status_t foc_motion_adapter_enable(
    foc_motion_adapter_t *adapter,
    const foc_config_apply_guard_t *guard);
foc_motion_status_t foc_motion_adapter_disable(foc_motion_adapter_t *adapter);
foc_motion_status_t foc_motion_adapter_step(
    foc_motion_adapter_t *adapter,
    const foc_product_command_t *command,
    const foc_motion_feedback_t *feedback,
    foc_motion_output_t *output);

#endif /* FOC_MOTION_ADAPTER_H */
