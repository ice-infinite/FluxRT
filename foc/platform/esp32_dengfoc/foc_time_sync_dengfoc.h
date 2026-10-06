#ifndef FOC_TIME_SYNC_DENGFOC_H
#define FOC_TIME_SYNC_DENGFOC_H

#include "foc_board_dengfoc_v04.h"
#include "foc_time_sync.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Truth-side binding. The AS5600 sample remains a separate stream; this
 * adapter records only a verified shared edge in the DengFOC local clock. */
uint32_t foc_time_sync_dengfoc_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr);

/* Resource arbitration for the optional truth-side synchronization input.
 * Power-stage PWM/current pins are never reusable.  An I2C pin may only be
 * reused when its whole Axis is disabled; this is how the diagnostic target
 * can use the physically exposed Axis-1 SDA connector without disturbing the
 * Axis-0 AS5600 truth source.  This function proves only logical non-conflict,
 * not connector pin order or electrical continuity. */
#define FOC_TIME_SYNC_DENGFOC_GPIO_UNCONFIGURED (UINT32_MAX)

uint32_t foc_time_sync_dengfoc_input_resource_valid(
    const foc_dengfoc_board_profile_t *board,
    uint32_t enabled_axis_mask,
    uint32_t input_gpio);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TIME_SYNC_DENGFOC_H */
