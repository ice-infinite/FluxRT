#ifndef FOC_TIME_SYNC_TOGGLE_PLAN_H
#define FOC_TIME_SYNC_TOGGLE_PLAN_H

#include <stdint.h>

#include "foc_time_sync_wire.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A timer output-compare channel toggles the pin at every compare event.  The
 * first compare is programmed directly; DMA supplies all later transitions
 * plus one never-executed sentinel value so transfer-complete coincides with
 * the final falling edge. */
#define FOC_TIME_SYNC_TOGGLE_PLAN_ABI_VERSION       (0x00010000UL)
#define FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS  \
    (FOC_TIME_SYNC_WIRE_SYMBOL_COUNT + 1UL)
#define FOC_TIME_SYNC_TOGGLE_MAX_TRANSITIONS        \
    (FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS * 2UL)
#define FOC_TIME_SYNC_TOGGLE_MAX_DMA_VALUES         \
    (FOC_TIME_SYNC_TOGGLE_MAX_TRANSITIONS)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t physical_symbol_count;
    uint32_t transition_count;
    uint32_t dma_value_count;
    uint16_t first_compare_tick;
    uint16_t final_falling_tick;
    uint16_t sentinel_compare_tick;
    uint16_t reserved;
    uint16_t dma_compare_ticks[FOC_TIME_SYNC_TOGGLE_MAX_DMA_VALUES];
} foc_time_sync_toggle_plan_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_time_sync_toggle_plan_t) == 676U,
               "time-sync toggle plan size mismatch");
#endif

uint32_t foc_time_sync_toggle_plan_build(
    const foc_time_sync_wire_symbol_t *symbols,
    uint32_t symbol_count,
    uint16_t first_compare_tick,
    uint16_t sentinel_gap_ticks,
    foc_time_sync_toggle_plan_t *plan);

#ifdef __cplusplus
}
#endif

#endif /* FOC_TIME_SYNC_TOGGLE_PLAN_H */
