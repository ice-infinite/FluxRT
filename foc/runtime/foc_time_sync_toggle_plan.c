#include "foc_time_sync_toggle_plan.h"

#include <limits.h>
#include <string.h>

uint32_t foc_time_sync_toggle_plan_build(
    const foc_time_sync_wire_symbol_t *symbols,
    uint32_t symbol_count,
    uint16_t first_compare_tick,
    uint16_t sentinel_gap_ticks,
    foc_time_sync_toggle_plan_t *plan)
{
    uint32_t symbol_index;
    uint32_t transition_index = 0U;
    uint32_t cursor;

    if (plan != NULL)
    {
        (void)memset(plan, 0, sizeof(*plan));
        plan->struct_size = sizeof(*plan);
        plan->version = FOC_TIME_SYNC_TOGGLE_PLAN_ABI_VERSION;
    }
    if ((symbols == NULL) || (plan == NULL) || (symbol_count == 0U) ||
        (symbol_count > FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS) ||
        (first_compare_tick == 0U) || (sentinel_gap_ticks == 0U))
    {
        return 0U;
    }

    cursor = (uint32_t)first_compare_tick;
    plan->first_compare_tick = first_compare_tick;
    for (symbol_index = 0U; symbol_index < symbol_count; ++symbol_index)
    {
        if ((symbols[symbol_index].high_ticks == 0U) ||
            (symbols[symbol_index].low_ticks == 0U))
        {
            return 0U;
        }

        cursor += (uint32_t)symbols[symbol_index].high_ticks;
        if (cursor > UINT16_MAX)
        {
            return 0U;
        }
        plan->dma_compare_ticks[transition_index++] = (uint16_t)cursor;

        if ((symbol_index + 1U) < symbol_count)
        {
            cursor += (uint32_t)symbols[symbol_index].low_ticks;
            if (cursor > UINT16_MAX)
            {
                return 0U;
            }
            plan->dma_compare_ticks[transition_index++] = (uint16_t)cursor;
        }
    }

    plan->final_falling_tick = (uint16_t)cursor;
    cursor += (uint32_t)sentinel_gap_ticks;
    if ((cursor > UINT16_MAX) ||
        (transition_index >= FOC_TIME_SYNC_TOGGLE_MAX_DMA_VALUES))
    {
        return 0U;
    }
    plan->dma_compare_ticks[transition_index++] = (uint16_t)cursor;
    plan->sentinel_compare_tick = (uint16_t)cursor;
    plan->physical_symbol_count = symbol_count;
    plan->transition_count = symbol_count * 2U;
    plan->dma_value_count = transition_index;
    return 1U;
}
