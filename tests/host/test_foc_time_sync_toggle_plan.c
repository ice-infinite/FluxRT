#include "foc_time_sync_toggle_plan.h"

#include <assert.h>
#include <string.h>

static void simple_plan_has_exact_absolute_transitions(void)
{
    const foc_time_sync_wire_symbol_t symbols[2] = {
        {2U, 3U},
        {4U, 5U},
    };
    foc_time_sync_toggle_plan_t plan;

    assert(foc_time_sync_toggle_plan_build(
               symbols, 2U, 10U, 7U, &plan) == 1U);
    assert(plan.struct_size == sizeof(plan));
    assert(plan.version == FOC_TIME_SYNC_TOGGLE_PLAN_ABI_VERSION);
    assert(plan.physical_symbol_count == 2U);
    assert(plan.transition_count == 4U);
    assert(plan.dma_value_count == 4U);
    assert(plan.first_compare_tick == 10U);
    assert(plan.dma_compare_ticks[0] == 12U);
    assert(plan.dma_compare_ticks[1] == 15U);
    assert(plan.dma_compare_ticks[2] == 19U);
    assert(plan.final_falling_tick == 19U);
    assert(plan.dma_compare_ticks[3] == 26U);
    assert(plan.sentinel_compare_tick == 26U);
}

static void full_wire_frame_and_guard_fit_one_16_bit_timer_epoch(void)
{
    foc_time_sync_edge_identity_t identity = {
        sizeof(foc_time_sync_edge_identity_t),
        FOC_TIME_SYNC_IDENTITY_VERSION,
        0x10203040UL,
        7U,
        0x55667788UL,
        FOC_TIME_SYNC_FLAG_RISING_EDGE |
            FOC_TIME_SYNC_FLAG_IDENTITY_VERIFIED,
    };
    const foc_time_sync_wire_timing_t timing = {
        sizeof(foc_time_sync_wire_timing_t),
        FOC_TIME_SYNC_WIRE_ABI_VERSION,
        20U,
        40U,
        20U,
        80U,
        40U,
        3U,
    };
    foc_time_sync_wire_symbol_t symbols[
        FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS];
    foc_time_sync_toggle_plan_t plan;
    uint32_t symbol_count = 0U;
    uint32_t index;

    assert(foc_time_sync_wire_encode(
               &identity,
               &timing,
               symbols,
               FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
               &symbol_count) == 1U);
    assert(symbol_count == FOC_TIME_SYNC_WIRE_SYMBOL_COUNT);
    symbols[symbol_count].high_ticks = 10U;
    symbols[symbol_count].low_ticks = 20U;
    ++symbol_count;
    assert(foc_time_sync_toggle_plan_build(
               symbols, symbol_count, 100U, 100U, &plan) == 1U);
    assert(plan.physical_symbol_count ==
           FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS);
    assert(plan.transition_count ==
           FOC_TIME_SYNC_TOGGLE_MAX_TRANSITIONS);
    assert(plan.dma_value_count ==
           FOC_TIME_SYNC_TOGGLE_MAX_DMA_VALUES);
    assert(plan.final_falling_tick < plan.sentinel_compare_tick);
    for (index = 1U; index < plan.dma_value_count; ++index)
    {
        assert(plan.dma_compare_ticks[index] >
               plan.dma_compare_ticks[index - 1U]);
    }
}

static void invalid_or_wrapping_plans_fail_closed(void)
{
    foc_time_sync_wire_symbol_t symbols[
        FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS + 1U];
    foc_time_sync_toggle_plan_t plan;

    (void)memset(symbols, 0, sizeof(symbols));
    symbols[0].high_ticks = 1U;
    symbols[0].low_ticks = 1U;
    assert(foc_time_sync_toggle_plan_build(
               NULL, 1U, 1U, 1U, &plan) == 0U);
    assert(foc_time_sync_toggle_plan_build(
               symbols, 0U, 1U, 1U, &plan) == 0U);
    assert(foc_time_sync_toggle_plan_build(
               symbols,
               FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS + 1U,
               1U,
               1U,
               &plan) == 0U);
    assert(foc_time_sync_toggle_plan_build(
               symbols, 1U, 0U, 1U, &plan) == 0U);
    assert(foc_time_sync_toggle_plan_build(
               symbols, 1U, 1U, 0U, &plan) == 0U);
    symbols[0].high_ticks = 0U;
    assert(foc_time_sync_toggle_plan_build(
               symbols, 1U, 1U, 1U, &plan) == 0U);
    symbols[0].high_ticks = UINT16_MAX;
    assert(foc_time_sync_toggle_plan_build(
               symbols, 1U, 1U, 1U, &plan) == 0U);
    assert(plan.physical_symbol_count == 0U);
}

int main(void)
{
    simple_plan_has_exact_absolute_transitions();
    full_wire_frame_and_guard_fit_one_16_bit_timer_epoch();
    invalid_or_wrapping_plans_fail_closed();
    return 0;
}
