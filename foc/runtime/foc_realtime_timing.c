#include "foc_realtime_timing.h"

#include <limits.h>
#include <string.h>

static void foc_realtime_timing_increment(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

void foc_realtime_timing_reset(foc_realtime_timing_stats_t *stats)
{
    if (stats == 0)
    {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    stats->struct_size = sizeof(*stats);
    stats->version = FOC_REALTIME_TIMING_VERSION;
}

uint32_t foc_realtime_timing_sample_is_valid(
    const foc_realtime_timing_sample_t *sample)
{
    uint32_t remaining;

    if ((sample == 0) ||
        (sample->trace_enabled > 1U) ||
        (sample->trace_sampled > sample->trace_enabled) ||
        (sample->precontrol_cycles > sample->total_cycles))
    {
        return 0U;
    }
    remaining = sample->total_cycles - sample->precontrol_cycles;
    if (sample->control_cycles > remaining)
    {
        return 0U;
    }
    remaining -= sample->control_cycles;
    return (sample->postcontrol_cycles == remaining) ? 1U : 0U;
}

uint32_t foc_realtime_timing_record(
    foc_realtime_timing_stats_t *stats,
    const foc_realtime_timing_sample_t *sample)
{
    uint32_t replace_wcet;

    if (stats == 0)
    {
        return 0U;
    }
    if ((stats->struct_size != sizeof(*stats)) ||
        (stats->version != FOC_REALTIME_TIMING_VERSION))
    {
        foc_realtime_timing_reset(stats);
    }
    if (foc_realtime_timing_sample_is_valid(sample) == 0U)
    {
        foc_realtime_timing_increment(&stats->invalid_sample_count);
        return 0U;
    }

    replace_wcet = ((stats->sample_count == 0U) ||
                    (sample->total_cycles > stats->wcet.total_cycles)) ? 1U : 0U;
    foc_realtime_timing_increment(&stats->sample_count);
    if (sample->precontrol_cycles > stats->peak_precontrol_cycles)
    {
        stats->peak_precontrol_cycles = sample->precontrol_cycles;
    }
    if (sample->control_cycles > stats->peak_control_cycles)
    {
        stats->peak_control_cycles = sample->control_cycles;
    }
    if (sample->postcontrol_cycles > stats->peak_postcontrol_cycles)
    {
        stats->peak_postcontrol_cycles = sample->postcontrol_cycles;
    }
    if (replace_wcet != 0U)
    {
        stats->wcet = *sample;
    }
    return replace_wcet;
}
