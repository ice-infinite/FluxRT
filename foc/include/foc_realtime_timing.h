#ifndef FOC_REALTIME_TIMING_H
#define FOC_REALTIME_TIMING_H

#include <stdint.h>

#define FOC_REALTIME_TIMING_VERSION (1UL)

typedef struct
{
    uint32_t step;
    uint32_t total_cycles;
    uint32_t precontrol_cycles;
    uint32_t control_cycles;
    uint32_t postcontrol_cycles;
    uint32_t trace_enabled;
    uint32_t trace_sampled;
} foc_realtime_timing_sample_t;

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sample_count;
    uint32_t invalid_sample_count;
    foc_realtime_timing_sample_t wcet;
    uint32_t peak_precontrol_cycles;
    uint32_t peak_control_cycles;
    uint32_t peak_postcontrol_cycles;
} foc_realtime_timing_stats_t;

void foc_realtime_timing_reset(foc_realtime_timing_stats_t *stats);
uint32_t foc_realtime_timing_sample_is_valid(
    const foc_realtime_timing_sample_t *sample);
uint32_t foc_realtime_timing_record(
    foc_realtime_timing_stats_t *stats,
    const foc_realtime_timing_sample_t *sample);

#endif
