/*
 * FluxRT —— 快环同拍时序记录的实现。
 * FluxRT - implementation of the same-tick fast-loop timing records.
 *
 * 本文件只做数据累计与校验，不读 DWT、不碰 TIM/ADC，因此可以在主机上
 * 完整单元测试（tests/host/test_foc_platform.c）。
 * This file only accumulates and validates data. It does not read DWT and does
 * not touch TIM/ADC, so it is fully unit-testable on the host.
 *
 * 调用上下文 / Call context:
 *   foc_realtime_timing_record() 由 ADC ISR 每拍调用一次，因此实现中
 *   不使用动态分配、不加锁、不做除法。
 *   foc_realtime_timing_record() is called once per tick from the ADC ISR; the
 *   implementation therefore performs no allocation, no locking and no
 *   division.
 *
 * 参考 / Reference: docs/performance/2026-09-23-A0关联WCET基线.md
 */

#include "foc_realtime_timing.h"

#include <limits.h>
#include <string.h>

/*
 * 饱和自增，防止 wrap 让"最坏值"看起来变小。
 * Saturating increment, so a wrap cannot make a worst case look smaller.
 *
 * 计数器溢出后回绕会把一个大值变成小值，从而使最坏值统计失真；这里宁可
 * 停在 UINT32_MAX（170 MHz 下约 25 秒才可能到达，正常一次运行远小于此）。
 * A wrapping counter would turn a large value into a small one and corrupt the
 * worst-case statistics. Stopping at UINT32_MAX is safe: at 170 MHz that takes
 * about 25 seconds of continuous ticking, far beyond a normal run.
 */
static void foc_realtime_timing_increment(uint32_t *value)
{
    if (*value != UINT32_MAX)
    {
        ++(*value);
    }
}

/*
 * 清零统计并把 ABI 自检字段写成当前值。
 * Clears the statistics and writes the current ABI self-check fields.
 *
 * 参数 stats 为 NULL 时直接返回 / Returns immediately when `stats` is NULL.
 */
void foc_realtime_timing_reset(foc_realtime_timing_stats_t *stats)
{
    if (stats == 0)
    {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    /* 先清零再写版本，顺序不能颠倒：record() 用这两个字段判断是否需要重置。
     * Write the version after the clear; record() keys its auto-reset on them. */
    stats->struct_size = sizeof(*stats);
    stats->version = FOC_REALTIME_TIMING_VERSION;
}

/*
 * 校验一个时序样本是否自洽。
 * Validates that one timing sample is self-consistent.
 *
 * 校验收敛为一条恒等式 total == pre + control + post。这是本模块的核心不变量：
 * 只要它成立，三段就必然来自同一拍，pre/control/post 才可以放在一起比较。
 * The check reduces to one identity: total == pre + control + post. That is the
 * core invariant of this module. While it holds, the three segments provably
 * come from the same tick and may be compared with each other.
 *
 * 校验顺序经过安排，全部用减法避免加法溢出。
 * The order is arranged so that only subtractions are used and no addition can
 * overflow.
 *
 * 返回 / Returns: 1 合法，0 非法。
 */
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
    /* 先减 precontrol，再逐段减，任一步借位即为非法。
     * Subtract precontrol first, then each remaining segment; any borrow is
     * a validation failure. */
    remaining = sample->total_cycles - sample->precontrol_cycles;
    if (sample->control_cycles > remaining)
    {
        return 0U;
    }
    remaining -= sample->control_cycles;
    return (sample->postcontrol_cycles == remaining) ? 1U : 0U;
}

/*
 * 记录一个样本，更新最坏完整拍与三个分段峰值。
 * Records one sample and updates the worst complete tick plus the three peaks.
 *
 * 版本或 struct_size 不匹配说明 stats 来自旧固件或未初始化，先自动重置；
 * 否则会把不同格式的数据累计到一起，得到无法解释的时序结论。
 * A version or struct_size mismatch means `stats` came from older firmware or
 * was never initialized; auto-reset first, otherwise different formats would be
 * accumulated together and the timing conclusions would be unexplainable.
 *
 * 返回 / Returns: 1 表示本样本替换了 wcet，0 表示未替换或样本被拒绝。
 */
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
    /* 非法样本单独计数，不参与最坏值。持续增长本身就是一条诊断信息：
     * 它意味着 ISR 里的计时边界被改坏了，而不是控制变差了。
     * Invalid samples are counted separately and never feed the worst case.
     * Sustained growth is itself a diagnosis: the ISR timing boundaries are
     * broken, the control did not get worse. */
    if (foc_realtime_timing_sample_is_valid(sample) == 0U)
    {
        foc_realtime_timing_increment(&stats->invalid_sample_count);
        return 0U;
    }

    /* 用 sample_count == 0 处理"首个样本必胜"的情况，避免把 wcet 的
     * 零初始化值当成一次真实测量。
     * sample_count == 0 handles the "first sample always wins" case, so the
     * zero-initialised wcet is never mistaken for a real measurement. */
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
    /* 整体赋值而非逐字段，保证 wcet 的三个分段来自同一拍。
     * Assign the whole struct, not field by field, so that wcet's three
     * segments are guaranteed to come from the same tick. */
    if (replace_wcet != 0U)
    {
        stats->wcet = *sample;
    }
    return replace_wcet;
}
