#ifndef FOC_REALTIME_TIMING_H
#define FOC_REALTIME_TIMING_H

/*
 * FluxRT —— 快环同拍时序记录的量纲与接口（纯 C，无硬件依赖）。
 * FluxRT - units and interface for same-tick fast-loop timing records.
 *
 * 职责 / Responsibility:
 *   - 定义"一个 ADC ISR 周期"内各分段的 cycles 记录格式；
 *   - 提供合法性校验和最坏值/峰值累计，供 `foc_status` 和主机测试使用。
 *   - Defines the per-ADC-ISR cycle record, its validity rule, and the
 *     worst-case/peak accumulation used by `foc_status` and host tests.
 *
 * 为什么要"同拍" / Why "same tick" matters:
 *   早期版本把 pre/control/post 三段各自跨所有拍取最大值再相加，得到的是一个
 *   从不存在的"合成拍"，会高估预算占用。现在每个样本自带 total_cycles，
 *   并被强制校验 total == pre + control + post；三个峰值仅用于定位热点，
 *   禁止相加。
 *   An earlier version took each of pre/control/post as an independent
 *   maximum over all ticks and added them, which describes a tick that never
 *   happened and overstates the budget. Each sample now carries
 *   total_cycles and is validated as total == pre + control + post. The three
 *   peak fields are for hotspot location only and must never be summed.
 *
 * 单位 / Units: 所有 *_cycles 均为 CPU 周期数（170 MHz 内核时钟）。
 *              All *_cycles fields are CPU cycles at the 170 MHz core clock.
 *
 * 边界 / Boundary: 本模块不读 DWT、不接触 TIM/ADC，纯粹是数据记录与校验，
 * 因此可以完整地在主机上做单元测试。
 * This module does not touch DWT, TIM or ADC; it is pure bookkeeping, so it is
 * fully unit-testable on the host.
 *
 * 参考 / Reference: docs/performance/2026-09-23-A0关联WCET基线.md
 */

#include <stdint.h>

/* 记录格式版本。字段变化时必须递增，否则旧数据会被当成新格式解析。
 * Record format version. Bump on any field change, otherwise stale data is
 * silently parsed as the new layout. */
#define FOC_REALTIME_TIMING_VERSION (2UL)

/*
 * 单个控制拍的时序样本。
 * Timing sample for one control tick.
 *
 * step 为控制器步计数，用于把时序与 trace 样本对齐。
 * `step` is the controller step counter and aligns timing with trace samples.
 */
typedef struct
{
    /* 控制器步序号 / Controller step index. */
    uint32_t step;
    /* 完整 ISR 区间 [cycles]：ADC ISR 入口到 ADC 标志清理之后。
     * Full ISR span [cycles]: from ISR entry to after the ADC flags are cleared. */
    uint32_t total_cycles;
    /* 快环调用之前：IRQ 入口、电流重构、保护检查 [cycles]。
     * Before the fast-loop call: IRQ entry, current reconstruction, protection. */
    uint32_t precontrol_cycles;
    /* 一次 foc_rust_realtime_step() [cycles] / One fast-loop call [cycles]. */
    uint32_t control_cycles;
    /* 快环之后：输出复核、CCR 更新、trace 采样、标志清理 [cycles]。
     * After the fast loop: output recheck, CCR update, trace, flag clear. */
    uint32_t postcontrol_cycles;
    /* 本拍 trace 是否启用（0/1），用于区分调试开销 / Whether trace was on (0/1). */
    uint32_t trace_enabled;
    /* 本拍是否真的写入了一个 trace 样本（0/1），受分频控制。
     * Whether a trace sample was actually stored this tick (0/1). */
    uint32_t trace_sampled;
} foc_realtime_timing_sample_t;

/*
 * 累计统计。`wcet` 保存已观测到的最坏**完整拍**，三个 peak 保存各分段
 * 的独立最大值。
 * Accumulated statistics. `wcet` holds the worst complete tick observed;
 * the three peak fields hold independent per-segment maxima.
 */
typedef struct
{
    /* 结构体尺寸，用于 ABI 自检 / Struct size for ABI self-check. */
    uint32_t struct_size;
    /* 记录格式版本 / Record format version. */
    uint32_t version;
    /* 已接受的合法样本数 / Number of accepted valid samples. */
    uint32_t sample_count;
    /* 被拒绝的非法样本数；持续增长说明计时边界写错了。
     * Rejected samples; sustained growth means the timing boundaries are wrong. */
    uint32_t invalid_sample_count;
    /* 最坏完整拍的完整样本 / The full sample of the worst complete tick. */
    foc_realtime_timing_sample_t wcet;
    /* precontrol 段独立峰值 [cycles]，仅用于定位热点，禁止与其他 peak 相加。
     * Independent precontrol peak [cycles]; for hotspot location only. */
    uint32_t peak_precontrol_cycles;
    /* control 段独立峰值 [cycles]，仅用于定位热点 / Independent control peak. */
    uint32_t peak_control_cycles;
    /* postcontrol 段独立峰值 [cycles]，仅用于定位热点 / Independent postcontrol peak. */
    uint32_t peak_postcontrol_cycles;
} foc_realtime_timing_stats_t;

/*
 * 清零统计并写入 struct_size / version。
 * Clears the statistics and writes struct_size / version.
 *
 * 参数 stats 允许为 NULL（空操作）。
 * `stats` may be NULL; the call is then a no-op.
 */
void foc_realtime_timing_reset(foc_realtime_timing_stats_t *stats);

/*
 * 判断一个样本是否自洽。
 * Checks whether one sample is self-consistent.
 *
 * 返回 / Returns: 1 合法，0 非法。非法条件为指针为空、trace 标志非 0/1、
 * trace_sampled 超过 trace_enabled、或 total != pre + control + post。
 * 1 if valid, 0 otherwise. Invalid when the pointer is NULL, the trace flags
 * are not 0/1, trace_sampled exceeds trace_enabled, or the three segments do
 * not add up to total_cycles.
 */
uint32_t foc_realtime_timing_sample_is_valid(
    const foc_realtime_timing_sample_t *sample);

/*
 * 记录一个样本并更新累计统计。
 * Records one sample and updates the accumulated statistics.
 *
 * 版本或 struct_size 不匹配时会先自动 reset，避免把旧格式数据混入统计。
 * Auto-resets first when version or struct_size disagree, so stale-format data
 * cannot be mixed into the statistics.
 *
 * 返回 / Returns: 1 表示本样本成为新的最坏完整拍，0 表示未替换或样本非法。
 * 1 if this sample became the new worst complete tick, 0 otherwise.
 */
uint32_t foc_realtime_timing_record(
    foc_realtime_timing_stats_t *stats,
    const foc_realtime_timing_sample_t *sample);

#endif
