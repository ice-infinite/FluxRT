#ifndef FOC_MATH_ACCEL_H
#define FOC_MATH_ACCEL_H

#include "foc_build_profile.h"

/*
 * FluxRT —— 快环硬件数学加速的最小 C/Rust 契约。
 * FluxRT - minimal C/Rust contract for fast-loop hardware math acceleration.
 *
 * 职责 / Responsibility:
 *   - 把"要不要用片上加速器"这件事限制在平台层；
 *   - Rust 算法只看到本头文件的三个函数，不认识 CORDIC、FPU 或任何寄存器。
 *   - Confines the accelerator decision to the platform layer. The Rust
 *     algorithms see only these three functions and know nothing about CORDIC,
 *     the FPU or any register.
 *
 * 失败语义（重要）/ Failure semantics (important):
 *   返回 0 表示"本次加速不可用"，**不是控制故障**。Rust bridge 会在同一次
 *   控制采样内立即用 `CpuMath` 重算同一结果，因此启用硬件后端不等于失去
 *   软件实现。
 *   A return value of 0 means "the accelerator is unavailable for this call",
 *   NOT a control fault. The Rust bridge immediately recomputes the same result
 *   with `CpuMath` inside the same control sample, so enabling a hardware
 *   backend never removes the software implementation.
 *
 * 量纲与 Q 格式 / Units and Q format:
 *   角度用弧度 [rad]，模长无量纲，sin/cos 输出为 [-1, 1] 浮点。
 *   定点缩放（Q1.15 / Q1.31）是**平台适配器的内部细节**，不得泄漏到
 *   `foc-control` 或算法库；换 MCU 时只需要重新实现本头文件。
 *   Angles are [rad], magnitudes are dimensionless and sin/cos outputs are
 *   floats in [-1, 1]. Fixed-point scaling (Q1.15 / Q1.31) is an internal
 *   detail of the platform adapter and must not leak into `foc-control` or the
 *   algorithm crate; porting to another MCU only reimplements this header.
 *
 * 实时约束 / Real-time constraints:
 *   三个函数都会被 12 kHz 的 ADC ISR 调用。实现中禁止动态分配、禁止阻塞、
 *   禁止日志。共享外设（如 CORDIC）必须由本层自行做互斥，不能在快环里
 *   等待 RTOS 互斥量。
 *   All three are called from the 12 kHz ADC ISR. Implementations must not
 *   allocate, block or log. A shared peripheral such as CORDIC must be
 *   arbitrated here, never by waiting on an RTOS mutex in the fast loop.
 *
 * 参考 / Reference: docs/硬件数学加速与CPU回退.md
 */

#include <stdint.h>

/*
 * 当前生效的数学后端。
 * Currently active math backend.
 *
 * 该值只反映**初始化结果**，不保证每次调用都成功；单次调用仍可能回退。
 * This reflects the initialization result only. It does not guarantee that any
 * individual call succeeds; a single call may still fall back.
 */
typedef enum
{
    /* 纯 CPU（libm）实现，任何 MCU 都可用 / Portable CPU (libm) path. */
    FOC_MATH_BACKEND_CPU = 0,
    /* 片上加速器已使能 / On-chip accelerator is enabled. */
    FOC_MATH_BACKEND_CORDIC = 1
} foc_math_backend_t;

/*
 * 使能硬件数学加速并返回最终后端。
 * Enables the hardware math acceleration and returns the resulting backend.
 *
 * 必须在任何快环调用之前由平台初始化路径调用一次。重复调用是安全的。
 * Must be called once from the platform init path before any fast-loop call.
 * Calling it repeatedly is safe.
 *
 * 返回 / Returns: 实际生效的后端。返回 CPU 不表示错误，只表示没有加速器。
 * The backend actually in effect. Returning CPU is not an error; it only means
 * no accelerator is present.
 */
foc_math_backend_t foc_math_accel_init(void);

/*
 * 查询当前后端，用于启动日志和诊断。
 * Queries the current backend, for the boot log and diagnostics.
 */
foc_math_backend_t foc_math_accel_backend(void);

/*
 * 同时计算 sin 和 cos。
 * Computes sin and cos together.
 *
 * 之所以是"一次调用出两个值"，是因为 Park 与逆 Park 共用同一组 sin/cos，
 * 合并后每次控制采样只需要一次加速器事务。
 * The pairing exists because Park and inverse Park share one sin/cos pair;
 * combining them costs a single accelerator transaction per control sample.
 *
 * 参数 / Parameters:
 *   angle_rad [rad]  输入角度 / input angle
 *   sin_out          [out] sin，[-1, 1] / sin in [-1, 1]
 *   cos_out          [out] cos，[-1, 1] / cos in [-1, 1]
 *
 * 返回 / Returns: 1 成功；0 表示不可用或输入非法，此时两个输出已被写成 0，
 * 调用方必须回退到 CPU 实现。
 * 1 on success. 0 means unavailable or invalid input; both outputs are set to 0
 * and the caller must fall back to the CPU implementation.
 */
uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out);

/*
 * 计算二维矢量模长 sqrt(x^2 + y^2)。
 * Computes the 2-D vector magnitude sqrt(x^2 + y^2).
 *
 * 用于电流环的电压圆限幅：只有模长超过母线允许值时才需要缩放。
 * Used by the current loop's voltage circle limit: scaling is needed only when
 * the magnitude exceeds what the bus voltage allows.
 *
 * 参数 / Parameters:
 *   x, y            输入分量，无量纲 / input components, dimensionless
 *   magnitude_out   [out] 模长，非负 / non-negative magnitude
 *
 * 返回 / Returns: 1 成功；0 表示不可用或输入非法，此时输出被写成 0，
 * 调用方必须回退。
 * 1 on success. 0 means unavailable or invalid input; the output is set to 0
 * and the caller must fall back.
 */
uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out);

/*
 * 计算 atan2(y, x)，结果归一化到 [0, 2*pi)。
 * Computes atan2(y, x), normalised to [0, 2*pi).
 *
 * 用于把观测器估计的反电势矢量转成电角度。
 * Used to turn the observer's estimated back-EMF vector into an electrical
 * angle.
 *
 * 参数 / Parameters:
 *   y, x           输入分量 / input components
 *   angle_out      [out] 角度 [rad]，[0, 2*pi) / angle in [0, 2*pi)
 *
 * 返回 / Returns: 1 成功；0 表示不可用或输入非法，此时输出被写成 0，调用方
 * 必须回退。零矢量按约定成功返回角度 0；观察器是否采用该角度由幅值可靠性门决定。
 * 1 on success. 0 means unavailable or invalid input; the output is set to 0
 * and the caller must fall back. By convention a zero vector succeeds with
 * angle 0; the observer's magnitude reliability gate decides whether to use it.
 */
uint32_t foc_math_accel_atan2(float y, float x, float *angle_out);

/* 内部诊断操作编号；即使健康功能关闭，平台实现也用它把注入分支优化成常量 0。
 * Internal diagnostic operation IDs. The platform implementation also uses
 * them when health diagnostics are disabled, where injection folds to zero. */
#define FOC_MATH_INJECT_OPERATION_NONE     (0UL)
#define FOC_MATH_INJECT_OPERATION_SIN_COS  (1UL)
#define FOC_MATH_INJECT_OPERATION_ATAN2    (2UL)

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
/*
 * Diagnostic 数学后端健康快照。它不属于 Rust 实时 ABI，只用于回答三个问题：
 * 硬件调用是否成功、是否要求 CpuMath 回退、以及 CORDIC 未就绪后是否已复位恢复。
 * Diagnostic math-backend health snapshot. This is not part of the Rust realtime
 * ABI; it answers whether hardware calls succeeded, required CpuMath fallback,
 * or recovered the CORDIC after a not-ready result.
 */
#define FOC_MATH_HEALTH_VERSION            (1UL)

typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t sin_cos_calls;
    uint32_t sin_cos_successes;
    uint32_t sin_cos_fallback_required;
    uint32_t sin_cos_not_ready;
    uint32_t magnitude_calls;
    uint32_t magnitude_successes;
    uint32_t magnitude_fallback_required;
    uint32_t atan2_calls;
    uint32_t atan2_successes;
    uint32_t atan2_fallback_required;
    uint32_t atan2_not_ready;
    uint32_t cordic_recoveries;
    uint32_t injections_requested;
    uint32_t injections_consumed;
    uint32_t pending_injection_operation;
} foc_math_health_t;

/* 读取或清零计数；应用层必须在停机后调用，才能得到一致快照。
 * Gets or clears the counters. The application must call these while disarmed
 * to obtain a coherent snapshot. */
uint32_t foc_math_accel_health_get(foc_math_health_t *result);
void foc_math_accel_health_reset(void);

/*
 * 让指定的下一次 CORDIC 运算模拟一次 RRDY 未就绪。注入只消费一次；失败路径
 * 必须复位 CORDIC 并返回 0，使 Rust 在同一控制拍使用 CpuMath 重算。
 * Makes the next selected CORDIC operation simulate one not-ready RRDY result.
 * The injection is consumed once; the failure path resets CORDIC and returns 0,
 * making Rust recompute through CpuMath in the same control sample.
 */
uint32_t foc_math_accel_inject_not_ready_once(uint32_t operation);
#endif

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_BENCHMARK)
/* Diagnostic 固件的离线数学基准版本。字段或缩放改变时必须递增。
 * Version of the stopped-state math benchmark record in Diagnostic firmware. */
#define FOC_MATH_BENCHMARK_VERSION       (3UL)
#define FOC_MATH_BENCHMARK_VECTOR_COUNT  (16UL)
#define FOC_MATH_BENCHMARK_MAX_REPEATS   (128UL)
#define FOC_MATH_BENCHMARK_FIXED_NOPS    (6UL)

/* 一个运算的隔离调用周期统计。total_cycles 在最大允许重复次数下不会溢出。
 * Isolated-call cycle statistics for one operation. total_cycles cannot
 * overflow at the maximum supported repeat count. */
typedef struct
{
    uint32_t calls;
    uint32_t successes;
    uint32_t failures;
    uint32_t minimum_cycles;
    uint32_t maximum_cycles;
    uint32_t total_cycles;
} foc_math_benchmark_stat_t;

/* 固定 16 向量的数值误差与周期结果，只用于停机诊断，不属于实时 ABI。
 * Numeric error and cycle results for 16 fixed vectors. This is stopped-state
 * diagnostics only and is not part of the Rust realtime ABI. */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t vector_count;
    uint32_t repeats;
    foc_math_benchmark_stat_t measurement_overhead;
    /* 当前默认的有界 RRDY 轮询参考。/ Current bounded-RRDY polling reference. */
    foc_math_benchmark_stat_t sin_cos;
    foc_math_benchmark_stat_t atan2;
    uint32_t maximum_sin_cos_error_ppb;
    uint32_t maximum_atan2_error_urad;
    /* 6 NOP + 单次 RRDY 检查候选；失败时仍拒绝读 RDATA。
     * Six NOPs plus one RRDY check; a not-ready result is still rejected. */
    foc_math_benchmark_stat_t fixed_delay_sin_cos;
    foc_math_benchmark_stat_t fixed_delay_atan2;
    uint32_t maximum_fixed_delay_sin_cos_error_ppb;
    uint32_t maximum_fixed_delay_atan2_error_urad;
    uint32_t fixed_delay_nops;
    uint32_t realtime_uses_fixed_delay;
    /* 可移植 Rust CPU 快速近似；只有专项 benchmark 配置才填充。
     * Portable Rust CPU fast approximation, populated only by the dedicated
     * benchmark configuration. */
    foc_math_benchmark_stat_t fast_approx_sin_cos;
    foc_math_benchmark_stat_t fast_approx_atan2;
    uint32_t maximum_fast_approx_sin_cos_error_ppb;
    uint32_t maximum_fast_approx_atan2_error_urad;
    uint32_t fast_approx_available;
    uint32_t realtime_uses_fast_approx;
    uint32_t invalid_rejections;
    uint32_t invalid_tests;
} foc_math_benchmark_result_t;

/*
 * 在功率级关闭时对硬件后端做固定向量对拍与隔离周期测量。
 * Benchmarks the hardware backend with fixed vectors while the power stage is off.
 *
 * repeats 是每个向量的重复次数，范围 1..128。每次调用只在单个数学调用期间
 * 短暂屏蔽中断；应用层仍必须在调用前检查 REALTIME_ARMED 为 0。
 * repeats is per vector, 1..128. Interrupts are masked only for one math call at
 * a time; the application must still verify REALTIME_ARMED is clear first.
 *
 * 返回 / Returns: 1 表示基准已执行并填充 result；0 表示参数非法或后端不可用。
 * A successful benchmark may still report individual failures in the counters.
 */
uint32_t foc_math_accel_benchmark(foc_math_benchmark_result_t *result,
                                  uint32_t repeats);
#endif

#endif
