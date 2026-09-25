/*
 * FluxRT —— STM32G431 快环硬件数学后端（CORDIC + FPU）。
 * FluxRT - STM32G431 fast-loop hardware math backend (CORDIC + FPU).
 *
 * 职责 / Responsibility:
 *   实现 foc_math_accel.h 的三个契约函数，把浮点角度/矢量转换成片上外设
 *   能接受的定点格式，再把结果还原成浮点。
 *   Implements the three contract functions from foc_math_accel.h: converts
 *   floats into the fixed-point form the on-chip peripheral accepts, then
 *   converts the results back.
 *
 * 后端分工 / Backend split:
 *   - sin/cos、atan2 走 CORDIC。M4F 没有硬件三角函数，软件 sinf/cosf 需要
 *     数百周期，CORDIC 一次事务更便宜。
 *     sin/cos and atan2 use CORDIC: the M4F has no hardware trig, and software
 *     sinf/cosf costs hundreds of cycles per call.
 *   - magnitude 走 FPU 的 VSQRT.F32。模长只需一次乘加加一次硬件开方，
 *     用 CORDIC 反而要额外付出 Q15 缩放、打包和一次中断临界区。
 *     magnitude uses the FPU's VSQRT.F32: it needs one multiply-add plus one
 *     hardware sqrt, whereas CORDIC would additionally cost Q15 scaling,
 *     packing and an interrupt-critical section.
 *
 * 中断与并发 / Interrupts and concurrency:
 *   CORDIC 是共享外设。每次事务都保存 PRIMASK、短暂屏蔽中断、完成寄存器
 *   读写后恢复调用前的中断状态，防止线程与 ISR 同时改写 CORDIC 配置。
 *   CORDIC is a shared peripheral. Each transaction saves PRIMASK, briefly
 *   masks interrupts, completes the register accesses, then restores the
 *   previous interrupt state, so a thread and an ISR cannot interleave.
 *
 * 无加速器时的行为 / Behaviour without the accelerator:
 *   文件末尾的 #else 分支把三个函数实现成"永远返回 0"，Rust bridge 会自动
 *   回落到 CpuMath。因此本文件在非 G431 目标上仍然必须能编译通过。
 *   The #else branch at the end implements all three as "always return 0", and
 *   the Rust bridge falls back to CpuMath. This file must therefore still
 *   compile on non-G431 targets.
 *
 * 参考 / Reference: docs/硬件数学加速与CPU回退.md
 */

#if defined(FOC_TARGET_STM32G431)
#include "rtconfig.h"
#endif

#include "foc_math_accel.h"
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && \
    defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
#include "foc_rust_bridge.h"
#endif

/* 只有同时满足"配置要求 CORDIC"和"当前是 G431"才编译硬件实现。
 * The hardware implementation is compiled only when CORDIC is configured AND
 * the target really is a G431. */
#if defined(FOC_MATH_BACKEND_STM32G4_CORDIC) && defined(STM32G431xx)

#include <float.h>
#include <limits.h>

#include "stm32g4xx.h"
#include "stm32g4xx_ll_cordic.h"

#define FOC_PI_F                    (3.14159265358979323846f)
#define FOC_TWO_PI_F                (2.0f * FOC_PI_F)
/* CORDIC 轮询上限。6-cycle 精度下正常几次就能就绪；64 次是"外设挂了"的
 * 上限，避免在快环里无限自旋。
 * Polling bound. With 6-cycle precision a few iterations normally suffice;
 * 64 is the "peripheral is dead" bound that keeps the fast loop from spinning
 * forever. */
#define FOC_CORDIC_TIMEOUT_LOOPS    (64U)
#define FOC_CORDIC_FIXED_DELAY_NOPS (6U)
/* Q1.31 的缩放因子 2^31，用于角度与 sin/cos 的定点往返。
 * Q1.31 scale factor 2^31, used for the angle and sin/cos round trip. */
#define FOC_Q31_SCALE_F             (2147483648.0f)

/* cosine 模式：一次写角度 + 一次写 0.5，读回 cos 和 sin 两个 Q1.31 结果。
 * 输入输出都用 32 位，精度 6 周期，无缩放。
 * cosine mode: write the angle then 0.5, read back cos and sin as two Q1.31
 * values. 32-bit in/out, 6-cycle precision, unscaled. */
#define FOC_CORDIC_CONFIG_COSSIN                                               \
    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES |                \
     LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_2 |           \
     LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* phase 模式：写 y、x 两个 Q1.31，读回 atan2(y, x) / pi，单个 Q1.31。
 * phase mode: write y and x as Q1.31, read back atan2(y, x) / pi as one Q1.31. */
#define FOC_CORDIC_CONFIG_PHASE                                                \
    (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES |                 \
     LL_CORDIC_SCALE_0 | LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |           \
     LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC 初始化失败时保持 CPU，Rust bridge 会走软件实现。
 * Stays at CPU when CORDIC init fails; the Rust bridge then uses software. */
static volatile foc_math_backend_t g_foc_math_backend = FOC_MATH_BACKEND_CPU;

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
/* 只在专项 Diagnostic 镜像中存在；普通 Diagnostic/Production 没有热路径计数开销。
 * Present only in the special Diagnostic image, leaving normal Diagnostic and
 * Production builds without hot-path counter writes. */
static volatile foc_math_health_t g_foc_math_health =
{
    .struct_size = sizeof(foc_math_health_t),
    .version = FOC_MATH_HEALTH_VERSION,
};

static uint32_t foc_math_consume_not_ready_injection(uint32_t operation,
                                                     uint32_t allow_injection)
{
    if ((allow_injection != 0U) &&
        (g_foc_math_health.pending_injection_operation == operation))
    {
        g_foc_math_health.pending_injection_operation =
            FOC_MATH_INJECT_OPERATION_NONE;
        ++g_foc_math_health.injections_consumed;
        return 1U;
    }
    return 0U;
}
#else
__attribute__((always_inline)) static inline uint32_t
foc_math_consume_not_ready_injection(uint32_t operation,
                                     uint32_t allow_injection)
{
    (void)operation;
    (void)allow_injection;
    return 0U;
}
#endif

/*
 * RRDY 超时后复位 CORDIC，丢弃可能稍后到达的旧结果。若只返回 0 而不复位，
 * 下一次事务可能把前一笔 RDATA 误当成自己的结果；这条恢复路径因此在所有档位保留。
 * Resets CORDIC after an RRDY failure, discarding any late result. Returning 0
 * without a reset could let the next transaction consume stale RDATA, so this
 * recovery path is retained in every build profile.
 */
static void foc_cordic_recover(void)
{
    RCC->AHB1RSTR |= RCC_AHB1RSTR_CORDICRST;
    __DSB();
    RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CORDICRST;
    (void)RCC->AHB1RSTR;
    __DSB();
    __ISB();
}

/*
 * 轮询 CORDIC 结果就绪（RRDY），带固定上限。
 * Polls the CORDIC result-ready flag with a fixed bound.
 *
 * 返回 / Returns: 1 已就绪，0 超时（调用方必须放弃本次事务并返回 0，
 * 让 Rust 回落到 CpuMath；绝不能返回未就绪的垃圾数据）。
 * 1 if ready, 0 on timeout. On timeout the caller must abandon the
 * transaction and return 0 so Rust falls back to CpuMath. Never return stale
 * data from an unfinished transaction.
 */
static uint32_t foc_cordic_wait_ready(void)
{
    uint32_t remaining = FOC_CORDIC_TIMEOUT_LOOPS;
    while (((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) && (remaining > 0U))
    {
        --remaining;
    }
    return (remaining > 0U) ? 1U : 0U;
}

/*
 * Diagnostic 候选：固定等待 6 个 CPU 指令周期，再只检查一次 RRDY。
 * Diagnostic candidate: wait six CPU instruction cycles, then check RRDY once.
 *
 * 这里没有像 ST 生成例程那样盲读 RDATA：若结果仍未就绪就返回失败，让 Rust
 * CpuMath 回退。这样候选可以减少轮询分支，却不会把旧 RDATA 当成新结果。
 * Unlike the generated ST example this never reads RDATA blindly. A not-ready
 * result fails into the Rust CpuMath fallback, avoiding stale data.
 */
static uint32_t foc_cordic_wait_ready_fixed_delay(void)
{
    __asm volatile (
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        ::: "memory");
    return ((CORDIC->CSR & CORDIC_CSR_RRDY) != 0U) ? 1U : 0U;
}

/* strategy=0 是有界轮询基线，strategy=1 是 6 NOP + 单次检查候选。
 * strategy=0 is bounded polling; strategy=1 is six NOPs plus one check. */
__attribute__((always_inline)) static inline uint32_t
foc_cordic_wait_ready_strategy(uint32_t strategy)
{
    return (strategy != 0U) ? foc_cordic_wait_ready_fixed_delay()
                            : foc_cordic_wait_ready();
}

/*
 * 按调用前保存的 PRIMASK 决定是否重新开中断。
 * Re-enables interrupts only if they were enabled before the critical section.
 *
 * 这段逻辑必须保留：如果本函数被从"已经关中断"的上下文调用（例如调试器
 * 或另一个临界区），无条件 __enable_irq() 会破坏外层的中断状态。
 * This matters: when called from a context that already had interrupts masked
 * (a debugger, or an enclosing critical section), an unconditional
 * __enable_irq() would corrupt the caller's interrupt state.
 */
static void foc_restore_interrupts(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/*
 * float -> Q1.31，带饱和。
 * float to Q1.31 with saturation.
 *
 * (int32_t)(1.0f * 2^31) 是未定义行为（溢出），因此先用 >= / <= 把边界
 * 单独处理，再对开区间做转换。
 * Casting 1.0f * 2^31 to int32_t is undefined behaviour, so the boundary is
 * handled separately and the cast is only applied to the open interval.
 */
static int32_t foc_float_to_q31(float value)
{
    if (value >= 1.0f)
    {
        return INT32_MAX;
    }
    if (value <= -1.0f)
    {
        return INT32_MIN;
    }
    return (int32_t)(value * FOC_Q31_SCALE_F);
}

/*
 * 硬件单指令开方（Cortex-M4F VSQRT.F32）。
 * Single-instruction hardware square root (Cortex-M4F VSQRT.F32).
 *
 * 用内联汇编而不是 sqrtf()：sqrtf 可能被链接到 libm 的通用实现，而
 * VSQRT.F32 是单指令、无分支、无表。arm-none-eabi-gcc 在
 * -mfpu=fpv4-sp-d16 -mfloat-abi=hard 下必然支持。
 * Inline assembly rather than sqrtf(): sqrtf may resolve to a generic libm
 * routine, whereas VSQRT.F32 is one instruction with no branch and no table.
 * arm-none-eabi-gcc always supports it under
 * -mfpu=fpv4-sp-d16 -mfloat-abi=hard.
 *
 * 注意 / Caveat: 这段代码绑定 GNU 编译器扩展；将来换 starm-clang 时必须
 * 重新验证。
 * This binds to a GNU compiler extension; it must be re-verified if the
 * toolchain changes to starm-clang.
 */
static float foc_fpu_sqrt(float value)
{
    float result;
    __asm volatile ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (value));
    return result;
}

/*
 * 使能 CORDIC 时钟并把后端切到 CORDIC。
 * Enables the CORDIC clock and switches the backend to CORDIC.
 *
 * 回读 AHB1ENR 是为了确认写时钟使能位真的生效，再回读一次寄存器以把
 * 写操作flush 掉；只写不读在某些总线上会被重排。
 * AHB1ENR is read back to confirm the clock-enable bit really took effect, and
 * the register is read again to flush the write; a write-only access may be
 * reordered on some buses.
 */
foc_math_backend_t foc_math_accel_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    (void)RCC->AHB1ENR;
    __DSB();
    if ((RCC->AHB1ENR & RCC_AHB1ENR_CORDICEN) != 0U)
    {
        g_foc_math_backend = FOC_MATH_BACKEND_CORDIC;
    }
    return g_foc_math_backend;
}

foc_math_backend_t foc_math_accel_backend(void)
{
    return g_foc_math_backend;
}

/*
 * CORDIC sin/cos。见 foc_math_accel.h 的契约说明。
 * CORDIC sin/cos. See the contract in foc_math_accel.h.
 *
 * 角度先归一化到 [-pi, pi]。CORDIC 的 cosine 函数以 pi 为单位输入，
 * 因此写 WDATA 时用 angle/pi。
 * The angle is first wrapped into [-pi, pi]. The CORDIC cosine function takes
 * its input in units of pi, hence angle/pi when writing WDATA.
 */
__attribute__((always_inline)) static inline uint32_t
foc_math_accel_sin_cos_impl(float angle_rad,
                            float *sin_out,
                            float *cos_out,
                            uint32_t fixed_delay,
                            uint32_t allow_injection,
                            uint32_t *not_ready_out)
{
    int32_t cos_q31;
    int32_t sin_q31;
    uint32_t primask;

    if (not_ready_out != 0)
    {
        *not_ready_out = 0U;
    }
    if ((sin_out == 0) || (cos_out == 0))
    {
        return 0U;
    }
    /* 先清零输出：任何失败路径返回时调用方看到的都是确定的 0，
     * 而不是未初始化栈值。
     * Zero the outputs first so that every failure path leaves a defined 0
     * rather than an uninitialised stack value. */
    *sin_out = 0.0f;
    *cos_out = 0.0f;
    /* 后端未就绪、输入非有限、或角度超出快速归一化范围时回退。
     * 用 !(x <= FLT_MAX && x >= -FLT_MAX) 而不是 isnan()：这个写法同时
     * 排除 NaN（比较恒假）和 Inf，并且不依赖 math.h。
     * Fall back when the backend is not ready, the input is not finite, or the
     * angle is outside the fast wrap range. The comparison form rejects both
     * NaN (all comparisons false) and Inf without including math.h. */
    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        !(angle_rad <= FLT_MAX && angle_rad >= -FLT_MAX) ||
        (angle_rad > (8.0f * FOC_PI_F)) || (angle_rad < (-8.0f * FOC_PI_F)))
    {
        return 0U;
    }

    /* 逐次减 2*pi 而不是用 fmodf：角度已在 8*pi 以内，最多两三次减法，
     * 比一次库函数调用便宜且无分支预测风险。
     * Repeated subtraction instead of fmodf: the angle is already within 8*pi,
     * so at most a couple of subtractions are needed, cheaper than a library
     * call. */
    while (angle_rad >= FOC_PI_F)
    {
        angle_rad -= FOC_TWO_PI_F;
    }
    while (angle_rad < -FOC_PI_F)
    {
        angle_rad += FOC_TWO_PI_F;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    CORDIC->CSR = FOC_CORDIC_CONFIG_COSSIN;
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(angle_rad / FOC_PI_F);
    /* 第二个参数固定为 1.0 的 Q1.31 值；cosine 模式需要一个"余弦参数"占位。
     * The second argument is the Q1.31 representation of 1.0; cosine mode needs
     * a cosine argument placeholder. */
    CORDIC->WDATA = (uint32_t)INT32_MAX;
    if ((foc_math_consume_not_ready_injection(
             FOC_MATH_INJECT_OPERATION_SIN_COS, allow_injection) != 0U) ||
        (foc_cordic_wait_ready_strategy(fixed_delay) == 0U))
    {
        foc_cordic_recover();
        if (not_ready_out != 0)
        {
            *not_ready_out = 1U;
        }
        foc_restore_interrupts(primask);
        return 0U;
    }
    /* 读回顺序必须与 CSR 中的 NBREAD_2 配置一致：先 cos 后 sin。
     * 顺序写反不会报错，只会让 Park 变换悄悄用错三角函数。
     * The read order must match NBREAD_2 in CSR: cos first, then sin. Getting
     * it backwards raises no error; it silently feeds the wrong trig function
     * into the Park transform. */
    cos_q31 = (int32_t)CORDIC->RDATA;
    sin_q31 = (int32_t)CORDIC->RDATA;
    foc_restore_interrupts(primask);

    *sin_out = (float)sin_q31 / FOC_Q31_SCALE_F;
    *cos_out = (float)cos_q31 / FOC_Q31_SCALE_F;
    return 1U;
}

uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    uint32_t result;
    uint32_t not_ready = 0U;

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && \
    defined(FOC_MATH_CORDIC_FIXED_DELAY_CANDIDATE)
    result = foc_math_accel_sin_cos_impl(angle_rad, sin_out, cos_out,
                                         1U, 1U, &not_ready);
#else
    result = foc_math_accel_sin_cos_impl(angle_rad, sin_out, cos_out,
                                         0U, 1U, &not_ready);
#endif
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_health.sin_cos_calls;
    if (result != 0U)
    {
        ++g_foc_math_health.sin_cos_successes;
    }
    else
    {
        ++g_foc_math_health.sin_cos_fallback_required;
    }
    if (not_ready != 0U)
    {
        ++g_foc_math_health.sin_cos_not_ready;
        ++g_foc_math_health.cordic_recoveries;
    }
#else
    (void)not_ready;
#endif
    return result;
}

/*
 * 矢量模长。见 foc_math_accel.h 的契约说明。
 * Vector magnitude. See the contract in foc_math_accel.h.
 *
 * 这里不使用 CORDIC：模长只需要一次乘加加一次硬件开方。改用 CORDIC 反而
 * 要额外付出 Q15 缩放、打包成 32 位、一次中断临界区和一次反缩放，
 * 在当前工程中实测更贵。
 * CORDIC is deliberately not used here: the magnitude needs only one
 * multiply-add plus one hardware sqrt. Using CORDIC would add Q15 scaling, 32-bit
 * packing, an interrupt-critical section and rescaling, which measured more
 * expensive on this target.
 */
uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out)
{
    float magnitude_squared;

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_health.magnitude_calls;
#endif
    if (magnitude_out == 0)
    {
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
        ++g_foc_math_health.magnitude_fallback_required;
#endif
        return 0U;
    }
    *magnitude_out = 0.0f;
    /* 硬浮点 STM32G431 目标始终有 VSQRT.F32 可用，因此这里不检查后端状态：
     * 即使 CORDIC 初始化失败，模长仍然可以正确计算。
     * The hard-float STM32G431 target always has VSQRT.F32, so the backend
     * state is not checked here: the magnitude is correct even when CORDIC
     * initialization failed. */
    if (!(x <= FLT_MAX && x >= -FLT_MAX) || !(y <= FLT_MAX && y >= -FLT_MAX))
    {
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
        ++g_foc_math_health.magnitude_fallback_required;
#endif
        return 0U;
    }

    /* 先算平方和再开方，避免对 x、y 分别开方。
     * Square first, then take one root, instead of two separate roots. */
    magnitude_squared = x * x + y * y;
    /* 平方和可能溢出为 +Inf；VSQRT.F32 对 Inf 返回 Inf，会让限幅逻辑
     * 拿到非有限值，因此这里显式拒绝。
     * The sum of squares can overflow to +Inf; VSQRT.F32 would then return Inf
     * and the limiter would receive a non-finite value, so it is rejected. */
    if (!(magnitude_squared <= FLT_MAX && magnitude_squared >= 0.0f))
    {
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
        ++g_foc_math_health.magnitude_fallback_required;
#endif
        return 0U;
    }
    *magnitude_out = foc_fpu_sqrt(magnitude_squared);
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_health.magnitude_successes;
#endif
    return 1U;
}

/*
 * CORDIC atan2，结果归一化到 [0, 2*pi)。
 * CORDIC atan2, result normalised to [0, 2*pi).
 *
 * 与 CORDIC 的 phase 函数一致，输出单位为 pi，因此最后乘 FOC_PI_F。
 * Matching the CORDIC phase function, the raw output is in units of pi, hence
 * the final multiplication by FOC_PI_F.
 */
__attribute__((always_inline)) static inline uint32_t
foc_math_accel_atan2_impl(float y,
                          float x,
                          float *angle_out,
                          uint32_t fixed_delay,
                          uint32_t allow_injection,
                          uint32_t *not_ready_out)
{
    float abs_x;
    float abs_y;
    float scale;
    int32_t angle_q31;
    uint32_t primask;

    if (not_ready_out != 0)
    {
        *not_ready_out = 0U;
    }
    if (angle_out == 0)
    {
        return 0U;
    }
    *angle_out = 0.0f;
    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        !(x <= FLT_MAX && x >= -FLT_MAX) || !(y <= FLT_MAX && y >= -FLT_MAX))
    {
        return 0U;
    }
    /* CORDIC 要求输入在 Q1.31 范围内，所以先按最大分量归一化，算完再还原。
     * 乘以 2.0 留出余量，避免最大分量刚好等于 1.0 时饱和。
     * CORDIC needs inputs inside Q1.31, so they are normalised by the largest
     * component and rescaled afterwards. The factor 2.0 leaves headroom so the
     * largest component does not saturate at exactly 1.0. */
    abs_x = (x < 0.0f) ? -x : x;
    abs_y = (y < 0.0f) ? -y : y;
    scale = (abs_x > abs_y) ? abs_x : abs_y;
    if (scale == 0.0f)
    {
        /* 零矢量没有确定角度。返回成功且角度为 0，让调用方自行决定是否
         * 使用；这与"加速器不可用"是两种不同情况。
         * A zero vector has no defined angle. Report success with angle 0 and
         * let the caller decide; this is distinct from "accelerator
         * unavailable". */
        return 1U;
    }
    scale *= 2.0f;

    primask = __get_PRIMASK();
    __disable_irq();
    CORDIC->CSR = FOC_CORDIC_CONFIG_PHASE;
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(x / scale);
    CORDIC->WDATA = (uint32_t)foc_float_to_q31(y / scale);
    if ((foc_math_consume_not_ready_injection(
             FOC_MATH_INJECT_OPERATION_ATAN2, allow_injection) != 0U) ||
        (foc_cordic_wait_ready_strategy(fixed_delay) == 0U))
    {
        foc_cordic_recover();
        if (not_ready_out != 0)
        {
            *not_ready_out = 1U;
        }
        foc_restore_interrupts(primask);
        return 0U;
    }
    angle_q31 = (int32_t)CORDIC->RDATA;
    foc_restore_interrupts(primask);
    *angle_out = ((float)angle_q31 / FOC_Q31_SCALE_F) * FOC_PI_F;
    /* CORDIC phase 输出范围是 (-pi, pi]，算法侧约定 [0, 2*pi)。
     * The CORDIC phase output is (-pi, pi]; the algorithm side expects
     * [0, 2*pi). */
    if (*angle_out < 0.0f)
    {
        *angle_out += FOC_TWO_PI_F;
    }
    return 1U;
}

uint32_t foc_math_accel_atan2(float y, float x, float *angle_out)
{
    uint32_t result;
    uint32_t not_ready = 0U;

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && \
    defined(FOC_MATH_CORDIC_FIXED_DELAY_CANDIDATE)
    result = foc_math_accel_atan2_impl(y, x, angle_out, 1U, 1U, &not_ready);
#else
    result = foc_math_accel_atan2_impl(y, x, angle_out, 0U, 1U, &not_ready);
#endif
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_health.atan2_calls;
    if (result != 0U)
    {
        ++g_foc_math_health.atan2_successes;
    }
    else
    {
        ++g_foc_math_health.atan2_fallback_required;
    }
    if (not_ready != 0U)
    {
        ++g_foc_math_health.atan2_not_ready;
        ++g_foc_math_health.cordic_recoveries;
    }
#else
    (void)not_ready;
#endif
    return result;
}

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
uint32_t foc_math_accel_health_get(foc_math_health_t *result)
{
    uint32_t primask;

    if (result == 0)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *result = g_foc_math_health;
    foc_restore_interrupts(primask);
    return 1U;
}

void foc_math_accel_health_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_foc_math_health = (foc_math_health_t)
    {
        .struct_size = sizeof(foc_math_health_t),
        .version = FOC_MATH_HEALTH_VERSION,
    };
    foc_restore_interrupts(primask);
}

uint32_t foc_math_accel_inject_not_ready_once(uint32_t operation)
{
    uint32_t primask;
    uint32_t accepted = 0U;

    if ((g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        ((operation != FOC_MATH_INJECT_OPERATION_SIN_COS) &&
         (operation != FOC_MATH_INJECT_OPERATION_ATAN2)))
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_foc_math_health.pending_injection_operation ==
        FOC_MATH_INJECT_OPERATION_NONE)
    {
        g_foc_math_health.pending_injection_operation = operation;
        ++g_foc_math_health.injections_requested;
        accepted = 1U;
    }
    foc_restore_interrupts(primask);
    return accepted;
}
#endif

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_BENCHMARK)

/* Diagnostic 停机基准使用的 16 个单位圆向量（间隔 pi/8）。参考值是编译期
 * 常量，不调用 libm，也不会把另一套三角函数实现拉进固件。
 * Sixteen unit-circle vectors at pi/8 spacing for the stopped-state Diagnostic
 * benchmark. Compile-time references avoid linking a second trig implementation. */
typedef struct
{
    float angle_rad;
    float sin_value;
    float cos_value;
} foc_math_reference_vector_t;

/* 同一专用镜像里强制保留两个策略，避免用两次构建的代码布局差异冒充性能差异。
 * Force both strategies into one special image so build-layout changes cannot
 * masquerade as an A/B performance difference. */
__attribute__((noinline)) static uint32_t
foc_math_benchmark_poll_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    return foc_math_accel_sin_cos_impl(angle_rad, sin_out, cos_out,
                                       0U, 0U, 0);
}

__attribute__((noinline)) static uint32_t
foc_math_benchmark_fixed_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    return foc_math_accel_sin_cos_impl(angle_rad, sin_out, cos_out,
                                       1U, 0U, 0);
}

__attribute__((noinline)) static uint32_t
foc_math_benchmark_poll_atan2(float y, float x, float *angle_out)
{
    return foc_math_accel_atan2_impl(y, x, angle_out, 0U, 0U, 0);
}

__attribute__((noinline)) static uint32_t
foc_math_benchmark_fixed_atan2(float y, float x, float *angle_out)
{
    return foc_math_accel_atan2_impl(y, x, angle_out, 1U, 0U, 0);
}

#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
/* 通过独立 noinline 包装器让 DWT 测到与 CORDIC 包装相同的 C 调用边界。
 * A separate noinline wrapper gives the DWT measurement the same C call
 * boundary as the CORDIC candidates. */
__attribute__((noinline)) static uint32_t
foc_math_benchmark_fast_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    return foc_rust_fast_math_sin_cos(angle_rad, sin_out, cos_out);
}

__attribute__((noinline)) static uint32_t
foc_math_benchmark_fast_atan2(float y, float x, float *angle_out)
{
    return foc_rust_fast_math_atan2(y, x, angle_out);
}
#endif

typedef uint32_t (*foc_math_sin_cos_fn_t)(float, float *, float *);
typedef uint32_t (*foc_math_atan2_fn_t)(float, float, float *);

static const foc_math_reference_vector_t g_foc_math_reference_vectors[FOC_MATH_BENCHMARK_VECTOR_COUNT] =
{
    {0.0000000000f,  0.0000000000f,  1.0000000000f},
    {0.3926990817f,  0.3826834324f,  0.9238795325f},
    {0.7853981634f,  0.7071067812f,  0.7071067812f},
    {1.1780972451f,  0.9238795325f,  0.3826834324f},
    {1.5707963268f,  1.0000000000f,  0.0000000000f},
    {1.9634954085f,  0.9238795325f, -0.3826834324f},
    {2.3561944902f,  0.7071067812f, -0.7071067812f},
    {2.7488935719f,  0.3826834324f, -0.9238795325f},
    {3.1415926536f,  0.0000000000f, -1.0000000000f},
    {3.5342917353f, -0.3826834324f, -0.9238795325f},
    {3.9269908170f, -0.7071067812f, -0.7071067812f},
    {4.3196898987f, -0.9238795325f, -0.3826834324f},
    {4.7123889804f, -1.0000000000f,  0.0000000000f},
    {5.1050880621f, -0.9238795325f,  0.3826834324f},
    {5.4977871438f, -0.7071067812f,  0.7071067812f},
    {5.8904862255f, -0.3826834324f,  0.9238795325f},
};

/* 空的不可内联调用，用来量出同样的 DWT 读数与函数调用固定开销。
 * Empty non-inlined call used to measure the fixed DWT-read/call overhead. */
__attribute__((noinline)) static uint32_t foc_math_benchmark_noop(float first,
                                                                  float second,
                                                                  volatile float *out_first,
                                                                  volatile float *out_second)
{
    *out_first = first;
    *out_second = second;
    return 1U;
}

static float foc_math_benchmark_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t foc_math_benchmark_scale_error(float error, float scale)
{
    float scaled = error * scale + 0.5f;
    return (scaled >= (float)UINT32_MAX) ? UINT32_MAX : (uint32_t)scaled;
}

static void foc_math_benchmark_record(foc_math_benchmark_stat_t *stats,
                                      uint32_t cycles,
                                      uint32_t success)
{
    if ((stats->calls == 0U) || (cycles < stats->minimum_cycles))
    {
        stats->minimum_cycles = cycles;
    }
    if (cycles > stats->maximum_cycles)
    {
        stats->maximum_cycles = cycles;
    }
    if (stats->total_cycles <= (UINT32_MAX - cycles))
    {
        stats->total_cycles += cycles;
    }
    else
    {
        stats->total_cycles = UINT32_MAX;
    }
    ++stats->calls;
    if (success != 0U)
    {
        ++stats->successes;
    }
    else
    {
        ++stats->failures;
    }
}

/* 每次只屏蔽一个调用，避免 SysTick/ADC 把“操作自身周期”污染成中断延迟。
 * Interrupts are masked for one call only, preventing SysTick/ADC preemption
 * from being misreported as operation latency. Full ISR WCET remains the final
 * realtime evidence because this isolated measurement changes PRIMASK context. */
static uint32_t foc_math_benchmark_measure_noop(void)
{
    volatile float first;
    volatile float second;
    uint32_t primask = __get_PRIMASK();
    uint32_t start;
    uint32_t end;

    __disable_irq();
    __DSB();
    __ISB();
    start = DWT->CYCCNT;
    (void)foc_math_benchmark_noop(0.25f, -0.5f, &first, &second);
    end = DWT->CYCCNT;
    foc_restore_interrupts(primask);
    return end - start;
}

static uint32_t foc_math_benchmark_measure_sin_cos(foc_math_sin_cos_fn_t operation,
                                                    float angle_rad,
                                                    float *sin_out,
                                                    float *cos_out,
                                                    uint32_t *success_out)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t start;
    uint32_t end;

    __disable_irq();
    __DSB();
    __ISB();
    start = DWT->CYCCNT;
    *success_out = operation(angle_rad, sin_out, cos_out);
    end = DWT->CYCCNT;
    foc_restore_interrupts(primask);
    return end - start;
}

static uint32_t foc_math_benchmark_measure_atan2(foc_math_atan2_fn_t operation,
                                                 float y,
                                                 float x,
                                                 float *angle_out,
                                                 uint32_t *success_out)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t start;
    uint32_t end;

    __disable_irq();
    __DSB();
    __ISB();
    start = DWT->CYCCNT;
    *success_out = operation(y, x, angle_out);
    end = DWT->CYCCNT;
    foc_restore_interrupts(primask);
    return end - start;
}

uint32_t foc_math_accel_benchmark(foc_math_benchmark_result_t *result,
                                  uint32_t repeats)
{
    uint32_t repeat;
    uint32_t index;
    union
    {
        uint32_t bits;
        float value;
    } not_a_number = {0x7FC00000UL};

    if (result == 0)
    {
        return 0U;
    }
    *result = (foc_math_benchmark_result_t){0};
    result->struct_size = sizeof(*result);
    result->version = FOC_MATH_BENCHMARK_VERSION;
    result->vector_count = FOC_MATH_BENCHMARK_VECTOR_COUNT;
    result->repeats = repeats;
    result->fixed_delay_nops = FOC_CORDIC_FIXED_DELAY_NOPS;
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && \
    defined(FOC_MATH_CORDIC_FIXED_DELAY_CANDIDATE)
    result->realtime_uses_fixed_delay = 1U;
#endif
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
    result->fast_approx_available = 1U;
#if defined(FOC_MATH_CPU_FAST_APPROX_CANDIDATE)
    result->realtime_uses_fast_approx = 1U;
#endif
    result->invalid_tests = 4U;
#else
    result->invalid_tests = 2U;
#endif
    if ((repeats == 0U) || (repeats > FOC_MATH_BENCHMARK_MAX_REPEATS) ||
        (g_foc_math_backend != FOC_MATH_BACKEND_CORDIC) ||
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U))
    {
        return 0U;
    }

    for (repeat = 0U; repeat < repeats; ++repeat)
    {
        for (index = 0U; index < FOC_MATH_BENCHMARK_VECTOR_COUNT; ++index)
        {
            const foc_math_reference_vector_t *reference =
                &g_foc_math_reference_vectors[index];
            float sin_value = 0.0f;
            float cos_value = 0.0f;
            float angle_value = 0.0f;
            float error;
            uint32_t success;
            uint32_t cycles;

            cycles = foc_math_benchmark_measure_noop();
            foc_math_benchmark_record(&result->measurement_overhead, cycles, 1U);

            cycles = foc_math_benchmark_measure_sin_cos(foc_math_benchmark_poll_sin_cos,
                                                        reference->angle_rad,
                                                        &sin_value,
                                                        &cos_value,
                                                        &success);
            foc_math_benchmark_record(&result->sin_cos, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(sin_value - reference->sin_value);
                if (foc_math_benchmark_abs(cos_value - reference->cos_value) > error)
                {
                    error = foc_math_benchmark_abs(cos_value - reference->cos_value);
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000000.0f);
                if (cycles > result->maximum_sin_cos_error_ppb)
                {
                    result->maximum_sin_cos_error_ppb = cycles;
                }
            }

            cycles = foc_math_benchmark_measure_atan2(foc_math_benchmark_poll_atan2,
                                                      reference->sin_value,
                                                      reference->cos_value,
                                                      &angle_value,
                                                      &success);
            foc_math_benchmark_record(&result->atan2, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(angle_value - reference->angle_rad);
                if (error > FOC_PI_F)
                {
                    error = FOC_TWO_PI_F - error;
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000.0f);
                if (cycles > result->maximum_atan2_error_urad)
                {
                    result->maximum_atan2_error_urad = cycles;
                }
            }

            sin_value = 0.0f;
            cos_value = 0.0f;
            cycles = foc_math_benchmark_measure_sin_cos(foc_math_benchmark_fixed_sin_cos,
                                                        reference->angle_rad,
                                                        &sin_value,
                                                        &cos_value,
                                                        &success);
            foc_math_benchmark_record(&result->fixed_delay_sin_cos, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(sin_value - reference->sin_value);
                if (foc_math_benchmark_abs(cos_value - reference->cos_value) > error)
                {
                    error = foc_math_benchmark_abs(cos_value - reference->cos_value);
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000000.0f);
                if (cycles > result->maximum_fixed_delay_sin_cos_error_ppb)
                {
                    result->maximum_fixed_delay_sin_cos_error_ppb = cycles;
                }
            }

            angle_value = 0.0f;
            cycles = foc_math_benchmark_measure_atan2(foc_math_benchmark_fixed_atan2,
                                                      reference->sin_value,
                                                      reference->cos_value,
                                                      &angle_value,
                                                      &success);
            foc_math_benchmark_record(&result->fixed_delay_atan2, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(angle_value - reference->angle_rad);
                if (error > FOC_PI_F)
                {
                    error = FOC_TWO_PI_F - error;
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000.0f);
                if (cycles > result->maximum_fixed_delay_atan2_error_urad)
                {
                    result->maximum_fixed_delay_atan2_error_urad = cycles;
                }
            }

#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
            sin_value = 0.0f;
            cos_value = 0.0f;
            cycles = foc_math_benchmark_measure_sin_cos(foc_math_benchmark_fast_sin_cos,
                                                        reference->angle_rad,
                                                        &sin_value,
                                                        &cos_value,
                                                        &success);
            foc_math_benchmark_record(&result->fast_approx_sin_cos, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(sin_value - reference->sin_value);
                if (foc_math_benchmark_abs(cos_value - reference->cos_value) > error)
                {
                    error = foc_math_benchmark_abs(cos_value - reference->cos_value);
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000000.0f);
                if (cycles > result->maximum_fast_approx_sin_cos_error_ppb)
                {
                    result->maximum_fast_approx_sin_cos_error_ppb = cycles;
                }
            }

            angle_value = 0.0f;
            cycles = foc_math_benchmark_measure_atan2(foc_math_benchmark_fast_atan2,
                                                      reference->sin_value,
                                                      reference->cos_value,
                                                      &angle_value,
                                                      &success);
            foc_math_benchmark_record(&result->fast_approx_atan2, cycles, success);
            if ((success != 0U) && (repeat == 0U))
            {
                error = foc_math_benchmark_abs(angle_value - reference->angle_rad);
                if (error > FOC_PI_F)
                {
                    error = FOC_TWO_PI_F - error;
                }
                cycles = foc_math_benchmark_scale_error(error, 1000000.0f);
                if (cycles > result->maximum_fast_approx_atan2_error_urad)
                {
                    result->maximum_fast_approx_atan2_error_urad = cycles;
                }
            }
#endif
        }
    }

    {
        float first = 1.0f;
        float second = 1.0f;
        if ((foc_math_accel_sin_cos(not_a_number.value, &first, &second) == 0U) &&
            (first == 0.0f) && (second == 0.0f))
        {
            ++result->invalid_rejections;
        }
        first = 1.0f;
        if ((foc_math_accel_atan2(not_a_number.value, 1.0f, &first) == 0U) &&
            (first == 0.0f))
        {
            ++result->invalid_rejections;
        }
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
        first = 1.0f;
        second = 1.0f;
        if ((foc_rust_fast_math_sin_cos(not_a_number.value, &first, &second) == 0U) &&
            (first == 0.0f) && (second == 0.0f))
        {
            ++result->invalid_rejections;
        }
        first = 1.0f;
        if ((foc_rust_fast_math_atan2(not_a_number.value, 1.0f, &first) == 0U) &&
            (first == 0.0f))
        {
            ++result->invalid_rejections;
        }
#endif
    }
    return 1U;
}

#endif

#else

/*
 * 无 CORDIC 目标的桩实现。
 * Stub implementation for targets without CORDIC.
 *
 * 三个函数全部返回 0 并把输出清零，表示"加速器不可用"。Rust bridge 见到 0
 * 会立刻用 CpuMath 重算，因此控制精度不受影响，只是每次多花软件实现的周期。
 * All three return 0 with zeroed outputs, meaning "accelerator unavailable".
 * The Rust bridge recomputes with CpuMath on seeing 0, so control accuracy is
 * unaffected; only the software cost is paid.
 *
 * 这些桩必须存在：本文件在非 G431 目标上同样参与编译，缺少符号会导致
 * 链接失败，而不是优雅回退。
 * These stubs must exist: this file also compiles on non-G431 targets, and a
 * missing symbol would fail the link instead of degrading gracefully.
 */

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
static foc_math_health_t g_foc_math_stub_health =
{
    .struct_size = sizeof(foc_math_health_t),
    .version = FOC_MATH_HEALTH_VERSION,
};
#endif

foc_math_backend_t foc_math_accel_init(void)
{
    return FOC_MATH_BACKEND_CPU;
}

foc_math_backend_t foc_math_accel_backend(void)
{
    return FOC_MATH_BACKEND_CPU;
}

uint32_t foc_math_accel_sin_cos(float angle_rad, float *sin_out, float *cos_out)
{
    (void)angle_rad;
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_stub_health.sin_cos_calls;
    ++g_foc_math_stub_health.sin_cos_fallback_required;
#endif
    if (sin_out != 0)
    {
        *sin_out = 0.0f;
    }
    if (cos_out != 0)
    {
        *cos_out = 0.0f;
    }
    return 0U;
}

uint32_t foc_math_accel_magnitude(float x, float y, float *magnitude_out)
{
    (void)x;
    (void)y;
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_stub_health.magnitude_calls;
    ++g_foc_math_stub_health.magnitude_fallback_required;
#endif
    if (magnitude_out != 0)
    {
        *magnitude_out = 0.0f;
    }
    return 0U;
}

uint32_t foc_math_accel_atan2(float y, float x, float *angle_out)
{
    (void)y;
    (void)x;
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
    ++g_foc_math_stub_health.atan2_calls;
    ++g_foc_math_stub_health.atan2_fallback_required;
#endif
    if (angle_out != 0)
    {
        *angle_out = 0.0f;
    }
    return 0U;
}

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
uint32_t foc_math_accel_health_get(foc_math_health_t *result)
{
    if (result == 0)
    {
        return 0U;
    }
    *result = g_foc_math_stub_health;
    return 1U;
}

void foc_math_accel_health_reset(void)
{
    g_foc_math_stub_health = (foc_math_health_t)
    {
        .struct_size = sizeof(foc_math_health_t),
        .version = FOC_MATH_HEALTH_VERSION,
    };
}

uint32_t foc_math_accel_inject_not_ready_once(uint32_t operation)
{
    (void)operation;
    return 0U;
}
#endif

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_BENCHMARK)
uint32_t foc_math_accel_benchmark(foc_math_benchmark_result_t *result,
                                  uint32_t repeats)
{
    (void)repeats;
    if (result != 0)
    {
        *result = (foc_math_benchmark_result_t){0};
        result->struct_size = sizeof(*result);
        result->version = FOC_MATH_BENCHMARK_VERSION;
    }
    return 0U;
}
#endif

#endif
