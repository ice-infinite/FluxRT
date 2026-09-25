/*
 * FluxRT —— STM32G431 芯片平台适配层。
 * FluxRT - STM32G431 chip platform adapter.
 *
 * 职责 / Responsibility:
 *   - 配置 TIM1 中心对齐 PWM、ADC1/ADC2 注入同步采样、栅极使能与硬件关断；
 *   - 实现快环 ISR（ADC1_2_IRQHandler），在其中有且只有一次
 *     `foc_rust_realtime_step()` 调用；
 *   - 持有并发布诊断、遥测、时序统计和 trace 缓冲。
 *     Configures TIM1 centre-aligned PWM, ADC1/ADC2 injected synchronous
 *     sampling, gate enables and the hardware shutdown path; implements the
 *     fast-loop ISR with exactly one `foc_rust_realtime_step()` call; and owns
 *     the diagnostics, telemetry, timing statistics and trace buffer.
 *
 * 边界 / Boundary:
 *   - 不实现任何 FOC 算法。Clarke/Park/PI/SVPWM/观测器全部在 Rust 侧。
 *     Implements no FOC algorithm; Clarke, Park, PI, SVPWM and the observer all
 *     live in Rust.
 *   - 只通过固定 C ABI（foc_rust_bridge.h）与控制器交互，不认识其内部状态。
 *     Talks to the controller only through the fixed C ABI and knows nothing
 *     about its internal state.
 *   - Rust 可以算出占空比，但**不能自行使能栅极**；写 PWM 之前必须由本层
 *     复核硬件故障、控制状态和输出范围。
 *     Rust may compute the duty, but it can never enable the gate itself. This
 *     layer re-checks the hardware fault inputs, the control state and the
 *     output range before anything reaches the PWM registers.
 *
 * 安全不变量 / Safety invariants:
 *   - 上电默认 CH1..CH3、MOE 和 PB13..PB15 全部关闭；
 *   - `foc_platform_emergency_stop()` 在每一个失败返回路径上被调用；
 *   - 输出被拒或越界时立刻关断，不沿用上一周期占空比；
 *   - Break（PA11）与软件过流都会立即关断功率级。
 *     On boot CH1..CH3, MOE and PB13..PB15 are all off; every failure path
 *     calls `foc_platform_emergency_stop()`; a rejected or out-of-range output
 *     shuts down immediately instead of reusing the previous duty; and both the
 *     Break input (PA11) and the software over-current trip disable the power
 *     stage at once.
 *
 * 参考 / Reference:
 *   docs/架构与安全边界.md, docs/C与Rust混合架构.md,
 *   docs/2026-09-22实机烧录记录.md
 */

#include "foc_platform.h"
#include "foc_math_accel.h"
#include "foc_pwm_timing.h"

/* 平台级可调参数：母线窗口、软件过流阈值、占空比窗口、ISR 截止周期。
 * 字段顺序必须与 foc_platform.h 中的 foc_platform_config_t 一致。
 *
 * Platform-level tunables: bus window, software over-current trip, duty window
 * and the ISR deadline. The field order must match foc_platform_config_t.
 *
 * 取值依据 / Rationale:
 *   [HW] 母线窗口 7..18 V —— 实测母线约 12.3 V，窗口覆盖电源设定范围；
 *   [HW] 过流 1.15 A —— 额定 0.8 A 的 1.44 倍，高于正常峰值、低于堵转；
 *   [FW] 占空比窗口 3%..97% —— 给自举电容充电和高侧驱动留出最小脉宽；
 *   [HW] 截止 12750 cycles —— CM4.1 接管拍实测 12548 cycles；12 kHz 周期
 *   14167 cycles，仍保留 1417 cycles（10.0%）物理裕量。
 *   [HW] bus window 7..18 V around the measured 12.3 V; [HW] a 1.15 A trip at
 *   1.44x rated current, above the normal peak and below a stall; [FW] a
 *   3%..97% duty window that leaves minimum pulse width for bootstrap charging
 *   and the high-side driver; [HW] a 12750-cycle deadline after a measured
 *   12548-cycle CM4.1 handover peak, retaining 1417 cycles (10.0%) of the
 *   14167-cycle period at 12 kHz. */
static foc_platform_config_t g_foc_platform_config =
{
    sizeof(foc_platform_config_t),
    FOC_PLATFORM_CONFIG_VERSION,
    7.0f,
    18.0f,
    1.15f,
    0.03f,
    0.97f,
    FOC_DEFAULT_ISR_DEADLINE_CYCLES,
};
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
/*
 * X-NUCLEO-IHM16M1 官方名义端电压网络：3.3 V ADC、12 bit、10 kOhm/2.2 kOhm。
 * phase_full_scale = 3.3 * (10k + 2.2k) / 2.2k = 18.3 V；
 * 18.3 V / 4095 = 4.468864 mV/count，显示值四舍五入为 4469 uV/count。
 *
 * 这不是逐板标定值，所以同时设置 NOMINAL_COMPONENTS 与 DIAGNOSTIC_ONLY，且绝不
 * 设置 BOARD_CALIBRATED / OBSERVER_ELIGIBLE。当前观察器继续只用上一拍命令电压和
 * 实测 Vbus 重构，与 ST MCSDK 参考工程的默认数据流一致。
 */
static const foc_phase_voltage_model_t g_foc_phase_voltage_nominal_model =
{
    sizeof(foc_phase_voltage_model_t),
    FOC_PHASE_VOLTAGE_MODEL_VERSION,
    FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL,
    FOC_PHASE_VOLTAGE_MODEL_FLAG_NOMINAL_COMPONENTS |
        FOC_PHASE_VOLTAGE_MODEL_FLAG_DIAGNOSTIC_ONLY,
    3300U,
    4095U,
    10000U,
    2200U,
    18300U,
    4469U,
};
#endif
/* 同拍时序统计；由 ISR 写入，RT-Thread 线程读取。
 * Same-tick timing statistics; written by the ISR, read by the RT-Thread thread. */
static foc_realtime_timing_stats_t g_foc_timing_stats;

#if defined(FOC_TARGET_STM32G431)
#include "rtconfig.h"
#include "stm32g4xx_hal.h"

/* 引脚映射来自 ST 官方参考工程：NUCLEO-G431RB + X-NUCLEO-IHM16M1。
 * Pin mapping taken from the official ST reference project for the
 * NUCLEO-G431RB + X-NUCLEO-IHM16M1 combination. */
#define FOC_GATE_ENABLE_PORT             GPIOB
/* PB13/PB14/PB15 = U/V/W 三相栅极使能，高有效。必须默认拉低。
 * PB13/PB14/PB15 are the U/V/W gate enables, active high; they must default
 * low. */
#define FOC_GATE_ENABLE_PINS             (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
/* PA8/PA9/PA10 = TIM1_CH1/CH2/CH3，三相高侧命令。
 * PA8/PA9/PA10 are TIM1_CH1/CH2/CH3, the three high-side commands. */
#define FOC_PWM_PORT                     GPIOA
#define FOC_PWM_PINS                     (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
/* PA11 = TIM1_BKIN2 / 驱动器保护，低有效并带上拉。
 * PA11 is TIM1_BKIN2 / driver protection, active low with a pull-up. */
#define FOC_DRIVER_PROTECTION_PORT       GPIOA
#define FOC_DRIVER_PROTECTION_PIN        GPIO_PIN_11
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
/*
 * IHM16M1 端电压网络：U/V/W 分别接 PC0/PC3/PC1（ADC12_IN6/9/7）。
 * PC9=IO_BEMF，拉低后接通板载 10 kOhm / 2.2 kOhm 采样支路；默认拉高禁用。
 * 该网络只服务 Diagnostic/Calibration 原始码采集，绝不进入控制反馈。
 */
#define FOC_BEMF_DIVIDER_ENABLE_PORT     GPIOC
#define FOC_BEMF_DIVIDER_ENABLE_PIN      GPIO_PIN_9
#endif

/* 定时器时钟 = SYSCLK = 170 MHz（APB2 不分频，见 board/board.c）。
 * Timer clock equals SYSCLK at 170 MHz; APB2 is undivided, see board/board.c. */
#define FOC_PWM_TIMER_CLOCK_HZ           (170000000UL)
/*
 * 默认仍是已经过板端 WCET 验证的 12/12 kHz。24/12 kHz 仅是 Diagnostic
 * 设计候选：TIM1 CH4 保留有效三分流采样窗口，TIM2 通过内部 ITR0 对
 * OC4REF/TRGO 上升沿二分频后触发 ADC；RCR=3 只负责让 CCR 每两个 PWM 周期
 * 装载一次。Production 无条件忽略候选宏。
 *
 * The default remains the board-measured 12/12 kHz path. The 24/12 kHz plan is
 * Diagnostic-only: TIM1 CH4 keeps the valid three-shunt sampling window and
 * TIM2 divides its internal OC4REF/TRGO edges before triggering the ADC. RCR=3
 * only controls the two-PWM-period CCR preload update. Production ignores a
 * stale candidate define unconditionally.
 */
#if defined(FOC_PWM_24K_CONTROL_12K_CANDIDATE) && defined(FLUXRT_DIAGNOSTIC_BUILD)
#define FOC_PWM_FREQUENCY_HZ             (24000UL)
#define FOC_CONTROL_FREQUENCY_HZ         (12000UL)
#define FOC_TIM1_REPETITION_COUNTER      (3UL)
#define FOC_ACTUATION_DELAY_PWM_TICKS    (2UL)
#define FOC_ADC_TRIGGER_KIND             FOC_PWM_ADC_TRIGGER_DIVIDED_OC4REF
#define FOC_TIM1_MASTER_TRIGGER          TIM_TRGO_OC4REF
#define FOC_TIM1_USES_OC4_TRIGGER        (1U)
#define FOC_ADC_EXTERNAL_TRIGGER         ADC_EXTERNALTRIGINJEC_T2_TRGO
#define FOC_ADC_TRIGGER_USES_TIM2_DIVIDER (1U)
#else
#define FOC_PWM_FREQUENCY_HZ             (12000UL)
#define FOC_CONTROL_FREQUENCY_HZ         (12000UL)
#define FOC_TIM1_REPETITION_COUNTER      (1UL)
#define FOC_ACTUATION_DELAY_PWM_TICKS    (1UL)
#define FOC_ADC_TRIGGER_KIND             FOC_PWM_ADC_TRIGGER_OC4REF_RISING
#define FOC_TIM1_MASTER_TRIGGER          TIM_TRGO_OC4REF
#define FOC_TIM1_USES_OC4_TRIGGER        (1U)
#define FOC_ADC_EXTERNAL_TRIGGER         ADC_EXTERNALTRIGINJEC_T1_TRGO
#define FOC_ADC_TRIGGER_USES_TIM2_DIVIDER (0U)
#endif

/* ARR 值（中心对齐）/ ARR value in centre-aligned mode. */
#define FOC_PWM_PERIOD_TICKS             (FOC_PWM_TIMER_CLOCK_HZ / (2UL * FOC_PWM_FREQUENCY_HZ))
#define FOC_PWM_TICKS_PER_CONTROL        (FOC_PWM_FREQUENCY_HZ / FOC_CONTROL_FREQUENCY_HZ)
#define FOC_PWM_CYCLE_BUDGET             (FOC_PWM_TIMER_CLOCK_HZ / FOC_PWM_FREQUENCY_HZ)
#define FOC_CONTROL_CYCLE_BUDGET         (FOC_PWM_TIMER_CLOCK_HZ / FOC_CONTROL_FREQUENCY_HZ)

_Static_assert((FOC_PWM_FREQUENCY_HZ % FOC_CONTROL_FREQUENCY_HZ) == 0U,
               "PWM/control rates must have an integer ratio");
_Static_assert((FOC_TIM1_REPETITION_COUNTER + 1U) == (2U * FOC_PWM_TICKS_PER_CONTROL),
               "TIM1 RCR must divide centre-aligned half periods to the control rate");
_Static_assert(FOC_ACTUATION_DELAY_PWM_TICKS == FOC_PWM_TICKS_PER_CONTROL,
               "CCR preload latency must match one control update interval");
_Static_assert(FOC_DEFAULT_ISR_DEADLINE_CYCLES < FOC_CONTROL_CYCLE_BUDGET,
               "ISR deadline must fit inside one control period");

static const foc_pwm_timing_plan_t g_foc_pwm_timing_plan =
{
    FOC_PWM_TIMER_CLOCK_HZ,
    FOC_PWM_FREQUENCY_HZ,
    FOC_CONTROL_FREQUENCY_HZ,
    FOC_PWM_PERIOD_TICKS,
    FOC_TIM1_REPETITION_COUNTER,
    FOC_ACTUATION_DELAY_PWM_TICKS,
    FOC_ADC_TRIGGER_KIND,
};
/* ADC 与采样链常量。[HW] 实测值，来自 IHM16M1 板载参数。
 * ADC and sensing-chain constants; [HW] measured, from the IHM16M1 board. */
#define FOC_ADC_FULL_SCALE               (4095.0f)
#define FOC_ADC_REFERENCE_VOLTAGE        (3.3f)
/* 母线分压比 1/16 = 0.0625 [HW] / Bus divider ratio, 16:1. */
#define FOC_BUS_PARTITIONING_FACTOR      (0.0625f)
/* 分流电阻 0.33 ohm、放大倍数 1.53 [HW] / Shunt and amplifier gain. */
#define FOC_CURRENT_SHUNT_OHM            (0.33f)
#define FOC_CURRENT_AMPLIFIER_GAIN       (1.53f)
/* 静态监测用软件触发 ADC 的轮询超时 [ms] / Poll timeout for monitor reads [ms]. */
#define FOC_ADC_TIMEOUT_MS               (5UL)
/* 电流零点标定：32 次平均；有效区间取满量程的 6.25%~93.75%，用于识别
 * "通道悬空/短路"这类明显异常。
 * Current offset calibration: 32-sample average, valid range 6.25%..93.75% of
 * full scale, which catches obviously broken channels. */
#define FOC_OFFSET_SAMPLE_COUNT          (32UL)
#define FOC_OFFSET_MIN_VALID             (256U)
#define FOC_OFFSET_MAX_VALID             (3839U)
/* 电流 counts -> 安培 的换算系数 [counts/A]，约 626.5。
 * 推导 / Derivation: counts * Vref / (full_scale * Rshunt * gain)。
 * Current counts-to-ampere factor [counts/A], about 626.5, derived as
 * counts * Vref / (full_scale * Rshunt * gain). */
#define FOC_CURRENT_COUNTS_PER_AMP       ((FOC_ADC_FULL_SCALE * FOC_CURRENT_SHUNT_OHM * \
                                           FOC_CURRENT_AMPLIFIER_GAIN) / \
                                          FOC_ADC_REFERENCE_VOLTAGE)
#if defined(FLUXRT_TRACE_BUILD)
/* trace 缓冲只在 Diagnostic 档编译。Production 档裁掉它可以回收约 3 KB
 * Flash（缓冲本身）及其 ISR/输出代码。
 * The trace buffer is compiled only in the Diagnostic profile. Production
 * drops it, recovering about 3 KB of Flash plus its ISR and output code. */
#define FOC_TRACE_CAPACITY               (64U)
/* 最小分频 = 120 -> 12 kHz 下最高 100 Hz 采样。
 * Minimum divider 120 gives at most 100 Hz sampling at 12 kHz. */
#define FOC_TRACE_MIN_DIVIDER            (120U)
#endif
#if defined(FOC_ISR_TIMING_PROBE)
/* 可选示波器探针：PA5（NUCLEO 的 LD2/D13）在 ISR 区间输出高脉冲。
 * 用于把 DWT 计时与示波器实测对拍；默认关闭，因为它会占用一个 LED 引脚并
 * 增加极少量指令。
 * Optional oscilloscope probe: PA5 (NUCLEO LD2/D13) goes high for the ISR span,
 * used to cross-check the DWT numbers against a scope. Off by default because it
 * consumes an LED pin and adds a few instructions. */
#define FOC_TIMING_PROBE_PORT            GPIOA
#define FOC_TIMING_PROBE_PIN             GPIO_PIN_5
#define FOC_TIMING_PROBE_HIGH()          (FOC_TIMING_PROBE_PORT->BSRR = FOC_TIMING_PROBE_PIN)
#define FOC_TIMING_PROBE_LOW()           (FOC_TIMING_PROBE_PORT->BSRR = ((uint32_t)FOC_TIMING_PROBE_PIN << 16U))
static uint32_t g_foc_sync_previous_cycle;
static uint32_t g_foc_sync_interval_valid;
#else
/* 默认展开为无操作，保证探针分支对时序测量本身没有影响。
 * Expands to nothing by default, so the probe branch cannot influence the
 * measurement it is meant to validate. */
#define FOC_TIMING_PROBE_HIGH()          ((void)0)
#define FOC_TIMING_PROBE_LOW()           ((void)0)
#endif
/* arm 功率级之前必须全部置位的诊断位。
 * Diagnostics that must all be set before the power stage may be armed. */
#define FOC_TRIAL_REQUIRED_FLAGS          (FOC_PLATFORM_DIAG_TIM1_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_ADC_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_ADC_CALIBRATED | \
                                           FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID | \
                                           FOC_PLATFORM_DIAG_SYNC_RUNNING | \
                                           FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID)
/* 任一置位就禁止 arm 的诊断位。注意 TRIAL_ARMED 也在其中：这使 arm 成为
 * 一次性的，重复 arm 会被拒绝而不是叠加。
 * Any of these blocks arming. TRIAL_ARMED is included deliberately, which makes
 * arming one-shot and rejects a repeated arm instead of stacking. */
#define FOC_TRIAL_FORBIDDEN_FLAGS         (FOC_PLATFORM_DIAG_DRIVER_FAULT | \
                                           FOC_PLATFORM_DIAG_OUTPUT_ACTIVE | \
                                           FOC_PLATFORM_DIAG_ADC_READ_ERROR | \
                                           FOC_PLATFORM_DIAG_BREAK_LATCHED | \
                                           FOC_PLATFORM_DIAG_CURRENT_TRIP | \
                                           FOC_PLATFORM_DIAG_TRIAL_ARMED)

/* ---------------------------------------------------------------- 平台状态 */

/* HAL 句柄。三个都是静态存储期，中断与线程共享，因此所有写操作都必须
 * 发生在 ISR 使能之前。
 * HAL handles. All three have static storage duration and are shared between the
 * ISR and threads, so every write must happen before the ISR is enabled. */
static TIM_HandleTypeDef g_foc_tim1;
static ADC_HandleTypeDef g_foc_adc1;
static ADC_HandleTypeDef g_foc_adc2;
/* 诊断位与遥测由 ISR 写、线程读；volatile 保证不被优化掉，
 * 但复合字段的读一致性由短临界区（见 foc_platform_get_telemetry）保证。
 * Diagnostics and telemetry are written by the ISR and read by threads.
 * `volatile` prevents removal but not tearing; consistency of the compound
 * fields is handled by a short critical section in
 * foc_platform_get_telemetry. */
static volatile foc_platform_diagnostics_t g_foc_diagnostics;
/* 非 0 表示功率级已 arm（CCER/MOE/栅极已开）。由 ISR 与线程共同读写，
 * 因此关断路径一律先清它再动寄存器。
 * Non-zero means the power stage is armed (CCER/MOE/gate on). It is touched by
 * both the ISR and threads, so every shutdown path clears it before touching
 * registers. */
static volatile uint32_t g_foc_control_armed;
/* 绑定的 Rust 控制器上下文；由 foc_platform_bind_controller() 设置一次。
 * Bound Rust controller context; set once by foc_platform_bind_controller(). */
static foc_rust_context_t *g_foc_controller;
static volatile foc_telemetry_t g_foc_telemetry;
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
/* 256 * 16 B = 4096 B；Production 不实例化这块采集 RAM。 */
static foc_phase_voltage_capture_t g_foc_phase_voltage_capture;
#endif
#if defined(FLUXRT_TRACE_BUILD)
/* ISR 只保存原始定长快照；float -> 定点编码延后到线程侧 trace_pop()。
 * 外部 foc_trace_sample_t/FTR 协议保持 72 B 和原单位不变。把 24 次浮点缩放
 * 从偶发的 trace 采样 ISR 拍移走，避免诊断功能占用控制截止预算。
 * The ISR stores only a fixed-size raw snapshot. Float-to-fixed encoding is
 * deferred to trace_pop() in thread context, while the public 72-byte trace ABI
 * and units remain unchanged. */
typedef struct
{
    uint32_t step;
    uint32_t flags;
    foc_feedback_t feedback;
    foc_output_t output;
    foc_telemetry_t telemetry;
} foc_trace_raw_sample_t;

/* trace 环形缓冲：ISR 是唯一生产者，foc_trace_pop() 是唯一消费者。
 * 单生产者单消费者，因此 head/tail 两个索引足够，不需要锁。
 * Trace ring buffer: the ISR is the sole producer and foc_trace_pop() the sole
 * consumer. With one producer and one consumer, two indices suffice and no lock
 * is needed. */
static volatile foc_trace_raw_sample_t g_foc_trace_buffer[FOC_TRACE_CAPACITY];
static volatile uint32_t g_foc_trace_head;
static volatile uint32_t g_foc_trace_tail;
static volatile uint32_t g_foc_trace_enabled;
/* 默认分频 240 -> 12 kHz 下 50 Hz 采样。 */
static volatile uint32_t g_foc_trace_divider = 240U;
static volatile uint32_t g_foc_trace_counter;
/* 因缓冲满而丢弃的样本数。这个计数必须上报：它决定一次 trace 是否可信。
 * Samples dropped because the buffer was full. It must be reported: it decides
 * whether a trace can be trusted at all. */
static volatile uint32_t g_foc_trace_dropped_count;


/*
 * float -> int16 定长缩放，带饱和和四舍五入。
 * Fixed-point scale from float to int16, with saturation and rounding.
 *
 * 这些换算只在线程侧 trace_pop() 调用，ISR 环形缓冲保存原始定长快照。
 * These conversions run only from thread-side trace_pop(); the ISR ring stores
 * a fixed-size raw snapshot.
 */
static int16_t foc_trace_scaled_i16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }
    /* 手动四舍五入（±0.5 后截断），避免依赖运行时的 lroundf。
     * Manual rounding (add ±0.5 then truncate) avoids pulling in lroundf. */
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

/*
 * float -> uint16 定长缩放。占空比与母线电压非负，因此负数直接钳到 0。
 * Fixed-point scale from float to uint16. Duty and bus voltage are non-negative,
 * so a negative input clamps to 0 rather than wrapping.
 */
static uint16_t foc_trace_scaled_u16(float value, float scale)
{
    float scaled = value * scale;
    if (scaled <= 0.0f)
    {
        return 0U;
    }
    if (scaled >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(scaled + 0.5f);
}

/* 方差等非负大范围量使用 uint32；`!(scaled > 0)` 同时把 NaN 安全钳到 0。 */
static uint32_t foc_trace_scaled_u32(float value, float scale)
{
    float scaled = value * scale;
    if (!(scaled > 0.0f))
    {
        return 0U;
    }
    if (scaled >= 4294967040.0f)
    {
        return 0xFFFFFFFFUL;
    }
    return (uint32_t)(scaled + 0.5f);
}

/*
 * 把一个控制拍的遥测快照写入 trace 环形缓冲（仅 Diagnostic 档）。
 * Stores one control tick's telemetry into the trace ring buffer
 * (Diagnostic profile only).
 *
 * 调用上下文 / Context: 由 ADC ISR 在输出已写入 CCR 之后调用，因此这里
 * 不做任何阻塞、分配或日志。
 * Called by the ADC ISR after the output has been written to the CCRs, so it
 * performs no blocking, allocation or logging.
 *
 * 返回 / Returns: 1 表示写入成功，0 表示未启用、未到分频点或缓冲已满。
 * 缓冲满时会增加 dropped 计数而不是覆盖旧数据 —— 保留"最早的一段"
 * 比保留"最新的一段"更有利于观察启动过程。
 * 1 on success; 0 when disabled, not due for sampling, or the buffer is full.
 * A full buffer increments the dropped counter instead of overwriting: keeping
 * the earliest samples is more useful than keeping the latest ones for
 * observing the start-up sequence.
 */
static uint32_t foc_platform_trace_capture(const foc_feedback_t *feedback,
                                           const foc_output_t *output,
                                           const foc_telemetry_t *telemetry)
{
    foc_trace_raw_sample_t sample;
    uint32_t head;
    uint32_t next;

    if (g_foc_trace_enabled == 0U)
    {
        return 0U;
    }
    /* 分频计数放在启用判断之后：未启用时完全不增加 ISR 开销。
     * The divider check comes after the enable check, so a disabled trace adds
     * no ISR cost at all. */
    ++g_foc_trace_counter;
    if (g_foc_trace_counter < g_foc_trace_divider)
    {
        return 0U;
    }
    g_foc_trace_counter = 0U;

    sample.step = g_foc_diagnostics.realtime_step_count;
    sample.flags = g_foc_diagnostics.flags;
    sample.feedback = *feedback;
    sample.output = *output;
    sample.telemetry = *telemetry;

    /* 容量是 2 的幂，所以用掩码而不是取模；掩码在 Cortex-M4 上快得多。
     * The capacity is a power of two, so a mask replaces the modulo, which is
     * much cheaper on Cortex-M4. */
    head = g_foc_trace_head;
    next = (head + 1U) & (FOC_TRACE_CAPACITY - 1U);
    if (next == g_foc_trace_tail)
    {
        ++g_foc_trace_dropped_count;
        return 0U;
    }
    g_foc_trace_buffer[head] = sample;
    /* 数据写完之后才发布 head。__DMB() 保证消费者看到 head 更新时
     * 一定也能看到完整的样本，否则会读到半写状态。
     * Publish `head` only after the data is written. __DMB() guarantees the
     * consumer that sees the updated head also sees a complete sample;
     * otherwise it could observe a half-written struct. */
    __DMB();
    g_foc_trace_head = next;
    return 1U;
}

/* 在线程上下文把原始 SI 快照编码成稳定的 72 B trace ABI。
 * Encodes the raw SI snapshot into the stable 72-byte trace ABI in thread
 * context, outside the real-time deadline. */
static void foc_platform_trace_encode(const foc_trace_raw_sample_t *raw,
                                      foc_trace_sample_t *sample)
{
    const foc_feedback_t *feedback = &raw->feedback;
    const foc_output_t *output = &raw->output;
    const foc_telemetry_t *telemetry = &raw->telemetry;

    sample->step = raw->step;
    sample->flags = raw->flags;
    sample->state = (uint16_t)telemetry->state;
    sample->observer_reliable = (uint16_t)telemetry->observer_reliable;
    sample->phase_a_ma = foc_trace_scaled_i16(feedback->phase_current_a, 1000.0f);
    sample->phase_b_ma = foc_trace_scaled_i16(feedback->phase_current_b, 1000.0f);
    sample->phase_c_ma = foc_trace_scaled_i16(feedback->phase_current_c, 1000.0f);
    sample->id_reference_ma = foc_trace_scaled_i16(telemetry->id_reference_a, 1000.0f);
    sample->iq_reference_ma = foc_trace_scaled_i16(telemetry->iq_reference_a, 1000.0f);
    sample->id_measured_ma = foc_trace_scaled_i16(telemetry->id_measured_a, 1000.0f);
    sample->iq_measured_ma = foc_trace_scaled_i16(telemetry->iq_measured_a, 1000.0f);
    sample->vd_command_mv = foc_trace_scaled_i16(telemetry->vd_command_v, 1000.0f);
    sample->vq_command_mv = foc_trace_scaled_i16(telemetry->vq_command_v, 1000.0f);
    sample->duty_a_per_mille = foc_trace_scaled_u16(output->duty_a, 1000.0f);
    sample->duty_b_per_mille = foc_trace_scaled_u16(output->duty_b, 1000.0f);
    sample->duty_c_per_mille = foc_trace_scaled_u16(output->duty_c, 1000.0f);
    sample->bus_voltage_mv = foc_trace_scaled_u16(feedback->dc_bus_voltage, 1000.0f);
    sample->control_angle_mrad =
        foc_trace_scaled_i16(telemetry->electrical_angle_rad, 1000.0f);
    sample->forced_angle_mrad =
        foc_trace_scaled_i16(telemetry->forced_electrical_angle_rad, 1000.0f);
    sample->observer_angle_mrad =
        foc_trace_scaled_i16(telemetry->observer_electrical_angle_rad, 1000.0f);
    sample->observer_speed_rpm =
        foc_trace_scaled_i16(telemetry->measured_speed_rpm, 1.0f);
    sample->observer_bemf_alpha_mv =
        foc_trace_scaled_i16(telemetry->observer_bemf_alpha_v, 1000.0f);
    sample->observer_bemf_beta_mv =
        foc_trace_scaled_i16(telemetry->observer_bemf_beta_v, 1000.0f);
    sample->observer_pll_phase_error_mrad =
        foc_trace_scaled_i16(telemetry->observer_pll_phase_error_rad, 1000.0f);
    sample->observer_speed_mean_rpm =
        foc_trace_scaled_i16(telemetry->observer_speed_mean_rpm, 1.0f);
    sample->observer_speed_variance_rpm2 =
        foc_trace_scaled_u32(telemetry->observer_speed_variance_rpm2, 1.0f);
    sample->observer_reliability_flags =
        (uint16_t)(telemetry->observer_reliability_flags & 0xFFFFU);
    sample->observer_reliable_samples =
        (telemetry->observer_reliable_samples > 65535U) ?
            65535U : (uint16_t)telemetry->observer_reliable_samples;
    sample->observer_wait_elapsed_ms =
        foc_trace_scaled_u16(telemetry->observer_wait_elapsed_s, 1000.0f);
    sample->observer_loss_elapsed_ms =
        foc_trace_scaled_u16(telemetry->observer_loss_elapsed_s, 1000.0f);
    sample->voltage_limited = (telemetry->voltage_limited != 0U) ? 1U : 0U;
    sample->reserved_diagnostic = 0U;
}
#endif

/*
 * 把软件过流阈值从安培换算成 ADC counts，供 ISR 直接比较。
 * Converts the software over-current threshold from amperes to ADC counts so
 * the ISR can compare directly.
 *
 * 每拍都会调用一次，所以这里做的是浮点乘加而不是查表；阈值可被
 * `foc_cfg trip` 在线修改，因此不能预先算好。
 * Called every tick, hence a float multiply-add rather than a lookup: the
 * threshold can be changed online by `foc_cfg trip`, so it cannot be
 * precomputed.
 *
 * 返回 / Returns: 1..32767 范围内的 counts，避免下游用 int16 比较时溢出。
 */
static uint16_t foc_platform_current_trip_counts(void)
{
    float counts = g_foc_platform_config.software_current_trip_a *
                   FOC_CURRENT_COUNTS_PER_AMP;
    if (counts < 1.0f)
    {
        counts = 1.0f;
    }
    if (counts > 32767.0f)
    {
        counts = 32767.0f;
    }
    return (uint16_t)(counts + 0.5f);
}

/*
 * 把母线电压窗口端点从伏特换算成 ADC counts。
 * Converts a bus-voltage window endpoint from volts to ADC counts.
 *
 * 只在 arm 时调用一次，用于把配置的电压窗口与 ADC 回读值比较。
 * Called once at arm time to compare the configured voltage window against the
 * ADC reading.
 */
#if !defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD)
static uint16_t foc_platform_bus_voltage_to_raw(float voltage_v)
{
    float raw = (voltage_v * FOC_ADC_FULL_SCALE * FOC_BUS_PARTITIONING_FACTOR) /
                FOC_ADC_REFERENCE_VOLTAGE;
    if (raw < 0.0f)
    {
        raw = 0.0f;
    }
    if (raw > FOC_ADC_FULL_SCALE)
    {
        raw = FOC_ADC_FULL_SCALE;
    }
    return (uint16_t)(raw + 0.5f);
}
#endif

/*
 * 最短路径立即关断功率级。这是整个工程最关键的一个函数。
 * Fastest-path immediate power-stage shutdown. This is the single most
 * important function in the project.
 *
 * 顺序是关键 / The order matters:
 *   1. 先拉低栅极使能（BSRR 写高 16 位，单次寄存器写，最快）；
 *   2. 再清 MOE / CCER / CCR（只在 TIM1 时钟已使能时才动，否则写无效）；
 *   3. 最后清 arm 标志和诊断位。
 *   1. Gate enables go low first, via a single BSRR write to the upper half.
 *   2. Then MOE, CCER and the CCRs are cleared, but only when the TIM1 clock is
 *      actually enabled; otherwise the writes do nothing.
 *   3. The arm flag and diagnostics are cleared last.
 *
 * 先关栅极再关定时器：STSPIN830 的使能脚是最终的执行器，即使定时器仍短暂
 * 输出，栅极已断。反过来做会留下一段"定时器还在跑、栅极还开着"的窗口。
 * The gate goes first because the STSPIN830 enable pins are the actual actuator:
 * even if the timer outputs briefly, the gates are already cut. The reverse
 * order would leave a window where the timer still runs with the gates open.
 *
 * 本函数可从 ISR 调用，因此只使用寄存器操作，不调用 HAL，不阻塞。
 * Callable from the ISR, so it uses register access only: no HAL, no blocking.
 */
static void foc_platform_disable_power_fast(void)
{
    /* BSRR 高 16 位为复位位，写 1 即拉低；单次写完成三路关断。 */
    FOC_GATE_ENABLE_PORT->BSRR = ((uint32_t)FOC_GATE_ENABLE_PINS << 16U);
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
    }
    g_foc_control_armed = 0U;
    g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_TRIAL_ARMED |
                                 FOC_PLATFORM_DIAG_REALTIME_ARMED);
}

/*
 * 读回栅极使能的实际电平，用于寄存器级证据。
 * Reads back the actual gate-enable level, providing register-level evidence.
 *
 * 这是"软件以为关了"和"硬件确实关了"的区别。安全回读必须看 ODR 而不是
 * 内部变量。
 * This is the difference between "software believes it is off" and "hardware
 * really is off". The safety readback must look at ODR, not at an internal
 * variable.
 */
static uint32_t foc_platform_gate_is_low(void)
{
    return ((FOC_GATE_ENABLE_PORT->ODR & FOC_GATE_ENABLE_PINS) == 0U) ? 1U : 0U;
}

/*
 * 读驱动器保护输入（PA11）。IHM16M1 上为低有效并带外部上拉。
 * Reads the driver protection input (PA11), active low with a pull-up on the
 * IHM16M1.
 *
 * 这里读的是电平而不是锁存位：STSPIN830 的故障输出在故障消失后会自行恢复，
 * 因此本函数返回的是"当前是否有故障"，锁存语义由调用方负责。
 * This reads the level, not a latch: the STSPIN830 fault output self-clears once
 * the fault goes away, so this returns "is there a fault now" and latching is
 * the caller's responsibility.
 */
static uint32_t foc_platform_driver_faulted(void)
{
    /* IHM16M1 driver-protection input is active low and has a pull-up. */
    return ((FOC_DRIVER_PROTECTION_PORT->IDR & FOC_DRIVER_PROTECTION_PIN) == 0U) ? 1U : 0U;
}

/*
 * 多速率候选的采样分频器：TIM1_TRGO(OC4REF) -> TIM2_ITR0 -> TIM2_TRGO(Update)。
 * TIM2 以外部时钟模式 1 只计 OC4REF 上升沿，ARR=ratio-1，因此每两个 24 kHz
 * 有效窗口产生一次 12 kHz ADC 触发。CNT 预置为 ARR，使第一条 OC4REF 就触发，
 * 从而给下一次 RCR 分频 UEV 前的完整控制步留出最大执行时间。
 */
static uint32_t foc_platform_init_adc_trigger_divider(void)
{
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();

    TIM2->CR1 = 0U;
    TIM2->PSC = 0U;
    TIM2->ARR = FOC_PWM_TICKS_PER_CONTROL - 1U;
    TIM2->CR2 = TIM_TRGO_UPDATE;
    TIM2->SMCR = TIM_TS_ITR0 | TIM_SLAVEMODE_EXTERNAL1;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CNT = TIM2->ARR;
    return (((TIM2->CR2 & TIM_CR2_MMS) == TIM_TRGO_UPDATE) &&
            ((TIM2->SMCR & TIM_SMCR_TS) == TIM_TS_ITR0) &&
            ((TIM2->SMCR & TIM_SMCR_SMS) == TIM_SLAVEMODE_EXTERNAL1) &&
            (TIM2->ARR == (FOC_PWM_TICKS_PER_CONTROL - 1U))) ? 1U : 0U;
#else
    return 1U;
#endif
}

static void foc_platform_refresh_safety_flags(void)
{
    uint32_t outputs_active = 0U;
    uint32_t timer_safe = 1U;

    g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_GATE_SAFE |
                                 FOC_PLATFORM_DIAG_DRIVER_FAULT |
                                 FOC_PLATFORM_DIAG_OUTPUT_ACTIVE);

    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
    {
        outputs_active = (((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ||
                          ((TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC2E |
                                         TIM_CCER_CC3E)) != 0U)) ? 1U : 0U;
        timer_safe = (outputs_active == 0U) ? 1U : 0U;
    }

    if ((foc_platform_gate_is_low() != 0U) && (timer_safe != 0U))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_GATE_SAFE;
    }
    if (foc_platform_driver_faulted() != 0U)
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_DRIVER_FAULT;
    }
    if ((foc_platform_gate_is_low() == 0U) || (outputs_active != 0U))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_OUTPUT_ACTIVE;
    }
}

static uint32_t foc_platform_init_timer_disabled(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIMEx_BreakInputConfigTypeDef break_input = {0};
    TIM_OC_InitTypeDef output = {0};
    TIM_BreakDeadTimeConfigTypeDef break_dead_time = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();

#if defined(FOC_ISR_TIMING_PROBE)
    FOC_TIMING_PROBE_LOW();
    gpio.Pin = FOC_TIMING_PROBE_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(FOC_TIMING_PROBE_PORT, &gpio);
#endif

    gpio.Pin = FOC_DRIVER_PROTECTION_PIN;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF12_TIM1_COMP1;
    HAL_GPIO_Init(FOC_DRIVER_PROTECTION_PORT, &gpio);

    gpio.Pin = FOC_PWM_PINS;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(FOC_PWM_PORT, &gpio);

    g_foc_tim1.Instance = TIM1;
    g_foc_tim1.Init.Prescaler = 0U;
    g_foc_tim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    g_foc_tim1.Init.Period = FOC_PWM_PERIOD_TICKS;
    g_foc_tim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    g_foc_tim1.Init.RepetitionCounter = FOC_TIM1_REPETITION_COUNTER;
    g_foc_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&g_foc_tim1) != HAL_OK)
    {
        return 0U;
    }

    master.MasterOutputTrigger = FOC_TIM1_MASTER_TRIGGER;
    master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_foc_tim1, &master) != HAL_OK)
    {
        return 0U;
    }

    break_input.Source = TIM_BREAKINPUTSOURCE_BKIN;
    break_input.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
    break_input.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
    if (HAL_TIMEx_ConfigBreakInput(&g_foc_tim1, TIM_BREAKINPUT_BRK2, &break_input) != HAL_OK)
    {
        return 0U;
    }

    output.OCMode = TIM_OCMODE_PWM1;
    output.Pulse = 0U;
    output.OCPolarity = TIM_OCPOLARITY_HIGH;
    output.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    output.OCFastMode = TIM_OCFAST_DISABLE;
    output.OCIdleState = TIM_OCIDLESTATE_RESET;
    output.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if ((HAL_TIM_PWM_ConfigChannel(&g_foc_tim1, &output, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_foc_tim1, &output, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&g_foc_tim1, &output, TIM_CHANNEL_3) != HAL_OK))
    {
        return 0U;
    }

#if FOC_TIM1_USES_OC4_TRIGGER
    /* 单速率基线在顶点前一个 timer count 拉高 OC4REF；ADC 上升沿先采样，
     * 随后的 UEV 再装载上一控制拍写入的 CCR preload。 */
    output.OCMode = TIM_OCMODE_PWM2;
    output.Pulse = FOC_PWM_PERIOD_TICKS - 1U;
    if (HAL_TIM_PWM_ConfigChannel(&g_foc_tim1, &output, TIM_CHANNEL_4) != HAL_OK)
    {
        return 0U;
    }
#endif

    break_dead_time.OffStateRunMode = TIM_OSSR_ENABLE;
    break_dead_time.OffStateIDLEMode = TIM_OSSI_ENABLE;
    break_dead_time.LockLevel = TIM_LOCKLEVEL_OFF;
    break_dead_time.DeadTime = 0U;
    break_dead_time.BreakState = TIM_BREAK_DISABLE;
    break_dead_time.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    break_dead_time.BreakFilter = 0U;
    break_dead_time.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
    break_dead_time.Break2State = TIM_BREAK2_ENABLE;
    break_dead_time.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    break_dead_time.Break2Filter = 3U;
    break_dead_time.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    break_dead_time.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&g_foc_tim1, &break_dead_time) != HAL_OK)
    {
        return 0U;
    }

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    if (foc_platform_init_adc_trigger_divider() == 0U)
    {
        return 0U;
    }

    g_foc_diagnostics.pwm_frequency_hz = FOC_PWM_FREQUENCY_HZ;
    g_foc_diagnostics.control_frequency_hz = FOC_CONTROL_FREQUENCY_HZ;
    g_foc_diagnostics.pwm_period_ticks = FOC_PWM_PERIOD_TICKS;
    g_foc_diagnostics.pwm_ticks_per_control = FOC_PWM_TICKS_PER_CONTROL;
    g_foc_diagnostics.actuation_delay_pwm_ticks = FOC_ACTUATION_DELAY_PWM_TICKS;
    g_foc_diagnostics.adc_trigger_source = FOC_ADC_TRIGGER_KIND;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_TIM1_CONFIGURED;
    return 1U;
}

static void foc_platform_init_analog_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* PA0=Vbus, PA1=IU, PA7=IW. */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* PB11=IV (available to ADC1 and ADC2). */
    gpio.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* PC0/PC3/PC1=BEMF U/V/W, PC2=potentiometer, PC4=temperature. */
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
#else
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_4;
#endif
    HAL_GPIO_Init(GPIOC, &gpio);

#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    /* 先把 ODR 预装为高，再改输出模式，避免上电时短暂接通分压支路。 */
    HAL_GPIO_WritePin(FOC_BEMF_DIVIDER_ENABLE_PORT,
                      FOC_BEMF_DIVIDER_ENABLE_PIN,
                      GPIO_PIN_SET);
    gpio.Pin = FOC_BEMF_DIVIDER_ENABLE_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FOC_BEMF_DIVIDER_ENABLE_PORT, &gpio);
    HAL_GPIO_WritePin(FOC_BEMF_DIVIDER_ENABLE_PORT,
                      FOC_BEMF_DIVIDER_ENABLE_PIN,
                      GPIO_PIN_SET);
#endif
}

static void foc_platform_fill_adc_init(ADC_HandleTypeDef *adc, ADC_TypeDef *instance)
{
    adc->Instance = instance;
    adc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    adc->Init.Resolution = ADC_RESOLUTION_12B;
    adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    adc->Init.GainCompensation = 0U;
    /* HAL uses ScanConvMode to decide whether injected ranks 2..4 exist. */
    adc->Init.ScanConvMode = ADC_SCAN_ENABLE;
    adc->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    adc->Init.LowPowerAutoWait = DISABLE;
    adc->Init.ContinuousConvMode = DISABLE;
    adc->Init.NbrOfConversion = 1U;
    adc->Init.DiscontinuousConvMode = DISABLE;
    adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc->Init.DMAContinuousRequests = DISABLE;
    adc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    adc->Init.OversamplingMode = DISABLE;
}

static uint32_t foc_platform_read_adc(ADC_HandleTypeDef *adc,
                                      uint32_t channel,
                                      volatile uint16_t *value)
{
    ADC_ChannelConfTypeDef config = {0};

    config.Channel = channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0U;
    if ((HAL_ADC_ConfigChannel(adc, &config) != HAL_OK) ||
        (HAL_ADC_Start(adc) != HAL_OK) ||
        (HAL_ADC_PollForConversion(adc, FOC_ADC_TIMEOUT_MS) != HAL_OK))
    {
        return 0U;
    }
    *value = (uint16_t)HAL_ADC_GetValue(adc);
    return 1U;
}

static uint32_t foc_platform_sample_monitor_inputs(void)
{
    uint32_t ok = 1U;

    if ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_SYNC_RUNNING) == 0U)
    {
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_2,
                                    &g_foc_diagnostics.phase_u_raw);
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_14,
                                    &g_foc_diagnostics.phase_v_raw);
        ok &= foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_4,
                                    &g_foc_diagnostics.phase_w_raw);
    }
    ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_1,
                                &g_foc_diagnostics.bus_voltage_raw);
    ok &= foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_5,
                                &g_foc_diagnostics.temperature_raw);
    ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_8,
                                &g_foc_diagnostics.potentiometer_raw);
    if (ok == 0U)
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_ADC_READ_ERROR;
    }
    else
    {
        g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_ADC_READ_ERROR;
    }
    return ok;
}

static uint32_t foc_platform_calibrate_current_offsets(void)
{
    uint32_t sample;
    uint32_t sum_u = 0U;
    uint32_t sum_v = 0U;
    uint32_t sum_w = 0U;

    for (sample = 0U; sample < FOC_OFFSET_SAMPLE_COUNT; ++sample)
    {
        if ((foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_2,
                                   &g_foc_diagnostics.phase_u_raw) == 0U) ||
            (foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_14,
                                   &g_foc_diagnostics.phase_v_raw) == 0U) ||
            (foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_4,
                                   &g_foc_diagnostics.phase_w_raw) == 0U))
        {
            g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_ADC_READ_ERROR;
            return 0U;
        }
        sum_u += g_foc_diagnostics.phase_u_raw;
        sum_v += g_foc_diagnostics.phase_v_raw;
        sum_w += g_foc_diagnostics.phase_w_raw;
    }

    g_foc_diagnostics.phase_u_offset = (uint16_t)(sum_u / FOC_OFFSET_SAMPLE_COUNT);
    g_foc_diagnostics.phase_v_offset = (uint16_t)(sum_v / FOC_OFFSET_SAMPLE_COUNT);
    g_foc_diagnostics.phase_w_offset = (uint16_t)(sum_w / FOC_OFFSET_SAMPLE_COUNT);
    if ((g_foc_diagnostics.phase_u_offset >= FOC_OFFSET_MIN_VALID) &&
        (g_foc_diagnostics.phase_u_offset <= FOC_OFFSET_MAX_VALID) &&
        (g_foc_diagnostics.phase_v_offset >= FOC_OFFSET_MIN_VALID) &&
        (g_foc_diagnostics.phase_v_offset <= FOC_OFFSET_MAX_VALID) &&
        (g_foc_diagnostics.phase_w_offset >= FOC_OFFSET_MIN_VALID) &&
        (g_foc_diagnostics.phase_w_offset <= FOC_OFFSET_MAX_VALID))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID;
        return 1U;
    }
    return 0U;
}

static uint32_t foc_platform_configure_injected_adc(void)
{
    ADC_InjectionConfTypeDef injected = {0};

    injected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    injected.InjectedSingleDiff = ADC_SINGLE_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedOffset = 0U;
    /* ADC1 IU and ADC2 IV are sampled on the same TIM1 trigger. The third
     * current is reconstructed as -(IU + IV), avoiding the old rank-1/rank-2
     * time skew. */
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    /*
     * ADC1 rank1 保持电流 U 的同步采样；rank2..4 只增加 Diagnostic 端电压
     * U/V/W。ADC2 仍只有电流 V 一个 rank，因此两相电流采样时刻没有改变，
     * 只是 ADC1 JEOS/ISR 入口延后了三个转换。Production 保持原来单 rank。
     */
    injected.InjectedNbrOfConversion = 4U;
#else
    injected.InjectedNbrOfConversion = 1U;
#endif
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = FOC_ADC_EXTERNAL_TRIGGER;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injected.InjecOversamplingMode = DISABLE;

    injected.InjectedChannel = ADC_CHANNEL_2;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    injected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    injected.InjectedChannel = ADC_CHANNEL_6; /* PC0 = BEMF1 = phase U */
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
    injected.InjectedChannel = ADC_CHANNEL_9; /* PC3 = BEMF2 = phase V */
    injected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
    injected.InjectedChannel = ADC_CHANNEL_7; /* PC1 = BEMF3 = phase W */
    injected.InjectedRank = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
#endif
    injected.InjectedNbrOfConversion = 1U;
    injected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    injected.InjectedChannel = ADC_CHANNEL_14;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    return (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc2, &injected) == HAL_OK) ? 1U : 0U;
}

static uint32_t foc_platform_start_sync_monitor(void)
{
    g_foc_diagnostics.sync_sample_count = 0U;
    g_foc_diagnostics.sync_interval_last_cycles = 0U;
    g_foc_diagnostics.sync_interval_min_cycles = 0U;
    g_foc_diagnostics.sync_interval_max_cycles = 0U;
    g_foc_diagnostics.sync_interval_sum_cycles = 0U;
    g_foc_diagnostics.sync_interval_sample_count = 0U;
    g_foc_diagnostics.monitor_isr_last_cycles = 0U;
    g_foc_diagnostics.monitor_isr_min_cycles = 0U;
    g_foc_diagnostics.monitor_isr_max_cycles = 0U;
#if defined(FOC_ISR_TIMING_PROBE)
    g_foc_sync_previous_cycle = 0U;
    g_foc_sync_interval_valid = 0U;
#endif
    __HAL_ADC_CLEAR_FLAG(&g_foc_adc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(&g_foc_adc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    if ((HAL_ADCEx_InjectedStart(&g_foc_adc2) != HAL_OK) ||
        (HAL_ADCEx_InjectedStart_IT(&g_foc_adc1) != HAL_OK))
    {
        return 0U;
    }

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    TIM1->CCR4 = FOC_PWM_PERIOD_TICKS - 1U;
    TIM1->CCER |= TIM_CCER_CC4E;
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
    /* 先 arm 从定时器并预置相位，再启动 TIM1；第一条有效 OC4REF 即产生 ADC 触发。 */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->SR = 0U;
    TIM2->CNT = TIM2->ARR;
    TIM2->CR1 |= TIM_CR1_CEN;
#endif
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CNT = 0U;
    TIM1->CR1 |= TIM_CR1_CEN;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_SYNC_RUNNING;
    return 1U;
}

static uint32_t foc_platform_init_adc_monitor(void)
{
    RCC_PeriphCLKInitTypeDef clock = {0};
    ADC_MultiModeTypeDef multimode = {0};

    clock.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    clock.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
    {
        return 0U;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC12_FORCE_RESET();
    __HAL_RCC_ADC12_RELEASE_RESET();
    foc_platform_init_analog_gpio();
    foc_platform_fill_adc_init(&g_foc_adc1, ADC1);
    foc_platform_fill_adc_init(&g_foc_adc2, ADC2);
    if ((HAL_ADC_Init(&g_foc_adc1) != HAL_OK) ||
        (HAL_ADC_Init(&g_foc_adc2) != HAL_OK))
    {
        return 0U;
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&g_foc_adc1, &multimode) != HAL_OK)
    {
        return 0U;
    }
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_ADC_CONFIGURED;

    if ((HAL_ADCEx_Calibration_Start(&g_foc_adc1, ADC_SINGLE_ENDED) != HAL_OK) ||
        (HAL_ADCEx_Calibration_Start(&g_foc_adc2, ADC_SINGLE_ENDED) != HAL_OK))
    {
        return 0U;
    }
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_ADC_CALIBRATED;
    (void)foc_platform_calibrate_current_offsets();
    if ((foc_platform_sample_monitor_inputs() == 0U) ||
        (foc_platform_configure_injected_adc() == 0U))
    {
        return 0U;
    }
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_PHASE_VOLTAGE_CONFIGURED;
#endif
    return 1U;
}
#endif

foc_status_t foc_platform_init(void)
{
    foc_platform_emergency_stop();
    foc_realtime_timing_reset(&g_foc_timing_stats);
    (void)foc_math_accel_init();
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    foc_phase_voltage_capture_init(&g_foc_phase_voltage_capture,
                                   FOC_CONTROL_FREQUENCY_HZ);
#endif
#if defined(FOC_TARGET_STM32G431)
    /* 先验证纯数据时序契约，再触碰 TIM1/ADC。宏组合错误时保持功率级关闭。 */
    if ((foc_pwm_timing_plan_is_valid(&g_foc_pwm_timing_plan) == 0U) ||
        (foc_platform_init_timer_disabled() == 0U) ||
        (foc_platform_init_adc_monitor() == 0U))
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    foc_platform_emergency_stop();
    if (foc_platform_start_sync_monitor() == 0U)
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    foc_platform_refresh_safety_flags();
    if ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_DRIVER_FAULT) != 0U)
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_HARDWARE_FAULT;
    }
#endif
#if defined(FOC_TARGET_STM32G431)
    return FOC_STATUS_OK;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

/*
 * 应用平台级配置（母线窗口、过流阈值、占空比窗口、ISR 截止）。
 * Applies the platform-level configuration (bus window, trip, duty window,
 * ISR deadline).
 *
 * 校验是"全部通过才生效"：任一字段非法直接返回且不改动任何状态，
 * 避免出现"一半新一半旧"的配置。
 * Validation is all-or-nothing: any invalid field returns without changing
 * anything, avoiding a half-updated configuration.
 *
 * 语法说明 / Note on the syntax: 这里用 `!(a > b)` 而不是 `a <= b`，
 * 是为了同时拒绝 NaN —— NaN 参与任何比较都为假，因此 `!(NaN > 0)` 为真。
 * `!(a > b)` is used instead of `a <= b` so that NaN is rejected too: any
 * comparison with NaN is false, so `!(NaN > 0)` is true.
 *
 * 调用约束 / Constraint: 功率级已 arm 时拒绝修改，返回 FOC_STATUS_DISABLED。
 * Refuses changes while the power stage is armed, returning FOC_STATUS_DISABLED.
 */
foc_status_t foc_platform_configure(const foc_platform_config_t *config)
{
    if ((config == 0) ||
        (config->struct_size != sizeof(foc_platform_config_t)) ||
        (config->config_version != FOC_PLATFORM_CONFIG_VERSION) ||
        !(config->minimum_bus_voltage_v > 0.0f) ||
        !(config->maximum_bus_voltage_v > config->minimum_bus_voltage_v) ||
        !(config->software_current_trip_a > 0.0f) ||
        !(config->minimum_duty >= 0.0f) ||
        !(config->maximum_duty <= 1.0f) ||
        !(config->maximum_duty > config->minimum_duty) ||
        (config->isr_deadline_cycles == 0U))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431)
    if (g_foc_control_armed != 0U)
    {
        return FOC_STATUS_DISABLED;
    }
#endif
    g_foc_platform_config = *config;
    return FOC_STATUS_OK;
}

/*
 * 把 Rust 控制器上下文绑定给平台层，使 ADC ISR 可以驱动快环。
 * Binds the Rust controller context to the platform so the ADC ISR can drive the
 * fast loop.
 *
 * 绑定之前 ISR 不会调用 Rust（g_foc_controller 为空时直接关断），
 * 因此"绑定"实际上是快环启用的第二道门。
 * Before binding, the ISR never calls into Rust: a null g_foc_controller causes
 * an immediate shutdown. Binding is therefore the second enable gate for the
 * fast loop.
 *
 * 尺寸/对齐在运行时复查一次，与编译期 _Static_assert 形成双保险：C 侧的
 * 存储必须真的装得下 Rust 控制器。
 * The size and alignment are re-checked at run time as a second line of defence
 * alongside the compile-time _Static_assert: the C storage must really hold the
 * Rust controller.
 */
foc_status_t foc_platform_bind_controller(foc_rust_context_t *context)
{
#if defined(FOC_TARGET_STM32G431)
    if ((context == 0) ||
        (foc_rust_context_required_size() > sizeof(*context)) ||
        (foc_rust_context_required_align() > _Alignof(foc_rust_context_t)))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    if (g_foc_control_armed != 0U)
    {
        return FOC_STATUS_DISABLED;
    }
    g_foc_controller = context;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROLLER_BOUND;
    return FOC_STATUS_OK;
#else
    (void)context;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

/*
 * 全量紧急关断：栅极、定时器输出、同步时基、诊断位。
 * Full emergency stop: gates, timer outputs, sync time base and diagnostics.
 *
 * 与非实时路径的区别 / Difference from the non-realtime path:
 *   本函数会配置 GPIO 为下拉推挽输出（HAL 调用、较慢），并把 TIM1 的
 *   CEN/CC4E 也清掉，因此同步采样时基也停止；`disable_power_fast()` 只
 *   做寄存器写，保持采样时基运行。
 *   This function reconfigures the GPIO as push-pull with a pull-down via HAL
 *   (slower) and also clears CEN and CC4E, stopping the synchronous sampling
 *   time base. `disable_power_fast()` only writes registers and leaves the
 *   sampling time base running.
 *
 * 用途 / Usage: 初始化、失败返回路径、Shell 停机。不能从快环 ISR 调用。
 * Used by initialization, failure paths and the shell stop command. Must not be
 * called from the fast-loop ISR.
 *
 * 关键点 / Key point: 先把 GPIO 配成"下拉推挽并输出低"，是为了让安全电平
 * 由引脚配置本身保证，而不仅仅依赖 ODR 的值 —— 万一后续有代码把引脚
 * 重配为输入，下拉电阻仍会把栅极使能拉低。
 * The GPIO is configured as push-pull output driving low with a pull-down so
 * that the safe level is guaranteed by the pin configuration, not only by the
 * ODR value: even if later code reconfigures the pin as an input, the pull-down
 * still holds the gate enable low.
 */
void foc_platform_emergency_stop(void)
{
#if defined(FOC_TARGET_STM32G431)
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(FOC_GATE_ENABLE_PORT, FOC_GATE_ENABLE_PINS, GPIO_PIN_RESET);
    gpio.Pin = FOC_GATE_ENABLE_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FOC_GATE_ENABLE_PORT, &gpio);
    HAL_GPIO_WritePin(FOC_GATE_ENABLE_PORT, FOC_GATE_ENABLE_PINS, GPIO_PIN_RESET);

    /* disable_power_fast() 已负责 MOE/CCER/CCR 和 arm 标志。
     * disable_power_fast() already handles MOE, CCER, the CCRs and the arm flag. */
    foc_platform_disable_power_fast();
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
    {
        /* 默认路径的 CH4 与候选路径的 Update TRGO 都依赖 CEN；清 CEN 一定停止
         * 同步采样，额外清 CC4E 让默认路径也回到显式安全状态。 */
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
    }
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
    if ((RCC->APB1ENR1 & RCC_APB1ENR1_TIM2EN) != 0U)
    {
        TIM2->CR1 &= ~TIM_CR1_CEN;
    }
#endif
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_SYNC_RUNNING;
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    foc_phase_voltage_capture_stop(&g_foc_phase_voltage_capture);
    if ((RCC->AHB2ENR & RCC_AHB2ENR_GPIOCEN) != 0U)
    {
        FOC_BEMF_DIVIDER_ENABLE_PORT->BSRR = FOC_BEMF_DIVIDER_ENABLE_PIN;
    }
#endif
    foc_platform_refresh_safety_flags();
#endif
}

/*
 * ADC1/ADC2 注入组转换完成中断 —— 本工程的唯一快环入口。
 * ADC1/ADC2 injected-conversion-complete interrupt: the only fast-loop entry.
 *
 * 执行链 / Execution chain:
 *   读取注入结果 -> 两相采样 + 第三相重构 -> 软件过流与驱动故障检查
 *   -> foc_rust_realtime_step() -> 输出范围复核 -> 写 CCR
 *   -> trace 采样 -> 同拍时序记录 -> 清中断标志
 *
 * 时序测量口径 / Timing measurement convention:
 *   区间从本函数入口开始，到 ADC 标志清理之后结束；同一拍的
 *   total = precontrol + control + postcontrol。之所以强制这个恒等式，
 *   是因为早期版本把三段各自跨所有拍取最大值再相加，得到的"合成拍"
 *   会显著高估预算占用。
 *   The span runs from function entry to after the ADC flags are cleared, and
 *   within one tick total = precontrol + control + postcontrol. The identity is
 *   enforced because an earlier version took each segment's maximum across all
 *   ticks and added them, which overstated the budget.
 *
 * 实时约束 / Real-time constraints:
 *   无动态分配、无阻塞、无日志。实测最坏约 8,200 cycles（约 59% 的 12 kHz
 *   周期），其中 Rust 控制步约占 74%。超出软件截止即计入 deadline miss
 *   并立即关断，不尝试补救。
 *   No allocation, blocking or logging. The measured worst case is about 8,200
 *   cycles (about 59% of the 12 kHz period), of which the Rust control step is
 *   about 74%. Exceeding the software deadline counts as a miss and shuts down
 *   immediately rather than attempting recovery.
 */
#if defined(FOC_TARGET_STM32G431)
void ADC1_2_IRQHandler(void)
{
    uint32_t cycle_start = DWT->CYCCNT;
    uint32_t cycle_end;
    uint32_t control_cycle_start = cycle_start;
    uint32_t control_cycle_end = cycle_start;
    uint32_t control_executed = 0U;
    uint32_t timing_active = 0U;
#if defined(FLUXRT_TRACE_BUILD)
    uint32_t trace_enabled = g_foc_trace_enabled;
#else
    /* 非 trace 档没有 trace，把变量固定为 0，分支会被编译器消除。
     * Profiles without trace use constant 0 and the branch is
     * eliminated by the compiler. */
    uint32_t trace_enabled = 0U;
#endif
    uint32_t trace_sampled = 0U;
    int32_t current_u_counts;
    int32_t current_v_counts;
    int32_t current_w_counts;
    int32_t phase_w_raw;
    uint16_t delta_u;
    uint16_t delta_v;
    uint16_t delta_w;
    uint16_t peak;
    uint16_t trip_counts;

    FOC_TIMING_PROBE_HIGH();
    if ((ADC1->ISR & ADC_ISR_JEOS) != 0U)
    {
#if defined(FOC_ISR_TIMING_PROBE)
        if (g_foc_sync_interval_valid != 0U)
        {
            uint32_t interval_cycles = cycle_start - g_foc_sync_previous_cycle;

            g_foc_diagnostics.sync_interval_last_cycles = interval_cycles;
            g_foc_diagnostics.sync_interval_sum_cycles += interval_cycles;
            ++g_foc_diagnostics.sync_interval_sample_count;
            if ((g_foc_diagnostics.sync_interval_min_cycles == 0U) ||
                (interval_cycles < g_foc_diagnostics.sync_interval_min_cycles))
            {
                g_foc_diagnostics.sync_interval_min_cycles = interval_cycles;
            }
            if (interval_cycles > g_foc_diagnostics.sync_interval_max_cycles)
            {
                g_foc_diagnostics.sync_interval_max_cycles = interval_cycles;
            }
        }
        g_foc_sync_previous_cycle = cycle_start;
        g_foc_sync_interval_valid = 1U;
#endif
        /* ADC1 注入 rank1 采 U 相；ADC2 注入 rank1 采 V 相。
         * 两者由同一个 TIM1 TRGO 触发，因此严格同拍。
         * ADC1 injected rank 1 samples phase U and ADC2 injected rank 1 samples
         * phase V. Both are triggered by the same TIM1 TRGO, so they are exactly
         * co-timed. */
        g_foc_diagnostics.phase_u_raw = (uint16_t)ADC1->JDR1;
        g_foc_diagnostics.phase_v_raw = (uint16_t)ADC2->JDR1;
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
        if (g_foc_phase_voltage_capture.state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED)
        {
            foc_phase_voltage_sample_t phase_sample = {0};

            phase_sample.phase_u_raw = (uint16_t)ADC1->JDR2;
            phase_sample.phase_v_raw = (uint16_t)ADC1->JDR3;
            phase_sample.phase_w_raw = (uint16_t)ADC1->JDR4;
            phase_sample.current_u_raw = g_foc_diagnostics.phase_u_raw;
            phase_sample.current_v_raw = g_foc_diagnostics.phase_v_raw;
            /* 母线值来自最近一次慢速监测，不宣称与本拍严格同步。 */
            phase_sample.bus_voltage_raw = g_foc_diagnostics.bus_voltage_raw;
            (void)foc_phase_voltage_capture_record_isr(
                &g_foc_phase_voltage_capture, &phase_sample);
            if (g_foc_phase_voltage_capture.state ==
                FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE)
            {
                /* 完成即断开分压支路；固定窗口之后不再产生额外板载损耗。 */
                FOC_BEMF_DIVIDER_ENABLE_PORT->BSRR =
                    FOC_BEMF_DIVIDER_ENABLE_PIN;
            }
        }
#endif
        /* 极性约定为 (offset - raw)：正电流使采样电压下降。
         * 写反会让 Id 与 Iq 同时变号 —— 电流环依然"稳定"，但观测器方向
         * 持续错误。这个约定与 IHM16M1/MCSDK 参考工程一致。
         * Polarity is (offset - raw): positive current lowers the sampled
         * voltage. Reversing it flips both Id and Iq; the loop still looks
         * stable while the observer stays wrong. This matches the IHM16M1 /
         * MCSDK reference. */
        current_u_counts = (int32_t)g_foc_diagnostics.phase_u_offset -
                           (int32_t)g_foc_diagnostics.phase_u_raw;
        current_v_counts = (int32_t)g_foc_diagnostics.phase_v_offset -
                           (int32_t)g_foc_diagnostics.phase_v_raw;
        /* 第三相由基尔霍夫定律重构，而不是单独采样：三相电流和为零，
         * 这样避开了第三路 ADC 与另两路的时序偏斜，也省下一个注入通道。
         * The third phase is reconstructed from Kirchhoff's law rather than
         * sampled, since the three currents sum to zero. This avoids the timing
         * skew of a third ADC channel and frees an injected rank. */
        current_w_counts = -current_u_counts - current_v_counts;
        /* W 相没有原始 ADC 值，这里反推一个"等效原始值"仅用于显示和 trace，
         * 因此必须钳位到 12 位量程，否则会打印出 4095 以上的假值。
         * Phase W has no raw ADC value; the equivalent raw value is derived only
         * for display and trace, so it must be clamped to the 12-bit range or it
         * would report values above 4095. */
        phase_w_raw = (int32_t)g_foc_diagnostics.phase_w_offset - current_w_counts;
        if (phase_w_raw < 0)
        {
            phase_w_raw = 0;
        }
        else if (phase_w_raw > 4095)
        {
            phase_w_raw = 4095;
        }
        g_foc_diagnostics.phase_w_raw = (uint16_t)phase_w_raw;
        ++g_foc_diagnostics.sync_sample_count;
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID;
        if (g_foc_control_armed != 0U)
        {
            foc_status_t control_status;
            foc_feedback_t feedback;
            foc_output_t output;
            foc_telemetry_t telemetry;

            timing_active = 1U;
            /* 取三相电流绝对偏移的最大值作为跳闸判据。用最大值而不是
             * 某一相，是为了在任意相序或任意单相故障下都能触发。
             * The trip uses the largest absolute offset of the three phases, so
             * it fires regardless of phase order or which single phase faults. */
            delta_u = (uint16_t)((current_u_counts < 0) ? -current_u_counts : current_u_counts);
            delta_v = (uint16_t)((current_v_counts < 0) ? -current_v_counts : current_v_counts);
            delta_w = (uint16_t)((current_w_counts < 0) ? -current_w_counts : current_w_counts);
            peak = (delta_u > delta_v) ? delta_u : delta_v;
            peak = (peak > delta_w) ? peak : delta_w;
            if (peak > g_foc_diagnostics.peak_current_delta_counts)
            {
                g_foc_diagnostics.peak_current_delta_counts = peak;
            }
            /* 三道独立检查：软件过流、驱动器故障引脚、控制器未绑定。
             * 任一命中都立即关断功率级且不调用 Rust。
             * Three independent checks: software over-current, the driver fault
             * pin, and an unbound controller. Any of them shuts the power stage
             * down immediately without calling into Rust. */
            trip_counts = foc_platform_current_trip_counts();
            if ((peak > trip_counts) ||
                (foc_platform_driver_faulted() != 0U) ||
                (g_foc_controller == 0))
            {
                if (peak > trip_counts)
                {
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CURRENT_TRIP;
                }
                foc_platform_disable_power_fast();
            }
            else
            {
                /* counts -> 安培 的浮点换算放在这里而不是 Rust 侧：
                 * 平台层拥有标定常量（分流电阻、放大倍数、Vref），
                 * Rust 侧只看到 SI 单位，换板子时不需要重新标定算法。
                 * The counts-to-ampere conversion lives here rather than in
                 * Rust: the platform owns the calibration constants (shunt,
                 * gain, Vref) and Rust only ever sees SI units, so changing the
                 * board does not require re-validating the algorithms. */
                feedback.phase_current_a = (float)current_u_counts /
                                           FOC_CURRENT_COUNTS_PER_AMP;
                feedback.phase_current_b = (float)current_v_counts /
                                           FOC_CURRENT_COUNTS_PER_AMP;
                feedback.phase_current_c = (float)current_w_counts /
                                           FOC_CURRENT_COUNTS_PER_AMP;
                feedback.dc_bus_voltage =
                    ((float)g_foc_diagnostics.bus_voltage_raw * FOC_ADC_REFERENCE_VOLTAGE) /
                    (FOC_ADC_FULL_SCALE * FOC_BUS_PARTITIONING_FACTOR);
                feedback.electrical_angle_rad = 0.0f;
                control_cycle_start = DWT->CYCCNT;
                control_executed = 1U;
                control_status = foc_rust_realtime_step(g_foc_controller,
                                                        &feedback,
                                                        &output,
                                                        &telemetry);
                control_cycle_end = DWT->CYCCNT;
                g_foc_diagnostics.last_control_status = control_status;
                if (control_status != FOC_STATUS_OK)
                {
                    g_foc_diagnostics.control_fault_flags =
                        foc_rust_fault_flags(g_foc_controller);
                    g_foc_diagnostics.last_duty_a_per_mille =
                        (uint16_t)(output.duty_a * 1000.0f + 0.5f);
                    g_foc_diagnostics.last_duty_b_per_mille =
                        (uint16_t)(output.duty_b * 1000.0f + 0.5f);
                    g_foc_diagnostics.last_duty_c_per_mille =
                        (uint16_t)(output.duty_c * 1000.0f + 0.5f);
                    ++g_foc_diagnostics.realtime_error_count;
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
                    foc_platform_disable_power_fast();
                }
                else if (!(output.duty_a >= g_foc_platform_config.minimum_duty &&
                           output.duty_a <= g_foc_platform_config.maximum_duty) ||
                         !(output.duty_b >= g_foc_platform_config.minimum_duty &&
                           output.duty_b <= g_foc_platform_config.maximum_duty) ||
                         !(output.duty_c >= g_foc_platform_config.minimum_duty &&
                           output.duty_c <= g_foc_platform_config.maximum_duty))
                {
                    g_foc_diagnostics.control_fault_flags =
                        foc_rust_fault_flags(g_foc_controller);
                    g_foc_diagnostics.last_duty_a_per_mille =
                        (uint16_t)(output.duty_a * 1000.0f + 0.5f);
                    g_foc_diagnostics.last_duty_b_per_mille =
                        (uint16_t)(output.duty_b * 1000.0f + 0.5f);
                    g_foc_diagnostics.last_duty_c_per_mille =
                        (uint16_t)(output.duty_c * 1000.0f + 0.5f);
                    ++g_foc_diagnostics.realtime_error_count;
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_OUTPUT_REJECTED;
                    foc_platform_disable_power_fast();
                }
                else
                {
                    TIM1->CCR1 = (uint32_t)(output.duty_a * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
                    TIM1->CCR2 = (uint32_t)(output.duty_b * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
                    TIM1->CCR3 = (uint32_t)(output.duty_c * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
                    ++g_foc_diagnostics.realtime_step_count;
#if defined(FLUXRT_TRACE_BUILD)
                    trace_sampled = foc_platform_trace_capture(&feedback, &output, &telemetry);
                    /* trace 采样拍已经把同一份 telemetry 写入诊断环形缓冲；该拍不再
                     * 立刻复制整个全局快照，下一控制拍会更新它（最多滞后 1/12 kHz）。
                     * 这消除诊断拍上的重复结构体搬运，而不降低控制或保护采样率。
                     * A sampled trace tick already serialises the same telemetry into
                     * the diagnostic ring. Skip the duplicate global-structure copy on
                     * that tick; the next 12 kHz tick refreshes it, so shell telemetry
                     * is at most one control period stale while control/safety remain
                     * full-rate. */
                    if (trace_sampled == 0U)
                    {
                        g_foc_telemetry = telemetry;
                    }
#else
                    g_foc_telemetry = telemetry;
#endif
                }
            }
            if (control_executed == 0U)
            {
                control_cycle_start = DWT->CYCCNT;
                control_cycle_end = control_cycle_start;
            }
        }
        ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
        FOC_TIMING_PROBE_LOW();
#if defined(FOC_ISR_TIMING_PROBE)
        if (timing_active == 0U)
        {
            uint32_t monitor_cycles = DWT->CYCCNT - cycle_start;

            g_foc_diagnostics.monitor_isr_last_cycles = monitor_cycles;
            if ((g_foc_diagnostics.monitor_isr_min_cycles == 0U) ||
                (monitor_cycles < g_foc_diagnostics.monitor_isr_min_cycles))
            {
                g_foc_diagnostics.monitor_isr_min_cycles = monitor_cycles;
            }
            if (monitor_cycles > g_foc_diagnostics.monitor_isr_max_cycles)
            {
                g_foc_diagnostics.monitor_isr_max_cycles = monitor_cycles;
            }
        }
#endif
        if (timing_active != 0U)
        {
            foc_realtime_timing_sample_t timing_sample;

            cycle_end = DWT->CYCCNT;
            timing_sample.step = g_foc_diagnostics.realtime_step_count;
            timing_sample.total_cycles = cycle_end - cycle_start;
            timing_sample.precontrol_cycles = control_cycle_start - cycle_start;
            timing_sample.control_cycles = control_cycle_end - control_cycle_start;
            timing_sample.postcontrol_cycles = cycle_end - control_cycle_end;
            timing_sample.trace_enabled = (trace_enabled != 0U) ? 1U : 0U;
            timing_sample.trace_sampled = trace_sampled;
            (void)foc_realtime_timing_record(&g_foc_timing_stats, &timing_sample);

            /* Compatibility fields remain independent peaks; never add them. */
            g_foc_diagnostics.maximum_isr_cycles =
                g_foc_timing_stats.wcet.total_cycles;
            g_foc_diagnostics.maximum_precontrol_cycles =
                g_foc_timing_stats.peak_precontrol_cycles;
            g_foc_diagnostics.maximum_control_cycles =
                g_foc_timing_stats.peak_control_cycles;
            g_foc_diagnostics.maximum_postcontrol_cycles =
                g_foc_timing_stats.peak_postcontrol_cycles;
            if (timing_sample.total_cycles > g_foc_platform_config.isr_deadline_cycles)
            {
                ++g_foc_diagnostics.deadline_miss_count;
                g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_DEADLINE_MISSED;
                foc_platform_disable_power_fast();
            }
        }
    }
    else
    {
        FOC_TIMING_PROBE_LOW();
    }
}

/*
 * TIM1 Break 中断（BKIN / BKIN2），即硬件过流或驱动器故障路径。
 * TIM1 Break interrupt (BKIN / BKIN2): the hardware over-current or driver
 * fault path.
 *
 * 这条路径通常比软件快：一旦 STSPIN830 拉低 PA11，TIM1 硬件立即清除 MOE
 * 并强制输出到空闲电平，**不依赖本中断**。本中断的作用是记录故障、
 * 关断栅极使能、并停掉同步采样时基。
 * This path is normally slower than the hardware: as soon as the STSPIN830
 * pulls PA11 low, TIM1 clears MOE in hardware and forces the outputs to their
 * idle state without any interrupt involved. This handler records the fault,
 * cuts the gate enables and stops the sampling time base.
 *
 * 优先级被设为 0（最高），高于 ADC ISR 的 1：故障处理必须能打断正在执行的
 * 控制步。
 * Priority is 0 (highest), above the ADC ISR at 1, so fault handling can
 * preempt an in-flight control step.
 */
void TIM1_BRK_TIM15_IRQHandler(void)
{
    if ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U)
    {
        /* 先清标志再处理，避免在处理期间重复进入。
         * Clear the flags first so re-entry cannot happen during handling. */
        TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_B2IF);
        ++g_foc_diagnostics.break_fault_count;
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_BREAK_LATCHED;
        foc_platform_disable_power_fast();
        /* 同步采样时基也停掉：栅极已断，继续采样只会产生误导性的数据。
         * Also stop the sampling time base: with the gates cut, continued
         * sampling would only produce misleading data. */
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
        TIM2->CR1 &= ~TIM_CR1_CEN;
#endif
        g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_SYNC_RUNNING;
    }
}
#endif

/*
 * 读取一份反馈快照（非实时路径）。
 * Reads one feedback snapshot (non-realtime path).
 *
 * 与 ISR 内的路径不同，本函数使用**软件触发**轮询 ADC，因此数值不与 PWM
 * 同步，只能用于静态监测、零点检查和 Shell 显示，不能用于闭环。
 * Unlike the ISR path, this reads the ADC with software-triggered polling, so
 * the values are not PWM-synchronised. It is suitable only for static
 * monitoring, offset checks and shell display, never for closed loop.
 *
 * 边界 / Boundary: angle 保持为 0 并返回 NOT_CONFIGURED —— 当前没有转子
 * 位置传感器，本函数不会伪造一个角度。
 * The angle is left at 0 and NOT_CONFIGURED is returned: there is no rotor
 * position sensor, and this function does not fabricate an angle.
 */
foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback)
{
    if (feedback == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    /* 先全部清零，保证任何提前返回路径下调用方看到的都是确定的 0。
     * Zero everything first so every early return leaves defined values. */
    feedback->phase_current_a = 0.0f;
    feedback->phase_current_b = 0.0f;
    feedback->phase_current_c = 0.0f;
    feedback->dc_bus_voltage = 0.0f;
    feedback->electrical_angle_rad = 0.0f;
#if defined(FOC_TARGET_STM32G431)
    if ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_ADC_CONFIGURED) == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    /* 已经 arm 时不再软件触发采样：ISR 正在以 12 kHz 更新同一批字段，
     * 此时再轮询 ADC 会与 ISR 争用注入组并破坏刚采到的数据。
     * Once armed, software sampling is skipped: the ISR is updating the same
     * fields at 12 kHz, and polling the ADC now would contend for the injected
     * group and corrupt a fresh sample. */
    if ((g_foc_control_armed == 0U) && (foc_platform_sample_monitor_inputs() == 0U))
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }

    /* IHM16M1/MCSDK polarity is offset minus sample. */
    feedback->phase_current_a =
        (float)((int32_t)g_foc_diagnostics.phase_u_offset -
                (int32_t)g_foc_diagnostics.phase_u_raw) / FOC_CURRENT_COUNTS_PER_AMP;
    feedback->phase_current_b =
        (float)((int32_t)g_foc_diagnostics.phase_v_offset -
                (int32_t)g_foc_diagnostics.phase_v_raw) / FOC_CURRENT_COUNTS_PER_AMP;
    /* 与 ISR 一致：第三相由基尔霍夫定律重构，不单独采样。
     * Consistent with the ISR: the third phase is reconstructed, not sampled. */
    feedback->phase_current_c = -feedback->phase_current_a - feedback->phase_current_b;
    feedback->dc_bus_voltage =
        ((float)g_foc_diagnostics.bus_voltage_raw * FOC_ADC_REFERENCE_VOLTAGE) /
        (FOC_ADC_FULL_SCALE * FOC_BUS_PARTITIONING_FACTOR);
    foc_platform_refresh_safety_flags();
    if ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_DRIVER_FAULT) != 0U)
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }
#endif
#if defined(FOC_TARGET_STM32G431)
    return ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID) != 0U) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

/*
 * 把一相占空比写进 TIM1 比较寄存器（兼容/早期试转路径）。
 * Writes a three-phase duty into the TIM1 compare registers (compatibility /
 * early trial path).
 *
 * 与 ISR 内写字机的区别 / Difference from the ISR write path: 本函数在写
 * 之前和之后都做完整复核，因为它可能被管理线程调用，缺乏 ISR 的固定节拍
 * 保证。实时路径不经过本函数。
 * This function re-checks thoroughly before and after writing because it may be
 * called from a management thread without the ISR's fixed cadence. The realtime
 * path does not go through it.
 *
 * 返回 / Returns:
 *   FOC_STATUS_OK            已写入且复核通过 / written and re-verified
 *   FOC_STATUS_DISABLED      未 arm，未写寄存器 / not armed, no write
 *   FOC_STATUS_INVALID_ARGUMENT 占空比越界，已关断 / out of range, shut down
 *   FOC_STATUS_HARDWARE_FAULT 有故障，已关断 / fault present, shut down
 */
foc_status_t foc_platform_apply_output(const foc_output_t *output)
{
    if (output == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

#if defined(FOC_TARGET_STM32G431)
    if (g_foc_control_armed == 0U)
    {
        return FOC_STATUS_DISABLED;
    }
    if ((foc_platform_driver_faulted() != 0U) ||
        ((g_foc_diagnostics.flags & (FOC_PLATFORM_DIAG_BREAK_LATCHED |
                                     FOC_PLATFORM_DIAG_CURRENT_TRIP |
                                     FOC_PLATFORM_DIAG_ADC_READ_ERROR)) != 0U))
    {
        foc_platform_disable_power_fast();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    /* The comparisons deliberately reject NaN as well as out-of-range duty. */
    if (!(output->duty_a >= g_foc_platform_config.minimum_duty &&
          output->duty_a <= g_foc_platform_config.maximum_duty) ||
        !(output->duty_b >= g_foc_platform_config.minimum_duty &&
          output->duty_b <= g_foc_platform_config.maximum_duty) ||
        !(output->duty_c >= g_foc_platform_config.minimum_duty &&
          output->duty_c <= g_foc_platform_config.maximum_duty))
    {
        /* 越界即关断，不钳位后继续。钳位会让一个错误的控制输出看起来
         * "被容忍了"，掩盖上游的算法故障。
         * Out of range means shutdown, not clamp-and-continue: clamping would
         * make a wrong control output look tolerated and hide an upstream
         * algorithm fault. */
        foc_platform_disable_power_fast();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    /* 归一化占空比 -> CCR counts，+0.5 实现四舍五入。
     * Normalised duty to CCR counts, with +0.5 for rounding. */
    TIM1->CCR1 = (uint32_t)((output->duty_a * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    TIM1->CCR2 = (uint32_t)((output->duty_b * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    TIM1->CCR3 = (uint32_t)((output->duty_c * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    ++g_foc_diagnostics.trial_apply_count;
    /* 写后复核：如果在这几条指令之间驱动器报了故障，必须立刻撤销而不是
     * 等下一个小周期。
     * Post-write re-check: if the driver reported a fault between these
     * instructions, it must be undone immediately rather than on the next
     * period. */
    if ((g_foc_control_armed == 0U) || (foc_platform_driver_faulted() != 0U))
    {
        foc_platform_disable_power_fast();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    return FOC_STATUS_OK;
#else
    foc_platform_emergency_stop();
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

/*
 * 使能功率级：全部门通过后 arm 栅极与 PWM 通道。
 * Enables the power stage: arms the gates and PWM channels once every gate
 * passes.
 *
 * 这是本工程唯一会让栅极使能变高的函数，因此所有前置检查都集中在这里。
 * This is the only function in the project that can raise the gate enables, so
 * every precondition is concentrated here.
 *
 * 门禁条件 / Gating conditions (all must hold):
 *   1. 控制器已绑定且当前未 arm（arm 是一次性的）；
 *   2. 一次同步反馈读取成功；
 *   3. FOC_TRIAL_REQUIRED_FLAGS 全部置位；
 *   4. FOC_TRIAL_FORBIDDEN_FLAGS 一个都没置位；
 *   5. 母线电压落在配置窗口内（ADC 原始值比较）。
 *   1. Controller bound and not already armed (arming is one-shot); 2. one
 *   synchronised feedback read succeeds; 3. all FOC_TRIAL_REQUIRED_FLAGS set;
 *   4. no FOC_TRIAL_FORBIDDEN_FLAGS set; 5. bus voltage inside the configured
 *   window.
 *
 * 上电顺序 / Power-up order: 先写中性占空比并产生一次更新事件，让三相在
 * 栅极开启之前就已经处于 50% 的对称状态；再置 CCER/MOE/栅极。反过来会让
 * 第一拍带着上一轮的残留占空比上电。
 * The neutral duty is written and an update event generated before the gates
 * open, so all three phases are already at a symmetric 50% before the gate
 * enables are raised. The opposite order would power up with the previous
 * period's stale duty on the first tick.
 *
 * 参数 / Parameters: target_speed_rpm 为闭环目标转速 [rpm]；开环阶段不使用
 * 它，但要先通过 Rust 的合法性检查（必须 >= 升速终点且 <= 最高转速）。
 * target_speed_rpm is the closed-loop target in rpm; the open-loop phase does
 * not use it, but Rust validates it first.
 */
foc_status_t foc_platform_control_start(float target_speed_rpm)
{
#if defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD)
    /* Calibration 是编译期只采集档。即使应用层或未来脚本误调用 start，平台层
     * 仍先执行硬关断再拒绝，保证不存在绕过 Shell 的 arm 路径。 */
    (void)target_speed_rpm;
    foc_platform_emergency_stop();
    return FOC_STATUS_DISABLED;
#elif defined(FOC_TARGET_STM32G431)
    uint32_t neutral_ticks;
    uint16_t bus_min_raw;
    uint16_t bus_max_raw;
    foc_feedback_t feedback;
    foc_status_t status;

    if ((g_foc_controller == 0) || (g_foc_control_armed != 0U)
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
        || (g_foc_phase_voltage_capture.state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED)
#endif
       )
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    status = foc_platform_read_feedback(&feedback);
    if (status != FOC_STATUS_OK)
    {
        return status;
    }
    foc_platform_refresh_safety_flags();
    bus_min_raw = foc_platform_bus_voltage_to_raw(g_foc_platform_config.minimum_bus_voltage_v);
    bus_max_raw = foc_platform_bus_voltage_to_raw(g_foc_platform_config.maximum_bus_voltage_v);
    if (((g_foc_diagnostics.flags & FOC_TRIAL_REQUIRED_FLAGS) != FOC_TRIAL_REQUIRED_FLAGS) ||
        ((g_foc_diagnostics.flags & FOC_TRIAL_FORBIDDEN_FLAGS) != 0U) ||
        ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_CONTROLLER_BOUND) == 0U) ||
        (g_foc_diagnostics.bus_voltage_raw < bus_min_raw) ||
        (g_foc_diagnostics.bus_voltage_raw > bus_max_raw))
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    status = foc_rust_start_realtime(g_foc_controller, 1U, target_speed_rpm);
    if (status != FOC_STATUS_OK)
    {
        return status;
    }

    neutral_ticks = FOC_PWM_PERIOD_TICKS / 2U;
    g_foc_diagnostics.peak_current_delta_counts = 0U;
    g_foc_diagnostics.trial_apply_count = 0U;
    g_foc_diagnostics.realtime_step_count = 0U;
    g_foc_diagnostics.realtime_error_count = 0U;
    foc_realtime_timing_reset(&g_foc_timing_stats);
    g_foc_diagnostics.maximum_isr_cycles = 0U;
    g_foc_diagnostics.maximum_precontrol_cycles = 0U;
    g_foc_diagnostics.maximum_control_cycles = 0U;
    g_foc_diagnostics.maximum_postcontrol_cycles = 0U;
    g_foc_diagnostics.deadline_miss_count = 0U;
    g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
    g_foc_diagnostics.control_fault_flags = 0U;
    g_foc_diagnostics.last_duty_a_per_mille = 500U;
    g_foc_diagnostics.last_duty_b_per_mille = 500U;
    g_foc_diagnostics.last_duty_c_per_mille = 500U;
    g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_CURRENT_TRIP |
                                 FOC_PLATFORM_DIAG_DEADLINE_MISSED |
                                 FOC_PLATFORM_DIAG_CONTROL_ERROR |
                                 FOC_PLATFORM_DIAG_OUTPUT_REJECTED);
    TIM1->CCR1 = neutral_ticks;
    TIM1->CCR2 = neutral_ticks;
    TIM1->CCR3 = neutral_ticks;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_B2IF);
    TIM1->DIER |= TIM_DIER_BIE;

    g_foc_control_armed = 1U;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_TRIAL_ARMED |
                               FOC_PLATFORM_DIAG_REALTIME_ARMED;
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    FOC_GATE_ENABLE_PORT->BSRR = FOC_GATE_ENABLE_PINS;
    if ((g_foc_control_armed == 0U) || (foc_platform_driver_faulted() != 0U))
    {
        foc_platform_disable_power_fast();
        foc_rust_stop(g_foc_controller);
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    foc_platform_refresh_safety_flags();
    return FOC_STATUS_OK;
#else
    (void)target_speed_rpm;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

void foc_platform_control_stop(void)
{
#if defined(FOC_TARGET_STM32G431)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    foc_platform_disable_power_fast();
    if (g_foc_controller != 0)
    {
        foc_rust_stop(g_foc_controller);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    foc_platform_refresh_safety_flags();
#endif
}

foc_status_t foc_platform_trial_arm(void)
{
    return foc_platform_control_start(582.0f);
}

void foc_platform_trial_disarm(void)
{
#if defined(FOC_TARGET_STM32G431)
    foc_platform_control_stop();
#endif
}

foc_status_t foc_platform_get_telemetry(foc_telemetry_t *telemetry)
{
    if (telemetry == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *telemetry = g_foc_telemetry;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
#else
    {
        foc_telemetry_t empty = {0};
        *telemetry = empty;
    }
#endif
    return FOC_STATUS_OK;
}

foc_status_t foc_platform_trace_start(uint32_t sample_divider)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_TRACE_BUILD)
    uint32_t primask;
    if (sample_divider < FOC_TRACE_MIN_DIVIDER)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    g_foc_trace_head = 0U;
    g_foc_trace_tail = 0U;
    g_foc_trace_counter = 0U;
    g_foc_trace_dropped_count = 0U;
    g_foc_trace_divider = sample_divider;
    g_foc_trace_enabled = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return FOC_STATUS_OK;
#else
    (void)sample_divider;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

void foc_platform_trace_stop(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_TRACE_BUILD)
    g_foc_trace_enabled = 0U;
#endif
}

uint32_t foc_platform_trace_is_enabled(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_TRACE_BUILD)
    return g_foc_trace_enabled;
#else
    return 0U;
#endif
}

uint32_t foc_platform_trace_pop(foc_trace_sample_t *sample)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_TRACE_BUILD)
    foc_trace_raw_sample_t raw;
    uint32_t tail;
    if (sample == 0)
    {
        return 0U;
    }
    tail = g_foc_trace_tail;
    if (tail == g_foc_trace_head)
    {
        return 0U;
    }
    /* 单生产者/单消费者环形缓冲不需要关中断：生产者发布 head 前已 DMB，
     * 消费者先完整复制到线程栈，再发布 tail。这样较大的原始快照也不会阻塞
     * 12 kHz ADC ISR。
     * The SPSC ring needs no IRQ masking: the producer publishes head only after
     * its DMB; the consumer first copies the complete raw item to its thread stack
     * and only then publishes tail. A larger raw snapshot therefore never blocks
     * the 12 kHz ADC ISR. */
    raw = g_foc_trace_buffer[tail];
    __DMB();
    g_foc_trace_tail = (tail + 1U) & (FOC_TRACE_CAPACITY - 1U);
    foc_platform_trace_encode(&raw, sample);
    return 1U;
#else
    (void)sample;
    return 0U;
#endif
}

uint32_t foc_platform_trace_dropped(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_TRACE_BUILD)
    return g_foc_trace_dropped_count;
#else
    return 0U;
#endif
}

foc_status_t foc_platform_phase_voltage_capture_start(
    foc_phase_voltage_divider_mode_t divider_mode)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    uint32_t primask;

    if ((divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_DISABLED) &&
        (divider_mode != FOC_PHASE_VOLTAGE_DIVIDER_ENABLED))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    if ((g_foc_control_armed != 0U) ||
        ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_SYNC_RUNNING) == 0U))
    {
        return FOC_STATUS_DISABLED;
    }
    if ((g_foc_diagnostics.flags &
         FOC_PLATFORM_DIAG_PHASE_VOLTAGE_CONFIGURED) == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (foc_phase_voltage_capture_arm(&g_foc_phase_voltage_capture,
                                      divider_mode) == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_DISABLED;
    }
    /* IO_BEMF active-low. off/异常路径始终保持 PC9 high（网络断开）。 */
    if (divider_mode == FOC_PHASE_VOLTAGE_DIVIDER_ENABLED)
    {
        FOC_BEMF_DIVIDER_ENABLE_PORT->BSRR =
            ((uint32_t)FOC_BEMF_DIVIDER_ENABLE_PIN << 16U);
    }
    else
    {
        FOC_BEMF_DIVIDER_ENABLE_PORT->BSRR = FOC_BEMF_DIVIDER_ENABLE_PIN;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return FOC_STATUS_OK;
#else
    (void)divider_mode;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

void foc_platform_phase_voltage_capture_stop(void)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    foc_phase_voltage_capture_stop(&g_foc_phase_voltage_capture);
    FOC_BEMF_DIVIDER_ENABLE_PORT->BSRR = FOC_BEMF_DIVIDER_ENABLE_PIN;
    if (primask == 0U)
    {
        __enable_irq();
    }
#endif
}

foc_status_t foc_platform_phase_voltage_capture_status(
    foc_phase_voltage_capture_status_t *status)
{
    if (status == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        foc_phase_voltage_capture_get_status(&g_foc_phase_voltage_capture, status);
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
    return FOC_STATUS_OK;
#else
    foc_phase_voltage_capture_get_status(0, status);
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

uint32_t foc_platform_phase_voltage_capture_pop(
    foc_phase_voltage_sample_t *sample)
{
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    return foc_phase_voltage_capture_pop(&g_foc_phase_voltage_capture, sample);
#else
    (void)sample;
    return 0U;
#endif
}

foc_status_t foc_platform_get_phase_voltage_model(
    foc_phase_voltage_model_t *model)
{
    if (model == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    *model = (foc_phase_voltage_model_t){0};
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    if (foc_phase_voltage_model_validate(
            &g_foc_phase_voltage_nominal_model) == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    *model = g_foc_phase_voltage_nominal_model;
    return FOC_STATUS_OK;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_phase_voltage_raw_to_mv(
    foc_phase_voltage_divider_mode_t divider_mode,
    uint16_t raw,
    uint32_t *phase_voltage_mv)
{
    if (phase_voltage_mv == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    *phase_voltage_mv = 0U;
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    return (foc_phase_voltage_raw_to_mv(&g_foc_phase_voltage_nominal_model,
                                        divider_mode,
                                        raw,
                                        phase_voltage_mv) != 0U) ?
        FOC_STATUS_OK : FOC_STATUS_INVALID_ARGUMENT;
#else
    (void)divider_mode;
    (void)raw;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_get_diagnostics(foc_platform_diagnostics_t *diagnostics)
{
    if (diagnostics == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431)
    {
        uint32_t primask;

        primask = __get_PRIMASK();
        __disable_irq();
        foc_platform_refresh_safety_flags();
        *diagnostics = g_foc_diagnostics;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
#else
    {
        foc_platform_diagnostics_t empty = {0};
        *diagnostics = empty;
    }
#endif
    return FOC_STATUS_OK;
}

foc_status_t foc_platform_get_timing(foc_realtime_timing_stats_t *timing)
{
    if (timing == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431)
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        *timing = g_foc_timing_stats;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }
#else
    *timing = g_foc_timing_stats;
#endif
    return FOC_STATUS_OK;
}
