/*
 * FluxRT —— STM32G431 芯片平台适配层。
 * FluxRT - STM32G431 chip platform adapter.
 *
 * 职责 / Responsibility:
 *   - 配置 TIM1 中心对齐 PWM、ADC1/ADC2 注入同步采样、栅极使能与硬件关断；
 *   - 实现快环 ISR（ADC1_2_IRQHandler），在其中有且只有一次 Rust realtime
 *     调用（默认旧入口，P4.2E1 候选为 combined motion 入口）；
 *   - 持有并发布诊断、遥测、时序统计和 trace 缓冲。
 *     Configures TIM1 centre-aligned PWM, ADC1/ADC2 injected synchronous
 *     sampling, gate enables and the hardware shutdown path; implements the
 *     fast-loop ISR with exactly one Rust realtime call (the legacy entry by
 *     default, or the P4.2E1 combined motion candidate); and owns diagnostics,
 *     telemetry, timing statistics and the trace buffer.
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

#include <string.h>

#include "foc_platform.h"
#include "foc_arm_diagnostics.h"
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
#include "foc_platform_as5600_alignment_candidate.h"
#endif
#include "foc_lsi_actuation_executor.h"
#include "foc_lsi_capture_service.h"
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
#include "foc_lsi_management.h"
#endif
#include "foc_lsi_preload_sink_internal.h"
#include "foc_math_accel.h"
#include "foc_monitor_sampling.h"
#include "foc_platform_advanced_candidate.h"
#include "foc_platform_motion_candidate.h"
#include "foc_power_safety.h"
#include "foc_pwm_timing.h"
#include "foc_sensorless_platform.h"
#include "foc_time_sync.h"
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
#include "foc_external_input_platform.h"
#endif

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

#define FOC_CONTROL_START_AUTHORITY_GENERIC  (0U)
#define FOC_CONTROL_START_AUTHORITY_MOTION   (1U)
#define FOC_CONTROL_START_AUTHORITY_ADVANCED (2U)
#define FOC_CONTROL_START_AUTHORITY_AS5600_ALIGNMENT (3U)

#if defined(FOC_TARGET_STM32G431)
#include "rtconfig.h"
#include "stm32g4xx_hal.h"
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
#include "rtthread.h"
#endif

#include <string.h>

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
/* Standard IHM16M1 assembly fits R35 and R37 as 0-ohm links.  PB12 is unused
 * by the G431 product profile, so the Identification image may briefly use it
 * as an open-drain-low stimulus for the passive external fault network. */
#define FOC_BREAK_EXTERNAL_STIMULUS_PORT GPIOB
#define FOC_BREAK_EXTERNAL_STIMULUS_PIN  GPIO_PIN_12
#define FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS (4096UL)
/* Match the official MCSDK bounded clear budget, but additionally require a
 * short run of coherent clear/high samples before accepting the history as
 * stale. This executes only while every physical output is closed. */
#define FOC_BREAK2_REARM_MAX_ATTEMPTS    (1000UL)
#define FOC_BREAK2_REARM_STABLE_READS    (8UL)
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
/* trace 缓冲只在 Diagnostic 档编译。E1B motion probe 和外部 I/O 无功率
 * 探针镜像都还要从堆中创建 4 KiB main 与 4 KiB Finsh 线程；若仍
 * 保留 64 项原始 trace，静态 BSS 会让线程创建失败。这两种镜像都不依赖
 * 长 trace 记录，因此缩为 16 项；普通 Diagnostic 仍保持 64 项。
 * The trace ring is Diagnostic-only.  Both the E1B motion probe and the
 * no-power external-I/O probe still need heap for the 4 KiB main and Finsh
 * threads.  A 64-entry raw ring exhausts that heap, while neither probe relies
 * on a long trace capture, so those dedicated images use 16 entries and normal
 * Diagnostic images retain 64. */
#if defined(FOC_MOTION_CONTROL_CANDIDATE) || \
    defined(FOC_EXTERNAL_IO_FRAMEWORK)
#define FOC_TRACE_CAPACITY               (16U)
#else
#define FOC_TRACE_CAPACITY               (64U)
#endif
/* 最小分频 = 120 -> 12 kHz 下最高 100 Hz 采样。
 * Minimum divider 120 gives at most 100 Hz sampling at 12 kHz. */
#define FOC_TRACE_MIN_DIVIDER            (120U)
#endif
#if defined(FOC_ISR_TIMING_PROBE) || \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD) || \
    defined(FOC_EXTERNAL_IO_FRAMEWORK)
/* Identification 的无功率门必须能量化增加同步 Vbus rank 后的 ISR 开销，
 * 外部 I/O 无功率探针同样需要对比 ADC1 regular 占用前后的间隔和包络，
 * 因此即使没有占用 PA5 的物理探针，也保留 DWT 周期统计。 */
#define FOC_MONITOR_DWT_TIMING            (1U)
#endif
#if defined(FOC_MONITOR_DWT_TIMING) || \
    defined(FLUXRT_H3_EDGE_CONTROL_TICK_BUILD)
/* The H3 edge-to-control-tick mapping is a separate requirement from the
 * optional ISR timing statistics above.  The dynamic H3 image intentionally
 * omits the heavier monitor report, but it still has to latch the latest ADC
 * control boundary.  Both fields cross from the higher-priority ADC ISR to
 * the lower-priority TIM4 ISR, so keep the accesses observable to the compiler. */
#define FOC_SYNC_EDGE_CONTROL_TICK         (1U)
static volatile uint32_t g_foc_sync_previous_cycle;
static volatile uint32_t g_foc_sync_interval_valid;
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
                                           FOC_PLATFORM_DIAG_BUS_VOLTAGE_TRIP | \
                                           FOC_PLATFORM_DIAG_DEADLINE_MISSED | \
                                           FOC_PLATFORM_DIAG_CONTROL_ERROR | \
                                           FOC_PLATFORM_DIAG_OUTPUT_REJECTED | \
                                           FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR | \
                                           FOC_PLATFORM_DIAG_TRIAL_ARMED)
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
#define FOC_LSI_REQUIRED_FLAGS            (FOC_TRIAL_REQUIRED_FLAGS | \
                                           FOC_PLATFORM_DIAG_LSI_SYNC_BUS_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_LSI_ACTUATION_PLAN_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_LSI_EXECUTOR_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_LSI_PRELOAD_SINK_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_LSI_ACTIVE_SESSION_CONFIGURED)
#define FOC_LSI_FORBIDDEN_FLAGS           (FOC_TRIAL_FORBIDDEN_FLAGS | \
                                           FOC_PLATFORM_DIAG_DEADLINE_MISSED | \
                                           FOC_PLATFORM_DIAG_CONTROL_ERROR | \
                                           FOC_PLATFORM_DIAG_OUTPUT_REJECTED | \
                                           FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING)
#define FOC_LSI_START_MAX_CURRENT_A       (0.05f)
#endif

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
/* The C platform is the single owner of power-stage state. fault_epoch is
 * captured before Rust runs and rechecked immediately before a CCR commit, so a
 * higher-priority Break interrupt invalidates the in-flight result. */
static foc_power_safety_t g_foc_power_safety;
static volatile uint32_t g_foc_power_safety_initialized;
static volatile uint32_t g_foc_pending_rust_faults;
static volatile uint32_t g_foc_control_sequence;
static volatile uint16_t g_foc_bus_min_raw;
static volatile uint16_t g_foc_bus_max_raw;
/* 绑定的 Rust 控制器上下文；由 foc_platform_bind_controller() 设置一次。
 * Bound Rust controller context; set once by foc_platform_bind_controller(). */
static foc_rust_context_t *g_foc_controller;

#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
static foc_as5600_alignment_trial_t g_foc_as5600_alignment_trial;
#endif

static foc_status_t foc_platform_control_start_internal(
    float target_speed_rpm,
    uint32_t candidate_authority);

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
static foc_advanced_probe_t g_foc_advanced_probe;
static volatile uint32_t g_foc_advanced_probe_active;

static uint32_t foc_platform_advanced_enter_critical(void)
{
    uint32_t was_enabled =
        (NVIC_GetEnableIRQ(ADC1_2_IRQn) != 0U) ? 1U : 0U;
    NVIC_DisableIRQ(ADC1_2_IRQn);
    __DMB();
    return was_enabled;
}

static void foc_platform_advanced_exit_critical(uint32_t key)
{
    __DMB();
    if (key != 0U)
    {
        NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
static foc_advanced_probe_input_t g_foc_advanced_probe_input;
/* The no-power probe and powered owner are mutually exclusive.  Sharing their
 * latest telemetry/snapshot preserves the Advanced-Lab heap gate on the 32 KiB
 * target.  The powered management API reconstructs its two published words
 * from the owner state, so it never interprets the compact union member as full
 * telemetry. */
typedef union
{
    foc_advanced_telemetry_t telemetry;
    foc_advanced_power_trial_snapshot_t power_trial_snapshot;
} foc_advanced_shared_status_t;
_Static_assert(sizeof(foc_advanced_shared_status_t) ==
               sizeof(foc_advanced_telemetry_t),
               "Advanced shared status must not grow target BSS");
static foc_advanced_shared_status_t g_foc_advanced_shared_status;
static foc_advanced_power_trial_t g_foc_advanced_power_trial;
static float g_foc_advanced_probe_nominal_bus_voltage_v;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
static foc_sensorless_context_t g_foc_sensorless_probe_context;
static foc_sensorless_realtime_output_t g_foc_sensorless_probe_output;
static foc_sensorless_voltage_output_t g_foc_sensorless_probe_voltage_output;
static foc_sensorless_composite_output_t g_foc_sensorless_composite_output;
static volatile uint32_t g_foc_sensorless_probe_mode;
/* Composite (mode 3) probe state.  The formal combined entry consumes the prior
 * tick's applied injection, so the platform must carry that ledger across the
 * ISR boundary.  These objects live only in the compile-time motor-arm-disabled
 * commissioning image and are cleared at every probe start/finish. */
static foc_sensorless_composite_output_t g_foc_sensorless_composite_probe_output;
static volatile uint32_t g_foc_sensorless_composite_applied_sequence;
static volatile float g_foc_sensorless_composite_applied_alpha_v;
static volatile float g_foc_sensorless_composite_applied_beta_v;
static volatile uint32_t g_foc_sensorless_composite_applied_limited;
#define FOC_SENSORLESS_COMMISSIONING_BUS_VOLTAGE_V (12.3f)
#define FOC_SENSORLESS_COMMISSIONING_VOLTAGE_LIMIT_V (6.5f)
/* Mode 3 exercises the formal combined transaction in the real ADC ISR while
 * every output enable stays off.  It is not a drive mode and grants no
 * capability; the public provider stays zero. */
#define FOC_SENSORLESS_PROBE_MODE_COMPOSITE (3UL)
/* The combined entry runs foc_rust_realtime_step_core(), whose state gate only
 * accepts the five startup-chain states and returns Disabled for everything
 * else.  Mode 3 therefore has to put the controller into Alignment through the
 * normal foc_rust_start_realtime() ABI, exactly as the P5.3 advanced probe
 * does.  The controller is stopped again when the probe finishes, and no
 * output can reach the pins because every commit stays behind
 * g_foc_control_armed.  The target speed only feeds the startup sequencer that
 * this probe never follows; it is fixed, not operator-controlled. */
#define FOC_SENSORLESS_COMMISSIONING_TARGET_SPEED_RPM (582.0f)
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
/* P4.2E1 ownership: these objects are private to this platform translation
 * unit.  Management can publish only through the fixed dispatcher API; once
 * the route is committed, the ADC ISR is the sole Rust-state writer. */
static foc_motion_context_t g_foc_motion_context;
static foc_motion_dispatcher_t g_foc_motion_dispatcher;
static foc_motion_realtime_request_t g_foc_motion_realtime_request;
static foc_motion_output_t g_foc_motion_realtime_output;
static foc_motion_probe_t g_foc_motion_probe;
static foc_motion_torque_trial_t g_foc_motion_torque_trial;
static volatile uint32_t g_foc_motion_runtime_enabled;
static volatile uint32_t g_foc_motion_command_ready;
static volatile uint32_t g_foc_motion_probe_execute_motion;
static float g_foc_motion_probe_nominal_bus_voltage_v;
#define FOC_MOTION_PROBE_WARMUP_TICKS       (12U)

/* Mask only the consuming ADC IRQ.  The priority-0 timer Break interrupt stays
 * deliverable, and the timer's asynchronous Break action is never masked. */
static uint32_t foc_platform_motion_enter_critical(void *context)
{
    uint32_t was_enabled;

    (void)context;
    was_enabled = (NVIC_GetEnableIRQ(ADC1_2_IRQn) != 0U) ? 1U : 0U;
    NVIC_DisableIRQ(ADC1_2_IRQn);
    __DMB();
    return was_enabled;
}

static void foc_platform_motion_exit_critical(void *context, uint32_t key)
{
    (void)context;
    __DMB();
    if (key != 0U)
    {
        NVIC_EnableIRQ(ADC1_2_IRQn);
    }
}

static foc_status_t foc_platform_motion_status_to_foc(
    foc_motion_status_t status)
{
    switch (status)
    {
    case FOC_MOTION_STATUS_OK:
        return FOC_STATUS_OK;
    case FOC_MOTION_STATUS_INVALID_ARGUMENT:
    case FOC_MOTION_STATUS_INVALID_CONFIG:
    case FOC_MOTION_STATUS_INVALID_COMMAND:
    case FOC_MOTION_STATUS_INVALID_FEEDBACK:
        return FOC_STATUS_INVALID_ARGUMENT;
    case FOC_MOTION_STATUS_NOT_INITIALIZED:
    case FOC_MOTION_STATUS_NOT_CONFIGURED:
        return FOC_STATUS_NOT_CONFIGURED;
    case FOC_MOTION_STATUS_DISABLED:
    case FOC_MOTION_STATUS_TRANSITION_REQUIRED:
    case FOC_MOTION_STATUS_INVALID_STATE:
        return FOC_STATUS_DISABLED;
    case FOC_MOTION_STATUS_CONTROL_FAILURE:
    default:
        return FOC_STATUS_HARDWARE_FAULT;
    }
}

static foc_status_t foc_platform_motion_dispatch_result_to_foc(
    foc_motion_dispatcher_result_t result)
{
    if (result == FOC_MOTION_DISPATCHER_OK)
    {
        return FOC_STATUS_OK;
    }
    if (result == FOC_MOTION_DISPATCHER_INVALID_ARGUMENT)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    return FOC_STATUS_NOT_CONFIGURED;
}

/* The motion bundle and the already configured base FOC context must be one
 * transaction.  Exact equality is deliberate: this is an enable gate, not a
 * tolerant estimator comparison, and any independently rounded or stale copy
 * must be reconciled before the route can own power output. */
static uint32_t foc_platform_motion_config_matches_base(
    const foc_config_bundle_t *config,
    const foc_runtime_config_t *base,
    uint32_t active_bundle_revision)
{
    if ((config == 0) || (base == 0) || (active_bundle_revision == 0U) ||
        (config->struct_size != (uint32_t)sizeof(*config)) ||
        (config->abi_version != FOC_CONFIG_ABI_VERSION) ||
        (config->bundle_revision != active_bundle_revision) ||
        (base->struct_size != (uint32_t)sizeof(*base)) ||
        (base->config_version != FOC_RUST_CONFIG_VERSION) ||
        (config->board.pwm_frequency_hz != FOC_PWM_FREQUENCY_HZ) ||
        (config->board.control_frequency_hz != FOC_CONTROL_FREQUENCY_HZ) ||
        (base->pwm_frequency_hz != FOC_CONTROL_FREQUENCY_HZ) ||
        (config->motor.pole_pairs != base->pole_pairs) ||
        (config->motor.phase_resistance_ohm != base->stator_resistance_ohm) ||
        (config->motor.d_inductance_h != base->stator_inductance_h) ||
        (config->motor.q_inductance_h != base->stator_inductance_h) ||
        (config->motor.flux_linkage_v_s != base->flux_linkage_wb) ||
        (config->motor.continuous_current_a != base->rated_current_a) ||
        (config->axis.current_kp != base->id_pi.kp) ||
        (config->axis.current_kp != base->iq_pi.kp) ||
        (config->axis.current_ki != base->id_pi.ki) ||
        (config->axis.current_ki != base->iq_pi.ki) ||
        (base->id_pi.ts != (1.0f / (float)FOC_CONTROL_FREQUENCY_HZ)) ||
        (base->iq_pi.ts != (1.0f / (float)FOC_CONTROL_FREQUENCY_HZ)))
    {
        return 0U;
    }
    return 1U;
}
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
/* S4.9 keeps the physical capability private and exposes only the bounded
 * platform start operation.  The ADC ISR is the sole active-session caller. */
static foc_lsi_executor_t g_foc_lsi_executor;
static foc_lsi_preload_session_t g_foc_lsi_preload_session;
static foc_power_arm_token_t g_foc_lsi_arm_token;
static volatile uint32_t g_foc_lsi_session_running;
static volatile foc_lsi_drive_request_t g_foc_lsi_active_drive_request;
#endif
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
    /* `head` still points at an unpublished SPSC slot.  Fill that slot
     * directly instead of first constructing a large stack snapshot and then
     * copying the complete struct a second time.  The consumer cannot observe
     * this slot until the final head publication below, so the ordering and
     * public trace ABI stay unchanged while the sampled ISR path performs only
     * one payload copy. */
    g_foc_trace_buffer[head].step =
        g_foc_diagnostics.realtime_step_count;
    g_foc_trace_buffer[head].flags = g_foc_diagnostics.flags;
    g_foc_trace_buffer[head].feedback = *feedback;
    g_foc_trace_buffer[head].output = *output;
    g_foc_trace_buffer[head].telemetry = *telemetry;
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
static void foc_platform_disable_output_registers_fast(void)
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
    if (g_foc_power_safety_initialized != 0U)
    {
        foc_power_safety_stop(&g_foc_power_safety);
    }
    g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_TRIAL_ARMED |
                                 FOC_PLATFORM_DIAG_REALTIME_ARMED);
}

static void foc_platform_disable_power_fast(void)
{
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    /* Fault/abort paths close the evidence window before touching outputs. */
    foc_lsi_capture_service_stop();
#endif
    foc_platform_disable_output_registers_fast();
}

static uint32_t foc_platform_gate_is_low(void);
static uint32_t foc_platform_driver_faulted(void);

/* Latch first, then cut power.  The pending Rust mirror is intentionally not
 * consumed from the higher-priority Break ISR because it may have interrupted
 * Rust itself.  ADC/thread context drains it only after no Rust call is active. */
static __attribute__((noinline)) void foc_platform_latch_fault_fast(
    uint32_t platform_fault,
    uint32_t rust_fault)
{
    if (g_foc_diagnostics.power_fault_count == 0U)
    {
        g_foc_diagnostics.first_power_fault = platform_fault;
    }
    g_foc_diagnostics.last_power_fault = platform_fault;
    ++g_foc_diagnostics.power_fault_count;
    foc_power_safety_latch_fault(&g_foc_power_safety, platform_fault);
    g_foc_diagnostics.power_fault_epoch = g_foc_power_safety.fault_epoch;
    g_foc_pending_rust_faults |= rust_fault;
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_torque_trial_fail(
        &g_foc_motion_torque_trial,
        FOC_MOTION_TORQUE_TRIAL_RESULT_PLATFORM_FAULT,
        g_foc_power_safety.fault_epoch);
    /* A fault revokes the candidate route immediately.  This also closes the
     * configure-vs-Break race without ever masking the priority-0 Break IRQ. */
    g_foc_motion_runtime_enabled = 0U;
    g_foc_motion_command_ready = 0U;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_power_trial_fail(
        &g_foc_advanced_power_trial,
        FOC_ADVANCED_POWER_TRIAL_RESULT_PLATFORM_FAULT,
        g_foc_power_safety.fault_epoch,
        g_foc_diagnostics.deadline_miss_count);
    if (g_foc_advanced_probe.state == FOC_ADVANCED_PROBE_RUNNING)
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            g_foc_power_safety.fault_epoch);
    }
    g_foc_advanced_probe_active = 0U;
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    foc_as5600_alignment_trial_fail(
        &g_foc_as5600_alignment_trial,
        FOC_AS5600_ALIGNMENT_TRIAL_RESULT_PLATFORM_FAULT,
        g_foc_power_safety.fault_epoch);
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
    if (g_foc_advanced_probe.state == FOC_ADVANCED_PROBE_RUNNING)
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED,
            g_foc_power_safety.fault_epoch);
    }
    g_foc_advanced_probe_active = 0U;
#endif
    foc_platform_disable_power_fast();
}

static void foc_platform_mirror_pending_rust_fault(void)
{
    uint32_t primask;
    uint32_t faults;

    if ((g_foc_controller == 0) ||
        (g_foc_power_safety.state == FOC_POWER_SAFETY_ARMING))
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    faults = g_foc_pending_rust_faults;
    g_foc_pending_rust_faults = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (faults != 0U)
    {
        (void)foc_rust_latch_fault(g_foc_controller, faults);
        g_foc_diagnostics.control_fault_flags =
            foc_rust_fault_flags(g_foc_controller);
    }
}

/* Atomic software commit around the hardware Break path. Interrupt masking does
 * not mask the timer's asynchronous Break action: MOE still drops and BIF/B2IF
 * still latch. The post-write register check therefore detects a Break that
 * arrived between the state check and the preload writes. */
static inline __attribute__((always_inline)) uint32_t
foc_platform_commit_output(const foc_output_t *output,
                           uint32_t expected_fault_epoch)
{
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t permitted;

    /* This helper has one caller: ADC1_2_IRQHandler. Exception entry does not
     * set PRIMASK, so it is known to be zero here. Re-enable immediately after
     * the short register transaction to let the priority-0 Break handler run;
     * the asynchronous timer Break action remains effective throughout. */
    __disable_irq();
    permitted = foc_power_safety_output_permitted_inline(
        &g_foc_power_safety, expected_fault_epoch);
    if ((permitted == 0U) ||
        ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
        ((TIM1->CCER & channel_mask) != channel_mask) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) ||
        (foc_platform_gate_is_low() != 0U) ||
        (foc_platform_driver_faulted() != 0U))
    {
        permitted = 0U;
    }
    if (permitted != 0U)
    {
        TIM1->CCR1 = (uint32_t)(output->duty_a *
                                (float)FOC_PWM_PERIOD_TICKS + 0.5f);
        TIM1->CCR2 = (uint32_t)(output->duty_b *
                                (float)FOC_PWM_PERIOD_TICKS + 0.5f);
        TIM1->CCR3 = (uint32_t)(output->duty_c *
                                (float)FOC_PWM_PERIOD_TICKS + 0.5f);
        __DSB();
        if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) ||
            (foc_platform_driver_faulted() != 0U))
        {
            permitted = 0U;
        }
    }
    if (permitted == 0U)
    {
        foc_platform_latch_fault_fast(
            (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ?
                FOC_POWER_FAULT_BREAK : FOC_POWER_FAULT_PLATFORM),
            FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    __enable_irq();
    /* A pending priority-0 Break handler runs before this check after PRIMASK is
     * restored, invalidating the epoch even if the register check raced it. */
    return foc_power_safety_output_permitted_inline(
        &g_foc_power_safety, expected_fault_epoch);
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

/* Re-arm only a historical Break2 latch. A live PA11 fault, BIF, enabled
 * output, enabled Break IRQ or non-disabled safety state is a hard blocker.
 * Hardware Break remains enabled in BDTR throughout; the short critical
 * section only prevents software IRQ delivery while the sticky flag is
 * cleared and sampled. Any reassertion after this function is caught by the
 * existing pre/post arm checks and by the asynchronous timer Break action. */
#if !defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD) || \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
    defined(FLUXRT_AS5600_ALIGNMENT_BUILD) || \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
static uint16_t foc_platform_rearm_break2_before_arm(void)
{
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t attempts;
    uint32_t stable_reads = 0U;
    uint32_t primask;
    uint16_t facts;

    if ((TIM1->SR & TIM_SR_B2IF) == 0U)
    {
        return 0U;
    }

    facts = FOC_ARM_REJECT_FACT_B2IF |
        foc_arm_diagnostics_rearm_blockers(
            TIM1->SR,
            TIM_SR_BIF,
            foc_platform_driver_faulted(),
            (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) ? 1U : 0U,
            ((TIM1->CCER & channel_mask) == 0U) ? 1U : 0U,
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) ? 1U : 0U,
            foc_platform_gate_is_low(),
            ((TIM1->DIER & TIM_DIER_BIE) == 0U) ? 1U : 0U);
    if (facts != FOC_ARM_REJECT_FACT_B2IF)
    {
        return facts;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    facts = FOC_ARM_REJECT_FACT_B2IF |
        foc_arm_diagnostics_rearm_blockers(
            TIM1->SR,
            TIM_SR_BIF,
            foc_platform_driver_faulted(),
            (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) ? 1U : 0U,
            ((TIM1->CCER & channel_mask) == 0U) ? 1U : 0U,
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) ? 1U : 0U,
            foc_platform_gate_is_low(),
            ((TIM1->DIER & TIM_DIER_BIE) == 0U) ? 1U : 0U);
    if (facts == FOC_ARM_REJECT_FACT_B2IF)
    {
        for (attempts = 0U;
             attempts < FOC_BREAK2_REARM_MAX_ATTEMPTS;
             ++attempts)
        {
            if (foc_platform_driver_faulted() != 0U)
            {
                facts |= FOC_ARM_REJECT_FACT_DRIVER;
                break;
            }
            /* TIM status flags clear by writing zero to the selected bit and
             * one to the others; this is the same operation as
             * LL_TIM_ClearFlag_BRK2(). */
            TIM1->SR = ~TIM_SR_B2IF;
            __DSB();
            if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) == 0U) &&
                (foc_platform_driver_faulted() == 0U))
            {
                ++stable_reads;
                if (stable_reads >= FOC_BREAK2_REARM_STABLE_READS)
                {
                    facts |= FOC_ARM_REJECT_FACT_B2IF_REARMED;
                    break;
                }
            }
            else
            {
                stable_reads = 0U;
            }
        }
    }
    {
        uint16_t final_blockers = foc_arm_diagnostics_rearm_blockers(
            TIM1->SR,
            TIM_SR_BIF,
            foc_platform_driver_faulted(),
            (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) ? 1U : 0U,
            ((TIM1->CCER & channel_mask) == 0U) ? 1U : 0U,
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) ? 1U : 0U,
            foc_platform_gate_is_low(),
            ((TIM1->DIER & TIM_DIER_BIE) == 0U) ? 1U : 0U);

        facts |= final_blockers;
        if ((final_blockers != 0U) ||
            ((TIM1->SR & TIM_SR_B2IF) != 0U))
        {
            facts &= (uint16_t)~FOC_ARM_REJECT_FACT_B2IF_REARMED;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return facts;
}
#endif

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
/* One coherent register fact set for the E1B no-power guard.  The snapshot is
 * deliberately built from physical registers, not from the route flag. */
static void foc_platform_motion_probe_snapshot(
    foc_motion_probe_register_snapshot_t *snapshot)
{
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;

    /* Every field in the fixed 32-byte snapshot is assigned below.  Do not
     * clear the object first: the redundant stores execute several times per
     * probe tick and add noise to the very WCET path this guard measures. */
    snapshot->control_armed = g_foc_control_armed;
    snapshot->power_safety_state = g_foc_power_safety.state;
    snapshot->fault_epoch = g_foc_power_safety.fault_epoch;
    snapshot->break_latched =
        ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ? 1U : 0U;
    snapshot->driver_faulted = foc_platform_driver_faulted();
    snapshot->gate_is_low = foc_platform_gate_is_low();
    snapshot->moe_enabled =
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    snapshot->phase_channels_enabled =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
}

static uint32_t foc_platform_motion_probe_inject_isr(
    foc_motion_probe_injection_point_t point)
{
    if (foc_motion_probe_take_injection(&g_foc_motion_probe, point) == 0U)
    {
        return 0U;
    }
    foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                  FOC_RUST_FAULT_PLATFORM_INPUT);
    return 1U;
}

/* Write preload registers while every physical enable remains off, then prove
 * that the output path and fault epoch stayed unchanged.  AFTER_COMMIT is
 * injected after the three CCR stores but before the post-write snapshot, so
 * the same check covers the final race window. */
static foc_motion_probe_result_t foc_platform_motion_probe_commit_isr(
    const foc_output_t *output)
{
    foc_motion_probe_register_snapshot_t before;
    foc_motion_probe_register_snapshot_t after;

    foc_platform_motion_probe_snapshot(&before);
    TIM1->CCR1 = (uint32_t)(output->duty_a *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR2 = (uint32_t)(output->duty_b *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR3 = (uint32_t)(output->duty_c *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    __DSB();
    (void)foc_platform_motion_probe_inject_isr(
        FOC_MOTION_PROBE_INJECT_AFTER_COMMIT);
    foc_platform_motion_probe_snapshot(&after);
    return foc_motion_probe_complete_commit(&g_foc_motion_probe,
                                            &before,
                                            &after);
}

static foc_status_t foc_platform_motion_probe_step_isr(
    int32_t current_u_counts,
    int32_t current_v_counts,
    int32_t current_w_counts)
{
    foc_motion_probe_register_snapshot_t snapshot;
    foc_motion_dispatcher_result_t dispatch_result;
    foc_motion_probe_result_t probe_result;
    foc_feedback_t feedback;
    foc_realtime_input_t input;
    foc_output_t output;
    foc_status_t control_status = FOC_STATUS_NOT_CONFIGURED;
    uint32_t fault_was_latched = 0U;

    (void)memset(&output, 0, sizeof(output));
    foc_platform_motion_probe_snapshot(&snapshot);
    probe_result = foc_motion_probe_begin_tick(&g_foc_motion_probe,
                                               &snapshot);
    if (probe_result != FOC_MOTION_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }
    if (foc_platform_motion_probe_inject_isr(
            FOC_MOTION_PROBE_INJECT_BEFORE_COMBINED) != 0U)
    {
        fault_was_latched = 1U;
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }

    dispatch_result = foc_motion_dispatcher_consume_isr(
        &g_foc_motion_dispatcher,
        (uint32_t)rt_tick_get_millisecond(),
        &g_foc_motion_realtime_request);
    if (dispatch_result != FOC_MOTION_DISPATCHER_OK)
    {
        control_status = FOC_STATUS_NOT_CONFIGURED;
        goto fail_closed;
    }

    feedback.phase_current_a = (float)current_u_counts /
                               FOC_CURRENT_COUNTS_PER_AMP;
    feedback.phase_current_b = (float)current_v_counts /
                               FOC_CURRENT_COUNTS_PER_AMP;
    feedback.phase_current_c = (float)current_w_counts /
                               FOC_CURRENT_COUNTS_PER_AMP;
    /* E1B is a no-power execution probe: the physical bus is deliberately 0 V,
     * but the numerical controller requires a positive bus voltage for SVPWM
     * normalisation.  Use the configured nominal voltage only as an algorithm
     * input.  The probe's hardware snapshots still prove Gate/MOE/CCER are off,
     * and this path never commits the calculated duty to TIM1. */
    feedback.dc_bus_voltage = g_foc_motion_probe_nominal_bus_voltage_v;
    feedback.electrical_angle_rad = 0.0f;
    foc_realtime_input_from_legacy(&feedback,
                                   g_foc_control_sequence,
                                   1.0f /
                                       (float)FOC_CONTROL_FREQUENCY_HZ,
                                   &input);
    /* The powered route cannot execute a motion reference during Alignment.
     * Give the controller one real outer-loop period on the normal combined
     * path before the diagnostic entry forces ClosedLoop motion.  This avoids
     * an impossible cold-start + outer-loop coincidence while the same hard
     * 12,750-cycle deadline still covers every warm motion/observer overlap. */
    if ((g_foc_motion_probe_execute_motion != 0U) &&
        (g_foc_motion_probe.executed_ticks >=
         FOC_MOTION_PROBE_WARMUP_TICKS))
    {
        control_status = foc_rust_realtime_step_with_motion_no_power(
            g_foc_controller,
            &g_foc_motion_context,
            &input,
            &g_foc_motion_realtime_request,
            &output,
            0,
            &g_foc_motion_realtime_output);
    }
    else
    {
        /* Same combined ABI and platform path, but normal Alignment semantics
         * skip motion reference generation.  The delta to mode 1 is the
         * isolated target estimate used by the E1B timing gate. */
        control_status = foc_rust_realtime_step_with_motion(
            g_foc_controller,
            &g_foc_motion_context,
            &input,
            &g_foc_motion_realtime_request,
            &output,
            0,
            &g_foc_motion_realtime_output);
    }
    ++g_foc_control_sequence;

    if (foc_platform_motion_probe_inject_isr(
            FOC_MOTION_PROBE_INJECT_DURING_COMBINED) != 0U)
    {
        fault_was_latched = 1U;
    }
    foc_platform_motion_probe_snapshot(&snapshot);
    probe_result = foc_motion_probe_complete_combined(
        &g_foc_motion_probe, control_status, &snapshot);
    if ((probe_result != FOC_MOTION_PROBE_RESULT_OK) ||
        (control_status != FOC_STATUS_OK))
    {
        goto fail_closed;
    }
    if (!(output.duty_a >= g_foc_platform_config.minimum_duty &&
          output.duty_a <= g_foc_platform_config.maximum_duty) ||
        !(output.duty_b >= g_foc_platform_config.minimum_duty &&
          output.duty_b <= g_foc_platform_config.maximum_duty) ||
        !(output.duty_c >= g_foc_platform_config.minimum_duty &&
          output.duty_c <= g_foc_platform_config.maximum_duty))
    {
        probe_result = foc_motion_probe_fail(
            &g_foc_motion_probe,
            FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE,
            g_foc_power_safety.fault_epoch);
        (void)probe_result;
        control_status = FOC_STATUS_INVALID_ARGUMENT;
        goto fail_closed;
    }
    if (foc_platform_motion_probe_inject_isr(
            FOC_MOTION_PROBE_INJECT_BEFORE_COMMIT) != 0U)
    {
        fault_was_latched = 1U;
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }

    probe_result = foc_platform_motion_probe_commit_isr(&output);
    if ((probe_result == FOC_MOTION_PROBE_RESULT_OK) ||
        (probe_result == FOC_MOTION_PROBE_RESULT_COMPLETE))
    {
        ++g_foc_diagnostics.realtime_step_count;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        return FOC_STATUS_OK;
    }
    fault_was_latched =
        (g_foc_power_safety.state == FOC_POWER_SAFETY_FAULT_LATCHED) ?
            1U : fault_was_latched;
    control_status = FOC_STATUS_HARDWARE_FAULT;

fail_closed:
    foc_platform_motion_probe_snapshot(&snapshot);
    if (g_foc_motion_probe.state == FOC_MOTION_PROBE_RUNNING)
    {
        (void)foc_motion_probe_fail(
            &g_foc_motion_probe,
            (snapshot.fault_epoch !=
             g_foc_motion_probe.expected_fault_epoch) ?
                FOC_MOTION_PROBE_RESULT_FAULT_EPOCH_CHANGED :
                FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE,
            snapshot.fault_epoch);
    }
    ++g_foc_diagnostics.realtime_error_count;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
    g_foc_diagnostics.last_control_status = control_status;
    if (fault_was_latched == 0U)
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_CONTROL,
                                      FOC_RUST_FAULT_MOTION_CONTROL);
    }
    return control_status;
}
#endif

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
static inline void foc_platform_advanced_probe_snapshot(
    foc_advanced_probe_register_snapshot_t *snapshot)
{
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;

    /* Every field is assigned below.  A preceding 32-byte memset performed
     * three times per ISR added probe overhead without initializing any byte
     * that the explicit hardware snapshot did not immediately overwrite. */
    snapshot->control_armed = g_foc_control_armed;
    snapshot->power_safety_state = g_foc_power_safety.state;
    snapshot->fault_epoch = g_foc_power_safety.fault_epoch;
    snapshot->break_latched =
        ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ? 1U : 0U;
    snapshot->driver_faulted = foc_platform_driver_faulted();
    snapshot->gate_is_low = foc_platform_gate_is_low();
    snapshot->moe_enabled =
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    snapshot->phase_channels_enabled =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
}
#endif

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
static foc_advanced_probe_result_t foc_platform_advanced_probe_commit_isr(
    const foc_output_t *output,
    foc_status_t control_status)
{
    foc_advanced_probe_register_snapshot_t before;
    foc_advanced_probe_register_snapshot_t after;
    foc_advanced_probe_result_t result;

    /* This is both the post-Rust and immediately-pre-CCR snapshot.  No hardware
     * write exists between those two logical gates, so taking two identical
     * snapshots only inflated the measured ISR. */
    foc_platform_advanced_probe_snapshot(&before);
    result = foc_advanced_probe_complete_control(&g_foc_advanced_probe,
                                                 control_status,
                                                 &before);
    if (result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        return result;
    }
    TIM1->CCR1 = (uint32_t)(output->duty_a *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR2 = (uint32_t)(output->duty_b *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR3 = (uint32_t)(output->duty_c *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    __DSB();
    foc_platform_advanced_probe_snapshot(&after);
    return foc_advanced_probe_complete_commit(&g_foc_advanced_probe,
                                              &before,
                                              &after);
}

static foc_status_t foc_platform_advanced_probe_step_isr(
    int32_t current_u_counts,
    int32_t current_v_counts,
    int32_t current_w_counts)
{
    foc_advanced_probe_register_snapshot_t snapshot;
    foc_advanced_probe_result_t probe_result;
    foc_feedback_t feedback;
    foc_realtime_input_t input;
    foc_output_t output = {0};
    foc_status_t control_status;

    foc_platform_advanced_probe_snapshot(&snapshot);
    probe_result = foc_advanced_probe_begin_tick(&g_foc_advanced_probe,
                                                 &snapshot);
    if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }

    (void)current_u_counts;
    (void)current_v_counts;
    (void)current_w_counts;
    feedback.phase_current_a = g_foc_advanced_probe_input.phase_current_a;
    feedback.phase_current_b = g_foc_advanced_probe_input.phase_current_b;
    feedback.phase_current_c = g_foc_advanced_probe_input.phase_current_c;
    feedback.dc_bus_voltage = g_foc_advanced_probe_nominal_bus_voltage_v;
    feedback.electrical_angle_rad =
        g_foc_advanced_probe_input.electrical_angle_rad;
    foc_realtime_input_from_legacy(&feedback,
                                   g_foc_control_sequence,
                                   1.0f /
                                       (float)FOC_CONTROL_FREQUENCY_HZ,
                                   &input);
    control_status = foc_rust_realtime_step_advanced_no_power(
        g_foc_controller,
        &input,
        &g_foc_advanced_probe_input,
        &output,
        0,
        &g_foc_advanced_shared_status.telemetry);
    ++g_foc_control_sequence;

    /* FNV-1a over discrete decisions gives a deterministic per-tick signature
     * without requiring bit-identical target/PC floating-point duties. */
    g_foc_advanced_probe.decision_signature ^=
        g_foc_advanced_shared_status.telemetry.active_features;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_advanced_shared_status.telemetry.region;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_advanced_shared_status.telemetry.modulation_mode;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_advanced_shared_status.telemetry.status_flags;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    ++g_foc_advanced_probe.decision_samples;

    if ((control_status == FOC_STATUS_OK) &&
        (!(output.duty_a >= g_foc_platform_config.minimum_duty &&
          output.duty_a <= g_foc_platform_config.maximum_duty) ||
        !(output.duty_b >= g_foc_platform_config.minimum_duty &&
          output.duty_b <= g_foc_platform_config.maximum_duty) ||
        !(output.duty_c >= g_foc_platform_config.minimum_duty &&
          output.duty_c <= g_foc_platform_config.maximum_duty)))
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
            g_foc_power_safety.fault_epoch);
        control_status = FOC_STATUS_INVALID_ARGUMENT;
        goto fail_closed;
    }

    probe_result = foc_platform_advanced_probe_commit_isr(&output,
                                                          control_status);
    if ((probe_result == FOC_ADVANCED_PROBE_RESULT_OK) ||
        (probe_result == FOC_ADVANCED_PROBE_RESULT_COMPLETE))
    {
        ++g_foc_diagnostics.realtime_step_count;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        return FOC_STATUS_OK;
    }
    control_status = FOC_STATUS_HARDWARE_FAULT;

fail_closed:
    foc_platform_advanced_probe_snapshot(&snapshot);
    if (g_foc_advanced_probe.state == FOC_ADVANCED_PROBE_RUNNING)
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            (snapshot.fault_epoch !=
             g_foc_advanced_probe.expected_fault_epoch) ?
                FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED :
                FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
            snapshot.fault_epoch);
    }
    ++g_foc_diagnostics.realtime_error_count;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
    g_foc_diagnostics.last_control_status = control_status;
    foc_platform_latch_fault_fast(FOC_POWER_FAULT_CONTROL,
                                  FOC_RUST_FAULT_ADVANCED_CONTROL);
    return control_status;
}
#endif

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
static foc_status_t foc_platform_sensorless_probe_step_isr(
    int32_t current_u_counts,
    int32_t current_v_counts,
    int32_t current_w_counts)
{
    foc_advanced_probe_register_snapshot_t before;
    foc_advanced_probe_register_snapshot_t after;
    foc_sensorless_realtime_input_t input = {0};
    foc_sensorless_voltage_input_t voltage_input = {0};
    foc_sensorless_status_t sensorless_status;
    foc_advanced_probe_result_t probe_result;
    foc_status_t control_status = FOC_STATUS_OK;
    float current_u_a;
    float current_v_a;

    foc_platform_advanced_probe_snapshot(&before);
    probe_result = foc_advanced_probe_begin_tick(&g_foc_advanced_probe,
                                                 &before);
    if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }

    input.struct_size = sizeof(input);
    input.version = FOC_SENSORLESS_INPUT_VERSION;
    input.sample_sequence = g_foc_control_sequence;
    input.applied_request_sequence = g_foc_control_sequence;
    /* The public provider remains zero. This synthetic all-bits mask exists
     * only inside the compile-time motor-arm-disabled commissioning image so
     * the ABI can exercise its ledger; it is never published as board proof. */
    input.platform_capabilities = FOC_SENSORLESS_REQUIRED_CAPABILITIES;
    input.applied_injection_alpha_v =
        g_foc_sensorless_probe_voltage_output.applied_injection_alpha_v;
    input.applied_injection_beta_v =
        g_foc_sensorless_probe_voltage_output.applied_injection_beta_v;
    if ((g_foc_sensorless_probe_voltage_output.status_flags &
         FOC_SENSORLESS_VOLTAGE_OUTPUT_LIMITED) != 0U)
    {
        input.input_flags |=
            FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED;
    }
    current_u_a = (float)current_u_counts / FOC_CURRENT_COUNTS_PER_AMP;
    current_v_a = (float)current_v_counts / FOC_CURRENT_COUNTS_PER_AMP;
    (void)current_w_counts;
    input.measured_current_alpha_a = current_u_a;
    input.measured_current_beta_a =
        (current_u_a + (2.0f * current_v_a)) * 0.57735026919f;
    if (g_foc_sensorless_probe_mode == 1U)
    {
        input.input_flags |= FOC_SENSORLESS_INPUT_BEMF_VALID;
        input.bemf_angle_rad = 0.70f;
        input.bemf_electrical_speed_rad_s = 200.0f;
    }
    else
    {
        input.input_flags |= FOC_SENSORLESS_INPUT_INJECTION_PERMITTED;
    }
    sensorless_status = foc_rust_sensorless_step(
        &g_foc_sensorless_probe_context,
        &input,
        &g_foc_sensorless_probe_output);
    ++g_foc_control_sequence;
    if (sensorless_status != FOC_SENSORLESS_STATUS_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
    }

    if (control_status == FOC_STATUS_OK)
    {
        voltage_input.struct_size = sizeof(voltage_input);
        voltage_input.version = FOC_SENSORLESS_VOLTAGE_INPUT_VERSION;
        voltage_input.request_apply_sequence =
            g_foc_sensorless_probe_output.request_apply_sequence;
        voltage_input.pwm_sequence =
            g_foc_sensorless_probe_output.request_apply_sequence;
        /* S12 commissioning deliberately uses a zero fundamental vector.  The
         * same Rust transaction accepts the basic current-loop alpha/beta
         * voltage once the combined controller ABI is added; until then this
         * target path proves real synchronized current -> HFI request -> final
         * circle -> N+1 inactive preload without pretending to close the motor
         * loop or back-calculate the basic current PI. */
        voltage_input.base_voltage_alpha_v = 0.0f;
        voltage_input.base_voltage_beta_v = 0.0f;
        voltage_input.injection_alpha_v =
            g_foc_sensorless_probe_output.injection_alpha_v;
        voltage_input.injection_beta_v =
            g_foc_sensorless_probe_output.injection_beta_v;
        voltage_input.dc_bus_voltage_v =
            FOC_SENSORLESS_COMMISSIONING_BUS_VOLTAGE_V;
        voltage_input.voltage_limit_v =
            FOC_SENSORLESS_COMMISSIONING_VOLTAGE_LIMIT_V;
        voltage_input.minimum_duty = g_foc_platform_config.minimum_duty;
        voltage_input.maximum_duty = g_foc_platform_config.maximum_duty;
        sensorless_status = foc_rust_sensorless_compose_voltage(
            &voltage_input,
            &g_foc_sensorless_probe_voltage_output);
        if (sensorless_status != FOC_SENSORLESS_STATUS_OK)
        {
            control_status = FOC_STATUS_HARDWARE_FAULT;
        }
    }

    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_probe_output.status_flags;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_probe_output.stage;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_probe_output.angle_source;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    ++g_foc_advanced_probe.decision_samples;

    foc_platform_advanced_probe_snapshot(&before);
    probe_result = foc_advanced_probe_complete_control(
        &g_foc_advanced_probe, control_status, &before);
    if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }
    /* Gate/MOE/CCER were rechecked immediately above.  These writes therefore
     * update inactive preload registers only; they neither grant arm authority
     * nor prove powered N+1 actuation.  The next snapshot must remain safe-off. */
    if (!(g_foc_sensorless_probe_voltage_output.duty_a >=
          g_foc_platform_config.minimum_duty &&
          g_foc_sensorless_probe_voltage_output.duty_a <=
          g_foc_platform_config.maximum_duty) ||
        !(g_foc_sensorless_probe_voltage_output.duty_b >=
          g_foc_platform_config.minimum_duty &&
          g_foc_sensorless_probe_voltage_output.duty_b <=
          g_foc_platform_config.maximum_duty) ||
        !(g_foc_sensorless_probe_voltage_output.duty_c >=
          g_foc_platform_config.minimum_duty &&
          g_foc_sensorless_probe_voltage_output.duty_c <=
          g_foc_platform_config.maximum_duty))
    {
        control_status = FOC_STATUS_INVALID_ARGUMENT;
        goto fail_closed;
    }
    TIM1->CCR1 = (uint32_t)(g_foc_sensorless_probe_voltage_output.duty_a *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR2 = (uint32_t)(g_foc_sensorless_probe_voltage_output.duty_b *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR3 = (uint32_t)(g_foc_sensorless_probe_voltage_output.duty_c *
                            (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    __DSB();
    foc_platform_advanced_probe_snapshot(&after);
    probe_result = foc_advanced_probe_complete_commit(
        &g_foc_advanced_probe, &before, &after);
    if ((probe_result == FOC_ADVANCED_PROBE_RESULT_OK) ||
        (probe_result == FOC_ADVANCED_PROBE_RESULT_COMPLETE))
    {
        ++g_foc_diagnostics.realtime_step_count;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        return FOC_STATUS_OK;
    }
    control_status = FOC_STATUS_HARDWARE_FAULT;

fail_closed:
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    foc_platform_advanced_probe_snapshot(&after);
    if (g_foc_advanced_probe.state == FOC_ADVANCED_PROBE_RUNNING)
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            (after.fault_epoch !=
             g_foc_advanced_probe.expected_fault_epoch) ?
                FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED :
                FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
            after.fault_epoch);
    }
    ++g_foc_diagnostics.realtime_error_count;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
    g_foc_diagnostics.last_control_status = control_status;
    return control_status;
}

/*
 * Mode 3: the formal combined transaction, measured in the real ADC ISR with
 * every output enable held off.
 *
 * Modes 1 and 2 drive the standalone sensorless ABI plus the standalone
 * compose step, so they cannot bound the cost of the single shared Clarke /
 * fused-angle / one-final-limit transaction that the armed path uses.  This
 * step calls the same foc_rust_realtime_step_sensorless() entry the armed route
 * calls, and supplies the previous tick's applied injection so the N/N+1 ledger
 * closes for real instead of being asserted by the caller.
 *
 * Contract of this route:
 *   - Gate, MOE and all three phase channels stay off for the whole run; the
 *     platform start gate rejects arm while it exists and the probe register
 *     snapshot is re-verified before and after the commit;
 *   - only TIM1 inactive preload registers change, exactly as in mode 1/2;
 *   - the bus voltage, control sequence and dt come from the synthetic
 *     commissioning envelope, never from a claim that 12.3 V is present.
 */
/* The platform hand-builds this struct, so freeze its layout against the Rust
 * contract.  foc_sensorless_bridge.h already pins the total size; these pin the
 * offsets the hand-written initialiser depends on. */
_Static_assert(sizeof(foc_sensorless_composite_input_t) == 44U,
               "composite input ABI size drifted");
_Static_assert(sizeof(foc_sensorless_composite_output_t) == 24U,
               "composite output ABI size drifted");
_Static_assert(offsetof(foc_sensorless_composite_input_t,
                        applied_request_sequence) == 16U,
               "composite input ledger offset drifted");
_Static_assert(offsetof(foc_sensorless_composite_input_t,
                        applied_injection_alpha_v) == 24U,
               "composite input applied-injection offset drifted");
_Static_assert(offsetof(foc_sensorless_composite_output_t,
                        applied_injection_alpha_v) == 16U,
               "composite output applied-injection offset drifted");
_Static_assert(FOC_SENSORLESS_PROBE_MODE_COMPOSITE == 3UL,
               "composite probe mode id changed");
/* The diagnostic below uses rt_kprintf, which the shared include block only
 * pulls in for the motion candidate.  Request it here so the sensorless
 * commissioning image can report its own first-tick rejection. */
#if defined(__RTTHREAD__)
#include "rtthread.h"
#endif
static foc_status_t foc_platform_sensorless_composite_probe_step_isr(
    int32_t current_u_counts,
    int32_t current_v_counts,
    int32_t current_w_counts)
{
    foc_advanced_probe_register_snapshot_t before;
    foc_advanced_probe_register_snapshot_t after;
    foc_feedback_t feedback;
    foc_realtime_input_t input;
    foc_output_t output;
    foc_sensorless_composite_input_t composite_input = {0};
    foc_advanced_probe_result_t probe_result;
    foc_status_t control_status = FOC_STATUS_OK;

    foc_platform_advanced_probe_snapshot(&before);
    probe_result = foc_advanced_probe_begin_tick(&g_foc_advanced_probe, &before);
    if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }

    /* The formal entry owns the current sample: it Clarkes once and feeds both
     * the full-speed angle chain and the basic current loop from that single
     * snapshot.  Raw ADC codes stay on this side of the ABI. */
    feedback.phase_current_a = (float)current_u_counts / FOC_CURRENT_COUNTS_PER_AMP;
    feedback.phase_current_b = (float)current_v_counts / FOC_CURRENT_COUNTS_PER_AMP;
    feedback.phase_current_c = (float)current_w_counts / FOC_CURRENT_COUNTS_PER_AMP;
    feedback.dc_bus_voltage =
        ((float)g_foc_diagnostics.bus_voltage_raw * FOC_ADC_REFERENCE_VOLTAGE) /
        (FOC_ADC_FULL_SCALE * FOC_BUS_PARTITIONING_FACTOR);
    feedback.electrical_angle_rad = 0.0f;
    foc_realtime_input_from_legacy(
        &feedback,
        g_foc_control_sequence,
        1.0f / (float)FOC_CONTROL_FREQUENCY_HZ,
        &input);

    composite_input.struct_size = sizeof(composite_input);
    composite_input.version = FOC_SENSORLESS_COMPOSITE_INPUT_VERSION;
    /* The published provider stays zero.  This synthetic all-bits mask exists
     * only inside the compile-time motor-arm-disabled commissioning image so
     * the combined ABI can exercise its ledger; it is never board proof. */
    composite_input.platform_capabilities = FOC_SENSORLESS_REQUIRED_CAPABILITIES;
    composite_input.input_flags =
        FOC_SENSORLESS_INPUT_INJECTION_PERMITTED |
        FOC_SENSORLESS_INPUT_BEMF_VALID;
    if (g_foc_sensorless_composite_applied_limited != 0U)
    {
        composite_input.input_flags |=
            FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED;
    }
    /* The prior composite tick published this value; the combined entry reads
     * it as its N/N+1 ledger. */
    composite_input.applied_request_sequence =
        g_foc_sensorless_composite_applied_sequence;
    composite_input.applied_injection_alpha_v =
        g_foc_sensorless_composite_applied_alpha_v;
    composite_input.applied_injection_beta_v =
        g_foc_sensorless_composite_applied_beta_v;
    composite_input.voltage_limit_v = FOC_SENSORLESS_COMMISSIONING_VOLTAGE_LIMIT_V;
    composite_input.minimum_duty = g_foc_platform_config.minimum_duty;
    composite_input.maximum_duty = g_foc_platform_config.maximum_duty;

    control_status = foc_rust_realtime_step_sensorless(
        g_foc_controller,
        &g_foc_sensorless_probe_context,
        &input,
        &composite_input,
        &output,
        0,
        &g_foc_sensorless_probe_output,
        &g_foc_sensorless_composite_probe_output);

    /* Mode 3 only: surface the first combined-entry rejection.  A silent
     * CONTROL_FAILURE gives no way to tell a rejected envelope from a rejected
     * ledger, and this image exists precisely to make that measurable. */
    if ((g_foc_advanced_probe.decision_samples == 0U) &&
        (control_status != FOC_STATUS_OK))
    {
        rt_kprintf("FSLSC,cst=%u,cs=%u,arq=%u,dmin=%u,dmax=%u,vlim=%u\n",
                   (unsigned int)control_status,
                   (unsigned int)input.control_sequence,
                   (unsigned int)composite_input.applied_request_sequence,
                   (unsigned int)(composite_input.minimum_duty * 1000.0f),
                   (unsigned int)(composite_input.maximum_duty * 1000.0f),
                   (unsigned int)composite_input.voltage_limit_v);
    }

    /* Carry the ledger forward only from a Coherent tick.  A rejected tick
     * keeps the previous applied values so the next attempt fails closed on the
     * same mismatch instead of silently resynchronising. */
    if (control_status == FOC_STATUS_OK)
    {
        g_foc_sensorless_composite_applied_sequence =
            g_foc_sensorless_composite_probe_output.pwm_sequence;
        g_foc_sensorless_composite_applied_alpha_v =
            g_foc_sensorless_composite_probe_output.applied_injection_alpha_v;
        g_foc_sensorless_composite_applied_beta_v =
            g_foc_sensorless_composite_probe_output.applied_injection_beta_v;
        g_foc_sensorless_composite_applied_limited =
            ((g_foc_sensorless_composite_probe_output.status_flags &
              FOC_SENSORLESS_COMPOSITE_OUTPUT_INJECTION_LIMITED) != 0U) ? 1U : 0U;
    }
    else
    {
        goto fail_closed;
    }

    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_composite_probe_output.status_flags;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_probe_output.stage;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    g_foc_advanced_probe.decision_signature ^=
        g_foc_sensorless_probe_output.angle_source;
    g_foc_advanced_probe.decision_signature *= 16777619UL;
    ++g_foc_advanced_probe.decision_samples;

    foc_platform_advanced_probe_snapshot(&before);
    probe_result = foc_advanced_probe_complete_control(
        &g_foc_advanced_probe, control_status, &before);
    if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
    {
        control_status = FOC_STATUS_HARDWARE_FAULT;
        goto fail_closed;
    }
    /* Gate/MOE/CCER were rechecked immediately above, so these writes reach
     * inactive preload registers only; they grant no arm authority and prove no
     * powered N+1 actuation.  The next snapshot must still read safe-off. */
    if (!((output.duty_a >= g_foc_platform_config.minimum_duty) &&
          (output.duty_a <= g_foc_platform_config.maximum_duty)) ||
        !((output.duty_b >= g_foc_platform_config.minimum_duty) &&
          (output.duty_b <= g_foc_platform_config.maximum_duty)) ||
        !((output.duty_c >= g_foc_platform_config.minimum_duty) &&
          (output.duty_c <= g_foc_platform_config.maximum_duty)))
    {
        control_status = FOC_STATUS_INVALID_ARGUMENT;
        goto fail_closed;
    }
    TIM1->CCR1 = (uint32_t)(output.duty_a * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR2 = (uint32_t)(output.duty_b * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    TIM1->CCR3 = (uint32_t)(output.duty_c * (float)FOC_PWM_PERIOD_TICKS + 0.5f);
    __DSB();
    foc_platform_advanced_probe_snapshot(&after);
    probe_result = foc_advanced_probe_complete_commit(
        &g_foc_advanced_probe, &before, &after);
    if ((probe_result == FOC_ADVANCED_PROBE_RESULT_OK) ||
        (probe_result == FOC_ADVANCED_PROBE_RESULT_COMPLETE))
    {
        ++g_foc_diagnostics.realtime_step_count;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        return FOC_STATUS_OK;
    }
    control_status = FOC_STATUS_HARDWARE_FAULT;

fail_closed:
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    foc_platform_advanced_probe_snapshot(&after);
    if (g_foc_advanced_probe.state == FOC_ADVANCED_PROBE_RUNNING)
    {
        (void)foc_advanced_probe_fail(
            &g_foc_advanced_probe,
            (after.fault_epoch != g_foc_advanced_probe.expected_fault_epoch) ?
                FOC_ADVANCED_PROBE_RESULT_FAULT_EPOCH_CHANGED :
                FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
            after.fault_epoch);
    }
    ++g_foc_diagnostics.realtime_error_count;
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
    g_foc_diagnostics.last_control_status = control_status;
    return control_status;
}
#endif

#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
/*
 * The only physical TIM1 binding for the EXP-B3 preload sink.  S4.6 uses a
 * one-shot validation write; S4.8 also exercises active mode's first write.
 * Both startup paths keep CCER/MOE/gates off.  The internal sink itself has no
 * power-enable operation; S4.9's ADC ISR may call it only with a private permit,
 * while Shell can only request the separately gated platform session.
 */
static foc_lsi_preload_result_t foc_platform_lsi_apply_preload(
    const foc_lsi_preload_permit_t *permit,
    const foc_lsi_executor_output_t *output)
{
    foc_lsi_preload_registers_t registers = {
        &TIM1->CCR1,
        &TIM1->CCR2,
        &TIM1->CCR3,
        &TIM1->ARR,
    };
    foc_lsi_preload_safety_t safety = {0};
    foc_lsi_preload_result_t result;
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    const uint32_t ccmr1_preload_mask = TIM_CCMR1_OC1PE |
                                        TIM_CCMR1_OC2PE;

    safety.timer_clock_enabled =
        ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U) ? 1U : 0U;
    safety.timer_configured =
        ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_TIM1_CONFIGURED) != 0U) ?
            1U : 0U;
    safety.compare_preload_enabled =
        (((TIM1->CCMR1 & ccmr1_preload_mask) == ccmr1_preload_mask) &&
         ((TIM1->CCMR2 & TIM_CCMR2_OC3PE) != 0U)) ? 1U : 0U;
    safety.gate_enabled = (foc_platform_gate_is_low() == 0U) ? 1U : 0U;
    safety.main_output_enabled =
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    safety.channel_outputs_enabled =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    safety.hardware_fault =
        ((foc_platform_driver_faulted() != 0U) ||
         ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_BREAK_LATCHED) != 0U)) ?
            1U : 0U;
    safety.software_trip =
        ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_CURRENT_TRIP) != 0U) ?
            1U : 0U;

    result = foc_lsi_preload_sink_apply(&g_foc_lsi_preload_session,
                                        permit,
                                        output,
                                        &registers,
                                        &safety);
    __DSB();
    if (result != FOC_LSI_PRELOAD_RESULT_OK)
    {
        foc_platform_disable_power_fast();
    }
    return result;
}

static void foc_platform_lsi_abort_session_isr(void)
{
    foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
    foc_lsi_management_shared_abort_isr();
    g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
    g_foc_lsi_session_running = 0U;
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
    foc_platform_disable_power_fast();
}

static void foc_platform_lsi_complete_session_isr(void)
{
    /* Keep the synchronous monitor alive, but close both power and capture. */
    foc_platform_disable_output_registers_fast();
    foc_lsi_capture_service_stop();
    foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
    g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
    g_foc_lsi_session_running = 0U;
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
}

static uint32_t foc_platform_lsi_enable_first_output_isr(void)
{
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t master_mode = TIM1->CR2;
    uint32_t primask;
    uint32_t armed = 0U;

    /* The first bounded preload was written with every output off. Transfer it
     * before enabling the physical path so stale startup CCRs cannot escape.
     * The baseline ADC trigger is OC4REF. Mask MMS defensively while generating
     * UG so this software-only transfer cannot leak any master-trigger event
     * into the injected ADC path if the trigger plan changes later. */
    TIM1->CR2 = master_mode & ~TIM_CR2_MMS;
    __DSB();
    TIM1->EGR = TIM_EGR_UG;
    __DSB();
    TIM1->CR2 = master_mode;
    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_foc_power_safety.state == FOC_POWER_SAFETY_ARMING) &&
        (g_foc_power_safety.fault_epoch == g_foc_lsi_arm_token.fault_epoch) &&
        ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) == 0U) &&
        (foc_platform_driver_faulted() == 0U))
    {
        TIM1->DIER |= TIM_DIER_BIE;
        TIM1->CCER |= channel_mask;
        TIM1->BDTR |= TIM_BDTR_MOE;
        FOC_GATE_ENABLE_PORT->BSRR = FOC_GATE_ENABLE_PINS;
        __DSB();
        if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) == 0U) &&
            ((TIM1->CCER & channel_mask) == channel_mask) &&
            ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) &&
            (foc_platform_gate_is_low() == 0U) &&
            (foc_platform_driver_faulted() == 0U) &&
            (foc_power_safety_commit_arm(
                 &g_foc_power_safety, &g_foc_lsi_arm_token) != 0U))
        {
            g_foc_control_armed = 1U;
            armed = 1U;
        }
    }
    if (armed == 0U)
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_PLATFORM,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (armed == 0U)
    {
        foc_platform_lsi_abort_session_isr();
        return 0U;
    }
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_OUTPUT_ACTIVE;
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_GATE_SAFE;
    return 1U;
}

static void foc_platform_lsi_process_isr(
    const foc_lsi_raw_sample_t *raw,
    uint16_t peak_current_counts,
    foc_lsi_capture_record_result_t capture_result)
{
    foc_lsi_input_t input = {0};
    foc_lsi_output_t request = {0};
    foc_lsi_executor_command_t command = {0};
    foc_lsi_executor_output_t execution = {0};
    foc_lsi_preload_permit_t permit = {0};
    foc_lsi_state_t state;
    foc_status_t executor_status;

    if ((raw == 0) || (g_foc_lsi_session_running == 0U))
    {
        return;
    }
    if ((capture_result == FOC_LSI_CAPTURE_RECORD_CONTRACT_ERROR) ||
        (capture_result == FOC_LSI_CAPTURE_RECORD_STORAGE_ERROR) ||
        (capture_result == FOC_LSI_CAPTURE_RECORD_COMPLETE))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR;
        foc_platform_lsi_abort_session_isr();
        return;
    }

    input.struct_size = sizeof(input);
    input.version = FOC_LSI_INPUT_VERSION;
    input.identification_build_authorized = 1U;
    input.power_stage_idle = ((g_foc_control_armed == 0U) &&
                              (foc_platform_gate_is_low() != 0U) &&
                              ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
                              ((TIM1->CCER & (TIM_CCER_CC1E |
                                             TIM_CCER_CC2E |
                                             TIM_CCER_CC3E)) == 0U)) ? 1U : 0U;
    /* Sensorless Ls(I) has no independent rotor-motion sensor. The exact LSI1
     * operator confirmation plus outputs-off/current-zero preflight is the
     * explicit stationary-rotor assumption recorded for this experiment. */
    input.motor_stopped = 1U;
    input.hardware_fault = ((raw->flags &
        FOC_LSI_RAW_FLAG_HARDWARE_FAULT) != 0U) ? 1U : 0U;
    input.software_trip = ((raw->flags &
        FOC_LSI_RAW_FLAG_SOFTWARE_TRIP) != 0U) ? 1U : 0U;
    input.offset_sample_valid = 1U;
    input.bus_voltage_v = (float)raw->bus_voltage_raw *
        g_foc_lsi_executor.platform.bus_volts_per_count;
    input.abs_phase_current_a = (float)peak_current_counts /
        FOC_CURRENT_COUNTS_PER_AMP;
    state = foc_lsi_management_shared_step(&input, &request);
    if ((state == FOC_LSI_STATE_ABORTED) ||
        (request.abort_reason != FOC_LSI_ABORT_NONE))
    {
        foc_platform_lsi_abort_session_isr();
        return;
    }

    if ((request.capture_raw_sample != 0U) &&
        (foc_lsi_capture_service_is_armed() == 0U) &&
        (foc_lsi_capture_service_arm() == 0U))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR;
        foc_platform_lsi_abort_session_isr();
        return;
    }

    command.struct_size = sizeof(command);
    command.version = FOC_LSI_EXECUTOR_COMMAND_VERSION;
    command.drive_request =
        (foc_lsi_drive_request_abi_t)request.drive_request;
    command.force_safe_output = request.force_safe_output;
    command.capture_ready = foc_lsi_capture_service_is_armed();
    command.hardware_fault = input.hardware_fault;
    command.software_trip = input.software_trip;
    command.requested_bias_current_a = request.requested_bias_current_a;
    command.requested_perturbation_voltage_v =
        request.requested_perturbation_voltage_v;
    executor_status = foc_lsi_executor_step(&g_foc_lsi_executor,
                                            &command,
                                            raw,
                                            &execution);
    if ((executor_status != FOC_STATUS_OK) ||
        ((request.drive_request != FOC_LSI_DRIVE_OFF) &&
         (execution.action != FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD)) ||
        ((request.drive_request == FOC_LSI_DRIVE_OFF) &&
         (execution.action != FOC_LSI_EXECUTOR_ACTION_SAFE)))
    {
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
        foc_platform_lsi_abort_session_isr();
        return;
    }

    if (execution.action == FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD)
    {
        uint32_t first_write =
            (g_foc_lsi_preload_session.active_write_count == 0U) ? 1U : 0U;

        if ((foc_lsi_preload_session_issue(&g_foc_lsi_preload_session,
                                           &execution,
                                           &permit) !=
             FOC_LSI_PRELOAD_RESULT_OK) ||
            (foc_platform_lsi_apply_preload(&permit, &execution) !=
             FOC_LSI_PRELOAD_RESULT_OK))
        {
            g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_OUTPUT_REJECTED;
            foc_platform_lsi_abort_session_isr();
            return;
        }
        if ((first_write != 0U) &&
            (foc_platform_lsi_enable_first_output_isr() == 0U))
        {
            return;
        }
        g_foc_lsi_active_drive_request = request.drive_request;
    }
    else
    {
        g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
        if (g_foc_control_armed != 0U)
        {
            /* Expected active->cooldown transition: power off immediately but
             * keep collecting the zero-current tail for 24 control ticks. */
            foc_platform_disable_output_registers_fast();
        }
    }

    if (state == FOC_LSI_STATE_COMPLETE)
    {
        foc_platform_lsi_complete_session_isr();
    }
}
#endif

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
    uint32_t sync_running =
        ((g_foc_diagnostics.flags & FOC_PLATFORM_DIAG_SYNC_RUNNING) != 0U) ?
            1U : 0U;
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
    uint32_t adc1_regular_owned =
        foc_external_input_platform_regular_adc_owned();
#else
    uint32_t adc1_regular_owned = 0U;
#endif
    foc_monitor_sampling_plan_t plan = foc_monitor_sampling_plan(
        sync_running, adc1_regular_owned);

    if ((plan & FOC_MONITOR_SAMPLE_PHASE_CURRENTS) != 0U)
    {
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_2,
                                    &g_foc_diagnostics.phase_u_raw);
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_14,
                                    &g_foc_diagnostics.phase_v_raw);
        ok &= foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_4,
                                    &g_foc_diagnostics.phase_w_raw);
    }
    if ((plan & FOC_MONITOR_SAMPLE_VBUS_ADC1) != 0U)
    {
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_1,
                                    &g_foc_diagnostics.bus_voltage_raw);
    }
    if ((plan & FOC_MONITOR_SAMPLE_VBUS_ADC2) != 0U)
    {
        /* PA0 is ADC12_IN1. When PC2 owns ADC1 regular, use the same physical
         * Vbus divider through ADC2 instead of stealing ADC1 SQR1/ADSTART. */
        ok &= foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_1,
                                    &g_foc_diagnostics.bus_voltage_raw);
        if (ok != 0U)
        {
            g_foc_diagnostics.flags |=
                FOC_PLATFORM_DIAG_ADC2_VBUS_FALLBACK_USED;
        }
    }
    if ((plan & FOC_MONITOR_SAMPLE_TEMPERATURE_ADC2) != 0U)
    {
        ok &= foc_platform_read_adc(&g_foc_adc2, ADC_CHANNEL_5,
                                    &g_foc_diagnostics.temperature_raw);
    }
    if ((plan & FOC_MONITOR_SAMPLE_POTENTIOMETER_ADC1) != 0U)
    {
        ok &= foc_platform_read_adc(&g_foc_adc1, ADC_CHANNEL_8,
                                    &g_foc_diagnostics.potentiometer_raw);
    }
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
    /* EXP-B3 只读监测门：rank1 仍为 U 相电流，rank2 增加同序列 Vbus。
     * 这里不连接辨识状态机、不写 CCR，也不授予 start 能力。 */
    injected.InjectedNbrOfConversion = 2U;
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
#else
    /* 与 ST MCSDK/IHM16M1 的 Vbus regular-channel 配置保持同一 47.5-cycle
     * 采样时间；本工程把它放进注入 rank2 以获得同序列证据。 */
    injected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    injected.InjectedChannel = ADC_CHANNEL_1; /* PA0 = VBUS */
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
#endif
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    /* ADC1 already uses all four injected ranks for IU + U/V/W phase voltage.
     * ADC2 rank2 carries synchronous Vbus; ADC1's longer four-rank sequence
     * guarantees ADC2 rank2 is complete before the ADC1 JEOS interrupt. */
    injected.InjectedNbrOfConversion = 2U;
#else
    injected.InjectedNbrOfConversion = 1U;
#endif
    injected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    injected.InjectedChannel = ADC_CHANNEL_14;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc2, &injected) != HAL_OK)
    {
        return 0U;
    }
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    injected.InjectedSamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    injected.InjectedChannel = ADC_CHANNEL_1; /* PA0 = VBUS */
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc2, &injected) != HAL_OK)
    {
        return 0U;
    }
#endif
    return 1U;
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
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    g_foc_diagnostics.lsi_sync_bus_voltage_raw = 0U;
    g_foc_diagnostics.lsi_sync_bus_valid = 0U;
    g_foc_diagnostics.lsi_sync_bus_sample_count = 0U;
#endif
#if defined(FOC_SYNC_EDGE_CONTROL_TICK)
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

/* Restart only the stopped trigger/ADC machinery after an explicit fault-clear
 * preflight. It never enables CH1..CH3, MOE, BIE or the gate pins. */
static uint32_t foc_platform_restart_sync_monitor_safe(void)
{
    __HAL_ADC_CLEAR_FLAG(&g_foc_adc1, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    __HAL_ADC_CLEAR_FLAG(&g_foc_adc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    if (((ADC2->CR & ADC_CR_JADSTART) == 0U) &&
        (HAL_ADCEx_InjectedStart(&g_foc_adc2) != HAL_OK))
    {
        return 0U;
    }
    if (((ADC1->CR & ADC_CR_JADSTART) == 0U) &&
        (HAL_ADCEx_InjectedStart_IT(&g_foc_adc1) != HAL_OK))
    {
        return 0U;
    }
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
    TIM1->DIER &= ~TIM_DIER_BIE;
    TIM1->CCR4 = FOC_PWM_PERIOD_TICKS - 1U;
    TIM1->CCER |= TIM_CCER_CC4E;
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
    TIM2->SR = 0U;
    TIM2->CNT = TIM2->ARR;
    TIM2->CR1 |= TIM_CR1_CEN;
#endif
    TIM1->CNT = 0U;
    TIM1->CR1 |= TIM_CR1_CEN;
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID;
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
#elif defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_LSI_SYNC_BUS_CONFIGURED;
#endif
    return 1U;
}
#endif

#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
/* Map the application-owned H2 sequence into the broader Rust planner ABI.
 * This operation may only tighten limits.  Keeping the generic Rust maximum
 * separate allows later experiments to define their own reviewed envelope,
 * while this Identification image remains locked to the H2 first pulse. */
static uint32_t foc_platform_apply_lsi_h2_envelope(
    foc_lsi_actuation_config_t *actuation,
    foc_lsi_config_t *sequence)
{
    if ((actuation == 0) || (sequence == 0) ||
        (foc_lsi_management_shared_get_config(sequence) == 0U) ||
        (foc_lsi_management_apply_h2_actuation_envelope(
             sequence, actuation) == 0U))
    {
        return 0U;
    }
    return 1U;
}

/*
 * 只验证 Rust 计划器和 C ABI 的静态契约，不写 CCR、不打开 CCER/MOE/栅极。
 * 真实 PWM 适配器在后续独立门完成前仍不存在。
 */
static uint32_t foc_platform_validate_lsi_actuation_plan(void)
{
    foc_lsi_actuation_config_t config = {0};
    foc_lsi_config_t sequence = {0};
    foc_lsi_actuation_input_t input = {0};
    foc_lsi_actuation_output_t output = {0};

    if ((foc_rust_lsi_default_actuation_config(&config) != FOC_STATUS_OK) ||
        (foc_platform_apply_lsi_h2_envelope(&config, &sequence) == 0U))
    {
        return 0U;
    }
    if ((config.sample_rate_hz != FOC_CONTROL_FREQUENCY_HZ) ||
        ((config.actuation_delay_control_ticks *
          FOC_PWM_TICKS_PER_CONTROL) != FOC_ACTUATION_DELAY_PWM_TICKS) ||
        (config.current_trip_a > g_foc_platform_config.software_current_trip_a) ||
        (config.minimum_bus_voltage_v <
         g_foc_platform_config.minimum_bus_voltage_v) ||
        (config.maximum_bus_voltage_v >
         g_foc_platform_config.maximum_bus_voltage_v) ||
        (config.minimum_duty < g_foc_platform_config.minimum_duty) ||
        (config.maximum_duty > g_foc_platform_config.maximum_duty))
    {
        return 0U;
    }
    if ((config.maximum_bias_current_a != sequence.bias_current_a) ||
        (config.maximum_perturbation_voltage_v !=
         sequence.perturbation_voltage_v) ||
        (config.current_trip_a != sequence.current_trip_a))
    {
        return 0U;
    }

    input.struct_size = sizeof(input);
    input.version = FOC_LSI_ACTUATION_INPUT_VERSION;
    input.drive_request = FOC_LSI_DRIVE_ABI_OFF;
    input.force_safe_output = 1U;
    if ((foc_rust_lsi_plan(&config, &input, &output) != FOC_STATUS_OK) ||
        (output.safe_output_required == 0U) ||
        (output.drive_active != 0U) ||
        (output.duty_u != 0.0f) ||
        (output.duty_v != 0.0f) ||
        (output.duty_w != 0.0f))
    {
        return 0U;
    }

    input.drive_request = FOC_LSI_DRIVE_ABI_BIAS;
    input.force_safe_output = 0U;
    input.capture_ready = 1U;
    input.control_tick = 0U;
    input.requested_bias_current_a = config.maximum_bias_current_a;
    input.bus_voltage_v = 12.3f;
    if ((foc_rust_lsi_plan(&config, &input, &output) != FOC_STATUS_OK) ||
        (output.safe_output_required != 0U) ||
        (output.drive_active == 0U) ||
        (output.source_control_tick != 0U) ||
        (output.expected_active_control_tick != 1U) ||
        (output.duty_u < g_foc_platform_config.minimum_duty) ||
        (output.duty_u > g_foc_platform_config.maximum_duty) ||
        (output.duty_v < g_foc_platform_config.minimum_duty) ||
        (output.duty_v > g_foc_platform_config.maximum_duty) ||
        (output.duty_w < g_foc_platform_config.minimum_duty) ||
        (output.duty_w > g_foc_platform_config.maximum_duty))
    {
        return 0U;
    }

    g_foc_diagnostics.flags |=
        FOC_PLATFORM_DIAG_LSI_ACTUATION_PLAN_CONFIGURED;
    return 1U;
}
#endif

#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
/*
 * Compose the S4.5 C executor from real board calibration facts, then run the
 * S4.6 validation one-shot and the S4.8 bounded-active first-write contract.
 * Both startup probes physically write/read/clear CCR1/2/3 while CCER, MOE and
 * all gates remain disabled.  This remains a startup self-test after the
 * separately gated S4.9 ISR/start caller was added.
 */
static uint32_t foc_platform_validate_lsi_execution_adapter(void)
{
    foc_lsi_actuation_config_t actuation = {0};
    foc_lsi_config_t sequence = {0};
    foc_lsi_executor_config_t platform = {0};
    foc_lsi_executor_command_t command = {0};
    foc_lsi_executor_output_t output = {0};
    foc_lsi_preload_permit_t permit = {0};
    foc_lsi_raw_sample_t raw = {0};
    uint16_t minimum_compare;
    uint16_t maximum_compare;

    if ((foc_rust_lsi_default_actuation_config(&actuation) != FOC_STATUS_OK) ||
        (foc_platform_apply_lsi_h2_envelope(&actuation, &sequence) == 0U))
    {
        return 0U;
    }
    platform.struct_size = sizeof(platform);
    platform.version = FOC_LSI_EXECUTOR_CONFIG_VERSION;
    platform.sample_rate_hz = FOC_CONTROL_FREQUENCY_HZ;
    platform.pwm_period_ticks = FOC_PWM_PERIOD_TICKS;
    platform.adc_max_code = (uint32_t)FOC_ADC_FULL_SCALE;
    platform.actuation_delay_control_ticks = 1U;
    platform.current_u_offset_raw = g_foc_diagnostics.phase_u_offset;
    platform.current_v_offset_raw = g_foc_diagnostics.phase_v_offset;
    platform.current_counts_per_amp = FOC_CURRENT_COUNTS_PER_AMP;
    platform.bus_volts_per_count =
        FOC_ADC_REFERENCE_VOLTAGE /
        (FOC_ADC_FULL_SCALE * FOC_BUS_PARTITIONING_FACTOR);
    platform.minimum_duty = g_foc_platform_config.minimum_duty;
    platform.maximum_duty = g_foc_platform_config.maximum_duty;
    if (foc_lsi_executor_init(&g_foc_lsi_executor,
                              &platform,
                              &actuation) != FOC_STATUS_OK)
    {
        return 0U;
    }
    minimum_compare = (uint16_t)((platform.minimum_duty *
                                  (float)platform.pwm_period_ticks) + 0.5f);
    maximum_compare = (uint16_t)((platform.maximum_duty *
                                  (float)platform.pwm_period_ticks) + 0.5f);
    foc_lsi_preload_session_init(&g_foc_lsi_preload_session,
                                 platform.pwm_period_ticks,
                                 platform.actuation_delay_control_ticks,
                                 minimum_compare,
                                 maximum_compare);
    if (foc_lsi_preload_session_open_validation(
            &g_foc_lsi_preload_session,
            FOC_LSI_PRELOAD_CONFIRMATION) != FOC_LSI_PRELOAD_RESULT_OK)
    {
        return 0U;
    }

    raw.control_tick = 0U;
    raw.current_u_raw = platform.current_u_offset_raw;
    raw.current_v_raw = platform.current_v_offset_raw;
    raw.bus_voltage_raw = foc_platform_bus_voltage_to_raw(12.3f);
    raw.pwm_period_ticks = (uint16_t)FOC_PWM_PERIOD_TICKS;
    raw.flags = FOC_LSI_RAW_FLAG_ADC_VALID;
    command.struct_size = sizeof(command);
    command.version = FOC_LSI_EXECUTOR_COMMAND_VERSION;
    command.drive_request = FOC_LSI_DRIVE_ABI_BIAS;
    command.capture_ready = 1U;
    command.requested_bias_current_a = actuation.maximum_bias_current_a;
    if ((foc_lsi_executor_step(&g_foc_lsi_executor,
                               &command,
                               &raw,
                               &output) != FOC_STATUS_OK) ||
        (output.action != FOC_LSI_EXECUTOR_ACTION_WRITE_PRELOAD) ||
        (output.expected_active_control_tick != 1U))
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_platform_disable_power_fast();
        return 0U;
    }
    if ((foc_lsi_preload_session_issue(&g_foc_lsi_preload_session,
                                       &output,
                                       &permit) !=
         FOC_LSI_PRELOAD_RESULT_OK) ||
        (foc_platform_lsi_apply_preload(&permit, &output) !=
         FOC_LSI_PRELOAD_RESULT_OK) ||
        (TIM1->CCR1 != output.compare_u) ||
        (TIM1->CCR2 != output.compare_v) ||
        (TIM1->CCR3 != output.compare_w))
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_platform_disable_power_fast();
        return 0U;
    }

    raw.control_tick = 1U;
    raw.compare_u = (uint16_t)TIM1->CCR1;
    raw.compare_v = (uint16_t)TIM1->CCR2;
    raw.compare_w = (uint16_t)TIM1->CCR3;
    /* Synthetic active flag is only for the executor ledger. Hardware stayed off. */
    raw.flags = FOC_LSI_RAW_FLAG_ADC_VALID |
                FOC_LSI_RAW_FLAG_DRIVE_ACTIVE;
    command.drive_request = FOC_LSI_DRIVE_ABI_OFF;
    command.force_safe_output = 1U;
    command.capture_ready = 0U;
    command.requested_bias_current_a = 0.0f;
    if ((foc_lsi_executor_step(&g_foc_lsi_executor,
                               &command,
                               &raw,
                               &output) != FOC_STATUS_OK) ||
        (output.action != FOC_LSI_EXECUTOR_ACTION_SAFE) ||
        (output.result != FOC_LSI_EXECUTOR_RESULT_SAFE_REQUESTED) ||
        (output.ledger_checked == 0U) ||
        (output.ledger_matched == 0U))
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_platform_disable_power_fast();
        return 0U;
    }

    /* The validation capability is one-shot; leave no compare or open permit. */
    foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
    foc_platform_disable_power_fast();
    if ((TIM1->CCR1 != 0U) || (TIM1->CCR2 != 0U) || (TIM1->CCR3 != 0U) ||
        (g_foc_lsi_preload_session.validation_open != 0U))
    {
        return 0U;
    }

    /* S4.8: exercise only the first, outputs-off write of a bounded active
     * session.  The second-write path requires physical outputs on and is Host
     * tested only until the separate ISR/start gate exists. */
    if (foc_lsi_executor_init(&g_foc_lsi_executor,
                              &platform,
                              &actuation) != FOC_STATUS_OK)
    {
        return 0U;
    }
    foc_lsi_preload_session_init(&g_foc_lsi_preload_session,
                                 platform.pwm_period_ticks,
                                 platform.actuation_delay_control_ticks,
                                 minimum_compare,
                                 maximum_compare);
    if (foc_lsi_preload_session_open_active(
            &g_foc_lsi_preload_session,
            FOC_LSI_ACTIVE_CONFIRMATION,
            2U,
            4U,
            2U) != FOC_LSI_PRELOAD_RESULT_OK)
    {
        return 0U;
    }
    memset(&command, 0, sizeof(command));
    memset(&output, 0, sizeof(output));
    memset(&permit, 0, sizeof(permit));
    memset(&raw, 0, sizeof(raw));
    raw.control_tick = 2U;
    raw.current_u_raw = platform.current_u_offset_raw;
    raw.current_v_raw = platform.current_v_offset_raw;
    raw.bus_voltage_raw = foc_platform_bus_voltage_to_raw(12.3f);
    raw.pwm_period_ticks = (uint16_t)FOC_PWM_PERIOD_TICKS;
    raw.flags = FOC_LSI_RAW_FLAG_ADC_VALID;
    command.struct_size = sizeof(command);
    command.version = FOC_LSI_EXECUTOR_COMMAND_VERSION;
    command.drive_request = FOC_LSI_DRIVE_ABI_BIAS;
    command.capture_ready = 1U;
    command.requested_bias_current_a = actuation.maximum_bias_current_a;
    if ((foc_lsi_executor_step(&g_foc_lsi_executor,
                               &command,
                               &raw,
                               &output) != FOC_STATUS_OK) ||
        (foc_lsi_preload_session_issue(&g_foc_lsi_preload_session,
                                       &output,
                                       &permit) !=
         FOC_LSI_PRELOAD_RESULT_OK) ||
        (permit.session_mode != FOC_LSI_PRELOAD_MODE_ACTIVE) ||
        (permit.permit_sequence != 1U) ||
        (foc_platform_lsi_apply_preload(&permit, &output) !=
         FOC_LSI_PRELOAD_RESULT_OK))
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_platform_disable_power_fast();
        return 0U;
    }
    raw.control_tick = 3U;
    raw.compare_u = (uint16_t)TIM1->CCR1;
    raw.compare_v = (uint16_t)TIM1->CCR2;
    raw.compare_w = (uint16_t)TIM1->CCR3;
    raw.flags = FOC_LSI_RAW_FLAG_ADC_VALID |
                FOC_LSI_RAW_FLAG_DRIVE_ACTIVE;
    command.drive_request = FOC_LSI_DRIVE_ABI_OFF;
    command.force_safe_output = 1U;
    command.capture_ready = 0U;
    command.requested_bias_current_a = 0.0f;
    if ((foc_lsi_executor_step(&g_foc_lsi_executor,
                               &command,
                               &raw,
                               &output) != FOC_STATUS_OK) ||
        (output.action != FOC_LSI_EXECUTOR_ACTION_SAFE) ||
        (output.ledger_checked == 0U) ||
        (output.ledger_matched == 0U) ||
        (g_foc_lsi_preload_session.active_write_count != 1U))
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_platform_disable_power_fast();
        return 0U;
    }
    foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
    foc_platform_disable_power_fast();
    if ((TIM1->CCR1 != 0U) || (TIM1->CCR2 != 0U) ||
        (TIM1->CCR3 != 0U) ||
        (g_foc_lsi_preload_session.validation_open != 0U) ||
        (g_foc_lsi_preload_session.active_open != 0U))
    {
        return 0U;
    }

    /* Leave both persistent objects clean; self-test history is not a session. */
    if (foc_lsi_executor_init(&g_foc_lsi_executor,
                              &platform,
                              &actuation) != FOC_STATUS_OK)
    {
        return 0U;
    }
    foc_lsi_preload_session_init(&g_foc_lsi_preload_session,
                                 platform.pwm_period_ticks,
                                 platform.actuation_delay_control_ticks,
                                 minimum_compare,
                                 maximum_compare);
    g_foc_diagnostics.flags |=
        FOC_PLATFORM_DIAG_LSI_EXECUTOR_CONFIGURED |
        FOC_PLATFORM_DIAG_LSI_PRELOAD_SINK_CONFIGURED |
        FOC_PLATFORM_DIAG_LSI_ACTIVE_SESSION_CONFIGURED;
    return 1U;
}
#endif

foc_status_t foc_platform_init(void)
{
#if defined(FOC_TARGET_STM32G431)
    if (g_foc_power_safety_initialized == 0U)
    {
        foc_power_safety_init(&g_foc_power_safety);
        g_foc_pending_rust_faults = 0U;
        g_foc_power_safety_initialized = 1U;
    }
    else if (g_foc_power_safety.state == FOC_POWER_SAFETY_FAULT_LATCHED)
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_HARDWARE_FAULT;
    }
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    (void)foc_motion_probe_init(&g_foc_motion_probe);
    (void)foc_motion_torque_trial_init(&g_foc_motion_torque_trial);
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
    (void)foc_advanced_probe_init(&g_foc_advanced_probe);
    g_foc_advanced_probe_active = 0U;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    (void)foc_advanced_power_trial_init(&g_foc_advanced_power_trial);
    g_foc_advanced_probe_nominal_bus_voltage_v = 0.0f;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
    g_foc_sensorless_probe_mode = 0U;
    (void)foc_rust_sensorless_init(&g_foc_sensorless_probe_context);
    (void)memset(&g_foc_sensorless_probe_output, 0,
                 sizeof(g_foc_sensorless_probe_output));
    (void)memset(&g_foc_sensorless_composite_output, 0,
                 sizeof(g_foc_sensorless_composite_output));
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    (void)foc_as5600_alignment_trial_init(&g_foc_as5600_alignment_trial);
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    (void)memset(&g_foc_advanced_probe_input, 0,
                 sizeof(g_foc_advanced_probe_input));
    (void)memset(&g_foc_advanced_shared_status, 0,
                 sizeof(g_foc_advanced_shared_status));
#endif
    g_foc_bus_min_raw = foc_platform_bus_voltage_to_raw(
        g_foc_platform_config.minimum_bus_voltage_v);
    g_foc_bus_max_raw = foc_platform_bus_voltage_to_raw(
        g_foc_platform_config.maximum_bus_voltage_v);
#endif
    foc_platform_emergency_stop();
    foc_realtime_timing_reset(&g_foc_timing_stats);
    (void)foc_math_accel_init();
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_capture_service_init(FOC_CONTROL_FREQUENCY_HZ,
                                 (uint32_t)FOC_ADC_FULL_SCALE,
                                 FOC_PWM_PERIOD_TICKS);
    if (foc_lsi_management_shared_init() == 0U)
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }
    g_foc_lsi_session_running = 0U;
    g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
#endif
#if defined(FOC_TARGET_STM32G431) && defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
    foc_phase_voltage_capture_init(&g_foc_phase_voltage_capture,
                                   FOC_CONTROL_FREQUENCY_HZ);
#endif
#if defined(FOC_TARGET_STM32G431)
    /* 先验证纯数据时序契约，再触碰 TIM1/ADC。宏组合错误时保持功率级关闭。 */
    if ((foc_pwm_timing_plan_is_valid(&g_foc_pwm_timing_plan) == 0U)
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
        || (foc_platform_validate_lsi_actuation_plan() == 0U)
#endif
        || (foc_platform_init_timer_disabled() == 0U) ||
        (foc_platform_init_adc_monitor() == 0U)
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
        || (foc_platform_validate_lsi_execution_adapter() == 0U)
#endif
       )
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
        !(config->minimum_bus_voltage_v >= FOC_PLATFORM_HARD_MIN_BUS_VOLTAGE_V) ||
        !(config->maximum_bus_voltage_v > config->minimum_bus_voltage_v) ||
        !(config->maximum_bus_voltage_v <= FOC_PLATFORM_HARD_MAX_BUS_VOLTAGE_V) ||
        !(config->software_current_trip_a > 0.0f) ||
        !(config->software_current_trip_a <= FOC_PLATFORM_HARD_MAX_CURRENT_TRIP_A) ||
        !(config->minimum_duty >= FOC_PLATFORM_HARD_MIN_DUTY) ||
        !(config->maximum_duty <= FOC_PLATFORM_HARD_MAX_DUTY) ||
        !(config->maximum_duty > config->minimum_duty) ||
        (config->isr_deadline_cycles == 0U) ||
        (config->isr_deadline_cycles >
         FOC_PLATFORM_HARD_MAX_ISR_DEADLINE_CYCLES))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
#if defined(FOC_TARGET_STM32G431)
    if ((g_foc_control_armed != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }
#endif
    g_foc_platform_config = *config;
#if defined(FOC_TARGET_STM32G431)
    g_foc_bus_min_raw = foc_platform_bus_voltage_to_raw(
        config->minimum_bus_voltage_v);
    g_foc_bus_max_raw = foc_platform_bus_voltage_to_raw(
        config->maximum_bus_voltage_v);
#endif
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

foc_status_t foc_platform_motion_candidate_configure(
    const foc_config_bundle_t *config,
    const foc_runtime_config_t *active_realtime_config,
    uint32_t active_bundle_revision,
    uint32_t outer_loop_divider)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    const foc_motion_dispatcher_ops_t dispatcher_ops = {
        sizeof(foc_motion_dispatcher_ops_t),
        FOC_MOTION_DISPATCHER_VERSION,
        foc_platform_motion_enter_critical,
        foc_platform_motion_exit_critical,
        0,
    };
    foc_motion_status_t motion_status;
    foc_motion_dispatcher_result_t dispatcher_result;
    uint32_t expected_fault_epoch;
    uint32_t key;

    if ((g_foc_motion_probe.struct_size != sizeof(g_foc_motion_probe)) ||
        (g_foc_motion_probe.version != FOC_MOTION_PROBE_VERSION))
    {
        (void)foc_motion_probe_init(&g_foc_motion_probe);
    }
    if ((g_foc_motion_torque_trial.struct_size !=
         sizeof(g_foc_motion_torque_trial)) ||
        (g_foc_motion_torque_trial.version !=
         FOC_MOTION_TORQUE_TRIAL_VERSION))
    {
        (void)foc_motion_torque_trial_init(&g_foc_motion_torque_trial);
    }
    if ((outer_loop_divider == 0U) ||
        (foc_platform_motion_config_matches_base(
             config,
             active_realtime_config,
             active_bundle_revision) == 0U))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    if ((g_foc_control_armed != 0U) ||
        (g_foc_motion_runtime_enabled != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }
    expected_fault_epoch = g_foc_power_safety.fault_epoch;
    if ((foc_rust_motion_abi_version() != FOC_MOTION_ABI_VERSION) ||
        (foc_rust_motion_context_required_size() >
         sizeof(g_foc_motion_context)) ||
        (foc_rust_motion_context_required_align() >
         _Alignof(foc_motion_context_t)))
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }

    motion_status = foc_rust_motion_init(&g_foc_motion_context);
    if (motion_status == FOC_MOTION_STATUS_OK)
    {
        motion_status = foc_rust_motion_configure_realtime(
            &g_foc_motion_context, config, outer_loop_divider);
    }
    if (motion_status == FOC_MOTION_STATUS_OK)
    {
        motion_status = foc_rust_motion_enable(&g_foc_motion_context);
    }
    if (motion_status != FOC_MOTION_STATUS_OK)
    {
        (void)foc_rust_motion_disable(&g_foc_motion_context);
        return foc_platform_motion_status_to_foc(motion_status);
    }

    dispatcher_result = foc_motion_dispatcher_init(
        &g_foc_motion_dispatcher, &dispatcher_ops);
    if (dispatcher_result != FOC_MOTION_DISPATCHER_OK)
    {
        (void)foc_rust_motion_disable(&g_foc_motion_context);
        return foc_platform_motion_dispatch_result_to_foc(dispatcher_result);
    }
    (void)memset(&g_foc_motion_realtime_request,
                 0,
                 sizeof(g_foc_motion_realtime_request));
    (void)memset(&g_foc_motion_realtime_output,
                 0,
                 sizeof(g_foc_motion_realtime_output));
    g_foc_motion_command_ready = 0U;
    g_foc_motion_probe_nominal_bus_voltage_v =
        active_realtime_config->nominal_bus_voltage_v;

    (void)foc_platform_motion_probe_inject_isr(
        FOC_MOTION_PROBE_INJECT_CONFIGURE_BEFORE_ROUTE);

    /* Publish ownership last.  Once this word becomes one, only the ADC ISR
     * may mutate the Rust motion context until stopped-state shutdown removes
     * the route again. */
    key = foc_platform_motion_enter_critical(0);
    if ((g_foc_control_armed == 0U) &&
        (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) &&
        (g_foc_power_safety.fault_epoch == expected_fault_epoch))
    {
        g_foc_motion_runtime_enabled = 1U;
    }
    foc_platform_motion_exit_critical(0, key);
    (void)foc_platform_motion_probe_inject_isr(
        FOC_MOTION_PROBE_INJECT_CONFIGURE_AFTER_ROUTE);
    /* Break remains preemptible inside the ADC-only critical section.  It may
     * therefore arrive after the pre-commit check but before the route store;
     * this post-commit epoch/state check closes that exact interleaving. */
    if ((g_foc_motion_runtime_enabled == 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED) ||
        (g_foc_power_safety.fault_epoch != expected_fault_epoch))
    {
        key = foc_platform_motion_enter_critical(0);
        g_foc_motion_runtime_enabled = 0U;
        g_foc_motion_command_ready = 0U;
        foc_platform_motion_exit_critical(0, key);
        (void)foc_rust_motion_disable(&g_foc_motion_context);
        (void)foc_motion_dispatcher_reset_stopped(&g_foc_motion_dispatcher);
        return FOC_STATUS_HARDWARE_FAULT;
    }
    if (foc_motion_probe_configure(&g_foc_motion_probe,
                                   expected_fault_epoch) !=
        FOC_MOTION_PROBE_RESULT_OK)
    {
        key = foc_platform_motion_enter_critical(0);
        g_foc_motion_runtime_enabled = 0U;
        g_foc_motion_command_ready = 0U;
        foc_platform_motion_exit_critical(0, key);
        (void)foc_rust_motion_disable(&g_foc_motion_context);
        (void)foc_motion_dispatcher_reset_stopped(&g_foc_motion_dispatcher);
        return FOC_STATUS_NOT_CONFIGURED;
    }
    return FOC_STATUS_OK;
#else
    (void)config;
    (void)active_realtime_config;
    (void)active_bundle_revision;
    (void)outer_loop_divider;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_publish(
    const foc_product_command_t *command,
    uint32_t position_valid,
    float mechanical_position_rad,
    uint32_t position_sampled_at_ms)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    if (g_foc_motion_runtime_enabled == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    {
        foc_motion_dispatcher_result_t result =
            foc_motion_dispatcher_publish(&g_foc_motion_dispatcher,
                                          command,
                                          position_valid,
                                          mechanical_position_rad,
                                          position_sampled_at_ms);
        if (result == FOC_MOTION_DISPATCHER_OK)
        {
            g_foc_motion_command_ready = 1U;
        }
        return foc_platform_motion_dispatch_result_to_foc(result);
    }
#else
    (void)command;
    (void)position_valid;
    (void)mechanical_position_rad;
    (void)position_sampled_at_ms;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_request_stop(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    if (g_foc_motion_runtime_enabled == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    g_foc_motion_command_ready = 0U;
    return foc_platform_motion_dispatch_result_to_foc(
        foc_motion_dispatcher_request_stop(&g_foc_motion_dispatcher));
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_request_fault(uint32_t fault_detail)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    if (g_foc_motion_runtime_enabled == 0U)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    g_foc_motion_command_ready = 0U;
    return foc_platform_motion_dispatch_result_to_foc(
        foc_motion_dispatcher_request_fault(&g_foc_motion_dispatcher,
                                            fault_detail));
#else
    (void)fault_detail;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_shutdown(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_status_t motion_status;
    foc_motion_dispatcher_result_t dispatcher_result;
    uint32_t key;

    if (g_foc_control_armed != 0U)
    {
        return FOC_STATUS_DISABLED;
    }

    /* Revoke ISR ownership before touching either state machine.  Because the
     * caller runs in task context on a single Cortex-M core, no ADC handler is
     * active at this point; masking the IRQ prevents the next one from starting
     * during the handoff. */
    key = foc_platform_motion_enter_critical(0);
    g_foc_motion_runtime_enabled = 0U;
    g_foc_motion_command_ready = 0U;
    g_foc_motion_probe_execute_motion = 0U;
    g_foc_motion_probe_nominal_bus_voltage_v = 0.0f;
    foc_platform_motion_exit_critical(0, key);

    motion_status = foc_rust_motion_disable(&g_foc_motion_context);
    dispatcher_result = foc_motion_dispatcher_reset_stopped(
        &g_foc_motion_dispatcher);
    (void)foc_motion_probe_init(&g_foc_motion_probe);
    (void)foc_motion_torque_trial_init(&g_foc_motion_torque_trial);
    if (dispatcher_result != FOC_MOTION_DISPATCHER_OK)
    {
        return foc_platform_motion_dispatch_result_to_foc(dispatcher_result);
    }
    return foc_platform_motion_status_to_foc(motion_status);
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

uint32_t foc_platform_motion_candidate_is_enabled(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    return (g_foc_motion_runtime_enabled != 0U) ? 1U : 0U;
#else
    return 0U;
#endif
}

foc_status_t foc_platform_motion_candidate_probe_start(
    uint32_t requested_ticks,
    float target_speed_rpm,
    uint32_t execute_motion)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_probe_register_snapshot_t snapshot;
    foc_motion_probe_result_t probe_result;
    foc_status_t status;
    uint32_t key;

    if ((execute_motion > 1U) ||
        (g_foc_controller == 0) ||
        (g_foc_motion_runtime_enabled == 0U) ||
        (g_foc_motion_command_ready == 0U))
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    if ((g_foc_control_armed != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }

    key = foc_platform_motion_enter_critical(0);
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    foc_platform_motion_probe_snapshot(&snapshot);
    status = foc_rust_start_realtime(g_foc_controller,
                                     1U,
                                     target_speed_rpm);
    if (status == FOC_STATUS_OK)
    {
        probe_result = foc_motion_probe_start(&g_foc_motion_probe,
                                              requested_ticks,
                                              &snapshot);
        if (probe_result != FOC_MOTION_PROBE_RESULT_OK)
        {
            status = FOC_STATUS_NOT_CONFIGURED;
        }
    }
    if (status == FOC_STATUS_OK)
    {
        g_foc_motion_probe_execute_motion = execute_motion;
        g_foc_control_sequence = 0U;
        g_foc_diagnostics.realtime_step_count = 0U;
        g_foc_diagnostics.realtime_error_count = 0U;
        g_foc_diagnostics.deadline_miss_count = 0U;
        g_foc_diagnostics.maximum_isr_cycles = 0U;
        g_foc_diagnostics.maximum_precontrol_cycles = 0U;
        g_foc_diagnostics.maximum_control_cycles = 0U;
        g_foc_diagnostics.maximum_postcontrol_cycles = 0U;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        foc_realtime_timing_reset(&g_foc_timing_stats);
    }
    else
    {
        foc_rust_stop(g_foc_controller);
    }
    foc_platform_motion_exit_critical(0, key);
    return status;
#else
    (void)requested_ticks;
    (void)target_speed_rpm;
    (void)execute_motion;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_probe_get_status(
    foc_motion_probe_status_t *status)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_probe_result_t result;
    uint32_t key;

    if (status == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_motion_enter_critical(0);
    result = foc_motion_probe_get_status(&g_foc_motion_probe, status);
    foc_platform_motion_exit_critical(0, key);
    return (result == FOC_MOTION_PROBE_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)status;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_probe_finish(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    uint32_t key = foc_platform_motion_enter_critical(0);

    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    g_foc_motion_probe_execute_motion = 0U;
    foc_rust_stop(g_foc_controller);
    foc_platform_motion_exit_critical(0, key);
    return FOC_STATUS_OK;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_probe_inject_once(
    foc_motion_probe_injection_point_t point)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_probe_result_t result;
    uint32_t key;

    if ((g_foc_motion_probe.struct_size != sizeof(g_foc_motion_probe)) ||
        (g_foc_motion_probe.version != FOC_MOTION_PROBE_VERSION))
    {
        (void)foc_motion_probe_init(&g_foc_motion_probe);
    }
    key = foc_platform_motion_enter_critical(0);
    result = foc_motion_probe_set_injection(&g_foc_motion_probe, point);
    foc_platform_motion_exit_critical(0, key);
    return (result == FOC_MOTION_PROBE_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_DISABLED;
#else
    (void)point;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_torque_trial_start(
    const foc_product_command_t *command,
    const foc_runtime_config_t *active_realtime_config,
    float startup_target_speed_rpm)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_torque_trial_result_t trial_result;
    foc_status_t status;

    if ((g_foc_controller == 0) ||
        (g_foc_motion_runtime_enabled == 0U) ||
        (g_foc_motion_command_ready != 0U) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }
    trial_result = foc_motion_torque_trial_prepare(
        &g_foc_motion_torque_trial, command, active_realtime_config);
    if (trial_result != FOC_MOTION_TORQUE_TRIAL_RESULT_OK)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    status = foc_platform_motion_candidate_publish(command, 0U, 0.0f, 0U);
    if (status != FOC_STATUS_OK)
    {
        foc_motion_torque_trial_fail(
            &g_foc_motion_torque_trial,
            FOC_MOTION_TORQUE_TRIAL_RESULT_INVALID_ENVELOPE,
            g_foc_power_safety.fault_epoch);
        return status;
    }
    trial_result = foc_motion_torque_trial_mark_armed(
        &g_foc_motion_torque_trial);
    if (trial_result != FOC_MOTION_TORQUE_TRIAL_RESULT_OK)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    status = foc_platform_control_start_internal(
        startup_target_speed_rpm, FOC_CONTROL_START_AUTHORITY_MOTION);
    if (status != FOC_STATUS_OK)
    {
        foc_motion_torque_trial_fail(
            &g_foc_motion_torque_trial,
            FOC_MOTION_TORQUE_TRIAL_RESULT_PLATFORM_FAULT,
            g_foc_power_safety.fault_epoch);
    }
    return status;
#else
    (void)command;
    (void)active_realtime_config;
    (void)startup_target_speed_rpm;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_motion_candidate_torque_trial_get_status(
    foc_motion_torque_trial_status_t *status)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_torque_trial_result_t result;
    uint32_t key;

    if (status == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_motion_enter_critical(0);
    result = foc_motion_torque_trial_get_status(
        &g_foc_motion_torque_trial, status);
    foc_platform_motion_exit_critical(0, key);
    return (result == FOC_MOTION_TORQUE_TRIAL_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)status;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_probe_start(
    const foc_advanced_runtime_config_t *config,
    const foc_advanced_probe_input_t *probe_input,
    uint32_t requested_ticks,
    float nominal_bus_voltage_v,
    float target_speed_rpm)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_probe_register_snapshot_t snapshot;
    foc_advanced_probe_result_t probe_result;
    foc_status_t status;
    uint32_t expected_fault_epoch;
    uint32_t key;

    if ((config == 0) || (probe_input == 0) ||
        !(nominal_bus_voltage_v > 0.0f) ||
        (g_foc_controller == 0))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    if ((g_foc_control_armed != 0U) ||
        (g_foc_advanced_probe_active != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }

    key = foc_platform_advanced_enter_critical();
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    expected_fault_epoch = g_foc_power_safety.fault_epoch;
    foc_platform_advanced_probe_snapshot(&snapshot);
    (void)foc_advanced_probe_init(&g_foc_advanced_probe);
    status = foc_rust_configure_advanced(g_foc_controller, config);
    if (status == FOC_STATUS_OK)
    {
        status = foc_rust_start_realtime(g_foc_controller,
                                         1U,
                                         target_speed_rpm);
    }
    if (status == FOC_STATUS_OK)
    {
        probe_result = foc_advanced_probe_start(&g_foc_advanced_probe,
                                                requested_ticks,
                                                expected_fault_epoch,
                                                &snapshot);
        if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
        {
            status = FOC_STATUS_NOT_CONFIGURED;
        }
    }
    if ((status == FOC_STATUS_OK) &&
        (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) &&
        (g_foc_power_safety.fault_epoch == expected_fault_epoch))
    {
        g_foc_advanced_probe_input = *probe_input;
        g_foc_advanced_probe_nominal_bus_voltage_v = nominal_bus_voltage_v;
        g_foc_advanced_probe_active = 1U;
        g_foc_control_sequence = 0U;
        g_foc_diagnostics.realtime_step_count = 0U;
        g_foc_diagnostics.realtime_error_count = 0U;
        g_foc_diagnostics.deadline_miss_count = 0U;
        g_foc_diagnostics.maximum_isr_cycles = 0U;
        g_foc_diagnostics.maximum_precontrol_cycles = 0U;
        g_foc_diagnostics.maximum_control_cycles = 0U;
        g_foc_diagnostics.maximum_postcontrol_cycles = 0U;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        foc_realtime_timing_reset(&g_foc_timing_stats);
    }
    else
    {
        g_foc_advanced_probe_active = 0U;
        foc_rust_stop(g_foc_controller);
        if (status == FOC_STATUS_OK)
        {
            status = FOC_STATUS_HARDWARE_FAULT;
        }
    }
    foc_platform_advanced_exit_critical(key);
    return status;
#else
    (void)config;
    (void)probe_input;
    (void)requested_ticks;
    (void)nominal_bus_voltage_v;
    (void)target_speed_rpm;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_probe_get_status(
    foc_advanced_probe_status_t *status,
    foc_advanced_telemetry_t *telemetry)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_probe_result_t result;
    uint32_t key;

    if ((status == 0) || (telemetry == 0))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_advanced_enter_critical();
    result = foc_advanced_probe_get_status(&g_foc_advanced_probe, status);
    *telemetry = g_foc_advanced_shared_status.telemetry;
    foc_platform_advanced_exit_critical(key);
    return (result == FOC_ADVANCED_PROBE_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)status;
    (void)telemetry;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_probe_finish(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_runtime_config_t disabled_config;
    foc_status_t status;
    uint32_t key = foc_platform_advanced_enter_critical();

    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    g_foc_advanced_probe_active = 0U;
    foc_rust_stop(g_foc_controller);
    status = foc_rust_default_advanced_config(g_foc_controller,
                                              &disabled_config);
    if (status == FOC_STATUS_OK)
    {
        status = foc_rust_configure_advanced(g_foc_controller,
                                             &disabled_config);
    }
    foc_platform_advanced_exit_critical(key);
    return status;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_sensorless_candidate_probe_start(
    uint32_t mode,
    uint32_t requested_ticks)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
    foc_advanced_probe_register_snapshot_t snapshot;
    foc_sensorless_runtime_config_t config;
    foc_sensorless_configure_guard_t guard;
    foc_sensorless_status_t sensorless_status;
    foc_advanced_probe_result_t probe_result;
    foc_status_t controller_status;
    uint32_t expected_fault_epoch;
    uint32_t key;

    if ((mode < 1U) || (mode > FOC_SENSORLESS_PROBE_MODE_COMPOSITE) ||
        (requested_ticks < FOC_ADVANCED_PROBE_MIN_TICKS) ||
        (requested_ticks > FOC_ADVANCED_PROBE_MAX_TICKS) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_advanced_probe_active != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_advanced_enter_critical();
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    expected_fault_epoch = g_foc_power_safety.fault_epoch;
    foc_platform_advanced_probe_snapshot(&snapshot);
    if (foc_advanced_probe_registers_are_safe_off(&snapshot) == 0U)
    {
        foc_platform_advanced_exit_critical(key);
        return FOC_STATUS_HARDWARE_FAULT;
    }
    sensorless_status = foc_rust_sensorless_init(
        &g_foc_sensorless_probe_context);
    if (sensorless_status == FOC_SENSORLESS_STATUS_OK)
    {
        sensorless_status = foc_rust_sensorless_default_config(&config);
    }
    if (sensorless_status == FOC_SENSORLESS_STATUS_OK)
    {
        config.enabled = 1U;
        /* Keep the HFI timing probe in axis acquisition for the whole maximum
         * 10 s window. This executes the hot negative-sequence path without
         * pretending that synthetic current proves polarity or a real motor. */
        config.hfi.axis_stable_samples = FOC_ADVANCED_PROBE_MAX_TICKS + 1U;
        (void)memset(&guard, 0, sizeof(guard));
        guard.struct_size = sizeof(guard);
        guard.version = FOC_SENSORLESS_GUARD_VERSION;
        guard.controller_stopped = 1U;
        guard.outputs_disabled = 1U;
        guard.no_faults = 1U;
        guard.platform_capabilities = FOC_SENSORLESS_REQUIRED_CAPABILITIES;
        sensorless_status = foc_rust_sensorless_configure(
            &g_foc_sensorless_probe_context, &config, &guard);
    }
    /* Mode 3 only: the combined entry runs the shared controller core, which
     * accepts only the five startup-chain states.  Entering Alignment through
     * the normal ABI is what makes the composite cost measurable without
     * arming; modes 1 and 2 drive the standalone sensorless ABI and never
     * needed a runnable controller, so they are deliberately left untouched. */
    if ((sensorless_status == FOC_SENSORLESS_STATUS_OK) &&
        (mode == FOC_SENSORLESS_PROBE_MODE_COMPOSITE))
    {
        controller_status = foc_rust_start_realtime(
            g_foc_controller,
            1U,
            FOC_SENSORLESS_COMMISSIONING_TARGET_SPEED_RPM);
        if (controller_status != FOC_STATUS_OK)
        {
            sensorless_status = FOC_SENSORLESS_STATUS_UNSAFE_CONFIGURATION_STATE;
        }
    }
    if (sensorless_status == FOC_SENSORLESS_STATUS_OK)
    {
        (void)foc_advanced_probe_init(&g_foc_advanced_probe);
        probe_result = foc_advanced_probe_start(
            &g_foc_advanced_probe,
            requested_ticks,
            expected_fault_epoch,
            &snapshot);
        if (probe_result != FOC_ADVANCED_PROBE_RESULT_OK)
        {
            sensorless_status = FOC_SENSORLESS_STATUS_UNSAFE_CONFIGURATION_STATE;
        }
    }
    if ((sensorless_status == FOC_SENSORLESS_STATUS_OK) &&
        (g_foc_power_safety.state == FOC_POWER_SAFETY_DISABLED) &&
        (g_foc_power_safety.fault_epoch == expected_fault_epoch))
    {
        (void)memset(&g_foc_sensorless_probe_output, 0,
                     sizeof(g_foc_sensorless_probe_output));
        (void)memset(&g_foc_sensorless_probe_voltage_output, 0,
                     sizeof(g_foc_sensorless_probe_voltage_output));
        (void)memset(&g_foc_sensorless_composite_output, 0,
                     sizeof(g_foc_sensorless_composite_output));
        /* Mode 3 restarts the N/N+1 ledger from a known zero-applied state; a
         * stale value from an earlier probe would be rejected by the combined
         * entry as a sequence mismatch. */
        (void)memset(&g_foc_sensorless_composite_probe_output, 0,
                     sizeof(g_foc_sensorless_composite_probe_output));
        g_foc_sensorless_composite_applied_sequence = 0U;
        g_foc_sensorless_composite_applied_alpha_v = 0.0f;
        g_foc_sensorless_composite_applied_beta_v = 0.0f;
        g_foc_sensorless_composite_applied_limited = 0U;
        g_foc_sensorless_probe_mode = mode;
        g_foc_advanced_probe_active = 1U;
        g_foc_control_sequence = 0U;
        g_foc_diagnostics.realtime_step_count = 0U;
        g_foc_diagnostics.realtime_error_count = 0U;
        g_foc_diagnostics.deadline_miss_count = 0U;
        g_foc_diagnostics.maximum_isr_cycles = 0U;
        g_foc_diagnostics.maximum_precontrol_cycles = 0U;
        g_foc_diagnostics.maximum_control_cycles = 0U;
        g_foc_diagnostics.maximum_postcontrol_cycles = 0U;
        g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
        foc_realtime_timing_reset(&g_foc_timing_stats);
    }
    else
    {
        g_foc_sensorless_probe_mode = 0U;
        g_foc_advanced_probe_active = 0U;
    }
    foc_platform_advanced_exit_critical(key);
    return (sensorless_status == FOC_SENSORLESS_STATUS_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)mode;
    (void)requested_ticks;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_sensorless_candidate_probe_get_status(
    foc_advanced_probe_status_t *status,
    foc_sensorless_realtime_output_t *output)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
    foc_advanced_probe_result_t probe_result;
    uint32_t key;

    if ((status == 0) || (output == 0))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_advanced_enter_critical();
    probe_result = foc_advanced_probe_get_status(
        &g_foc_advanced_probe, status);
    *output = g_foc_sensorless_probe_output;
    foc_platform_advanced_exit_critical(key);
    return ((g_foc_sensorless_probe_mode != 0U) &&
            (probe_result == FOC_ADVANCED_PROBE_RESULT_OK)) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)status;
    (void)output;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_sensorless_candidate_probe_finish(void)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
    uint32_t key = foc_platform_advanced_enter_critical();
    uint32_t finished_mode = g_foc_sensorless_probe_mode;

    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    g_foc_advanced_probe_active = 0U;
    g_foc_sensorless_probe_mode = 0U;
    (void)memset(&g_foc_sensorless_probe_output, 0,
                 sizeof(g_foc_sensorless_probe_output));
    (void)memset(&g_foc_sensorless_probe_voltage_output, 0,
                 sizeof(g_foc_sensorless_probe_voltage_output));
    (void)memset(&g_foc_sensorless_composite_output, 0,
                 sizeof(g_foc_sensorless_composite_output));
    (void)memset(&g_foc_sensorless_composite_probe_output, 0,
                 sizeof(g_foc_sensorless_composite_probe_output));
    g_foc_sensorless_composite_applied_sequence = 0U;
    g_foc_sensorless_composite_applied_alpha_v = 0.0f;
    g_foc_sensorless_composite_applied_beta_v = 0.0f;
    g_foc_sensorless_composite_applied_limited = 0U;
    /* Mode 3 put the controller into Alignment so the combined core would run.
     * Stop it again so a finished probe cannot leave a runnable controller
     * behind; this is what the P5.3 advanced probe also does on completion. */
    if (finished_mode == FOC_SENSORLESS_PROBE_MODE_COMPOSITE)
    {
        foc_rust_stop(g_foc_controller);
    }
    (void)foc_rust_sensorless_init(&g_foc_sensorless_probe_context);
    (void)foc_advanced_probe_init(&g_foc_advanced_probe);
    foc_platform_advanced_exit_critical(key);
    return FOC_STATUS_OK;
#else
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_break2_rearm_test(
    uint16_t *before_flags,
    uint16_t *rearm_facts,
    uint16_t *after_flags)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint16_t facts;

    if ((before_flags == 0) || (rearm_facts == 0) || (after_flags == 0))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    *before_flags = 0U;
    *rearm_facts = 0U;
    *after_flags = 0U;
    if ((g_foc_controller == 0) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_advanced_probe_active != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED) ||
        (g_foc_diagnostics.bus_voltage_raw >= g_foc_bus_min_raw))
    {
        return FOC_STATUS_DISABLED;
    }

    foc_platform_disable_power_fast();
    TIM1->DIER &= ~TIM_DIER_BIE;
    if (((TIM1->CCER & channel_mask) != 0U) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ||
        (foc_platform_gate_is_low() == 0U) ||
        (foc_platform_driver_faulted() != 0U))
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }

    TIM1->EGR = TIM_EGR_B2G;
    __DSB();
    *before_flags = (uint16_t)(TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF));
    facts = foc_platform_rearm_break2_before_arm();
    *rearm_facts = facts;
    *after_flags = (uint16_t)(TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF));
    g_foc_diagnostics.arm_reject_stage =
        FOC_ARM_REJECT_STAGE_PRE_ENABLE;
    g_foc_diagnostics.arm_reject_facts = facts;
    if (((*before_flags & (uint16_t)TIM_SR_B2IF) == 0U) ||
        (facts != (FOC_ARM_REJECT_FACT_B2IF |
                   FOC_ARM_REJECT_FACT_B2IF_REARMED)) ||
        (*after_flags != 0U))
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
        return FOC_STATUS_HARDWARE_FAULT;
    }
    return FOC_STATUS_OK;
#else
    (void)before_flags;
    (void)rearm_facts;
    (void)after_flags;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_power_trial_start(
    foc_advanced_power_trial_mode_t mode,
    const foc_runtime_config_t *active_realtime_config)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_runtime_config_t trial_runtime_config;
    foc_advanced_runtime_config_t advanced_config;
    foc_advanced_power_trial_result_t trial_result;
    foc_status_t status;
    uint32_t key;

    if ((active_realtime_config == 0) || (g_foc_controller == 0) ||
        ((mode != FOC_ADVANCED_POWER_TRIAL_MODE_BASIC) &&
         (mode != FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING)))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    if ((g_foc_control_armed != 0U) ||
        (g_foc_advanced_probe_active != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED))
    {
        return FOC_STATUS_DISABLED;
    }

    key = foc_platform_advanced_enter_critical();
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    (void)foc_advanced_power_trial_init(&g_foc_advanced_power_trial);
    (void)memset(&g_foc_advanced_shared_status, 0,
                 sizeof(g_foc_advanced_shared_status));

    /* The boot production profile deliberately keeps closed loop disabled.
     * This exact-token owner creates a private, bounded candidate instead of
     * weakening that persistent profile or requiring a preceding foc_cfg
     * mutation.  The caller-owned base configuration is restored on every
     * exit path. */
    trial_runtime_config = *active_realtime_config;
    trial_runtime_config.closed_loop_enable = 1U;
    trial_runtime_config.startup_final_speed_rpm =
        FOC_ADVANCED_POWER_TRIAL_TARGET_SPEED_RPM;

    /* Re-applying the base transaction resets every previous advanced policy
     * before this trial builds its one-bit allow-listed candidate. */
    status = foc_rust_configure(g_foc_controller, &trial_runtime_config);
    if (status == FOC_STATUS_OK)
    {
        status = foc_rust_default_advanced_config(g_foc_controller,
                                                  &advanced_config);
    }
    if (status == FOC_STATUS_OK)
    {
        advanced_config.algorithm.enabled_features =
            (mode == FOC_ADVANCED_POWER_TRIAL_MODE_DECOUPLING) ?
                FOC_ADVANCED_FEATURE_DECOUPLING : 0U;
        advanced_config.platform_capabilities = 0U;
        advanced_config.minimum_duty = g_foc_platform_config.minimum_duty;
        advanced_config.maximum_duty = g_foc_platform_config.maximum_duty;
        status = foc_rust_configure_advanced(g_foc_controller,
                                             &advanced_config);
    }
    if (status == FOC_STATUS_OK)
    {
        trial_result = foc_advanced_power_trial_prepare(
            &g_foc_advanced_power_trial,
            mode,
            &trial_runtime_config,
            &advanced_config,
            g_foc_power_safety.fault_epoch,
            0U);
        status = (trial_result == FOC_ADVANCED_POWER_TRIAL_RESULT_OK) ?
            FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
    }
    if (status == FOC_STATUS_OK)
    {
        trial_result = foc_advanced_power_trial_mark_armed(
            &g_foc_advanced_power_trial);
        status = (trial_result == FOC_ADVANCED_POWER_TRIAL_RESULT_OK) ?
            FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
    }
    foc_platform_advanced_exit_critical(key);

    if (status == FOC_STATUS_OK)
    {
        status = foc_platform_control_start_internal(
            FOC_ADVANCED_POWER_TRIAL_COMMAND_SPEED_RPM,
            FOC_CONTROL_START_AUTHORITY_ADVANCED);
    }
    if (status != FOC_STATUS_OK)
    {
        foc_advanced_runtime_config_t disabled_config;

        key = foc_platform_advanced_enter_critical();
        foc_platform_disable_power_fast();
        foc_advanced_power_trial_fail(
            &g_foc_advanced_power_trial,
            FOC_ADVANCED_POWER_TRIAL_RESULT_PLATFORM_FAULT,
            g_foc_power_safety.fault_epoch,
            g_foc_diagnostics.deadline_miss_count);
        foc_rust_stop(g_foc_controller);
        if (foc_rust_configure(g_foc_controller,
                               active_realtime_config) == FOC_STATUS_OK &&
            foc_rust_default_advanced_config(g_foc_controller,
                                             &disabled_config) == FOC_STATUS_OK)
        {
            (void)foc_rust_configure_advanced(g_foc_controller,
                                              &disabled_config);
        }
        foc_platform_advanced_exit_critical(key);
    }
    return status;
#else
    (void)mode;
    (void)active_realtime_config;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_power_trial_get_status(
    foc_advanced_power_trial_status_t *status,
    foc_advanced_telemetry_t *telemetry,
    foc_advanced_power_trial_snapshot_t *snapshot)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_power_trial_result_t result;
    uint32_t key;

    if ((status == 0) || (telemetry == 0) || (snapshot == 0))
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    key = foc_platform_advanced_enter_critical();
    result = foc_advanced_power_trial_get_status(
        &g_foc_advanced_power_trial, status);
    (void)memset(telemetry, 0, sizeof(*telemetry));
    telemetry->struct_size = sizeof(*telemetry);
    telemetry->abi_version = FOC_ADVANCED_ABI_VERSION;
    telemetry->active_features = status->last_active_features;
    telemetry->status_flags = status->last_advanced_status_flags;
    *snapshot = g_foc_advanced_shared_status.power_trial_snapshot;
    if ((snapshot->struct_size != sizeof(*snapshot)) ||
        (snapshot->version != FOC_ADVANCED_POWER_TRIAL_SNAPSHOT_VERSION))
    {
        /* A start rejected before the first realtime step has no coherent
         * controller snapshot.  Publish an explicit invalid/zero record
         * instead of interpreting whichever union member was active. */
        (void)memset(snapshot, 0, sizeof(*snapshot));
    }
    foc_platform_advanced_exit_critical(key);
    return (result == FOC_ADVANCED_POWER_TRIAL_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
#else
    (void)status;
    (void)telemetry;
    (void)snapshot;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_advanced_candidate_power_trial_finish(
    const foc_runtime_config_t *restore_realtime_config)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_runtime_config_t disabled_config;
    foc_status_t status;
    uint32_t key = foc_platform_advanced_enter_critical();

    foc_advanced_power_trial_abort(&g_foc_advanced_power_trial);
    foc_platform_disable_power_fast();
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    foc_rust_stop(g_foc_controller);
    status = (restore_realtime_config != 0) ?
        foc_rust_configure(g_foc_controller, restore_realtime_config) :
        FOC_STATUS_INVALID_ARGUMENT;
    if (status == FOC_STATUS_OK)
    {
        status = foc_rust_default_advanced_config(g_foc_controller,
                                                  &disabled_config);
    }
    if (status == FOC_STATUS_OK)
    {
        status = foc_rust_configure_advanced(g_foc_controller,
                                             &disabled_config);
    }
    foc_platform_advanced_exit_critical(key);
    return status;
#else
    (void)restore_realtime_config;
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

#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if (g_foc_lsi_session_running != 0U)
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_lsi_management_shared_abort_isr();
        g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
        g_foc_lsi_session_running = 0U;
        g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
    }
#endif

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
    g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_SYNC_RUNNING |
                                 FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID);
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

foc_status_t foc_platform_clear_faults(void)
{
#if defined(FOC_TARGET_STM32G431)
    const uint32_t self_test_flags = FOC_PLATFORM_DIAG_TIM1_CONFIGURED |
                                     FOC_PLATFORM_DIAG_ADC_CONFIGURED |
                                     FOC_PLATFORM_DIAG_ADC_CALIBRATED |
                                     FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID;
    const uint32_t clear_flags = FOC_PLATFORM_DIAG_DRIVER_FAULT |
                                 FOC_PLATFORM_DIAG_ADC_READ_ERROR |
                                 FOC_PLATFORM_DIAG_BREAK_LATCHED |
                                 FOC_PLATFORM_DIAG_CURRENT_TRIP |
                                 FOC_PLATFORM_DIAG_DEADLINE_MISSED |
                                 FOC_PLATFORM_DIAG_CONTROL_ERROR |
                                 FOC_PLATFORM_DIAG_OUTPUT_REJECTED |
                                 FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR |
                                 FOC_PLATFORM_DIAG_BUS_VOLTAGE_TRIP;
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t primask;
    foc_status_t rust_status;

    foc_platform_emergency_stop();
    if ((g_foc_controller == 0) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_FAULT_LATCHED))
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
    if ((foc_platform_sample_monitor_inputs() == 0U) ||
        ((g_foc_diagnostics.flags & self_test_flags) != self_test_flags) ||
        (foc_platform_driver_faulted() != 0U) ||
        (g_foc_diagnostics.bus_voltage_raw < g_foc_bus_min_raw) ||
        (g_foc_diagnostics.bus_voltage_raw > g_foc_bus_max_raw) ||
        (foc_platform_gate_is_low() == 0U) ||
        ((TIM1->CCER & channel_mask) != 0U) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    TIM1->DIER &= ~TIM_DIER_BIE;
    TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_B2IF);
    __DSB();
    if ((foc_platform_driver_faulted() != 0U) ||
        ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
        (foc_power_safety_clear_faults(
             &g_foc_power_safety, 1U) == 0U))
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_HARDWARE_FAULT;
    }
    g_foc_pending_rust_faults = 0U;
    g_foc_diagnostics.flags &= ~clear_flags;
    g_foc_diagnostics.control_fault_flags = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED)
    {
        return FOC_STATUS_HARDWARE_FAULT;
    }

    rust_status = foc_rust_clear_fault(g_foc_controller);
    if (rust_status != FOC_STATUS_OK)
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_CONTROL,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
        return rust_status;
    }
    if (foc_platform_restart_sync_monitor_safe() == 0U)
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_PLATFORM,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
        foc_platform_mirror_pending_rust_fault();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    foc_platform_refresh_safety_flags();
    return FOC_STATUS_OK;
#else
    return FOC_STATUS_NOT_CONFIGURED;
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
 *   无动态分配、无阻塞、无日志。周期数与具体源码/编译器/固件哈希绑定，权威
 *   实测值记录在 docs/performance 的板端复验报告中，不在源码里复制一个会漂移的
 *   数字。该证据不覆盖 trace-on、闭环或故障注入；超出软件截止即计入 deadline
 *   miss 并立即关断。
 *   No allocation, blocking or logging. Cycle counts are tied to an exact
 *   source/compiler/image hash; the authoritative measurements live in the
 *   board-validation report under docs/performance rather than being copied
 *   here. They do not cover trace-on, closed-loop or injected-fault paths.
 *   A software-deadline miss shuts down immediately.
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
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_torque_trial_action_t torque_trial_action =
        FOC_MOTION_TORQUE_TRIAL_ACTION_CONTINUE;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_power_trial_action_t advanced_trial_action =
        FOC_ADVANCED_POWER_TRIAL_ACTION_CONTINUE;
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    foc_as5600_alignment_trial_action_t alignment_trial_action =
        FOC_AS5600_ALIGNMENT_TRIAL_ACTION_CONTINUE;
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    foc_lsi_raw_sample_t lsi_raw_sample = {0};
    foc_lsi_capture_record_result_t lsi_capture_result =
        FOC_LSI_CAPTURE_RECORD_IGNORED;
#endif

    FOC_TIMING_PROBE_HIGH();
    if ((ADC1->ISR & ADC_ISR_JEOS) != 0U)
    {
#if defined(FOC_SYNC_EDGE_CONTROL_TICK)
#if defined(FOC_MONITOR_DWT_TIMING)
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
#endif
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
        g_foc_diagnostics.bus_voltage_raw = (uint16_t)ADC2->JDR2;
#else
        g_foc_diagnostics.bus_voltage_raw = (uint16_t)ADC1->JDR2;
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
        /* JEOS 在 rank2 完成后到达，因此 JDR1 电流与 JDR2 Vbus 属于同一
         * ADC1 注入序列。先抓取本拍寄存器事实，状态机和任何 CCR 写入都在
         * 三相电流重构之后执行。 */
        g_foc_diagnostics.lsi_sync_bus_voltage_raw =
            g_foc_diagnostics.bus_voltage_raw;
        g_foc_diagnostics.lsi_sync_bus_valid = 1U;
        ++g_foc_diagnostics.lsi_sync_bus_sample_count;
        lsi_raw_sample.control_tick = g_foc_diagnostics.sync_sample_count;
        lsi_raw_sample.current_u_raw = g_foc_diagnostics.phase_u_raw;
        lsi_raw_sample.current_v_raw = g_foc_diagnostics.phase_v_raw;
        lsi_raw_sample.bus_voltage_raw =
            g_foc_diagnostics.lsi_sync_bus_voltage_raw;
        lsi_raw_sample.compare_u = (uint16_t)TIM1->CCR1;
        lsi_raw_sample.compare_v = (uint16_t)TIM1->CCR2;
        lsi_raw_sample.compare_w = (uint16_t)TIM1->CCR3;
        lsi_raw_sample.pwm_period_ticks = (uint16_t)TIM1->ARR;
        lsi_raw_sample.flags = FOC_LSI_RAW_FLAG_ADC_VALID;
        if (g_foc_control_armed != 0U)
        {
            lsi_raw_sample.flags |= FOC_LSI_RAW_FLAG_DRIVE_ACTIVE;
            if (g_foc_lsi_active_drive_request ==
                FOC_LSI_DRIVE_PULSE_POSITIVE)
            {
                lsi_raw_sample.flags |= FOC_LSI_RAW_FLAG_PULSE_POSITIVE;
            }
            else if (g_foc_lsi_active_drive_request ==
                     FOC_LSI_DRIVE_PULSE_NEGATIVE)
            {
                lsi_raw_sample.flags |= FOC_LSI_RAW_FLAG_PULSE_NEGATIVE;
            }
        }
        if ((foc_platform_driver_faulted() != 0U) ||
            ((g_foc_diagnostics.flags &
              FOC_PLATFORM_DIAG_BREAK_LATCHED) != 0U))
        {
            lsi_raw_sample.flags |= FOC_LSI_RAW_FLAG_HARDWARE_FAULT;
        }
#endif
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
        if (g_foc_phase_voltage_capture.state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED)
        {
            foc_phase_voltage_sample_t phase_sample = {0};

            phase_sample.phase_u_raw = (uint16_t)ADC1->JDR2;
            phase_sample.phase_v_raw = (uint16_t)ADC1->JDR3;
            phase_sample.phase_w_raw = (uint16_t)ADC1->JDR4;
            phase_sample.current_u_raw = g_foc_diagnostics.phase_u_raw;
            phase_sample.current_v_raw = g_foc_diagnostics.phase_v_raw;
            /* 母线值来自 ADC2 rank2，与本拍同一 TIM1 触发；
             * 相电压由 ADC1 的较长四 rank 序列完成时，Vbus 已转换完毕。 */
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
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
        /* P2.5 raw input scheduler is a constant-time no-op while disabled.
         * It may only read/start the ADC regular group; injected current data,
         * control state and motor PWM remain outside its reach. */
        foc_external_input_platform_control_tick();
#endif
        delta_u = (uint16_t)((current_u_counts < 0) ?
            -current_u_counts : current_u_counts);
        delta_v = (uint16_t)((current_v_counts < 0) ?
            -current_v_counts : current_v_counts);
        delta_w = (uint16_t)((current_w_counts < 0) ?
            -current_w_counts : current_w_counts);
        peak = (delta_u > delta_v) ? delta_u : delta_v;
        peak = (peak > delta_w) ? peak : delta_w;
        trip_counts = foc_platform_current_trip_counts();
        if (((g_foc_control_armed != 0U) ||
             (g_foc_power_safety.state == FOC_POWER_SAFETY_ARMING)
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
             || (g_foc_lsi_session_running != 0U)
#endif
            ) &&
            ((g_foc_diagnostics.bus_voltage_raw < g_foc_bus_min_raw) ||
             (g_foc_diagnostics.bus_voltage_raw > g_foc_bus_max_raw)))
        {
            g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_BUS_VOLTAGE_TRIP;
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
            if (g_foc_lsi_session_running != 0U)
            {
                foc_platform_lsi_abort_session_isr();
            }
#endif
            foc_platform_latch_fault_fast(FOC_POWER_FAULT_BUS_VOLTAGE,
                                          FOC_RUST_FAULT_PLATFORM_INPUT);
        }
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
        if ((g_foc_control_armed != 0U) &&
            (foc_motion_torque_trial_begin_tick(
                 &g_foc_motion_torque_trial) ==
             FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP))
        {
            foc_platform_latch_fault_fast(
                FOC_POWER_FAULT_CONTROL,
                FOC_RUST_FAULT_MOTION_CONTROL);
        }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
        if (g_foc_control_armed != 0U)
        {
            advanced_trial_action = foc_advanced_power_trial_begin_tick(
                &g_foc_advanced_power_trial,
                g_foc_power_safety.fault_epoch,
                g_foc_diagnostics.deadline_miss_count);
            if (advanced_trial_action ==
                FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP)
            {
                foc_platform_latch_fault_fast(
                    FOC_POWER_FAULT_CONTROL,
                    FOC_RUST_FAULT_ADVANCED_CONTROL);
            }
        }
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
        if ((g_foc_control_armed != 0U) &&
            (foc_as5600_alignment_trial_begin_tick(
                 &g_foc_as5600_alignment_trial) ==
             FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP))
        {
            foc_platform_latch_fault_fast(
                FOC_POWER_FAULT_CONTROL,
                FOC_RUST_FAULT_PLATFORM_INPUT);
        }
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
        if (peak > trip_counts)
        {
            g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CURRENT_TRIP;
            lsi_raw_sample.flags |= FOC_LSI_RAW_FLAG_SOFTWARE_TRIP;
            if (g_foc_lsi_session_running != 0U)
            {
                foc_platform_latch_fault_fast(
                    FOC_POWER_FAULT_CURRENT,
                    FOC_RUST_FAULT_PLATFORM_INPUT);
            }
        }
        if (foc_lsi_capture_service_is_armed() != 0U)
        {
            lsi_capture_result =
                foc_lsi_capture_service_record_isr(&lsi_raw_sample);
            if (((lsi_capture_result ==
                  FOC_LSI_CAPTURE_RECORD_CONTRACT_ERROR) ||
                 (lsi_capture_result ==
                  FOC_LSI_CAPTURE_RECORD_STORAGE_ERROR)) &&
                (g_foc_lsi_session_running == 0U))
            {
                g_foc_diagnostics.flags |=
                    FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR;
                foc_platform_disable_power_fast();
            }
            else if ((lsi_capture_result ==
                      FOC_LSI_CAPTURE_RECORD_COMPLETE) &&
                     (g_foc_lsi_session_running == 0U))
            {
                foc_platform_disable_power_fast();
            }
        }
        if (g_foc_lsi_session_running != 0U)
        {
            timing_active = 1U;
            control_executed = 1U;
            control_cycle_start = DWT->CYCCNT;
            foc_platform_lsi_process_isr(&lsi_raw_sample,
                                         peak,
                                         lsi_capture_result);
            control_cycle_end = DWT->CYCCNT;
        }
        if ((g_foc_lsi_session_running == 0U) &&
            (g_foc_control_armed != 0U))
#else
        if (g_foc_control_armed != 0U)
#endif
        {
            foc_status_t control_status;
            foc_feedback_t feedback;
            foc_realtime_input_t input;
            foc_output_t output;
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
            foc_sensorless_composite_input_t sensorless_composite_input;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
            uint32_t motion_call_used = 0U;
            uint32_t motion_fault_requested = 0U;
            uint32_t motion_stop_requested = 0U;
#endif
#if defined(FLUXRT_TRACE_BUILD)
            foc_telemetry_t telemetry;
#endif
            foc_telemetry_t *telemetry_output = 0;

            timing_active = 1U;
            /* 取三相电流绝对偏移的最大值作为跳闸判据。用最大值而不是
             * 某一相，是为了在任意相序或任意单相故障下都能触发。
             * The trip uses the largest absolute offset of the three phases, so
             * it fires regardless of phase order or which single phase faults. */
            if (peak > g_foc_diagnostics.peak_current_delta_counts)
            {
                g_foc_diagnostics.peak_current_delta_counts = peak;
            }
            /* 三道独立检查：软件过流、驱动器故障引脚、控制器未绑定。
             * 任一命中都立即关断功率级且不调用 Rust。
             * Three independent checks: software over-current, the driver fault
             * pin, and an unbound controller. Any of them shuts the power stage
             * down immediately without calling into Rust. */
            uint32_t control_fault_epoch = g_foc_power_safety.fault_epoch;

            if ((foc_power_safety_output_permitted_inline(
                     &g_foc_power_safety, control_fault_epoch) == 0U) ||
                (peak > trip_counts) ||
                (foc_platform_driver_faulted() != 0U) ||
                (g_foc_controller == 0))
            {
                if (peak > trip_counts)
                {
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CURRENT_TRIP;
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_CURRENT,
                        FOC_RUST_FAULT_PLATFORM_INPUT);
                }
                else if (foc_platform_driver_faulted() != 0U)
                {
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_DRIVER,
                        FOC_RUST_FAULT_PLATFORM_INPUT);
                }
                else
                {
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_PLATFORM,
                        FOC_RUST_FAULT_PLATFORM_INPUT);
                }
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
                /* A23 V19 只迁移完整物理量快照，不开放实测相电压源。平台把
                 * 旧反馈确定性包装成 CommandModel；逐板标定门完成前相电压
                 * 始终没有 valid 位，也不会进入 Rust 观察器。
                 * A23 V19 only migrates the complete physical snapshot. The
                 * platform deterministically wraps legacy feedback as
                 * CommandModel; measured phase voltage remains unavailable. */
                foc_realtime_input_from_legacy(
                    &feedback,
                    g_foc_control_sequence,
                    1.0f / (float)FOC_CONTROL_FREQUENCY_HZ,
                    &input);
#if defined(FLUXRT_TRACE_BUILD)
                /* Rust 始终在控制器内部更新最近遥测；只有下一拍确实要写 trace 时，
                 * 才额外把 100 B 快照复制到 ISR 栈。管理线程通过
                 * foc_rust_get_telemetry() 直接读取内部快照，不再要求每拍复制两次。
                 * Rust always updates its internal snapshot. Copy the 100-byte
                 * value to the ISR stack only when the next trace tick is due;
                 * management reads the internal snapshot directly. */
                if ((g_foc_trace_enabled != 0U) &&
                    ((g_foc_trace_counter + 1U) >= g_foc_trace_divider))
                {
                    telemetry_output = &telemetry;
                }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
                if ((g_foc_advanced_probe.state ==
                     FOC_ADVANCED_PROBE_RUNNING) ||
                    (g_foc_advanced_probe.state ==
                     FOC_ADVANCED_PROBE_COMPLETE))
                {
                    (void)foc_advanced_probe_fail(
                        &g_foc_advanced_probe,
                        FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
                        g_foc_power_safety.fault_epoch);
                    g_foc_advanced_probe_active = 0U;
                }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                /* The bounded owner reads its compact coherent snapshot after
                 * the realtime step; do not copy the full 100-byte telemetry
                 * into the ISR stack solely for this trial. */
#endif
                control_cycle_start = DWT->CYCCNT;
                control_executed = 1U;
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
                if (g_foc_motion_runtime_enabled != 0U)
                {
                    foc_motion_dispatcher_result_t dispatch_result;

                    dispatch_result = foc_motion_dispatcher_consume_isr(
                        &g_foc_motion_dispatcher,
                        (uint32_t)rt_tick_get_millisecond(),
                        &g_foc_motion_realtime_request);
                    if (dispatch_result == FOC_MOTION_DISPATCHER_OK)
                    {
                        motion_call_used = 1U;
                        motion_fault_requested =
                            ((g_foc_motion_realtime_request.request_flags &
                              FOC_MOTION_REALTIME_REQUEST_FAULT) != 0U) ?
                                1U : 0U;
                        motion_stop_requested =
                            ((g_foc_motion_realtime_request.request_flags &
                              FOC_MOTION_REALTIME_REQUEST_STOP) != 0U) ?
                                1U : 0U;
                        control_status = foc_rust_realtime_step_with_motion(
                            g_foc_controller,
                            &g_foc_motion_context,
                            &input,
                            &g_foc_motion_realtime_request,
                            &output,
                            telemetry_output,
                            &g_foc_motion_realtime_output);
                    }
                    else if (dispatch_result ==
                             FOC_MOTION_DISPATCHER_NO_COMMAND)
                    {
                        /* A selected motion route may never fall through to the
                         * legacy speed controller.  Missing ownership data is a
                         * configuration failure and must cut power. */
                        (void)memset(&output, 0, sizeof(output));
                        motion_call_used = 1U;
                        motion_fault_requested = 1U;
                        control_status = FOC_STATUS_NOT_CONFIGURED;
                    }
                    else
                    {
                        (void)memset(&output, 0, sizeof(output));
                        motion_call_used = 1U;
                        motion_fault_requested = 1U;
                        control_status = FOC_STATUS_NOT_CONFIGURED;
                    }
                }
                else
#endif
                {
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_SENSORLESS_CONTROL_CANDIDATE)
                /* Formal full-speed composition point.  This branch is built
                 * into the motor-arm-disabled candidate today; with public
                 * platform capabilities still zero it deterministically fails
                 * closed if reached.  A future approval changes only the
                 * stopped-state capability/config transaction, not the ISR
                 * ordering or CCR ownership below. */
                (void)memset(&sensorless_composite_input, 0,
                             sizeof(sensorless_composite_input));
                sensorless_composite_input.struct_size =
                    sizeof(sensorless_composite_input);
                sensorless_composite_input.version =
                    FOC_SENSORLESS_COMPOSITE_INPUT_VERSION;
                sensorless_composite_input.platform_capabilities =
                    foc_sensorless_platform_capabilities();
                sensorless_composite_input.input_flags =
                    FOC_SENSORLESS_INPUT_INJECTION_PERMITTED |
                    FOC_SENSORLESS_INPUT_BEMF_VALID;
                if ((g_foc_sensorless_composite_output.status_flags &
                     FOC_SENSORLESS_COMPOSITE_OUTPUT_INJECTION_LIMITED) != 0U)
                {
                    sensorless_composite_input.input_flags |=
                        FOC_SENSORLESS_INPUT_APPLIED_INJECTION_LIMITED;
                }
                sensorless_composite_input.applied_request_sequence =
                    input.control_sequence;
                sensorless_composite_input.applied_injection_alpha_v =
                    g_foc_sensorless_composite_output.applied_injection_alpha_v;
                sensorless_composite_input.applied_injection_beta_v =
                    g_foc_sensorless_composite_output.applied_injection_beta_v;
                sensorless_composite_input.voltage_limit_v =
                    feedback.dc_bus_voltage * 0.95f * 0.577350269f;
                sensorless_composite_input.minimum_duty =
                    g_foc_platform_config.minimum_duty;
                sensorless_composite_input.maximum_duty =
                    g_foc_platform_config.maximum_duty;
                control_status = foc_rust_realtime_step_sensorless(
                    g_foc_controller,
                    &g_foc_sensorless_probe_context,
                    &input,
                    &sensorless_composite_input,
                    &output,
                    telemetry_output,
                    &g_foc_sensorless_probe_output,
                    &g_foc_sensorless_composite_output);
#else
                control_status = foc_rust_realtime_step(g_foc_controller,
                                                        &input,
                                                        &output,
                                                        telemetry_output);
#endif
                }
                ++g_foc_control_sequence;
                control_cycle_end = DWT->CYCCNT;
                g_foc_diagnostics.last_control_status = control_status;
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                if (((g_foc_advanced_power_trial.state ==
                      FOC_ADVANCED_POWER_TRIAL_STARTUP) ||
                     (g_foc_advanced_power_trial.state ==
                      FOC_ADVANCED_POWER_TRIAL_ACTIVE)) &&
                    (control_status == FOC_STATUS_OK))
                {
                    foc_status_t advanced_status =
                        foc_rust_get_advanced_power_trial_snapshot(
                            g_foc_controller,
                            &g_foc_advanced_shared_status.power_trial_snapshot);
                    if (advanced_status == FOC_STATUS_OK)
                    {
                        advanced_trial_action =
                            foc_advanced_power_trial_validate_control(
                                &g_foc_advanced_power_trial,
                                &g_foc_advanced_shared_status.power_trial_snapshot);
                    }
                    else
                    {
                        foc_advanced_power_trial_fail(
                            &g_foc_advanced_power_trial,
                            FOC_ADVANCED_POWER_TRIAL_RESULT_ADVANCED_FAULT,
                            g_foc_power_safety.fault_epoch,
                            g_foc_diagnostics.deadline_miss_count);
                        advanced_trial_action =
                            FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP;
                    }
                }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                if (advanced_trial_action ==
                    FOC_ADVANCED_POWER_TRIAL_ACTION_FAULT_STOP)
                {
                    ++g_foc_diagnostics.realtime_error_count;
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_CONTROL,
                        FOC_RUST_FAULT_ADVANCED_CONTROL);
                }
                else
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
                if ((motion_call_used != 0U) &&
                    (control_status == FOC_STATUS_OK))
                {
                    torque_trial_action =
                        foc_motion_torque_trial_record_commit(
                            &g_foc_motion_torque_trial,
                            &g_foc_motion_realtime_output);
                }
                if (motion_fault_requested != 0U)
                {
                    g_foc_diagnostics.control_fault_flags =
                        foc_rust_fault_flags(g_foc_controller);
                    ++g_foc_diagnostics.realtime_error_count;
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_CONTROL,
                        FOC_RUST_FAULT_MOTION_CONTROL);
                }
                else if ((motion_stop_requested != 0U) &&
                    ((control_status == FOC_STATUS_OK) ||
                     (control_status == FOC_STATUS_DISABLED)))
                {
                    /* A requested stop is a normal state transition, not a
                     * sticky control failure.  The combined Rust entry has
                     * already reset motion state; C owns the physical cut. */
                    g_foc_motion_runtime_enabled = 0U;
                    g_foc_motion_command_ready = 0U;
                    foc_platform_disable_power_fast();
                }
                else if (torque_trial_action ==
                         FOC_MOTION_TORQUE_TRIAL_ACTION_FAULT_STOP)
                {
                    ++g_foc_diagnostics.realtime_error_count;
                    g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_CONTROL_ERROR;
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_CONTROL,
                        FOC_RUST_FAULT_MOTION_CONTROL);
                }
                else
#endif
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
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_CONTROL,
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
                        ((motion_call_used != 0U) ||
                         (motion_fault_requested != 0U)) ?
                            FOC_RUST_FAULT_MOTION_CONTROL :
#endif
                        FOC_RUST_FAULT_ALGORITHM_OUTPUT);
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
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_OUTPUT,
                        FOC_RUST_FAULT_ALGORITHM_OUTPUT);
                }
                else
                {
                    if (foc_platform_commit_output(
                            &output, control_fault_epoch) != 0U)
                    {
                        ++g_foc_diagnostics.realtime_step_count;
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
                        alignment_trial_action =
                            foc_as5600_alignment_trial_record_commit(
                                &g_foc_as5600_alignment_trial,
                                foc_rust_state(g_foc_controller));
                        if (alignment_trial_action ==
                            FOC_AS5600_ALIGNMENT_TRIAL_ACTION_FAULT_STOP)
                        {
                            foc_platform_latch_fault_fast(
                                FOC_POWER_FAULT_CONTROL,
                                FOC_RUST_FAULT_PLATFORM_INPUT);
                        }
                        else if (alignment_trial_action ==
                                 FOC_AS5600_ALIGNMENT_TRIAL_ACTION_COMPLETE_STOP)
                        {
                            foc_platform_disable_power_fast();
                        }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                        if (g_foc_advanced_power_trial.state ==
                            FOC_ADVANCED_POWER_TRIAL_ACTIVE)
                        {
                            advanced_trial_action =
                                foc_advanced_power_trial_record_commit(
                                    &g_foc_advanced_power_trial);
                        }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
                        if (torque_trial_action ==
                            FOC_MOTION_TORQUE_TRIAL_ACTION_COMPLETE_STOP)
                        {
                            /* The 1,200th accepted Torque tick is committed, then
                             * Gate/MOE/CCER are cut in the same ADC ISR. Shell
                             * scheduling cannot extend the 100 ms window. */
                            g_foc_motion_runtime_enabled = 0U;
                            g_foc_motion_command_ready = 0U;
                            foc_platform_disable_power_fast();
                        }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                        if (advanced_trial_action ==
                            FOC_ADVANCED_POWER_TRIAL_ACTION_COMPLETE_STOP)
                        {
                            /* Commit the 1,200th accepted tick, then cut the
                             * physical path in this same ADC ISR. */
                            foc_platform_disable_power_fast();
                        }
#endif
#if defined(FLUXRT_TRACE_BUILD)
                        if (g_foc_control_armed != 0U)
                        {
                            trace_sampled = foc_platform_trace_capture(
                                &feedback, &output, &telemetry);
                        }
#endif
                    }
                }
            }
            if (control_executed == 0U)
            {
                control_cycle_start = DWT->CYCCNT;
                control_cycle_end = control_cycle_start;
            }
        }
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
        else if ((g_foc_advanced_probe_active != 0U) &&
                 (g_foc_advanced_probe.state ==
                  FOC_ADVANCED_PROBE_RUNNING))
        {
            /* P5.3 target evidence runs in the real ADC ISR while every physical
             * output enable stays off. Only inactive CCR preload values change. */
            timing_active = 1U;
            control_executed = 1U;
            control_cycle_start = DWT->CYCCNT;
#if defined(FOC_ADVANCED_CONTROL_CANDIDATE)
            (void)foc_platform_advanced_probe_step_isr(current_u_counts,
                                                       current_v_counts,
                                                       current_w_counts);
#else
            /* Mode 3 measures the formal combined transaction; modes 1 and 2
             * measure the standalone sensorless ABI and standalone compose
             * step, which the armed route no longer calls. */
            if (g_foc_sensorless_probe_mode ==
                FOC_SENSORLESS_PROBE_MODE_COMPOSITE)
            {
                (void)foc_platform_sensorless_composite_probe_step_isr(
                    current_u_counts,
                    current_v_counts,
                    current_w_counts);
            }
            else
            {
                (void)foc_platform_sensorless_probe_step_isr(current_u_counts,
                                                             current_v_counts,
                                                             current_w_counts);
            }
#endif
            control_cycle_end = DWT->CYCCNT;
        }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
        else if (g_foc_motion_probe.state == FOC_MOTION_PROBE_RUNNING)
        {
            /* No-power E1B evidence path.  It is intentionally outside the
             * armed branch and the platform start gate rejects arm while this
             * route exists.  Only CCR preload registers may change. */
            timing_active = 1U;
            control_executed = 1U;
            control_cycle_start = DWT->CYCCNT;
            (void)foc_platform_motion_probe_step_isr(current_u_counts,
                                                     current_v_counts,
                                                     current_w_counts);
            control_cycle_end = DWT->CYCCNT;
        }
#endif
        /* The steady-state path has no pending mirror. Avoid entering a second
         * PRIMASK critical section on every 12 kHz tick; a Break that arrives
         * after this volatile read is still cut off in hardware immediately
         * and is mirrored on the following monitor/control tick. */
        if (g_foc_pending_rust_faults != 0U)
        {
            foc_platform_mirror_pending_rust_fault();
        }
        ADC1->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
        ADC2->ISR = ADC_ISR_JEOC | ADC_ISR_JEOS;
        FOC_TIMING_PROBE_LOW();
#if defined(FOC_MONITOR_DWT_TIMING)
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
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
                if ((g_foc_motion_probe.state ==
                     FOC_MOTION_PROBE_RUNNING) ||
                    (g_foc_motion_probe.state ==
                     FOC_MOTION_PROBE_COMPLETE))
                {
                    (void)foc_motion_probe_fail(
                        &g_foc_motion_probe,
                        FOC_MOTION_PROBE_RESULT_CONTROL_FAILURE,
                        g_foc_power_safety.fault_epoch);
                }
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
                foc_advanced_power_trial_fail(
                    &g_foc_advanced_power_trial,
                    FOC_ADVANCED_POWER_TRIAL_RESULT_DEADLINE_MISSED,
                    g_foc_power_safety.fault_epoch,
                    g_foc_diagnostics.deadline_miss_count);
                if ((g_foc_advanced_probe.state ==
                     FOC_ADVANCED_PROBE_RUNNING) ||
                    (g_foc_advanced_probe.state ==
                     FOC_ADVANCED_PROBE_COMPLETE))
                {
                    (void)foc_advanced_probe_fail(
                        &g_foc_advanced_probe,
                        FOC_ADVANCED_PROBE_RESULT_CONTROL_FAILURE,
                        g_foc_power_safety.fault_epoch);
                    g_foc_advanced_probe_active = 0U;
                }
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
                if (g_foc_lsi_session_running != 0U)
                {
                    foc_platform_lsi_abort_session_isr();
                }
                else
#endif
                {
                    foc_platform_latch_fault_fast(
                        FOC_POWER_FAULT_DEADLINE,
                        FOC_RUST_FAULT_PLATFORM_INPUT);
                }
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
        /* Keep BIF/B2IF sticky until explicit recovery. Disable the interrupt
         * source before returning so the uncleared status cannot retrigger. */
        TIM1->DIER &= ~TIM_DIER_BIE;
        ++g_foc_diagnostics.break_fault_count;
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_BREAK_LATCHED;
        if (g_foc_diagnostics.power_fault_count == 0U)
        {
            g_foc_diagnostics.first_power_fault = FOC_POWER_FAULT_BREAK;
        }
        g_foc_diagnostics.last_power_fault = FOC_POWER_FAULT_BREAK;
        ++g_foc_diagnostics.power_fault_count;
        foc_power_safety_latch_fault(&g_foc_power_safety,
                                     FOC_POWER_FAULT_BREAK);
        g_foc_diagnostics.power_fault_epoch =
            g_foc_power_safety.fault_epoch;
        g_foc_pending_rust_faults |= FOC_RUST_FAULT_PLATFORM_INPUT;
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
        if (g_foc_lsi_session_running != 0U)
        {
            foc_platform_lsi_abort_session_isr();
        }
        else
#endif
        {
            foc_platform_disable_power_fast();
        }
        /* 同步采样时基也停掉：栅极已断，继续采样只会产生误导性的数据。
         * Also stop the sampling time base: with the gates cut, continued
         * sampling would only produce misleading data. */
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
#if FOC_ADC_TRIGGER_USES_TIM2_DIVIDER
        TIM2->CR1 &= ~TIM_CR1_CEN;
#endif
        g_foc_diagnostics.flags &= ~(FOC_PLATFORM_DIAG_SYNC_RUNNING |
                                     FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID);
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
    uint32_t fault_epoch = g_foc_power_safety.fault_epoch;

    if ((g_foc_control_armed == 0U) ||
        (foc_power_safety_output_permitted(
             &g_foc_power_safety, fault_epoch) == 0U))
    {
        return FOC_STATUS_DISABLED;
    }
    if ((foc_platform_driver_faulted() != 0U) ||
        ((g_foc_diagnostics.flags & (FOC_PLATFORM_DIAG_BREAK_LATCHED |
                                     FOC_PLATFORM_DIAG_CURRENT_TRIP |
                                     FOC_PLATFORM_DIAG_ADC_READ_ERROR)) != 0U))
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_PLATFORM,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
        foc_platform_mirror_pending_rust_fault();
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
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_OUTPUT,
                                      FOC_RUST_FAULT_ALGORITHM_OUTPUT);
        foc_platform_mirror_pending_rust_fault();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    /* 归一化占空比 -> CCR counts，+0.5 实现四舍五入。
     * Normalised duty to CCR counts, with +0.5 for rounding. */
    if (foc_platform_commit_output(output, fault_epoch) == 0U)
    {
        foc_platform_mirror_pending_rust_fault();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    ++g_foc_diagnostics.trial_apply_count;
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
static foc_status_t foc_platform_control_start_internal(
    float target_speed_rpm,
    uint32_t candidate_authority)
{
#if defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD) && \
    !defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    /* Calibration/Identification 都在编译期禁用普通 arm。即使应用层或未来脚本
     * 误调用 start，平台层仍先执行硬关断再拒绝，保证不存在绕过 Shell 的路径。 */
    (void)target_speed_rpm;
    (void)candidate_authority;
    foc_platform_emergency_stop();
    return FOC_STATUS_DISABLED;
#elif defined(FOC_TARGET_STM32G431)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t neutral_ticks;
    uint32_t primask;
    uint16_t bus_min_raw;
    uint16_t bus_max_raw;
    uint16_t arm_reject_facts = 0U;
    foc_power_arm_token_t arm_token;
    foc_feedback_t feedback;
    foc_status_t status;

    if ((g_foc_controller == 0) || (g_foc_control_armed != 0U)
#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
        || (g_foc_phase_voltage_capture.state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED)
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
        || (g_foc_advanced_probe_active != 0U)
#endif
       )
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    /* This dedicated image has exactly one arm authority: the fixed-envelope
     * Torque coordinator.  Generic foc_start is rejected even before a motion
     * route exists, so no Shell/API ordering can bypass the bounded trial. */
    if (candidate_authority != FOC_CONTROL_START_AUTHORITY_MOTION)
    {
        return FOC_STATUS_DISABLED;
    }
    if ((g_foc_motion_runtime_enabled == 0U) ||
        (g_foc_motion_torque_trial.state !=
         FOC_MOTION_TORQUE_TRIAL_STARTUP))
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
#else
    (void)candidate_authority;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    /* Advanced Lab also has one arm authority.  Generic foc_start and the
     * no-power probe cannot bypass the bounded P5.4A coordinator. */
    if (candidate_authority != FOC_CONTROL_START_AUTHORITY_ADVANCED)
    {
        return FOC_STATUS_DISABLED;
    }
    if (g_foc_advanced_power_trial.state !=
        FOC_ADVANCED_POWER_TRIAL_STARTUP)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    /* The truth image exposes one authority only. Generic foc_start remains
     * denied by both the application compile guard and this platform check. */
    if (candidate_authority !=
        FOC_CONTROL_START_AUTHORITY_AS5600_ALIGNMENT)
    {
        return FOC_STATUS_DISABLED;
    }
    if (g_foc_as5600_alignment_trial.state !=
        FOC_AS5600_ALIGNMENT_TRIAL_ARMED)
    {
        return FOC_STATUS_NOT_CONFIGURED;
    }
#endif
    status = foc_platform_read_feedback(&feedback);
    if (status != FOC_STATUS_OK)
    {
        return status;
    }
    g_foc_diagnostics.arm_reject_stage = FOC_ARM_REJECT_STAGE_NONE;
    g_foc_diagnostics.arm_reject_facts = 0U;
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
    arm_reject_facts = foc_platform_rearm_break2_before_arm();
    if ((arm_reject_facts != 0U) &&
        ((arm_reject_facts & FOC_ARM_REJECT_FACT_B2IF_REARMED) == 0U))
    {
        g_foc_diagnostics.arm_reject_stage =
            FOC_ARM_REJECT_STAGE_PRE_ENABLE;
        g_foc_diagnostics.arm_reject_facts = arm_reject_facts;
        foc_platform_latch_fault_fast(
            ((arm_reject_facts & (FOC_ARM_REJECT_FACT_BIF |
                                  FOC_ARM_REJECT_FACT_B2IF)) != 0U) ?
                FOC_POWER_FAULT_BREAK : FOC_POWER_FAULT_PLATFORM,
            FOC_RUST_FAULT_PLATFORM_INPUT);
        foc_platform_mirror_pending_rust_fault();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_HARDWARE_FAULT;
    }
    if ((arm_reject_facts & FOC_ARM_REJECT_FACT_B2IF_REARMED) != 0U)
    {
        /* Preserve evidence that a historical, inactive B2IF was re-armed.
         * Later pre/post failures overwrite this with the rejecting fact set. */
        g_foc_diagnostics.arm_reject_stage =
            FOC_ARM_REJECT_STAGE_PRE_ENABLE;
        g_foc_diagnostics.arm_reject_facts = arm_reject_facts;
    }
    if (foc_power_safety_begin_arm(&g_foc_power_safety, &arm_token) == 0U)
    {
        g_foc_diagnostics.arm_reject_stage = FOC_ARM_REJECT_STAGE_BEGIN;
        g_foc_diagnostics.arm_reject_facts =
            FOC_ARM_REJECT_FACT_SAFETY_STATE;
        return FOC_STATUS_HARDWARE_FAULT;
    }
    status = foc_rust_start_realtime(g_foc_controller, 1U, target_speed_rpm);
    if (status != FOC_STATUS_OK)
    {
        foc_power_safety_abort_arm(&g_foc_power_safety);
        return status;
    }
    if ((g_foc_power_safety.state != FOC_POWER_SAFETY_ARMING) ||
        (g_foc_power_safety.fault_epoch != arm_token.fault_epoch))
    {
        foc_platform_disable_power_fast();
        foc_platform_mirror_pending_rust_fault();
        return FOC_STATUS_HARDWARE_FAULT;
    }

    neutral_ticks = FOC_PWM_PERIOD_TICKS / 2U;
    g_foc_diagnostics.peak_current_delta_counts = 0U;
    g_foc_diagnostics.trial_apply_count = 0U;
    g_foc_diagnostics.realtime_step_count = 0U;
    g_foc_control_sequence = 0U;
    g_foc_diagnostics.realtime_error_count = 0U;
    foc_realtime_timing_reset(&g_foc_timing_stats);
    g_foc_diagnostics.maximum_isr_cycles = 0U;
    g_foc_diagnostics.maximum_precontrol_cycles = 0U;
    g_foc_diagnostics.maximum_control_cycles = 0U;
    g_foc_diagnostics.maximum_postcontrol_cycles = 0U;
    g_foc_diagnostics.deadline_miss_count = 0U;
    g_foc_diagnostics.last_control_status = FOC_STATUS_OK;
    g_foc_diagnostics.control_fault_flags = 0U;
    /* These fields mean "last rejected duty", not the neutral preload below.
     * Leave them at zero until an actual control/output rejection records a
     * triplet, so a normal stop cannot be mistaken for a rejected 50% command. */
    g_foc_diagnostics.last_duty_a_per_mille = 0U;
    g_foc_diagnostics.last_duty_b_per_mille = 0U;
    g_foc_diagnostics.last_duty_c_per_mille = 0U;
    TIM1->CCR1 = neutral_ticks;
    TIM1->CCR2 = neutral_ticks;
    TIM1->CCR3 = neutral_ticks;
    TIM1->EGR = TIM_EGR_UG;
    __DSB();

    /* The final arm commit is one short critical section. Hardware Break stays
     * asynchronous while IRQ delivery is masked, so BIF/MOE are checked both
     * before and after opening the physical path. */
    primask = __get_PRIMASK();
    __disable_irq();
    arm_reject_facts = foc_arm_diagnostics_pre_facts(
        TIM1->SR,
        TIM_SR_BIF,
        TIM_SR_B2IF,
        foc_platform_driver_faulted(),
        (g_foc_power_safety.state == FOC_POWER_SAFETY_ARMING) ? 1U : 0U,
        (g_foc_power_safety.fault_epoch == arm_token.fault_epoch) ? 1U : 0U,
        ((g_foc_diagnostics.bus_voltage_raw >= bus_min_raw) &&
         (g_foc_diagnostics.bus_voltage_raw <= bus_max_raw)) ? 1U : 0U);
    if (arm_reject_facts != 0U)
    {
        g_foc_diagnostics.arm_reject_stage =
            FOC_ARM_REJECT_STAGE_PRE_ENABLE;
        g_foc_diagnostics.arm_reject_facts = arm_reject_facts;
        status = FOC_STATUS_HARDWARE_FAULT;
    }
    else
    {
        TIM1->DIER |= TIM_DIER_BIE;
        TIM1->CCER |= channel_mask;
        TIM1->BDTR |= TIM_BDTR_MOE;
        FOC_GATE_ENABLE_PORT->BSRR = FOC_GATE_ENABLE_PINS;
        __DSB();
        arm_reject_facts = foc_arm_diagnostics_post_facts(
            TIM1->SR,
            TIM_SR_BIF,
            TIM_SR_B2IF,
            foc_platform_driver_faulted(),
            (g_foc_power_safety.state == FOC_POWER_SAFETY_ARMING) ? 1U : 0U,
            (g_foc_power_safety.fault_epoch == arm_token.fault_epoch) ? 1U : 0U,
            ((g_foc_diagnostics.bus_voltage_raw >= bus_min_raw) &&
             (g_foc_diagnostics.bus_voltage_raw <= bus_max_raw)) ? 1U : 0U,
            ((TIM1->CCER & channel_mask) == channel_mask) ? 1U : 0U,
            ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U,
            (foc_platform_gate_is_low() == 0U) ? 1U : 0U,
            1U);
        if ((arm_reject_facts == 0U) &&
            (foc_power_safety_commit_arm(
                 &g_foc_power_safety, &arm_token) == 0U))
        {
            arm_reject_facts |= FOC_ARM_REJECT_FACT_COMMIT;
        }
        if (arm_reject_facts != 0U)
        {
            g_foc_diagnostics.arm_reject_stage =
                FOC_ARM_REJECT_STAGE_POST_ENABLE;
            g_foc_diagnostics.arm_reject_facts = arm_reject_facts;
            status = FOC_STATUS_HARDWARE_FAULT;
        }
        else
        {
            g_foc_control_armed = 1U;
            g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_TRIAL_ARMED |
                                       FOC_PLATFORM_DIAG_REALTIME_ARMED;
            status = FOC_STATUS_OK;
        }
    }
    if (status != FOC_STATUS_OK)
    {
        foc_platform_latch_fault_fast(
            (((arm_reject_facts & (FOC_ARM_REJECT_FACT_BIF |
                                   FOC_ARM_REJECT_FACT_B2IF)) != 0U) ?
                FOC_POWER_FAULT_BREAK : FOC_POWER_FAULT_PLATFORM),
            FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (status != FOC_STATUS_OK)
    {
        foc_platform_mirror_pending_rust_fault();
        foc_platform_refresh_safety_flags();
        return status;
    }
    foc_platform_refresh_safety_flags();
    return FOC_STATUS_OK;
#else
    (void)target_speed_rpm;
    (void)candidate_authority;
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
foc_status_t foc_platform_as5600_alignment_trial_start(
    const foc_runtime_config_t *runtime_config,
    float startup_target_speed_rpm)
{
    foc_as5600_alignment_trial_result_t trial_result;
    foc_status_t status;

    trial_result = foc_as5600_alignment_trial_prepare(
        &g_foc_as5600_alignment_trial, runtime_config);
    if (trial_result != FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK)
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    trial_result = foc_as5600_alignment_trial_mark_armed(
        &g_foc_as5600_alignment_trial);
    if (trial_result != FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK)
    {
        foc_platform_emergency_stop();
        return FOC_STATUS_NOT_CONFIGURED;
    }
    status = foc_platform_control_start_internal(
        startup_target_speed_rpm,
        FOC_CONTROL_START_AUTHORITY_AS5600_ALIGNMENT);
    if (status != FOC_STATUS_OK)
    {
        foc_as5600_alignment_trial_fail(
            &g_foc_as5600_alignment_trial,
            FOC_AS5600_ALIGNMENT_TRIAL_RESULT_PLATFORM_FAULT,
            g_foc_power_safety.fault_epoch);
        foc_platform_emergency_stop();
    }
    return status;
}

foc_status_t foc_platform_as5600_alignment_trial_get_status(
    foc_as5600_alignment_trial_status_t *status)
{
    foc_as5600_alignment_trial_result_t result;
    uint32_t primask;

    if (status == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    result = foc_as5600_alignment_trial_get_status(
        &g_foc_as5600_alignment_trial, status);
    if (primask == 0U)
    {
        __enable_irq();
    }
    return (result == FOC_AS5600_ALIGNMENT_TRIAL_RESULT_OK) ?
        FOC_STATUS_OK : FOC_STATUS_NOT_CONFIGURED;
}
#endif

foc_status_t foc_platform_control_start(float target_speed_rpm)
{
    return foc_platform_control_start_internal(
        target_speed_rpm, FOC_CONTROL_START_AUTHORITY_GENERIC);
}

void foc_platform_control_stop(void)
{
#if defined(FOC_TARGET_STM32G431)
    uint32_t primask = __get_PRIMASK();
    uint32_t pending_faults;
    __disable_irq();
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    foc_motion_torque_trial_abort(&g_foc_motion_torque_trial);
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_ADVANCED_CONTROL_CANDIDATE)
    foc_advanced_power_trial_abort(&g_foc_advanced_power_trial);
#endif
#if defined(FLUXRT_AS5600_ALIGNMENT_BUILD)
    foc_as5600_alignment_trial_abort(&g_foc_as5600_alignment_trial);
#endif
#if defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if (g_foc_lsi_session_running != 0U)
    {
        foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
        foc_lsi_management_shared_abort_isr();
        g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
        g_foc_lsi_session_running = 0U;
        g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
    }
#endif
    foc_platform_disable_power_fast();
    pending_faults = g_foc_pending_rust_faults;
    g_foc_pending_rust_faults = 0U;
    if (g_foc_controller != 0)
    {
        /* Break may have stopped the ADC timebase before its deferred Rust
         * mirror could run. Drain that latch while interrupts are masked, then
         * stop; stop() preserves Fault when a fault bit is present. */
        if (pending_faults != 0U)
        {
            (void)foc_rust_latch_fault(g_foc_controller, pending_faults);
            g_foc_diagnostics.control_fault_flags =
                foc_rust_fault_flags(g_foc_controller);
        }
        foc_rust_stop(g_foc_controller);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    foc_platform_refresh_safety_flags();
#endif
}

foc_status_t foc_platform_lsi_start(uint32_t confirmation)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    foc_lsi_management_status_t management_status;
    foc_lsi_config_t config;
    foc_lsi_management_result_t management_result;
    uint16_t bus_min_raw;
    uint16_t bus_max_raw;
    uint16_t current_limit_counts;
    int32_t current_u_counts;
    int32_t current_v_counts;
    int32_t current_w_counts;
    uint32_t peak_counts;
    uint32_t primask;
    foc_status_t result = FOC_STATUS_NOT_CONFIGURED;

    if (confirmation != FOC_LSI_START_CONFIRMATION)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    foc_platform_refresh_safety_flags();
    bus_min_raw = foc_platform_bus_voltage_to_raw(
        g_foc_platform_config.minimum_bus_voltage_v);
    bus_max_raw = foc_platform_bus_voltage_to_raw(
        g_foc_platform_config.maximum_bus_voltage_v);
    current_limit_counts = (uint16_t)(FOC_LSI_START_MAX_CURRENT_A *
                                      FOC_CURRENT_COUNTS_PER_AMP + 0.5f);
    current_u_counts = (int32_t)g_foc_diagnostics.phase_u_offset -
                       (int32_t)g_foc_diagnostics.phase_u_raw;
    current_v_counts = (int32_t)g_foc_diagnostics.phase_v_offset -
                       (int32_t)g_foc_diagnostics.phase_v_raw;
    current_w_counts = -current_u_counts - current_v_counts;
    peak_counts = (uint32_t)((current_u_counts < 0) ?
        -current_u_counts : current_u_counts);
    if ((uint32_t)((current_v_counts < 0) ?
                   -current_v_counts : current_v_counts) > peak_counts)
    {
        peak_counts = (uint32_t)((current_v_counts < 0) ?
            -current_v_counts : current_v_counts);
    }
    if ((uint32_t)((current_w_counts < 0) ?
                   -current_w_counts : current_w_counts) > peak_counts)
    {
        peak_counts = (uint32_t)((current_w_counts < 0) ?
            -current_w_counts : current_w_counts);
    }

    if ((foc_lsi_management_shared_get_status(&management_status) == 0U) ||
        (foc_lsi_management_shared_get_config(&config) == 0U))
    {
        result = FOC_STATUS_NOT_CONFIGURED;
    }
    else if ((management_status.output.state != FOC_LSI_STATE_IDLE) ||
             (management_status.start_consumed != 0U) ||
             (g_foc_lsi_session_running != 0U))
    {
        result = FOC_STATUS_DISABLED;
    }
    else if (((g_foc_diagnostics.flags & FOC_LSI_REQUIRED_FLAGS) !=
              FOC_LSI_REQUIRED_FLAGS) ||
             ((g_foc_diagnostics.flags & FOC_LSI_FORBIDDEN_FLAGS) != 0U) ||
             (g_foc_diagnostics.lsi_sync_bus_valid == 0U) ||
             (g_foc_diagnostics.lsi_sync_bus_voltage_raw < bus_min_raw) ||
             (g_foc_diagnostics.lsi_sync_bus_voltage_raw > bus_max_raw) ||
             (peak_counts > current_limit_counts) ||
             (foc_lsi_capture_service_is_armed() != 0U) ||
             (foc_platform_gate_is_low() == 0U) ||
             ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ||
             ((TIM1->CCER & channel_mask) != 0U))
    {
        result = FOC_STATUS_NOT_CONFIGURED;
    }
    else if ((g_foc_lsi_executor.actuation.maximum_bias_current_a !=
              config.bias_current_a) ||
             (g_foc_lsi_executor.actuation.maximum_perturbation_voltage_v !=
              config.perturbation_voltage_v) ||
             (g_foc_lsi_executor.actuation.current_trip_a !=
              config.current_trip_a) ||
             (g_foc_lsi_executor.actuation.minimum_bus_voltage_v !=
              config.bus_voltage_min_v) ||
             (g_foc_lsi_executor.actuation.maximum_bus_voltage_v !=
              config.bus_voltage_max_v))
    {
        result = FOC_STATUS_NOT_CONFIGURED;
    }
    else if (foc_power_safety_begin_arm(
                 &g_foc_power_safety, &g_foc_lsi_arm_token) == 0U)
    {
        result = FOC_STATUS_HARDWARE_FAULT;
    }
    else if (foc_lsi_preload_session_open_active(
                 &g_foc_lsi_preload_session,
                 FOC_LSI_ACTIVE_CONFIRMATION,
                 config.max_active_ticks,
                 config.total_timeout_ticks,
                 g_foc_diagnostics.sync_sample_count) !=
             FOC_LSI_PRELOAD_RESULT_OK)
    {
        foc_power_safety_abort_arm(&g_foc_power_safety);
        result = FOC_STATUS_HARDWARE_FAULT;
    }
    else
    {
        management_result = foc_lsi_management_shared_request_start(
            confirmation,
            &management_status);
        if (management_result != FOC_LSI_MANAGEMENT_OK)
        {
            foc_lsi_preload_session_abort(&g_foc_lsi_preload_session);
            foc_power_safety_abort_arm(&g_foc_power_safety);
            result = FOC_STATUS_DISABLED;
        }
        else
        {
            g_foc_lsi_active_drive_request = FOC_LSI_DRIVE_OFF;
            g_foc_lsi_session_running = 1U;
            g_foc_diagnostics.flags |=
                FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING;
            result = FOC_STATUS_OK;
        }
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
#elif defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    (void)confirmation;
    return FOC_STATUS_NOT_CONFIGURED;
#else
    (void)confirmation;
    return FOC_STATUS_DISABLED;
#endif
}

foc_status_t foc_platform_lsi_break2_self_test(
    uint32_t confirmation,
    foc_lsi_break_test_result_t *result)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_IDENTIFICATION_BUILD)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    uint32_t primask;
    uint16_t facts;
    uint32_t passed;

    if (result == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(result, 0, sizeof(*result));
    result->version = FOC_LSI_BREAK_TEST_RESULT_VERSION;
    if (confirmation != FOC_LSI_BREAK_TEST_CONFIRMATION)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_foc_controller == 0) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_lsi_session_running != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED) ||
        (g_foc_diagnostics.bus_voltage_raw >= g_foc_bus_min_raw) ||
        (foc_platform_driver_faulted() != 0U))
    {
        foc_platform_disable_power_fast();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_DISABLED;
    }

    foc_platform_disable_power_fast();
    TIM1->DIER &= ~TIM_DIER_BIE;
    result->before_sr = TIM1->SR;
    result->before_bdtr = TIM1->BDTR;
    result->gate_low_before = foc_platform_gate_is_low();
    result->phase_outputs_before =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
        (result->gate_low_before == 0U) ||
        (result->phase_outputs_before != 0U) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
    {
        foc_platform_disable_power_fast();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_HARDWARE_FAULT;
    }

    /* CH1..CH3 and PB13..PB15 remain disabled.  MOE is raised only to give
     * the timer Break2 hardware a state that it must autonomously clear. */
    TIM1->BDTR |= TIM_BDTR_MOE;
    __DSB();
    result->armed_bdtr = TIM1->BDTR;
    if ((result->armed_bdtr & TIM_BDTR_MOE) == 0U)
    {
        foc_platform_disable_power_fast();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_HARDWARE_FAULT;
    }

    TIM1->EGR = TIM_EGR_B2G;
    __DSB();
    __NOP();
    __NOP();
    result->event_sr = TIM1->SR;
    result->event_bdtr = TIM1->BDTR;

    foc_platform_disable_power_fast();
    facts = foc_platform_rearm_break2_before_arm();
    result->rearm_facts = (uint32_t)facts;
    result->after_sr = TIM1->SR;
    result->after_bdtr = TIM1->BDTR;
    result->gate_low_after = foc_platform_gate_is_low();
    result->phase_outputs_after =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    passed = (((result->event_sr & TIM_SR_B2IF) != 0U) &&
              ((result->event_bdtr & TIM_BDTR_MOE) == 0U) &&
              (facts == (FOC_ARM_REJECT_FACT_B2IF |
                         FOC_ARM_REJECT_FACT_B2IF_REARMED)) &&
              ((result->after_sr & (TIM_SR_BIF | TIM_SR_B2IF)) == 0U) &&
              ((result->after_bdtr & TIM_BDTR_MOE) == 0U) &&
              (result->gate_low_after != 0U) &&
              (result->phase_outputs_after == 0U)) ? 1U : 0U;
    g_foc_diagnostics.arm_reject_stage =
        FOC_ARM_REJECT_STAGE_PRE_ENABLE;
    g_foc_diagnostics.arm_reject_facts = facts;
    if (passed == 0U)
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return (passed != 0U) ? FOC_STATUS_OK : FOC_STATUS_HARDWARE_FAULT;
#else
    (void)confirmation;
    if (result != 0)
    {
        (void)memset(result, 0, sizeof(*result));
        result->version = FOC_LSI_BREAK_TEST_RESULT_VERSION;
    }
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_lsi_break2_external_self_test(
    uint32_t confirmation,
    foc_lsi_break_external_test_result_t *result)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_IDENTIFICATION_BUILD)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    const uint32_t pin = FOC_BREAK_EXTERNAL_STIMULUS_PIN;
    const uint32_t pin_index = 12U;
    const uint32_t mode_mask = 3UL << (pin_index * 2U);
    const uint32_t af_mask = 0xFUL << ((pin_index - 8U) * 4U);
    uint32_t saved_moder;
    uint32_t saved_otyper;
    uint32_t saved_ospeedr;
    uint32_t saved_pupdr;
    uint32_t saved_afr;
    uint32_t saved_odr;
    uint32_t primask;
    uint32_t attempts;
    uint32_t stable_high = 0U;
    uint32_t configured = 0U;
    uint16_t facts = 0U;
    uint32_t passed;

    if (result == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(result, 0, sizeof(*result));
    result->version = FOC_LSI_BREAK_EXTERNAL_TEST_RESULT_VERSION;
    if (confirmation != FOC_LSI_BREAK_EXTERNAL_CONFIRMATION)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_foc_controller == 0) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_lsi_session_running != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED) ||
        (g_foc_diagnostics.bus_voltage_raw >= g_foc_bus_min_raw) ||
        (foc_platform_driver_faulted() != 0U))
    {
        foc_platform_disable_power_fast();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_DISABLED;
    }

    foc_platform_disable_power_fast();
    TIM1->DIER &= ~TIM_DIER_BIE;
    result->before_sr = TIM1->SR;
    result->before_bdtr = TIM1->BDTR;
    result->gate_low_before = foc_platform_gate_is_low();
    result->phase_outputs_before =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
        (result->gate_low_before == 0U) ||
        (result->phase_outputs_before != 0U) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
    {
        foc_platform_disable_power_fast();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return FOC_STATUS_HARDWARE_FAULT;
    }

    saved_moder = GPIOB->MODER;
    saved_otyper = GPIOB->OTYPER;
    saved_ospeedr = GPIOB->OSPEEDR;
    saved_pupdr = GPIOB->PUPDR;
    saved_afr = GPIOB->AFR[1];
    saved_odr = GPIOB->ODR;

    /* Release first, then select open-drain output.  At no point does PB12
     * actively drive the shared EN_FAULT network high. */
    GPIOB->BSRR = pin;
    GPIOB->OTYPER |= pin;
    GPIOB->OSPEEDR &= ~mode_mask;
    GPIOB->PUPDR &= ~mode_mask;
    GPIOB->MODER = (GPIOB->MODER & ~mode_mask) |
                   (1UL << (pin_index * 2U));
    __DSB();
    configured = 1U;

    /* PB12 normally boots in analog mode, where its digital IDR is not useful.
     * Validate the released high level only after enabling its digital output
     * buffer in open-drain/high state. */
    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        if (((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) != 0U) &&
            ((GPIOB->IDR & pin) != 0U))
        {
            ++stable_high;
            if (stable_high >= FOC_BREAK2_REARM_STABLE_READS)
            {
                result->line_high_before = 1U;
                break;
            }
        }
        else
        {
            stable_high = 0U;
        }
    }
    if (result->line_high_before != 0U)
    {
        TIM1->BDTR |= TIM_BDTR_MOE;
        __DSB();
        result->armed_bdtr = TIM1->BDTR;
        if ((result->armed_bdtr & TIM_BDTR_MOE) != 0U)
        {
            GPIOB->BSRR = pin << 16U;
            __DSB();
            for (attempts = 0U;
                 attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
                 ++attempts)
            {
                if (((TIM1->SR & TIM_SR_B2IF) != 0U) &&
                    ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
                    ((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) == 0U) &&
                    ((GPIOB->IDR & pin) == 0U))
                {
                    result->line_low_event = 1U;
                    break;
                }
            }
        }
    }
    result->event_sr = TIM1->SR;
    result->event_bdtr = TIM1->BDTR;

    /* Release the stimulus and make PB12 an input before restoring every
     * saved field.  This prevents a saved low ODR value from producing a
     * second low pulse while the pin configuration is being restored. */
    GPIOB->BSRR = pin;
    GPIOB->MODER &= ~mode_mask;
    GPIOB->OTYPER = (GPIOB->OTYPER & ~pin) | (saved_otyper & pin);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~mode_mask) |
                     (saved_ospeedr & mode_mask);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~mode_mask) |
                   (saved_pupdr & mode_mask);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~af_mask) | (saved_afr & af_mask);
    if ((saved_odr & pin) != 0U)
    {
        GPIOB->BSRR = pin;
    }
    else
    {
        GPIOB->BSRR = pin << 16U;
    }
    GPIOB->MODER = (GPIOB->MODER & ~mode_mask) | (saved_moder & mode_mask);
    __DSB();
    configured = 0U;

    stable_high = 0U;
    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        /* PB12 has now been restored to its original analog mode, whose
         * digital IDR is intentionally unavailable.  PA11 is the authoritative
         * released-level observation; stimulus_restored below independently
         * proves every PB12 configuration field was restored. */
        if ((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) != 0U)
        {
            ++stable_high;
            if (stable_high >= FOC_BREAK2_REARM_STABLE_READS)
            {
                result->line_high_after = 1U;
                break;
            }
        }
        else
        {
            stable_high = 0U;
        }
    }
    result->stimulus_restored =
        (((GPIOB->MODER & mode_mask) == (saved_moder & mode_mask)) &&
         ((GPIOB->OTYPER & pin) == (saved_otyper & pin)) &&
         ((GPIOB->OSPEEDR & mode_mask) == (saved_ospeedr & mode_mask)) &&
         ((GPIOB->PUPDR & mode_mask) == (saved_pupdr & mode_mask)) &&
         ((GPIOB->AFR[1] & af_mask) == (saved_afr & af_mask)) &&
         ((GPIOB->ODR & pin) == (saved_odr & pin))) ? 1U : 0U;

    foc_platform_disable_power_fast();
    if ((result->line_high_after != 0U) &&
        ((TIM1->SR & TIM_SR_B2IF) != 0U))
    {
        facts = foc_platform_rearm_break2_before_arm();
    }
    result->rearm_facts = (uint32_t)facts;
    result->after_sr = TIM1->SR;
    result->after_bdtr = TIM1->BDTR;
    result->gate_low_after = foc_platform_gate_is_low();
    result->phase_outputs_after =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    passed = (((result->event_sr & TIM_SR_B2IF) != 0U) &&
              ((result->event_bdtr & TIM_BDTR_MOE) == 0U) &&
              (result->line_low_event != 0U) &&
              (result->line_high_after != 0U) &&
              (result->stimulus_restored != 0U) &&
              (facts == (FOC_ARM_REJECT_FACT_B2IF |
                         FOC_ARM_REJECT_FACT_B2IF_REARMED)) &&
              ((result->after_sr & (TIM_SR_BIF | TIM_SR_B2IF)) == 0U) &&
              ((result->after_bdtr & TIM_BDTR_MOE) == 0U) &&
              (result->gate_low_after != 0U) &&
              (result->phase_outputs_after == 0U)) ? 1U : 0U;
    g_foc_diagnostics.arm_reject_stage = FOC_ARM_REJECT_STAGE_PRE_ENABLE;
    g_foc_diagnostics.arm_reject_facts = facts;
    if ((passed == 0U) || (configured != 0U))
    {
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return (passed != 0U) ? FOC_STATUS_OK : FOC_STATUS_HARDWARE_FAULT;
#else
    (void)confirmation;
    if (result != 0)
    {
        (void)memset(result, 0, sizeof(*result));
        result->version = FOC_LSI_BREAK_EXTERNAL_TEST_RESULT_VERSION;
    }
    return FOC_STATUS_NOT_CONFIGURED;
#endif
}

foc_status_t foc_platform_lsi_break2_isr_self_test(
    uint32_t confirmation,
    foc_lsi_break_isr_test_result_t *result)
{
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_IDENTIFICATION_BUILD)
    const uint32_t channel_mask = TIM_CCER_CC1E |
                                  TIM_CCER_CC2E |
                                  TIM_CCER_CC3E;
    const uint32_t pin = FOC_BREAK_EXTERNAL_STIMULUS_PIN;
    const uint32_t pin_index = 12U;
    const uint32_t mode_mask = 3UL << (pin_index * 2U);
    const uint32_t af_mask = 0xFUL << ((pin_index - 8U) * 4U);
    uint32_t saved_moder;
    uint32_t saved_otyper;
    uint32_t saved_ospeedr;
    uint32_t saved_pupdr;
    uint32_t saved_afr;
    uint32_t saved_odr;
    uint32_t primask;
    uint32_t attempts;
    uint32_t stable_high = 0U;
    uint32_t passed;

    if (result == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    (void)memset(result, 0, sizeof(*result));
    result->version = FOC_LSI_BREAK_ISR_TEST_RESULT_VERSION;
    if (confirmation != FOC_LSI_BREAK_ISR_CONFIRMATION)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }
    primask = __get_PRIMASK();
    if (primask != 0U)
    {
        return FOC_STATUS_DISABLED;
    }

    __disable_irq();
    if ((g_foc_controller == 0) ||
        (g_foc_control_armed != 0U) ||
        (g_foc_lsi_session_running != 0U) ||
        (g_foc_power_safety.state != FOC_POWER_SAFETY_DISABLED) ||
        (g_foc_diagnostics.bus_voltage_raw >= g_foc_bus_min_raw) ||
        (foc_platform_driver_faulted() != 0U))
    {
        foc_platform_disable_power_fast();
        __enable_irq();
        return FOC_STATUS_DISABLED;
    }

    foc_platform_disable_power_fast();
    TIM1->DIER &= ~TIM_DIER_BIE;
    result->before_sr = TIM1->SR;
    result->before_bdtr = TIM1->BDTR;
    result->break_count_before = g_foc_diagnostics.break_fault_count;
    result->fault_count_before = g_foc_diagnostics.power_fault_count;
    result->fault_epoch_before = g_foc_power_safety.fault_epoch;
    result->gate_low_before = foc_platform_gate_is_low();
    result->phase_outputs_before =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    if (((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U) ||
        (result->gate_low_before == 0U) ||
        (result->phase_outputs_before != 0U) ||
        ((TIM1->BDTR & TIM_BDTR_MOE) != 0U))
    {
        foc_platform_disable_power_fast();
        __enable_irq();
        return FOC_STATUS_HARDWARE_FAULT;
    }

    saved_moder = GPIOB->MODER;
    saved_otyper = GPIOB->OTYPER;
    saved_ospeedr = GPIOB->OSPEEDR;
    saved_pupdr = GPIOB->PUPDR;
    saved_afr = GPIOB->AFR[1];
    saved_odr = GPIOB->ODR;
    GPIOB->BSRR = pin;
    GPIOB->OTYPER |= pin;
    GPIOB->OSPEEDR &= ~mode_mask;
    GPIOB->PUPDR &= ~mode_mask;
    GPIOB->MODER = (GPIOB->MODER & ~mode_mask) |
                   (1UL << (pin_index * 2U));
    __DSB();
    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        if (((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) != 0U) &&
            ((GPIOB->IDR & pin) != 0U))
        {
            ++stable_high;
            if (stable_high >= FOC_BREAK2_REARM_STABLE_READS)
            {
                result->line_high_before = 1U;
                break;
            }
        }
        else
        {
            stable_high = 0U;
        }
    }
    if (result->line_high_before == 0U)
    {
        GPIOB->BSRR = pin;
        GPIOB->MODER &= ~mode_mask;
        GPIOB->OTYPER = (GPIOB->OTYPER & ~pin) | (saved_otyper & pin);
        GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~mode_mask) |
                         (saved_ospeedr & mode_mask);
        GPIOB->PUPDR = (GPIOB->PUPDR & ~mode_mask) |
                       (saved_pupdr & mode_mask);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~af_mask) | (saved_afr & af_mask);
        if ((saved_odr & pin) != 0U)
        {
            GPIOB->BSRR = pin;
        }
        else
        {
            GPIOB->BSRR = pin << 16U;
        }
        GPIOB->MODER = (GPIOB->MODER & ~mode_mask) |
                       (saved_moder & mode_mask);
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
        __enable_irq();
        return FOC_STATUS_HARDWARE_FAULT;
    }

    TIM1->DIER |= TIM_DIER_BIE;
    TIM1->BDTR |= TIM_BDTR_MOE;
    __DSB();
    result->armed_bdtr = TIM1->BDTR;
    GPIOB->BSRR = pin << 16U;
    __DSB();
    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        if (((TIM1->SR & TIM_SR_B2IF) != 0U) &&
            ((TIM1->BDTR & TIM_BDTR_MOE) == 0U) &&
            ((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) == 0U) &&
            ((GPIOB->IDR & pin) == 0U))
        {
            result->line_low_event = 1U;
            break;
        }
    }
    result->event_sr = TIM1->SR;
    result->event_bdtr = TIM1->BDTR;
    __enable_irq();

    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        if (g_foc_diagnostics.break_fault_count >
            result->break_count_before)
        {
            break;
        }
        __NOP();
    }

    __disable_irq();
    GPIOB->BSRR = pin;
    GPIOB->MODER &= ~mode_mask;
    GPIOB->OTYPER = (GPIOB->OTYPER & ~pin) | (saved_otyper & pin);
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~mode_mask) |
                     (saved_ospeedr & mode_mask);
    GPIOB->PUPDR = (GPIOB->PUPDR & ~mode_mask) |
                   (saved_pupdr & mode_mask);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~af_mask) | (saved_afr & af_mask);
    if ((saved_odr & pin) != 0U)
    {
        GPIOB->BSRR = pin;
    }
    else
    {
        GPIOB->BSRR = pin << 16U;
    }
    GPIOB->MODER = (GPIOB->MODER & ~mode_mask) | (saved_moder & mode_mask);
    __DSB();

    stable_high = 0U;
    for (attempts = 0U;
         attempts < FOC_BREAK_EXTERNAL_WAIT_ATTEMPTS;
         ++attempts)
    {
        if ((GPIOA->IDR & FOC_DRIVER_PROTECTION_PIN) != 0U)
        {
            ++stable_high;
            if (stable_high >= FOC_BREAK2_REARM_STABLE_READS)
            {
                result->line_high_after = 1U;
                break;
            }
        }
        else
        {
            stable_high = 0U;
        }
    }
    result->stimulus_restored =
        (((GPIOB->MODER & mode_mask) == (saved_moder & mode_mask)) &&
         ((GPIOB->OTYPER & pin) == (saved_otyper & pin)) &&
         ((GPIOB->OSPEEDR & mode_mask) == (saved_ospeedr & mode_mask)) &&
         ((GPIOB->PUPDR & mode_mask) == (saved_pupdr & mode_mask)) &&
         ((GPIOB->AFR[1] & af_mask) == (saved_afr & af_mask)) &&
         ((GPIOB->ODR & pin) == (saved_odr & pin))) ? 1U : 0U;
    result->break_count_after = g_foc_diagnostics.break_fault_count;
    result->fault_count_after = g_foc_diagnostics.power_fault_count;
    result->fault_epoch_after = g_foc_power_safety.fault_epoch;
    result->safety_state_after = (uint32_t)g_foc_power_safety.state;
    result->gate_low_after = foc_platform_gate_is_low();
    result->phase_outputs_after =
        ((TIM1->CCER & channel_mask) != 0U) ? 1U : 0U;
    result->break_irq_disabled_after =
        ((TIM1->DIER & TIM_DIER_BIE) == 0U) ? 1U : 0U;
    result->timer_stopped_after =
        ((TIM1->CR1 & TIM_CR1_CEN) == 0U) ? 1U : 0U;
    passed = (((result->event_sr & TIM_SR_B2IF) != 0U) &&
              ((result->event_bdtr & TIM_BDTR_MOE) == 0U) &&
              (result->break_count_after ==
               (result->break_count_before + 1U)) &&
              (result->fault_count_after ==
               (result->fault_count_before + 1U)) &&
              (result->fault_epoch_after > result->fault_epoch_before) &&
              (result->safety_state_after ==
               FOC_POWER_SAFETY_FAULT_LATCHED) &&
              (result->line_low_event != 0U) &&
              (result->line_high_after != 0U) &&
              (result->stimulus_restored != 0U) &&
              (result->gate_low_after != 0U) &&
              (result->phase_outputs_after == 0U) &&
              (result->break_irq_disabled_after != 0U) &&
              (result->timer_stopped_after != 0U)) ? 1U : 0U;
    if (passed == 0U)
    {
        TIM1->DIER &= ~TIM_DIER_BIE;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        foc_platform_latch_fault_fast(FOC_POWER_FAULT_BREAK,
                                      FOC_RUST_FAULT_PLATFORM_INPUT);
    }
    __enable_irq();
    return (passed != 0U) ? FOC_STATUS_OK : FOC_STATUS_HARDWARE_FAULT;
#else
    (void)confirmation;
    if (result != 0)
    {
        (void)memset(result, 0, sizeof(*result));
        result->version = FOC_LSI_BREAK_ISR_TEST_RESULT_VERSION;
    }
    return FOC_STATUS_NOT_CONFIGURED;
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
        foc_status_t status;
        __disable_irq();
        if (g_foc_controller != 0)
        {
            status = foc_rust_get_telemetry(g_foc_controller, telemetry);
        }
        else
        {
            *telemetry = (foc_telemetry_t){0};
            status = FOC_STATUS_NOT_CONFIGURED;
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
        return status;
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

uint32_t foc_platform_realtime_work_active(void)
{
#if defined(FOC_TARGET_STM32G431)
    if (g_foc_control_armed != 0U)
    {
        return 1U;
    }
#endif
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    (defined(FOC_ADVANCED_CONTROL_CANDIDATE) || \
     defined(FOC_SENSORLESS_CONTROL_CANDIDATE))
    if (g_foc_advanced_probe_active != 0U)
    {
        return 1U;
    }
#endif
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
    if (g_foc_motion_probe.state == FOC_MOTION_PROBE_RUNNING)
    {
        return 1U;
    }
#endif
#if defined(FOC_TARGET_STM32G431) && \
    defined(FLUXRT_LSI_IDENTIFICATION_BUILD)
    if (g_foc_lsi_session_running != 0U)
    {
        return 1U;
    }
#endif
    return 0U;
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

uint32_t foc_platform_time_sync_control_tick_isr(
    uint32_t edge_cycle_tick,
    uint32_t *control_tick)
{
    if (control_tick == 0)
    {
        return 0U;
    }
    *control_tick = UINT32_MAX;
#if defined(FOC_TARGET_STM32G431) && \
    defined(FOC_SYNC_EDGE_CONTROL_TICK)
    if (g_foc_sync_interval_valid != 0U)
    {
        /* H3 must use the same sequence domain exported by FTR.step.  The
         * continuously running sync_sample_count has the same rate but a
         * boot-relative origin, so Host-side bracketing can never compare it
         * directly with trace rows from a bounded control transaction. */
        uint32_t tick = g_foc_diagnostics.realtime_step_count;
        uint32_t latest_sample_cycle = g_foc_sync_previous_cycle;

        /* TIM4 runs below the ADC interrupt.  If the physical edge occurred
         * first but its IRQ was delayed by a new ADC sample, the latest sample
         * cycle lies after edge_cycle_tick.  Attribute the edge to the previous
         * completed control tick.  Signed modular subtraction is valid because
         * the IRQ delay is many orders below half a 32-bit DWT wrap. */
        *control_tick = foc_time_sync_control_tick_at_edge(
            edge_cycle_tick, latest_sample_cycle, tick);
        return 1U;
    }
#else
    (void)edge_cycle_tick;
#endif
    return 0U;
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

        /* A priority-0 Break cannot call Rust safely because it may pre-empt
         * the realtime bridge. Diagnostics are management-thread context, so
         * use this read boundary to complete the deferred C -> Rust fault
         * mirror before publishing a coherent snapshot. */
        foc_platform_mirror_pending_rust_fault();
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
