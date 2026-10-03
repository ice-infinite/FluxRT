/*
 * FluxRT —— RT-Thread 应用层与 Shell 命令层（唯一的操作入口）。
 * FluxRT - RT-Thread application and shell command layer (the only operator entry).
 *
 * 职责 / Responsibility:
 *   - 持有 Rust 控制器上下文与运行时配置，完成启动时的一次性初始化；
 *   - 提供 foc_start / foc_stop / foc_status；Diagnostic 增加 foc_cfg、foc_trace、
 *     foc_math_bench，Calibration 只增加停机相电压采集；
 *   - 周期性打印心跳与同拍 WCET，作为离线判读的唯一数据源。
 *
 * 边界 / Boundary:
 *   - 这里不是实时路径：所有命令都在 Shell 线程里跑，ISR 在
 *     foc/platform/stm32g431/ 里，本文件不碰任何寄存器；
 *   - 只通过 foc_platform.h 与 foc_rust_bridge.h 两个固定 C ABI 交互，不认识
 *     Rust 控制器内部状态，也不做任何 FOC 算法。
 *
 * 安全语义 / Safety semantics（本文件最重要的一条约定）:
 *   - 默认构建中 foc_start 是唯一可能 arm 的命令；motion candidate 专用构建另有
 *     固定包络 foc_motion_torque_trial，且只能经同一平台 arm 事务进入；
 *   - foc_stop 无条件调 foc_platform_control_stop()，一定把栅极使能和 TIM1
 *     输出关掉，之后才返回给 Shell；
 *   - foc_cfg 在检测到 FOC_PLATFORM_DIAG_REALTIME_ARMED 时直接拒绝改参数，
 *     避免运行中改配置；
 *   - 任何命令返回前功率级都处于安全状态，这是本项目不可协商的约束。
 *
 * 参考 / Reference: docs/架构与安全边界.md, docs/C与Rust混合架构.md,
 *                   docs/构建档与优化等级.md
 */

#include <rtthread.h>
#include <finsh.h>
#include <string.h>

#include "foc_math_accel.h"
#include "foc_platform.h"
#include "foc_production_profile.h"
#include "foc_rust_bridge.h"
#if defined(FLUXRT_PROTOCOL_NATIVE)
#include "foc_native_bridge.h"
#endif
#include "foc_shell_parse.h"
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
#include "foc_external_input_owner.h"
#include "foc_external_io_management.h"
#include "foc_external_io_platform.h"
#include "foc_external_input_platform.h"
#endif
#if defined(FLUXRT_ADVANCED_CANDIDATE_BUILD)
#include "foc_advanced_bridge.h"
#include "foc_platform_advanced_candidate.h"
#endif
#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
#include "foc_power_management.h"
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
#include "foc_config_bridge.h"
#include "foc_platform_motion_candidate.h"
#endif

/* 构建档名与 Rust 优化等级名由 custom.cmake 以 -D 传入，只用于打印。
 * 缺省值让本文件脱离 CMake 单独编译时仍可构建。
 * The profile name and Rust opt-level name are injected by custom.cmake as -D
 * and are only printed. The fallbacks keep this file compilable outside CMake. */
#ifndef FLUXRT_BUILD_PROFILE_NAME
#define FLUXRT_BUILD_PROFILE_NAME "diagnostic"
#endif
#ifndef FLUXRT_RUST_OPT_LEVEL_NAME
#define FLUXRT_RUST_OPT_LEVEL_NAME "s"
#endif
#ifndef FLUXRT_PRODUCT_PROFILE_NAME
#define FLUXRT_PRODUCT_PROFILE_NAME FLUXRT_G431_PRODUCT_PROFILE_NAME
#endif

/* Rust 控制器上下文 [bytes]。内容是 Rust 私有状态，C 侧只当作不透明缓冲；
 * 容量由 FOC_RUST_CONTEXT_CAPACITY 与 foc_rust_bridge.h 的 _Static_assert 固定。
 * Rust controller context in bytes. Its contents are Rust-private state and the C
 * side treats it as an opaque buffer; the capacity is fixed by
 * FOC_RUST_CONTEXT_CAPACITY and the _Static_assert in foc_rust_bridge.h. */
static foc_rust_context_t g_foc_controller;
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
/* Management-only external I/O composition. It remains default-off and never
 * enters the ADC ISR or owns the power stage. */
static foc_external_io_management_t g_foc_external_io_management;
static foc_external_input_port_t g_foc_external_input_port;
static foc_external_input_owner_t g_foc_external_input_owner;
#endif
/* 当前生效的运行时配置。只有 foc_cfg 成功提交后才整体替换，失败时保持原值。
 * The active runtime configuration. It is replaced as a whole only after foc_cfg
 * commits successfully, and keeps its old value on failure. */
static foc_runtime_config_t g_foc_runtime_config;
#if defined(FLUXRT_ADVANCED_CANDIDATE_BUILD)
/* Optional advanced policy keeps an independent ABI and management snapshot.
 * Boot always commits the generated disabled configuration first; merely
 * compiling the candidate can therefore never enable an advanced feature. */
static foc_advanced_runtime_config_t g_foc_advanced_config;
static foc_advanced_telemetry_t g_foc_advanced_telemetry;
#endif
#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
/* Independent management-rate policy. Boot commits only the generated disabled
 * configuration; there is deliberately no sensor or brake driver here. */
static foc_power_management_t g_foc_power_management;
#endif
#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
static foc_config_bundle_t g_foc_motion_probe_bundle;
static foc_product_command_t g_foc_motion_probe_command;
static foc_motion_probe_status_t g_foc_motion_probe_status;
static foc_realtime_timing_stats_t g_foc_motion_probe_timing;
static foc_platform_diagnostics_t g_foc_motion_probe_diagnostics;
static foc_runtime_config_t g_foc_motion_trial_runtime_saved;
static foc_motion_torque_trial_status_t g_foc_motion_trial_status;
#endif
#if defined(FLUXRT_RUNTIME_TUNING_BUILD)
/* foc_cfg 只由单一 tshell 线程调用。候选配置和诊断快照放在静态
 * 管理存储中，避免 296 B + 144 B 固定局部量与 Rust 配置/
 * rt_kprintf 深度叠加后压穿 2 KiB tshell 栈。它们绝不在 ISR 中读写。
 * foc_cfg has exactly one caller: the tshell thread. Keep its 296-byte candidate
 * and 144-byte diagnostics snapshot in static management storage so they do not
 * compound the Rust configuration and rt_kprintf call depth on the 2 KiB shell
 * stack. Neither object is ever touched by the ISR. */
static foc_runtime_config_t g_foc_runtime_candidate;
static foc_platform_diagnostics_t g_foc_cfg_diagnostics;
#endif
/* 启动时只读参数档的校验结果。Diagnostic 后续可在停机状态临时调参，
 * 因此该报告明确是 boot-profile 证据，不冒充运行中参数快照。 */
static foc_production_profile_report_t g_foc_profile_report;
static foc_production_profile_status_t g_foc_profile_status =
    FOC_PROFILE_INVALID_ARGUMENT;
/* A command sets this before opening a bounded realtime evidence window and
 * clears it only after its final report.  The platform activity gate below is
 * the steady-state guard; this flag also closes the start/finish race. */
static volatile uint32_t g_foc_management_log_inhibit;
#if defined(FLUXRT_TRACE_BUILD)
/* 实际生效的 trace 采样率 [Hz]，即 PWM 频率整除分频器后的结果，非请求值。
 * Effective trace rate in Hz: the PWM frequency divided by the integer divider,
 * not the requested value. */
static uint32_t g_foc_trace_rate_hz;
#endif
/* 平台层安全窗口。Flash 紧张，因此这里用 float 只在启动阶段配置一次，运行期
 * 全部由平台层使用；各字段量纲见表内注释。
 * Platform safety window. Because Flash is tight these floats are written once at
 * boot and used only by the platform layer; units are noted inline. */
static foc_platform_config_t g_foc_platform_config =
{
    sizeof(foc_platform_config_t),      /* 结构体自检大小 [bytes] / struct self-check size */
    FOC_PLATFORM_CONFIG_VERSION,        /* 配置 ABI 版本 / configuration ABI version */
    7.0f,                               /* 母线欠压下限 [V]：低于此值拒绝 arm，防止弱驱动下失控 */
    18.0f,                              /* 母线过压上限 [V]：12 V 供电下留出回馈制动余量 */
    1.15f,                              /* 软件过流跳闸 [A]：约额定 0.8 A 的 1.44 倍、低于堵转 */
    0.03f,                              /* 最小占空比 [0,1]：给自举电容留出最小充电脉宽 */
    0.97f,                              /* 最大占空比 [0,1]：与 0.03 一起构成 3%..97% 输出窗口 */
    FOC_DEFAULT_ISR_DEADLINE_CYCLES,    /* ISR 软件截止 [cycles]，预留嵌套中断与 DWT 开销 */
};

/*
 * 把内部 FOC 状态枚举翻译成给人看的名字。
 * Maps the internal FOC state enum to a human-readable name.
 *
 * 注意 "legacy-running"：FOC_STATE_RUNNING 属于早期有界试转包络，实时快环不用它。
 * Note "legacy-running": FOC_STATE_RUNNING belongs to the legacy bounded trial
 * envelope and is not used by the realtime fast loop.
 */
static const char *foc_state_name(foc_state_t state)
{
    switch (state)
    {
    case FOC_STATE_DISABLED: return "disabled";
    case FOC_STATE_RUNNING: return "legacy";
    case FOC_STATE_ALIGNMENT: return "align";
    case FOC_STATE_OPEN_LOOP_RAMP: return "ramp";
    case FOC_STATE_OPEN_LOOP_HOLD: return "hold";
    case FOC_STATE_OBSERVER_TRANSITION: return "handoff";
    case FOC_STATE_CLOSED_LOOP: return "closed";
    case FOC_STATE_FAULT: return "fault";
    default: return "uninitialized";
    }
}

/*
 * 把观测器后端枚举翻译成打印名。
 * Maps the observer backend enum to its printed name.
 *
 * "st-sto-pll-reserved" 只是占位：该后端尚未实现，选中它不会得到可用观测器。
 * "st-sto-pll-reserved" is a placeholder: that backend is not implemented, so
 * selecting it does not yield a working observer.
 */
static const char *foc_observer_name(foc_observer_backend_t backend)
{
    switch (backend)
    {
    case FOC_OBSERVER_SMO_PLL: return "rust-smo-pll";
    case FOC_OBSERVER_BEMF_PLL: return "rust-bemf-pll";
    case FOC_OBSERVER_ST_STO_PLL: return "st-sto-pll-reserved";
    default: return "invalid";
    }
}

/*
 * 把母线 ADC 原始码换算成 [mV]，供人读打印使用。
 * Converts the raw bus ADC code to millivolts for human-readable output.
 *
 * 换算 / Conversion:
 *   Vbus[mV] = raw * 52800 / 4095 = raw * Vref[3300 mV] / 4095 / 0.0625
 *   其中 0.0625 是 IHM16M1 的 1/16 母线分压比 [HW]，满量程约 52.8 V。
 *   The 0.0625 factor is the IHM16M1 16:1 bus divider [HW]; full scale is 52.8 V.
 *
 * 先乘后加 2047 再整除，做一次四舍五入；用 uint32_t 是因为 65535 * 52800
 * 会溢出 16 位。
 * The multiply-then-add-2047-then-divide performs one rounding step, and uint32_t
 * is required because 65535 * 52800 overflows 16 bits.
 */
static uint32_t foc_bus_voltage_mv(uint16_t raw)
{
    return ((uint32_t)raw * 52800UL + 2047UL) / 4095UL;
}

/*
 * foc_start [target_rpm] —— 唯一可能 arm 功率级的命令。
 * foc_start [target_rpm] - the only command that may arm the power stage.
 *
 * 参数 / Parameters:
 *   argv[1] 目标转速 [rpm]，可省略；省略时用配置里的 startup_final_speed_rpm。
 *   argv[1] target speed in rpm, optional; when omitted the configured
 *   startup_final_speed_rpm is used.
 *
 * 返回 / Returns:
 *   0   已 arm（各使能位与 TIM1 输出由平台层接管）；
 *   0   armed (the enable lines and TIM1 outputs are now owned by the platform);
 *   -1  参数错误或平台拒绝。**拒绝路径不改变任何输出**，功率级保持关闭。
 *   -1  bad arguments or a platform refusal. The refusal path changes no output;
 *       the power stage stays disabled.
 *
 * 为什么会被拒绝 / Why a start can be refused:
 *   foc_platform_control_start() 要求平台已配置/初始化/绑定控制器，且母线电压
 *   落在配置的窗口内 [V]。因此"电机不动"最常见的原因是母线窗口或诊断位未满足，
 *   拒绝时会把 status、诊断 flags 和实测 Vbus [mV] 一起打印出来。
 *   foc_platform_control_start() requires a configured, initialised and
 *   controller-bound platform plus a bus voltage [V] inside the configured
 *   window. That is why "the motor does not move" is most often a bus-window or
 *   diagnostic-bit problem; the refusal prints status, diagnostic flags and the
 *   measured Vbus in mV.
 *
 * 上下文 / Context: Shell 线程。函数返回时功率级若未 arm 就一定是关断的。
 * Shell thread. If the power stage is not armed when this returns, it is off.
 */
static int foc_start(int argc, char **argv)
{
#if defined(FLUXRT_MOTOR_ARM_DISABLED_BUILD)
    /* Calibration/Identification 固件都不允许普通电机 arm。应用层先停机，
     * 平台层还会做第二道编译期拒绝，防止未来新增调用点绕过 Shell。 */
    (void)argc;
    (void)argv;
    foc_platform_control_stop();
    rt_kprintf("FOC start REFUSED: this profile compiles out normal motor arm.\n");
    return -1;
#else
    float target_rpm = g_foc_runtime_config.startup_final_speed_rpm;
    foc_status_t status;

    if (argc > 2)
    {
        rt_kprintf("foc_start [rpm]\n");
        return -1;
    }
    if (argc == 2)
    {
        if (foc_shell_parse_i32_scaled(argv[1], 1U, &target_rpm) == 0U)
        {
            rt_kprintf("foc_start [rpm]\n");
            return -1;
        }
    }
    status = foc_platform_control_start(target_rpm);
    if (status != FOC_STATUS_OK)
    {
        foc_platform_diagnostics_t diagnostics = {0};
        (void)foc_platform_get_diagnostics(&diagnostics);
        rt_kprintf("FSTART,refused,%u,%08x,%u\n",
                   (unsigned int)status,
                   (unsigned int)diagnostics.flags,
                   (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
        return -1;
    }
    rt_kprintf("FSTART,armed,%d,%d,%d,%s,%u,%u,%u\n",
               (int)target_rpm,
               (int)g_foc_runtime_config.startup_final_speed_rpm,
               (int)(g_foc_runtime_config.startup_current_a * 1000.0f),
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.observer_enable,
               (unsigned int)g_foc_runtime_config.observer_update_divider,
               (unsigned int)g_foc_runtime_config.closed_loop_enable);
    return 0;
#endif
}
MSH_CMD_EXPORT(foc_start, -);

/*
 * foc_stop —— 无条件停机。
 * foc_stop - unconditional stop.
 *
 * 没有失败分支：即使从未启动过，foc_platform_control_stop() 也一定要把栅极
 * 使能和 TIM1 输出关掉。自动化脚本在结束时发送本命令，因此它必须幂等。
 * There is no failure branch: even if the motor never started,
 * foc_platform_control_stop() must disable the gate enable and the TIM1 outputs.
 * Scripts send this command on their exit path, so it has to be idempotent.
 */
static int foc_stop(int argc, char **argv)
{
    if (argc != 1)
    {
        rt_kprintf("foc_stop\n");
        return -1;
    }
    (void)argv;
    foc_platform_control_stop();
    rt_kprintf("FSTOP\n");
    return 0;
}
MSH_CMD_EXPORT(foc_stop, -);

/* Sticky platform/Rust faults can only be cleared through this explicit
 * recovery preflight. The command never arms PWM; a separate foc_start is still
 * required after current inputs, Vbus and board self-tests are healthy. */
static int foc_fault_clear(int argc, char **argv)
{
    foc_status_t status;

    if (argc != 1)
    {
        rt_kprintf("foc_fault_clear\n");
        return -1;
    }
    (void)argv;
    status = foc_platform_clear_faults();
    rt_kprintf("FCLEAR,%u\n", (unsigned int)status);
    return (status == FOC_STATUS_OK) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_fault_clear, -);

/*
 * 打印同拍 WCET 统计：FTIMING 同时是紧凑人读记录和机器接口。
 * Prints the same-tick WCET statistics. FTIMING is both the compact human-readable
 * record and the machine interface.
 *
 * 参数 / Parameters:
 *   diagnostics - foc_status 已取到的诊断快照，用于补齐 deadline miss、控制错误、
 *                 Rust fault 和平台 flags（本函数不再单独读取）。
 *                 the snapshot already fetched by foc_status, used to complete the
 *                 record with deadline misses, control errors, Rust faults and
 *                 platform flags (this function does not re-read them).
 *
 * 单位 / Units: 所有 cycles 字段是 DWT 周期 [cycles]，样本数是计数 [counts]，
 * deadline 是软件截止 [cycles]。
 * All cycle fields are DWT cycles, sample counts are plain counts, and the
 * deadline is the software deadline in cycles.
 *
 * 字段顺序 / Field order:
 *   由 simulation/capture_timing_baseline.py 直接解析，改动时必须同步脚本。
 *   Parsed directly by simulation/capture_timing_baseline.py; any change must be
 *   mirrored in that script.
 */
static void foc_print_timing(const foc_platform_diagnostics_t *diagnostics)
{
    foc_realtime_timing_stats_t timing = {0};

    (void)foc_platform_get_timing(&timing);
    /* FTIMING 同时是人可读的紧凑记录和机器接口；不再复制一行等价的长文本，避免
     * Diagnostic 档在 128 KiB Flash 中为重复格式串付费。字段顺序改动必须同步脚本。
     * FTIMING is the machine-readable record: the field order is the interface and
     * any change must be mirrored in capture_timing_baseline.py. */
    rt_kprintf("FTIMING,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)timing.version,
               (unsigned int)timing.sample_count,
               (unsigned int)timing.invalid_sample_count,
               (unsigned int)timing.wcet.step,
               (unsigned int)timing.wcet.total_cycles,
               (unsigned int)timing.wcet.precontrol_cycles,
               (unsigned int)timing.wcet.control_cycles,
               (unsigned int)timing.wcet.postcontrol_cycles,
               (unsigned int)timing.wcet.trace_enabled,
               (unsigned int)timing.wcet.trace_sampled,
               (unsigned int)timing.peak_precontrol_cycles,
               (unsigned int)timing.peak_control_cycles,
               (unsigned int)timing.peak_postcontrol_cycles,
               (unsigned int)g_foc_platform_config.isr_deadline_cycles,
               (unsigned int)diagnostics->deadline_miss_count,
               (unsigned int)diagnostics->realtime_error_count,
               (unsigned int)diagnostics->control_fault_flags,
               (unsigned int)diagnostics->flags,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.enabled,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.observer_voltage_correction_enable,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.pwm_feedforward_enable);
}

/*
 * foc_status —— 打印状态、观测器、电流/电压、诊断、时序与 ADC 原始值。
 * foc_status - dumps state, observer, currents/voltages, diagnostics, timing and
 * raw ADC values.
 *
 * 只读：不启动、不修改任何配置，因此在任何状态下都能安全调用。
 * Read-only: it neither starts anything nor changes configuration, so it is safe
 * to call in any state.
 *
 * 单位 / Units: 速度 [rpm]，电流 [mA]，电压 [mV]，角度 [mrad]，占空比 [per-mille]，
 * 时序 [cycles]，ADC 值为原始码 [counts]。
 * Speed in rpm, currents in mA, voltages in mV, angles in mrad, duty in
 * per-mille, timing in cycles and ADC values as raw counts.
 *
 * 上下文 / Context: Shell 线程。另外两行打印是给 capture 脚本解析的口径说明。
 * Shell thread. Two of the printed lines exist purely to state the measurement
 * convention for the capture scripts.
 */
static int foc_status(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    foc_telemetry_t telemetry = {0};
    uint32_t peak_current_ma;

    if (argc != 1)
    {
        rt_kprintf("foc_status\n");
        return -1;
    }
    (void)argv;
    (void)foc_platform_get_diagnostics(&diagnostics);
    (void)foc_platform_get_telemetry(&telemetry);
    /* 峰值电流以"相对标定零点的 ADC delta 码"上报，再换算成 [mA]：
     * 1595 uA/count = 3300 mV / (4095 * 0.33 ohm * 1.53)，只含 [HW] 电流链
     * 参数（3.3 V 参考、0.33 ohm 分流、1.53 倍放大），与母线分压无关；
     * 余数补 500 再整除，做一次四舍五入；用整数运算避免在 Shell 线程引入
     * 浮点打印依赖。
     * Peak current is reported as an ADC delta relative to the calibrated offset
     * and then converted to mA: 1595 uA/count = 3300 mV / (4095 * 0.33 ohm *
     * 1.53), which contains only the [HW] current-chain parameters (3.3 V
     * reference, 0.33 ohm shunt, 1.53x gain) and no bus divider. Adding 500
     * before the division rounds once, and integer math avoids pulling float
     * formatting into the shell thread. */
    peak_current_ma = ((uint32_t)diagnostics.peak_current_delta_counts * 1595UL + 500UL) / 1000UL;
    rt_kprintf("FSTAT,%s,%s,%u,%u,%d,%d\n",
               foc_state_name(telemetry.state),
               foc_observer_name(telemetry.observer_backend),
               (unsigned int)telemetry.observer_reliable,
               (unsigned int)telemetry.closed_loop_active,
               (int)telemetry.target_speed_rpm,
               (int)telemetry.measured_speed_rpm);
    rt_kprintf("FSTAT,iref=%d/%d,i=%d/%d,v=%d/%d\n",
               (int)(telemetry.id_reference_a * 1000.0f),
               (int)(telemetry.iq_reference_a * 1000.0f),
               (int)(telemetry.id_measured_a * 1000.0f),
               (int)(telemetry.iq_measured_a * 1000.0f),
               (int)(telemetry.vd_command_v * 1000.0f),
               (int)(telemetry.vq_command_v * 1000.0f));
    rt_kprintf("FOBS,%d/%d,%d,%d/%u,%02x,%u,%u/%u,%u\n",
               (int)(telemetry.observer_bemf_alpha_v * 1000.0f),
               (int)(telemetry.observer_bemf_beta_v * 1000.0f),
               (int)(telemetry.observer_pll_phase_error_rad * 1000.0f),
               (int)telemetry.observer_speed_mean_rpm,
               (unsigned int)telemetry.observer_speed_variance_rpm2,
               (unsigned int)telemetry.observer_reliability_flags,
               (unsigned int)telemetry.observer_reliable_samples,
               (unsigned int)(telemetry.observer_wait_elapsed_s * 1000.0f),
               (unsigned int)(telemetry.observer_loss_elapsed_s * 1000.0f),
               (unsigned int)telemetry.voltage_limited);
    rt_kprintf("FOC f=%08x s=%u e=%u ISR=%u/%u miss=%u pk=%u(%umA)\n",
               (unsigned int)diagnostics.flags,
               (unsigned int)diagnostics.realtime_step_count,
               (unsigned int)diagnostics.realtime_error_count,
               (unsigned int)diagnostics.maximum_isr_cycles,
               (unsigned int)g_foc_platform_config.isr_deadline_cycles,
               (unsigned int)diagnostics.deadline_miss_count,
               (unsigned int)diagnostics.peak_current_delta_counts,
               (unsigned int)peak_current_ma);
    /* Per-section maxima remain available in the machine-readable FTIMING line
     * below. Do not duplicate the explanatory text here: Diagnostic is within
     * hundreds of bytes of the G431RB Flash limit, and these independent peaks
     * must not be summed anyway. */
    /* Rates, divider, actuation delay and ADC trigger are also encoded in
     * FTIMING; keep one canonical diagnostic representation to save Flash. */
    /* Sync/monitor timing is retained in FTIMING below. The human-readable copy
     * was removed to keep Diagnostic inside the 128 KiB device. */
    foc_print_timing(&diagnostics);
    rt_kprintf("FOC st=%u rf=%08x duty=%u/%u/%u\n",
               (unsigned int)diagnostics.last_control_status,
               (unsigned int)diagnostics.control_fault_flags,
               (unsigned int)diagnostics.last_duty_a_per_mille,
               (unsigned int)diagnostics.last_duty_b_per_mille,
               (unsigned int)diagnostics.last_duty_c_per_mille);
    rt_kprintf("FFAULT,%08x,%08x,%u,%u,ARM,%u,%04x\n",
               (unsigned int)diagnostics.first_power_fault,
               (unsigned int)diagnostics.last_power_fault,
               (unsigned int)diagnostics.power_fault_count,
               (unsigned int)diagnostics.power_fault_epoch,
               (unsigned int)diagnostics.arm_reject_stage,
               (unsigned int)diagnostics.arm_reject_facts);
    rt_kprintf("FPROF,%u,%u,%08x,%08x,%02x,%08x,%08x\n",
               (unsigned int)g_foc_profile_status,
               (unsigned int)g_foc_profile_report.profile_revision,
               (unsigned int)g_foc_production_profile.board_id,
               (unsigned int)g_foc_production_profile.motor_id,
               (unsigned int)g_foc_profile_report.approval_flags,
               (unsigned int)g_foc_profile_report.computed_runtime_config_crc32,
               (unsigned int)g_foc_profile_report.computed_record_crc32);
    /* 采样链换算 / Sampling-chain conversion:
     *   U/V/W raw  - ADC 注入组原始码 [counts]，未减零点；
     *   offsets    - 32 次静态标定得到的零点 [counts]；
     *   Vbus       - raw 经 1/16 分压换算后的 [mV]（见 foc_bus_voltage_mv）；
     *   Pot        - 电位器原始码 [counts]，仅诊断用，不参与控制。
     *   the injected-group raw codes, the 32-sample calibrated offsets, the mV
     *   bus value (see foc_bus_voltage_mv) and the potentiometer code, which is
     *   diagnostic only and never used by the control loop. */
    rt_kprintf("FADC,%u/%u/%u,%u/%u/%u,%u,%u,%u\n",
               (unsigned int)diagnostics.phase_u_raw,
               (unsigned int)diagnostics.phase_v_raw,
               (unsigned int)diagnostics.phase_w_raw,
               (unsigned int)diagnostics.phase_u_offset,
               (unsigned int)diagnostics.phase_v_offset,
               (unsigned int)diagnostics.phase_w_offset,
               (unsigned int)diagnostics.bus_voltage_raw,
               (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw),
               (unsigned int)diagnostics.potentiometer_raw);
    return 0;
}
MSH_CMD_EXPORT(foc_status, -);

#if defined(FOC_EXTERNAL_IO_FRAMEWORK) && \
    defined(FLUXRT_INPUT_ANALOG) && \
    defined(FLUXRT_DIAGNOSTIC_BUILD)
/*
 * foc_input_probe -- one-shot, no-power raw Analog/ADC ownership probe.
 *
 * This command deliberately bypasses the product capability/config path: the
 * board input mask remains zero until S4 is accepted.  It only proves that the
 * already bound raw port can temporarily own ADC1 regular conversion while
 * stopped-state Vbus monitoring moves to ADC2.  It never submits a ProductCommand,
 * never arms the Axis and always tries to return ownership before it exits.
 */
static int foc_input_probe(int argc, char **argv)
{
    const uint32_t required_flags =
        FOC_PLATFORM_DIAG_GATE_SAFE |
        FOC_PLATFORM_DIAG_ADC_CONFIGURED |
        FOC_PLATFORM_DIAG_ADC_CALIBRATED |
        FOC_PLATFORM_DIAG_SYNC_RUNNING;
    const uint32_t forbidden_flags =
        FOC_PLATFORM_DIAG_DRIVER_FAULT |
        FOC_PLATFORM_DIAG_OUTPUT_ACTIVE |
        FOC_PLATFORM_DIAG_ADC_READ_ERROR |
        FOC_PLATFORM_DIAG_BREAK_LATCHED |
        FOC_PLATFORM_DIAG_CURRENT_TRIP |
        FOC_PLATFORM_DIAG_TRIAL_ARMED |
        FOC_PLATFORM_DIAG_REALTIME_ARMED |
        FOC_PLATFORM_DIAG_DEADLINE_MISSED |
        FOC_PLATFORM_DIAG_CONTROL_ERROR |
        FOC_PLATFORM_DIAG_OUTPUT_REJECTED |
        FOC_PLATFORM_DIAG_LSI_CAPTURE_ERROR |
        FOC_PLATFORM_DIAG_LSI_SESSION_RUNNING |
        FOC_PLATFORM_DIAG_BUS_VOLTAGE_TRIP;
    foc_platform_diagnostics_t diagnostics = {0};
    foc_feedback_t feedback = {0};
    foc_external_input_raw_sample_t sample = {0};
    foc_external_input_raw_health_t health = {0};
    foc_external_input_port_result_t start_status;
    foc_external_input_port_result_t read_status;
    foc_external_input_port_result_t health_status;
    foc_external_input_port_result_t stop_status =
        FOC_EXTERNAL_INPUT_PORT_BUSY;
    foc_status_t diagnostics_status;
    uint32_t feedback_failures = 0U;
    uint32_t bus_min = 0xFFFFU;
    uint32_t bus_max = 0U;
    uint32_t sample_index;
    uint32_t stop_attempt;
    uint32_t baseline_interval_samples;
    uint32_t baseline_interval_min;
    uint32_t baseline_interval_max;
    uint32_t baseline_monitor_min;
    uint32_t baseline_monitor_max;

    if (argc != 1)
    {
        rt_kprintf("foc_input_probe\n");
        return -1;
    }
    (void)argv;

    /* Idempotently force the power-output path off before inspecting guards. */
    foc_platform_control_stop();
    diagnostics_status = foc_platform_get_diagnostics(&diagnostics);
    if ((diagnostics_status != FOC_STATUS_OK) ||
        ((diagnostics.flags & required_flags) != required_flags) ||
        ((diagnostics.flags & forbidden_flags) != 0U) ||
        (g_foc_external_input_port.initialized == 0U) ||
        ((g_foc_external_input_port.supported_input_mask &
          FOC_EXTERNAL_INPUT_ANALOG) == 0U) ||
        (g_foc_external_input_port.enabled_input_mask != 0U) ||
        (foc_external_input_platform_regular_adc_owned() != 0U))
    {
        rt_kprintf("FIPROBE,refused,%u,%08x,%08x,%08x,%u\n",
                   (unsigned int)diagnostics_status,
                   (unsigned int)diagnostics.flags,
                   (unsigned int)required_flags,
                   (unsigned int)forbidden_flags,
                   (unsigned int)g_foc_external_input_port.enabled_input_mask);
        return -1;
    }

    baseline_interval_samples = diagnostics.sync_interval_sample_count;
    baseline_interval_min = diagnostics.sync_interval_min_cycles;
    baseline_interval_max = diagnostics.sync_interval_max_cycles;
    baseline_monitor_min = diagnostics.monitor_isr_min_cycles;
    baseline_monitor_max = diagnostics.monitor_isr_max_cycles;

    start_status = foc_external_input_port_start(
        &g_foc_external_input_port, FOC_EXTERNAL_INPUT_ANALOG);
    if (start_status != FOC_EXTERNAL_INPUT_PORT_OK)
    {
        rt_kprintf("FIPROBE,start,%u\n", (unsigned int)start_status);
        return -1;
    }

    /* The real ADC ISR owns raw capture.  The Shell only requests ten monitor
     * snapshots so the ADC2 Vbus fallback is exercised without faking ISR ticks. */
    for (sample_index = 0U; sample_index < 10U; ++sample_index)
    {
        rt_thread_mdelay(10);
        if ((foc_platform_read_feedback(&feedback) != FOC_STATUS_OK) ||
            (foc_platform_get_diagnostics(&diagnostics) != FOC_STATUS_OK))
        {
            ++feedback_failures;
            continue;
        }
        if ((uint32_t)diagnostics.bus_voltage_raw < bus_min)
        {
            bus_min = diagnostics.bus_voltage_raw;
        }
        if ((uint32_t)diagnostics.bus_voltage_raw > bus_max)
        {
            bus_max = diagnostics.bus_voltage_raw;
        }
    }

    read_status = foc_external_input_port_read_latest(
        &g_foc_external_input_port,
        FOC_EXTERNAL_INPUT_ANALOG,
        &sample);
    health_status = foc_external_input_port_get_health(
        &g_foc_external_input_port, &health);

    /* A conversion may be in its few-cycle active window.  Retry only until
     * that already-started conversion completes; never abort or reconfigure ADC1. */
    for (stop_attempt = 0U; stop_attempt < 64U; ++stop_attempt)
    {
        stop_status = foc_external_input_port_stop(
            &g_foc_external_input_port, FOC_EXTERNAL_INPUT_ANALOG);
        if (stop_status != FOC_EXTERNAL_INPUT_PORT_BUSY)
        {
            break;
        }
    }
    (void)foc_platform_get_diagnostics(&diagnostics);
    if (bus_min == 0xFFFFU)
    {
        bus_min = diagnostics.bus_voltage_raw;
        bus_max = diagnostics.bus_voltage_raw;
    }
    rt_kprintf("FIPROBE,timing,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)baseline_interval_samples,
               (unsigned int)diagnostics.sync_interval_sample_count,
               (unsigned int)baseline_interval_min,
               (unsigned int)baseline_interval_max,
               (unsigned int)diagnostics.sync_interval_min_cycles,
               (unsigned int)diagnostics.sync_interval_max_cycles,
               (unsigned int)baseline_monitor_min,
               (unsigned int)baseline_monitor_max,
               (unsigned int)diagnostics.monitor_isr_min_cycles,
               (unsigned int)diagnostics.monitor_isr_max_cycles);
    rt_kprintf("FIPROBE,result,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)start_status,
               (unsigned int)read_status,
               (unsigned int)health_status,
               (unsigned int)stop_status,
               (unsigned int)health.sample_count,
               (unsigned int)health.missed_capture_count,
               (unsigned int)sample.sequence,
               (unsigned int)sample.sampled_at_us,
               (int)sample.raw_value,
               (unsigned int)sample.quality_flags,
               (unsigned int)bus_min,
               (unsigned int)bus_max,
               (unsigned int)feedback_failures,
               (unsigned int)((diagnostics.flags &
                   FOC_PLATFORM_DIAG_ADC2_VBUS_FALLBACK_USED) != 0U),
               (unsigned int)health.last_error,
               (unsigned int)g_foc_external_input_port.enabled_input_mask,
               (unsigned int)foc_external_input_platform_regular_adc_owned());

    return ((read_status == FOC_EXTERNAL_INPUT_PORT_OK) &&
            (health_status == FOC_EXTERNAL_INPUT_PORT_OK) &&
            (stop_status == FOC_EXTERNAL_INPUT_PORT_OK) &&
            (health.sample_count != 0U) &&
            (feedback_failures == 0U) &&
            ((diagnostics.flags &
              FOC_PLATFORM_DIAG_ADC2_VBUS_FALLBACK_USED) != 0U) &&
            (g_foc_external_input_port.enabled_input_mask == 0U) &&
            (foc_external_input_platform_regular_adc_owned() == 0U)) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_input_probe, -);
#endif

#if defined(FLUXRT_PHASE_VOLTAGE_CAPTURE_BUILD)
#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_BENCHMARK)
/*
 * foc_math_bench [1..128] —— 功率级关闭时测 CORDIC 分项周期与固定向量误差。
 * foc_math_bench [1..128] - measures isolated CORDIC calls and fixed-vector
 * error while the power stage is off.
 *
 * 参数是每个 16 向量的重复次数，默认 64。每次只短暂屏蔽一个数学调用，输出
 * min/avg/max 原始周期和空调用开销；完整 ISR WCET 仍是实时性的最终证据。
 * The argument is repeats per 16-vector set, default 64. Interrupts are masked
 * for one math call at a time. Raw min/average/max cycles and empty-call
 * overhead are printed; full ISR WCET remains the final realtime evidence.
 *
 * 安全 / Safety: 只读诊断，不 arm；若 REALTIME_ARMED 已置位则拒绝，必须先
 * foc_stop。Production 完全不编译本命令和基准向量。
 * Read-only and never arms. It refuses while REALTIME_ARMED is set. Production
 * compiles out both this command and the benchmark vectors.
 */
static int foc_math_bench(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    foc_math_benchmark_result_t result = {0};
    uint32_t repeats = 64U;
    uint32_t overhead_average;
    uint32_t sin_cos_average;
    uint32_t atan2_average;
    uint32_t fixed_sin_cos_average;
    uint32_t fixed_atan2_average;
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
    uint32_t fast_sin_cos_average;
    uint32_t fast_atan2_average;
#endif
    uint32_t health_ok;

    if ((argc < 1) || (argc > 2))
    {
        rt_kprintf("FMATH,ERR,usage\n");
        return -1;
    }
    if (argc == 2)
    {
        if (foc_shell_parse_u32(argv[1], &repeats) == 0U)
        {
            rt_kprintf("FMATH,ERR,repeats\n");
            return -1;
        }
    }
    if ((repeats == 0U) || (repeats > FOC_MATH_BENCHMARK_MAX_REPEATS))
    {
        rt_kprintf("FMATH,ERR,repeats\n");
        return -1;
    }
    if (foc_platform_get_diagnostics(&diagnostics) != FOC_STATUS_OK)
    {
        rt_kprintf("FMATH,ERR,diag\n");
        return -1;
    }
    if ((diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) != 0U)
    {
        rt_kprintf("FMATH,ERR,armed\n");
        return -1;
    }
    if (foc_math_accel_backend() != FOC_MATH_BACKEND_CORDIC)
    {
        rt_kprintf("FMATH,ERR,backend\n");
        return -1;
    }
    if (foc_math_accel_benchmark(&result, repeats) == 0U)
    {
        rt_kprintf("FMATH,ERR,exec\n");
        return -1;
    }

    overhead_average = result.measurement_overhead.total_cycles /
                       result.measurement_overhead.calls;
    sin_cos_average = result.sin_cos.total_cycles / result.sin_cos.calls;
    atan2_average = result.atan2.total_cycles / result.atan2.calls;
    fixed_sin_cos_average = result.fixed_delay_sin_cos.total_cycles /
                            result.fixed_delay_sin_cos.calls;
    fixed_atan2_average = result.fixed_delay_atan2.total_cycles /
                          result.fixed_delay_atan2.calls;
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
    fast_sin_cos_average = result.fast_approx_sin_cos.total_cycles /
                           result.fast_approx_sin_cos.calls;
    fast_atan2_average = result.fast_approx_atan2.total_cycles /
                         result.fast_approx_atan2.calls;
#endif
    health_ok = ((result.sin_cos.failures == 0U) &&
                 (result.atan2.failures == 0U) &&
                 (result.fixed_delay_sin_cos.failures == 0U) &&
                 (result.fixed_delay_atan2.failures == 0U) &&
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
                 (result.fast_approx_available == 1U) &&
                 (result.fast_approx_sin_cos.failures == 0U) &&
                 (result.fast_approx_atan2.failures == 0U) &&
#endif
                 (result.invalid_rejections == result.invalid_tests)) ? 1U : 0U;

    /* 128 KiB Flash 不再重复保存人读长句；完整结果仍由稳定的
     * FMATH/FMathF/FMathC 机器记录输出。 */
    /* 保持单行低于 RT-Thread 控制台缓冲上限：轮询基线沿用 FMATH，固定延迟
     * 候选单独使用 FMATHF。解析器必须按记录前缀和 version 判断字段布局。
     * Keep each line below the RT-Thread console buffer limit. FMATH carries the
     * polling reference and FMATHF carries the fixed-delay candidate. */
    rt_kprintf("FMATH,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)result.version,
               (unsigned int)result.vector_count,
               (unsigned int)result.repeats,
               (unsigned int)result.measurement_overhead.minimum_cycles,
               (unsigned int)overhead_average,
               (unsigned int)result.measurement_overhead.maximum_cycles,
               (unsigned int)result.sin_cos.calls,
               (unsigned int)result.sin_cos.successes,
               (unsigned int)result.sin_cos.failures,
               (unsigned int)result.sin_cos.minimum_cycles,
               (unsigned int)sin_cos_average,
               (unsigned int)result.sin_cos.maximum_cycles,
               (unsigned int)result.maximum_sin_cos_error_ppb,
               (unsigned int)result.atan2.calls,
               (unsigned int)result.atan2.successes,
               (unsigned int)result.atan2.failures,
               (unsigned int)result.atan2.minimum_cycles,
               (unsigned int)atan2_average,
               (unsigned int)result.atan2.maximum_cycles,
               (unsigned int)result.maximum_atan2_error_urad,
               (unsigned int)result.invalid_rejections,
               (unsigned int)result.invalid_tests,
               (unsigned int)health_ok);
    rt_kprintf("FMATHF,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)result.version,
               (unsigned int)result.fixed_delay_nops,
               (unsigned int)result.realtime_uses_fixed_delay,
               (unsigned int)result.fixed_delay_sin_cos.calls,
               (unsigned int)result.fixed_delay_sin_cos.successes,
               (unsigned int)result.fixed_delay_sin_cos.failures,
               (unsigned int)result.fixed_delay_sin_cos.minimum_cycles,
               (unsigned int)fixed_sin_cos_average,
               (unsigned int)result.fixed_delay_sin_cos.maximum_cycles,
               (unsigned int)result.maximum_fixed_delay_sin_cos_error_ppb,
               (unsigned int)result.fixed_delay_atan2.calls,
               (unsigned int)result.fixed_delay_atan2.successes,
               (unsigned int)result.fixed_delay_atan2.failures,
               (unsigned int)result.fixed_delay_atan2.minimum_cycles,
               (unsigned int)fixed_atan2_average,
               (unsigned int)result.fixed_delay_atan2.maximum_cycles,
               (unsigned int)result.maximum_fixed_delay_atan2_error_urad,
               (unsigned int)health_ok);
#if defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
    rt_kprintf("FMATHC,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
               (unsigned int)result.version,
               (unsigned int)result.fast_approx_available,
               (unsigned int)result.realtime_uses_fast_approx,
               (unsigned int)result.fast_approx_sin_cos.calls,
               (unsigned int)result.fast_approx_sin_cos.successes,
               (unsigned int)result.fast_approx_sin_cos.failures,
               (unsigned int)result.fast_approx_sin_cos.minimum_cycles,
               (unsigned int)fast_sin_cos_average,
               (unsigned int)result.fast_approx_sin_cos.maximum_cycles,
               (unsigned int)result.maximum_fast_approx_sin_cos_error_ppb,
               (unsigned int)result.fast_approx_atan2.calls,
               (unsigned int)result.fast_approx_atan2.successes,
               (unsigned int)result.fast_approx_atan2.failures,
               (unsigned int)result.fast_approx_atan2.minimum_cycles,
               (unsigned int)fast_atan2_average,
               (unsigned int)result.fast_approx_atan2.maximum_cycles,
               (unsigned int)result.maximum_fast_approx_atan2_error_urad,
               (unsigned int)health_ok);
#endif
    return (health_ok != 0U) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_math_bench, -);
#endif

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && defined(FOC_MATH_DIAGNOSTIC_HEALTH)
/*
 * foc_math_health show | reset | inject sincos|atan2
 *
 * show 读取成功/回退/未就绪/恢复计数；reset 清空；inject 只把下一次指定 CORDIC
 * 运算标成未就绪。三者在功率级 arm 时一律拒绝，避免快环运行时关中断复制快照或
 * 改变诊断状态。
 * show reads success/fallback/not-ready/recovery counters, reset clears them,
 * and inject marks only the next selected CORDIC operation as not ready.
 * All three are refused while the power stage is armed.
 */
static int foc_math_health(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    foc_math_health_t health = {0};
    uint32_t operation;

    if ((argc < 2) || (argc > 3) ||
        (foc_platform_get_diagnostics(&diagnostics) != FOC_STATUS_OK))
    {
        rt_kprintf("FMATHH,ERR,usage\n");
        return -1;
    }
    if ((diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) != 0U)
    {
        rt_kprintf("FMATHH,ERR,armed\n");
        return -1;
    }
    if (strcmp(argv[1], "show") == 0)
    {
        if ((argc != 2) || (foc_math_accel_health_get(&health) == 0U))
        {
            rt_kprintf("FMATHH,ERR,snapshot\n");
            return -1;
        }
        /* 人读重复字段已裁剪；FMATHH 是唯一完整健康快照。 */
        rt_kprintf("FMATHH,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                   (unsigned int)health.version,
                   (unsigned int)health.sin_cos_calls,
                   (unsigned int)health.sin_cos_successes,
                   (unsigned int)health.sin_cos_fallback_required,
                   (unsigned int)health.sin_cos_not_ready,
                   (unsigned int)health.magnitude_calls,
                   (unsigned int)health.magnitude_successes,
                   (unsigned int)health.magnitude_fallback_required,
                   (unsigned int)health.atan2_calls,
                   (unsigned int)health.atan2_successes,
                   (unsigned int)health.atan2_fallback_required,
                   (unsigned int)health.atan2_not_ready,
                   (unsigned int)health.cordic_recoveries,
                   (unsigned int)health.injections_requested,
                   (unsigned int)health.injections_consumed,
                   (unsigned int)health.pending_injection_operation);
        return 0;
    }
    if ((strcmp(argv[1], "reset") == 0) && (argc == 2))
    {
        foc_math_accel_health_reset();
        rt_kprintf("FMATHH,reset\n");
        return 0;
    }
    if ((strcmp(argv[1], "inject") != 0) || (argc != 3))
    {
        rt_kprintf("FMATHH,ERR,usage\n");
        return -1;
    }
    if (strcmp(argv[2], "sincos") == 0)
    {
        operation = FOC_MATH_INJECT_OPERATION_SIN_COS;
    }
    else if (strcmp(argv[2], "atan2") == 0)
    {
        operation = FOC_MATH_INJECT_OPERATION_ATAN2;
    }
    else
    {
        rt_kprintf("FMATHH,ERR,operation\n");
        return -1;
    }
    if (foc_math_accel_inject_not_ready_once(operation) == 0U)
    {
        rt_kprintf("FMATHH,ERR,pending\n");
        return -1;
    }
    rt_kprintf("FMATHH,armed,%s\n", argv[2]);
    return 0;
}
MSH_CMD_EXPORT(foc_math_health, -);
#endif

/*
 * foc_phase_capture start [on|off]|stop|status|dump —— 12 kHz 原始码固定窗。
 * 该命令只在 Diagnostic/Calibration 存在，平台会在功率级已 arm 时拒绝 start。
 */
static const char *foc_phase_capture_state_name(uint32_t state)
{
    switch (state)
    {
    case FOC_PHASE_VOLTAGE_CAPTURE_IDLE: return "idle";
    case FOC_PHASE_VOLTAGE_CAPTURE_ARMED: return "armed";
    case FOC_PHASE_VOLTAGE_CAPTURE_COMPLETE: return "complete";
    default: return "invalid";
    }
}

static const char *foc_phase_capture_divider_name(uint32_t mode)
{
    return (mode == FOC_PHASE_VOLTAGE_DIVIDER_ENABLED) ? "on" : "off";
}

static const char *foc_phase_voltage_model_source_name(uint32_t source)
{
    switch (source)
    {
    case FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL:
        return "st-ihm16m1-nominal";
    case FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED:
        return "board-calibrated";
    default:
        return "none";
    }
}

static void foc_phase_capture_print_model(void)
{
    foc_phase_voltage_model_t model = {0};
    foc_status_t status = foc_platform_get_phase_voltage_model(&model);
    const char *observer_use;
    const char *quality;

    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FPV_MODEL,source=none,status=%u\n",
                   (unsigned int)status);
        return;
    }
    quality = ((model.flags &
                FOC_PHASE_VOLTAGE_MODEL_FLAG_BOARD_CALIBRATED) != 0U) ?
        "board-calibrated" : "nominal-not-calibrated";
    observer_use = ((model.flags &
                     FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE) != 0U) ?
        "eligible" : "disabled";
    rt_kprintf("FPV_MODEL,source=%s,quality=%s,observer=%s,vref_mv=%u,adc_max=%u,upper_ohm=%u,lower_ohm=%u,full_scale_mv=%u,uv_per_count=%u\n",
               foc_phase_voltage_model_source_name(model.source),
               quality,
               observer_use,
               (unsigned int)model.adc_reference_mv,
               (unsigned int)model.adc_max_code,
               (unsigned int)model.divider_upper_ohm,
               (unsigned int)model.divider_lower_ohm,
               (unsigned int)model.phase_full_scale_mv,
               (unsigned int)model.volts_per_count_uv);
}

static int foc_phase_capture(int argc, char **argv)
{
    foc_phase_voltage_capture_status_t capture_status = {0};
    foc_status_t status;

    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "status") == 0)))
    {
        status = foc_platform_phase_voltage_capture_status(&capture_status);
        if (status != FOC_STATUS_OK)
        {
            rt_kprintf("FPV unavailable=%u.\n",
                       (unsigned int)status);
            return -1;
        }
    rt_kprintf("FPV,%s,%s,%u,%u/%u,%u,%u\n",
                   foc_phase_capture_state_name(capture_status.state),
                   foc_phase_capture_divider_name(capture_status.divider_mode),
                   (unsigned int)capture_status.sample_rate_hz,
                   (unsigned int)capture_status.sample_count,
                   (unsigned int)capture_status.capacity,
                   (unsigned int)capture_status.unread_count,
                   (unsigned int)((capture_status.sample_rate_hz != 0U) ?
                       ((capture_status.sample_count * 1000000UL) /
                         capture_status.sample_rate_hz) : 0U));
        foc_phase_capture_print_model();
        return 0;
    }
    if (((argc == 2) || (argc == 3)) && (strcmp(argv[1], "start") == 0))
    {
        foc_phase_voltage_divider_mode_t divider_mode =
            FOC_PHASE_VOLTAGE_DIVIDER_ENABLED;

        if (argc == 3)
        {
            if (strcmp(argv[2], "off") == 0)
            {
                divider_mode = FOC_PHASE_VOLTAGE_DIVIDER_DISABLED;
            }
            else if (strcmp(argv[2], "on") != 0)
            {
                rt_kprintf("FPV refused: divider must be on|off.\n");
                return -1;
            }
        }
        status = foc_platform_phase_voltage_capture_start(divider_mode);
        if (status != FOC_STATUS_OK)
        {
            rt_kprintf("FPV start refused=%u.\n",
                       (unsigned int)status);
            return -1;
        }
        rt_kprintf("FPV armed,%s,256,12000,off\n",
                   foc_phase_capture_divider_name(divider_mode));
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "stop") == 0))
    {
        foc_platform_phase_voltage_capture_stop();
        rt_kprintf("FPV stopped.\n");
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "dump") == 0))
    {
        foc_phase_voltage_sample_t sample;

        status = foc_platform_phase_voltage_capture_status(&capture_status);
        if ((status != FOC_STATUS_OK) ||
            (capture_status.state == FOC_PHASE_VOLTAGE_CAPTURE_ARMED))
        {
            rt_kprintf("FPV dump refused.\n");
            return -1;
        }
        rt_kprintf("FPV_META,divider=%s\n",
                   foc_phase_capture_divider_name(capture_status.divider_mode));
        foc_phase_capture_print_model();
        rt_kprintf("FPV_HEADER,sequence,phase_u_raw,phase_v_raw,phase_w_raw,current_u_raw,current_v_raw,bus_last_raw\n");
        while (foc_platform_phase_voltage_capture_pop(&sample) != 0U)
        {
            rt_kprintf("FPV,%u,%u,%u,%u,%u,%u,%u\n",
                       (unsigned int)sample.sequence,
                       (unsigned int)sample.phase_u_raw,
                       (unsigned int)sample.phase_v_raw,
                       (unsigned int)sample.phase_w_raw,
                       (unsigned int)sample.current_u_raw,
                       (unsigned int)sample.current_v_raw,
                       (unsigned int)sample.bus_voltage_raw);
        }
        return 0;
    }
    rt_kprintf("FPV usage\n");
    return -1;
}
MSH_CMD_EXPORT(foc_phase_capture, -);

#if defined(FLUXRT_TRACE_BUILD)
/*
 * foc_trace start [Hz] | stop | status —— Diagnostic 档的降采样波形输出。
 * foc_trace start [Hz] | stop | status - decimated waveform output, Diagnostic only.
 *
 * 参数 / Parameters:
 *   请求采样率 [Hz]，允许 10..50；省略时用 50。实际速率是 PWM 频率整除分频器
 *   的结果，通常不等于请求值，因此两者都会打印。
 *   The requested rate in Hz, 10..50, defaulting to 50. The effective rate is
 *   the PWM frequency divided by an integer divider and usually differs from the
 *   request, so both are printed.
 *
 * 代价 / Cost:
 *   trace 分频命中的拍需要完成整数缩放和 72 B 样本拷贝；增加观察器诊断后必须
 *   重新实测 WCET。Production 档仍整体裁掉 trace，未开启时只有一次 enable 判断。
 *   A trace-sampled tick performs integer scaling and copies a 72-byte sample;
 *   WCET must be re-measured after the observer extension. Production still
 *   compiles the mechanism out, and a disabled Diagnostic trace only checks enable.
 *
 * 返回 / Returns: 0 已启动或已打印状态；-1 参数错误、PWM 频率不可用或平台拒绝。
 * Returns 0 when started or when status was printed; -1 on bad arguments, an
 * unavailable PWM frequency, or a platform refusal.
 *
 * 上下文 / Context: Shell 线程发起，样本由 ISR 生产，主循环消费并打印。
 * Started from the shell thread; samples are produced by the ISR and consumed and
 * printed by the main loop.
 */
static int foc_trace(int argc, char **argv)
{
    foc_platform_diagnostics_t diagnostics = {0};
    /* 默认 50 Hz：在 12 kHz 控制拍下对应分频 240，64 槽环形缓冲约 1.28 s 填满；
     * 既能看出电流波形，又不至于持续丢样本。
     * Default 50 Hz: that is a divider of 240 at 12 kHz, so the 64-slot ring buffer holds
     * about 1.28 s of data, enough to see the current waveform without dropping samples
     * continuously. */
    uint32_t sample_hz = 50U;
    uint32_t sample_divider;
    foc_status_t status;

    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "status") == 0)))
    {
        (void)foc_platform_get_diagnostics(&diagnostics);
        rt_kprintf("FTRACE,%u,%u,%u\n",
                   (unsigned int)foc_platform_trace_is_enabled(),
                   (unsigned int)g_foc_trace_rate_hz,
                   (unsigned int)foc_platform_trace_dropped());
        return 0;
    }
    if ((argc == 2) && (strcmp(argv[1], "stop") == 0))
    {
        foc_platform_trace_stop();
        g_foc_trace_rate_hz = 0U;
        rt_kprintf("FTRACE,stop,%u\n",
                   (unsigned int)foc_platform_trace_dropped());
        return 0;
    }
    if ((argc < 2) || (argc > 3) || (strcmp(argv[1], "start") != 0))
    {
        rt_kprintf("FTRACE usage\n");
        return -1;
    }
    if (argc == 3)
    {
        if (foc_shell_parse_u32(argv[2], &sample_hz) == 0U)
        {
            rt_kprintf("FTRACE rate\n");
            return -1;
        }
    }
    if ((sample_hz < 10U) || (sample_hz > 50U))
    {
        rt_kprintf("FTRACE rate\n");
        return -1;
    }
    (void)foc_platform_get_diagnostics(&diagnostics);
    /* 扩展到 31 列的文本 trace 在 115200 baud 下按 100 Hz 输出会丢行；50 Hz 仍能
     * 观察 25 ms 接管和 50 ms 失锁窗口，同时给 Shell 留出带宽。
     * The expanded 31-column text trace drops lines at 100 Hz on 115200 baud.
     * 50 Hz still resolves the 25 ms handover and 50 ms loss window while leaving
     * serial headroom for the shell. */
    if (diagnostics.control_frequency_hz == 0U)
    {
        rt_kprintf("FTRACE nofreq\n");
        return -1;
    }
    /* 分频器 [控制拍/样本] 由控制频率 [Hz] 整除得到，因此实际采样率会有取整
     * 误差；平台侧还有自己的最小分频约束，是否可接受由
     * foc_platform_trace_start() 决定，本命令只负责如实上报。
     * The divider (control ticks per sample) comes from an integer division of the
     * control rate, so the effective rate carries a truncation error. The
     * platform enforces its own minimum divider; foc_platform_trace_start()
     * decides whether the value is acceptable and this command only reports it. */
    sample_divider = diagnostics.control_frequency_hz / sample_hz;
    status = foc_platform_trace_start(sample_divider);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FTRACE,refused,%u,%u\n",
                   (unsigned int)status,
                   (unsigned int)sample_divider);
        return -1;
    }
    /* 记录实际速率（控制频率 / 分频器），而不是请求值：状态输出必须反映真实
     * 采样率，否则离线分析会按错误的时间轴对齐数据。
     * Store the effective rate (control rate divided by the divider), not the
     * request: the status output has to reflect the real sampling rate, otherwise
     * offline analysis aligns data on a wrong time axis. */
    g_foc_trace_rate_hz = diagnostics.control_frequency_hz / sample_divider;
    /* FTR rows keep their 31-column V2 schema; print only the version and width.
     * Repeating every field name cost about 0.35 KiB in the 128 KiB Diagnostic
     * image.  The canonical ordered names and units live in
     * simulation/capture_hardware_trace.py, which accepts both this compact marker
     * and archived expanded headers. */
    rt_kprintf("FTR_HEADER,2,31\n");
    rt_kprintf("FTRACE,start,%u,%u,%u\n",
               (unsigned int)sample_hz,
               (unsigned int)(diagnostics.control_frequency_hz / sample_divider),
               (unsigned int)sample_divider);
    return 0;
}
MSH_CMD_EXPORT(foc_trace, -);
#endif

#if defined(FLUXRT_RUNTIME_TUNING_BUILD)
/*
 * 打印当前生效的运行时配置与平台安全窗口。
 * Prints the active runtime configuration and the platform safety window.
 *
 * 单位 / Units: 转速 [rpm]，电流 [mA]，时间 [ms]，占空比与比率 [per-mille]，
 * 电压 [mV]，截止 [cycles]。为便于一行读完，标量都用整数打印，因此代码里要
 * 乘 1000.0f 再取整，这会截断小数部分。
 * Speed in rpm, currents in mA, times in ms, duty and ratios in per-mille,
 * voltages in mV and the deadline in cycles. Scalars are printed as integers so a
 * whole group fits on one line, hence the multiply by 1000.0f, which truncates
 * the fractional part.
 *
 * 注意 / Caveat:
 *   util 一行打印的是 voltage_utilization [per-mille]，不是百分比，也不是伏特。
 *   The util field is voltage_utilization in per-mille, not a percentage and not
 *   a voltage.
 */
static void foc_print_config(void)
{
    rt_kprintf("C0,%s,%u/%u,%u,%d/%d/%d,%d/%d/%d,%d\n",
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.observer_enable,
               (unsigned int)g_foc_runtime_config.observer_update_divider,
               (unsigned int)g_foc_runtime_config.closed_loop_enable,
               (int)g_foc_runtime_config.startup_final_speed_rpm,
               (int)(g_foc_runtime_config.startup_alignment_current_a * 1000.0f),
               (int)(g_foc_runtime_config.startup_current_a * 1000.0f),
               (int)(g_foc_runtime_config.alignment_duration_s * 1000.0f),
               (int)(g_foc_runtime_config.open_loop_ramp_duration_s * 1000.0f),
               (int)(g_foc_runtime_config.observer_transition_duration_s * 1000.0f),
               (int)(g_foc_runtime_config.voltage_utilization * 1000.0f));
    rt_kprintf("CL,%d/%d,%d,%d/%d,%u\n",
               (int)(g_foc_platform_config.minimum_bus_voltage_v * 1000.0f),
               (int)(g_foc_platform_config.maximum_bus_voltage_v * 1000.0f),
               (int)(g_foc_platform_config.software_current_trip_a * 1000.0f),
               (int)(g_foc_platform_config.minimum_duty * 1000.0f),
               (int)(g_foc_platform_config.maximum_duty * 1000.0f),
               (unsigned int)g_foc_platform_config.isr_deadline_cycles);
    /* CH 是板级不可放宽边界；CL 是本次实际生效值。两行同时打印，
     * 让串口审计能直接发现“请求值越过硬上限”，而不需要回查头文件。
     * CH is the non-relaxable board envelope and CL is the effective runtime
     * configuration. Printing both makes a relaxed request visible in a serial audit. */
    rt_kprintf("CH,%d/%d,%d,%d/%d,%u\n",
               (int)(FOC_PLATFORM_HARD_MIN_BUS_VOLTAGE_V * 1000.0f),
               (int)(FOC_PLATFORM_HARD_MAX_BUS_VOLTAGE_V * 1000.0f),
               (int)(FOC_PLATFORM_HARD_MAX_CURRENT_TRIP_A * 1000.0f),
               (int)(FOC_PLATFORM_HARD_MIN_DUTY * 1000.0f),
               (int)(FOC_PLATFORM_HARD_MAX_DUTY * 1000.0f),
               (unsigned int)FOC_PLATFORM_HARD_MAX_ISR_DEADLINE_CYCLES);
    rt_kprintf("CO,%d/%d/%d,%d/%d/%d\n",
               (int)(g_foc_runtime_config.observer_smo_k_slide_v * 1000.0f),
               (int)(g_foc_runtime_config.observer_smo_boundary_a * 1000.0f),
               (int)(g_foc_runtime_config.observer_emf_filter_alpha * 1000.0f),
               (int)g_foc_runtime_config.observer_pll_kp,
               (int)(g_foc_runtime_config.observer_acquisition_pll_kp_ratio * 1000.0f),
               (int)g_foc_runtime_config.observer_pll_ki);
    rt_kprintf("CG,%d/%d,%d/%d,%d,%d,%u,%d/%d,%d,%d/%d,%d\n",
               (int)g_foc_runtime_config.observer_minimum_speed_rpm,
               (int)g_foc_runtime_config.observer_run_reliability.minimum_speed_rpm,
               (int)(g_foc_runtime_config.observer_acquisition_maximum_phase_error_rad * 1000.0f),
               (int)(g_foc_runtime_config.observer_run_reliability.maximum_phase_error_rad * 1000.0f),
               (int)(g_foc_runtime_config.observer_minimum_bemf_v * 1000.0f),
               (int)(g_foc_runtime_config.observer_speed_variance_ratio * 1000.0f),
               (unsigned int)g_foc_runtime_config.observer_consecutive_samples,
               (int)(g_foc_runtime_config.observer_acquisition_timeout_s * 1000.0f),
               (int)(g_foc_runtime_config.observer_loss_timeout_s * 1000.0f),
               (int)g_foc_runtime_config.closed_loop_speed_ramp_rpm_per_s,
               (int)(g_foc_runtime_config.handoff_torque_support_ratio * 1000.0f),
               (int)(g_foc_runtime_config.speed_pi_preload_ratio * 1000.0f),
               (int)(g_foc_runtime_config.closed_loop_current_slew_a_per_s * 1000.0f));
    rt_kprintf("CA,%d/%d\n",
               (int)(g_foc_runtime_config.angle_compensation.park_prediction_ticks * 1000.0f),
               (int)(g_foc_runtime_config.angle_compensation.reverse_park_prediction_ticks * 1000.0f));
    rt_kprintf("CI,%u/%u/%u,%u,%d,%d,%d,%d,%d\n",
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.enabled,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.observer_voltage_correction_enable,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.pwm_feedforward_enable,
               (unsigned int)g_foc_runtime_config.inverter_voltage_model.pwm_carrier_frequency_hz,
               (int)(g_foc_runtime_config.inverter_voltage_model.dead_time_s * 1000000000.0f),
               (int)(g_foc_runtime_config.inverter_voltage_model.compensation_gain * 1000.0f),
               (int)(g_foc_runtime_config.inverter_voltage_model.current_zero_band_a * 1000.0f),
               (int)(g_foc_runtime_config.inverter_voltage_model.current_sign_filter_alpha * 1000.0f),
               (int)(g_foc_runtime_config.inverter_voltage_model.device_drop_v * 1000.0f));
}

/*
 * foc_cfg show | <field> <value> —— Diagnostic 档的在线调参。
 * foc_cfg show | <field> <value> - online tuning, Diagnostic profile only.
 *
 * 参数 / Parameters: 见 usage 字符串。<mA>/<mV>/<permille>/<ms> 这类单位由本命令
 * 换算成浮点基础单位后再提交，Shell 侧一律用整数输入，避免为调参依赖 strtof 的
 * 小数解析行为。
 * See the usage string. Units such as mA, mV, per-mille and ms are converted to
 * base float units here; the shell takes integers only so tuning never depends on
 * strtof's fractional parsing.
 *
 * 返回 / Returns:
 *   0   已提交（或已打印），配置与平台侧保持一致；
 *   -1  用法错误、参数越界、**运行中拒绝修改**，或平台/Rust 侧拒绝了候选配置。
 *   0 when committed (or printed) with the platform kept consistent; -1 on a
 *   usage error, an out-of-range value, a refusal because the loop is running, or
 *   a rejected candidate configuration on the platform or Rust side.
 *
 * 安全语义 / Safety semantics:
 *   本命令绝不 arm 功率级。检测到 FOC_PLATFORM_DIAG_REALTIME_ARMED 时直接拒绝，
 *   必须先 foc_stop，避免运行中改变电流环/观测器参数。
 *   This command never arms the power stage. It refuses outright while
 *   FOC_PLATFORM_DIAG_REALTIME_ARMED is set, so foc_stop is required first; the
 *   current-loop and observer parameters must not change while running.
 *
 * 提交流程 / Commit protocol:
 *   先在 tshell 单写的静态候选结构体上改一个逻辑设置（invstage 会原子改三个开关），
 *   再交给 foc_rust_configure() 校验；只有返回 OK 才整体写回
 *   g_foc_runtime_config。校验失败时旧配置原样保留，不做部分提交。
 *   One logical setting is modified in a tshell-single-writer static candidate
 *   (invstage atomically changes three gates), which is then validated by
 *   foc_rust_configure();
 *   g_foc_runtime_config is replaced as a whole only on OK.
 *   On failure the previous configuration stays untouched; there is no partial
 *   commit.
 */
static int foc_cfg(int argc, char **argv)
{
    foc_runtime_config_t *candidate = &g_foc_runtime_candidate;
    foc_status_t status;
    uint32_t parsed_u32;

    if ((argc == 1) || ((argc == 2) && (strcmp(argv[1], "show") == 0)))
    {
        foc_print_config();
        return 0;
    }
    if (argc != 3)
    {
        rt_kprintf("foc_cfg show|field value\n");
        return -1;
    }
    /* 运行中拒绝改参：电流环/观测器参数在 arm 期间被 ISR 读取，改动会同时破坏
     * 本拍计算的一致性和 WCET 证据，因此必须先 foc_stop。
     * Refuse while running: the ISR reads the current-loop and observer parameters,
     * so changing them mid-arm breaks both the consistency of the tick being
     * computed and the WCET evidence. foc_stop is required first. */
    status = foc_platform_get_diagnostics(&g_foc_cfg_diagnostics);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FCFG,diag-unavailable\n");
        return -1;
    }
    if ((g_foc_cfg_diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) != 0U)
    {
        rt_kprintf("FCFG,stop-first\n");
        return -1;
    }
    *candidate = g_foc_runtime_config;

    if (strcmp(argv[1], "observer") == 0)
    {
        if (strcmp(argv[2], "smo") == 0) candidate->observer_backend = FOC_OBSERVER_SMO_PLL;
        else if (strcmp(argv[2], "bemf") == 0) candidate->observer_backend = FOC_OBSERVER_BEMF_PLL;
        else if (strcmp(argv[2], "st") == 0)
        {
            rt_kprintf("FCFG,observer-st-reserved\n");
            return -1;
        }
        else return -1;
    }
    else if (strcmp(argv[1], "closedloop") == 0)
    {
        if ((foc_shell_parse_u32(argv[2], &parsed_u32) == 0U) ||
            (parsed_u32 > 1U)) return -1;
        candidate->closed_loop_enable = parsed_u32;
    }
    else if (strcmp(argv[1], "observe") == 0)
    {
        if ((foc_shell_parse_u32(argv[2], &parsed_u32) == 0U) ||
            (parsed_u32 > 1U)) return -1;
        candidate->observer_enable = parsed_u32;
    }
    else if (strcmp(argv[1], "invstage") == 0)
    {
        if ((foc_shell_parse_u32(argv[2], &parsed_u32) == 0U) ||
            (parsed_u32 > 4U)) return -1;
        candidate->inverter_voltage_model.enabled =
            (parsed_u32 != 0U) ? 1U : 0U;
        candidate->inverter_voltage_model.observer_voltage_correction_enable =
            ((parsed_u32 == 2U) || (parsed_u32 == 4U)) ? 1U : 0U;
        candidate->inverter_voltage_model.pwm_feedforward_enable =
            ((parsed_u32 == 3U) || (parsed_u32 == 4U)) ? 1U : 0U;
    }
    else if (strcmp(argv[1], "obsdiv") == 0)
    {
        if (foc_shell_parse_u32(
                argv[2], &candidate->observer_update_divider) == 0U) return -1;
    }
    else if (strcmp(argv[1], "startup") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U, &candidate->startup_final_speed_rpm) == 0U) return -1;
    }
    else if (strcmp(argv[1], "current") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U, &candidate->startup_current_a) == 0U) return -1;
    }
    else if (strcmp(argv[1], "aligncurrent") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->startup_alignment_current_a) == 0U) return -1;
    }
    else if (strcmp(argv[1], "align") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U, &candidate->alignment_duration_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "ramp") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->open_loop_ramp_duration_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "util") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U, &candidate->voltage_utilization) == 0U) return -1;
    }
    else if (strcmp(argv[1], "slide") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U, &candidate->observer_smo_k_slide_v) == 0U) return -1;
    }
    else if (strcmp(argv[1], "boundary") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U, &candidate->observer_smo_boundary_a) == 0U) return -1;
    }
    else if (strcmp(argv[1], "filter") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_emf_filter_alpha) == 0U) return -1;
    }
    else if (strcmp(argv[1], "pll_kp") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U, &candidate->observer_pll_kp) == 0U) return -1;
    }
    else if (strcmp(argv[1], "pll_acq") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_acquisition_pll_kp_ratio) == 0U) return -1;
    }
    else if (strcmp(argv[1], "pll_ki") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U, &candidate->observer_pll_ki) == 0U) return -1;
    }
    else if (strcmp(argv[1], "minspeed") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U,
                &candidate->observer_minimum_speed_rpm) == 0U) return -1;
    }
    else if (strcmp(argv[1], "runminspeed") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U,
                &candidate->observer_run_reliability.minimum_speed_rpm) == 0U) return -1;
    }
    else if (strcmp(argv[1], "runphase") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_run_reliability.maximum_phase_error_rad) == 0U) return -1;
    }
    else if (strcmp(argv[1], "acqphase") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_acquisition_maximum_phase_error_rad) == 0U) return -1;
    }
    else if (strcmp(argv[1], "parkdelay") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->angle_compensation.park_prediction_ticks) == 0U) return -1;
    }
    else if (strcmp(argv[1], "revparkdelay") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->angle_compensation.reverse_park_prediction_ticks) == 0U) return -1;
    }
    else if (strcmp(argv[1], "minbemf") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_minimum_bemf_v) == 0U) return -1;
    }
    else if (strcmp(argv[1], "variance") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_speed_variance_ratio) == 0U) return -1;
    }
    else if (strcmp(argv[1], "confirm") == 0)
    {
        if (foc_shell_parse_u32(
                argv[2], &candidate->observer_consecutive_samples) == 0U) return -1;
    }
    else if (strcmp(argv[1], "acquire") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_acquisition_timeout_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "transition") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_transition_duration_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "loss") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->observer_loss_timeout_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "accel") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1U,
                &candidate->closed_loop_speed_ramp_rpm_per_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "preload") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->speed_pi_preload_ratio) == 0U) return -1;
    }
    else if (strcmp(argv[1], "support") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->handoff_torque_support_ratio) == 0U) return -1;
    }
    else if (strcmp(argv[1], "islew") == 0)
    {
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &candidate->closed_loop_current_slew_a_per_s) == 0U) return -1;
    }
    else if (strcmp(argv[1], "trip") == 0)
    {
        /* 过流跳闸是平台层安全窗口的一部分，不属于 Rust 运行时配置，因此走
         * foc_platform_configure() 单独校验，并在成功后同步本地副本。
         * The over-current trip belongs to the platform safety window rather than
         * the Rust runtime configuration, so it is validated separately through
         * foc_platform_configure() and the local copy is synced on success. */
        foc_platform_config_t platform_candidate = g_foc_platform_config;
        if (foc_shell_parse_i32_scaled(
                argv[2], 1000U,
                &platform_candidate.software_current_trip_a) == 0U) return -1;
        status = foc_platform_configure(&platform_candidate);
        if (status != FOC_STATUS_OK)
        {
            rt_kprintf("FCFG,platform,%u\n", (unsigned int)status);
            return -1;
        }
        g_foc_platform_config = platform_candidate;
        foc_print_config();
        return 0;
    }
    else
    {
        return -1;
    }

    status = foc_rust_configure(&g_foc_controller, candidate);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FCFG,rust,%u\n", (unsigned int)status);
        return -1;
    }
    g_foc_runtime_config = *candidate;
    foc_print_config();
    return 0;
}
MSH_CMD_EXPORT(foc_cfg, -);
#endif
#endif

#if defined(FLUXRT_ADVANCED_CANDIDATE_BUILD)
/* foc_advanced_wcet [ticks] [mode]
 * mode: 0=basic, 1=MTPA, 2=FW, 3=MTPV, 4=decoupling, 5=DPWM,
 *       6=overmodulation, 7=all target-supported features.
 *
 * The command never arms the power stage.  Capability bits for modes 5/6/7 are
 * scoped to this synthetic no-power transaction and are removed by finish(). */
static int foc_advanced_wcet(int argc, char **argv)
{
    foc_advanced_runtime_config_t probe_config;
    foc_advanced_probe_input_t probe_input;
    foc_advanced_probe_status_t probe_status;
    foc_advanced_telemetry_t telemetry;
    foc_realtime_timing_stats_t timing;
    foc_platform_diagnostics_t diagnostics;
    uint32_t requested_ticks = 36000U;
    uint32_t mode = 7U;
    uint32_t features = 0U;
    uint32_t capabilities = 0U;
    uint32_t started_ms;
    uint32_t timeout_ms;
    uint32_t saved_observer_update_divider;
    foc_status_t status;
    foc_status_t finish_status;
    foc_status_t restore_status;
    int result = -1;

    if ((argc > 3) ||
        ((argc >= 2) &&
         (foc_shell_parse_u32(argv[1], &requested_ticks) == 0U)) ||
        ((argc == 3) &&
         (foc_shell_parse_u32(argv[2], &mode) == 0U)) ||
        (requested_ticks < FOC_ADVANCED_PROBE_MIN_TICKS) ||
        (requested_ticks > FOC_ADVANCED_PROBE_MAX_TICKS) ||
        (mode > 7U))
    {
        return -1;
    }
    switch (mode)
    {
    case 0U:
        break;
    case 1U:
        features = FOC_ADVANCED_FEATURE_MTPA;
        break;
    case 2U:
        features = FOC_ADVANCED_FEATURE_FIELD_WEAKENING;
        break;
    case 3U:
        features = FOC_ADVANCED_FEATURE_MTPV;
        break;
    case 4U:
        features = FOC_ADVANCED_FEATURE_DECOUPLING;
        break;
    case 5U:
        features = FOC_ADVANCED_FEATURE_DPWM;
        capabilities = FOC_ADVANCED_CAP_DPWM_CURRENT_RECONSTRUCTION;
        break;
    case 6U:
        features = FOC_ADVANCED_FEATURE_OVERMODULATION;
        capabilities = FOC_ADVANCED_CAP_OVERMOD_MIN_PULSE;
        break;
    default:
        features = FOC_ADVANCED_FEATURE_MTPA |
                   FOC_ADVANCED_FEATURE_FIELD_WEAKENING |
                   FOC_ADVANCED_FEATURE_MTPV |
                   FOC_ADVANCED_FEATURE_DECOUPLING |
                   FOC_ADVANCED_FEATURE_DPWM |
                   FOC_ADVANCED_FEATURE_OVERMODULATION;
        capabilities = FOC_ADVANCED_CAP_DPWM_CURRENT_RECONSTRUCTION |
                       FOC_ADVANCED_CAP_OVERMOD_MIN_PULSE;
        break;
    }

    status = foc_rust_default_advanced_config(&g_foc_controller,
                                              &probe_config);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FADV,default,%u\n", (unsigned int)status);
        return -1;
    }
    probe_config.algorithm.enabled_features = features;
    /* Keep the production 1 kHz operating-region slot from the default
     * configuration.  Forcing this divider to one would execute the bounded
     * MTPA/MTPV search on every 12 kHz current-loop tick, which is neither the
     * deployed schedule nor a valid end-to-end WCET model. */
    /* A 24-segment MTPV sweep does not fit the G431's 83.3 us ISR even in its
     * isolated region slot.  Eight is the validated configuration lower bound;
     * the generic library remains tunable up to 64 for faster targets. */
    probe_config.algorithm.mtpv_search_steps = 8U;
    probe_config.platform_capabilities = capabilities;
    probe_config.minimum_duty = g_foc_platform_config.minimum_duty;
    probe_config.maximum_duty = g_foc_platform_config.maximum_duty;

    (void)memset(&probe_input, 0, sizeof(probe_input));
    probe_input.struct_size = sizeof(probe_input);
    probe_input.version = FOC_ADVANCED_PROBE_INPUT_VERSION;
    probe_input.base_id_reference_a = 0.0f;
    probe_input.base_iq_reference_a = 0.6f;
    probe_input.electrical_angle_rad = 0.35f;
    probe_input.mechanical_speed_rad_s = 150.0f;
    probe_input.previous_vd_command_v = 0.0f;
    /* 13 V nominal bus gives a 7.505 V linear SVPWM radius.  Keep this
     * deliberately above it so modes 6/7 actually exercise overmodulation on
     * both PC and target instead of depending on a mismatched 12.3 V fixture. */
    probe_input.previous_vq_command_v = 7.6f;
    probe_input.phase_current_a = 0.10f;
    probe_input.phase_current_b = -0.04f;
    probe_input.phase_current_c = -0.06f;

    g_foc_management_log_inhibit = 1U;
    /* G431 Advanced Lab uses the existing alternating observer slot: SMO/PLL
     * runs at 6 kHz while the current ISR remains at 12 kHz.  This is a target
     * scheduling choice, not a generic Rust algorithm default. */
    saved_observer_update_divider =
        g_foc_runtime_config.observer_update_divider;
    g_foc_runtime_config.observer_update_divider = 2U;
    status = foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
    if (status != FOC_STATUS_OK)
    {
        g_foc_runtime_config.observer_update_divider =
            saved_observer_update_divider;
        (void)foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
        rt_kprintf("FADV,schedule,%u\n", (unsigned int)status);
        g_foc_management_log_inhibit = 0U;
        return -1;
    }
    started_ms = (uint32_t)rt_tick_get_millisecond();
    timeout_ms = (requested_ticks / 12U) + 2000U;
    status = foc_platform_advanced_candidate_probe_start(
        &probe_config,
        &probe_input,
        requested_ticks,
        g_foc_runtime_config.nominal_bus_voltage_v,
        g_foc_runtime_config.default_target_speed_rpm);
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FADV,start,%u\n", (unsigned int)status);
        (void)foc_platform_advanced_candidate_probe_finish();
        g_foc_runtime_config.observer_update_divider =
            saved_observer_update_divider;
        (void)foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
        g_foc_management_log_inhibit = 0U;
        return -1;
    }

    do
    {
        rt_thread_mdelay(1);
        status = foc_platform_advanced_candidate_probe_get_status(
            &probe_status,
            &telemetry);
        if ((status != FOC_STATUS_OK) ||
            (probe_status.state == FOC_ADVANCED_PROBE_COMPLETE) ||
            (probe_status.state == FOC_ADVANCED_PROBE_FAILED))
        {
            break;
        }
    } while ((uint32_t)((uint32_t)rt_tick_get_millisecond() -
                        started_ms) < timeout_ms);

    (void)foc_platform_get_timing(&timing);
    (void)foc_platform_get_diagnostics(&diagnostics);
    rt_kprintf("FADV,m=%u,f=%02x,s=%u,r=%u,n=%u/%u,sig=%08x/%u,"
               "a=%02x,st=%08x,rg=%u,mod=%u,wcet=%u,ctrl=%u,miss=%u\n",
               (unsigned int)mode,
               (unsigned int)features,
               (unsigned int)probe_status.state,
               (unsigned int)probe_status.last_result,
               (unsigned int)probe_status.executed_ticks,
               (unsigned int)probe_status.requested_ticks,
               (unsigned int)probe_status.decision_signature,
               (unsigned int)probe_status.decision_samples,
               (unsigned int)telemetry.active_features,
               (unsigned int)telemetry.status_flags,
               (unsigned int)telemetry.region,
               (unsigned int)telemetry.modulation_mode,
               (unsigned int)timing.wcet.total_cycles,
               (unsigned int)timing.peak_control_cycles,
               (unsigned int)diagnostics.deadline_miss_count);

    result = ((status == FOC_STATUS_OK) &&
              (probe_status.state == FOC_ADVANCED_PROBE_COMPLETE) &&
              (probe_status.executed_ticks == requested_ticks) &&
              (probe_status.decision_samples == requested_ticks) &&
              (probe_status.rejected_tick_count == 0U) &&
              (diagnostics.deadline_miss_count == 0U) &&
              ((features == 0U) ||
               ((telemetry.status_flags &
                 (FOC_ADVANCED_STATUS_CONFIGURED |
                  FOC_ADVANCED_STATUS_BASIC_FALLBACK |
                  FOC_ADVANCED_STATUS_FAULTED)) ==
                FOC_ADVANCED_STATUS_CONFIGURED))) ? 0 : -1;
    finish_status = foc_platform_advanced_candidate_probe_finish();
    g_foc_runtime_config.observer_update_divider =
        saved_observer_update_divider;
    restore_status = foc_rust_configure(&g_foc_controller,
                                        &g_foc_runtime_config);
    if ((finish_status == FOC_STATUS_OK) &&
        (restore_status != FOC_STATUS_OK))
    {
        finish_status = restore_status;
    }
    rt_kprintf("FADV,end,%u,%u\n",
               (unsigned int)((result == 0) ? 0U : 1U),
               (unsigned int)finish_status);
    g_foc_management_log_inhibit = 0U;
    return ((finish_status == FOC_STATUS_OK) && (result == 0)) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_advanced_wcet, -);

/* foc_arm_rearm_test
 *
 * No-power S4 proof for the exact register path used by pre-arm Break2
 * recovery. The platform refuses this command once Vbus reaches the motor
 * operating window and keeps Gate/MOE/CH1..3 closed throughout. */
static int foc_arm_rearm_test(int argc, char **argv)
{
    uint16_t before_flags = 0U;
    uint16_t rearm_facts = 0U;
    uint16_t after_flags = 0U;
    foc_status_t status;

    (void)argv;
    if (argc != 1)
    {
        rt_kprintf("foc_arm_rearm_test\n");
        return -1;
    }
    status = foc_platform_advanced_candidate_break2_rearm_test(
        &before_flags, &rearm_facts, &after_flags);
    rt_kprintf("FARMR,%u,%04x,%04x,%04x\n",
               (unsigned int)status,
               (unsigned int)before_flags,
               (unsigned int)rearm_facts,
               (unsigned int)after_flags);
    return (status == FOC_STATUS_OK) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_arm_rearm_test, -);

/* foc_advanced_trial P54-BASIC-100MS
 *
 * The first P5.4 powered gate exposes one exact token and only the feature=0
 * baseline.  There is deliberately no caller-controlled speed, current,
 * feature mask or duration.  The decoupling mode exists behind the platform
 * contract but receives a Shell token only after this baseline is accepted. */
static int foc_advanced_trial(int argc, char **argv)
{
    foc_advanced_power_trial_status_t trial;
    foc_advanced_telemetry_t advanced;
    foc_advanced_power_trial_snapshot_t snapshot;
    foc_realtime_timing_stats_t timing;
    foc_platform_diagnostics_t diagnostics;
    foc_status_t status;
    foc_status_t finish_status;
    uint32_t started_ms;
    int result = -1;

    if ((argc != 2) || (strcmp(argv[1], "P54-BASIC-100MS") != 0))
    {
        rt_kprintf("FADVP,token\n");
        return -1;
    }
    (void)memset(&trial, 0, sizeof(trial));
    (void)memset(&advanced, 0, sizeof(advanced));
    (void)memset(&snapshot, 0, sizeof(snapshot));
    g_foc_management_log_inhibit = 1U;
    status = foc_platform_advanced_candidate_power_trial_start(
        FOC_ADVANCED_POWER_TRIAL_MODE_BASIC,
        &g_foc_runtime_config);
    if (status != FOC_STATUS_OK)
    {
        (void)foc_platform_advanced_candidate_power_trial_get_status(
            &trial, &advanced, &snapshot);
        finish_status =
            foc_platform_advanced_candidate_power_trial_finish(
                &g_foc_runtime_config);
        rt_kprintf("FADVP,start=%u,state=%u,result=%u,finish=%u,"
                   "snap=%u,ctrlstate=%u,orel=%u,closed=%u,ogates=%02x,"
                   "phase=%d,speed=%d,lossus=%u\n",
                   (unsigned int)status,
                   (unsigned int)trial.state,
                   (unsigned int)trial.result,
                   (unsigned int)finish_status,
                   (unsigned int)((snapshot.struct_size == sizeof(snapshot)) &&
                                  (snapshot.version ==
                                   FOC_ADVANCED_POWER_TRIAL_SNAPSHOT_VERSION)),
                   (unsigned int)snapshot.state,
                   (unsigned int)snapshot.observer_reliable,
                   (unsigned int)snapshot.closed_loop_active,
                   (unsigned int)snapshot.observer_reliability_flags,
                   (int)(snapshot.observer_pll_phase_error_rad * 1000.0f),
                   (int)snapshot.observer_speed_mean_rpm,
                   (unsigned int)(snapshot.observer_loss_elapsed_s * 1000000.0f));
        g_foc_management_log_inhibit = 0U;
        return -1;
    }

    started_ms = (uint32_t)rt_tick_get_millisecond();
    do
    {
        rt_thread_mdelay(1);
        status = foc_platform_advanced_candidate_power_trial_get_status(
            &trial, &advanced, &snapshot);
        if ((status != FOC_STATUS_OK) ||
            (trial.state == FOC_ADVANCED_POWER_TRIAL_COMPLETE) ||
            (trial.state == FOC_ADVANCED_POWER_TRIAL_FAILED) ||
            (trial.state == FOC_ADVANCED_POWER_TRIAL_ABORTED))
        {
            break;
        }
    } while ((uint32_t)((uint32_t)rt_tick_get_millisecond() - started_ms) <
             16000U);

    (void)foc_platform_get_timing(&timing);
    (void)foc_platform_get_diagnostics(&diagnostics);
    finish_status = foc_platform_advanced_candidate_power_trial_finish(
        &g_foc_runtime_config);
    rt_kprintf("FADVP,state=%u,result=%u,ticks=%u/%u,first=%u,"
               "epoch=%u/%u,miss=%u/%u,features=%02x,status=%08x,"
               "snap=%u,ctrlstate=%u,orel=%u,closed=%u,ogates=%02x,"
               "phase=%d,speed=%d,lossus=%u,"
               "wcet=%u,ctrl=%u,diagmiss=%u,finish=%u\n",
               (unsigned int)trial.state,
               (unsigned int)trial.result,
               (unsigned int)trial.active_ticks,
               (unsigned int)trial.total_ticks,
               (unsigned int)trial.first_active_tick,
               (unsigned int)trial.expected_fault_epoch,
               (unsigned int)trial.observed_fault_epoch,
               (unsigned int)trial.initial_deadline_miss_count,
               (unsigned int)trial.observed_deadline_miss_count,
               (unsigned int)trial.last_active_features,
               (unsigned int)trial.last_advanced_status_flags,
               (unsigned int)((snapshot.struct_size == sizeof(snapshot)) &&
                              (snapshot.version ==
                               FOC_ADVANCED_POWER_TRIAL_SNAPSHOT_VERSION)),
               (unsigned int)snapshot.state,
               (unsigned int)snapshot.observer_reliable,
               (unsigned int)snapshot.closed_loop_active,
               (unsigned int)snapshot.observer_reliability_flags,
               (int)(snapshot.observer_pll_phase_error_rad * 1000.0f),
               (int)snapshot.observer_speed_mean_rpm,
               (unsigned int)(snapshot.observer_loss_elapsed_s * 1000000.0f),
               (unsigned int)timing.wcet.total_cycles,
               (unsigned int)timing.peak_control_cycles,
               (unsigned int)diagnostics.deadline_miss_count,
               (unsigned int)finish_status);
    result = ((status == FOC_STATUS_OK) &&
              (trial.state == FOC_ADVANCED_POWER_TRIAL_COMPLETE) &&
              (trial.result == FOC_ADVANCED_POWER_TRIAL_RESULT_OK) &&
              (trial.active_ticks ==
               FOC_ADVANCED_POWER_TRIAL_ACTIVE_TICKS) &&
              (trial.last_active_features == 0U) &&
              (trial.last_advanced_status_flags == 0U) &&
              (diagnostics.deadline_miss_count == 0U) &&
              (finish_status == FOC_STATUS_OK)) ? 0 : -1;
    g_foc_management_log_inhibit = 0U;
    return result;
}
MSH_CMD_EXPORT(foc_advanced_trial, -);
#endif

#if defined(FLUXRT_DIAGNOSTIC_BUILD) && \
    defined(FOC_MOTION_CONTROL_CANDIDATE)
static void foc_motion_probe_build_bundle(uint32_t revision)
{
    foc_config_bundle_t *bundle = &g_foc_motion_probe_bundle;
    const uint32_t board_id = g_foc_production_profile.board_id;
    const uint32_t motor_id = g_foc_production_profile.motor_id;

    (void)memset(bundle, 0, sizeof(*bundle));
    bundle->struct_size = sizeof(*bundle);
    bundle->abi_version = FOC_CONFIG_ABI_VERSION;
    bundle->bundle_revision = revision;
    bundle->generated_by_version = 1U;

    bundle->board.version = FOC_CONFIG_GROUP_VERSION;
    bundle->board.board_id = board_id;
    bundle->board.pwm_frequency_hz =
        g_foc_runtime_config.inverter_voltage_model.pwm_carrier_frequency_hz;
    bundle->board.control_frequency_hz =
        g_foc_runtime_config.pwm_frequency_hz;
    bundle->board.capability_flags =
        FOC_CONFIG_BOARD_CAP_THREE_SHUNT_CURRENT;
    bundle->board.adc_reference_v = 3.3f;
    bundle->board.current_gain_a_per_count = 0.001f;
    bundle->board.bus_voltage_v_per_count = 0.012890625f;

    bundle->motor.version = FOC_CONFIG_GROUP_VERSION;
    bundle->motor.motor_id = motor_id;
    bundle->motor.pole_pairs = g_foc_runtime_config.pole_pairs;
    bundle->motor.phase_resistance_ohm =
        g_foc_runtime_config.stator_resistance_ohm;
    bundle->motor.d_inductance_h =
        g_foc_runtime_config.stator_inductance_h;
    bundle->motor.q_inductance_h =
        g_foc_runtime_config.stator_inductance_h;
    bundle->motor.flux_linkage_v_s =
        g_foc_runtime_config.flux_linkage_wb;
    bundle->motor.continuous_current_a =
        g_foc_runtime_config.rated_current_a;
    bundle->motor.maximum_speed_rad_s =
        g_foc_runtime_config.max_speed_rpm * 0.104719755f;

    bundle->inverter.version = FOC_CONFIG_GROUP_VERSION;
    bundle->inverter.inverter_id = 1U;
    bundle->inverter.maximum_phase_current_a =
        g_foc_platform_config.software_current_trip_a;
    bundle->inverter.minimum_bus_voltage_v =
        g_foc_platform_config.minimum_bus_voltage_v;
    bundle->inverter.maximum_bus_voltage_v =
        g_foc_platform_config.maximum_bus_voltage_v;
    bundle->inverter.dead_time_s =
        g_foc_runtime_config.inverter_voltage_model.dead_time_s;
    bundle->inverter.transistor_drop_v =
        g_foc_runtime_config.inverter_voltage_model.device_drop_v;
    bundle->inverter.brake_resistance_ohm = 0.0f;
    bundle->inverter.maximum_duty = g_foc_platform_config.maximum_duty;

    bundle->axis.version = FOC_CONFIG_GROUP_VERSION;
    bundle->axis.axis_id = 0U;
    bundle->axis.feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    bundle->axis.direction = 1;
    bundle->axis.current_kp = g_foc_runtime_config.id_pi.kp;
    bundle->axis.current_ki = g_foc_runtime_config.id_pi.ki;
    bundle->axis.velocity_kp = 0.02f;
    bundle->axis.velocity_ki = 0.2f;
    bundle->axis.position_kp = 1.0f;
    bundle->axis.soft_limit_min_rad = -100.0f;
    bundle->axis.soft_limit_max_rad = 100.0f;
    bundle->axis.motion_control_enabled = 1U;
    bundle->axis.torque_ramp_rate_nm_s = 1.0f;
    bundle->axis.velocity_ramp_rate_rad_s2 = 20.0f;
    bundle->axis.position_filter_bandwidth_rad_s = 40.0f;
    bundle->axis.trajectory_acceleration_rad_s2 = 30.0f;
    bundle->axis.trajectory_deceleration_rad_s2 = 35.0f;

    bundle->app.version = FOC_CONFIG_GROUP_VERSION;
    bundle->app.can_node_id = 1U;
    bundle->app.uart_baud = 115200U;
    bundle->app.command_source_count = 1U;
    bundle->app.command_sources[0].source_id = 1U;
    bundle->app.command_sources[0].priority = 10U;
    bundle->app.command_sources[0].permissions =
        FOC_CONFIG_COMMAND_PERMISSION_SETPOINT;
    bundle->app.command_sources[0].lease_ms = 1000U;
    bundle->app.command_sources[0].command_timeout_ms = 15000U;

    bundle->external_io.struct_size = sizeof(bundle->external_io);
    bundle->external_io.version = FOC_EXTERNAL_IO_CONFIG_VERSION;
    bundle->external_io.revision = 1U;
    bundle->external_io.simple_inputs.struct_size =
        sizeof(bundle->external_io.simple_inputs);
    bundle->external_io.simple_inputs.version =
        FOC_SIMPLE_INPUT_CONFIG_VERSION;

    bundle->calibration.version = FOC_CONFIG_GROUP_VERSION;
    bundle->calibration.board_id = board_id;
    bundle->calibration.motor_id = motor_id;
}

static void foc_motion_probe_build_command(uint32_t now_ms,
                                           uint32_t valid_for_ms)
{
    foc_product_command_t *command = &g_foc_motion_probe_command;

    (void)memset(command, 0, sizeof(*command));
    command->struct_size = sizeof(*command);
    command->version = FOC_PRODUCT_COMMAND_VERSION;
    command->axis_id = 0U;
    command->source_id = 1U;
    command->sequence = 1U;
    command->created_at_ms = now_ms;
    command->valid_until_ms = now_ms + valid_for_ms;
    command->command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    command->control_mode = FOC_CONTROL_MODE_VELOCITY;
    command->input_mode = FOC_INPUT_MODE_VELOCITY_RAMP;
    command->feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    command->velocity_ref_rad_s =
        g_foc_runtime_config.startup_final_speed_rpm * 0.104719755f;
    command->flags = FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT;
    command->current_limit_a = g_foc_runtime_config.rated_current_a;
}

static void foc_motion_torque_trial_build_command(uint32_t now_ms)
{
    foc_product_command_t *command = &g_foc_motion_probe_command;

    (void)memset(command, 0, sizeof(*command));
    command->struct_size = sizeof(*command);
    command->version = FOC_PRODUCT_COMMAND_VERSION;
    command->axis_id = 0U;
    command->source_id = 1U;
    command->sequence = 1U;
    command->created_at_ms = now_ms;
    command->valid_until_ms =
        now_ms + FOC_MOTION_TORQUE_TRIAL_COMMAND_WINDOW_MS;
    command->command_kind = FOC_PRODUCT_COMMAND_SETPOINT;
    command->control_mode = FOC_CONTROL_MODE_TORQUE;
    command->input_mode = FOC_INPUT_MODE_TORQUE_RAMP;
    command->feedback_mode = FOC_FEEDBACK_MODE_SENSORLESS;
    command->flags = FOC_PRODUCT_COMMAND_FLAG_CURRENT_LIMIT |
                     FOC_PRODUCT_COMMAND_FLAG_TORQUE_LIMIT;
    command->torque_ref_nm = FOC_MOTION_TORQUE_TRIAL_TORQUE_NM;
    command->current_limit_a = FOC_MOTION_TORQUE_TRIAL_CURRENT_LIMIT_A;
    command->torque_limit_nm = FOC_MOTION_TORQUE_TRIAL_TORQUE_LIMIT_NM;
}

/* foc_motion_wcet [ticks] [mode]: mode 0 measures the same combined/platform
 * path with normal Alignment semantics; mode 1 forces the motion reference
 * path.  Their difference estimates isolated motion cost.  The 26,000-tick
 * default deliberately crosses the 24,000-tick Alignment -> OpenLoopRamp
 * boundary at 12 kHz.  Both keep Gate/MOE/CCER proven off and never call
 * foc_platform_control_start(). */
static int foc_motion_wcet(int argc, char **argv)
{
    uint32_t requested_ticks = 26000U;
    uint32_t execute_motion = 0U;
    uint32_t now_ms;
    uint32_t timeout_ms;
    uint32_t revision = g_foc_profile_report.profile_revision;
    foc_status_t status;

    if ((argc > 3) ||
        ((argc == 2) &&
         (foc_shell_parse_u32(argv[1], &requested_ticks) == 0U)) ||
        ((argc == 3) &&
         ((foc_shell_parse_u32(argv[1], &requested_ticks) == 0U) ||
          (foc_shell_parse_u32(argv[2], &execute_motion) == 0U) ||
          (execute_motion > 1U))) ||
        (requested_ticks < FOC_MOTION_PROBE_MIN_TICKS) ||
        (requested_ticks > FOC_MOTION_PROBE_MAX_TICKS))
    {
        return -1;
    }
    if (foc_platform_motion_candidate_is_enabled() != 0U)
    {
        rt_kprintf("FMWCET,busy\n");
        return -1;
    }
    if (revision == 0U)
    {
        revision = 1U;
    }
    foc_motion_probe_build_bundle(revision);
    now_ms = (uint32_t)rt_tick_get_millisecond();
    timeout_ms = (requested_ticks / 12U) + 2000U;
    foc_motion_probe_build_command(now_ms, timeout_ms + 1000U);

    status = foc_platform_motion_candidate_configure(
        &g_foc_motion_probe_bundle,
        &g_foc_runtime_config,
        revision,
        12U);
    if (status == FOC_STATUS_OK)
    {
        status = foc_platform_motion_candidate_publish(
            &g_foc_motion_probe_command, 0U, 0.0f, 0U);
    }
    if (status == FOC_STATUS_OK)
    {
        status = foc_platform_motion_candidate_probe_start(
            requested_ticks,
            g_foc_runtime_config.startup_final_speed_rpm,
            execute_motion);
    }
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FMWCET,start,%u\n", (unsigned int)status);
        (void)foc_platform_motion_candidate_probe_finish();
        if (foc_platform_motion_candidate_is_enabled() != 0U)
        {
            (void)foc_platform_motion_candidate_shutdown();
        }
        return -1;
    }

    do
    {
        rt_thread_mdelay(1);
        status = foc_platform_motion_candidate_probe_get_status(
            &g_foc_motion_probe_status);
        if (status != FOC_STATUS_OK)
        {
            break;
        }
        if ((g_foc_motion_probe_status.state ==
             FOC_MOTION_PROBE_COMPLETE) ||
            (g_foc_motion_probe_status.state ==
             FOC_MOTION_PROBE_FAILED))
        {
            break;
        }
    } while ((uint32_t)((uint32_t)rt_tick_get_millisecond() - now_ms) <
             timeout_ms);

    (void)foc_platform_get_timing(&g_foc_motion_probe_timing);
    (void)foc_platform_get_diagnostics(&g_foc_motion_probe_diagnostics);
    rt_kprintf("FMWCET,m=%u,s=%u,r=%u,n=%u/%u,c=%u,inj=%u,"
               "wcet=%u,ctrl=%u,miss=%u\n",
               (unsigned int)execute_motion,
               (unsigned int)g_foc_motion_probe_status.state,
               (unsigned int)g_foc_motion_probe_status.last_result,
               (unsigned int)g_foc_motion_probe_status.executed_ticks,
               (unsigned int)g_foc_motion_probe_status.requested_ticks,
               (unsigned int)g_foc_motion_probe_status.no_power_commit_count,
               (unsigned int)g_foc_motion_probe_status.injection_count,
               (unsigned int)g_foc_motion_probe_timing.wcet.total_cycles,
               (unsigned int)g_foc_motion_probe_timing.peak_control_cycles,
               (unsigned int)g_foc_motion_probe_diagnostics.deadline_miss_count);
    (void)foc_platform_motion_candidate_probe_finish();
    (void)foc_platform_motion_candidate_shutdown();
    return ((g_foc_motion_probe_status.state == FOC_MOTION_PROBE_COMPLETE) &&
            (g_foc_motion_probe_diagnostics.deadline_miss_count == 0U)) ?
        0 : -1;
}
MSH_CMD_EXPORT(foc_motion_wcet, -);

/* foc_motion_torque_trial TORQUE1: one fixed P4.2E2 powered transaction.
 * Sensorless startup keeps the previously proven 0.8 A ceiling; only after the
 * observer reaches ClosedLoop does the Torque command become active. The ADC ISR
 * cuts Gate/MOE/CCER on the 1,200th accepted Torque tick (100 ms at 12 kHz), so
 * Shell scheduling cannot extend the active window. */
static int foc_motion_torque_trial(int argc, char **argv)
{
    uint32_t now_ms;
    uint32_t revision = g_foc_profile_report.profile_revision;
    uint32_t wait_started_ms;
    uint32_t candidate_configured = 0U;
    foc_status_t status;
    foc_status_t restore_status = FOC_STATUS_NOT_CONFIGURED;
    int result = -1;

    if ((argc != 2) || (strcmp(argv[1], "TORQUE1") != 0))
    {
        rt_kprintf("FMT,confirm\n");
        return -1;
    }
    if (foc_platform_motion_candidate_is_enabled() != 0U)
    {
        rt_kprintf("FMT,busy\n");
        return -1;
    }
    g_foc_motion_trial_runtime_saved = g_foc_runtime_config;
    g_foc_runtime_config.closed_loop_enable = 1U;
    g_foc_runtime_config.inverter_voltage_model.enabled = 1U;
    g_foc_runtime_config.inverter_voltage_model.
        observer_voltage_correction_enable = 1U;
    g_foc_runtime_config.inverter_voltage_model.pwm_feedforward_enable = 0U;
    status = foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
    if (status != FOC_STATUS_OK)
    {
        g_foc_runtime_config = g_foc_motion_trial_runtime_saved;
        rt_kprintf("FMT,config,%u\n", (unsigned int)status);
        return -1;
    }
    if (revision == 0U)
    {
        revision = 1U;
    }
    foc_motion_probe_build_bundle(revision);
    now_ms = (uint32_t)rt_tick_get_millisecond();
    foc_motion_torque_trial_build_command(now_ms);
    status = foc_platform_motion_candidate_configure(
        &g_foc_motion_probe_bundle,
        &g_foc_runtime_config,
        revision,
        12U);
    if (status == FOC_STATUS_OK)
    {
        candidate_configured = 1U;
        status = foc_platform_motion_candidate_torque_trial_start(
            &g_foc_motion_probe_command,
            &g_foc_runtime_config,
            g_foc_runtime_config.startup_final_speed_rpm);
    }
    if (status != FOC_STATUS_OK)
    {
        rt_kprintf("FMT,start,%u\n", (unsigned int)status);
        goto cleanup;
    }

    wait_started_ms = (uint32_t)rt_tick_get_millisecond();
    do
    {
        rt_thread_mdelay(1);
        status = foc_platform_motion_candidate_torque_trial_get_status(
            &g_foc_motion_trial_status);
        if (status != FOC_STATUS_OK)
        {
            break;
        }
        if ((g_foc_motion_trial_status.state ==
             FOC_MOTION_TORQUE_TRIAL_COMPLETE) ||
            (g_foc_motion_trial_status.state ==
             FOC_MOTION_TORQUE_TRIAL_FAILED) ||
            (g_foc_motion_trial_status.state ==
             FOC_MOTION_TORQUE_TRIAL_ABORTED))
        {
            break;
        }
    } while ((uint32_t)((uint32_t)rt_tick_get_millisecond() -
                        wait_started_ms) < 16000U);

    (void)foc_platform_get_timing(&g_foc_motion_probe_timing);
    (void)foc_platform_get_diagnostics(&g_foc_motion_probe_diagnostics);
    rt_kprintf("FMT,s=%u,r=%u,n=%u/%u,first=%u,peak=%u,"
               "wcet=%u,miss=%u,f=%08x\n",
               (unsigned int)g_foc_motion_trial_status.state,
               (unsigned int)g_foc_motion_trial_status.result,
               (unsigned int)g_foc_motion_trial_status.active_ticks,
               (unsigned int)g_foc_motion_trial_status.total_ticks,
               (unsigned int)g_foc_motion_trial_status.first_active_tick,
               (unsigned int)g_foc_motion_probe_diagnostics.
                   peak_current_delta_counts,
               (unsigned int)g_foc_motion_probe_timing.wcet.total_cycles,
               (unsigned int)g_foc_motion_probe_diagnostics.
                   deadline_miss_count,
               (unsigned int)g_foc_motion_probe_diagnostics.
                   control_fault_flags);
    result = ((status == FOC_STATUS_OK) &&
              (g_foc_motion_trial_status.state ==
               FOC_MOTION_TORQUE_TRIAL_COMPLETE) &&
              (g_foc_motion_trial_status.active_ticks ==
               FOC_MOTION_TORQUE_TRIAL_ACTIVE_TICKS) &&
              (g_foc_motion_probe_diagnostics.deadline_miss_count == 0U) &&
              (g_foc_motion_probe_diagnostics.control_fault_flags == 0U)) ?
        0 : -1;

cleanup:
    foc_platform_control_stop();
    if (candidate_configured != 0U)
    {
        /* Auto-stop and faults may have revoked the route before Shell wakes.
         * control_stop() has already returned Rust ownership to task context. */
        (void)foc_platform_motion_candidate_shutdown();
    }
    restore_status = foc_rust_configure(
        &g_foc_controller, &g_foc_motion_trial_runtime_saved);
    if (restore_status == FOC_STATUS_OK)
    {
        g_foc_runtime_config = g_foc_motion_trial_runtime_saved;
    }
    rt_kprintf("FMT,end,%u,%u\n",
               (unsigned int)((result == 0) ? 0U : 1U),
               (unsigned int)restore_status);
    return result;
}
MSH_CMD_EXPORT(foc_motion_torque_trial, -);

/* foc_motion_inject <1..6>: arm one deterministic E1B fault point for the next
 * configure/run.  It never starts the route and never touches power outputs. */
static int foc_motion_inject(int argc, char **argv)
{
    uint32_t point;
    foc_status_t status;

    if ((argc != 2) ||
        (foc_shell_parse_u32(argv[1], &point) == 0U) ||
        (point < FOC_MOTION_PROBE_INJECT_CONFIGURE_BEFORE_ROUTE) ||
        (point > FOC_MOTION_PROBE_INJECT_AFTER_COMMIT))
    {
        return -1;
    }
    status = foc_platform_motion_candidate_probe_inject_once(
        (foc_motion_probe_injection_point_t)point);
    rt_kprintf("FMINJ,%u,%u\n",
               (unsigned int)point,
               (unsigned int)status);
    return (status == FOC_STATUS_OK) ? 0 : -1;
}
MSH_CMD_EXPORT(foc_motion_inject, -);
#endif

#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
static void foc_power_management_force_safe_callback(void *context)
{
    (void)context;
    foc_platform_emergency_stop();
}
#endif

/*
 * RT-Thread 入口：完成一次性初始化，然后进入心跳循环。
 * RT-Thread entry point: one-time initialisation followed by the heartbeat loop.
 *
 * 启动顺序 / Boot order（顺序不可交换）:
 *   1. 先 emergency_stop()：在任何初始化之前就把功率级置为安全态，这样后面
 *      任何一步失败都不会留下已 arm 的驱动；
 *      emergency_stop() first, before any initialisation, so no later failure can
 *      leave an armed power stage behind;
 *   2. C/Rust ABI 自检（见下）；失败则死循环挂起，绝不继续跑；
 *   3. foc_rust_init() -> 默认配置 -> 覆盖本项目选择 -> foc_rust_configure()；
 *      foc_rust_init(), then the default configuration, then the choices this
 *      project overrides, then foc_rust_configure();
 *   4. 平台 configure/init/bind，然后延迟 100 ms 让 ADC 标定与同步采样起效；
 *      platform configure/init/bind, then a 100 ms delay so ADC calibration and
 *      synchronous sampling take effect;
 *   5. 进入 10 ms 心跳循环，本函数永不返回。
 *      enter the 10 ms heartbeat loop; this function never returns.
 *
 * ABI 自检为什么存在 / Why the ABI self-check exists:
 *   C 侧只按大小和对齐持有一块不透明缓冲（g_foc_controller），Rust 侧按自己的
 *   编译期结构解释它。若两边的结构布局或函数集不一致，两边**都能正常链接**，
 *   却会在运行时按错位的字段读写。启动时比对 ABI 版本、所需上下文大小和对齐，
 *   可以让这种组合直接拒绝运行（PWM 保持关闭并挂起），而不是带着错位的结构体
 *   继续执行到电机上。
 *   The C side owns an opaque buffer sized and aligned by contract, while the Rust
 *   side interprets it with its own compile-time layout. If the two disagree, both
 *   still link but read and write misaligned fields at runtime. Comparing the ABI
 *   version, the required context size and the required alignment at boot makes
 *   such a combination refuse to run (PWM stays disabled) instead of executing
 *   with silently misaligned structs.
 *
 * 失败语义 / Failure semantics:
 *   自检失败 -> 打印 FATAL 并 1 s 周期空转，**不**尝试退化运行；
 *   后续 init/config 失败 -> 只打印状态码，功率级仍保持关闭，由操作者用
 *   foc_status 判读；主循环不会因此自动 arm。
 *   A failed self-check prints FATAL and idles on a 1 s period; it never attempts
 *   a degraded run. Later init/config failures only print status codes while the
 *   power stage stays disabled, to be diagnosed with foc_status; the main loop
 *   never arms anything on its own.
 *
 * 上下文 / Context: 调度器启动后的第一个线程，非实时路径，允许 mdelay。
 * The first thread after scheduler start; not a real-time path, so mdelay is fine.
 */
int main(void)
{
    foc_status_t rust_status;
    foc_status_t platform_config_status;
    foc_status_t platform_status;
    foc_status_t bind_status;
    foc_status_t feedback_status;
    foc_feedback_t feedback = {0};
    foc_platform_diagnostics_t diagnostics = {0};
    uint32_t heartbeat_ms;
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
    foc_external_io_management_result_t external_io_status =
        FOC_EXTERNAL_IO_MANAGEMENT_RUST_ABI_FAILED;
    foc_external_input_port_result_t external_input_port_status =
        FOC_EXTERNAL_INPUT_PORT_DISABLED;
    foc_external_input_owner_result_t external_input_owner_status =
        FOC_EXTERNAL_INPUT_OWNER_INVALID_ARGUMENT;
    foc_external_io_capabilities_t external_io_capabilities;
#endif
#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
    foc_power_management_result_t power_management_status =
        FOC_POWER_MANAGEMENT_ABI_ERROR;
    const foc_power_management_ops_t power_management_ops =
    {
        sizeof(foc_power_management_ops_t),
        FOC_POWER_MANAGEMENT_VERSION,
        foc_power_management_force_safe_callback,
        0,
    };
#endif

    /* 在任何初始化之前先把功率级置为安全态。这是"失败不留 arm 状态"这条约束的
     * 起点：后面每一步失败时，功率级都还没有被 arm 过。
     * Put the power stage into a safe state before any initialisation: this is the
     * starting point of the "no failure leaves an armed stage" rule, because no
     * later step has armed anything yet. */
    foc_platform_emergency_stop();
    /* ABI 自检 / ABI self-check：桥接版本、产品契约版本、Rust 要求的上下文大小与对齐。
     * 任一项不满足都说明 C 与 Rust 的编译配置不匹配，此时宁可挂起也不运行。
     * The bridge version, product-contract version, Rust context size and context
     * alignment must all agree. Any mismatch means the C and Rust build settings
     * disagree, and idling is strictly better than running. */
    if ((foc_rust_abi_version() != FOC_RUST_ABI_VERSION) ||
        (foc_rust_product_contract_version() != FOC_PRODUCT_CONTRACT_VERSION) ||
        (foc_rust_context_required_size() > sizeof(g_foc_controller)) ||
        (foc_rust_context_required_align() > _Alignof(foc_rust_context_t))
#if defined(FLUXRT_ADVANCED_CANDIDATE_BUILD)
        || (foc_rust_advanced_abi_version() != FOC_ADVANCED_ABI_VERSION)
#endif
#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
        || (foc_rust_power_abi_version() != FOC_POWER_ABI_VERSION)
#endif
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
        || (foc_rust_command_abi_version() != FOC_COMMAND_ABI_VERSION)
        || (foc_rust_command_context_required_size() >
            FOC_COMMAND_CONTEXT_CAPACITY)
        || (foc_rust_command_context_required_align() >
            _Alignof(foc_command_context_storage_t))
#endif
#if defined(FLUXRT_PROTOCOL_NATIVE)
        || (foc_rust_native_protocol_version() != FOC_NATIVE_ABI_VERSION)
#endif
       )
    {
        rt_kprintf("FATAL ABI; PWM off\n");
        while (1) rt_thread_mdelay(1000);
    }

    /* init 与默认配置成链：前一步失败就跳过后面的步骤，只把状态码留到启动打印里，
     * 不做部分初始化。
     * init and default configuration are chained: a failure skips the remaining
     * steps and leaves only the status code in the boot report, so there is no
     * partial initialisation. */
    rust_status = foc_rust_init(&g_foc_controller);
    if (rust_status == FOC_STATUS_OK)
    {
        rust_status = foc_rust_default_st_config(&g_foc_runtime_config);
    }
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
    /* The framework candidate must initialize in an all-disabled state. Any
     * ABI/composition failure blocks controller binding instead of silently
     * running a partially requested product image. */
    if (rust_status == FOC_STATUS_OK)
    {
        foc_external_io_platform_capabilities(&external_io_capabilities);
        external_io_status =
            foc_external_io_management_init_with_capabilities(
                &g_foc_external_io_management,
                &external_io_capabilities);
        if (external_io_status != FOC_EXTERNAL_IO_MANAGEMENT_OK)
        {
            rust_status = FOC_STATUS_INVALID_ARGUMENT;
        }
    }
#endif
    if (rust_status == FOC_STATUS_OK)
    {
        /* 应用编译期只读的板卡/电机参数档。档案会以规范化 CRC 绑定完整
         * runtime config，并且只有“参数已审批 + 闭环已审批”两个位同时合法
         * 时才可把 closed_loop_enable 设为 1。当前 revision 1 的 approvals=0，
         * 所以 Diagnostic/Production 启动都保持开环；Production 也没有 Shell 旁路。 */
        g_foc_profile_status = foc_production_profile_apply(
            &g_foc_production_profile,
            &g_foc_runtime_config,
            &g_foc_profile_report);
        if (FOC_PRODUCTION_PROFILE_STATUS_IS_VALID(g_foc_profile_status))
        {
            rust_status = foc_rust_configure(&g_foc_controller, &g_foc_runtime_config);
        }
        else
        {
            rust_status = FOC_STATUS_INVALID_ARGUMENT;
        }
    }
#if defined(FLUXRT_POWER_CANDIDATE_BUILD)
    /* Retain the P5.1 ABI in the target candidate and commit only its generated
     * disabled policy.  No runtime step is scheduled until calibrated sources
     * and a separately approved brake/sink capability are available. */
    if (rust_status == FOC_STATUS_OK)
    {
        power_management_status = foc_power_management_init(
            &g_foc_power_management, &power_management_ops);
        if ((power_management_status != FOC_POWER_MANAGEMENT_OK) ||
            (g_foc_power_management.active_config.enabled != 0U) ||
            (g_foc_power_management.active_config.regeneration_allowed != 0U) ||
            (g_foc_power_management.active_config.brake_resistor_available != 0U))
        {
            rust_status = FOC_STATUS_INVALID_ARGUMENT;
        }
    }
#endif
#if defined(FLUXRT_ADVANCED_CANDIDATE_BUILD)
    /* Retain and exercise the target-side advanced ABI without activating any
     * feature.  The generated default must be feature-free/capability-free;
     * otherwise boot fails closed before the platform can bind the controller. */
    if (rust_status == FOC_STATUS_OK)
    {
        rust_status = foc_rust_default_advanced_config(
            &g_foc_controller, &g_foc_advanced_config);
    }
    if (rust_status == FOC_STATUS_OK)
    {
        /* Bind the hardware-independent advanced policy to this board's
         * validated PWM/minimum-pulse window before any feature can be enabled. */
        g_foc_advanced_config.minimum_duty =
            g_foc_platform_config.minimum_duty;
        g_foc_advanced_config.maximum_duty =
            g_foc_platform_config.maximum_duty;
    }
    if ((rust_status == FOC_STATUS_OK) &&
        ((g_foc_advanced_config.struct_size != sizeof(g_foc_advanced_config)) ||
         (g_foc_advanced_config.abi_version != FOC_ADVANCED_ABI_VERSION) ||
         (g_foc_advanced_config.platform_capabilities != 0U) ||
         (g_foc_advanced_config.algorithm.enabled_features != 0U)))
    {
        rust_status = FOC_STATUS_INVALID_ARGUMENT;
    }
    if (rust_status == FOC_STATUS_OK)
    {
        rust_status = foc_rust_configure_advanced(
            &g_foc_controller, &g_foc_advanced_config);
    }
    if (rust_status == FOC_STATUS_OK)
    {
        rust_status = foc_rust_get_advanced_telemetry(
            &g_foc_controller, &g_foc_advanced_telemetry);
    }
    if ((rust_status == FOC_STATUS_OK) &&
        ((g_foc_advanced_telemetry.struct_size != sizeof(g_foc_advanced_telemetry)) ||
         (g_foc_advanced_telemetry.abi_version != FOC_ADVANCED_ABI_VERSION) ||
         (g_foc_advanced_telemetry.active_features != 0U)))
    {
        rust_status = FOC_STATUS_INVALID_ARGUMENT;
    }
#endif
    /* 平台侧三个调用各自独立记录状态：configure 建立安全窗口，init 配置 TIM1/ADC/
     * 栅极与标定，bind 把平台输出接到 Rust 控制器。任一失败都会让 foc_start 走
     * 拒绝路径，而不会 arm。
     * The three platform calls record their status independently: configure
     * establishes the safety window, init sets up TIM1/ADC/gates and calibration,
     * and bind connects the platform output to the Rust controller. Any failure
     * makes foc_start refuse rather than arm. */
    platform_config_status = foc_platform_configure(&g_foc_platform_config);
    if (platform_config_status == FOC_STATUS_OK)
    {
        platform_status = foc_platform_init();
    }
    else
    {
        foc_platform_emergency_stop();
        platform_status = FOC_STATUS_NOT_CONFIGURED;
    }
    if (platform_status == FOC_STATUS_OK)
    {
        (void)foc_platform_get_diagnostics(&diagnostics);
        /* Rust 历史字段 pwm_frequency_hz 表示完整控制调用频率，不是多速率载波。
         * 两侧不一致时必须在绑定控制器之前关断并拒绝启动。 */
        if ((g_foc_runtime_config.pwm_frequency_hz != diagnostics.control_frequency_hz) ||
            (g_foc_runtime_config.inverter_voltage_model.pwm_carrier_frequency_hz !=
             diagnostics.pwm_frequency_hz))
        {
            foc_platform_emergency_stop();
            platform_status = FOC_STATUS_INVALID_ARGUMENT;
        }
    }
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
    /* Bind the raw-capture provider only after ADC/timers exist. Binding is
     * passive: every input remains stopped until a later approved management
     * transaction explicitly starts it. */
    if ((platform_status == FOC_STATUS_OK) &&
        (rust_status == FOC_STATUS_OK))
    {
        external_input_port_status =
            foc_external_input_platform_bind(&g_foc_external_input_port);
        if (external_input_port_status == FOC_EXTERNAL_INPUT_PORT_OK)
        {
            external_input_owner_status = foc_external_input_owner_init(
                &g_foc_external_input_owner,
                &g_foc_external_io_management,
                &g_foc_external_input_port);
        }
        if ((external_input_port_status != FOC_EXTERNAL_INPUT_PORT_OK) ||
            (external_input_owner_status != FOC_EXTERNAL_INPUT_OWNER_OK))
        {
            rust_status = FOC_STATUS_INVALID_ARGUMENT;
        }
    }
#endif
    bind_status = ((platform_config_status == FOC_STATUS_OK) &&
                   (platform_status == FOC_STATUS_OK) &&
                   (rust_status == FOC_STATUS_OK)) ?
        foc_platform_bind_controller(&g_foc_controller) : FOC_STATUS_NOT_CONFIGURED;
    /* 等 ADC 零点标定（32 次平均）与同步采样跑起来，否则第一次读回的是全 0。
     * 诊断位是在这些步骤里逐步置位的，因此这 100 ms 也是"平台就绪"标志位开始
     * 有效的时刻。
     * Wait for the 32-sample ADC offset calibration and synchronous sampling to
     * run; otherwise the first read comes back all zeros. The diagnostic bits are
     * published step by step inside those calls, so this delay is also when the
     * "platform ready" flags start to be meaningful. */
    rt_thread_mdelay(100);
    feedback_status = foc_platform_read_feedback(&feedback);
    (void)foc_platform_get_diagnostics(&diagnostics);
    heartbeat_ms = (uint32_t)rt_tick_get_millisecond();

    rt_kprintf("FBOOT,STM32G431\n");
    /* 启动报告是本工程的"开机自证"：构建档 + Rust 优化等级说明这份固件是哪一次
     * 测量的产物（Flash 与 WCET 都是按档位记录的，见 docs/构建档与优化等级.md），
     * 后面两行给出 ABI 与各初始化状态码，便于把"电机不动"定位到具体一步。
     * The boot report is this project's power-on evidence: the profile and Rust
     * opt-level identify which measured artefact this binary is (Flash and WCET are
     * recorded per profile, see docs/构建档与优化等级.md), and the following lines
     * give the ABI and every init status code so a "motor does not move" report can
     * be pinned to one step. */
    rt_kprintf("FBOOT,p=%s,o=%s\n",
               FLUXRT_BUILD_PROFILE_NAME,
               FLUXRT_RUST_OPT_LEVEL_NAME);
    rt_kprintf("FBOOT,fp=%s\n", FLUXRT_PRODUCT_PROFILE_NAME);
    rt_kprintf("FBOOT,pc=%08x\n",
               (unsigned int)foc_rust_product_contract_version());
    rt_kprintf("FBOOT,a=%08x,c=%u/%u,r=%u,p=%u/%u/%u\n",
               (unsigned int)foc_rust_abi_version(),
               (unsigned int)foc_rust_context_required_size(),
               (unsigned int)sizeof(g_foc_controller),
               (unsigned int)rust_status,
               (unsigned int)platform_config_status,
               (unsigned int)platform_status,
               (unsigned int)bind_status);
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
    rt_kprintf("FEXT,%u,%08x,%08x,%08x,%u\n",
               (unsigned int)external_io_status,
               (unsigned int)g_foc_external_io_management.capabilities.compiled_transport_mask,
               (unsigned int)g_foc_external_io_management.capabilities.board_transport_mask,
               (unsigned int)g_foc_external_io_management.active_config.transport_enable_mask,
               (unsigned int)g_foc_external_io_management.command_service.enabled);
    rt_kprintf("FRAW,%u,%08x,%08x\n",
               (unsigned int)external_input_port_status,
               (unsigned int)g_foc_external_input_port.supported_input_mask,
               (unsigned int)g_foc_external_input_port.enabled_input_mask);
    rt_kprintf("FINPUT,%u,%08x,%08x,%08x\n",
               (unsigned int)external_input_owner_status,
               (unsigned int)g_foc_external_input_owner.status.managed_input_mask,
               (unsigned int)g_foc_external_input_owner.status.healthy_input_mask,
               (unsigned int)g_foc_external_input_owner.status.fault_input_mask);
#endif
    rt_kprintf("FPROF,%u,%u,%08x,%08x,%02x,%08x,%08x\n",
               (unsigned int)g_foc_profile_status,
               (unsigned int)g_foc_profile_report.profile_revision,
               (unsigned int)g_foc_production_profile.board_id,
               (unsigned int)g_foc_production_profile.motor_id,
               (unsigned int)g_foc_profile_report.approval_flags,
               (unsigned int)g_foc_profile_report.computed_runtime_config_crc32,
               (unsigned int)g_foc_profile_report.computed_record_crc32);
    rt_kprintf("FMON,%u,%08x,%u,%u,%u,%u,%u,off\n",
               (unsigned int)feedback_status,
               (unsigned int)diagnostics.flags,
               (unsigned int)diagnostics.pwm_frequency_hz,
               (unsigned int)diagnostics.control_frequency_hz,
               (unsigned int)diagnostics.actuation_delay_pwm_ticks,
               (unsigned int)diagnostics.sync_sample_count,
               (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
    rt_kprintf("FBOOT,m=%s,o=%s,c=%u\n",
               (foc_math_accel_backend() == FOC_MATH_BACKEND_CORDIC)
                   ? "STM32G4-CORDIC+FPU"
                   : "CPU",
               foc_observer_name(g_foc_runtime_config.observer_backend),
               (unsigned int)g_foc_runtime_config.closed_loop_enable);

    while (1)
    {
#if defined(FLUXRT_TRACE_BUILD)
        foc_trace_sample_t trace_sample;
#endif
        uint32_t now_ms = (uint32_t)rt_tick_get_millisecond();
#if defined(FOC_EXTERNAL_IO_FRAMEWORK)
        /* P2.5 input processing has one low-frequency owner. Raw capture stays
         * in the ADC IRQ, while normalization, timeout Release and arbiter poll
         * execute here and therefore cannot extend motor ISR WCET. */
        (void)foc_platform_get_diagnostics(&diagnostics);
        external_input_owner_status = foc_external_input_owner_poll(
            &g_foc_external_input_owner,
            now_ms,
            ((diagnostics.flags &
              (FOC_PLATFORM_DIAG_DRIVER_FAULT |
               FOC_PLATFORM_DIAG_BREAK_LATCHED |
               FOC_PLATFORM_DIAG_CURRENT_TRIP |
               FOC_PLATFORM_DIAG_DEADLINE_MISSED |
               FOC_PLATFORM_DIAG_CONTROL_ERROR |
               FOC_PLATFORM_DIAG_OUTPUT_REJECTED)) != 0U) ? 1U : 0U);
        if (external_input_owner_status != FOC_EXTERNAL_INPUT_OWNER_OK)
        {
            foc_platform_emergency_stop();
        }
#endif
#if defined(FLUXRT_TRACE_BUILD)
        /* Shell 线程是环形缓冲的唯一消费者：把 ISR 攒下的样本排空并打成 FTR 行。
         * 这里允许长时间打印，代价是 Shell 阻塞期间心跳会推迟——trace 开启时本就
         * 不应该同时依赖心跳判活。
         * The shell thread is the sole consumer of the ring buffer: it drains the
         * samples produced by the ISR and prints them as FTR rows. Long printing is
         * acceptable here; the price is that the heartbeat slips while the shell is
         * blocked, and a heartbeat is not a valid liveness check while tracing. */
        while (foc_platform_trace_pop(&trace_sample) != 0U)
        {
            rt_kprintf("FTR,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u\n",
                       (unsigned int)trace_sample.step,
                       (unsigned int)trace_sample.state,
                       (int)trace_sample.phase_a_ma,
                       (int)trace_sample.phase_b_ma,
                       (int)trace_sample.phase_c_ma,
                       (int)trace_sample.id_reference_ma,
                       (int)trace_sample.iq_reference_ma,
                       (int)trace_sample.id_measured_ma,
                       (int)trace_sample.iq_measured_ma,
                       (int)trace_sample.vd_command_mv,
                       (int)trace_sample.vq_command_mv,
                       (unsigned int)trace_sample.duty_a_per_mille,
                       (unsigned int)trace_sample.duty_b_per_mille,
                       (unsigned int)trace_sample.duty_c_per_mille,
                       (unsigned int)trace_sample.bus_voltage_mv,
                       (int)trace_sample.control_angle_mrad,
                       (int)trace_sample.forced_angle_mrad,
                       (int)trace_sample.observer_angle_mrad,
                       (int)trace_sample.observer_speed_rpm,
                       (unsigned int)trace_sample.observer_reliable,
                       (unsigned int)trace_sample.flags,
                       (int)trace_sample.observer_bemf_alpha_mv,
                       (int)trace_sample.observer_bemf_beta_mv,
                       (int)trace_sample.observer_pll_phase_error_mrad,
                       (int)trace_sample.observer_speed_mean_rpm,
                       (unsigned int)trace_sample.observer_speed_variance_rpm2,
                       (unsigned int)trace_sample.observer_reliability_flags,
                       (unsigned int)trace_sample.observer_reliable_samples,
                       (unsigned int)trace_sample.observer_wait_elapsed_ms,
                       (unsigned int)trace_sample.observer_loss_elapsed_ms,
                       (unsigned int)trace_sample.voltage_limited);
        }
#endif
        /* 心跳只在 trace 和实时工作窗口都关闭时发送。板端 UART 后端可能短暂
         * 屏蔽中断；让 ALV 与 12 kHz 控制/探针并发会污染 WCET，甚至触发正确的
         * deadline 关断。命令侧 inhibit 关闭 start/finish 竞争，平台门覆盖正式
         * arm、motion/advanced 探针和 Ls(I) 实时事务。
         * Heartbeats are emitted only outside trace and realtime work windows.
         * Some board UART backends briefly mask IRQs, so concurrent ALV output
         * would contaminate WCET and can legitimately trip the deadline guard. */
        if (((now_ms - heartbeat_ms) >= 5000U) &&
            (g_foc_management_log_inhibit == 0U) &&
            (foc_platform_realtime_work_active() == 0U) &&
#if defined(FLUXRT_TRACE_BUILD)
            (foc_platform_trace_is_enabled() == 0U))
#else
            1)
#endif
        {
            heartbeat_ms = now_ms;
            (void)foc_platform_get_diagnostics(&diagnostics);
            /* 只在未 arm 时读反馈：arm 之后 ADC 与输出由 ISR 独占，Shell 线程再去
             * 发软件触发采样会与实时路径抢 ADC，并污染同步采样的时序证据。
             * Read feedback only while not armed: once armed the ADC and outputs are
             * owned by the ISR, and a shell-thread software-triggered read would
             * contend with the realtime path and pollute the synchronous-sampling
             * timing evidence. */
            if ((diagnostics.flags & FOC_PLATFORM_DIAG_REALTIME_ARMED) == 0U)
            {
                (void)foc_platform_read_feedback(&feedback);
            }
            /* Feedback reads update slow-monitor fields, so publish a snapshot
             * only afterwards; otherwise ALV reports the previous Vbus sample. */
            (void)foc_platform_get_diagnostics(&diagnostics);
            rt_kprintf("ALV,%08x,%u,%u,%u,%u\n",
                       (unsigned int)diagnostics.flags,
                       (unsigned int)diagnostics.sync_sample_count,
                       (unsigned int)diagnostics.realtime_step_count,
                       (unsigned int)diagnostics.realtime_error_count,
                       (unsigned int)foc_bus_voltage_mv(diagnostics.bus_voltage_raw));
        }
        /* 10 ms 的轮询周期：既让 FTR 流能被及时排空，又远低于 5 s 心跳粒度，
         * 不对实时路径产生任何影响。本循环没有其它职责，也不参与控制。
         * A 10 ms poll period drains the FTR stream promptly while staying far
         * below the 5 s heartbeat granularity, and has no effect on the realtime
         * path. This loop has no other duty and takes no part in control. */
        rt_thread_mdelay(10);
    }
}
