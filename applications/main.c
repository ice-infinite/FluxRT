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
 *   - foc_start 是**唯一**可能让功率级 arm 的命令；它在平台未就绪或母线电压
 *     不在窗口内时拒绝启动，拒绝时不改任何输出；
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
#include <stdlib.h>
#include <string.h>

#include "foc_math_accel.h"
#include "foc_platform.h"
#include "foc_production_profile.h"
#include "foc_rust_bridge.h"

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

/* Rust 控制器上下文 [bytes]。内容是 Rust 私有状态，C 侧只当作不透明缓冲；
 * 容量由 FOC_RUST_CONTEXT_CAPACITY 与 foc_rust_bridge.h 的 _Static_assert 固定。
 * Rust controller context in bytes. Its contents are Rust-private state and the C
 * side treats it as an opaque buffer; the capacity is fixed by
 * FOC_RUST_CONTEXT_CAPACITY and the _Static_assert in foc_rust_bridge.h. */
static foc_rust_context_t g_foc_controller;
/* 当前生效的运行时配置。只有 foc_cfg 成功提交后才整体替换，失败时保持原值。
 * The active runtime configuration. It is replaced as a whole only after foc_cfg
 * commits successfully, and keeps its old value on failure. */
static foc_runtime_config_t g_foc_runtime_config;
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
#if defined(FLUXRT_CALIBRATION_CAPTURE_ONLY_BUILD)
    /* Calibration 固件不允许 arm。应用层先停机，平台层还会做第二道编译期拒绝，
     * 防止未来新增调用点绕过 Shell。 */
    (void)argc;
    (void)argv;
    foc_platform_control_stop();
    rt_kprintf("FOC start REFUSED: Calibration profile is capture-only; motor arm is compiled out.\n");
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
/* Production 档裁掉了 strtof，改用 strtol 走整数路径再转 float：转速是
 * 整数 rpm，精度足够，且避免把 newlib strtod 依赖链拉进镜像。
 * Production drops strtof and parses an integer with strtol before widening to
 * float: speed is an integer number of rpm, which is precise enough, and this
 * keeps newlib strtod out of the image. */
#if defined(FLUXRT_PRODUCTION_BUILD)
        target_rpm = (float)strtol(argv[1], RT_NULL, 10);
#else
        target_rpm = strtof(argv[1], RT_NULL);
#endif
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

#if defined(FLUXRT_DIAGNOSTIC_BUILD) || defined(FLUXRT_CALIBRATION_BUILD)
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
        repeats = (uint32_t)strtoul(argv[1], RT_NULL, 10);
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
        sample_hz = (uint32_t)strtoul(argv[2], RT_NULL, 10);
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
        else if (strcmp(argv[2], "st") == 0) candidate->observer_backend = FOC_OBSERVER_ST_STO_PLL;
        else return -1;
    }
    else if (strcmp(argv[1], "closedloop") == 0)
    {
        int value = atoi(argv[2]);
        if ((value != 0) && (value != 1)) return -1;
        candidate->closed_loop_enable = (uint32_t)value;
    }
    else if (strcmp(argv[1], "observe") == 0)
    {
        int value = atoi(argv[2]);
        if ((value != 0) && (value != 1)) return -1;
        candidate->observer_enable = (uint32_t)value;
    }
    else if (strcmp(argv[1], "invstage") == 0)
    {
        uint32_t stage = (uint32_t)strtoul(argv[2], RT_NULL, 10);
        if (stage > 4U) return -1;
        candidate->inverter_voltage_model.enabled = (stage != 0U) ? 1U : 0U;
        candidate->inverter_voltage_model.observer_voltage_correction_enable =
            ((stage == 2U) || (stage == 4U)) ? 1U : 0U;
        candidate->inverter_voltage_model.pwm_feedforward_enable =
            ((stage == 3U) || (stage == 4U)) ? 1U : 0U;
    }
    else if (strcmp(argv[1], "obsdiv") == 0)
    {
        candidate->observer_update_divider = (uint32_t)strtoul(argv[2], RT_NULL, 10);
    }
    else if (strcmp(argv[1], "startup") == 0)
    {
        candidate->startup_final_speed_rpm = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "current") == 0)
    {
        candidate->startup_current_a = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "aligncurrent") == 0)
    {
        candidate->startup_alignment_current_a = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "align") == 0)
    {
        candidate->alignment_duration_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "ramp") == 0)
    {
        candidate->open_loop_ramp_duration_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "util") == 0)
    {
        candidate->voltage_utilization = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "slide") == 0)
    {
        candidate->observer_smo_k_slide_v = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "boundary") == 0)
    {
        candidate->observer_smo_boundary_a = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "filter") == 0)
    {
        candidate->observer_emf_filter_alpha = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "pll_kp") == 0)
    {
        candidate->observer_pll_kp = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "pll_acq") == 0)
    {
        candidate->observer_acquisition_pll_kp_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "pll_ki") == 0)
    {
        candidate->observer_pll_ki = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "minspeed") == 0)
    {
        candidate->observer_minimum_speed_rpm = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "runminspeed") == 0)
    {
        candidate->observer_run_reliability.minimum_speed_rpm = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "runphase") == 0)
    {
        candidate->observer_run_reliability.maximum_phase_error_rad =
            strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "acqphase") == 0)
    {
        candidate->observer_acquisition_maximum_phase_error_rad =
            strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "parkdelay") == 0)
    {
        candidate->angle_compensation.park_prediction_ticks =
            strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "revparkdelay") == 0)
    {
        candidate->angle_compensation.reverse_park_prediction_ticks =
            strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "minbemf") == 0)
    {
        candidate->observer_minimum_bemf_v = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "variance") == 0)
    {
        candidate->observer_speed_variance_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "confirm") == 0)
    {
        candidate->observer_consecutive_samples = (uint32_t)strtoul(argv[2], RT_NULL, 10);
    }
    else if (strcmp(argv[1], "acquire") == 0)
    {
        candidate->observer_acquisition_timeout_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "transition") == 0)
    {
        candidate->observer_transition_duration_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "loss") == 0)
    {
        candidate->observer_loss_timeout_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "accel") == 0)
    {
        candidate->closed_loop_speed_ramp_rpm_per_s = strtof(argv[2], RT_NULL);
    }
    else if (strcmp(argv[1], "preload") == 0)
    {
        candidate->speed_pi_preload_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "support") == 0)
    {
        candidate->handoff_torque_support_ratio = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "islew") == 0)
    {
        candidate->closed_loop_current_slew_a_per_s = strtof(argv[2], RT_NULL) / 1000.0f;
    }
    else if (strcmp(argv[1], "trip") == 0)
    {
        /* 过流跳闸是平台层安全窗口的一部分，不属于 Rust 运行时配置，因此走
         * foc_platform_configure() 单独校验，并在成功后同步本地副本。
         * The over-current trip belongs to the platform safety window rather than
         * the Rust runtime configuration, so it is validated separately through
         * foc_platform_configure() and the local copy is synced on success. */
        foc_platform_config_t platform_candidate = g_foc_platform_config;
        platform_candidate.software_current_trip_a = strtof(argv[2], RT_NULL) / 1000.0f;
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

    /* 在任何初始化之前先把功率级置为安全态。这是"失败不留 arm 状态"这条约束的
     * 起点：后面每一步失败时，功率级都还没有被 arm 过。
     * Put the power stage into a safe state before any initialisation: this is the
     * starting point of the "no failure leaves an armed stage" rule, because no
     * later step has armed anything yet. */
    foc_platform_emergency_stop();
    /* ABI 自检 / ABI self-check：版本、Rust 要求的上下文大小、Rust 要求的对齐。
     * 三项任一不满足都说明 C 与 Rust 的编译配置不匹配，此时宁可挂起也不运行。
     * Three independent checks: the version, the context size Rust requires and the
     * alignment Rust requires. Any mismatch means the C and Rust build settings
     * disagree, and idling is strictly better than running. */
    if ((foc_rust_abi_version() != FOC_RUST_ABI_VERSION) ||
        (foc_rust_context_required_size() > sizeof(g_foc_controller)) ||
        (foc_rust_context_required_align() > _Alignof(foc_rust_context_t)))
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
    /* 平台侧三个调用各自独立记录状态：configure 建立安全窗口，init 配置 TIM1/ADC/
     * 栅极与标定，bind 把平台输出接到 Rust 控制器。任一失败都会让 foc_start 走
     * 拒绝路径，而不会 arm。
     * The three platform calls record their status independently: configure
     * establishes the safety window, init sets up TIM1/ADC/gates and calibration,
     * and bind connects the platform output to the Rust controller. Any failure
     * makes foc_start refuse rather than arm. */
    platform_config_status = foc_platform_configure(&g_foc_platform_config);
    platform_status = foc_platform_init();
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
    bind_status = ((platform_status == FOC_STATUS_OK) &&
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
    rt_kprintf("FBOOT,a=%08x,c=%u/%u,r=%u,p=%u/%u/%u\n",
               (unsigned int)foc_rust_abi_version(),
               (unsigned int)foc_rust_context_required_size(),
               (unsigned int)sizeof(g_foc_controller),
               (unsigned int)rust_status,
               (unsigned int)platform_config_status,
               (unsigned int)platform_status,
               (unsigned int)bind_status);
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
        /* 心跳每 5 s 一次，且只在 trace 关闭时发送：FTR 流量与心跳混在同一串口上
         * 会互相插入，使离线解析难以按行切分。
         * The heartbeat fires every 5 s and only while trace is off: interleaving
         * FTR traffic with the heartbeat on the same UART makes line-based offline
         * parsing unreliable. */
        if (((now_ms - heartbeat_ms) >= 5000U) &&
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
