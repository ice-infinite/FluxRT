#ifndef FOC_PLATFORM_H
#define FOC_PLATFORM_H

#include "foc_build_profile.h"

/*
 * FluxRT —— 芯片平台层契约（C 侧）。换 MCU 时重写实现，不改本头文件。
 * FluxRT - chip platform contract (C side). Porting to another MCU means
 * reimplementing this header's symbols, not changing the header.
 *
 * 职责 / Responsibility:
 *   - 定义平台层向应用层和 Rust 桥接层暴露的全部入口；
 *   - 定义诊断位、平台配置、诊断快照和 trace 样本的数据模型。
 *   - Defines every entry point the platform exposes, plus the data model for
 *     diagnostics, platform configuration, the diagnostic snapshot and trace
 *     samples.
 *
 * 分层位置 / Position in the layering:
 *   applications/  ->  foc_platform_*()  ->  Rust 快环
 *   C 平台层是唯一允许写寄存器的位置；Rust 侧永远不碰硬件。
 *   The C platform layer is the only place allowed to write registers; the Rust
 *   side never touches hardware.
 *
 * 移植要求 / Porting requirements:
 *   新的 `foc/platform/<mcu>/` 必须实现本文件声明的**全部**符号，且语义一致，
 *   尤其是失败路径必须已经关断功率级。缺符号会导致链接失败（刻意的）。
 *   A new `foc/platform/<mcu>/` must implement every symbol declared here with
 *   the same semantics; in particular every failure path must already have
 *   disabled the power stage. A missing symbol fails the link, deliberately.
 *
 * 参考 / Reference: docs/架构与安全边界.md, docs/更换MCU移植步骤.md
 */

#include "foc_rust_bridge.h"
#include "foc_phase_voltage_capture.h"
#include "foc_phase_voltage_model.h"
#include "foc_realtime_timing.h"
#include "foc_types.h"

/* 平台配置结构体版本，与 Rust 侧的 config_version 自检配合使用。
 * Platform configuration struct version, paired with the Rust-side
 * config_version self-check. */
#define FOC_PLATFORM_CONFIG_VERSION (1UL)
/* 默认 ISR 软件截止 [cycles]。CM4.1 接管拍实测峰值 12,548 cycles；12 kHz
 * 物理周期约 14,167 cycles，取 12,750 后仍保留 1,417 cycles（10.0%）硬余量。
 * Default ISR software deadline. A measured CM4.1 handover peak was 12,548
 * cycles; 12,750 retains 1,417 cycles (10.0%) of the 14,167-cycle period. */
#define FOC_DEFAULT_ISR_DEADLINE_CYCLES (12750UL)

/*
 * 诊断位。这是一套**硬件视角**的位域，与 foc_rust_bridge.h 中
 * FOC_RUST_FAULT_* 那套"算法视角"的位域相互独立，不要混用。
 * Diagnostic bits. This is a hardware-perspective bit space, independent of the
 * algorithm-perspective FOC_RUST_FAULT_* bits in foc_rust_bridge.h. Do not mix
 * them.
 *
 * 位序即 ABI：只能追加，不能重排或复用已删除的位。
 * The bit order is ABI: only append; never reorder or reuse a retired bit.
 */
enum
{
    /* 栅极使能为低且 TIM1 无输出。这是"寄存器级证据"位。
     * Gate enables low and TIM1 outputs off, i.e. register-level evidence. */
    FOC_PLATFORM_DIAG_GATE_SAFE = (1UL << 0),
    /* TIM1 已完成初始化（含当前载波/控制时序、Break2、死区配置）。
     * TIM1 fully configured, including the active carrier/control timing,
     * Break2 and dead time. */
    FOC_PLATFORM_DIAG_TIM1_CONFIGURED = (1UL << 1),
    /* ADC1/ADC2 已完成初始化与多模式配置。
     * ADC1/ADC2 initialized, including multi-mode configuration. */
    FOC_PLATFORM_DIAG_ADC_CONFIGURED = (1UL << 2),
    /* ADC 自校准完成 / ADC self-calibration completed. */
    FOC_PLATFORM_DIAG_ADC_CALIBRATED = (1UL << 3),
    /* 三相电流静态零点在有效区间内 / Static current offsets within valid range. */
    FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID = (1UL << 4),
    /* 驱动器保护输入为低（active-low 故障）/ Driver protection input asserted. */
    FOC_PLATFORM_DIAG_DRIVER_FAULT = (1UL << 5),
    /* 栅极为高或 TIM1 有输出，即"输出已激活"。
     * Gate high or TIM1 outputs enabled, i.e. output is active. */
    FOC_PLATFORM_DIAG_OUTPUT_ACTIVE = (1UL << 6),
    /* 软件触发 ADC 轮询失败 / A software-triggered ADC poll failed. */
    FOC_PLATFORM_DIAG_ADC_READ_ERROR = (1UL << 7),
    /* TIM1 计数与内部 TRGO 已使能，同步采样时基在运行。
     * TIM1 is counting with its internal TRGO sampling time base enabled. */
    FOC_PLATFORM_DIAG_SYNC_RUNNING = (1UL << 8),
    /* 至少收到过一次注入组采样 / At least one injected sample has arrived. */
    FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID = (1UL << 9),
    /* 硬件 Break 已触发并锁存 / A hardware Break fired and is latched. */
    FOC_PLATFORM_DIAG_BREAK_LATCHED = (1UL << 10),
    /* 软件过流跳闸已触发 / The software over-current trip fired. */
    FOC_PLATFORM_DIAG_CURRENT_TRIP = (1UL << 11),
    /* 早期试转包络已 arm（非实时路径）/ The legacy trial envelope is armed. */
    FOC_PLATFORM_DIAG_TRIAL_ARMED = (1UL << 12),
    /* Rust 控制器已绑定到平台 / The Rust controller is bound to the platform. */
    FOC_PLATFORM_DIAG_CONTROLLER_BOUND = (1UL << 13),
    /* 实时快环已 arm / The realtime fast loop is armed. */
    FOC_PLATFORM_DIAG_REALTIME_ARMED = (1UL << 14),
    /* ISR 超出软件截止周期 / The ISR exceeded the software deadline. */
    FOC_PLATFORM_DIAG_DEADLINE_MISSED = (1UL << 15),
    /* Rust 快环返回了非 OK / The Rust fast loop returned non-OK. */
    FOC_PLATFORM_DIAG_CONTROL_ERROR = (1UL << 16),
    /* 快环输出越界被拒 / The fast-loop output was rejected as out of range. */
    FOC_PLATFORM_DIAG_OUTPUT_REJECTED = (1UL << 17),
    /* Diagnostic 档已把 PC0/PC3/PC1 配为 U/V/W 端电压同步注入通道。 */
    FOC_PLATFORM_DIAG_PHASE_VOLTAGE_CONFIGURED = (1UL << 18),
};

/*
 * 平台级可调参数。前两个字段是版本自检，与
 * foc_runtime_config_t 的做法一致。
 * Platform-level tunables. The first two fields are version self-checks, matching
 * the approach used by foc_runtime_config_t.
 *
 * 与 foc_runtime_config_t 的分工 / Division of labour: 本结构体管"硬件安全
 * 包络"（电压窗口、过流、占空比窗口、时序截止），那个管"控制算法参数"。
 * This struct governs the hardware safety envelope (voltage window, over-current,
 * duty window, timing deadline); the other governs control-algorithm parameters.
 */
typedef struct
{
    /* 结构体尺寸自检 / Struct size self-check. */
    uint32_t struct_size;
    /* 配置版本自检 / Config version self-check. */
    uint32_t config_version;
    /* 允许 arm 的最低母线电压 [V] / Minimum bus voltage for arming [V]. */
    float minimum_bus_voltage_v;
    /* 允许 arm 的最高母线电压 [V] / Maximum bus voltage for arming [V]. */
    float maximum_bus_voltage_v;
    /* 软件过流阈值 [A]，在 ISR 内与三相电流峰值比较。
     * Software over-current trip [A], compared against the peak phase current in
     * the ISR. */
    float software_current_trip_a;
    /* 允许写入的最小/最大占空比 [-]，用于保证自举电容充电和高侧驱动
     * 所需的最小脉宽。
     * Minimum and maximum writable duty, preserving the minimum pulse width
     * needed for bootstrap charging and the high-side driver. */
    float minimum_duty;
    float maximum_duty;
    /* ISR 软件截止 [cycles]，超限即视为 deadline miss 并立即关断。
     * ISR software deadline in cycles; exceeding it counts as a miss and shuts
     * the power stage down. */
    uint32_t isr_deadline_cycles;
} foc_platform_config_t;

/*
 * 诊断快照，供 Shell 与上层读取。
 * Diagnostic snapshot for the shell and upper layers.
 *
 * 时序字段的口径 / Convention for the timing fields:
 *   `maximum_*_cycles` 是**三个独立峰值**，来自不同控制拍，禁止相加。
 *   同拍的完整数值请用 foc_platform_get_timing()。
 *   The `maximum_*_cycles` fields are three INDEPENDENT peaks taken from
 *   different ticks and must never be summed. For same-tick values use
 *   foc_platform_get_timing().
 *
 * 计数单位 / Counting units: 除 `*_raw` 与 `*_offset` 是 12 位 ADC counts 外，
 * 其余均为计数或 cycles；占空比用 per-mille（千分比）。
 * Except for the `*_raw` and `*_offset` fields, which are 12-bit ADC counts, all
 * fields are counters or cycles; duty is expressed in per-mille.
 */
typedef struct
{
    /* 诊断位，取值见上方 enum / Diagnostic bits, see the enum above. */
    uint32_t flags;
    /* 当前 PWM 载波频率 [Hz] / Current PWM carrier rate [Hz]. */
    uint32_t pwm_frequency_hz;
    /* ADC 注入组与完整电流控制链频率 [Hz] / ADC and full current-control rate [Hz]. */
    uint32_t control_frequency_hz;
    /* ARR 值 [counts] / ARR value in timer counts. */
    uint32_t pwm_period_ticks;
    /* 每个控制拍包含的 PWM 周期数 / PWM periods per control tick. */
    uint32_t pwm_ticks_per_control;
    /* CCR preload 写入到生效的名义整 PWM 拍延迟；精确相位需目标测量。 */
    uint32_t actuation_delay_pwm_ticks;
    /* ADC 触发源，取值见 foc_pwm_adc_trigger_t；为避免平台头耦合这里只存 uint32_t。 */
    uint32_t adc_trigger_source;
    /* 相邻同步 ADC ISR 入口间隔；仅 timing probe 构建记录 [cycles]。 */
    uint32_t sync_interval_last_cycles;
    uint32_t sync_interval_min_cycles;
    uint32_t sync_interval_max_cycles;
    uint64_t sync_interval_sum_cycles;
    uint32_t sync_interval_sample_count;
    /* 未 arm 监测路径的 ISR 包络；不等于完整 FOC WCET [cycles]。 */
    uint32_t monitor_isr_last_cycles;
    uint32_t monitor_isr_min_cycles;
    uint32_t monitor_isr_max_cycles;
    /* 注入组采样累计数 / Cumulative injected sample count. */
    uint32_t sync_sample_count;
    /* 硬件 Break 触发次数 / Number of hardware Break events. */
    uint32_t break_fault_count;
    /* 兼容路径写入 CCR 的次数 / Number of CCR writes through the legacy path. */
    uint32_t trial_apply_count;
    /* 实时快环成功步数 / Successful realtime fast-loop steps. */
    uint32_t realtime_step_count;
    /* 实时快环错误步数 / Failed realtime fast-loop steps. */
    uint32_t realtime_error_count;
    /* 最坏完整 ISR 拍 [cycles] / Worst complete ISR tick [cycles]. */
    uint32_t maximum_isr_cycles;
    /* precontrol 段独立峰值 [cycles]，仅定位热点 / Independent peaks only. */
    uint32_t maximum_precontrol_cycles;
    /* control 段独立峰值 [cycles]，仅定位热点 / Independent peaks only. */
    uint32_t maximum_control_cycles;
    /* postcontrol 段独立峰值 [cycles]，仅定位热点 / Independent peaks only. */
    uint32_t maximum_postcontrol_cycles;
    /* 超出软件截止的拍数 / Ticks exceeding the software deadline. */
    uint32_t deadline_miss_count;
    /* 最近一次 foc_rust_realtime_step() 的返回状态 / Last fast-loop status. */
    uint32_t last_control_status;
    /* 最近一次锁存的 Rust 故障位 / Last latched Rust fault bits. */
    uint32_t control_fault_flags;
    /* 最近一次被拒输出的三相占空比 [per-mille]，用于定位是哪一相越界。
     * Last rejected duty triplet in per-mille, so the offending phase is visible. */
    uint16_t last_duty_a_per_mille;
    uint16_t last_duty_b_per_mille;
    uint16_t last_duty_c_per_mille;
    uint16_t reserved1;
    /* arm 期间观测到的最大单相电流偏移 [counts]（相对标定零点）。
     * Largest single-phase current offset seen while armed, in counts relative
     * to the calibrated zero. */
    uint16_t peak_current_delta_counts;
    uint16_t reserved0;
    /* 三相电流与母线/温度/电位器的原始 ADC 值及标定零点 [counts]。
     * Raw ADC values and calibrated offsets for the three currents, bus,
     * temperature and potentiometer, in counts. */
    uint16_t phase_u_raw;
    uint16_t phase_v_raw;
    uint16_t phase_w_raw;
    uint16_t phase_u_offset;
    uint16_t phase_v_offset;
    uint16_t phase_w_offset;
    uint16_t bus_voltage_raw;
    uint16_t temperature_raw;
    uint16_t potentiometer_raw;
    uint16_t reserved;
} foc_platform_diagnostics_t;

/*
 * 定长定点 trace 样本，由 ISR 生产、Shell 线程消费。
 * Fixed-size fixed-point trace sample, produced by the ISR and consumed by the
 * shell thread.
 *
 * 为什么要定长定点 / Why fixed size and fixed point: 环形缓冲必须能在 ISR 里
 * 无锁搬运，因此样本不能含指针、不能变长、也不能是浮点（浮点的位模式对
 * 离线工具不友好）。所有量纲转换成整数是刻意的。
 * The ring buffer must be movable from the ISR without locks, so a sample cannot
 * contain pointers, cannot vary in length, and is not stored as float. Converting
 * every quantity to an integer is deliberate.
 *
 * 单位 / Units: 电流 mA、电压 mV、角度 mrad、占空比 per-mille、转速 rpm。
 * Currents in mA, voltages in mV, angles in mrad, duty in per-mille, speed in rpm.
 *
 * 尺寸 / Size: 72 字节。改变字段会同时影响 Rust 桥接、Shell 输出和主机
 * 解析脚本，必须同步修改三处。
 * 72 bytes. Changing a field affects the Rust bridge, the shell output and the
 * host-side parsers; all three must be updated together.
 */
typedef struct
{
    /* 控制器步序号 / Controller step index. */
    uint32_t step;
    /* 该拍的诊断位快照 / Diagnostic bit snapshot for this tick. */
    uint32_t flags;
    /* 控制器逻辑状态 / Logical controller state (foc_state_t). */
    uint16_t state;
    /* 观测器可靠性门结果（0/1）/ Observer reliability gate result. */
    uint16_t observer_reliable;
    /* 三相电流 [mA] / Three-phase currents in mA. */
    int16_t phase_a_ma;
    int16_t phase_b_ma;
    int16_t phase_c_ma;
    /* Id/Iq 给定 [mA] / Id and Iq references in mA. */
    int16_t id_reference_ma;
    int16_t iq_reference_ma;
    /* Id/Iq 实测 [mA] / Measured Id and Iq in mA. */
    int16_t id_measured_ma;
    int16_t iq_measured_ma;
    /* dq 电压指令 [mV] / dq voltage commands in mV. */
    int16_t vd_command_mv;
    int16_t vq_command_mv;
    /* 三相占空比 [per-mille] / Three-phase duty in per-mille. */
    uint16_t duty_a_per_mille;
    uint16_t duty_b_per_mille;
    uint16_t duty_c_per_mille;
    /* 母线电压 [mV] / Bus voltage in mV. */
    uint16_t bus_voltage_mv;
    /* 三个角度分开记录 / Three separately recorded angles:
     *   control  - 本次 Park/逆 Park 真正使用的角度 [mrad]
     *   forced   - Rev-Up 强制角 [mrad]
     *   observer - 观测器估计角 [mrad]
     * 混成一个字段会导致事后无法判断失锁发生在观测器还是交接混合。
     * Merging them would make it impossible to tell afterwards whether a loss of
     * lock happened in the observer or in the handover blend. */
    int16_t control_angle_mrad;
    int16_t forced_angle_mrad;
    int16_t observer_angle_mrad;
    /* 观测器估计机械转速 [rpm]（估计值，非真值测量）。
     * Observer-estimated mechanical speed in rpm; an estimate, not ground truth. */
    int16_t observer_speed_rpm;
    /* 失锁前诊断：滤波 BEMF [mV]、PLL 鉴相误差 [mrad]、64 ms 窗口速度均值 [rpm]。 */
    int16_t observer_bemf_alpha_mv;
    int16_t observer_bemf_beta_mv;
    int16_t observer_pll_phase_error_mrad;
    int16_t observer_speed_mean_rpm;
    /* 速度窗口方差 [rpm^2]。 */
    uint32_t observer_speed_variance_rpm2;
    /* FOC_OBSERVER_GATE_* 位和连续通过窗口数。 */
    uint16_t observer_reliability_flags;
    uint16_t observer_reliable_samples;
    /* 获取等待/连续失锁计时 [ms]，均在故障前最后成功拍冻结。 */
    uint16_t observer_wait_elapsed_ms;
    uint16_t observer_loss_elapsed_ms;
    /* dq 电压圆限幅是否生效（0/1）。 */
    uint16_t voltage_limited;
    uint16_t reserved_diagnostic;
} foc_trace_sample_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_trace_sample_t) == 72U, "FOC trace ABI changed");
#endif

/*
 * 芯片/板级端口契约。未来的 MCU 适配层必须实现以下全部符号。
 * Chip/board port contract. A future MCU adapter must implement all of these.
 *
 * STM32G431 适配层上电时所有 IHM16M1 输出均关闭。常规闭环路径在观测器
 * 通过可靠性门之前不可用；早期 bring-up 试转必须通过
 * foc_platform_trial_arm() 显式、有界地 arm 功率级。
 * The STM32G431 adapter boots with every IHM16M1 output disabled. The normal
 * closed-loop path stays unavailable until the observer passes its reliability
 * gate; an early bring-up trial must arm the power stage explicitly and in a
 * bounded way through foc_platform_trial_arm().
 *
 * 返回约定 / Return convention: 所有返回 foc_status_t 的函数在非 OK 返回时
 * 必须已经保证功率级处于安全状态。
 * Every function returning foc_status_t must already have left the power stage in
 * a safe state on any non-OK return.
 */
foc_status_t foc_platform_init(void);
foc_status_t foc_platform_configure(const foc_platform_config_t *config);
foc_status_t foc_platform_bind_controller(foc_rust_context_t *context);
foc_status_t foc_platform_control_start(float target_speed_rpm);
void foc_platform_control_stop(void);
void foc_platform_emergency_stop(void);
foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback);
foc_status_t foc_platform_apply_output(const foc_output_t *output);
foc_status_t foc_platform_get_diagnostics(foc_platform_diagnostics_t *diagnostics);
foc_status_t foc_platform_get_telemetry(foc_telemetry_t *telemetry);
/* 复制同拍时序统计。与诊断里的三个独立峰值不同，这里的 total 与三个分段
 * 来自同一拍，可以互相比较。
 * Copies the same-tick timing statistics. Unlike the three independent peaks in
 * the diagnostics, here total and the three segments come from one tick and may
 * be compared. */
foc_status_t foc_platform_get_timing(foc_realtime_timing_stats_t *timing);
/*
 * trace 接口。Production 构建档会裁掉整个 trace 机制，此时
 * trace_start 返回 FOC_STATUS_NOT_CONFIGURED，其余函数返回未启用/0。
 * Trace interface. The Production profile removes the whole mechanism; then
 * trace_start returns FOC_STATUS_NOT_CONFIGURED and the others report disabled
 * or 0.
 */
foc_status_t foc_platform_trace_start(uint32_t sample_divider);
void foc_platform_trace_stop(void);
uint32_t foc_platform_trace_is_enabled(void);
uint32_t foc_platform_trace_pop(foc_trace_sample_t *sample);
uint32_t foc_platform_trace_dropped(void);
/*
 * Diagnostic-only 三相端电压短窗。只能在功率级未 arm 时启动；调用方必须
 * 明确选择 PC9 分压网络开/关，Production 返回 NOT_CONFIGURED。样本保留 ADC
 * 原始码，不冒充已标定的相电压。
 */
foc_status_t foc_platform_phase_voltage_capture_start(
    foc_phase_voltage_divider_mode_t divider_mode);
void foc_platform_phase_voltage_capture_stop(void);
foc_status_t foc_platform_phase_voltage_capture_status(
    foc_phase_voltage_capture_status_t *status);
uint32_t foc_platform_phase_voltage_capture_pop(
    foc_phase_voltage_sample_t *sample);
/*
 * 返回当前板卡编入固件的相电压换算模型。官方名义模型只用于诊断，
 * 不代表逐板标定完成，也不改变观察器电压来源。
 */
foc_status_t foc_platform_get_phase_voltage_model(
    foc_phase_voltage_model_t *model);
foc_status_t foc_platform_phase_voltage_raw_to_mv(
    foc_phase_voltage_divider_mode_t divider_mode,
    uint16_t raw,
    uint32_t *phase_voltage_mv);
/*
 * 早期有界试转包络（兼容路径）。当前实时路径改用
 * foc_platform_control_start()；这两个函数保留给历史脚本。
 * The legacy bounded trial envelope. The realtime path now uses
 * foc_platform_control_start(); these two remain for historical scripts.
 */
foc_status_t foc_platform_trial_arm(void);
void foc_platform_trial_disarm(void);

#endif
