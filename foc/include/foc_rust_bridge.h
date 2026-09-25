#ifndef FOC_RUST_BRIDGE_H
#define FOC_RUST_BRIDGE_H

#include "foc_build_profile.h"

/*
 * FluxRT —— C 与 Rust 之间的唯一公共 ABI 契约。
 * FluxRT - the single public ABI contract between C and Rust.
 *
 * 职责 / Responsibility:
 *   声明 C 侧可以调用的全部 Rust 入口，以及跨边界传输的定宽结构体。
 *   对应的 Rust 实现在 rust/crates/foc-rt-bridge/src/lib.rs。
 *   Declares every Rust entry point callable from C and the fixed-width structs
 *   that cross the boundary. The Rust side lives in
 *   rust/crates/foc-rt-bridge/src/lib.rs.
 *
 * 为什么不用函数表 / Why no function-pointer table:
 *   芯片适配与语言边界都用固定 C 符号链接，而不是运行时函数表。这样调用
 *   路径清楚、便于内联、不会在运行时选错适配器，也更容易从 map 文件确认
 *   最终链接的实现。
 *   Both the chip adapter and the language boundary use fixed C symbol linkage
 *   rather than a runtime function table: the call path stays clear, inlining
 *   works, a wrong adapter cannot be selected at runtime, and the map file
 *   shows the linked implementation unambiguously.
 *
 * 跨边界规则（必须遵守）/ Boundary rules (mandatory):
 *   1. 不使用 C 短枚举；状态与状态码是固定 uint32_t。
 *      No C short enums; states and status codes are fixed uint32_t.
 *   2. 结构体字段顺序即 ABI 顺序，两侧必须逐字段一致；尺寸由本文件末尾的
 *      _Static_assert 在编译期校验。
 *      Field order is ABI order; sizes are enforced by the _Static_assert
 *      block at the end of this file.
 *   3. 不跨边界传引用、String、切片、trait object、泛型或带析构的类型。
 *      No references, String, slices, trait objects, generics or destructible
 *      types cross the boundary.
 *   4. 不跨边界分配内存。C 提供静态存储，Rust 在其中原位构造控制器。
 *      Nothing allocates across the boundary; C provides static storage and
 *      Rust constructs the controller in place.
 *   5. 必须先 foc_rust_init()。同一上下文不得被线程与中断并发修改。
 *      foc_rust_init() must come first, and one context must never be mutated
 *      concurrently by a thread and an ISR.
 *   6. 改动任何字段或函数签名都必须提升 FOC_RUST_ABI_VERSION 并同时修改
 *      两侧，否则 main.c 的启动自检会拒绝运行（这是设计意图）。
 *      Any field or signature change must bump FOC_RUST_ABI_VERSION and update
 *      both sides; otherwise the boot self-check in main.c refuses to run. That
 *      refusal is intentional.
 *
 * 参考 / Reference: docs/C与Rust混合架构.md §4
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ABI 版本。主/次版本各占 16 位：0x0011_0000 表示第 17 代主版本。
 * ABI version. Major and minor occupy one 16-bit half each: 0x0011_0000 is
 * major revision 17.
 *
 * 提升规则 / Bump rule: 任何结构体字段、函数签名或语义（不仅是尺寸）变化
 * 都必须提升；main.c 会在启动时比对并拒绝不匹配的固件组合。
 * Bump for any struct field, signature or semantic change, not only size
 * changes. main.c compares this at boot and rejects a mismatched combination.
 */
#define FOC_RUST_ABI_VERSION        (0x00110000UL)
/* 运行时配置结构体的版本，与 ABI 版本独立演进，用于结构体内自检。
 * Version of the runtime configuration struct; it evolves independently of the
 * ABI version and is used for the struct's internal self-check. */
#define FOC_RUST_CONFIG_VERSION     (11UL)
/* C 侧提供的控制器存储容量 [bytes]，对齐 8 字节。
 * 容量大于当前控制器尺寸（编译期 assert 保证装得下），留有余量供后续
 * 增加状态字段而不必立刻改动 C 侧。
 * Controller storage capacity supplied by C [bytes], 8-byte aligned. It exceeds
 * the current controller size (compile-time asserted) so that new state fields
 * do not immediately force a change on the C side. */
#define FOC_RUST_CONTEXT_CAPACITY   (2048U)

/*
 * 逻辑故障位。与 foc_platform_diagnostics_t.flags 是两套独立位域：
 * 这一组由 Rust 控制器置位，描述控制算法自身的问题；平台 flags 描述硬件。
 * Logical fault bits. These are a separate bit space from
 * foc_platform_diagnostics_t.flags: this set is raised by the Rust controller
 * and describes the control algorithm, while the platform flags describe the
 * hardware.
 */
/* 算法算出的占空比非有限或超出 [0,1]，已立即停止输出。
 * The algorithm produced a non-finite or out-of-range duty; output stopped. */
#define FOC_RUST_FAULT_ALGORITHM_OUTPUT  (1UL << 0)
/* 反馈含非有限值或母线电压 <= 0，快环拒绝使用该快照。
 * Feedback contained non-finite values or Vbus <= 0; the snapshot was
 * rejected. */
#define FOC_RUST_FAULT_INVALID_FEEDBACK  (1UL << 1)
/* 在 acquisition_timeout 内观测器未达到可靠性门，启动失败。
 * The observer did not reach its reliability gate within
 * observer_acquisition_timeout_s. */
#define FOC_RUST_FAULT_OBSERVER_STARTUP  (1UL << 2)
/* 闭环运行中观测器连续失锁超过 observer_loss_timeout_s。
 * The observer stayed unlocked for longer than observer_loss_timeout_s while
 * closed loop was active. */
#define FOC_RUST_FAULT_OBSERVER_LOST     (1UL << 3)

/*
 * 观测器后端选择。数值与 ABI 绑定，只能追加不能重排。
 * Observer backend selection. The values are part of the ABI; only append.
 */
typedef uint32_t foc_observer_backend_t;
enum
{
    /* 滑模观测器 + PLL，默认后端，全浮点，已在本工程实机验证。
     * Sliding-mode observer + PLL; default backend, all-float, hardware
     * validated in this project. */
    FOC_OBSERVER_SMO_PLL = 0,
    /* 浮点反电动势 + PLL，与 MCSDK STO-PLL 同拓扑，便于主机实验。
     * Float BEMF + PLL, same topology as MCSDK STO-PLL, for host experiments. */
    FOC_OBSERVER_BEMF_PLL = 1,
    /* ST 定点 STO-PLL 预留位；当前代码不提供逐位等价实现，
     * is_reliable() 对其恒返回 false，禁止用于闭环。
     * Reserved for the exact ST fixed-point STO-PLL. No bit-equivalent
     * implementation exists here and is_reliable() returns false for it, so it
     * must not be used to close the loop. */
    FOC_OBSERVER_ST_STO_PLL = 2,
};

/* 默认 SMO 可靠性门的分解诊断位。控制决策仍只使用 observer_reliable；这些位
 * 只用于说明一次失锁到底卡在哪道门，不能由 C 侧反向改变 Rust 状态机。
 * Decomposed default-SMO reliability gates. They are diagnostic only and must
 * never be fed back into the Rust state machine by C. */
#define FOC_OBSERVER_GATE_WINDOW_READY           (1UL << 0)
#define FOC_OBSERVER_GATE_SPEED_FINITE           (1UL << 1)
#define FOC_OBSERVER_GATE_SPEED_ABOVE_MINIMUM    (1UL << 2)
#define FOC_OBSERVER_GATE_SPEED_BELOW_MAXIMUM    (1UL << 3)
#define FOC_OBSERVER_GATE_BEMF_ABOVE_MINIMUM     (1UL << 4)
#define FOC_OBSERVER_GATE_VARIANCE_BELOW_MAXIMUM (1UL << 5)
#define FOC_OBSERVER_GATE_PHASE_ERROR_BELOW_MAXIMUM (1UL << 6)
#define FOC_OBSERVER_GATE_ALL                     (0x7FUL)

/*
 * C 提供的控制器存储。
 * Controller storage provided by C.
 *
 * 用 union 而不是裸数组：alignment 成员强制 8 字节对齐，使 Rust 侧可以
 * 在其中原位构造对齐要求更高的控制器结构。容器本身不含任何状态。
 * A union rather than a bare array: the alignment member forces 8-byte
 * alignment so Rust can construct a controller with stricter alignment inside.
 * The container itself holds no state.
 *
 * 生命周期 / Lifetime: 必须是静态存储期或至少活过整个控制会话。
 * 控制器持有指针语义的状态，容器被回收后任何调用都是未定义行为。
 * Must have static storage duration, or at least outlive the control session;
 * the controller keeps state by address and any call after the container is
 * released is undefined behaviour.
 */
typedef union
{
    uint64_t alignment;
    uint8_t bytes[FOC_RUST_CONTEXT_CAPACITY];
} foc_rust_context_t;

/*
 * 单个 PI 调节器配置。
 * Configuration for one PI regulator.
 *
 * 量纲 / Units: kp [V/A]（或 [A/(rad/s)] 用于速度环），ki [V/(A*s)]，
 * ts [s] 为该环的实际执行周期。
 * kp is in V/A (or A/(rad/s) for the speed loop), ki in V/(A*s), and ts is the
 * actual execution period of that loop in seconds.
 *
 * 关于 ki 与 ts 的约定（重要）/ Convention for ki and ts (important):
 *   `ki` 是**连续时间**积分增益，离散化由算法侧 `ki * ts * error` 完成。
 *   因此改变控制频率只需改 `ts`，`ki` 不需要重算。
 *   `ki` is a CONTINUOUS-TIME integral gain; the算法 side discretises it as
 *   `ki * ts * error`. Changing the control frequency therefore only changes
 *   `ts`; `ki` does not need to be recomputed.
 *
 * 饱和与抗积分饱和 / Saturation and anti-windup:
 *   out_min/out_max 限制输出电压；integrator_min/max 限制积分器状态。
 *   输出饱和时算法会回算积分器，避免继续积分。
 *   out_min/out_max bound the output voltage, integrator_min/max bound the
 *   integrator state, and the algorithm back-calculates the integrator when the
 *   output saturates so it cannot keep winding up.
 */
typedef struct
{
    float kp;
    float ki;
    float ts;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
} foc_pi_config_t;

/*
 * 旧版基础配置（Id/Iq 双 PI + 标称母线电压）。
 * Legacy basic configuration (Id/Iq PI pair plus nominal bus voltage).
 *
 * 仅供兼容入口 foc_rust_configure_basic() 使用；当前实时路径使用
 * foc_runtime_config_t。保留它是为了让早期主机测试与独立调用者仍可编译。
 * Retained for the compatibility entry point foc_rust_configure_basic() only.
 * The realtime path uses foc_runtime_config_t; this exists so early host tests
 * and standalone callers still compile.
 */
typedef struct
{
    foc_pi_config_t id_pi;
    foc_pi_config_t iq_pi;
    float nominal_dc_bus_voltage;
} foc_basic_config_t;

/*
 * 观测器接管后的运行期可靠性配置。获取相位门在 V17 中已拆成顶层独立字段，
 * 避免为了抗运行纹波而同时放宽接管资格。
 * Observer retention settings after handover. V17 separates the acquisition
 * phase gate so runtime ripple tolerance cannot silently relax handover.
 */
typedef struct
{
    /* 观测器已接管后允许的最低估计转速 [rpm]。它可以低于启动接管门限，但不能
     * 高于 observer_minimum_speed_rpm。ST 参考工程对应 MIN_APPLICATION_SPEED_RPM。
     * Minimum estimated speed after handover [rpm]. It may be lower than, but not
     * exceed, the acquisition threshold. This corresponds to MCSDK's
     * MIN_APPLICATION_SPEED_RPM. */
    float minimum_speed_rpm;
    /* 接管后运行保持允许的最大原始包角相位误差 [rad]，合法范围 (0, pi/2]。
     * Maximum raw wrapped phase error retained after handover. */
    float maximum_phase_error_rad;
} foc_observer_run_reliability_config_t;

/*
 * Park/逆 Park 执行延迟补偿，单位是当前电流控制拍 [control ticks]。
 * Park/inverse-Park execution-delay compensation in current-control ticks.
 *
 * 两个字段都是相对“补偿前基础控制角”的绝对预测量，不使用 MCSDK 的累加系数
 * 语义。合法范围 -2.0..=2.0；默认 0/0 与实际 ST 参考工程一致。
 * Both values are absolute predictions from the uncompensated base angle, not
 * cumulative MCSDK factors. Valid range is -2.0..=2.0; default 0/0 matches the
 * actual ST reference project.
 */
typedef struct
{
    float park_prediction_ticks;
    float reverse_park_prediction_ticks;
} foc_angle_compensation_config_t;

/*
 * 逆变器平均电压模型的目标配置。总门与两个使用点分开，允许按
 * “只开观测器修正 -> 再独立评估 PWM 前馈”的顺序做 A/B。
 * Target configuration for the average inverter-voltage model. The master gate
 * and the two consumers are separate so observer correction can be evaluated
 * before PWM feed-forward.
 *
 * pwm_carrier_frequency_hz 是实际载波频率，不是历史字段
 * foc_runtime_config_t.pwm_frequency_hz 表示的控制 ISR 频率；24/12 kHz 多速率时
 * 两者必须分别为 24000/12000。
 * pwm_carrier_frequency_hz is the applied carrier, not the control ISR rate.
 */
typedef struct
{
    uint32_t enabled;
    uint32_t observer_voltage_correction_enable;
    uint32_t pwm_feedforward_enable;
    uint32_t pwm_carrier_frequency_hz;
    float dead_time_s;
    float compensation_gain;
    float current_zero_band_a;
    float current_sign_filter_alpha;
    float device_drop_v;
} foc_inverter_voltage_model_config_t;

/*
 * 完整运行时配置。这是 C 侧唯一需要理解的"业务级"结构体。
 * Full runtime configuration; the only "business level" struct C must know.
 *
 * 前两个字段是自检字段：Rust 会比对 struct_size 与 config_version，不匹配
 * 直接拒绝配置，避免把旧结构体的字节当作新字段解释。
 * The first two fields are self-check fields: Rust compares struct_size and
 * config_version and rejects the configuration on mismatch, preventing old
 * bytes from being interpreted as new fields.
 *
 * 参数来源标注 / Provenance of the defaults:
 *   [ST] 来自 MCSDK 6.4.1 参考工程（见 docs/ST_MCSDK参考参数与仿真.md）
 *   [HW] 在本套件上实测 / measured on this kit
 *   未实测辨识的值（Rs、Ls、磁链、惯量）必须视为起点，不是最终整定值。
 *   Values that were never identified on the physical motor (Rs, Ls, flux
 *   linkage, inertia) are starting points, not final tuning.
 */
typedef struct
{
    /* 结构体尺寸自检，必须等于 sizeof(foc_runtime_config_t)。
     * Struct size self-check; must equal sizeof(foc_runtime_config_t). */
    uint32_t struct_size;
    /* 配置版本自检，必须等于 FOC_RUST_CONFIG_VERSION。
     * Config version self-check; must equal FOC_RUST_CONFIG_VERSION. */
    uint32_t config_version;
    /* 观测器后端，取值见 foc_observer_backend_t。/ Observer backend. */
    foc_observer_backend_t observer_backend;
    /* 是否每拍更新观测器（0/1）。关掉则纯遥测。/ Update observer each tick (0/1). */
    uint32_t observer_enable;
    /* 是否允许观测器接管角度（0/1）。上电默认必须为 0。
     * Allow the observer to take over the angle (0/1). Must default to 0. */
    uint32_t closed_loop_enable;
    /* 观测器分频：1 = 每拍更新，>1 = 每 N 拍更新一次。
     * Observer divider: 1 = every tick, >1 = every N ticks. */
    uint32_t observer_update_divider;
    /* Id 电流环 / Id current loop. */
    foc_pi_config_t id_pi;
    /* Iq 电流环 / Iq current loop. */
    foc_pi_config_t iq_pi;
    /* 速度环，执行频率由 speed_loop_frequency_hz 决定。
     * Speed loop; its rate is speed_loop_frequency_hz. */
    foc_pi_config_t speed_pi;
    /* 极对数，影响电角度与机械转速的换算 [--] / Pole pairs. */
    uint32_t pole_pairs;
    /* 电流环执行频率 [Hz]，必须与平台 ISR 频率一致，且是
     * speed_loop_frequency_hz 的整数倍。
     * Current loop rate [Hz]. Must match the platform ISR rate and be an
     * integer multiple of speed_loop_frequency_hz. */
    uint32_t pwm_frequency_hz;
    /* 速度环执行频率 [Hz] / Speed loop rate [Hz]. */
    uint32_t speed_loop_frequency_hz;
    /* 定子电阻 [ohm] [ST]，未在本电机上辨识 / Stator resistance, not identified. */
    float stator_resistance_ohm;
    /* 定子电感 [H] [ST]，未在本电机上辨识；当前按 Ld = Lq 处理。
     * Stator inductance [H], not identified; Ld = Lq is assumed. */
    float stator_inductance_h;
    /* 永磁磁链 [Wb] [ST]，由 Workbench 内部角基准换算而来。
     * Permanent-magnet flux linkage [Wb], converted from the Workbench internal
     * angular basis. */
    float flux_linkage_wb;
    /* 额定电流 [A] [ST]，同时作为速度环输出上限和 SMO 边界层基准。
     * Rated current [A]; also caps the speed loop output and scales the SMO
     * boundary layer. */
    float rated_current_a;
    /* 最高机械转速 [rpm] [ST]，用于参数合法性检查。
     * Maximum mechanical speed [rpm], used for parameter validation. */
    float max_speed_rpm;
    /* 标称母线电压 [V] / Nominal bus voltage [V]. */
    float nominal_bus_voltage_v;
    /* 电压利用率 [--]：可用于调制的母线比例，圆限幅按
     * utilization * Vbus / sqrt(3) 计算。
     * Voltage utilisation: the fraction of the bus available to modulation. The
     * circle limit uses utilization * Vbus / sqrt(3). */
    float voltage_utilization;
    /* 默认目标转速 [rpm] / Default target speed [rpm]. */
    float default_target_speed_rpm;
    /* 定向时长 [s] [ST] / Alignment duration [s]. */
    float alignment_duration_s;
    /* 开环升速时长 [s] [ST] / Open-loop ramp duration [s]. */
    float open_loop_ramp_duration_s;
    /* 观测器交接时长 [s] [ST] / Observer handover duration [s]. */
    float observer_transition_duration_s;
    /* 升速终点转速 [rpm] [ST] / Rev-up final speed [rpm]. */
    float startup_final_speed_rpm;
    /* 对齐结束 q 轴电流 [A]；升速阶段平滑过渡到 startup_current_a。
     * Alignment-end q-axis current [A]; the ramp tapers to startup_current_a. */
    float startup_alignment_current_a;
    /* 升速与保持段电流 [A] [ST] / Rev-up and hold current [A]. */
    float startup_current_a;
    /* SMO 滑模增益 [V]。参考实现取 nominal_bus * 0.9，对低压小电流电机偏大，
     * 实测该值过大会导致观测器无法锁定。
     * SMO sliding gain [V]. The reference form is nominal_bus * 0.9, which is
     * far too large for a low-voltage low-current machine and prevents lock. */
    float observer_smo_k_slide_v;
    /* SMO 边界层 [A]，决定滑模项的等效增益 k/boundary。
     * SMO boundary layer [A]; sets the effective gain k/boundary. */
    float observer_smo_boundary_a;
    /* 反电动势提取低通系数 [--]。该值过小会让 SMO 输出相位滞后大于电频率，
     * 是观测器失锁的主要诱因之一。
     * BEMF extraction low-pass coefficient. Too small a value makes the SMO
     * output lag more than the electrical frequency and is a leading cause of
     * observer loss of lock. */
    float observer_emf_filter_alpha;
    /* PLL 比例增益 [rad/s per rad] / PLL proportional gain. */
    float observer_pll_kp;
    /* 强拖捕获阶段的 PLL Kp/Ki 校正比例 [--]，合法范围 (0, 1]；终速前馈不缩放。
     * Acquisition PLL Kp/Ki correction ratio; forced-speed feed-forward remains. */
    float observer_acquisition_pll_kp_ratio;
    /* PLL 积分增益 [rad/s^2 per rad] / PLL integral gain. */
    float observer_pll_ki;
    /* 可靠性门：最低估计转速 [rpm]，低于此值不判可靠。
     * Reliability gate: minimum estimated speed [rpm]. */
    float observer_minimum_speed_rpm;
    /* 启动获取窗允许的平均绝对包角相位误差 [rad]，合法范围 (0, pi/2]。
     * Maximum mean absolute wrapped phase error for acquisition. */
    float observer_acquisition_maximum_phase_error_rad;
    /* 可靠性门：最低反电动势幅值 [V]，避免在零速噪声上判可靠。
     * Reliability gate: minimum BEMF magnitude [V]. */
    float observer_minimum_bemf_v;
    /* 可靠性门：转速方差上限，为均方值的比例 [--]。
     * Reliability gate: speed variance ceiling as a fraction of mean squared. */
    float observer_speed_variance_ratio;
    /* 可靠性门：需要连续通过的评估窗口数。
     * Reliability gate: consecutive passing windows required. */
    uint32_t observer_consecutive_samples;
    /* 启动期获取超时 [s]，超时则锁存 FOC_RUST_FAULT_OBSERVER_STARTUP。
     * Acquisition timeout [s]; latches FOC_RUST_FAULT_OBSERVER_STARTUP. */
    float observer_acquisition_timeout_s;
    /* 闭环期失锁超时 [s]，超时则锁存 FOC_RUST_FAULT_OBSERVER_LOST。
     * Loss-of-lock timeout [s]; latches FOC_RUST_FAULT_OBSERVER_LOST. */
    float observer_loss_timeout_s;
    /* 闭环接管时目标转速的斜坡速率 [rpm/s]，避免单周期阶跃。
     * Target speed ramp rate on handover [rpm/s], avoiding a single-cycle step. */
    float closed_loop_speed_ramp_rpm_per_s;
    /* 无感接管转矩支撑比例 [--]；非零时终点为强拖电流乘该比例。
     * Sensorless handoff torque-support ratio. */
    float handoff_torque_support_ratio;
    /* 速度环积分器预装比例 [--]；只改变 PI 状态，不改变接管转矩终点。
     * Speed-loop integrator preload ratio; independent of handoff torque. */
    float speed_pi_preload_ratio;
    /* 闭环时 Iq 指令的限速器 [A/s]，抑制单周期电流阶跃。
     * Iq command slew limiter in closed loop [A/s]. */
    float closed_loop_current_slew_a_per_s;
    /* 运行期观测器保持门 / Closed-loop observer-retention gate. */
    foc_observer_run_reliability_config_t observer_run_reliability;
    /* Park/逆 Park 执行延迟补偿 / Park/inverse-Park delay compensation. */
    foc_angle_compensation_config_t angle_compensation;
    /* 逆变器平均电压模型；上电默认总门和两个出口均为 0。
     * Average inverter-voltage model; all gates default to zero. */
    foc_inverter_voltage_model_config_t inverter_voltage_model;
} foc_runtime_config_t;

/*
 * 一次快环调用的遥测快照，供 Shell 与 trace 使用。
 * Telemetry snapshot of one fast-loop call, for the shell and trace.
 *
 * 三个角度字段刻意分开，不能混为一个："控制角"是执行延迟补偿前的基础角，
 * 它可能来自强制角、观测角或两者的混合。
 * The three angle fields are deliberately distinct. `electrical_angle_rad` is
 * the base angle before delay compensation and may come from the forced angle,
 * the observer, or a blend of both.
 *
 * 单位 / Units: 角度 [rad]，转速 [rpm]，电流 [A]，电压 [V]。
 */
typedef struct
{
    /* 控制器逻辑状态，取值见 foc_state_t。/ Logical controller state. */
    foc_state_t state;
    /* 本次生效的观测器后端 / Backend in effect for this step. */
    foc_observer_backend_t observer_backend;
    /* 观测器是否已通过可靠性门（0/1）/ Observer passed its reliability gate. */
    uint32_t observer_reliable;
    /* 观测器是否正在接管角度（0/1）/ Observer currently owns the angle. */
    uint32_t closed_loop_active;
    /* 目标机械转速 [rpm] / Target mechanical speed [rpm]. */
    float target_speed_rpm;
    /* 观测器估计的机械转速 [rpm]。注意这是**估计值**，不是真值测量。
     * Observer-estimated mechanical speed [rpm]. This is an estimate, not a
     * measured ground truth. */
    float measured_speed_rpm;
    /* 本次 Park/逆 Park 延迟补偿前的基础电角度 [rad]。
     * Base electrical angle before Park/inverse-Park delay compensation [rad]. */
    float electrical_angle_rad;
    /* Id/Iq 给定 [A] / Id and Iq references [A]. */
    float id_reference_a;
    float iq_reference_a;
    /* Id/Iq 实测（Park 之后）[A] / Measured Id and Iq after Park [A]. */
    float id_measured_a;
    float iq_measured_a;
    /* 电流环输出的 dq 电压指令（圆限幅之后）[V]。
     * dq voltage command out of the current loop, after the circle limit [V]. */
    float vd_command_v;
    float vq_command_v;
    /* Rev-Up 强制电角度 [rad] / Rev-up forced electrical angle [rad]. */
    float forced_electrical_angle_rad;
    /* 观测器估计电角度 [rad] / Observer-estimated electrical angle [rad]. */
    float observer_electrical_angle_rad;
    /* 观测器滤波后的 α/β 反电势 [V]。幅值由主机离线求平方根，避免 ISR 增加开方。
     * Filtered alpha/beta BEMF [V]. Magnitude is derived on the host to avoid an
     * extra square root in the ISR. */
    float observer_bemf_alpha_v;
    float observer_bemf_beta_v;
    /* PLL 最短路径鉴相误差 [rad] / PLL shortest-path phase error [rad]. */
    float observer_pll_phase_error_rad;
    /* 最近一次 1 kHz/64 ms 可靠性窗口的速度均值与方差 [rpm], [rpm^2]。 */
    float observer_speed_mean_rpm;
    float observer_speed_variance_rpm2;
    /* 分解门控位与连续通过窗口数 / Decomposed gate bits and passing streak. */
    uint32_t observer_reliability_flags;
    uint32_t observer_reliable_samples;
    /* 等待收敛与闭环连续失锁计时 [s]。 */
    float observer_wait_elapsed_s;
    float observer_loss_elapsed_s;
    /* 本拍 dq 电压是否触发圆限幅（0/1）/ Voltage-circle limiter active. */
    uint32_t voltage_limited;
} foc_telemetry_t;

/* ---------------------------------------------------------------- 版本查询 */

/* 返回编译期 ABI 版本；C 侧启动时与 FOC_RUST_ABI_VERSION 比对。
 * Returns the compiled ABI version; C compares it at boot. */
uint32_t foc_rust_abi_version(void);
/* 返回控制器结构体的实际尺寸 [bytes]，C 侧用于确认存储够用。
 * Returns the actual controller size [bytes]; C uses it to confirm capacity. */
uint32_t foc_rust_context_required_size(void);
/* 返回控制器结构体的对齐要求 [bytes] / Returns the controller alignment. */
uint32_t foc_rust_context_required_align(void);

#if defined(FLUXRT_MATH_DIAGNOSTICS_BUILD) && \
    defined(FOC_MATH_CPU_FAST_APPROX_BENCHMARK)
/* 停机数学基准专用：直接调用 Rust CPU 快速近似。它们不属于实时控制输入/输出，
 * Production 不声明也不链接这两个符号。
 * Stopped-state math benchmark only: calls the Rust CPU fast approximation
 * directly. Production neither declares nor links these symbols. */
uint32_t foc_rust_fast_math_sin_cos(float angle_rad,
                                    float *sin_out,
                                    float *cos_out);
uint32_t foc_rust_fast_math_atan2(float y, float x, float *angle_out);
#endif

/* -------------------------------------------------------------- 生命周期 */

/*
 * 在 C 提供的存储中原地构造控制器，不分配内存。
 * Constructs the controller in place in C-provided storage; no allocation.
 *
 * 上下文 / Context: 初始化阶段调用一次，任何实时调用之前。
 * Call once during initialization, before any realtime call.
 * 返回 / Returns: FOC_STATUS_OK，或指针为空时 FOC_STATUS_INVALID_ARGUMENT。
 */
foc_status_t foc_rust_init(foc_rust_context_t *context);

/*
 * 用当前基线填充一份完整的运行时配置。
 * Fills a complete runtime configuration with the current baseline.
 *
 * 这是 C 侧获取默认参数的正确方式：先取默认值再覆盖需要改的字段，避免
 * 手工填写 30 多个字段时漏填导致合法性检查失败。
 * This is how C should obtain defaults: take them, then override only what
 * needs changing, rather than hand-filling 30+ fields and failing validation.
 */
foc_status_t foc_rust_default_st_config(foc_runtime_config_t *config);

/* 对一份完整且合法的运行时配置计算规范化 CRC-32/ISO-HDLC。
 * Computes a canonical CRC-32/ISO-HDLC over a complete valid runtime config.
 *
 * 每个 ABI 字段按小端字节序输入，不读取结构体填充字节。该 CRC 只用于
 * 证明参数档与编译固件中的实际配置逐位一致，不代表参数已辨识或
 * 已完成实机验收。任一指针为空或配置不合法时返回 INVALID_ARGUMENT。
 * The checksum proves exact parameter identity only; it is not an approval.
 */
foc_status_t foc_rust_runtime_config_crc32(const foc_runtime_config_t *config,
                                           uint32_t *crc_out);

/*
 * 应用运行时配置。校验失败时不改变控制器状态。
 * Applies a runtime configuration; on validation failure no state changes.
 *
 * 校验范围包括：struct_size / config_version、枚举取值、所有 PI 参数有限且
 * 区间合法、频率关系（速度环整除电流环）、电机参数为正、以及启动时序合法。
 * Validation covers struct_size / config_version, enum values, finite and
 * consistent PI parameters, the frequency relationship (speed loop divides the
 * current loop), positive motor parameters, and a valid start-up sequence.
 *
 * 调用约束 / Constraint: 必须在停机状态调用；运行中改配置会被拒绝。
 * Must be called while stopped; changes during a run are rejected.
 */
foc_status_t foc_rust_configure(foc_rust_context_t *context,
                                const foc_runtime_config_t *config);

/*
 * 兼容入口：应用旧版基础配置。
 * Compatibility entry point: applies the legacy basic configuration.
 *
 * 仅供早期主机测试与独立调用者使用；实时路径请用 foc_rust_configure()。
 * For early host tests and standalone callers only; the realtime path should
 * use foc_rust_configure().
 */
foc_status_t foc_rust_configure_basic(foc_rust_context_t *context,
                                      const foc_basic_config_t *config);

/*
 * 兼容入口：装入 ST MCSDK 参考参数并保持停机。
 * Compatibility entry point: loads the ST MCSDK reference parameters, stopped.
 *
 * 等价于 default_st_config() + configure()，保留是为了让既有测试不退化和
 * 让"只想要参考参数"的调用者少写一步。
 * Equivalent to default_st_config() plus configure(); kept so existing tests do
 * not regress and callers that only want the reference parameters write less.
 */
foc_status_t foc_rust_configure_st_reference(foc_rust_context_t *context);

/*
 * 进入遗留的 RUNNING 状态（不含启动时序）。
 * Enters the legacy RUNNING state (no start-up sequencing).
 *
 * platform_ready 为 0 时拒绝并保持 DISABLED。该参数不是装饰：C 平台没有
 * 准备好时，即使 Rust 参数完整也不允许进入运行。
 * Refuses and stays DISABLED when platform_ready is 0. That parameter is not
 * decorative: when the C platform is not ready, complete Rust parameters still
 * do not permit a run.
 *
 * 当前实时路径使用 foc_rust_start_realtime()；本函数保留用于兼容测试。
 * The realtime path uses foc_rust_start_realtime(); this is kept for
 * compatibility tests.
 */
foc_status_t foc_rust_request_start(foc_rust_context_t *context,
                                    uint32_t platform_ready);

/* -------------------------------------------------------------- 实时快环 */

/*
 * 启动 ISR 持有的启动时序 + 电流控制运行实例。
 * Starts the ISR-owned start-up sequencer plus current-control runtime.
 *
 * 与 request_start 的区别：本函数复位观测器、启动时序、电流环和速度环，
 * 并把状态置为 ALIGNMENT，随后由实时步推进整条启动链。
 * Unlike request_start, this resets the observer, the sequencer, the current
 * loop and the speed loop, sets the state to ALIGNMENT, and lets the realtime
 * step drive the whole start-up chain.
 *
 * 调用约束 / Constraint: 必须在 ADC ISR 被使能之前调用，且调用期间不得有
 * 其他上下文访问该 controller。
 * Must be called before the ADC ISR is enabled, with no other context touching
 * the controller during the call.
 *
 * 参数 / Parameters:
 *   target_speed_rpm - 闭环目标转速 [rpm]，必须 >= 升速终点且 <= 最高转速
 *                      Closed-loop target [rpm]; must be between the rev-up
 *                      endpoint and max_speed_rpm.
 */
foc_status_t foc_rust_start_realtime(foc_rust_context_t *context,
                                     uint32_t platform_ready,
                                     float target_speed_rpm);

/*
 * 执行一次完整快环：观测器 -> 启动时序 -> 电流环 -> 圆限幅 -> SVPWM。
 * Runs one complete fast loop: observer, sequencer, current loop, circle limit,
 * SVPWM.
 *
 * C 的 ADC 中断是**唯一**调用者，因此在功率级 arm 期间它独占控制器状态。
 * 这是本工程的并发模型：不做锁，靠"单一调用者"保证一致性。
 * The C ADC interrupt is the SOLE caller, so it exclusively owns the controller
 * state while the power stage is armed. That is this project's concurrency
 * model: no locks, single-caller discipline instead.
 *
 * 关键语义 / Key semantics:
 *   - 任何错误返回之前都把 output 清零；调用方不得沿用上一周期的占空比。
 *     Every error return zeroes `output`; the caller must never reuse the
 *     previous period's duty.
 *   - 输出非法时锁存 FOC_RUST_FAULT_ALGORITHM_OUTPUT 并进入 FAULT。
 *     An invalid output latches FOC_RUST_FAULT_ALGORITHM_OUTPUT and enters
 *     FAULT.
 *   - telemetry 可以为 NULL；非 NULL 时每拍被完整覆盖。
 *     `telemetry` may be NULL; when non-NULL it is fully overwritten each tick.
 *
 * 实时约束 / Real-time constraints: 本函数在 12 kHz ISR 中执行，无动态分配、
 * 无阻塞、无日志。实测单次调用是 ISR 的主要开销（约 7,300 cycles @170 MHz）。
 * Executes inside the 12 kHz ISR with no allocation, blocking or logging. It
 * dominates the ISR cost (about 7,300 cycles at 170 MHz).
 */
foc_status_t foc_rust_realtime_step(foc_rust_context_t *context,
                                    const foc_feedback_t *feedback,
                                    foc_output_t *output,
                                    foc_telemetry_t *telemetry);

/*
 * 复制最近一次实时遥测快照。
 * Copies the last realtime telemetry snapshot.
 *
 * 并发要求（重要）/ Concurrency requirement (important):
 *   实现是一次普通的多字段结构体赋值，**没有**临界区、原子拷贝或 seqlock。
 *   因此若功率级已 arm（ISR 正在每拍更新 telemetry），本调用可能与 ISR
 *   交错并读到**字段年龄不一致**的快照。
 *   Rust 侧不做保护是刻意的：在 12 kHz 快环里关中断比"读到一个混合快照"
 *   代价更高。**互斥由调用方负责** —— 当前 in-tree 没有调用者。
 *   The implementation is a plain multi-field struct assignment with NO critical
 *   section, no atomic copy and no seqlock. While the power stage is armed (the
 *   ISR updates telemetry every tick) a call may therefore interleave with the
 *   ISR and return a snapshot whose fields have different ages.
 *   Rust deliberately does not protect this: masking interrupts inside the
 *   12 kHz fast loop costs more than occasionally reading a mixed snapshot.
 *   Mutual exclusion is the CALLER's responsibility. There is currently no
 *   in-tree caller.
 *
 * 建议用法 / Recommended usage: 停机状态下读取，或由调用方自行关中断后再调用。
 * Read while stopped, or have the caller mask interrupts around the call.
 */
foc_status_t foc_rust_get_telemetry(foc_rust_context_t *context,
                                    foc_telemetry_t *telemetry);

/*
 * 停机：复位电流环、速度环、观测器与启动时序，状态回到 DISABLED。
 * Stops: resets current loop, speed loop, observer and sequencer; state returns
 * to DISABLED.
 *
 * 本函数**不接触硬件**，栅极使能与 PWM 通道由 C 平台层负责关闭。
 * This function does NOT touch hardware; the C platform layer is responsible
 * for the gate enables and PWM channels.
 */
void foc_rust_stop(foc_rust_context_t *context);

/* ------------------------------------------------------------ 兼容/测试入口 */

/*
 * 兼容入口：给定 Id/Iq 给定跑一次电流环（不做启动时序、不跑观测器）。
 * Compatibility entry point: runs one current-loop step for a given Id/Iq
 * reference, with no sequencer and no observer.
 *
 * 仅供主机测试与仿真使用；实时路径请用 foc_rust_realtime_step()。
 * For host tests and simulation only; the realtime path uses
 * foc_rust_realtime_step().
 */
foc_status_t foc_rust_fast_step(foc_rust_context_t *context,
                                const foc_feedback_t *feedback,
                                const foc_reference_t *reference,
                                foc_output_t *output);

/*
 * 兼容入口：开环电压矢量步进，用于早期 bring-up 试验。
 * Compatibility entry point: one open-loop voltage vector step, used by the
 * early bring-up trial.
 *
 * 电压幅值被强制限制在母线的 8% 以内，是刻意留给调试的保守包络。
 * The voltage magnitude is capped at 8% of the bus, a deliberately保守 debug
 * envelope.
 */
foc_status_t foc_rust_open_loop_step(foc_rust_context_t *context,
                                     float electrical_speed_rad_s,
                                     float voltage_magnitude_v,
                                     float dc_bus_voltage_v,
                                     float sample_time_s,
                                     foc_output_t *output);

/*
 * 兼容入口：单独跑一次速度环，返回 Iq 给定。
 * Compatibility entry point: runs the speed loop alone and returns the Iq
 * reference.
 *
 * 主机测试使用；实时路径由 realtime_step 内部按分频调用速度环。
 * Used by host tests; the realtime path calls the speed loop internally at its
 * divided rate.
 */
foc_status_t foc_rust_speed_step(foc_rust_context_t *context,
                                 float target_speed_rpm,
                                 float measured_speed_rpm,
                                 foc_reference_t *reference);

/* ---------------------------------------------------------------- 故障管理 */

/*
 * 锁存故障位并进入 FAULT 状态。
 * Latches fault bits and enters FAULT.
 *
 * C 平台在检测到硬件故障并已关断 PWM 之后调用。Rust 侧不再尝试恢复控制，
 * 复位必须由上层显式 foc_rust_clear_fault()。
 * Called by C after a hardware fault has been detected and PWM disabled. Rust
 * does not attempt recovery; clearing is an explicit upper-layer action.
 */
foc_status_t foc_rust_latch_fault(foc_rust_context_t *context,
                                  uint32_t fault_flags);

/*
 * 清除逻辑故障并回到 DISABLED。
 * Clears logical faults and returns to DISABLED.
 *
 * 调用前硬件故障必须已经由 C 平台层检查并清除，否则会立刻重新锁存。
 * Hardware faults must already have been checked and cleared by the C platform
 * layer, otherwise the fault is immediately re-latched.
 */
foc_status_t foc_rust_clear_fault(foc_rust_context_t *context);

/*
 * 查询逻辑状态。上下文为空或未初始化时返回 FOC_STATE_UNINITIALIZED。
 * Queries the logical state; returns FOC_STATE_UNINITIALIZED when the context
 * is null or uninitialized.
 */
foc_state_t foc_rust_state(foc_rust_context_t *context);

/*
 * 查询已锁存的故障位。
 * Queries the latched fault bits.
 */
uint32_t foc_rust_fault_flags(foc_rust_context_t *context);

#ifdef __cplusplus
}
#endif

/*
 * ABI 尺寸编译期自检。
 * Compile-time ABI size self-checks.
 *
 * 这些断言是本项目防止 C/Rust 静默错位的**第一道防线**：任何一侧改了字段
 * 而忘了同步，构建会立刻失败，而不是在板上表现为难以定位的数值错误。
 * These assertions are the first line of defence against silent C/Rust
 * misalignment: if either side changes a field without syncing, the build fails
 * immediately instead of producing hard-to-locate numeric errors on the board.
 *
 * 只有 C11 及以上提供 _Static_assert，C++ 编译时跳过。
 * _Static_assert requires C11 or later; C++ compilation skips the block.
 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_status_t) == sizeof(uint32_t), "FOC status ABI changed");
_Static_assert(sizeof(foc_state_t) == sizeof(uint32_t), "FOC state ABI changed");
_Static_assert(sizeof(foc_feedback_t) == 20U, "FOC feedback ABI changed");
_Static_assert(sizeof(foc_reference_t) == 8U, "FOC reference ABI changed");
_Static_assert(sizeof(foc_output_t) == 12U, "FOC output ABI changed");
_Static_assert(sizeof(foc_basic_config_t) == 60U, "FOC config ABI changed");
_Static_assert(sizeof(foc_observer_run_reliability_config_t) == 8U,
               "FOC observer run reliability ABI changed");
_Static_assert(sizeof(foc_angle_compensation_config_t) == 8U,
               "FOC angle compensation config ABI changed");
_Static_assert(sizeof(foc_inverter_voltage_model_config_t) == 36U,
               "FOC inverter model config ABI changed");
_Static_assert(sizeof(foc_runtime_config_t) == 296U, "FOC runtime config ABI changed");
_Static_assert(sizeof(foc_telemetry_t) == 100U, "FOC telemetry ABI changed");
_Static_assert(sizeof(foc_rust_context_t) == FOC_RUST_CONTEXT_CAPACITY,
               "Rust context storage ABI changed");
#endif

#endif
