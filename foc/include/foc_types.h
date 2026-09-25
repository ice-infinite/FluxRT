#ifndef FOC_TYPES_H
#define FOC_TYPES_H

/*
 * FluxRT —— C / Rust 共用定宽 ABI 数据模型（无逻辑，只有类型）。
 * FluxRT - fixed-width ABI data model shared by C and Rust (types only).
 *
 * 职责 / Responsibility:
 *   - 定义跨 C/Rust 边界的定宽状态码、状态枚举和传输结构体；
 *   - 本头文件由 C 和 Rust 两侧共同遵守，是 ABI 契约的一部分。
 *
 * 设计约束 / Design constraints:
 *   - 状态码与状态使用固定 uint32_t，不使用 C 短枚举。
 *     ARM GCC 默认短枚举与 Rust `#[repr(u32)]` 宽度不一致，混用会让
 *     返回值在边界处被截断或错位，因此此处显式定宽。
 *     Status codes and states are fixed uint32_t rather than C enums: ARM GCC
 *     short enums and Rust `#[repr(u32)]` disagree in width, which would
 *     corrupt return values across the boundary.
 *   - 结构体字段顺序即 ABI 顺序，两侧必须逐字段一致；
 *     尺寸由 foc_rust_bridge.h 中的 _Static_assert 在编译期校验。
 *     Field order is ABI order and is enforced at compile time by the
 *     _Static_assert block in foc_rust_bridge.h.
 *   - 结构体只含 float，无指针、无拥有析构逻辑的类型；
 *     跨边界不传引用、String、切片或 trait object。
 *     Structs contain only float. No pointers, no owning/destructible types,
 *     no references, String, slices or trait objects cross the boundary.
 *
 * 参考 / Reference: docs/C与Rust混合架构.md §4
 */

#include <stdint.h>

/*
 * 平台与 Rust 共用的返回状态。
 * Return status shared by the platform layer and the Rust bridge.
 *
 * 该值同时是 C 侧错误分类和 Rust `#[repr(u32)] FocStatus` 的取值，
 * 因此数值本身不可重排；新增只能追加在末尾。
 * The numeric values map 1:1 onto Rust `#[repr(u32)] FocStatus`; never
 * reorder them, only append.
 */
typedef uint32_t foc_status_t;
enum
{
    /* 调用成功 / The call completed successfully. */
    FOC_STATUS_OK = 0,
    /* 控制器未使能，未写任何硬件输出 / Not armed; no hardware output was written. */
    FOC_STATUS_DISABLED,
    /* 前置条件未满足（未配置、未绑定、母线越界等）/ A precondition is unmet. */
    FOC_STATUS_NOT_CONFIGURED,
    /* 参数非法，含 NaN / Invalid argument, including NaN. */
    FOC_STATUS_INVALID_ARGUMENT,
    /* 硬件故障：调用方必须已关断功率级 / Hardware fault; the caller must have disabled the power stage. */
    FOC_STATUS_HARDWARE_FAULT,
};

/*
 * FOC 控制器逻辑状态。
 * Logical FOC controller state.
 *
 * ALIGNMENT -> OPEN_LOOP_RAMP -> OPEN_LOOP_HOLD 为强制角度启动链；
 * OBSERVER_TRANSITION -> CLOSED_LOOP 只有在观测器通过可靠性门之后才会进入。
 * ALIGNMENT, OPEN_LOOP_RAMP and OPEN_LOOP_HOLD are the forced-angle start-up
 * chain. OBSERVER_TRANSITION and CLOSED_LOOP are reachable only after the
 * observer passes its reliability gate.
 *
 * RUNNING 是早期版本遗留的“仅使能”状态，不属于当前启动链。
 * RUNNING is a legacy "enabled only" state and is not part of the current
 * start-up chain.
 */
typedef uint32_t foc_state_t;
enum
{
    /* 上下文尚未初始化 / Context has not been initialized. */
    FOC_STATE_UNINITIALIZED = 0,
    /* 已配置但未运行 / Configured but not running. */
    FOC_STATE_DISABLED,
    /* 遗留状态：仅表示已通过使能门 / Legacy: enable gate passed only. */
    FOC_STATE_RUNNING,
    /* 定向：强制角度固定，电流按斜坡上升 / Alignment: fixed forced angle, current ramps up. */
    FOC_STATE_ALIGNMENT,
    /* 开环升速：强制角度按斜坡推进 / Open-loop ramp: forced angle advances along a ramp. */
    FOC_STATE_OPEN_LOOP_RAMP,
    /* 开环保持：升速完成后维持最终强制角速度，闭环仍被禁用
     * Open-loop hold: keeps the final forced speed after the ramp; closed loop stays disabled. */
    FOC_STATE_OPEN_LOOP_HOLD,
    /* 观测器交接：强制角与观测角按进度混合，用于向闭环过渡
     * Observer handover: blends forced and estimated angle on the way to closed loop. */
    FOC_STATE_OBSERVER_TRANSITION,
    /* 闭环：角度与速度由观测器接管 / Closed loop: observer owns angle and speed. */
    FOC_STATE_CLOSED_LOOP,
    /* 故障锁存：必须由上层显式清除 / Latched fault; requires an explicit clear from above. */
    FOC_STATE_FAULT,
};

/*
 * 一个 PWM 周期内的同步反馈快照。
 * One synchronised feedback snapshot for a single PWM period.
 *
 * 三相电流必须来自同一采样时刻，不能分别读取后拼装，否则 Clarke/Park
 * 会把跨周期的数据混在一起，表现为 Id/Iq 上的低频拍频。
 * All three currents must come from one sampling instant. Reading them
 * separately and assembling them would mix across periods and show up as a
 * low-frequency beat in Id/Iq.
 *
 * 单位 / Units:
 *   phase_current_*     [A]     相电流，正方向为流入电机
 *   dc_bus_voltage      [V]     母线电压，必须 > 0，否则快环拒绝
 *   electrical_angle_rad [rad]  电角度，范围未强制，算法内部自行归一化
 */
typedef struct
{
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float dc_bus_voltage;
    float electrical_angle_rad;
} foc_feedback_t;

/*
 * Id/Iq 电流给定。
 * Id/Iq current reference.
 *
 * 单位均为 [A]。id_ref 通常为 0（表贴式电机）；弱磁或 MTPA 生效时才非零。
 * Both in [A]. id_ref is normally 0 for a surface-mounted PM; it becomes
 * non-zero only under field weakening or MTPA.
 */
typedef struct
{
    float id_ref;
    float iq_ref;
} foc_reference_t;

/*
 * 三相占空比输出（写入 CCR/ARR 的比例）。
 * Three-phase duty output (the CCR/ARR ratio written to TIM1).
 *
 * 单位 / Unit: 归一化 [-]，有效范围 [0, 1]，平台层还会按
 * `foc_platform_config_t.minimum_duty` / `maximum_duty` 二次钳位。
 * Normalised [-], nominally [0, 1]; the platform layer clamps again using
 * the configured minimum_duty / maximum_duty window.
 */
typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} foc_output_t;

#endif
