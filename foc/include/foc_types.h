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
 * V19 实时输入中的有效位。可选传感器必须用这些位区分“真实为零”和
 * “本拍没有数据”；未知位一律由 Rust 拒绝。
 * Validity bits for the V19 realtime input. Unknown bits are rejected.
 */
enum
{
    FOC_REALTIME_VALID_PHASE_CURRENTS = (1UL << 0),
    FOC_REALTIME_VALID_DC_BUS_VOLTAGE = (1UL << 1),
    FOC_REALTIME_VALID_PHASE_VOLTAGES = (1UL << 2),
    FOC_REALTIME_VALID_ELECTRICAL_ANGLE = (1UL << 3),
    FOC_REALTIME_VALID_SENSOR_TEMPERATURE = (1UL << 4),
    FOC_REALTIME_VALID_KNOWN_MASK =
        FOC_REALTIME_VALID_PHASE_CURRENTS |
        FOC_REALTIME_VALID_DC_BUS_VOLTAGE |
        FOC_REALTIME_VALID_PHASE_VOLTAGES |
        FOC_REALTIME_VALID_ELECTRICAL_ANGLE |
        FOC_REALTIME_VALID_SENSOR_TEMPERATURE,
};

/*
 * 跨 ABI 的稳定硬件故障镜像。它不是 foc_platform_diagnostics_t.flags 的副本；
 * 平台诊断位可以继续增长，而下面的位序属于 C/Rust ABI，只能追加。
 * Stable hardware-fault mirror. This is deliberately independent from the
 * platform diagnostics bitfield and may only be extended by appending bits.
 */
enum
{
    FOC_REALTIME_HW_FAULT_DRIVER = (1UL << 0),
    FOC_REALTIME_HW_FAULT_BREAK = (1UL << 1),
    FOC_REALTIME_HW_FAULT_SOFTWARE_CURRENT_TRIP = (1UL << 2),
    FOC_REALTIME_HW_FAULT_BUS_UNDERVOLTAGE = (1UL << 3),
    FOC_REALTIME_HW_FAULT_BUS_OVERVOLTAGE = (1UL << 4),
    FOC_REALTIME_HW_FAULT_ADC_SAMPLE_ERROR = (1UL << 5),
    FOC_REALTIME_HW_FAULT_DEADLINE_MISSED = (1UL << 6),
    FOC_REALTIME_HW_FAULT_KNOWN_MASK =
        FOC_REALTIME_HW_FAULT_DRIVER |
        FOC_REALTIME_HW_FAULT_BREAK |
        FOC_REALTIME_HW_FAULT_SOFTWARE_CURRENT_TRIP |
        FOC_REALTIME_HW_FAULT_BUS_UNDERVOLTAGE |
        FOC_REALTIME_HW_FAULT_BUS_OVERVOLTAGE |
        FOC_REALTIME_HW_FAULT_ADC_SAMPLE_ERROR |
        FOC_REALTIME_HW_FAULT_DEADLINE_MISSED,
};

/* 相电压来源、质量状态、原因和实际选择的稳定 ABI 数值。 */
typedef uint32_t foc_realtime_phase_voltage_provenance_t;
enum
{
    FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_NONE = 0,
    FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_ST_NOMINAL = 1,
    FOC_REALTIME_PHASE_VOLTAGE_PROVENANCE_BOARD_CALIBRATED = 2,
};

typedef uint32_t foc_realtime_phase_voltage_quality_t;
enum
{
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCONFIGURED = 0,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_UNCALIBRATED = 1,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_VALID = 2,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_STALE = 3,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_OPEN_SUSPECT = 4,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_LOW_SATURATION = 5,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_HIGH_SATURATION = 6,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_THREE_PHASE_INCONSISTENT = 7,
    FOC_REALTIME_PHASE_VOLTAGE_QUALITY_INVALID_SAMPLE = 8,
};

enum
{
    FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCONFIGURED = (1UL << 0),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCALIBRATED = (1UL << 1),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE = (1UL << 2),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE = (1UL << 3),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_LOW_SATURATION = (1UL << 4),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_HIGH_SATURATION = (1UL << 5),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_OPEN_SUSPECT = (1UL << 6),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT = (1UL << 7),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS = (1UL << 8),
    FOC_REALTIME_PHASE_VOLTAGE_REASON_KNOWN_MASK =
        FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCONFIGURED |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_UNCALIBRATED |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_INVALID_SAMPLE |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_STALE |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_LOW_SATURATION |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_HIGH_SATURATION |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_OPEN_SUSPECT |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_THREE_PHASE_INCONSISTENT |
        FOC_REALTIME_PHASE_VOLTAGE_REASON_RECOVERY_HYSTERESIS,
};

typedef uint32_t foc_realtime_observer_voltage_selection_t;
enum
{
    FOC_REALTIME_OBSERVER_VOLTAGE_COMMAND_MODEL = 0,
    FOC_REALTIME_OBSERVER_VOLTAGE_MEASURED = 1,
    FOC_REALTIME_OBSERVER_VOLTAGE_UNAVAILABLE = 2,
};

/*
 * 一次实时控制拍的完整物理量输入。字段顺序和 offset 属于 V19 ABI。
 * 原始 ADC 码、HAL 类型和 A22 内部滞回计数不得跨过此边界。
 * Complete physical-input snapshot for one realtime control tick. Raw ADC
 * codes, HAL types and A22's internal hysteresis state stay on the C side.
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    /* First accepted tick after start is 0; each later accepted tick must be
     * wrapping +1. Duplicate, skipped or stale control inputs are rejected. */
    uint32_t control_sequence;
    uint32_t valid_flags;
    uint32_t hardware_fault_flags;
    uint32_t phase_voltage_sequence;
    uint32_t phase_voltage_age_ticks;
    uint32_t phase_voltage_provenance;
    uint32_t phase_voltage_quality_state;
    uint32_t phase_voltage_reason_mask;
    uint32_t observer_voltage_selection;
    uint32_t phase_voltage_fallback_event_count;
    /* Scheduler evidence only in ABI V19. Fixed-step control math remains in
     * use, so this value must be within +/-1% of the configured period. */
    float actual_dt_s;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float dc_bus_voltage;
    float phase_voltage_a_v;
    float phase_voltage_b_v;
    float phase_voltage_c_v;
    float electrical_angle_rad;
    float sensor_temperature_c;
} foc_realtime_input_t;

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
