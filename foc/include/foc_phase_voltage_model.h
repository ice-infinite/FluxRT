#ifndef FOC_PHASE_VOLTAGE_MODEL_H
#define FOC_PHASE_VOLTAGE_MODEL_H

/*
 * FluxRT —— 相电压 ADC 换算模型的纯 C 契约。
 * FluxRT - hardware-independent phase-voltage ADC conversion contract.
 *
 * 本文件只描述“原始码怎样换算成端电压”，不决定这些数据能否进入观察器。
 * 名义元件值与逐板实测标定必须用 source/flags 明确区分，不能把官方原理图参数
 * 冒充为已标定结果。
 * This contract only converts raw ADC codes to terminal voltage.  Nominal
 * component values and per-board calibration are deliberately distinct; the
 * nominal model must never be presented as measured calibration evidence.
 */

#include <stdint.h>

#include "foc_phase_voltage_capture.h"

#define FOC_PHASE_VOLTAGE_MODEL_VERSION (1UL)

typedef enum
{
    FOC_PHASE_VOLTAGE_MODEL_NONE = 0,
    FOC_PHASE_VOLTAGE_MODEL_ST_IHM16M1_NOMINAL = 1,
    /* 为未来逐板/逐通道模型保留；v1 校验器会明确拒绝，不能静默降级为共用斜率。 */
    FOC_PHASE_VOLTAGE_MODEL_BOARD_CALIBRATED = 2,
} foc_phase_voltage_model_source_t;

enum
{
    /* 参数来自官方原理图/参考配置，不是本板实测值。 */
    FOC_PHASE_VOLTAGE_MODEL_FLAG_NOMINAL_COMPONENTS = (1UL << 0),
    /* 为未来通过审批的逐板模型保留；v1 不接受此位。 */
    FOC_PHASE_VOLTAGE_MODEL_FLAG_BOARD_CALIBRATED = (1UL << 1),
    /* 只允许诊断、采集和对拍，禁止作为闭环观察器输入。 */
    FOC_PHASE_VOLTAGE_MODEL_FLAG_DIAGNOSTIC_ONLY = (1UL << 2),
    /* 通过质量门后才允许作为观察器候选；名义模型不得设置此位。 */
    FOC_PHASE_VOLTAGE_MODEL_FLAG_OBSERVER_ELIGIBLE = (1UL << 3),
};

/*
 * 全部使用整数，避免在 Shell/证据元数据里引入不同的浮点格式。
 * volts_per_count_uv 是便于显示的四舍五入值；实际换算使用
 * phase_full_scale_mv / adc_max_code，保证满量程端点准确。
 */
typedef struct
{
    uint32_t struct_size;
    uint32_t version;
    uint32_t source;
    uint32_t flags;
    uint32_t adc_reference_mv;
    uint32_t adc_max_code;
    uint32_t divider_upper_ohm;
    uint32_t divider_lower_ohm;
    uint32_t phase_full_scale_mv;
    uint32_t volts_per_count_uv;
} foc_phase_voltage_model_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(foc_phase_voltage_model_t) == 40U,
               "phase-voltage model ABI changed");
#endif

/*
 * 返回 1 表示 v1 官方名义模型内部一致；它不等于“逐板标定已通过”。
 * v1 没有每相 slope/intercept，因而会拒绝 BOARD_CALIBRATED；后者必须升版扩展。
 */
uint32_t foc_phase_voltage_model_validate(
    const foc_phase_voltage_model_t *model);

/*
 * divider 必须处于 enabled；off 窗没有物理换算意义。成功返回 1 并写入 mV，
 * 失败返回 0 且把输出清零。
 */
uint32_t foc_phase_voltage_raw_to_mv(
    const foc_phase_voltage_model_t *model,
    foc_phase_voltage_divider_mode_t divider_mode,
    uint16_t raw,
    uint32_t *phase_voltage_mv);

#endif /* FOC_PHASE_VOLTAGE_MODEL_H */
