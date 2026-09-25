#ifndef FOC_PWM_TIMING_H
#define FOC_PWM_TIMING_H

/*
 * FluxRT —— PWM、ADC 触发与控制拍之间的纯 C 时序契约。
 * FluxRT - pure-C timing contract between PWM, ADC triggering and control ticks.
 *
 * 本模块不访问寄存器。目标平台在配置 TIM1 前用它拒绝不自洽的常量，Host
 * 测试则用同一份规则锁定 12/12 kHz 基线与 24/12 kHz 候选。
 * This module touches no registers. The target validates its constants before
 * configuring TIM1, while Host tests pin the same rules for the 12/12 kHz
 * baseline and the optional 24/12 kHz candidate.
 */

#include <stdint.h>

typedef uint32_t foc_pwm_adc_trigger_t;
enum
{
    /* TIM1 CH4 PWM2 的 OC4REF 上升沿；每个载波周期触发一次。 */
    FOC_PWM_ADC_TRIGGER_OC4REF_RISING = 1U,
    /* 已废弃：RCR 分频后的 TIM1 Update Event 不保证位于三分流有效窗口。 */
    FOC_PWM_ADC_TRIGGER_UPDATE = 2U,
    /* TIM1 OC4REF -> TIM2 ITR0 计数分频 -> TIM2 TRGO，保持有效采样窗口。 */
    FOC_PWM_ADC_TRIGGER_DIVIDED_OC4REF = 3U,
};

typedef struct
{
    uint32_t timer_clock_hz;
    uint32_t pwm_frequency_hz;
    uint32_t control_frequency_hz;
    uint32_t pwm_period_ticks;
    uint32_t repetition_counter;
    uint32_t actuation_delay_pwm_ticks;
    foc_pwm_adc_trigger_t adc_trigger;
} foc_pwm_timing_plan_t;

/* 返回 PWM/控制整数比；非法或空指针返回 0。 */
uint32_t foc_pwm_timing_ratio(const foc_pwm_timing_plan_t *plan);

/*
 * 校验本工程采用的中心对齐时序：
 *   ARR = timer/(2*fpwm)
 *   RCR + 1 = 2*(fpwm/fcontrol)
 *   多速率必须先在 OC4REF 指定的有效电流窗口采样，再由辅助定时器按整数比
 *   分频；禁止直接用 TIM1 Update，因为实机已证明该相位会读到近零的三分流值。
 * OC4REF 直连只允许 1:1，DIVIDED_OC4REF 只允许多速率。延迟字段仍是整 PWM
 * 拍的名义仿真值；OC4REF、TIM2 TRGO 与 UEV 的相位必须在目标板上复核。
 */
uint32_t foc_pwm_timing_plan_is_valid(const foc_pwm_timing_plan_t *plan);

#endif
