#include "foc_platform.h"
#include "foc_math_accel.h"

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
static foc_realtime_timing_stats_t g_foc_timing_stats;

#if defined(FOC_TARGET_STM32G431)
#include "rtconfig.h"
#include "stm32g4xx_hal.h"

/* ST reference: NUCLEO-G431RB + X-NUCLEO-IHM16M1. */
#define FOC_GATE_ENABLE_PORT             GPIOB
#define FOC_GATE_ENABLE_PINS             (GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
#define FOC_PWM_PORT                     GPIOA
#define FOC_PWM_PINS                     (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)
#define FOC_DRIVER_PROTECTION_PORT       GPIOA
#define FOC_DRIVER_PROTECTION_PIN        GPIO_PIN_11

#define FOC_PWM_TIMER_CLOCK_HZ           (170000000UL)
#define FOC_PWM_FREQUENCY_HZ             (12000UL)
#define FOC_PWM_PERIOD_TICKS             (FOC_PWM_TIMER_CLOCK_HZ / (2UL * FOC_PWM_FREQUENCY_HZ))
#define FOC_ADC_FULL_SCALE               (4095.0f)
#define FOC_ADC_REFERENCE_VOLTAGE        (3.3f)
#define FOC_BUS_PARTITIONING_FACTOR      (0.0625f)
#define FOC_CURRENT_SHUNT_OHM            (0.33f)
#define FOC_CURRENT_AMPLIFIER_GAIN       (1.53f)
#define FOC_ADC_TIMEOUT_MS               (5UL)
#define FOC_OFFSET_SAMPLE_COUNT          (32UL)
#define FOC_OFFSET_MIN_VALID             (256U)
#define FOC_OFFSET_MAX_VALID             (3839U)
#define FOC_CURRENT_COUNTS_PER_AMP       ((FOC_ADC_FULL_SCALE * FOC_CURRENT_SHUNT_OHM * \
                                           FOC_CURRENT_AMPLIFIER_GAIN) / \
                                          FOC_ADC_REFERENCE_VOLTAGE)
#if !defined(FLUXRT_PRODUCTION_BUILD)
#define FOC_TRACE_CAPACITY               (64U)
#define FOC_TRACE_MIN_DIVIDER            (120U)
#endif
#if defined(FOC_ISR_TIMING_PROBE)
#define FOC_TIMING_PROBE_PORT            GPIOA
#define FOC_TIMING_PROBE_PIN             GPIO_PIN_5
#define FOC_TIMING_PROBE_HIGH()          (FOC_TIMING_PROBE_PORT->BSRR = FOC_TIMING_PROBE_PIN)
#define FOC_TIMING_PROBE_LOW()           (FOC_TIMING_PROBE_PORT->BSRR = ((uint32_t)FOC_TIMING_PROBE_PIN << 16U))
#else
#define FOC_TIMING_PROBE_HIGH()          ((void)0)
#define FOC_TIMING_PROBE_LOW()           ((void)0)
#endif
#define FOC_TRIAL_REQUIRED_FLAGS          (FOC_PLATFORM_DIAG_TIM1_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_ADC_CONFIGURED | \
                                           FOC_PLATFORM_DIAG_ADC_CALIBRATED | \
                                           FOC_PLATFORM_DIAG_CURRENT_OFFSETS_VALID | \
                                           FOC_PLATFORM_DIAG_SYNC_RUNNING | \
                                           FOC_PLATFORM_DIAG_SYNC_SAMPLES_VALID)
#define FOC_TRIAL_FORBIDDEN_FLAGS         (FOC_PLATFORM_DIAG_DRIVER_FAULT | \
                                           FOC_PLATFORM_DIAG_OUTPUT_ACTIVE | \
                                           FOC_PLATFORM_DIAG_ADC_READ_ERROR | \
                                           FOC_PLATFORM_DIAG_BREAK_LATCHED | \
                                           FOC_PLATFORM_DIAG_CURRENT_TRIP | \
                                           FOC_PLATFORM_DIAG_TRIAL_ARMED)

static TIM_HandleTypeDef g_foc_tim1;
static ADC_HandleTypeDef g_foc_adc1;
static ADC_HandleTypeDef g_foc_adc2;
static volatile foc_platform_diagnostics_t g_foc_diagnostics;
static volatile uint32_t g_foc_control_armed;
static foc_rust_context_t *g_foc_controller;
static volatile foc_telemetry_t g_foc_telemetry;
#if !defined(FLUXRT_PRODUCTION_BUILD)
static volatile foc_trace_sample_t g_foc_trace_buffer[FOC_TRACE_CAPACITY];
static volatile uint32_t g_foc_trace_head;
static volatile uint32_t g_foc_trace_tail;
static volatile uint32_t g_foc_trace_enabled;
static volatile uint32_t g_foc_trace_divider = 240U;
static volatile uint32_t g_foc_trace_counter;
static volatile uint32_t g_foc_trace_dropped_count;

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
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

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

static uint32_t foc_platform_trace_capture(const foc_feedback_t *feedback,
                                           const foc_output_t *output,
                                           const foc_telemetry_t *telemetry)
{
    foc_trace_sample_t sample;
    uint32_t head;
    uint32_t next;

    if (g_foc_trace_enabled == 0U)
    {
        return 0U;
    }
    ++g_foc_trace_counter;
    if (g_foc_trace_counter < g_foc_trace_divider)
    {
        return 0U;
    }
    g_foc_trace_counter = 0U;

    sample.step = g_foc_diagnostics.realtime_step_count;
    sample.flags = g_foc_diagnostics.flags;
    sample.state = (uint16_t)telemetry->state;
    sample.observer_reliable = (uint16_t)telemetry->observer_reliable;
    sample.phase_a_ma = foc_trace_scaled_i16(feedback->phase_current_a, 1000.0f);
    sample.phase_b_ma = foc_trace_scaled_i16(feedback->phase_current_b, 1000.0f);
    sample.phase_c_ma = foc_trace_scaled_i16(feedback->phase_current_c, 1000.0f);
    sample.id_reference_ma = foc_trace_scaled_i16(telemetry->id_reference_a, 1000.0f);
    sample.iq_reference_ma = foc_trace_scaled_i16(telemetry->iq_reference_a, 1000.0f);
    sample.id_measured_ma = foc_trace_scaled_i16(telemetry->id_measured_a, 1000.0f);
    sample.iq_measured_ma = foc_trace_scaled_i16(telemetry->iq_measured_a, 1000.0f);
    sample.vd_command_mv = foc_trace_scaled_i16(telemetry->vd_command_v, 1000.0f);
    sample.vq_command_mv = foc_trace_scaled_i16(telemetry->vq_command_v, 1000.0f);
    sample.duty_a_per_mille = foc_trace_scaled_u16(output->duty_a, 1000.0f);
    sample.duty_b_per_mille = foc_trace_scaled_u16(output->duty_b, 1000.0f);
    sample.duty_c_per_mille = foc_trace_scaled_u16(output->duty_c, 1000.0f);
    sample.bus_voltage_mv = foc_trace_scaled_u16(feedback->dc_bus_voltage, 1000.0f);
    sample.control_angle_mrad =
        foc_trace_scaled_i16(telemetry->electrical_angle_rad, 1000.0f);
    sample.forced_angle_mrad =
        foc_trace_scaled_i16(telemetry->forced_electrical_angle_rad, 1000.0f);
    sample.observer_angle_mrad =
        foc_trace_scaled_i16(telemetry->observer_electrical_angle_rad, 1000.0f);
    sample.observer_speed_rpm =
        foc_trace_scaled_i16(telemetry->measured_speed_rpm, 1.0f);

    head = g_foc_trace_head;
    next = (head + 1U) & (FOC_TRACE_CAPACITY - 1U);
    if (next == g_foc_trace_tail)
    {
        ++g_foc_trace_dropped_count;
        return 0U;
    }
    g_foc_trace_buffer[head] = sample;
    __DMB();
    g_foc_trace_head = next;
    return 1U;
}
#endif

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

static void foc_platform_disable_power_fast(void)
{
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

static uint32_t foc_platform_gate_is_low(void)
{
    return ((FOC_GATE_ENABLE_PORT->ODR & FOC_GATE_ENABLE_PINS) == 0U) ? 1U : 0U;
}

static uint32_t foc_platform_driver_faulted(void)
{
    /* IHM16M1 driver-protection input is active low and has a pull-up. */
    return ((FOC_DRIVER_PROTECTION_PORT->IDR & FOC_DRIVER_PROTECTION_PIN) == 0U) ? 1U : 0U;
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
    g_foc_tim1.Init.RepetitionCounter = 1U;
    g_foc_tim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&g_foc_tim1) != HAL_OK)
    {
        return 0U;
    }

    master.MasterOutputTrigger = TIM_TRGO_OC4REF;
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

    output.OCMode = TIM_OCMODE_PWM2;
    output.Pulse = FOC_PWM_PERIOD_TICKS - 1U;
    if (HAL_TIM_PWM_ConfigChannel(&g_foc_tim1, &output, TIM_CHANNEL_4) != HAL_OK)
    {
        return 0U;
    }

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

    g_foc_diagnostics.pwm_frequency_hz = FOC_PWM_FREQUENCY_HZ;
    g_foc_diagnostics.pwm_period_ticks = FOC_PWM_PERIOD_TICKS;
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
    /* PC2=potentiometer, PC4=temperature. */
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &gpio);
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
    injected.InjectedNbrOfConversion = 1U;
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injected.InjecOversamplingMode = DISABLE;

    injected.InjectedChannel = ADC_CHANNEL_2;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc1, &injected) != HAL_OK)
    {
        return 0U;
    }
    injected.InjectedChannel = ADC_CHANNEL_14;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    return (HAL_ADCEx_InjectedConfigChannel(&g_foc_adc2, &injected) == HAL_OK) ? 1U : 0U;
}

static uint32_t foc_platform_start_sync_monitor(void)
{
    g_foc_diagnostics.sync_sample_count = 0U;
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
    return 1U;
}
#endif

foc_status_t foc_platform_init(void)
{
    foc_platform_emergency_stop();
    foc_realtime_timing_reset(&g_foc_timing_stats);
    (void)foc_math_accel_init();
#if defined(FOC_TARGET_STM32G431)
    if ((foc_platform_init_timer_disabled() == 0U) ||
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

    foc_platform_disable_power_fast();
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
    {
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
    }
    g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_SYNC_RUNNING;
    foc_platform_refresh_safety_flags();
#endif
}

#if defined(FOC_TARGET_STM32G431)
void ADC1_2_IRQHandler(void)
{
    uint32_t cycle_start = DWT->CYCCNT;
    uint32_t cycle_end;
    uint32_t control_cycle_start = cycle_start;
    uint32_t control_cycle_end = cycle_start;
    uint32_t control_executed = 0U;
    uint32_t timing_active = 0U;
#if !defined(FLUXRT_PRODUCTION_BUILD)
    uint32_t trace_enabled = g_foc_trace_enabled;
#else
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
        g_foc_diagnostics.phase_u_raw = (uint16_t)ADC1->JDR1;
        g_foc_diagnostics.phase_v_raw = (uint16_t)ADC2->JDR1;
        current_u_counts = (int32_t)g_foc_diagnostics.phase_u_offset -
                           (int32_t)g_foc_diagnostics.phase_u_raw;
        current_v_counts = (int32_t)g_foc_diagnostics.phase_v_offset -
                           (int32_t)g_foc_diagnostics.phase_v_raw;
        current_w_counts = -current_u_counts - current_v_counts;
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
            delta_u = (uint16_t)((current_u_counts < 0) ? -current_u_counts : current_u_counts);
            delta_v = (uint16_t)((current_v_counts < 0) ? -current_v_counts : current_v_counts);
            delta_w = (uint16_t)((current_w_counts < 0) ? -current_w_counts : current_w_counts);
            peak = (delta_u > delta_v) ? delta_u : delta_v;
            peak = (peak > delta_w) ? peak : delta_w;
            if (peak > g_foc_diagnostics.peak_current_delta_counts)
            {
                g_foc_diagnostics.peak_current_delta_counts = peak;
            }
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
                    g_foc_telemetry = telemetry;
                    ++g_foc_diagnostics.realtime_step_count;
#if !defined(FLUXRT_PRODUCTION_BUILD)
                    trace_sampled = foc_platform_trace_capture(&feedback, &output, &telemetry);
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

void TIM1_BRK_TIM15_IRQHandler(void)
{
    if ((TIM1->SR & (TIM_SR_BIF | TIM_SR_B2IF)) != 0U)
    {
        TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_B2IF);
        ++g_foc_diagnostics.break_fault_count;
        g_foc_diagnostics.flags |= FOC_PLATFORM_DIAG_BREAK_LATCHED;
        foc_platform_disable_power_fast();
        TIM1->CCER &= ~TIM_CCER_CC4E;
        TIM1->CR1 &= ~TIM_CR1_CEN;
        g_foc_diagnostics.flags &= ~FOC_PLATFORM_DIAG_SYNC_RUNNING;
    }
}
#endif

foc_status_t foc_platform_read_feedback(foc_feedback_t *feedback)
{
    if (feedback == 0)
    {
        return FOC_STATUS_INVALID_ARGUMENT;
    }

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
        foc_platform_disable_power_fast();
        foc_platform_refresh_safety_flags();
        return FOC_STATUS_INVALID_ARGUMENT;
    }

    TIM1->CCR1 = (uint32_t)((output->duty_a * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    TIM1->CCR2 = (uint32_t)((output->duty_b * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    TIM1->CCR3 = (uint32_t)((output->duty_c * (float)FOC_PWM_PERIOD_TICKS) + 0.5f);
    ++g_foc_diagnostics.trial_apply_count;
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

foc_status_t foc_platform_control_start(float target_speed_rpm)
{
#if defined(FOC_TARGET_STM32G431)
    uint32_t neutral_ticks;
    uint16_t bus_min_raw;
    uint16_t bus_max_raw;
    foc_feedback_t feedback;
    foc_status_t status;

    if ((g_foc_controller == 0) || (g_foc_control_armed != 0U))
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
#if defined(FOC_TARGET_STM32G431) && !defined(FLUXRT_PRODUCTION_BUILD)
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
#if defined(FOC_TARGET_STM32G431) && !defined(FLUXRT_PRODUCTION_BUILD)
    g_foc_trace_enabled = 0U;
#endif
}

uint32_t foc_platform_trace_is_enabled(void)
{
#if defined(FOC_TARGET_STM32G431) && !defined(FLUXRT_PRODUCTION_BUILD)
    return g_foc_trace_enabled;
#else
    return 0U;
#endif
}

uint32_t foc_platform_trace_pop(foc_trace_sample_t *sample)
{
#if defined(FOC_TARGET_STM32G431) && !defined(FLUXRT_PRODUCTION_BUILD)
    uint32_t primask;
    uint32_t tail;
    if (sample == 0)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    tail = g_foc_trace_tail;
    if (tail == g_foc_trace_head)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }
    *sample = g_foc_trace_buffer[tail];
    g_foc_trace_tail = (tail + 1U) & (FOC_TRACE_CAPACITY - 1U);
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
#else
    (void)sample;
    return 0U;
#endif
}

uint32_t foc_platform_trace_dropped(void)
{
#if defined(FOC_TARGET_STM32G431) && !defined(FLUXRT_PRODUCTION_BUILD)
    return g_foc_trace_dropped_count;
#else
    return 0U;
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
