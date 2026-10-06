#include "foc_time_sync_stm32g431.h"

#include "foc_build_profile.h"

#if defined(FLUXRT_H3_TIME_SYNC_BUILD)

#include <string.h>

#include "foc_platform.h"
#include "stm32g4xx_hal.h"

#define FOC_TIME_SYNC_TX_TIMER_CLOCK_HZ    (170000000UL)
#define FOC_TIME_SYNC_TX_TIMER_PRESCALER   \
    ((FOC_TIME_SYNC_TX_TIMER_CLOCK_HZ / FOC_TIME_SYNC_STM32G431_TIMER_HZ) - 1UL)
#define FOC_TIME_SYNC_TX_FIRST_COMPARE     (100U)
#define FOC_TIME_SYNC_TX_SENTINEL_GAP      (100U)
#define FOC_TIME_SYNC_TX_GUARD_HIGH_TICKS  (10U)
#define FOC_TIME_SYNC_TX_GUARD_LOW_TICKS   (20U)

static TIM_HandleTypeDef g_foc_time_sync_tim4;
static DMA_HandleTypeDef g_foc_time_sync_dma1_channel3;
static foc_time_sync_toggle_plan_t g_foc_time_sync_tx_plan;
static foc_time_sync_stm32g431_tx_status_t g_foc_time_sync_tx_status = {
    .struct_size = sizeof(foc_time_sync_stm32g431_tx_status_t),
    .version = FOC_TIME_SYNC_STM32G431_TX_ABI_VERSION,
    .state = (uint32_t)FOC_TIME_SYNC_STM32G431_TX_UNINITIALIZED,
};

static uint32_t foc_time_sync_stm32g431_saturating_increment(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : (value + 1U);
}

static void foc_time_sync_stm32g431_pb6_low(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void foc_time_sync_stm32g431_pb6_tim4(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void foc_time_sync_stm32g431_pb7_input(uint32_t timer_alternate)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = (timer_alternate != 0U) ? GPIO_MODE_AF_PP : GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = (timer_alternate != 0U) ? GPIO_AF2_TIM4 : 0U;
    HAL_GPIO_Init(GPIOB, &gpio);
}

static void foc_time_sync_stm32g431_tx_fail(uint32_t hal_status)
{
    CLEAR_BIT(TIM4->DIER,
              TIM_DIER_CC1IE | TIM_DIER_CC1DE | TIM_DIER_CC2IE);
    CLEAR_BIT(TIM4->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);
    CLEAR_BIT(DMA1_Channel3->CCR, DMA_CCR_EN);
    foc_time_sync_stm32g431_pb6_low();
    foc_time_sync_stm32g431_pb7_input(0U);
    g_foc_time_sync_tx_status.last_hal_status = hal_status;
    g_foc_time_sync_tx_status.failed_count =
        foc_time_sync_stm32g431_saturating_increment(
            g_foc_time_sync_tx_status.failed_count);
    g_foc_time_sync_tx_status.state =
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_FAILED;
}

#endif /* FLUXRT_H3_TIME_SYNC_BUILD */

uint32_t foc_time_sync_stm32g431_adapter_init(
    foc_time_sync_adapter_t *adapter,
    foc_time_sync_capture_t *capture,
    void *port_context,
    foc_time_sync_read_local_tick_isr_fn read_local_tick_isr,
    foc_time_sync_read_control_tick_isr_fn read_control_tick_isr)
{
    return foc_time_sync_adapter_init(
        adapter,
        capture,
        (uint32_t)FOC_TIME_SYNC_SOURCE_CONTROL_REFERENCE,
        port_context,
        read_local_tick_isr,
        read_control_tick_isr);
}

#if defined(FLUXRT_H3_TIME_SYNC_BUILD)

uint32_t foc_time_sync_stm32g431_tx_init(void)
{
    TIM_OC_InitTypeDef output_compare = {0};

    if (g_foc_time_sync_tx_status.state ==
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_BUSY)
    {
        return 0U;
    }

    foc_time_sync_stm32g431_pb6_low();
    foc_time_sync_stm32g431_pb7_input(0U);
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    (void)memset(&g_foc_time_sync_tim4, 0, sizeof(g_foc_time_sync_tim4));
    g_foc_time_sync_tim4.Instance = TIM4;
    g_foc_time_sync_tim4.Init.Prescaler = FOC_TIME_SYNC_TX_TIMER_PRESCALER;
    g_foc_time_sync_tim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_foc_time_sync_tim4.Init.Period = UINT16_MAX;
    g_foc_time_sync_tim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_foc_time_sync_tim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&g_foc_time_sync_tim4) != HAL_OK)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)HAL_ERROR);
        return 0U;
    }

    output_compare.OCMode = TIM_OCMODE_TOGGLE;
    output_compare.Pulse = FOC_TIME_SYNC_TX_FIRST_COMPARE;
    output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
    output_compare.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_OC_ConfigChannel(&g_foc_time_sync_tim4,
                                 &output_compare,
                                 TIM_CHANNEL_1) != HAL_OK)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)HAL_ERROR);
        return 0U;
    }

    /* Optional PB6 -> PB7 loopback. CH2 captures the actual pin edge on the
     * same 1 MHz TIM4 counter; it remains disabled until an explicit frame is
     * started and is harmless while PB7 is not wired. */
    MODIFY_REG(TIM4->CCMR1,
               TIM_CCMR1_CC2S | TIM_CCMR1_IC2PSC | TIM_CCMR1_IC2F,
               TIM_CCMR1_CC2S_0);
    CLEAR_BIT(TIM4->CCER,
              TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2E);

    (void)memset(&g_foc_time_sync_dma1_channel3,
                 0,
                 sizeof(g_foc_time_sync_dma1_channel3));
    g_foc_time_sync_dma1_channel3.Instance = DMA1_Channel3;
    g_foc_time_sync_dma1_channel3.Init.Request = DMA_REQUEST_TIM4_CH1;
    g_foc_time_sync_dma1_channel3.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_foc_time_sync_dma1_channel3.Init.PeriphInc = DMA_PINC_DISABLE;
    g_foc_time_sync_dma1_channel3.Init.MemInc = DMA_MINC_ENABLE;
    g_foc_time_sync_dma1_channel3.Init.PeriphDataAlignment =
        DMA_PDATAALIGN_HALFWORD;
    g_foc_time_sync_dma1_channel3.Init.MemDataAlignment =
        DMA_MDATAALIGN_HALFWORD;
    g_foc_time_sync_dma1_channel3.Init.Mode = DMA_NORMAL;
    g_foc_time_sync_dma1_channel3.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&g_foc_time_sync_dma1_channel3) != HAL_OK)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)HAL_ERROR);
        return 0U;
    }
    __HAL_LINKDMA(&g_foc_time_sync_tim4,
                  hdma[TIM_DMA_ID_CC1],
                  g_foc_time_sync_dma1_channel3);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 8U, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    g_foc_time_sync_tx_status.state =
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_READY;
    g_foc_time_sync_tx_status.last_hal_status = (uint32_t)HAL_OK;
    return 1U;
}

uint32_t foc_time_sync_stm32g431_tx_start(
    const foc_time_sync_edge_identity_t *identity)
{
    foc_time_sync_wire_symbol_t symbols[
        FOC_TIME_SYNC_TOGGLE_MAX_PHYSICAL_SYMBOLS];
    foc_time_sync_wire_timing_t timing = {
        sizeof(foc_time_sync_wire_timing_t),
        FOC_TIME_SYNC_WIRE_ABI_VERSION,
        20U,
        40U,
        20U,
        80U,
        40U,
        3U,
    };
    uint32_t symbol_count = 0U;
    HAL_StatusTypeDef hal_status;

    if ((identity == NULL) ||
        ((g_foc_time_sync_tx_status.state !=
          (uint32_t)FOC_TIME_SYNC_STM32G431_TX_READY) &&
         (g_foc_time_sync_tx_status.state !=
          (uint32_t)FOC_TIME_SYNC_STM32G431_TX_COMPLETE)))
    {
        return 0U;
    }
    if (foc_time_sync_wire_encode(
            identity,
            &timing,
            symbols,
            FOC_TIME_SYNC_WIRE_SYMBOL_COUNT,
            &symbol_count) == 0U)
    {
        return 0U;
    }
    symbols[symbol_count].high_ticks =
        FOC_TIME_SYNC_TX_GUARD_HIGH_TICKS;
    symbols[symbol_count].low_ticks =
        FOC_TIME_SYNC_TX_GUARD_LOW_TICKS;
    ++symbol_count;
    if (foc_time_sync_toggle_plan_build(
            symbols,
            symbol_count,
            FOC_TIME_SYNC_TX_FIRST_COMPARE,
            FOC_TIME_SYNC_TX_SENTINEL_GAP,
            &g_foc_time_sync_tx_plan) == 0U)
    {
        return 0U;
    }

    /* Keep PB6 actively low while TIM4/DMA is prepared.  GPIO23 is also the
     * disabled Axis1 I2C SDA on DengFOC and can be externally pulled high; if
     * PB6 is switched to AF before CC1 is enabled, that pull-up creates a
     * short false SOF edge. */
    foc_time_sync_stm32g431_pb6_low();
    __HAL_TIM_SET_COUNTER(&g_foc_time_sync_tim4, 0U);
    __HAL_TIM_SET_COMPARE(&g_foc_time_sync_tim4,
                          TIM_CHANNEL_1,
                          g_foc_time_sync_tx_plan.first_compare_tick);
    __HAL_TIM_CLEAR_FLAG(&g_foc_time_sync_tim4, TIM_FLAG_CC1);
    __HAL_TIM_CLEAR_FLAG(&g_foc_time_sync_tim4, TIM_FLAG_CC2);
    g_foc_time_sync_tx_status.first_edge_timer_tick = 0U;
    g_foc_time_sync_tx_status.first_edge_cycle_tick = 0U;
    g_foc_time_sync_tx_status.first_edge_control_tick =
        FOC_TIME_SYNC_NO_CONTROL_TICK;
    g_foc_time_sync_tx_status.first_edge_flags = 0U;
    g_foc_time_sync_tx_status.loopback_edge_timer_tick = 0U;
    g_foc_time_sync_tx_status.loopback_edge_cycle_tick = 0U;
    g_foc_time_sync_tx_status.loopback_delta_cycles = 0;
    g_foc_time_sync_tx_status.last_session_id = identity->session_id;
    g_foc_time_sync_tx_status.last_edge_sequence = identity->edge_sequence;
    g_foc_time_sync_tx_status.last_edge_tag = identity->edge_tag;
    g_foc_time_sync_tx_status.state =
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_BUSY;
    foc_time_sync_stm32g431_pb7_input(1U);
    SET_BIT(TIM4->CCER, TIM_CCER_CC2E);
    SET_BIT(TIM4->DIER, TIM_DIER_CC2IE);
    __HAL_TIM_ENABLE_IT(&g_foc_time_sync_tim4, TIM_IT_CC1);
    hal_status = HAL_TIM_OC_Start_DMA(
        &g_foc_time_sync_tim4,
        TIM_CHANNEL_1,
        (const uint32_t *)(const void *)
            g_foc_time_sync_tx_plan.dma_compare_ticks,
        (uint16_t)g_foc_time_sync_tx_plan.dma_value_count);
    if (hal_status != HAL_OK)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)hal_status);
        return 0U;
    }
    /* TIM4 CH1 is now enabled with an inactive-low output and the first
     * compare remains 100 us away.  Connecting AF2 at this point preserves
     * the low idle level until the intended first edge. */
    foc_time_sync_stm32g431_pb6_tim4();
    g_foc_time_sync_tx_status.started_count =
        foc_time_sync_stm32g431_saturating_increment(
            g_foc_time_sync_tx_status.started_count);
    return 1U;
}

void foc_time_sync_stm32g431_tx_abort(void)
{
    if (g_foc_time_sync_tx_status.state ==
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_BUSY)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)HAL_ERROR);
    }
    else
    {
        foc_time_sync_stm32g431_pb6_low();
        foc_time_sync_stm32g431_pb7_input(0U);
    }
}

void foc_time_sync_stm32g431_tx_get_status(
    foc_time_sync_stm32g431_tx_status_t *status)
{
    if (status != NULL)
    {
        *status = g_foc_time_sync_tx_status;
    }
}

void DMA1_Channel3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_foc_time_sync_dma1_channel3);
}

void TIM4_IRQHandler(void)
{
    if ((__HAL_TIM_GET_FLAG(&g_foc_time_sync_tim4, TIM_FLAG_CC2) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&g_foc_time_sync_tim4, TIM_IT_CC2) != RESET))
    {
        uint32_t capture_tick;
        uint32_t timer_tick;
        uint32_t cycle_tick;
        uint32_t elapsed_timer_ticks;

        __HAL_TIM_CLEAR_IT(&g_foc_time_sync_tim4, TIM_IT_CC2);
        __HAL_TIM_DISABLE_IT(&g_foc_time_sync_tim4, TIM_IT_CC2);
        CLEAR_BIT(TIM4->CCER, TIM_CCER_CC2E);
        capture_tick = (uint32_t)__HAL_TIM_GET_COMPARE(
            &g_foc_time_sync_tim4, TIM_CHANNEL_2);
        timer_tick = (uint32_t)__HAL_TIM_GET_COUNTER(&g_foc_time_sync_tim4);
        cycle_tick = DWT->CYCCNT;
        elapsed_timer_ticks =
            (uint32_t)(uint16_t)(timer_tick - capture_tick);
        cycle_tick -= elapsed_timer_ticks *
            (FOC_TIME_SYNC_STM32G431_CYCLE_HZ /
             FOC_TIME_SYNC_STM32G431_TIMER_HZ);
        g_foc_time_sync_tx_status.loopback_edge_timer_tick = capture_tick;
        g_foc_time_sync_tx_status.loopback_edge_cycle_tick = cycle_tick;
        g_foc_time_sync_tx_status.loopback_edge_count =
            foc_time_sync_stm32g431_saturating_increment(
                g_foc_time_sync_tx_status.loopback_edge_count);
        g_foc_time_sync_tx_status.first_edge_flags |=
            FOC_TIME_SYNC_STM32G431_LOOPBACK_VALID;
        if ((g_foc_time_sync_tx_status.first_edge_flags &
             FOC_TIME_SYNC_STM32G431_EDGE_CYCLE_VALID) != 0U)
        {
            g_foc_time_sync_tx_status.loopback_delta_cycles =
                (int32_t)(cycle_tick -
                    g_foc_time_sync_tx_status.first_edge_cycle_tick);
        }
    }
    if ((__HAL_TIM_GET_FLAG(&g_foc_time_sync_tim4, TIM_FLAG_CC1) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&g_foc_time_sync_tim4, TIM_IT_CC1) != RESET))
    {
        uint32_t timer_tick;
        uint32_t cycle_tick;
        uint32_t elapsed_timer_ticks;
        uint32_t control_tick;

        __HAL_TIM_CLEAR_IT(&g_foc_time_sync_tim4, TIM_IT_CC1);
        __HAL_TIM_DISABLE_IT(&g_foc_time_sync_tim4, TIM_IT_CC1);
        timer_tick = (uint32_t)__HAL_TIM_GET_COUNTER(&g_foc_time_sync_tim4);
        cycle_tick = DWT->CYCCNT;
        elapsed_timer_ticks =
            (uint32_t)(uint16_t)(timer_tick - FOC_TIME_SYNC_TX_FIRST_COMPARE);
        /* TIM4 and DWT are both derived from the 170 MHz core clock.  Correct
         * the ISR-entry CYCCNT by the elapsed 1 us TIM4 ticks so the recorded
         * value represents the hardware compare edge, not priority latency.
         * A future PB7 input-capture loopback remains the independent physical
         * validation of this generated-edge latch. */
        cycle_tick -= elapsed_timer_ticks *
            (FOC_TIME_SYNC_STM32G431_CYCLE_HZ /
             FOC_TIME_SYNC_STM32G431_TIMER_HZ);
        g_foc_time_sync_tx_status.first_edge_timer_tick = timer_tick;
        g_foc_time_sync_tx_status.first_edge_cycle_tick = cycle_tick;
        g_foc_time_sync_tx_status.first_edge_flags |=
            FOC_TIME_SYNC_STM32G431_EDGE_CYCLE_VALID;
        if (foc_platform_time_sync_control_tick_isr(
                cycle_tick, &control_tick) != 0U)
        {
            g_foc_time_sync_tx_status.first_edge_control_tick = control_tick;
            g_foc_time_sync_tx_status.first_edge_flags |=
                FOC_TIME_SYNC_STM32G431_CONTROL_TICK_VALID;
        }
        if ((g_foc_time_sync_tx_status.first_edge_flags &
             FOC_TIME_SYNC_STM32G431_LOOPBACK_VALID) != 0U)
        {
            g_foc_time_sync_tx_status.loopback_delta_cycles =
                (int32_t)(
                    g_foc_time_sync_tx_status.loopback_edge_cycle_tick -
                    cycle_tick);
        }
        g_foc_time_sync_tx_status.first_edge_count =
            foc_time_sync_stm32g431_saturating_increment(
                g_foc_time_sync_tx_status.first_edge_count);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *timer)
{
    if ((timer != &g_foc_time_sync_tim4) ||
        (timer->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }
    /* DMA transfer-complete is raised immediately after the final guard
     * falling edge loads the never-executed sentinel compare.  PB6 is wired
     * to DengFOC's former I2C SDA and is externally pulled high, so disabling
     * CC1 while the pin is still in AF mode would truncate the guard-low
     * interval.  Take the already-low line over as GPIO before stopping the
     * timer; the receiver then owns the idle timeout that terminates capture. */
    foc_time_sync_stm32g431_pb6_low();
    foc_time_sync_stm32g431_pb7_input(0U);
    CLEAR_BIT(TIM4->DIER,
              TIM_DIER_CC1IE | TIM_DIER_CC1DE | TIM_DIER_CC2IE);
    CLEAR_BIT(TIM4->CCER, TIM_CCER_CC1E | TIM_CCER_CC2E);
    CLEAR_BIT(TIM4->CR1, TIM_CR1_CEN);
    g_foc_time_sync_tx_status.completed_count =
        foc_time_sync_stm32g431_saturating_increment(
            g_foc_time_sync_tx_status.completed_count);
    g_foc_time_sync_tx_status.last_hal_status = (uint32_t)HAL_OK;
    g_foc_time_sync_tx_status.state =
        (uint32_t)FOC_TIME_SYNC_STM32G431_TX_COMPLETE;
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *timer)
{
    if (timer == &g_foc_time_sync_tim4)
    {
        foc_time_sync_stm32g431_tx_fail((uint32_t)HAL_ERROR);
    }
}

#endif /* FLUXRT_H3_TIME_SYNC_BUILD */
