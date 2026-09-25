/*
 * FluxRT —— 板级启动：时钟树配置（NUCLEO-G431RB）。
 * FluxRT - board bring-up: clock tree configuration (NUCLEO-G431RB).
 *
 * 职责 / Responsibility:
 *   建立本项目全部时序所依赖的时钟：
 *     - SYSCLK 170 MHz，供 TIM1 计数、CPU 执行和 DWT 计时；
 *     - ADC12 异步时钟 42.5 MHz，与 ST IHM16M1 参考工程一致；
 *     - RTC 时钟源 LSE 32.768 kHz。
 *
 *   Establishes every clock the project depends on: 170 MHz SYSCLK for TIM1,
 *   the CPU and DWT; a 42.5 MHz asynchronous ADC12 clock matching the ST
 *   IHM16M1 reference; and LSE as the RTC source.
 *
 * 编译期强制 / Compile-time enforcement:
 *   文件开头的 #error 要求存在 24 MHz HSE 与 32.768 kHz LSE 晶振。这是刻意的：
 *   本项目所有派生量（PWM 周期、电流环 ts、观测器 dt、死区归一化）都按
 *   170 MHz 推导，静默回落到内部 RC 会让时序整体偏移而不报错。
 *   The #error guards require a 24 MHz HSE and a 32.768 kHz LSE crystal. This
 *   is deliberate: every derived quantity in this project (PWM period, current
 *   loop ts, observer dt, dead-time normalisation) is derived from 170 MHz, and
 *   a silent fallback to the internal RC would shift all timing without any
 *   error being reported.
 *
 * 失败策略 / Failure policy:
 *   HSE 或 LSE 起振失败直接进入 Error_Handler()，不会换用另一套控制时基。
 *   电机控制里"时钟悄悄变了"比"启动失败"危险得多。
 *   A failed HSE or LSE start goes straight to Error_Handler(); the project
 *   never switches to a different control time base. In motor control a clock
 *   that silently changed is far more dangerous than a failed boot.
 */

#include <board.h>
#include <drv_common.h>

/* 与 board.h / rtconfig.h 中的期望值做编译期一致性检查。
 * Compile-time consistency check against the expected values. */
#if (HSE_VALUE != 24000000UL)
#error "FluxRT STM32G431 reference target requires a 24 MHz external HSE crystal"
#endif

#if (LSE_VALUE != 32768UL)
#error "FluxRT STM32G431 reference target requires a 32.768 kHz external LSE crystal"
#endif

/*
 * 配置系统时钟、外设时钟源与闪存等待周期。
 * Configures the system clock, peripheral clock sources and Flash latency.
 *
 * 上下文 / Context: 调度器启动前调用一次，非实时路径，允许阻塞等待起振。
 * Called once before the scheduler starts; not a real-time path, so blocking on
 * oscillator start-up is acceptable.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral = {0};

    /* 170 MHz 需要 Boost 模式调压器，否则内核无法在该频率下稳定工作。
     * 170 MHz requires the boost-mode regulator; the core is not stable at this
     * frequency under the default scale. */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* 外部 32.768 kHz 晶振用于备份/RTC 时钟域。
     * External 32.768 kHz crystal for the backup/RTC clock domain.
     *
     * LSE 驱动设为 LOW：NUCLEO 板载晶振的 ESR 较低，提高驱动强度反而可能
     * 造成过驱。更换晶振或自研板时必须重新评估。
     * LSEDRIVE is set to LOW: the on-board crystal has low ESR and a stronger
     * drive could over-drive it. Re-evaluate when changing the crystal or
     * moving to a custom board. */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /*
     * NUCLEO-G431RB X3：外部 24 MHz HSE / 6 * 85 / 2 = 170 MHz。
     * PLLP / 8 提供 ADC12 异步时钟 42.5 MHz，与 ST IHM16M1 参考工程一致。
     *
     * NUCLEO-G431RB X3: external 24 MHz HSE / 6 * 85 / 2 = 170 MHz.
     * PLLP / 8 supplies the ADC12 asynchronous clock at 42.5 MHz, matching the
     * ST IHM16M1 reference project.
     *
     * 各分频的作用 / Role of each divider:
     *   PLLM = /6   HSE 24 MHz -> 4 MHz 参考
     *   PLLN = 85   4 MHz * 85 = 340 MHz VCO
     *   PLLR = /2   340 MHz / 2 = 170 MHz SYSCLK（也是 TIM1 的时钟源）
     *   PLLP = /8   340 MHz / 8 = 42.5 MHz ADC12（参考工程用 /8，不是 /2）
     *   PLLQ = /2   340 MHz / 2 = 170 MHz，本项目未使用
     *
     * 改动任何一项都会同时改变 PWM 周期、电流环 ts、观测器 dt 和死区归一化，
     * 必须重新验证全部时序。
     * Changing any of these also changes the PWM period, current loop ts,
     * observer dt and dead-time normalisation; all timing must be re-verified.
     */
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE |
                                RCC_OSCILLATORTYPE_LSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.LSEState = RCC_LSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = RCC_PLLM_DIV6;
    oscillator.PLL.PLLN = 85;
    oscillator.PLL.PLLP = RCC_PLLP_DIV8;
    oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
    oscillator.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
    {
        Error_Handler();
    }

    /* AHB 与两条 APB 都不分频：TIM1 直接跑在 170 MHz，LPUART1 跑在 PCLK1。
     * AHB and both APB buses run undivided: TIM1 runs at 170 MHz and LPUART1 at
     * PCLK1. */
    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    /* FLASH_LATENCY_8 是 170 MHz（Boost、VOS1）下的必要等待周期。
     * 设小了会读到错误指令，表现为随机崩溃而不是编译错误。
     * FLASH_LATENCY_8 is required at 170 MHz (boost, VOS1). Setting it too low
     * causes wrong instruction reads, i.e. random crashes rather than a build
     * error. */
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_8) != HAL_OK)
    {
        Error_Handler();
    }

    /* 外设时钟源选择 / Peripheral clock source selection:
     *   LPUART1 取自 PCLK1（115200 控制台，见 board/Kconfig）
     *   RTC 取自 LSE，为将来长时间脱机记录保留低功耗时基
     *   LPUART1 from PCLK1 (the 115200 console); RTC from LSE, keeping a
     *   low-power time base for future long unattended logging. */
    peripheral.PeriphClockSelection = RCC_PERIPHCLK_LPUART1 |
                                      RCC_PERIPHCLK_RTC;
    peripheral.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    peripheral.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK)
    {
        Error_Handler();
    }
}
