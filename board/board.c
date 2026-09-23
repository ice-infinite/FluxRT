#include <board.h>
#include <drv_common.h>

#if (HSE_VALUE != 24000000UL)
#error "FluxRT STM32G431 reference target requires a 24 MHz external HSE crystal"
#endif

#if (LSE_VALUE != 32768UL)
#error "FluxRT STM32G431 reference target requires a 32.768 kHz external LSE crystal"
#endif

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* External 32.768 kHz crystal for the backup/RTC clock domain. */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /*
     * NUCLEO-G431RB X3: external 24 MHz HSE / 6 * 85 / 2 = 170 MHz.
     * PLLP / 8 supplies the ADC12 asynchronous clock at 42.5 MHz, matching
     * the ST IHM16M1 reference project.
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

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_8) != HAL_OK)
    {
        Error_Handler();
    }

    peripheral.PeriphClockSelection = RCC_PERIPHCLK_LPUART1 |
                                      RCC_PERIPHCLK_RTC;
    peripheral.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    peripheral.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK)
    {
        Error_Handler();
    }
}
