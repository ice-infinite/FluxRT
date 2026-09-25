/*
 * FluxRT —— STM32G4 HAL 配置头（裁剪版，只保留本工程实际编译的模块）。
 * FluxRT - STM32G4 HAL configuration header (trimmed to the modules this project compiles).
 *
 * 这是什么 / What this is:
 *   ST 的 HAL 源文件（packages/stm32g4_hal_driver-latest/Src/ 下的 *.c）整个文件被
 *   `#ifdef HAL_<外设>_MODULE_ENABLED` 包住，每个句柄的"可注册回调"成员被
 *   `#if (USE_HAL_<外设>_REGISTER_CALLBACKS == 1)` 包住。本文件因此同时是
 *   "哪些外设参与编译"和"哪些回调指针占用 RAM/Flash"的开关集合，由 RT-Thread 的
 *   HAL 驱动与 foc/platform/stm32g431/ 共同包含。
 *   Each ST HAL source file (the *.c files under
 *   packages/stm32g4_hal_driver-latest/Src/) is wrapped in
 *   `#ifdef HAL_<peripheral>_MODULE_ENABLED`, and each handle's registerable-callback
 *   members are wrapped in `#if (USE_HAL_<peripheral>_REGISTER_CALLBACKS == 1)`. This file
 *   is therefore both the "which peripherals are compiled" and the "which callback
 *   pointers cost RAM/Flash" switch set, included by the RT-Thread HAL drivers and by
 *   foc/platform/stm32g431/.
 *
 * FluxRT 的选择 / FluxRT's choices:
 *   - 使能：GPIO、EXTI、DMA、RCC、FLASH、PWR、CORTEX、UART、ADC、TIM。
 *     这正好覆盖控制台（LPUART1）、功率级（TIM1 + 注入组 ADC）、时钟与低层中断，
 *     没有为未使用的外设支付任何 Flash。
 *     Enabled: GPIO, EXTI, DMA, RCC, FLASH, PWR, CORTEX, UART, ADC and TIM. That is
 *     exactly the console (LPUART1), the power stage (TIM1 plus the injected-group ADC),
 *     the clocks and the low-level interrupts, with no Flash spent on unused peripherals.
 *   - 所有 USE_HAL_*_REGISTER_CALLBACKS 保持 0：HAL 句柄只由 RT-Thread 驱动与 FOC 平台
 *     层直接初始化，不使用回调注册机制，因此不需要在句柄里为回调指针预留空间。
 *     误改成 1 会同时增加 SRAM 与 Flash，并且必须相应修改初始化代码才能生效。
 *     All USE_HAL_*_REGISTER_CALLBACKS stay at 0: HAL handles are initialised directly by
 *     the RT-Thread drivers and the FOC platform layer, never through callback
 *     registration, so the handle has no reason to carry callback pointers. Flipping one
 *     to 1 costs both SRAM and Flash and would have to be matched by initialisation code.
 *
 * 边界 / Boundary:
 *   本文件只选择 HAL 模块，不选择 RT-Thread 驱动：控制台与 pin 框架由 Kconfig
 *   （board/Kconfig）决定，功率级外设由 foc/platform/stm32g431/ 独占初始化。
 *   This file selects HAL modules only, not RT-Thread drivers: the console and pin
 *   framework are chosen in Kconfig (board/Kconfig) and the power-stage peripherals are
 *   initialised exclusively by foc/platform/stm32g431/.
 *
 * 参考 / Reference: board/board.c（时钟）, board/stm32g4xx_hal_msp.c（MSP 钩子）,
 *                   foc/SConscript（哪些 HAL 源文件参与编译）
 */
#ifndef STM32G4XX_HAL_CONF_H
#define STM32G4XX_HAL_CONF_H

/* 使能的 HAL 模块 / Enabled HAL modules:
 *   GPIO/EXTI  - 中断线映射与引脚控制；
 *   DMA/RCC/FLASH/PWR/CORTEX - HAL 基础设施与低层中断；
 *   UART       - LPUART1 控制台（115200，PA2/PA3）；
 *   ADC        - 注入组同步采样与静态监测（由 FOC 平台层驱动）；
 *   TIM        - TIM1 中心对齐 PWM 与 CH4 内部触发。
 *   GPIO/EXTI for line mapping and pin control, DMA/RCC/FLASH/PWR/CORTEX for HAL
 *   infrastructure and low-level interrupts, UART for the LPUART1 console at 115200 on
 *   PA2/PA3, ADC for injected-group synchronous sampling and static monitoring (driven by
 *   the FOC platform layer) and TIM for the TIM1 centre-aligned PWM with its CH4 internal
 *   trigger.
 * 未使能 / Not enabled: COMP、CORDIC、HRTIM、I2C、SPI、DAC、OPAMP、RTC 等。其中 CORDIC
 * 只通过 foc_math_accel_stm32g431.c 直接操作寄存器使用，不需要 HAL CORDIC 模块；未来
 * 若改用 HAL CORDIC API，必须同时打开对应模块与回调设置。
 * COMP, CORDIC, HRTIM, I2C, SPI, DAC, OPAMP, RTC and the rest stay off. CORDIC is driven
 * through direct register access in foc_math_accel_stm32g431.c and needs no HAL module;
 * switching to the HAL CORDIC API later would require enabling the module here. */
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED

/* 全部回调注册保持关闭 [0/1]；见文件头"所有 USE_HAL_*_REGISTER_CALLBACKS 保持 0"。
 * Every callback registration stays off (0/1); see the file header. */
#define USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define USE_HAL_COMP_REGISTER_CALLBACKS       0U
#define USE_HAL_CORDIC_REGISTER_CALLBACKS     0U
#define USE_HAL_CRYP_REGISTER_CALLBACKS       0U
#define USE_HAL_DAC_REGISTER_CALLBACKS        0U
#define USE_HAL_EXTI_REGISTER_CALLBACKS       0U
#define USE_HAL_FDCAN_REGISTER_CALLBACKS      0U
#define USE_HAL_FMAC_REGISTER_CALLBACKS       0U
#define USE_HAL_HRTIM_REGISTER_CALLBACKS      0U
#define USE_HAL_I2C_REGISTER_CALLBACKS        0U
#define USE_HAL_I2S_REGISTER_CALLBACKS        0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS       0U
#define USE_HAL_LPTIM_REGISTER_CALLBACKS      0U
#define USE_HAL_NAND_REGISTER_CALLBACKS       0U
#define USE_HAL_NOR_REGISTER_CALLBACKS        0U
#define USE_HAL_OPAMP_REGISTER_CALLBACKS      0U
#define USE_HAL_PCD_REGISTER_CALLBACKS        0U
#define USE_HAL_QSPI_REGISTER_CALLBACKS       0U
#define USE_HAL_RNG_REGISTER_CALLBACKS        0U
#define USE_HAL_RTC_REGISTER_CALLBACKS        0U
#define USE_HAL_SAI_REGISTER_CALLBACKS        0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS  0U
#define USE_HAL_SMBUS_REGISTER_CALLBACKS      0U
#define USE_HAL_SPI_REGISTER_CALLBACKS        0U
#define USE_HAL_SRAM_REGISTER_CALLBACKS       0U
#define USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define USE_HAL_UART_REGISTER_CALLBACKS       0U
#define USE_HAL_USART_REGISTER_CALLBACKS      0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS       0U

/* 时钟与启动超时 / Clock values and startup timeouts:
 *   HSE_VALUE 24 MHz 必须与 NUCLEO-G431RB 的 X3 晶振一致；board/board.c 用它推导
 *   PLLM 并做编译期 #error 检查，写错会让 170 MHz 与实际不符而没有任何编译错误。
 *   HSE_VALUE (24 MHz) must match the NUCLEO-G431RB X3 crystal; board/board.c derives PLLM
 *   from it and enforces it with a compile-time #error. A wrong value silently changes the
 *   real clock away from 170 MHz.
 *   LSE_VALUE 32.768 kHz 是 RTC 时基，同样有编译期检查。LSI/HSI/HSI48/EXTERNAL_CLOCK 是
 *   HAL 的备用值，本工程不使用对应时钟源，保留模板值。
 *   LSE_VALUE (32.768 kHz) is the RTC time base and is checked at compile time as well. LSI,
 *   HSI, HSI48 and EXTERNAL_CLOCK are HAL fallbacks for clock sources this project does not
 *   use; the template values are kept. */
#ifndef HSE_VALUE
#define HSE_VALUE                 (24000000UL)
#endif
#define HSE_STARTUP_TIMEOUT       (100UL)
#define HSI_VALUE                 (16000000UL)
#define HSI48_VALUE               (48000000UL)
#define LSI_VALUE                 (32000UL)
#define LSE_VALUE                 (32768UL)
#define LSE_STARTUP_TIMEOUT       (5000UL)
#define EXTERNAL_CLOCK_VALUE      (12288000UL)

/* HAL 运行时设置 / HAL runtime settings [mV, priority, 0/1]:
 *   VDD_VALUE 3300 mV 只用于 HAL 的电压相关分支；
 *   TICK_INT_PRIORITY 0 在本工程里**不生效**：RT-Thread 的 drv_common.c 提供了自己的
 *   HAL_InitTick()，它忽略传入的优先级并把 SysTick 优先级设为 0xFF（最低），因此 0 这个
 *   字面值不代表"最高优先级"。真实优先级以 RT-Thread 驱动为准。
 *   TICK_INT_PRIORITY (0) has no effect in this project: RT-Thread's drv_common.c supplies
 *   its own HAL_InitTick(), which ignores the priority argument and sets the SysTick priority
 *   to 0xFF, the lowest. The literal 0 therefore does not mean "highest priority"; the
 *   RT-Thread driver is authoritative.
 *   USE_RTOS = 0 与 RT-Thread 并存是刻意的：HAL 的阻塞等待不需要 RTOS 感知，保持 0 可
 *   避免把 HAL 的 RTOS 适配代码链进镜像；
 *   USE_RTOS = 0 alongside RT-Thread is deliberate: HAL's blocking waits need no RTOS
 *   awareness and keeping it 0 avoids linking HAL's RTOS adaptation.
 *   PREFETCH_ENABLE = 0、INSTRUCTION_CACHE_ENABLE / DATA_CACHE_ENABLE = 1 一起决定
 *   FLASH_ACR 的预取与缓存位，而这直接影响 12 kHz ISR 的取指周期，改动后必须重新测量
 *   板端 WCET；
 *   PREFETCH_ENABLE = 0 with INSTRUCTION_CACHE_ENABLE / DATA_CACHE_ENABLE = 1 sets the
 *   FLASH_ACR prefetch and cache bits, which directly affect instruction fetch cycles in the
 *   12 kHz ISR, so the on-board WCET must be re-measured after any change.
 *   USE_SPI_CRC = 0 因为没有启用 SPI 模块。
 *   USE_SPI_CRC = 0 because the SPI module is not enabled. */
#define VDD_VALUE                 (3300UL)
#define TICK_INT_PRIORITY         (0UL)
#define USE_RTOS                  0U
#define PREFETCH_ENABLE           0U
#define INSTRUCTION_CACHE_ENABLE  1U
#define DATA_CACHE_ENABLE         1U
#define USE_SPI_CRC               0U

/* 只包含已使能模块的头文件，与上面的 HAL_*_MODULE_ENABLED 列表一一对应。
 * Only headers of enabled modules are included, matching the HAL_*_MODULE_ENABLED list
 * above one to one. */
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_exti.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_tim.h"

/* USE_FULL_ASSERT 默认未定义，assert_param 展开为无操作：HAL 的参数检查只在开发排错时
 * 打开，量产/日常构建不留这部分代码与字符串（Flash 只有 128 KiB）。
 * USE_FULL_ASSERT is undefined by default and assert_param expands to nothing: HAL's
 * parameter checks are only enabled while debugging, and normal builds pay neither the code
 * nor the strings, given only 128 KiB of Flash. */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#else
#define assert_param(expr) ((void)0U)
#endif

#endif
