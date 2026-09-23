#ifndef FLUXRT_STM32G431_BOARD_H
#define FLUXRT_STM32G431_BOARD_H

#include <stm32g4xx.h>

/* NUCLEO-G431RB validation baseline: STM32G431RBT6, 128 KiB Flash, 32 KiB SRAM. */
#define STM32_FLASH_START_ADDRESS  ((uint32_t)0x08000000)
#define STM32_FLASH_SIZE           (128U * 1024U)
#define STM32_FLASH_END_ADDRESS    (STM32_FLASH_START_ADDRESS + STM32_FLASH_SIZE)

#define STM32_SRAM_SIZE            (32U * 1024U)
#define STM32_SRAM_END             (0x20000000U + STM32_SRAM_SIZE)

#if defined(__GNUC__)
extern int __bss_end;
#define HEAP_BEGIN                 ((void *)&__bss_end)
#define HEAP_END                   ((void *)STM32_SRAM_END)
#endif

void SystemClock_Config(void);

#endif
