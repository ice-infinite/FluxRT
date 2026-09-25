#ifndef FLUXRT_STM32G431_BOARD_H
#define FLUXRT_STM32G431_BOARD_H

/*
 * FluxRT —— 板级常量与启动声明（NUCLEO-G431RB 验证基线）。
 * FluxRT - board constants and boot declarations (NUCLEO-G431RB baseline).
 *
 * 职责 / Responsibility:
 *   - 给出 Flash / SRAM 的物理边界，供链接脚本与堆分配使用；
 *   - 声明 SystemClock_Config()，由 RT-Thread 启动流程调用。
 *   - Provides the physical Flash/SRAM bounds used by the linker script and
 *     the heap, and declares SystemClock_Config() for the RT-Thread boot path.
 *
 * 为什么这些常量写在头文件里 / Why these constants live here:
 *   RT-Thread 的堆起点取 `__bss_end`，堆终点取 SRAM 末端。两者必须与
 *   board/linker_scripts/link.lds 里的 RAM 区域一致，否则堆会伸进栈区或
 *   越出物理 SRAM。
 *   RT-Thread's heap starts at `__bss_end` and ends at the top of SRAM. Both
 *   must agree with the RAM region in board/linker_scripts/link.lds, otherwise
 *   the heap grows into the stack or past the physical SRAM.
 *
 * 参考 / Reference: docs/2026-09-22实机烧录记录.md
 */

#include <stm32g4xx.h>

/* NUCLEO-G431RB 验证基线：STM32G431RBT6，128 KiB Flash，32 KiB SRAM。
 * NUCLEO-G431RB validation baseline: STM32G431RBT6, 128 KiB Flash, 32 KiB SRAM.
 *
 * 注意 SRAM 为 32 KiB，是当前工程最紧张的资源之一（Flash 更紧张）。
 * Caveat: 32 KiB of SRAM is one of the tightest resources in this project
 * (Flash is tighter still). */
#define STM32_FLASH_START_ADDRESS  ((uint32_t)0x08000000)
#define STM32_FLASH_SIZE           (128U * 1024U)
#define STM32_FLASH_END_ADDRESS    (STM32_FLASH_START_ADDRESS + STM32_FLASH_SIZE)

#define STM32_SRAM_SIZE            (32U * 1024U)
#define STM32_SRAM_END             (0x20000000U + STM32_SRAM_SIZE)

#if defined(__GNUC__)
/* `__bss_end` 由链接脚本提供，标记 .bss 段末尾；堆从它开始。
 * `__bss_end` is provided by the linker script and marks the end of .bss; the
 * heap starts there. */
extern int __bss_end;
#define HEAP_BEGIN                 ((void *)&__bss_end)
#define HEAP_END                   ((void *)STM32_SRAM_END)
#endif

/*
 * 配置 HSE/PLL/LSE 与总线分频。
 * Configures HSE/PLL/LSE and the bus dividers.
 *
 * 由 RT-Thread 的启动流程在调度器运行前调用，只执行一次。
 * Called once by the RT-Thread boot path before the scheduler starts.
 */
void SystemClock_Config(void);

#endif
