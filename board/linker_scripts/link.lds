/*
 * FluxRT —— STM32G431RBT6 链接脚本（NUCLEO-G431RB 验证基线）。
 * FluxRT - STM32G431RBT6 linker script (NUCLEO-G431RB validation baseline).
 *
 * 职责 / Responsibility:
 *   - 给出物理内存边界、段顺序与 RT-Thread 启动所需的符号；
 *   - 导出 `__bss_end` / `_end`，RT-Thread 以 `__bss_end` 作为堆起点。
 *   - Defines the physical memory bounds, section order and the symbols the RT-Thread
 *     boot path needs, and exports `__bss_end` / `_end`, with the heap starting at
 *     `__bss_end`.
 *
 * 内存布局 / Memory layout:
 *   Flash 128 KiB @ 0x08000000（ROM，只读可执行）；SRAM 32 KiB @ 0x20000000
 *   （RAM，可读写可执行）。这两个数值必须与 board/board.h 里的
 *   STM32_FLASH_SIZE / STM32_SRAM_SIZE / STM32_SRAM_END 保持一致：board.h 用它们
 *   计算堆的终点，本文件用它们限制段的终点。不一致会出现两种后果——堆伸进 .stack
 *   区域，或者堆越过物理 SRAM 末尾。
 *   Flash 128 KiB at 0x08000000 (ROM, read-only executable) and SRAM 32 KiB at
 *   0x20000000 (RAM, read/write/executable). These numbers must stay consistent with
 *   STM32_FLASH_SIZE / STM32_SRAM_SIZE / STM32_SRAM_END in board/board.h: board.h uses
 *   them to compute the heap end while this file uses them to bound the sections.
 *   A mismatch either lets the heap grow into the .stack area or past the end of the
 *   physical SRAM.
 *
 * Flash 是最紧张资源 / Flash is the tightest resource:
 *   128 KiB 里 Diagnostic 档已用掉约 122.8 KB（余量约 8 KB），因此任何以"加功能"
 *   为名的改动都必须先看本文件里 ROM 的长度和构建后的 text 大小。
 *   The Diagnostic profile already uses about 122.8 KB of the 128 KiB (roughly 8 KB of
 *   headroom), so any change that adds features must first be checked against the ROM
 *   length here and the built text size.
 *
 * 参考 / Reference: board/board.h, docs/构建档与优化等级.md
 */
MEMORY
{
    ROM (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(Reset_Handler)
/* 主栈大小 [bytes]，由 .stack 段预留 1 KiB；RT-Thread 的线程栈是另外分配的，
 * 这里只放 Reset_Handler 到调度器启动前的主栈以及异常入口。
 * Main stack size in bytes, 1 KiB reserved by the .stack section. RT-Thread thread
 * stacks are allocated separately; this only covers the main stack from
 * Reset_Handler until the scheduler starts, plus exception entry. */
_system_stack_size = 0x400;

SECTIONS
{
    /* .text 放在 Flash 起点：向量表必须最先，Reset_Handler 才会被正确的入口地址取到。
     * .text stays at the start of Flash: the vector table has to come first so
     * Reset_Handler is fetched from the expected entry address. */
    .text :
    {
        . = ALIGN(4);
        _stext = .;
        KEEP(*(.isr_vector))
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        *(.gnu.linkonce.t*)

        /* MSH/finsh 命令表与 RT-Thread 组件初始化表依赖段的相对顺序，因此必须 KEEP
         * 并按名字排序：链接器在本段内按记录顺序遍历，运行期不再排序。
         * The MSH/finsh command table and the RT-Thread component init table depend on
         * the relative order of their records, so they are KEEP-ed and sorted by name:
         * the runtime walks them in link order and never sorts. */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;

        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;

        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;

        /* C 构造函数（RT-Thread 之前也可能有全局对象）。.ctors 的 SORT 保证依赖顺序。
         * C constructors, in case RT-Thread or a package uses global objects; the SORT
         * keeps the dependency order. */
        . = ALIGN(4);
        PROVIDE(__ctors_start__ = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__ctors_end__ = .);

        . = ALIGN(4);
        _etext = .;
    } > ROM

    /* 异常展开表与索引表必须跟 .text 同区，否则 Cortex-M4 的异常展开会读到无效地址。
     * The exception unwind tables and the index table must stay in the same region as
     * .text, otherwise Cortex-M4 exception unwinding reads invalid addresses. */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > ROM

    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > ROM
    __exidx_end = .;

    /* .data 的初值存放在 Flash（LOADADDR），启动代码用 _sidata -> _sdata 搬运到 RAM。
     * 因此 .data 同时占用 Flash 与 SRAM，改一个全局初值会同时影响两处统计。
     * The .data initial values live in Flash (LOADADDR) and the startup code copies them
     * from _sidata to _sdata in RAM, so .data consumes both Flash and SRAM and changing
     * one global initialiser moves both numbers. */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data.*)
        *(.gnu.linkonce.d*)
        PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .);
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > ROM

    /* 主栈：NOLOAD 表示不占用 Flash 初值，只在 SRAM 里预留 1 KiB。对齐到 8 是因为
     * ARM EABI 要求栈指针对齐到 8 字节，未对齐会在浮点/双字访问时触发 UsageFault。
     * Main stack: NOLOAD means no Flash initialiser, only a 1 KiB reservation in SRAM.
     * The 8-byte alignment comes from the ARM EABI stack alignment requirement; a
     * misaligned stack faults on double-word and floating-point accesses. */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + _system_stack_size;
        . = ALIGN(8);
        _estack = .;
    } > RAM

    /* `__bss_start` 在 .bss 输出段之前赋值，`. = ALIGN(4)` 在段内把起址对齐；
     * `__bss_end` 在段之后赋值，并且是 RT-Thread 的堆起点（见 board/board.h 的
     * HEAP_BEGIN）。.bss 清零范围由这两个符号界定，堆不能早于 `__bss_end`。
     * `__bss_start` is assigned before the .bss output section and the alignment inside
     * the section fixes its start; `__bss_end` is assigned after the section and is the
     * RT-Thread heap start (see HEAP_BEGIN in board/board.h). The zeroing range is
     * bounded by those two symbols, and the heap must never begin before `__bss_end`. */
    __bss_start = .;
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        *(.bss.init)
        . = ALIGN(4);
        _ebss = .;
    } > RAM
    /* 堆起点（RT-Thread 用 HEAP_BEGIN = &__bss_end）；`_end` 是等价的传统别名。
     * 这里到 board.h 的 STM32_SRAM_END(0x20008000) 之间就是堆空间，因此 .bss 每
     * 增长 1 字节，堆就少 1 字节——trace 环形缓冲的 3,072 B 就是这样计费的。
     * Heap start (RT-Thread uses HEAP_BEGIN = &__bss_end); `_end` is the equivalent
     * traditional alias. The space from here to STM32_SRAM_END (0x20008000) in board.h
     * is the heap, so every byte of .bss costs one byte of heap: the 3,072-byte trace
     * ring buffer is charged exactly this way. */
    __bss_end = .;
    _end = .;

    .ARM.attributes 0 : { *(.ARM.attributes) }
    .comment 0 : { *(.comment) }
    .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_aranges 0 : { *(.debug_aranges) }
    .debug_line 0 : { *(.debug_line) }
    .debug_frame 0 : { *(.debug_frame) }
    .debug_str 0 : { *(.debug_str) }
    .debug_loc 0 : { *(.debug_loc) }
    .debug_ranges 0 : { *(.debug_ranges) }
}
