# FluxRT —— RT-Thread 交叉编译工具链与全局编译选项（SCons 侧）。
# FluxRT - RT-Thread cross toolchain and global compile options (SCons side).
#
# 职责 / Responsibility:
#   - 声明目标架构（Cortex-M4F / thumbv7em）、工具链前缀与各工具路径；
#   - 给出 C/汇编/链接选项，包括目标宏、内存布局脚本与产物后处理（bin/hex/size）。
#   - Declares the target architecture (Cortex-M4F / thumbv7em), the toolchain prefix and
#     every tool path, and provides the C, assembly and link options including the target
#     macros, the memory-layout script and the artefact post-processing (bin/hex/size).
#
# 与 CMake 路径的关系 / Relation to the CMake path:
#   本文件同时是 SCons 直接构建与 CMake 生成（`scons --target=cmake`）的输入：CMake 侧
#   的编译选项从这里的 CFLAGS/AFLAGS/LFLAGS 抄过去，所以改动本文件会影响两条构建路径。
#   This file feeds both a direct SCons build and CMake generation (`scons --target=cmake`):
#   the CMake side copies CFLAGS/AFLAGS/LFLAGS from here, so editing this file affects both
#   build paths.
#
# 边界 / Boundary:
#   Rust 的优化等级**不**在这里：它由 custom.cmake 的 FLUXRT_RUST_OPT_LEVEL 决定，因为
#   Flash 预算（STM32G431RBT6 只有 128 KiB，是最紧的资源）要求 Rust 档位是实测选择。
#   切换档位或优化等级后必须重新记录 fluxrt.bin 的 SHA-256、text/data/bss 与板端 WCET，
#   见 docs/构建档与优化等级.md。
#   The Rust opt-level is deliberately not here: it comes from FLUXRT_RUST_OPT_LEVEL in
#   custom.cmake, because the Flash budget (the STM32G431RBT6 has only 128 KiB and it is the
#   tightest resource) requires the Rust setting to be a measured choice. After changing the
#   profile or opt-level the fluxrt.bin SHA-256, text/data/bss and on-board WCET must be
#   re-recorded; see docs/构建档与优化等级.md.
import os

ARCH = 'arm'
CPU = 'cortex-m4'
CROSS_TOOL = os.getenv('RTT_CC', 'gcc')
BSP_LIBRARY_TYPE = None

# 编译期拒绝非 GCC 工具链：本工程的启动文件、链接脚本与内联汇编都按 GCC 语法编写，
# 换成 IAR/Keil 会在链接阶段才以难以定位的方式失败，因此宁可在配置阶段直接报错。
# Rejecting non-GCC toolchains at configure time is deliberate: the startup code, linker
# script and inline assembly in this project are GCC-specific, and switching to IAR or Keil
# would fail late and confusingly during linking, so an early error is preferable.
if CROSS_TOOL != 'gcc':
    raise RuntimeError('This project currently supports the GCC toolchain only.')

PLATFORM = 'gcc'
# RTT_EXEC_PATH 由 build.ps1 设为实际工具链 bin 目录；缺省值只是"没有设置环境变量时的
# 兜底"，与 build.ps1 实际使用的路径无关。
# RTT_EXEC_PATH is set by build.ps1 to the real toolchain bin directory; the default is only
# a fallback for a session without that variable and is unrelated to build.ps1's path.
EXEC_PATH = os.getenv('RTT_EXEC_PATH', r'C:\Program Files (x86)\GNU Arm Embedded Toolchain\bin')

PREFIX = 'arm-none-eabi-'
CC = PREFIX + 'gcc'
AS = PREFIX + 'gcc'
AR = PREFIX + 'ar'
CXX = PREFIX + 'g++'
LINK = PREFIX + 'gcc'
TARGET_EXT = 'elf'
SIZE = PREFIX + 'size'
OBJDUMP = PREFIX + 'objdump'
OBJCPY = PREFIX + 'objcopy'

# 目标 ISA 与浮点 ABI：Cortex-M4F、thumb、单精度 FPU（fpv4-sp-d16）、硬浮点。硬浮点必须
# 与 Rust 的 thumbv7em-none-eabihf 目标一致，否则链接期会出现 ABI 不匹配的调用约定。
# Target ISA and float ABI: Cortex-M4F, thumb, single-precision FPU (fpv4-sp-d16), hard
# float. This must match Rust's thumbv7em-none-eabihf target, otherwise the two sides link
# with incompatible calling conventions.
DEVICE = ' -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections'
# 全局 C 选项：gnu11、-Os（体积优先，因为 Flash 是最紧资源）、dwarf-2 -g3 保留完整调试
# 信息以便对 WCET 热点定位。实时适配层随后由 custom.cmake 用 -O3 单独覆盖。
# Global C options: gnu11, -Os (size first, because Flash is the tightest resource) and
# dwarf-2 -g3 to keep full debug info for WCET hot-spot analysis. The realtime adapter is
# then compiled with -O3 by custom.cmake.
CFLAGS = DEVICE + ' -Dgcc -std=gnu11 -Wall -Os -gdwarf-2 -g3'
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb -gdwarf-2'
# --gc-sections 回收未引用段（配合 -ffunction-sections/-fdata-sections，是 Flash 预算的
# 主要手段）；-u Reset_Handler 保证入口不被回收；-Map=fluxrt.map 供体积分析；-cref 输出
# 交叉引用表；-T 指定 board/linker_scripts/link.lds（内存布局见该文件与 board/board.h）。
# --gc-sections reclaims unreferenced sections (together with -ffunction-sections and
# -fdata-sections this is the main Flash-budget tool); -u Reset_Handler keeps the entry
# point alive; -Map=fluxrt.map feeds size analysis; -cref emits a cross-reference table; -T
# selects board/linker_scripts/link.lds, whose memory layout is documented there and in
# board/board.h.
LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=fluxrt.map,-cref,-u,Reset_Handler -T board/linker_scripts/link.lds'
CXXFLAGS = CFLAGS + ' -fno-exceptions -fno-rtti'

# 链接后处理：生成烧录用的 bin/hex 并打印 text/data/bss。这三项是每次改动的体积证据，
# 与板端 WCET 一起构成发布记录。
# Post-link processing: produce the bin/hex images and print text/data/bss. These numbers
# are the size evidence for every change and, together with the on-board WCET, form the
# release record.
POST_ACTION = (
    OBJCPY + ' -O binary $TARGET fluxrt.bin\n' +
    OBJCPY + ' -O ihex $TARGET fluxrt.hex\n' +
    SIZE + ' $TARGET\n'
)

# RT-Thread 的 sdk_dist 打包钩子。本工程不使用 SDK 分发流程，因此这里只保留 RT-Thread
# 的标准实现，不做任何 FluxRT 定制。
# RT-Thread's sdk_dist packaging hook. This project does not use the SDK distribution flow,
# so this is the stock RT-Thread implementation with no FluxRT customisation.
def dist_handle(BSP_ROOT, dist_dir):
    import sys
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
