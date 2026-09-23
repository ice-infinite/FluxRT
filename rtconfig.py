import os

ARCH = 'arm'
CPU = 'cortex-m4'
CROSS_TOOL = os.getenv('RTT_CC', 'gcc')
BSP_LIBRARY_TYPE = None

if CROSS_TOOL != 'gcc':
    raise RuntimeError('This project currently supports the GCC toolchain only.')

PLATFORM = 'gcc'
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

DEVICE = ' -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections'
CFLAGS = DEVICE + ' -Dgcc -std=gnu11 -Wall -Os -gdwarf-2 -g3'
AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb -gdwarf-2'
LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=fluxrt.map,-cref,-u,Reset_Handler -T board/linker_scripts/link.lds'
CXXFLAGS = CFLAGS + ' -fno-exceptions -fno-rtti'

POST_ACTION = (
    OBJCPY + ' -O binary $TARGET fluxrt.bin\n' +
    OBJCPY + ' -O ihex $TARGET fluxrt.hex\n' +
    SIZE + ' $TARGET\n'
)

def dist_handle(BSP_ROOT, dist_dir):
    import sys
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
