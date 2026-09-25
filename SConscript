# FluxRT —— 工程根构建描述（SCons 侧）。
# FluxRT - project root build description (SCons side).
#
# 职责 / Responsibility:
#   - 注入两个目标宏：STM32G431xx（HAL/CMSIS 需要它选择寄存器定义）与
#     FOC_TARGET_STM32G431（FluxRT 自己的目标判定，见 foc/SConscript 与平台适配层）；
#   - 遍历本目录下的子目录，凡是含 SConscript 就执行并收集其目标文件。
#   - Injects the two target macros: STM32G431xx, which the HAL/CMSIS headers need to pick
#     their register definitions, and FOC_TARGET_STM32G431, FluxRT's own target switch; and
#     walks the subdirectories, running every SConscript it finds and collecting the
#     objects.
#
# 为什么用遍历 / Why the walk:
#   新增子目录只要放一个 SConscript 就自动参与构建，不需要改本文件；但 groups 的顺序
#   因此取决于 os.listdir 的返回顺序，任何跨组的链接顺序依赖都不能依赖这里的次序。
# A new subdirectory only needs an SConscript to take part, with no edit here; the
# consequence is that group order follows os.listdir, so no cross-group link-order
# dependency may rely on this sequence.
import os
Import('env')
from building import *

cwd = GetCurrentDir()
objs = []

env.Append(CPPDEFINES = [
    'STM32G431xx',
    'FOC_TARGET_STM32G431',
])

# 只把"目录 + SConscript"当作子构建单元，普通文件被忽略，因此这里不需要维护清单。
# Only a directory containing an SConscript counts as a sub-build unit and plain files are
# ignored, so no manifest needs to be maintained here.
for item in os.listdir(cwd):
    path = os.path.join(cwd, item, 'SConscript')
    if os.path.isfile(path):
        objs += SConscript(path)

Return('objs')
