# FluxRT —— 顶层 SCons 构建入口（rtconfig.h 与 CMakeLists.txt 的生成器）。
# FluxRT - top-level SCons build entry point (the generator for rtconfig.h and
# CMakeLists.txt).
#
# 职责 / Responsibility:
#   - 定位 RT-Thread 根目录并导入 building.py；
#   - 注册两个 PreBuilding 动作：STM32G4 软件包存在性检查、Rust 静态库交叉构建；
#   - 用 rtconfig.py 的工具链设置建立 Environment，然后 PrepareBuilding/DoBuilding
#     产出 fluxrt.elf，并由 rtconfig.POST_ACTION 生成 bin/hex 与 size 报告。
#   - Locates the RT-Thread root and imports building.py; registers two pre-build actions
#     (the STM32G4 package check and the Rust static-library cross build); builds the
#     Environment from rtconfig.py's toolchain settings; then PrepareBuilding/DoBuilding
#     produce fluxrt.elf, from which rtconfig.POST_ACTION generates the bin/hex files and
#     the size report.
#
# 与 build.ps1 的关系 / Relation to build.ps1:
#   build.ps1 用 `scons --pyconfig-silent` 生成 rtconfig.h、用 `scons --target=cmake -s`
#   生成 CMakeLists.txt；直接运行 SCons 则走本文件的完整路径，语义是 Diagnostic 且
#   Rust 优化等级取 rust/Cargo.toml 的默认值。日常与 Production 构建统一走 build.ps1。
#   build.ps1 uses `scons --pyconfig-silent` for rtconfig.h and `scons --target=cmake -s`
#   for CMakeLists.txt; running SCons directly takes the full path here, which means
#   Diagnostic semantics with the Rust opt-level default from rust/Cargo.toml. Day-to-day
#   and Production builds go through build.ps1.
#
# 注意 / Caveat:
#   本文件里的 Rust 构建是"预构建"动作，产物固定放在 build/rust-target，且不区分优化
#   等级；CMake 路径下 Rust 由 custom.cmake 按档位独立构建。两者不要混用构建目录。
#   The Rust build here is a pre-build action with a fixed build/rust-target output and no
#   opt-level separation; on the CMake path Rust is built per profile by custom.cmake. Do
#   not mix the two build directories.
import os
import shutil
import subprocess
import sys
import rtconfig

if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')
else:
    # 默认按"本工程位于 <workspace>/projects/FluxRT"推导，与 build.ps1 的布局一致；
    # 布局不同时必须显式设置 RTT_ROOT。
    # Defaults to assuming this project sits at <workspace>/projects/FluxRT, matching the
    # layout build.ps1 expects. RTT_ROOT must be set explicitly for any other layout.
    RTT_ROOT = os.path.normpath(os.getcwd() + '/../../rt-thread')

sys.path += [os.path.join(RTT_ROOT, 'tools')]
try:
    from building import *
except ImportError:
    print('Cannot find RT-Thread root directory:')
    print(RTT_ROOT)
    exit(-1)

# 软件包存在性检查：缺 STM32G4 包时在构建最开始就给出可执行的修复命令，而不是让
# 编译器在几百个 include 里报错。RegisterPreBuildingAction 保证它在真正编译前运行。
# The package check reports an actionable fix at the very start of the build instead of
# letting the compiler fail across hundreds of includes.
# RegisterPreBuildingAction makes it run before any compilation.
def bsp_pkg_check():
    required = [
        os.path.join('packages', 'CMSIS-Core-latest'),
        os.path.join('packages', 'stm32g4_cmsis_driver-latest'),
        os.path.join('packages', 'stm32g4_hal_driver-latest'),
    ]
    if not all(os.path.exists(path) for path in required):
        print('STM32G4 packages are missing. Run: .\\build.ps1 -UpdatePackages -Regenerate')
        exit(1)

RegisterPreBuildingAction(bsp_pkg_check)

# Rust 静态库交叉构建（SCons 路径；CMake 路径见 custom.cmake）。
# Rust static-library cross build on the SCons path; the CMake path is in custom.cmake.
#
# 为什么在这里调 cargo 而不是用 CMake：SCons 直接驱动链接，没有 custom command 机制，
# 只能作为预构建动作先把 libfoc_rt_bridge.a 准备好。--locked 固定依赖版本，因为 Flash
# 体积与板端 WCET 都是被记录的发布证据，不能被依赖漂移改变。
# Why cargo is invoked here instead of through CMake: SCons drives the link directly and
# has no custom-command mechanism, so the archive must be prepared as a pre-build action.
# --locked pins the dependency versions because the Flash size and the on-board WCET are
# recorded release evidence that dependency drift must not change.
def rust_library_build():
    project_dir = os.getcwd()
    rust_dir = os.path.join(project_dir, 'rust')
    target_dir = os.path.join(project_dir, 'build', 'rust-target')
    cargo = shutil.which('cargo')
    if cargo is None:
        cargo = os.path.join(os.path.expanduser('~'), '.cargo', 'bin', 'cargo.exe')
    if not os.path.isfile(cargo):
        print('Rust Cargo was not found. Install Rust stable and thumbv7em-none-eabihf.')
        exit(1)

    rust_env = os.environ.copy()
    rust_env['CARGO_TARGET_DIR'] = target_dir
    command = [
        cargo,
        'build',
        '--manifest-path', os.path.join(rust_dir, 'Cargo.toml'),
        '--package', 'foc-rt-bridge',
        '--release',
        '--target', 'thumbv7em-none-eabihf',
        '--locked',
    ]
    # CORDIC 特性来自 Kconfig（生成到 rtconfig.h，由 GetDepend 读取）。关掉它只是让
    # Rust 走 CPU 数学回退，算法行为不变，只有周期数变化。
    # The CORDIC feature comes from Kconfig (generated into rtconfig.h and read through
    # GetDepend). Disabling it only sends Rust to the CPU math fallback: cycles change,
    # algorithms do not.
    if GetDepend('FOC_MATH_BACKEND_STM32G4_CORDIC'):
        command.extend(['--features', 'stm32g4-cordic'])
    result = subprocess.run(command, cwd=rust_dir, env=rust_env)
    if result.returncode != 0:
        print('Rust FOC static library build failed.')
        exit(result.returncode)

RegisterPreBuildingAction(rust_library_build)

TARGET = 'fluxrt.' + rtconfig.TARGET_EXT

# DefaultEnvironment(tools=[]) 清空默认工具探测：交叉编译的编译器/链接器全部由
# rtconfig.py 显式给出，避免 SCons 拿宿主的 gcc 去链接目标代码。
# DefaultEnvironment(tools=[]) clears the default tool detection: the cross compiler and
# linker are given explicitly by rtconfig.py, so SCons can never link target code with the
# host gcc.
DefaultEnvironment(tools=[])
env = Environment(
    tools=['mingw'],
    AS=rtconfig.AS,
    ASFLAGS=rtconfig.AFLAGS,
    CC=rtconfig.CC,
    CFLAGS=rtconfig.CFLAGS,
    AR=rtconfig.AR,
    ARFLAGS='-rc',
    CXX=rtconfig.CXX,
    CXXFLAGS=rtconfig.CXXFLAGS,
    LINK=rtconfig.LINK,
    LINKFLAGS=rtconfig.LFLAGS,
)
env.PrependENVPath('PATH', rtconfig.EXEC_PATH)

Export('env')
Export('RTT_ROOT')
Export('rtconfig')

stm32_libraries = os.path.join(RTT_ROOT, 'bsp', 'stm32', 'libraries')

# HAL_Drivers 组提供 RT-Thread 的 console/pin/serial 驱动框架（LPUART1 控制台就是
# 从这里来的）；FOC 自己的 ADC/TIM HAL 源文件在 foc/SConscript 里单独编入。
# The HAL_Drivers group supplies the RT-Thread console/pin/serial driver framework, which
# is where the LPUART1 console comes from; FOC's own ADC/TIM HAL sources are compiled
# separately in foc/SConscript.
objs = PrepareBuilding(env, RTT_ROOT, has_libcpu=False)
objs.extend(SConscript(
    os.path.join(stm32_libraries, 'HAL_Drivers', 'SConscript'),
    variant_dir='build/libraries/HAL_Drivers',
    duplicate=0,
))

DoBuilding(TARGET, objs)
