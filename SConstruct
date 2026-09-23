import os
import shutil
import subprocess
import sys
import rtconfig

if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')
else:
    RTT_ROOT = os.path.normpath(os.getcwd() + '/../../rt-thread')

sys.path += [os.path.join(RTT_ROOT, 'tools')]
try:
    from building import *
except ImportError:
    print('Cannot find RT-Thread root directory:')
    print(RTT_ROOT)
    exit(-1)

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
    if GetDepend('FOC_MATH_BACKEND_STM32G4_CORDIC'):
        command.extend(['--features', 'stm32g4-cordic'])
    result = subprocess.run(command, cwd=rust_dir, env=rust_env)
    if result.returncode != 0:
        print('Rust FOC static library build failed.')
        exit(result.returncode)

RegisterPreBuildingAction(rust_library_build)

TARGET = 'fluxrt.' + rtconfig.TARGET_EXT

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

objs = PrepareBuilding(env, RTT_ROOT, has_libcpu=False)
objs.extend(SConscript(
    os.path.join(stm32_libraries, 'HAL_Drivers', 'SConscript'),
    variant_dir='build/libraries/HAL_Drivers',
    duplicate=0,
))

DoBuilding(TARGET, objs)
