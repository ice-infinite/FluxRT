# FluxRT —— 目标固件构建脚本（SCons 生成 + CMake/Ninja 构建 + Rust 静态库）。
# FluxRT - target firmware build script (SCons generation, CMake/Ninja build, Rust static library).
#
# 职责 / Responsibility:
#   - 解析工具链（cmake/ninja/scons/pkgs/cargo/arm-none-eabi-gcc），只在本次进程内
#     设置 RTT_ROOT/RTT_CC/RTT_EXEC_PATH/PATH/PKGS_*，退出时恢复；
#   - 需要时用 SCons 生成 rtconfig.h（Kconfig）与 CMakeLists.txt；
#   - 用 CMake 配置并构建固定构建档的固件，最后列出 BIN/HEX/ELF/MAP 体积。
#   - Resolves the toolchain (cmake/ninja/scons/pkgs/cargo/arm-none-eabi-gcc), setting
#     RTT_ROOT/RTT_CC/RTT_EXEC_PATH/PATH/PKGS_* for this process only and restoring
#     them on exit; regenerates rtconfig.h (Kconfig) and CMakeLists.txt through SCons
#     when needed; configures and builds one fixed profile; then lists the sizes of
#     the BIN/HEX/ELF/MAP artefacts.
#
# Flash 预算 / Flash budget（本脚本存在的核心理由 / the main reason this script exists）:
#   STM32G431RBT6 只有 128 KiB Flash，是本工程最紧张的资源（SRAM 32 KiB 反而宽松）。
#   A17 混合 CORDIC/FPU Diagnostic + Rust `s` 的 BIN 为 128,600 B，剩余 2,472 B；
#   其中包含 4 KiB 的相电压诊断固定窗，因此 Rust 优化
#   等级与构建档是**实测选择**而不是猜的：`s` 相对 `3` 回收约 5 KB Flash，完整 ISR
#   WCET 只增加约 2%；`z` 再省 800 B 却让 WCET 增加约 7%。
#   The STM32G431RBT6 has only 128 KiB of Flash, the tightest resource in this
#   project (32 KiB of SRAM is roomier). A17 hybrid CORDIC/FPU Diagnostic plus Rust
#   `s` produces a 128,600-byte BIN, leaving 2,472 bytes; this includes the 4 KiB
#   diagnostic phase-voltage window. The Rust opt-level and the profile are
#   therefore measured choices, not guesses: `s` recovers about 5 KB of Flash over
#   `3` while raising the full ISR WCET by about 2%, and `z` saves another 800 B at
#   the cost of about 7% more WCET.
#
# 切换档位的前提 / Precondition for switching:
#   改动 Profile 或 RustOptLevel 后必须重新记录 fluxrt.bin 的 SHA-256、text/data/bss
#   和板端 WCET；不得凭"构建成功"或"电机能转"就认定可用。见 docs/构建档与优化等级.md。
#   After changing Profile or RustOptLevel the fluxrt.bin SHA-256, the text/data/bss
#   numbers and the on-board WCET must be re-recorded; a successful build or a
#   spinning motor is not evidence of fitness. See docs/构建档与优化等级.md.
#
# 参考 / Reference: docs/构建档与优化等级.md, docs/架构与安全边界.md
param(
    # 重新生成 rtconfig.h 与 CMakeLists.txt（Kconfig 或 SConscript 改动后必须带）。
    # Regenerate rtconfig.h and CMakeLists.txt (required after Kconfig or SConscript edits).
    [switch]$Regenerate,
    # 强制 scons --pyconfig-silent + pkgs --update，重新拉取 STM32G4 软件包。
    # Force scons --pyconfig-silent plus pkgs --update to re-fetch the STM32G4 packages.
    [switch]$UpdatePackages,
    # 只做 CMake 配置不构建，用于核对档位/优化等级是否生效。
    # Configure with CMake but do not build; used to check that profile and opt-level took effect.
    [switch]$ConfigureOnly,
    # 只构建，并校验 CMake cache 里的档位与本次请求一致；不一致直接报错退出。
    # Build only, after checking that the CMake cache profile matches this request;
    # a mismatch aborts instead of mislabelling one configuration as another.
    [switch]$BuildOnly,
    # 删除本档位的构建目录与该优化等级的 Rust 产物，其它档位不受影响。
    # Remove this profile's build directory and this opt-level's Rust artefacts only.
    [switch]$Clean,
    # Diagnostic 保留调试能力；Calibration 只保留停机相电压采集且禁止 arm；
    # Production 裁掉两类能力。三档分别定义唯一的主档位宏。
    # Diagnostic keeps diagnostics; Calibration keeps stopped-state phase capture
    # and refuses arming; Production removes both. Each profile defines exactly one
    # primary build-profile macro.
    [ValidateSet('Diagnostic', 'Calibration', 'Production')]
    [string]$Profile = 'Diagnostic',
    # Rust release 优化等级，直接映射到 CARGO_PROFILE_RELEASE_OPT_LEVEL。
    # Rust release opt-level, passed straight through as CARGO_PROFILE_RELEASE_OPT_LEVEL.
    [ValidateSet('3', 's', 'z')]
    [string]$RustOptLevel = 's'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = $PSScriptRoot
# 工作区布局为 <workspace>\projects\FluxRT 与 <workspace>\rt-thread 并列。
# The workspace keeps <workspace>\projects\FluxRT next to <workspace>\rt-thread.
$workspaceDir = Split-Path -Parent (Split-Path -Parent $projectDir)
$rttRoot = Join-Path $workspaceDir 'rt-thread'
$profileName = $Profile.ToLowerInvariant()
# 三个构建档必须用**不同的输出目录**：否则 CMake cache 会跨档复用，把上一次档位的
# 目标文件和新档位的 Rust 静态库混在一个镜像里。
# The three profiles must use different output directories; sharing one would let CMake
# reuse a stale cache and mix the previous profile's objects with the new Rust archive
# in a single image.
$buildDirectoryName = switch ($Profile)
{
    'Diagnostic'  { 'cmake-build' }
    'Calibration' { 'cmake-build-calibration' }
    'Production'  { 'cmake-build-production' }
}
$buildDir = Join-Path $projectDir $buildDirectoryName
# 本机工具链环境根目录；路径不存在时 Find-Tool 会回落到 PATH。
# Local toolchain environment root; Find-Tool falls back to PATH when it is absent.
$envRoot = 'D:\Environment\env-windows-latest'
$envScripts = Join-Path $envRoot '.venv\Scripts'

# 先在给定候选路径里找工具，再退回 PATH，都找不到就抛错。
# Looks for a tool in the given candidate paths, then on PATH, and throws if neither
# yields one: a missing arm-none-eabi-gcc must stop the build rather than produce a
# host-compiled ELF.
function Find-Tool
{
    param([string]$Name, [string[]]$Candidates)

    foreach ($candidate in $Candidates)
    {
        if (Test-Path -LiteralPath $candidate) { return $candidate }
    }
    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -ne $command) { return $command.Source }
    throw "Required tool not found: $Name"
}

$cmake = Find-Tool 'cmake' @((Join-Path $envScripts 'cmake.exe'))
$ninja = Find-Tool 'ninja' @((Join-Path $envScripts 'ninja.exe'))
$scons = Find-Tool 'scons' @((Join-Path $envScripts 'scons.exe'))
$pkgs = Find-Tool 'pkgs' @((Join-Path $envScripts 'pkgs.exe'))
$cargo = Find-Tool 'cargo' @((Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'))
$rustc = Find-Tool 'rustc' @((Join-Path $env:USERPROFILE '.cargo\bin\rustc.exe'))
$gcc = Find-Tool 'arm-none-eabi-gcc' @(
    (Join-Path $envRoot 'tools\gnu_gcc\arm-gnu-toolchain-15.3.rel1\bin\arm-none-eabi-gcc.exe'),
    'D:\software\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe'
)

$toolchainDir = Split-Path -Parent $gcc
$ninjaDir = Split-Path -Parent $ninja
$cargoDir = Split-Path -Parent $cargo
# 环境变量备份/恢复：本脚本会改写 RTT_*、PATH、PKGS_*，而这些变量影响同一
# PowerShell 会话里的其它命令（例如 VSCode 任务），因此必须在 finally 里还原。
# Environment backup and restore: the script rewrites RTT_*, PATH and PKGS_*, which
# affect other commands in the same PowerShell session (VSCode tasks, for example), so
# they must be restored in the finally block.
$previousProcessEnvironment = @{
    RTT_ROOT = [Environment]::GetEnvironmentVariable('RTT_ROOT', 'Process')
    RTT_CC = [Environment]::GetEnvironmentVariable('RTT_CC', 'Process')
    RTT_EXEC_PATH = [Environment]::GetEnvironmentVariable('RTT_EXEC_PATH', 'Process')
    PATH = [Environment]::GetEnvironmentVariable('PATH', 'Process')
    ENV_ROOT = [Environment]::GetEnvironmentVariable('ENV_ROOT', 'Process')
    PKGS_ROOT = [Environment]::GetEnvironmentVariable('PKGS_ROOT', 'Process')
    PKGS_DIR = [Environment]::GetEnvironmentVariable('PKGS_DIR', 'Process')
}

function Restore-ProcessEnvironment
{
    foreach ($entry in $previousProcessEnvironment.GetEnumerator())
    {
        [Environment]::SetEnvironmentVariable($entry.Key, $entry.Value, 'Process')
    }
}

# RTT_EXEC_PATH 指向 arm-none-eabi 工具链 bin，SCons 与 rtconfig.py 都用它找交叉编译器。
# RTT_EXEC_PATH points at the arm-none-eabi toolchain bin directory; both SCons and
# rtconfig.py use it to locate the cross compiler.
$env:RTT_ROOT = $rttRoot
$env:RTT_CC = 'gcc'
$env:RTT_EXEC_PATH = $toolchainDir
$env:PATH = "$cargoDir;$toolchainDir;$ninjaDir;$env:PATH"
$env:ENV_ROOT = $envRoot
$env:PKGS_ROOT = Join-Path $envRoot 'packages'
$env:PKGS_DIR = $env:PKGS_ROOT

# 删除前重复构造并比较目标路径，避免变量写错时 Remove-Item 删掉工程目录本身。
# The target path is rebuilt and compared before deletion, so a mistake in the
# variable can never make Remove-Item delete the project directory itself.
if ($Clean)
{
    $expected = Join-Path $projectDir $buildDirectoryName
    if ((Test-Path -LiteralPath $buildDir) -and ($buildDir -eq $expected))
    {
        Remove-Item -LiteralPath $buildDir -Recurse -Force
    }
    $rustBuildDirectoryName = "rust-target-$profileName-$RustOptLevel"
    $rustBuildDir = Join-Path $projectDir "build\$rustBuildDirectoryName"
    $expectedRustBuildDir = Join-Path $projectDir "build\$rustBuildDirectoryName"
    if ((Test-Path -LiteralPath $rustBuildDir) -and ($rustBuildDir -eq $expectedRustBuildDir))
    {
        Remove-Item -LiteralPath $rustBuildDir -Recurse -Force
    }
    Write-Host "[Clean] $buildDirectoryName and build\$rustBuildDirectoryName removed." -ForegroundColor Green
    Restore-ProcessEnvironment
    exit 0
}

Push-Location $projectDir
try
{
    Write-Host '[Tools]' -ForegroundColor Cyan
    Write-Host "Profile=$profileName RustOptLevel=$RustOptLevel BuildDir=$buildDirectoryName"
    & $cmake --version | Select-Object -First 1
    & $ninja --version
    & $scons --version | Select-Object -First 1
    & $gcc --version | Select-Object -First 1
    & $cargo --version
    & $rustc --version

    # 缺包时自动补拉：SCons 的 bsp_pkg_check 与 cmake 生成都会因为缺 STM32G4 包而失败，
    # 提前在这里处理可以把错误信息留在构建日志里，而不是让 CMake 报一堆找不到头文件。
    # Missing packages are fetched automatically: both the SCons bsp_pkg_check and the
    # cmake generation fail without the STM32G4 packages, and handling it here keeps one
    # clear error in the log instead of a pile of missing-header errors from CMake.
    $requiredPackages = @(
        '.\packages\CMSIS-Core-latest',
        '.\packages\stm32g4_cmsis_driver-latest',
        '.\packages\stm32g4_hal_driver-latest'
    )
    $packagesMissing = @($requiredPackages | Where-Object { -not (Test-Path -LiteralPath $_) }).Count -gt 0
    if ($UpdatePackages -or $packagesMissing)
    {
        # 先让 SCons 按 Kconfig 生成一次 rtconfig.h，pkgs 依赖它决定要拉哪些包。
        # Let SCons generate rtconfig.h from Kconfig first; pkgs uses it to decide which
        # packages to fetch.
        Write-Host '[Packages] Sync Kconfig defaults ...' -ForegroundColor Cyan
        & $scons --pyconfig-silent
        if ($LASTEXITCODE -ne 0) { throw "Initial rtconfig generation failed: $LASTEXITCODE" }

        Write-Host '[Packages] Fetch STM32G4 dependencies ...' -ForegroundColor Cyan
        & $pkgs --update
        if ($LASTEXITCODE -ne 0) { throw "Package update failed: $LASTEXITCODE" }
    }

    # CMakeLists.txt 由 SCons 从 Kconfig/SConscript 生成，因此不能手工维护；缺失或显式
    # 要求重新生成时都要先跑 SCons，再交给 CMake。Rust 集成在 custom.cmake 里，生成器
    # 会包含但不会覆盖它（见 custom.cmake 的文件头）。
    # CMakeLists.txt is generated by SCons from Kconfig/SConscript and must never be
    # hand-edited, so SCons runs whenever it is missing or regeneration was requested.
    # The Rust integration lives in custom.cmake, which the generator includes but does
    # not overwrite (see the custom.cmake header).
    if ($Regenerate -or -not (Test-Path -LiteralPath '.\CMakeLists.txt'))
    {
        Write-Host '[1/3] Generate rtconfig.h ...' -ForegroundColor Cyan
        & $scons --pyconfig-silent
        if ($LASTEXITCODE -ne 0) { throw "rtconfig generation failed: $LASTEXITCODE" }

        Write-Host '[2/3] Generate CMakeLists.txt ...' -ForegroundColor Cyan
        & $scons --target=cmake -s
        if ($LASTEXITCODE -ne 0) { throw "CMake generation failed: $LASTEXITCODE" }
    }

    # -BuildOnly 的第二道防线：把 cache 里实际生效的档位与本次请求逐字比较（:STRING=），
    # 不一致就拒绝构建。这是"不得把一种配置的固件当成另一种"的机器检查。
    # The second line of defence for -BuildOnly: compare the profile that actually
    # landed in the cache with this request (the :STRING= form) and refuse to build on
    # a mismatch. This is the mechanical check behind "never label one configuration as
    # another".
    $cache = Join-Path $buildDir 'CMakeCache.txt'
    if ($BuildOnly -and (Test-Path -LiteralPath $cache))
    {
        $cacheContent = Get-Content -LiteralPath $cache -Raw
        if (($cacheContent -notmatch "(?m)^FLUXRT_BUILD_PROFILE:STRING=$profileName\r?$") -or
            ($cacheContent -notmatch "(?m)^FLUXRT_RUST_OPT_LEVEL:STRING=$RustOptLevel\r?$"))
        {
            throw 'BuildOnly profile does not match CMake cache. Run build.ps1 -Regenerate with the requested profile and RustOptLevel.'
        }
    }
    if (-not $BuildOnly -or -not (Test-Path -LiteralPath $cache))
    {
        # 档位与优化等级以 CACHE STRING 传入，custom.cmake 会再校验一次取值合法性。
        # --fresh 只在 -Regenerate 时使用：它会丢弃 cache，也就会丢掉上一次的档位，
        # 这正是"重新生成"的语义。
        # The profile and opt-level arrive as CACHE STRINGs and custom.cmake validates
        # their values again. --fresh is used only with -Regenerate: it drops the cache
        # and with it the previously selected profile, which is exactly what
        # "regenerate" means here.
        Write-Host '[Configure] CMake + Ninja ...' -ForegroundColor Cyan
        $args = @(
            '-S', $projectDir,
            '-B', $buildDir,
            '-G', 'Ninja',
            "-DCMAKE_MAKE_PROGRAM=$ninja",
            "-DFLUXRT_BUILD_PROFILE=$profileName",
            "-DFLUXRT_RUST_OPT_LEVEL=$RustOptLevel"
        )
        if ($Regenerate) { $args = @('--fresh') + $args }
        & $cmake @args
        if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
    }

    if ($ConfigureOnly) { exit 0 }

    Write-Host '[3/3] Build firmware ...' -ForegroundColor Cyan
    & $cmake --build $buildDir --parallel 8
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $LASTEXITCODE" }

    Write-Host '[Done]' -ForegroundColor Green
    # 列出产物体积：Flash 余量本来就只剩几 KB，改动优化等级或档位后必须在这里（以及
    # 板端 WCET 与 BIN SHA-256）留下记录，才算完成一次档位变更。
    # List artefact sizes: with only a few KB of Flash headroom, a change of opt-level or
    # profile is only complete once this number, the on-board WCET and the BIN SHA-256
    # have been recorded.
    Get-ChildItem -LiteralPath $buildDir -File |
        Where-Object { $_.Name -in @('rtthread.elf', 'fluxrt.bin', 'fluxrt.hex', 'fluxrt.map') } |
        Select-Object Name, Length, LastWriteTime |
        Format-Table -AutoSize
}
finally
{
    Pop-Location
    Restore-ProcessEnvironment
}
