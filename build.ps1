param(
    [switch]$Regenerate,
    [switch]$UpdatePackages,
    [switch]$ConfigureOnly,
    [switch]$BuildOnly,
    [switch]$Clean
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = $PSScriptRoot
$workspaceDir = Split-Path -Parent (Split-Path -Parent $projectDir)
$rttRoot = Join-Path $workspaceDir 'rt-thread'
$buildDir = Join-Path $projectDir 'cmake-build'
$envRoot = 'D:\Environment\env-windows-latest'
$envScripts = Join-Path $envRoot '.venv\Scripts'

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
$env:RTT_ROOT = $rttRoot
$env:RTT_CC = 'gcc'
$env:RTT_EXEC_PATH = $toolchainDir
$env:PATH = "$cargoDir;$toolchainDir;$ninjaDir;$env:PATH"
$env:ENV_ROOT = $envRoot
$env:PKGS_ROOT = Join-Path $envRoot 'packages'
$env:PKGS_DIR = $env:PKGS_ROOT

if ($Clean)
{
    $expected = Join-Path $projectDir 'cmake-build'
    if ((Test-Path -LiteralPath $buildDir) -and ($buildDir -eq $expected))
    {
        Remove-Item -LiteralPath $buildDir -Recurse -Force
    }
    $rustBuildDir = Join-Path $projectDir 'build\rust-target'
    $expectedRustBuildDir = Join-Path $projectDir 'build\rust-target'
    if ((Test-Path -LiteralPath $rustBuildDir) -and ($rustBuildDir -eq $expectedRustBuildDir))
    {
        Remove-Item -LiteralPath $rustBuildDir -Recurse -Force
    }
    Write-Host '[Clean] cmake-build and build\rust-target removed.' -ForegroundColor Green
    exit 0
}

Push-Location $projectDir
try
{
    Write-Host '[Tools]' -ForegroundColor Cyan
    & $cmake --version | Select-Object -First 1
    & $ninja --version
    & $scons --version | Select-Object -First 1
    & $gcc --version | Select-Object -First 1
    & $cargo --version
    & $rustc --version

    $requiredPackages = @(
        '.\packages\CMSIS-Core-latest',
        '.\packages\stm32g4_cmsis_driver-latest',
        '.\packages\stm32g4_hal_driver-latest'
    )
    $packagesMissing = @($requiredPackages | Where-Object { -not (Test-Path -LiteralPath $_) }).Count -gt 0
    if ($UpdatePackages -or $packagesMissing)
    {
        Write-Host '[Packages] Sync Kconfig defaults ...' -ForegroundColor Cyan
        & $scons --pyconfig-silent
        if ($LASTEXITCODE -ne 0) { throw "Initial rtconfig generation failed: $LASTEXITCODE" }

        Write-Host '[Packages] Fetch STM32G4 dependencies ...' -ForegroundColor Cyan
        & $pkgs --update
        if ($LASTEXITCODE -ne 0) { throw "Package update failed: $LASTEXITCODE" }
    }

    if ($Regenerate -or -not (Test-Path -LiteralPath '.\CMakeLists.txt'))
    {
        Write-Host '[1/3] Generate rtconfig.h ...' -ForegroundColor Cyan
        & $scons --pyconfig-silent
        if ($LASTEXITCODE -ne 0) { throw "rtconfig generation failed: $LASTEXITCODE" }

        Write-Host '[2/3] Generate CMakeLists.txt ...' -ForegroundColor Cyan
        & $scons --target=cmake -s
        if ($LASTEXITCODE -ne 0) { throw "CMake generation failed: $LASTEXITCODE" }
    }

    $cache = Join-Path $buildDir 'CMakeCache.txt'
    if (-not $BuildOnly -or -not (Test-Path -LiteralPath $cache))
    {
        Write-Host '[Configure] CMake + Ninja ...' -ForegroundColor Cyan
        $args = @('-S', $projectDir, '-B', $buildDir, '-G', 'Ninja', "-DCMAKE_MAKE_PROGRAM=$ninja")
        if ($Regenerate) { $args = @('--fresh') + $args }
        & $cmake @args
        if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
    }

    if ($ConfigureOnly) { exit 0 }

    Write-Host '[3/3] Build firmware ...' -ForegroundColor Cyan
    & $cmake --build $buildDir --parallel 8
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $LASTEXITCODE" }

    Write-Host '[Done]' -ForegroundColor Green
    Get-ChildItem -LiteralPath $buildDir -File |
        Where-Object { $_.Name -in @('rtthread.elf', 'fluxrt.bin', 'fluxrt.hex', 'fluxrt.map') } |
        Select-Object Name, Length, LastWriteTime |
        Format-Table -AutoSize
}
finally
{
    Pop-Location
}
