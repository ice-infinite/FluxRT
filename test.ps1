# FluxRT —— 全套回归入口（Rust + 仿真 + 交叉编译 + C 主机测试）。
# FluxRT - full regression entry point (Rust, simulation, cross build, C host test).
#
# 职责 / Responsibility:
#   依次执行 cargo fmt --check、cargo test、闭环 PMSM 仿真、clippy -D warnings、
#   两个 thumbv7em-none-eabihf 交叉构建（CPU 后端与 CORDIC 特性），最后配置、构建
#   并运行 tests/host 的 C 安全测试。任一步非零退出即抛错停止。
#   Runs cargo fmt --check, cargo test, the closed-loop PMSM simulation,
#   clippy with -D warnings, two thumbv7em-none-eabihf cross builds (CPU backend and
#   the CORDIC feature), and finally configures, builds and runs the C safety test in
#   tests/host. The first non-zero exit throws and stops the run.
#
# 边界 / Boundary:
#   本脚本**不**构建目标固件（那是 build.ps1 的职责），也**不**碰硬件：所有验证都在
#   PC 上完成。C 侧只验证空状态拒绝路径与同拍时序不变量。
#   This script does not build the target firmware (that is build.ps1's job) and does not
#   touch hardware; every check runs on the PC. The C side only covers the empty-state
#   refusal paths and the same-tick timing invariant.
#
# 为什么 clippy 带 -D warnings / Why clippy runs with -D warnings:
#   Rust 侧跑在 12 kHz 的 ISR 里，而 12500 cycles 的软件截止没有余量容纳"以后再说"；
#   因此警告即失败。真实的有效性证据仍是板端 WCET，不是"clippy 通过"。
#   The Rust side runs inside a 12 kHz ISR and the 12500-cycle software deadline has no
#   room for "later", so a warning is a failure. The real validity evidence is still the
#   on-board WCET; a clean clippy run is not a substitute.
#
# 参考 / Reference: docs/注释规范.md, docs/构建档与优化等级.md
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = $PSScriptRoot
$testSource = Join-Path $projectDir 'tests\host'
$testBuild = Join-Path $projectDir 'build\host-tests'
# 与 build.ps1 相同：优先用固定环境根目录，缺失时回落到 PATH。
# Same as build.ps1: prefer the fixed environment root and fall back to PATH.
$envRoot = 'D:\Environment\env-windows-latest'
$cmake = Join-Path $envRoot '.venv\Scripts\cmake.exe'
$ninja = Join-Path $envRoot '.venv\Scripts\ninja.exe'
$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'
$python = Join-Path $envRoot '.venv\Scripts\python.exe'

if (-not (Test-Path -LiteralPath $cmake)) { $cmake = (Get-Command cmake).Source }
if (-not (Test-Path -LiteralPath $ninja)) { $ninja = (Get-Command ninja).Source }
if (-not (Test-Path -LiteralPath $cargo)) { $cargo = (Get-Command cargo).Source }
if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python).Source }

Write-Host '[Profile] Candidate/identification schemas, CRC, evidence gates and snapshots ...' -ForegroundColor Cyan
& $python -m unittest discover -s (Join-Path $projectDir 'tests\profile') -p 'test_*.py'
if ($LASTEXITCODE -ne 0) { throw 'Production-profile tool tests failed.' }

& $python (Join-Path $projectDir 'tools\foc_profile_tool.py') generate `
    (Join-Path $projectDir 'profiles\candidates\rev1-gbm2804h-unapproved.json') `
    --output (Join-Path $projectDir 'profiles\generated\rev1-gbm2804h-unapproved.inc') `
    --check
if ($LASTEXITCODE -ne 0) { throw 'Generated production-profile candidate is stale.' }

& $python (Join-Path $projectDir 'tools\foc_identification_tool.py') analyze `
    (Join-Path $projectDir 'profiles\identification\sessions\a16-screening-20260924.json') `
    --output (Join-Path $projectDir 'profiles\identification\reports\a16-screening-20260924.json') `
    --check
if ($LASTEXITCODE -ne 0) { throw 'Identification evidence or screening report is stale.' }

$rustManifest = Join-Path $projectDir 'rust\Cargo.toml'
# 主机测试用独立的 CARGO_TARGET_DIR，与固件产物（build\rust-target[-3|s|z]）隔离，
# 避免测试构建的 artifact 被后续固件链接误用。
# Host tests use a separate CARGO_TARGET_DIR, isolated from the firmware artefacts in
# build\rust-target[-3|s|z], so a test build can never be linked into the firmware.
$env:CARGO_TARGET_DIR = Join-Path $projectDir 'build\rust-test-target'

Write-Host '[Rust] Formatting check ...' -ForegroundColor Cyan
& $cargo fmt --manifest-path $rustManifest --all -- --check
if ($LASTEXITCODE -ne 0) { throw 'Rust formatting check failed.' }

Write-Host '[Rust] Unit tests (algorithm library + C ABI bridge) ...' -ForegroundColor Cyan
# --locked 是刻意的：依赖版本漂移会同时改变 Flash 体积与 WCET，而这两者都是被记录的
# 发布证据，不能在回归里悄悄变。
# --locked is deliberate: a dependency drift would change both the Flash size and the
# WCET, and both are recorded release evidence that must not change silently.
& $cargo test --manifest-path $rustManifest --workspace --all-features --locked
if ($LASTEXITCODE -ne 0) { throw 'Rust tests failed.' }

Write-Host '[Rust] Closed-loop PMSM simulation ...' -ForegroundColor Cyan
& $cargo run --manifest-path $rustManifest --package foc-sim --bin foc-sim --locked --quiet
if ($LASTEXITCODE -ne 0) { throw 'Closed-loop FOC simulation failed.' }

Write-Host '[Rust] Clippy ...' -ForegroundColor Cyan
& $cargo clippy --manifest-path $rustManifest --workspace --all-targets --all-features --locked -- -D warnings
if ($LASTEXITCODE -ne 0) { throw 'Rust clippy failed.' }

Write-Host '[Rust] Cortex-M4F CPU-only static library ...' -ForegroundColor Cyan
$env:CARGO_TARGET_DIR = Join-Path $projectDir 'build\rust-target'
& $cargo build --manifest-path $rustManifest --package foc-rt-bridge --release --target thumbv7em-none-eabihf --locked
if ($LASTEXITCODE -ne 0) { throw 'Rust cross build failed.' }

Write-Host '[Rust] Cortex-M4F STM32G4 CORDIC static library ...' -ForegroundColor Cyan
& $cargo build --manifest-path $rustManifest --package foc-rt-bridge --release --target thumbv7em-none-eabihf --features stm32g4-cordic --locked
if ($LASTEXITCODE -ne 0) { throw 'Rust CORDIC cross build failed.' }

Write-Host '[C] Platform safety test ...' -ForegroundColor Cyan
# 主机 C 测试用 tests/host/CMakeLists.txt 单独配置，不复用固件 build 目录：它的编译
# 选项、宏定义与链接目标都不同，混在一起会互相污染。
# The host C test is configured separately through tests/host/CMakeLists.txt and never
# reuses the firmware build directory: its compile options, defines and link targets
# differ, and sharing a directory would let the two contaminate each other.
& $cmake --fresh -S $testSource -B $testBuild -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninja"
if ($LASTEXITCODE -ne 0) { throw 'Host test configure failed.' }

& $cmake --build $testBuild
if ($LASTEXITCODE -ne 0) { throw 'Host test build failed.' }

# `--target test` 通过 CTest 运行测试可执行文件；CTest 返回非零即视为断言失败。
# `--target test` runs the test executable through CTest, whose non-zero exit means an
# assertion failed.
& $cmake --build $testBuild --target test
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
