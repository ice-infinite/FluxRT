Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = $PSScriptRoot
$testSource = Join-Path $projectDir 'tests\host'
$testBuild = Join-Path $projectDir 'build\host-tests'
$envRoot = 'D:\Environment\env-windows-latest'
$cmake = Join-Path $envRoot '.venv\Scripts\cmake.exe'
$ninja = Join-Path $envRoot '.venv\Scripts\ninja.exe'
$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'

if (-not (Test-Path -LiteralPath $cmake)) { $cmake = (Get-Command cmake).Source }
if (-not (Test-Path -LiteralPath $ninja)) { $ninja = (Get-Command ninja).Source }
if (-not (Test-Path -LiteralPath $cargo)) { $cargo = (Get-Command cargo).Source }

$rustManifest = Join-Path $projectDir 'rust\Cargo.toml'
$env:CARGO_TARGET_DIR = Join-Path $projectDir 'build\rust-test-target'

Write-Host '[Rust] Formatting check ...' -ForegroundColor Cyan
& $cargo fmt --manifest-path $rustManifest --all -- --check
if ($LASTEXITCODE -ne 0) { throw 'Rust formatting check failed.' }

Write-Host '[Rust] Unit tests (algorithm library + C ABI bridge) ...' -ForegroundColor Cyan
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
& $cmake --fresh -S $testSource -B $testBuild -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninja"
if ($LASTEXITCODE -ne 0) { throw 'Host test configure failed.' }

& $cmake --build $testBuild
if ($LASTEXITCODE -ne 0) { throw 'Host test build failed.' }

& $cmake --build $testBuild --target test
if ($LASTEXITCODE -ne 0) { throw 'Host tests failed.' }
