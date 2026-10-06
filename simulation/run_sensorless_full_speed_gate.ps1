[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\sensorless-full-speed'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$revision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'cannot resolve Git revision' }
if (@(& git -C $projectRoot status --porcelain).Count -gt 0) { $revision += '+dirty' }
$previousRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $revision

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-sensorless-full-speed-contract-check -- --root $projectRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust sensorless full-speed gate failed: $LASTEXITCODE" }
    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'", "''")
    $escapedRoot = $projectRoot.Replace("'", "''")
    $escapedOutput = $resultRoot.Replace("'", "''")
    & $MatlabExe -batch "addpath('$simulinkDir'); run_sensorless_full_speed_chain_contract('$escapedRoot','$escapedOutput');"
    if ($LASTEXITCODE -ne 0) { throw "MATLAB sensorless full-speed gate failed: $LASTEXITCODE" }
    & $PythonExe (Join-Path $PSScriptRoot 'compare_sensorless_full_speed_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-sensorless-full-speed-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-sensorless-full-speed-trace.csv') `
        --rust-metrics (Join-Path $resultRoot 'rust-sensorless-full-speed-metrics.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-sensorless-full-speed-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-sensorless-full-speed-trace.csv') `
        --matlab-metrics (Join-Path $resultRoot 'matlab-sensorless-full-speed-metrics.csv')
    if ($LASTEXITCODE -ne 0) { throw "sensorless full-speed comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB sensorless full-speed angle-chain gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
