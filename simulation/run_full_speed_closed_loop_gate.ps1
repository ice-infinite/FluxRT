[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\full-speed'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$revision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'cannot resolve Git revision' }
$dirtyEntries = @(& git -C $projectRoot status --porcelain)
if ($dirtyEntries.Count -gt 0) { $revision += '+dirty' }
$previousRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $revision

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-full-speed-contract-check -- --root $projectRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust full-speed gate failed: $LASTEXITCODE" }
    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'", "''")
    $escapedRoot = $projectRoot.Replace("'", "''")
    $escapedOutput = $resultRoot.Replace("'", "''")
    & $MatlabExe -batch "addpath('$simulinkDir'); run_full_speed_closed_loop_contract('$escapedRoot','$escapedOutput');"
    if ($LASTEXITCODE -ne 0) { throw "MATLAB full-speed gate failed: $LASTEXITCODE" }
    & $PythonExe (Join-Path $PSScriptRoot 'compare_full_speed_closed_loop_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-full-speed-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-full-speed-trace.csv') `
        --rust-metrics (Join-Path $resultRoot 'rust-full-speed-metrics.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-full-speed-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-full-speed-trace.csv') `
        --matlab-metrics (Join-Path $resultRoot 'matlab-full-speed-metrics.csv')
    if ($LASTEXITCODE -ne 0) { throw "full-speed comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB full-speed closed-loop design gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
