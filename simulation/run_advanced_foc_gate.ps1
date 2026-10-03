[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\advanced'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null

$revision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'cannot resolve Git revision' }
$dirty = @(& git -C $projectRoot status --porcelain)
if ($dirty.Count -gt 0) { $revision += '+dirty' }
$previousRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $revision

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-advanced-contract-check -- --root $projectRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust advanced FOC gate failed: $LASTEXITCODE" }

    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'", "''")
    $escapedRoot = $projectRoot.Replace("'", "''")
    $escapedOutput = $resultRoot.Replace("'", "''")
    & $MatlabExe -batch "addpath('$simulinkDir'); run_advanced_foc_contract('$escapedRoot','$escapedOutput');"
    if ($LASTEXITCODE -ne 0) { throw "MATLAB advanced FOC gate failed: $LASTEXITCODE" }

    & $PythonExe (Join-Path $PSScriptRoot 'compare_advanced_foc_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-advanced-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-advanced-trace.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-advanced-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-advanced-trace.csv')
    if ($LASTEXITCODE -ne 0) { throw "advanced FOC comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB advanced FOC policy gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
