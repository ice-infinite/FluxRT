[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\lsi-h2-replay'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$revision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'cannot resolve Git revision' }
$dirtyEntries = @(& git -C $projectRoot status --porcelain)
if ($dirtyEntries.Count -gt 0) { $revision += '+dirty' }
$previousRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $revision

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-lsi-h2-replay-contract-check -- --root $projectRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust H2 replay failed: $LASTEXITCODE" }

    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'", "''")
    $escapedRoot = $projectRoot.Replace("'", "''")
    $escapedOutput = $resultRoot.Replace("'", "''")
    & $MatlabExe -batch "addpath('$simulinkDir'); run_lsi_h2_replay_contract('$escapedRoot','$escapedOutput');"
    if ($LASTEXITCODE -ne 0) { throw "MATLAB H2 replay failed: $LASTEXITCODE" }

    & $PythonExe (Join-Path $PSScriptRoot 'compare_lsi_h2_replay_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-lsi-h2-replay-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-lsi-h2-replay.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-lsi-h2-replay-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-lsi-h2-replay.csv')
    if ($LASTEXITCODE -ne 0) { throw "H2 replay comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB H2 Ls(I) replay contract' -ForegroundColor Green
Write-Host '[BLOCKED] Historical fixture has no valid AS5600 angle index.' -ForegroundColor Yellow
Write-Host "  Results: $resultRoot"
