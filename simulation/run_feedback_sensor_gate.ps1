[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\feedback'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$workspaceRevision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($workspaceRevision)) {
    throw 'cannot resolve Git workspace revision'
}
$dirtyLines = @(& git -C $projectRoot status --porcelain)
if ($LASTEXITCODE -ne 0) { throw 'cannot inspect Git workspace state' }
if ($dirtyLines.Count -gt 0) { $workspaceRevision += '+dirty' }
$previousRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $workspaceRevision

if (-not (Test-Path -LiteralPath $CargoExe -PathType Leaf)) { throw "cargo not found: $CargoExe" }
if (-not (Test-Path -LiteralPath $MatlabExe -PathType Leaf)) { throw "MATLAB not found: $MatlabExe" }

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-feedback-contract-check -- --root $projectRoot --out $resultRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust feedback gate failed: $LASTEXITCODE" }

    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'","''")
    $matlabProject = $projectRoot.Replace("'","''")
    $matlabOutput = $resultRoot.Replace("'","''")
    $expression = "addpath('$simulinkDir'); run_feedback_sensor_contract('$matlabProject','$matlabOutput');"
    & $MatlabExe -batch $expression
    if ($LASTEXITCODE -ne 0) { throw "MATLAB feedback gate failed: $LASTEXITCODE" }

    & $PythonExe (Join-Path $PSScriptRoot 'compare_feedback_sensor_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-feedback-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-feedback-trace.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-feedback-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-feedback-trace.csv') `
        --comparison (Join-Path $PSScriptRoot 'contracts\comparison-gates-v1.json')
    if ($LASTEXITCODE -ne 0) { throw "feedback result comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB feedback D0/D2/D3/D4 gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
