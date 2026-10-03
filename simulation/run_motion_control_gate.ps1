[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\motion'
$scenarioPath = Join-Path $PSScriptRoot 'scenarios\motion_control_matrix_v1.json'
$comparisonPath = Join-Path $PSScriptRoot 'contracts\comparison-gates-v1.json'
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
if (-not (Test-Path -LiteralPath $scenarioPath -PathType Leaf)) { throw "scenario not found: $scenarioPath" }
if (-not (Test-Path -LiteralPath $comparisonPath -PathType Leaf)) { throw "comparison contract not found: $comparisonPath" }

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-motion-contract-check -- --root $projectRoot
    if ($LASTEXITCODE -ne 0) { throw "Rust motion gate failed: $LASTEXITCODE" }

    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'", "''")
    $matlabProject = $projectRoot.Replace("'", "''")
    $matlabOutput = $resultRoot.Replace("'", "''")
    $expression = "addpath('$simulinkDir'); run_motion_control_contract('$matlabProject','$matlabOutput');"
    & $MatlabExe -batch $expression
    if ($LASTEXITCODE -ne 0) { throw "MATLAB motion gate failed: $LASTEXITCODE" }

    & $PythonExe (Join-Path $PSScriptRoot 'compare_motion_control_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-motion-d0.txt') `
        --rust-trace (Join-Path $resultRoot 'rust-motion-trace.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-motion-d0.txt') `
        --matlab-trace (Join-Path $resultRoot 'matlab-motion-trace.csv') `
        --scenario $scenarioPath `
        --comparison $comparisonPath
    if ($LASTEXITCODE -ne 0) { throw "motion result comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousRevision
}

Write-Host '[PASS] Rust/MATLAB motion D0-D4 gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
