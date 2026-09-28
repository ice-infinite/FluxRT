[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot = Join-Path $PSScriptRoot 'results\contract'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$rustResult = Join-Path $resultRoot 'rust-d0-d1.txt'
$matlabResult = Join-Path $resultRoot 'matlab-d0-d1.txt'

$workspaceRevision = (& git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($workspaceRevision)) {
    throw 'cannot resolve Git workspace revision'
}
$dirtyLines = @(& git -C $projectRoot status --porcelain)
if ($LASTEXITCODE -ne 0) { throw 'cannot inspect Git workspace state' }
if ($dirtyLines.Count -gt 0) { $workspaceRevision += '+dirty' }
$previousWorkspaceRevision = $env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION = $workspaceRevision

if (-not (Test-Path -LiteralPath $CargoExe -PathType Leaf)) {
    throw "cargo not found: $CargoExe"
}
if (-not (Test-Path -LiteralPath $MatlabExe -PathType Leaf)) {
    throw "MATLAB not found: $MatlabExe"
}

try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-contract-check -- --root $projectRoot --out $rustResult
    if ($LASTEXITCODE -ne 0) { throw "Rust contract gate failed: $LASTEXITCODE" }

    $simulinkDir = (Join-Path $projectRoot 'simulink').Replace("'","''")
    $matlabProject = $projectRoot.Replace("'","''")
    $matlabOutput = $matlabResult.Replace("'","''")
    $expression = "addpath('$simulinkDir'); validate_fluxrt_simulation_contract('$matlabProject','$matlabOutput');"
    & $MatlabExe -batch $expression
    if ($LASTEXITCODE -ne 0) { throw "MATLAB contract gate failed: $LASTEXITCODE" }

    & $PythonExe (Join-Path $PSScriptRoot 'compare_contract_gate_results.py') `
        --rust $rustResult --matlab $matlabResult
    if ($LASTEXITCODE -ne 0) { throw "dual result comparison failed: $LASTEXITCODE" }
}
finally {
    $env:FLUXRT_WORKSPACE_REVISION = $previousWorkspaceRevision
}

Write-Host "[PASS] Rust/MATLAB D0/D1 contract gate" -ForegroundColor Green
Write-Host "  Rust:  $rustResult"
Write-Host "  MATLAB: $matlabResult"
