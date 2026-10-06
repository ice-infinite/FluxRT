[CmdletBinding()]
param(
    [string]$MatlabExe = 'D:\software\MATLAB\R2025b\bin\matlab.exe',
    [string]$CargoExe = "$env:USERPROFILE\.cargo\bin\cargo.exe",
    [string]$PythonExe = 'python'
)

$ErrorActionPreference='Stop'
$projectRoot=[System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$resultRoot=Join-Path $PSScriptRoot 'results\hfi-dynamic'
New-Item -ItemType Directory -Path $resultRoot -Force | Out-Null
$revision=(& git -C $projectRoot rev-parse HEAD).Trim()
if($LASTEXITCODE -ne 0){throw 'cannot resolve Git revision'}
$dirty=@(& git -C $projectRoot status --porcelain)
if($dirty.Count -gt 0){$revision+='+dirty'}
$previous=$env:FLUXRT_WORKSPACE_REVISION
$env:FLUXRT_WORKSPACE_REVISION=$revision
try {
    & $CargoExe run --quiet --manifest-path (Join-Path $projectRoot 'rust\Cargo.toml') `
        -p foc-sim --bin foc-hfi-dynamic-contract-check -- --root $projectRoot
    if($LASTEXITCODE -ne 0){throw "Rust dynamic HFI gate failed: $LASTEXITCODE"}
    $simulinkDir=(Join-Path $projectRoot 'simulink').Replace("'","''")
    $escapedRoot=$projectRoot.Replace("'","''")
    $escapedOutput=$resultRoot.Replace("'","''")
    & $MatlabExe -batch "addpath('$simulinkDir'); run_hfi_dynamic_plant_contract('$escapedRoot','$escapedOutput');"
    if($LASTEXITCODE -ne 0){throw "MATLAB dynamic HFI gate failed: $LASTEXITCODE"}
    & $PythonExe (Join-Path $PSScriptRoot 'compare_hfi_dynamic_results.py') `
        --rust-metadata (Join-Path $resultRoot 'rust-hfi-dynamic-d0.txt') `
        --rust-metrics (Join-Path $resultRoot 'rust-hfi-dynamic-metrics.csv') `
        --matlab-metadata (Join-Path $resultRoot 'matlab-hfi-dynamic-d0.txt') `
        --matlab-metrics (Join-Path $resultRoot 'matlab-hfi-dynamic-metrics.csv')
    if($LASTEXITCODE -ne 0){throw "dynamic HFI comparison failed: $LASTEXITCODE"}
}
finally {$env:FLUXRT_WORKSPACE_REVISION=$previous}
Write-Host '[PASS] Rust/MATLAB dynamic active-HFI gate' -ForegroundColor Green
Write-Host "  Results: $resultRoot"
