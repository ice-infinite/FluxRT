# FluxRT CM2 Rust/MATLAB inverter-voltage numerical gate. PC only; no serial or motor access.
param(
    [string]$OutputDir = ''
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = Split-Path -Parent $PSScriptRoot
if ($OutputDir -eq '')
{
    $OutputDir = Join-Path $PSScriptRoot 'results\inverter_model_gate'
}
elseif (-not [System.IO.Path]::IsPathRooted($OutputDir))
{
    $OutputDir = Join-Path $projectDir $OutputDir
}
$OutputDir = [System.IO.Path]::GetFullPath($OutputDir)
New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null

$cargo = Join-Path $env:USERPROFILE '.cargo\bin\cargo.exe'
if (-not (Test-Path -LiteralPath $cargo)) { throw "Cargo was not found: $cargo" }
$matlabCandidates = @(
    'D:\software\MATLAB\R2025b\bin\matlab.exe',
    'C:\Program Files\MATLAB\R2025b\bin\matlab.exe'
)
$matlab = $matlabCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if ($null -eq $matlab)
{
    $command = Get-Command matlab -ErrorAction SilentlyContinue
    if ($null -eq $command) { throw 'MATLAB executable was not found.' }
    $matlab = $command.Source
}

$csvPath = Join-Path $OutputDir 'rust_inverter_voltage_vectors.csv'
$jsonPath = Join-Path $OutputDir 'matlab_inverter_voltage_report.json'
Push-Location (Join-Path $projectDir 'rust')
try
{
    & $cargo run --quiet -p foc-control --example inverter_voltage_vectors -- $csvPath
    if ($LASTEXITCODE -ne 0) { throw "Rust vector generation failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}

$simulinkDir = (Join-Path $projectDir 'simulink').Replace("'", "''")
$escapedCsv = $csvPath.Replace("'", "''")
$escapedJson = $jsonPath.Replace("'", "''")
$batch = "addpath('$simulinkDir'); validate_inverter_voltage_model('$escapedCsv','$escapedJson');"
Push-Location $projectDir
try
{
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB inverter-model gate failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}

if (-not (Test-Path -LiteralPath $csvPath)) { throw "Missing Rust vectors: $csvPath" }
if (-not (Test-Path -LiteralPath $jsonPath)) { throw "Missing MATLAB report: $jsonPath" }
Write-Host "INVERTER_MODEL_GATE_PASS csv=$csvPath report=$jsonPath" -ForegroundColor Green
