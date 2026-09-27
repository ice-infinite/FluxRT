# FluxRT A24.3 independent MATLAB phase-voltage fault-matrix runner.
param()

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = Split-Path -Parent $PSScriptRoot
$matlabScriptDir = Join-Path $PSScriptRoot 'matlab'
$referencePath = Join-Path $projectDir 'profiles\identification\evidence\phase-voltage-20260927\a24_fault_scenario_matrix.json'
$outputPath = Join-Path $projectDir 'profiles\identification\evidence\phase-voltage-20260927\a24_matlab_fault_scenario_matrix.json'
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

$escapedScriptDir = $matlabScriptDir.Replace("'", "''")
$escapedReference = $referencePath.Replace("'", "''")
$escapedOutput = $outputPath.Replace("'", "''")
$batch = "addpath('$escapedScriptDir'); run_phase_voltage_fault_scenarios(ReferencePath='$escapedReference', OutputPath='$escapedOutput');"

Write-Host '[MATLAB] Independent phase-voltage fault matrix ...' -ForegroundColor Cyan
Push-Location $projectDir
try
{
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB phase-voltage matrix failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}
if (-not (Test-Path -LiteralPath $outputPath))
{
    throw "MATLAB did not create the expected JSON: $outputPath"
}
Write-Host "[Done] $outputPath" -ForegroundColor Green
