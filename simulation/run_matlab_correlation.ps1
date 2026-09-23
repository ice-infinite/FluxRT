param(
    [string]$SimulationCsv = '',
    [string]$HardwareCsv = '',
    [string]$OutputStem = 'foc_sim_vs_hardware',
    [switch]$OpenResult
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = Split-Path -Parent $PSScriptRoot
$matlabScriptDir = Join-Path $PSScriptRoot 'matlab'
$resultImage = Join-Path $PSScriptRoot "results\$OutputStem.png"
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
$escapedStem = $OutputStem.Replace("'", "''")
$arguments = "Visible=false, OutputStem='$escapedStem'"
if ($SimulationCsv -ne '')
{
    $simulationPath = [System.IO.Path]::GetFullPath((Join-Path $projectDir $SimulationCsv))
    $arguments += ", SimulationCsv='$($simulationPath.Replace("'", "''"))'"
}
if ($HardwareCsv -ne '')
{
    $hardwarePath = [System.IO.Path]::GetFullPath((Join-Path $projectDir $HardwareCsv))
    $arguments += ", HardwareCsv='$($hardwarePath.Replace("'", "''"))'"
}
$batch = "addpath('$escapedScriptDir'); compare_foc_traces($arguments);"

Write-Host '[MATLAB] Plot same-scenario simulation and hardware traces ...' -ForegroundColor Cyan
Push-Location $projectDir
try
{
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB correlation failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}

if (-not (Test-Path -LiteralPath $resultImage))
{
    throw "MATLAB did not create the expected plot: $resultImage"
}
Write-Host "[Done] $resultImage" -ForegroundColor Green
if ($OpenResult)
{
    Start-Process -FilePath $resultImage
}
