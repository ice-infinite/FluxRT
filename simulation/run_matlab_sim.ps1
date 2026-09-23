param(
    [double]$DurationS = 3.0,
    [double]$TargetRpm = 524.0,
    [double]$LoadStepTimeS = 1.0,
    [double]$LoadTorqueNm = 0.004,
    [int]$SampleEvery = 10,
    [switch]$OpenResult
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectDir = Split-Path -Parent $PSScriptRoot
$matlabScriptDir = Join-Path $PSScriptRoot 'matlab'
$resultImage = Join-Path $PSScriptRoot 'results\foc_sim_overview.png'
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

$culture = [System.Globalization.CultureInfo]::InvariantCulture
$durationText = $DurationS.ToString('G17', $culture)
$targetText = $TargetRpm.ToString('G17', $culture)
$loadStepText = $LoadStepTimeS.ToString('G17', $culture)
$loadText = $LoadTorqueNm.ToString('G17', $culture)
$escapedScriptDir = $matlabScriptDir.Replace("'", "''")
$batch = "addpath('$escapedScriptDir'); run_foc_matlab(DurationS=$durationText, TargetRpm=$targetText, LoadStepTimeS=$loadStepText, LoadTorqueNm=$loadText, SampleEvery=$SampleEvery, Visible=false);"

Write-Host '[MATLAB] Rust closed-loop simulation and plotting ...' -ForegroundColor Cyan
Push-Location $projectDir
try
{
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB simulation failed: $LASTEXITCODE" }
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
