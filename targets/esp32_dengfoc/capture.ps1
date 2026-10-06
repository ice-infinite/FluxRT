[CmdletBinding()]
param(
    [string]$Port = 'COM13',
    [ValidateRange(1, 3600)]
    [int]$DurationSeconds = 10,
    [string]$Name = 'angle-capture',
    [double]$RequireMotionRad = 0.0,
    [double]$RequireOneDirectionRad = 0.0,
    [double]$MaxReverseRad = [double]::PositiveInfinity,
    [ValidateRange(0, 1000)]
    [int]$RequireWraps = 0
)

$ErrorActionPreference = 'Stop'
$targetDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$projectDirectory = Split-Path -Parent (Split-Path -Parent $targetDirectory)
$python = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\python.exe'
$tool = Join-Path $projectDirectory 'tools\dengfoc_angle_capture.py'
$captureDirectory = Join-Path $targetDirectory 'captures'
$timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$output = Join-Path $captureDirectory "$timestamp-$Name.csv"

if (-not (Test-Path -LiteralPath $python)) {
    throw "PlatformIO Python not found: $python"
}

$arguments = @(
    $tool,
    '--port', $Port,
    '--duration', $DurationSeconds,
    '--output', $output,
    '--require-motion-rad', $RequireMotionRad,
    '--require-one-direction-rad', $RequireOneDirectionRad,
    '--max-reverse-rad', $MaxReverseRad,
    '--require-wraps', $RequireWraps
)
& $python @arguments
if ($LASTEXITCODE -ne 0) {
    throw "Angle capture did not satisfy its evidence gates. CSV: $output"
}
Write-Host "Capture: $output"
