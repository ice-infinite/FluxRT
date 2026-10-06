[CmdletBinding()]
param(
    [switch]$Clean,
    [ValidateSet('dengfoc_usb_angle_diag', 'dengfoc_usb_pwm_prepare', 'dengfoc_usb_mcpwm_timing', 'dengfoc_usb_rust_probe')]
    [string]$Environment = 'dengfoc_usb_angle_diag'
)

$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
$platformIo = Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe'

if (-not (Test-Path -LiteralPath $platformIo)) {
    throw "PlatformIO CLI not found: $platformIo"
}
$arguments = @(
    'run',
    '--project-dir', $projectDirectory,
    '--environment', $Environment
)
if ($Clean) {
    $arguments += '--target'
    $arguments += 'clean'
}

& $platformIo @arguments
if ($LASTEXITCODE -ne 0) {
    throw "PlatformIO build failed with exit code $LASTEXITCODE"
}
