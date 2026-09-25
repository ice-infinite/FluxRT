# FluxRT 仿真—实机叠加绘图的 PowerShell 包装器。
# FluxRT PowerShell wrapper for the simulation-vs-hardware overlay plot.
#
# 职责 / Responsibility: 定位 MATLAB，拼出 MATLAB 的 name=value 调用串，
# 以批处理模式调用 simulation/matlab/compare_foc_traces.m，并核对图片确实生成。
# Locates MATLAB, builds the name=value call, runs compare_foc_traces.m in batch mode, and
# verifies that the expected image was actually produced.
#
# 安全边界 / Safety boundary: 本脚本只做离线绘图，不打开串口、不驱动电机，
# 因此没有 foc_stop 路径；功率级安全由 capture_hardware_trace.py 负责。
# Offline plotting only: no serial port and no motor, hence no foc_stop path. Power-stage
# safety lives in capture_hardware_trace.py.
#
# 输入 / Inputs: -SimulationCsv / -HardwareCsv 为相对于工程根的路径（可省略，
# 省略时由 MATLAB 使用 bringup_sim_582rpm_12v3_default_final.csv 等默认值）。
# Paths are relative to the project root; when omitted, MATLAB falls back to its own
# defaults.
# 输出 / Outputs: simulation/results/<OutputStem>.png/.fig/.mat/_summary.csv，
# 该目录已被 /simulation/results/ 规则忽略，属运行产物。
# Artifacts under the git-ignored simulation/results directory.
#
# 参考 / Reference: docs/仿真实机相关性验证.md §4.3
param(
    [string]$SimulationCsv = '',
    [string]$HardwareCsv = '',
    [string]$OutputStem = 'foc_sim_vs_hardware',
    [switch]$OpenResult
)

# StrictMode + ErrorActionPreference=Stop：任何未定义变量或未处理错误立即终止，
# 避免"图上少了一条曲线"却以成功退出码收场。
# StrictMode plus ErrorActionPreference=Stop fails fast, so a half-finished run cannot exit
# with a success code.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# $PSScriptRoot 即 simulation/，其父目录是工程根：MATLAB 与 CSV 的相对路径都以此为准。
# $PSScriptRoot is simulation/, whose parent is the project root; all relative paths resolve
# against it rather than the caller's current directory.
$projectDir = Split-Path -Parent $PSScriptRoot
$matlabScriptDir = Join-Path $PSScriptRoot 'matlab'
$resultImage = Join-Path $PSScriptRoot "results\$OutputStem.png"
# 先查两个已知安装位置，再退回 PATH 上的 matlab；都没有就报错而不是静默跳过绘图。
# Probes two known install locations before falling back to matlab on PATH, and fails
# loudly instead of silently skipping the plot.
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

# MATLAB 字符串用单引号定界，因此路径里的单引号必须按 '' 规则翻倍，
# 否则含引号的目录会把后面的参数一起吞进字符串。
# MATLAB strings are single-quoted, so embedded quotes must be doubled; otherwise a path
# containing a quote would swallow the rest of the argument list.
$escapedScriptDir = $matlabScriptDir.Replace("'", "''")
$escapedStem = $OutputStem.Replace("'", "''")
# Visible=false 让 MATLAB 在无桌面会话下也能出图，与本脚本的批处理用途一致。
# Visible=false keeps the run headless, which is what a batch wrapper needs.
$arguments = "Visible=false, OutputStem='$escapedStem'"
# 只有显式给出时才传路径，且按工程根展开为绝对路径：
# 否则相对路径会相对 MATLAB 的当前目录解析，容易读错文件。
# Paths are passed only when given and expanded against the project root, because a
# relative path would otherwise resolve against MATLAB's own working directory.
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
# Push/Pop-Location：MATLAB 批处理继承当前目录，绘图结束后必须恢复调用者的目录，
# 否则同一会话里后续脚本会意外在工程根下操作。
# Push/Pop-Location: MATLAB batch inherits the current directory, so it must be restored or
# later steps in the same session would run from the project root by surprise.
Push-Location $projectDir
try
{
    # -batch 无界面运行；非零退出码一律当成失败，避免把错误当成"没有输出"。
    # -batch runs headless; any non-zero exit code is a failure rather than "no output".
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB correlation failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}

# 以文件存在性作为最终判据：MATLAB 可能成功退出但没写出图片（例如 OutputStem 不符）。
# File existence is the final criterion: MATLAB can exit zero without writing the image
# (for example when OutputStem does not match).
if (-not (Test-Path -LiteralPath $resultImage))
{
    throw "MATLAB did not create the expected plot: $resultImage"
}
Write-Host "[Done] $resultImage" -ForegroundColor Green
# -OpenResult 仅用于人工查看，默认关闭以免 CI/批处理弹出窗口。
# -OpenResult is for manual inspection only and is off by default to keep batch runs quiet.
if ($OpenResult)
{
    Start-Process -FilePath $resultImage
}
