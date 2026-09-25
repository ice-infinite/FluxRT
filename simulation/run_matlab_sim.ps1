# FluxRT "Rust 闭环仿真 + MATLAB 绘图"的 PowerShell 包装器。
# FluxRT PowerShell wrapper for "Rust closed-loop simulation plus MATLAB plotting".
#
# 职责 / Responsibility: 把场景参数以固定区域设置（InvariantCulture）格式化后
# 以批处理模式调用 simulation/matlab/run_foc_matlab.m，并核对图片确实生成。
# Formats the scenario parameters with a fixed culture, runs run_foc_matlab.m in batch mode,
# and verifies that the expected image was produced.
#
# 安全边界 / Safety boundary: 只跑 PC 仿真与离线绘图，不打开串口、不驱动电机，
# 因此没有 foc_stop 路径；实机采集的安全停机见 capture_hardware_trace.py。
# PC simulation and offline plotting only: no serial port, no motor, no foc_stop path.
#
# 参数 / Parameters（默认值与 run_foc_matlab.m 保持一致）:
#   -DurationS      仿真时长 [s]，默认 3.0
#   -TargetRpm      速度目标 [rpm]，默认 524.0
#   -LoadStepTimeS  负载阶跃时刻 [s]，默认 1.0
#   -LoadTorqueNm   负载转矩 [N·m]，默认 0.004
#   -SampleEvery    每多少个 12 kHz 周期记一个样本；只影响数据密度，不影响控制频率
#   -OpenResult     结束后用默认程序打开 PNG（人工查看用，默认关闭）
#
# 输出 / Outputs: simulation/results/foc_sim_overview.png/.fig、
# foc_sim_results.mat、foc_sim_summary.csv、foc_sim_trace.csv；
# 该目录已被 /simulation/results/ 规则忽略，属运行产物，不提交 git。
# All artifacts land in the git-ignored simulation/results directory.
#
# 参考 / Reference: docs/MATLAB联合仿真.md §2-3
param(
    [double]$DurationS = 3.0,
    [double]$TargetRpm = 524.0,
    [double]$LoadStepTimeS = 1.0,
    [double]$LoadTorqueNm = 0.004,
    [int]$SampleEvery = 10,
    [switch]$OpenResult
)

# StrictMode + ErrorActionPreference=Stop：未定义变量或未处理错误立即终止，
# 避免仿真出图不全却以成功退出码收场。
# StrictMode plus ErrorActionPreference=Stop fails fast so an incomplete run cannot exit
# with a success code.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# $PSScriptRoot 即 simulation/，其父目录是工程根，用它推导 MATLAB 脚本目录与结果路径。
# $PSScriptRoot is simulation/, whose parent is the project root.
$projectDir = Split-Path -Parent $PSScriptRoot
$matlabScriptDir = Join-Path $PSScriptRoot 'matlab'
$resultImage = Join-Path $PSScriptRoot 'results\foc_sim_overview.png'
# 先查两个已知安装位置，再退回 PATH 上的 matlab；都没有就报错而不是静默跳过绘图。
# Probes two known install locations, then falls back to matlab on PATH, and fails loudly
# rather than silently skipping the plot.
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

# 必须用 InvariantCulture：逗号小数点的区域设置会把 3.5 变成 "3,5"，
# MATLAB 只会读成 3 并在参数位置报错。
# G17 显式要求 17 位有效数字，使往返精度不随 .NET 版本或区域设置变化。
# InvariantCulture is mandatory: a comma-decimal locale would turn 3.5 into "3,5", which
# MATLAB reads as 3. G17 requests 17 significant digits so round-trip precision does not
# depend on the .NET version or locale.
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$durationText = $DurationS.ToString('G17', $culture)
$targetText = $TargetRpm.ToString('G17', $culture)
$loadStepText = $LoadStepTimeS.ToString('G17', $culture)
$loadText = $LoadTorqueNm.ToString('G17', $culture)
# MATLAB 字符串用单引号定界，路径里的单引号必须翻倍（'' 规则）。
# MATLAB strings are single-quoted, so embedded quotes in the path must be doubled.
$escapedScriptDir = $matlabScriptDir.Replace("'", "''")
# Visible=false 保证无桌面会话也能出图；SampleEvery 是整数，不需要区域格式化。
# Visible=false keeps the run headless; SampleEvery is an integer and needs no formatting.
$batch = "addpath('$escapedScriptDir'); run_foc_matlab(DurationS=$durationText, TargetRpm=$targetText, LoadStepTimeS=$loadStepText, LoadTorqueNm=$loadText, SampleEvery=$SampleEvery, Visible=false);"

Write-Host '[MATLAB] Rust closed-loop simulation and plotting ...' -ForegroundColor Cyan
# Push/Pop-Location：MATLAB 批处理继承当前目录，这里统一在工程根下运行，
# 结束后恢复调用者的目录，避免影响同一会话里的后续步骤。
# Push/Pop-Location: the MATLAB batch inherits the current directory, so it is pinned to
# the project root and the caller's directory is restored afterwards.
Push-Location $projectDir
try
{
    # -batch 无界面运行；非零退出码一律视为失败（cargo 失败会在这里体现）。
    # -batch runs headless; a non-zero exit code is a failure, which is how a cargo error
    # surfaces here.
    & $matlab -batch $batch
    if ($LASTEXITCODE -ne 0) { throw "MATLAB simulation failed: $LASTEXITCODE" }
}
finally
{
    Pop-Location
}

# 文件存在性是最终判据：MATLAB 可能成功退出但没写出图片。
# File existence is the final criterion: MATLAB can exit zero without writing the image.
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
