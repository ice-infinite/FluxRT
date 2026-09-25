# FluxRT —— 注释改动的代码等价性校验（临时工具，验证后删除）
# FluxRT - code-equivalence check for comment-only edits (temporary tool).
#
# 用途 / Purpose:
#   剥离 C/Rust 注释与空白后，把当前工作区的代码本体与指定 git 修订比对。
#   只加注释的改动必须得到"零差异"；出现差异即说明误改了代码。
#   Strips C/Rust comments and whitespace, then compares the current working tree
#   against a git revision. A comment-only edit must show zero differences; any
#   difference means code was changed by mistake.

param(
    [string]$Revision = 'HEAD',
    [string[]]$Files
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root

function Get-CodeBody {
    param([string[]]$Lines, [string]$Ext)
    $txt = ($Lines -join "`n")
    if ($Ext -in '.c', '.h', '.cpp', '.lds') {
        $txt = [regex]::Replace($txt, '/\*.*?\*/', '', 'Singleline')
        $txt = [regex]::Replace($txt, '//[^\n]*', '')
    }
    elseif ($Ext -eq '.rs') {
        $txt = [regex]::Replace($txt, '/\*.*?\*/', '', 'Singleline')
        $txt = [regex]::Replace($txt, '//[^\n]*', '')
    }
    elseif ($Ext -in '.py', '.ps1', '.cmake') {
        # 注意 / Caveat: 本行级剥离对 Python 是**不充分**的 —— docstring 不是
        # '#' 注释，会被当成代码而误报差异。校验 .py 请改用 AST 比对：
        # This line-based strip is INSUFFICIENT for Python: a docstring is not a
        # '#' comment and is counted as code, producing false differences. For
        # .py files use an AST comparison instead.
        $txt = [regex]::Replace($txt, '#[^\n]*', '')
    }
    elseif ($Ext -eq '.m') {
        $txt = [regex]::Replace($txt, '%[^\n]*', '')
    }
    ($txt -split "`n") | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' }
}

$bad = 0
$okCount = 0
foreach ($file in $Files) {
    if (-not (Test-Path -LiteralPath $file)) { Write-Host "  MISSING  $file"; $bad++; continue }
    $ext = [System.IO.Path]::GetExtension($file)
    $head = & git show "${Revision}:$file" 2>$null
    $cur = Get-Content -LiteralPath $file
    $a = Get-CodeBody -Lines $head -Ext $ext
    $b = Get-CodeBody -Lines $cur -Ext $ext
    $d = Compare-Object $a $b
    if (($d | Measure-Object).Count -eq 0) {
        Write-Host "  OK       $file"
        $okCount++
    }
    else {
        Write-Host "  CHANGED  $file  ($(($d | Measure-Object).Count) code-body differences)"
        $d | Select-Object -First 10 | ForEach-Object { Write-Host "             $($_.SideIndicator) $($_.InputObject)" }
        $bad++
    }
}
Pop-Location
Write-Host ""
Write-Host "  equivalent: $okCount    changed: $bad"
if ($bad -gt 0) { exit 1 }
