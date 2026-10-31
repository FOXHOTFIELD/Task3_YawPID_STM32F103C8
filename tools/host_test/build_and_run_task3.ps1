#!/usr/bin/env pwsh
<#
.SYNOPSIS
    Build and run the PC-side unit tests for Task 3 (TB6612 + Yaw PID).

.DESCRIPTION
    Compiles only the hardware-independent modules of Task3:
      app_pid.c    position / incremental PID, anti-windup, derivative-on-measurement
      app_yaw.c    yaw integration, wrap to [-180,180), shortest path
      app_util.c   float/fixed parsing, Newton sqrt, fixed-point formatting
    No HAL stub is needed because none of these files touch the hardware.

.NOTES
    Same two environment traps as build_and_run.ps1:
      1) msys2 gcc needs C:\msys64\ucrt64\bin on PATH;
      2) gcc mangles non-ASCII paths, so sources go to an ASCII temp dir first.

    Usage: powershell -NoProfile -ExecutionPolicy Bypass -File build_and_run_task3.ps1
#>
param(
    [string]$Root = ''
)

$ErrorActionPreference = 'Stop'
$env:PATH = 'C:\msys64\ucrt64\bin;C:\msys64\usr\bin;' + $env:PATH

if (-not $Root -or $Root -eq '') {
    # 本脚本在 <仓库>/tools/host_test/ 下，往上两级就是仓库根
    $Root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
    if (-not $Root -or $Root -eq '') { throw 'cannot resolve repo root from $PSScriptRoot' }
}

$t3   = $Root
$ht   = $PSScriptRoot
$tmp  = Join-Path $env:TEMP 'dsh_host_test_task3'
$gcc  = 'C:\msys64\ucrt64\bin\gcc.exe'

if (-not (Test-Path $gcc)) { throw "gcc not found: $gcc" }
if (Test-Path $tmp) { Remove-Item $tmp -Recurse -Force }
New-Item -ItemType Directory -Force -Path "$tmp\app" | Out-Null

Copy-Item (Join-Path $ht 'test_task3.c') "$tmp\" -Force
foreach ($f in @('app_pid.h', 'app_yaw.h', 'app_util.h')) {
    Copy-Item (Join-Path $t3 "App\Inc\$f") "$tmp\app\" -Force
}
foreach ($f in @('app_pid.c', 'app_yaw.c', 'app_util.c')) {
    Copy-Item (Join-Path $t3 "App\Src\$f") "$tmp\app\" -Force
}

Push-Location $tmp
try {
    $args = @(
        '-std=c99', '-Wall', '-Wextra', '-O2',
        '-I', 'app',
        'test_task3.c',
        'app\app_pid.c', 'app\app_yaw.c', 'app\app_util.c',
        '-lm', '-o', 'test_task3.exe'
    )
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $gcc @args *> (Join-Path $tmp 'build.log')
    $code = $LASTEXITCODE
    $ErrorActionPreference = $prev

    Get-Content (Join-Path $tmp 'build.log') -ErrorAction SilentlyContinue | ForEach-Object { Write-Host $_ }
    if ($code -ne 0 -or -not (Test-Path "$tmp\test_task3.exe")) {
        throw "build failed (gcc exit=$code)"
    }
    Write-Host "build OK: $tmp\test_task3.exe"
    Write-Host ''
    & "$tmp\test_task3.exe"
    $rc = $LASTEXITCODE
    Write-Host ''
    Write-Host "task3 logic test exit code = $rc  (0 = all passed)"
    exit $rc
}
finally {
    Pop-Location
}
