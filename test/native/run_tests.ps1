<#
    Builds and runs the host-side storage tests with MSVC.

    These compile src/storage.cpp for the H7 path against a RAM model of the
    flash peripheral (flash_model.cpp), so the log/compaction/recovery state
    machine can be exercised without hardware.

    Usage:  pwsh test/native/run_tests.ps1
#>

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$out  = Join-Path $here 'build'

# Locate MSVC via vswhere, falling back to a cl.exe already on PATH.
$cl = (Get-Command cl.exe -ErrorAction SilentlyContinue).Source
if (-not $cl) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        throw "MSVC not found: no cl.exe on PATH and no vswhere.exe at $vswhere"
    }
    $vsPath = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if (-not $vsPath) { throw "vswhere found no VC++ toolset installation" }

    $verFile = Join-Path $vsPath 'VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt'
    $ver = (Get-Content $verFile).Trim()
    $cl = Join-Path $vsPath "VC\Tools\MSVC\$ver\bin\Hostx64\x64\cl.exe"
    $incVC = Join-Path $vsPath "VC\Tools\MSVC\$ver\include"

    # Windows SDK headers/libs
    $sdkRoot = (Get-ItemProperty 'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\Windows\v10.0').InstallationFolder
    $sdkVer  = (Get-ChildItem (Join-Path $sdkRoot 'Include') |
                Sort-Object Name -Descending | Select-Object -First 1).Name

    $env:INCLUDE = @(
        $incVC,
        (Join-Path $sdkRoot "Include\$sdkVer\ucrt"),
        (Join-Path $sdkRoot "Include\$sdkVer\shared"),
        (Join-Path $sdkRoot "Include\$sdkVer\um")
    ) -join ';'
    $env:LIB = @(
        (Join-Path $vsPath "VC\Tools\MSVC\$ver\lib\x64"),
        (Join-Path $sdkRoot "Lib\$sdkVer\ucrt\x64"),
        (Join-Path $sdkRoot "Lib\$sdkVer\um\x64")
    ) -join ';'
}

New-Item -ItemType Directory -Force -Path $out | Out-Null

$src = @(
    (Join-Path $here '..\..\src\storage.cpp'),
    (Join-Path $here 'flash_model.cpp'),
    (Join-Path $here 'test_storage_h7.cpp')
)

# STM32H7 + STM32H7xx select the log-structured path; BOARD_FLASH_SIZE keeps
# storage.cpp's expectations aligned with the 1 MB / 8-sector flash model.
$defines = @(
    '/DSTM32H7', '/DSTM32H7xx', '/DBOARD_FLASH_SIZE=1024', '/DSTORAGE_HOST_TEST',
    '/DWIN32_LEAN_AND_MEAN', '/D_CRT_SECURE_NO_WARNINGS'
)

& $cl /nologo /std:c++17 /EHsc /W3 /Zi `
    $defines `
    "/I$here" `
    $src `
    /Fe:"$out\test_storage.exe" `
    /Fo:"$out\" /Fd:"$out\test_storage.pdb" `
    | Write-Host

if ($LASTEXITCODE -ne 0) { throw "compilation failed ($LASTEXITCODE)" }

Write-Host ''
& "$out\test_storage.exe"
$rc = $LASTEXITCODE
Write-Host ''
if ($rc -eq 0) { Write-Host 'PASS' -ForegroundColor Green }
else { Write-Host "FAIL (exit $rc)" -ForegroundColor Red }
exit $rc
