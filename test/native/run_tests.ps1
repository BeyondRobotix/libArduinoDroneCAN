<#
    Builds and runs the host-side storage tests with MSVC.

    src/storage.cpp is compiled against a RAM model of the flash peripheral
    (flash_model.cpp) so the log/compaction/recovery state machine can be
    exercised without hardware. It is built and run TWICE, once per flash
    geometry, because the two families differ in ways the algorithm depends
    on -- 32-byte write-once words on H7 vs 8-byte double words on L4:

        H7 : 128 KB sectors, 32-byte records -> 4093 records/sector
        L4 :   2 KB pages,    8-byte records ->  250 records/page

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
    (Join-Path $here 'test_storage.cpp')
)

$common = @(
    '/DSTORAGE_HOST_TEST', '/DWIN32_LEAN_AND_MEAN', '/D_CRT_SECURE_NO_WARNINGS'
)

# Per-geometry defines. MODEL_H7 switches flash_model.h; the STM32* defines
# switch storage.cpp. BOARD_FLASH_SIZE and STORAGE_FLASH_PAGE match each
# model's page count, with the storage pair at the top of the modelled flash.
$targets = @(
    @{ Name = 'H7'
       Defines = @('/DMODEL_H7', '/DSTM32H7', '/DSTM32H7xx',
                   '/DBOARD_FLASH_SIZE=1024', '/DSTORAGE_FLASH_PAGE=6') },
    @{ Name = 'L4'
       Defines = @('/DSTM32L4', '/DSTM32L4xx',
                   '/DBOARD_FLASH_SIZE=256', '/DSTORAGE_FLASH_PAGE=126') }
)

$failed = 0
foreach ($t in $targets) {
    $exe = Join-Path $out "test_storage_$($t.Name).exe"
    $obj = Join-Path $out "$($t.Name)"
    New-Item -ItemType Directory -Force -Path $obj | Out-Null

    Write-Host "=== building $($t.Name) geometry ===" -ForegroundColor Cyan
    & $cl /nologo /std:c++17 /EHsc /W3 /Zi `
        ($common + $t.Defines) `
        "/I$here" `
        $src `
        /Fe:"$exe" `
        /Fo:"$obj\" /Fd:"$obj\test_storage.pdb" `
        | Write-Host

    if ($LASTEXITCODE -ne 0) { throw "$($t.Name) compilation failed ($LASTEXITCODE)" }

    Write-Host ''
    & $exe
    if ($LASTEXITCODE -ne 0) {
        $failed++
        Write-Host "$($t.Name): FAIL (exit $LASTEXITCODE)" -ForegroundColor Red
    } else {
        Write-Host "$($t.Name): PASS" -ForegroundColor Green
    }
    Write-Host ''
}

if ($failed -eq 0) { Write-Host 'ALL GEOMETRIES PASS' -ForegroundColor Green; exit 0 }
Write-Host "$failed geometry/geometries FAILED" -ForegroundColor Red
exit 1
