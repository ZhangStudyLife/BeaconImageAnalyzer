param(
    [switch]$Force,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$BuildScript = Join-Path $PSScriptRoot "build.ps1"
$Exe = Join-Path $ProjectRoot "build/BeaconImageAnalyzer.exe"
$TwoBl3DiagnosticDll = Join-Path $ProjectRoot "build/two_bl3_diagnostic.dll"
$WindeployQt = Join-Path $MsysBin "windeployqt.exe"
$Objdump = Join-Path $MsysBin "objdump.exe"
$DistRoot = Join-Path $ProjectRoot "dist"
$PackageName = "BeaconImageAnalyzer-portable"
$StageDir = Join-Path $DistRoot $PackageName
$ZipPath = Join-Path $DistRoot "$PackageName.zip"

function Assert-FileExists($Path, $Name) {
    if (!(Test-Path -LiteralPath $Path)) {
        throw "Missing ${Name}: $Path"
    }
}

function Get-ImportedDllNames($BinaryPath) {
    $lines = & $Objdump -p $BinaryPath | Select-String "DLL Name:"
    foreach ($line in $lines) {
        $name = ($line.Line -replace ".*DLL Name:\s*", "").Trim()
        if ($name.Length -gt 0) {
            $name
        }
    }
}

function Test-IsSystemDll($DllName) {
    $lower = $DllName.ToLowerInvariant()
    if ($lower.StartsWith("api-ms-") -or $lower.StartsWith("ext-ms-")) {
        return $true
    }

    $systemDlls = @(
        "advapi32.dll", "authz.dll", "comdlg32.dll", "crypt32.dll", "d3d11.dll",
        "d3d12.dll", "dcomp.dll", "dwmapi.dll", "dwrite.dll", "dxgi.dll",
        "gdi32.dll", "imm32.dll", "iphlpapi.dll", "kernel32.dll", "mpr.dll",
        "msvcrt.dll", "netapi32.dll", "ntdll.dll", "ole32.dll", "oleaut32.dll",
        "opengl32.dll", "rpcrt4.dll", "secur32.dll", "setupapi.dll", "shell32.dll",
        "shlwapi.dll", "user32.dll", "userenv.dll", "uxtheme.dll", "version.dll",
        "winmm.dll", "winspool.drv", "ws2_32.dll"
    )
    return $systemDlls -contains $lower
}

function Find-StagedBinary($DllName) {
    Get-ChildItem -LiteralPath $StageDir -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { $_.Name.Equals($DllName, [System.StringComparison]::OrdinalIgnoreCase) } |
        Select-Object -First 1
}

function Copy-MsysDll($DllName) {
    if (Test-IsSystemDll $DllName) {
        return $null
    }

    $existing = Find-StagedBinary $DllName
    if ($null -ne $existing) {
        return $existing.FullName
    }

    $source = Join-Path $MsysBin $DllName
    if (!(Test-Path -LiteralPath $source)) {
        Write-Warning "Dependency DLL was not found in MSYS2, it may be a system or delay-loaded library: $DllName"
        return $null
    }

    $target = Join-Path $StageDir $DllName
    Copy-Item -LiteralPath $source -Destination $target -Force
    Write-Host "Copied dependency: $DllName"
    return $target
}

function Copy-RecursiveDependencies() {
    $queue = New-Object System.Collections.Generic.Queue[string]
    $seen = New-Object "System.Collections.Generic.HashSet[string]"

    Get-ChildItem -LiteralPath $StageDir -Recurse -File |
        Where-Object { $_.Extension -in @(".exe", ".dll") } |
        ForEach-Object { $queue.Enqueue($_.FullName) }

    while ($queue.Count -gt 0) {
        $binary = $queue.Dequeue()
        $key = [System.IO.Path]::GetFullPath($binary).ToLowerInvariant()
        if (!$seen.Add($key)) {
            continue
        }

        foreach ($dllName in Get-ImportedDllNames $binary) {
            $copied = Copy-MsysDll $dllName
            if ($null -ne $copied) {
                $copiedKey = [System.IO.Path]::GetFullPath($copied).ToLowerInvariant()
                if (!$seen.Contains($copiedKey)) {
                    $queue.Enqueue($copied)
                }
            }
        }
    }
}

Assert-FileExists $BuildScript "build script"
Assert-FileExists $WindeployQt "windeployqt"
Assert-FileExists $Objdump "objdump"

if (!$SkipBuild) {
    Write-Host "Building Release..."
    & $BuildScript
}

Assert-FileExists $Exe "BeaconImageAnalyzer.exe"

New-Item -ItemType Directory -Force -Path $DistRoot | Out-Null

if ((Test-Path -LiteralPath $StageDir) -or (Test-Path -LiteralPath $ZipPath)) {
    if (!$Force) {
        throw "Output already exists: $StageDir or $ZipPath. To overwrite it, run: ./tools/package.ps1 -Force"
    }

    if (Test-Path -LiteralPath $StageDir) {
        Remove-Item -LiteralPath $StageDir -Recurse -Force
    }
    if (Test-Path -LiteralPath $ZipPath) {
        Remove-Item -LiteralPath $ZipPath -Force
    }
}

New-Item -ItemType Directory -Force -Path $StageDir | Out-Null
Copy-Item -LiteralPath $Exe -Destination $StageDir -Force
if (Test-Path -LiteralPath $TwoBl3DiagnosticDll) {
    Copy-Item -LiteralPath $TwoBl3DiagnosticDll -Destination $StageDir -Force
}

$env:Path = "$MsysBin;$env:Path"

Write-Host "Deploying Qt runtime..."
& $WindeployQt --release --compiler-runtime --no-translations (Join-Path $StageDir "BeaconImageAnalyzer.exe")

Write-Host "Collecting OpenCV / MinGW / FFmpeg dependencies..."
Copy-RecursiveDependencies

$LauncherPath = Join-Path $StageDir "Start BeaconImageAnalyzer.bat"
@(
    '@echo off',
    'setlocal',
    'set "APP_DIR=%~dp0"',
    'set "PATH=%APP_DIR%;%APP_DIR%platforms;%PATH%"',
    'start "" "%APP_DIR%BeaconImageAnalyzer.exe" %*'
) | Set-Content -LiteralPath $LauncherPath -Encoding ASCII

$ReadmePath = Join-Path $StageDir "README-portable.txt"
@(
    'BeaconImageAnalyzer portable package',
    '',
    'Usage:',
    '1. Extract the whole BeaconImageAnalyzer-portable folder.',
    '2. Double-click BeaconImageAnalyzer.exe or Start BeaconImageAnalyzer.bat.',
    '3. Qt, OpenCV, MSYS2, and PATH configuration are not required on the target machine.',
    '',
    'Notes:',
    '- Do not copy only the exe. Keep all DLL files and plugin folders together.',
    '- If Windows SmartScreen blocks the app, choose to run it anyway.',
    '- If an AVI cannot be opened, verify the file path is readable and the file is not locked by another app.'
) | Set-Content -LiteralPath $ReadmePath -Encoding UTF8

Write-Host "Creating ZIP..."
Compress-Archive -LiteralPath $StageDir -DestinationPath $ZipPath -CompressionLevel Optimal

Write-Host "Portable package generated:"
Write-Host "  $StageDir"
Write-Host "  $ZipPath"
