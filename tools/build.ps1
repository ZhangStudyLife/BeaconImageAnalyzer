$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$CMake = "$MsysRoot/bin/cmake.exe"
$Ninja = "$MsysRoot/bin/ninja.exe"
$BuildDir = "$ProjectRoot/build"
$Exe = "$BuildDir/BeaconImageAnalyzer.exe"

if (!(Test-Path -LiteralPath $CMake)) {
    throw "CMake not found: $CMake"
}
if (!(Test-Path -LiteralPath $Ninja)) {
    throw "Ninja not found: $Ninja"
}

$ConfigureArgs = @()
$CacheFile = "$BuildDir/CMakeCache.txt"
if (Test-Path -LiteralPath $CacheFile) {
    $HomeLine = Select-String -LiteralPath $CacheFile -Pattern "^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($HomeLine -ne $null) {
        $CachedSource = $HomeLine.Matches[0].Groups[1].Value
        $CurrentSource = [System.IO.Path]::GetFullPath($ProjectRoot).TrimEnd('\', '/')
        $CachedFull = [System.IO.Path]::GetFullPath($CachedSource).TrimEnd('\', '/')
        if ([string]::Compare($CachedFull, $CurrentSource, $true) -ne 0) {
            $ConfigureArgs += "--fresh"
        }
    }
}

$ConfigureArgs += @(
    "-S", "$ProjectRoot",
    "-B", "$BuildDir",
    "-G", "Ninja",
    "-DCMAKE_PREFIX_PATH=$MsysRoot",
    "-DCMAKE_BUILD_TYPE=Release"
)

& $CMake @ConfigureArgs
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

if (Test-Path -LiteralPath $Exe) {
    $RunningExe = Get-Process -ErrorAction SilentlyContinue | Where-Object {
        try {
            $_.Path -eq $Exe
        } catch {
            $false
        }
    } | Select-Object -First 1
    if ($RunningExe -ne $null) {
        throw "BeaconImageAnalyzer.exe is running from build directory. Close it before rebuilding. PID=$($RunningExe.Id)"
    }
}

& $CMake --build "$BuildDir" --config Release
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
