$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$MsysRoot = "C:/msys64/mingw64"
$CMake = "$MsysRoot/bin/cmake.exe"
$Ninja = "$MsysRoot/bin/ninja.exe"

if (!(Test-Path -LiteralPath $CMake)) {
    throw "未找到 CMake: $CMake"
}
if (!(Test-Path -LiteralPath $Ninja)) {
    throw "未找到 Ninja: $Ninja"
}

$env:Path = "$MsysRoot/bin;$env:Path"

& $CMake -S "$ProjectRoot" -B "$ProjectRoot/build" -G "Ninja" `
    -DCMAKE_PREFIX_PATH="$MsysRoot" `
    -DCMAKE_BUILD_TYPE=Release

& $CMake --build "$ProjectRoot/build" --config Release
