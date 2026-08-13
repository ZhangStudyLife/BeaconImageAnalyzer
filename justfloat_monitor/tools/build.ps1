$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"

& "$MsysRoot/bin/cmake.exe" -S $ProjectRoot -B "$ProjectRoot/build" -G Ninja `
    "-DCMAKE_PREFIX_PATH=$MsysRoot" "-DCMAKE_BUILD_TYPE=Release" "-DBUILD_TESTING=ON"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& "$MsysRoot/bin/cmake.exe" --build "$ProjectRoot/build"
exit $LASTEXITCODE
