$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$Exe = "$ProjectRoot/build/BeaconImageAnalyzer.exe"

if (!(Test-Path -LiteralPath $Exe)) {
    & "$PSScriptRoot/build.ps1"
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

if (!(Test-Path -LiteralPath $Exe)) {
    throw "Executable not found after build: $Exe"
}

& $Exe @args
exit $LASTEXITCODE
