$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$Exe = "$ProjectRoot/build/JustFloatMonitor.exe"

if (!(Test-Path -LiteralPath $Exe)) {
    & "$PSScriptRoot/build.ps1"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

& $Exe @args
exit $LASTEXITCODE
