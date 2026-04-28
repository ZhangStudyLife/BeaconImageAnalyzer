$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$Exe = "$ProjectRoot/build/BeaconImageAnalyzer.exe"

if (!(Test-Path -LiteralPath $Exe)) {
    throw "未找到可执行文件: $Exe，请先运行 tools/build.ps1"
}

& $Exe @args
