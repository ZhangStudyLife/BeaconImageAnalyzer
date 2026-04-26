$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$MsysRoot = "C:/msys64/mingw64"
$Exe = "$ProjectRoot/build/BeaconImageAnalyzer.exe"

if (!(Test-Path -LiteralPath $Exe)) {
    throw "未找到可执行文件: $Exe，请先运行 tools/build.ps1"
}

$env:Path = "$MsysRoot/bin;$env:Path"
& $Exe @args
