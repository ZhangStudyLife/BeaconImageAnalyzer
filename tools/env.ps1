$ErrorActionPreference = "Stop"

$env:BEACON_IMAGE_ANALYZER_ENV = "BeaconImageAnalyzer"

if ([string]::IsNullOrWhiteSpace($env:BEACON_IMAGE_ANALYZER_MSYS_ROOT)) {
    $env:BEACON_IMAGE_ANALYZER_MSYS_ROOT = "C:/code/msys64/mingw64"
}

$MsysRoot = $env:BEACON_IMAGE_ANALYZER_MSYS_ROOT
$MsysBin = "$MsysRoot/bin"

if (!(Test-Path -LiteralPath $MsysBin)) {
    throw "未找到 BeaconImageAnalyzer MSYS2 环境: $MsysBin"
}

$env:Path = "$MsysBin;$env:Path"
