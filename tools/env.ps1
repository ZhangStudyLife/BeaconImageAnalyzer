$ErrorActionPreference = "Stop"

$env:BEACON_IMAGE_ANALYZER_ENV = "BeaconImageAnalyzer"

if ([string]::IsNullOrWhiteSpace($env:BEACON_IMAGE_ANALYZER_MSYS_ROOT)) {
    $CandidateRoots = @(
        "C:/code/msys64/mingw64",
        "C:/msys64/mingw64"
    )
    foreach ($CandidateRoot in $CandidateRoots) {
        if (Test-Path -LiteralPath "$CandidateRoot/bin") {
            $env:BEACON_IMAGE_ANALYZER_MSYS_ROOT = $CandidateRoot
            break
        }
    }
}

$MsysRoot = $env:BEACON_IMAGE_ANALYZER_MSYS_ROOT
$MsysBin = "$MsysRoot/bin"

if (!(Test-Path -LiteralPath $MsysBin)) {
    throw "未找到 BeaconImageAnalyzer MSYS2 环境: $MsysBin"
}

$env:Path = "$MsysBin;$env:Path"
