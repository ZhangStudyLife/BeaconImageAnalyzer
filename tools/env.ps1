$ErrorActionPreference = "Stop"

$MsysRoot = "C:/msys64/mingw64"
if (!(Test-Path -LiteralPath "$MsysRoot/bin")) {
    throw "未找到 MSYS2 MinGW64 环境: $MsysRoot/bin"
}

$env:Path = "$MsysRoot/bin;$env:Path"
