param([switch]$Force)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"
$Exe = "$ProjectRoot/build/JustFloatMonitor.exe"
$DistRoot = "$ProjectRoot/dist"
$StageDir = "$DistRoot/JustFloatMonitor-portable"
$ZipPath = "$DistRoot/JustFloatMonitor-portable.zip"

& "$PSScriptRoot/build.ps1"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ((Test-Path -LiteralPath $StageDir) -or (Test-Path -LiteralPath $ZipPath)) {
    if (!$Force) { throw "输出已存在，请使用 -Force 覆盖。" }
    if (Test-Path -LiteralPath $StageDir) { Remove-Item -LiteralPath $StageDir -Recurse -Force }
    if (Test-Path -LiteralPath $ZipPath) { Remove-Item -LiteralPath $ZipPath -Force }
}

New-Item -ItemType Directory -Force -Path $StageDir | Out-Null
Copy-Item -LiteralPath $Exe -Destination $StageDir
& "$MsysRoot/bin/windeployqt.exe" --release --compiler-runtime --no-translations `
    "$StageDir/JustFloatMonitor.exe"

$Objdump = "$MsysRoot/bin/objdump.exe"
$Queue = New-Object System.Collections.Generic.Queue[string]
$Seen = New-Object "System.Collections.Generic.HashSet[string]"
Get-ChildItem -LiteralPath $StageDir -Recurse -File | Where-Object { $_.Extension -in @(".exe", ".dll") } |
    ForEach-Object { $Queue.Enqueue($_.FullName) }

while ($Queue.Count -gt 0) {
    $Binary = $Queue.Dequeue()
    if (!$Seen.Add([System.IO.Path]::GetFullPath($Binary).ToLowerInvariant())) { continue }
    $Lines = & $Objdump -p $Binary | Select-String "DLL Name:"
    foreach ($Line in $Lines) {
        $Name = ($Line.Line -replace ".*DLL Name:\s*", "").Trim()
        $Source = "$MsysRoot/bin/$Name"
        $Target = "$StageDir/$Name"
        if ((Test-Path -LiteralPath $Source) -and !(Test-Path -LiteralPath $Target)) {
            Copy-Item -LiteralPath $Source -Destination $Target
            $Queue.Enqueue($Target)
        }
    }
}

@(
    "JustFloat Monitor portable package",
    "",
    "Run JustFloatMonitor.exe.",
    "UDP format: 43 little-endian float32 values, with optional 00 00 80 7F tail.",
    "Default UDP port: 1347."
) | Set-Content -LiteralPath "$StageDir/README.txt" -Encoding UTF8

Compress-Archive -LiteralPath $StageDir -DestinationPath $ZipPath -CompressionLevel Optimal
Write-Host "Portable package: $ZipPath"
