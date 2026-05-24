param(
    [switch]$Force,
    [switch]$SkipBuild,
    [string]$Version = (Get-Date -Format "yyyy.MM.dd.HHmm"),
    [string]$InnoSetupPath = "D:/Inno Setup 6/ISCC.exe"
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. "$PSScriptRoot/env.ps1"

$BuildScript = Join-Path $PSScriptRoot "build.ps1"
$PackageScript = Join-Path $PSScriptRoot "package.ps1"
$InnoScript = Join-Path $ProjectRoot "installer/BeaconImageAnalyzer.iss"
$DistRoot = Join-Path $ProjectRoot "dist"
$StageDir = Join-Path $DistRoot "BeaconImageAnalyzer-portable"
$SetupExe = Join-Path $DistRoot "BeaconImageAnalyzer-Setup-$Version.exe"

function Assert-FileExists($Path, $Name) {
    if (!(Test-Path -LiteralPath $Path)) {
        throw "Missing ${Name}: $Path"
    }
}

if (!(Test-Path -LiteralPath $InnoSetupPath)) {
    $Candidates = @(
        "D:/Inno Setup 6/ISCC.exe",
        "C:/Program Files (x86)/Inno Setup 6/ISCC.exe",
        "C:/Program Files/Inno Setup 6/ISCC.exe",
        "C:/Program Files (x86)/Inno Setup 5/ISCC.exe",
        "C:/Program Files/Inno Setup 5/ISCC.exe"
    )
    foreach ($Candidate in $Candidates) {
        if (Test-Path -LiteralPath $Candidate) {
            $InnoSetupPath = $Candidate
            break
        }
    }
}

Assert-FileExists $BuildScript "build script"
Assert-FileExists $PackageScript "package script"
Assert-FileExists $InnoScript "Inno Setup script"
Assert-FileExists $InnoSetupPath "Inno Setup compiler"

New-Item -ItemType Directory -Force -Path $DistRoot | Out-Null

if ((Test-Path -LiteralPath $SetupExe) -and !$Force) {
    throw "Output already exists: $SetupExe. To overwrite it, run: ./tools/installer.ps1 -Force"
}
if (Test-Path -LiteralPath $SetupExe) {
    Remove-Item -LiteralPath $SetupExe -Force
}

if (!$SkipBuild) {
    Write-Host "Building Release..."
    & $BuildScript
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

Write-Host "Preparing portable payload..."
& $PackageScript -Force -SkipBuild
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Assert-FileExists $StageDir "portable payload"

$env:BEACON_INSTALLER_VERSION = $Version
$env:BEACON_PROJECT_ROOT = $ProjectRoot
$env:BEACON_STAGE_DIR = $StageDir
$env:BEACON_OUTPUT_DIR = $DistRoot

Write-Host "Compiling Inno Setup installer..."
& $InnoSetupPath $InnoScript
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Assert-FileExists $SetupExe "installer"
Write-Host "Installer generated:"
Write-Host "  $SetupExe"
