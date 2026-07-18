$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$script = Join-Path $PSScriptRoot "parameter_compare_gui.py"
$python = (Get-Command python -ErrorAction Stop).Source

Start-Process -FilePath $python `
    -ArgumentList @($script) `
    -WorkingDirectory $root
