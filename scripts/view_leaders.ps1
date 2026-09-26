param(
    [string]$Profile = '',
    [string]$Calibration = '',
    [int]$Port = 8767,
    [ValidateRange(25, 100)][int]$Rate = 100
)
$ErrorActionPreference = 'Stop'
$leaderRoot = Split-Path $PSScriptRoot -Parent
$leaderPython = Join-Path $leaderRoot '.venv-platform/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $leaderPython)) { $leaderPython = 'python' }
if (-not $Profile) { $Profile = Join-Path $leaderRoot 'config/devices/gello-dual-viewer.json' }
if (-not $Calibration) { $Calibration = Join-Path $leaderRoot 'calibration_runs/gello-dual/display.json' }
& $leaderPython -m robot_platform leaders --profile $Profile --calibration $Calibration --port $Port --rate $Rate
exit $LASTEXITCODE
