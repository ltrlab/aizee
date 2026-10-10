param(
    [int]$Port = 8760,
    [string]$TeleopProfile = 'config/devices/minerva-teleop.json'
)
$ErrorActionPreference = 'Stop'
$helmRoot = Split-Path $PSScriptRoot -Parent
$helmPython = Join-Path $helmRoot '.venv-platform/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $helmPython)) { $helmPython = 'python' }
$helmArgs = @('-m', 'robot_platform', 'helm',
    '--profile', (Join-Path $helmRoot 'config/devices/minerva-joint-calibration.json'),
    '--runs', (Join-Path $helmRoot 'calibration_runs'),
    '--seed', (Join-Path $helmRoot 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'),
    '--teleop', (Join-Path $helmRoot $TeleopProfile),
    '--port', "$Port")
& $helmPython @helmArgs
exit $LASTEXITCODE
