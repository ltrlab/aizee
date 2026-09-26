param(
    [ValidateSet('Live', 'Directions', 'Check', 'Plan')][string]$Mode = 'Live',
    [string]$Output = '',
    [int]$Port = 8766
)
$ErrorActionPreference = 'Stop'
$calibrationRoot = Split-Path $PSScriptRoot -Parent
$calibrationPython = Join-Path $calibrationRoot '.venv-platform/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $calibrationPython)) { $calibrationPython = 'python' }
if (-not $Output) { $Output = Join-Path $calibrationRoot 'calibration_runs/minerva-production-01-directions' }
$calibrationArgs = @('-m', 'robot_platform', 'calibrate', '--profile',
    (Join-Path $calibrationRoot 'config/devices/minerva-joint-calibration.json'),
    '--output', $Output, '--resume', '--seed',
    (Join-Path $calibrationRoot 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'), '--port', "$Port")
if ($Mode -eq 'Live') { $calibrationArgs += '--live' }
if ($Mode -eq 'Check') { $calibrationArgs += '--check' }
if ($Mode -eq 'Plan') { $calibrationArgs += '--plan' }
& $calibrationPython @calibrationArgs
exit $LASTEXITCODE
