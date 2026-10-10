param(
    [int]$Port = 8760,
    [string]$TeleopProfile = 'config/devices/minerva-teleop.json',
    [string]$SshKeyFile
)
$ErrorActionPreference = 'Stop'
$helmRoot = Split-Path $PSScriptRoot -Parent
$helmPython = Join-Path $helmRoot '.venv-dev/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $helmPython)) {
    $helmPython = Join-Path $helmRoot '.venv-platform/Scripts/python.exe'
}
if (-not (Test-Path -LiteralPath $helmPython)) {
    throw 'No checkout-owned Python environment. Run python scripts/dev.py setup in this checkout first.'
}
$helmImportCheck = @'
from pathlib import Path
import sys
import robot_platform
expected = (Path(sys.argv[1]) / "src/robot_platform/__init__.py").resolve()
actual = Path(robot_platform.__file__).resolve()
if actual != expected:
    raise SystemExit(f"Refusing Helm from another checkout: {actual}; expected {expected}. Run scripts/dev.py setup here.")
'@
& $helmPython -c $helmImportCheck $helmRoot
if ($LASTEXITCODE -ne 0) { throw 'Helm import verification failed; no session was started.' }
$helmArgs = @('-m', 'robot_platform', 'helm',
    '--profile', (Join-Path $helmRoot 'config/devices/minerva-joint-calibration.json'),
    '--runs', (Join-Path $helmRoot 'calibration_runs'),
    '--seed', (Join-Path $helmRoot 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'),
    '--teleop', (Join-Path $helmRoot $TeleopProfile),
    '--port', "$Port")
$helmKeyPath = $null
if ($SshKeyFile) {
    if (-not (Test-Path -LiteralPath $SshKeyFile -PathType Leaf)) {
        throw 'The explicitly supplied SshKeyFile must identify an existing file.'
    }
    $helmKeyPath = (Resolve-Path -LiteralPath $SshKeyFile -ErrorAction Stop).ProviderPath
}
$previousSshKey = [Environment]::GetEnvironmentVariable('AIZEE_SSH_KEY_FILE', 'Process')
try {
    # Use the existing key in place for this invocation; never copy or discover keys.
    if ($helmKeyPath) { $env:AIZEE_SSH_KEY_FILE = $helmKeyPath }
    & $helmPython @helmArgs
    $helmExit = $LASTEXITCODE
} finally {
    [Environment]::SetEnvironmentVariable('AIZEE_SSH_KEY_FILE', $previousSshKey, 'Process')
}
exit $helmExit
