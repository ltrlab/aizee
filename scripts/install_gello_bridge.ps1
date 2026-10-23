<#
.SYNOPSIS
Install or update the GELLO bridge on the operator PC, for Helm on Minerva (docs/development/HELM_ON_JETSON.md).

.DESCRIPTION
Run as the operator, not as administrator, on the PC the GELLO leader arms and the tether are plugged into:

    powershell -ExecutionPolicy Bypass -File install_gello_bridge.ps1 -Package gello-bridge-<commit>.zip

It checks every file of the package against its manifest, unpacks it into %LOCALAPPDATA%\Minerva, installs the
hash-locked dependencies (requirements\gello-bridge.lock) into a per-lock environment, makes the bridge's SSH key once,
and registers the logon task Minerva-GELLO-Bridge, which keeps the bridge running and restarts it if it stops. Running
it again with a newer package moves the bridge to it.

The bridge's key does nothing until the Jetson authorizes it, for Helm's relay only and over the tether only:
    cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service authorize-bridge --name <pc> --key "<the public key it prints>"
#>
param(
    [Parameter(Mandatory = $true)][string]$Package,
    [string]$Robot = '10.42.0.1',
    [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
$base = Join-Path $env:LOCALAPPDATA 'Minerva'
$taskName = 'Minerva-GELLO-Bridge'
New-Item -ItemType Directory -Force -Path $base, (Join-Path $base 'logs'), (Join-Path $base 'keys') | Out-Null

# The package: every file as its manifest lists it, nothing else.
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zip = [IO.Compression.ZipFile]::OpenRead((Resolve-Path -LiteralPath $Package))
try {
    $entry = $zip.GetEntry('gello-bridge.json')
    if (-not $entry) { throw 'Not a GELLO bridge package: no gello-bridge.json' }
    $reader = New-Object IO.StreamReader($entry.Open())
    $manifest = $reader.ReadToEnd() | ConvertFrom-Json
    $reader.Close()
} finally { $zip.Dispose() }
if ($manifest.kind -ne 'gello_bridge_package' -or $manifest.commit -notmatch '^[0-9a-f]{40}$') { throw 'Unexpected package manifest' }
$root = Join-Path $base ('gello-bridge-' + $manifest.commit.Substring(0, 10))
if (-not (Test-Path -LiteralPath $root)) {
    $partial = "$root.partial"
    if (Test-Path -LiteralPath $partial) { Remove-Item -Recurse -Force -LiteralPath $partial }
    [IO.Compression.ZipFile]::ExtractToDirectory((Resolve-Path -LiteralPath $Package), $partial)
    Rename-Item -LiteralPath $partial -NewName (Split-Path $root -Leaf)
}
$listed = @{}
foreach ($file in $manifest.files.PSObject.Properties) {
    $path = Join-Path $root $file.Name
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLower() -ne $file.Value) { throw "Package file differs: $($file.Name)" }
    $listed[[IO.Path]::GetFullPath($path)] = $true
}
foreach ($file in Get-ChildItem -LiteralPath $root -Recurse -File) {
    if ($file.Name -ne 'gello-bridge.json' -and $file.FullName -notmatch '\\__pycache__\\' -and -not $listed[[IO.Path]::GetFullPath($file.FullName)]) {
        throw "Unexpected file in the package folder: $($file.FullName)"
    }
}

# Python 3.12 and the hash-locked dependencies, one environment per lock.
if (-not $Python) {
    $local = Join-Path $base 'Python312\python.exe'
    if (Test-Path -LiteralPath $local) { $Python = $local } else { $Python = (Get-Command python -ErrorAction Stop).Source }
}
$version = & $Python -c "import sys; print('%d.%d' % sys.version_info[:2])"
if ($version -ne '3.12') { throw "The bridge's lock is for Python 3.12; $Python is $version" }
$lock = Join-Path $root 'requirements\gello-bridge.lock'
$lockId = (Get-FileHash -LiteralPath $lock -Algorithm SHA256).Hash.ToLower().Substring(0, 12)
$venv = Join-Path $base "bridge-env-$lockId"
if (-not (Test-Path -LiteralPath (Join-Path $venv '.complete'))) {
    if (Test-Path -LiteralPath $venv) { Remove-Item -Recurse -Force -LiteralPath $venv }
    & $Python -m venv $venv
    if ($LASTEXITCODE) { throw 'Creating the environment failed' }
    & "$venv\Scripts\python.exe" -m pip --disable-pip-version-check install --require-hashes --only-binary=:all: -r $lock
    if ($LASTEXITCODE) { throw 'Installing the dependencies failed' }
    & "$venv\Scripts\python.exe" -m pip check
    if ($LASTEXITCODE) { throw 'The dependencies do not fit together' }
    Set-Content -LiteralPath (Join-Path $venv '.complete') -Value $lockId
}

# The bridge's SSH key, once, inheriting no access from its folder.
$key = Join-Path $base 'keys\gello_bridge_ed25519'
if (-not (Test-Path -LiteralPath $key)) {
    $keygen = Join-Path $env:WINDIR 'System32\OpenSSH\ssh-keygen.exe'
    if (-not (Test-Path -LiteralPath $keygen)) { throw 'ssh-keygen not found: add the Windows OpenSSH Client feature' }
    # Through Python: Windows PowerShell cannot pass ssh-keygen an empty passphrase reliably.
    & "$venv\Scripts\python.exe" -c "import subprocess, sys; subprocess.run([sys.argv[1], '-q', '-t', 'ed25519', '-N', '', '-C', sys.argv[2], '-f', sys.argv[3]], check=True)" $keygen "minerva-gello-bridge@$env:COMPUTERNAME" $key
    if ($LASTEXITCODE) { throw 'Making the bridge key failed' }
}
$identity = [Security.Principal.WindowsIdentity]::GetCurrent().Name
# icacls, not Set-Acl: a new ACL object makes Set-Acl write the audit list too, which an ordinary user may not.
& icacls.exe $key /inheritance:r /grant:r "${identity}:(F)" | Out-Null
if ($LASTEXITCODE) { throw 'Restricting the bridge key failed' }

# The runner the task starts: the bridge, writing its own log (Windows PowerShell would turn each redirected stderr
# line into an error record), started again if it ever exits (it reconnects to Helm by itself; this covers a crash).
$runner = Join-Path $base 'Run-GelloBridge.ps1'
$log = Join-Path $base 'logs\gello-bridge.log'
@"
`$env:PYTHONPATH = '$root\src'
`$env:PYTHONUNBUFFERED = '1'
Set-Location -LiteralPath '$root'
while (`$true) {
    & '$venv\Scripts\python.exe' -m robot_platform.teleop.gello_bridge --key '$key' --robot '$Robot' --log '$log'
    Add-Content -LiteralPath '$log' -Encoding ascii -Value "`$(Get-Date -Format s) the bridge exited (`$LASTEXITCODE); starting it again in 5 s"
    Start-Sleep -Seconds 5
}
"@ | Set-Content -LiteralPath $runner -Encoding UTF8

# The logon task: started now and at each logon, restarted when it stops.
if (Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue) {
    Stop-ScheduledTask -TaskName $taskName
    Unregister-ScheduledTask -TaskName $taskName -Confirm:$false
}
# Each install starts a fresh log (the bridge writes UTF-8); the last one is kept beside it.
if (Test-Path -LiteralPath $log) { Move-Item -LiteralPath $log -Destination "$log.$(Get-Date -Format yyyyMMddTHHmmss)" }
$exe = Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe'
$action = New-ScheduledTaskAction -Execute $exe -Argument "-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$runner`"" -WorkingDirectory $base
$trigger = New-ScheduledTaskTrigger -AtLogOn -User $identity
$principal = New-ScheduledTaskPrincipal -UserId $identity -LogonType Interactive -RunLevel Limited
$settings = New-ScheduledTaskSettingsSet -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -RestartCount 999 -RestartInterval (New-TimeSpan -Minutes 1) -MultipleInstances IgnoreNew
Register-ScheduledTask -TaskName $taskName -Action $action -Trigger $trigger -Principal $principal -Settings $settings -Description 'The GELLO bridge for Helm on Minerva: the leader arms and their link over the tether' | Out-Null
Start-ScheduledTask -TaskName $taskName
Start-Sleep -Seconds 5

[pscustomobject]@{
    package     = $root
    commit      = $manifest.commit
    environment = $venv
    task        = (Get-ScheduledTask -TaskName $taskName).State.ToString()
    log         = $log
    public_key  = (Get-Content -LiteralPath "$key.pub" -Raw).Trim()
    log_tail    = (Get-Content -LiteralPath $log -Tail 5 -ErrorAction SilentlyContinue) -join "`n"
} | ConvertTo-Json
