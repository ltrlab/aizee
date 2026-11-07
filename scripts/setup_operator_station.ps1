<#
.SYNOPSIS
Set up Minerva's operator station once, as Administrator: the PC with the GELLO arms, the pedals and the tether.

.DESCRIPTION
For Helm on Minerva (docs/architecture/HELM_ON_JETSON.md). With the tether plugged in, in PowerShell run as
Administrator:

    powershell -ExecutionPolicy Bypass -File setup_operator_station.ps1 -AuthorizeKey "ssh-ed25519 AAAA... name"

- OpenSSH Server, keys only, reachable from the LAN, the tether and Tailscale only. The given public key may log in.
- No sleep or hibernation on mains power; USB selective suspend off.
- The tether adapter without power saving (Energy-Efficient Ethernet, Green Ethernet, Idle Power Saving, Selective
  Suspend, Miscellaneous Transfer Settings), as on the old terminal: a power-saving tether delays leader replies past
  the robot's 4 ms deadline. Keep a monitor off the adapter's USB hub for the same reason.

It prints what setting up the bridge needs: the SSH host key's fingerprint, the user and the addresses.
#>
param([Parameter(Mandatory = $true)][string]$AuthorizeKey)
$ErrorActionPreference = 'Stop'
$principal = [Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { throw 'Run this in PowerShell as Administrator' }
$user = $principal.Identity.Name
if ($AuthorizeKey -notmatch '^(ssh-ed25519|ecdsa-sha2-nistp256) [A-Za-z0-9+/=]+( .*)?$') { throw 'Expected one public key line' }

# OpenSSH Server, started now and at boot.
if ((Get-WindowsCapability -Online -Name 'OpenSSH.Server~~~~0.0.1.0').State -ne 'Installed') {
    try {
        Add-WindowsCapability -Online -Name 'OpenSSH.Server~~~~0.0.1.0' | Out-Null
    } catch {
        throw "Windows could not add OpenSSH Server ($($_.Exception.Message)). If its component store is damaged, " +
              'repair it (DISM /Online /Cleanup-Image /RestoreHealth, then sfc /scannow) and run this again.'
    }
}
Set-Service sshd -StartupType Automatic
Start-Service sshd

# The key. This runs as an administrator, whose keys sshd reads from ProgramData (Administrators and SYSTEM only).
$file = Join-Path $env:ProgramData 'ssh\administrators_authorized_keys'
$blob = $AuthorizeKey.Split(' ')[1]
if (-not ((Test-Path $file) -and (Select-String -LiteralPath $file -SimpleMatch $blob -Quiet))) {
    Add-Content -LiteralPath $file -Value $AuthorizeKey -Encoding ascii
}
& icacls.exe $file /inheritance:r /grant '*S-1-5-32-544:F' /grant '*S-1-5-18:F' | Out-Null

# Keys only, then restart sshd to apply it.
$config = Join-Path $env:ProgramData 'ssh\sshd_config'
$text = Get-Content -Raw -LiteralPath $config
$setting = '(?m)^#?\s*PasswordAuthentication\b[^\r\n]*'
if ($text -notmatch '(?m)^PasswordAuthentication no') {
    if ($text -match $setting) { $text = $text -replace $setting, 'PasswordAuthentication no' }
    else { $text = "PasswordAuthentication no`r`n" + $text }  # The top is the global section, before any Match block.
    Set-Content -LiteralPath $config -Value $text -Encoding ascii -NoNewline
}
Restart-Service sshd

# SSH from the LAN, the tether and Tailscale only.
if (-not (Get-NetFirewallRule -Name 'OpenSSH-Server-In-TCP' -ErrorAction SilentlyContinue)) {
    New-NetFirewallRule -Name 'OpenSSH-Server-In-TCP' -DisplayName 'OpenSSH SSH Server (sshd)' -Direction Inbound -Protocol TCP -LocalPort 22 -Action Allow | Out-Null
}
Set-NetFirewallRule -Name 'OpenSSH-Server-In-TCP' -Enabled True -Profile Any -RemoteAddress LocalSubnet, '100.64.0.0/10'

# Never asleep on mains power, USB selective suspend off: the bridge and the leader arms must stay up.
& powercfg.exe /change standby-timeout-ac 0
& powercfg.exe /change hibernate-timeout-ac 0
& powercfg.exe /setacvalueindex SCHEME_CURRENT 2a737441-1930-4402-8d77-b2bebba308a3 48e6b7a6-50f5-4782-a5d4-53bb8f07e226 0
& powercfg.exe /setactive SCHEME_CURRENT

# The tether, without power saving: the adapter on the Jetson's network (10.42.0.x), and any USB Ethernet adapter.
$onTether = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.IPAddress -like '10.42.0.*' } | ForEach-Object InterfaceAlias)
$tether = @(Get-NetAdapter -Physical | Where-Object {
    $onTether -contains $_.Name -or ($_.InterfaceDescription -match 'USB' -and $_.InterfaceDescription -notmatch 'Wi-?Fi|Wireless|802\.11|Bluetooth') })
foreach ($adapter in $tether) {
    # Device Manager's "Allow the computer to turn off this device to save power" (read-only on the NetAdapter cmdlets).
    Get-CimInstance -Namespace root/WMI -ClassName MSPower_DeviceEnable |
        Where-Object { $_.InstanceName.StartsWith($adapter.PnPDeviceID, [StringComparison]::OrdinalIgnoreCase) } |
        Set-CimInstance -Property @{Enable = $false}
    Set-NetAdapterPowerManagement -Name $adapter.Name -SelectiveSuspend Disabled -NoRestart -ErrorAction SilentlyContinue
    foreach ($property in @(Get-NetAdapterAdvancedProperty -Name $adapter.Name -ErrorAction SilentlyContinue)) {
        if ($property.DisplayName -match 'Energy[- ]Efficient|Advanced EEE|Green Ethernet|Idle Power|Selective Suspend|Miscellaneous Transfer' -and $property.ValidDisplayValues -contains 'Disabled') {
            Set-NetAdapterAdvancedProperty -Name $adapter.Name -DisplayName $property.DisplayName -DisplayValue 'Disabled' -NoRestart
        }
    }
    Restart-NetAdapter -Name $adapter.Name
}
if (-not $tether) { Write-Warning 'No tether adapter found: plug the tether in and run this again' }

Start-Sleep -Seconds 8
[pscustomobject]@{
    host_key  = (& (Join-Path $env:WINDIR 'System32\OpenSSH\ssh-keygen.exe') -lf (Join-Path $env:ProgramData 'ssh\ssh_host_ed25519_key.pub'))
    user      = $user
    addresses = (Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notmatch '^(127|169\.254)\.' } | ForEach-Object { "$($_.InterfaceAlias)=$($_.IPAddress)" }) -join ', '
    tether    = ($tether | ForEach-Object { "$($_.Name): $($_.InterfaceDescription) ($($_.Status))" }) -join '; '
} | Format-List
