<#
.SYNOPSIS
Cross-compile the shared Rust services for JetPack 6 from Windows x64.
.DESCRIPTION
Tools and caches stay under rust/target/jetson-toolchain. Requires existing
Visual Studio C++ Build Tools. -Bootstrap downloads pinned tools if missing.
Builds locally; never connects to a robot or installs services.
#>
[CmdletBinding()]
param(
    [switch]$Bootstrap,
    [switch]$Offline,
    [ValidateSet('motor_control', 'lidar_control')]
    [string[]]$Package = @('motor_control', 'lidar_control')
)

$ErrorActionPreference = 'Stop'
if ([Environment]::OSVersion.Platform -ne 'Win32NT' -or ![Environment]::Is64BitProcess) {
    throw 'This build helper requires a 64-bit Windows PowerShell process.'
}
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$tools = Join-Path $repo 'rust/target/jetson-toolchain'
$rustVersion = '1.98.1'
$zigVersion = '0.14.1'
$zigbuildVersion = '0.23.3'
$cargo = Join-Path $tools 'cargo-home/bin/cargo.exe'
$rustup = Join-Path $tools 'cargo-home/bin/rustup.exe'
$zig = Join-Path $tools "zig-x86_64-windows-$zigVersion/zig.exe"
$zigbuildDir = Join-Path $tools 'cargo-zigbuild'

function Get-VerifiedDownload([string]$Url, [string]$Destination, [string]$Sha256) {
    if (!(Test-Path -LiteralPath $Destination)) {
        Write-Host "Downloading $Url"
        Invoke-WebRequest -UseBasicParsing -Uri $Url -OutFile $Destination
    }
    $actual = (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash
    if ($actual -ne $Sha256) { throw "Checksum mismatch: $Destination" }
}

$saved = @{}
$names = @('CARGO_HOME', 'RUSTUP_HOME', 'RUSTUP_TOOLCHAIN', 'PATH',
    'CARGO_TARGET_DIR', 'CARGO_ZIGBUILD_ZIG_PATH', 'CARGO_ZIGBUILD_CACHE_DIR',
    'ZIG_GLOBAL_CACHE_DIR')
foreach ($name in $names) { $saved[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
$oldLocation = Get-Location
try {
    Set-Location -LiteralPath $repo
    New-Item -ItemType Directory -Force -Path $tools | Out-Null
    $env:CARGO_HOME = Join-Path $tools 'cargo-home'
    $env:RUSTUP_HOME = Join-Path $tools 'rustup-home'
    $env:CARGO_TARGET_DIR = Join-Path $repo 'rust/target'
    $env:CARGO_ZIGBUILD_ZIG_PATH = $zig
    $env:CARGO_ZIGBUILD_CACHE_DIR = Join-Path $tools 'zigbuild-cache'
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $tools 'zig-cache'

    if ($Bootstrap) {
        if ($Offline) { throw '-Bootstrap and -Offline cannot be used together.' }
        if (!(Test-Path -LiteralPath $rustup)) {
            $installer = Join-Path $tools 'rustup-init.exe'
            # rustup verifies downloaded Rust component hashes.
            Invoke-WebRequest -UseBasicParsing -Uri 'https://win.rustup.rs/x86_64' -OutFile $installer
            & $installer -y --no-modify-path --profile minimal --default-host x86_64-pc-windows-msvc --default-toolchain $rustVersion --target aarch64-unknown-linux-gnu
            if ($LASTEXITCODE -ne 0) { throw 'Rust installation failed.' }
        }
        if (!(Test-Path -LiteralPath $zig)) {
            $archive = Join-Path $tools "zig-$zigVersion.zip"
            Get-VerifiedDownload "https://ziglang.org/download/$zigVersion/zig-x86_64-windows-$zigVersion.zip" $archive '554f5378228923ffd558eac35e21af020c73789d87afeabf4bfd16f2e6feed2c'
            Expand-Archive -LiteralPath $archive -DestinationPath $tools -Force
        }
        if (!(Get-ChildItem -LiteralPath $zigbuildDir -Recurse -Filter cargo-zigbuild.exe -ErrorAction SilentlyContinue)) {
            $archive = Join-Path $tools "cargo-zigbuild-$zigbuildVersion.zip"
            Get-VerifiedDownload "https://github.com/rust-cross/cargo-zigbuild/releases/download/v$zigbuildVersion/cargo-zigbuild-x86_64-pc-windows-msvc.zip" $archive '7c1edab4815fa6542f1899c0fd8921c3995b2b109f65dd96e81247036f3a5970'
            Expand-Archive -LiteralPath $archive -DestinationPath $zigbuildDir -Force
        }
    }

    if (!(Test-Path -LiteralPath $cargo) -or !(Test-Path -LiteralPath $zig)) {
        throw 'Local tools are missing. Run scripts/build_jetson.ps1 -Bootstrap.'
    }
    $zigbuild = Get-ChildItem -LiteralPath $zigbuildDir -Recurse -Filter cargo-zigbuild.exe | Select-Object -First 1
    if (!$zigbuild) { throw 'cargo-zigbuild is missing. Run with -Bootstrap.' }

    # Reuse the already installed stable toolchain only when its version matches.
    $toolchain = "$rustVersion-x86_64-pc-windows-msvc"
    $stableCompiler = Join-Path $env:RUSTUP_HOME 'toolchains/stable-x86_64-pc-windows-msvc/bin/rustc.exe'
    if (!(Test-Path (Join-Path $env:RUSTUP_HOME "toolchains/$toolchain/bin/rustc.exe"))) {
        if ((Test-Path $stableCompiler) -and ((& $stableCompiler --version) -match "^rustc $([regex]::Escape($rustVersion)) ")) {
            $toolchain = 'stable-x86_64-pc-windows-msvc'
        } elseif ($Bootstrap) {
            & $rustup toolchain install $rustVersion --profile minimal --target aarch64-unknown-linux-gnu
            if ($LASTEXITCODE -ne 0) { throw 'Pinned Rust toolchain installation failed.' }
        } else {
            throw "Rust $rustVersion is required. Run with -Bootstrap."
        }
    }
    $env:RUSTUP_TOOLCHAIN = $toolchain
    $env:PATH = "$env:CARGO_HOME/bin;$($zigbuild.DirectoryName);$env:PATH"
    if ($Bootstrap) {
        & $rustup target add aarch64-unknown-linux-gnu --toolchain $toolchain
        if ($LASTEXITCODE -ne 0) { throw 'Rust target installation failed.' }
    }

    & $cargo --version
    & $zig version
    & $zigbuild.FullName --version
    if (!(Test-Path -LiteralPath (Join-Path $repo 'Cargo.lock'))) {
        throw 'Cargo.lock is missing; restore the shared workspace lockfile before building.'
    }
    $arguments = @('zigbuild', '--release', '--locked', '--target', 'aarch64-unknown-linux-gnu.2.35')
    foreach ($item in $Package) { $arguments += @('-p', $item) }
    if ($Offline) { $arguments += '--offline' }
    & $cargo @arguments
    if ($LASTEXITCODE -ne 0) { throw "Jetson cross-build failed (exit $LASTEXITCODE)." }

    foreach ($item in $Package) {
        $binary = Join-Path $env:CARGO_TARGET_DIR "aarch64-unknown-linux-gnu/release/$item"
        $bytes = [IO.File]::ReadAllBytes($binary)
        if ($bytes.Length -lt 64 -or $bytes[0] -ne 127 -or [Text.Encoding]::ASCII.GetString($bytes, 1, 3) -ne 'ELF' -or $bytes[4] -ne 2 -or $bytes[5] -ne 1 -or [BitConverter]::ToUInt16($bytes, 18) -ne 183) {
            throw "Build output is not a 64-bit little-endian AArch64 ELF: $binary"
        }
        Get-FileHash -LiteralPath $binary -Algorithm SHA256
    }
} finally {
    Set-Location -LiteralPath $oldLocation
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name, $saved[$name], 'Process') }
}
