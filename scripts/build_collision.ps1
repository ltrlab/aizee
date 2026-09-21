<# Build the native collision library using the existing local Rust toolchain. #>
[CmdletBinding()]
param([switch]$Test)
$ErrorActionPreference='Stop'
$repo=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$toolRoot=Join-Path $repo 'rust/target/jetson-toolchain'
$saved=@{}
foreach ($key in @('CARGO_HOME','RUSTUP_HOME','PATH')) { $saved[$key]=[Environment]::GetEnvironmentVariable($key,'Process') }
$previous=Get-Location
try {
    Set-Location -LiteralPath $repo
    $env:CARGO_HOME=Join-Path $toolRoot 'cargo-home'
    $env:RUSTUP_HOME=Join-Path $toolRoot 'rustup-home'
    $env:PATH="$(Join-Path $env:CARGO_HOME 'bin');$env:PATH"
    if ($Test) {
        & cargo test -p robot_collision --locked --offline
        if ($LASTEXITCODE -ne 0) { throw 'Collision tests failed' }
    }
    & cargo build -p robot_collision --release --locked --offline
    if ($LASTEXITCODE -ne 0) { throw 'Collision build failed' }
} finally {
    Set-Location -LiteralPath $previous
    foreach ($key in $saved.Keys) { [Environment]::SetEnvironmentVariable($key,$saved[$key],'Process') }
}
