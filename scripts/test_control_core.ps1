<# Build/test the portable controller core with the repository-local Rust toolchain. #>
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$toolRoot = Join-Path $repo 'rust/target/jetson-toolchain'
$saved = @{}
foreach ($key in @('CARGO_HOME', 'RUSTUP_HOME', 'PATH')) { $saved[$key] = [Environment]::GetEnvironmentVariable($key, 'Process') }
$previous = Get-Location
try {
    Set-Location -LiteralPath $repo
    $env:CARGO_HOME = Join-Path $toolRoot 'cargo-home'
    $env:RUSTUP_HOME = Join-Path $toolRoot 'rustup-home'
    $env:PATH = "$(Join-Path $env:CARGO_HOME 'bin');$env:PATH"
    & cargo test -p robot_control --locked --offline
    if ($LASTEXITCODE -ne 0) { throw 'Controller core tests failed' }
    & cargo build -p robot_control --bin control_harness --locked --offline
    if ($LASTEXITCODE -ne 0) { throw 'Controller harness build failed' }
} finally {
    Set-Location -LiteralPath $previous
    foreach ($key in $saved.Keys) { [Environment]::SetEnvironmentVariable($key, $saved[$key], 'Process') }
}
