# The splat trainer's Linux wheels (train_head_scene.py), downloaded on the operator station's Windows side: its Ubuntu
# (WSL2) has no network while WSL runs bridged (the station's .wslconfig, 2026-10-31), so install_wsl.sh installs from
# this folder. CPython 3.10, x86-64 Linux, binary wheels only: CUDA PyTorch 2.4.1 (cu124), gsplat's prebuilt wheel
# for it, and torch's Linux-only dependencies (the CUDA libraries, triton), which pip skips on Windows because it
# evaluates their platform_system == "Linux" markers on the machine that downloads.
#
#     powershell -File download_wheels.ps1        (about 2.7 GB into %USERPROFILE%\minerva-splat\wheels)
$ErrorActionPreference = 'Stop'
$python = "$env:LOCALAPPDATA\Minerva\Python312\python.exe"
$dest = "$env:USERPROFILE\minerva-splat\wheels"
New-Item -ItemType Directory -Force $dest | Out-Null
$target = @('--platform', 'linux_x86_64', '--platform', 'manylinux1_x86_64', '--platform', 'manylinux2014_x86_64',
            '--platform', 'manylinux_2_17_x86_64', '--platform', 'manylinux_2_28_x86_64',
            '--python-version', '3.10', '--implementation', 'cp', '--abi', 'cp310', '--abi', 'abi3', '--abi', 'none',
            '--only-binary=:all:', '--dest', $dest, '--quiet', '--disable-pip-version-check')
"== torch"
& $python -m pip download @target 'torch==2.4.1' --index-url https://download.pytorch.org/whl/cu124 --extra-index-url https://pypi.org/simple
if ($LASTEXITCODE) { throw "torch download failed: $LASTEXITCODE" }
"== gsplat"
& $python -m pip download @target gsplat --index-url https://docs.gsplat.studio/whl/pt24cu124 --extra-index-url https://pypi.org/simple
if ($LASTEXITCODE) { throw "gsplat download failed: $LASTEXITCODE" }
"== the rest"
& $python -m pip download @target 'numpy<2' pillow plyfile tqdm pip wheel setuptools
if ($LASTEXITCODE) { throw "download failed: $LASTEXITCODE" }
"== torch's Linux-only dependencies"
Remove-Item "$dest\torch-2.1[0-9]*.whl", "$dest\numpy-2.*.whl" -ErrorAction SilentlyContinue  # gsplat's loose pins.
Add-Type -AssemblyName System.IO.Compression.FileSystem
$wheel = Get-ChildItem $dest -Filter 'torch-2.4.1+cu124-*.whl' | Select-Object -First 1
$zip = [System.IO.Compression.ZipFile]::OpenRead($wheel.FullName)
try {
  $entry = $zip.Entries | Where-Object { $_.FullName -like '*.dist-info/METADATA' } | Select-Object -First 1
  $reader = New-Object System.IO.StreamReader($entry.Open())
  $metadata = $reader.ReadToEnd()
  $reader.Close()
} finally { $zip.Dispose() }
$linux = $metadata -split "`n" | Where-Object { $_ -like 'Requires-Dist:*' -and $_ -match 'platform_system == "Linux"' } |
  ForEach-Object { ($_ -replace '^Requires-Dist:\s*', '' -split ';')[0].Trim() -replace '\s+', '' }
$linuxTarget = @('--platform', 'manylinux1_x86_64', '--platform', 'manylinux2014_x86_64', '--platform', 'manylinux_2_17_x86_64',
            '--platform', 'manylinux_2_28_x86_64', '--platform', 'linux_x86_64', '--python-version', '3.10',
            '--implementation', 'cp', '--abi', 'cp310', '--abi', 'abi3', '--abi', 'none', '--only-binary=:all:',
            '--no-deps', '--dest', $dest, '--quiet', '--disable-pip-version-check')
& $python -m pip download @linuxTarget @linux --index-url https://download.pytorch.org/whl/cu124 --extra-index-url https://pypi.org/simple
if ($LASTEXITCODE) { throw "download failed: $LASTEXITCODE" }
$files = Get-ChildItem $dest -Filter *.whl
"{0} wheels, {1:N2} GB" -f $files.Count, (($files | Measure-Object Length -Sum).Sum / 1GB)
$files | Where-Object { $_.Name -match '^(nvidia|triton)' } | ForEach-Object { "{0,8:N0} MB  {1}" -f ($_.Length / 1MB), $_.Name }
