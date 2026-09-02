param(
    [Parameter(Mandatory = $false)]
    [string]$ArchivePath = 'C:\Users\CAPTAIN·YAN\Downloads\filament-main.zip',

    [Parameter(Mandatory = $false)]
    [string]$DestinationRoot = (Join-Path $PSScriptRoot '..\.deps')
)

$ErrorActionPreference = 'Stop'

$resolvedArchive = (Resolve-Path -LiteralPath $ArchivePath).Path
$resolvedWorkspace = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$resolvedDestinationRoot = [System.IO.Path]::GetFullPath($DestinationRoot)

if (-not $resolvedDestinationRoot.StartsWith($resolvedWorkspace, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Destination must stay inside the workspace: $resolvedDestinationRoot"
}

$filamentRoot = Join-Path $resolvedDestinationRoot 'filament-main'
$readyMarker = Join-Path $filamentRoot '.volume-surface-ready'
if ((Test-Path -LiteralPath (Join-Path $filamentRoot 'CMakeLists.txt')) -and
    (Test-Path -LiteralPath $readyMarker)) {
    Write-Host "Filament source is already prepared: $filamentRoot"
    exit 0
}

if (Test-Path -LiteralPath $filamentRoot) {
    throw "Filament destination exists without a completion marker: $filamentRoot"
}

New-Item -ItemType Directory -Force -Path $resolvedDestinationRoot | Out-Null
& tar.exe -xf $resolvedArchive `
    --exclude='filament-main/GEMINI.md' `
    --exclude='filament-main/third_party/dawn/src/cmake/HermeticXcode/ranlib' `
    --exclude='filament-main/third_party/zstd/tests/cli-tests/bin/unzstd' `
    --exclude='filament-main/third_party/zstd/tests/cli-tests/bin/zstdcat' `
    -C $resolvedDestinationRoot
if ($LASTEXITCODE -ne 0) {
    throw "tar.exe failed with exit code $LASTEXITCODE"
}

if (-not (Test-Path -LiteralPath (Join-Path $filamentRoot 'CMakeLists.txt'))) {
    throw "The archive did not contain filament-main/CMakeLists.txt"
}

Set-Content -LiteralPath $readyMarker -Value $resolvedArchive -Encoding UTF8
Write-Host "Prepared Filament source: $filamentRoot"
