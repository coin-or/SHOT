# Download the pinned native x64 solver packages used by Windows wheels.
param([string]$Destination = "")
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
$root = Split-Path -Parent $PSScriptRoot
if (!$Destination) { $Destination = Join-Path $root "build/wheel-deps" }
$manifest = Get-Content -LiteralPath (Join-Path $root "ThirdParty/versions.json") -Raw | ConvertFrom-Json
New-Item -ItemType Directory -Path $Destination -Force | Out-Null
foreach ($name in @("Cbc", "Ipopt")) {
    $solver = $manifest.external_solvers.$name
    $package = $solver.windows
    $archive = Join-Path $Destination $package.archive
    if (!(Test-Path -LiteralPath $archive)) {
        $url = "https://github.com/coin-or/$name/releases/download/$($solver.release)/$($package.archive)"
        Invoke-WebRequest -Uri $url -OutFile $archive
    }
    if ((Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $package.sha256) {
        throw "Checksum mismatch for $archive. Remove the archive and retry."
    }
    $prefix = Join-Path $Destination $name
    Expand-Archive -LiteralPath $archive -DestinationPath $prefix -Force
    $libraryRoot = Join-Path $prefix $package.directory
    foreach ($directory in @("include", "lib", "bin")) {
        if (!(Test-Path -LiteralPath (Join-Path $libraryRoot $directory))) {
            throw "Missing $directory in $libraryRoot"
        }
    }
    Write-Output "$name available at $libraryRoot"
}
