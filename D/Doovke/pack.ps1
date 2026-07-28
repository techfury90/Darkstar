# Doovke rigid-pack snapshot / restore.
#
# WHY THIS EXISTS: formatting a Micropolis1325 is 122,880 sectors and takes real time, and
# the install workflow requires a power cycle between format and install -- so every install
# attempt used to start with another full format.  Format once, snapshot, then restore before
# each attempt.
#
# A pack is THREE files and all three must move together:
#   <name>            raw 62,914,560-byte image
#   <name>.labels     10-word Pilot label per sector
#   <name>.formatted  one bit per sector: is it formatted?
# The sidecar is not optional.  A formatted-but-empty sector is all zeros on the platter and
# so is an unformatted one; without the bitmap a freshly formatted pack reloads as blank and
# the format is silently thrown away.
#
# Usage (Doovke must be CLOSED -- it holds the pack in memory and autosaves over it):
#   .\pack.ps1 save    baseline      # snapshot the live pack
#   .\pack.ps1 restore baseline      # put it back, ready for another install run
#   .\pack.ps1 list

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][ValidateSet('save', 'restore', 'list')][string]$Action,
    [string]$Name,
    [string]$Pack,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
$suffixes = @('', '.labels', '.formatted')

# Locate the pack HERE, not as a parameter default: $PSScriptRoot is not reliably
# populated at parameter-bind time under `powershell -File`, where it collapsed to
# an empty string and produced 'C:\..\..\..\dove_rigid.img'.
if (-not $Pack) {
    $here = $PSScriptRoot
    if (-not $here) { $here = Split-Path -Parent $MyInvocation.MyCommand.Path }
    $Pack = Join-Path $here '..\..\..\..\..\dove_rigid.img'
}
# GetFullPath collapses the .. chain without requiring the target to exist.
# Resolve-Path and Split-Path both fail on a relative path that walks above the root.
$Pack = [System.IO.Path]::GetFullPath($Pack)
$packDir = Split-Path -Parent $Pack
if (-not (Test-Path $packDir)) { throw "Pack directory not found: $packDir" }
$store = Join-Path $packDir 'packs'

if ($Action -eq 'list') {
    if (-not (Test-Path $store)) { Write-Host "No snapshots yet ($store)"; exit 0 }
    Get-ChildItem $store -Filter *.img |
        Select-Object @{n = 'Snapshot'; e = { $_.BaseName } },
                      @{n = 'Taken'; e = { $_.LastWriteTime } },
                      @{n = 'MB'; e = { [int]($_.Length / 1MB) } } |
        Format-Table -AutoSize
    exit 0
}

if (-not $Name) { throw "-Name is required for '$Action' (e.g. .\pack.ps1 $Action baseline)" }

# Doovke autosaves every two minutes; copying underneath it yields a torn image whose
# damage looks exactly like an emulator bug.  Refuse rather than produce a bad snapshot.
$running = Get-Process -Name Doovke -ErrorAction SilentlyContinue
if ($running -and -not $Force) {
    throw "Doovke is running (PID $($running.Id -join ',')). Close it first, or pass -Force if you are certain it is idle and saved."
}

function Copy-Set($fromBase, $toBase) {
    # Copy to .part first and rename, so an interrupted run cannot leave a half-written
    # member that later reads as a valid pack.
    $staged = @()
    foreach ($s in $suffixes) {
        $src = "$fromBase$s"
        if (-not (Test-Path $src)) { throw "Missing pack member: $src" }
        $tmp = "$toBase$s.part"
        Copy-Item $src $tmp -Force
        $staged += , @($tmp, "$toBase$s")
    }
    foreach ($pair in $staged) { Move-Item $pair[0] $pair[1] -Force }
}

switch ($Action) {
    'save' {
        if (-not (Test-Path $store)) { New-Item -ItemType Directory $store | Out-Null }
        $dest = Join-Path $store "$Name.img"
        if ((Test-Path $dest) -and -not $Force) { throw "Snapshot '$Name' exists. Pass -Force to overwrite." }
        Copy-Set $Pack $dest
        Write-Host "Saved snapshot '$Name' <- $Pack"
    }
    'restore' {
        $src = Join-Path $store "$Name.img"
        if (-not (Test-Path $src)) { throw "No snapshot '$Name' in $store" }
        Copy-Set $src $Pack
        Write-Host "Restored '$Name' -> $Pack"
    }
}
