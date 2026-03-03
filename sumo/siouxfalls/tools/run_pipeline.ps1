param(
  [double]$Scale = 0.1,
  [int]$Begin = 0,
  [int]$End = 3600
)

$ErrorActionPreference = "Stop"

# repo root = parent of tools
$ROOT = Split-Path -Parent $PSScriptRoot
Set-Location $ROOT

$TNTP  = Join-Path $ROOT "tntp"
$NETDIR = Join-Path $ROOT "net"
$DEMDIR = Join-Path $ROOT "demand"
$WORK  = Join-Path $ROOT "work"

New-Item -ItemType Directory -Force $NETDIR, $DEMDIR, $WORK | Out-Null

# 1) Import TNTP -> nodes/edges (work/)
Set-Location $WORK
python "$env:SUMO_HOME\tools\import\transportationTestProblems.py" (Join-Path $TNTP "SiouxFalls_node.tntp") (Join-Path $TNTP "SiouxFalls_net.tntp")

# 2) Build projected net (UTM zone 14N)
& "$env:SUMO_HOME\bin\netconvert.exe" --node-files ".\nodes.nod.xml" --edge-files ".\edges.edg.xml" `
  --proj "+proj=utm +zone=14 +datum=WGS84 +units=m +no_defs" `
  -o (Join-Path $NETDIR "siouxfalls_utm_clean.net.xml")

# 3) Add traffic lights
& "$env:SUMO_HOME\bin\netconvert.exe" -s (Join-Path $NETDIR "siouxfalls_utm_clean.net.xml") --tls.guess `
  -o (Join-Path $NETDIR "siouxfalls_utm_tls_fixed.net.xml")

# 4) Create TAZ + tazRel (repo root outputs)
Set-Location $ROOT
python (Join-Path $ROOT "tools\make_sumo_od_inputs.py") --net (Join-Path $NETDIR "siouxfalls_utm_tls_fixed.net.xml") --tntp (Join-Path $TNTP "SiouxFalls_trips.tntp") --begin $Begin --end $End

# 5) OD -> trips
$Trips = Join-Path $DEMDIR "siouxfalls.trips.xml"
od2trips -n (Join-Path $ROOT "siouxfalls.taz.xml") -z (Join-Path $ROOT "siouxfalls.tazRel.xml") `
  -o $Trips --begin $Begin --end $End --spread.uniform --scale $Scale

# 6) Trips -> routes
$Rou = Join-Path $DEMDIR ("siouxfalls_{0}p_utm.rou.xml" -f [int]($Scale*100))
$VType = Join-Path $DEMDIR "vtypes.add.xml"

if (Test-Path $VType) {
  duarouter -n (Join-Path $NETDIR "siouxfalls_utm_tls_fixed.net.xml") -r $Trips -a $VType -o $Rou
} else {
  duarouter -n (Join-Path $NETDIR "siouxfalls_utm_tls_fixed.net.xml") -r $Trips -o $Rou
}

Write-Host "DONE"
Write-Host "  Net:  " (Join-Path $NETDIR "siouxfalls_utm_tls_fixed.net.xml")
Write-Host "  Rou:  " $Rou