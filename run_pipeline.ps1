param(
    [Parameter(Mandatory=$true)] [string]$SplineNLElem,
    [Parameter(Mandatory=$true)] [string]$TangentNLElem,
    [Parameter(Mandatory=$true)] [string]$Mesh3ds,
    [Parameter(Mandatory=$true)] [string]$OutDir,
    [string]$PythonExe = "python",
    [string]$BlenderExe = "",
    [int]$SamplesPerSegment = 20,
    [string]$AxisMapping = "nl2_to_ue",
    [double]$InitialSpeed = 6.0
)

$ErrorActionPreference = "Stop"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$bundlePath = Join-Path $OutDir "coaster_ue5_bundle.json"
$csvPath = Join-Path $OutDir "coaster_timeline.csv"

& $PythonExe "$PSScriptRoot\convert_nlelem_to_ue.py" `
    --spline "$SplineNLElem" `
    --tangent "$TangentNLElem" `
    --mesh "$Mesh3ds" `
    --output "$bundlePath" `
    --csv "$csvPath" `
    --samples-per-segment $SamplesPerSegment `
    --axis-mapping $AxisMapping `
    --initial-speed $InitialSpeed

if($BlenderExe -and (Test-Path $BlenderExe)) {
    $fbxPath = Join-Path $OutDir "CoasterModel.fbx"
    & $BlenderExe -b -P "$PSScriptRoot\blender_3ds_to_fbx.py" -- --in "$Mesh3ds" --out "$fbxPath"
    Write-Output "FBX created: $fbxPath"
} else {
    Write-Output "Blender path not provided; skipped 3DS->FBX conversion."
}

Write-Output "Bundle: $bundlePath"
Write-Output "Timeline CSV: $csvPath"
