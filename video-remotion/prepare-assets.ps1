param(
    [string]$GazeboInput = "..\\videos\\gazebo_gui_final_20260820.mp4",
    [string]$AirportInput = "..\\data\\demo_inputs\\airport_tracked.mp4",
    [string]$ParkingInput = "..\\data\\demo_inputs\\parking_tracked.mp4"
)

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$mediaRoot = Join-Path $projectRoot "public\\media"

New-Item -ItemType Directory -Path $mediaRoot -Force | Out-Null

$sources = @{
    "gazebo_gui_final_20260820.mp4" = $GazeboInput
    "airport_tracked.mp4" = $AirportInput
    "parking_tracked.mp4" = $ParkingInput
}

foreach ($entry in $sources.GetEnumerator()) {
    $sourcePath = Join-Path $projectRoot $entry.Value
    if (-not (Test-Path -LiteralPath $sourcePath)) {
        throw "Missing source video: $sourcePath"
    }

    Copy-Item -LiteralPath $sourcePath -Destination (Join-Path $mediaRoot $entry.Key) -Force
}

Write-Host "Prepared Remotion media in $mediaRoot"
