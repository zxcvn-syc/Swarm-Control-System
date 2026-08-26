param(
    [int]$Duration = 55,
    [int]$TailSeconds = 18,
    [string]$Scenario = "rain_wind_3ddisplay",
    [string]$VmHost = "192.168.88.135",
    [string]$VmUser = "hhh",
    [string]$SshKey = "$HOME\.ssh\codex_cvtrack_vm_20260819",
    [string]$DemoRoot = "/home/hhh/Downloads/cvtrack-rfly-enhanced-20260821",
    [string]$OutputRoot = "",
    [string]$RflySdkRoot = "F:\RflySimAPIs\RflySimSDK",
    [int]$Ue4BridgePort = 30010,
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$scriptRoot = Join-Path $repoRoot "examples\rfly_ros2\scripts"
if ([string]::IsNullOrWhiteSpace($OutputRoot)) {
    $OutputRoot = Join-Path $repoRoot ("outputs\rfly_full_demo_{0}" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Force -Path $OutputRoot | Out-Null

$config = Join-Path $scriptRoot "Config.json"
$weights = Join-Path (Split-Path -Parent (Split-Path -Parent $repoRoot)) "yolov8s.pt"
if (-not (Test-Path $weights)) {
    $weights = Join-Path (Get-Location) "yolov8s.pt"
}
if (-not (Test-Path $weights)) {
    throw "YOLO weights not found; pass a local yolov8s.pt at the workspace root."
}

$runId = "rfly_{0}" -f (Get-Date -Format "yyyyMMdd_HHmmss")
$remoteLog = "$DemoRoot/logs/$runId"
$sshTarget = "$VmUser@$VmHost"
$liveDuration = $Duration + $TailSeconds
$remoteRunSeconds = $liveDuration + 20
$bridgeProcess = $null
$bridgeScript = Join-Path $repoRoot "tools\rfly_ue4_udp_bridge.py"
$bridgeListener = Get-NetUDPEndpoint -LocalPort $Ue4BridgePort -ErrorAction SilentlyContinue | Select-Object -First 1
if ($null -eq $bridgeListener) {
    $bridgeLog = Join-Path $OutputRoot "ue4_udp_bridge.log"
    $bridgeArgs = "`"$bridgeScript`" --listen-host 0.0.0.0 --listen-port $Ue4BridgePort --target-host 127.0.0.1 --target-port 20010"
    $bridgeProcess = Start-Process -FilePath $Python -ArgumentList $bridgeArgs -RedirectStandardOutput $bridgeLog -RedirectStandardError "$bridgeLog.err" -PassThru -WindowStyle Hidden
    $bridgeDeadline = (Get-Date).AddSeconds(5)
    do {
        Start-Sleep -Milliseconds 150
        $bridgeListener = Get-NetUDPEndpoint -LocalPort $Ue4BridgePort -ErrorAction SilentlyContinue | Select-Object -First 1
    } while ($null -eq $bridgeListener -and (Get-Date) -lt $bridgeDeadline)
    if ($null -eq $bridgeListener) {
        throw "UE4 UDP bridge did not bind port $Ue4BridgePort. See $bridgeLog.err."
    }
}
$remoteCommand = @"
source /opt/ros/humble/setup.bash
source /home/hhh/Downloads/Swarm-Control-System/ros2_ws/install_validation/setup.bash
export RFLY_DEMO_ROOT='$DemoRoot'
export ROS2_SETUP=/home/hhh/Downloads/Swarm-Control-System/ros2_ws/install_validation/setup.bash
export RFLY_LOG_ROOT='$remoteLog'
export RFLY_SDK_ROOT='$DemoRoot/rfly_sdk'
export RFLY_HOST_IP=192.168.88.1
export RFLY_UE4_BRIDGE_HOST=192.168.88.1
export RFLY_UE4_BRIDGE_PORT=$Ue4BridgePort
export RFLY_STATUS_BRIDGE_HOST=192.168.88.1
export RFLY_STATUS_BRIDGE_PORT=35671
export RFLY_VISION_PORT=35687
export RFLY_RUN_ID='$runId'
export RFLY_EVIDENCE_DURATION=$remoteRunSeconds
exec '$DemoRoot/scripts/run_ros_chain.sh' $remoteRunSeconds '$Scenario'
"@
$remoteCommand = ($remoteCommand -replace "`r?`n", "; ")

$sshArgs = @(
    "-o", "BatchMode=yes",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "ConnectTimeout=5",
    "-o", "ServerAliveInterval=4",
    "-o", "ServerAliveCountMax=2",
    "-i", $SshKey,
    $sshTarget
)
$remoteSceneFile = Join-Path $scriptRoot "rfly_ros_scene.py"
& scp -q -i $SshKey $remoteSceneFile "$sshTarget`:$DemoRoot/scripts/"
if ($LASTEXITCODE -ne 0) {
    throw "Could not sync $remoteSceneFile to $DemoRoot/scripts/."
}
$remoteScenarioFile = Join-Path $scriptRoot "scenario_presets.json"
& scp -q -i $SshKey $remoteScenarioFile "$sshTarget`:$DemoRoot/scripts/"
if ($LASTEXITCODE -ne 0) {
    throw "Could not sync $remoteScenarioFile to $DemoRoot/scripts/."
}
$remoteChainFile = Join-Path $scriptRoot "run_ros_chain.sh"
& scp -q -i $SshKey $remoteChainFile "$sshTarget`:$DemoRoot/scripts/"
if ($LASTEXITCODE -ne 0) {
    throw "Could not sync $remoteChainFile to $DemoRoot/scripts/."
}
& ssh @sshArgs "find '$DemoRoot/scripts/__pycache__' -maxdepth 1 -type f -name 'rfly_ros_scene.*.pyc' -delete"
if ($LASTEXITCODE -ne 0) {
    throw "Could not clear the remote Rfly scene bytecode cache."
}
$calibrationFile = if ($Scenario -like "*_3ddisplay") {
    Join-Path $scriptRoot "rfly_world_sensor_calibration_3ddisplay.json"
} else {
    Join-Path $scriptRoot "rfly_world_sensor_calibration.json"
}
if (-not (Test-Path $calibrationFile)) {
    throw "No camera-ground calibration is available for scenario $Scenario."
}
& scp -q -i $SshKey $calibrationFile "$sshTarget`:$DemoRoot/scripts/rfly_world_sensor_calibration.json"
if ($LASTEXITCODE -ne 0) {
    throw "Could not sync calibration $calibrationFile to $DemoRoot/scripts/."
}
$sshProcess = Start-Process -FilePath "ssh" -ArgumentList ($sshArgs + $remoteCommand) -RedirectStandardOutput (Join-Path $OutputRoot "ros_chain_launcher.log") -RedirectStandardError (Join-Path $OutputRoot "ros_chain_launcher.err") -PassThru -WindowStyle Hidden
try {
    Start-Sleep -Seconds 4
    $sceneReady = $false
    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        $probe = & ssh @sshArgs "if test -s '$remoteLog/scene_telemetry.jsonl'; then printf ready; fi" 2>$null
        if (($probe -join "") -match "ready") {
            $sceneReady = $true
            break
        }
        Start-Sleep -Milliseconds 500
    }
    if (-not $sceneReady) {
        throw "ROS scene did not produce telemetry within 14 seconds. See $OutputRoot\ros_chain_launcher.log."
    }

    $video = Join-Path $OutputRoot "uav_live.mp4"
    $tracks = Join-Path $OutputRoot "tracks.csv"
    $summary = Join-Path $OutputRoot "detection_summary.json"
    $liveArgs = @(
        (Join-Path $scriptRoot "rfly_live_cvtrack.py"),
        "--duration", $liveDuration,
        "--config", $config,
        "--weights", $weights,
        "--scenario", $Scenario,
        "--udp-host", $VmHost,
        "--udp-port", "35687",
        "--status-udp-port", "35671",
        "--output", $video,
        "--csv", $tracks,
        "--summary", $summary,
        "--output-fps", "30"
    )
    & $Python @liveArgs 2>&1 | Tee-Object -FilePath (Join-Path $OutputRoot "live.log")
    if ($LASTEXITCODE -ne 0) {
        throw "Live capture failed with exit code $LASTEXITCODE."
    }

    $remoteTimeoutSeconds = $remoteRunSeconds + 60
    if (-not $sshProcess.WaitForExit([int]($remoteTimeoutSeconds * 1000))) {
        throw "Remote ROS chain did not finish before evidence deadline."
    }
    $sshProcess.WaitForExit()
    $sshProcess.Refresh()
    $remoteExitCode = $sshProcess.ExitCode
    if ($null -eq $remoteExitCode) {
        $completionProbe = & ssh @sshArgs "test -s '$remoteLog/run_complete.json' && cat '$remoteLog/run_complete.json'" 2>$null
        if (($completionProbe -join "") -match '"status"\s*:\s*"ok"') {
            $remoteExitCode = 0
        } else {
            throw "Remote ROS chain exited but did not report an exit code or completion marker. See $OutputRoot\\ros_chain_launcher.err."
        }
    }
    if ($remoteExitCode -ne 0) {
        throw "Remote ROS chain failed with exit code $remoteExitCode. See $OutputRoot\\ros_chain_launcher.err."
    }

    $remoteTelemetry = Join-Path $OutputRoot "scene_telemetry.jsonl"
    function Copy-RemoteEvidenceFile {
        param([string]$RemoteName, [string]$Destination)
        & scp -q -i $SshKey "$sshTarget`:$remoteLog/$RemoteName" $Destination
        if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $Destination -PathType Leaf)) {
            throw "Could not retrieve required ROS evidence: $RemoteName"
        }
    }
    Copy-RemoteEvidenceFile "scene_telemetry.jsonl" $remoteTelemetry
    Copy-RemoteEvidenceFile "run_complete.json" (Join-Path $OutputRoot "run_complete.json")
    Copy-RemoteEvidenceFile "capture_summary.json" (Join-Path $OutputRoot "capture_summary.json")
    Copy-RemoteEvidenceFile "evidence_manifest.json" (Join-Path $OutputRoot "evidence_manifest.json")
    $evidenceFiles = @(
        "task_assignment.yaml",
        "planned_path.yaml",
        "enclosure_command.yaml",
        "target_track_world.yaml",
        "target_track_truth.yaml",
        "drone_states.yaml",
        "ground_vehicle_states.yaml"
    )
    foreach ($evidenceFile in $evidenceFiles) {
        Copy-RemoteEvidenceFile $evidenceFile (Join-Path $OutputRoot $evidenceFile)
    }
    $decisionVideo = Join-Path $OutputRoot "decision_god_view.mp4"
    $decisionSummary = Join-Path $OutputRoot "decision_god_view.json"
    $detectionData = Get-Content -LiteralPath $summary -Raw | ConvertFrom-Json
    $videoOcclusionStart = $null
    if ($detectionData.physical_occlusion_windows -and $detectionData.physical_occlusion_windows.Count -gt 0) {
        $videoOcclusionStart = [double]$detectionData.physical_occlusion_windows[0].start_s
    }
    $telemetryOcclusionStart = $null
    foreach ($line in Get-Content -LiteralPath $remoteTelemetry) {
        try {
            $record = $line | ConvertFrom-Json
        } catch {
            continue
        }
        if ($record.physical_occlusion_engaged -eq $true -and $null -ne $record.time_s) {
            $telemetryOcclusionStart = [double]$record.time_s
            break
        }
    }
    $telemetryOffset = 0.0
    if ($null -ne $videoOcclusionStart -and $null -ne $telemetryOcclusionStart) {
        $telemetryOffset = [Math]::Round($telemetryOcclusionStart - $videoOcclusionStart, 1)
    }
    Write-Host ("Decision visualization telemetry offset: {0:N1}s" -f $telemetryOffset)
    & $Python (Join-Path $scriptRoot "make_decision_visualization.py") --input $video --telemetry $remoteTelemetry --output $decisionVideo --summary $decisionSummary --telemetry-offset $telemetryOffset
    if ($LASTEXITCODE -ne 0) {
        throw "Decision visualization failed with exit code $LASTEXITCODE."
    }
    $validation = Join-Path $OutputRoot "validation.json"
    & $Python (Join-Path $scriptRoot "validate_rfly_run.py") --summary $summary --telemetry $remoteTelemetry --video $video --ros-summary (Join-Path $OutputRoot "capture_summary.json") --evidence-manifest (Join-Path $OutputRoot "evidence_manifest.json") --output $validation --require-physical-occlusion --maximum-reacquisition-seconds 3.0 --minimum-centered-track-ratio 0.35 --minimum-online-fps 10
    if ($LASTEXITCODE -ne 0) {
        throw "Validation failed with exit code $LASTEXITCODE."
    }
}
finally {
    $pidFile = "$remoteLog/ros_chain_$runId.pids"
    try {
        & ssh @sshArgs "if test -f '$pidFile'; then while IFS= read -r pid; do test -n \"`$pid\" || continue; kill -- -\"`$pid\" 2>/dev/null || kill \"`$pid\" 2>/dev/null || true; done < '$pidFile'; rm -f '$pidFile'; fi" 2>$null | Out-Null
    } catch {
        Write-Warning "Could not clean remote ROS processes for run $runId. Check $remoteLog."
    }
    if ($sshProcess -and -not $sshProcess.HasExited) {
        Stop-Process -Id $sshProcess.Id -Force -ErrorAction SilentlyContinue
    }
    if ($bridgeProcess -and -not $bridgeProcess.HasExited) {
        Stop-Process -Id $bridgeProcess.Id -Force -ErrorAction SilentlyContinue
    }
}

Write-Host "Full Rfly demo completed: $OutputRoot"
