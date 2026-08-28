function config = lidar_swarm_config()
%LIDAR_SWARM_CONFIG Deterministic base-MATLAB LiDAR swarm scenario.

config.rngSeed = 20260828;
config.durationSec = 60.0;
config.timeStepSec = 0.5;
config.maxRangeM = 45.0;
config.rangeNoiseStdM = 0.12;
config.bearingNoiseStdRad = deg2rad(0.45);
config.extentNoiseStdM = 0.12;
config.clusterRadiusM = 1.8;
config.trackGateM = 2.5;
config.maxMissedFrames = 2;
config.lockHitCount = 3;
config.lockSensorSupport = 2;
config.personExtentThresholdM = 2.0;

platformId = (1:6)';
platformType = ["UAV"; "UAV"; "UAV"; "UGV"; "UGV"; "UGV"];
platformX = [10.0; 70.0; 40.0; 8.0; 72.0; 40.0];
platformY = [10.0; 10.0; 70.0; 45.0; 45.0; 5.0];
platformZ = [25.0; 25.0; 25.0; 0.8; 0.8; 0.8];
config.platforms = table( ...
    platformId, platformType, platformX, platformY, platformZ, ...
    VariableNames=["PlatformId", "PlatformType", "X", "Y", "Z"]);

config.targets = struct( ...
    "Id", {1, 2}, ...
    "ClassName", {"person", "vehicle"}, ...
    "FootprintM", {0.8, 3.5});
end
