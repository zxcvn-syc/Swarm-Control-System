function result = simulate_lidar_swarm(config)
%SIMULATE_LIDAR_SWARM Six-platform 2D LiDAR measurement-level simulation.
%   The horizontal-plane model simulates range/bearing returns from three UAV
%   and three UGV mounted sensors.  It clusters world-frame returns, classifies
%   a target from estimated footprint, locks tracks after temporal and
%   multi-sensor confirmation, and rejects transient single-sensor ghosts.

arguments
    config (1,1) struct
end

validateattributes(config.durationSec, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.timeStepSec, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.maxRangeM, {'double'}, {'scalar', 'positive', 'finite'});
validateattributes(config.lockHitCount, {'double'}, {'scalar', 'integer', 'positive'});
validateattributes(config.lockSensorSupport, {'double'}, {'scalar', 'integer', 'positive'});

rng(config.rngSeed, "twister");
timeSec = (0:config.timeStepSec:config.durationSec)';
tracks = empty_tracks();
trackHistory = empty_history();
measurements = empty_measurements();
rejectedGhostCount = 0;
nextTrackId = 1;

for step = 1:numel(timeSec)
    currentTime = timeSec(step);
    truth = target_state(config, currentTime);
    frameMeasurements = generate_measurements(config, truth, step, currentTime);
    measurements = [measurements; frameMeasurements]; %#ok<AGROW>
    clusters = cluster_measurements(frameMeasurements, config.clusterRadiusM);

    for trackIndex = 1:numel(tracks)
        if tracks(trackIndex).Active
            tracks(trackIndex).MissedFrames = tracks(trackIndex).MissedFrames + 1;
        end
    end

    for clusterIndex = 1:numel(clusters)
        cluster = clusters(clusterIndex);
        matchedIndex = match_track(tracks, cluster, config.trackGateM);
        if matchedIndex == 0
            tracks(end + 1) = make_track(nextTrackId, cluster, currentTime, step); %#ok<AGROW>
            matchedIndex = numel(tracks);
            nextTrackId = nextTrackId + 1;
        else
            tracks(matchedIndex) = update_track( ...
                tracks(matchedIndex), cluster, currentTime, step, config);
        end

        if ~tracks(matchedIndex).Locked ...
                && tracks(matchedIndex).Hits >= config.lockHitCount ...
                && tracks(matchedIndex).MaxSensorSupport >= config.lockSensorSupport
            tracks(matchedIndex).Locked = true;
            tracks(matchedIndex).LockTimeSec = currentTime;
            tracks(matchedIndex).ClassName = class_from_votes(tracks(matchedIndex).ClassVotes);
        end
        trackHistory(end + 1) = make_history_row( ...
            currentTime, tracks(matchedIndex), cluster); %#ok<AGROW>
    end

    for trackIndex = 1:numel(tracks)
        if tracks(trackIndex).Active && tracks(trackIndex).MissedFrames > config.maxMissedFrames
            tracks(trackIndex).Active = false;
            if tracks(trackIndex).AllGhost && ~tracks(trackIndex).Locked ...
                    && ~tracks(trackIndex).Rejected
                tracks(trackIndex).Rejected = true;
                rejectedGhostCount = rejectedGhostCount + 1;
            end
        end
    end
end

for trackIndex = 1:numel(tracks)
    if tracks(trackIndex).AllGhost && ~tracks(trackIndex).Locked ...
            && ~tracks(trackIndex).Rejected
        tracks(trackIndex).Rejected = true;
        rejectedGhostCount = rejectedGhostCount + 1;
    end
end

observationTable = measurements_table(measurements);
trackTable = history_table(trackHistory);
lockedTracks = tracks([tracks.Locked]);
trueLocked = lockedTracks([lockedTracks.TruthId] > 0);
falseLockCount = sum([lockedTracks.AllGhost]);
classificationAccuracy = locked_classification_accuracy(trueLocked, config);
lockLatencies = [trueLocked.LockTimeSec];
if isempty(lockLatencies)
    meanLockLatencySec = NaN;
else
    meanLockLatencySec = mean(lockLatencies);
end

summary = struct( ...
    "RngSeed", config.rngSeed, ...
    "PlatformCount", height(config.platforms), ...
    "UavCount", sum(config.platforms.PlatformType == "UAV"), ...
    "UgvCount", sum(config.platforms.PlatformType == "UGV"), ...
    "TrueTargetCount", numel(config.targets), ...
    "TotalMeasurements", height(observationTable), ...
    "GhostMeasurements", sum(observationTable.IsGhost), ...
    "LockedTrackCount", numel(trueLocked), ...
    "RejectedGhostTrackCount", rejectedGhostCount, ...
    "FalseLockCount", falseLockCount, ...
    "ClassificationAccuracy", classificationAccuracy, ...
    "MeanLockLatencySec", meanLockLatencySec);

result = struct( ...
    "config", config, ...
    "summary", summary, ...
    "observations", observationTable, ...
    "trackHistory", trackTable, ...
    "tracks", tracks, ...
    "truth", truth_table(config, timeSec));
end

function measurements = generate_measurements(config, truth, step, currentTime)
measurements = empty_measurements();
for platformIndex = 1:height(config.platforms)
    platform = config.platforms(platformIndex, :);
    for targetIndex = 1:numel(truth)
        dx = truth(targetIndex).X - platform.X;
        dy = truth(targetIndex).Y - platform.Y;
        idealRange = hypot(dx, dy);
        if idealRange > config.maxRangeM
            continue
        end
        measuredRange = max(0.05, idealRange + config.rangeNoiseStdM * randn());
        measuredBearing = atan2(dy, dx) + config.bearingNoiseStdRad * randn();
        worldX = platform.X + measuredRange * cos(measuredBearing);
        worldY = platform.Y + measuredRange * sin(measuredBearing);
        extent = max(0.1, truth(targetIndex).FootprintM + config.extentNoiseStdM * randn());
        measurements(end + 1) = make_measurement( ...
            currentTime, platform.PlatformId, platform.PlatformType, measuredRange, ...
            measuredBearing, worldX, worldY, extent, false, truth(targetIndex).Id); %#ok<AGROW>
    end

    % A deterministic ghost is a one-sensor, rapidly moving return.  It has a
    % plausible range/bearing but never has independent sensor confirmation.
    if mod(step + platformIndex, 2) == 0
        ghostRange = 8.0 + mod(7 * step + 3 * platformIndex, 25);
        ghostBearing = mod(0.91 * step + 1.73 * platformIndex, 2 * pi) - pi;
        ghostX = platform.X + ghostRange * cos(ghostBearing);
        ghostY = platform.Y + ghostRange * sin(ghostBearing);
        ghostExtent = 0.7 + mod(step + platformIndex, 4) * 0.35;
        measurements(end + 1) = make_measurement( ...
            currentTime, platform.PlatformId, platform.PlatformType, ghostRange, ...
            ghostBearing, ghostX, ghostY, ghostExtent, true, 0); %#ok<AGROW>
    end
end
measurements = measurements(:);
end

function truth = target_state(config, currentTime)
truth = repmat(struct("Id", 0, "ClassName", "", "FootprintM", 0, "X", 0, "Y", 0), 1, numel(config.targets));
truth(1) = struct( ...
    "Id", config.targets(1).Id, ...
    "ClassName", config.targets(1).ClassName, ...
    "FootprintM", config.targets(1).FootprintM, ...
    "X", 22.0 + 0.20 * currentTime, ...
    "Y", 30.0 + 2.5 * sin(0.08 * currentTime));
truth(2) = struct( ...
    "Id", config.targets(2).Id, ...
    "ClassName", config.targets(2).ClassName, ...
    "FootprintM", config.targets(2).FootprintM, ...
    "X", 62.0 - 0.15 * currentTime, ...
    "Y", 50.0 - 1.5 * sin(0.06 * currentTime));
end

function clusters = cluster_measurements(measurements, radiusM)
clusters = empty_clusters();
if isempty(measurements)
    return
end
used = false(numel(measurements), 1);
for seed = 1:numel(measurements)
    if used(seed)
        continue
    end
    x = [measurements.WorldX]';
    y = [measurements.WorldY]';
    inCluster = ~used & hypot(x - x(seed), y - y(seed)) <= radiusM;
    used(inCluster) = true;
    members = measurements(inCluster);
    sensorIds = unique([members.SensorId]);
    trueIds = [members.TrueId];
    trueIds = trueIds(trueIds > 0);
    if isempty(trueIds)
        trueId = 0;
    else
        trueId = mode(trueIds);
    end
    meanExtent = mean([members.ExtentM]);
    if meanExtent < 2.0
        className = "person";
    else
        className = "vehicle";
    end
    clusters(end + 1) = struct( ...
        "X", mean([members.WorldX]), ...
        "Y", mean([members.WorldY]), ...
        "SensorSupport", numel(sensorIds), ...
        "ClassName", className, ...
        "TrueId", trueId, ...
        "AllGhost", all([members.IsGhost])); %#ok<AGROW>
end
end

function matchIndex = match_track(tracks, cluster, gateM)
matchIndex = 0;
bestDistance = Inf;
for trackIndex = 1:numel(tracks)
    track = tracks(trackIndex);
    differentKnownTarget = track.TruthId ~= cluster.TrueId ...
        && (track.TruthId > 0 || cluster.TrueId > 0);
    if ~track.Active || differentKnownTarget
        continue
    end
    distance = hypot(track.X - cluster.X, track.Y - cluster.Y);
    if distance <= gateM && distance < bestDistance
        matchIndex = trackIndex;
        bestDistance = distance;
    end
end
end

function track = make_track(trackId, cluster, currentTime, step)
votes = [0, 0];
votes = add_class_vote(votes, cluster.ClassName);
track = struct( ...
    "Id", trackId, "X", cluster.X, "Y", cluster.Y, ...
    "LastTimeSec", currentTime, "LastStep", step, "Hits", 1, ...
    "MissedFrames", 0, "MaxSensorSupport", cluster.SensorSupport, ...
    "ClassVotes", votes, "ClassName", cluster.ClassName, ...
    "TruthId", cluster.TrueId, "AllGhost", cluster.AllGhost, ...
    "Locked", false, "LockTimeSec", NaN, "Rejected", false, "Active", true);
end

function track = update_track(track, cluster, currentTime, step, config)
alpha = 0.65;
track.X = alpha * cluster.X + (1 - alpha) * track.X;
track.Y = alpha * cluster.Y + (1 - alpha) * track.Y;
track.LastTimeSec = currentTime;
track.LastStep = step;
track.Hits = track.Hits + 1;
track.MissedFrames = 0;
track.MaxSensorSupport = max(track.MaxSensorSupport, cluster.SensorSupport);
track.ClassVotes = add_class_vote(track.ClassVotes, cluster.ClassName);
track.ClassName = class_from_votes(track.ClassVotes);
if cluster.TrueId > 0
    track.TruthId = cluster.TrueId;
end
track.AllGhost = track.AllGhost && cluster.AllGhost;
track.Active = true;
if hypot(track.X - cluster.X, track.Y - cluster.Y) > config.trackGateM
    error("lidar_swarm:TrackGate", "Track update violated the configured gate.");
end
end

function votes = add_class_vote(votes, className)
if className == "person"
    votes(1) = votes(1) + 1;
else
    votes(2) = votes(2) + 1;
end
end

function className = class_from_votes(votes)
if votes(1) >= votes(2)
    className = "person";
else
    className = "vehicle";
end
end

function row = make_measurement(timeSec, sensorId, sensorType, rangeM, bearingRad, worldX, worldY, extentM, isGhost, trueId)
row = struct( ...
    "TimeSec", timeSec, "SensorId", sensorId, "SensorType", string(sensorType), ...
    "RangeM", rangeM, "BearingRad", bearingRad, "WorldX", worldX, "WorldY", worldY, ...
    "ExtentM", extentM, "IsGhost", logical(isGhost), "TrueId", trueId);
end

function row = make_history_row(timeSec, track, cluster)
row = struct( ...
    "TimeSec", timeSec, "TrackId", track.Id, "WorldX", track.X, "WorldY", track.Y, ...
    "ClassName", string(track.ClassName), "SensorSupport", cluster.SensorSupport, ...
    "Hits", track.Hits, "Locked", track.Locked, "ClusterAllGhost", cluster.AllGhost, ...
    "TruthId", track.TruthId);
end

function tableOut = measurements_table(rows)
if isempty(rows)
    tableOut = table();
    return
end
tableOut = struct2table(rows);
end

function tableOut = history_table(rows)
if isempty(rows)
    tableOut = table();
    return
end
tableOut = struct2table(rows);
end

function accuracy = locked_classification_accuracy(lockedTracks, config)
if isempty(lockedTracks)
    accuracy = NaN;
    return
end
matches = false(1, numel(lockedTracks));
for index = 1:numel(lockedTracks)
    expected = config.targets([config.targets.Id] == lockedTracks(index).TruthId).ClassName;
    matches(index) = string(lockedTracks(index).ClassName) == string(expected);
end
accuracy = mean(matches);
end

function tableOut = truth_table(config, timeSec)
rows = struct("TimeSec", {}, "TargetId", {}, "ClassName", {}, "WorldX", {}, "WorldY", {});
for timeIndex = 1:numel(timeSec)
    truth = target_state(config, timeSec(timeIndex));
    for targetIndex = 1:numel(truth)
        rows(end + 1) = struct( ...
            "TimeSec", timeSec(timeIndex), "TargetId", truth(targetIndex).Id, ...
            "ClassName", string(truth(targetIndex).ClassName), ...
            "WorldX", truth(targetIndex).X, "WorldY", truth(targetIndex).Y); %#ok<AGROW>
    end
end
tableOut = struct2table(rows);
end

function rows = empty_measurements()
rows = struct("TimeSec", {}, "SensorId", {}, "SensorType", {}, "RangeM", {}, ...
    "BearingRad", {}, "WorldX", {}, "WorldY", {}, "ExtentM", {}, "IsGhost", {}, "TrueId", {});
end

function rows = empty_history()
rows = struct("TimeSec", {}, "TrackId", {}, "WorldX", {}, "WorldY", {}, ...
    "ClassName", {}, "SensorSupport", {}, "Hits", {}, "Locked", {}, "ClusterAllGhost", {}, "TruthId", {});
end

function rows = empty_clusters()
rows = struct("X", {}, "Y", {}, "SensorSupport", {}, "ClassName", {}, "TrueId", {}, "AllGhost", {});
end

function rows = empty_tracks()
rows = struct("Id", {}, "X", {}, "Y", {}, "LastTimeSec", {}, "LastStep", {}, ...
    "Hits", {}, "MissedFrames", {}, "MaxSensorSupport", {}, "ClassVotes", {}, ...
    "ClassName", {}, "TruthId", {}, "AllGhost", {}, "Locked", {}, "LockTimeSec", {}, ...
    "Rejected", {}, "Active", {});
end
