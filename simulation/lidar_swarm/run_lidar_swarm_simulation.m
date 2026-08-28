function summary = run_lidar_swarm_simulation(outputDir)
%RUN_LIDAR_SWARM_SIMULATION Run the deterministic scenario and write evidence.

arguments
    outputDir (1,1) string = fullfile(fileparts(mfilename("fullpath")), "results")
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end
config = lidar_swarm_config();
result = simulate_lidar_swarm(config);
summary = result.summary;

metricsTable = struct2table(summary);
writetable(metricsTable, fullfile(outputDir, "lidar_swarm_metrics.csv"));
writetable(result.observations, fullfile(outputDir, "lidar_swarm_measurements.csv"));
writetable(result.trackHistory, fullfile(outputDir, "lidar_swarm_tracks.csv"));
writetable(result.truth, fullfile(outputDir, "lidar_swarm_truth.csv"));

summaryFile = fullfile(outputDir, "lidar_swarm_summary.json");
fileId = fopen(summaryFile, "w");
if fileId < 0
    error("lidar_swarm:Output", "Cannot open summary output: %s", summaryFile);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, "%s\n", jsonencode(summary));

figureHandle = figure( ...
    Visible="off", Color="white", Position=[100, 100, 1200, 540], Renderer="painters");
layout = tiledlayout(1, 2, TileSpacing="compact", Padding="compact");

leftAxes = nexttile(layout);
style_axes(leftAxes);
hold(leftAxes, "on");
isGhost = result.observations.IsGhost;
scatter(result.observations.WorldX(~isGhost), result.observations.WorldY(~isGhost), ...
    9, [0.25, 0.55, 0.85], "filled", DisplayName="LiDAR target returns");
scatter(result.observations.WorldX(isGhost), result.observations.WorldY(isGhost), ...
    9, [0.85, 0.25, 0.25], "x", DisplayName="Single-sensor ghosts");
for targetId = 1:numel(config.targets)
    mask = result.truth.TargetId == targetId;
    plot(result.truth.WorldX(mask), result.truth.WorldY(mask), LineWidth=2.0, ...
        DisplayName="True " + config.targets(targetId).ClassName + " trajectory");
end
uavMask = config.platforms.PlatformType == "UAV";
scatter(config.platforms.X(uavMask), config.platforms.Y(uavMask), 90, "^", ...
    "filled", DisplayName="UAV LiDAR");
scatter(config.platforms.X(~uavMask), config.platforms.Y(~uavMask), 90, "s", ...
    "filled", DisplayName="UGV LiDAR");
axis equal;
xlim([0, 80]);
ylim([0, 80]);
xlabel("World X (m)");
ylabel("World Y (m)");
title("Six-platform LiDAR observations and ghost returns");
grid on;
legendHandle = legend(Location="eastoutside");
style_legend(legendHandle);

rightAxes = nexttile(layout);
style_axes(rightAxes);
lockRows = result.trackHistory(result.trackHistory.Locked, :);
if isempty(lockRows)
    plot(rightAxes, 0, 0, "w.");
else
    uniqueTracks = unique(lockRows.TrackId);
    hold(rightAxes, "on");
    for index = 1:numel(uniqueTracks)
        mask = lockRows.TrackId == uniqueTracks(index);
        plot(lockRows.TimeSec(mask), lockRows.Hits(mask), LineWidth=2.0, ...
            DisplayName="Locked track " + string(uniqueTracks(index)));
    end
    thresholdLine = yline(config.lockHitCount, "--");
    thresholdLine.DisplayName = "Lock hit threshold";
    grid on;
    legendHandle = legend(Location="northwest");
    style_legend(legendHandle);
end
xlabel("Time (s)");
ylabel("Confirmed cluster hits");
title("Temporal confirmation before lock");

exportgraphics(figureHandle, fullfile(outputDir, "lidar_swarm_overview.png"), Resolution=160);
close(figureHandle);
end

function style_axes(axesHandle)
axesHandle.Color = [1, 1, 1];
axesHandle.XColor = [0, 0, 0];
axesHandle.YColor = [0, 0, 0];
axesHandle.GridColor = [0.7, 0.7, 0.7];
axesHandle.Title.Color = [0, 0, 0];
axesHandle.XLabel.Color = [0, 0, 0];
axesHandle.YLabel.Color = [0, 0, 0];
end

function style_legend(legendHandle)
legendHandle.Color = [1, 1, 1];
legendHandle.TextColor = [0, 0, 0];
legendHandle.EdgeColor = [0.35, 0.35, 0.35];
end
