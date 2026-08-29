classdef TestLidarSwarmSimulation < matlab.unittest.TestCase
    methods (Test)
        function locksTrueTargetsAndRejectsTransientGhosts(testCase)
            result = simulate_lidar_swarm(lidar_swarm_config());

            testCase.verifyEqual(result.summary.PlatformCount, 6);
            testCase.verifyEqual(result.summary.UavCount, 3);
            testCase.verifyEqual(result.summary.UgvCount, 3);
            testCase.verifyEqual(result.summary.LockedTrackCount, 2);
            testCase.verifyGreaterThan(result.summary.RejectedGhostTrackCount, 0);
            testCase.verifyEqual(result.summary.FalseLockCount, 0);
            testCase.verifyEqual(result.summary.ClassificationAccuracy, 1.0, AbsTol=1e-12);
        end

        function repeatedRunsAreDeterministic(testCase)
            first = simulate_lidar_swarm(lidar_swarm_config());
            second = simulate_lidar_swarm(lidar_swarm_config());

            testCase.verifyEqual(first.summary, second.summary, AbsTol=1e-12);
            testCase.verifyEqual(first.trackHistory, second.trackHistory);
        end
    end
end
