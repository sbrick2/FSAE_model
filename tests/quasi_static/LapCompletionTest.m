classdef LapCompletionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "simulation")));
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testValidFinishGateAcceptsProjectedSampleShortfall(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyTrue(completed);
        end

        function testInvalidTimingDoesNotCompleteShortLap(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();
            result.Metrics.EventTiming.IsValid = false;

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyFalse(completed);
        end

        function testTimedSectorDoesNotCompleteWholeEvent(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();
            result.Track.EventDistance = [0.0; 57.0];
            result.Metrics.EventTiming.Distance = 57.0;

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyFalse(completed);
        end

        function testSingleGateDoesNotCompleteMultipleLaps(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();
            scenario.NumberOfLaps = 2;

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyFalse(completed);
        end

        function testProjectedDistanceCompletesWithoutTiming(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();
            result.Track.EventDistance = [0.0; 1000.0];
            result = rmfield(result, "Metrics");

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyTrue(completed);
        end

        function testNonfiniteTimingDoesNotCompleteShortLap(testCase)
            [result, scenario] = LapCompletionTest.makeInputs();
            result.Metrics.EventTiming.Time = NaN;

            completed = isLapSimulationComplete(result, scenario);

            testCase.verifyFalse(completed);
        end
    end

    methods (Static, Access = private)
        function [result, scenario] = makeInputs()
            scenario = struct("Track", struct("Length", 1000.0, ...
                "IsClosed", true), "NumberOfLaps", 1);
            result = struct("Track", struct("EventDistance", [0.0; 999.5]), ...
                "Metrics", struct("EventTiming", struct("IsValid", true, ...
                "Time", 76.0, "Distance", 1000.0)));
        end
    end
end
