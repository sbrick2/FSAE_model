classdef QuasiStaticRobustnessTest < matlab.unittest.TestCase
    %QuasiStaticROBUSTNESSTEST Regression tests for GGV and event-sweep edge cases.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "ggv")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "lap_time")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "tire")));
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testZeroSpeedLapIsInvalid(testCase)
            speedProfile = struct("Speed", zeros(6, 1), "SampleDistance", 1.0);
            track = struct("IsClosed", false);

            result = calculateLapTime(speedProfile, track);

            testCase.verifyFalse(result.IsValid);
            testCase.verifyEqual(result.LapTime, 50.0, AbsTol = 1.0e-12);
            testCase.verifyNotEmpty(result.FailureReason);
        end

        function testFractionalSweepGridIsAccepted(testCase)
            values = createSweepValues(0.0, 40.0, 0.1);

            testCase.verifyEqual(numel(values), 401);
            testCase.verifyEqual(values(1), 0.0, AbsTol = 1.0e-12);
            testCase.verifyEqual(values(end), 40.0, AbsTol = 1.0e-12);
            testCase.verifyLessThanOrEqual( ...
                max(abs(diff(values) - 0.1)), 1.0e-12);
        end

        function testSkidpadUsesSecondTimedLaps(testCase)
            progress = (0:pi / 10:8 * pi)';
            speedProfile = struct( ...
                "LapTime", 8.0 * pi / 10.0, ...
                "TrackLength", 8.0 * pi, ...
                "ProgressS", progress, ...
                "Time", progress / 10.0, ...
                "Speed", 10.0 * ones(size(progress)), ...
                "IsValid", true, "FailureReason", "", ...
                "Metadata", struct("SpeedEpsilon", 0.1));
            track = struct( ...
                "CenterlineRadius", 1.0, ...
                "RightLoopsEndS", 4.0 * pi, ...
                "FinishLineS", 8.0 * pi);

            metric = calculateEventMetric("skidpad", speedProfile, track);

            expectedLap = 2.0 * pi / 10.0;
            testCase.verifyTrue(metric.IsValid);
            testCase.verifyEqual(metric.RightTimedLap, expectedLap, ...
                AbsTol = 1.0e-12);
            testCase.verifyEqual(metric.LeftTimedLap, expectedLap, ...
                AbsTol = 1.0e-12);
            testCase.verifyEqual(metric.Time, expectedLap, AbsTol = 1.0e-12);
        end
    end

    methods (Test, TestTags = {'Integration', 'Slow'})
        function testGGVFindsBrakingWhenZeroAccelerationIsInfeasible(testCase)
            config = QuasiStaticRobustnessTest.makeVerificationConfig();
            config.Powertrain.MotorSpeedLimit = 1.0;

            ggv = generateGGV(config, ...
                SpeedGrid = 10.0, ...
                LateralPointCount = 3, ...
                MaxLateralAcceleration = 12.0, ...
                MaxLongitudinalAcceleration = 2.0, ...
                MaxBisectionIterations = 12, ...
                LongitudinalBracketPointCount = 17, ...
                MaxSearchExpansionCount = 4, ...
                AllocationMode = "FAST");

            centerIndex = 2;
            testCase.verifyTrue(all(ggv.PointFeasible, "all"));
            testCase.verifyLessThan(ggv.AxMax(1, centerIndex), 0.0);
            testCase.verifyLessThan(ggv.AxMin(1, centerIndex), -2.0);
            testCase.verifyFalse(ggv.Diagnostics.SearchLimitReached);
        end
    end

    methods (Static, Access = private)
        function config = makeVerificationConfig()
            parameters = createQuasiStaticVerificationParameters();
            tireProfile = getSelectedTireProfile();
            config = createQuasiStaticConfiguration(parameters, ...
                TireProfile = tireProfile, TireModel = "MF62", ...
                AllocationMode = "FAST", EnvelopePointCount = 16, ...
                KappaGrid = linspace(-0.40, 0.40, 11), ...
                AlphaGrid = linspace(-0.35, 0.35, 11));
        end
    end
end
