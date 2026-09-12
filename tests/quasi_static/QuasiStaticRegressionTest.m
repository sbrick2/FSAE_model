classdef QuasiStaticRegressionTest < matlab.unittest.TestCase
    %QuasiStaticREGRESSIONTEST Regression tests for the QuasiStatic GGV and lap-time chain.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "ggv")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "lap_time")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "tire")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "track")));
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testWheelMomentArmsMatchCGGeometry(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();

            result = solveSteadyState(config, 10.0, [0.0, 2.0]);

            expectedX = [config.Vehicle.CGToFrontAxle; ...
                config.Vehicle.CGToFrontAxle; ...
                -config.Vehicle.CGToRearAxle; ...
                -config.Vehicle.CGToRearAxle];
            testCase.verifyTrue(result.Feasible);
            testCase.verifyEqual(result.WheelPosition(:, 1), expectedX, ...
                AbsTol = 1.0e-12);
        end

        function testLightlyLoadedWheelKeepsValidTireEnvelope(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            config.Solver.AllocationMode = "FAST";
            vehicle = config.Vehicle;
            rearStaticLoad = vehicle.Mass * config.Solver.Gravity * ...
                vehicle.CGToFrontAxle / vehicle.Wheelbase;
            rearTransferPerAy = ...
                (1.0 - vehicle.RollStiffnessDistributionFront) * ...
                vehicle.Mass * vehicle.CGHeight / vehicle.TrackRear;
            targetWheelLoad = 0.1;
            targetAy = (rearStaticLoad / 2.0 - targetWheelLoad) / ...
                rearTransferPerAy;

            result = solveSteadyState(config, 10.0, [0.0, targetAy]);

            testCase.verifyEqual(result.NormalLoad(3), targetWheelLoad, ...
                AbsTol = 1.0e-9);
            testCase.verifyGreaterThan(result.NormalLoad(3), 0.0);
            testCase.verifyLessThan(result.TireFxMax(3), 1.0);
            testCase.verifyLessThan(result.TireFyMax(3), 1.0);
        end

        function testZeroSlipTTCMapIsRejectedForGGV(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            config.Tire.Model = "TTCMAP";
            config.Solver.AllocationMode = "FAST";

            operation = @() solveSteadyState(config, 10.0, [0.0, 0.0]);

            testCase.verifyError(operation, "FSAE:QuasiStatic:TireEnvelope");
        end

        function testDefaultRoadSurfaceRetainsMF62Capacity(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();

            result = solveSteadyState(config, 0.0, [0.0, 0.0]);

            testCase.verifyTrue(result.Feasible);
            testCase.verifyEqual(config.Environment.RoadGripScale, ...
                ones(4, 1));
            testCase.verifyEqual(config.Environment.RoadMuLimit, ...
                inf(4, 1));
            testCase.verifyGreaterThan(max(result.TireFxMax ./ ...
                result.NormalLoad), 1.0);
            testCase.verifyEqual(result.TireEnvelopeMethod, ...
                "MF62 combined-slip directional support");
        end

        function testDefaultGGVSteeringIsZeroAtLowSpeed(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();

            result = solveSteadyState(config, 1.0, [0.0, 1.5]);

            testCase.verifyEqual(result.SteeringAngle, 0.0, ...
                AbsTol = 1.0e-12);
            testCase.verifyEqual(result.WheelSteerAngle, zeros(4, 1), ...
                AbsTol = 1.0e-12);
        end

        function testExplicitFixedGGVSteeringIsHonored(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            fixedSteeringAngle = 0.2;

            result = solveSteadyState(config, 1.0, [0.0, 1.5], ...
                SteeringAngle = fixedSteeringAngle);

            testCase.verifyEqual(result.SteeringAngle, ...
                fixedSteeringAngle, AbsTol = 1.0e-12);
            testCase.verifyEqual(result.WheelSteerAngle, ...
                [fixedSteeringAngle; fixedSteeringAngle; 0.0; 0.0], ...
                AbsTol = 1.0e-12);
        end

        function testFiniteRoadMuLimitCapsCombinedEnvelope(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();

            result = solveSteadyState(config, 0.0, [0.0, 0.0], ...
                RoadMuLimit = 1.0);

            testCase.verifyTrue(result.Feasible);
            testCase.verifyLessThanOrEqual(result.TireFxMax, ...
                result.NormalLoad + 1.0e-9);
            testCase.verifyLessThanOrEqual(result.TireFyMax, ...
                result.NormalLoad + 1.0e-9);
        end

        function testTireEnvelopeLookupIsExactAtLoadGridNode(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            lookup = buildTireEnvelopeLookup(config, ...
                LoadPointCount = 17, NormalLoadUpper = 2000.0);
            normalLoad = lookup.NormalLoadGrid(9) .* ones(4, 1);

            exact = evaluateTireEnvelopeMF62(config, normalLoad, ...
                config.Environment.RoadGripScale, ...
                config.Environment.RoadMuLimit, zeros(4, 1));
            interpolated = interpolateTireEnvelopeLookup(lookup, normalLoad);

            testCase.verifyEqual([interpolated.Support], [exact.Support], ...
                AbsTol = 1.0e-10);
            testCase.verifyEqual([interpolated.FxMax], [exact.FxMax], ...
                AbsTol = 1.0e-10);
            testCase.verifyEqual([interpolated.FyMax], [exact.FyMax], ...
                AbsTol = 1.0e-10);
        end

        function testTireEnvelopeLookupInterpolationErrorIsSmall(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            lookup = buildTireEnvelopeLookup(config, ...
                LoadPointCount = 81, NormalLoadUpper = 2000.0);
            normalLoad = [375.0; 725.0; 1125.0; 1575.0];

            exact = evaluateTireEnvelopeMF62(config, normalLoad, ...
                config.Environment.RoadGripScale, ...
                config.Environment.RoadMuLimit, zeros(4, 1));
            interpolated = interpolateTireEnvelopeLookup(lookup, normalLoad);
            exactSupport = [exact.Support];
            interpolatedSupport = [interpolated.Support];
            relativeError = max(abs(interpolatedSupport - exactSupport), ...
                [], "all") / max(abs(exactSupport), [], "all");

            testCase.verifyLessThan(relativeError, 5.0e-3);
        end

        function testTireEnvelopeLookupRejectsOutOfRangeLoad(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            lookup = buildTireEnvelopeLookup(config, ...
                LoadPointCount = 17, NormalLoadUpper = 1000.0);
            operation = @() interpolateTireEnvelopeLookup( ...
                lookup, [1001.0; 500.0; 500.0; 500.0]);

            testCase.verifyError(operation, ...
                "FSAE:QuasiStatic:TireEnvelopeLookupRange");
        end

        function testSteadyStateUsesSuppliedTireEnvelopeLookup(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            config.Solver.AllocationMode = "FAST";
            lookup = buildTireEnvelopeLookup(config, SpeedGrid = 10.0, ...
                LoadPointCount = 17);

            result = solveSteadyState(config, 10.0, [0.0, 1.0], ...
                TireEnvelopeLookup = lookup);

            testCase.verifyTrue(result.Feasible);
            testCase.verifyEqual(result.TireEnvelopeSource, lookup.Method);
        end

        function testOpenTrackHonorsBoundarySpeedsAndDistance(testCase)
            ggv = QuasiStaticRegressionTest.makeSyntheticGGV();
            track = struct("Curvature", zeros(6, 1), ...
                "SampleDistance", 1.0, "IsClosed", false);

            profile = calculateSpeedProfile(track, ggv, ...
                InitialSpeed = 0.0, FinalSpeed = 0.0);
            lap = calculateLapTime(profile, track);

            testCase.verifyEqual(profile.Speed(1), 0.0, AbsTol = 1.0e-12);
            testCase.verifyEqual(profile.Speed(end), 0.0, AbsTol = 1.0e-12);
            testCase.verifyEqual(profile.TrackLength, 5.0, AbsTol = 1.0e-12);
            testCase.verifyEqual(lap.Distance, profile.TrackLength, ...
                AbsTol = 1.0e-12);
        end

        function testCurvatureDirectionUsesMatchingLateralBoundary(testCase)
            ggv = QuasiStaticRegressionTest.makeAsymmetricGGV();
            track = struct("Curvature", [0.1; -0.1], ...
                "SampleDistance", 20.0, "IsClosed", true);

            profile = calculateSpeedProfile(track, ggv);

            testCase.verifyEqual(profile.Speed(1), sqrt(40.0), ...
                AbsTol = 1.0e-4);
            testCase.verifyEqual(profile.Speed(2), sqrt(90.0), ...
                AbsTol = 1.0e-4);
        end

        function testCrossValidationRejectsLateralViolation(testCase)
            ggv = QuasiStaticRegressionTest.makeSyntheticGGV();
            timeDomain = struct("Time", (0:2)', "Vehicle", struct( ...
                "Ux", 5.0 * ones(3, 1), "Ax", zeros(3, 1), ...
                "Ay", 100.0 * ones(3, 1)));

            report = crossValidateQuasiStaticWithTimeDomain( ...
                ggv, timeDomain, Tolerance = 0.01);

            testCase.verifyFalse(report.Pass);
            testCase.verifyEqual(report.MaxLateralUpperViolation, 95.0, ...
                AbsTol = 1.0e-12);
        end

        function testCrossValidationRejectsMismatchedSignalLengths(testCase)
            ggv = QuasiStaticRegressionTest.makeSyntheticGGV();
            timeDomain = struct("Time", (0:2)', "Vehicle", struct( ...
                "Ux", 5.0 * ones(2, 1), "Ax", zeros(3, 1), ...
                "Ay", zeros(3, 1)));

            operation = @() crossValidateQuasiStaticWithTimeDomain(ggv, timeDomain);

            testCase.verifyError(operation, "FSAE:QuasiStatic:TimeDomainSignalSize");
        end

        function test10DOFQuasiStaticReductionSatisfiesBodyEquilibrium(testCase)
            parameters = createQuasiStaticVerificationParameters();
            tireProfile = getSelectedTireProfile();
            config = createQuasiStaticConfiguration(parameters, ...
                TireProfile = tireProfile, TireModel = "MF62", ...
                DynamicsModel = "10DOF", AllocationMode = "FAST", ...
                EnvelopePointCount = 16);
            targetAcceleration = [0.5, 1.0];

            result = solveSteadyState(config, 8.0, targetAcceleration);

            vehicle = config.Vehicle;
            xCorner = [vehicle.CGToFrontAxle; vehicle.CGToFrontAxle; ...
                -vehicle.CGToRearAxle; -vehicle.CGToRearAxle];
            yCorner = [vehicle.TrackFront / 2; -vehicle.TrackFront / 2; ...
                vehicle.TrackRear / 2; -vehicle.TrackRear / 2];
            geometry = [ones(4, 1), yCorner, -xCorner];
            expectedGeneralizedLoad = [ ...
                vehicle.Mass * config.Solver.Gravity; ...
                -vehicle.Mass * targetAcceleration(2) * vehicle.CGHeight; ...
                vehicle.Mass * targetAcceleration(1) * vehicle.CGHeight];
            testCase.verifyEqual(config.Vehicle.DynamicsModel, "10DOF");
            testCase.verifyEqual(config.Metadata.QuasiStaticMethod, ...
                "Vehicle10DOFStaticSuspensionEquilibrium");
            testCase.verifyEqual(config.Suspension.DamperForceAtEquilibrium, 0.0);
            testCase.verifyEqual(geometry.' * result.NormalLoad, ...
                expectedGeneralizedLoad, AbsTol = 1.0e-9);
        end
    end

    methods (Test, TestTags = {'Integration'})
        function testGGVTransitionsFromTireToDrivePowerLimit(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            config.Solver.AllocationMode = "FAST";

            ggv = generateGGV(config, SpeedGrid = [0.0, 25.0], ...
                LateralPointCount = 3, MaxLateralAcceleration = 30.0, ...
                MaxLongitudinalAcceleration = 30.0, ...
                MaxBisectionIterations = 12, ...
                LongitudinalBracketPointCount = 11);

            centerIndex = 2;
            testCase.verifyGreaterThan(ggv.AxMax(1, centerIndex), ...
                config.Solver.Gravity);
            testCase.verifyTrue(any(ggv.ActiveConstraintMax{2, ...
                centerIndex} == "DrivePower"));
        end

        function testParameterSweepRetainsTraceability(testCase)
            config = QuasiStaticRegressionTest.makeVerificationConfig();
            sweep = createQuasiStaticParameterSweep(CdA = 0.70, ...
                ClAFront = 0.0, ClARear = 0.0, ...
                RollStiffnessDistributionFront = 0.50, ...
                GearEfficiency = 0.95);
            generatorOptions = struct("SpeedGrid", 0.0, ...
                "LateralPointCount", 3, ...
                "MaxLateralAcceleration", 10.0, ...
                "MaxLongitudinalAcceleration", 12.0, ...
                "MaxBisectionIterations", 1);
            runMetadata = struct("GitCommit", "TEST_COMMIT", ...
                "Scenario", "QuasiStatic_REGRESSION");

            result = runQuasiStaticParameterSweep(config, sweep, ...
                GenerateGGVOptions = generatorOptions, ...
                RunMetadata = runMetadata);

            testCase.verifyTrue(result.Success);
            testCase.verifyEqual(result.RunMetadata.GitCommit, "TEST_COMMIT");
            testCase.verifyEqual(result.RunMetadata.Scenario, "QuasiStatic_REGRESSION");
            testCase.verifyEqual(height(result.ParameterTrace), height(sweep));
            testCase.verifyEqual(result.ParameterTrace.Source, sweep.Source);
        end
    end

    methods (Static, Access = private)
        function config = makeVerificationConfig()
            parameters = createQuasiStaticVerificationParameters();
            tireProfile = getSelectedTireProfile();
            config = createQuasiStaticConfiguration(parameters, ...
                TireProfile = tireProfile, TireModel = "MF62", ...
                AllocationMode = "LP", EnvelopePointCount = 16, ...
                KappaGrid = linspace(-0.40, 0.40, 11), ...
                AlphaGrid = linspace(-0.35, 0.35, 11));
        end

        function ggv = makeSyntheticGGV()
            ggv = struct( ...
                "Speed", [0.0; 10.0], ...
                "AyPositive", [5.0; 5.0], ...
                "AyNegative", [5.0; 5.0], ...
                "LateralFraction", [-1.0; 0.0; 1.0], ...
                "AxMax", [0.0, 2.0, 0.0; 0.0, 2.0, 0.0], ...
                "AxMin", [0.0, -2.0, 0.0; 0.0, -2.0, 0.0]);
        end

        function ggv = makeAsymmetricGGV()
            ggv = struct( ...
                "Speed", [0.0; 10.0], ...
                "AyPositive", [4.0; 4.0], ...
                "AyNegative", [9.0; 9.0], ...
                "LateralFraction", [-1.0; 0.0; 1.0], ...
                "AxMax", 2.0 * ones(2, 3), ...
                "AxMin", -2.0 * ones(2, 3));
        end
    end
end
