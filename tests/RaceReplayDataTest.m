classdef RaceReplayDataTest < matlab.unittest.TestCase
    properties (Access = private)
        Result
    end
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "scripts", "replay")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "scripts", "reporting")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "scripts", "simulation")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "scripts", "tire")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, "tests", "helpers")));
        end
    end
    methods (TestMethodSetup)
        function createResult(testCase)
            testCase.Result = createRaceReplayTestResult();
        end
    end
    methods (Test)
        function rejectsRepeatedTime(testCase)
            result = testCase.Result;
            result.Time(2) = 0;
            testCase.verifyError(@() prepareRaceReplayData(result), "FSAE:Replay:Time");
        end
        function rejectsPowertrainColumnMismatch(testCase)
            result = testCase.Result;
            result.Powertrain.MotorTorqueActual = zeros(3, 3);
            testCase.verifyError(@() prepareRaceReplayData(result), "FSAE:Replay:Dimension");
        end
        function rejectsInconsistentRPM(testCase)
            result = testCase.Result;
            result.Powertrain.MotorRPM(1, 1) = 1;
            testCase.verifyError(@() prepareRaceReplayData(result), "FSAE:Replay:MotorSpeedMismatch");
        end
        function usesOneClockAndHoldsDriverCommands(testCase)
            data = prepareRaceReplayData(testCase.Result);
            frame = sampleRaceReplayFrame(data, 0.5);
            testCase.verifyEqual(frame.Vehicle.X, 2.5);
            testCase.verifyEqual(frame.Driver.AccelerationRequest, 4);
            testCase.verifyEqual(frame.Pedals.Accelerator, 100);
            testCase.verifyEqual(frame.Pedals.Brake, 0);
            testCase.verifyEqual(frame.Motor.Power, [2250, 6250, -5250, -11250]);
            testCase.verifyEqual(frame.Motor.TotalPower, -8000);
            testCase.verifyEqual(frame.Track.LapTime, 0.5);
        end
        function negativeDriverRequestAndActualAccelerationDiffer(testCase)
            frame = sampleRaceReplayFrame(prepareRaceReplayData(testCase.Result), 2);
            testCase.verifyEqual(frame.Pedals.Accelerator, 0);
            testCase.verifyEqual(frame.Pedals.Brake, 100);
            testCase.verifyEqual(frame.Vehicle.Ax, 0);
            testCase.verifyTrue(frame.Pedals.Equivalent);
        end
        function originalPedalsTakePriority(testCase)
            result = testCase.Result;
            result.Driver.AcceleratorPedalRequest = [0.35; 0.2; 0];
            result.Driver.BrakePedalRequest = [0.1; 0; 0];
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0.5);
            testCase.verifyEqual([frame.Pedals.Accelerator, frame.Pedals.Brake], [35, 10]);
            testCase.verifyFalse(frame.Pedals.Equivalent);
        end
        function missingSignalsStayUnavailable(testCase)
            result = testCase.Result;
            result.Driver = struct;
            result.Powertrain = struct;
            result.Meta = rmfield(result.Meta, "Replay");
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            data = prepareRaceReplayData(result, ProjectRoot = string(fixture.Folder));
            frame = sampleRaceReplayFrame(data, 0.5);
            testCase.verifyTrue(isnan(frame.Pedals.Accelerator));
            testCase.verifyTrue(isnan(frame.Motor.TotalPower));
            testCase.verifyTrue(all(isnan(frame.Tire.Capacity.X)));
            testCase.verifyTrue(data.Geometry.Approximate);
            testCase.verifyEmpty(fieldnames(data.Track));
        end
        function automaticallyLoadsEventTrack(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            file = RaceReplayDataTest.writeTrackFile(fixture.Folder, 100);
            result = testCase.Result;
            result.Meta.Replay.Track = struct;
            data = prepareRaceReplayData(result, ProjectRoot = string(fixture.Folder));
            testCase.verifyEqual(data.Track.X, [-10; 0; 10] + 100);
            testCase.verifySubstring(data.TrackSource, file);
        end
        function automaticallyLoadsRecordedTrackFolder(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            file = RaceReplayDataTest.writeTrackFile(fixture.Folder, 200);
            result = testCase.Result;
            result.Meta.Replay.Track = struct;
            result.Config.Track = struct("Name", "autocross", "DataFolder", string(fileparts(file)));
            data = prepareRaceReplayData(result, ProjectRoot = string(fullfile(fixture.Folder, "missing")));
            testCase.verifyEqual(data.Track.X, [-10; 0; 10] + 200);
            testCase.verifySubstring(data.TrackSource, file);
        end
        function snapshotTrackHasPriorityOverCurrentCache(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            RaceReplayDataTest.writeTrackFile(fixture.Folder, 100);
            data = prepareRaceReplayData(testCase.Result, ProjectRoot = string(fixture.Folder));
            testCase.verifyEqual(data.Track.X, testCase.Result.Meta.Replay.Track.X);
            testCase.verifyEqual(data.TrackSource, "运行快照");
        end
        function relocatedRecordedFolderFallsBackToMatchingProjectTrack(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            file = RaceReplayDataTest.writeTrackFile(fixture.Folder, 300);
            result = testCase.Result;
            result.Meta.Replay.Track = struct;
            result.Config.Track = struct("Name", "autocross", "DataFolder", ...
                string(fullfile(fixture.Folder, "old_checkout", "trackdata")));
            data = prepareRaceReplayData(result, ProjectRoot = string(fixture.Folder));
            testCase.verifyEqual(data.Track.X, [-10; 0; 10] + 300);
            testCase.verifySubstring(data.TrackSource, file);
        end
        function malformedAutomaticTrackKeepsTrajectoryReplay(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            file = RaceReplayDataTest.writeTrackFile(fixture.Folder, 0);
            loaded = load(file);
            track = loaded.track;
            track.IsClosed = [true, false];
            save(file, "track");
            result = testCase.Result;
            result.Meta.Replay.Track = struct;

            data = prepareRaceReplayData(result, ProjectRoot = string(fixture.Folder));
            frame = sampleRaceReplayFrame(data, 0.5);

            testCase.verifyEmpty(fieldnames(data.Track));
            testCase.verifyTrue(frame.Vehicle.Valid);
            testCase.verifySubstring(data.TrackSource, "仅显示车辆轨迹");
        end
        function missingOneMotorInvalidatesTotal(testCase)
            result = testCase.Result;
            result.Powertrain.MotorTorqueActual(:, 2) = NaN;
            result.Powertrain.MotorMechanicalPower(:, 2) = NaN;
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0.5);
            testCase.verifyTrue(isnan(frame.Motor.TotalPower));
            testCase.verifyEqual(frame.Motor.Power([1, 3, 4]), [2250, -5250, -11250]);
        end
        function headingWrapUsesShortRotation(testCase)
            result = testCase.Result;
            result.Vehicle.Psi = deg2rad([179; -179; -170]);
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0.5);
            testCase.verifyEqual(frame.Vehicle.Psi, pi, AbsTol = 1e-12);
        end
        function missingPoseIsNotBridged(testCase)
            result = testCase.Result;
            result.Vehicle.Psi(2) = NaN;
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0.5);
            testCase.verifyFalse(frame.Vehicle.Valid);
        end
        function historicalPartialLapHasUnknownLapTime(testCase)
            result = testCase.Result;
            result.Track.PathS = [4; 9; 19];
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 1);
            testCase.verifyTrue(isnan(frame.Track.LapTime));
        end
        function forceComponentsRotateWithWheelAndYaw(testCase)
            data = prepareRaceReplayData(testCase.Result);
            frame = sampleRaceReplayFrame(data, 0);
            frame.Vehicle.Psi = pi / 2;
            frame.Wheel.SteerAngle(1) = pi / 2;
            geometry = transformRaceReplayGeometry(frame, data.Geometry);
            testCase.verifyEqual(geometry.FxVectors(1, :), [-300, 0], AbsTol = 1e-10);
            testCase.verifyEqual(geometry.FyVectors(1, :), [0, -400], AbsTol = 1e-10);
            testCase.verifyEqual(geometry.FxVectors(2, :), [0, -300], AbsTol = 1e-10);
            testCase.verifyEqual(geometry.FyVectors(3, :), [500, 0], AbsTol = 1e-10);
        end
        function headingViewPreservesForceSigns(testCase)
            data = prepareRaceReplayData(testCase.Result);
            frame = sampleRaceReplayFrame(data, 0);
            frame.Vehicle.Psi = -pi / 3;
            geometry = transformRaceReplayGeometry(frame, data.Geometry, View = "heading");
            testCase.verifyEqual(geometry.FxVectors, [zeros(4, 1), frame.Tire.Fx(:)], AbsTol = 1e-10);
            testCase.verifyEqual(geometry.FyVectors, [-frame.Tire.Fy(:), zeros(4, 1)], AbsTol = 1e-10);
            testCase.verifyEqual(geometry.Center, [0, 0]);
        end
        function simpleCapacityMatchesRegularizedModel(testCase)
            frame = sampleRaceReplayFrame(prepareRaceReplayData(testCase.Result), 0);
            testCase.verifyEqual(frame.Tire.Capacity.X, [900, 900, 900, 900]);
            testCase.verifyEqual(frame.Tire.Capacity.Utilization, ...
                testCase.Result.Tire.MuUtilization(1, :), AbsTol = 1e-12);
        end
        function mf62UtilizationMatchesSimulationEvaluator(testCase)
            [wheel, tire, metadata, expected] = RaceReplayDataTest.mf62Case();
            capacity = deriveRaceReplayTireCapacity(wheel, tire, metadata);
            testCase.verifyEqual(capacity.Utilization, expected, AbsTol = 1e-12);
            testCase.verifyGreaterThan(capacity.X, zeros(1, 4));
        end
        function ttcCapacityUsesSavedMapPeak(testCase)
            result = testCase.Result;
            result.Meta.Replay.Tire = struct("Model", "ttc_map", ...
                "RoadGripScale", 0.8, "RoadMuLimit", Inf, ...
                "MapPoints", [0, 0, 0, 500, 80000; 0, 0, 0, 1000, 80000], ...
                "MapForces", [600, 800; 0, 1500]);
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0);
            testCase.verifyEqual(frame.Tire.Capacity.X, ones(1, 4) * 1600);
        end
        function roadInputsChangeWithRecordedClock(testCase)
            result = testCase.Result;
            result.Environment = struct("RoadGripScale", [1; 0.5; 0.2], ...
                "RoadMuLimit", [0.9; 0.8; 0.7]);
            data = prepareRaceReplayData(result);
            before = sampleRaceReplayFrame(data, 0.9);
            after = sampleRaceReplayFrame(data, 1.1);
            testCase.verifyEqual(before.Tire.Capacity.X, ones(1, 4) * 900);
            testCase.verifyEqual(after.Tire.Capacity.X, ones(1, 4) * 500);
        end
        function lowLoadAndMissingLoadAreDistinct(testCase)
            result = testCase.Result;
            result.Wheel.NormalLoad(:, 1:2) = repmat([0, NaN], 3, 1);
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0);
            testCase.verifyEqual(frame.Tire.Capacity.LowLoad, [true, false, false, false]);
            testCase.verifyTrue(isnan(frame.Tire.Capacity.X(2)));
            testCase.verifyEqual(frame.Tire.Capacity.Utilization(1), 0);
        end
        function savesAndLoadsHistoricalSnapshot(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            result = testCase.Result;
            file = fullfile(fixture.Folder, "result.mat");
            save(file, "result");
            data = prepareRaceReplayData(file);
            testCase.verifyEqual(data.Geometry.Wheelbase, 2);
            testCase.verifyEqual(data.TireMetadata.ForceEpsilon, 20);
            testCase.verifyEqual(data.SourceFile, string(file));
        end
        function resultSaverPreservesCapturedSnapshot(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            cfg = struct("Output", struct("ResultsRoot", fixture.Folder, ...
                "RunName", "replay", "Overwrite", false, ...
                "SaveRawSimulationOutput", false, "SaveFinalTrackView", false));
            scenario = struct("Event", "autocross", "Track", testCase.Result.Meta.Replay.Track);
            output = saveLapSimulationResults(testCase.Result, scenario, cfg);
            loaded = load(output.ResultFile, "result");
            testCase.verifyEqual(loaded.result.Meta.Replay, testCase.Result.Meta.Replay);
        end
        function capturesSuppliedEffectiveParameters(testCase)
            scenario = struct("Track", testCase.Result.Meta.Replay.Track);
            parameters = struct("Vehicle", testCase.Result.Meta.Replay.Geometry, ...
                "Tire", struct("UnloadedRadius", struct("Value", 0.21)));
            cfg = struct("Driver", testCase.Result.Config.Driver);
            snapshot = createRaceReplaySnapshot(scenario, parameters, cfg, ...
                TireMetadata = testCase.Result.Meta.Replay.Tire);
            testCase.verifyEqual(snapshot.Geometry.WheelLength, 0.42, AbsTol = 1e-12);
            testCase.verifyEqual(snapshot.Driver.SteeringWheelRatio, 1);
            testCase.verifyEqual(snapshot.Tire, testCase.Result.Meta.Replay.Tire);
        end
        function zeroGripHasKnownInactiveUtilization(testCase)
            result = testCase.Result;
            result.Meta.Replay.Tire.RoadGripScale = 0;
            result.Tire.FxWheel(:) = 0;
            result.Tire.FyWheel(:) = 0;
            frame = sampleRaceReplayFrame(prepareRaceReplayData(result), 0);
            testCase.verifyTrue(all(frame.Tire.Capacity.NoGrip));
            testCase.verifyEqual(frame.Tire.Capacity.Utilization, zeros(1, 4));
            testCase.verifyFalse(any(frame.Tire.Capacity.Available));
        end
        function rejectsWrongWheelOrderInSnapshot(testCase)
            result = testCase.Result;
            result.Meta.Replay.WheelOrder = ["FR", "FL", "RR", "RL"];
            testCase.verifyError(@() prepareRaceReplayData(result), "FSAE:Replay:WheelOrder");
        end
    end
    methods (Static, Access = private)
        function file = writeTrackFile(folder, offset)
            data = createRaceReplayTestResult();
            track = data.Meta.Replay.Track;
            track.X = track.X + offset;
            metadata = struct("TrackName", "autocross");
            cache = fullfile(folder, "trackdata");
            mkdir(cache);
            file = string(fullfile(cache, "autocross.mat"));
            save(file, "track", "metadata");
        end
        function [wheel, tire, metadata, expected] = mf62Case()
            profile = getSelectedTireProfile();
            parameters = serializeTireModelMFParameters(profile);
            wheel = struct("NormalLoad", [500, 700, 900, 1100], ...
                "CamberAngle", [0, 0.02, -0.02, 0.04], ...
                "SlipRatio", [0.12, -0.15, 0.08, -0.04]);
            input = [wheel.SlipRatio, [0.05, -0.06, 0.03, 0.04], ...
                wheel.CamberAngle, wheel.NormalLoad, [0.8, 0.9, 1, 1], [Inf, 1, 1.2, Inf]].';
            output = evaluateTireMF62Codegen(input, parameters);
            tire = struct("Fx", output(1:4).', "Fy", output(5:8).');
            expected = output(9:12).';
            metadata = struct("Model", "mf62", "Parameters", parameters, ...
                "RoadGripScale", [0.8, 0.9, 1, 1], "RoadMuLimit", [Inf, 1, 1.2, Inf]);
        end
    end
end
