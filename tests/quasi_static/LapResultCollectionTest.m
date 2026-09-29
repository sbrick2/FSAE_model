classdef LapResultCollectionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addProjectPaths(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "simulation")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "track")));
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testFinishTruncatesDriverDebugWithVehicleSignals(testCase)
            [simulationOutput, scenario] = LapResultCollectionTest.makeInputs();

            result = collectLapSimulationResults(simulationOutput, scenario);

            testCase.verifyEqual(result.Time, [0.0; 1.0; 2.0]);
            testCase.verifyEqual(result.Vehicle.X, [0.0; 1.0; 2.0]);
            testCase.verifyEqual(result.SchemaVersion, "1.2");
            testCase.verifySize(result.DriverDebug.SafeSpeed, [3, 1]);
            testCase.verifySize(result.DriverDebug.PreviewBrakingDeceleration, [3, 1]);
            testCase.verifySize(result.DriverDebug.PreviewBrakingDistance, [3, 1]);
            testCase.verifySize(result.DriverDebug.PreviewBrakingTargetSpeed, [3, 1]);
            testCase.verifySize(result.DriverDebug.PreviewBrakingActive, [3, 1]);
        end
    end

    methods (Static, Access = private)
        function [out, scenario] = makeInputs()
            time = (0.0:4.0).';
            zeroSignal = timeseries(zeros(5, 1), time);
            vehicle = struct("X", timeseries(time, time), "Y", zeroSignal, ...
                "Psi", zeroSignal, "Ux", timeseries(ones(5, 1), time), ...
                "Uy", zeroSignal);
            debug = struct("SafeSpeed", timeseries(3.0 * ones(5, 1), time), ...
                "PreviewBrakingDeceleration", zeroSignal, ...
                "PreviewBrakingDistance", zeroSignal, ...
                "PreviewBrakingTargetSpeed", zeroSignal, ...
                "PreviewBrakingActive", timeseries(false(5, 1), time));
            dataset = Simulink.SimulationData.Dataset;
            dataset = dataset.addElement( ...
                LapResultCollectionTest.makeSignal("VehicleState", vehicle));
            dataset = dataset.addElement( ...
                LapResultCollectionTest.makeSignal("DriverDebug", debug));
            out = Simulink.SimulationOutput;
            out.tout = time;
            out.yout = dataset;
            track = struct("X", [0.0; 1.0; 2.0], "Y", zeros(3, 1), ...
                "Heading", zeros(3, 1), "Curvature", zeros(3, 1), ...
                "ReferenceSpeed", 3.0 * ones(3, 1), ...
                "LeftHalfWidth", ones(3, 1), "RightHalfWidth", ones(3, 1), ...
                "SampleDistance", 1.0, "Length", 2.0, "IsClosed", false);
            scenario = struct("Track", track, "NumberOfLaps", 1, ...
                "Event", "autocross", "ID", "synthetic_finish");
        end

        function signal = makeSignal(name, values)
            signal = Simulink.SimulationData.Signal;
            signal.Name = char(name);
            signal.BlockPath = Simulink.SimulationData.BlockPath( ...
                char("synthetic/" + name));
            signal.Values = values;
        end
    end
end
