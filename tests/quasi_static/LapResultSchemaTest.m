classdef LapResultSchemaTest < matlab.unittest.TestCase
    %LAPRESULTSCHEMATEST Compatibility checks for normalized lap results.

    methods (TestClassSetup)
        function addReportingPath(testCase)
            projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "reporting")));
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testLoadsLegacySchemaWithoutDriverDebug(testCase)
            result = LapResultSchemaTest.baseResult("1.0");
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            loaded = loadLapSimulationResult(filePath);

            testCase.verifyEqual(loaded.SchemaVersion, "1.0");
            testCase.verifyFalse(isfield(loaded, "DriverDebug"));
            testCase.verifyFalse(any(listLapResultFields(loaded) == ...
                "DriverDebug.SafeSpeed"));
        end

        function testLoadsAndExposesSchema11DriverDebug(testCase)
            result = LapResultSchemaTest.baseResult("1.1");
            result.DriverDebug = LapResultSchemaTest.driverDebug();
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            loaded = loadLapSimulationResult(filePath);
            [values, info] = getLapResultSignal(loaded, ...
                "DriverDebug.SafeSpeed");

            testCase.verifyEqual(loaded.SchemaVersion, "1.1");
            testCase.verifyTrue(any(listLapResultFields(loaded) == ...
                "DriverDebug.ResetActive"));
            testCase.verifyEqual(values, [10; 11]);
            testCase.verifyEqual(info.Unit, "m/s");
        end

        function testSchema11RequiresDriverDebugGroup(testCase)
            result = LapResultSchemaTest.baseResult("1.1");
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            testCase.verifyError(@() loadLapSimulationResult(filePath), ...
                "FSAE:Lap:ResultDriverDebugGroupMissing");
        end

        function testSchema10RejectsDriverDebugGroup(testCase)
            result = LapResultSchemaTest.baseResult("1.0");
            result.DriverDebug = LapResultSchemaTest.driverDebug();
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            testCase.verifyError(@() loadLapSimulationResult(filePath), ...
                "FSAE:Lap:ResultDriverDebugUnexpected");
        end

        function testLoadsSchema12PreviewBrakingDiagnostics(testCase)
            result = LapResultSchemaTest.baseResult("1.2");
            result.DriverDebug = LapResultSchemaTest.driverDebug();
            result.DriverDebug.PreviewBrakingDeceleration = [0; 2.5];
            result.DriverDebug.PreviewBrakingDistance = [0; 18];
            result.DriverDebug.PreviewBrakingTargetSpeed = [0; 7];
            result.DriverDebug.PreviewBrakingActive = logical([false; true]);
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            loaded = loadLapSimulationResult(filePath);
            [values, info] = getLapResultSignal(loaded, ...
                "DriverDebug.PreviewBrakingDeceleration");

            testCase.verifyEqual(loaded.SchemaVersion, "1.2");
            testCase.verifyEqual(values, [0; 2.5]);
            testCase.verifyEqual(info.Unit, "m/s^2");
            testCase.verifyTrue(any(listLapResultFields(loaded) == ...
                "DriverDebug.PreviewBrakingActive"));
        end

        function testSchema12RequiresPreviewBrakingFields(testCase)
            result = LapResultSchemaTest.baseResult("1.2");
            result.DriverDebug = LapResultSchemaTest.driverDebug();
            filePath = LapResultSchemaTest.writeResult(testCase, result);

            testCase.verifyError(@() loadLapSimulationResult(filePath), ...
                "FSAE:Lap:ResultDriverDebugFieldMissing");
        end
    end

    methods (Static, Access = private)
        function result = baseResult(schemaVersion)
            result = struct("SchemaVersion", schemaVersion, ...
                "Time", [0; 0.01], "Distance", [0; 0.1], ...
                "Track", struct, "Vehicle", struct, "Wheel", struct, ...
                "Tire", struct, "Powertrain", struct, "Battery", struct, ...
                "Driver", struct, "Actuator", struct, ...
                "Controller", struct, "Sensor", struct);
        end

        function debug = driverDebug()
            names = ["SafeSpeed", "LateralError", "HeadingError", ...
                "BoundaryMargin", "TargetCurvature", "LimitingCurvature", ...
                "LateralUtilization", "ProjectedX", "ProjectedY", ...
                "ResetActive", "StatePreviousIndex", ...
                "StatePreviousReferenceIndex", "StatePreviousSteering", ...
                "StateSpeedIntegrator"];
            debug = cell2struct(repmat({[1; 2]}, 1, numel(names)), ...
                cellstr(names), 2);
            debug.SafeSpeed = [10; 11];
            debug.ResetActive = logical([true; false]);
        end

        function filePath = writeResult(testCase, result)
            filePath = string(tempname) + ".mat";
            save(filePath, "result");
            testCase.addTeardown(@() delete(filePath));
        end
    end
end
