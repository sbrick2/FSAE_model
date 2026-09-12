classdef ReferenceSpeedClosedLoopTest < matlab.unittest.TestCase
    %REFERENCESPEEDCLOSEDLOOPTEST Full 7DOF Autocross regression.

    methods (Test, TestTags = {'Integration', 'Slow'})
        function testAutocrossVehicleEnvelopeStaysWithinTrack(testCase)
            metrics = ReferenceSpeedClosedLoopTest.runAutocross();

            testCase.verifyTrue(metrics.Finished);
            testCase.verifyEqual(metrics.MaximumCGBoundaryViolation, ...
                0.0, AbsTol = 1.0e-9);
            testCase.verifyEqual(metrics.MaximumEnvelopeViolation, ...
                0.0, AbsTol = 1.0e-9);
            testCase.verifyGreaterThan(metrics.MinimumEnvelopeClearance, 0.0);
            testCase.verifyLessThan(metrics.LateralErrorRMS, 0.5);
            testCase.verifyLessThan(metrics.MaximumOverspeed, 1.5);
            testCase.verifyLessThan(metrics.FinishTime, 130.0);
        end
    end

    methods (Static, Access = private)
        function metrics = runAutocross()
            projectRoot = string(fileparts(fileparts(fileparts( ...
                mfilename("fullpath")))));
            project = initProject();
            configFile = fullfile(projectRoot, "results", ...
                "time_domain_closed_loop", "autocross", ...
                "20260816_235359_test", "run_config.json");
            cfg = jsondecode(fileread(configFile)).Config;
            cfg.Driver.Model = "reference_speed";
            cfg.SpeedPlanner.ReferenceMaximumSpeed = 22.5;
            cfg.SpeedPlanner.ReferenceMaximumAcceleration = 6.25;
            cfg.SpeedPlanner.ReferencePlanningDeceleration = 2.50;
            cfg.SpeedPlanner.ReferenceLateralAccelerationLimit = 4.10;
            cfg.SpeedPlanner.ReferenceSpeedSafetyFactor = 1.00;
            cfg.Simulation.StopTime = 160.0;
            cfg.Simulation.SignalLogging = false;
            parameters = ReferenceSpeedClosedLoopTest.readParameters( ...
                project.RootFolder);
            scenario = createLapScenario(cfg);
            scenario = applyGGVReferenceSpeed(scenario, parameters, cfg);
            in = createLapSimulationInput(scenario, parameters, cfg);
            out = sim(in);
            result = collectLapSimulationResults(out, scenario, cfg);
            result = deriveVehicleEnvelopeBoundaryMetrics( ...
                result, scenario, parameters, ...
                TireSectionWidth = cfg.Driver.TireSectionWidth);

            finishIndex = find(double(result.Track.EventDistance(:)) >= ...
                scenario.Track.Length, 1, "first");
            finished = ~isempty(finishIndex);
            if ~finished
                finishIndex = numel(result.Time);
            end
            indices = 1:finishIndex;
            speed = double(result.Vehicle.Speed(indices));
            referenceSpeed = double(result.Track.ReferenceSpeed(indices));
            metrics = struct( ...
                "Finished", finished, ...
                "FinishTime", double(result.Time(finishIndex)), ...
                "MaximumCGBoundaryViolation", max(double( ...
                    result.Track.BoundaryViolation(indices))), ...
                "MaximumEnvelopeViolation", max(double( ...
                    result.Track.VehicleEnvelopeViolation(indices))), ...
                "MinimumEnvelopeClearance", min(double( ...
                    result.Track.VehicleEnvelopeClearance(indices))), ...
                "LateralErrorRMS", sqrt(mean(double( ...
                    result.Track.LateralError(indices)).^2, "omitnan")), ...
                "MaximumOverspeed", max(speed - referenceSpeed));
        end

        function parameters = readParameters(projectRoot)
            dictionary = Simulink.data.dictionary.open( ...
                fullfile(projectRoot, "data", "VehicleData.sldd"));
            cleanup = onCleanup(@() close(dictionary));
            designData = getSection(dictionary, "Design Data");
            groups = ["Vehicle", "Tire", "Aero", ...
                "Powertrain", "Battery", "Brake"];
            parameters = struct;
            for groupName = groups
                parameters.(groupName) = getValue( ...
                    getEntry(designData, groupName));
            end
            if ~isfield(parameters.Brake, "MaxTotalForce")
                parameters.Brake.MaxTotalForce = Inf;
            end
        end
    end
end
