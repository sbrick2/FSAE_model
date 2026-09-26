classdef AdaptiveAutocrossRoutingTest < matlab.unittest.TestCase
    %ADAPTIVEAUTOCROSSROUTINGTEST Verify adaptive driver top-model routing.

    properties (TestParameter)
        routingCase = struct( ...
            "sevenDof", struct( ...
                "DynamicsModel", "7DOF", ...
                "TopModel", "FSAE_AdaptiveAutocross_7DOF", ...
                "PlantModel", "VehiclePlant", ...
                "ControllerModel", "UnifiedControlVehicleController"), ...
            "tenDof", struct( ...
                "DynamicsModel", "10DOF", ...
                "TopModel", "FSAE_AdaptiveAutocross_10DOF", ...
                "PlantModel", "VehiclePlant10DOF", ...
                "ControllerModel", "UnifiedControlVehicleController"))
    end

    methods (TestClassSetup)
        function initializeProject(~)
            initProject();
        end
    end

    methods (Test)
        function testAdaptiveDriverRoutesToMatchingPlant( ...
                testCase, routingCase)
            cfg = createLapSimulationConfig();
            cfg.Vehicle.DynamicsModel = routingCase.DynamicsModel;
            cfg.Driver.Model = "adaptive_autocross";
            cfg.Simulation.StopTime = 0.1;
            scenario = createLapScenario(cfg);
            parameters = AdaptiveAutocrossRoutingTest.readParameters();

            [in, selection] = createLapSimulationInput( ...
                scenario, parameters, cfg);

            testCase.verifyEqual(string(in.ModelName), ...
                routingCase.TopModel);
            testCase.verifyEqual(selection.TopModel, ...
                routingCase.TopModel);
            testCase.verifyEqual(selection.DriverModel, ...
                "AdaptiveAutocrossDriver");
            testCase.verifyEqual(selection.PlantModel, ...
                routingCase.PlantModel);
            testCase.verifyEqual(selection.ControllerModel, ...
                routingCase.ControllerModel);
        end

        function testAdaptiveTopModelUsesUnifiedController( ...
                testCase, routingCase)
            topModel = routingCase.TopModel;
            load_system(topModel);
            cleanup = onCleanup(@() close_system(topModel, 0));
            references = find_system(topModel, "SearchDepth", 1, ...
                "BlockType", "ModelReference");
            referencedModels = string(get_param(references, "ModelName"));

            testCase.verifyTrue(any(referencedModels == ...
                routingCase.ControllerModel));
            testCase.verifyFalse(any(referencedModels == ...
                "TorqueVectoringVehicleController"));
        end

        function testDriverModelExposesCommandAndDiagnosticBuses(testCase)
            driverModel = "AdaptiveAutocrossDriver";
            load_system(driverModel);
            cleanup = onCleanup(@() close_system(driverModel, 0));

            testCase.verifyEqual(AdaptiveAutocrossRoutingTest.topLevelPortNames( ...
                driverModel, "Inport"), ["TrackData"; "Sensor"]);
            testCase.verifyEqual(AdaptiveAutocrossRoutingTest.topLevelPortNames( ...
                driverModel, "Outport"), ["DriverCommand"; "DriverDebug"]);

            debugBus = AdaptiveAutocrossRoutingTest.readBus("DriverDebugBus");
            expectedElements = ["SafeSpeed", "LateralError", ...
                "HeadingError", "BoundaryMargin", "TargetCurvature", ...
                "LimitingCurvature", "LateralUtilization", "ProjectedX", ...
                "ProjectedY", "ResetActive", "StatePreviousIndex", ...
                "StatePreviousReferenceIndex", "StatePreviousSteering", ...
                "StateSpeedIntegrator", "PreviewBrakingDeceleration", ...
                "PreviewBrakingDistance", "PreviewBrakingTargetSpeed", ...
                "PreviewBrakingActive"];
            testCase.verifyEqual(string({debugBus.Elements.Name}).', ...
                expectedElements.');
        end

        function testDriverResetIsOneShotAndFeedsCoreThirdInput(testCase)
            driverModel = "AdaptiveAutocrossDriver";
            load_system(driverModel);
            cleanup = onCleanup(@() close_system(driverModel, 0));
            resetBlock = driverModel + "/StartOfRunReset";
            coreBlock = driverModel + "/AdaptiveDriverCore";

            testCase.verifyEqual(string(get_param(resetBlock, ...
                "InitialCondition")), "true");
            testCase.verifyEqual(string(get_param(resetBlock, ...
                "SampleTime")), "AADControlSampleTime");
            corePorts = get_param(coreBlock, "PortHandles");
            testCase.verifyGreaterThanOrEqual(numel(corePorts.Inport), 3);
            resetLine = get_param(corePorts.Inport(3), "Line");
            testCase.verifyNotEqual(resetLine, -1);
            sourceHandle = get_param(resetLine, "SrcBlockHandle");
            testCase.verifyEqual(string(get_param(sourceHandle, "Name")), ...
                "StartOfRunReset");
        end

        function testTopModelsExposeDriverDebugAsEighthOutput( ...
                testCase, routingCase)
            topModel = routingCase.TopModel;
            load_system(topModel);
            cleanup = onCleanup(@() close_system(topModel, 0));

            outports = AdaptiveAutocrossRoutingTest.topLevelPorts( ...
                topModel, "Outport");
            testCase.verifyEqual(numel(outports), 8);
            eighthOutport = outports{end};
            testCase.verifyEqual(string(get_param(eighthOutport, "Name")), ...
                "DriverDebug");
            outputPorts = get_param(eighthOutport, "PortHandles");
            outputLine = get_param(outputPorts.Inport, "Line");
            testCase.verifyNotEqual(outputLine, -1);
            sourceHandle = get_param(outputLine, "SrcBlockHandle");
            testCase.verifyEqual(string(get_param(sourceHandle, "Name")), ...
                "AdaptiveAutocrossDriver");
        end

        function testCollectedDriverDebugRoundTripsAsBoolean( ...
                testCase, routingCase)
            cfg = createLapSimulationConfig();
            cfg.Vehicle.DynamicsModel = routingCase.DynamicsModel;
            cfg.Driver.Model = "adaptive_autocross";
            cfg.Simulation.StopTime = 0.02;
            cfg.Simulation.SolverProfile = "standard";
            cfg.Simulation.SignalLogging = false;
            scenario = createLapScenario(cfg);
            parameters = AdaptiveAutocrossRoutingTest.readParameters();

            in = createLapSimulationInput(scenario, parameters, cfg);
            out = sim(in);
            result = collectLapSimulationResults(out, scenario, cfg);

            expectedOutputs = ["VehicleState"; "PowertrainState"; ...
                "Sensor"; "TrackReference"; "ControllerDebug"; ...
                "DriverCommand"; "ActuatorCommand"; "DriverDebug"];
            expectedFields = ["SafeSpeed"; "LateralError"; ...
                "HeadingError"; "BoundaryMargin"; "TargetCurvature"; ...
                "LimitingCurvature"; "LateralUtilization"; "ProjectedX"; ...
                "ProjectedY"; "ResetActive"; "StatePreviousIndex"; ...
                "StatePreviousReferenceIndex"; "StatePreviousSteering"; ...
                "StateSpeedIntegrator"; "PreviewBrakingDeceleration"; ...
                "PreviewBrakingDistance"; "PreviewBrakingTargetSpeed"; ...
                "PreviewBrakingActive"];
            testCase.verifyEqual(string(result.SchemaVersion), "1.2");
            testCase.verifyEqual(string( ...
                result.Meta.TopLevelOutputs(:)), expectedOutputs);
            testCase.verifyEqual(string( ...
                fieldnames(result.DriverDebug)), expectedFields);
            testCase.verifyTrue(result.Meta.Valid);

            resetActive = result.DriverDebug.ResetActive(:);
            testCase.verifyTrue(islogical(resetActive));
            testCase.verifyTrue(resetActive(1));
            testCase.verifyTrue(any(~resetActive));
            lastResetTime = double( ...
                result.Time(find(resetActive, 1, "last")));
            testCase.verifyLessThanOrEqual(lastResetTime, ...
                0.01 + max(diff(double(result.Time))) + 1.0e-9);

            filePath = string(tempname) + ".mat";
            save(filePath, "result");
            testCase.addTeardown(@() delete(filePath));
            loaded = loadLapSimulationResult(filePath);
            testCase.verifyEqual(string(loaded.SchemaVersion), "1.2");
            testCase.verifyEqual(loaded.DriverDebug.ResetActive(:), ...
                resetActive);
        end
    end

    methods (Static, Access = private)
        function parameters = readParameters()
            project = currentProject;
            dictionary = Simulink.data.dictionary.open(fullfile( ...
                project.RootFolder, "data", "VehicleData.sldd"));
            cleanup = onCleanup(@() close(dictionary));
            designData = getSection(dictionary, "Design Data");
            groups = ["Vehicle", "Tire", "Aero", "Powertrain", ...
                "Battery", "Brake", "Vehicle10DOFSuspension", ...
                "Vehicle10DOFInitialState"];
            parameters = struct;
            for groupName = groups
                parameters.(groupName) = getValue(getEntry( ...
                    designData, char(groupName)));
            end
            if ~isfield(parameters.Brake, "MaxTotalForce")
                parameters.Brake.MaxTotalForce = Inf;
            end
        end

        function bus = readBus(busName)
            project = currentProject;
            dictionary = Simulink.data.dictionary.open(fullfile( ...
                project.RootFolder, "data", "VehicleData.sldd"));
            cleanup = onCleanup(@() close(dictionary));
            designData = getSection(dictionary, "Design Data");
            bus = getValue(getEntry(designData, char(busName)));
        end

        function names = topLevelPortNames(modelName, blockType)
            ports = AdaptiveAutocrossRoutingTest.topLevelPorts( ...
                modelName, blockType);
            names = string(get_param(ports, "Name"));
        end

        function ports = topLevelPorts(modelName, blockType)
            ports = find_system(modelName, "SearchDepth", 1, ...
                "BlockType", blockType);
            portNumbers = cellfun(@(block) str2double( ...
                get_param(block, "Port")), ports);
            [~, order] = sort(portNumbers);
            ports = ports(order);
        end
    end
end
