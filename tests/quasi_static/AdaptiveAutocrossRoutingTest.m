classdef AdaptiveAutocrossRoutingTest < matlab.unittest.TestCase
    %ADAPTIVEAUTOCROSSROUTINGTEST Verify adaptive driver top-model routing.

    properties (TestParameter)
        routingCase = struct( ...
            "sevenDof", struct( ...
                "DynamicsModel", "7DOF", ...
                "TopModel", "FSAE_AdaptiveAutocross_7DOF", ...
                "PlantModel", "VehiclePlant"), ...
            "tenDof", struct( ...
                "DynamicsModel", "10DOF", ...
                "TopModel", "FSAE_AdaptiveAutocross_10DOF", ...
                "PlantModel", "VehiclePlant10DOF"))
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
    end
end
