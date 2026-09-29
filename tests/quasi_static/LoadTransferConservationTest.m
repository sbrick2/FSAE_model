classdef LoadTransferConservationTest < matlab.unittest.TestCase
    properties (TestParameter)
        lateralAcceleration = struct("leftTurn", 10.0, ...
            "rightTurn", -10.0, "straight", 0.0, ...
            "frontLeftLift", 16.0, "frontRightLift", -16.0)
    end

    methods (TestClassSetup)
        function initializeProject(~)
            initProject();
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testPlantBalancesForcesAndMoments(testCase, lateralAcceleration)
            [vehicle, simulation] = LoadTransferConservationTest.parameters();
            time = [0.0; 0.01];
            dataset = Simulink.SimulationData.Dataset;
            dataset = dataset.addElement(timeseries([2.0; 2.0], time));
            dataset = dataset.addElement(timeseries( ...
                lateralAcceleration * ones(2, 1), time));
            dataset = dataset.addElement(timeseries([100.0; 100.0], time));
            dataset = dataset.addElement(timeseries([150.0; 150.0], time));
            in = Simulink.SimulationInput("LoadTransferModel");
            in = in.setVariable("Vehicle", vehicle);
            in = in.setVariable("Simulation", simulation);
            in = in.setExternalInput(dataset);
            in = in.setModelParameter("StopTime", "0.01", ...
                "SaveOutput", "on", "OutputSaveName", "yout", ...
                "SaveFormat", "Dataset", "ReturnWorkspaceOutputs", "on");

            out = sim(in);
            signal = out.yout{1}.Values;
            normalLoad = reshape(getdatasamples(signal, numel(signal.Time)), 4, 1);

            [xCorner, yCorner] = LoadTransferConservationTest.wheelPositions();
            testCase.verifyEqual(sum(normalLoad), ...
                320.0 * 9.80665 + 250.0, AbsTol = 1.0e-9);
            testCase.verifyEqual(dot(yCorner, normalLoad), ...
                -320.0 * lateralAcceleration * 0.30, AbsTol = 1.0e-9);
            testCase.verifyEqual(dot(xCorner, normalLoad), ...
                -320.0 * 2.0 * 0.30 + 0.847 * 100.0 - 0.693 * 150.0, ...
                AbsTol = 1.0e-9);
            testCase.verifyGreaterThanOrEqual(normalLoad, zeros(4, 1));
        end

        function testAllocatorBalancesRollMoment(testCase, lateralAcceleration)
            normalLoad = unifiedControlEstimateWheelConstraints( ...
                10.0, 10.0 / 0.2286 * ones(4, 1), 2.0, ...
                lateralAcceleration, true(4, 1), false, false(4, 1), ...
                320.0, 9.80665, 1.540, 0.30, 0.45, 0.55, ...
                1.20, 1.16, 0.2286, 0.10, 2.0, 0.0, 0.12, 0.08, 0.25);

            [xCorner, yCorner] = LoadTransferConservationTest.wheelPositions();
            testCase.verifyEqual(sum(normalLoad), 320.0 * 9.80665, AbsTol = 1.0e-9);
            testCase.verifyEqual(dot(yCorner, normalLoad), ...
                -320.0 * lateralAcceleration * 0.30, AbsTol = 1.0e-9);
            testCase.verifyEqual(dot(xCorner, normalLoad), ...
                -320.0 * 2.0 * 0.30, AbsTol = 1.0e-9);
            testCase.verifyGreaterThanOrEqual(normalLoad, zeros(4, 1));
        end

        function testLiftedWheelHasNoAllocatedForceCapacity(testCase)
            [normalLoad, ~, capacity] = unifiedControlEstimateWheelConstraints( ...
                10.0, 10.0 / 0.2286 * ones(4, 1), 2.0, ...
                16.0, true(4, 1), false, false(4, 1), ...
                320.0, 9.80665, 1.540, 0.30, 0.45, 0.55, ...
                1.20, 1.16, 0.2286, 0.10, 2.0, 50.0, 0.12, 0.08, 0.25);

            testCase.verifyEqual(normalLoad(1), 0.0, AbsTol = 1.0e-9);
            testCase.verifyEqual(capacity(1), 0.0);
            testCase.verifyGreaterThan(capacity(2:4), zeros(3, 1));
        end

        function testSupportLimitCannotProduceNegativeLoads(testCase)
            frontLoad = 320.0 * 9.80665 * 0.45;
            rearLoad = 320.0 * 9.80665 * 0.55;
            normalLoad = distributeWheelNormalLoads(frontLoad, rearLoad, ...
                320.0 * 40.0 * 0.30 * 0.55 / 1.20, ...
                320.0 * 40.0 * 0.30 * 0.45 / 1.16, 1.20, 1.16);

            testCase.verifyGreaterThanOrEqual(normalLoad, zeros(4, 1));
            testCase.verifyEqual(sum(normalLoad), 320.0 * 9.80665, ...
                AbsTol = 1.0e-9);
            testCase.verifyEqual(normalLoad([1, 3]), zeros(2, 1), ...
                AbsTol = 1.0e-9);
        end
    end

    methods (Static, Access = private)
        function [vehicle, simulation] = parameters()
            values = struct("Mass", 320.0, "Wheelbase", 1.540, ...
                "CGToFrontAxle", 0.847, "CGHeight", 0.30, ...
                "TrackFront", 1.20, "TrackRear", 1.16, ...
                "RollStiffnessDistributionFront", 0.55);
            vehicle = struct;
            for name = string(fieldnames(values)).'
                vehicle.(name) = struct("Value", values.(name));
            end
            simulation = struct("Gravity", struct("Value", 9.80665));
        end

        function [xCorner, yCorner] = wheelPositions()
            xCorner = [0.847; 0.847; -0.693; -0.693];
            yCorner = [0.60; -0.60; 0.58; -0.58];
        end
    end
end
