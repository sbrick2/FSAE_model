classdef ControlFrictionTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function initializeProject(~)
            initProject();
        end
    end

    methods (Test, TestTags = {'Unit'})
        function testRoadGripScaleChangesEstimate(testCase)
            load = 780.0 * ones(4, 1);
            nominal = estimateMF62ControlFriction(load, ones(4, 1), inf(4, 1));

            reduced = estimateMF62ControlFriction(load, 0.5 * ones(4, 1), inf(4, 1));

            testCase.verifyGreaterThan(nominal, 1.0);
            testCase.verifyEqual(reduced, 0.5 * nominal, AbsTol = 1.0e-12);
        end

        function testAbsoluteRoadLimitCapsEstimate(testCase)
            load = 780.0 * ones(4, 1);

            friction = estimateMF62ControlFriction(load, ones(4, 1), 0.7 * ones(4, 1));

            testCase.verifyLessThanOrEqual(friction, 0.7 + 1.0e-12);
            testCase.verifyGreaterThan(friction, 0.65);
        end

        function testWeakestWheelSetsSharedEstimate(testCase)
            load = 780.0 * ones(4, 1);

            friction = estimateMF62ControlFriction(load, ...
                [1.0; 0.5; 1.0; 1.0], inf(4, 1));
            reduced = estimateMF62ControlFriction(load, 0.5 * ones(4, 1), inf(4, 1));

            testCase.verifyEqual(friction, reduced, AbsTol = 1.0e-12);
        end

        function testZeroGripInhibitsForceCapacity(testCase)
            load = 780.0 * ones(4, 1);

            friction = estimateMF62ControlFriction(load, [1.0; 0.0; 1.0; 1.0], inf(4, 1));

            testCase.verifyEqual(friction, 0.0);
        end
    end
end
