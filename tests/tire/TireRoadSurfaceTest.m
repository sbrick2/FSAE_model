classdef TireRoadSurfaceTest < matlab.unittest.TestCase
    %TIREROADSURFACETEST Verify road scaling and optional absolute limiting.

    methods (TestClassSetup)
        function addTirePath(testCase)
            projectRoot = fileparts(fileparts(fileparts( ...
                mfilename("fullpath"))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(projectRoot, "scripts", "tire")));
        end
    end

    methods (Test)
        function testUnitGripScalePreservesReferenceMF62(testCase)
            profile = getSelectedTireProfile();
            [slipRatio, slipAngle, camberAngle, normalLoad] = ...
                TireRoadSurfaceTest.representativeInputs();

            [forceX, forceY] = evaluateTireMF62( ...
                slipRatio, slipAngle, camberAngle, normalLoad, ...
                1.0, Inf, profile);
            [scaledX, scaledY] = evaluateTireMF62( ...
                slipRatio, slipAngle, camberAngle, normalLoad, ...
                0.8, Inf, profile);

            testCase.verifyEqual(scaledX, 0.8 .* forceX, ...
                AbsTol = 1.0e-10);
            testCase.verifyEqual(scaledY, 0.8 .* forceY, ...
                AbsTol = 1.0e-10);
            testCase.verifyGreaterThan(max(hypot(forceX, forceY) ./ ...
                normalLoad), 1.0);
        end

        function testFiniteAbsoluteLimitCapsForceMagnitude(testCase)
            profile = getSelectedTireProfile();
            [slipRatio, slipAngle, camberAngle, normalLoad] = ...
                TireRoadSurfaceTest.representativeInputs();

            [forceX, forceY, utilization, saturationScale] = ...
                evaluateTireMF62(slipRatio, slipAngle, camberAngle, ...
                normalLoad, 1.0, 1.0, profile);

            testCase.verifyLessThanOrEqual(hypot(forceX, forceY), ...
                normalLoad + 1.0e-9);
            testCase.verifyEqual(max(utilization), 1.0, ...
                AbsTol = 1.0e-12);
            testCase.verifyLessThan(min(saturationScale), 1.0);
        end

        function testVectorAdapterMatchesDirectEvaluator(testCase)
            profile = getSelectedTireProfile();
            [slipRatio, slipAngle, camberAngle, normalLoad] = ...
                TireRoadSurfaceTest.representativeInputs();
            input = [slipRatio; slipAngle; camberAngle; normalLoad; ...
                0.9 .* ones(4, 1); inf(4, 1)];

            direct = evaluateTireMF62Vector(input, profile);
            codegen = evaluateTireMF62Codegen(input, ...
                serializeTireModelMFParameters(profile));

            testCase.verifyEqual(codegen, direct, AbsTol = 1.0e-10);
        end
    end

    methods (Static, Access = private)
        function [slipRatio, slipAngle, camberAngle, normalLoad] = ...
                representativeInputs()
            slipRatio = [0.15; 0.12; -0.15; -0.12];
            slipAngle = deg2rad([6; -5; 6; -5]);
            camberAngle = zeros(4, 1);
            normalLoad = 780.0 .* ones(4, 1);
        end
    end
end
