function output = evaluateTTCMapVector(input, profile)
%EVALUATETTCMAPVECTOR Vector adapter for the Simulink interpreted block.

arguments
    input (24, 1) double
    profile (1, 1) struct = getSelectedTireProfile()
end

[forceX, forceY, utilization, saturationScale] = evaluateTTCMap( ...
    input(1:4), input(5:8), input(9:12), input(13:16), ...
    input(17:20), input(21:24), profile);
output = [forceX; forceY; utilization; saturationScale];
end
