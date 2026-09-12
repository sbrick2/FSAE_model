function output = evaluateTTCMapCodegen(input, points, forces, scale, ...
        lowerBound, upperBound, inputPressure)
%EVALUATETTCMAPCODEGEN Code-generation implementation of the TireModel TTC map.
%#codegen

slipRatio = input(1:4);
slipAngle = input(5:8);
camberAngle = input(9:12);
normalLoad = input(13:16);
roadGripScale = input(17:20);
roadMuLimit = input(21:24);
forceX = zeros(4, 1);
forceY = zeros(4, 1);

for tireIndex = 1:4
    query = [ ...
        slipRatio(tireIndex), slipAngle(tireIndex), ...
        camberAngle(tireIndex), normalLoad(tireIndex), inputPressure];
    query = min(max(query, lowerBound), upperBound);
    biasQuery = [0.0, 0.0, 0.0, normalLoad(tireIndex), inputPressure];
    biasQuery = min(max(biasQuery, lowerBound), upperBound);
    [rawForceX, rawForceY] = interpolateLocalForce(query, points, ...
        forces, scale);
    [biasForceX, biasForceY] = interpolateLocalForce(biasQuery, points, ...
        forces, scale);
    forceX(tireIndex) = rawForceX - biasForceX;
    forceY(tireIndex) = rawForceY - biasForceY;
end

fz = max(normalLoad, 0.0);
gripScale = max(roadGripScale, 0.0);
muLimit = max(roadMuLimit, 0.0);
[forceX, forceY, saturationScale] = applyRoadGripAndLimit( ...
    forceX, forceY, fz, gripScale, muLimit);
mapPeakMu = 0.05;
for pointIndex = 1:size(points, 1)
    if points(pointIndex, 4) > 1.0
        pointMu = hypot(forces(pointIndex, 1), forces(pointIndex, 2)) / ...
            points(pointIndex, 4);
        mapPeakMu = max(mapPeakMu, pointMu);
    end
end
modelAvailable = gripScale .* fz .* max(mapPeakMu, 0.05);
modelUtilization = hypot(forceX, forceY) ./ max(modelAvailable, 1.0);
finiteLimit = isfinite(muLimit);
limitAvailable = zeros(4, 1);
limitAvailable(finiteLimit) = muLimit(finiteLimit) .* fz(finiteLimit);
limitUtilization = zeros(4, 1);
limitUtilization(finiteLimit) = hypot( ...
    forceX(finiteLimit), forceY(finiteLimit)) ./ ...
    max(limitAvailable(finiteLimit), 1.0);
utilization = min(1.0, max(modelUtilization, limitUtilization));
active = double(fz > 0.0 & gripScale > 0.0 & ...
    (~finiteLimit | muLimit > 0.0));
forceX = forceX .* active;
forceY = forceY .* active;
utilization = utilization .* active;
saturationScale = saturationScale .* active;
output = [forceX; forceY; utilization; saturationScale];
end

function [forceX, forceY] = interpolateLocalForce(query, points, forces, ...
        scale)
distanceSquared = zeros(size(points, 1), 1);
for pointIndex = 1:size(points, 1)
    pointDistance = 0.0;
    for dimensionIndex = 1:5
        delta = (points(pointIndex, dimensionIndex) - ...
            query(dimensionIndex)) / scale(dimensionIndex);
        pointDistance = pointDistance + delta * delta;
    end
    distanceSquared(pointIndex) = pointDistance;
end
[nearestDistance, nearestIndex] = mink(distanceSquared, 8);
weightSum = 0.0;
weightedForceX = 0.0;
weightedForceY = 0.0;
for neighborIndex = 1:8
    weight = 1.0 / max(nearestDistance(neighborIndex), 1.0e-10);
    weightSum = weightSum + weight;
    weightedForceX = weightedForceX + ...
        weight * forces(nearestIndex(neighborIndex), 1);
    weightedForceY = weightedForceY + ...
        weight * forces(nearestIndex(neighborIndex), 2);
end
forceX = weightedForceX / weightSum;
forceY = weightedForceY / weightSum;
end
