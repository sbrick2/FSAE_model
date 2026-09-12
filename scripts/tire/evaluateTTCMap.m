function [forceX, forceY, utilization, saturationScale, ...
        outOfDomain] = evaluateTTCMap(slipRatio, slipAngle, camberAngle, ...
        normalLoad, roadGripScale, roadMuLimit, profile)
%EVALUATETTCMAP Evaluate the compact TTC reference map by local IDW.

arguments
    slipRatio double
    slipAngle double
    camberAngle double
    normalLoad double
    roadGripScale double
    roadMuLimit double
    profile (1, 1) struct
end

targetSize = size(slipRatio);
assert(isequal(size(slipAngle), targetSize) && ...
    isequal(size(camberAngle), targetSize) && ...
    isequal(size(normalLoad), targetSize), ...
    "FSAE:TireModel:TTCMapInputSize", ...
    "Slip, camber, and normal-load inputs must have equal size.");
roadGripScale = expandRoadInput(roadGripScale, targetSize, ...
    "RoadGripScale");
roadMuLimit = expandRoadInput(roadMuLimit, targetSize, "RoadMuLimit");
query = [slipRatio(:), slipAngle(:), camberAngle(:), ...
    normalLoad(:), repmat(profile.InputPressurePa, numel(slipRatio), 1)];
map = profile.Map;
outOfDomain = any(query < map.LowerBound | query > map.UpperBound, 2);
query = min(max(query, map.LowerBound), map.UpperBound);
biasQuery = query;
biasQuery(:, 1:3) = 0.0;
biasQuery = min(max(biasQuery, map.LowerBound), map.UpperBound);

force = zeros(size(query, 1), 2);
neighborCount = min(double(map.NeighborCount), size(map.Points, 1));
normalizedPoints = map.Points ./ map.Scale;
for index = 1:size(query, 1)
    normalizedQuery = query(index, :) ./ map.Scale;
    normalizedBiasQuery = biasQuery(index, :) ./ map.Scale;
    force(index, :) = interpolateLocalForce(normalizedQuery, ...
        normalizedPoints, map.Forces, neighborCount) - ...
        interpolateLocalForce(normalizedBiasQuery, normalizedPoints, ...
        map.Forces, neighborCount);
end

fz = max(normalLoad(:), 0.0);
gripScale = max(roadGripScale(:), 0.0);
muLimit = max(roadMuLimit(:), 0.0);
[forceX, forceY, saturationScale] = applyRoadGripAndLimit( ...
    force(:, 1), force(:, 2), fz, gripScale, muLimit);

validLoad = map.Points(:, 4) > 1.0;
mapPeakMu = max(hypot(map.Forces(validLoad, 1), ...
    map.Forces(validLoad, 2)) ./ map.Points(validLoad, 4));
modelAvailable = gripScale .* fz .* max(mapPeakMu, 0.05);
modelUtilization = hypot(forceX, forceY) ./ max(modelAvailable, 1.0);
finiteLimit = isfinite(muLimit);
limitAvailable = zeros(size(fz));
limitAvailable(finiteLimit) = muLimit(finiteLimit) .* fz(finiteLimit);
limitUtilization = zeros(size(fz));
limitUtilization(finiteLimit) = hypot( ...
    forceX(finiteLimit), forceY(finiteLimit)) ./ ...
    max(limitAvailable(finiteLimit), 1.0);
utilization = min(1.0, max(modelUtilization, limitUtilization));
unloaded = fz <= 0.0 | gripScale <= 0.0 | ...
    (finiteLimit & muLimit <= 0.0);
forceX(unloaded) = 0.0;
forceY(unloaded) = 0.0;
utilization(unloaded) = 0.0;
saturationScale(unloaded) = 0.0;

forceX = reshape(forceX, targetSize);
forceY = reshape(forceY, targetSize);
utilization = reshape(utilization, targetSize);
saturationScale = reshape(saturationScale, targetSize);
outOfDomain = reshape(outOfDomain, targetSize);
end

function value = expandRoadInput(value, targetSize, inputName)
if isscalar(value)
    value = repmat(value, targetSize);
else
    assert(isequal(size(value), targetSize), ...
        "FSAE:TireModel:" + inputName + "Size", ...
        inputName + " must be scalar or match the tire input size.");
end
end

function force = interpolateLocalForce(normalizedQuery, normalizedPoints, ...
        mapForces, neighborCount)
distanceSquared = sum((normalizedPoints - normalizedQuery).^2, 2);
[nearestDistance, nearestIndex] = mink(distanceSquared, neighborCount);
weights = 1.0 ./ max(nearestDistance, 1.0e-10);
weights = weights ./ sum(weights);
force = weights' * mapForces(nearestIndex, :);
end
