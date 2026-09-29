function friction = estimateMF62ControlFriction( ...
        normalLoad, roadGripScale, roadMuLimit, profile)
%ESTIMATEMF62CONTROLFRICTION Estimate shared controller grip from tire data.
%   Use the weaker pure-slip direction at each static wheel load. The
%   estimate includes configured road grip and the absolute road limit,
%   and uses no plant force or dynamic wheel-load measurements.
arguments
    normalLoad (4, 1) double {mustBePositive, mustBeFinite}
    roadGripScale (4, 1) double {mustBeNonnegative, mustBeFinite}
    roadMuLimit (4, 1) double {mustBeNonnegative}
    profile (1, 1) struct = getSelectedTireProfile()
end
assert(all(~isnan(roadMuLimit)), "FSAE:Control:RoadMuLimit", ...
    "RoadMuLimit must not contain NaN.");
slip = linspace(-0.40, 0.40, 81).';
slipGrid = repmat(slip, 1, 4);
zeroGrid = zeros(size(slipGrid));
loadGrid = repmat(normalLoad.', numel(slip), 1);
gripGrid = repmat(roadGripScale.', numel(slip), 1);
limitGrid = repmat(roadMuLimit.', numel(slip), 1);
[forceX, ~] = evaluateTireMF62(slipGrid, zeroGrid, zeroGrid, ...
    loadGrid, gripGrid, limitGrid, profile);
[~, forceY] = evaluateTireMF62(zeroGrid, slipGrid, zeroGrid, ...
    loadGrid, gripGrid, limitGrid, profile);
directionalCapacity = [max(forceX, [], 1); -min(forceX, [], 1); ...
    max(forceY, [], 1); -min(forceY, [], 1)];
wheelFriction = min(directionalCapacity, [], 1) ./ normalLoad.';
friction = max(0.0, min(wheelFriction));
assert(isfinite(friction), "FSAE:Control:TireFriction", ...
    "The tire model must provide a finite control friction estimate.");
end
