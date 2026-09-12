function [forceX, forceY, limitScale] = applyRoadGripAndLimit( ...
        rawForceX, rawForceY, normalLoad, roadGripScale, roadMuLimit)
%APPLYROADGRIPANDLIMIT Scale a reference tire model and apply an optional cap.
%   ROADGRIPSCALE is relative to the reference surface represented by the
%   tire data. ROADMULIMIT is an optional absolute friction-circle limit;
%   Inf disables that additional limit without removing the tire model's
%   own force boundary.

targetSize = size(rawForceX);
assert(isequal(size(rawForceY), targetSize) && ...
    isequal(size(normalLoad), targetSize), ...
    "FSAE:TireModel:RoadLimitInputSize", ...
    "Raw tire forces and normal load must have equal size.");
roadGripScale = expandRoadInput(roadGripScale, targetSize, ...
    "RoadGripScale");
roadMuLimit = expandRoadInput(roadMuLimit, targetSize, "RoadMuLimit");

normalLoad = max(normalLoad, 0.0);
roadGripScale = max(roadGripScale, 0.0);
roadMuLimit = max(roadMuLimit, 0.0);

forceX = rawForceX .* roadGripScale;
forceY = rawForceY .* roadGripScale;
limitScale = ones(size(forceX));

finiteLimit = isfinite(roadMuLimit);
forceMagnitude = hypot(forceX, forceY);
available = zeros(size(forceMagnitude));
available(finiteLimit) = roadMuLimit(finiteLimit) .* ...
    normalLoad(finiteLimit);
limitScale(finiteLimit) = min(1.0, available(finiteLimit) ./ ...
    max(forceMagnitude(finiteLimit), 1.0));

forceX = forceX .* limitScale;
forceY = forceY .* limitScale;
inactive = normalLoad <= 0.0 | roadGripScale <= 0.0 | ...
    (finiteLimit & roadMuLimit <= 0.0);
forceX(inactive) = 0.0;
forceY(inactive) = 0.0;
limitScale(inactive) = 0.0;
end

function value = expandRoadInput(value, targetSize, inputName)
if isscalar(value)
    value = repmat(value, targetSize);
else
    assert(isequal(size(value), targetSize), ...
        "FSAE:TireModel:" + inputName + "Size", ...
        inputName + " must be scalar or match the tire-force size.");
end
end
