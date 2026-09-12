function [forceX, forceY, utilization, saturationScale] = ...
        evaluateTireMF62(slipRatio, slipAngle, camberAngle, ...
        normalLoad, roadGripScale, roadMuLimit, profile)
%EVALUATETIREMF62 Evaluate the TireModel steady-state MF6.2-equivalent model.
%   This model uses Pacejka-form pure-slip curves with identified load,
%   pressure and camber dependencies, followed by an identified generalized
%   friction-ellipse combined-slip reduction. It is not a complete
%   coefficient-for-coefficient implementation of the MF-Tyre 6.2 standard.

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
    "FSAE:TireModel:MFInputSize", "All tire inputs must have equal size.");
roadGripScale = expandRoadInput(roadGripScale, targetSize, ...
    "RoadGripScale");
roadMuLimit = expandRoadInput(roadMuLimit, targetSize, "RoadMuLimit");

fz = max(normalLoad, 0.0);
gripScale = max(roadGripScale, 0.0);
muLimit = max(roadMuLimit, 0.0);
referenceLoad = profile.Reference.NormalLoadN;
referencePressure = profile.Reference.PressurePa;
loadDelta = (fz - referenceLoad) ./ max(referenceLoad, 1.0);
pressureDelta = (profile.InputPressurePa + 0.0 .* fz - ...
    referencePressure) ./ max(referencePressure, 1.0);
if isfield(profile, "UseRuntimePressure") && profile.UseRuntimePressure
    error("FSAE:TireModel:RuntimePressureInterface", ...
        "The frozen TireModel tire interface has no runtime pressure input.");
end

longitudinal = profile.Longitudinal;
muX = longitudinal.Mu0 + longitudinal.MuLoad .* loadDelta + ...
    longitudinal.MuPressure .* pressureDelta + ...
    longitudinal.MuCamber2 .* camberAngle.^2 + ...
    longitudinal.MuAsymmetry .* tanh(50.0 .* slipRatio);
muX = max(muX, 0.05);
x = slipRatio + longitudinal.HorizontalShift;
bx = longitudinal.B .* x;
pureX = fz .* muX .* sin(longitudinal.C .* atan( ...
    bx - longitudinal.E .* (bx - atan(bx)))) + ...
    fz .* longitudinal.VerticalShift;

lateral = profile.Lateral;
muY = lateral.Mu0 + lateral.MuLoad .* loadDelta + ...
    lateral.MuPressure .* pressureDelta + ...
    lateral.MuCamber2 .* camberAngle.^2;
muY = max(muY, 0.05);
alpha = slipAngle + lateral.HorizontalShift + ...
    lateral.CamberShift .* camberAngle;
by = lateral.B .* alpha;
pureY = -fz .* muY .* sin(lateral.C .* atan( ...
    by - lateral.E .* (by - atan(by)))) + ...
    fz .* (lateral.VerticalShift + ...
    lateral.CamberVerticalShift .* camberAngle);

combined = profile.Combined;
alphaRatio = abs(slipAngle) ./ combined.AlphaScaleRad;
kappaRatio = abs(slipRatio) ./ combined.KappaScale;
baseScaleX = (1.0 + alphaRatio .^ combined.Exponent) .^ ...
    (-1.0 / combined.Exponent);
baseScaleY = (1.0 + kappaRatio .^ combined.Exponent) .^ ...
    (-1.0 / combined.Exponent);
scaleX = baseScaleX .^ combined.LongitudinalScaleExponent;
scaleY = baseScaleY .^ combined.LateralScaleExponent;

combinedX = pureX .* scaleX;
combinedY = pureY .* scaleY;
[forceX, forceY, limitScale] = applyRoadGripAndLimit( ...
    combinedX, combinedY, fz, gripScale, muLimit);

peakMuX = max(abs(muX) + abs(longitudinal.VerticalShift), 0.05);
lateralShift = lateral.VerticalShift + ...
    lateral.CamberVerticalShift .* camberAngle;
peakMuY = max(abs(muY) + abs(lateralShift), 0.05);
capacityX = gripScale .* fz .* peakMuX;
capacityY = gripScale .* fz .* peakMuY;
modelUtilization = hypot( ...
    forceX ./ max(capacityX, 1.0), ...
    forceY ./ max(capacityY, 1.0));
limitUtilization = zeros(targetSize);
finiteLimit = isfinite(muLimit);
limitAvailable = zeros(targetSize);
limitAvailable(finiteLimit) = muLimit(finiteLimit) .* fz(finiteLimit);
limitUtilization(finiteLimit) = hypot( ...
    forceX(finiteLimit), forceY(finiteLimit)) ./ ...
    max(limitAvailable(finiteLimit), 1.0);
utilization = min(1.0, max(modelUtilization, limitUtilization));
saturationScale = min(min(scaleX, scaleY), limitScale);

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
