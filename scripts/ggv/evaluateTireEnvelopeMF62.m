function envelopes = evaluateTireEnvelopeMF62( ...
        config, normalLoad, roadGripScale, roadMuLimit, camber)
%EVALUATETIREENVELOPEMF62 Build exact MF62 directional-support envelopes.

arguments
    config (1, 1) struct
    normalLoad (4, 1) double
    roadGripScale (4, 1) double
    roadMuLimit (4, 1) double
    camber (4, 1) double
end

if config.Tire.Model ~= "MF62"
    error("FSAE:QuasiStatic:TireEnvelope", ...
        "Speed-dependent GGV requires longitudinal and combined-slip " + ...
        "capability. The selected TTC Map contains zero-slip lateral data only.");
end

kappa = config.Tire.KappaGrid(:);
alpha = config.Tire.AlphaGrid(:);
kappaMatrix = repmat(kappa, 1, 4);
alphaMatrix = repmat(alpha, 1, 4);
zeroKappa = zeros(size(alphaMatrix));
zeroAlpha = zeros(size(kappaMatrix));
normalKappa = repmat(normalLoad', numel(kappa), 1);
normalAlpha = repmat(normalLoad', numel(alpha), 1);
gripKappa = ones(size(kappaMatrix));
gripAlpha = ones(size(alphaMatrix));
limitKappa = inf(size(kappaMatrix));
limitAlpha = inf(size(alphaMatrix));
camberKappa = repmat(camber', numel(kappa), 1);
camberAlpha = repmat(camber', numel(alpha), 1);
[fxPure, ~] = evaluateTireMF62(kappaMatrix, zeroAlpha, camberKappa, ...
    normalKappa, gripKappa, limitKappa, config.Tire.Profile);
[~, fyPure] = evaluateTireMF62(zeroKappa, alphaMatrix, camberAlpha, ...
    normalAlpha, gripAlpha, limitAlpha, config.Tire.Profile);
fxSpan = (max(fxPure, [], 1) - min(fxPure, [], 1))';
fySpan = (max(fyPure, [], 1) - min(fyPure, [], 1))';
meaningfulGrip = normalLoad > 1.0e-6 & roadGripScale > 0.0 & ...
    (~isfinite(roadMuLimit) | roadMuLimit > 0.0);
minimumCapacity = 1.0e-6 .* normalLoad;
invalidFx = ~isfinite(fxSpan) | ...
    (meaningfulGrip & fxSpan <= minimumCapacity);
invalidFy = ~isfinite(fySpan) | ...
    (meaningfulGrip & fySpan <= minimumCapacity);
if any(invalidFx) || any(invalidFy)
    error("FSAE:QuasiStatic:TireEnvelope", ...
        "Tire model does not provide both longitudinal and lateral capacity. " + ...
        "Use the TireModel MF62-equivalent profile for GGV generation; the 43075 TTC Map has zero-slip data only.");
end

combined = config.Tire.Profile.Combined;
alphaRatio = abs(alpha) ./ combined.AlphaScaleRad;
kappaRatio = abs(kappa) ./ combined.KappaScale;
baseScaleX = (1.0 + alphaRatio .^ combined.Exponent) .^ ...
    (-1.0 / combined.Exponent);
baseScaleY = (1.0 + kappaRatio .^ combined.Exponent) .^ ...
    (-1.0 / combined.Exponent);
combinedScaleX = baseScaleX .^ combined.LongitudinalScaleExponent;
combinedScaleY = baseScaleY .^ combined.LateralScaleExponent;

pointCount = config.Solver.EnvelopePointCount;
theta = (0:pointCount - 1)' * (2.0 * pi / pointCount);
directions = [cos(theta), sin(theta)];
template = struct("Directions", directions, ...
    "Support", zeros(pointCount, 1), "FxMax", 0.0, "FyMax", 0.0);
envelopes = repmat(template, 4, 1);
for wheel = 1:4
    forceX = fxPure(:, wheel) * combinedScaleX';
    forceY = combinedScaleY * fyPure(:, wheel)';
    sampleSize = size(forceX);
    [forceX, forceY] = applyRoadGripAndLimit(forceX, forceY, ...
        normalLoad(wheel) .* ones(sampleSize), ...
        roadGripScale(wheel) .* ones(sampleSize), ...
        roadMuLimit(wheel) .* ones(sampleSize));
    forcePoints = [forceX(:), forceY(:); 0.0, 0.0];
    support = max(forcePoints * directions', [], 1)';
    assert(all(isfinite(support)) && all(support >= 0.0), ...
        "FSAE:QuasiStatic:TireEnvelope", ...
        "The MF62 combined-slip support must be finite and nonnegative.");
    envelopes(wheel).Support = support;
    envelopes(wheel).FxMax = max(abs(forcePoints(:, 1)));
    envelopes(wheel).FyMax = max(abs(forcePoints(:, 2)));
end
end
