function output = evaluateTireMF62Codegen(input, parameters)
%EVALUATETIREMF62CODEGEN Code-generation implementation of the TireModel MF model.
%#codegen

slipRatio = input(1:4);
slipAngle = input(5:8);
camberAngle = input(9:12);
normalLoad = input(13:16);
roadGripScale = input(17:20);
roadMuLimit = input(21:24);

fz = max(normalLoad, 0.0);
gripScale = max(roadGripScale, 0.0);
muLimit = max(roadMuLimit, 0.0);
loadDelta = (fz - parameters(2)) ./ max(parameters(2), 1.0);
pressureDelta = (parameters(1) - parameters(3)) ./ ...
    max(parameters(3), 1.0);

muX = parameters(7) + parameters(8) .* loadDelta + ...
    parameters(9) .* pressureDelta + ...
    parameters(10) .* camberAngle.^2 + ...
    parameters(11) .* tanh(50.0 .* slipRatio);
muX = max(muX, 0.05);
x = slipRatio + parameters(12);
bx = parameters(4) .* x;
pureX = fz .* muX .* sin(parameters(5) .* atan( ...
    bx - parameters(6) .* (bx - atan(bx)))) + ...
    fz .* parameters(13);

muY = parameters(17) + parameters(18) .* loadDelta + ...
    parameters(19) .* pressureDelta + ...
    parameters(20) .* camberAngle.^2;
muY = max(muY, 0.05);
alpha = slipAngle + parameters(21) + parameters(22) .* camberAngle;
by = parameters(14) .* alpha;
pureY = -fz .* muY .* sin(parameters(15) .* atan( ...
    by - parameters(16) .* (by - atan(by)))) + ...
    fz .* (parameters(23) + parameters(24) .* camberAngle);

alphaRatio = abs(slipAngle) ./ parameters(25);
kappaRatio = abs(slipRatio) ./ parameters(26);
baseScaleX = (1.0 + alphaRatio .^ parameters(27)) .^ ...
    (-1.0 / parameters(27));
baseScaleY = (1.0 + kappaRatio .^ parameters(27)) .^ ...
    (-1.0 / parameters(27));
scaleX = baseScaleX .^ parameters(28);
scaleY = baseScaleY .^ parameters(29);

combinedX = pureX .* scaleX;
combinedY = pureY .* scaleY;
[forceX, forceY, limitScale] = applyRoadGripAndLimit( ...
    combinedX, combinedY, fz, gripScale, muLimit);

peakMuX = max(abs(muX) + abs(parameters(13)), 0.05);
lateralShift = parameters(23) + parameters(24) .* camberAngle;
peakMuY = max(abs(muY) + abs(lateralShift), 0.05);
capacityX = gripScale .* fz .* peakMuX;
capacityY = gripScale .* fz .* peakMuY;
modelUtilization = hypot( ...
    forceX ./ max(capacityX, 1.0), ...
    forceY ./ max(capacityY, 1.0));
finiteLimit = isfinite(muLimit);
limitAvailable = zeros(4, 1);
limitAvailable(finiteLimit) = muLimit(finiteLimit) .* fz(finiteLimit);
limitUtilization = zeros(4, 1);
limitUtilization(finiteLimit) = hypot( ...
    forceX(finiteLimit), forceY(finiteLimit)) ./ ...
    max(limitAvailable(finiteLimit), 1.0);
utilization = min(1.0, max(modelUtilization, limitUtilization));
saturationScale = min(min(scaleX, scaleY), limitScale);

active = double(fz > 0.0 & gripScale > 0.0 & ...
    (~finiteLimit | muLimit > 0.0));
forceX = forceX .* active;
forceY = forceY .* active;
utilization = utilization .* active;
saturationScale = saturationScale .* active;
output = [forceX; forceY; utilization; saturationScale];
end
