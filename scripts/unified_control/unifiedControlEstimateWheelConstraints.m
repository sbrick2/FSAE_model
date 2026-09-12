function [normalLoad, slipRatio, tireForceCapacity, tcScale, ...
    tcStateNext, degraded] = unifiedControlEstimateWheelConstraints( ...
    longitudinalSpeed, wheelSpeed, accelX, accelY, wheelSpeedValid, ...
    enableTC, tcStatePrevious, vehicleMass, gravity, wheelbase, ...
    cgHeight, staticLoadDistributionFront, ...
    rollStiffnessDistributionFront, trackFront, trackRear, tireRadius, ...
    lowSpeedEpsilon, frictionEstimate, minimumNormalLoad, tcSlipOn, ...
    tcSlipOff, tcSlipFullCut)
%UnifiedControlESTIMATEWHEELCONSTRAINTS Estimate controller-available tire constraints.
%   Wheel order is [FL; FR; RL; RR]. No Plant tire-force truth is used.

wheelSpeed = sanitizeVector(wheelSpeed, 0.0);
wheelSpeedValid = logical(reshape(wheelSpeedValid, 4, 1));
tcStatePrevious = logical(reshape(tcStatePrevious, 4, 1));
longitudinalSpeed = finiteOr(longitudinalSpeed, 0.0);
accelX = finiteOr(accelX, 0.0);
accelY = finiteOr(accelY, 0.0);

safeMass = max(abs(finiteOr(vehicleMass, 0.0)), 1.0e-6);
safeGravity = max(abs(finiteOr(gravity, 0.0)), 1.0e-6);
safeWheelbase = max(abs(finiteOr(wheelbase, 0.0)), 1.0e-6);
safeTrackFront = max(abs(finiteOr(trackFront, 0.0)), 1.0e-6);
safeTrackRear = max(abs(finiteOr(trackRear, 0.0)), 1.0e-6);
safeRadius = max(abs(finiteOr(tireRadius, 0.0)), 1.0e-6);
safeSpeed = max(abs(finiteOr(lowSpeedEpsilon, 0.0)), 1.0e-3);
frontFraction = clamp(finiteOr(staticLoadDistributionFront, 0.5), ...
    0.05, 0.95);
rollFrontFraction = clamp(finiteOr( ...
    rollStiffnessDistributionFront, 0.5), 0.0, 1.0);
safeCGHeight = max(finiteOr(cgHeight, 0.0), 0.0);

speedDenominator = max(abs(longitudinalSpeed), safeSpeed);
lowSpeedBlend = clamp(abs(longitudinalSpeed) / safeSpeed, 0.0, 1.0);
slipRatio = lowSpeedBlend .* ...
    (safeRadius .* wheelSpeed - longitudinalSpeed) ./ speedDenominator;
slipRatio(~wheelSpeedValid) = 0.0;

totalLoad = safeMass * safeGravity;
longitudinalTransfer = safeMass * accelX * safeCGHeight / safeWheelbase;
frontAxleLoad = frontFraction * totalLoad - longitudinalTransfer;
rearAxleLoad = (1.0 - frontFraction) * totalLoad + longitudinalTransfer;
frontLateralTransfer = safeMass * accelY * safeCGHeight * ...
    rollFrontFraction / safeTrackFront;
rearLateralTransfer = safeMass * accelY * safeCGHeight * ...
    (1.0 - rollFrontFraction) / safeTrackRear;

normalLoad = [0.5 * (frontAxleLoad - frontLateralTransfer); ...
    0.5 * (frontAxleLoad + frontLateralTransfer); ...
    0.5 * (rearAxleLoad - rearLateralTransfer); ...
    0.5 * (rearAxleLoad + rearLateralTransfer)];
loadFloor = max(finiteOr(minimumNormalLoad, 0.0), 0.0);
normalLoad = max(normalLoad, loadFloor);
normalLoad = normalLoad .* (totalLoad / max(sum(normalLoad), 1.0e-6));

mu = max(finiteOr(frictionEstimate, 0.0), 0.0);
lateralUtilization = abs(accelY) / safeGravity;
longitudinalMuAvailable = sqrt(max(mu * mu - ...
    lateralUtilization * lateralUtilization, 0.0));
tireForceCapacity = longitudinalMuAvailable .* normalLoad;

slipOn = max(finiteOr(tcSlipOn, 0.0), 0.0);
slipOff = clamp(finiteOr(tcSlipOff, 0.0), 0.0, slipOn);
slipFullCut = max(finiteOr(tcSlipFullCut, slipOn), ...
    slipOn + 1.0e-6);
tcStateNext = false(4, 1);
tcScale = ones(4, 1);
for wheelIndex = 1:4
    if ~wheelSpeedValid(wheelIndex)
        tcStateNext(wheelIndex) = enableTC;
        if enableTC
            tcScale(wheelIndex) = 0.0;
        end
    elseif enableTC
        state = tcStatePrevious(wheelIndex);
        if state
            state = slipRatio(wheelIndex) > slipOff;
        else
            state = slipRatio(wheelIndex) >= slipOn;
        end
        tcStateNext(wheelIndex) = state;
        if state
            tcScale(wheelIndex) = clamp((slipFullCut - ...
                slipRatio(wheelIndex)) / (slipFullCut - slipOn), ...
                0.0, 1.0);
        end
    end
end
degraded = any(~wheelSpeedValid);
end

function vector = sanitizeVector(vector, fallback)
vector = reshape(vector, 4, 1);
for index = 1:4
    vector(index) = finiteOr(vector(index), fallback);
end
end

function value = finiteOr(value, fallback)
if ~isfinite(value)
    value = fallback;
end
end

function value = clamp(value, lower, upper)
value = min(max(value, lower), upper);
end
