function [wheelForce, wheelForceUnconstrained, drivePowerActive] = ...
    unifiedControlProjectWheelForces(desiredLongitudinalForce, desiredYawMoment, ...
    lowerForce, upperForce, motorSpeed, drivePowerLimit, tireRadius, ...
    gearProduct, inverterEfficiency, trackFront, trackRear, ...
    forcePriorityWeight, yawPriorityWeight, regularizationWeight, ...
    allocatorIterations, powerEpsilon)
%UnifiedControlPROJECTWHEELFORCES Fixed-iteration projected weighted allocation.
%   The four forces are in wheel coordinates ordered [FL; FR; RL; RR].

lowerForce = sanitizeVector(lowerForce, 0.0);
upperForce = sanitizeVector(upperForce, 0.0);
motorSpeed = sanitizeVector(motorSpeed, 0.0);
swapMask = lowerForce > upperForce;
midpoint = 0.5 .* (lowerForce + upperForce);
lowerForce(swapMask) = midpoint(swapMask);
upperForce(swapMask) = midpoint(swapMask);

desiredLongitudinalForce = finiteOr(desiredLongitudinalForce, 0.0);
desiredYawMoment = finiteOr(desiredYawMoment, 0.0);
safeTrackFront = max(abs(finiteOr(trackFront, 0.0)), 1.0e-6);
safeTrackRear = max(abs(finiteOr(trackRear, 0.0)), 1.0e-6);
safeRadius = max(abs(finiteOr(tireRadius, 0.0)), 1.0e-6);
safeGearProduct = max(abs(finiteOr(gearProduct, 0.0)), 1.0e-6);
efficiency = min(max(finiteOr(inverterEfficiency, 0.0), 1.0e-6), 1.0);

yawRow = [-0.5 * safeTrackFront, 0.5 * safeTrackFront, ...
    -0.5 * safeTrackRear, 0.5 * safeTrackRear];
forceScale = max([abs(desiredLongitudinalForce), ...
    sum(max(abs(lowerForce), abs(upperForce))), 1.0]);
yawScale = max([abs(desiredYawMoment), ...
    max(abs(yawRow)) * forceScale, 1.0]);

commonForce = desiredLongitudinalForce / 4.0;
yawDelta = desiredYawMoment / (safeTrackFront + safeTrackRear);
wheelForceUnconstrained = commonForce + ...
    [-yawDelta; yawDelta; -yawDelta; yawDelta];

forceRow = ones(1, 4);
normalizedYawRow = yawRow .* (forceScale / yawScale);
forceWeight = max(finiteOr(forcePriorityWeight, 0.0), 0.0);
yawWeight = max(finiteOr(yawPriorityWeight, 0.0), 0.0);
regularization = max(finiteOr(regularizationWeight, 0.0), 0.0);
targetForce = desiredLongitudinalForce / forceScale;
targetYaw = desiredYawMoment / yawScale;
targetBalanced = wheelForceUnconstrained ./ forceScale;

hessian = forceWeight .* (forceRow' * forceRow) + ...
    yawWeight .* (normalizedYawRow' * normalizedYawRow) + ...
    regularization .* eye(4);
linearTerm = forceWeight .* forceRow' .* targetForce + ...
    yawWeight .* normalizedYawRow' .* targetYaw + ...
    regularization .* targetBalanced;
stepSize = 1.0 / max(trace(hessian), 1.0e-9);

normalizedLower = lowerForce ./ forceScale;
normalizedUpper = upperForce ./ forceScale;
normalizedForce = min(max(targetBalanced, normalizedLower), ...
    normalizedUpper);
powerCoefficient = abs(motorSpeed) .* safeRadius ./ ...
    (safeGearProduct * efficiency) .* forceScale;
activeDriveLimit = max(finiteOr(drivePowerLimit, 0.0), 0.0);
safePowerEpsilon = max(abs(finiteOr(powerEpsilon, 0.0)), 1.0e-9);
iterations = max(1, min(100, double(allocatorIterations)));
drivePowerActive = false;

for iteration = 1:iterations
    gradient = hessian * normalizedForce - linearTerm;
    normalizedForce = normalizedForce - stepSize .* gradient;
    normalizedForce = min(max(normalizedForce, normalizedLower), ...
        normalizedUpper);

    positiveForce = max(normalizedForce, 0.0);
    drivePower = sum(powerCoefficient .* positiveForce);
    if drivePower > activeDriveLimit + safePowerEpsilon
        activeMask = normalizedForce > 0.0 & powerCoefficient > 0.0;
        denominator = sum(powerCoefficient(activeMask) .^ 2);
        if denominator > safePowerEpsilon * safePowerEpsilon
            correction = (drivePower - activeDriveLimit) / denominator;
            normalizedForce(activeMask) = normalizedForce(activeMask) - ...
                correction .* powerCoefficient(activeMask);
        else
            normalizedForce(activeMask) = 0.0;
        end
        normalizedForce = min(max(normalizedForce, normalizedLower), ...
            normalizedUpper);
        drivePowerActive = true;
    end
end
wheelForce = normalizedForce .* forceScale;
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
