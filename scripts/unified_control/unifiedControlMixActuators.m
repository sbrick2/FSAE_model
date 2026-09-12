function [motorTorque, frictionBrakeTorque, regenTorque, totalPower, ...
    regenPowerActive, actuatorLimited, actualWheelForce] = ...
    unifiedControlMixActuators(wheelForceTarget, motorSpeed, motorTorqueLower, ...
    motorTorqueUpper, tireForceCapacity, ...
    enableRegen, regenPowerLimit, brakeMaxTorque, tireRadius, ...
    gearProduct, inverterEfficiency, powerEpsilon)
%UnifiedControlMIXACTUATORS Mix motor regeneration and fixed-proportion friction brake.

wheelForceTarget = sanitizeVector(wheelForceTarget, 0.0);
motorSpeed = sanitizeVector(motorSpeed, 0.0);
motorTorqueLower = sanitizeVector(motorTorqueLower, 0.0);
motorTorqueUpper = sanitizeVector(motorTorqueUpper, 0.0);
tireForceCapacity = max(sanitizeVector(tireForceCapacity, 0.0), 0.0);
brakeMaxTorque = max(sanitizeVector(brakeMaxTorque, 0.0), 0.0);

safeRadius = max(abs(finiteOr(tireRadius, 0.0)), 1.0e-6);
safeGearProduct = max(abs(finiteOr(gearProduct, 0.0)), 1.0e-6);
efficiency = min(max(finiteOr(inverterEfficiency, 0.0), 1.0e-6), 1.0);
safePowerEpsilon = max(abs(finiteOr(powerEpsilon, 0.0)), 1.0e-9);
regenMechanicalLimit = max(finiteOr(regenPowerLimit, 0.0), 0.0) / ...
    efficiency;

requestedMotorTorque = wheelForceTarget .* safeRadius ./ safeGearProduct;
if ~enableRegen
    requestedMotorTorque = max(requestedMotorTorque, 0.0);
end
motorTorque = min(max(requestedMotorTorque, motorTorqueLower), ...
    motorTorqueUpper);

regenPowerActive = false;
for projectionIndex = 1:4
    mechanicalPower = sum(motorTorque .* motorSpeed);
    if mechanicalPower >= -regenMechanicalLimit - safePowerEpsilon
        break
    end
    adjustable = motorTorque < 0.0 & motorSpeed > 0.0 & ...
        motorTorque < motorTorqueUpper;
    speedSum = sum(motorSpeed(adjustable));
    if speedSum <= safePowerEpsilon
        break
    end
    commonIncrease = (-regenMechanicalLimit - mechanicalPower) / speedSum;
    motorTorque(adjustable) = min(motorTorqueUpper(adjustable), ...
        motorTorque(adjustable) + commonIncrease);
    regenPowerActive = true;
end

mechanicalPowerPerWheel = motorTorque .* motorSpeed;
driveMechanicalPower = sum(max(mechanicalPowerPerWheel, 0.0));
regenMechanicalPower = sum(min(mechanicalPowerPerWheel, 0.0));
totalPower = driveMechanicalPower / efficiency + ...
    regenMechanicalPower * efficiency;
motorWheelForce = motorTorque .* safeGearProduct ./ safeRadius;

targetTotalForce = sum(wheelForceTarget);
requiredFrictionForce = max(0.0, sum(motorWheelForce) - targetTotalForce);
brakeCapacityTotal = sum(brakeMaxTorque) / safeRadius;
if sum(brakeMaxTorque) > safePowerEpsilon
    brakeShare = brakeMaxTorque ./ sum(brakeMaxTorque);
else
    brakeShare = zeros(4, 1);
end

availableFrictionPerWheel = max(tireForceCapacity - ...
    abs(motorWheelForce), 0.0);
frictionCapacityByTire = inf;
for wheelIndex = 1:4
    if brakeShare(wheelIndex) > 1.0e-12
        frictionCapacityByTire = min(frictionCapacityByTire, ...
            availableFrictionPerWheel(wheelIndex) / brakeShare(wheelIndex));
    end
end
if ~isfinite(frictionCapacityByTire)
    frictionCapacityByTire = 0.0;
end
frictionForceTotal = min(requiredFrictionForce, ...
    min(brakeCapacityTotal, frictionCapacityByTire));
frictionBrakeTorque = frictionForceTotal .* brakeShare .* safeRadius;
frictionWheelForce = frictionForceTotal .* brakeShare;
actualWheelForce = motorWheelForce - frictionWheelForce;
regenTorque = max(-motorTorque, 0.0);

actuatorLimited = any(abs(motorTorque - requestedMotorTorque) > 1.0e-9) || ...
    requiredFrictionForce > frictionForceTotal + 1.0e-6;
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
