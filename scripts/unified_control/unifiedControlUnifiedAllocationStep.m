function [motorTorqueRequest, frictionBrakeTorqueRequest, ...
    regenTorqueRequest, totalPowerRequest, allocatedYawMoment, ...
    wheelTorqueUnconstrained, controllerSaturated, tcActive, ...
    regenActive, energyManagementActive, controllerMode, ...
    constraintFlags, previousMotorTorqueNext, tcStateNext] = ...
    unifiedControlUnifiedAllocationStep(desiredLongitudinalForce, desiredYawMoment, ...
    longitudinalSpeed, wheelSpeed, motorSpeed, accelX, accelY, batterySOC, ...
    wheelSpeedValid, powertrainValid, enableTV, enableTC, enableRegen, ...
    enableEnergyManagement, enableActuation, previousMotorTorque, ...
    tcStatePrevious, controlSampleTime, vehicleMass, gravity, wheelbase, ...
    cgHeight, staticLoadDistributionFront, ...
    rollStiffnessDistributionFront, trackFront, trackRear, tireRadius, ...
    lowSpeedEpsilon, gearRatio, gearEfficiency, motorTorqueLimit, ...
    motorSpeedLimit, inverterEfficiency, drivePowerLimit, regenPowerLimit, ...
    brakeMaxTorque, batterySOCUpperLimit, powerEpsilon, ...
    motorTorqueRateLimit, frictionEstimate, tcSlipOn, tcSlipOff, ...
    tcSlipFullCut, minimumNormalLoad, energyDriveFraction, ...
    forcePriorityWeight, yawPriorityWeight, regularizationWeight, ...
    allocatorIterations)
%UnifiedControlUNIFIEDALLOCATIONSTEP Execute one UnifiedControl allocation sample.
%   All arrays use [FL; FR; RL; RR]. Positive yaw is a left-turn moment.

motorTorqueRequest = zeros(4, 1);
frictionBrakeTorqueRequest = zeros(4, 1);
regenTorqueRequest = zeros(4, 1);
totalPowerRequest = 0.0;
allocatedYawMoment = 0.0;
wheelTorqueUnconstrained = zeros(4, 1);
controllerSaturated = false;
tcActive = false;
regenActive = false;
energyManagementActive = false;
controllerMode = uint8(0);
constraintFlags = uint16(0);
previousMotorTorqueNext = zeros(4, 1);
tcStateNext = false(4, 1);

if ~enableActuation
    return
end

previousMotorTorque = sanitizeVector(previousMotorTorque, 0.0);
motorSpeed = sanitizeVector(motorSpeed, 0.0);
safeRadius = max(abs(finiteOr(tireRadius, 0.0)), 1.0e-6);
safeGearProduct = max(abs(finiteOr(gearRatio, 0.0) * ...
    finiteOr(gearEfficiency, 0.0)), 1.0e-6);
torqueLimit = max(abs(finiteOr(motorTorqueLimit, 0.0)), 0.0);
speedLimit = max(abs(finiteOr(motorSpeedLimit, 0.0)), 1.0e-6);
sampleTime = max(abs(finiteOr(controlSampleTime, 0.0)), 1.0e-6);
torqueStep = max(abs(finiteOr(motorTorqueRateLimit, 0.0)), 0.0) * ...
    sampleTime;
safePowerEpsilon = max(abs(finiteOr(powerEpsilon, 0.0)), 1.0e-9);
powertrainValid = logical(powertrainValid);
wheelSpeedValid = logical(reshape(wheelSpeedValid, 4, 1));

[normalLoad, ~, tireForceCapacity, tcScale, tcStateNext, ...
    wheelDegraded] = unifiedControlEstimateWheelConstraints(longitudinalSpeed, ...
    wheelSpeed, accelX, accelY, wheelSpeedValid, enableTC, ...
    tcStatePrevious, vehicleMass, gravity, wheelbase, cgHeight, ...
    staticLoadDistributionFront, rollStiffnessDistributionFront, ...
    trackFront, trackRear, safeRadius, lowSpeedEpsilon, ...
    frictionEstimate, minimumNormalLoad, tcSlipOn, tcSlipOff, ...
    tcSlipFullCut);

degraded = wheelDegraded || ~powertrainValid;
if degraded
    controllerMode = uint8(3);
    constraintFlags = bitor(constraintFlags, uint16(32));
else
    controllerMode = uint8(2);
end

enableTV = logical(enableTV);
enableTC = logical(enableTC);
effectiveYawMoment = finiteOr(desiredYawMoment, 0.0) * double(enableTV);
tcActive = enableTC && any(tcStateNext);
if tcActive
    constraintFlags = bitor(constraintFlags, uint16(16));
end

motorTorqueLower = max(-torqueLimit, previousMotorTorque - torqueStep);
motorTorqueUpper = min(torqueLimit, previousMotorTorque + torqueStep);
for wheelIndex = 1:4
    if abs(motorSpeed(wheelIndex)) >= speedLimit && ...
            motorSpeed(wheelIndex) >= 0.0
        motorTorqueUpper(wheelIndex) = min(motorTorqueUpper(wheelIndex), 0.0);
        constraintFlags = bitor(constraintFlags, uint16(512));
    end
    if ~wheelSpeedValid(wheelIndex)
        motorTorqueUpper(wheelIndex) = min(motorTorqueUpper(wheelIndex), 0.0);
    end
end

motorDriveForceUpper = motorTorqueUpper .* safeGearProduct ./ safeRadius;
motorDriveForceUpper = min(motorDriveForceUpper, ...
    tireForceCapacity .* tcScale);
lowerForce = -tireForceCapacity;
upperForce = motorDriveForceUpper;

physicalDrivePowerLimit = max(finiteOr(drivePowerLimit, 0.0), 0.0);
energyManagementActive = logical(enableEnergyManagement) && powertrainValid;
activeDrivePowerLimit = physicalDrivePowerLimit;
if energyManagementActive
    activeDrivePowerLimit = physicalDrivePowerLimit * ...
        min(max(finiteOr(energyDriveFraction, 1.0), 0.0), 1.0);
    constraintFlags = bitor(constraintFlags, uint16(256));
end

[wheelForceTarget, wheelForceTargetUnconstrained, drivePowerActive] = ...
    unifiedControlProjectWheelForces(desiredLongitudinalForce, effectiveYawMoment, ...
    lowerForce, upperForce, motorSpeed, activeDrivePowerLimit, safeRadius, ...
    safeGearProduct, inverterEfficiency, trackFront, trackRear, ...
    forcePriorityWeight, yawPriorityWeight, regularizationWeight, ...
    allocatorIterations, safePowerEpsilon);
if drivePowerActive
    constraintFlags = bitor(constraintFlags, uint16(2));
end

regenActive = logical(enableRegen) && powertrainValid && ...
    finiteOr(batterySOC, 1.0) < finiteOr(batterySOCUpperLimit, 0.0);
if logical(enableRegen) && ~regenActive
    constraintFlags = bitor(constraintFlags, uint16(64));
end
activeRegenPowerLimit = max(finiteOr(regenPowerLimit, 0.0), 0.0);
if ~regenActive
    activeRegenPowerLimit = 0.0;
    motorTorqueLower = max(motorTorqueLower, 0.0);
end

[motorTorqueRequest, frictionBrakeTorqueRequest, regenTorqueRequest, ...
    totalPowerRequest, regenPowerActive, actuatorLimited, ...
    actualWheelForce] = unifiedControlMixActuators(wheelForceTarget, motorSpeed, ...
    motorTorqueLower, motorTorqueUpper, ...
    tireForceCapacity, regenActive, activeRegenPowerLimit, brakeMaxTorque, ...
    safeRadius, safeGearProduct, inverterEfficiency, safePowerEpsilon);
if regenPowerActive
    constraintFlags = bitor(constraintFlags, uint16(4));
end
if actuatorLimited
    constraintFlags = bitor(constraintFlags, uint16(1));
end

yawRow = [-0.5 * max(abs(trackFront), 1.0e-6), ...
    0.5 * max(abs(trackFront), 1.0e-6), ...
    -0.5 * max(abs(trackRear), 1.0e-6), ...
    0.5 * max(abs(trackRear), 1.0e-6)];
motorWheelForce = motorTorqueRequest .* safeGearProduct ./ safeRadius;
allocatedYawMoment = yawRow * motorWheelForce;
wheelTorqueUnconstrained = wheelForceTargetUnconstrained .* ...
    safeRadius ./ safeGearProduct;

forceResidual = abs(sum(actualWheelForce) - ...
    finiteOr(desiredLongitudinalForce, 0.0));
yawResidual = abs(allocatedYawMoment - effectiveYawMoment);
forceTolerance = max(1.0, 1.0e-3 * ...
    abs(finiteOr(desiredLongitudinalForce, 0.0)));
yawTolerance = max(1.0, 1.0e-3 * abs(effectiveYawMoment));
if forceResidual > forceTolerance || yawResidual > yawTolerance
    constraintFlags = bitor(constraintFlags, uint16(128));
end
if any(abs(wheelForceTarget) >= tireForceCapacity - 1.0e-6 & ...
        tireForceCapacity > 0.0)
    constraintFlags = bitor(constraintFlags, uint16(8));
end

controllerSaturated = constraintFlags ~= uint16(0);
previousMotorTorqueNext = motorTorqueRequest;
if any(~isfinite(normalLoad)) || any(~isfinite(motorTorqueRequest)) || ...
        ~isfinite(totalPowerRequest)
    motorTorqueRequest(:) = 0.0;
    frictionBrakeTorqueRequest(:) = 0.0;
    regenTorqueRequest(:) = 0.0;
    totalPowerRequest = 0.0;
    allocatedYawMoment = 0.0;
    previousMotorTorqueNext(:) = 0.0;
    tcStateNext(:) = false;
    controllerMode = uint8(3);
    constraintFlags = bitor(constraintFlags, uint16(32));
    controllerSaturated = true;
end
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
