function [motorTorqueRequest, frictionBrakeTorqueRequest, ...
    regenTorqueRequest, totalPowerRequest, allocatedYawMoment, ...
    wheelTorqueUnconstrained, controllerSaturated] = ...
    torqueVectoringAllocateWheelTorque(desiredLongitudinalForce, desiredYawMoment, ...
    motorSpeed, enableTV, enableRegen, enableActuation, tireRadius, ...
    gearRatio, gearEfficiency, motorTorqueLimit, motorSpeedLimit, ...
    inverterEfficiency, drivePowerLimit, regenPowerLimit, trackFront, ...
    trackRear, brakeMaxTorque, powerEpsilon)
%TorqueVectoringALLOCATEWHEELTORQUE Allocate TV0/TV1 motor and friction-brake requests.
%   Wheel order is [FL; FR; RL; RR]. Motor torque is motor-shaft torque in
%   N*m. Positive allocated yaw moment is a left-turn moment. The allocator
%   removes common longitudinal torque before sacrificing differential
%   yaw torque when the total motor mechanical-power limit is reached.

motorTorqueRequest = zeros(4, 1);
frictionBrakeTorqueRequest = zeros(4, 1);
regenTorqueRequest = zeros(4, 1);
totalPowerRequest = 0.0;
allocatedYawMoment = 0.0;
wheelTorqueUnconstrained = zeros(4, 1);
controllerSaturated = false;

if ~enableActuation
    return
end

safeRadius = max(tireRadius, 1.0e-6);
safeGearProduct = max(gearRatio * gearEfficiency, 1.0e-6);
torqueLimit = max(abs(motorTorqueLimit), 0.0);
speedLimit = max(abs(motorSpeedLimit), 1.0e-6);
safePowerEpsilon = max(abs(powerEpsilon), 1.0e-6);
effectiveInverterEfficiency = min(1.0, ...
    max(inverterEfficiency, 1.0e-6));

commonTorque = desiredLongitudinalForce * safeRadius / ...
    (4.0 * safeGearProduct);
yawTorqueDelta = 0.0;
enableDifferential = enableTV && ...
    (desiredLongitudinalForce >= 0.0 || enableRegen);
if enableDifferential
    yawTorqueDelta = desiredYawMoment * safeRadius / ...
        (max(trackFront + trackRear, 1.0e-6) * safeGearProduct);
end
rawDifferential = [-yawTorqueDelta; yawTorqueDelta; ...
    -yawTorqueDelta; yawTorqueDelta];
wheelTorqueUnconstrained = commonTorque + rawDifferential;

% Preserve the requested yaw component first, within the four individual
% motor limits. Common longitudinal torque consumes only the remaining
% per-wheel margin.
differentialScale = min(1.0, ...
    torqueLimit / max(max(abs(rawDifferential)), safePowerEpsilon));
differentialTorque = differentialScale * rawDifferential;
if abs(commonTorque) <= safePowerEpsilon
    commonScaleMotor = 0.0;
elseif commonTorque > 0.0
    commonScaleMotor = min((torqueLimit - differentialTorque) / ...
        commonTorque);
else
    commonScaleMotor = min((-torqueLimit - differentialTorque) / ...
        commonTorque);
end
commonScaleMotor = min(1.0, max(0.0, commonScaleMotor));

driveMechanicalPowerLimit = max(drivePowerLimit, 0.0) * ...
    effectiveInverterEfficiency;
regenMechanicalPowerLimit = max(regenPowerLimit, 0.0) / ...
    effectiveInverterEfficiency;
differentialPower = sum(differentialTorque .* motorSpeed);

% If the yaw component alone violates a power limit, it is the last
% component to be reduced.
if differentialPower > driveMechanicalPowerLimit + safePowerEpsilon
    differentialScalePower = driveMechanicalPowerLimit / ...
        max(differentialPower, safePowerEpsilon);
    differentialTorque = differentialTorque * differentialScalePower;
    differentialPower = sum(differentialTorque .* motorSpeed);
    controllerSaturated = true;
elseif differentialPower < -regenMechanicalPowerLimit - safePowerEpsilon
    differentialScalePower = regenMechanicalPowerLimit / ...
        max(abs(differentialPower), safePowerEpsilon);
    differentialTorque = differentialTorque * differentialScalePower;
    differentialPower = sum(differentialTorque .* motorSpeed);
    controllerSaturated = true;
end

commonPower = commonTorque * sum(motorSpeed);
commonScalePower = 1.0;
powerAtFullCommon = differentialPower + commonPower;
if powerAtFullCommon > driveMechanicalPowerLimit + safePowerEpsilon && ...
        commonPower > safePowerEpsilon
    commonScalePower = (driveMechanicalPowerLimit - ...
        differentialPower) / commonPower;
elseif powerAtFullCommon < -regenMechanicalPowerLimit - ...
        safePowerEpsilon && commonPower < -safePowerEpsilon
    commonScalePower = (-regenMechanicalPowerLimit - ...
        differentialPower) / commonPower;
end
commonScalePower = min(1.0, max(0.0, commonScalePower));
commonScale = min(commonScaleMotor, commonScalePower);
motorTorqueRequest = differentialTorque + commonScale * commonTorque;

% Do not command torque that accelerates a motor farther beyond its speed
% limit. Opposing regenerative torque remains available.
for wheelIndex = 1:4
    motorTorqueRequest(wheelIndex) = min(torqueLimit, ...
        max(-torqueLimit, motorTorqueRequest(wheelIndex)));
    if abs(motorSpeed(wheelIndex)) >= speedLimit && ...
            motorTorqueRequest(wheelIndex) * motorSpeed(wheelIndex) > 0.0
        motorTorqueRequest(wheelIndex) = 0.0;
        controllerSaturated = true;
    end
end

if ~enableRegen
    motorTorqueRequest = max(motorTorqueRequest, 0.0);
end
regenTorqueRequest = max(0.0, -motorTorqueRequest);
totalPowerRequest = sum(motorTorqueRequest .* motorSpeed);

requestedBrakeForce = max(0.0, -desiredLongitudinalForce);
availableRegenForce = sum(regenTorqueRequest) * safeGearProduct / ...
    safeRadius;
residualBrakeForce = max(0.0, requestedBrakeForce - ...
    availableRegenForce);
totalBrakeTorqueDemand = residualBrakeForce * safeRadius;
brakeCapacity = max(sum(max(brakeMaxTorque, 0.0)), ...
    safePowerEpsilon);
brakeScale = min(1.0, totalBrakeTorqueDemand / brakeCapacity);
frictionBrakeTorqueRequest = brakeScale * max(brakeMaxTorque, 0.0);

allocatedYawMoment = ( ...
    0.5 * trackFront * ...
    (motorTorqueRequest(2) - motorTorqueRequest(1)) + ...
    0.5 * trackRear * ...
    (motorTorqueRequest(4) - motorTorqueRequest(3))) * ...
    safeGearProduct / safeRadius;

controllerSaturated = controllerSaturated || ...
    differentialScale < 1.0 - 1.0e-9 || ...
    commonScale < 1.0 - 1.0e-9 || ...
    any(abs(motorTorqueRequest - wheelTorqueUnconstrained) > 1.0e-9) || ...
    totalBrakeTorqueDemand > brakeCapacity + 1.0e-9;
end
