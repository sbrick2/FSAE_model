function [SteeringRackAngleRequest, MotorTorqueRequest, ...
    FrictionBrakeTorqueRequest, RegenTorqueRequest, TotalPowerRequest, ...
    ControllerMode, EnableActuation, ReferenceYawRate, YawRateError, ...
    DesiredLongitudinalForce, DesiredYawMoment, AllocatedYawMoment, ...
    WheelTorqueUnconstrained, TVActive, TCActive, RegenActive, ...
    EnergyManagementActive, ControllerSaturated] = ...
    fsaePathTrackingVehicleControllerCore(driver, sensor, powertrain)
%FSAEPathTrackingVEHICLECONTROLLERCORE Baseline acceleration-to-four-motor allocator.

SteeringRackAngleRequest = driver.SteeringRackAngleRequest;
MotorTorqueRequest = zeros(4, 1);
FrictionBrakeTorqueRequest = zeros(4, 1);
RegenTorqueRequest = zeros(4, 1);
TotalPowerRequest = 0;
ControllerMode = uint8(0);
EnableActuation = false;
ReferenceYawRate = 0;
YawRateError = 0;
DesiredLongitudinalForce = 0;
DesiredYawMoment = 0;
AllocatedYawMoment = 0;
WheelTorqueUnconstrained = zeros(4, 1);
TVActive = false;
TCActive = false;
RegenActive = false;
EnergyManagementActive = false;
ControllerSaturated = false;

if driver.DriverMode == uint8(0) || ~sensor.PoseValid
    return
end
ControllerMode = uint8(2);
EnableActuation = driver.DriverMode == uint8(2);
TVActive = driver.EnableTV;
TCActive = driver.EnableTC;
RegenActive = driver.EnableRegen;
EnergyManagementActive = driver.EnableEnergyManagement;

DesiredLongitudinalForce = PathTrackingVehicleMass * ...
    min(PathTrackingMaximumAcceleration, max(-PathTrackingMaximumDeceleration, ...
    driver.LongitudinalAccelerationRequest));
WheelTorqueUnconstrained(:) = DesiredLongitudinalForce * ...
    PathTrackingTireEffectiveRadius / (4 * PathTrackingGearRatio * PathTrackingGearEfficiency);
MotorTorqueRequest(:) = min(PathTrackingMotorTorqueLimit, ...
    max(-PathTrackingMotorTorqueLimit, WheelTorqueUnconstrained));
ControllerSaturated = any(abs( ...
    MotorTorqueRequest - WheelTorqueUnconstrained) > 1.0e-9);

% Negative motor torque is the regenerative path.  Friction braking is added
% only for the residual force that the bounded motor torque cannot supply.
RegenTorqueRequest = max(0, -MotorTorqueRequest);
if ~RegenActive
    MotorTorqueRequest = max(0, MotorTorqueRequest);
    RegenTorqueRequest(:) = 0;
end
requestedBrakeForce = max(0, -DesiredLongitudinalForce);
availableRegenForce = sum(max(0, -MotorTorqueRequest)) * ...
    PathTrackingGearRatio * PathTrackingGearEfficiency / ...
    max(PathTrackingTireEffectiveRadius, 1.0e-6);
residualBrakeForce = max(0, requestedBrakeForce - availableRegenForce);
FrictionBrakeTorqueRequest(:) = residualBrakeForce * ...
    PathTrackingTireEffectiveRadius / 4;
ControllerSaturated = ControllerSaturated || residualBrakeForce > 1.0e-9;
TotalPowerRequest = sum(MotorTorqueRequest .* powertrain.MotorSpeed);
ReferenceYawRate = trackReferenceYawRate(driver, sensor);
YawRateError = ReferenceYawRate - sensor.YawRate;
end

function referenceYawRate = trackReferenceYawRate(driver, sensor)
% PathTracking DriverCommand does not carry curvature; the zero-yaw baseline is
% intentional until TV1 introduces a curvature/yaw reference bus.
referenceYawRate = 0;
if driver.DriverMode == uint8(2) && sensor.PoseValid
    referenceYawRate = 0;
end
end
