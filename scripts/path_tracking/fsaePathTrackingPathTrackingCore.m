function [SteeringWheelAngleRequest, SteeringRackAngleRequest, ...
    LongitudinalAccelerationRequest, DriveTorqueRequest, ...
    BrakePressureRequest, EnableTV, EnableTC, EnableRegen, ...
    EnableEnergyManagement, DriverMode] = fsaePathTrackingPathTrackingCore(track, sensor)
%FSAEPathTrackingPATHTRACKINGCORE Discrete Stanley-style lateral and PI speed control.
%   Sensor is used instead of VehicleState, so the production driver model
%   does not depend directly on plant truth.

persistent speedIntegrator previousSteering
if isempty(speedIntegrator)
    speedIntegrator = 0;
    previousSteering = 0;
end

SteeringWheelAngleRequest = 0;
SteeringRackAngleRequest = 0;
LongitudinalAccelerationRequest = 0;
DriveTorqueRequest = 0;
BrakePressureRequest = 0;
EnableTV = false;
EnableTC = false;
EnableRegen = true;
EnableEnergyManagement = false;
DriverMode = uint8(0);

if ~track.Valid || ~sensor.PoseValid
    return
end
DriverMode = uint8(2);

deltaX = sensor.PositionX - track.ReferenceX;
deltaY = sensor.PositionY - track.ReferenceY;
lateralError = -deltaX * sin(track.ReferenceHeading) + ...
    deltaY * cos(track.ReferenceHeading);
headingError = atan2(sin(track.ReferenceHeading - sensor.Heading), ...
    cos(track.ReferenceHeading - sensor.Heading));
speedForSteering = max(abs(sensor.LongitudinalSpeed), PathTrackingMinimumSteeringSpeed);
curvatureFeedForward = atan(PathTrackingWheelbase * track.ReferenceCurvature);
feedbackSteering = PathTrackingHeadingGain * headingError - ...
    atan2(PathTrackingCrossTrackGain * lateralError, speedForSteering);
requestedSteering = curvatureFeedForward + feedbackSteering;
requestedSteering = min(PathTrackingMaximumSteeringAngle, ...
    max(-PathTrackingMaximumSteeringAngle, requestedSteering));
steeringRateLimit = PathTrackingMaximumSteeringRate * PathTrackingControlSampleTime;
SteeringRackAngleRequest = previousSteering + min(steeringRateLimit, ...
    max(-steeringRateLimit, requestedSteering - previousSteering));
previousSteering = SteeringRackAngleRequest;
SteeringWheelAngleRequest = PathTrackingSteeringWheelRatio * SteeringRackAngleRequest;

speedError = track.ReferenceSpeed - sensor.LongitudinalSpeed;
unsaturatedAcceleration = PathTrackingSpeedKp * speedError + speedIntegrator;
LongitudinalAccelerationRequest = min(PathTrackingMaximumAcceleration, ...
    max(-PathTrackingMaximumDeceleration, unsaturatedAcceleration));
integratorError = LongitudinalAccelerationRequest - unsaturatedAcceleration;
speedIntegrator = speedIntegrator + PathTrackingSpeedKi * speedError * ...
    PathTrackingControlSampleTime + PathTrackingAntiWindupGain * integratorError * ...
    PathTrackingControlSampleTime;

% The PathTracking baseline uses the acceleration request as the controller contract;
% the controller converts it to motor and regenerative torque.  The brake
% pressure field is kept explicit but disabled until a pressure map exists.
BrakePressureRequest = max(0, -LongitudinalAccelerationRequest) * ...
    PathTrackingBrakePressurePerAcceleration;
EnableTC = true;
end
