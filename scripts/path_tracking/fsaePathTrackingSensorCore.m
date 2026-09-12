function [PositionX, PositionY, Heading, LongitudinalSpeed, LateralSpeed, ...
    YawRate, AccelX, AccelY, WheelSpeed, SteeringRackAngle, MotorSpeed, ...
    MotorTorqueEstimate, BatteryVoltage, BatteryCurrent, PoseValid, ...
    IMUValid, WheelSpeedValid, PowertrainValid] = ...
    fsaePathTrackingSensorCore(vehicle, powertrain)
%FSAEPathTrackingSENSORCORE Truth-derived sensor emulation for PathTracking MIL.
%   No tire force or friction truth is exposed.  Noise, latency and an
%   estimator are intentionally deferred to the deployment milestone.

PositionX = vehicle.X;
PositionY = vehicle.Y;
Heading = vehicle.Psi;
LongitudinalSpeed = vehicle.Ux;
LateralSpeed = vehicle.Uy;
YawRate = vehicle.YawRate;
AccelX = vehicle.Ax;
AccelY = vehicle.Ay;
WheelSpeed = vehicle.WheelSpeed;
SteeringRackAngle = mean(vehicle.WheelSteerAngle(1:2));
MotorSpeed = powertrain.MotorSpeed;
MotorTorqueEstimate = powertrain.MotorTorqueActual;
BatteryVoltage = powertrain.BatteryVoltage;
BatteryCurrent = powertrain.BatteryCurrent;
PoseValid = true;
IMUValid = true;
WheelSpeedValid = true(4, 1);
PowertrainValid = true;
end
