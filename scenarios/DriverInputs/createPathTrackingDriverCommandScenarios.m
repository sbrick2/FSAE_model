function scenarios = createPathTrackingDriverCommandScenarios(options)
%CREATEPathTrackingDRIVERCOMMANDSCENARIOS Create configurable open-loop driver inputs.

arguments
    options.SampleTime (1, 1) double {mustBePositive} = 0.01
    options.Duration (1, 1) double {mustBePositive} = 8.0
    options.StepTime (1, 1) double {mustBeNonnegative} = 1.0
    options.StepSteering (1, 1) double = 0.05
    options.SineSteeringAmplitude (1, 1) double = 0.04
    options.SineFrequency (1, 1) double {mustBeNonnegative} = 0.5
    options.DriveAcceleration (1, 1) double = 3.0
    options.BrakeAcceleration (1, 1) double = -3.0
end

time = (0:options.SampleTime:options.Duration)';
scenarios = repmat(emptyScenario(), 5, 1);
scenarios(1) = makeScenario("PathTracking-DRIVER-STEP-STEERING", time, ...
    options.StepSteering * (time >= options.StepTime), zeros(size(time)));
scenarios(2) = makeScenario("PathTracking-DRIVER-SINE-STEERING", time, ...
    options.SineSteeringAmplitude * ...
    sin(2 * pi * options.SineFrequency * time), zeros(size(time)));
scenarios(3) = makeScenario("PathTracking-DRIVER-CONSTANT-STEERING", time, ...
    options.StepSteering * ones(size(time)), zeros(size(time)));
scenarios(4) = makeScenario("PathTracking-DRIVER-ACCELERATION", time, ...
    zeros(size(time)), options.DriveAcceleration * ones(size(time)));
scenarios(5) = makeScenario("PathTracking-DRIVER-BRAKING", time, ...
    zeros(size(time)), options.BrakeAcceleration * ones(size(time)));
end

function scenario = emptyScenario()
scenario = struct("ID", "", "Time", [], "SteeringRackAngle", [], ...
    "LongitudinalAcceleration", []);
end

function scenario = makeScenario(id, time, steering, acceleration)
scenario = struct("ID", id, "Time", time, ...
    "SteeringRackAngle", steering, ...
    "LongitudinalAcceleration", acceleration);
end
