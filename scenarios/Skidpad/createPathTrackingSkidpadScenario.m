function scenario = createPathTrackingSkidpadScenario(options)
%CREATEPathTrackingSKIDPADSCENARIO Create a complete 2026 FSAE skidpad run.
%   The path follows the official sequence: entry straight, two right-hand
%   laps, two left-hand laps, then an exit straight in the entry direction.

arguments
    options.SampleDistance (1, 1) double {mustBePositive} = 0.1
    options.TargetSpeed (1, 1) double {mustBeNonnegative} = 8.0
    options.LateralAccelerationLimit (1, 1) double {mustBePositive} = 4.0
    options.ReferenceAccelerationLimit (1, 1) double {mustBePositive} = 3.0
    options.ReferenceDecelerationLimit (1, 1) double {mustBePositive} = 4.0
    options.EntryStraightLength (1, 1) double {mustBePositive} = 20.0
    options.ExitStraightLength (1, 1) double {mustBePositive} = 20.0
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "track"));

innerDiameter = 15.25;
outerDiameter = 21.25;
circleCenterSeparation = 18.25;
centerlineRadius = circleCenterSeparation / 2;

entryY = linspace(-options.EntryStraightLength, 0, 1001)';
entryX = zeros(size(entryY));

% At the common tangent point, increasing Y enters the course. Decreasing
% angle therefore produces the required two clockwise right-hand laps.
rightAngle = linspace(pi, -3 * pi, 5001)';
rightLoopX = centerlineRadius + centerlineRadius * cos(rightAngle);
rightLoopY = centerlineRadius * sin(rightAngle);

% Increasing angle produces two counterclockwise left-hand laps while
% retaining the same tangent direction through the crossover.
leftAngle = linspace(0, 4 * pi, 5001)';
leftLoopX = -centerlineRadius + centerlineRadius * cos(leftAngle);
leftLoopY = centerlineRadius * sin(leftAngle);

exitY = linspace(0, options.ExitStraightLength, 1001)';
exitX = zeros(size(exitY));

% Remove duplicate samples at the three segment junctions.
x = [entryX(1:end-1); rightLoopX(1:end-1); ...
    leftLoopX(1:end-1); exitX];
y = [entryY(1:end-1); rightLoopY(1:end-1); ...
    leftLoopY(1:end-1); exitY];
track = makePathTrackingParametricTrack(x, y, options.SampleDistance, 3.0, ...
    "FSAE_Skidpad_CompleteRun", false);
curvatureMagnitude = max(abs(track.Curvature), 1.0e-4);
curvatureLimitedSpeed = sqrt( ...
    options.LateralAccelerationLimit ./ curvatureMagnitude);
track.ReferenceSpeed(:) = applyOpenSpeedEnvelope( ...
    min(options.TargetSpeed, curvatureLimitedSpeed), ...
    track.SampleDistance, options.ReferenceAccelerationLimit, ...
    options.ReferenceDecelerationLimit);
track.RuleSource = "Formula SAE Rules 2026 D.10.1-D.10.2";
track.InnerCircleDiameter = innerDiameter;
track.OuterCircleDiameter = outerDiameter;
track.CircleCenterSeparation = circleCenterSeparation;
track.CenterlineRadius = centerlineRadius;
track.PathWidth = 3.0;
track.EntryStraightLength = options.EntryStraightLength;
track.ExitStraightLength = options.ExitStraightLength;
track.RightCircleLapCount = 2;
track.LeftCircleLapCount = 2;
track.TimingLineS = options.EntryStraightLength;
track.RightLoopsEndS = options.EntryStraightLength + ...
    4 * pi * centerlineRadius;
track.FinishLineS = options.EntryStraightLength + ...
    8 * pi * centerlineRadius;
track.RunSequence = "Entry -> Right x2 -> Left x2 -> Exit";
track.LateralAccelerationLimit = options.LateralAccelerationLimit;
track.ReferenceAccelerationLimit = options.ReferenceAccelerationLimit;
track.ReferenceDecelerationLimit = options.ReferenceDecelerationLimit;
track.WidthSource = ...
    "Formula SAE Rules 2026 D.10.1.1 and D.10.1.3: 3.0 m path";
track.StraightLengthSource = ...
    "2026 FSAE event handbook: start about 20 m before timing line; " + ...
    "exit length uses the same configurable site-layout baseline";

scenario = struct( ...
    "ID", "PathTracking-SKIDPAD-COMPLETE-RUN", ...
    "Event", "Skidpad", ...
    "Track", track, ...
    "Environment", makeEnvironment(), ...
    "InitialState", makeInitialState(track), ...
    "StopTime", max(45, track.Length / max(options.TargetSpeed, 1.0) * 1.3), ...
    "NumberOfLaps", 1, ...
    "EventDescription", ...
    "2026 FSAE complete skidpad run: entry, right x2, left x2, exit.", ...
    "ReferenceSpeedSource", ...
    "curvature-limited target with acceleration/deceleration envelope; not a performance claim");
end

function speed = applyOpenSpeedEnvelope( ...
    speed, sampleDistance, accelerationLimit, decelerationLimit)
speed = double(speed(:));
for index = numel(speed) - 1:-1:1
    brakingSpeed = sqrt(speed(index + 1)^2 + ...
        2 * decelerationLimit * sampleDistance);
    speed(index) = min(speed(index), brakingSpeed);
end
for index = 2:numel(speed)
    accelerationSpeed = sqrt(speed(index - 1)^2 + ...
        2 * accelerationLimit * sampleDistance);
    speed(index) = min(speed(index), accelerationSpeed);
end
end

function environment = makeEnvironment()
environment = struct( ...
    "RoadGripScale", ones(4, 1), "RoadMuLimit", inf(4, 1), ...
    "RoadGrade", 0, "RoadBank", 0, ...
    "RoadHeight", zeros(4, 1), "AirDensity", 1.225, ...
    "WindVelocityGlobal", zeros(3, 1), "AmbientTemperature", 25, ...
    "Gravity", 9.80665, "EnableRoadDisturbance", false, ...
    "EnvironmentValid", true);
end

function initialState = makeInitialState(track)
initialState = struct( ...
    "X", track.X(1), "Y", track.Y(1), "Psi", track.Heading(1), ...
    "Ux", 0.2, "Uy", 0, ...
    "YawRate", 0, "WheelSpeed", 0.8 * ones(4, 1));
end
