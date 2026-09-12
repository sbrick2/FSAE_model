function metric = calculateLapSimulationEventMetric(result, scenario)
%CALCULATELAPSIMULATIONEVENTMETRIC Apply event timing rules to time-domain data.
%   Acceleration uses complete-course distance, Autocross uses the vehicle's
%   actual finish-gate crossing, and Skidpad uses the mean time of the
%   second right-hand and second left-hand laps.

arguments
    result (1, 1) struct
    scenario (1, 1) struct
end

eventName = lower(string(scenario.Event));
metric = invalidMetric(eventName, "Timed distance data is unavailable.");
[progress, time, speed, preparationIssue] = prepareTimedData(result);
if strlength(preparationIssue) > 0
    metric.FailureReason = preparationIssue;
    return
end
projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "lap_time"), "-begin");
if eventName == "autocross"
    [gateProfile, gateValid] = autocrossFinishGateProfile(result, scenario.Track);
    if gateValid
        metric = calculateEventMetric(eventName, gateProfile, scenario.Track);
        metric.Metadata.Source = "Vehicle.X/Y finish-gate crossing";
        return
    end
end
distanceIssue = validateDistanceIncrements(progress, scenario.Track);
if strlength(distanceIssue) > 0
    metric.FailureReason = distanceIssue;
    return
end

trackLength = double(scenario.Track.Length);
targetDistance = trackLength;
if isfield(scenario.Track, "IsClosed") && scenario.Track.IsClosed && ...
        isfield(scenario, "NumberOfLaps")
    targetDistance = trackLength * double(scenario.NumberOfLaps);
end
requiredEndDistance = targetDistance;
if eventName == "skidpad"
    requiredFields = ["CenterlineRadius", "RightLoopsEndS", "FinishLineS"];
    if ~all(isfield(scenario.Track, requiredFields))
        metric.FailureReason = ...
            "Skidpad timing geometry is unavailable.";
        return
    end
    requiredEndDistance = double(scenario.Track.FinishLineS);
end

[courseTime, courseValid] = sectionElapsedTime( ...
    progress, time, speed, 0.0, requiredEndDistance);
if ~courseValid
    metric.FailureReason = ...
        "The simulation did not complete the scored timing section.";
    return
end

timingDistances = [0.0; requiredEndDistance];
if eventName == "skidpad"
    lapDistance = 2.0 * pi * double(scenario.Track.CenterlineRadius);
    timingDistances = [ ...
        0.0; ...
        double(scenario.Track.RightLoopsEndS) - lapDistance; ...
        double(scenario.Track.RightLoopsEndS); ...
        double(scenario.Track.FinishLineS) - lapDistance; ...
        double(scenario.Track.FinishLineS)];
end
[timingTime, timingSpeed, timingLinesValid] = sampleTimingLines( ...
    progress, time, speed, timingDistances);
if ~timingLinesValid
    metric.FailureReason = "A scoring timing line was not crossed.";
    return
end
speedProfile = struct( ...
    "LapTime", courseTime, ...
    "TrackLength", targetDistance, ...
    "ProgressS", timingDistances, ...
    "Time", timingTime, ...
    "Speed", timingSpeed, ...
    "IsValid", true, ...
    "FailureReason", "");
try
    metric = calculateEventMetric(eventName, speedProfile, scenario.Track);
    metric.Metadata.Source = "time-domain Track.EventDistance";
catch exception
    metric = invalidMetric(eventName, string(exception.message));
end
end

function [speedProfile, valid] = autocrossFinishGateProfile(result, track)
speedProfile = struct;
valid = false;
requiredTrackFields = ["X", "Y", "Length", ...
    "LeftHalfWidth", "RightHalfWidth"];
if ~all(isfield(track, requiredTrackFields)) || ...
        ~isfield(result, "Vehicle") || ...
        ~isfield(result.Vehicle, "X") || ...
        ~isfield(result.Vehicle, "Y") || ...
        ~isfield(result, "Track") || ...
        ~isfield(result.Track, "EventDistance") || ...
        ~isfield(result, "Time")
    return
end

time = double(result.Time(:));
positionX = double(result.Vehicle.X(:));
positionY = double(result.Vehicle.Y(:));
progress = double(result.Track.EventDistance(:));
speed = readVehicleSpeed(result.Vehicle);
sampleCount = min([numel(time), numel(positionX), numel(positionY), ...
    numel(progress), numel(speed)]);
if sampleCount < 2
    return
end
time = time(1:sampleCount);
positionX = positionX(1:sampleCount);
positionY = positionY(1:sampleCount);
progress = progress(1:sampleCount);
speed = speed(1:sampleCount);
finiteSamples = isfinite(time) & isfinite(positionX) & ...
    isfinite(positionY) & isfinite(progress) & isfinite(speed);
time = time(finiteSamples);
positionX = positionX(finiteSamples);
positionY = positionY(finiteSamples);
progress = progress(finiteSamples);
speed = speed(finiteSamples);
if numel(time) < 2 || any(diff(time) <= 0.0)
    return
end

finishPoint = [double(track.X(end)), double(track.Y(end))];
finishTangent = finishPoint - ...
    [double(track.X(end - 1)), double(track.Y(end - 1))];
tangentNorm = hypot(finishTangent(1), finishTangent(2));
if tangentNorm <= eps
    return
end
finishTangent = finishTangent / tangentNorm;
finishNormal = [-finishTangent(2), finishTangent(1)];
relativePosition = [positionX, positionY] - finishPoint;
longitudinalOffset = relativePosition * finishTangent(:);
lateralOffset = relativePosition * finishNormal(:);
crossingCandidates = find(longitudinalOffset(1:end - 1) < 0.0 & ...
    longitudinalOffset(2:end) >= 0.0);
trackLength = double(track.Length);
gateHalfWidth = max([double(track.LeftHalfWidth(end)), ...
    double(track.RightHalfWidth(end))]);

for candidateIndex = reshape(crossingCandidates, 1, [])
    longitudinalSpan = longitudinalOffset(candidateIndex + 1) - ...
        longitudinalOffset(candidateIndex);
    if longitudinalSpan <= 0.0
        continue
    end
    fraction = -longitudinalOffset(candidateIndex) / longitudinalSpan;
    crossingProgress = progress(candidateIndex) + fraction * ...
        (progress(candidateIndex + 1) - progress(candidateIndex));
    crossingLateralOffset = lateralOffset(candidateIndex) + fraction * ...
        (lateralOffset(candidateIndex + 1) - lateralOffset(candidateIndex));
    if crossingProgress < 0.5 * trackLength || ...
            abs(crossingLateralOffset) > gateHalfWidth
        continue
    end
    crossingTime = time(candidateIndex) + fraction * ...
        (time(candidateIndex + 1) - time(candidateIndex));
    crossingSpeed = speed(candidateIndex) + fraction * ...
        (speed(candidateIndex + 1) - speed(candidateIndex));
    elapsedTime = crossingTime - time(1);
    if ~isfinite(elapsedTime) || elapsedTime <= 0.0 || ...
            ~isfinite(crossingSpeed)
        continue
    end
    speedProfile = struct( ...
        "LapTime", elapsedTime, ...
        "TrackLength", trackLength, ...
        "ProgressS", [0.0; trackLength], ...
        "Time", [0.0; elapsedTime], ...
        "Speed", [speed(1); crossingSpeed], ...
        "IsValid", true, ...
        "FailureReason", "");
    valid = true;
    return
end
end

function [progress, time, speed, issue] = prepareTimedData(result)
progress = zeros(0, 1);
time = zeros(0, 1);
speed = zeros(0, 1);
issue = "";
if ~isfield(result, "Time") || ~isfield(result, "Track") || ...
        ~isfield(result.Track, "EventDistance") || ...
        ~isfield(result, "Vehicle")
    issue = "Required timing signals are missing.";
    return
end

time = double(result.Time(:));
progress = double(result.Track.EventDistance(:));
speed = readVehicleSpeed(result.Vehicle);
sampleCount = min([numel(time), numel(progress), numel(speed)]);
if sampleCount < 2
    issue = "Too few timing samples are available.";
    return
end
time = time(1:sampleCount);
progress = progress(1:sampleCount);
speed = speed(1:sampleCount);
valid = isfinite(time) & isfinite(progress) & isfinite(speed);
time = time(valid);
progress = progress(valid);
speed = speed(valid);
if numel(time) < 2 || any(diff(time) <= 0)
    issue = "Timing samples are not strictly increasing in time.";
    return
end

distanceTolerance = max(1.0e-6, ...
    100 * eps(max(max(abs(progress)), 1.0)));
if any(diff(progress) < -distanceTolerance)
    issue = "EventDistance is not monotonic cumulative distance.";
    return
end
progress = cummax(progress);
if progress(1) > distanceTolerance
    issue = "The timing data does not include the start line.";
    return
end
if progress(1) ~= 0.0
    progress(1) = 0.0;
end
end

function issue = validateDistanceIncrements(progress, track)
issue = "";
increments = diff(progress);
positiveIncrements = increments(increments > 0.0);
if isempty(positiveIncrements)
    issue = "EventDistance contains no forward progress.";
    return
end
sampleDistance = median(positiveIncrements);
if isfield(track, "SampleDistance") && ...
        isnumeric(track.SampleDistance) && ...
        isscalar(track.SampleDistance) && ...
        isfinite(track.SampleDistance) && track.SampleDistance > 0.0
    sampleDistance = double(track.SampleDistance);
end
maximumAllowedJump = max(10.0, 20.0 * sampleDistance);
if any(increments > maximumAllowedJump)
    issue = "EventDistance contains an implausible timing-index jump.";
end
end

function [timingTime, timingSpeed, valid] = sampleTimingLines( ...
        progress, time, speed, timingDistances)
timingTime = NaN(size(timingDistances));
timingSpeed = NaN(size(timingDistances));
valid = true;
for lineIndex = 1:numel(timingDistances)
    [timingTime(lineIndex), timingSpeed(lineIndex), lineValid] = ...
        crossingSample(progress, time, speed, ...
        timingDistances(lineIndex));
    valid = valid && lineValid;
end
end

function [crossingTime, crossingSpeed, valid] = crossingSample( ...
        progress, time, speed, targetDistance)
crossingTime = NaN;
crossingSpeed = NaN;
valid = false;
crossingIndex = find(progress >= targetDistance, 1);
if isempty(crossingIndex)
    return
end
if crossingIndex == 1 || progress(crossingIndex) == targetDistance
    crossingTime = time(crossingIndex);
    crossingSpeed = speed(crossingIndex);
    valid = isfinite(crossingTime) && isfinite(crossingSpeed);
    return
end
previousIndex = crossingIndex - 1;
distanceSpan = progress(crossingIndex) - progress(previousIndex);
if distanceSpan <= 0.0
    return
end
fraction = (targetDistance - progress(previousIndex)) / distanceSpan;
crossingTime = time(previousIndex) + fraction * ...
    (time(crossingIndex) - time(previousIndex));
crossingSpeed = speed(previousIndex) + fraction * ...
    (speed(crossingIndex) - speed(previousIndex));
valid = isfinite(crossingTime) && isfinite(crossingSpeed);
end

function speed = readVehicleSpeed(vehicle)
speed = zeros(0, 1);
if isfield(vehicle, "Speed") && ~isempty(vehicle.Speed)
    speed = double(vehicle.Speed(:));
elseif isfield(vehicle, "Ux") && isfield(vehicle, "Uy") && ...
        ~isempty(vehicle.Ux) && ~isempty(vehicle.Uy)
    sampleCount = min(numel(vehicle.Ux), numel(vehicle.Uy));
    speed = hypot(double(vehicle.Ux(1:sampleCount)), ...
        double(vehicle.Uy(1:sampleCount)));
end
end

function [elapsedTime, valid] = sectionElapsedTime( ...
        progress, time, speed, startDistance, endDistance)
elapsedTime = NaN;
valid = false;
if startDistance < progress(1) || endDistance > progress(end) || ...
        endDistance <= startDistance
    return
end
[startTime, ~, startValid] = crossingSample( ...
    progress, time, speed, startDistance);
[endTime, ~, endValid] = crossingSample( ...
    progress, time, speed, endDistance);
section = progress >= startDistance & progress <= endDistance;
elapsedTime = endTime - startTime;
valid = startValid && endValid && isfinite(elapsedTime) && ...
    elapsedTime > 0.0 && ...
    any(speed(section) > 0.1);
end

function metric = invalidMetric(eventName, reason)
metric = struct( ...
    "Name", eventMetricName(eventName), ...
    "Time", NaN, ...
    "Distance", NaN, ...
    "RightTimedLap", NaN, ...
    "LeftTimedLap", NaN, ...
    "IsValid", false, ...
    "FailureReason", string(reason), ...
    "Metadata", struct( ...
        "PenaltySeconds", 0.0, ...
        "IsPerformanceClaim", false, ...
        "Source", "time-domain Track.EventDistance"));
end

function name = eventMetricName(eventName)
if eventName == "skidpad"
    name = "Skidpad mean of second right and left laps";
elseif eventName == "acceleration"
    name = "Acceleration 75 m time";
elseif eventName == "autocross"
    name = "Autocross course time";
else
    name = "Full course time";
end
end
