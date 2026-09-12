function result = calculateLapTime(speedProfile, track, options)
%CALCULATELAPTIME Integrate distance over the QuasiStatic speed profile.

arguments
    speedProfile (1, 1) struct
    track (1, 1) struct
    options.SpeedEpsilon (1, 1) double {mustBePositive} = 0.1
end

speed = max(0.0, double(speedProfile.Speed(:)));
sampleCount = numel(speed);
assert(sampleCount >= 2, "FSAE:QuasiStatic:LapSpeedSamples", ...
    "At least two speed samples are required.");
ds = double(speedProfile.SampleDistance);
assert(isscalar(ds) && isfinite(ds) && ds > 0, ...
    "FSAE:QuasiStatic:LapSpacing", "Speed profile sample spacing must be positive.");

if logical(track.IsClosed)
    nextIndex = [2:sampleCount, 1]';
    intervalDistance = ds * ones(sampleCount, 1);
else
    nextIndex = (2:sampleCount)';
    intervalDistance = ds * ones(sampleCount - 1, 1);
end
intervalSpeed = max(options.SpeedEpsilon, 0.5 * (speed(1:numel(nextIndex)) + speed(nextIndex)));
deltaTime = intervalDistance ./ intervalSpeed;
time = [0.0; cumsum(deltaTime)];
stationaryInterval = speed(1:numel(nextIndex)) <= options.SpeedEpsilon & ...
    speed(nextIndex) <= options.SpeedEpsilon;
isValid = all(isfinite(speed)) && all(isfinite(deltaTime)) && ...
    any(speed > options.SpeedEpsilon) && ~any(stationaryInterval);
invalidReason = makeInvalidReason(speed, deltaTime, stationaryInterval, options.SpeedEpsilon);
result = struct( ...
    "LapTime", sum(deltaTime), ...
    "Distance", sum(intervalDistance), ...
    "Time", time, ...
    "Speed", speed, ...
    "DeltaTime", deltaTime, ...
    "IntervalSpeed", intervalSpeed, ...
    "ZeroSpeedFraction", mean(speed <= options.SpeedEpsilon), ...
    "StationaryIntervalFraction", mean(stationaryInterval), ...
    "IsValid", logical(isValid), ...
    "FailureReason", invalidReason, ...
    "IsClosed", logical(track.IsClosed), ...
    "Metadata", struct("SpeedEpsilon", options.SpeedEpsilon, ...
        "IsPerformanceClaim", false));
end

function reason = makeInvalidReason(speed, deltaTime, stationaryInterval, speedEpsilon)
if any(~isfinite(speed)) || any(~isfinite(deltaTime))
    reason = "Speed profile or integrated time contains a non-finite value.";
elseif ~any(speed > speedEpsilon)
    reason = "Speed profile contains no motion above SpeedEpsilon.";
elseif any(stationaryInterval)
    reason = "Speed profile contains an interval with no motion at either endpoint.";
else
    reason = "";
end
end
