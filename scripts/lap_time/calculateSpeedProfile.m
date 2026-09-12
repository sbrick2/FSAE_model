function profile = calculateSpeedProfile(track, ggv, options)
%CALCULATESPEEDPROFILE Apply curvature, acceleration and braking limits.
%   PROFILE = CALCULATESPEEDPROFILE(TRACK, GGV) performs repeated forward
%   and backward passes over the PathTracking track contract. The longitudinal bounds
%   are interpolated from the speed-dependent QuasiStatic GGV surface at the local
%   lateral-acceleration demand.

arguments
    track (1, 1) struct
    ggv (1, 1) struct
    options.UseReferenceSpeed (1, 1) logical = false
    options.InitialSpeed (1, 1) double {mustBeNonnegative} = 0.0
    options.FinalSpeed (1, 1) double {mustBeNonnegative} = 0.0
    options.PassCount (1, 1) double {mustBeInteger, mustBePositive} = 4
    options.MinimumSpeed (1, 1) double {mustBeNonnegative} = 0.0
    options.MaximumSpeed (1, 1) double {mustBePositive} = Inf
end

requiredTrackFields = ["Curvature", "SampleDistance", "IsClosed"];
assert(all(isfield(track, requiredTrackFields)), "FSAE:QuasiStatic:TrackContract", ...
    "Track must contain Curvature, SampleDistance and IsClosed.");
curvature = double(track.Curvature(:));
sampleCount = numel(curvature);
assert(sampleCount >= 2, "FSAE:QuasiStatic:TrackSamples", ...
    "At least two track samples are required.");
sampleDistance = double(track.SampleDistance);
assert(isscalar(sampleDistance) && sampleDistance > 0 && isfinite(sampleDistance), ...
    "FSAE:QuasiStatic:TrackSpacing", "Track.SampleDistance must be a finite positive scalar.");
speedGrid = double(ggv.Speed(:));
maximumSpeed = min(options.MaximumSpeed, max(speedGrid));

curvatureLimit = zeros(sampleCount, 1);
for index = 1:sampleCount
    curvatureLimit(index) = findCurvatureSpeedLimit( ...
        curvature(index), ggv, maximumSpeed);
end
speed = curvatureLimit;
if options.UseReferenceSpeed && isfield(track, "ReferenceSpeed")
    referenceSpeed = max(0.0, double(track.ReferenceSpeed(:)));
    assert(numel(referenceSpeed) == sampleCount, ...
        "FSAE:QuasiStatic:ReferenceSpeedSize", ...
        "Track.ReferenceSpeed must match the curvature sample count.");
    speed = min(speed, referenceSpeed);
end
speed = max(options.MinimumSpeed, min(speed, maximumSpeed));

if track.IsClosed
    for pass = 1:options.PassCount
        speed = forwardPass(speed, curvature, sampleDistance, ggv);
        speed = backwardPass(speed, curvature, sampleDistance, ggv);
    end
else
    speed(1) = min(speed(1), options.InitialSpeed);
    speed(end) = min(speed(end), options.FinalSpeed);
    for pass = 1:options.PassCount
        speed = forwardPassOpen(speed, curvature, sampleDistance, ggv);
        speed = backwardPassOpen(speed, curvature, sampleDistance, ggv);
    end
end

progress = sampleDistance * (0:sampleCount - 1)';
intervalCount = sampleCount;
if ~track.IsClosed
    intervalCount = sampleCount - 1;
end
profile = struct( ...
    "Speed", speed, ...
    "Curvature", curvature, ...
    "LateralAcceleration", speed.^2 .* curvature, ...
    "CurvatureSpeedLimit", curvatureLimit, ...
    "ProgressS", progress, ...
    "SampleDistance", sampleDistance, ...
    "IsClosed", logical(track.IsClosed), ...
    "TrackLength", sampleDistance * intervalCount, ...
    "GGVSpeedGrid", speedGrid, ...
    "Metadata", struct( ...
        "UseReferenceSpeed", options.UseReferenceSpeed, ...
        "PassCount", options.PassCount, ...
        "IsPerformanceClaim", false));
timeResult = calculateLapTime(profile, track);
profile.LapTime = timeResult.LapTime;
profile.Time = timeResult.Time;
profile.IsValid = timeResult.IsValid;
profile.FailureReason = timeResult.FailureReason;
profile.ZeroSpeedFraction = timeResult.ZeroSpeedFraction;
profile.StationaryIntervalFraction = timeResult.StationaryIntervalFraction;
end

function speed = forwardPass(speed, curvature, ds, ggv)
count = numel(speed);
for index = 1:count
    next = mod(index, count) + 1;
    ay = speed(index)^2 * curvature(index);
    ax = lookupAx(ggv, speed(index), ay, true);
    candidate = sqrt(max(0.0, speed(index)^2 + 2.0 * max(ax, 0.0) * ds));
    speed(next) = min(speed(next), candidate);
end
end

function speed = backwardPass(speed, curvature, ds, ggv)
count = numel(speed);
for index = count:-1:1
    previous = mod(index - 2, count) + 1;
    ay = speed(index)^2 * curvature(index);
    ax = lookupAx(ggv, speed(index), ay, false);
    deceleration = max(-ax, 0.0);
    candidate = sqrt(max(0.0, speed(index)^2 + 2.0 * deceleration * ds));
    speed(previous) = min(speed(previous), candidate);
end
end

function speed = forwardPassOpen(speed, curvature, ds, ggv)
for index = 1:numel(speed) - 1
    ay = speed(index)^2 * curvature(index);
    ax = lookupAx(ggv, speed(index), ay, true);
    candidate = sqrt(max(0.0, speed(index)^2 + 2.0 * max(ax, 0.0) * ds));
    speed(index + 1) = min(speed(index + 1), candidate);
end
end

function speed = backwardPassOpen(speed, curvature, ds, ggv)
for index = numel(speed):-1:2
    ay = speed(index)^2 * curvature(index);
    ax = lookupAx(ggv, speed(index), ay, false);
    deceleration = max(-ax, 0.0);
    candidate = sqrt(max(0.0, speed(index)^2 + 2.0 * deceleration * ds));
    speed(index - 1) = min(speed(index - 1), candidate);
end
end

function limit = findCurvatureSpeedLimit(curvature, ggv, maximumSpeed)
curvatureMagnitude = abs(curvature);
if curvatureMagnitude <= 1.0e-12
    limit = maximumSpeed;
    return
end
low = 0.0;
high = maximumSpeed;
if lateralLimit(ggv, high, curvature) >= high^2 * curvatureMagnitude
    limit = high;
    return
end
for iteration = 1:40
    mid = 0.5 * (low + high);
    if lateralLimit(ggv, mid, curvature) >= mid^2 * curvatureMagnitude
        low = mid;
    else
        high = mid;
    end
    if high - low < 1.0e-5
        break
    end
end
limit = low;
end

function value = lateralLimit(ggv, speed, curvature)
positive = interp1(ggv.Speed, ggv.AyPositive, speed, "linear", "extrap");
negative = interp1(ggv.Speed, ggv.AyNegative, speed, "linear", "extrap");
if curvature >= 0.0
    value = max(0.0, positive);
else
    value = max(0.0, negative);
end
end

function ax = lookupAx(ggv, speed, ay, useMaximum)
if abs(ay) <= 1.0e-12
    fraction = 0.0;
else
    positive = interp1(ggv.Speed, ggv.AyPositive, speed, "linear", "extrap");
    negative = interp1(ggv.Speed, ggv.AyNegative, speed, "linear", "extrap");
    if ay >= 0
        limit = positive;
    else
        limit = negative;
    end
    fraction = ay / max(limit, 1.0e-9);
end
fraction = max(-1.0, min(1.0, fraction));
rowValues = zeros(numel(ggv.Speed), 1);
for index = 1:numel(ggv.Speed)
    if useMaximum
        rowValues(index) = interp1(ggv.LateralFraction, ...
            ggv.AxMax(index, :), fraction, "linear", "extrap");
    else
        rowValues(index) = interp1(ggv.LateralFraction, ...
            ggv.AxMin(index, :), fraction, "linear", "extrap");
    end
end
ax = interp1(ggv.Speed, rowValues, speed, "linear", "extrap");
if ~isfinite(ax)
    ax = 0.0;
end
end
