function metric = calculateEventMetric(eventName, speedProfile, track)
%CALCULATEEVENTMETRIC Calculate the event-specific quasi-steady time metric.
%   Skidpad uses the average of the second right-hand and second left-hand
%   laps. Acceleration and autocross use the complete speed-profile time.

arguments
    eventName (1, 1) string
    speedProfile (1, 1) struct
    track (1, 1) struct
end

requiredProfileFields = ["LapTime", "TrackLength", "ProgressS", "Time", "Speed"];
assert(all(isfield(speedProfile, requiredProfileFields)), ...
    "FSAE:QuasiStatic:EventMetricProfileContract", ...
    "Speed profile does not contain the fields required for event timing.");

eventKey = lower(strtrim(eventName));
if isfield(speedProfile, "IsValid")
    profileIsValid = logical(speedProfile.IsValid);
else
    profileIsValid = isfinite(speedProfile.LapTime) && speedProfile.LapTime > 0.0;
end
profileFailureReason = "";
if isfield(speedProfile, "FailureReason")
    profileFailureReason = string(speedProfile.FailureReason);
end

switch eventKey
    case {"skidpad", "figureeight", "八字绕环"}
        metric = calculateSkidpadMetric(speedProfile, track, ...
            profileIsValid, profileFailureReason);
    otherwise
        metric = struct( ...
            "Name", "Full course time", ...
            "Time", double(speedProfile.LapTime), ...
            "Distance", double(speedProfile.TrackLength), ...
            "RightTimedLap", NaN, ...
            "LeftTimedLap", NaN, ...
            "IsValid", profileIsValid, ...
            "FailureReason", profileFailureReason, ...
            "Metadata", struct("PenaltySeconds", 0.0, ...
                "IsPerformanceClaim", false));
end
end

function metric = calculateSkidpadMetric( ...
        speedProfile, track, profileIsValid, profileFailureReason)
requiredTrackFields = ["CenterlineRadius", "RightLoopsEndS", "FinishLineS"];
assert(all(isfield(track, requiredTrackFields)), ...
    "FSAE:QuasiStatic:SkidpadTimingContract", ...
    "Skidpad track must contain CenterlineRadius, RightLoopsEndS and FinishLineS.");

lapDistance = 2.0 * pi * double(track.CenterlineRadius);
rightEnd = double(track.RightLoopsEndS);
rightStart = rightEnd - lapDistance;
leftEnd = double(track.FinishLineS);
leftStart = leftEnd - lapDistance;

[rightTime, rightValid] = sectionTime(speedProfile, rightStart, rightEnd);
[leftTime, leftValid] = sectionTime(speedProfile, leftStart, leftEnd);
metricValid = profileIsValid && rightValid && leftValid;
if ~profileIsValid
    failureReason = profileFailureReason;
elseif ~rightValid || ~leftValid
    failureReason = "A skidpad timed lap contains no valid vehicle motion.";
else
    failureReason = "";
end

metric = struct( ...
    "Name", "Skidpad mean of second right and left laps", ...
    "Time", mean([rightTime, leftTime]), ...
    "Distance", lapDistance, ...
    "RightTimedLap", rightTime, ...
    "LeftTimedLap", leftTime, ...
    "IsValid", logical(metricValid), ...
    "FailureReason", failureReason, ...
    "Metadata", struct( ...
        "RightStartS", rightStart, "RightEndS", rightEnd, ...
        "LeftStartS", leftStart, "LeftEndS", leftEnd, ...
        "PenaltySeconds", 0.0, "IsPerformanceClaim", false));
end

function [elapsedTime, isValid] = sectionTime(speedProfile, startS, endS)
progress = double(speedProfile.ProgressS(:));
time = double(speedProfile.Time(:));
speed = double(speedProfile.Speed(:));
sampleCount = numel(progress);
assert(numel(speed) == sampleCount && numel(time) >= sampleCount, ...
    "FSAE:QuasiStatic:EventMetricSize", ...
    "Speed, progress and time vectors must describe the same track samples.");
assert(startS >= progress(1) && endS <= progress(end) && endS > startS, ...
    "FSAE:QuasiStatic:EventMetricRange", ...
    "Timed section must lie within the available open-track progress range.");

sampleTime = time(1:sampleCount);
startTime = interp1(progress, sampleTime, startS, "linear");
endTime = interp1(progress, sampleTime, endS, "linear");
sectionMask = progress >= startS & progress <= endS;
speedEpsilon = 0.1;
if isfield(speedProfile, "Metadata") && ...
        isfield(speedProfile.Metadata, "SpeedEpsilon")
    speedEpsilon = double(speedProfile.Metadata.SpeedEpsilon);
end
elapsedTime = endTime - startTime;
isValid = isfinite(elapsedTime) && elapsedTime > 0.0 && ...
    any(speed(sectionMask) > speedEpsilon);
end
