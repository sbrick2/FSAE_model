function report = crossValidateQuasiStaticWithTimeDomain(ggv, timeDomainResult, options)
%CROSSVALIDATEQuasiStaticWITHTIMEDOMAIN Compare PathTracking signals with the QuasiStatic test envelope.
%   The input can be the normalized result returned by
%   collectOpenLoopResults. The comparison is diagnostic: it reports
%   envelope violations and does not tune either model to force agreement.

arguments
    ggv (1, 1) struct
    timeDomainResult (1, 1) struct
    options.Tolerance (1, 1) double {mustBeNonnegative} = 0.25
end

required = ["Time", "Vehicle"];
assert(all(isfield(timeDomainResult, required)), ...
    "FSAE:QuasiStatic:TimeDomainContract", ...
    "timeDomainResult must contain Time and Vehicle fields.");
assert(all(isfield(timeDomainResult.Vehicle, ["Ux", "Ax", "Ay"])), ...
    "FSAE:QuasiStatic:TimeDomainSignals", ...
    "Vehicle.Ux, Vehicle.Ax and Vehicle.Ay are required.");

time = double(timeDomainResult.Time(:));
speed = double(timeDomainResult.Vehicle.Ux(:));
ax = double(timeDomainResult.Vehicle.Ax(:));
ay = double(timeDomainResult.Vehicle.Ay(:));
sampleCount = numel(time);
assert(numel(speed) == sampleCount && numel(ax) == sampleCount && ...
    numel(ay) == sampleCount, "FSAE:QuasiStatic:TimeDomainSignalSize", ...
    "Time, Vehicle.Ux, Vehicle.Ax and Vehicle.Ay must have equal lengths.");
valid = isfinite(time) & isfinite(speed) & isfinite(ax) & isfinite(ay);
time = time(valid);
speed = max(0.0, speed(valid));
ax = ax(valid);
ay = ay(valid);
assert(~isempty(time), "FSAE:QuasiStatic:NoTimeDomainSamples", ...
    "No finite time-domain samples were provided.");

upperBound = zeros(size(time));
lowerBound = zeros(size(time));
ayUpperBound = zeros(size(time));
ayLowerBound = zeros(size(time));
upperViolation = zeros(size(time));
lowerViolation = zeros(size(time));
lateralUpperViolation = zeros(size(time));
lateralLowerViolation = zeros(size(time));
for index = 1:numel(time)
    ayUpperBound(index) = max(0.0, interp1(ggv.Speed, ...
        ggv.AyPositive, speed(index), "linear", "extrap"));
    ayLowerBound(index) = -max(0.0, interp1(ggv.Speed, ...
        ggv.AyNegative, speed(index), "linear", "extrap"));
    upperBound(index) = lookupAx(ggv, speed(index), ay(index), true);
    lowerBound(index) = lookupAx(ggv, speed(index), ay(index), false);
    upperViolation(index) = max(0.0, ax(index) - upperBound(index));
    lowerViolation(index) = max(0.0, lowerBound(index) - ax(index));
    lateralUpperViolation(index) = max(0.0, ay(index) - ayUpperBound(index));
    lateralLowerViolation(index) = max(0.0, ayLowerBound(index) - ay(index));
end

report = struct( ...
    "Pass", max([upperViolation; lowerViolation; ...
        lateralUpperViolation; lateralLowerViolation]) <= options.Tolerance, ...
    "Tolerance", options.Tolerance, ...
    "Time", time, "Speed", speed, "Ax", ax, "Ay", ay, ...
    "AxUpperBound", upperBound, "AxLowerBound", lowerBound, ...
    "AyUpperBound", ayUpperBound, "AyLowerBound", ayLowerBound, ...
    "UpperViolation", upperViolation, "LowerViolation", lowerViolation, ...
    "LateralUpperViolation", lateralUpperViolation, ...
    "LateralLowerViolation", lateralLowerViolation, ...
    "MaxUpperViolation", max(upperViolation), ...
    "MaxLowerViolation", max(lowerViolation), ...
    "MaxLateralUpperViolation", max(lateralUpperViolation), ...
    "MaxLateralLowerViolation", max(lateralLowerViolation), ...
    "PeakAx", max(ax), "PeakBrakeAx", min(ax), ...
    "PeakAy", max(abs(ay)), ...
    "Metadata", struct("Method", ...
        "PathTracking normalized signals against QuasiStatic longitudinal and lateral GGV bounds", ...
        "IsPerformanceClaim", false));
end

function ax = lookupAx(ggv, speed, ay, useMaximum)
positive = interp1(ggv.Speed, ggv.AyPositive, speed, "linear", "extrap");
negative = interp1(ggv.Speed, ggv.AyNegative, speed, "linear", "extrap");
if ay >= 0
    fraction = ay / max(positive, 1.0e-9);
else
    fraction = ay / max(negative, 1.0e-9);
end
fraction = max(-1.0, min(1.0, fraction));
values = zeros(numel(ggv.Speed), 1);
for index = 1:numel(ggv.Speed)
    if useMaximum
        values(index) = interp1(ggv.LateralFraction, ggv.AxMax(index, :), ...
            fraction, "linear", "extrap");
    else
        values(index) = interp1(ggv.LateralFraction, ggv.AxMin(index, :), ...
            fraction, "linear", "extrap");
    end
end
ax = interp1(ggv.Speed, values, speed, "linear", "extrap");
end
