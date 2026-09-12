function values = createSweepValues(startValue, stopValue, stepValue)
%CREATESWEEPVALUES Create a finite regularly spaced sweep vector.

arguments
    startValue (1, 1) double {mustBeFinite}
    stopValue (1, 1) double {mustBeFinite}
    stepValue (1, 1) double {mustBePositive, mustBeFinite}
end

assert(stopValue >= startValue, "FSAE:QuasiStatic:InvalidScanRange", ...
    "Stop must be greater than or equal to Start.");
scale = max([1.0, abs(startValue), abs(stopValue), abs(stepValue)]);
tolerance = 64.0 * eps(scale);
stepCount = floor((stopValue - startValue) / stepValue + tolerance);
values = startValue + (0:stepCount) * stepValue;
if stopValue - values(end) >= 0.0 && stopValue - values(end) <= tolerance
    values(end) = stopValue;
end
assert(~isempty(values) && values(end) <= stopValue + tolerance, ...
    "FSAE:QuasiStatic:EmptyScanRange", "The requested sweep range is empty.");

if numel(values) >= 2
    spacingError = abs(diff(values) - stepValue);
    spacingTolerance = 128.0 * eps(max(1.0, max(abs(values))));
    assert(all(spacingError <= spacingTolerance), ...
        "FSAE:QuasiStatic:IrregularScanRange", ...
        "The sweep grid cannot be represented with the requested regular spacing.");
end
end
