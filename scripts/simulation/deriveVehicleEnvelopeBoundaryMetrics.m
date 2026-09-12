function result = deriveVehicleEnvelopeBoundaryMetrics( ...
        result, scenario, parameters, options)
%DERIVEVEHICLEENVELOPEBOUNDARYMETRICS Check nominal tire outer-edge clearance.
%   The four evaluated points are the left/right outer edges at the front
%   and rear axle lines. Tire width defaults to the nominal 7.5 in section
%   width used by the selected Formula Student tire. This is a geometric
%   envelope check, not a deforming-tire contact-patch reconstruction.

arguments
    result (1, 1) struct
    scenario (1, 1) struct
    parameters (1, 1) struct
    options.TireSectionWidth (1, 1) double {mustBePositive} = 0.1905
    options.SearchWindow (1, 1) double ...
        {mustBeInteger, mustBePositive} = 100
end

sampleCount = numel(result.Time);
clearance = NaN(sampleCount, 1);
violation = NaN(sampleCount, 1);
if sampleCount == 0 || isempty(result.Vehicle.X) || ...
        isempty(result.Vehicle.Y) || isempty(result.Vehicle.Psi)
    result = installMetrics(result, clearance, violation, options);
    return
end

wheelbase = requiredValue(parameters, "Vehicle", "Wheelbase");
frontDistance = requiredValue(parameters, "Vehicle", "CGToFrontAxle");
rearDistance = wheelbase - frontDistance;
frontHalfEnvelope = 0.5 * requiredValue( ...
    parameters, "Vehicle", "TrackFront") + ...
    0.5 * options.TireSectionWidth;
rearHalfEnvelope = 0.5 * requiredValue( ...
    parameters, "Vehicle", "TrackRear") + ...
    0.5 * options.TireSectionWidth;
localPoints = [ ...
    frontDistance, frontHalfEnvelope; ...
    frontDistance, -frontHalfEnvelope; ...
    -rearDistance, rearHalfEnvelope; ...
    -rearDistance, -rearHalfEnvelope];
previousIndex = NaN(4, 1);

for sampleIndex = 1:sampleCount
    positionX = result.Vehicle.X(sampleIndex);
    positionY = result.Vehicle.Y(sampleIndex);
    heading = result.Vehicle.Psi(sampleIndex);
    if ~isfinite(positionX) || ~isfinite(positionY) || ~isfinite(heading)
        continue
    end
    cosineHeading = cos(heading);
    sineHeading = sin(heading);
    sampleClearance = Inf;
    for pointIndex = 1:4
        localX = localPoints(pointIndex, 1);
        localY = localPoints(pointIndex, 2);
        globalX = positionX + localX * cosineHeading - ...
            localY * sineHeading;
        globalY = positionY + localX * sineHeading + ...
            localY * cosineHeading;
        [pointClearance, previousIndex(pointIndex)] = ...
            projectPointClearance(scenario.Track, globalX, globalY, ...
            previousIndex(pointIndex), options.SearchWindow);
        if isfinite(pointClearance)
            sampleClearance = min(sampleClearance, pointClearance);
        end
    end
    if isfinite(sampleClearance)
        clearance(sampleIndex) = sampleClearance;
        violation(sampleIndex) = max(0.0, -sampleClearance);
    end
end

result = installMetrics(result, clearance, violation, options);
end

function result = installMetrics(result, clearance, violation, options)
result.Track.VehicleEnvelopeClearance = clearance;
result.Track.VehicleEnvelopeViolation = violation;
valid = isfinite(clearance) & isfinite(violation);
result.Meta.VehicleEnvelopeDataValid = ...
    ~isempty(valid) && all(valid);
if any(valid)
    result.Metrics.MinimumVehicleEnvelopeClearance = min(clearance(valid));
    result.Metrics.MaximumVehicleEnvelopeViolation = max(violation(valid));
    result.Metrics.VehicleEnvelopeViolationCount = ...
        nnz(violation(valid) > 0);
    result.Metrics.VehicleEnvelopeViolationFraction = ...
        mean(violation(valid) > 0);
else
    result.Metrics.MinimumVehicleEnvelopeClearance = NaN;
    result.Metrics.MaximumVehicleEnvelopeViolation = NaN;
    result.Metrics.VehicleEnvelopeViolationCount = 0;
    result.Metrics.VehicleEnvelopeViolationFraction = NaN;
end
result.Meta.VehicleEnvelopeWithinBoundary = ...
    result.Meta.VehicleEnvelopeDataValid && ...
    result.Metrics.MaximumVehicleEnvelopeViolation <= 1.0e-9;
result.Meta.VehicleEnvelopeMethod = ...
    "four nominal tire outer-edge points";
result.Meta.VehicleEnvelopeTireSectionWidth = ...
    options.TireSectionWidth;
end

function [clearance, bestIndex] = projectPointClearance( ...
        track, pointX, pointY, previousIndex, searchWindow)
x = reshape(track.X, [], 1);
y = reshape(track.Y, [], 1);
sampleCount = numel(x);
if sampleCount < 2
    clearance = NaN;
    bestIndex = NaN;
    return
end
if isfinite(previousIndex) && previousIndex >= 1 && ...
        previousIndex <= sampleCount
    centerIndex = min(sampleCount - 1, max(1, round(previousIndex)));
    window = min(searchWindow, floor(sampleCount / 2));
    if track.IsClosed
        offsets = (-window:window).';
        candidate = mod(centerIndex - 1 + offsets, sampleCount) + 1;
    else
        candidate = (max(1, centerIndex - window): ...
            min(sampleCount - 1, centerIndex + window)).';
    end
else
    if track.IsClosed
        candidate = (1:sampleCount).';
    else
        candidate = (1:sampleCount - 1).';
    end
end
nextCandidate = candidate + 1;
if track.IsClosed
    nextCandidate = mod(candidate, sampleCount) + 1;
end

segmentX = x(nextCandidate) - x(candidate);
segmentY = y(nextCandidate) - y(candidate);
segmentLengthSquared = segmentX.^2 + segmentY.^2;
fraction = ((pointX - x(candidate)) .* segmentX + ...
    (pointY - y(candidate)) .* segmentY) ./ ...
    max(segmentLengthSquared, eps);
fraction = min(1.0, max(0.0, fraction));
projectedX = x(candidate) + fraction .* segmentX;
projectedY = y(candidate) + fraction .* segmentY;
distanceSquared = (pointX - projectedX).^2 + ...
    (pointY - projectedY).^2;
[~, localIndex] = min(distanceSquared);
bestIndex = candidate(localIndex);
bestFraction = fraction(localIndex);
bestNext = nextCandidate(localIndex);
segmentHeading = atan2(segmentY(localIndex), segmentX(localIndex));
lateralError = -(pointX - projectedX(localIndex)) * ...
    sin(segmentHeading) + (pointY - projectedY(localIndex)) * ...
    cos(segmentHeading);
leftWidth = track.LeftHalfWidth(bestIndex) + bestFraction * ...
    (track.LeftHalfWidth(bestNext) - track.LeftHalfWidth(bestIndex));
rightWidth = track.RightHalfWidth(bestIndex) + bestFraction * ...
    (track.RightHalfWidth(bestNext) - track.RightHalfWidth(bestIndex));
if lateralError >= 0
    clearance = leftWidth - lateralError;
else
    clearance = rightWidth + lateralError;
end
end

function value = requiredValue(parameters, groupName, fieldName)
assert(isfield(parameters, groupName) && ...
    isfield(parameters.(groupName), fieldName), ...
    "FSAE:Envelope:MissingParameter", ...
    "Missing vehicle-envelope parameter %s.%s.", groupName, fieldName);
value = parameters.(groupName).(fieldName);
if isstruct(value) && isfield(value, "Value")
    value = value.Value;
end
assert(isnumeric(value) && isscalar(value) && isfinite(value), ...
    "FSAE:Envelope:InvalidParameter", ...
    "Vehicle-envelope parameter %s.%s must be finite and scalar.", ...
    groupName, fieldName);
value = double(value);
end
