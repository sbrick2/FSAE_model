function projection = projectToTrack(track, positionX, positionY, previousIndex, searchWindow)
%PROJECTTOTRACK Project a global position onto a sampled track centerline.
%   The local search window keeps the projection on the same branch at a
%   figure-eight crossing.  The complete-track fallback handles start-up.

arguments
    track (1, 1) struct
    positionX (1, 1) double
    positionY (1, 1) double
    previousIndex (1, 1) double = NaN
    searchWindow (1, 1) double {mustBeInteger, mustBeNonnegative} = 200
end

x = reshape(track.X, [], 1);
y = reshape(track.Y, [], 1);
n = numel(x);
assert(n >= 2, "FSAE:TrackProjectionSamples", ...
    "At least two track samples are required.");

if isfinite(previousIndex) && previousIndex >= 1 && previousIndex <= n
    previousIndex = round(previousIndex);
    if track.IsClosed
        offsets = (-min(searchWindow, floor(n / 2)):min(searchWindow, floor(n / 2)))';
        candidateIndex = mod(previousIndex - 1 + offsets, n) + 1;
    else
        candidateIndex = max(1, previousIndex - searchWindow):min(n, previousIndex + searchWindow);
        candidateIndex = candidateIndex(:);
    end
else
    candidateIndex = (1:n)';
end

distanceSquared = (x(candidateIndex) - positionX).^2 + ...
    (y(candidateIndex) - positionY).^2;
[minimumDistanceSquared, localIndex] = min(distanceSquared);
index = candidateIndex(localIndex);
heading = track.Heading(index);
deltaX = positionX - x(index);
deltaY = positionY - y(index);
lateralError = -deltaX * sin(heading) + deltaY * cos(heading);

projection = struct( ...
    "Index", index, ...
    "PathS", track.SampleDistance * (index - 1), ...
    "ReferenceX", x(index), ...
    "ReferenceY", y(index), ...
    "ReferenceHeading", heading, ...
    "ReferenceCurvature", track.Curvature(index), ...
    "LateralError", lateralError, ...
    "Distance", sqrt(minimumDistanceSquared), ...
    "LeftHalfWidth", track.LeftHalfWidth(index), ...
    "RightHalfWidth", track.RightHalfWidth(index));
end
