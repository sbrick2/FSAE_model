function track = orientPathTrackingClosedTrack(track, desiredDirection)
%ORIENTPathTrackingCLOSEDTRACK Enforce a traversal direction on a closed PathTracking track.
%   The first sample remains the start point. When the sample order is
%   reversed, left/right widths are exchanged and direction-dependent
%   heading and curvature are recalculated.

arguments
    track (1, 1) struct
    desiredDirection (1, 1) string {mustBeMember( ...
        desiredDirection, ["counterclockwise", "clockwise"])} = ...
        "counterclockwise"
end

requiredFields = ["IsClosed", "SampleDistance", "X", "Y", "Heading", ...
    "Curvature", "ReferenceSpeed", "LeftHalfWidth", "RightHalfWidth"];
assert(all(isfield(track, requiredFields)), "FSAE:TrackDirectionFields", ...
    "Track is missing fields required to enforce its traversal direction.");
assert(logical(track.IsClosed), "FSAE:TrackDirectionOpenTrack", ...
    "Traversal direction can only be enforced on a closed track.");

x = reshape(double(track.X), [], 1);
y = reshape(double(track.Y), [], 1);
sampleCount = numel(x);
assert(sampleCount >= 3 && numel(y) == sampleCount, ...
    "FSAE:TrackDirectionSamples", ...
    "A closed track requires at least three paired X/Y samples.");

signedArea = calculateSignedArea(x, y);
areaScale = max((max(x) - min(x)) * (max(y) - min(y)), 1.0);
assert(abs(signedArea) > sqrt(eps) * areaScale, ...
    "FSAE:TrackDirectionDegenerate", ...
    "Track signed area is too small to determine a traversal direction.");

isCounterclockwise = signedArea > 0;
shouldReverse = (desiredDirection == "counterclockwise" && ...
    ~isCounterclockwise) || (desiredDirection == "clockwise" && ...
    isCounterclockwise);

if shouldReverse
    order = [1; (sampleCount:-1:2)'];
    track.X = x(order);
    track.Y = y(order);
    track.ReferenceSpeed = reshape(track.ReferenceSpeed, [], 1);
    track.ReferenceSpeed = track.ReferenceSpeed(order);

    originalLeftWidth = reshape(track.LeftHalfWidth, [], 1);
    originalRightWidth = reshape(track.RightHalfWidth, [], 1);
    track.LeftHalfWidth = originalRightWidth(order);
    track.RightHalfWidth = originalLeftWidth(order);

    sampleDistance = double(track.SampleDistance);
    dx = (circshift(track.X, -1) - circshift(track.X, 1)) / ...
        (2 * sampleDistance);
    dy = (circshift(track.Y, -1) - circshift(track.Y, 1)) / ...
        (2 * sampleDistance);
    ddx = (circshift(track.X, -1) - 2 * track.X + ...
        circshift(track.X, 1)) / sampleDistance^2;
    ddy = (circshift(track.Y, -1) - 2 * track.Y + ...
        circshift(track.Y, 1)) / sampleDistance^2;
    track.Heading = unwrap(atan2(dy, dx));
    track.Curvature = (dx .* ddy - dy .* ddx) ./ ...
        max((dx.^2 + dy.^2).^(3/2), eps);
end

track.TraversalDirection = desiredDirection;
track.DirectionWasReversed = shouldReverse;
track.SignedArea = calculateSignedArea(track.X, track.Y);
end

function signedArea = calculateSignedArea(x, y)
x = reshape(double(x), [], 1);
y = reshape(double(y), [], 1);
signedArea = 0.5 * sum(x .* circshift(y, -1) - ...
    circshift(x, -1) .* y);
end
