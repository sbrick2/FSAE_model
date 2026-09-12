function [ReferenceX, ReferenceY, ReferenceHeading, ReferenceCurvature, ...
    ReferenceSpeed, PathS, LeftHalfWidth, RightHalfWidth, Valid] = ...
    fsaePathTrackingTrackReferenceCore(sensor, trackData)
%FSAEPathTrackingTRACKREFERENCECORE Code-generation-safe projection and look-ahead.
%   The vehicle is projected onto centerline segments instead of selecting
%   the nearest sampled point.  Look-ahead outputs are interpolated by path
%   distance so steering does not receive a staircase input at the track
%   sample-passing frequency.

persistent previousIndex
if isempty(previousIndex)
    previousIndex = 1;
end

sampleCount = double(trackData.SampleCount);
Valid = false;
ReferenceX = 0;
ReferenceY = 0;
ReferenceHeading = 0;
ReferenceCurvature = 0;
ReferenceSpeed = 0;
PathS = 0;
LeftHalfWidth = 0;
RightHalfWidth = 0;
if sampleCount < 2 || ~sensor.PoseValid
    return
end

if previousIndex < 1 || previousIndex > sampleCount
    previousIndex = 1;
end
window = min(PathTrackingProjectionSearchWindow, floor(sampleCount / 2));
bestIndex = previousIndex;
bestFraction = 0;
bestDistance = realmax;
for offset = -window:window
    if trackData.IsClosed
        candidate = mod(previousIndex - 1 + offset, sampleCount) + 1;
        nextCandidate = mod(candidate, sampleCount) + 1;
    else
        candidate = min(sampleCount - 1, ...
            max(1, previousIndex + offset));
        nextCandidate = candidate + 1;
    end

    segmentX = trackData.X(nextCandidate) - trackData.X(candidate);
    segmentY = trackData.Y(nextCandidate) - trackData.Y(candidate);
    segmentLengthSquared = segmentX^2 + segmentY^2;
    if segmentLengthSquared > eps
        projectionFraction = ( ...
            (sensor.PositionX - trackData.X(candidate)) * segmentX + ...
            (sensor.PositionY - trackData.Y(candidate)) * segmentY) / ...
            segmentLengthSquared;
        projectionFraction = min(1, max(0, projectionFraction));
    else
        projectionFraction = 0;
    end
    projectedX = trackData.X(candidate) + ...
        projectionFraction * segmentX;
    projectedY = trackData.Y(candidate) + ...
        projectionFraction * segmentY;
    distance = (projectedX - sensor.PositionX)^2 + ...
        (projectedY - sensor.PositionY)^2;
    if distance < bestDistance
        bestDistance = distance;
        bestIndex = candidate;
        bestFraction = projectionFraction;
    end
end
previousIndex = bestIndex;

segmentStartS = (bestIndex - 1) * trackData.SampleDistance;
segmentPathLength = min(trackData.SampleDistance, ...
    max(0, trackData.TrackLength - segmentStartS));
if trackData.IsClosed && bestIndex == sampleCount
    segmentPathLength = trackData.TrackLength - segmentStartS;
end
currentS = segmentStartS + bestFraction * segmentPathLength;
lookAhead = max(PathTrackingMinLookahead, ...
    abs(sensor.LongitudinalSpeed) * PathTrackingLookaheadTime);
if trackData.IsClosed
    targetS = mod(currentS + lookAhead, trackData.TrackLength);
    targetCoordinate = targetS / trackData.SampleDistance;
    targetBase = floor(targetCoordinate);
    targetIndex = mod(targetBase, sampleCount) + 1;
    nextTargetIndex = mod(targetIndex, sampleCount) + 1;
    targetFraction = targetCoordinate - targetBase;
else
    targetS = min(trackData.TrackLength, currentS + lookAhead);
    if targetS >= trackData.TrackLength
        targetIndex = sampleCount;
        nextTargetIndex = sampleCount;
        targetFraction = 0;
    else
        targetCoordinate = targetS / trackData.SampleDistance;
        targetBase = min(sampleCount - 2, floor(targetCoordinate));
        targetIndex = targetBase + 1;
        nextTargetIndex = targetIndex + 1;
        targetFraction = targetCoordinate - targetBase;
    end
end

ReferenceX = trackData.X(targetIndex) + targetFraction * ...
    (trackData.X(nextTargetIndex) - trackData.X(targetIndex));
ReferenceY = trackData.Y(targetIndex) + targetFraction * ...
    (trackData.Y(nextTargetIndex) - trackData.Y(targetIndex));
headingDifference = atan2( ...
    sin(trackData.Heading(nextTargetIndex) - ...
    trackData.Heading(targetIndex)), ...
    cos(trackData.Heading(nextTargetIndex) - ...
    trackData.Heading(targetIndex)));
ReferenceHeading = trackData.Heading(targetIndex) + ...
    targetFraction * headingDifference;
ReferenceHeading = atan2(sin(ReferenceHeading), cos(ReferenceHeading));
ReferenceCurvature = trackData.Curvature(targetIndex) + ...
    targetFraction * (trackData.Curvature(nextTargetIndex) - ...
    trackData.Curvature(targetIndex));
ReferenceSpeed = trackData.ReferenceSpeed(targetIndex) + ...
    targetFraction * (trackData.ReferenceSpeed(nextTargetIndex) - ...
    trackData.ReferenceSpeed(targetIndex));
PathS = targetS;
LeftHalfWidth = trackData.LeftHalfWidth(targetIndex) + targetFraction * ...
    (trackData.LeftHalfWidth(nextTargetIndex) - ...
    trackData.LeftHalfWidth(targetIndex));
RightHalfWidth = trackData.RightHalfWidth(targetIndex) + targetFraction * ...
    (trackData.RightHalfWidth(nextTargetIndex) - ...
    trackData.RightHalfWidth(targetIndex));
Valid = true;
end
