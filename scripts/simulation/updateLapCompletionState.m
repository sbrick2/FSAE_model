function [state, isComplete] = updateLapCompletionState( ...
        state, scenario, snapshot)
%UPDATELAPCOMPLETIONSTATE Update event progress from the vehicle position.
%   The vehicle X/Y position is projected onto the local track branch.
%   Open tracks complete at the path end. Closed tracks complete after the
%   requested number of laps. An open track also completes when the vehicle
%   crosses the finish gate in the forward direction while the reference
%   has reached the end of the path. Progress never decreases when
%   projection jitter or brief reverse motion occurs.

arguments
    state (1, 1) struct
    scenario (1, 1) struct
    snapshot (1, 1) struct
end

state = initializeState(state, scenario);
isComplete = state.Completed;
if state.Completed || ~hasValidPosition(snapshot)
    return
end

positionX = double(snapshot.Vehicle.X(end));
positionY = double(snapshot.Vehicle.Y(end));
track = scenario.Track;
finishGateCrossed = crossedOpenTrackFinish( ...
    state, track, snapshot, positionX, positionY);
searchWindow = max(5, ceil(10.0 / track.SampleDistance));
projection = projectToTrack(track, positionX, positionY, ...
    state.PreviousIndex, searchWindow);
pathS = double(projection.PathS);

if isfinite(state.LastPathS)
    delta = pathS - state.LastPathS;
    if track.IsClosed && delta < -0.5 * track.Length
        delta = delta + track.Length;
    elseif track.IsClosed && delta > 0.5 * track.Length
        delta = delta - track.Length;
    end
    state.UnwrappedDistance = state.UnwrappedDistance + delta;
    state.EventDistance = max(state.EventDistance, max(state.UnwrappedDistance, 0));
end

state.PreviousIndex = projection.Index;
state.LastPathS = pathS;
state.PreviousPositionX = positionX;
state.PreviousPositionY = positionY;
state.SampleCount = state.SampleCount + 1;
state.Progress = min(1, state.EventDistance / state.TargetDistance);
distanceComplete = state.EventDistance >= state.TargetDistance - ...
    sqrt(eps) * max(state.TargetDistance, 1);
state.Completed = distanceComplete || finishGateCrossed;
isComplete = state.Completed;
end

function state = initializeState(state, scenario)
requiredFields = ["PreviousIndex", "LastPathS", "EventDistance", ...
    "TargetDistance", "Progress", "SampleCount", "Completed", ...
    "PreviousPositionX", "PreviousPositionY", "UnwrappedDistance"];
if all(isfield(state, requiredFields))
    return
end

track = scenario.Track;
targetLaps = 1;
if track.IsClosed
    targetLaps = double(scenario.NumberOfLaps);
end
state = struct( ...
    "PreviousIndex", NaN, ...
    "LastPathS", NaN, ...
    "UnwrappedDistance", 0, ...
    "EventDistance", 0, ...
    "TargetDistance", double(track.Length) * targetLaps, ...
    "Progress", 0, ...
    "SampleCount", 0, ...
    "Completed", false, ...
    "PreviousPositionX", NaN, ...
    "PreviousPositionY", NaN);
end

function crossed = crossedOpenTrackFinish( ...
        state, track, snapshot, positionX, positionY)
crossed = false;
if track.IsClosed || ~isfinite(state.PreviousPositionX) || ...
        ~isfinite(state.PreviousPositionY) || ...
        ~referenceAtPathEnd(snapshot, track)
    return
end

finishPoint = [double(track.X(end)), double(track.Y(end))];
tangent = finishPoint - ...
    [double(track.X(end - 1)), double(track.Y(end - 1))];
tangentNorm = hypot(tangent(1), tangent(2));
if tangentNorm <= eps
    return
end
tangent = tangent / tangentNorm;
normal = [-tangent(2), tangent(1)];
previousPosition = [state.PreviousPositionX, state.PreviousPositionY];
currentPosition = [positionX, positionY];
previousSide = dot(previousPosition - finishPoint, tangent);
currentSide = dot(currentPosition - finishPoint, tangent);
if previousSide >= 0 || currentSide < 0
    return
end

crossingFraction = -previousSide / (currentSide - previousSide);
crossingPosition = previousPosition + ...
    crossingFraction * (currentPosition - previousPosition);
crossingLateralOffset = abs(dot(crossingPosition - finishPoint, normal));
gateHalfWidth = max( ...
    double(track.LeftHalfWidth(end)), ...
    double(track.RightHalfWidth(end)));
crossed = crossingLateralOffset <= gateHalfWidth;
end

function atEnd = referenceAtPathEnd(snapshot, track)
atEnd = false;
if ~isfield(snapshot, "Track") || ...
        ~isfield(snapshot.Track, "PathS") || ...
        isempty(snapshot.Track.PathS)
    return
end
pathS = double(snapshot.Track.PathS(end));
tolerance = max(double(track.SampleDistance), ...
    sqrt(eps) * max(double(track.Length), 1));
atEnd = isfinite(pathS) && pathS >= double(track.Length) - tolerance;
end

function valid = hasValidPosition(snapshot)
valid = isfield(snapshot, "Vehicle") && ...
    isfield(snapshot.Vehicle, "X") && ...
    isfield(snapshot.Vehicle, "Y") && ...
    ~isempty(snapshot.Vehicle.X) && ~isempty(snapshot.Vehicle.Y) && ...
    isfinite(snapshot.Vehicle.X(end)) && ...
    isfinite(snapshot.Vehicle.Y(end));
end
