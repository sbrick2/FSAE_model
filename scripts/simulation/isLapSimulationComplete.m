function completed = isLapSimulationComplete(result, scenario)
%ISLAPSIMULATIONCOMPLETE Check full-event progress or valid finish timing.
targetDistance = double(scenario.Track.Length);
if scenario.Track.IsClosed
    targetDistance = targetDistance * double(scenario.NumberOfLaps);
end
tolerance = sqrt(eps) * max(targetDistance, 1.0);
completed = false;
if isfield(result, "Track") && isfield(result.Track, "EventDistance")
    distance = double(result.Track.EventDistance(:));
    completed = any(isfinite(distance) & ...
        distance >= targetDistance - tolerance);
end
if completed || ~isfield(result, "Metrics") || ...
        ~isfield(result.Metrics, "EventTiming")
    return
end

% A finish-gate crossing is timed from the vehicle pose. Its projected
% distance can remain one track sample short after truncation at the gate.
timing = result.Metrics.EventTiming;
if ~isfield(timing, "IsValid") || ~isscalar(timing.IsValid) || ...
        ~isequal(timing.IsValid, true) || ...
        ~isfield(timing, "Time") || ~isfield(timing, "Distance")
    return
end
completed = isnumeric(timing.Time) && isscalar(timing.Time) && ...
    isfinite(timing.Time) && timing.Time > 0.0 && ...
    isnumeric(timing.Distance) && isscalar(timing.Distance) && ...
    isfinite(timing.Distance) && ...
    timing.Distance >= targetDistance - tolerance;
end
