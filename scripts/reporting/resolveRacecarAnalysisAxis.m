function [x, label, path] = resolveRacecarAnalysisAxis(result, axisName)
%RESOLVERACECARANALYSISAXIS 解析标准时间轴或距离轴。

arguments
    result (1, 1) struct
    axisName (1, 1) string = "Distance"
end

key = lower(strrep(axisName, ".", ""));
switch key
    case "time"
        x = result.Time;
        label = "Time (s)";
        path = "Time";
    case "distance"
        x = result.Distance;
        label = "Distance (m)";
        path = "Distance";
    case {"eventdistance", "trackeventdistance"}
        [x, path] = firstAvailable(result, ...
            ["Track.EventDistance", "Distance"]);
        label = "Event distance (m)";
    case {"paths", "trackpaths"}
        [x, path] = firstAvailable(result, ["Track.PathS", "Distance"]);
        label = "Track path s (m)";
    otherwise
        error("FSAE:Analysis:InvalidAxis", ...
            "Axis must be Time, Distance, EventDistance or PathS.");
end
x = double(x(:));
assert(~isempty(x) && all(isfinite(x)), "FSAE:Analysis:AxisUnavailable", ...
    "The requested analysis axis %s is unavailable or nonfinite.", axisName);
end

function [value, selectedPath] = firstAvailable(result, paths)
value = [];
selectedPath = "";
for path = paths
    parts = split(path, ".");
    if isscalar(parts) && isfield(result, char(parts(1)))
        candidate = result.(char(parts(1)));
    elseif numel(parts) == 2 && isfield(result, char(parts(1))) && ...
            isfield(result.(char(parts(1))), char(parts(2)))
        candidate = result.(char(parts(1))).(char(parts(2)));
    else
        candidate = [];
    end
    if ~isempty(candidate)
        value = candidate;
        selectedPath = path;
        return
    end
end
end
