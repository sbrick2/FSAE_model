function [axisCatalog, parameterCatalog] = ...
        listRacecarAnalysisCatalog(result, resultType)
%LISTRACECARANALYSISCATALOG 列出通用分析绘图的横轴和参数目录。
%   目录只包含当前结果中实际存在、可与轨迹样本对齐的数值时序。

arguments
    result (1, 1) struct
    resultType (1, 1) string {mustBeMember(resultType, ...
        ["time_domain", "quasi_static"])}
end

switch resultType
    case "time_domain"
        [axisCatalog, parameterCatalog] = timeDomainCatalog(result);
    case "quasi_static"
        [axisCatalog, parameterCatalog] = quasiStaticCatalog(result);
end
end

function [axisCatalog, parameterCatalog] = timeDomainCatalog(result)
available = listLapResultFields(result);
axisPaths = ["Time", "Distance", "Track.EventDistance", ...
    "Track.PathS", "Track.Progress", "Track.LapIndex", ...
    "Vehicle.X", "Vehicle.Y"];
axisNames = ["仿真时间", "实际累计距离", "赛项累计距离", ...
    "单圈赛道弧长", "赛道进度", "圈序号", "全局 X", "全局 Y"];
axisUnits = ["s", "m", "m", "m", "1", "1", "m", "m"];
present = ismember(axisPaths, available);
axisCatalog = makeCatalog(axisNames(present), axisPaths(present), ...
    axisUnits(present));

parameterPaths = available(~ismember(available, ...
    trackOnlyParameterPaths("time_domain")));
displayNames = strings(numel(parameterPaths), 1);
units = strings(numel(parameterPaths), 1);
for index = 1:numel(parameterPaths)
    [~, info] = getLapResultSignal(result, parameterPaths(index));
    displayNames(index) = info.DisplayName;
    units(index) = info.Unit;
end
parameterCatalog = makeCatalog(displayNames, parameterPaths, units);
derived = deriveRacecarAnalysisSignals(result, "time_domain");
derived = derived(~ismember(derived.Path, ...
    trackOnlyDerivedPaths("time_domain")), :);
parameterCatalog = [parameterCatalog; derived(:, 1:4)];
end

function [axisCatalog, parameterCatalog] = quasiStaticCatalog(result)
assert(isfield(result, "Track") && isfield(result.Track, "X") && ...
    isfield(result.Track, "Y"), "FSAE:Analysis:MissingQuasiTrack", ...
    "准静态结果缺少 Track.X 或 Track.Y。");
sampleCount = min(numel(result.Track.X), numel(result.Track.Y));
axisPaths = ["SpeedProfile.Time", "SpeedProfile.ProgressS", ...
    "Derived.NormalizedProgress", "Track.X", "Track.Y"];
axisNames = ["仿真时间", "赛道累计距离", "归一化赛道进度", ...
    "全局 X", "全局 Y"];
axisUnits = ["s", "m", "1", "m", "m"];
present = false(size(axisPaths));
for index = 1:numel(axisPaths)
    if startsWith(axisPaths(index), "Derived.")
        derived = deriveRacecarAnalysisSignals(result, "quasi_static");
        present(index) = any(derived.Path == axisPaths(index));
    else
        value = readNestedField(result, axisPaths(index));
        present(index) = isAnalysisArray(value, sampleCount);
    end
end
axisCatalog = makeCatalog(axisNames(present), axisPaths(present), ...
    axisUnits(present));

candidatePaths = strings(0, 1);
for groupName = ["SpeedProfile", "Track"]
    if ~isfield(result, char(groupName)) || ...
            ~isstruct(result.(char(groupName)))
        continue
    end
    names = fieldnames(result.(char(groupName)));
    for fieldIndex = 1:numel(names)
        path = groupName + "." + string(names{fieldIndex});
        value = readNestedField(result, path);
        if isAnalysisArray(value, sampleCount)
            candidatePaths(end + 1, 1) = path; %#ok<AGROW>
        end
    end
end
candidatePaths = unique(candidatePaths, "stable");
candidatePaths = candidatePaths(~ismember(candidatePaths, ...
    trackOnlyParameterPaths("quasi_static")));
displayNames = strings(numel(candidatePaths), 1);
units = strings(numel(candidatePaths), 1);
for index = 1:numel(candidatePaths)
    info = quasiSignalInfo(candidatePaths(index));
    displayNames(index) = info.DisplayName;
    units(index) = info.Unit;
end
parameterCatalog = makeCatalog(displayNames, candidatePaths, units);
derived = deriveRacecarAnalysisSignals(result, "quasi_static");
derived = derived(~ismember(derived.Path, ...
    trackOnlyDerivedPaths("quasi_static")), :);
parameterCatalog = [parameterCatalog; derived(:, 1:4)];
end

function paths = trackOnlyParameterPaths(resultType)
if resultType == "time_domain"
    paths = ["Time", "Distance", "Track.PathS", ...
        "Track.EventDistance", "Track.LapIndex", "Track.Progress", ...
        "Track.ReferenceSpeed", "Track.Curvature", ...
        "Track.ReportedPathS", "Vehicle.X", "Vehicle.Y", ...
        "Sensor.PositionX", "Sensor.PositionY"];
else
    paths = ["SpeedProfile.Time", "SpeedProfile.ProgressS", ...
        "SpeedProfile.Curvature", "Track.X", "Track.Y", ...
        "Track.Heading", "Track.Curvature", "Track.ReferenceSpeed", ...
        "Track.LeftHalfWidth", "Track.RightHalfWidth"];
end
end

function paths = trackOnlyDerivedPaths(resultType)
paths = ["Derived.DistanceRemaining", "Derived.TurnRadius", ...
    "Derived.TurnDirection"];
if resultType == "quasi_static"
    paths = [paths, "Derived.NormalizedProgress", "Derived.TrackWidth"];
end
end

function catalog = makeCatalog(displayNames, paths, units)
displayNames = string(displayNames(:));
paths = string(paths(:));
units = string(units(:));
labels = displayNames + " | " + paths;
hasUnit = strlength(units) > 0;
labels(hasUnit) = labels(hasUnit) + " (" + units(hasUnit) + ")";
catalog = table(labels, paths, displayNames, units, ...
    VariableNames = ["Label", "Path", "DisplayName", "Unit"]);
end

function tf = isAnalysisArray(value, sampleCount)
tf = ~isempty(value) && (isnumeric(value) || islogical(value)) && ...
    ~isscalar(value) && ismatrix(value) && ...
    abs(size(value, 1) - sampleCount) <= 1;
end

function value = readNestedField(result, path)
parts = split(path, ".");
value = [];
if numel(parts) ~= 2 || ~isfield(result, char(parts(1))) || ...
        ~isstruct(result.(char(parts(1)))) || ...
        ~isfield(result.(char(parts(1))), char(parts(2)))
    return
end
value = result.(char(parts(1))).(char(parts(2)));
end

function info = quasiSignalInfo(path)
info = struct("DisplayName", path, "Unit", "1");
switch path
    case "SpeedProfile.Speed"
        info.DisplayName = "车速"; info.Unit = "m/s";
    case "SpeedProfile.Time"
        info.DisplayName = "仿真时间"; info.Unit = "s";
    case "SpeedProfile.ProgressS"
        info.DisplayName = "赛道累计距离"; info.Unit = "m";
    case "SpeedProfile.LateralAcceleration"
        info.DisplayName = "横向加速度"; info.Unit = "m/s^2";
    case "SpeedProfile.CurvatureSpeedLimit"
        info.DisplayName = "曲率限速"; info.Unit = "m/s";
    case {"SpeedProfile.Curvature", "Track.Curvature"}
        info.DisplayName = "赛道曲率"; info.Unit = "1/m";
    case "Track.ReferenceSpeed"
        info.DisplayName = "参考速度"; info.Unit = "m/s";
    case "Track.Heading"
        info.DisplayName = "赛道航向角"; info.Unit = "rad";
    case "Track.LeftHalfWidth"
        info.DisplayName = "左侧半宽"; info.Unit = "m";
    case "Track.RightHalfWidth"
        info.DisplayName = "右侧半宽"; info.Unit = "m";
    case "Track.X"
        info.DisplayName = "全局 X"; info.Unit = "m";
    case "Track.Y"
        info.DisplayName = "全局 Y"; info.Unit = "m";
end
end
