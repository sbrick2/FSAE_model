function [figureHandle, plotInfo] = plotQuasiStaticTrackParameterMap(source, options)
%PLOTQUASISTATICTRACKPARAMETERMAP 按指定参数给准静态赛道轨迹着色。
%   PLOTQUASISTATICTRACKPARAMETERMAP() 读取最新 Autocross 结果，并按
%   SpeedProfile.Speed 着色。SOURCE 可为结果结构体、MAT 文件路径或空字符串；
%   为空时按 Event 读取最新结果。
%
%   Field 为相对于 quasiStaticResult 的字段路径，常用值有：
%     SpeedProfile.Speed、SpeedProfile.LateralAcceleration、
%     SpeedProfile.CurvatureSpeedLimit、SpeedProfile.Time、
%     Track.Curvature、Track.ReferenceSpeed。
%   其他名称-值参数：Event（acceleration/skidpad/autocross）、ColorLabel、
%   DistanceLimits、ColorLimits、XLimits、YLimits（范围均可用 [NaN NaN] 自动）、
%   PointSize、Visible（on/off）、SaveFigure 和 OutputFolder。
%   返回 FIGUREHANDLE；PLOTINFO 记录来源、字段、有效距离范围和保存路径。
%
%   示例：
%     plotQuasiStaticTrackParameterMap( ...
%         Field="SpeedProfile.LateralAcceleration", ...
%         DistanceLimits=[100 700], ColorLimits=[-12 12]);

arguments
    source = ""
    options.Event (1, 1) string = "autocross"
    options.Field (1, 1) string = "SpeedProfile.Speed"
    options.ColorLabel (1, 1) string = ""
    options.DistanceLimits (1, 2) double = [NaN, NaN]
    options.ColorLimits (1, 2) double = [NaN, NaN]
    options.XLimits (1, 2) double = [NaN, NaN]
    options.YLimits (1, 2) double = [NaN, NaN]
    options.PointSize (1, 1) double {mustBePositive} = 14
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

validateLimits(options.DistanceLimits, "DistanceLimits");
validateLimits(options.ColorLimits, "ColorLimits");
validateLimits(options.XLimits, "XLimits");
validateLimits(options.YLimits, "YLimits");
assert(any(options.Visible == ["on", "off"]), ...
    "FSAE:Analysis:FigureVisibility", ...
    "Visible must be on or off.");

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
[result, sourceFile] = loadQuasiStaticResult( ...
    projectRoot, source, options.Event);
assert(isfield(result, "Track") && isfield(result.Track, "X") && ...
    isfield(result.Track, "Y"), "FSAE:Analysis:MissingTrack", ...
    "The quasi-static result must contain Track.X and Track.Y.");

x = double(result.Track.X(:));
y = double(result.Track.Y(:));
sampleCount = numel(x);
assert(numel(y) == sampleCount && sampleCount >= 2, ...
    "FSAE:Analysis:TrackSize", ...
    "Track.X and Track.Y must be equally sized vectors.");
parameter = resolveNumericVector(result, options.Field);
parameter = alignTrackVector(parameter, sampleCount, options.Field);
distance = resolveTrackDistance(result, sampleCount);

valid = isfinite(x) & isfinite(y) & isfinite(parameter) & isfinite(distance);
if isfinite(options.DistanceLimits(1))
    valid = valid & distance >= options.DistanceLimits(1);
end
if isfinite(options.DistanceLimits(2))
    valid = valid & distance <= options.DistanceLimits(2);
end
assert(nnz(valid) >= 2, "FSAE:Analysis:NoTrackSamples", ...
    "No finite track samples remain inside DistanceLimits.");

figureHandle = figure( ...
    "Name", "Quasi-static track parameter map", ...
    "NumberTitle", "off", "Color", "white", ...
    "Visible", options.Visible, "Position", [120, 80, 980, 760]);
axesHandle = axes(figureHandle);
scatter(axesHandle, x(valid), y(valid), options.PointSize, ...
    parameter(valid), "filled");
axis(axesHandle, "equal");
grid(axesHandle, "on");
box(axesHandle, "on");
xlabel(axesHandle, "X (m)");
ylabel(axesHandle, "Y (m)");
title(axesHandle, options.Field + " colored track map", ...
    "Interpreter", "none");
colorbarHandle = colorbar(axesHandle);
if strlength(options.ColorLabel) == 0
    colorbarHandle.Label.String = char(options.Field);
else
    colorbarHandle.Label.String = char(options.ColorLabel);
end
colorbarHandle.Label.Interpreter = "none";

colorLimits = resolveDisplayLimits( ...
    options.ColorLimits, parameter(valid), true);
clim(axesHandle, colorLimits);
applyAxisLimits(axesHandle, options.XLimits, options.YLimits, ...
    x(valid), y(valid));

figureFile = saveAnalysisFigure(figureHandle, projectRoot, ...
    options.SaveFigure, options.OutputFolder, ...
    "track_map_" + makeSafeName(options.Field), options.Event);
plotInfo = struct( ...
    "SourceFile", sourceFile, ...
    "Field", options.Field, ...
    "SampleCount", nnz(valid), ...
    "DistanceLimits", options.DistanceLimits, ...
    "ColorLimits", colorLimits, ...
    "XLimits", xlim(axesHandle), ...
    "YLimits", ylim(axesHandle), ...
    "FigureFile", figureFile);
end

function [result, sourceFile] = loadQuasiStaticResult( ...
        projectRoot, source, eventName)
sourceFile = "";
if isstruct(source)
    result = source;
    if isfield(result, "quasiStaticResult")
        result = result.quasiStaticResult;
    end
else
    sourceFile = resolveResultFile(projectRoot, string(source), eventName);
    saved = load(char(sourceFile));
    assert(isfield(saved, "quasiStaticResult"), ...
        "FSAE:Analysis:MissingQuasiStaticResult", ...
        "MAT file does not contain quasiStaticResult: %s", sourceFile);
    result = saved.quasiStaticResult;
end
assert(isstruct(result) && isscalar(result), ...
    "FSAE:Analysis:QuasiStaticResultType", ...
    "Source must resolve to one quasiStaticResult structure.");
end

function resultFile = resolveResultFile(projectRoot, requestedFile, eventName)
if strlength(requestedFile) > 0
    if isfile(requestedFile)
        resultFile = requestedFile;
    else
        resultFile = fullfile(projectRoot, requestedFile);
    end
    assert(isfile(resultFile), "FSAE:Analysis:ResultFileNotFound", ...
        "Quasi-static result file was not found: %s", requestedFile);
    return
end

eventKey = lower(strtrim(eventName));
assert(any(eventKey == ["acceleration", "skidpad", "autocross"]), ...
    "FSAE:Analysis:UnknownEvent", ...
    "Event must be acceleration, skidpad or autocross.");
folder = fullfile(projectRoot, "results", "quasi_static", ...
    "lap_time", eventKey);
files = dir(fullfile(folder, "quasi_static_" + eventKey + "_*.mat"));
assert(~isempty(files), "FSAE:Analysis:NoQuasiStaticResult", ...
    "No saved %s quasi-static result was found under %s.", eventKey, folder);
[~, order] = sort([files.datenum], "descend");
for index = order
    candidate = string(fullfile(files(index).folder, files(index).name));
    variables = whos("-file", char(candidate));
    if any(string({variables.name}) == "quasiStaticResult")
        resultFile = candidate;
        return
    end
end
error("FSAE:Analysis:NoValidQuasiStaticResult", ...
    "No MAT file under %s contains quasiStaticResult.", folder);
end

function value = resolveNumericVector(result, fieldPath)
fieldPath = strip(fieldPath);
prefix = "quasiStaticResult.";
if startsWith(fieldPath, prefix)
    fieldPath = extractAfter(fieldPath, strlength(prefix));
end
assert(strlength(fieldPath) > 0, "FSAE:Analysis:EmptyFieldPath", ...
    "Field must not be empty.");
parts = split(fieldPath, ".");
cursor = result;
for index = 1:numel(parts)
    name = char(parts(index));
    assert(isstruct(cursor) && isscalar(cursor) && isfield(cursor, name), ...
        "FSAE:Analysis:UnknownField", ...
        "Field path does not exist: %s", fieldPath);
    cursor = cursor.(name);
end
assert((isnumeric(cursor) || islogical(cursor)) && isvector(cursor) && ...
    ~isempty(cursor), "FSAE:Analysis:FieldNotNumericVector", ...
    "Field must resolve to a nonempty numeric vector: %s", fieldPath);
value = double(cursor(:));
end

function value = alignTrackVector(value, sampleCount, fieldPath)
if numel(value) == sampleCount + 1
    value = value(1:sampleCount);
end
assert(numel(value) == sampleCount, "FSAE:Analysis:FieldSize", ...
    "Field %s has %d samples; the track has %d samples.", ...
    fieldPath, numel(value), sampleCount);
end

function distance = resolveTrackDistance(result, sampleCount)
if isfield(result, "SpeedProfile") && ...
        isfield(result.SpeedProfile, "ProgressS") && ...
        numel(result.SpeedProfile.ProgressS) >= sampleCount
    distance = double(result.SpeedProfile.ProgressS(1:sampleCount));
    distance = distance(:);
elseif isfield(result.Track, "SampleDistance")
    distance = double(result.Track.SampleDistance) * (0:sampleCount - 1)';
else
    distance = (0:sampleCount - 1)';
end
end

function validateLimits(limits, name)
assert(all(isnan(limits) | isfinite(limits)), ...
    "FSAE:Analysis:NonfiniteLimits", ...
    "%s entries must be finite or NaN.", name);
assert(~all(isfinite(limits)) || limits(1) < limits(2), ...
    "FSAE:Analysis:LimitOrder", ...
    "%s lower bound must be smaller than its upper bound.", name);
end

function limits = resolveDisplayLimits(requested, values, requirePair)
finiteValues = values(isfinite(values));
assert(~isempty(finiteValues), "FSAE:Analysis:NoFiniteValues", ...
    "Selected field contains no finite values.");
limits = requested;
if isnan(limits(1)), limits(1) = min(finiteValues); end
if isnan(limits(2)), limits(2) = max(finiteValues); end
if limits(1) == limits(2)
    padding = max(1.0, abs(limits(1))) * 0.01;
    limits = limits + [-padding, padding];
end
if requirePair
    assert(all(isfinite(limits)) && limits(1) < limits(2), ...
        "FSAE:Analysis:ResolvedLimits", ...
        "Resolved display limits must be finite and increasing.");
end
end

function applyAxisLimits(axesHandle, requestedX, requestedY, x, y)
if any(isfinite(requestedX))
    xlim(axesHandle, resolveDisplayLimits(requestedX, x, true));
end
if any(isfinite(requestedY))
    ylim(axesHandle, resolveDisplayLimits(requestedY, y, true));
end
end

function figureFile = saveAnalysisFigure( ...
        figureHandle, projectRoot, saveRequested, outputFolder, stem, eventName)
figureFile = "";
if ~saveRequested
    return
end
if strlength(outputFolder) == 0
    outputFolder = fullfile(projectRoot, "results", ...
        "quasi_static", "lap_time", lower(eventName), "plots");
elseif ~isAbsolutePath(outputFolder)
    outputFolder = fullfile(projectRoot, outputFolder);
end
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end
timestamp = string(datetime("now", TimeZone="UTC", ...
    Format="yyyyMMdd_HHmmss_SSS"));
figureFile = string(fullfile(outputFolder, stem + "_" + timestamp + ".fig"));
savefig(figureHandle, char(figureFile));
end

function name = makeSafeName(value)
name = string(regexprep(lower(char(value)), "[^a-zA-Z0-9]+", "_"));
name = strip(name, "_");
end

function tf = isAbsolutePath(path)
tf = ~isempty(regexp(char(path), '^[A-Za-z]:[\\/]|^\\\\', 'once'));
end
