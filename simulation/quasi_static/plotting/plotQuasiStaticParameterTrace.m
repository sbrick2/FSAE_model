function [figureHandle, plotInfo] = plotQuasiStaticParameterTrace(source, options)
%PLOTQUASISTATICPARAMETERTRACE 绘制两个准静态结果字段之间的折线关系。
%   PLOTQUASISTATICPARAMETERTRACE() 读取最新 Autocross 结果，默认绘制
%   SpeedProfile.Speed 随 SpeedProfile.ProgressS 的变化。SOURCE 可为结果结构体、
%   MAT 文件路径或空字符串；为空时按 Event 读取最新结果。
%
%   可选字段路径 XField/YField 相对于 quasiStaticResult。常用字段包括：
%     SpeedProfile.ProgressS、SpeedProfile.Time、SpeedProfile.Speed、
%     SpeedProfile.LateralAcceleration、SpeedProfile.CurvatureSpeedLimit、
%     Track.Curvature、Track.ReferenceSpeed。
%   两个字段应等长；仅相差一个终点样本时函数会自动对齐。
%
%   其他名称-值参数：Event（acceleration/skidpad/autocross）、XLabel、YLabel、
%   XLimits/YLimits（[NaN NaN] 自动）、LineWidth、Visible（on/off）、
%   SaveFigure 和 OutputFolder。保存目录为空时使用
%   results/quasi_static/lap_time/<event>/plots。
%   返回 FIGUREHANDLE；PLOTINFO 记录来源、字段、范围和保存路径。
%
%   示例：
%     plotQuasiStaticParameterTrace( ...
%         XField="SpeedProfile.ProgressS", ...
%         YField="SpeedProfile.LateralAcceleration", ...
%         XLimits=[100 700], YLimits=[-12 12]);

arguments
    source = ""
    options.Event (1, 1) string = "autocross"
    options.XField (1, 1) string = "SpeedProfile.ProgressS"
    options.YField (1, 1) string = "SpeedProfile.Speed"
    options.XLabel (1, 1) string = ""
    options.YLabel (1, 1) string = ""
    options.XLimits (1, 2) double = [NaN, NaN]
    options.YLimits (1, 2) double = [NaN, NaN]
    options.LineWidth (1, 1) double {mustBePositive} = 1.4
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

validateLimits(options.XLimits, "XLimits");
validateLimits(options.YLimits, "YLimits");
assert(any(options.Visible == ["on", "off"]), ...
    "FSAE:Analysis:FigureVisibility", ...
    "Visible must be on or off.");

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
[result, sourceFile] = loadQuasiStaticResult( ...
    projectRoot, source, options.Event);
x = resolveNumericVector(result, options.XField);
y = resolveNumericVector(result, options.YField);
[x, y] = alignVectors(x, y, options.XField, options.YField);
valid = isfinite(x) & isfinite(y);
assert(nnz(valid) >= 2, "FSAE:Analysis:NoTraceSamples", ...
    "Selected fields do not contain at least two paired finite samples.");
x = x(valid);
y = y(valid);

figureHandle = figure( ...
    "Name", "Quasi-static parameter trace", ...
    "NumberTitle", "off", "Color", "white", ...
    "Visible", options.Visible, "Position", [140, 100, 1080, 620]);
axesHandle = axes(figureHandle);
plot(axesHandle, x, y, "LineWidth", options.LineWidth);
grid(axesHandle, "on");
box(axesHandle, "on");
if strlength(options.XLabel) == 0
    xLabel = options.XField;
else
    xLabel = options.XLabel;
end
if strlength(options.YLabel) == 0
    yLabel = options.YField;
else
    yLabel = options.YLabel;
end
xlabel(axesHandle, xLabel, "Interpreter", "none");
ylabel(axesHandle, yLabel, "Interpreter", "none");
title(axesHandle, options.YField + " versus " + options.XField, ...
    "Interpreter", "none");
applyAxisLimits(axesHandle, options.XLimits, options.YLimits, x, y);

figureFile = saveAnalysisFigure(figureHandle, projectRoot, ...
    options.SaveFigure, options.OutputFolder, ...
    "trace_" + makeSafeName(options.YField), options.Event);
plotInfo = struct( ...
    "SourceFile", sourceFile, ...
    "XField", options.XField, ...
    "YField", options.YField, ...
    "SampleCount", numel(x), ...
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

function [x, y] = alignVectors(x, y, xField, yField)
if numel(x) == numel(y)
    return
end
if abs(numel(x) - numel(y)) == 1
    sampleCount = min(numel(x), numel(y));
    x = x(1:sampleCount);
    y = y(1:sampleCount);
    return
end
error("FSAE:Analysis:TraceFieldSize", ...
    "Fields %s and %s have incompatible lengths (%d and %d).", ...
    xField, yField, numel(x), numel(y));
end

function validateLimits(limits, name)
assert(all(isnan(limits) | isfinite(limits)), ...
    "FSAE:Analysis:NonfiniteLimits", ...
    "%s entries must be finite or NaN.", name);
assert(~all(isfinite(limits)) || limits(1) < limits(2), ...
    "FSAE:Analysis:LimitOrder", ...
    "%s lower bound must be smaller than its upper bound.", name);
end

function limits = resolveDisplayLimits(requested, values)
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
assert(all(isfinite(limits)) && limits(1) < limits(2), ...
    "FSAE:Analysis:ResolvedLimits", ...
    "Resolved display limits must be finite and increasing.");
end

function applyAxisLimits(axesHandle, requestedX, requestedY, x, y)
if any(isfinite(requestedX))
    xlim(axesHandle, resolveDisplayLimits(requestedX, x));
end
if any(isfinite(requestedY))
    ylim(axesHandle, resolveDisplayLimits(requestedY, y));
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
