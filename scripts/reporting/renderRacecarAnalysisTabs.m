function [axesHandles, plotInfo] = renderRacecarAnalysisTabs( ...
        tabGroup, result, resultType, plotFormat, xParameter, parameter)
%RENDERRACECARANALYSISTABS 在标签页中绘制折线图或参数着色赛道图。
%   四轮赛道参数按 [FL, FR, RL, RR] 拆成四个标签页；折线图在一个
%   标签页中叠加所有通道。函数不会保存文件或修改结果结构。

arguments
    tabGroup (1, 1)
    result (1, 1) struct
    resultType (1, 1) string {mustBeMember(resultType, ...
        ["time_domain", "quasi_static"])}
    plotFormat (1, 1) string {mustBeMember(plotFormat, ["line", "track"])}
    xParameter (1, 1) string
    parameter (1, 1) string
end

delete(tabGroup.Children);
[data, info] = analysisSignal(result, resultType, parameter);
if plotFormat == "line"
    [x, xInfo] = analysisSignal(result, resultType, xParameter);
    [x, data] = alignRows(x, data, xParameter, parameter);
    valid = isfinite(x) & any(isfinite(data), 2);
    assert(nnz(valid) >= 2, "FSAE:Analysis:NoLineSamples", ...
        "所选横轴和分析参数没有至少两个配对的有限样本。");
    tab = uitab(tabGroup, "Title", char(info.DisplayName));
    axesHandles = createFillingAxes(tab);
    plotLines(axesHandles, x(valid), data(valid, :), info);
    xlabel(axesHandles, axisLabel(xInfo));
    ylabel(axesHandles, axisLabel(info));
    title(axesHandles, info.DisplayName + " - " + xInfo.DisplayName);
    sampleCounts = nnz(valid);
    channelNames = info.ChannelNames;
else
    [trackX, trackY] = trackCoordinates(result, resultType);
    [trackX, data] = alignRows(trackX, data, "Track.X", parameter);
    trackY = alignSingleVector(trackY, numel(trackX), "Track.Y");
    channelNames = resolveChannelNames(info, size(data, 2));
    axesHandles = gobjects(size(data, 2), 1);
    sampleCounts = zeros(size(data, 2), 1);
    for channel = 1:size(data, 2)
        tab = uitab(tabGroup, "Title", char(channelNames(channel)));
        axesHandles(channel) = createFillingAxes(tab);
        values = data(:, channel);
        valid = isfinite(trackX) & isfinite(trackY) & isfinite(values);
        assert(nnz(valid) >= 2, "FSAE:Analysis:NoTrackSamples", ...
            "参数 %s 的 %s 通道没有足够的有限赛道样本。", ...
            parameter, channelNames(channel));
        scatter(axesHandles(channel), trackX(valid), trackY(valid), ...
            16, values(valid), "filled");
        axis(axesHandles(channel), "equal");
        xlabel(axesHandles(channel), "Global X (m)");
        ylabel(axesHandles(channel), "Global Y (m)");
        title(axesHandles(channel), info.DisplayName + " - " + ...
            channelNames(channel));
        colorbarHandle = colorbar(axesHandles(channel));
        colorbarHandle.Label.String = char(axisLabel(info));
        sampleCounts(channel) = nnz(valid);
    end
end

for axesHandle = reshape(axesHandles, 1, [])
    grid(axesHandle, "on");
    box(axesHandle, "on");
end
plotInfo = struct( ...
    "Format", plotFormat, ...
    "XParameter", xParameter, ...
    "Parameter", parameter, ...
    "DisplayName", info.DisplayName, ...
    "Unit", info.Unit, ...
    "ChannelNames", channelNames, ...
    "AxesCount", numel(axesHandles), ...
    "SampleCounts", sampleCounts, ...
    "Minimum", minimumFinite(data), ...
    "Maximum", maximumFinite(data));
drawnow;
end

function axesHandle = createFillingAxes(tab)
gridLayout = uigridlayout(tab, [1, 1]);
gridLayout.RowHeight = {'1x'};
gridLayout.ColumnWidth = {'1x'};
gridLayout.Padding = [8, 8, 8, 8];
axesHandle = uiaxes(gridLayout);
axesHandle.Layout.Row = 1;
axesHandle.Layout.Column = 1;
end

function [data, info] = analysisSignal(result, resultType, parameter)
derived = deriveRacecarAnalysisSignals(result, resultType);
derivedRow = derived(derived.Path == parameter, :);
if height(derivedRow) == 1
    data = derivedRow.Data{1};
    info = struct("Path", parameter, ...
        "DisplayName", derivedRow.DisplayName(1), ...
        "Unit", derivedRow.Unit(1), ...
        "SourceUnit", derivedRow.Unit(1), ...
        "IsWheel", size(data, 2) == 4, ...
        "ChannelNames", derivedRow.ChannelNames{1});
elseif resultType == "time_domain"
    [data, info] = getLapResultSignal(result, parameter);
else
    value = readQuasiField(result, parameter);
    data = double(value);
    if isvector(data)
        data = data(:);
    end
    [axisCatalog, catalog] = ...
        listRacecarAnalysisCatalog(result, "quasi_static");
    row = catalog(catalog.Path == parameter, :);
    if isempty(row)
        row = axisCatalog(axisCatalog.Path == parameter, :);
    end
    assert(height(row) == 1, "FSAE:Analysis:UnknownQuasiField", ...
        "准静态分析字段不存在：%s。", parameter);
    info = struct("Path", parameter, ...
        "DisplayName", row.DisplayName(1), "Unit", row.Unit(1), ...
        "SourceUnit", row.Unit(1), "IsWheel", size(data, 2) == 4, ...
        "ChannelNames", resolveDirectChannelNames(data));
end
data = double(data);
if isvector(data)
    data = data(:);
end
end

function names = resolveDirectChannelNames(data)
if size(data, 2) == 4
    names = ["FL"; "FR"; "RL"; "RR"];
else
    names = "标量";
end
end

function value = readQuasiField(result, path)
parts = split(path, ".");
assert(numel(parts) == 2 && isfield(result, char(parts(1))) && ...
    isstruct(result.(char(parts(1)))) && ...
    isfield(result.(char(parts(1))), char(parts(2))), ...
    "FSAE:Analysis:UnknownQuasiField", ...
    "准静态分析字段不存在：%s。", path);
value = result.(char(parts(1))).(char(parts(2)));
assert((isnumeric(value) || islogical(value)) && ~isempty(value), ...
    "FSAE:Analysis:InvalidQuasiField", ...
    "准静态分析字段不是非空数值数组：%s。", path);
end

function [x, y] = alignRows(x, y, xName, yName)
x = double(x(:));
if size(x, 1) == size(y, 1)
    return
end
if abs(size(x, 1) - size(y, 1)) <= 1
    count = min(size(x, 1), size(y, 1));
    x = x(1:count);
    y = y(1:count, :);
    return
end
error("FSAE:Analysis:FieldLength", ...
    "字段 %s 与 %s 的样本数不兼容（%d 与 %d）。", ...
    xName, yName, size(x, 1), size(y, 1));
end

function value = alignSingleVector(value, count, fieldName)
value = double(value(:));
if numel(value) == count + 1
    value = value(1:count);
end
assert(numel(value) == count, "FSAE:Analysis:TrackLength", ...
    "赛道字段 %s 的样本数与分析数据不兼容。", fieldName);
end

function [x, y] = trackCoordinates(result, resultType)
if resultType == "time_domain"
    assert(isfield(result, "Vehicle") && isfield(result.Vehicle, "X") && ...
        isfield(result.Vehicle, "Y"), "FSAE:Analysis:MissingVehicleTrack", ...
        "时域结果缺少 Vehicle.X 或 Vehicle.Y。");
    x = double(result.Vehicle.X(:));
    y = double(result.Vehicle.Y(:));
else
    assert(isfield(result, "Track") && isfield(result.Track, "X") && ...
        isfield(result.Track, "Y"), "FSAE:Analysis:MissingQuasiTrack", ...
        "准静态结果缺少 Track.X 或 Track.Y。");
    x = double(result.Track.X(:));
    y = double(result.Track.Y(:));
end
end

function plotLines(axesHandle, x, data, info)
hold(axesHandle, "on");
colors = [0.00, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; ...
    0.4660, 0.6740, 0.1880; 0.6350, 0.0780, 0.1840];
channelNames = resolveChannelNames(info, size(data, 2));
for channel = 1:size(data, 2)
    plot(axesHandle, x, data(:, channel), "LineWidth", 1.2, ...
        "Color", colors(1 + mod(channel - 1, size(colors, 1)), :), ...
        "DisplayName", channelNames(channel));
end
hold(axesHandle, "off");
if size(data, 2) > 1
    legend(axesHandle, "Location", "best");
end
end

function names = resolveChannelNames(info, count)
names = string(info.ChannelNames(:));
if numel(names) ~= count || (count > 1 && isscalar(names))
    if count == 4
        names = ["FL"; "FR"; "RL"; "RR"];
    else
        names = "通道 " + (1:count)';
    end
end
if count == 1
    names = info.DisplayName;
end
end

function label = axisLabel(info)
label = string(info.DisplayName);
if strlength(string(info.Unit)) > 0
    label = label + " (" + string(info.Unit) + ")";
end
end

function value = minimumFinite(data)
finiteData = double(data(isfinite(data)));
if isempty(finiteData), value = NaN; else, value = min(finiteData); end
end

function value = maximumFinite(data)
finiteData = double(data(isfinite(data)));
if isempty(finiteData), value = NaN; else, value = max(finiteData); end
end
