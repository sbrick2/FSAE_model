function figures = plotLapResultSelection(result, plotCfg)
%PLOTLAPRESULTSELECTION 按配置绘制圈速结果并可选导出文件。
%   FIGURES = PLOTLAPRESULTSELECTION(RESULT, PLOTCFG) 支持时间或距离横轴、
%   标量或 Nx4 四轮纵轴、范围裁剪、显示单位转换、tabs/overlay 布局和
%   MATLAB FIG 导出。Y.Ranges 多余行会忽略，不足行使用自动范围。范围
%   筛选始终使用布尔掩码，因此不假定横轴严格单调。
%   tabs 模式在一个标准图窗中为每个纵轴参数创建标签页；旧的 separate
%   和 subplots 配置也按 tabs 模式处理。
%   标量字段为单条曲线；四轮曲线颜色固定为蓝/橙/绿/红，顺序为
%   [FL, FR, RL, RR]。空字段会警告并跳过，不补零。
%
%   角度、速度和转速仅在显示层转换；结果文件内的单位不被修改。

arguments
    result (1, 1) struct
    plotCfg (1, 1) struct
end

validatePlotConfig(plotCfg);
plotCfg.Layout = normalizePlotLayout(plotCfg.Layout);
[xData, xInfo] = getLapResultSignal(result, plotCfg.X.Parameter, plotCfg);
if isempty(xData)
    error("FSAE:Lap:EmptyXAxis", ...
        "横轴字段 '%s' 为空，无法绘图。", plotCfg.X.Parameter);
end
xData = double(xData(:));
xRange = resolveRange(plotCfg.X.Range, xData, "X");
xMask = isfinite(xData) & xData >= xRange(1) & xData <= xRange(2);
if ~any(xMask)
    actual = finiteRange(xData);
    error("FSAE:Lap:EmptyRange", ...
        "横轴范围 [%g, %g] 内没有数据；当前 '%s' 实际范围为 [%g, %g]。", ...
        xRange(1), xRange(2), plotCfg.X.Parameter, actual(1), actual(2));
end

yParameters = string(plotCfg.Y.Parameters(:));
yRanges = makeYRanges(plotCfg.Y.Ranges, numel(yParameters));
valid = false(numel(yParameters), 1);
yData = cell(numel(yParameters), 1);
yInfo = repmat(emptyInfo(), numel(yParameters), 1);
for index = 1:numel(yParameters)
    try
        [candidate, info] = getLapResultSignal(result, yParameters(index), plotCfg);
    catch exception
        warning("FSAE:Lap:YFieldSkipped", "%s", exception.message);
        continue
    end
    if isempty(candidate)
        warning("FSAE:Lap:YFieldEmpty", ...
            "纵轴字段 '%s' 为空，已跳过且未补零。", yParameters(index));
        continue
    end
    if size(candidate, 1) ~= numel(xData)
        warning("FSAE:Lap:YLengthSkipped", ...
            "纵轴字段 '%s' 第一维为 %d，横轴长度为 %d，已跳过。", ...
            yParameters(index), size(candidate, 1), numel(xData));
        continue
    end
    valid(index) = true;
    yData{index} = candidate;
    yInfo(index) = info;
end
if ~any(valid)
    fields = listLapResultFields(result);
    error("FSAE:Lap:NoPlottableFields", ...
        "所有选定纵轴字段均不可用。可用字段：%s。", strjoin(fields, ", "));
end

yParameters = yParameters(valid);
yRanges = yRanges(valid, :);
yData = yData(valid);
yInfo = yInfo(valid);

if plotCfg.Layout == "overlay"
    units = string({yInfo.Unit});
    if numel(unique(units)) > 1
        error("FSAE:Lap:OverlayUnitMismatch", ...
            "overlay 模式中的纵轴单位不一致：%s。请改用 tabs。", ...
            strjoin(unique(units), ", "));
    end
    figures = gobjects(1, 1);
    figures(1) = figure("Name", "FSAE 圈速结果 - overlay", "Color", "w");
    axesHandle = axes(figures(1));
    hold(axesHandle, "on");
    plotSelectionLines(axesHandle, xData(xMask), yData, yInfo, xMask);
    applyAxesLabels(axesHandle, xInfo, yInfo(1));
    setYLimit(axesHandle, yRanges(1, :), yData, xMask);
    if plotCfg.ShowLegend
        legend(axesHandle, buildLegend(yInfo), "Location", "best");
    end
    title(axesHandle, "圈速结果叠加图");
    finalizeAxes(axesHandle, plotCfg);
else
    tabNames = string({yInfo.DisplayName});
    [figures, axesHandles] = createRacecarFigureTabs( ...
        "FSAE 圈速结果", tabNames, "on");
    for index = 1:numel(yParameters)
        axesHandle = axesHandles(index);
        hold(axesHandle, "on");
        plotSelectionLines(axesHandle, xData(xMask), yData(index), ...
            yInfo(index), xMask);
        applyAxesLabels(axesHandle, xInfo, yInfo(index));
        setYLimit(axesHandle, yRanges(index, :), yData(index), xMask);
        title(axesHandle, yInfo(index).DisplayName + " (" + yInfo(index).Unit + ")");
        if plotCfg.ShowLegend && size(yData{index}, 2) > 1
            legend(axesHandle, buildLegend(yInfo(index)), "Location", "best");
        end
        finalizeAxes(axesHandle, plotCfg);
    end
end

exportedFiles = exportSelectionFigures(figures, plotCfg, result, yParameters);
for index = 1:numel(figures)
    if isgraphics(figures(index))
        setappdata(figures(index), "FSAE_ExportedFiles", exportedFiles);
    end
end
drawnow;
end

function validatePlotConfig(plotCfg)
validLayouts = ["tabs", "separate", "subplots", "overlay"];
if ~isfield(plotCfg, "Layout") || ~any(string(plotCfg.Layout) == validLayouts)
    error("FSAE:Lap:InvalidPlotLayout", ...
        "Layout 收到 '%s'，允许值为 tabs 或 overlay。", plotCfg.Layout);
end
if ~isfield(plotCfg, "X") || ~isfield(plotCfg.X, "Parameter") || ...
        ~isfield(plotCfg.X, "Range")
    error("FSAE:Lap:MissingXAxisConfig", "plotCfg.X 必须包含 Parameter 和 Range。");
end
if numel(plotCfg.X.Range) ~= 2
    error("FSAE:Lap:InvalidXAxisRange", ...
        "X.Range 收到 %s，必须为 [下限 上限]；NaN 表示自动缩放。", ...
        mat2str(plotCfg.X.Range));
end
if ~isfield(plotCfg, "Y") || ~isfield(plotCfg.Y, "Parameters")
    error("FSAE:Lap:MissingYAxisConfig", "plotCfg.Y 必须包含 Parameters。");
end
end

function ranges = makeYRanges(ranges, count)
if isempty(ranges)
    ranges = NaN(count, 2);
elseif size(ranges, 2) ~= 2
    error("FSAE:Lap:InvalidYAxisRanges", ...
        "Y.Ranges 尺寸为 %s，第二维必须为 2。", mat2str(size(ranges)));
elseif size(ranges, 1) > count
    ranges = ranges(1:count, :);
elseif size(ranges, 1) < count
    ranges(end + 1:count, :) = NaN;
end
end

function range = resolveRange(requested, data, axisName)
requested = double(requested(:).');
if numel(requested) ~= 2
    error("FSAE:Lap:InvalidAxisRange", ...
        "%s.Range 收到 %s，必须为 [下限 上限]。", axisName, mat2str(requested));
end
actual = finiteRange(data);
range = requested;
if isnan(range(1))
    range(1) = actual(1);
end
if isnan(range(2))
    range(2) = actual(2);
end
if any(~isfinite(range)) || range(1) > range(2)
    error("FSAE:Lap:InvalidAxisRange", ...
        "%s.Range 收到 %s，当前数据实际范围为 %s。", ...
        axisName, mat2str(requested), mat2str(actual));
end
end

function range = finiteRange(data)
finiteData = double(data(isfinite(data)));
if isempty(finiteData)
    range = [NaN, NaN];
else
    range = [min(finiteData), max(finiteData)];
end
end

function plotSelectionLines(axesHandle, xData, yDataCell, infoArray, xMask)
colors = [0.00, 0.4470, 0.7410; ...
    0.8500, 0.3250, 0.0980; ...
    0.4660, 0.6740, 0.1880; ...
    0.6350, 0.0780, 0.1840];
if iscell(yDataCell)
    for signalIndex = 1:numel(yDataCell)
        yData = yDataCell{signalIndex};
        info = infoArray(signalIndex);
        plotOneSelection(axesHandle, xData, yData, info, xMask, colors);
    end
    return
else
    yData = yDataCell;
    info = infoArray;
end
plotOneSelection(axesHandle, xData, yData, info, xMask, colors);
end

function plotOneSelection(axesHandle, xData, yData, info, xMask, colors)
yData = yData(xMask, :);
for column = 1:size(yData, 2)
    if size(yData, 2) == 1
        color = colors(1, :);
        displayName = info.DisplayName;
    else
        color = colors(column, :);
        displayName = info.ChannelNames(column);
    end
    plot(axesHandle, xData, yData(:, column), ...
        "Color", color, "LineWidth", 1.1, "DisplayName", displayName);
end
end

function applyAxesLabels(axesHandle, xInfo, yInfo)
xlabel(axesHandle, xInfo.DisplayName + " (" + xInfo.Unit + ")");
ylabel(axesHandle, yInfo.DisplayName + " (" + yInfo.Unit + ")");
end

function setYLimit(axesHandle, requested, yData, xMask)
requested = double(requested(:).');
if all(isnan(requested))
    return
end
if iscell(yData)
    values = yData{1};
else
    values = yData;
end
values = values(xMask, :);
actual = finiteRange(values);
limit = requested;
if isnan(limit(1))
    limit(1) = actual(1);
end
if isnan(limit(2))
    limit(2) = actual(2);
end
if any(~isfinite(limit)) || limit(1) > limit(2)
    error("FSAE:Lap:InvalidYAxisRange", ...
        "Y.Range 收到 %s，当前筛选数据实际范围为 %s。", ...
        mat2str(requested), mat2str(actual));
end
if limit(1) == limit(2)
    padding = max(1, abs(limit(1)) * 0.05);
    limit = limit + [-padding, padding];
end
ylim(axesHandle, limit);
end

function labels = buildLegend(infoArray)
labels = strings(0, 1);
for index = 1:numel(infoArray)
    info = infoArray(index);
    if numel(info.ChannelNames) > 1
        labels = [labels; info.ChannelNames(:)]; %#ok<AGROW>
    else
        labels(end + 1, 1) = info.DisplayName; %#ok<AGROW>
    end
end
end

function finalizeAxes(axesHandle, plotCfg)
if plotCfg.ShowGrid
    grid(axesHandle, "on");
else
    grid(axesHandle, "off");
end
if isfield(plotCfg, "X") && ...
        any(string(plotCfg.X.Parameter) == ["Vehicle.X", "Distance", "Track.EventDistance"])
    % 对常用轨迹/距离横轴不改变坐标比例；XY 情况下另行等比例。
    if string(plotCfg.X.Parameter) == "Vehicle.X"
        axis(axesHandle, "equal");
    end
end
end

function info = emptyInfo()
info = struct("Path", "", "DisplayName", "", "Unit", "", ...
    "SourceUnit", "", "IsWheel", false, "ChannelNames", "");
end

function files = exportSelectionFigures(figures, plotCfg, result, yParameters)
files = strings(0, 1);
if ~isfield(plotCfg, "Export") || ~plotCfg.Export.Enabled
    return
end
if strlength(string(plotCfg.Export.Directory)) > 0
    exportDirectory = string(plotCfg.Export.Directory);
elseif isfield(result, "Meta") && isfield(result.Meta, "OutputFolder")
    exportDirectory = string(result.Meta.OutputFolder);
else
    exportDirectory = string(pwd);
end
if ~isfolder(exportDirectory)
    mkdir(exportDirectory);
end
base = "lap_result_" + lower(string(plotCfg.Layout));
if isscalar(yParameters)
    base = base + "_" + string(regexprep(char(yParameters(1)), "[^A-Za-z0-9_-]", "_"));
end
formats = lower(string(plotCfg.Export.Formats));
for figureIndex = 1:numel(figures)
    if ~isgraphics(figures(figureIndex))
        continue
    end
    for format = formats
        if format ~= "fig"
            warning("FSAE:Lap:UnknownExportFormat", ...
                "仅保存 MATLAB FIG 文件；忽略不支持的导出格式 '%s'。", format);
            continue
        end
        filePath = fullfile(exportDirectory, base + "_" + figureIndex + ".fig");
        switch format
            case "fig"
                savefig(figures(figureIndex), filePath);
        end
        files(end + 1, 1) = string(filePath); %#ok<AGROW>
    end
end
end

function layout = normalizePlotLayout(layout)
layout = string(layout);
if any(layout == ["separate", "subplots"])
    layout = "tabs";
end
end
