function [figureHandle, summary] = plotTireUtilization(source, options)
%PLOTTIREUTILIZATION 分析四轮轮胎力、滑移和附着利用率。
%   [FIGURES, SUMMARY] = PLOTTIREUTILIZATION(SOURCE) 绘制四轮纵/横向力、滑移率、
%   侧偏角、法向载荷及 MuUtilization，并给出轮胎力散点关系。
%   SOURCE 可为标准化 result 结构体、MAT 路径或空字符串（读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 tire_utilization 目录。
%   四轮字段顺序为 [FL, FR, RL, RR]；SUMMARY 返回峰值利用率和滑移指标。

arguments
    source = ""
    options.Track (1, 1) string = "autocross"
    options.Axis (1, 1) string = "EventDistance"
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[result, filePath] = loadRacecarResult(source, options.Track);
[x, xLabel] = resolveRacecarAnalysisAxis(result, options.Axis);
wheelNames = ["FL", "FR", "RL", "RR"];
colors = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; ...
    0.4660 0.6740 0.1880; 0.6350 0.0780 0.1840];

plotNames = [ ...
    "Tire utilization - friction-plane operating points"
    "Tire utilization - utilization history"
    "Tire utilization - dynamic normal load"
    "Tire utilization - longitudinal force build-up"
    "Tire utilization - lateral force build-up"
    "Tire utilization - load sensitivity"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Tire utilization", plotNames, options.Visible);

ax = axesHandles(1);
scatterWheelPairs(ax, result.Tire.FxWheel, result.Tire.FyWheel, ...
    wheelNames, colors, "Fx (N)", "Fy (N)");
axis(ax, "equal"); title(ax, "Friction-plane operating points");

ax = axesHandles(2);
plotRacecarChannels(ax, x, result.Tire.MuUtilization, ...
    "Friction utilization (-)", wheelNames, xLabel);
hold(ax, "on"); yline(ax, 1, "k--", "Limit"); hold(ax, "off");
title(ax, "Utilization history");

ax = axesHandles(3);
plotRacecarChannels(ax, x, result.Wheel.NormalLoad, ...
    "Normal load (N)", wheelNames, xLabel);
title(ax, "Dynamic normal load");

ax = axesHandles(4);
scatterWheelPairs(ax, result.Wheel.SlipRatio, result.Tire.FxWheel, ...
    wheelNames, colors, "Slip ratio (-)", "Fx (N)");
title(ax, "Longitudinal force build-up");

ax = axesHandles(5);
scatterWheelPairs(ax, rad2deg(result.Wheel.SlipAngle), ...
    result.Tire.FyWheel, wheelNames, colors, "Slip angle (deg)", "Fy (N)");
title(ax, "Lateral force build-up");

ax = axesHandles(6);
scatterWheelPairs(ax, result.Wheel.NormalLoad, ...
    result.Tire.MuUtilization, wheelNames, colors, ...
    "Normal load (N)", "Utilization (-)");
title(ax, "Load sensitivity in the run");

summary = struct( ...
    "SourceFile", filePath, ...
    "MaximumUtilizationByWheel", perWheelMetric( ...
        result.Tire.MuUtilization, "max"), ...
    "MinimumNormalLoadByWheel", perWheelMetric( ...
        result.Wheel.NormalLoad, "min"), ...
    "MaximumAbsSlipRatioByWheel", perWheelMetric( ...
        result.Wheel.SlipRatio, "maxabs"), ...
    "MaximumAbsSlipAngleDegByWheel", perWheelMetric( ...
        rad2deg(result.Wheel.SlipAngle), "maxabs"));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "tire_utilization", options.Track);
end

function scatterWheelPairs(ax, xData, yData, names, colors, xLabel, yLabel)
if isempty(xData) || isempty(yData) || ...
        size(xData, 1) ~= size(yData, 1)
    plotRacecarChannels(ax, (1:2)', [], yLabel, "", xLabel);
    return
end
count = min([4, size(xData, 2), size(yData, 2)]);
hold(ax, "on");
handles = gobjects(count, 1);
for wheel = 1:count
    handles(wheel, 1) = scatter(ax, xData(:, wheel), yData(:, wheel), ...
        8, colors(wheel, :), "filled", "MarkerFaceAlpha", 0.28);
end
hold(ax, "off"); grid(ax, "on");
xlabel(ax, xLabel); ylabel(ax, yLabel);
if count > 1
    legend(ax, handles, names(1:count), "Location", "best");
end
end

function values = perWheelMetric(data, operation)
values = NaN(1, 4);
if isempty(data)
    return
end
for wheel = 1:min(4, size(data, 2))
    values(wheel) = racecarFiniteMetric(data(:, wheel), operation);
end
end
