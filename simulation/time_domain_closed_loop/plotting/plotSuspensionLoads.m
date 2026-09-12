function [figureHandle, summary] = plotSuspensionLoads(source, options)
%PLOTSUSPENSIONLOADS 分析动态轮荷、车轮姿态和车身姿态。
%   [FIGURES, SUMMARY] = PLOTSUSPENSIONLOADS(SOURCE) 绘制四轮垂向载荷/轮荷转移、
%   外倾角、侧倾角、俯仰角及垂向加速度；当结果含
%   Wheel.SuspensionDeflection 时同时绘制悬架行程。
%   SOURCE 可为标准化 result 结构体、MAT 路径或空字符串（读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 suspension_loads 目录。
%   四轮字段顺序为 [FL, FR, RL, RR]；SUMMARY 返回最大轮荷、轮荷转移和姿态指标。

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
normalLoad = result.Wheel.NormalLoad;

plotNames = [ ...
    "Suspension loads - four-wheel normal load"
    "Suspension loads - front/rear distribution"
    "Suspension loads - left/right distribution"
    "Suspension loads - sprung-body attitude"
    "Suspension loads - vertical response"
    "Suspension loads - wheel travel or camber"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Suspension and wheel loads", plotNames, options.Visible);

ax = axesHandles(1);
plotRacecarChannels(ax, x, normalLoad, "Normal load (N)", ...
    wheelNames, xLabel);
title(ax, "Four-wheel normal load");

ax = axesHandles(2);
plotRacecarChannels(ax, x, axleLoads(normalLoad), "Axle load (N)", ...
    ["Front", "Rear"], xLabel);
title(ax, "Front/rear load distribution");

ax = axesHandles(3);
plotRacecarChannels(ax, x, sideLoads(normalLoad), "Side load (N)", ...
    ["Left", "Right"], xLabel);
title(ax, "Left/right load distribution");

ax = axesHandles(4);
plotRacecarChannels(ax, x, rad2deg([result.Vehicle.RollAngle, ...
    result.Vehicle.PitchAngle]), "Body angle (deg)", ...
    ["Roll", "Pitch"], xLabel);
title(ax, "Sprung-body attitude");

ax = axesHandles(5);
plotRacecarChannels(ax, x, result.Vehicle.Az, ...
    "Vertical acceleration (m/s^2)", "Az", xLabel);
title(ax, "Vertical response");

ax = axesHandles(6);
deflection = optionalField(result.Wheel, "SuspensionDeflection");
if ~isempty(deflection)
    plotRacecarChannels(ax, x, 1000 * deflection, ...
        "Suspension deflection (mm)", wheelNames, xLabel);
    title(ax, "Suspension travel");
else
    plotRacecarChannels(ax, x, rad2deg(result.Wheel.CamberAngle), ...
        "Camber angle (deg)", wheelNames, xLabel);
    title(ax, "Wheel camber");
end

summary = struct( ...
    "SourceFile", filePath, ...
    "MinimumWheelLoadByWheel", perWheelMetric(normalLoad, "min"), ...
    "MaximumWheelLoadByWheel", perWheelMetric(normalLoad, "max"), ...
    "MaximumAbsRollDeg", racecarFiniteMetric( ...
        rad2deg(result.Vehicle.RollAngle), "maxabs"), ...
    "MaximumAbsPitchDeg", racecarFiniteMetric( ...
        rad2deg(result.Vehicle.PitchAngle), "maxabs"), ...
    "MaximumAbsVerticalAcceleration", ...
        racecarFiniteMetric(result.Vehicle.Az, "maxabs"));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "suspension_loads", options.Track);
end

function loads = axleLoads(normalLoad)
if isempty(normalLoad) || size(normalLoad, 2) < 4
    loads = [];
else
    loads = [sum(normalLoad(:, 1:2), 2), sum(normalLoad(:, 3:4), 2)];
end
end

function loads = sideLoads(normalLoad)
if isempty(normalLoad) || size(normalLoad, 2) < 4
    loads = [];
else
    loads = [sum(normalLoad(:, [1, 3]), 2), ...
        sum(normalLoad(:, [2, 4]), 2)];
end
end

function value = optionalField(group, name)
if isfield(group, name)
    value = group.(name);
else
    value = [];
end
end

function values = perWheelMetric(data, operation)
values = NaN(1, 4);
if isempty(data), return; end
for wheel = 1:min(4, size(data, 2))
    values(wheel) = racecarFiniteMetric(data(:, wheel), operation);
end
end
