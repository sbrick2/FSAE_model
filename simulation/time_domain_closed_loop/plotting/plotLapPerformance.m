function [figureHandle, sectorTable, summary] = plotLapPerformance(source, options)
%PLOTLAPPERFORMANCE 分析速度规划、曲率和分段用时。
%   [FIGURES, SECTORTABLE, SUMMARY] = PLOTLAPPERFORMANCE(SOURCE) 绘制曲率、
%   实际/参考车速、纵横向加速度、时间损失和等距赛段表现。SOURCE 可为标准化
%   result 结构体、MAT 路径或空字符串（自动读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；
%   SectorCount 为正整数分段数；Visible 可选 on/off；SaveFigure 控制 FIG 保存；
%   OutputFolder 为空时使用默认 lap_performance 目录。
%   SECTORTABLE 为分段距离/用时/均速表，SUMMARY 返回整段指标和保存信息。

arguments
    source = ""
    options.Track (1, 1) string = "autocross"
    options.SectorCount (1, 1) double {mustBeInteger, mustBePositive} = 10
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[result, filePath] = loadRacecarResult(source, options.Track);
[distance, distanceLabel] = resolveRacecarAnalysisAxis(result, "EventDistance");
sectorTable = calculateSectors(distance, result.Time, ...
    result.Vehicle.Speed, options.SectorCount);
referenceSpeedUsed = lapResultUsesReferenceSpeed(result);

plotNames = [ ...
    "Lap performance - track curvature"
    "Lap performance - speed profile"
    "Lap performance - acceleration demand"
    "Lap performance - cumulative event time"
    "Lap performance - path accuracy"
    "Lap performance - sector times"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Lap performance", plotNames, options.Visible);

ax = axesHandles(1);
plotRacecarChannels(ax, distance, result.Track.Curvature, ...
    "Curvature (1/m)", "Curvature", distanceLabel);
title(ax, "Track curvature");

ax = axesHandles(2);
if referenceSpeedUsed
    speedChannels = [result.Vehicle.Speed, result.Track.ReferenceSpeed];
    speedNames = ["Actual", "Reference"];
else
    speedChannels = result.Vehicle.Speed;
    speedNames = "Actual";
end
plotRacecarChannels(ax, distance, speedChannels, "Speed (m/s)", ...
    speedNames, distanceLabel);
title(ax, "Speed profile");

ax = axesHandles(3);
plotRacecarChannels(ax, distance, [result.Vehicle.Ax, result.Vehicle.Ay], ...
    "Acceleration (m/s^2)", ["Ax", "Ay"], distanceLabel);
title(ax, "Acceleration demand");

ax = axesHandles(4);
plotRacecarChannels(ax, distance, result.Time, ...
    "Cumulative time (s)", "Time", distanceLabel);
title(ax, "Cumulative event time");

ax = axesHandles(5);
if isfield(result.Track, "VehicleEnvelopeClearance")
    plotRacecarChannels(ax, distance, [result.Track.LateralError, ...
        result.Track.VehicleEnvelopeClearance], "Distance (m)", ...
        ["Lateral error", "Envelope clearance"], distanceLabel);
    title(ax, "Path error and vehicle-envelope clearance");
else
    plotRacecarChannels(ax, distance, result.Track.LateralError, ...
        "Lateral error (m)", "Error", distanceLabel);
    title(ax, "Path error by distance");
end

ax = axesHandles(6);
if ~isempty(sectorTable)
    bar(ax, sectorTable.Sector, sectorTable.Time, ...
        "FaceColor", [0.0000, 0.4470, 0.7410]);
    grid(ax, "on"); xlabel(ax, "Sector"); ylabel(ax, "Sector time (s)");
else
    plotRacecarChannels(ax, (1:2)', [], "Sector time (s)", "", "Sector");
end
title(ax, "Equal-distance sector times");

summary = struct( ...
    "SourceFile", filePath, ...
    "ReferenceSpeedUsed", referenceSpeedUsed, ...
    "EventTime", eventTime(result), ...
    "MaximumSpeed", racecarFiniteMetric(result.Vehicle.Speed, "max"), ...
    "MinimumSpeed", racecarFiniteMetric(result.Vehicle.Speed, "min"), ...
    "MaximumAbsAx", racecarFiniteMetric(result.Vehicle.Ax, "maxabs"), ...
    "MaximumAbsAy", racecarFiniteMetric(result.Vehicle.Ay, "maxabs"), ...
    "LateralErrorRMSE", ...
        racecarFiniteMetric(result.Track.LateralError, "rms"), ...
    "VehicleEnvelopeMinimumClearance", metricValue(result, ...
        "MinimumVehicleEnvelopeClearance"), ...
    "VehicleEnvelopeMaximumViolation", metricValue(result, ...
        "MaximumVehicleEnvelopeViolation"), ...
    "SectorCount", height(sectorTable));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "lap_performance", options.Track);
end

function value = metricValue(result, fieldName)
value = NaN;
if isfield(result, "Metrics") && isfield(result.Metrics, fieldName)
    value = double(result.Metrics.(fieldName));
end
end

function sectors = calculateSectors(distance, time, speed, count)
sectors = table;
valid = isfinite(distance) & isfinite(time);
if nnz(valid) < 2, return; end
d = double(distance(valid)); t = double(time(valid));
[d, order] = sort(d); t = t(order);
[d, uniqueIndex] = unique(d, "stable"); t = t(uniqueIndex);
if numel(d) < 2 || d(end) <= d(1), return; end
edges = linspace(d(1), d(end), count + 1)';
edgeTime = interp1(d, t, edges, "linear");
sectorTime = diff(edgeTime);
minimumSpeed = NaN(count, 1); maximumSpeed = NaN(count, 1);
if ~isempty(speed) && numel(speed) == numel(time)
    v = double(speed(valid)); v = v(order); v = v(uniqueIndex);
    for index = 1:count
        mask = d >= edges(index) & d <= edges(index + 1);
        minimumSpeed(index) = racecarFiniteMetric(v(mask), "min");
        maximumSpeed(index) = racecarFiniteMetric(v(mask), "max");
    end
end
sectors = table((1:count)', edges(1:end - 1), edges(2:end), ...
    sectorTime, minimumSpeed, maximumSpeed, ...
    VariableNames = ["Sector", "StartDistance", "EndDistance", ...
    "Time", "MinimumSpeed", "MaximumSpeed"]);
end

function value = eventTime(result)
value = NaN;
if isfield(result, "Metrics") && isfield(result.Metrics, "EventTime")
    value = double(result.Metrics.EventTime);
elseif ~isempty(result.Time)
    value = result.Time(end) - result.Time(1);
end
end
