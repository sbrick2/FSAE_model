function [figureHandle, summary] = plotPathTrackingPerformance(source, options)
%PLOTPATHTRACKINGPERFORMANCE 分析路径、车速和横摆角速度跟踪质量。
%   [FIGURES, SUMMARY] = PLOTPATHTRACKINGPERFORMANCE(SOURCE) 绘制实际/参考轨迹、
%   横向误差、航向误差、实际/参考速度、实际/参考横摆角速度及转向输入。
%   SOURCE 可为标准化 result 结构体、MAT 路径或空字符串（读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 path_tracking_performance 目录。
%   SUMMARY 返回横向/航向/速度跟踪误差指标、来源和保存路径。

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
referenceSpeedUsed = lapResultUsesReferenceSpeed(result);

plotNames = [ ...
    "Path tracking - driven trajectory"
    "Path tracking - speed"
    "Path tracking - cross-track error"
    "Path tracking - yaw rate"
    "Path tracking - steering command"
    "Path tracking - boundary margin"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Path tracking performance", plotNames, options.Visible);

ax = axesHandles(1);
plot(ax, result.Vehicle.X, result.Vehicle.Y, "LineWidth", 1.3);
axis(ax, "equal"); grid(ax, "on");
xlabel(ax, "Global X (m)"); ylabel(ax, "Global Y (m)");
title(ax, "Driven trajectory");

ax = axesHandles(2);
if referenceSpeedUsed
    plotRacecarChannels(ax, x, [result.Vehicle.Speed, ...
        result.Track.ReferenceSpeed], "Speed (m/s)", ...
        ["Actual", "Reference"], xLabel);
    title(ax, "Speed tracking");
else
    plotRacecarChannels(ax, x, result.Vehicle.Speed, "Speed (m/s)", ...
        "Actual", xLabel);
    title(ax, "Online geometry-limited speed profile");
end

ax = axesHandles(3);
plotRacecarChannels(ax, x, result.Track.LateralError, ...
    "Lateral error (m)", "Error", xLabel);
hold(ax, "on"); yline(ax, 0, "k:"); hold(ax, "off");
title(ax, "Cross-track error");

ax = axesHandles(4);
plotRacecarChannels(ax, x, [result.Vehicle.YawRate, ...
    result.Controller.ReferenceYawRate], "Yaw rate (rad/s)", ...
    ["Actual", "Reference"], xLabel);
title(ax, "Yaw-rate tracking");

ax = axesHandles(5);
plotRacecarChannels(ax, x, [result.Driver.SteeringRackAngleRequest, ...
    result.Actuator.SteeringRackAngleRequest], "Steering (deg)", ...
    ["Driver request", "Actuator request"], xLabel);
if ~isempty(result.Driver.SteeringRackAngleRequest) || ...
        ~isempty(result.Actuator.SteeringRackAngleRequest)
    lines = findall(ax, "Type", "line");
    for line = reshape(lines, 1, [])
        line.YData = rad2deg(line.YData);
    end
end
title(ax, "Steering command");

ax = axesHandles(6);
if isfield(result.Track, "VehicleEnvelopeClearance") && ...
        isfield(result.Track, "VehicleEnvelopeViolation")
    plotRacecarChannels(ax, x, ...
        [result.Track.VehicleEnvelopeClearance, ...
        result.Track.VehicleEnvelopeViolation], "Distance (m)", ...
        ["Envelope clearance", "Envelope violation"], xLabel);
    title(ax, "Vehicle-envelope boundary margin");
else
    plotRacecarChannels(ax, x, [result.Track.Curvature, ...
        result.Track.BoundaryViolation], "Curvature / violation", ...
        ["Curvature (1/m)", "Boundary violation (m)"], xLabel);
    title(ax, "Track demand and boundary");
end

if referenceSpeedUsed
    speedError = pairedDifference( ...
        result.Vehicle.Speed, result.Track.ReferenceSpeed);
else
    speedError = [];
end
yawError = pairedDifference(result.Vehicle.YawRate, ...
    result.Controller.ReferenceYawRate);
summary = struct( ...
    "SourceFile", filePath, ...
    "ReferenceSpeedUsed", referenceSpeedUsed, ...
    "SpeedRMSE", racecarFiniteMetric(speedError, "rms"), ...
    "LateralErrorRMSE", ...
        racecarFiniteMetric(result.Track.LateralError, "rms"), ...
    "MaximumAbsLateralError", ...
        racecarFiniteMetric(result.Track.LateralError, "maxabs"), ...
    "YawRateRMSE", racecarFiniteMetric(yawError, "rms"), ...
    "BoundaryViolationFraction", activeFraction(result.Track.BoundaryViolation), ...
    "VehicleEnvelopeMinimumClearance", envelopeMetric(result, ...
        "MinimumVehicleEnvelopeClearance"), ...
    "VehicleEnvelopeViolationFraction", envelopeMetric(result, ...
        "VehicleEnvelopeViolationFraction"));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, ...
    "path_tracking_performance", options.Track);
end

function value = envelopeMetric(result, fieldName)
value = NaN;
if isfield(result, "Metrics") && isfield(result.Metrics, fieldName)
    value = double(result.Metrics.(fieldName));
end
end

function difference = pairedDifference(first, second)
if isempty(first) || isempty(second) || size(first, 1) ~= size(second, 1)
    difference = [];
else
    difference = double(first) - double(second);
end
end

function fraction = activeFraction(value)
if isempty(value)
    fraction = NaN;
else
    finite = isfinite(value);
    fraction = nnz(value(finite) > 0) / max(nnz(finite), 1);
end
end
