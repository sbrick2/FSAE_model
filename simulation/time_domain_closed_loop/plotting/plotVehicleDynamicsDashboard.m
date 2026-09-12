function [figureHandle, summary] = plotVehicleDynamicsDashboard(source, options)
%PLOTVEHICLEDYNAMICSDASHBOARD 绘制整车动力学开发总览。
%   [FIGURES, SUMMARY] = PLOTVEHICLEDYNAMICSDASHBOARD(SOURCE) 汇总轨迹、实际/参考
%   车速、纵横向加速度、横摆角速度、侧倾/俯仰、四轮法向载荷、轮胎利用率和
%   电池 SOC。SOURCE 可为标准化 result 结构体、MAT 路径或空字符串；为空时
%   自动读取 Track 下最新结果。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 vehicle_dynamics_dashboard 目录。
%   四轮字段顺序为 [FL, FR, RL, RR]；SUMMARY 返回圈时、最高车速、峰值加速度、
%   跟踪误差、SOC 变化、来源文件和保存路径。

arguments
    source = ""
    options.Track (1, 1) string = "autocross"
    options.Axis (1, 1) string = "Distance"
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[result, filePath] = loadRacecarResult(source, options.Track);
[x, xLabel] = resolveRacecarAnalysisAxis(result, options.Axis);
wheelNames = ["FL", "FR", "RL", "RR"];
referenceSpeedUsed = lapResultUsesReferenceSpeed(result);

plotNames = [ ...
    "Vehicle dynamics - trajectory"
    "Vehicle dynamics - speed"
    "Vehicle dynamics - body acceleration"
    "Vehicle dynamics - yaw response"
    "Vehicle dynamics - body attitude"
    "Vehicle dynamics - path accuracy"
    "Vehicle dynamics - wheel loads"
    "Vehicle dynamics - wheel slip"
    "Vehicle dynamics - motor torque"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Vehicle dynamics", plotNames, options.Visible);

ax = axesHandles(1);
if ~isempty(result.Vehicle.X) && ~isempty(result.Vehicle.Y)
    plot(ax, result.Vehicle.X, result.Vehicle.Y, "LineWidth", 1.3);
    axis(ax, "equal"); grid(ax, "on");
    xlabel(ax, "Global X (m)"); ylabel(ax, "Global Y (m)");
else
    plotRacecarChannels(ax, x, [], "Trajectory", "", xLabel);
end
title(ax, "Trajectory");

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
plotRacecarChannels(ax, x, [result.Vehicle.Ax, result.Vehicle.Ay], ...
    "Acceleration (m/s^2)", ["Ax", "Ay"], xLabel);
title(ax, "Body acceleration");

ax = axesHandles(4);
plotRacecarChannels(ax, x, [result.Vehicle.YawRate, ...
    result.Controller.ReferenceYawRate], "Yaw rate (rad/s)", ...
    ["Actual", "Reference"], xLabel);
title(ax, "Yaw response");

ax = axesHandles(5);
plotRacecarChannels(ax, x, rad2deg([result.Vehicle.RollAngle, ...
    result.Vehicle.PitchAngle]), "Angle (deg)", ...
    ["Roll", "Pitch"], xLabel);
title(ax, "Body attitude");

ax = axesHandles(6);
if isfield(result.Track, "VehicleEnvelopeClearance") && ...
        isfield(result.Track, "VehicleEnvelopeViolation")
    boundaryChannels = [result.Track.LateralError, ...
        result.Track.VehicleEnvelopeClearance, ...
        result.Track.VehicleEnvelopeViolation];
    boundaryNames = ["Lateral error", "Envelope clearance", ...
        "Envelope violation"];
else
    boundaryChannels = [result.Track.LateralError, ...
        result.Track.BoundaryViolation];
    boundaryNames = ["Lateral error", "CG boundary violation"];
end
plotRacecarChannels(ax, x, boundaryChannels, "Distance (m)", ...
    boundaryNames, xLabel);
title(ax, "Path accuracy");

ax = axesHandles(7);
plotRacecarChannels(ax, x, result.Wheel.NormalLoad, ...
    "Normal load (N)", wheelNames, xLabel);
title(ax, "Dynamic wheel loads");

ax = axesHandles(8);
plotRacecarChannels(ax, x, result.Wheel.SlipRatio, ...
    "Slip ratio (-)", wheelNames, xLabel);
title(ax, "Wheel slip");

ax = axesHandles(9);
plotRacecarChannels(ax, x, result.Powertrain.MotorTorqueActual, ...
    "Motor torque (N*m)", wheelNames, xLabel);
title(ax, "Motor torque");

summary = struct( ...
    "SourceFile", filePath, ...
    "ReferenceSpeedUsed", referenceSpeedUsed, ...
    "MaximumSpeed", racecarFiniteMetric(result.Vehicle.Speed, "max"), ...
    "MaximumAbsAx", racecarFiniteMetric(result.Vehicle.Ax, "maxabs"), ...
    "MaximumAbsAy", racecarFiniteMetric(result.Vehicle.Ay, "maxabs"), ...
    "MaximumAbsYawRate", racecarFiniteMetric(result.Vehicle.YawRate, "maxabs"), ...
    "MinimumWheelLoad", racecarFiniteMetric(result.Wheel.NormalLoad, "min"), ...
    "MaximumTireUtilization", ...
        racecarFiniteMetric(result.Tire.MuUtilization, "max"), ...
    "VehicleEnvelopeMinimumClearance", metricValue(result, ...
        "MinimumVehicleEnvelopeClearance"), ...
    "VehicleEnvelopeMaximumViolation", metricValue(result, ...
        "MaximumVehicleEnvelopeViolation"));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, ...
    "vehicle_dynamics_dashboard", options.Track);
end

function value = metricValue(result, fieldName)
value = NaN;
if isfield(result, "Metrics") && isfield(result.Metrics, fieldName)
    value = double(result.Metrics.(fieldName));
end
end
