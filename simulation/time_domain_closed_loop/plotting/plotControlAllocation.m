function [figureHandle, summary] = plotControlAllocation(source, options)
%PLOTCONTROLALLOCATION 分析横摆、轮胎力和执行器分配状态。
%   [FIGURES, SUMMARY] = PLOTCONTROLALLOCATION(SOURCE) 绘制期望/实际横摆力矩、
%   横摆角速度跟踪、四轮纵向力与电机转矩、转向和驱制动指令。SOURCE 可为
%   标准化 result 结构体、MAT 路径或空字符串（自动读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 control_allocation 目录。
%   SUMMARY 返回峰值分配误差等指标、来源文件和保存路径。

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

plotNames = [ ...
    "Control allocation - yaw moment"
    "Control allocation - yaw control"
    "Control allocation - longitudinal-force demand"
    "Control allocation - four-wheel motor torque"
    "Control allocation - brake blending"
    "Control allocation - modes and constraints"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Control allocation", plotNames, options.Visible);

ax = axesHandles(1);
plotRacecarChannels(ax, x, [result.Controller.DesiredYawMoment, ...
    result.Controller.AllocatedYawMoment], "Yaw moment (N*m)", ...
    ["Desired", "Allocated"], xLabel);
title(ax, "Yaw-moment allocation");

ax = axesHandles(2);
plotRacecarChannels(ax, x, [result.Controller.ReferenceYawRate, ...
    result.Vehicle.YawRate, result.Controller.YawRateError], ...
    "Yaw rate (rad/s)", ["Reference", "Actual", "Error"], xLabel);
title(ax, "Yaw control");

ax = axesHandles(3);
plotRacecarChannels(ax, x, result.Controller.DesiredLongitudinalForce, ...
    "Desired longitudinal force (N)", "Fx demand", xLabel);
title(ax, "Longitudinal-force demand");

ax = axesHandles(4);
plotRacecarChannels(ax, x, result.Actuator.MotorTorqueRequest, ...
    "Motor torque request (N*m)", wheelNames, xLabel);
title(ax, "Four-wheel motor allocation");

ax = axesHandles(5);
brakeData = [sumOrEmpty(result.Actuator.RegenTorqueRequest), ...
    sumOrEmpty(result.Actuator.FrictionBrakeTorqueRequest)];
plotRacecarChannels(ax, x, brakeData, "Wheel-end torque sum (N*m)", ...
    ["Regeneration", "Friction brake"], xLabel);
title(ax, "Brake blending");

ax = axesHandles(6);
flags = [result.Controller.TVActive, result.Controller.TCActive, ...
    result.Controller.RegenActive, result.Controller.EnergyManagementActive, ...
    result.Controller.ControllerSaturated];
plotRacecarChannels(ax, x, flags, "Active (-)", ...
    ["TV", "TC", "Regen", "Energy", "Saturated"], xLabel);
title(ax, "Controller modes and constraints");

allocationError = pairedDifference(result.Controller.AllocatedYawMoment, ...
    result.Controller.DesiredYawMoment);
summary = struct( ...
    "SourceFile", filePath, ...
    "YawRateErrorRMSE", ...
        racecarFiniteMetric(result.Controller.YawRateError, "rms"), ...
    "YawAllocationErrorRMSE", ...
        racecarFiniteMetric(allocationError, "rms"), ...
    "MaximumAbsDesiredYawMoment", racecarFiniteMetric( ...
        result.Controller.DesiredYawMoment, "maxabs"), ...
    "ControllerSaturatedFraction", activeFraction( ...
        result.Controller.ControllerSaturated), ...
    "TCActiveFraction", activeFraction(result.Controller.TCActive), ...
    "RegenActiveFraction", activeFraction(result.Controller.RegenActive));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "control_allocation", options.Track);
end

function value = sumOrEmpty(data)
if isempty(data), value = []; else, value = sum(data, 2); end
end

function difference = pairedDifference(first, second)
if isempty(first) || isempty(second) || size(first, 1) ~= size(second, 1)
    difference = [];
else
    difference = double(first) - double(second);
end
end

function fraction = activeFraction(data)
if isempty(data), fraction = NaN; return; end
finite = isfinite(data);
fraction = nnz(data(finite) > 0) / max(nnz(finite), 1);
end
