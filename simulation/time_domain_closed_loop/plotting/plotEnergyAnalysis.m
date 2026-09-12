function [figureHandle, summary] = plotEnergyAnalysis(source, options)
%PLOTENERGYANALYSIS 分析电池能量、再生制动和功率限制。
%   [FIGURES, SUMMARY] = PLOTENERGYANALYSIS(SOURCE) 绘制电池功率/电流/电压/SOC、
%   累计能量、四电机机械功率及驱动/回收限制。SOURCE 可为标准化 result 结构体、
%   MAT 路径或空字符串（自动读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 energy_analysis 目录。
%   SUMMARY 返回净能耗、驱动能耗、回收能量、峰值功率、来源和保存路径。

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
time = double(result.Time(:));
batteryPower = result.Battery.Power;
if isempty(batteryPower) && ~isempty(result.Powertrain.MotorMechanicalPower)
    batteryPower = sum(result.Powertrain.MotorMechanicalPower, 2);
end
[netEnergyWh, driveEnergyWh, regenEnergyWh, cumulativeWh] = ...
    integrateEnergy(time, batteryPower);
motorPower = result.Powertrain.MotorMechanicalPower;

plotNames = [ ...
    "Energy analysis - battery power"
    "Energy analysis - pack electrical state"
    "Energy analysis - state of charge"
    "Energy analysis - per-motor power"
    "Energy analysis - cumulative energy"
    "Energy analysis - constraints"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Energy analysis", plotNames, options.Visible);

ax = axesHandles(1);
plotRacecarChannels(ax, x, batteryPower / 1000, ...
    "Battery power (kW)", "Battery", xLabel);
hold(ax, "on"); yline(ax, 0, "k:"); hold(ax, "off");
title(ax, "Battery power");

ax = axesHandles(2);
plotRacecarChannels(ax, x, [result.Battery.Voltage, ...
    result.Battery.Current], "Voltage (V) / current (A)", ...
    ["Voltage", "Current"], xLabel);
title(ax, "Pack electrical state");

ax = axesHandles(3);
plotRacecarChannels(ax, x, 100 * result.Battery.SOC, ...
    "SOC (%)", "SOC", xLabel);
title(ax, "State of charge");

ax = axesHandles(4);
plotRacecarChannels(ax, x, motorPower / 1000, ...
    "Motor mechanical power (kW)", ["FL", "FR", "RL", "RR"], xLabel);
title(ax, "Per-motor power");

ax = axesHandles(5);
plotRacecarChannels(ax, x, cumulativeWh, ...
    "Cumulative net energy (Wh)", "Net energy", xLabel);
title(ax, "Cumulative energy");

ax = axesHandles(6);
flags = [result.Powertrain.TotalPowerLimitActive, ...
    result.Powertrain.RegenLimitActive, ...
    result.Controller.EnergyManagementActive];
plotRacecarChannels(ax, x, flags, "Active (-)", ...
    ["Drive power limit", "Regen limit", "Energy management"], xLabel);
title(ax, "Energy constraints");

summary = struct( ...
    "SourceFile", filePath, ...
    "NetBatteryEnergyWh", netEnergyWh, ...
    "DriveEnergyWh", driveEnergyWh, ...
    "RecoveredEnergyWh", regenEnergyWh, ...
    "SOCChange", socChange(result.Battery.SOC), ...
    "PeakDrivePowerKW", racecarFiniteMetric(batteryPower / 1000, "max"), ...
    "PeakRegenPowerKW", max(0, -racecarFiniteMetric( ...
        batteryPower / 1000, "min")), ...
    "DrivePowerLimitFraction", activeFraction( ...
        result.Powertrain.TotalPowerLimitActive), ...
    "RegenLimitFraction", activeFraction( ...
        result.Powertrain.RegenLimitActive));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "energy_analysis", options.Track);
end

function [netWh, driveWh, regenWh, cumulativeWh] = integrateEnergy(time, power)
netWh = NaN; driveWh = NaN; regenWh = NaN; cumulativeWh = [];
if isempty(power) || numel(power) ~= numel(time), return; end
power = double(power(:));
valid = isfinite(time) & isfinite(power);
if nnz(valid) < 2, return; end
cleanPower = power; cleanPower(~valid) = 0;
cumulativeWh = cumtrapz(time, cleanPower) / 3600;
netWh = trapz(time(valid), power(valid)) / 3600;
driveWh = trapz(time(valid), max(power(valid), 0)) / 3600;
regenWh = -trapz(time(valid), min(power(valid), 0)) / 3600;
end

function value = socChange(soc)
if isempty(soc), value = NaN; else, value = soc(end) - soc(1); end
end

function fraction = activeFraction(data)
if isempty(data), fraction = NaN; return; end
finite = isfinite(data);
fraction = nnz(data(finite) > 0) / max(nnz(finite), 1);
end
