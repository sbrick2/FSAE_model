function [figureHandle, summary] = plotMotorOperatingMap(source, options)
%PLOTMOTOROPERATINGMAP 绘制四电机工作点及公共限制。
%   [FIGURES, SUMMARY] = PLOTMOTOROPERATINGMAP(SOURCE) 分析四轮电机转速、实际
%   转矩、机械功率和转矩-转速散点，并叠加当前数据字典中的转矩/转速限制。
%   SOURCE 可为标准化 result 结构体、MAT 路径或空字符串（读取 Track 最新结果）。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 motor_operating_map 目录。
%   四轮字段顺序为 [FL, FR, RL, RR]；SUMMARY 返回峰值转速、转矩、功率等指标。

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
motorSpeedRPM = result.Powertrain.MotorRPM;
if isempty(motorSpeedRPM) && ~isempty(result.Powertrain.MotorSpeed)
    motorSpeedRPM = result.Powertrain.MotorSpeed * 60 / (2 * pi);
end
motorTorque = result.Powertrain.MotorTorqueActual;
[torqueLimit, speedLimitRPM] = readMotorLimits();

plotNames = [ ...
    "Motor operating map - torque-speed points"
    "Motor operating map - torque history"
    "Motor operating map - mechanical power"
    "Motor operating map - limit activity"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Motor operating map", plotNames, options.Visible);

ax = axesHandles(1);
scatterOperatingPoints(ax, motorSpeedRPM, motorTorque, wheelNames);
hold(ax, "on");
if isfinite(torqueLimit) && isfinite(speedLimitRPM)
    plot(ax, [0, speedLimitRPM], [torqueLimit, torqueLimit], "k--", ...
        "DisplayName", "Positive torque limit");
    plot(ax, [0, speedLimitRPM], [-torqueLimit, -torqueLimit], "k--", ...
        "HandleVisibility", "off");
    xline(ax, speedLimitRPM, "k:", "Speed limit");
end
hold(ax, "off"); grid(ax, "on");
xlabel(ax, "Motor speed (rpm)"); ylabel(ax, "Motor torque (N*m)");
title(ax, "Torque-speed operating points");

ax = axesHandles(2);
plotRacecarChannels(ax, x, motorTorque, "Motor torque (N*m)", ...
    wheelNames, xLabel);
title(ax, "Torque history");

ax = axesHandles(3);
plotRacecarChannels(ax, x, result.Powertrain.MotorMechanicalPower / 1000, ...
    "Mechanical power (kW)", wheelNames, xLabel);
title(ax, "Per-motor mechanical power");

ax = axesHandles(4);
plotRacecarChannels(ax, x, result.Powertrain.MotorLimitActive, ...
    "Limit active (-)", wheelNames, xLabel);
title(ax, "Motor limit activity");

summary = struct( ...
    "SourceFile", filePath, ...
    "MotorTorqueLimit", torqueLimit, ...
    "MotorSpeedLimitRPM", speedLimitRPM, ...
    "MaximumAbsMotorTorqueByWheel", perWheelMetric(motorTorque, "maxabs"), ...
    "MaximumAbsMotorSpeedRPMByWheel", ...
        perWheelMetric(motorSpeedRPM, "maxabs"), ...
    "MaximumAbsMechanicalPowerKWByWheel", perWheelMetric( ...
        result.Powertrain.MotorMechanicalPower / 1000, "maxabs"), ...
    "MotorLimitActiveFraction", activeFraction( ...
        result.Powertrain.MotorLimitActive));
summary.FigureFile = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "motor_operating_map", options.Track);
end

function scatterOperatingPoints(ax, speed, torque, names)
if isempty(speed) || isempty(torque)
    plotRacecarChannels(ax, (1:2)', [], "Motor torque (N*m)", "", ...
        "Motor speed (rpm)");
    return
end
count = min([4, size(speed, 2), size(torque, 2)]);
colors = lines(4);
hold(ax, "on"); handles = gobjects(count, 1);
for wheel = 1:count
    handles(wheel, 1) = scatter(ax, abs(speed(:, wheel)), ...
        torque(:, wheel), 9, colors(wheel, :), "filled", ...
        "MarkerFaceAlpha", 0.30);
end
hold(ax, "off");
if count > 1, legend(ax, handles, names(1:count), "Location", "best"); end
end

function [torqueLimit, speedLimitRPM] = readMotorLimits()
torqueLimit = NaN; speedLimitRPM = NaN;
dictionaryPath = fullfile(racecarAnalysisProjectRoot(), ...
    "data", "VehicleData.sldd");
if ~isfile(dictionaryPath), return; end
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
section = getSection(dictionary, "Design Data");
powertrain = getValue(getEntry(section, "Powertrain"));
torqueLimit = unwrap(powertrain.MotorTorqueLimit);
speedLimit = unwrap(powertrain.MotorSpeedLimit);
speedLimitRPM = speedLimit * 60 / (2 * pi);
end

function value = unwrap(value)
if isstruct(value) && isfield(value, "Value"), value = value.Value; end
value = double(value);
end

function values = perWheelMetric(data, operation)
values = NaN(1, 4);
if isempty(data), return; end
for wheel = 1:min(4, size(data, 2))
    values(wheel) = racecarFiniteMetric(data(:, wheel), operation);
end
end

function fraction = activeFraction(data)
if isempty(data), fraction = NaN; return; end
finite = isfinite(data);
fraction = nnz(data(finite) > 0) / max(nnz(finite), 1);
end
