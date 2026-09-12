function [figureHandle, comparison] = compareSimulationRuns(sourceA, sourceB, options)
%COMPARESIMULATIONRUNS 对比两组时域闭环仿真结果。
%   [FIGURES, COMPARISON] = COMPARESIMULATIONRUNS(SOURCEA, SOURCEB) 对两组结果
%   统一插值后比较圈时、最高车速、横向误差和净能耗。SOURCEA/SOURCEB 可为
%   标准化 result 结构体或 MAT 路径；两者都为空时读取 Track 下最新两组结果。
%
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Axis 可选
%   Time、Distance、EventDistance、PathS；GridPointCount 为公共插值网格点数
%   （至少 50）；LabelA/LabelB 为图例名称；Visible 可选 on/off；SaveFigure
%   控制 FIG 保存；OutputFolder 为空时使用默认 simulation_comparison 结果目录。
%   COMPARISON 为指标对比表，UserData 中记录两个来源文件。
%   示例：compareSimulationRuns("baseline.mat", "setup_b.mat", LabelB="Setup B");

arguments
    sourceA = ""
    sourceB = ""
    options.Track (1, 1) string = "autocross"
    options.Axis (1, 1) string = "EventDistance"
    options.GridPointCount (1, 1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(options.GridPointCount, 50)} = 1500
    options.LabelA (1, 1) string = "Run A"
    options.LabelB (1, 1) string = "Run B"
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

if isEmptySource(sourceA) && isEmptySource(sourceB)
    files = newestResultFiles(options.Track, 2);
    sourceA = files(2);
    sourceB = files(1);
end
[resultA, fileA] = loadRacecarResult(sourceA, options.Track);
[resultB, fileB] = loadRacecarResult(sourceB, options.Track);
[axisA, axisLabel] = resolveRacecarAnalysisAxis(resultA, options.Axis);
[axisB, ~] = resolveRacecarAnalysisAxis(resultB, options.Axis);
gridAxis = commonGrid(axisA, axisB, options.GridPointCount);

comparisonName = options.LabelA + " vs " + options.LabelB;
plotNames = comparisonName + [ ...
    " - trajectory"
    " - speed"
    " - path error"
    " - yaw response"
    " - tire demand"
    " - speed gain/loss"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    options.LabelA + " vs " + options.LabelB, plotNames, options.Visible);

ax = axesHandles(1);
plot(ax, resultA.Vehicle.X, resultA.Vehicle.Y, "LineWidth", 1.25);
hold(ax, "on");
plot(ax, resultB.Vehicle.X, resultB.Vehicle.Y, "--", "LineWidth", 1.25);
hold(ax, "off"); axis(ax, "equal"); grid(ax, "on");
xlabel(ax, "Global X (m)"); ylabel(ax, "Global Y (m)");
legend(ax, options.LabelA, options.LabelB, "Location", "best");
title(ax, "Trajectory");

speedA = interpolateSignal(axisA, resultA.Vehicle.Speed, gridAxis);
speedB = interpolateSignal(axisB, resultB.Vehicle.Speed, gridAxis);
ax = axesHandles(2);
plotRacecarChannels(ax, gridAxis, [speedA, speedB], "Speed (m/s)", ...
    [options.LabelA, options.LabelB], axisLabel);
title(ax, "Speed comparison");

errorA = interpolateSignal(axisA, resultA.Track.LateralError, gridAxis);
errorB = interpolateSignal(axisB, resultB.Track.LateralError, gridAxis);
ax = axesHandles(3);
plotRacecarChannels(ax, gridAxis, [errorA, errorB], ...
    "Lateral error (m)", [options.LabelA, options.LabelB], axisLabel);
title(ax, "Path error");

yawA = interpolateSignal(axisA, resultA.Vehicle.YawRate, gridAxis);
yawB = interpolateSignal(axisB, resultB.Vehicle.YawRate, gridAxis);
ax = axesHandles(4);
plotRacecarChannels(ax, gridAxis, [yawA, yawB], "Yaw rate (rad/s)", ...
    [options.LabelA, options.LabelB], axisLabel);
title(ax, "Yaw response");

utilA = interpolateSignal(axisA, maximumAcrossWheels( ...
    resultA.Tire.MuUtilization), gridAxis);
utilB = interpolateSignal(axisB, maximumAcrossWheels( ...
    resultB.Tire.MuUtilization), gridAxis);
ax = axesHandles(5);
plotRacecarChannels(ax, gridAxis, [utilA, utilB], ...
    "Peak wheel utilization (-)", [options.LabelA, options.LabelB], axisLabel);
title(ax, "Tire demand");

ax = axesHandles(6);
plotRacecarChannels(ax, gridAxis, speedB - speedA, ...
    "Speed delta B-A (m/s)", "Delta", axisLabel);
hold(ax, "on"); yline(ax, 0, "k:"); hold(ax, "off");
title(ax, "Speed gain/loss");

metricsA = runMetrics(resultA);
metricsB = runMetrics(resultB);
comparison = table([options.LabelA; options.LabelB; "Delta B-A"], ...
    [metricsA.EventTime; metricsB.EventTime; metricsB.EventTime-metricsA.EventTime], ...
    [metricsA.MaximumSpeed; metricsB.MaximumSpeed; metricsB.MaximumSpeed-metricsA.MaximumSpeed], ...
    [metricsA.LateralErrorRMSE; metricsB.LateralErrorRMSE; ...
        metricsB.LateralErrorRMSE-metricsA.LateralErrorRMSE], ...
    [metricsA.NetEnergyWh; metricsB.NetEnergyWh; metricsB.NetEnergyWh-metricsA.NetEnergyWh], ...
    VariableNames = ["Run", "EventTime", "MaximumSpeed", ...
    "LateralErrorRMSE", "NetEnergyWh"]);
comparison.Properties.UserData = struct("SourceA", fileA, "SourceB", fileB);
comparison.Properties.UserData.FigureFiles = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "simulation_comparison", ...
    options.Track);
comparison.Properties.Description = "Closed-loop simulation comparison metrics";
end

function tf = isEmptySource(source)
tf = (ischar(source) || isstring(source)) && strlength(string(source)) == 0;
end

function files = newestResultFiles(trackName, count)
root = fullfile(racecarAnalysisProjectRoot(), "results", ...
    "time_domain_closed_loop", lower(trackName));
candidates = dir(fullfile(root, "**", "lap_simulation_result.mat"));
assert(numel(candidates) >= count, "FSAE:Analysis:ComparisonFiles", ...
    "At least %d saved %s results are required.", count, trackName);
[~, order] = sort([candidates.datenum], "descend");
files = strings(count, 1);
for index = 1:count
    item = candidates(order(index));
    files(index) = string(fullfile(item.folder, item.name));
end
end

function grid = commonGrid(first, second, count)
lowerBound = max(min(first), min(second));
upperBound = min(max(first), max(second));
assert(upperBound > lowerBound, "FSAE:Analysis:NoAxisOverlap", ...
    "The two results do not overlap on the selected axis.");
grid = linspace(lowerBound, upperBound, count)';
end

function output = interpolateSignal(axisValue, signal, grid)
output = [];
if isempty(signal) || size(signal, 1) ~= numel(axisValue), return; end
[axisValue, order] = sort(double(axisValue(:)));
signal = double(signal(order, :));
[axisValue, uniqueIndex] = unique(axisValue, "stable");
signal = signal(uniqueIndex, :);
if numel(axisValue) < 2, return; end
output = interp1(axisValue, signal, grid, "linear", NaN);
end

function value = maximumAcrossWheels(data)
if isempty(data), value = []; else, value = max(data, [], 2); end
end

function metrics = runMetrics(result)
metrics.EventTime = result.Time(end) - result.Time(1);
if isfield(result.Metrics, "EventTime") && ~isempty(result.Metrics.EventTime) ...
        && isscalar(result.Metrics.EventTime) && isfinite(result.Metrics.EventTime)
    metrics.EventTime = result.Metrics.EventTime;
end
metrics.MaximumSpeed = racecarFiniteMetric(result.Vehicle.Speed, "max");
metrics.LateralErrorRMSE = racecarFiniteMetric(result.Track.LateralError, "rms");
metrics.NetEnergyWh = netEnergy(result.Time, result.Battery.Power);
end

function value = netEnergy(time, power)
value = NaN;
if isempty(power) || numel(power) ~= numel(time), return; end
valid = isfinite(time) & isfinite(power);
if nnz(valid) >= 2
    value = trapz(time(valid), power(valid)) / 3600;
end
end
