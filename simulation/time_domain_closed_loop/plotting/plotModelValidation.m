function [figureHandle, metrics] = plotModelValidation(simulationSource, measurementSource, options)
%PLOTMODELVALIDATION 将时域仿真结果与实测数据按时间对齐验证。
%   [FIGURES, METRICS] = PLOTMODELVALIDATION(SIMULATIONSOURCE, MEASUREMENTSOURCE)
%   比较 Speed、Ax、Ay、YawRate，并给出 RMSE、最大绝对误差和有效样本数。
%   SIMULATIONSOURCE 可为标准化 result、MAT 路径或空字符串；MEASUREMENTSOURCE
%   可为 table/timetable/struct、CSV/MAT 路径或空字符串（自动读取最新验证数据）。
%
%   实测数据必须使用 SI 单位。时间字段可为 Time/Timestamp/ElapsedTime；信号别名：
%   Speed/VehicleSpeed/Ux，Ax/AccelX/LongitudinalAcceleration，
%   Ay/AccelY/LateralAcceleration，YawRate/YawRateRadPerSec。
%   名称-值参数：Track 可选 acceleration/skidpad/autocross/endurance；Visible
%   可选 on/off；SaveFigure 控制 FIG 保存；OutputFolder 为空时使用默认目录。
%   METRICS 为验证指标表，UserData 中记录仿真和实测来源文件。

arguments
    simulationSource = ""
    measurementSource = ""
    options.Track (1, 1) string = "autocross"
    options.Visible (1, 1) string = "on"
    options.SaveFigure (1, 1) logical = false
    options.OutputFolder (1, 1) string = ""
end

[result, simulationFile] = loadRacecarResult( ...
    simulationSource, options.Track);
[measurement, measurementFile] = loadMeasurement(measurementSource);
measurementTime = readMeasurement(measurement, ...
    ["Time", "Timestamp", "ElapsedTime"]);
assert(~isempty(measurementTime), "FSAE:Analysis:MeasurementTime", ...
    "Measurement data must contain Time, Timestamp or ElapsedTime in seconds.");
measurementTime = double(measurementTime(:));

definitions = {
    "Speed", result.Vehicle.Speed, ["Speed", "VehicleSpeed", "Ux"], "m/s";
    "Ax", result.Vehicle.Ax, ["Ax", "AccelX", "LongitudinalAcceleration"], "m/s^2";
    "Ay", result.Vehicle.Ay, ["Ay", "AccelY", "LateralAcceleration"], "m/s^2";
    "YawRate", result.Vehicle.YawRate, ["YawRate", "YawRateRadPerSec"], "rad/s"};

plotNames = "Model validation - " + string(definitions(:, 1));
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "Model validation", plotNames, options.Visible);

signalName = strings(size(definitions, 1), 1);
rmse = NaN(size(definitions, 1), 1);
maximumAbsError = NaN(size(definitions, 1), 1);
sampleCount = zeros(size(definitions, 1), 1);
for index = 1:size(definitions, 1)
    signalName(index) = definitions{index, 1};
    simulated = definitions{index, 2};
    measured = readMeasurement(measurement, definitions{index, 3});
    unit = definitions{index, 4};
    ax = axesHandles(index);
    if isempty(simulated) || isempty(measured)
        plotRacecarChannels(ax, (1:2)', [], signalName(index), "", "Time (s)");
        title(ax, signalName(index) + " unavailable");
        continue
    end
    measured = double(measured(:));
    count = min(numel(measurementTime), numel(measured));
    tMeasured = measurementTime(1:count);
    measured = measured(1:count);
    simulatedAtMeasurement = interpolateTime(result.Time, simulated, tMeasured);
    valid = isfinite(tMeasured) & isfinite(measured) & ...
        isfinite(simulatedAtMeasurement);
    errorValue = simulatedAtMeasurement(valid) - measured(valid);
    rmse(index) = racecarFiniteMetric(errorValue, "rms");
    maximumAbsError(index) = racecarFiniteMetric(errorValue, "maxabs");
    sampleCount(index) = nnz(valid);
    plotRacecarChannels(ax, tMeasured, ...
        [simulatedAtMeasurement, measured], signalName(index) + " (" + unit + ")", ...
        ["Simulation", "Measurement"], "Time (s)");
    title(ax, signalName(index) + " | RMSE=" + compose("%.4g", rmse(index)));
end

metrics = table(signalName, rmse, maximumAbsError, sampleCount, ...
    VariableNames = ["Signal", "RMSE", "MaximumAbsError", "SampleCount"]);
metrics.Properties.UserData = struct( ...
    "SimulationFile", simulationFile, "MeasurementFile", measurementFile);
metrics.Properties.UserData.FigureFiles = finalizeRacecarFigure(figureHandle, ...
    options.SaveFigure, options.OutputFolder, "model_validation", options.Track);
metrics.Properties.Description = "Simulation-to-measurement validation metrics";
end

function [measurement, filePath] = loadMeasurement(source)
filePath = "";
if istable(source) || istimetable(source) || isstruct(source)
    measurement = source;
    return
end
filePath = string(source);
if strlength(filePath) == 0
    filePath = newestMeasurementFile();
end
assert(isfile(filePath), "FSAE:Analysis:MeasurementFile", ...
    "Measurement file does not exist: %s.", filePath);
[~, ~, extension] = fileparts(filePath);
if strcmpi(extension, ".csv")
    measurement = readtable(filePath, VariableNamingRule = "preserve");
elseif strcmpi(extension, ".mat")
    loaded = load(filePath);
    measurement = firstSupportedValue(loaded, filePath);
else
    error("FSAE:Analysis:MeasurementFormat", ...
        "Measurement file must be CSV or MAT.");
end
end

function filePath = newestMeasurementFile()
folder = fullfile(racecarAnalysisProjectRoot(), "data", "ValidationData");
candidates = [dir(fullfile(folder, "*.mat")); dir(fullfile(folder, "*.csv"))];
assert(~isempty(candidates), "FSAE:Analysis:NoMeasurement", ...
    "Provide measurementSource or add a CSV/MAT file under %s.", folder);
[~, newest] = max([candidates.datenum]);
filePath = string(fullfile(candidates(newest).folder, candidates(newest).name));
end

function value = firstSupportedValue(loaded, filePath)
fields = fieldnames(loaded);
value = [];
for index = 1:numel(fields)
    candidate = loaded.(fields{index});
    if istable(candidate) || istimetable(candidate) || isstruct(candidate)
        value = candidate;
        return
    end
end
assert(~isempty(value), "FSAE:Analysis:MeasurementVariable", ...
    "MAT file %s contains no table, timetable or struct measurement variable.", ...
    filePath);
end

function value = readMeasurement(data, candidates)
value = [];
if istimetable(data)
    names = string(data.Properties.VariableNames);
    match = findName(names, candidates);
    if ~isempty(match), value = data.(char(names(match))); return; end
    if any(lower(candidates) == "time")
        value = seconds(data.Properties.RowTimes - data.Properties.RowTimes(1));
    end
elseif istable(data)
    names = string(data.Properties.VariableNames);
    match = findName(names, candidates);
    if ~isempty(match), value = data.(char(names(match))); end
elseif isstruct(data)
    names = string(fieldnames(data));
    match = findName(names, candidates);
    if ~isempty(match), value = data.(char(names(match))); end
end
if iscell(value) && isscalar(value), value = value{1}; end
end

function match = findName(names, candidates)
match = [];
normalizedNames = lower(regexprep(names, "[^A-Za-z0-9]", ""));
for candidate = candidates
    normalized = lower(regexprep(candidate, "[^A-Za-z0-9]", ""));
    index = find(normalizedNames == normalized, 1);
    if ~isempty(index), match = index; return; end
end
end

function output = interpolateTime(time, signal, targetTime)
time = double(time(:)); signal = double(signal(:));
valid = isfinite(time) & isfinite(signal);
time = time(valid); signal = signal(valid);
[time, uniqueIndex] = unique(time, "stable"); signal = signal(uniqueIndex);
if numel(time) < 2
    output = NaN(size(targetTime));
else
    output = interp1(time, signal, targetTime, "linear", NaN);
end
end
