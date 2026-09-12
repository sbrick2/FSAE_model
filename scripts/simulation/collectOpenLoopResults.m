function result = collectOpenLoopResults(simulationOutput, runInfo)
%COLLECTOPENLOOPRESULTS Normalize logged open-loop signals into one result.
%   Missing signals are reported in RESULT.Meta and are never zero-filled.

arguments
    simulationOutput (1, 1) Simulink.SimulationOutput
    runInfo struct = struct
end

signalMap = containers.Map("KeyType", "char", "ValueType", "any");
datasetNames = strings(0, 1);
try
    outputNames = string(simulationOutput.who);
catch
    outputNames = ["yout", "logsout"];
end
for name = ["yout", "logsout"]
    if ~any(outputNames == name)
        continue
    end
    try
        candidate = simulationOutput.get(char(name));
    catch
        continue
    end
    if ~isa(candidate, "Simulink.SimulationData.Dataset")
        continue
    end
    datasetNames(end + 1, 1) = name; %#ok<AGROW>
    for index = 1:candidate.numElements
        element = candidate{index};
        [elementName, blockPath, values] = readDatasetElement(element, index);
        [signalTime, data] = readSignalValues(values);
        if isempty(signalTime)
            signalTime = readSimulationTime(simulationOutput);
        end
        registerSignal(signalMap, elementName, blockPath, signalTime, data);
        if isstruct(data)
            registerStructSignals(signalMap, elementName, ...
                blockPath, signalTime, data);
        end
    end
end
if isempty(datasetNames)
    error("FSAE:OpenLoopDatasetMissing", ...
        "SimulationOutput does not contain a yout or logsout Dataset.");
end
datasetName = strjoin(datasetNames, "+");

time = readSimulationTime(simulationOutput);
if isempty(time)
    keysInMap = keys(signalMap);
    if ~isempty(keysInMap)
        time = signalMap(keysInMap{1}).Time;
    end
end
time = normalizeTime(time);

result = initializeResult();
result.Time = time;
mapping = signalMapping();
missingSignals = strings(0, 1);
nonFiniteSignals = strings(0, 1);
resampledSignals = strings(0, 1);
for index = 1:size(mapping, 1)
    resultPath = mapping{index, 1};
    [data, signalTime, sourceName] = findSignal(signalMap, mapping{index, 2});
    if isempty(data)
        missingSignals(end + 1, 1) = resultPath; %#ok<AGROW>
        continue
    end
    if isempty(time)
        time = normalizeTime(signalTime);
        result.Time = time;
    elseif ~sameTimeVector(time, signalTime)
        data = resampleSignal(data, signalTime, time);
        resampledSignals(end + 1, 1) = resultPath; %#ok<AGROW>
    end
    data = normalizeSignalShape(data, numel(time));
    result = setResultPath(result, resultPath, data);
    if isnumeric(data) && any(~isfinite(data(:)))
        nonFiniteSignals(end + 1, 1) = sourceName; %#ok<AGROW>
    end
end

timeIssue = validateTime(time);
meta = makeMeta(runInfo, datasetName, signalMap, missingSignals, ...
    nonFiniteSignals, resampledSignals, timeIssue);
meta.DerivedMetrics = deriveMetrics(result);
result.Time = time;
result.Meta = meta;
end

function result = initializeResult()
result = struct;
result.Time = zeros(0, 1);
result.Vehicle = emptyFields(["X", "Y", "Psi", "Ux", "Uy", "YawRate", "Ax", "Ay"]);
result.Wheel = emptyFields(["Speed", "VelocityXWheel", "VelocityYWheel", ...
    "SlipRatio", "SlipAngle", "LowSpeedBlend", "NormalLoad"]);
result.Tire = emptyFields(["FxWheel", "FyWheel", "MuUtilization", "SaturationScale"]);
result.Aero = emptyFields(["ForceXBody", "ForceYBody", "DownforceFront", ...
    "DownforceRear", "RelativeAirSpeed", "DynamicPressure"]);
result.Powertrain = emptyFields(["MotorTorqueRequest", "MotorTorqueActual", ...
    "WheelAppliedTorque", "MotorSpeed", "MotorMechanicalPower", ...
    "ElectricalPowerUnconstrained", "ElectricalPowerActual", ...
    "SpeedLimitScale", "TorqueSaturationResidual"]);
result.Battery = emptyFields(["AllowedElectricalPower", "PowerScale", ...
    "Current", "SOC", "DrivePowerClipped", "RegenPowerClipped"]);
end

function values = emptyFields(names)
values = struct;
for name = names
    values.(char(name)) = [];
end
end

function mapping = signalMapping()
mapping = {
    "Vehicle.X", {"Vehicle.X", "VehicleState.X", "Vehicle7DOF.X", "X"};
    "Vehicle.Y", {"Vehicle.Y", "VehicleState.Y", "Vehicle7DOF.Y", "Y"};
    "Vehicle.Psi", {"Vehicle.Psi", "VehicleState.Psi", "Vehicle7DOF.Psi", "Psi"};
    "Vehicle.Ux", {"Vehicle.Ux", "VehicleState.Ux", "Vehicle7DOF.Ux", "Ux"};
    "Vehicle.Uy", {"Vehicle.Uy", "VehicleState.Uy", "Vehicle7DOF.Uy", "Uy"};
    "Vehicle.YawRate", {"Vehicle.YawRate", "VehicleState.YawRate", "YawRate"};
    "Vehicle.Ax", {"Vehicle.Ax", "VehicleState.Ax", "Vehicle7DOF.Ax", "Ax"};
    "Vehicle.Ay", {"Vehicle.Ay", "VehicleState.Ay", "Vehicle7DOF.Ay", "Ay"};
    "Wheel.Speed", {"Wheel.Speed", "WheelSpeed", "VehicleState.WheelSpeed"};
    "Wheel.VelocityXWheel", {"Wheel.VelocityXWheel", "WheelVelocityXWheel"};
    "Wheel.VelocityYWheel", {"Wheel.VelocityYWheel", "WheelVelocityYWheel"};
    "Wheel.SlipRatio", {"Wheel.SlipRatio", "SlipRatio", "VehicleState.SlipRatio"};
    "Wheel.SlipAngle", {"Wheel.SlipAngle", "SlipAngle", "VehicleState.SlipAngle"};
    "Wheel.LowSpeedBlend", {"Wheel.LowSpeedBlend", "LowSpeedBlend"};
    "Wheel.NormalLoad", {"Wheel.NormalLoad", "NormalLoad", "VehicleState.NormalLoad"};
    "Tire.FxWheel", {"Tire.FxWheel", "TireForceXWheel", "TireFx", "VehicleState.TireFx"};
    "Tire.FyWheel", {"Tire.FyWheel", "TireForceYWheel", "TireFy", "VehicleState.TireFy"};
    "Tire.MuUtilization", {"Tire.MuUtilization", "MuUtilization", "VehicleState.MuUtilization"};
    "Tire.SaturationScale", {"Tire.SaturationScale", "SaturationScale"};
    "Aero.ForceXBody", {"Aero.ForceXBody", "AeroModel.ForceXBody", "ForceXBody"};
    "Aero.ForceYBody", {"Aero.ForceYBody", "AeroModel.ForceYBody", "ForceYBody"};
    "Aero.DownforceFront", {"Aero.DownforceFront", "DownforceFront"};
    "Aero.DownforceRear", {"Aero.DownforceRear", "DownforceRear"};
    "Aero.RelativeAirSpeed", {"Aero.RelativeAirSpeed", "RelativeAirSpeed"};
    "Aero.DynamicPressure", {"Aero.DynamicPressure", "DynamicPressure"};
    "Powertrain.MotorTorqueRequest", {"Powertrain.MotorTorqueRequest", "MotorTorqueRequest"};
    "Powertrain.MotorTorqueActual", {"Powertrain.MotorTorqueActual", "MotorTorqueActual"};
    "Powertrain.WheelAppliedTorque", {"Powertrain.WheelAppliedTorque", "WheelAppliedTorque"};
    "Powertrain.MotorSpeed", {"Powertrain.MotorSpeed", "MotorSpeed"};
    "Powertrain.MotorMechanicalPower", {"Powertrain.MotorMechanicalPower", "MotorMechanicalPower"};
    "Powertrain.ElectricalPowerUnconstrained", {"Powertrain.ElectricalPowerUnconstrained", "ElectricalPowerUnconstrained"};
    "Powertrain.ElectricalPowerActual", {"Powertrain.ElectricalPowerActual", "ElectricalPowerActual"};
    "Powertrain.SpeedLimitScale", {"Powertrain.SpeedLimitScale", "SpeedLimitScale"};
    "Powertrain.TorqueSaturationResidual", {"Powertrain.TorqueSaturationResidual", "TorqueSaturationResidual"};
    "Battery.AllowedElectricalPower", {"Battery.AllowedElectricalPower", ...
        "AllowedElectricalPower", "BatteryPower"};
    "Battery.PowerScale", {"Battery.PowerScale", "PowerScale"};
    "Battery.Current", {"Battery.Current", "BatteryCurrent", "Current"};
    "Battery.SOC", {"Battery.SOC", "BatterySOC", "SOC"};
    "Battery.DrivePowerClipped", {"Battery.DrivePowerClipped", "DrivePowerClipped"};
    "Battery.RegenPowerClipped", {"Battery.RegenPowerClipped", "RegenPowerClipped"};
    };
end

function [name, blockPath, values] = readDatasetElement(element, index)
name = "Signal" + index;
blockPath = "";
values = element;
try
    if isprop(element, "Name") && strlength(string(element.Name)) > 0
        name = string(element.Name);
    end
catch
end
try
    if isprop(element, "BlockPath")
        blockPath = string(element.BlockPath.getBlock(1));
    end
catch
end
try
    values = element.Values;
catch
end
end

function [time, data] = readSignalValues(values)
time = zeros(0, 1);
data = values;
if isa(values, "timeseries")
    time = normalizeTime(values.Time);
    data = values.Data;
elseif isa(values, "timetable")
    time = normalizeTime(values.Properties.RowTimes);
    data = table2array(values);
elseif isstruct(values) && isfield(values, "time") && isfield(values, "signals")
    time = normalizeTime(values.time);
    if isfield(values.signals, "values")
        data = values.signals.values;
    else
        data = values.signals;
    end
end
end

function time = readSimulationTime(simulationOutput)
time = zeros(0, 1);
try
    if any(string(simulationOutput.who) == "tout")
        time = simulationOutput.get("tout");
    end
catch
end
time = normalizeTime(time);
end

function time = normalizeTime(time)
if isempty(time)
    time = zeros(0, 1);
    return
end
if isdatetime(time)
    time = seconds(time - time(1));
elseif isduration(time)
    time = seconds(time);
end
time = double(time(:));
end

function registerSignal(signalMap, name, blockPath, time, data)
if strlength(string(name)) == 0
    return
end
entry = struct("Time", normalizeTime(time), "Data", data, ...
    "Source", string(blockPath) + "/" + string(name));
names = unique([string(name), string(blockPath) + "/" + string(name)], "stable");
for candidate = names
    key = canonicalKey(candidate);
    if strlength(key) > 0 && ~isKey(signalMap, key)
        signalMap(key) = entry;
    end
end
end

function registerStructSignals(signalMap, name, blockPath, time, data)
if ~isstruct(data)
    return
end
fields = fieldnames(data);
for fieldIndex = 1:numel(fields)
    field = string(fields{fieldIndex});
    fieldData = extractStructField(data, field);
    fieldName = string(name) + "." + field;
    registerSignal(signalMap, fieldName, blockPath, time, fieldData);
    registerSignal(signalMap, fieldName, string(blockPath) + "/" + string(name), time, fieldData);
    if isstruct(fieldData)
        registerStructSignals(signalMap, fieldName, blockPath, time, fieldData);
    end
end
end

function fieldData = extractStructField(data, field)
try
    if isscalar(data)
        fieldData = data.(char(field));
    else
        samples = {data.(char(field))};
        firstSample = samples{1};
        if isnumeric(firstSample) && isscalar(firstSample)
            fieldData = vertcat(samples{:});
        elseif isnumeric(firstSample) && isvector(firstSample)
            fieldData = zeros(numel(samples), numel(firstSample), "like", firstSample);
            for sampleIndex = 1:numel(samples)
                fieldData(sampleIndex, :) = reshape(samples{sampleIndex}, 1, []);
            end
        elseif isstruct(firstSample)
            fieldData = [samples{:}];
        else
            fieldData = samples;
        end
    end
    if isa(fieldData, "timeseries")
        fieldData = fieldData.Data;
    elseif isa(fieldData, "timetable")
        fieldData = table2array(fieldData);
    end
catch
    fieldData = [];
end
end

function key = canonicalKey(name)
key = lower(string(name));
key = replace(key, "\\", "/");
key = replace(key, " ", "");
key = char(key);
end

function [data, signalTime, sourceName] = findSignal(signalMap, candidates)
data = [];
signalTime = zeros(0, 1);
sourceName = "";
keysInMap = string(keys(signalMap));
for candidate = string(candidates)
    candidateKey = string(canonicalKey(candidate));
    index = find(keysInMap == candidateKey, 1);
    if isempty(index)
        index = find(endsWith(keysInMap, "/" + candidateKey) | ...
            endsWith(keysInMap, "." + candidateKey), 1);
    end
    if isempty(index)
        continue
    end
    entry = signalMap(char(keysInMap(index)));
    if isempty(entry.Data) || isstruct(entry.Data)
        continue
    end
    data = entry.Data;
    signalTime = entry.Time;
    sourceName = entry.Source;
    return
end
end

function tf = sameTimeVector(first, second)
first = normalizeTime(first);
second = normalizeTime(second);
tf = isempty(first) || isempty(second) || ...
    (numel(first) == numel(second) && all(abs(first - second) <= 1e-10));
end

function data = resampleSignal(data, sourceTime, targetTime)
if isempty(sourceTime) || isempty(targetTime) || ~isnumeric(data)
    return
end
sourceTime = normalizeTime(sourceTime);
if numel(sourceTime) < 2 || any(diff(sourceTime) <= 0)
    return
end
data = normalizeSignalShape(data, numel(sourceTime));
if isvector(data)
    data = interp1(sourceTime, data(:), targetTime, "linear", "extrap");
else
    output = zeros(numel(targetTime), size(data, 2));
    for column = 1:size(data, 2)
        output(:, column) = interp1(sourceTime, data(:, column), ...
            targetTime, "linear", "extrap");
    end
    data = output;
end
end

function data = normalizeSignalShape(data, sampleCount)
if ~isnumeric(data) || isempty(data) || sampleCount == 0
    return
end
if ~ismatrix(data)
    dataSize = size(data);
    if dataSize(end) == sampleCount
        data = reshape(data, [], sampleCount).';
    elseif dataSize(1) == sampleCount
        data = reshape(data, sampleCount, []);
    elseif dataSize(2) == sampleCount
        data = reshape(data, [], sampleCount).';
    end
end
if isvector(data) && numel(data) == sampleCount
    data = data(:);
elseif size(data, 1) ~= sampleCount && size(data, 2) == sampleCount
    data = data.';
end
end

function result = setResultPath(result, path, value)
parts = split(string(path), ".");
result.(char(parts(1))).(char(parts(2))) = value;
end

function issue = validateTime(time)
issue = strings(0, 1);
if isempty(time)
    issue = "Time vector is missing.";
elseif any(~isfinite(time))
    issue = "Time vector contains NaN or Inf.";
elseif any(diff(time) <= 0)
    issue = "Time vector is not strictly increasing.";
end
end

function meta = makeMeta(runInfo, datasetName, signalMap, missingSignals, ...
    nonFiniteSignals, resampledSignals, timeIssue)
meta = struct;
meta.ScenarioName = getRunInfo(runInfo, "ScenarioName", "Unspecified");
meta.ParameterSet = getRunInfo(runInfo, "ParameterSet", "Unknown");
meta.ModelVariant = getRunInfo(runInfo, "ModelVariant", "Unknown");
meta.Solver = getRunInfo(runInfo, "Solver", "Unknown");
meta.Dataset = datasetName;
meta.MATLABVersion = version;
meta.MissingSignals = unique(missingSignals, "stable");
meta.NonFiniteSignals = unique(nonFiniteSignals, "stable");
meta.ResampledSignals = unique(resampledSignals, "stable");
meta.TimeIssue = timeIssue;
meta.Complete = isempty(meta.MissingSignals);
meta.Valid = isempty(timeIssue) && isempty(meta.NonFiniteSignals) && meta.Complete;
meta.SignalInventory = string(keys(signalMap));
end

function value = getRunInfo(runInfo, field, defaultValue)
if isfield(runInfo, field) && ~isempty(runInfo.(field))
    value = runInfo.(field);
else
    value = defaultValue;
end
end

function metrics = deriveMetrics(result)
metrics = struct;
metrics.MaxLongitudinalAcceleration = maxAbs(result.Vehicle.Ax);
metrics.MaxLateralAcceleration = maxAbs(result.Vehicle.Ay);
metrics.MaxMuUtilization = maxValue(result.Tire.MuUtilization);
metrics.PowerConstraintActiveTime = 0.0;
metrics.SOCChange = deltaValue(result.Battery.SOC);
metrics.NetBatteryEnergy = integrateSignal(result.Time, result.Battery.AllowedElectricalPower);
end

function value = maxAbs(data)
if isempty(data)
    value = NaN;
else
    value = max(abs(data(:)), [], "omitnan");
end
end

function value = maxValue(data)
if isempty(data)
    value = NaN;
else
    value = max(data(:), [], "omitnan");
end
end

function value = deltaValue(data)
if isempty(data)
    value = NaN;
else
    value = data(end) - data(1);
end
end

function value = integrateSignal(time, data)
if isempty(time) || isempty(data) || numel(time) ~= numel(data)
    value = NaN;
else
    value = trapz(time, data(:));
end
end
