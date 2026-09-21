function result = collectLapSimulationResults(simulationOutput, scenario, cfg)
%COLLECTLAPSIMULATIONRESULTS 整理圈速仿真的七组顶层输出。
%   RESULT = COLLECTLAPSIMULATIONRESULTS(SIMULATIONOUTPUT, SCENARIO, CFG)
%   从 yout 和 logsout 中读取 VehicleState、PowertrainState、Sensor、
%   TrackReference、ControllerDebug、DriverCommand、ActuatorCommand，统一
%   到可绘图的结果结构。所有时间序列第一维长度等于 numel(result.Time)，
%   四轮量为 Nx4，列顺序固定为 [FL, FR, RL, RR]。
%
%   角度内部保存为 rad；速度为 m/s；转速为 rad/s；距离为 m；加速度为
%   m/s^2。不能从输出中获得的字段保持 []，并在 result.Meta.MissingSignals
%   中报告，不用零值伪装。
arguments
    simulationOutput (1, 1) Simulink.SimulationOutput
    scenario (1, 1) struct
    cfg (1, 1) struct = struct
end

[signalMap, topLevelOutputs, datasetNames] = readLapOutputSignals(simulationOutput);
time = readSimulationTime(simulationOutput);
if isempty(time)
    keysInMap = keys(signalMap);
    if ~isempty(keysInMap)
        time = signalMap(keysInMap{1}).Time;
    end
end
time = normalizeTime(time);

result = initializeLapResult(time, cfg);
mapping = lapSignalMapping();
dimensionIssues = strings(0, 1);
sources = struct;
for index = 1:size(mapping, 1)
    resultPath = string(mapping{index, 1});
    [data, signalTime, sourceName] = findLapSignal(signalMap, mapping{index, 2});
    if isempty(data)
        continue
    end
    if isempty(result.Time)
        result.Time = normalizeTime(signalTime);
    elseif ~sameTimeVector(result.Time, signalTime)
        data = resampleLapSignal(data, signalTime, result.Time);
    end
    [data, issue] = normalizeLapSignalShape(data, numel(result.Time));
    if strlength(issue) > 0
        dimensionIssues(end + 1, 1) = resultPath + ": " + issue; %#ok<AGROW>
        continue
    end
    result = setLapResultPath(result, resultPath, data);
    sources.(matlab.lang.makeValidName(char(resultPath))) = sourceName;
end

% 由真值位置派生实际累计距离；没有位置时保持为空。
result.Distance = cumulativeDistance(result.Vehicle.X, result.Vehicle.Y);

% 派生字段只使用已有有效信号，不把缺失字段补成零。
result = deriveLapVehicleSignals(result);
result = deriveLapWheelSignals(result);
result = deriveLapPowertrainSignals(result);
result = deriveLapTrackSignals(result, scenario);
result = truncateLapSignalsAtFinish(result, scenario);
result.Metrics = deriveLapMetrics(result, scenario);
eventMetric = calculateLapSimulationEventMetric(result, scenario);
result.Metrics.LapTime = eventMetric.Time;
result.Metrics.EventTiming = eventMetric;

missingSignals = findMissingLapFields(result);
nonFiniteSignals = findNonFiniteLapFields(result);
timeIssue = validateLapTime(result.Time);
missingTopOutputs = setdiff( ...
    ["VehicleState", "PowertrainState", "Sensor", "TrackReference", ...
    "ControllerDebug", "DriverCommand", "ActuatorCommand"], ...
    topLevelOutputs, "stable");
if ~isempty(missingTopOutputs)
    missingSignals = [missingSignals; ...
        "TopLevelOutput." + missingTopOutputs(:)];
end

result.Meta = struct( ...
    "SchemaVersion", result.SchemaVersion, ...
    "ScenarioID", getStructValue(scenario, "ID", ""), ...
    "Event", getStructValue(scenario, "Event", ""), ...
    "DatasetNames", datasetNames, ...
    "TopLevelOutputs", topLevelOutputs, ...
    "MissingTopLevelOutputs", missingTopOutputs, ...
    "MissingSignals", unique(missingSignals, "stable"), ...
    "NonFiniteSignals", unique(nonFiniteSignals, "stable"), ...
    "DimensionIssues", unique(dimensionIssues, "stable"), ...
    "SignalSources", sources, ...
    "MATLABVersion", version, ...
    "Valid", isempty(timeIssue) && isempty(nonFiniteSignals) && ...
    isempty(dimensionIssues) && isempty(missingTopOutputs), ...
    "TimeIssue", timeIssue, ...
    "Notes", "车辆、气动和部分动力系统参数仍可能包含占位值；结果不是实车性能结论。");
end

function result = initializeLapResult(time, cfg)
result = struct;
result.SchemaVersion = "1.0";
result.Time = reshape(time, [], 1);
result.Distance = [];
result.Config = cfg;
result.Track = emptyFields(["PathS", "EventDistance", "LapIndex", ...
    "Progress", "ReferenceSpeed", "Curvature", "LateralError", ...
    "BoundaryViolation"]);
result.Vehicle = emptyFields(["X", "Y", "Psi", "Ux", "Uy", "Speed", ...
    "YawRate", "RollAngle", "PitchAngle", "Ax", "Ay", "Az"]);
result.Wheel = emptyFields(["Speed", "RPM", "SteerAngle", "NormalLoad", ...
    "SlipRatio", "SlipAngle", "CamberAngle"]);
result.Tire = emptyFields(["FxWheel", "FyWheel", "MuUtilization"]);
result.Powertrain = emptyFields(["MotorTorqueActual", "MotorSpeed", ...
    "MotorRPM", "MotorMechanicalPower", "MotorLimitActive", ...
    "TotalPowerLimitActive", "RegenLimitActive"]);
result.Battery = emptyFields(["Voltage", "Current", "Power", "SOC"]);
result.Driver = emptyFields(["SteeringWheelAngleRequest", ...
    "SteeringRackAngleRequest", "LongitudinalAccelerationRequest", ...
    "DriveTorqueRequest", "BrakePressureRequest"]);
result.Actuator = emptyFields(["SteeringRackAngleRequest", ...
    "MotorTorqueRequest", "FrictionBrakeTorqueRequest", ...
    "RegenTorqueRequest", "TotalPowerRequest"]);
result.Controller = emptyFields(["ReferenceYawRate", "YawRateError", ...
    "DesiredLongitudinalForce", "DesiredYawMoment", "AllocatedYawMoment", ...
    "TVActive", "TCActive", "RegenActive", "EnergyManagementActive", ...
    "ControllerSaturated"]);
result.Sensor = emptyFields(["PositionX", "PositionY", "Heading", ...
    "LongitudinalSpeed", "LateralSpeed", "YawRate", "AccelX", "AccelY", ...
    "WheelSpeed", "SteeringRackAngle", "MotorSpeed", ...
    "MotorTorqueEstimate", "BatteryVoltage", "BatteryCurrent", ...
    "PoseValid", "IMUValid", "WheelSpeedValid", "PowertrainValid"]);
result.Metrics = struct;
end

function values = emptyFields(names)
values = struct;
for name = reshape(string(names), 1, [])
    values.(char(name)) = [];
end
end

function mapping = lapSignalMapping()
% 四轮信号候选名覆盖顶层 Bus 名称、日志名称和 OpenLoopPlant 参考名称。
mapping = {
    "Vehicle.X", {"VehicleState.X", "Vehicle.X", "Vehicle7DOF.X", "X"};
    "Vehicle.Y", {"VehicleState.Y", "Vehicle.Y", "Vehicle7DOF.Y", "Y"};
    "Vehicle.Psi", {"VehicleState.Psi", "Vehicle.Psi", "Vehicle7DOF.Psi", "Psi"};
    "Vehicle.Ux", {"VehicleState.Ux", "Vehicle.Ux", "Vehicle7DOF.Ux", "Ux"};
    "Vehicle.Uy", {"VehicleState.Uy", "Vehicle.Uy", "Vehicle7DOF.Uy", "Uy"};
    "Vehicle.YawRate", {"VehicleState.YawRate", "Vehicle.YawRate", "YawRate"};
    "Vehicle.RollAngle", {"VehicleState.RollAngle", "Vehicle.RollAngle", "RollAngle"};
    "Vehicle.PitchAngle", {"VehicleState.PitchAngle", "Vehicle.PitchAngle", "PitchAngle"};
    "Vehicle.Ax", {"VehicleState.Ax", "Vehicle.Ax", "Vehicle7DOF.Ax", "Ax"};
    "Vehicle.Ay", {"VehicleState.Ay", "Vehicle.Ay", "Vehicle7DOF.Ay", "Ay"};
    "Vehicle.Az", {"VehicleState.Az", "Vehicle.Az", "Az"};
    "Wheel.Speed", {"VehicleState.WheelSpeed", "WheelState.WheelSpeed", "WheelSpeed", "Wheel.Speed"};
    "Wheel.SteerAngle", {"VehicleState.WheelSteerAngle", "WheelState.WheelSteerAngle", "WheelSteerAngle", "Wheel.SteerAngle"};
    "Wheel.NormalLoad", {"VehicleState.NormalLoad", "WheelState.NormalLoad", "NormalLoad", "Wheel.NormalLoad"};
    "Wheel.SlipRatio", {"VehicleState.SlipRatio", "WheelState.SlipRatio", "SlipRatio", "Wheel.SlipRatio"};
    "Wheel.SlipAngle", {"VehicleState.SlipAngle", "WheelState.SlipAngle", "SlipAngle", "Wheel.SlipAngle"};
    "Wheel.CamberAngle", {"VehicleState.CamberAngle", "WheelState.CamberAngle", "CamberAngle", "Wheel.CamberAngle"};
    "Tire.FxWheel", {"VehicleState.TireFx", "VehicleState.FxWheel", "WheelState.TireFx", "Tire.FxWheel", "TireFx"};
    "Tire.FyWheel", {"VehicleState.TireFy", "VehicleState.FyWheel", "WheelState.TireFy", "Tire.FyWheel", "TireFy"};
    "Tire.MuUtilization", {"VehicleState.MuUtilization", "WheelState.MuUtilization", "Tire.MuUtilization", "MuUtilization"};
    "Powertrain.MotorTorqueActual", {"PowertrainState.MotorTorqueActual", "MotorTorqueActual", "Powertrain.MotorTorqueActual"};
    "Powertrain.MotorSpeed", {"PowertrainState.MotorSpeed", "MotorSpeed", "Powertrain.MotorSpeed"};
    "Powertrain.MotorMechanicalPower", {"PowertrainState.MotorMechanicalPower", "MotorMechanicalPower", "Powertrain.MotorMechanicalPower"};
    "Powertrain.MotorLimitActive", {"PowertrainState.MotorLimitActive", "MotorLimitActive", "Powertrain.MotorLimitActive"};
    "Powertrain.TotalPowerLimitActive", {"PowertrainState.TotalPowerLimitActive", "TotalPowerLimitActive"};
    "Powertrain.RegenLimitActive", {"PowertrainState.RegenLimitActive", "RegenLimitActive"};
    "Battery.Voltage", {"PowertrainState.BatteryVoltage", "Battery.Voltage", "BatteryVoltage"};
    "Battery.Current", {"PowertrainState.BatteryCurrent", "Battery.Current", "BatteryCurrent"};
    "Battery.Power", {"PowertrainState.BatteryPower", "Battery.Power", "BatteryPower"};
    "Battery.SOC", {"PowertrainState.BatterySOC", "Battery.SOC", "BatterySOC", "SOC"};
    "Driver.SteeringWheelAngleRequest", {"DriverCommand.SteeringWheelAngleRequest", "SteeringWheelAngleRequest"};
    "Driver.SteeringRackAngleRequest", {"DriverCommand.SteeringRackAngleRequest", "SteeringRackAngleRequest"};
    "Driver.LongitudinalAccelerationRequest", {"DriverCommand.LongitudinalAccelerationRequest", "LongitudinalAccelerationRequest"};
    "Driver.DriveTorqueRequest", {"DriverCommand.DriveTorqueRequest", "DriveTorqueRequest"};
    "Driver.BrakePressureRequest", {"DriverCommand.BrakePressureRequest", "BrakePressureRequest"};
    "Actuator.SteeringRackAngleRequest", {"ActuatorCommand.SteeringRackAngleRequest", "Actuator.SteeringRackAngleRequest"};
    "Actuator.MotorTorqueRequest", {"ActuatorCommand.MotorTorqueRequest", "Actuator.MotorTorqueRequest"};
    "Actuator.FrictionBrakeTorqueRequest", {"ActuatorCommand.FrictionBrakeTorqueRequest", "FrictionBrakeTorqueRequest"};
    "Actuator.RegenTorqueRequest", {"ActuatorCommand.RegenTorqueRequest", "RegenTorqueRequest"};
    "Actuator.TotalPowerRequest", {"ActuatorCommand.TotalPowerRequest", "TotalPowerRequest"};
    "Controller.ReferenceYawRate", {"ControllerDebug.ReferenceYawRate", "Controller.ReferenceYawRate"};
    "Controller.YawRateError", {"ControllerDebug.YawRateError", "Controller.YawRateError"};
    "Controller.DesiredLongitudinalForce", {"ControllerDebug.DesiredLongitudinalForce", "Controller.DesiredLongitudinalForce"};
    "Controller.DesiredYawMoment", {"ControllerDebug.DesiredYawMoment", "Controller.DesiredYawMoment"};
    "Controller.AllocatedYawMoment", {"ControllerDebug.AllocatedYawMoment", "Controller.AllocatedYawMoment"};
    "Controller.TVActive", {"ControllerDebug.TVActive", "Controller.TVActive"};
    "Controller.TCActive", {"ControllerDebug.TCActive", "Controller.TCActive"};
    "Controller.RegenActive", {"ControllerDebug.RegenActive", "Controller.RegenActive"};
    "Controller.EnergyManagementActive", {"ControllerDebug.EnergyManagementActive", "Controller.EnergyManagementActive"};
    "Controller.ControllerSaturated", {"ControllerDebug.ControllerSaturated", "Controller.ControllerSaturated"};
    "Sensor.PositionX", {"Sensor.PositionX", "PositionX"};
    "Sensor.PositionY", {"Sensor.PositionY", "PositionY"};
    "Sensor.Heading", {"Sensor.Heading", "Heading"};
    "Sensor.LongitudinalSpeed", {"Sensor.LongitudinalSpeed", "LongitudinalSpeed"};
    "Sensor.LateralSpeed", {"Sensor.LateralSpeed", "LateralSpeed"};
    "Sensor.YawRate", {"Sensor.YawRate", "SensorYawRate"};
    "Sensor.AccelX", {"Sensor.AccelX", "AccelX"};
    "Sensor.AccelY", {"Sensor.AccelY", "AccelY"};
    "Sensor.WheelSpeed", {"Sensor.WheelSpeed", "SensorWheelSpeed"};
    "Sensor.SteeringRackAngle", {"Sensor.SteeringRackAngle", "SteeringRackAngle"};
    "Sensor.MotorSpeed", {"Sensor.MotorSpeed", "SensorMotorSpeed"};
    "Sensor.MotorTorqueEstimate", {"Sensor.MotorTorqueEstimate", "MotorTorqueEstimate"};
    "Sensor.BatteryVoltage", {"Sensor.BatteryVoltage", "SensorBatteryVoltage"};
    "Sensor.BatteryCurrent", {"Sensor.BatteryCurrent", "SensorBatteryCurrent"};
    "Sensor.PoseValid", {"Sensor.PoseValid", "PoseValid"};
    "Sensor.IMUValid", {"Sensor.IMUValid", "IMUValid"};
    "Sensor.WheelSpeedValid", {"Sensor.WheelSpeedValid", "WheelSpeedValid"};
    "Sensor.PowertrainValid", {"Sensor.PowertrainValid", "PowertrainValid"};
    "Track.PathS", {"TrackReference.PathS", "Track.PathS", "PathS"};
    "Track.ReferenceSpeed", {"TrackReference.ReferenceSpeed", "Track.ReferenceSpeed", "ReferenceSpeed"};
    "Track.Curvature", {"TrackReference.ReferenceCurvature", "TrackReference.Curvature", "Track.Curvature", "Curvature"};
    "Track.LateralError", {"TrackReference.LateralError", "Track.LateralError", "LateralError"};
    "Track.BoundaryViolation", {"TrackReference.BoundaryViolation", "Track.BoundaryViolation", "BoundaryViolation"};
    };
end

function result = deriveLapVehicleSignals(result)
if isempty(result.Vehicle.Speed) && ~isempty(result.Vehicle.Ux) && ~isempty(result.Vehicle.Uy)
    result.Vehicle.Speed = hypot(result.Vehicle.Ux, result.Vehicle.Uy);
end
end

function result = deriveLapWheelSignals(result)
if isempty(result.Wheel.RPM) && ~isempty(result.Wheel.Speed)
    result.Wheel.RPM = result.Wheel.Speed * 60 / (2 * pi);
end
end

function result = deriveLapPowertrainSignals(result)
if isempty(result.Powertrain.MotorRPM) && ~isempty(result.Powertrain.MotorSpeed)
    result.Powertrain.MotorRPM = result.Powertrain.MotorSpeed * 60 / (2 * pi);
end
if isempty(result.Powertrain.MotorMechanicalPower) && ...
        ~isempty(result.Powertrain.MotorTorqueActual) && ~isempty(result.Powertrain.MotorSpeed)
    result.Powertrain.MotorMechanicalPower = result.Powertrain.MotorTorqueActual .* result.Powertrain.MotorSpeed;
end
if isempty(result.Battery.Power) && ~isempty(result.Battery.Voltage) && ~isempty(result.Battery.Current)
    result.Battery.Power = result.Battery.Voltage .* result.Battery.Current;
end
end

function result = deriveLapTrackSignals(result, scenario)
track = scenario.Track;
sampleCount = numel(result.Time);
if sampleCount == 0
    return
end

projectedS = NaN(sampleCount, 1);
projectedCurvature = NaN(sampleCount, 1);
projectedError = NaN(sampleCount, 1);
boundaryViolation = NaN(sampleCount, 1);
previousIndex = NaN;
for index = 1:sampleCount
    if isempty(result.Vehicle.X) || isempty(result.Vehicle.Y)
        break
    end
    x = result.Vehicle.X(index);
    y = result.Vehicle.Y(index);
    if ~isfinite(x) || ~isfinite(y)
        continue
    end
    % 路径进度必须保持在同一分支；物理边界则应按整条赛道的最近
    % 中心线判断。Skidpad 自交点附近若两者共用局部搜索，投影可能
    % 锁到出口直线，并把仍在圆环内的车辆误报为大幅越界。
    pathProjection = projectToTrack(track, x, y, previousIndex, 250);
    boundaryProjection = projectToTrack(track, x, y);
    previousIndex = pathProjection.Index;
    projectedS(index) = pathProjection.PathS;
    projectedCurvature(index) = pathProjection.ReferenceCurvature;
    projectedError(index) = boundaryProjection.LateralError;
    if boundaryProjection.LateralError >= 0
        boundaryViolation(index) = max(0, ...
            boundaryProjection.LateralError - boundaryProjection.LeftHalfWidth);
    else
        boundaryViolation(index) = max(0, ...
            -boundaryProjection.LateralError - boundaryProjection.RightHalfWidth);
    end
end

if ~isempty(result.Track.PathS)
    result.Track.ReportedPathS = result.Track.PathS;
end
result.Track.PathS = projectedS;
if isempty(result.Track.Curvature)
    result.Track.Curvature = projectedCurvature;
else
    values = result.Track.Curvature;
    invalid = ~isfinite(values);
    values(invalid) = projectedCurvature(invalid);
    result.Track.Curvature = values;
end
if ~isempty(result.Track.LateralError)
    result.Track.ReportedLateralError = result.Track.LateralError;
end
if any(isfinite(projectedError))
    result.Track.LateralError = projectedError;
end
if ~isempty(result.Track.BoundaryViolation)
    result.Track.ReportedBoundaryViolation = result.Track.BoundaryViolation;
end
if any(isfinite(boundaryViolation))
    result.Track.BoundaryViolation = boundaryViolation;
end

pathS = result.Track.PathS;
if isempty(pathS)
    return
end
[eventDistance, lapIndex, progress] = unwrapLapPath(pathS, ...
    track.IsClosed, track.Length, scenario.NumberOfLaps);
result.Track.EventDistance = eventDistance;
result.Track.LapIndex = lapIndex;
result.Track.Progress = progress;
if isempty(result.Track.ReferenceSpeed)
    result.Track.ReferenceSpeed = lookupTrackValue(track, projectedS, "ReferenceSpeed");
end
end

function values = lookupTrackValue(track, projectedS, fieldName)
values = NaN(numel(projectedS), 1);
if ~isfield(track, fieldName)
    return
end
valid = isfinite(projectedS);
index = round(projectedS / track.SampleDistance) + 1;
index = min(numel(track.(fieldName)), max(1, index));
values(valid) = track.(fieldName)(index(valid));
end

function [eventDistance, lapIndex, progress] = unwrapLapPath(pathS, isClosed, trackLength, targetLaps)
pathS = double(pathS(:));
eventDistance = NaN(size(pathS));
valid = isfinite(pathS);
first = find(valid, 1);
if isempty(first)
    lapIndex = NaN(size(pathS));
    progress = NaN(size(pathS));
    return
end
unwrapped = NaN(size(pathS));
unwrapped(first) = pathS(first);
for index = first + 1:numel(pathS)
    if ~isfinite(pathS(index))
        continue
    end
    previous = find(isfinite(unwrapped(1:index - 1)), 1, "last");
    if isempty(previous)
        unwrapped(index) = pathS(index);
        continue
    end
    delta = pathS(index) - pathS(previous);
    if isClosed && delta < -0.5 * trackLength
        delta = delta + trackLength;
    elseif isClosed && delta > 0.5 * trackLength
        delta = delta - trackLength;
    end
    unwrapped(index) = unwrapped(previous) + delta;
end
eventDistance = unwrapped - unwrapped(first);
% 赛事进度在显示或指标时应为非负累计前向进度，使用累积最大值保证单调性，避免微小波动造成虚增里程
eventDistance = max(cummax(eventDistance), 0);
totalLength = max(trackLength * double(targetLaps), eps);
progress = min(1, eventDistance / totalLength);
lapIndex = floor(max(eventDistance, 0) / max(trackLength, eps)) + 1;
lapIndex = min(double(targetLaps), lapIndex);
end

function metrics = deriveLapMetrics(result, scenario)
metrics = struct( ...
    "LapTime", NaN, "EventTime", NaN, ...
    "MaximumAcceleration", NaN, ...
    "MaximumLongitudinalAcceleration", NaN, ...
    "MaximumLateralAcceleration", NaN, ...
    "MaximumMuUtilization", NaN, ...
    "BatteryEnergyUsed", NaN, ...
    "BoundaryViolationCount", NaN, ...
    "MaximumBoundaryViolation", NaN, ...
    "BoundaryViolationFraction", NaN, ...
    "LateralErrorRMS", NaN, ...
    "SimulationValid", false);
if isempty(result.Time)
    return
end
metrics.EventTime = result.Time(end) - result.Time(1);
if ~isempty(result.Vehicle.Ax) && ~isempty(result.Vehicle.Ay)
    acceleration = hypot(result.Vehicle.Ax, result.Vehicle.Ay);
    metrics.MaximumAcceleration = max(acceleration, [], "omitnan");
end
if ~isempty(result.Vehicle.Ax)
    metrics.MaximumLongitudinalAcceleration = max(abs(result.Vehicle.Ax), [], "omitnan");
end
if ~isempty(result.Vehicle.Ay)
    metrics.MaximumLateralAcceleration = max(abs(result.Vehicle.Ay), [], "omitnan");
end
if ~isempty(result.Tire.MuUtilization)
    metrics.MaximumMuUtilization = max(result.Tire.MuUtilization(:), [], "omitnan");
end
if ~isempty(result.Battery.Power)
    positivePower = max(double(result.Battery.Power(:)), 0);
    time = double(result.Time(:));
    if numel(time) == numel(positivePower)
        if numel(time) >= 2
            metrics.BatteryEnergyUsed = trapz(time, positivePower);
        else
            % 单个有效样本代表零积分区间，不把缺失功率伪装成零。
            metrics.BatteryEnergyUsed = 0;
        end
    end
end
if ~isempty(result.Track.BoundaryViolation)
    metrics.BoundaryViolationCount = nnz(result.Track.BoundaryViolation > 0);
    metrics.MaximumBoundaryViolation = max( ...
        result.Track.BoundaryViolation, [], "omitnan");
    metrics.BoundaryViolationFraction = mean( ...
        result.Track.BoundaryViolation > 0, "omitnan");
end
if ~isempty(result.Track.LateralError)
    metrics.LateralErrorRMS = sqrt(mean( ...
        result.Track.LateralError.^2, "omitnan"));
end

if ~isempty(result.Track.EventDistance) && scenario.Track.IsClosed
    targetDistance = scenario.Track.Length * double(scenario.NumberOfLaps);
    if result.Track.EventDistance(end) >= targetDistance
        metrics.LapTime = interpolateCrossingTime( ...
            result.Time, result.Track.EventDistance, targetDistance);
    end
elseif ~isempty(result.Track.EventDistance)
    metrics.LapTime = result.Time(end) - result.Time(1);
end
metrics.SimulationValid = true;
end

function crossingTime = interpolateCrossingTime(time, distance, target)
crossingTime = NaN;
index = find(distance >= target, 1);
if isempty(index)
    return
end
if index == 1
    crossingTime = time(1);
    return
end
fraction = (target - distance(index - 1)) / (distance(index) - distance(index - 1));
fraction = min(1, max(0, fraction));
crossingTime = time(index - 1) + fraction * (time(index) - time(index - 1));
end

function result = truncateLapSignalsAtFinish(result, scenario)
% 一次性求解可以越过终点；资格指标只统计起点到首次完赛。
if isempty(result.Time) || isempty(result.Track.EventDistance)
    return
end

sampleCount = numel(result.Time);
targetDistance = double(scenario.Track.Length);
if scenario.Track.IsClosed
    targetDistance = targetDistance * double(scenario.NumberOfLaps);
end

gateCrossingIndex = findFinishGateCrossingSample(result, scenario);
if ~isempty(gateCrossingIndex)
    finishIndex = gateCrossingIndex;
else
    finishIndex = find(double(result.Track.EventDistance(:)) >= ...
        targetDistance, 1, "first");
end
if isempty(finishIndex) || finishIndex >= sampleCount
    return
end

result.Time = result.Time(1:finishIndex, :);
timeSeriesFields = ["Distance", "Track", "Vehicle", "Wheel", "Tire", ...
    "Powertrain", "Battery", "Driver", "Actuator", "Controller", "Sensor"];
for fieldName = timeSeriesFields
    if isfield(result, fieldName)
        result.(fieldName) = truncateTimeSeriesValue( ...
            result.(fieldName), finishIndex, sampleCount);
    end
end
end

function value = truncateTimeSeriesValue(value, finishIndex, sampleCount)
if isstruct(value) && isscalar(value)
    fieldNames = string(fieldnames(value));
    for fieldName = fieldNames.'
        value.(fieldName) = truncateTimeSeriesValue( ...
            value.(fieldName), finishIndex, sampleCount);
    end
    return
end
if isempty(value) || size(value, 1) ~= sampleCount
    return
end
subscripts = repmat({':'}, 1, ndims(value));
subscripts{1} = 1:finishIndex;
value = value(subscripts{:});
end

function finishIndex = findFinishGateCrossingSample(result, scenario)
finishIndex = [];
track = scenario.Track;
requiredTrackFields = ["X", "Y", "Length", "LeftHalfWidth", "RightHalfWidth"];
if ~all(isfield(track, requiredTrackFields)) || ...
        ~isfield(result, "Vehicle") || ...
        ~isfield(result.Vehicle, "X") || ...
        ~isfield(result.Vehicle, "Y") || ...
        ~isfield(result, "Track") || ...
        ~isfield(result.Track, "EventDistance")
    return
end

positionX = double(result.Vehicle.X(:));
positionY = double(result.Vehicle.Y(:));
progress = double(result.Track.EventDistance(:));
sampleCount = min([numel(result.Time), numel(positionX), numel(positionY), numel(progress)]);
if sampleCount < 2
    return
end

positionX = positionX(1:sampleCount);
positionY = positionY(1:sampleCount);
progress = progress(1:sampleCount);

finishPoint = [double(track.X(end)), double(track.Y(end))];
finishTangent = finishPoint - [double(track.X(end - 1)), double(track.Y(end - 1))];
tangentNorm = hypot(finishTangent(1), finishTangent(2));
if tangentNorm <= eps
    return
end
finishTangent = finishTangent / tangentNorm;
finishNormal = [-finishTangent(2), finishTangent(1)];

relativePosition = [positionX, positionY] - finishPoint;
longitudinalOffset = relativePosition * finishTangent(:);
lateralOffset = relativePosition * finishNormal(:);

crossingCandidates = find(longitudinalOffset(1:end - 1) < 0.0 & ...
    longitudinalOffset(2:end) >= 0.0);
trackLength = double(track.Length);
gateHalfWidth = max([double(track.LeftHalfWidth(end)), ...
    double(track.RightHalfWidth(end))]);

for candidateIndex = reshape(crossingCandidates, 1, [])
    longitudinalSpan = longitudinalOffset(candidateIndex + 1) - ...
        longitudinalOffset(candidateIndex);
    if longitudinalSpan <= 0.0
        continue
    end
    fraction = -longitudinalOffset(candidateIndex) / longitudinalSpan;
    crossingProgress = progress(candidateIndex) + fraction * ...
        (progress(candidateIndex + 1) - progress(candidateIndex));
    crossingLateralOffset = lateralOffset(candidateIndex) + fraction * ...
        (lateralOffset(candidateIndex + 1) - lateralOffset(candidateIndex));
    if crossingProgress < 0.5 * trackLength || ...
            abs(crossingLateralOffset) > gateHalfWidth
        continue
    end
    % 包含跨过终点门线后的首个离散样本点，保证轨迹终点越过或正好位于终点线
    finishIndex = candidateIndex + 1;
    return
end
end

function distance = cumulativeDistance(x, y)
distance = [];
if isempty(x) || isempty(y) || numel(x) ~= numel(y)
    return
end
x = double(x(:));
y = double(y(:));
if any(~isfinite(x)) || any(~isfinite(y))
    distance = NaN(size(x));
    valid = isfinite(x) & isfinite(y);
    if any(valid)
        first = find(valid, 1);
        distance(1:first) = 0;
        for index = first + 1:numel(x)
            if valid(index) && valid(index - 1)
                distance(index) = distance(index - 1) + hypot(x(index) - x(index - 1), y(index) - y(index - 1));
            elseif valid(index)
                distance(index) = distance(index - 1);
            end
        end
    end
    return
end
distance = zeros(size(x));
distance(2:end) = cumsum(hypot(diff(x), diff(y)));
end

function [signalMap, topLevelOutputs, datasetNames] = readLapOutputSignals(out)
signalMap = containers.Map("KeyType", "char", "ValueType", "any");
topLevelOutputs = strings(0, 1);
datasetNames = strings(0, 1);
expectedYoutNames = ["VehicleState", "PowertrainState", "Sensor", ...
    "TrackReference", "ControllerDebug", "DriverCommand", ...
    "ActuatorCommand"];
outputNames = string(out.who);
for dataSetName = ["yout", "logsout"]
    if ~any(outputNames == dataSetName)
        continue
    end
    dataSet = out.get(char(dataSetName));
    if ~isa(dataSet, "Simulink.SimulationData.Dataset")
        continue
    end
    datasetNames(end + 1, 1) = dataSetName; %#ok<AGROW>
    for index = 1:dataSet.numElements
        element = dataSet{index};
        name = "Signal" + index;
        blockPath = "";
        if isprop(element, "Name") && strlength(string(element.Name)) > 0
            name = string(element.Name);
        end
        % 当前顶层 Outport 的 yout 元素没有 Name 属性，按模型 Outport
        % 顺序恢复冻结的七组输出名称，避免后处理依赖 Signal1 等临时名。
        if dataSetName == "yout" && name == "Signal" + index && ...
                index <= numel(expectedYoutNames)
            name = expectedYoutNames(index);
        end
        if isprop(element, "BlockPath")
            blockPath = string(element.BlockPath.getBlock(1));
        end
        topLevelOutputs(end + 1, 1) = name; %#ok<AGROW>
        values = element.Values;
        [signalTime, data] = readSignalValues(values);
        registerLapSignal(signalMap, name, blockPath, signalTime, data);
        if isstruct(data)
            registerStructSignals(signalMap, name, blockPath, signalTime, data);
        end
    end
end
topLevelOutputs = unique(topLevelOutputs, "stable");
datasetNames = unique(datasetNames, "stable");
end

function registerLapSignal(signalMap, name, blockPath, time, data)
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

function registerStructSignals(signalMap, prefix, blockPath, time, data)
fields = fieldnames(data);
for index = 1:numel(fields)
    fieldName = string(fields{index});
    fieldValue = extractStructField(data, fieldName);
    [fieldTime, fieldData] = readSignalValues(fieldValue);
    if isempty(fieldTime)
        fieldTime = time;
    end
    path = string(prefix) + "." + fieldName;
    if isstruct(fieldData)
        registerStructSignals(signalMap, path, blockPath, fieldTime, fieldData);
    else
        registerLapSignal(signalMap, path, blockPath, fieldTime, fieldData);
    end
end
end

function data = extractStructField(value, fieldName)
if isscalar(value)
    data = value.(char(fieldName));
    return
end
samples = {value.(char(fieldName))};
first = samples{1};
if (isnumeric(first) || islogical(first)) && isscalar(first) && ...
        all(cellfun(@(x) (isnumeric(x) || islogical(x)) && isscalar(x), samples))
    data = vertcat(samples{:});
elseif (isnumeric(first) || islogical(first)) && isvector(first) && ...
        all(cellfun(@(x) (isnumeric(x) || islogical(x)) && ...
        isvector(x) && numel(x) == numel(first), samples))
    data = zeros(numel(samples), numel(first), "like", first);
    for sampleIndex = 1:numel(samples)
        data(sampleIndex, :) = reshape(samples{sampleIndex}, 1, []);
    end
else
    data = samples;
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
elseif isstruct(values) && isfield(values, "time") && ...
        isfield(values, "signals")
    time = normalizeTime(values.time);
    if isfield(values.signals, "values")
        data = values.signals.values;
    else
        data = values.signals;
    end
end
if isa(data, "timeseries")
    [nestedTime, data] = readSignalValues(data);
    if isempty(time)
        time = nestedTime;
    end
elseif isa(data, "timetable")
    [nestedTime, data] = readSignalValues(data);
    if isempty(time)
        time = nestedTime;
    end
end
end

function [data, signalTime, sourceName] = findLapSignal(signalMap, candidates)
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
    if isempty(entry.Data) || isstruct(entry.Data) || iscell(entry.Data)
        continue
    end
    data = entry.Data;
    signalTime = entry.Time;
    sourceName = entry.Source;
    return
end
end

function key = canonicalKey(name)
key = lower(string(name));
key = replace(key, "\", "/");
key = replace(key, " ", "");
key = char(key);
end

function time = readSimulationTime(out)
time = zeros(0, 1);
outputNames = string(out.who);
if any(outputNames == "tout")
    time = normalizeTime(out.get("tout"));
end
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

function tf = sameTimeVector(first, second)
first = normalizeTime(first);
second = normalizeTime(second);
tf = isempty(first) || isempty(second) || ...
    (numel(first) == numel(second) && all(abs(first - second) <= 1e-10));
end

function data = resampleLapSignal(data, sourceTime, targetTime)
if isempty(sourceTime) || isempty(targetTime) || ...
        ~(isnumeric(data) || islogical(data))
    return
end
sourceTime = normalizeTime(sourceTime);
if isscalar(sourceTime)
    [data, issue] = normalizeLapSignalShape(data, 1);
    if strlength(issue) == 0
        data = repmat(data, numel(targetTime), 1);
    end
    return
end
if numel(sourceTime) < 2 || any(diff(sourceTime) <= 0)
    return
end
[data, ~] = normalizeLapSignalShape(data, numel(sourceTime));
if isempty(data)
    return
end
if size(data, 1) == 1
    data = repmat(data, numel(targetTime), 1);
else
    output = zeros(numel(targetTime), size(data, 2));
    for column = 1:size(data, 2)
        output(:, column) = interp1(sourceTime, double(data(:, column)), ...
            targetTime, "linear", "extrap");
    end
    data = output;
end
end

function [data, issue] = normalizeLapSignalShape(data, sampleCount)
issue = "";
if ~(isnumeric(data) || islogical(data)) || isempty(data)
    issue = "不是数值或逻辑数据";
    return
end
if sampleCount == 0
    issue = "结果缺少有效 Time 向量";
    return
end
if ~ismatrix(data)
    dataSize = size(data);
    if dataSize(end) == sampleCount
        data = reshape(data, [], sampleCount).';
    elseif dataSize(1) == sampleCount
        data = reshape(data, sampleCount, []);
    else
        issue = sprintf("数据尺寸 %s 与 Time 长度 %d 不一致", ...
            mat2str(dataSize), sampleCount);
        return
    end
end
if isvector(data) && numel(data) == sampleCount
    data = data(:);
elseif size(data, 1) == sampleCount
    % 已经是 N×1 或 N×4。
elseif size(data, 2) == sampleCount
    data = data.';
elseif sampleCount == 1 && numel(data) == 4
    data = reshape(data, 1, 4);
else
    issue = sprintf("数据尺寸 %s 与 Time 长度 %d 不一致", ...
        mat2str(size(data)), sampleCount);
end
end

function result = setLapResultPath(result, path, value)
parts = split(string(path), ".");
result.(char(parts(1))).(char(parts(2))) = value;
end

function values = findMissingLapFields(result)
values = strings(0, 1);
groups = ["Track", "Vehicle", "Wheel", "Tire", "Powertrain", ...
    "Battery", "Driver", "Actuator", "Controller", "Sensor"];
for groupName = groups
    fields = fieldnames(result.(char(groupName)));
    for fieldIndex = 1:numel(fields)
        value = result.(char(groupName)).(fields{fieldIndex});
        if isempty(value)
            values(end + 1, 1) = groupName + "." + fields{fieldIndex}; %#ok<AGROW>
        end
    end
end
end

function values = findNonFiniteLapFields(result)
values = strings(0, 1);
groups = ["Track", "Vehicle", "Wheel", "Tire", "Powertrain", ...
    "Battery", "Driver", "Actuator", "Controller", "Sensor"];
for groupName = groups
    fields = fieldnames(result.(char(groupName)));
    for fieldIndex = 1:numel(fields)
        value = result.(char(groupName)).(fields{fieldIndex});
        if (isnumeric(value) || islogical(value)) && ~isempty(value) && ...
                any(~isfinite(double(value(:))))
            values(end + 1, 1) = groupName + "." + fields{fieldIndex}; %#ok<AGROW>
        end
    end
end
end

function issue = validateLapTime(time)
issue = strings(0, 1);
if isempty(time)
    issue(end + 1, 1) = "Time 向量为空。";
elseif any(~isfinite(time))
    issue(end + 1, 1) = "Time 向量包含 NaN 或 Inf。";
elseif any(diff(time) <= 0)
    issue(end + 1, 1) = "Time 向量不是严格递增。";
end
end

function value = getStructValue(data, fieldName, defaultValue)
if isfield(data, fieldName)
    value = data.(fieldName);
else
    value = defaultValue;
end
end
