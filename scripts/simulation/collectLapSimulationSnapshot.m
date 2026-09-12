function snapshot = collectLapSimulationSnapshot(simulationOutput)
%COLLECTLAPSIMULATIONSNAPSHOT 读取终点检查和进度显示所需的最新仿真样本。
%   SNAPSHOT = COLLECTLAPSIMULATIONSNAPSHOT(SIMULATIONOUTPUT) 只访问 yout 中
%   VehicleState 和 TrackReference 的末样本，不归一化或重采样完整历史。
%   该函数用于仿真暂停点，避免随仿真数据增长反复处理全部输出。
%
%   输出单位：时间 s、位置 m、航向 rad、速度 m/s、路径里程 m。

arguments
    simulationOutput (1, 1) Simulink.SimulationOutput
end

snapshot = struct;
snapshot.Time = readLastSimulationTime(simulationOutput);
snapshot.Distance = [];
snapshot.Vehicle = struct( ...
    "X", [], "Y", [], "Psi", [], "Ux", [], "Uy", [], "Speed", []);
snapshot.Track = struct( ...
    "PathS", [], "EventDistance", [], "LapIndex", [], "Progress", []);

outputNames = string(simulationOutput.who);
if ~any(outputNames == "yout")
    return
end
yout = simulationOutput.get("yout");
if ~isa(yout, "Simulink.SimulationData.Dataset")
    return
end

sampleTimes = zeros(0, 1);
if yout.numElements >= 1
    vehicleState = yout{1}.Values;
    for fieldName = ["X", "Y", "Psi", "Ux", "Uy"]
        [value, sampleTime] = readLastScalarField(vehicleState, fieldName);
        snapshot.Vehicle.(char(fieldName)) = value;
        if isfinite(sampleTime)
            sampleTimes(end + 1, 1) = sampleTime; %#ok<AGROW>
        end
    end
    if ~isempty(snapshot.Vehicle.Ux) && ~isempty(snapshot.Vehicle.Uy)
        snapshot.Vehicle.Speed = hypot( ...
            snapshot.Vehicle.Ux, snapshot.Vehicle.Uy);
    end
end

if yout.numElements >= 4
    trackReference = yout{4}.Values;
    [snapshot.Track.PathS, sampleTime] = ...
        readLastScalarField(trackReference, "PathS");
    if isfinite(sampleTime)
        sampleTimes(end + 1, 1) = sampleTime;
    end
end

if isempty(snapshot.Time) && ~isempty(sampleTimes)
    snapshot.Time = max(sampleTimes);
end
end

function time = readLastSimulationTime(simulationOutput)
time = [];
outputNames = string(simulationOutput.who);
if ~any(outputNames == "tout")
    return
end
candidate = simulationOutput.get("tout");
if isa(candidate, "timeseries")
    candidate = candidate.Time;
elseif isa(candidate, "duration")
    candidate = seconds(candidate);
end
candidate = double(candidate(:));
candidate = candidate(isfinite(candidate));
if ~isempty(candidate)
    time = candidate(end);
end
end

function [value, time] = readLastScalarField(group, fieldName)
value = [];
time = NaN;
if ~isstruct(group) || ~isfield(group, char(fieldName))
    return
end
candidate = group.(char(fieldName));
if isa(candidate, "timeseries")
    if isempty(candidate.Time)
        return
    end
    sample = getdatasamples(candidate, numel(candidate.Time));
    time = double(candidate.Time(end));
elseif isa(candidate, "timetable")
    if height(candidate) == 0
        return
    end
    sample = table2array(candidate(end, :));
    rowTime = candidate.Properties.RowTimes(end);
    if isa(rowTime, "duration")
        time = seconds(rowTime);
    else
        time = double(rowTime);
    end
elseif isnumeric(candidate) || islogical(candidate)
    if isempty(candidate)
        return
    end
    sample = candidate(end);
else
    return
end
sample = squeeze(sample);
if isscalar(sample) && (isnumeric(sample) || islogical(sample))
    value = double(sample);
end
end
