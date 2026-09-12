function [result, filePath] = loadLapSimulationResult(filePath)
%LOADLAPSIMULATIONRESULT 加载并校验规范化圈速 MAT 结果。
%   [RESULT, FILEPATH] = LOADLAPSIMULATIONRESULT(FILEPATH) 读取变量名固定为
%   result 的 MAT 文件，校验 SchemaVersion、严格递增 Time、时序长度以及
%   四轮 Nx4 维度。角度应为 rad，速度为 m/s，转速为 rad/s；四轮顺序为
%   [FL, FR, RL, RR]。该函数不启动 Simulink。
%
%   结果版本不兼容、Time 无效或时序维度错误时直接报错，并给出期望值和
%   实际值。

arguments
    filePath (1, 1) string
end

assert(isfile(filePath), "FSAE:Lap:ResultFileMissing", ...
    "结果文件不存在：%s。", filePath);
loaded = load(filePath, "result");
if ~isfield(loaded, "result") || ~isstruct(loaded.result)
    error("FSAE:Lap:ResultVariableMissing", ...
        "结果文件 '%s' 中缺少名为 result 的结构变量。", filePath);
end
result = loaded.result;
expectedVersion = "1.0";
actualVersion = string(getValue(result, "SchemaVersion", ""));
if actualVersion ~= expectedVersion
    error("FSAE:Lap:ResultVersion", ...
        "结果版本不兼容：期望 %s，实际 %s。", expectedVersion, actualVersion);
end
if ~isfield(result, "Time") || isempty(result.Time)
    error("FSAE:Lap:ResultTimeMissing", "结果中缺少非空 result.Time。");
end

time = double(result.Time(:));
if any(~isfinite(time)) || any(diff(time) <= 0)
    error("FSAE:Lap:ResultTimeInvalid", ...
        "result.Time 必须是有限且严格递增的秒制向量。");
end
sampleCount = numel(time);
validateTimeSeriesFields(result, sampleCount);
if ~isfield(result, "Meta") || ~isstruct(result.Meta)
    result.Meta = struct;
end
result.Meta.LoadedFile = filePath;
result.Meta.OutputFolder = string(fileparts(filePath));
result.Meta.ResultFile = filePath;
end

function validateTimeSeriesFields(result, sampleCount)
groups = ["Track", "Vehicle", "Wheel", "Tire", "Powertrain", ...
    "Battery", "Driver", "Actuator", "Controller", "Sensor"];
fourWheelGroups = ["Wheel", "Tire", "Actuator"];
for groupName = groups
    if ~isfield(result, char(groupName))
        error("FSAE:Lap:ResultGroupMissing", ...
            "结果缺少字段 result.%s。", groupName);
    end
    group = result.(char(groupName));
    fields = fieldnames(group);
    for fieldIndex = 1:numel(fields)
        value = group.(fields{fieldIndex});
        if isempty(value)
            continue
        end
        if ~(isnumeric(value) || islogical(value))
            continue
        end
        if size(value, 1) ~= sampleCount
            error("FSAE:Lap:ResultLength", ...
                "result.%s.%s 第一维为 %d，期望 %d（Time 长度）。", ...
                groupName, fields{fieldIndex}, size(value, 1), sampleCount);
        end
        if any(groupName == fourWheelGroups) && isFourWheelField(groupName, fields{fieldIndex})
            if size(value, 2) ~= 4
                error("FSAE:Lap:ResultWheelDimension", ...
                    "result.%s.%s 尺寸为 %s，期望 N×4；四轮顺序为 [FL, FR, RL, RR]。", ...
                    groupName, fields{fieldIndex}, mat2str(size(value)));
            end
        end
    end
end
if ~isfield(result, "Distance") || isempty(result.Distance)
    error("FSAE:Lap:ResultDistanceMissing", ...
        "结果缺少非空 result.Distance。");
end
if numel(result.Distance) ~= sampleCount
    error("FSAE:Lap:ResultDistanceLength", ...
        "result.Distance 长度为 %d，期望 %d。", numel(result.Distance), sampleCount);
end
end

function tf = isFourWheelField(groupName, fieldName)
tf = any(groupName == ["Wheel", "Tire"]) || ...
    any(string(fieldName) == ["MotorTorqueRequest", ...
    "FrictionBrakeTorqueRequest", "RegenTorqueRequest"]);
end

function value = getValue(data, fieldName, defaultValue)
if isfield(data, fieldName)
    value = data.(fieldName);
else
    value = defaultValue;
end
end
