function fields = listLapResultFields(result, options)
%LISTLAPRESULTFIELDS 列出当前圈速结果中可绘制的数值时序字段。
%   FIELDS = LISTLAPRESULTFIELDS(RESULT) 返回字段路径字符串，如
%   Vehicle.Speed、Wheel.NormalLoad 和 Track.EventDistance。只列出第一维
%   等于 numel(result.Time) 的数值/逻辑时序；空字段和 Metrics 标量不列出。
%   四轮字段的列顺序固定为 [FL, FR, RL, RR]。
%
%   可选 Display=true（默认 false）会在命令窗口打印带维度的字段清单。

arguments
    result (1, 1) struct
    options.Display (1, 1) logical = false
end

fields = strings(0, 1);
if isfield(result, "Time") && ~isempty(result.Time)
    fields(end + 1, 1) = "Time";
end
if isfield(result, "Distance") && ~isempty(result.Distance)
    fields(end + 1, 1) = "Distance";
end
groups = ["Track", "Vehicle", "Wheel", "Suspension", "Tire", ...
    "Powertrain", "Battery", "Brake", "Aero", "Driver", ...
    "Actuator", "Controller", "Sensor", "Thermal", "Environment", ...
    "DriverDebug"];
sampleCount = numel(result.Time);
for groupName = groups
    if ~isfield(result, char(groupName))
        continue
    end
    fieldNames = fieldname(result.(char(groupName)));
    for fieldIndex = 1:numel(fieldNames)
        fieldName = string(fieldNames{fieldIndex});
        value = result.(char(groupName)).(char(fieldName));
        if isempty(value) || ~(isnumeric(value) || islogical(value))
            continue
        end
        if size(value, 1) ~= sampleCount
            continue
        end
        fields(end + 1, 1) = groupName + "." + fieldName; %#ok<AGROW>
    end
end
fields = unique(fields, "stable");

if options.Display
    fprintf("可绘制结果字段（四轮顺序 [FL, FR, RL, RR]）：\n");
    for fieldIndex = 1:numel(fields)
        field = fields(fieldIndex);
        value = readField(result, field);
        fprintf("  %-42s %s\n", field, mat2str(size(value)));
    end
end
end

function names = fieldname(value)
names = fieldnames(value);
end

function value = readField(result, path)
if any(path == ["Time", "Distance"])
    value = result.(char(path));
    return
end
parts = split(path, ".");
value = result.(char(parts(1))).(char(parts(2)));
end
