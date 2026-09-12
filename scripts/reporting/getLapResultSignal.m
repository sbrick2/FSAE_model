function [data, info] = getLapResultSignal(result, parameter, plotCfg)
%GETLAPRESULTSIGNAL 提取圈速结果字段并执行显示层单位转换。
%   [DATA, INFO] = GETLAPRESULTSIGNAL(RESULT, PARAMETER, PLOT CFG) 提取
%   标量或 Nx4 时序字段，按 plotCfg.Wheel.Channels 选择四轮通道，并把
%   m/s、rad、rad/s 按显示配置转换为 km/h、deg、rpm。结果内部单位不变。
%   四轮列顺序固定为 [FL, FR, RL, RR]。
%
%   支持的横轴包括 Time、Distance、Track.EventDistance、Track.PathS、
%   Track.Progress、Vehicle.X 和 Vehicle.Y；纵轴可为结果中任意可绘制的
%   数值字段。不存在的字段会打印相近字段和完整字段清单。

arguments
    result (1, 1) struct
    parameter (1, 1) string
    plotCfg (1, 1) struct = struct
end

available = listLapResultFields(result);
match = find(strcmpi(available, parameter), 1);
if isempty(match)
    similar = available(contains(lower(available), lower(parameter)) | ...
        contains(lower(parameter), lower(available)));
    if isempty(similar)
        similar = available(1:min(10, numel(available)));
    end
    error("FSAE:Lap:UnknownResultField", ...
        "结果字段 '%s' 不存在。相近字段：%s。全部可用字段：%s。", ...
        parameter, strjoin(similar, ", "), strjoin(available, ", "));
end
parameter = available(match);
data = readResultPath(result, parameter);
info = makeSignalInfo(parameter);

if isempty(data)
    return
end
data = double(data);
if size(data, 2) == 4
    info.IsWheel = true;
end
if info.IsWheel && size(data, 2) == 4
    channels = getWheelChannels(plotCfg);
    channelIndex = arrayfun(@(name) find(["FL", "FR", "RL", "RR"] == name, 1), channels);
    data = data(:, channelIndex);
    info.ChannelNames = channels;
else
    info.ChannelNames = "标量";
end

[data, info] = convertDisplayUnits(data, info, plotCfg);
end

function value = readResultPath(result, path)
if any(path == ["Time", "Distance"])
    value = result.(char(path));
    return
end
parts = split(path, ".");
if numel(parts) ~= 2 || ~isfield(result, char(parts(1))) || ...
        ~isfield(result.(char(parts(1))), char(parts(2)))
    value = [];
else
    value = result.(char(parts(1))).(char(parts(2)));
end
end

function info = makeSignalInfo(path)
info = struct( ...
    "Path", path, ...
    "DisplayName", path, ...
    "Unit", "1", ...
    "SourceUnit", "1", ...
    "IsWheel", false, ...
    "ChannelNames", "标量");
lowerPath = lower(path);
info.DisplayName = chineseSignalName(path);
if path == "Time"
    info.Unit = "s";
    info.SourceUnit = "s";
elseif any(path == ["Distance", "Vehicle.X", "Vehicle.Y", ...
        "Track.EventDistance", "Track.PathS", "Track.ReportedPathS"])
    info.Unit = "m";
    info.SourceUnit = "m";
elseif any(path == ["Track.LateralError", "Track.BoundaryViolation", ...
        "Track.VehicleEnvelopeClearance", ...
        "Track.VehicleEnvelopeViolation", ...
        "Sensor.PositionX", "Sensor.PositionY", ...
        "Wheel.SuspensionDeflection"])
    info.Unit = "m";
    info.SourceUnit = "m";
elseif path == "Track.Curvature"
    info.Unit = "1/m";
    info.SourceUnit = "1/m";
elseif endsWith(lowerPath, "active") || endsWith(lowerPath, "valid") || ...
        endsWith(lowerPath, "saturated")
    info.Unit = "1";
    info.SourceUnit = "1";
elseif any(path == ["Vehicle.Ux", "Vehicle.Uy", "Vehicle.Speed"])
    info.Unit = "m/s";
    info.SourceUnit = "m/s";
elseif path == "Wheel.RPM" || path == "Powertrain.MotorRPM"
    info.Unit = "rpm";
    info.SourceUnit = "rpm";
elseif any(path == ["Wheel.Speed", "Powertrain.MotorSpeed", ...
        "Sensor.MotorSpeed", "Sensor.WheelSpeed"])
    info.Unit = "rad/s";
    info.SourceUnit = "rad/s";
elseif contains(lowerPath, "speed") && ...
        ~contains(lowerPath, "motorlimit") && ~contains(lowerPath, "power")
    info.Unit = "m/s";
    info.SourceUnit = "m/s";
elseif any(contains(lowerPath, ["angle", "steer", "heading", "psi"])) || ...
        contains(lowerPath, "slipangle") || contains(lowerPath, "camber")
    info.Unit = "rad";
    info.SourceUnit = "rad";
elseif path == "Vehicle.YawRate" || path == "Sensor.YawRate" || ...
        path == "Controller.ReferenceYawRate" || path == "Controller.YawRateError"
    info.Unit = "rad/s";
    info.SourceUnit = "rad/s";
elseif contains(lowerPath, "acceleration") || ...
        any(path == ["Vehicle.Ax", "Vehicle.Ay", "Vehicle.Az", "Sensor.AccelX", "Sensor.AccelY"])
    info.Unit = "m/s^2";
    info.SourceUnit = "m/s^2";
elseif contains(lowerPath, "torque")
    info.Unit = "N*m";
    info.SourceUnit = "N*m";
elseif any(path == ["Controller.DesiredYawMoment", ...
        "Controller.AllocatedYawMoment"])
    info.Unit = "N*m";
    info.SourceUnit = "N*m";
elseif contains(lowerPath, "force") || ...
        any(path == ["Tire.FxWheel", "Tire.FyWheel"])
    info.Unit = "N";
    info.SourceUnit = "N";
elseif path == "Wheel.DamperVelocity"
    info.Unit = "m/s";
    info.SourceUnit = "m/s";
elseif contains(lowerPath, "pressure")
    info.Unit = "Pa";
    info.SourceUnit = "Pa";
elseif contains(lowerPath, "energy")
    info.Unit = "J";
    info.SourceUnit = "J";
elseif contains(lowerPath, "power")
    info.Unit = "W";
    info.SourceUnit = "W";
elseif contains(lowerPath, "voltage")
    info.Unit = "V";
    info.SourceUnit = "V";
elseif contains(lowerPath, "current")
    info.Unit = "A";
    info.SourceUnit = "A";
elseif contains(lowerPath, "load")
    info.Unit = "N";
    info.SourceUnit = "N";
end
if any(startsWith(path, ["Wheel.", "Tire."])) || ...
        any(path == ["Actuator.MotorTorqueRequest", ...
        "Actuator.FrictionBrakeTorqueRequest", "Actuator.RegenTorqueRequest"]) || ...
        any(path == ["Powertrain.MotorTorqueActual", ...
        "Powertrain.MotorSpeed", "Powertrain.MotorRPM", ...
        "Powertrain.MotorMechanicalPower", "Powertrain.MotorLimitActive"])
    info.IsWheel = true;
elseif startsWith(path, "Powertrain.")
    info.IsWheel = false;
end
if any(path == ["Track.Progress", "Track.LapIndex"])
    info.Unit = "1";
    info.SourceUnit = "1";
end
end

function channels = getWheelChannels(plotCfg)
channels = ["FL", "FR", "RL", "RR"];
if isfield(plotCfg, "Wheel") && isfield(plotCfg.Wheel, "Channels")
    channels = string(plotCfg.Wheel.Channels);
end
validChannels = ["FL", "FR", "RL", "RR"];
if isempty(channels) || any(~ismember(channels, validChannels)) || ...
        numel(unique(channels)) ~= numel(channels)
    error("FSAE:Lap:InvalidWheelChannels", ...
        "Wheel.Channels 收到 %s，允许值为 FL、FR、RL、RR 且不得重复。", ...
        strjoin(channels, ", "));
end
end

function [data, info] = convertDisplayUnits(data, info, plotCfg)
if info.SourceUnit == "m/s"
    speedUnit = getNestedString(plotCfg, "Units", "Speed", "m/s");
    if speedUnit == "km/h"
        data = data * 3.6;
        info.Unit = "km/h";
    elseif speedUnit ~= "m/s"
        error("FSAE:Lap:InvalidSpeedUnit", ...
            "Units.Speed 收到 '%s'，允许值为 m/s 或 km/h。", speedUnit);
    end
elseif info.SourceUnit == "rad"
    angleUnit = getNestedString(plotCfg, "Units", "Angle", "deg");
    if angleUnit == "deg"
        data = data * 180 / pi;
        info.Unit = "deg";
    elseif angleUnit == "rad"
        info.Unit = "rad";
    else
        error("FSAE:Lap:InvalidAngleUnit", ...
            "Units.Angle 收到 '%s'，允许值为 rad 或 deg。", angleUnit);
    end
elseif info.SourceUnit == "rad/s" && ...
        any(info.Path == ["Wheel.Speed", "Powertrain.MotorSpeed", "Sensor.MotorSpeed"])
    rotationalUnit = getNestedString(plotCfg, "Units", "RotationalSpeed", "rpm");
    if rotationalUnit == "rpm"
        data = data * 60 / (2 * pi);
        info.Unit = "rpm";
    elseif rotationalUnit == "rad/s"
        info.Unit = "rad/s";
    else
        error("FSAE:Lap:InvalidRotationalUnit", ...
            "Units.RotationalSpeed 收到 '%s'，允许值为 rad/s 或 rpm。", rotationalUnit);
    end
end
end

function value = getNestedString(data, groupName, fieldName, defaultValue)
value = string(defaultValue);
if isfield(data, groupName) && isfield(data.(groupName), fieldName)
    value = string(data.(groupName).(fieldName));
end
end

function name = chineseSignalName(path)
name = path;
switch path
    case "Time"
        name = "仿真时间";
    case "Distance"
        name = "实际累计距离";
    case "Track.EventDistance"
        name = "赛项中心线累计距离";
    case "Track.PathS"
        name = "单圈中心线弧长";
    case "Track.Progress"
        name = "赛道进度";
    case "Track.LapIndex"
        name = "圈序号";
    case "Track.ReferenceSpeed"
        name = "参考速度";
    case "Track.Curvature"
        name = "赛道曲率";
    case "Track.LateralError"
        name = "横向跟踪误差";
    case "Track.BoundaryViolation"
        name = "质心越界量";
    case "Track.ReportedPathS"
        name = "原始报告弧长";
    case "Track.VehicleEnvelopeClearance"
        name = "车辆包络边界净空";
    case "Track.VehicleEnvelopeViolation"
        name = "车辆包络越界量";
    case "Vehicle.Speed"
        name = "车速";
    case "Vehicle.X"
        name = "车辆全局 X";
    case "Vehicle.Y"
        name = "车辆全局 Y";
    case "Vehicle.Psi"
        name = "车辆航向角";
    case "Vehicle.Ux"
        name = "车身纵向速度";
    case "Vehicle.Uy"
        name = "车身横向速度";
    case "Vehicle.Ax"
        name = "车身纵向加速度";
    case "Vehicle.Ay"
        name = "车身横向加速度";
    case "Vehicle.YawRate"
        name = "横摆角速度";
    case "Vehicle.RollAngle"
        name = "车身侧倾角";
    case "Vehicle.PitchAngle"
        name = "车身俯仰角";
    case "Vehicle.Az"
        name = "车身垂向加速度";
    case "Wheel.NormalLoad"
        name = "四轮法向载荷";
    case "Wheel.SuspensionDeflection"
        name = "四轮悬架行程";
    case "Wheel.DamperVelocity"
        name = "四轮减振器速度";
    case "Wheel.SuspensionForce"
        name = "四轮悬架力";
    case "Wheel.Speed"
        name = "四轮轮速";
    case "Wheel.RPM"
        name = "四轮轮速";
    case "Wheel.SteerAngle"
        name = "四轮转角";
    case "Wheel.SlipRatio"
        name = "四轮滑移率";
    case "Wheel.SlipAngle"
        name = "四轮侧偏角";
    case "Wheel.CamberAngle"
        name = "四轮外倾角";
    case "Tire.FxWheel"
        name = "四轮纵向轮胎力";
    case "Tire.FyWheel"
        name = "四轮横向轮胎力";
    case "Tire.MuUtilization"
        name = "四轮摩擦利用率";
    case "Powertrain.MotorTorqueActual"
        name = "电机实际转矩";
    case "Powertrain.MotorSpeed"
        name = "电机转速";
    case "Powertrain.MotorRPM"
        name = "电机转速";
    case "Powertrain.MotorMechanicalPower"
        name = "四电机机械功率";
    case "Powertrain.MotorLimitActive"
        name = "四电机限制状态";
    case "Powertrain.TotalPowerLimitActive"
        name = "总功率限制状态";
    case "Powertrain.RegenLimitActive"
        name = "再生限制状态";
    case "Battery.Voltage"
        name = "电池电压";
    case "Battery.Current"
        name = "电池电流";
    case "Battery.Power"
        name = "电池功率";
    case "Battery.SOC"
        name = "电池荷电状态";
    case "Driver.SteeringWheelAngleRequest"
        name = "方向盘转角请求";
    case "Driver.SteeringRackAngleRequest"
        name = "驾驶员转向齿条角请求";
    case "Driver.LongitudinalAccelerationRequest"
        name = "纵向加速度请求";
    case "Driver.DriveTorqueRequest"
        name = "总驱动转矩请求";
    case "Driver.BrakePressureRequest"
        name = "制动压力请求";
    case "Actuator.SteeringRackAngleRequest"
        name = "执行器转向齿条角请求";
    case "Actuator.MotorTorqueRequest"
        name = "四电机转矩请求";
    case "Actuator.FrictionBrakeTorqueRequest"
        name = "四轮摩擦制动转矩请求";
    case "Actuator.RegenTorqueRequest"
        name = "四轮再生转矩请求";
    case "Actuator.TotalPowerRequest"
        name = "总功率请求";
    case "Controller.ReferenceYawRate"
        name = "参考横摆角速度";
    case "Controller.YawRateError"
        name = "横摆角速度误差";
    case "Controller.DesiredLongitudinalForce"
        name = "期望纵向合力";
    case "Controller.DesiredYawMoment"
        name = "期望横摆力矩";
    case "Controller.AllocatedYawMoment"
        name = "分配横摆力矩";
    case "Controller.TVActive"
        name = "扭矩矢量控制状态";
    case "Controller.TCActive"
        name = "牵引力控制状态";
    case "Controller.RegenActive"
        name = "再生制动状态";
    case "Controller.EnergyManagementActive"
        name = "能量管理状态";
    case "Controller.ControllerSaturated"
        name = "控制器饱和状态";
    case "Sensor.PositionX"
        name = "传感器位置 X";
    case "Sensor.PositionY"
        name = "传感器位置 Y";
    case "Sensor.Heading"
        name = "传感器航向角";
    case "Sensor.LongitudinalSpeed"
        name = "纵向速度测量";
    case "Sensor.LateralSpeed"
        name = "横向速度测量";
    case "Sensor.YawRate"
        name = "横摆角速度测量";
    case "Sensor.AccelX"
        name = "纵向加速度测量";
    case "Sensor.AccelY"
        name = "横向加速度测量";
    case "Sensor.WheelSpeed"
        name = "四轮轮速测量";
    case "Sensor.SteeringRackAngle"
        name = "转向齿条角测量";
    case "Sensor.MotorSpeed"
        name = "四电机转速测量";
    case "Sensor.MotorTorqueEstimate"
        name = "四电机转矩估计";
    case "Sensor.BatteryVoltage"
        name = "电池电压测量";
    case "Sensor.BatteryCurrent"
        name = "电池电流测量";
    case "Sensor.PoseValid"
        name = "位姿有效状态";
    case "Sensor.IMUValid"
        name = "IMU 有效状态";
    case "Sensor.WheelSpeedValid"
        name = "四轮轮速有效状态";
    case "Sensor.PowertrainValid"
        name = "动力系统测量有效状态";
end
end
