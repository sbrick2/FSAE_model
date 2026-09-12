function catalog = deriveRacecarAnalysisSignals(result, resultType)
%DERIVERACECARANALYSISSIGNALS 生成通用分析所需的可靠派生时序。
%   只返回能够由当前结果直接计算且与原始样本一一对应的数值信号。

arguments
    result (1, 1) struct
    resultType (1, 1) string {mustBeMember(resultType, ...
        ["time_domain", "quasi_static"])}
end

items = emptyItems();
if resultType == "time_domain"
    items = deriveTimeDomainSignals(items, result);
else
    items = deriveQuasiStaticSignals(items, result);
end
catalog = itemsToTable(items);
end

function items = deriveTimeDomainSignals(items, result)
time = readPath(result, "Time");
distance = readPath(result, "Distance");
eventDistance = readPath(result, "Track.EventDistance");
referenceSpeed = readPath(result, "Track.ReferenceSpeed");
speed = readPath(result, "Vehicle.Speed");
ux = readPath(result, "Vehicle.Ux");
uy = readPath(result, "Vehicle.Uy");
ax = readPath(result, "Vehicle.Ax");
ay = readPath(result, "Vehicle.Ay");
az = readPath(result, "Vehicle.Az");
curvature = readPath(result, "Track.Curvature");

items = addDifference(items, "Derived.SpeedError", "实际与参考速度误差", ...
    "m/s", speed, referenceSpeed);
items = addDifference(items, "Derived.SpeedMargin", "参考速度余量", ...
    "m/s", referenceSpeed, speed);
items = addRatio(items, "Derived.SpeedUtilization", "参考速度利用率", ...
    speed, referenceSpeed);
items = addHypot(items, "Derived.PlanarAcceleration", "平面合加速度", ...
    "m/s^2", ax, ay);
items = addScaled(items, "Derived.LongitudinalG", "纵向加速度 G 值", ...
    "g", ax, 1 / standardGravity());
items = addScaled(items, "Derived.LateralG", "横向加速度 G 值", ...
    "g", ay, 1 / standardGravity());
items = addScaled(items, "Derived.VerticalG", "垂向加速度 G 值", ...
    "g", az, 1 / standardGravity());
if ~isempty(ax) && ~isempty(ay)
    items = addItem(items, "Derived.PlanarAccelerationG", ...
        "平面合加速度 G 值", "g", hypot(ax, ay) / standardGravity());
end
if ~isempty(ux) && ~isempty(uy)
    items = addItem(items, "Derived.VehicleSideslipAngle", ...
        "车身侧偏角", "rad", atan2(uy, ux));
end
items = addDerivative(items, "Derived.LongitudinalJerk", ...
    "纵向冲击度", "m/s^3", ax, time);
items = addDerivative(items, "Derived.LateralJerk", ...
    "横向冲击度", "m/s^3", ay, time);
if ~isempty(ax) && ~isempty(ay) && ~isempty(time)
    jerkX = differentiateSignal(ax, time);
    jerkY = differentiateSignal(ay, time);
    items = addItem(items, "Derived.PlanarJerk", "平面合冲击度", ...
        "m/s^3", hypot(jerkX, jerkY));
end
items = addDerivative(items, "Derived.YawAcceleration", ...
    "横摆角加速度", "rad/s^2", readPath(result, "Vehicle.YawRate"), time);
items = addDerivative(items, "Derived.RollRate", ...
    "侧倾角速度", "rad/s", readPath(result, "Vehicle.RollAngle"), time);
items = addDerivative(items, "Derived.PitchRate", ...
    "俯仰角速度", "rad/s", readPath(result, "Vehicle.PitchAngle"), time);
items = addAbsolute(items, "Derived.AbsoluteLateralError", ...
    "绝对横向误差", "m", readPath(result, "Track.LateralError"));
items = addRadius(items, curvature);
items = addTurnDirection(items, curvature);
if ~isempty(eventDistance)
    items = addItem(items, "Derived.DistanceRemaining", "结果终点剩余距离", ...
        "m", max(eventDistance, [], "omitnan") - eventDistance);
end

normalLoad = readPath(result, "Wheel.NormalLoad");
if size(normalLoad, 2) == 4
    frontLoad = sum(normalLoad(:, 1:2), 2);
    rearLoad = sum(normalLoad(:, 3:4), 2);
    totalLoad = frontLoad + rearLoad;
    items = addItem(items, "Derived.FrontAxleNormalLoad", ...
        "前轴总法向载荷", "N", frontLoad);
    items = addItem(items, "Derived.RearAxleNormalLoad", ...
        "后轴总法向载荷", "N", rearLoad);
    items = addItem(items, "Derived.TotalNormalLoad", ...
        "整车总法向载荷", "N", totalLoad);
    items = addItem(items, "Derived.FrontLoadFraction", ...
        "前轴载荷比例", "1", safeRatio(frontLoad, totalLoad));
    items = addItem(items, "Derived.LeftSideNormalLoad", ...
        "左侧总法向载荷", "N", normalLoad(:, 1) + normalLoad(:, 3));
    items = addItem(items, "Derived.RightSideNormalLoad", ...
        "右侧总法向载荷", "N", normalLoad(:, 2) + normalLoad(:, 4));
    items = addItem(items, "Derived.FrontLateralLoadTransfer", ...
        "前轴横向载荷转移", "N", normalLoad(:, 2) - normalLoad(:, 1));
    items = addItem(items, "Derived.RearLateralLoadTransfer", ...
        "后轴横向载荷转移", "N", normalLoad(:, 4) - normalLoad(:, 3));
end

wheelSpeed = readPath(result, "Wheel.Speed");
if size(wheelSpeed, 2) == 4
    items = addItem(items, "Derived.AverageWheelSpeed", ...
        "四轮平均角速度", "rad/s", mean(wheelSpeed, 2));
    items = addItem(items, "Derived.FrontWheelSpeedDifference", ...
        "前轴左右轮速差", "rad/s", wheelSpeed(:, 2) - wheelSpeed(:, 1));
    items = addItem(items, "Derived.RearWheelSpeedDifference", ...
        "后轴左右轮速差", "rad/s", wheelSpeed(:, 4) - wheelSpeed(:, 3));
end

fx = readPath(result, "Tire.FxWheel");
fy = readPath(result, "Tire.FyWheel");
if size(fx, 2) == 4 && size(fy, 2) == 4
    items = addItem(items, "Derived.TireForceMagnitude", ...
        "四轮轮胎合力", "N", hypot(fx, fy));
    items = addItem(items, "Derived.TotalTireFx", ...
        "整车纵向轮胎合力", "N", sum(fx, 2));
    items = addItem(items, "Derived.TotalTireFy", ...
        "整车横向轮胎合力", "N", sum(fy, 2));
    if size(normalLoad, 2) == 4
        items = addItem(items, "Derived.TireMuXDemand", ...
            "四轮纵向附着需求", "1", safeRatio(fx, normalLoad));
        items = addItem(items, "Derived.TireMuYDemand", ...
            "四轮横向附着需求", "1", safeRatio(fy, normalLoad));
        items = addItem(items, "Derived.TireMuDemand", ...
            "四轮合附着需求", "1", safeRatio(hypot(fx, fy), normalLoad));
    end
end

motorTorque = readPath(result, "Powertrain.MotorTorqueActual");
motorPower = readPath(result, "Powertrain.MotorMechanicalPower");
items = addFourChannelSums(items, motorTorque, "MotorTorque", ...
    "电机实际转矩", "N*m");
items = addFourChannelSums(items, motorPower, "MotorPower", ...
    "电机机械功率", "W");

batteryPower = readPath(result, "Battery.Power");
if ~isempty(time) && ~isempty(batteryPower)
    items = addItem(items, "Derived.CumulativeDischargeEnergy", ...
        "累计放电能量", "J", cumtrapz(time, max(batteryPower, 0)));
    items = addItem(items, "Derived.CumulativeRegenEnergy", ...
        "累计回收能量", "J", cumtrapz(time, max(-batteryPower, 0)));
    netEnergy = cumtrapz(time, batteryPower);
    items = addItem(items, "Derived.NetBatteryEnergy", ...
        "电池累计净能量", "J", netEnergy);
    if ~isempty(distance)
        items = addItem(items, "Derived.EnergyPerDistance", ...
            "单位距离净能耗", "J/m", safeRatio(netEnergy, distance));
    end
end
items = addDerivative(items, "Derived.SOCChangeRate", ...
    "SOC 变化率", "1/s", readPath(result, "Battery.SOC"), time);

items = addDifference(items, "Derived.SteeringTrackingError", ...
    "转向执行跟踪误差", "rad", ...
    readPath(result, "Actuator.SteeringRackAngleRequest"), ...
    readPath(result, "Driver.SteeringRackAngleRequest"));
items = addDifference(items, "Derived.YawMomentAllocationError", ...
    "横摆力矩分配误差", "N*m", ...
    readPath(result, "Controller.AllocatedYawMoment"), ...
    readPath(result, "Controller.DesiredYawMoment"));
items = addDifference(items, "Derived.MotorTorqueTrackingError", ...
    "四电机转矩跟踪误差", "N*m", motorTorque, ...
    readPath(result, "Actuator.MotorTorqueRequest"));
items = addDifference(items, "Derived.SensorLongitudinalSpeedError", ...
    "纵向速度测量误差", "m/s", ...
    readPath(result, "Sensor.LongitudinalSpeed"), ux);
items = addDifference(items, "Derived.SensorLateralSpeedError", ...
    "横向速度测量误差", "m/s", ...
    readPath(result, "Sensor.LateralSpeed"), uy);
items = addDifference(items, "Derived.SensorYawRateError", ...
    "横摆角速度测量误差", "rad/s", ...
    readPath(result, "Sensor.YawRate"), readPath(result, "Vehicle.YawRate"));
items = addDifference(items, "Derived.SensorAccelXError", ...
    "纵向加速度测量误差", "m/s^2", ...
    readPath(result, "Sensor.AccelX"), ax);
items = addDifference(items, "Derived.SensorAccelYError", ...
    "横向加速度测量误差", "m/s^2", ...
    readPath(result, "Sensor.AccelY"), ay);
heading = readPath(result, "Sensor.Heading");
psi = readPath(result, "Vehicle.Psi");
if ~isempty(heading) && ~isempty(psi)
    items = addItem(items, "Derived.SensorHeadingError", ...
        "航向角测量误差", "rad", wrapAngle(heading - psi));
end
end

function items = deriveQuasiStaticSignals(items, result)
speed = readPath(result, "SpeedProfile.Speed");
time = readPath(result, "SpeedProfile.Time");
distance = readPath(result, "SpeedProfile.ProgressS");
curvature = readPath(result, "SpeedProfile.Curvature");
ay = readPath(result, "SpeedProfile.LateralAcceleration");
curvatureLimit = readPath(result, "SpeedProfile.CurvatureSpeedLimit");
referenceSpeed = readPath(result, "Track.ReferenceSpeed");

if ~isempty(distance)
    span = max(distance, [], "omitnan") - min(distance, [], "omitnan");
    items = addItem(items, "Derived.NormalizedProgress", ...
        "归一化赛道进度", "1", (distance - min(distance, [], "omitnan")) / max(span, eps));
    items = addItem(items, "Derived.DistanceRemaining", ...
        "剩余赛道距离", "m", max(distance, [], "omitnan") - distance);
end
if ~isempty(time)
    items = addItem(items, "Derived.SegmentTime", ...
        "相邻采样点用时", "s", [0; diff(time)]);
end
ax = differentiateSignal(speed, time);
items = addItem(items, "Derived.LongitudinalAcceleration", ...
    "纵向加速度", "m/s^2", ax);
items = addHypot(items, "Derived.PlanarAcceleration", "平面合加速度", ...
    "m/s^2", ax, ay);
items = addScaled(items, "Derived.LongitudinalG", "纵向加速度 G 值", ...
    "g", ax, 1 / standardGravity());
items = addScaled(items, "Derived.LateralG", "横向加速度 G 值", ...
    "g", ay, 1 / standardGravity());
if ~isempty(ax) && ~isempty(ay)
    items = addItem(items, "Derived.PlanarAccelerationG", ...
        "平面合加速度 G 值", "g", hypot(ax, ay) / standardGravity());
end
if ~isempty(speed) && ~isempty(curvature)
    items = addItem(items, "Derived.YawRate", ...
        "横摆角速度", "rad/s", speed .* curvature);
end
items = addRadius(items, curvature);
items = addTurnDirection(items, curvature);
items = addDifference(items, "Derived.CurvatureSpeedMargin", ...
    "曲率限速余量", "m/s", curvatureLimit, speed);
items = addRatio(items, "Derived.CurvatureSpeedUtilization", ...
    "曲率限速利用率", speed, curvatureLimit);
items = addDifference(items, "Derived.ReferenceSpeedError", ...
    "实际与参考速度误差", "m/s", speed, referenceSpeed);
items = addDifference(items, "Derived.ReferenceSpeedMargin", ...
    "参考速度余量", "m/s", referenceSpeed, speed);
items = addRatio(items, "Derived.ReferenceSpeedUtilization", ...
    "参考速度利用率", speed, referenceSpeed);
items = addDerivative(items, "Derived.SpeedGradient", ...
    "速度空间梯度", "1/s", speed, distance);
items = addDerivative(items, "Derived.LongitudinalJerk", ...
    "纵向冲击度", "m/s^3", ax, time);
items = addDerivative(items, "Derived.LateralJerk", ...
    "横向冲击度", "m/s^3", ay, time);
if ~isempty(ax) && ~isempty(ay) && ~isempty(time)
    items = addItem(items, "Derived.PlanarJerk", "平面合冲击度", ...
        "m/s^3", hypot(differentiateSignal(ax, time), ...
        differentiateSignal(ay, time)));
end
leftWidth = readPath(result, "Track.LeftHalfWidth");
rightWidth = readPath(result, "Track.RightHalfWidth");
items = addSum(items, "Derived.TrackWidth", "赛道总宽度", ...
    "m", leftWidth, rightWidth);
if ~isempty(ax)
    phase = zeros(size(ax));
    phase(ax > 0.05) = 1;
    phase(ax < -0.05) = -1;
    items = addItem(items, "Derived.LongitudinalPhase", ...
        "纵向工况（制动=-1/滑行=0/加速=1）", "1", phase);
end

items = addQuasiGGVSignals(items, result, speed, ay, ax);
end

function items = addQuasiGGVSignals(items, result, speed, ay, ax)
if isempty(speed) || isempty(ay) || ~isfield(result, "GGV")
    return
end
ggv = result.GGV;
required = ["Speed", "LateralFraction", "AyPositive", "AyNegative", ...
    "AxMax", "AxMin"];
if ~all(isfield(ggv, cellstr(required)))
    return
end
ggvSpeed = double(ggv.Speed(:));
positiveLimit = interp1(ggvSpeed, double(ggv.AyPositive(:)), ...
    speed, "linear", "extrap");
negativeLimit = interp1(ggvSpeed, double(ggv.AyNegative(:)), ...
    speed, "linear", "extrap");
lateralLimit = positiveLimit;
lateralLimit(ay < 0) = negativeLimit(ay < 0);
fraction = max(-1, min(1, safeRatio(ay, lateralLimit)));
querySpeed = max(min(speed, max(ggvSpeed)), min(ggvSpeed));
axMax = interp2(double(ggv.LateralFraction(:).'), ggvSpeed, ...
    double(ggv.AxMax), fraction, querySpeed, "linear");
axMin = interp2(double(ggv.LateralFraction(:).'), ggvSpeed, ...
    double(ggv.AxMin), fraction, querySpeed, "linear");
items = addItem(items, "Derived.GGVPositiveLateralLimit", ...
    "GGV 正向横向加速度极限", "m/s^2", positiveLimit);
items = addItem(items, "Derived.GGVNegativeLateralLimit", ...
    "GGV 负向横向加速度极限", "m/s^2", -negativeLimit);
items = addItem(items, "Derived.GGVAccelerationLimit", ...
    "GGV 最大纵向加速度", "m/s^2", axMax);
items = addItem(items, "Derived.GGVBrakingLimit", ...
    "GGV 最大制动减速度", "m/s^2", axMin);
lateralUtilization = abs(safeRatio(ay, lateralLimit));
items = addItem(items, "Derived.GGVLateralUtilization", ...
    "GGV 横向能力利用率", "1", lateralUtilization);
if ~isempty(ax)
    longitudinalBound = axMax;
    longitudinalBound(ax < 0) = abs(axMin(ax < 0));
    longitudinalDemand = ax;
    longitudinalDemand(ax < 0) = abs(ax(ax < 0));
    longitudinalUtilization = safeRatio(longitudinalDemand, longitudinalBound);
    items = addItem(items, "Derived.GGVLongitudinalUtilization", ...
        "GGV 纵向能力利用率", "1", longitudinalUtilization);
    items = addItem(items, "Derived.GGVCombinedUtilization", ...
        "GGV 综合能力利用率（估算）", "1", ...
        max(lateralUtilization, longitudinalUtilization));
    items = addItem(items, "Derived.GGVLongitudinalMargin", ...
        "GGV 纵向加速度裕量", "m/s^2", ...
        longitudinalBound - longitudinalDemand);
end
end

function items = addFourChannelSums(items, data, pathStem, nameStem, unit)
if size(data, 2) ~= 4
    return
end
items = addItem(items, "Derived.Total" + pathStem, ...
    "四轮" + nameStem + "合计", unit, sum(data, 2));
items = addItem(items, "Derived.FrontAxle" + pathStem, ...
    "前轴" + nameStem + "合计", unit, sum(data(:, 1:2), 2));
items = addItem(items, "Derived.RearAxle" + pathStem, ...
    "后轴" + nameStem + "合计", unit, sum(data(:, 3:4), 2));
end

function items = addDifference(items, path, name, unit, first, second)
if ~isempty(first) && ~isempty(second) && isequal(size(first), size(second))
    items = addItem(items, path, name, unit, first - second);
end
end

function items = addSum(items, path, name, unit, first, second)
if ~isempty(first) && ~isempty(second) && isequal(size(first), size(second))
    items = addItem(items, path, name, unit, first + second);
end
end

function items = addRatio(items, path, name, numerator, denominator)
if ~isempty(numerator) && ~isempty(denominator) && ...
        isequal(size(numerator), size(denominator))
    items = addItem(items, path, name, "1", safeRatio(numerator, denominator));
end
end

function items = addHypot(items, path, name, unit, first, second)
if ~isempty(first) && ~isempty(second) && isequal(size(first), size(second))
    items = addItem(items, path, name, unit, hypot(first, second));
end
end

function items = addScaled(items, path, name, unit, data, scale)
if ~isempty(data)
    items = addItem(items, path, name, unit, data * scale);
end
end

function items = addAbsolute(items, path, name, unit, data)
if ~isempty(data)
    items = addItem(items, path, name, unit, abs(data));
end
end

function items = addDerivative(items, path, name, unit, data, axisData)
derivative = differentiateSignal(data, axisData);
if ~isempty(derivative)
    items = addItem(items, path, name, unit, derivative);
end
end

function items = addRadius(items, curvature)
if isempty(curvature)
    return
end
radius = NaN(size(curvature));
curved = abs(curvature) > 1e-9;
radius(curved) = 1 ./ abs(curvature(curved));
items = addItem(items, "Derived.TurnRadius", "转弯半径", "m", radius);
end

function items = addTurnDirection(items, curvature)
if isempty(curvature)
    return
end
direction = sign(curvature);
direction(abs(curvature) <= 1e-9) = 0;
items = addItem(items, "Derived.TurnDirection", ...
    "弯道方向（右=-1/直=0/左=1）", "1", direction);
end

function derivative = differentiateSignal(data, axisData)
derivative = [];
if isempty(data) || isempty(axisData) || size(data, 1) ~= numel(axisData) || ...
        numel(axisData) < 2 || any(~isfinite(axisData)) || ...
        any(diff(double(axisData(:))) <= 0)
    return
end
axisData = double(axisData(:));
data = double(data);
derivative = NaN(size(data));
for channel = 1:size(data, 2)
    derivative(:, channel) = gradient(data(:, channel), axisData);
end
end

function ratio = safeRatio(numerator, denominator)
numerator = double(numerator);
denominator = double(denominator);
ratio = NaN(size(numerator));
valid = isfinite(numerator) & isfinite(denominator) & ...
    abs(denominator) > 1e-12;
ratio(valid) = numerator(valid) ./ denominator(valid);
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle), cos(angle));
end

function value = standardGravity()
value = 9.80665;
end

function value = readPath(result, path)
parts = split(string(path), ".");
value = result;
for index = 1:numel(parts)
    if ~isstruct(value) || ~isscalar(value) || ...
            ~isfield(value, char(parts(index)))
        value = [];
        return
    end
    value = value.(char(parts(index)));
end
if ~(isnumeric(value) || islogical(value)) || isempty(value) || ...
        ~ismatrix(value)
    value = [];
    return
end
value = double(value);
if isvector(value)
    value = value(:);
end
end

function items = addItem(items, path, displayName, unit, data)
if isempty(data) || ~(isnumeric(data) || islogical(data)) || ...
        ~ismatrix(data) || isscalar(data)
    return
end
data = double(data);
if isvector(data)
    data = data(:);
end
channels = "标量";
if size(data, 2) == 4
    channels = ["FL"; "FR"; "RL"; "RR"];
end
items(end + 1) = struct( ...
    "Path", string(path), "DisplayName", string(displayName), ...
    "Unit", string(unit), "Data", data, "ChannelNames", channels);
end

function items = emptyItems()
items = struct("Path", {}, "DisplayName", {}, "Unit", {}, ...
    "Data", {}, "ChannelNames", {});
end

function catalog = itemsToTable(items)
if isempty(items)
    catalog = table(strings(0, 1), strings(0, 1), strings(0, 1), ...
        strings(0, 1), cell(0, 1), cell(0, 1), ...
        VariableNames = ["Label", "Path", "DisplayName", "Unit", ...
        "Data", "ChannelNames"]);
    return
end
paths = string({items.Path}).';
displayNames = string({items.DisplayName}).';
units = string({items.Unit}).';
labels = displayNames + " | " + paths;
hasUnit = strlength(units) > 0;
labels(hasUnit) = labels(hasUnit) + " (" + units(hasUnit) + ")";
data = {items.Data}.';
channelNames = {items.ChannelNames}.';
catalog = table(labels, paths, displayNames, units, data, channelNames, ...
    VariableNames = ["Label", "Path", "DisplayName", "Unit", ...
    "Data", "ChannelNames"]);
end
