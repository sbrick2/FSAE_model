function figureHandle = createFinalTrackViewFigure( ...
        scenario, result, cfg, options)
%CREATEFINALTRACKVIEWFIGURE Build a labeled final lap-simulation overview.
%   The figure contains the track geometry, a speed-colored vehicle
%   trajectory, envelope violations, a legend, and a compact result summary.

arguments
    scenario (1, 1) struct
    result (1, 1) struct
    cfg (1, 1) struct
    options.Visible (1, 1) string {mustBeMember( ...
        options.Visible, ["on", "off"])} = "off"
end

figureHandle = figure( ...
    "Name", "FSAE 圈速仿真最终轨迹", ...
    "NumberTitle", "off", ...
    "Visible", options.Visible, ...
    "Color", "white", ...
    "Units", "pixels", ...
    "Position", [100, 100, 1450, 780]);
axesHandle = axes(figureHandle, ...
    "Position", [0.055, 0.15, 0.70, 0.78], ...
    "Color", "white", ...
    "XColor", [0.15, 0.15, 0.15], ...
    "YColor", [0.15, 0.15, 0.15], ...
    "GridColor", [0.78, 0.78, 0.78], ...
    "GridAlpha", 0.45, ...
    "FontName", "Microsoft YaHei", ...
    "FontSize", 10.5, ...
    "Tag", "FSAE_FinalTrackAxes");
hold(axesHandle, "on");

[leftX, leftY, rightX, rightY] = trackBoundaries(scenario.Track);
leftBoundary = plot(axesHandle, leftX, leftY, "-", ...
    "Color", [0.00, 0.00, 0.00], "LineWidth", 1.15, ...
    "DisplayName", "赛道边界", "Tag", "FSAE_LeftBoundary");
plot(axesHandle, rightX, rightY, "-", ...
    "Color", [0.00, 0.00, 0.00], "LineWidth", 1.15, ...
    "HandleVisibility", "off", "Tag", "FSAE_RightBoundary");
[centerlineX, centerlineY] = closeTrackPlotPath(scenario.Track, ...
    double(scenario.Track.X(:)), double(scenario.Track.Y(:)));
centerline = plot(axesHandle, centerlineX, centerlineY, "--", ...
    "Color", [0.55, 0.55, 0.55], "LineWidth", 1.0, ...
    "DisplayName", "赛道中心线", "Tag", "FSAE_TrackCenterline");

legendHandles = [leftBoundary, centerline];
legendNames = ["赛道边界", "赛道中心线"];
[trajectoryX, trajectoryY, trajectorySpeed, validTrajectory] = ...
    finiteTrajectory(result);
hasSpeedColorbar = false;
if ~isempty(trajectoryX)
    [~, trajectoryLegend, hasSpeedColorbar] = ...
        plotVehicleTrajectory(axesHandle, trajectoryX, trajectoryY, ...
        trajectorySpeed);
    firstValid = find(validTrajectory, 1, "first");
    lastValid = find(validTrajectory, 1, "last");
    [startX, startY, finishX, finishY, sharedMarker] = ...
        startFinishMarkerCoordinates(scenario.Track, trajectoryX, ...
        trajectoryY, firstValid, lastValid);
    startMarker = plot(axesHandle, ...
        startX, startY, "o", ...
        "MarkerSize", 7, "MarkerFaceColor", [0.00, 0.65, 0.20], ...
        "MarkerEdgeColor", [0.00, 0.35, 0.10], ...
        "DisplayName", "起点", "Tag", "FSAE_StartMarker");
    finishMarkerFaceColor = [0.85, 0.10, 0.10];
    finishMarkerSize = 7;
    if sharedMarker
        finishMarkerFaceColor = "none";
        finishMarkerSize = 10;
    end
    finishMarker = plot(axesHandle, ...
        finishX, finishY, "s", "MarkerSize", finishMarkerSize, ...
        "MarkerFaceColor", finishMarkerFaceColor, ...
        "MarkerEdgeColor", [0.45, 0.00, 0.00], ...
        "DisplayName", "终点", "Tag", "FSAE_FinishMarker");
    legendHandles = [legendHandles, trajectoryLegend, ...
        startMarker, finishMarker];
    legendNames = [legendNames, "车辆轨迹", "起点", "终点"];
end

axis(axesHandle, "equal");
grid(axesHandle, "on");
box(axesHandle, "on");
xlabel(axesHandle, "全局 X (m)");
ylabel(axesHandle, "全局 Y (m)");
boundaryStatus = boundaryStatusText(result);
title(axesHandle, "FSAE 圈速仿真最终轨迹", "FontWeight", "bold");

legendHandle = legend(axesHandle, legendHandles, legendNames, ...
    "Location", "southoutside", ...
    "Orientation", "horizontal", ...
    "NumColumns", 3, ...
    "Box", "off", ...
    "FontName", "Microsoft YaHei", ...
    "FontSize", 9);
legendHandle.Color = "white";
legendHandle.TextColor = "black";
if hasSpeedColorbar
    axesHandle.Position = [0.055, 0.15, 0.63, 0.78];
else
    axesHandle.Position = [0.055, 0.15, 0.70, 0.78];
end

[overviewText, statusColor] = simulationOverviewText( ...
    scenario, result, cfg, boundaryStatus);
annotation(figureHandle, "textbox", [0.78, 0.15, 0.205, 0.78], ...
    "String", overviewText, ...
    "Interpreter", "none", ...
    "VerticalAlignment", "top", ...
    "HorizontalAlignment", "left", ...
    "FontName", "Microsoft YaHei", ...
    "FontSize", 10.5, ...
    "Color", [0.08, 0.08, 0.08], ...
    "BackgroundColor", [0.97, 0.98, 1.00], ...
    "EdgeColor", statusColor, ...
    "LineWidth", 1.2, ...
    "Margin", 12, ...
    "FitBoxToText", "off", ...
    "Tag", "FSAE_SimulationOverview");
end

function [leftX, leftY, rightX, rightY] = trackBoundaries(track)
heading = double(track.Heading(:));
x = double(track.X(:));
y = double(track.Y(:));
leftWidth = double(track.LeftHalfWidth(:));
rightWidth = double(track.RightHalfWidth(:));
normalX = -sin(heading);
normalY = cos(heading);
leftX = x + leftWidth .* normalX;
leftY = y + leftWidth .* normalY;
rightX = x - rightWidth .* normalX;
rightY = y - rightWidth .* normalY;
[leftX, leftY] = closeTrackPlotPath(track, leftX, leftY);
[rightX, rightY] = closeTrackPlotPath(track, rightX, rightY);
end

function [x, y, speed, valid] = finiteTrajectory(result)
x = zeros(0, 1);
y = zeros(0, 1);
speed = zeros(0, 1);
valid = false(0, 1);
if ~isfield(result, "Vehicle") || ...
        ~isfield(result.Vehicle, "X") || ...
        ~isfield(result.Vehicle, "Y")
    return
end
x = double(result.Vehicle.X(:));
y = double(result.Vehicle.Y(:));
sampleCount = min(numel(x), numel(y));
x = x(1:sampleCount);
y = y(1:sampleCount);
speed = NaN(sampleCount, 1);
if isfield(result.Vehicle, "Speed") && ~isempty(result.Vehicle.Speed)
    candidate = double(result.Vehicle.Speed(:));
    speedCount = min(sampleCount, numel(candidate));
    speed(1:speedCount) = candidate(1:speedCount);
end
valid = isfinite(x) & isfinite(y);
if ~any(valid)
    x = zeros(0, 1);
    y = zeros(0, 1);
    speed = zeros(0, 1);
    valid = false(0, 1);
    return
end
x(~valid) = NaN;
y(~valid) = NaN;
end

function [startX, startY, finishX, finishY, sharedMarker] = ...
        startFinishMarkerCoordinates( ...
        track, trajectoryX, trajectoryY, firstValid, lastValid)
sharedMarker = isfield(track, "IsClosed") && ...
    isscalar(track.IsClosed) && logical(track.IsClosed);
if sharedMarker
    startX = double(track.X(1));
    startY = double(track.Y(1));
    finishX = startX;
    finishY = startY;
else
    startX = trajectoryX(firstValid);
    startY = trajectoryY(firstValid);
    finishX = trajectoryX(lastValid);
    finishY = trajectoryY(lastValid);
end
end

function [trajectory, trajectoryLegend, hasSpeedColorbar] = ...
        plotVehicleTrajectory(axesHandle, x, y, speed)
finiteSpeed = isfinite(speed);
hasSpeedColorbar = any(finiteSpeed) && numel(x) >= 2;
if ~hasSpeedColorbar
    trajectory = plot(axesHandle, x, y, ...
        "Color", [0.00, 0.45, 0.74], "LineWidth", 1.35, ...
        "DisplayName", "车辆轨迹", "Tag", "FSAE_VehicleTrajectory");
    trajectoryLegend = trajectory;
    return
end

speed = fillMissingTrajectorySpeed(speed, finiteSpeed);
trajectory = surface(axesHandle, ...
    "XData", [x.'; x.'], "YData", [y.'; y.'], ...
    "ZData", [speed.'; speed.'], ...
    "CData", [speed.'; speed.'], ...
    "FaceColor", "none", "EdgeColor", "interp", ...
    "LineWidth", 1.8, "HandleVisibility", "off", ...
    "Tag", "FSAE_VehicleTrajectory");
trajectoryLegend = plot(axesHandle, NaN, NaN, "-", ...
    "Color", [0.20, 0.55, 0.80], "LineWidth", 1.8, ...
    "DisplayName", "车辆轨迹", "Tag", "FSAE_VehicleTrajectoryLegend");
view(axesHandle, 2);
colormap(axesHandle, turbo(256));
setSpeedColorLimits(axesHandle, speed);
speedColorbar = colorbar(axesHandle);
speedColorbar.Label.String = "车速 (m/s)";
speedColorbar.Tag = "FSAE_SpeedColorbar";
speedColorbar.Position = [0.705, 0.22, 0.014, 0.64];
end

function speed = fillMissingTrajectorySpeed(speed, finiteSpeed)
validIndices = find(finiteSpeed);
if isscalar(validIndices)
    speed(:) = speed(validIndices);
    return
end
sampleIndices = (1:numel(speed)).';
speed = interp1(validIndices, speed(validIndices), sampleIndices, ...
    "linear", "extrap");
end

function setSpeedColorLimits(axesHandle, speed)
minimumSpeed = min(speed);
maximumSpeed = max(speed);
if maximumSpeed <= minimumSpeed
    margin = max(1.0, 0.05 * max(abs(minimumSpeed), 1.0));
    minimumSpeed = minimumSpeed - margin;
    maximumSpeed = maximumSpeed + margin;
end
clim(axesHandle, [minimumSpeed, maximumSpeed]);
end

function status = boundaryStatusText(result)
dataValid = logicalValue(result.Meta, "VehicleEnvelopeDataValid", false);
withinBoundary = logicalValue( ...
    result.Meta, "VehicleEnvelopeWithinBoundary", false);
if ~dataValid
    status = "未评估";
elseif withinBoundary
    status = "通过";
else
    status = "未通过";
end
end

function [textValue, statusColor] = simulationOverviewText( ...
        scenario, result, cfg, boundaryStatus)
if boundaryStatus == "通过"
    statusColor = [0.20, 0.60, 0.30];
elseif boundaryStatus == "未通过"
    statusColor = [0.82, 0.18, 0.16];
else
    statusColor = [0.55, 0.55, 0.55];
end

driverModel = stringValue(result.Meta, "SpeedControlMode", ...
    nestedString(cfg, "Driver", "Model", "unknown"));
dynamicsModel = nestedString(cfg, "Vehicle", "DynamicsModel", "unknown");
eventMetric = calculateLapSimulationEventMetric(result, scenario);
maximumSpeed = metricValue(result.Metrics, "MaximumSpeed");
if ~isfinite(maximumSpeed) && isfield(result, "Vehicle") && ...
        isfield(result.Vehicle, "Speed")
    maximumSpeed = finiteMaximum(result.Vehicle.Speed);
end
maximumLongitudinalAcceleration = metricValue( ...
    result.Metrics, "MaximumLongitudinalAcceleration");
maximumLateralAcceleration = metricValue( ...
    result.Metrics, "MaximumLateralAcceleration");
timingLines = eventTimingOverviewLines(scenario.Event, eventMetric);

lines = [ ...
    "仿真结果概况"; ...
    ""; ...
    "赛事：" + string(scenario.Event); ...
    "动力学模型：" + dynamicsModel; ...
    "驾驶员模型：" + driverModel; ...
    ""; ...
    timingLines; ...
    "最大速度：" + formatMetric(maximumSpeed, "m/s"); ...
    "最大纵向加速度：" + ...
        formatMetric(maximumLongitudinalAcceleration, "m/s^2"); ...
    "最大侧向加速度：" + ...
        formatMetric(maximumLateralAcceleration, "m/s^2")];
textValue = strjoin(lines, newline);
end

function lines = eventTimingOverviewLines(eventName, eventMetric)
if lower(string(eventName)) == "skidpad"
    lines = [ ...
        "左圈时间：" + formatMetric(eventMetric.LeftTimedLap, "s"); ...
        "右圈时间：" + formatMetric(eventMetric.RightTimedLap, "s"); ...
        "平均时间：" + formatMetric(eventMetric.Time, "s")];
else
    lines = "单圈时间：" + formatMetric(eventMetric.Time, "s");
end
end

function value = metricValue(data, fieldName)
value = NaN;
if isstruct(data) && isfield(data, fieldName)
    candidate = data.(fieldName);
    if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
        value = double(candidate);
    end
end
end

function value = finiteMaximum(data)
data = double(data(:));
data = data(isfinite(data));
if isempty(data)
    value = NaN;
else
    value = max(data);
end
end

function textValue = formatMetric(value, unit)
if ~isfinite(value)
    textValue = "—";
else
    textValue = string(sprintf("%.3f %s", value, unit));
end
end

function value = logicalValue(data, fieldName, defaultValue)
value = defaultValue;
if isstruct(data) && isfield(data, fieldName) && ...
        isscalar(data.(fieldName))
    value = logical(data.(fieldName));
end
end

function value = stringValue(data, fieldName, defaultValue)
value = string(defaultValue);
if isstruct(data) && isfield(data, fieldName) && ...
        ~isempty(data.(fieldName))
    value = string(data.(fieldName));
end
end

function value = nestedString(data, groupName, fieldName, defaultValue)
value = string(defaultValue);
if isstruct(data) && isfield(data, groupName) && ...
        isstruct(data.(groupName)) && ...
        isfield(data.(groupName), fieldName) && ...
        ~isempty(data.(groupName).(fieldName))
    value = string(data.(groupName).(fieldName));
end
end
