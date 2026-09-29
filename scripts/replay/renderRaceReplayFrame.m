function renderRaceReplayFrame(view, data, frame, options)
%RENDERRACEREPLAYFRAME Update shared figure handles without replotting.
arguments
    view (1, 1) struct
    data (1, 1) struct
    frame (1, 1) struct
    options.View (1, 1) string = "north"
    options.FollowSpan (1, 1) double {mustBePositive} = 22
    options.ShowValues (1, 1) logical = true
    options.ShowStats (1, 1) logical = true
end
if ~isempty(view.ScenePanel)
    view.ScenePanel.SizeChangedFcn(view.ScenePanel, []);
else
    layoutRaceReplayOverlays(view.MainAxes, view.GGAxes, view.StatsAxes, view.MapAxes, view.PedalAxes);
end
if frame.Vehicle.Valid
    geometry = transformRaceReplayGeometry(frame, data.Geometry, View = options.View);
    set(view.Body, XData = geometry.Body(:, 1), YData = geometry.Body(:, 2));
    set(view.FxArrows, XData = geometry.WheelCenters(:, 1).', ...
        YData = geometry.WheelCenters(:, 2).', ...
        UData = geometry.FxVectors(:, 1).' * data.ForceScale, ...
        VData = geometry.FxVectors(:, 2).' * data.ForceScale);
    set(view.FyArrows, XData = geometry.WheelCenters(:, 1).', ...
        YData = geometry.WheelCenters(:, 2).', ...
        UData = geometry.FyVectors(:, 1).' * data.ForceScale, ...
        VData = geometry.FyVectors(:, 2).' * data.ForceScale);
    for index = 1:4
        angle = geometry.WheelAngles(index);
        rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        w = data.Geometry.TireWidth / 2;
        r = data.Geometry.WheelLength / 2;
        wheel = [r, w; -r, w; -r, -w; r, -w] * rotation.' + ...
            geometry.WheelCenters(index, :);
        set(view.Wheels(index), XData = wheel(:, 1), YData = wheel(:, 2));
        offset = [-0.3, 0.3];
        alignment = "right";
        if geometry.WheelCenters(index, 1) >= geometry.Center(1)
            offset(1) = 0.3;
            alignment = "left";
        end
        if index > 2, offset(2) = -0.3; end
        position = geometry.WheelCenters(index, :) + offset;
        set(view.WheelLabels(index), Position = [position, 0], HorizontalAlignment = alignment, ...
            String = sprintf('%s\nFx %s / Fy %s kN', data.WheelNames(index), ...
            number(frame.Tire.Fx(index) / 1000, '%+.2f'), ...
            number(frame.Tire.Fy(index) / 1000, '%+.2f')));
    end
    updateTrack(view, data.Track, geometry);
    rows = max(1, frame.SampleIndex - 500):max(1, frame.SampleIndex);
    rows = rows(1:max(1, ceil(numel(rows) / 300)):end);
    trail = [data.Signals.X(rows), data.Signals.Y(rows); ...
        frame.Vehicle.X, frame.Vehicle.Y];
    trail = (trail - geometry.ViewOrigin) * geometry.ViewRotation.';
    set(view.Trail, XData = trail(:, 1), YData = trail(:, 2));
    if options.View == "overview"
        fitRaceReplayViewport(view.MainAxes, view.MapBounds);
    else
        bounds = geometry.Center + [-0.5, -0.34; 0.5, 0.34] * options.FollowSpan;
        fitRaceReplayViewport(view.MainAxes, bounds);
    end
else
    set(view.Body, XData = NaN, YData = NaN);
    set(view.FxArrows, UData = NaN(1, 4), VData = NaN(1, 4));
    set(view.FyArrows, UData = NaN(1, 4), VData = NaN(1, 4));
    set(view.Wheels, XData = NaN, YData = NaN);
    set(view.WheelLabels, String = "姿态不可用");
end
set(view.WheelLabels, Visible = onOff(options.ShowValues));
set(view.MapPosition, XData = frame.Vehicle.X, YData = frame.Vehicle.Y);
rows = 1:max(1, ceil(frame.SampleIndex / 2000)):frame.SampleIndex;
set(view.MapTrail, XData = data.Signals.X(rows), YData = data.Signals.Y(rows));
values = {
    number(frame.Elapsed, '%.2f');
    number(frame.Track.LapTime, '%.2f');
    number(frame.Vehicle.Speed * 3.6, '%.1f');
    number(frame.Motor.TotalPower / 1000, '%+.2f');
    number(frame.BatteryPower / 1000, '%+.2f');
    number(frame.Vehicle.Ay / 9.80665, '%+.2f');
    number(frame.Vehicle.Ax / 9.80665, '%+.2f');
    number(frame.Driver.SteeringRequest * 180 / pi, '%+.1f');
    [number(frame.Track.LapIndex, '%.0f'), ' / ', number(frame.Track.Progress * 100, '%.1f')];
    };
for index = 1:numel(values), view.StatsValues(index).String = values{index}; end
visibility = onOff(options.ShowStats);
if string(view.StatsAxes.Visible) ~= visibility
    set(view.StatsAxes, Visible = visibility);
    set(view.StatsAxes.Children, Visible = visibility);
end
% Only include acceleration samples at or before the playback clock. Include
% the interpolated point so seeking between saved samples stays synchronized.
rows = 1:max(1, ceil(frame.SampleIndex / 1500)):frame.SampleIndex;
set(view.GGTrail, XData = [data.Signals.Ay(rows); frame.Vehicle.Ay] / 9.80665, ...
    YData = [data.Signals.Ax(rows); frame.Vehicle.Ax] / 9.80665);
set(view.GGPoint, XData = frame.Vehicle.Ay / 9.80665, YData = frame.Vehicle.Ax / 9.80665);
fitRaceReplayViewport(view.MapAxes);
forceLimit = frictionForceLimit(view, frame);
for index = 1:4
    renderFriction(view, frame, index, forceLimit);
end
motor = [frame.Motor.RPM(:), frame.Motor.Torque(:), frame.Motor.Power(:) / 1000];
formats = {'%.0f', '%+.1f', '%+.2f'};
for index = 1:4
    for quantity = 1:3
        value = motor(index, quantity);
        view.MotorBars(index, quantity).YData = [0, 0, value, value];
        view.MotorText(index, quantity).String = number(motor(index, quantity), formats{quantity});
    end
end
values = [frame.Pedals.Accelerator, frame.Pedals.Brake];
view.PedalBars.YData = values;
for index = 1:2
    view.PedalText(index).String = [number(values(index), '%.1f'), ' %'];
end
if frame.Pedals.Equivalent
    pedalTitle = '驾驶员请求开度（等效）';
else
    pedalTitle = '驾驶员请求开度';
end
if ~strcmp(view.PedalAxes.Title.String, pedalTitle)
    view.PedalAxes.Title.String = pedalTitle;
    view.PedalAxes.Title.FontSize = 10;
    view.PedalAxes.Title.Color = [0.15, 0.15, 0.15];
end
end

function updateTrack(view, track, geometry)
if isempty(fieldnames(track))
    return
end
transform = [geometry.ViewRotation(:); geometry.ViewOrigin(:)];
if isequal(getappdata(view.MainAxes, "ReplayTrackTransform"), transform)
    return
end
setappdata(view.MainAxes, "ReplayTrackTransform", transform);
center = [track.X, track.Y];
left = [track.LeftX, track.LeftY];
right = [track.RightX, track.RightY];
center = (center - geometry.ViewOrigin) * geometry.ViewRotation.';
left = (left - geometry.ViewOrigin) * geometry.ViewRotation.';
right = (right - geometry.ViewOrigin) * geometry.ViewRotation.';
set(view.Centerline, XData = center(:, 1), YData = center(:, 2));
set(view.LeftBoundary, XData = left(:, 1), YData = left(:, 2));
set(view.RightBoundary, XData = right(:, 1), YData = right(:, 2));
set(view.RoadPatch, XData = [left(:, 1); flipud(right(:, 1))], ...
    YData = [left(:, 2); flipud(right(:, 2))]);
end

function renderFriction(view, frame, index, forceLimit)
ax = view.FrictionAxes(index);
capacity = frame.Tire.Capacity;
recorded = frame.Tire.RecordedUtilization(index) * 100;
x = frame.Tire.Fy(index) / 1000;
y = frame.Tire.Fx(index) / 1000;
updateLimits(ax, [-forceLimit, forceLimit], [-forceLimit, forceLimit]);
if capacity.Available(index)
    set(view.FrictionCircle(index), ...
        XData = capacity.Y(index) / 1000 * cos(view.CircleAngle), ...
        YData = capacity.X(index) / 1000 * sin(view.CircleAngle));
    setVisible(view.FrictionCircle(index), "on");
    road = capacity.RoadLimit(index);
    if isfinite(road) && road > 0
        set(view.RoadCircle(index), XData = road / 1000 * ...
            cos(view.CircleAngle), YData = road / 1000 * sin(view.CircleAngle));
    else
        hideRoadCircle(view.RoadCircle(index));
    end
    view.FrictionText(index).String = sprintf('参考 %s%%\n记录 %s%%', ...
        number(capacity.RawUtilization(index) * 100, '%.1f'), number(recorded, '%.1f'));
else
    setVisible(view.FrictionCircle(index), "off");
    hideRoadCircle(view.RoadCircle(index));
    status = '容量未记录';
    if capacity.NoGrip(index), status = '无抓地 / 上限为零'; end
    if capacity.LowLoad(index)
        status = '低载荷 / 卸载';
    end
    view.FrictionText(index).String = sprintf('%s\n记录 %s%%', ...
        status, number(recorded, '%.1f'));
end
set(view.FrictionVector(index), XData = [0, x], YData = [0, y]);
set(view.FrictionPoint(index), XData = x, YData = y);
util = capacity.RawUtilization(index);
if ~isfinite(util)
    util = recorded / 100;
end
color = [0.15, 0.65, 0.3];
if util > 1
    color = [0.85, 0.18, 0.18];
elseif util >= 0.9
    color = [0.95, 0.65, 0.1];
end
if ~isequal(view.FrictionPoint(index).MarkerFaceColor, color)
    view.FrictionPoint(index).MarkerFaceColor = color;
end
end

function limit = frictionForceLimit(view, frame)
% All four wheels share one physical scale; include every visible boundary.
capacity = frame.Tire.Capacity;
available = capacity.Available;
values = [capacity.X(available), capacity.Y(available), ...
    capacity.RoadLimit(available), frame.Tire.Fx, frame.Tire.Fy] / 1000;
values = abs(values(isfinite(values)));
limit = view.ForceLimitKN;
if ~isempty(values), limit = max(limit, 1.1 * max(values)); end
end

function updateLimits(ax, x, y)
if ~isequal(ax.XLim, x), ax.XLim = x; end
if ~isequal(ax.YLim, y), ax.YLim = y; end
end

function setVisible(handle, value)
if string(handle.Visible) ~= value, handle.Visible = value; end
end

function hideRoadCircle(handle)
if any(isfinite(handle.XData)), set(handle, XData = NaN, YData = NaN); end
end

function value = number(value, format)
if ~isfinite(value)
    value = '不可用';
else
    value = sprintf(format, value);
end
end

function value = onOff(value)
if value
    value = "on";
else
    value = "off";
end
end
