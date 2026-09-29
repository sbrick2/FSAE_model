function view = createRaceReplayView(parent, data, options)
%CREATERACEREPLAYVIEW Build the shared interactive/video dashboard.
arguments
    parent
    data (1, 1) struct
    options.Renderer (1, 1) string {mustBeMember(options.Renderer, ["ui", "graphics"])} = "ui"
end
view = createRaceReplayLayout(parent, data.WheelNames, options.Renderer);
configureReplayAxes(view.MainAxes);
title(view.MainAxes, "车辆与轮胎力 · 蓝 Fx / 橙 Fy");
view.MainAxes.XTick = [];
view.MainAxes.YTick = [];
view.MainAxes.Color = [0.94, 0.97, 0.94];
hold(view.MainAxes, "on");
view.RoadPatch = patch(view.MainAxes, NaN, NaN, [0.32, 0.34, 0.36], ...
    EdgeColor = "none", FaceAlpha = 0.95);
view.Centerline = plot(view.MainAxes, NaN, NaN, "--", ...
    Color = [0.65, 0.65, 0.65], LineWidth = 0.8);
view.LeftBoundary = plot(view.MainAxes, NaN, NaN, "-", ...
    Color = [0.90, 0.36, 0.27], LineWidth = 1.5);
view.RightBoundary = plot(view.MainAxes, NaN, NaN, "-", ...
    Color = [0.90, 0.36, 0.27], LineWidth = 1.5);
view.Trail = plot(view.MainAxes, NaN, NaN, "-", ...
    Color = [0.94, 0.70, 0.10], LineWidth = 1.8);
view.Body = patch(view.MainAxes, NaN, NaN, [0.56, 0.20, 0.76], ...
    EdgeColor = "white", LineWidth = 1.1);
view.Wheels = gobjects(1, 4);
view.WheelLabels = gobjects(1, 4);
for index = 1:4
    view.Wheels(index) = patch(view.MainAxes, NaN, NaN, [0.08, 0.09, 0.10], ...
        EdgeColor = [0.7, 0.7, 0.7]);
    view.WheelLabels(index) = text(view.MainAxes, NaN, NaN, "", ...
        FontSize = 9, Color = [0.12, 0.12, 0.12], ...
        BackgroundColor = [1, 1, 1], Margin = 2, Interpreter = "none");
end
view.FxArrows = quiver(view.MainAxes, NaN(1, 4), NaN(1, 4), ...
    NaN(1, 4), NaN(1, 4), 0, Color = [0.00, 0.55, 0.85], ...
    LineWidth = 2, MaxHeadSize = 0.6, Tag = "ReplayFxArrows");
view.FyArrows = quiver(view.MainAxes, NaN(1, 4), NaN(1, 4), ...
    NaN(1, 4), NaN(1, 4), 0, Color = [0.98, 0.48, 0.07], ...
    LineWidth = 2, MaxHeadSize = 0.6, Tag = "ReplayFyArrows");
view.ScaleText = text(view.MainAxes, 0.50, 0.015, "", Units = "normalized", ...
    HorizontalAlignment = "center", VerticalAlignment = "bottom", FontSize = 8, BackgroundColor = "white", ...
    Margin = 3, Interpreter = "none", Color = [0.1, 0.1, 0.1]);
view.ScaleText.String = sprintf('蓝 Fx / 橙 Fy · 1 kN = %.2f m 箭头 · %s', ...
    1000 * data.ForceScale, data.Geometry.Source);
configureReplayAxes(view.MapAxes);
view.MapAxes.Color = "white";
view.MapAxes.XColor = [0.25, 0.25, 0.25];
view.MapAxes.YColor = [0.25, 0.25, 0.25];
view.MapAxes.XTick = [];
view.MapAxes.YTick = [];
title(view.MapAxes, "赛道地图", FontSize = 10, Color = [0.15, 0.15, 0.15]);
hold(view.MapAxes, "on");
if ~isempty(fieldnames(data.Track))
    plot(view.MapAxes, data.Track.LeftX, data.Track.LeftY, ...
        Color = [0.6, 0.6, 0.6]);
    plot(view.MapAxes, data.Track.RightX, data.Track.RightY, ...
        Color = [0.6, 0.6, 0.6]);
    plot(view.MapAxes, data.Track.X, data.Track.Y, ...
        Color = [0.8, 0.6, 0.15], LineWidth = 1.2);
    points = [data.Track.X, data.Track.Y; ...
        data.Track.LeftX, data.Track.LeftY; data.Track.RightX, data.Track.RightY];
else
    points = [data.Signals.X, data.Signals.Y];
    plot(view.MapAxes, points(:, 1), points(:, 2), ...
        Color = [0.7, 0.7, 0.7], LineWidth = 1);
end
points = points(all(isfinite(points), 2), :);
view.MapBounds = [min(points, [], 1); max(points, [], 1)];
span = max(diff(view.MapBounds, 1, 1), 2);
padding = max(span * 0.025, 1);
view.MapBounds = view.MapBounds + [-padding; padding];
fitRaceReplayViewport(view.MapAxes, view.MapBounds);
view.MapTrail = plot(view.MapAxes, NaN, NaN, "-", ...
    Color = [0.00, 0.45, 0.74], LineWidth = 1.3);
view.MapPosition = plot(view.MapAxes, NaN, NaN, "o", ...
    MarkerSize = 7, MarkerFaceColor = [0.56, 0.20, 0.76], MarkerEdgeColor = "white");
first = find(isfinite(data.Signals.X) & isfinite(data.Signals.Y), 1);
last = find(isfinite(data.Signals.X) & isfinite(data.Signals.Y), 1, "last");
plot(view.MapAxes, data.Signals.X(first), data.Signals.Y(first), "s", ...
    MarkerSize = 5, MarkerFaceColor = [0.15, 0.7, 0.25], MarkerEdgeColor = "none");
plot(view.MapAxes, data.Signals.X(last), data.Signals.Y(last), "s", ...
    MarkerSize = 5, MarkerFaceColor = [0.8, 0.2, 0.2], MarkerEdgeColor = "none");
view.FrictionCircle = gobjects(1, 4);
view.RoadCircle = gobjects(1, 4);
view.FrictionVector = gobjects(1, 4);
view.FrictionPoint = gobjects(1, 4);
view.FrictionText = gobjects(1, 4);
view.MotorBars = gobjects(4, 3);
view.MotorText = gobjects(4, 3);
view.MotorLimits = motorLimits(data);
view.CircleAngle = linspace(0, 2 * pi, 81);
forces = abs([data.Signals.Fx(:); data.Signals.Fy(:)]);
forces = forces(isfinite(forces));
view.ForceLimitKN = 1;
if ~isempty(forces)
    view.ForceLimitKN = max(1, max(forces) / 1000 * 1.1);
end
for index = 1:4
    ax = view.FrictionAxes(index);
    configureReplayAxes(ax);
    ax.FontSize = 9;
    ax.XTickMode = "auto";
    ax.YTickMode = "auto";
    hold(ax, "on");
    grid(ax, "on");
    xlim(ax, [-view.ForceLimitKN, view.ForceLimitKN]);
    ylim(ax, [-view.ForceLimitKN, view.ForceLimitKN]);
    xlabel(ax, "Fy (kN)", FontSize = 9);
    ylabel(ax, "Fx (kN)", FontSize = 9);
    view.FrictionAxes(index) = ax;
    view.FrictionCircle(index) = plot(ax, NaN, NaN, ...
        Color = [0.55, 0.55, 0.55], LineWidth = 1.3);
    view.RoadCircle(index) = plot(ax, NaN, NaN, "--", ...
        Color = [0.95, 0.48, 0.05], LineWidth = 1);
    view.FrictionVector(index) = plot(ax, [0, NaN], [0, NaN], ...
        Color = [0.2, 0.5, 0.2], LineWidth = 1.5);
    view.FrictionPoint(index) = plot(ax, NaN, NaN, "o", ...
        MarkerSize = 6, MarkerFaceColor = [0.2, 0.7, 0.3], MarkerEdgeColor = "none");
    view.FrictionText(index) = text(ax, 0.02, 0.02, "", Units = "normalized", ...
        FontSize = 8, BackgroundColor = "white", Margin = 1, ...
        Interpreter = "none", VerticalAlignment = "bottom", Color = [0.1, 0.1, 0.1]);
    names = ["转速", "转矩", "功率"];
    units = ["rpm", "N·m", "kW"];
    colors = [0.00, 0.55, 0.85; 0.15, 0.72, 0.32; 0.60, 0.35, 0.85];
    for quantity = 1:3
        motorAx = view.MotorAxes(index, quantity);
        set(motorAx, FontSize = 7, XTick = [], YTick = [], Box = "on", Toolbar = []);
        disableDefaultInteractivity(motorAx);
        hold(motorAx, "on");
        view.MotorBars(index, quantity) = patch(motorAx, ...
            [0.675, 1.325, 1.325, 0.675], [0, 0, NaN, NaN], ...
            colors(quantity, :), EdgeColor = "none");
        yline(motorAx, 0, Color = [0.6, 0.6, 0.6]);
        xlim(motorAx, [0.4, 1.6]);
        ylim(motorAx, view.MotorLimits(:, quantity).');
        title(motorAx, names(quantity), FontSize = 8);
        xlabel(motorAx, units(quantity), FontSize = 8);
        view.MotorText(index, quantity) = text(motorAx, 0.5, 0.96, "", ...
            Units = "normalized", HorizontalAlignment = "center", ...
            VerticalAlignment = "top", FontSize = 8, Interpreter = "none");
        text(motorAx, 0.5, 0.02, sprintf('量程\n%g', ...
            view.MotorLimits(2, quantity)), Units = "normalized", ...
            HorizontalAlignment = "center", VerticalAlignment = "bottom", FontSize = 7);
        view.MotorAxes(index, quantity) = motorAx;
    end
end
view.PedalAxes.Units = "pixels";
view.PedalAxes.FontName = "Microsoft YaHei";
view.PedalAxes.FontSize = 9;
view.PedalAxes.Color = "white";
view.PedalAxes.XColor = [0.25, 0.25, 0.25];
view.PedalAxes.YColor = [0.25, 0.25, 0.25];
view.PedalAxes.Box = "on";
view.PedalAxes.Toolbar = [];
disableDefaultInteractivity(view.PedalAxes);
hold(view.PedalAxes, "on");
view.PedalBars = bar(view.PedalAxes, [1, 2], [NaN, NaN], ...
    FaceColor = "flat", BarWidth = 0.55);
view.PedalBars.CData = [0.15, 0.72, 0.32; 0.90, 0.28, 0.22];
view.PedalAxes.XTick = [1, 2];
view.PedalAxes.XTickLabel = {'加速请求', '制动请求'};
view.PedalAxes.YTick = [0, 50, 100];
view.PedalAxes.YTickLabel = {'0%', '50%', '100%'};
xlim(view.PedalAxes, [0.5, 2.5]);
ylim(view.PedalAxes, [0, 100]);
view.PedalAxes.YGrid = "on";
view.PedalText = gobjects(1, 2);
for index = 1:2
    view.PedalText(index) = text(view.PedalAxes, index, 97, "", ...
        HorizontalAlignment = "center", VerticalAlignment = "top", ...
        BackgroundColor = "white", Color = [0.1, 0.1, 0.1], ...
        Margin = 1, FontWeight = "bold", FontSize = 10);
end
[view.GGAxes, view.StatsAxes] = createOverlays(view.MainAxes, options.Renderer);
hold(view.GGAxes, "on");
view.GGTrail = plot(view.GGAxes, NaN, NaN, "-", ...
    Color = [0.90, 0.64, 0.08], LineWidth = 0.8, Tag = "ReplayGGTrail");
view.GGPoint = plot(view.GGAxes, NaN, NaN, "o", ...
    MarkerFaceColor = [0.56, 0.20, 0.76], MarkerEdgeColor = "white", ...
    MarkerSize = 7, Tag = "ReplayGGPoint");
values = abs([data.Signals.Ax(:); data.Signals.Ay(:)]) / 9.80665;
values = values(isfinite(values));
limit = 1.5;
if ~isempty(values), limit = max(limit, ceil(max(values) * 1.1 * 2) / 2); end
xlim(view.GGAxes, [-limit, limit]);
ylim(view.GGAxes, [-limit, limit]);
title(view.GGAxes, "实时 GG", FontSize = 10, Color = [0.15, 0.15, 0.15]);
xlabel(view.GGAxes, "Ay (g)", FontSize = 9);
ylabel(view.GGAxes, "Ax (g)", FontSize = 9);
grid(view.GGAxes, "on");
view.GGAxes.GridAlpha = 0.15;
labels = ["经过计时", "本圈计时", "车速", "总机械功率", "电池功率", ...
    "横向 Ay", "纵向 Ax", "转向角请求", "圈数 / 赛事"];
units = ["s", "s", "km/h", "kW", "kW", "g", "g", "deg", "%"];
view.StatsLabels = gobjects(1, numel(labels));
view.StatsValues = gobjects(1, numel(labels));
view.StatsUnits = gobjects(1, numel(labels));
hold(view.StatsAxes, "on");
xlim(view.StatsAxes, [0, 1]);
ylim(view.StatsAxes, [0, numel(labels)]);
view.StatsAxes.YDir = "reverse";
for index = 1:numel(labels)
    y = index - 0.5;
    if mod(index, 2) == 0
        patch(view.StatsAxes, [0, 1, 1, 0], index - [1, 1, 0, 0], ...
            [0.96, 0.97, 0.98], EdgeColor = "none");
    end
    view.StatsLabels(index) = text(view.StatsAxes, 0.04, y, labels(index), ...
        FontName = "Microsoft YaHei", FontSize = 10, Interpreter = "none", Color = [0.1, 0.1, 0.1]);
    view.StatsValues(index) = text(view.StatsAxes, 0.77, y, "", ...
        HorizontalAlignment = "right", FontName = "Consolas", FontSize = 10, ...
        Interpreter = "none", Color = [0.1, 0.1, 0.1], Tag = "ReplayStatisticsValue");
    view.StatsUnits(index) = text(view.StatsAxes, 0.80, y, units(index), ...
        FontName = "Consolas", FontSize = 9, Interpreter = "none", Color = [0.1, 0.1, 0.1]);
end
if ~isempty(view.ScenePanel)
    % Hidden UI containers update their positions without firing resize
    % callbacks. Flush the new layout once before fitting the initial view.
    view.ScenePanel.SizeChangedFcn = @(panel, ~) layoutSceneAxes(panel, ...
        view.MainAxes, view.MapAxes, view.GGAxes, view.StatsAxes, view.PedalAxes);
    drawnow;
    view.ScenePanel.SizeChangedFcn(view.ScenePanel, []);
else
    layoutRaceReplayOverlays(view.MainAxes, view.GGAxes, view.StatsAxes, view.MapAxes, view.PedalAxes);
end
end

function [ggAxes, statsAxes] = createOverlays(mainAxes, renderer)
parent = mainAxes.Parent;
if renderer == "ui"
    ggAxes = uiaxes(parent, PositionConstraint = "innerposition");
    statsAxes = uiaxes(parent, PositionConstraint = "innerposition");
else
    ggAxes = axes(parent, Units = "pixels", PositionConstraint = "innerposition");
    statsAxes = axes(parent, Units = "pixels", PositionConstraint = "innerposition");
end
configureReplayAxes(ggAxes);
ggAxes.Color = "white";
ggAxes.XColor = [0.25, 0.25, 0.25];
ggAxes.YColor = [0.25, 0.25, 0.25];
ggAxes.FontSize = 8;
ggAxes.Tag = "ReplayGGDiagram";
set(statsAxes, Units = "pixels", XTick = [], YTick = [], Box = "on", ...
    Color = "white", XColor = [0.6, 0.6, 0.6], YColor = [0.6, 0.6, 0.6], ...
    Toolbar = [], Tag = "ReplayStatistics");
disableDefaultInteractivity(statsAxes);
end

function limits = motorLimits(data)
power = data.Signals.RecordedMotorPower;
derived = data.Signals.MotorTorque .* data.Signals.MotorOmega;
power(isfinite(derived)) = derived(isfinite(derived));
% The product of interpolated torque and speed can peak between samples.
torque = data.Signals.MotorTorque;
omega = data.Signals.MotorOmega;
changeTorque = diff(torque, 1, 1);
changeOmega = diff(omega, 1, 1);
fraction = -(torque(1:end-1, :) .* changeOmega + ...
    omega(1:end-1, :) .* changeTorque) ./ (2 * changeTorque .* changeOmega);
peak = (torque(1:end-1, :) + fraction .* changeTorque) .* ...
    (omega(1:end-1, :) + fraction .* changeOmega);
peak(~isfinite(fraction) | fraction <= 0 | fraction >= 1) = NaN;
values = {data.Signals.MotorOmega * 60 / (2 * pi), ...
    data.Signals.MotorTorque, [power; peak] / 1000};
limits = zeros(2, 3);
for quantity = 1:3
    finite = values{quantity}(isfinite(values{quantity}));
    maximum = 1;
    if ~isempty(finite), maximum = max(1, max(abs(finite))); end
    % One fixed range per quantity is shared by all four motors for this run.
    scale = 10 ^ floor(log10(maximum));
    maximum = ceil(1.15 * maximum / scale) * scale;
    minimum = -maximum;
    if quantity == 1 && ~any(finite < 0), minimum = 0; end
    limits(:, quantity) = [minimum; maximum];
end
end

function configureReplayAxes(ax)
ax.Units = "pixels";
ax.FontName = "Microsoft YaHei";
ax.FontSize = 10;
ax.Toolbar = [];
disableDefaultInteractivity(ax);
axis(ax, "equal");
ax.PlotBoxAspectRatioMode = "auto";
box(ax, "on");
end

function layoutSceneAxes(panel, mainAxes, mapAxes, ggAxes, statsAxes, pedalAxes)
if ~isgraphics(mainAxes) || ~isgraphics(mapAxes), return; end
dimensions = panel.Position(3:4);
width = max(1, dimensions(1) - 8);
mainRectangle = [4, 4, width, max(1, dimensions(2) - 34)];
if ~isequal(mainAxes.InnerPosition, mainRectangle), mainAxes.InnerPosition = mainRectangle; end
fitRaceReplayViewport(mainAxes);
layoutRaceReplayOverlays(mainAxes, ggAxes, statsAxes, mapAxes, pedalAxes);
end
