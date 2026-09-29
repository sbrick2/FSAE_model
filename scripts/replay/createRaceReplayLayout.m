function view = createRaceReplayLayout(parent, wheelNames, renderer)
%CREATERACEREPLAYLAYOUT Share plot construction between UI and export layouts.
%   Graphics export uses axes in one figure, avoiding nested UI grids/panels.
arguments
    parent
    wheelNames (1, 4) string
    renderer (1, 1) string {mustBeMember(renderer, ["ui", "graphics"])}
end
if renderer == "ui"
    view = createUILayout(parent, wheelNames);
else
    view = createGraphicsLayout(parent, wheelNames);
end
end

function view = createUILayout(parent, wheelNames)
view.Grid = uigridlayout(parent, [1, 2], ...
    ColumnWidth = {'1.5x', '1x'}, Padding = [6, 6, 6, 6], ColumnSpacing = 10);
view.ScenePanel = uipanel(view.Grid, BorderType = "none", AutoResizeChildren = "off");
view.ScenePanel.Layout.Column = 1;
right = uigridlayout(view.Grid, [1, 1], Padding = [0, 0, 0, 0]);
right.Layout.Column = 2;
view.MainAxes = uiaxes(view.ScenePanel, PositionConstraint = "innerposition");
view.MapAxes = uiaxes(view.ScenePanel, PositionConstraint = "innerposition");
view.PedalAxes = uiaxes(view.ScenePanel, PositionConstraint = "innerposition");
frictionPanel = uipanel(right, Title = "轮胎利用率与电机状态", ...
    FontWeight = "bold");
frictionPanel.Layout.Row = 1;
frictionGrid = uigridlayout(frictionPanel, [2, 2], Padding = [4, 4, 4, 4]);
view.FrictionAxes = gobjects(1, 4);
view.MotorAxes = gobjects(4, 3);
for index = 1:4
    wheelPanel = uipanel(frictionGrid, Title = char(wheelNames(index)), ...
        FontWeight = "bold", FontSize = 12);
    wheelPanel.Layout.Row = ceil(index / 2);
    wheelPanel.Layout.Column = mod(index - 1, 2) + 1;
    wheelGrid = uigridlayout(wheelPanel, [1, 2], ...
        ColumnWidth = {'1.2x', '1x'}, Padding = [2, 2, 2, 2], ColumnSpacing = 2);
    view.FrictionAxes(index) = uiaxes(wheelGrid);
    view.FrictionAxes(index).Layout.Column = 1;
    motorGrid = uigridlayout(wheelGrid, [1, 3], ...
        ColumnWidth = {'1x', '1x', '1x'}, Padding = [0, 0, 0, 0], ColumnSpacing = 2);
    motorGrid.Layout.Column = 2;
    for quantity = 1:3
        view.MotorAxes(index, quantity) = uiaxes(motorGrid);
        view.MotorAxes(index, quantity).Layout.Column = quantity;
    end
end
end

function view = createGraphicsLayout(parent, wheelNames)
% Fixed pixel layout at the requested export size; all content is graphics.
dimensions = parent.Position(3:4);
width = dimensions(1);
height = dimensions(2);
leftWidth = (width - 22) * 0.6;
rightX = 6 + leftWidth + 10;
rightWidth = width - rightX - 6;
view.Grid = [];
view.ScenePanel = [];
view.MainAxes = graphicsAxes(parent, [6, 6, leftWidth, height - 12], [4, 4, 4, 26]);
% Corner insets get their final positions from the shared overlay layout.
view.MapAxes = graphicsAxes(parent, [6, 6, 60, 60], [0, 0, 0, 0]);
view.PedalAxes = graphicsAxes(parent, [6, 6, 60, 60], [0, 0, 0, 0]);
view.FrictionAxes = gobjects(1, 4);
view.MotorAxes = gobjects(4, 3);
frictionBottom = 6;
frictionHeight = height - frictionBottom - 6;
caption = "轮胎利用率与电机状态";
graphicsCard(parent, [rightX, frictionBottom, rightWidth, frictionHeight], caption, dimensions);
cellWidth = (rightWidth - 18) / 2;
cellHeight = (frictionHeight - 38) / 2;
for index = 1:4
    column = mod(index - 1, 2);
    row = ceil(index / 2);
    cellX = rightX + 4 + column * (cellWidth + 10);
    cellY = frictionBottom + 4 + (2 - row) * (cellHeight + 10);
    graphicsCard(parent, [cellX, cellY, cellWidth, cellHeight], wheelNames(index), dimensions);
    contentWidth = cellWidth - 4;
    circleWidth = (contentWidth - 2) * 1.2 / 2.2;
    contentHeight = cellHeight - 24;
    view.FrictionAxes(index) = graphicsAxes(parent, ...
        [cellX + 2, cellY + 2, circleWidth, contentHeight], [36, 38, 8, 24]);
    motorX = cellX + 4 + circleWidth;
    motorWidth = (contentWidth - circleWidth - 6) / 3;
    for quantity = 1:3
        view.MotorAxes(index, quantity) = graphicsAxes(parent, ...
            [motorX + (quantity - 1) * (motorWidth + 2), cellY + 2, motorWidth, contentHeight], [4, 26, 4, 22]);
    end
end
end

function ax = graphicsAxes(parent, rectangle, margins)
% Explicit inner positions prevent figure-wide default axes insets from
% consuming most of the small map and narrow motor columns.
inner = [rectangle(1:2) + margins(1:2), ...
    rectangle(3:4) - margins(1:2) - margins(3:4)];
ax = axes(parent, Units = "pixels", Position = inner, ...
    PositionConstraint = "innerposition", Toolbar = []);
end

function graphicsCard(parent, rectangle, caption, dimensions)
normalized = rectangle ./ [dimensions, dimensions];
annotation(parent, "rectangle", normalized, Color = [0.5, 0.5, 0.5]);
titleRectangle = [rectangle(1) + 4, rectangle(2) + rectangle(4) - 21, rectangle(3) - 8, 20];
annotation(parent, "textbox", titleRectangle ./ [dimensions, dimensions], ...
    String = caption, FontName = "Microsoft YaHei", FontSize = 10, FontWeight = "bold", ...
    EdgeColor = "none", Margin = 0, Interpreter = "none", FitBoxToText = "off");
end
