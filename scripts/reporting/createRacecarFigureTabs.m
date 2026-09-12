function [figureHandle, axesHandles, tabHandles] = createRacecarFigureTabs( ...
        windowName, tabNames, visibility)
%CREATERACECARFIGURETABS 在一个标准图窗中为每张分析图创建标签页。

arguments
    windowName (1, 1) string
    tabNames string
    visibility (1, 1) string = "on"
end

tabNames = tabNames(:);
assert(~isempty(tabNames), "FSAE:Analysis:EmptyFigureTabs", ...
    "At least one tab name is required.");
assert(any(visibility == ["on", "off"]), ...
    "FSAE:Analysis:InvalidFigureVisibility", ...
    "Visibility must be on or off.");

figureHandle = figure( ...
    "Name", windowName, ...
    "NumberTitle", "off", ...
    "Visible", visibility, ...
    "Color", "white", ...
    "WindowStyle", "normal");
tabGroup = uitabgroup("Parent", figureHandle);
tabHandles = gobjects(numel(tabNames), 1);
axesHandles = gobjects(numel(tabNames), 1);
for index = 1:numel(tabNames)
    tabHandles(index) = uitab(tabGroup, "Title", tabNames(index));
    axesHandles(index) = axes("Parent", tabHandles(index));
end
end
