function fitRaceReplayViewport(ax, bounds)
%FITRACEREPLAYVIEWPORT Fill the axes rectangle while retaining equal metre scales.
arguments
    ax
    bounds double = []
end
if ~isgraphics(ax), return; end
if isempty(bounds)
    bounds = getappdata(ax, "ReplayViewportBounds");
    if isempty(bounds), return; end
else
    setappdata(ax, "ReplayViewportBounds", bounds);
end
rectangle = ax.InnerPosition;
if any(rectangle(3:4) <= 0), return; end
span = max(diff(bounds, 1, 1), eps);
aspect = rectangle(3) / rectangle(4);
width = max(span(1), span(2) * aspect);
height = width / aspect;
center = mean(bounds, 1);
x = center(1) + [-0.5, 0.5] * width;
y = center(2) + [-0.5, 0.5] * height;
if ~isequal(ax.XLim, x), ax.XLim = x; end
if ~isequal(ax.YLim, y), ax.YLim = y; end
end
