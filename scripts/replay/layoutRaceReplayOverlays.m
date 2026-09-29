function layoutRaceReplayOverlays(mainAxes, ggAxes, statsAxes, mapAxes, pedalAxes)
%LAYOUTRACEREPLAYOVERLAYS Anchor all four corner insets inside the scene.
rectangle = mainAxes.InnerPosition;
% Reserve a left card for GG labels and a right card for the data table.
ggWidth = min(190, rectangle(3) * 0.36);
ggHeight = min(180, rectangle(4) * 0.48);
side = max(1, min(ggWidth - 48, ggHeight - 45));
ggPosition = [rectangle(1) + 36, rectangle(2) + rectangle(4) - 24 - side, side, side];
statsWidth = min(260, rectangle(3) * 0.46);
statsHeight = min(198, rectangle(4) * 0.50);
statsPosition = [rectangle(1) + rectangle(3) - statsWidth - 10, ...
    rectangle(2) + rectangle(4) - statsHeight - 10, statsWidth, statsHeight];
if ~isequal(ggAxes.InnerPosition, ggPosition), ggAxes.InnerPosition = ggPosition; end
if ~isequal(statsAxes.InnerPosition, statsPosition), statsAxes.InnerPosition = statsPosition; end
mapWidth = min(315, rectangle(3) * 0.46);
mapHeight = min(112, rectangle(4) * 0.24);
mapPosition = [rectangle(1) + 10, rectangle(2) + 30, mapWidth, mapHeight];
pedalWidth = min(220, rectangle(3) * 0.34);
pedalHeight = min(112, rectangle(4) * 0.24);
pedalPosition = [rectangle(1) + rectangle(3) - pedalWidth - 10, ...
    rectangle(2) + 30, pedalWidth, pedalHeight];
if ~isequal(mapAxes.InnerPosition, mapPosition), mapAxes.InnerPosition = mapPosition; end
if ~isequal(pedalAxes.InnerPosition, pedalPosition), pedalAxes.InnerPosition = pedalPosition; end
fitRaceReplayViewport(mapAxes);
end
