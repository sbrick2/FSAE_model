function lines = plotRacecarChannels(ax, x, data, yLabel, seriesNames, xLabel)
%PLOTRACECARCHANNELS 绘制单通道或多通道数据，缺失时显示安全占位提示。

arguments
    ax (1, 1) matlab.graphics.axis.Axes
    x (:, 1) double
    data = []
    yLabel (1, 1) string = ""
    seriesNames (1, :) string = strings(1, 0)
    xLabel (1, 1) string = ""
end

lines = gobjects(0);
if isempty(data)
    showMissing(ax, yLabel);
    xlabel(ax, xLabel);
    ylabel(ax, yLabel);
    return
end
data = double(data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(x) && size(data, 2) == numel(x)
    data = data.';
end
if size(data, 1) ~= numel(x)
    showMissing(ax, yLabel + " (length mismatch)");
    xlabel(ax, xLabel);
    ylabel(ax, yLabel);
    return
end

lines = gobjects(size(data, 2), 1);
colors = linesColorOrder(size(data, 2));
hold(ax, "on");
for index = 1:size(data, 2)
    lines(index, 1) = plot(ax, x, data(:, index), ...
        "Color", colors(index, :), "LineWidth", 1.15);
end
hold(ax, "off");
grid(ax, "on");
xlabel(ax, xLabel);
ylabel(ax, yLabel);
if isempty(seriesNames)
    seriesNames = "Signal " + string(1:size(data, 2));
end
if numel(seriesNames) >= size(data, 2) && size(data, 2) > 1
    legend(ax, lines, seriesNames(1:size(data, 2)), ...
        "Location", "best", "Interpreter", "none");
end
end

function colors = linesColorOrder(count)
base = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; ...
    0.4660 0.6740 0.1880; 0.6350 0.0780 0.1840; ...
    0.4940 0.1840 0.5560; 0.3010 0.7450 0.9330; ...
    0.9290 0.6940 0.1250];
colors = base(mod(0:count - 1, size(base, 1)) + 1, :);
end

function showMissing(ax, label)
axis(ax, "off");
text(ax, 0.5, 0.5, "Unavailable: " + label, ...
    "Units", "normalized", "HorizontalAlignment", "center", ...
    "Color", [0.35, 0.35, 0.35]);
end
