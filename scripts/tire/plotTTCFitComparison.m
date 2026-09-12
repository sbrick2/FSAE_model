function outputPaths = plotTTCFitComparison(profile, validationData, outputPath)
%PLOTTTCFITCOMPARISON Plot target-tire validation and fitted lateral force.

arguments
    profile (1, 1) struct
    validationData table
    outputPath (1, 1) string
end

stride = max(1, floor(height(validationData) / 4000));
data = validationData(1:stride:end, :);
[~, predictedY] = evaluateTireMF62( ...
    data.SlipRatio, data.SlipAngleRad, data.CamberAngleRad, ...
    data.NormalLoadN, ones(height(data), 1), ...
    inf(height(data), 1), profile);
[~, mapY, ~, ~, outOfDomain] = evaluateTTCMap( ...
    data.SlipRatio, data.SlipAngleRad, data.CamberAngleRad, ...
    data.NormalLoadN, ones(height(data), 1), ...
    inf(height(data), 1), profile);

plotNames = [ ...
    "Validation data"
    "Fitted target-size lateral model"
    "Measured vs fitted"
    "Reference map in-domain"
];
[figureHandle, axesHandles] = createComparisonTabs( ...
    profile.Name, plotNames);
cleanup = onCleanup(@() close(figureHandle));

ax = axesHandles(1);
scatter(ax, rad2deg(data.SlipAngleRad), data.TireForceYN, 8, ...
    data.NormalLoadN, "filled");
xlabel(ax, "Slip angle (deg)");
ylabel(ax, "TTC F_y (N)");
title(ax, "Validation data");
grid(ax, "on");
colorbar(ax);

ax = axesHandles(2);
scatter(ax, rad2deg(data.SlipAngleRad), predictedY, 8, ...
    data.NormalLoadN, "filled");
xlabel(ax, "Slip angle (deg)");
ylabel(ax, "MF-equivalent F_y (N)");
title(ax, "Fitted target-size lateral model");
grid(ax, "on");
colorbar(ax);

ax = axesHandles(3);
scatter(ax, data.TireForceYN, predictedY, 8, ...
    rad2deg(data.CamberAngleRad), "filled");
hold(ax, "on");
limit = max(abs([data.TireForceYN; predictedY]));
plot(ax, [-limit, limit], [-limit, limit], "k--");
hold(ax, "off");
xlabel(ax, "Measured F_y (N)");
ylabel(ax, "Predicted F_y (N)");
title(ax, "Measured vs fitted");
axis(ax, "equal");
grid(ax, "on");
colorbar(ax);

ax = axesHandles(4);
validMap = ~outOfDomain;
scatter(ax, data.TireForceYN(validMap), mapY(validMap), 8, ...
    data.PressurePa(validMap) / 1000.0, "filled");
hold(ax, "on");
if any(validMap)
    limit = max(abs([data.TireForceYN(validMap); mapY(validMap)]));
    plot(ax, [-limit, limit], [-limit, limit], "k--");
end
hold(ax, "off");
xlabel(ax, "Measured F_y (N)");
ylabel(ax, "TTC map F_y (N)");
title(ax, "Reference map in-domain");
axis(ax, "equal");
grid(ax, "on");
colorbar(ax);

allAxes = findall(figureHandle, Type = "axes");
set(allAxes, Color = "white", XColor = "black", ...
    YColor = "black", GridColor = [0.75, 0.75, 0.75]);
textHandles = findall(figureHandle, Type = "text");
set(textHandles, Color = "black");
colorBars = findall(figureHandle, Type = "ColorBar");
set(colorBars, Color = "black");
outputPaths = outputPath;
savefig(figureHandle, outputPaths);
end

function [figureHandle, axesHandles] = createComparisonTabs(profileName, plotNames)
figureHandle = figure(Name = profileName + " - TTC fit comparison", ...
    NumberTitle = "off", Visible = "off", Color = "white", ...
    WindowStyle = "normal");
tabGroup = uitabgroup("Parent", figureHandle);
axesHandles = gobjects(numel(plotNames), 1);
for index = 1:numel(plotNames)
    tabHandle = uitab(tabGroup, "Title", plotNames(index));
    axesHandles(index) = axes("Parent", tabHandle);
end
end
