function figures = plotQuasiStaticResults(results, options)
%PLOTQuasiStaticRESULTS Plot GGV surfaces and parameter-sweep summaries in tabs.
%   The default invisible figures make this helper suitable for batch runs;
%   callers can set Visible="on" for interactive review.

arguments
    results (1, :) struct
    options.Visible (1, 1) string = "off"
    options.OutputFolder (1, 1) string = ""
end

success = [results.Success];
assert(any(success), "FSAE:QuasiStatic:NoPlotResults", ...
    "At least one successful QuasiStatic result is required for plotting.");
first = results(find(success, 1, "first")).GGV;

tabNames = [ ...
    "Speed-dependent lateral boundary"
    "Maximum longitudinal acceleration"
    "Parameter sweep summary"
];
[figureHandle, axesHandles] = createRacecarFigureTabs( ...
    "QuasiStatic analysis", tabNames, options.Visible);
figures = struct("Main", figureHandle, ...
    "GGV", figureHandle, "Sweep", figureHandle);
ax = axesHandles(1);
plot(ax, first.Speed, first.AyPositive, "LineWidth", 1.5);
hold(ax, "on");
plot(ax, first.Speed, first.AyNegative, "--", "LineWidth", 1.5);
hold(ax, "off");
grid(ax, "on");
xlabel(ax, "Vehicle speed [m/s]");
ylabel(ax, "Lateral acceleration limit [m/s^2]");
legend(ax, "Left turn", "Right turn", Location = "best");
title(ax, "Speed-dependent lateral boundary");

ax = axesHandles(2);
imagesc(ax, first.LateralFraction, first.Speed, first.AxMax);
set(ax, YDir = "normal");
colorbar(ax);
xlabel(ax, "Normalized lateral-acceleration fraction [-]");
ylabel(ax, "Vehicle speed [m/s]");
title(ax, "Maximum longitudinal acceleration [m/s^2]");

ax = axesHandles(3);
if isfield(results, "PeakAx") && isfield(results, "PeakAy")
    peakAx = [results.PeakAx];
    peakAy = [results.PeakAy];
    barHandle = bar(ax, [peakAx(:), peakAy(:)]);
    grid(ax, "on");
    xlabel(ax, "Sweep combination");
    ylabel(ax, "Peak capability [SI]");
    if numel(barHandle) >= 2
        legend(ax, barHandle(1:2), "Peak ax", "Peak |ay|", ...
            Location = "best");
    else
        legend(ax, "Peak capability", Location = "best");
    end
    title(ax, "QuasiStatic capability sweep summary");
else
    parameter1 = [results.Parameter1Value];
    parameter2 = [results.Parameter2Value];
    lapTimes = [results.LapTime];
    valid = [results.Success] & isfinite(parameter1) & ...
        isfinite(parameter2) & isfinite(lapTimes);
    assert(any(valid), "FSAE:QuasiStatic:NoPlotResults", ...
        "At least one finite successful lap-time result is required.");
    parameter1Values = unique(parameter1(valid), "sorted");
    parameter2Values = unique(parameter2(valid), "sorted");
    [~, row] = ismember(parameter1(valid), parameter1Values);
    [~, column] = ismember(parameter2(valid), parameter2Values);
    lapTimeGrid = accumarray([row(:), column(:)], lapTimes(valid), ...
        [numel(parameter1Values), numel(parameter2Values)], @min, NaN);
    imagesc(ax, parameter2Values, parameter1Values, lapTimeGrid);
    set(ax, "YDir", "normal");
    colorbar(ax);
    xlabel(ax, char(string(results(1).Parameter2Name)), ...
        "Interpreter", "none");
    ylabel(ax, char(string(results(1).Parameter1Name)), ...
        "Interpreter", "none");
    title(ax, "QuasiStatic lap-time sweep summary");
end

if strlength(options.OutputFolder) > 0
    if ~isfolder(options.OutputFolder)
        mkdir(options.OutputFolder);
    end
    savefig(figures.Main, fullfile(options.OutputFolder, "QuasiStatic_Summary.fig"));
end
end
