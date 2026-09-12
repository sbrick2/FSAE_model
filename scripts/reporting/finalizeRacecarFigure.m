function filePath = finalizeRacecarFigure( ...
        fig, saveFigure, outputFolder, stem, trackName)
%FINALIZERACECARFIGURE 统一一个或多个图窗样式，并按需保存 MATLAB FIG。

arguments
    fig matlab.ui.Figure
    saveFigure (1, 1) logical = false
    outputFolder (1, 1) string = ""
    stem (1, 1) string = "racecar_analysis"
    trackName (1, 1) string = "autocross"
end

assert(isvector(fig) && ~isempty(fig), "FSAE:Analysis:InvalidFigureSet", ...
    "fig must contain at least one figure handle.");
for figureHandle = fig(:).'
    set(figureHandle, "Color", "white");
    axesHandles = findall(figureHandle, "Type", "axes");
    set(axesHandles, "Color", "white", "XColor", "black", ...
        "YColor", "black", "GridColor", [0.75, 0.75, 0.75]);
    textHandles = findall(figureHandle, "Type", "text");
    set(textHandles, "Color", "black");
    legendHandles = findall(figureHandle, "Type", "legend");
    set(legendHandles, "Color", "white", "TextColor", "black");
    colorBars = findall(figureHandle, "Type", "ColorBar");
    set(colorBars, "Color", "black");
end
drawnow;

filePath = strings(size(fig));
if ~saveFigure
    return
end
if strlength(outputFolder) == 0
    trackName = lower(trackName);
    assert(any(trackName == ["acceleration", "skidpad", "autocross", ...
        "endurance"]), "FSAE:Analysis:InvalidTrack", ...
        "Track must be acceleration, skidpad, autocross or endurance.");
    outputFolder = fullfile(racecarAnalysisProjectRoot(), ...
        "results", "time_domain_closed_loop", trackName, "plots", stem);
end
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end
timestamp = string(datetime("now", TimeZone = "UTC", ...
    Format = "yyyyMMdd_HHmmss_SSS"));
for index = 1:numel(fig)
    figureStem = stem;
    if numel(fig) > 1
        figureStem = stem + "_" + compose("%02d", index);
    end
    filePath(index) = string(fullfile(outputFolder, ...
        figureStem + "_" + timestamp + ".fig"));
    savefig(fig(index), filePath(index));
end
end
