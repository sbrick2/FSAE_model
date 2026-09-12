function output = saveLapSimulationResults(result, scenario, cfg, options)
%SAVELAPSIMULATIONRESULTS 保存规范化圈速结果及机器可读摘要。
%   OUTPUT = SAVELAPSIMULATIONRESULTS(RESULT, SCENARIO, CFG) 创建
%   results/time_domain_closed_loop/<event>/<timestamp>_<run_name>/，保存变量名固定为
%   result 的 MAT 文件、run_config.json、simulation_summary.json 和可选的
%   final_track_view.fig。默认不覆盖已有运行目录；原始
%   Simulink.SimulationOutput 只有在 CFG.Output.SaveRawSimulationOutput=true
%   时另存。长度 m、时间 s、速度 m/s、角度 rad，四轮顺序为 [FL, FR, RL, RR]。
%
%   可选参数：
%       SimulationOutput  原始 Simulink.SimulationOutput

arguments
    result (1, 1) struct
    scenario (1, 1) struct
    cfg (1, 1) struct
    options.SimulationOutput = []
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
if isfield(cfg.Output, "ResultsRoot") && strlength(string(cfg.Output.ResultsRoot)) > 0
    resultsRoot = string(cfg.Output.ResultsRoot);
else
    resultsRoot = fullfile(projectRoot, "results", ...
        "time_domain_closed_loop");
end
eventName = lower(string(scenario.Event));
runName = makeSafeRunName(cfg.Output.RunName);
timeStamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
baseName = timeStamp + "_" + runName;
runFolder = makeUniqueRunFolder(fullfile(resultsRoot, eventName), baseName, ...
    cfg.Output.Overwrite);
if ~isfolder(runFolder)
    mkdir(runFolder);
end

resultPath = fullfile(runFolder, "lap_simulation_result.mat");
configPath = fullfile(runFolder, "run_config.json");
summaryPath = fullfile(runFolder, "simulation_summary.json");
rawPath = "";
figurePath = "";
figureImagePath = "";

result.Meta.OutputFolder = string(runFolder);
result.Meta.ResultFile = string(resultPath);
result.Meta.SavedAt = string(datetime("now", ...
    "Format", "yyyy-MM-dd'T'HH:mm:ss"));
save(char(resultPath), "result", "-v7.3");

if cfg.Output.SaveRawSimulationOutput && ~isempty(options.SimulationOutput)
    rawPath = fullfile(runFolder, "raw_simulation_output.mat");
    simulationOutput = options.SimulationOutput;
    save(char(rawPath), "simulationOutput", "-v7.3");
end

configRecord = struct( ...
    "SchemaVersion", result.SchemaVersion, ...
    "Config", cfg, ...
    "Scenario", struct( ...
        "ID", getValue(scenario, "ID", ""), ...
        "Event", getValue(scenario, "Event", ""), ...
        "Description", getValue(scenario, "EventDescription", ""), ...
        "ReferenceSpeedSource", getValue(scenario, "ReferenceSpeedSource", ""), ...
        "SpeedPlanner", getValue(scenario, "SpeedPlannerMetadata", struct()), ...
        "NumberOfLaps", getValue(scenario, "NumberOfLaps", 1), ...
        "TrackLength", getValue(scenario.Track, "Length", NaN), ...
        "TrackSampleDistance", getValue(scenario.Track, "SampleDistance", NaN), ...
        "TrackSourceCsv", getValue(scenario.Track, "SourceCsv", ""), ...
        "TrackSourceImage", getValue(scenario.Track, "SourceImage", "")));
writeJson(configPath, configRecord);

summary = struct( ...
    "SchemaVersion", result.SchemaVersion, ...
    "Valid", result.Meta.Valid, ...
    "Completed", getValue(result.Meta, "Completed", false), ...
    "StoppedAtFinish", getValue(result.Meta, "StoppedAtFinish", false), ...
    "UserAborted", getValue(result.Meta, "UserAborted", false), ...
    "DriverModel", getValue(result.Meta, "SpeedControlMode", "unknown"), ...
    "ReferenceSpeedUsed", getValue(result.Meta, ...
        "ReferenceSpeedUsed", true), ...
    "AdaptiveQualificationPassed", getValue(result.Meta, ...
        "AdaptiveQualificationPassed", false), ...
    "AdaptiveQualificationFailedChecks", getValue(result.Meta, ...
        "AdaptiveQualificationFailedChecks", strings(0, 1)), ...
    "ReferenceQualificationPassed", getValue(result.Meta, ...
        "ReferenceQualificationPassed", false), ...
    "ReferenceQualificationFailedChecks", getValue(result.Meta, ...
        "ReferenceQualificationFailedChecks", strings(0, 1)), ...
    "VehicleEnvelope", struct( ...
        "DataValid", getValue(result.Meta, ...
            "VehicleEnvelopeDataValid", false), ...
        "WithinBoundary", getValue(result.Meta, ...
            "VehicleEnvelopeWithinBoundary", false), ...
        "Method", getValue(result.Meta, ...
            "VehicleEnvelopeMethod", "not evaluated"), ...
        "TireSectionWidth", getValue(result.Meta, ...
            "VehicleEnvelopeTireSectionWidth", NaN)), ...
    "ScenarioID", result.Meta.ScenarioID, ...
    "Event", result.Meta.Event, ...
    "SimulationTime", makeRange(result.Time), ...
    "Distance", makeRange(result.Distance), ...
    "Metrics", result.Metrics, ...
    "MissingSignals", result.Meta.MissingSignals, ...
    "NonFiniteSignals", result.Meta.NonFiniteSignals, ...
    "DimensionIssues", result.Meta.DimensionIssues, ...
    "SavedResult", string(resultPath));
writeJson(summaryPath, summary);

if cfg.Output.SaveFinalTrackView
    [figurePath, figureImagePath] = saveFinalTrackView( ...
        runFolder, scenario, result, cfg);
end

output = struct( ...
    "Folder", string(runFolder), ...
    "ResultFile", string(resultPath), ...
    "ConfigFile", string(configPath), ...
    "SummaryFile", string(summaryPath), ...
    "RawSimulationOutputFile", string(rawPath), ...
    "FinalTrackViewFile", string(figurePath), ...
    "FinalTrackViewImage", string(figureImagePath));
end

function runFolder = makeUniqueRunFolder(eventFolder, baseName, overwrite)
if ~isfolder(eventFolder)
    mkdir(eventFolder);
end
runFolder = fullfile(eventFolder, baseName);
if overwrite || ~isfolder(runFolder)
    return
end
counter = 1;
while isfolder(runFolder)
    runFolder = fullfile(eventFolder, baseName + "_" + sprintf("%02d", counter));
    counter = counter + 1;
end
end

function name = makeSafeRunName(value)
name = string(regexprep(char(string(value)), "[^A-Za-z0-9_-]", "_"));
if strlength(name) == 0
    name = "run";
end
end

function value = getValue(data, fieldName, defaultValue)
if isfield(data, fieldName)
    value = data.(fieldName);
else
    value = defaultValue;
end
end

function range = makeRange(data)
if isempty(data)
    range = [NaN, NaN];
else
    data = double(data(:));
    finiteData = data(isfinite(data));
    if isempty(finiteData)
        range = [NaN, NaN];
    else
        range = [finiteData(1), finiteData(end)];
    end
end
end

function writeJson(path, value)
jsonText = jsonencode(value, PrettyPrint = true);
fileIdentifier = fopen(path, "w", "n", "UTF-8");
assert(fileIdentifier >= 0, "FSAE:Lap:JsonOpenFailed", ...
    "无法写入 JSON 文件：%s。", path);
cleanup = onCleanup(@() fclose(fileIdentifier));
fwrite(fileIdentifier, jsonText, "char");
end

function [figurePath, imagePath] = ...
        saveFinalTrackView(runFolder, scenario, result, cfg)
try
    figureHandle = createFinalTrackViewFigure( ...
        scenario, result, cfg, Visible = "off");
    closeFigure = onCleanup(@() closeIfValid(figureHandle));
    figurePath = fullfile(runFolder, "final_track_view.fig");
    imagePath = fullfile(runFolder, "final_track_view.png");
    saveInteractiveFigure(figureHandle, figurePath);
    exportgraphics(figureHandle, imagePath, "Resolution", 180);
    clear closeFigure
catch exception
    warning("FSAE:Lap:FinalViewFailed", ...
        "最终赛道图保存失败，结果 MAT/JSON 仍已保存：%s", exception.message);
    figurePath = "";
    imagePath = "";
end
end

function closeIfValid(figureHandle)
if isgraphics(figureHandle, "figure")
    close(figureHandle);
end
end

function saveInteractiveFigure(figureHandle, figurePath)
% Save hidden batch figures so that a normal OPENFIG call displays them.
originalState = struct( ...
    "Visible", figureHandle.Visible, ...
    "Units", figureHandle.Units, ...
    "Position", figureHandle.Position, ...
    "WindowState", figureHandle.WindowState);
restoreState = onCleanup(@() restoreFigureState( ...
    figureHandle, originalState));
if string(originalState.Visible) == "off"
    % SAVEFIG persists Visible. Keep the temporary visible window outside
    % the desktop so batch runs do not flash it; OPENFIG moves it onscreen.
    figureHandle.WindowState = "normal";
    figureHandle.Units = "pixels";
    offscreenPosition = figureHandle.Position;
    offscreenPosition(1:2) = -100000;
    figureHandle.Position = offscreenPosition;
    figureHandle.Visible = "on";
end
savefig(figureHandle, figurePath);
clear restoreState
end

function restoreFigureState(figureHandle, state)
if isgraphics(figureHandle, "figure")
    figureHandle.Visible = "off";
    figureHandle.WindowState = state.WindowState;
    figureHandle.Units = state.Units;
    figureHandle.Position = state.Position;
    figureHandle.Visible = state.Visible;
end
end
