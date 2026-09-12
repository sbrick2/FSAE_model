%RUNQUASISTATICLAPTIME 使用已有 GGV 直接计算一次准静态赛事时间。
%   该文件是用户脚本，可点击“运行”或执行 run('runQuasiStaticLapTime.m')。
%   它加载已保存的 GGV、生成所选赛道并前向/后向传播加速与制动限制，不重新
%   生成 GGV。结束后工作区保留 quasiStaticResult、speedProfile、track 和 ggv。
%
%   配置字段：eventName（acceleration/skidpad/autocross）、ggvFile（为空时
%   自动选择最新含 ggv 的 MAT）、sampleDistance（NaN 使用赛事默认采样距离）、
%   initialSpeed、finalSpeed（NaN 使用赛事默认）、passCount、maximumSpeed、
%   figureVisible（on/off）、saveResults、outputFolder。
%   saveResults=true 时保存 MAT 和 JSON 摘要到
%   results/quasi_static/lap_time/<event>；运行结束仅绘制按速度着色
%   的赛道图。赛事时间见 quasiStaticResult.Summary.EventTime。

% ------------------------- USER CONFIGURATION -------------------------
useInjectedConfig = exist("FSAE_GUI_QUASI_CONFIG", "var") == 1 && ...
    isstruct(FSAE_GUI_QUASI_CONFIG) && isscalar(FSAE_GUI_QUASI_CONFIG);
if useInjectedConfig
    quasiCfg = FSAE_GUI_QUASI_CONFIG;
    clear FSAE_GUI_QUASI_CONFIG
    eventName = string(quasiCfg.EventName);
    ggvFile = string(quasiCfg.GGVFile);
    sampleDistance = double(quasiCfg.SampleDistance);
    initialSpeed = double(quasiCfg.InitialSpeed);
    finalSpeed = double(quasiCfg.FinalSpeed);
    passCount = double(quasiCfg.PassCount);
    maximumSpeed = double(quasiCfg.MaximumSpeed);
    figureVisible = string(quasiCfg.FigureVisible);
    saveResults = logical(quasiCfg.SaveResults);
    outputFolder = string(quasiCfg.OutputFolder);
else
eventName = "autocross";          % acceleration | skidpad | autocross
ggvFile = "";                     % empty -> latest MAT containing ggv
sampleDistance = NaN;             % NaN -> event default (0.5/0.2/0.5 m)
initialSpeed = 0.0;               % m/s; used for open tracks
finalSpeed = NaN;                 % NaN -> event default; acceleration uses Inf
passCount = 4;                    % repeated forward/backward propagation
maximumSpeed = Inf;               % m/s; additionally capped by the GGV grid
figureVisible = "on";             % "on" or "off"
saveResults = true;
outputFolder = "";                % empty -> results/quasi_static/lap_time/<event>
end
% -----------------------------------------------------------------------

assert(isscalar(sampleDistance) && ...
    (isnan(sampleDistance) || (isfinite(sampleDistance) && sampleDistance > 0)), ...
    "FSAE:QuasiStatic:QuasiStaticSampleDistance", ...
    "sampleDistance must be NaN or a finite positive value in metres.");
assert(isscalar(initialSpeed) && isfinite(initialSpeed) && initialSpeed >= 0, ...
    "FSAE:QuasiStatic:QuasiStaticInitialSpeed", ...
    "initialSpeed must be a finite nonnegative value in m/s.");
assert(isscalar(finalSpeed) && ...
    (isnan(finalSpeed) || (isreal(finalSpeed) && finalSpeed >= 0)), ...
    "FSAE:QuasiStatic:QuasiStaticFinalSpeed", ...
    "finalSpeed must be NaN or a nonnegative value in m/s.");
assert(isscalar(passCount) && isfinite(passCount) && passCount >= 1 && ...
    passCount == floor(passCount), "FSAE:QuasiStatic:QuasiStaticPassCount", ...
    "passCount must be a positive integer.");
assert(isscalar(maximumSpeed) && isreal(maximumSpeed) && maximumSpeed > 0, ...
    "FSAE:QuasiStatic:QuasiStaticMaximumSpeed", ...
    "maximumSpeed must be positive.");
assert(any(figureVisible == ["on", "off"]), ...
    "FSAE:QuasiStatic:QuasiStaticFigureVisibility", ...
    "figureVisible must be on or off.");

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(fullfile(projectRoot, "scripts", "lap_time"), "-begin");
addpath(fullfile(projectRoot, "scripts", "track"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Acceleration"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Skidpad"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Autocross"), "-begin");

ggvPath = resolveGGVFile(projectRoot, ggvFile);
savedGGV = load(char(ggvPath));
assert(isfield(savedGGV, "ggv"), "FSAE:QuasiStatic:QuasiStaticMissingGGV", ...
    "The selected MAT file does not contain a variable named ggv: %s", ggvPath);
ggv = validateSavedGGV(savedGGV.ggv);

[scenario, canonicalEvent, profileDefaults] = createEventScenario( ...
    eventName, sampleDistance);
if isnan(finalSpeed)
    finalSpeed = profileDefaults.FinalSpeed;
end

speedProfile = calculateSpeedProfile(scenario.Track, ggv, ...
    UseReferenceSpeed = false, ...
    InitialSpeed = initialSpeed, ...
    FinalSpeed = finalSpeed, ...
    PassCount = passCount, ...
    MinimumSpeed = 0.0, ...
    MaximumSpeed = maximumSpeed);
eventMetric = calculateEventMetric( ...
    canonicalEvent, speedProfile, scenario.Track);
assert(speedProfile.IsValid && eventMetric.IsValid, ...
    "FSAE:QuasiStatic:QuasiStaticInvalidResult", ...
    "Quasi-static result is invalid: %s", ...
    firstFailureReason(speedProfile, eventMetric));

eventAverageSpeed = eventMetric.Distance / eventMetric.Time;
fullCourseAverageSpeed = speedProfile.TrackLength / speedProfile.LapTime;
summary = table( ...
    canonicalEvent, string(eventMetric.Name), eventMetric.Time, ...
    speedProfile.LapTime, eventMetric.Distance, speedProfile.TrackLength, ...
    eventAverageSpeed, fullCourseAverageSpeed, max(speedProfile.Speed), ...
    min(speedProfile.Speed), ...
    VariableNames = ["Event", "Metric", "EventTime_s", ...
    "FullCourseTime_s", "EventDistance_m", "TrackLength_m", ...
    "EventAverageSpeed_mps", "FullCourseAverageSpeed_mps", ...
    "MaximumSpeed_mps", "MinimumSpeed_mps"]);

ggvConfig = struct;
if isfield(savedGGV, "config")
    ggvConfig = savedGGV.config;
elseif isfield(ggv, "Config")
    ggvConfig = ggv.Config;
end
ggvRunMetadata = struct;
if isfield(savedGGV, "runMetadata")
    ggvRunMetadata = savedGGV.runMetadata;
end

quasiStaticResult = struct( ...
    "SchemaVersion", "1.0", ...
    "GeneratedAtUTC", string(datetime("now", TimeZone = "UTC")), ...
    "Event", canonicalEvent, ...
    "ScenarioID", string(scenario.ID), ...
    "GGVSourceFile", ggvPath, ...
    "GGVConfig", ggvConfig, ...
    "GGVRunMetadata", ggvRunMetadata, ...
    "GGV", ggv, ...
    "Track", scenario.Track, ...
    "SpeedProfile", speedProfile, ...
    "EventMetric", eventMetric, ...
    "Summary", summary, ...
    "Configuration", struct( ...
        "SampleDistance", scenario.Track.SampleDistance, ...
        "InitialSpeed", initialSpeed, ...
        "FinalSpeed", finalSpeed, ...
        "PassCount", passCount, ...
        "MaximumSpeed", maximumSpeed, ...
        "UseReferenceSpeed", false));

figureHandle = plotQuasiStaticResult( ...
    scenario.Track, speedProfile, eventMetric, canonicalEvent, figureVisible);

if saveResults
    if strlength(outputFolder) == 0
        outputFolder = fullfile(projectRoot, "results", "quasi_static", ...
            "lap_time", lower(canonicalEvent));
    elseif ~isAbsolutePath(outputFolder)
        outputFolder = fullfile(projectRoot, outputFolder);
    end
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    timestamp = string(datetime("now", TimeZone = "UTC", ...
        Format = "yyyyMMdd_HHmmss_SSS"));
    runStem = "quasi_static_" + lower(canonicalEvent) + "_" + timestamp;
    resultPath = string(fullfile(outputFolder, runStem + ".mat"));
    figurePath = string(fullfile(outputFolder, runStem + ".fig"));
    save(char(resultPath), "quasiStaticResult", "-v7.3");
    savefig(figureHandle, char(figurePath));
end

disp(summary)
fprintf("GGV source: %s\n", ggvPath);
fprintf("Quasi-static %s result: %.6f s over %.3f m.\n", ...
    canonicalEvent, eventMetric.Time, eventMetric.Distance);
if saveResults
    fprintf("Result MAT: %s\n", resultPath);
    fprintf("Editable figure: %s\n", figurePath);
end

function ggvPath = resolveGGVFile(projectRoot, requestedFile)
requestedFile = string(requestedFile);
if strlength(requestedFile) > 0
    if isfile(requestedFile)
        ggvPath = string(whichExistingPath(requestedFile));
        return
    end
    projectRelative = fullfile(projectRoot, requestedFile);
    assert(isfile(projectRelative), "FSAE:QuasiStatic:QuasiStaticGGVFile", ...
        "GGV file was not found: %s", requestedFile);
    ggvPath = string(projectRelative);
    return
end

searchFolders = fullfile(projectRoot, "results", "quasi_static", ...
    "ggv");
candidatePaths = strings(0, 1);
candidateDates = zeros(0, 1);
for folder = searchFolders
    files = dir(fullfile(folder, "*.mat"));
    for index = 1:numel(files)
        path = string(fullfile(files(index).folder, files(index).name));
        variables = whos("-file", char(path));
        if any(string({variables.name}) == "ggv")
            candidatePaths(end + 1, 1) = path; %#ok<AGROW>
            candidateDates(end + 1, 1) = files(index).datenum; %#ok<AGROW>
        end
    end
end
assert(~isempty(candidatePaths), "FSAE:QuasiStatic:QuasiStaticNoGGV", ...
    "No MAT file containing ggv was found under results/quasi_static/ggv.");
[~, newest] = max(candidateDates);
ggvPath = candidatePaths(newest);
end

function path = whichExistingPath(path)
path = char(path);
if ~isfile(path)
    resolved = which(path);
    if ~isempty(resolved)
        path = resolved;
    end
end
end

function tf = isAbsolutePath(path)
path = char(path);
tf = ~isempty(regexp(path, '^[A-Za-z]:[\\/]|^\\\\', 'once'));
end

function ggv = validateSavedGGV(ggv)
required = ["Speed", "LateralFraction", "AxMax", "AxMin", ...
    "AyPositive", "AyNegative"];
assert(isstruct(ggv) && all(isfield(ggv, required)), ...
    "FSAE:QuasiStatic:QuasiStaticGGVContract", ...
    "Saved GGV is missing required speed or acceleration-boundary fields.");
ggv.Speed = double(ggv.Speed(:));
ggv.LateralFraction = double(ggv.LateralFraction(:).');
ggv.AyPositive = double(ggv.AyPositive(:));
ggv.AyNegative = double(ggv.AyNegative(:));
ggv.AxMax = double(ggv.AxMax);
ggv.AxMin = double(ggv.AxMin);
speedCount = numel(ggv.Speed);
fractionCount = numel(ggv.LateralFraction);
assert(speedCount >= 2 && all(isfinite(ggv.Speed)) && ...
    all(diff(ggv.Speed) > 0), "FSAE:QuasiStatic:QuasiStaticGGVSpeed", ...
    "GGV.Speed must contain at least two finite, strictly increasing values.");
assert(numel(ggv.AyPositive) == speedCount && ...
    numel(ggv.AyNegative) == speedCount && ...
    isequal(size(ggv.AxMax), [speedCount, fractionCount]) && ...
    isequal(size(ggv.AxMin), [speedCount, fractionCount]), ...
    "FSAE:QuasiStatic:QuasiStaticGGVSize", ...
    "GGV acceleration arrays do not match Speed and LateralFraction.");
assert(all(isfinite(ggv.AyPositive)) && all(isfinite(ggv.AyNegative)) && ...
    all(isfinite(ggv.AxMax), "all") && all(isfinite(ggv.AxMin), "all"), ...
    "FSAE:QuasiStatic:QuasiStaticGGVFinite", ...
    "GGV contains nonfinite acceleration limits.");
if isfield(ggv, "Diagnostics") && ...
        isfield(ggv.Diagnostics, "InvalidPointCount")
    assert(ggv.Diagnostics.InvalidPointCount == 0, ...
        "FSAE:QuasiStatic:QuasiStaticGGVInvalidPoints", ...
        "GGV reports %d invalid boundary points.", ...
        ggv.Diagnostics.InvalidPointCount);
end
end

function [scenario, canonicalEvent, defaults] = createEventScenario( ...
        eventName, sampleDistance)
eventKey = lower(strtrim(string(eventName)));
switch eventKey
    case {"acceleration", "linearacceleration", "直线加速"}
        canonicalEvent = "Acceleration";
        if isnan(sampleDistance), sampleDistance = 0.5; end
        scenario = createPathTrackingAccelerationScenario( ...
            TargetSpeed = 30.0, SampleDistance = sampleDistance);
        defaults = struct("FinalSpeed", Inf);
    case {"skidpad", "figureeight", "八字绕环"}
        canonicalEvent = "Skidpad";
        if isnan(sampleDistance), sampleDistance = 0.2; end
        scenario = createPathTrackingSkidpadScenario( ...
            TargetSpeed = 8.0, SampleDistance = sampleDistance);
        defaults = struct("FinalSpeed", 0.0);
    case {"autocross", "高速避障"}
        canonicalEvent = "Autocross";
        if isnan(sampleDistance), sampleDistance = 0.5; end
        scenario = createPathTrackingAutocrossScenario( ...
            TargetSpeed = 12.0, SampleDistance = sampleDistance);
        defaults = struct("FinalSpeed", 0.0);
    otherwise
        error("FSAE:QuasiStatic:QuasiStaticEvent", ...
            "eventName must be acceleration, skidpad or autocross.");
end
end

function figureHandle = plotQuasiStaticResult( ...
        track, profile, metric, eventName, visibility)
figureHandle = figure( ...
    "Name", "Quasi-static lap-time result", ...
    "Color", "white", "Visible", visibility, ...
    "Position", [120, 80, 980, 760]);
ax = axes(figureHandle);
scatter(ax, track.X, track.Y, 12, profile.Speed, "filled");
axis(ax, "equal"); grid(ax, "on"); box(ax, "on");
xlabel(ax, "X (m)"); ylabel(ax, "Y (m)");
title(ax, sprintf("%s quasi-static speed map: %.3f s", ...
    eventName, metric.Time));
colorbarHandle = colorbar(ax);
colorbarHandle.Label.String = "Speed (m/s)";
end

function reason = firstFailureReason(profile, metric)
reason = string(profile.FailureReason);
if strlength(reason) == 0
    reason = string(metric.FailureReason);
end
if strlength(reason) == 0
    reason = "unknown validation failure";
end
end
