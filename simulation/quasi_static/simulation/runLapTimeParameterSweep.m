%RUNLAPTIMEPARAMETERSWEEP 并行执行单参数或双参数准静态圈速扫描。
%   该文件是脚本，可点击“运行”或执行 run('runLapTimeParameterSweep.m')。
%   脚本读取 data/VehicleData.sldd 当前整车参数，生成选定赛道，并用 parfor
%   计算所有参数组合；结束后工作区保留 results、scanConfig 和 scanSummary。
%   每个完成工况按以下格式输出：
%
%       current/total-parameter1-value1-parameter2-value2-eventTime
%
%   results 为结构体数组；失败工况仍会保留，Success=false，错误原因写入
%   FailureReason。可保存 MAT、CSV 和汇总图到
%   results/quasi_static/parameter_sweep/<event>。
%   用户配置说明：
%   1) scanMode 可选 "single" 或 "double"；eventName 可选 acceleration、
%      skidpad、autocross，也接受“直线加速”“八字绕环”“高速避障”。
%   2) parameter1Path/parameter2Path 为 QuasiStatic 规范化字段。常用字段包括
%      Aero.CdA、Aero.ClAFront、Aero.ClARear、Vehicle.Mass、
%      Powertrain.MotorTorqueLimit、Battery.PowerLimitDrive；同时设置对应 Unit。
%   3) Start/Stop/Step 按 MATLAB 的 Start:Step:Stop 生成扫描点；单参数扫描时
%      parameter2 配置不参与计算。
%   4) GGV 分辨率字段以 ggv 开头；TireModel、VehicleDynamicsModel、
%      allocationMode 选择轮胎、车辆动力学和分配算法。
%   5) showSummaryPlot 控制汇总图，saveResults 控制保存，storeCaseDetails
%      控制是否在每个工况中保留精简 GGV 与速度剖面。

scanMode = "double";                 % "single" or "double"
eventName = "acceleration";             % acceleration | skidpad | autocross

parameter1Path = "Vehicle.Mass";         % Parameter 1 path; see examples above
parameter1Unit = "kg";
parameter1Start = 280;              % Parameter 1 lower bound
parameter1Stop = 320;               % Parameter 1 upper bound
parameter1Step = 1;               % Parameter 1 step

parameter2Path = "Powertrain.GearRatio";    % Used only when scanMode = "double"
parameter2Unit = "1";
parameter2Start = 11;              % Parameter 2 lower bound
parameter2Stop = 15;               % Parameter 2 upper bound
parameter2Step = 0.5;               % Parameter 2 step

% The ranges above are provisional design-study ranges, not measured
% vehicle data. Replace them with the range justified by your test/CAD data.

% GGV/speed-profile resolution. The first script is fixed at 1 m/s; this
% sweep also defaults to 1 m/s so that every event sees the same speed grid.
ggvSpeedMax = 40;                    % m/s
ggvSpeedStep = 1;                    % m/s
ggvLateralPointCount = 9;
ggvEnvelopePointCount = 24;
ggvEnvelopeLoadPointCount = 81;
ggvMaxBisectionIterations = 16;
ggvMaxLateralAcceleration = 20;
ggvMaxLongitudinalAcceleration = 20;
ggvLongitudinalBracketPointCount = 33;
ggvMaxSearchExpansionCount = 3;
tireModel = "MF62";
vehicleDynamicsModel = "7DOF";       % "7DOF" or "10DOF"
allocationMode = "FAST";             % "FAST" for scans, "LP" for validation

showSummaryPlot = true;
saveResults = true;
storeCaseDetails = true;             % Store compact GGV and speed profile per case

% GUI launches provide a validated override struct and a client-side
% progress callback. Direct script execution keeps the defaults above.
guiProgressCallback = [];
if exist("FSAE_GUI_SWEEP_CONFIG", "var")
    guiConfig = FSAE_GUI_SWEEP_CONFIG;
    requiredFields = [ ...
        "ScanMode", "EventName", ...
        "Parameter1Path", "Parameter1Unit", ...
        "Parameter1Start", "Parameter1Stop", "Parameter1Step", ...
        "Parameter2Path", "Parameter2Unit", ...
        "Parameter2Start", "Parameter2Stop", "Parameter2Step", ...
        "ShowSummaryPlot", "SaveResults", "StoreCaseDetails"];
    assert(all(isfield(guiConfig, cellstr(requiredFields))), ...
        "FSAE:QuasiStatic:GuiSweepContract", ...
        "GUI parameter sweep configuration is incomplete.");
    scanMode = lower(string(guiConfig.ScanMode));
    eventName = lower(string(guiConfig.EventName));
    parameter1Path = string(guiConfig.Parameter1Path);
    parameter1Unit = string(guiConfig.Parameter1Unit);
    parameter1Start = double(guiConfig.Parameter1Start);
    parameter1Stop = double(guiConfig.Parameter1Stop);
    parameter1Step = double(guiConfig.Parameter1Step);
    parameter2Path = string(guiConfig.Parameter2Path);
    parameter2Unit = string(guiConfig.Parameter2Unit);
    parameter2Start = double(guiConfig.Parameter2Start);
    parameter2Stop = double(guiConfig.Parameter2Stop);
    parameter2Step = double(guiConfig.Parameter2Step);
    showSummaryPlot = logical(guiConfig.ShowSummaryPlot);
    saveResults = logical(guiConfig.SaveResults);
    storeCaseDetails = logical(guiConfig.StoreCaseDetails);
end
if exist("FSAE_GUI_SWEEP_PROGRESS", "var") && ...
        isa(FSAE_GUI_SWEEP_PROGRESS, "function_handle")
    guiProgressCallback = FSAE_GUI_SWEEP_PROGRESS;
end

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(fullfile(projectRoot, "scripts", "ggv"), "-begin");
addpath(fullfile(projectRoot, "scripts", "lap_time"), "-begin");
addpath(fullfile(projectRoot, "scripts", "tire"), "-begin");
addpath(fullfile(projectRoot, "scripts", "track"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Acceleration"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Skidpad"), "-begin");
addpath(fullfile(projectRoot, "scenarios", "Autocross"), "-begin");

parameters = readCurrentParameters(projectRoot);
clear getSelectedTireProfile evaluateTireMF62 evaluateTireMF62Vector
profile = getSelectedTireProfile();
validateScanMode(scanMode);
parameter1Values = makeScanValues(parameters, parameter1Path, ...
    parameter1Start, parameter1Stop, parameter1Step);

if scanMode == "double"
    assert(parameter1Path ~= parameter2Path, "FSAE:QuasiStatic:DuplicateScanPath", ...
        "Double-parameter scans require two different parameter paths.");
    parameter2Values = makeScanValues(parameters, parameter2Path, ...
        parameter2Start, parameter2Stop, parameter2Step);
else
    parameter2Path = "none";
    parameter2Unit = "1";
    parameter2Values = NaN;
end

% Validate all non-scanned required fields before starting workers. The
% first values are only used for this check; every worker rebuilds its own
% normalized config from raw dictionary parameters, including derived
% geometry such as CGToRearAxle.
nominalParameters = setParameter(parameters, parameter1Path, parameter1Values(1));
if scanMode == "double"
    nominalParameters = setParameter(nominalParameters, ...
        parameter2Path, parameter2Values(1));
end
createQuasiStaticConfiguration(nominalParameters, ...
    TireProfile = profile, TireModel = tireModel, ...
    DynamicsModel = vehicleDynamicsModel, ...
    AllocationMode = allocationMode, ...
    EnvelopePointCount = ggvEnvelopePointCount);

[track, profileOptions, eventName] = createEventTrack(eventName);
speedGrid = createSweepValues(0.0, ggvSpeedMax, ggvSpeedStep);
assert(numel(speedGrid) >= 2, ...
    "FSAE:QuasiStatic:InvalidSweepSpeedGrid", ...
    "The sweep speed grid must contain at least two points.");
ggvOptions = struct( ...
    "SpeedGrid", speedGrid, ...
    "LateralPointCount", ggvLateralPointCount, ...
    "EnvelopePointCount", ggvEnvelopePointCount, ...
    "EnvelopeLoadPointCount", ggvEnvelopeLoadPointCount, ...
    "MaxBisectionIterations", ggvMaxBisectionIterations, ...
    "MaxLateralAcceleration", ggvMaxLateralAcceleration, ...
    "MaxLongitudinalAcceleration", ggvMaxLongitudinalAcceleration, ...
    "LongitudinalBracketPointCount", ggvLongitudinalBracketPointCount, ...
    "MaxSearchExpansionCount", ggvMaxSearchExpansionCount, ...
    "TireModel", tireModel, ...
    "DynamicsModel", vehicleDynamicsModel, ...
    "AllocationMode", allocationMode);
runMetadata = makeRunMetadata(eventName);

if scanMode == "single"
    totalCount = numel(parameter1Values);
else
    totalCount = numel(parameter1Values) * numel(parameter2Values);
end

pool = gcp("nocreate");
if isempty(pool)
    parpool("local");
end

results = repmat(emptySweepResult(), totalCount, 1);
progressQueue = parallel.pool.DataQueue;
dispatchSweepProgress(struct("Reset", true, "Total", totalCount), ...
    guiProgressCallback);
afterEach(progressQueue, ...
    @(message) dispatchSweepProgress(message, guiProgressCallback));

fprintf("Starting %d parallel %s-scan cases for %s.\n", ...
    totalCount, scanMode, eventName);
parfor caseIndex = 1:totalCount
    index1 = mod(caseIndex - 1, numel(parameter1Values)) + 1;
    if scanMode == "double"
        index2 = floor((caseIndex - 1) / numel(parameter1Values)) + 1;
        value2 = parameter2Values(index2); %#ok<PFBNS>
    else
        value2 = NaN;
    end
    value1 = parameter1Values(index1);
    caseSpec = struct( ...
        "Index", caseIndex, ...
        "Parameter1Path", parameter1Path, "Parameter1Unit", parameter1Unit, ...
        "Value1", value1, ...
        "Parameter2Path", parameter2Path, "Parameter2Unit", parameter2Unit, ...
        "Value2", value2, ...
        "ScanMode", scanMode, "EventName", eventName, ...
        "ProfileOptions", profileOptions, ...
        "StoreCaseDetails", storeCaseDetails);
    caseResult = runOneCase(parameters, profile, track, caseSpec, ggvOptions);
    results(caseIndex) = caseResult;
    send(progressQueue, struct( ...
        "Parameter1", parameter1Path, "Value1", value1, ...
        "Parameter2", parameter2Path, "Value2", value2, ...
        "LapTime", caseResult.LapTime));
end

% DataQueue callbacks are client-side. MATLAB drains the queue as parfor
% completes; drawnow services any final callback before plotting/saving.
drawnow;

success = [results.Success];
fprintf("Lap-time sweep complete: %d/%d successful.\n", nnz(success), totalCount);
dispatchSweepProgress(struct( ...
    "Complete", true, "Total", totalCount, ...
    "SuccessCount", nnz(success), ...
    "FailureCount", totalCount - nnz(success)), guiProgressCallback);
scanSummary = summarizeSweepResults(results);
if scanSummary.SuccessCount > 0
    fprintf("Best case: index %d, %s=%.6g, %s=%.6g, event time=%.6f s.\n", ...
        scanSummary.BestIndex, parameter1Path, ...
        scanSummary.BestParameter1Value, parameter2Path, ...
        scanSummary.BestParameter2Value, scanSummary.BestLapTime);
end

outputFolder = fullfile(projectRoot, "results", "quasi_static", ...
    "parameter_sweep", lower(eventName));
if saveResults && ~isfolder(outputFolder)
    mkdir(outputFolder);
end
fileTimestamp = string(datetime("now", TimeZone = "UTC", ...
    Format = "yyyyMMdd_HHmmss_SSS"));
runStem = "QuasiStatic_LapTimeSweep_" + makeSafeName(eventName) + "_" + ...
    makeSafeName(parameter1Path);
if scanMode == "double"
    runStem = runStem + "_" + makeSafeName(parameter2Path);
end
runStem = runStem + "_" + fileTimestamp;

scanConfig = struct( ...
    "Mode", scanMode, "Event", eventName, ...
    "Parameter1", parameter1Path, "Parameter1Unit", parameter1Unit, ...
    "Parameter1Values", parameter1Values, ...
    "Parameter2", parameter2Path, "Parameter2Unit", parameter2Unit, ...
    "Parameter2Values", parameter2Values, ...
    "GGVOptions", ggvOptions, ...
    "ProfileOptions", profileOptions, ...
    "TrackMetadata", makeTrackMetadata(track), ...
    "TireProfileName", string(profile.Name), ...
    "TireProfileValidity", profile.Validity, ...
    "StoreCaseDetails", storeCaseDetails, ...
    "RunStem", runStem);
if showSummaryPlot
    summaryFigure = plotSweepSummary(results, parameter1Values, parameter2Values, ...
        parameter1Path, parameter2Path, scanMode, eventName);
    if saveResults
        figurePath = fullfile(outputFolder, runStem + ".fig");
        savefig(summaryFigure, figurePath);
        fprintf("Sweep figure: %s\n", figurePath);
    end
end
if saveResults
    outputPath = fullfile(outputFolder, runStem + ".mat");
    csvPath = fullfile(outputFolder, runStem + ".csv");
    baseParameters = parameters;
    tireProfile = profile;
    save(char(outputPath), "results", "scanConfig", "baseParameters", ...
        "tireProfile", "runMetadata", "scanSummary", "-v7.3");
    resultTable = sweepResultsTable(results);
    writetable(resultTable, csvPath);
    fprintf("Sweep results: %s\n", outputPath);
    fprintf("Sweep table: %s\n", csvPath);
end

function progress = dispatchSweepProgress(message, callback)
persistent completedCount totalCountPersistent
if isfield(message, "Reset") && message.Reset
    completedCount = 0;
    totalCountPersistent = message.Total;
    progress = struct( ...
        "Reset", true, "Complete", false, ...
        "Completed", 0, "Total", totalCountPersistent, ...
        "Fraction", 0.0, "Parameter1", "", "Value1", NaN, ...
        "Parameter2", "none", "Value2", NaN, "LapTime", NaN, ...
        "SuccessCount", 0, "FailureCount", 0);
    notifySweepProgress(callback, progress);
    return
end
if isfield(message, "Complete") && message.Complete
    completedCount = totalCountPersistent;
    progress = struct( ...
        "Reset", false, "Complete", true, ...
        "Completed", completedCount, "Total", totalCountPersistent, ...
        "Fraction", 1.0, "Parameter1", "", "Value1", NaN, ...
        "Parameter2", "none", "Value2", NaN, "LapTime", NaN, ...
        "SuccessCount", message.SuccessCount, ...
        "FailureCount", message.FailureCount);
    notifySweepProgress(callback, progress);
    return
end
completedCount = completedCount + 1;
fprintf("%d/%d-%s-%.6g-%s-%.6g-%.6f\n", ...
    completedCount, totalCountPersistent, message.Parameter1, message.Value1, ...
    message.Parameter2, message.Value2, message.LapTime);
progress = struct( ...
    "Reset", false, "Complete", false, ...
    "Completed", completedCount, "Total", totalCountPersistent, ...
    "Fraction", completedCount / totalCountPersistent, ...
    "Parameter1", string(message.Parameter1), "Value1", message.Value1, ...
    "Parameter2", string(message.Parameter2), "Value2", message.Value2, ...
    "LapTime", message.LapTime, "SuccessCount", 0, "FailureCount", 0);
notifySweepProgress(callback, progress);
end

function notifySweepProgress(callback, progress)
if ~isempty(callback)
    callback(progress);
end
end

function result = runOneCase(parameters, profile, track, caseSpec, ggvOptions)
result = emptySweepResult();
result.Index = caseSpec.Index;
result.Parameter1Name = caseSpec.Parameter1Path;
result.Parameter1Unit = caseSpec.Parameter1Unit;
result.Parameter1Value = caseSpec.Value1;
result.Parameter2Name = caseSpec.Parameter2Path;
result.Parameter2Unit = caseSpec.Parameter2Unit;
result.Parameter2Value = caseSpec.Value2;
try
    caseParameters = setParameter( ...
        parameters, caseSpec.Parameter1Path, caseSpec.Value1);
    if caseSpec.ScanMode == "double"
        caseParameters = setParameter( ...
            caseParameters, caseSpec.Parameter2Path, caseSpec.Value2);
    end
    config = createQuasiStaticConfiguration(caseParameters, ...
        TireProfile = profile, TireModel = ggvOptions.TireModel, ...
        DynamicsModel = ggvOptions.DynamicsModel, ...
        AllocationMode = ggvOptions.AllocationMode, ...
        EnvelopePointCount = ggvOptions.EnvelopePointCount);
    ggv = generateGGV(config, ...
        SpeedGrid = ggvOptions.SpeedGrid, ...
        LateralPointCount = ggvOptions.LateralPointCount, ...
        MaxLateralAcceleration = ggvOptions.MaxLateralAcceleration, ...
        MaxLongitudinalAcceleration = ggvOptions.MaxLongitudinalAcceleration, ...
        MaxBisectionIterations = ggvOptions.MaxBisectionIterations, ...
        LongitudinalBracketPointCount = ...
            ggvOptions.LongitudinalBracketPointCount, ...
        MaxSearchExpansionCount = ggvOptions.MaxSearchExpansionCount, ...
        EnvelopeLoadPointCount = ggvOptions.EnvelopeLoadPointCount, ...
        AllocationMode = ggvOptions.AllocationMode);
    if caseSpec.StoreCaseDetails
        result.GGV = compactGGV(ggv);
    end
    result.SearchLimitReached = ggv.Diagnostics.SearchLimitReached;
    result.InvalidGGVPointCount = ggv.Diagnostics.InvalidPointCount;
    if result.SearchLimitReached
        result.FailureReason = ...
            "GGV search reached an acceleration limit before finding a physical boundary.";
        return
    end
    if result.InvalidGGVPointCount > 0
        result.FailureReason = sprintf( ...
            "GGV contains %d infeasible boundary points.", ...
            result.InvalidGGVPointCount);
        return
    end

    profileOptions = caseSpec.ProfileOptions;
    speedProfile = calculateSpeedProfile(track, ggv, ...
        UseReferenceSpeed = profileOptions.UseReferenceSpeed, ...
        InitialSpeed = profileOptions.InitialSpeed, ...
        FinalSpeed = profileOptions.FinalSpeed, ...
        PassCount = profileOptions.PassCount, ...
        MinimumSpeed = profileOptions.MinimumSpeed, ...
        MaximumSpeed = profileOptions.MaximumSpeed);
    if caseSpec.StoreCaseDetails
        result.SpeedProfile = speedProfile;
    end
    metric = calculateEventMetric(caseSpec.EventName, speedProfile, track);
    result.MetricName = metric.Name;
    result.LapTime = metric.Time;
    result.FullCourseTime = speedProfile.LapTime;
    result.RightTimedLap = metric.RightTimedLap;
    result.LeftTimedLap = metric.LeftTimedLap;
    result.Distance = metric.Distance;
    result.Success = metric.IsValid && isfinite(result.LapTime) && ...
        result.LapTime > 0.0;
    if ~result.Success
        result.FailureReason = metric.FailureReason;
    end
catch exception
    result.FailureReason = string(exception.identifier) + ": " + string(exception.message);
end
end

function [track, profileOptions, canonicalEvent] = createEventTrack(eventName)
eventKey = lower(strtrim(string(eventName)));
switch eventKey
    case {"acceleration", "linearacceleration", "直线加速"}
        canonicalEvent = "acceleration";
        scenario = createPathTrackingAccelerationScenario(TargetSpeed = 30.0, SampleDistance = 0.5);
        profileOptions = struct( ...
            "UseReferenceSpeed", false, "InitialSpeed", 0.0, ...
            "FinalSpeed", Inf, "PassCount", 4, "MinimumSpeed", 0.0, ...
            "MaximumSpeed", Inf);
    case {"skidpad", "figureeight", "八字绕环"}
        canonicalEvent = "skidpad";
        scenario = createPathTrackingSkidpadScenario(TargetSpeed = 8.0, SampleDistance = 0.2);
        profileOptions = struct( ...
            "UseReferenceSpeed", false, "InitialSpeed", 0.0, ...
            "FinalSpeed", 0.0, "PassCount", 4, "MinimumSpeed", 0.0, ...
            "MaximumSpeed", Inf);
    case {"autocross", "高速避障"}
        canonicalEvent = "autocross";
        scenario = createPathTrackingAutocrossScenario(TargetSpeed = 12.0, SampleDistance = 0.5);
        profileOptions = struct( ...
            "UseReferenceSpeed", false, "InitialSpeed", 0.0, ...
            "FinalSpeed", 0.0, "PassCount", 4, "MinimumSpeed", 0.0, ...
            "MaximumSpeed", Inf);
    otherwise
        error("FSAE:QuasiStatic:UnknownLapTimeEvent", ...
            "eventName must be acceleration, skidpad or autocross.");
end
track = scenario.Track;
end

function values = makeScanValues(parameters, path, startValue, stopValue, stepValue)
currentValue = getParameter(parameters, path);
assert(isscalar(currentValue) && isnumeric(currentValue) && isreal(currentValue), ...
    "FSAE:QuasiStatic:InvalidScanPath", ...
    "Scan path %s must resolve to a real numeric scalar.", path);
values = createSweepValues(startValue, stopValue, stepValue);
end

function validateScanMode(scanMode)
assert(any(scanMode == ["single", "double"]), "FSAE:QuasiStatic:ScanMode", ...
    "scanMode must be single or double.");
end

function value = getParameter(parameters, path)
parts = split(string(path), ".");
cursor = parameters;
for index = 1:numel(parts)
    assert(isstruct(cursor) && isfield(cursor, char(parts(index))), ...
        "FSAE:QuasiStatic:UnknownScanPath", "Unknown scan path: %s", path);
    cursor = cursor.(char(parts(index)));
end
if isstruct(cursor) && isfield(cursor, "Value")
    cursor = cursor.Value;
end
value = cursor;
end

function parameters = setParameter(parameters, path, value)
parts = split(string(path), ".");
assert(numel(parts) >= 2, "FSAE:QuasiStatic:ScanPath", ...
    "Scan path must contain a group and field, for example Aero.CdA.");
topName = char(parts(1));
assert(isfield(parameters, topName), "FSAE:QuasiStatic:UnknownScanPath", ...
    "Unknown scan group: %s", path);
parameters.(topName) = setParameterPart( ...
    parameters.(topName), parts(2:end), value, path);
end

function cursor = setParameterPart(cursor, parts, value, fullPath)
fieldName = char(parts(1));
assert(isstruct(cursor) && isfield(cursor, fieldName), ...
    "FSAE:QuasiStatic:UnknownScanPath", "Unknown scan path: %s", fullPath);
if numel(parts) > 1
    cursor.(fieldName) = setParameterPart( ...
        cursor.(fieldName), parts(2:end), value, fullPath);
    return
end
current = cursor.(fieldName);
if isstruct(current) && isfield(current, "Value")
    current.Value = value;
    cursor.(fieldName) = current;
else
    cursor.(fieldName) = value;
end
end

function result = emptySweepResult()
result = struct( ...
    "Index", 0, ...
    "Parameter1Name", "", "Parameter1Unit", "", "Parameter1Value", NaN, ...
    "Parameter2Name", "", "Parameter2Unit", "", "Parameter2Value", NaN, ...
    "MetricName", "", "LapTime", NaN, "FullCourseTime", NaN, ...
    "RightTimedLap", NaN, "LeftTimedLap", NaN, ...
    "Distance", NaN, "Success", false, ...
    "SearchLimitReached", false, "InvalidGGVPointCount", 0, ...
    "FailureReason", "", "GGV", struct(), "SpeedProfile", struct());
end

function summary = summarizeSweepResults(results)
success = [results.Success];
summary = struct( ...
    "TotalCount", numel(results), ...
    "SuccessCount", nnz(success), ...
    "FailureCount", nnz(~success), ...
    "BestIndex", NaN, ...
    "BestLapTime", NaN, ...
    "BestParameter1Value", NaN, ...
    "BestParameter2Value", NaN);
if ~any(success)
    return
end
successfulIndices = find(success);
[bestLapTime, bestOffset] = min([results(successfulIndices).LapTime]);
bestIndex = successfulIndices(bestOffset);
summary.BestIndex = results(bestIndex).Index;
summary.BestLapTime = bestLapTime;
summary.BestParameter1Value = results(bestIndex).Parameter1Value;
summary.BestParameter2Value = results(bestIndex).Parameter2Value;
end

function resultTable = sweepResultsTable(results)
scalarResults = rmfield(results, {'GGV', 'SpeedProfile'});
resultTable = struct2table(scalarResults);
end

function figureHandle = plotSweepSummary(results, parameter1Values, parameter2Values, ...
        parameter1Path, parameter2Path, scanMode, eventName)
success = [results.Success];
lapTimes = [results.LapTime];
lapTimes(~success) = NaN;
figureHandle = figure("Color", "white");
if scanMode == "single"
    plot(parameter1Values, lapTimes, "-o", "LineWidth", 1.2);
    grid on;
    xlabel(parameter1Path);
    ylabel("Event metric time (s)");
else
    lapMatrix = NaN(numel(parameter2Values), numel(parameter1Values));
    for index = 1:numel(results)
        i1 = find(parameter1Values == results(index).Parameter1Value, 1);
        i2 = find(parameter2Values == results(index).Parameter2Value, 1);
        if ~isempty(i1) && ~isempty(i2) && success(index)
            lapMatrix(i2, i1) = lapTimes(index);
        end
    end
    imagesc(parameter1Values, parameter2Values, lapMatrix);
    set(gca, "YDir", "normal");
    colorbar;
    xlabel(parameter1Path);
    ylabel(parameter2Path);
end
title("Parallel lap-time sweep: " + string(eventName));
drawnow;
end

function parameters = readCurrentParameters(projectRoot)
dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");
assert(isfile(dictionaryPath), "FSAE:QuasiStatic:MissingDictionary", ...
    "VehicleData.sldd was not found: %s", dictionaryPath);
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
groupNames = ["Vehicle", "Tire", "Aero", "Powertrain", "Battery", "Brake"];
parameters = struct();
for groupName = groupNames
    parameters.(char(groupName)) = getValue( ...
        getEntry(designData, char(groupName)));
end
try
    parameters.Vehicle10DOFSuspension = getValue( ...
        getEntry(designData, "Vehicle10DOFSuspension"));
catch exception
    if ~(contains(exception.identifier, "EntryNotFound") || ...
            contains(exception.message, "does not exist", IgnoreCase = true))
        rethrow(exception)
    end
end
if ~isfield(parameters.Brake, "MaxTotalForce")
    parameters.Brake.MaxTotalForce = Inf;
end
end

function name = makeSafeName(value)
name = regexprep(lower(char(value)), "[^a-z0-9_-]", "_");
end

function compact = compactGGV(ggv)
compact = ggv;
if isfield(compact, "Config")
    compact = rmfield(compact, "Config");
end
end

function metadata = makeTrackMetadata(track)
metadata = struct( ...
    "Name", string(track.Name), ...
    "IsClosed", logical(track.IsClosed), ...
    "Length", double(track.Length), ...
    "SampleDistance", double(track.SampleDistance), ...
    "SampleCount", numel(track.Curvature));
if isfield(track, "RuleSource")
    metadata.RuleSource = string(track.RuleSource);
end
end

function metadata = makeRunMetadata(eventName)
projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
[commitStatus, commitText] = system( ...
    sprintf('git -C "%s" rev-parse HEAD', projectRoot));
[statusCode, statusText] = system( ...
    sprintf('git -C "%s" status --porcelain', projectRoot));
if commitStatus == 0
    gitCommit = strip(string(commitText));
else
    gitCommit = "UNAVAILABLE";
end
metadata = struct( ...
    "MATLABVersion", string(version), ...
    "MATLABRelease", string(version("-release")), ...
    "GitCommit", gitCommit, ...
    "GitDirty", statusCode == 0 && strlength(strip(string(statusText))) > 0, ...
    "GitStatusAvailable", statusCode == 0, ...
    "Event", string(eventName), ...
    "GeneratedAtUTC", string(datetime("now", TimeZone = "UTC")));
end
