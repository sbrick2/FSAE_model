%PLOTCURRENTGGV3D 生成并绘制当前参数对应的三维 GGV 包络。
%   生成并绘制当前整车参数对应的三维 GGV 包络。该文件是脚本，可点击“运行”或
%   执行 run('plotCurrentGGV3D.m')；结束后工作区保留 ggv、config、speedGrid。
%   速度网格固定为 0:1:SpeedMax，各速度层使用进程池并行计算。
%
%   用户配置字段：SpeedMax、LateralPointCount、TireModel（MF62/TTCMap）、
%   VehicleDynamicsModel（7DOF/10DOF）、AllocationMode（FAST/LP）、
%   EnvelopePointCount、EnvelopeLoadPointCount、
%   MaxLateralAcceleration、MaxLongitudinalAcceleration、
%   RoadGripScale、RoadMuLimit、SteeringAngle、
%   MaxBisectionIterations、LongitudinalBracketPointCount、
%   MaxSearchExpansionCount、ParallelPoolType、ParallelWorkerCount、
%   Visible、OutputFolder、SaveMat。
%   Visible 可选 "on"/"off"；ParallelWorkerCount=0 时使用默认并行池规模；
%   OutputFolder 为空时保存到 results/quasi_static/ggv。
%
%   脚本读取 data/VehicleData.sldd 当前值；必需参数仍为 NaN 时会直接报错，
%   不会用虚构默认值替代。

% ------------------------- USER CONFIGURATION -------------------------
SpeedMax = 32;                       % m/s; speed spacing remains 1 m/s
LateralPointCount = 21;
TireModel = "MF62";
VehicleDynamicsModel = "7DOF";       % "7DOF" or "10DOF"
AllocationMode = "FAST";
EnvelopePointCount = 32;
EnvelopeLoadPointCount = 81;          % normal-load samples for MF62 lookup
RoadGripScale = 1.0;                  % 1 = TTC/MF62 reference surface
RoadMuLimit = Inf;                    % Inf = no additional absolute mu cap
SteeringAngle = 0.0;                  % rad; fixed front-wheel angle for GGV
MaxLateralAcceleration = 20;         % m/s^2, initial adaptive search span
MaxLongitudinalAcceleration = 20;    % m/s^2, initial adaptive search span
MaxBisectionIterations = 24;
LongitudinalBracketPointCount = 33;
MaxSearchExpansionCount = 3;
ParallelPoolType = "Processes";        % "Threads" preferred; "Processes" fallback
ParallelWorkerCount = 0;             % 0 -> default pool size
Visible = "on";                       % "on" or "off"
OutputFolder = "";                   % empty -> results/quasi_static/ggv
SaveMat = true;
% For a quick test, temporarily use SpeedMax=2, LateralPointCount=3 and
% MaxBisectionIterations=3. Restore the values above for the final plot.
% -----------------------------------------------------------------------

assert(isscalar(SpeedMax) && isfinite(SpeedMax) && SpeedMax >= 1 && ...
    SpeedMax == floor(SpeedMax), "FSAE:QuasiStatic:InvalidSpeedMax", ...
    "SpeedMax must be a positive integer in m/s.");
assert(isscalar(LateralPointCount) && LateralPointCount >= 3 && ...
    LateralPointCount == floor(LateralPointCount), ...
    "FSAE:QuasiStatic:InvalidLateralPointCount", ...
    "LateralPointCount must be an integer >= 3.");
assert(isscalar(EnvelopeLoadPointCount) && EnvelopeLoadPointCount >= 17 && ...
    EnvelopeLoadPointCount == floor(EnvelopeLoadPointCount), ...
    "FSAE:QuasiStatic:InvalidEnvelopeLoadPointCount", ...
    "EnvelopeLoadPointCount must be an integer >= 17.");
assert(isscalar(ParallelWorkerCount) && isfinite(ParallelWorkerCount) && ...
    ParallelWorkerCount >= 0 && ParallelWorkerCount == floor(ParallelWorkerCount), ...
    "FSAE:QuasiStatic:InvalidParallelWorkerCount", ...
    "ParallelWorkerCount must be zero or a positive integer.");

projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
addpath(fullfile(projectRoot, "scripts", "ggv"), "-begin");
addpath(fullfile(projectRoot, "scripts", "tire"), "-begin");

parameters = readCurrentParameters(projectRoot);
% The selected profile is cached by getSelectedTireProfile. Refresh it so
% a profile changed in the current MATLAB session cannot remain stale.
clear getSelectedTireProfile evaluateTireMF62 evaluateTireMF62Vector
profile = getSelectedTireProfile();

% The 1 m/s spacing is deliberately fixed here. Change SpeedMax only when
% a different upper speed limit is needed; do not replace this with a coarse
% linspace if the plot must retain 1 m/s resolution.
speedGrid = 0:1:SpeedMax;
config = createQuasiStaticConfiguration(parameters, ...
    TireProfile = profile, ...
    TireModel = TireModel, ...
    DynamicsModel = VehicleDynamicsModel, ...
    AllocationMode = AllocationMode, ...
    Environment = struct( ...
        "RoadGripScale", RoadGripScale, "RoadMuLimit", RoadMuLimit), ...
    EnvelopePointCount = EnvelopePointCount);
fprintf("GGV dynamics: %s (%s); tire model: %s; profile: %s\n", ...
    config.Vehicle.DynamicsModel, config.Metadata.QuasiStaticMethod, ...
    config.Tire.Model, string(profile.Name));

parallelPool = ensureParallelPool(ParallelPoolType, ParallelWorkerCount);
if isa(parallelPool, "parallel.ThreadPool")
    actualParallelPoolType = "Threads";
else
    actualParallelPoolType = "Processes";
end
speedCount = numel(speedGrid);
fprintf("GGV parallel mode: %s pool with %d workers across %d speed layers.\n", ...
    actualParallelPoolType, parallelPool.NumWorkers, speedCount);
fprintf("Precomputing MF62 combined-slip envelope lookup (%d load points)...\n", ...
    EnvelopeLoadPointCount);
lookupTimer = tic;
tireEnvelopeLookup = buildTireEnvelopeLookup(config, ...
    SpeedGrid = speedGrid, ...
    RoadGripScale = RoadGripScale, ...
    RoadMuLimit = RoadMuLimit, ...
    LoadPointCount = EnvelopeLoadPointCount, ...
    UseParallel = true);
lookupElapsedSeconds = toc(lookupTimer);
fprintf("Tire-envelope lookup complete in %.2f s; load range 0 to %.1f N.\n", ...
    lookupElapsedSeconds, tireEnvelopeLookup.NormalLoadUpper);

if isa(parallelPool, "parallel.ProcessPool")
    workerPathSetup = parallel.pool.Constant( ...
        @() setupWorkerPaths(projectRoot));
else
    workerPathSetup = parallel.pool.Constant(true);
end
tireEnvelopeLookupOnWorker = parallel.pool.Constant(tireEnvelopeLookup);
ggvLayers = cell(speedCount, 1);
progressQueue = parallel.pool.DataQueue;
printGGVProgress(struct("Reset", true, "Total", speedCount));
afterEach(progressQueue, @printGGVProgress);

parfor speedIndex = 1:speedCount
    assert(workerPathSetup.Value, ...
        "FSAE:QuasiStatic:WorkerPathSetup", ...
        "The process worker could not initialize the QuasiStatic paths.");
    layer = generateGGV(config, ...
        SpeedGrid = speedGrid(speedIndex), ...
        LateralPointCount = LateralPointCount, ...
        MaxLateralAcceleration = MaxLateralAcceleration, ...
        MaxLongitudinalAcceleration = MaxLongitudinalAcceleration, ...
        SteeringAngle = SteeringAngle, ...
        MaxBisectionIterations = MaxBisectionIterations, ...
        LongitudinalBracketPointCount = LongitudinalBracketPointCount, ...
        MaxSearchExpansionCount = MaxSearchExpansionCount, ...
        AllocationMode = AllocationMode, ...
        EnvelopeLoadPointCount = EnvelopeLoadPointCount, ...
        TireEnvelopeLookup = tireEnvelopeLookupOnWorker.Value);
    if isfield(layer, "Config")
        layer = rmfield(layer, "Config");
    end
    ggvLayers{speedIndex} = layer;
    send(progressQueue, struct( ...
        "Index", speedIndex, "Speed", speedGrid(speedIndex)));
end
drawnow;
ggv = mergeGGVLayers(ggvLayers, config);
delete(workerPathSetup);
delete(tireEnvelopeLookupOnWorker);
clear ggvLayers workerPathSetup tireEnvelopeLookupOnWorker progressQueue

assert(~ggv.Diagnostics.SearchLimitReached, ...
    "FSAE:QuasiStatic:GGVSearchSaturated", ...
    "GGV search reached its adaptive limit. Increase MaxSearchExpansionCount.");
assert(ggv.Diagnostics.InvalidPointCount == 0, ...
    "FSAE:QuasiStatic:InvalidGGVSurface", ...
    "GGV contains %d infeasible boundary points.", ...
    ggv.Diagnostics.InvalidPointCount);

% Traverse the drive boundary in increasing lateral acceleration, return
% along the braking boundary, and repeat the first point to close the loop.
% The two connector strips fill the previously open Ax ~= 0 region at the
% positive and negative lateral limits.
envelopeAx = [ggv.AxMax, fliplr(ggv.AxMin), ggv.AxMax(:, 1)];
envelopeAy = [ggv.LateralAcceleration, ...
    fliplr(ggv.LateralAcceleration), ggv.LateralAcceleration(:, 1)];
speedSurface = repmat(ggv.Speed, 1, size(envelopeAx, 2));

figureHandle = figure( ...
    "Color", "white", "Visible", Visible, "Position", [100, 100, 1100, 760]);
axesHandle = axes(figureHandle, "Position", [0.07, 0.12, 0.72, 0.80]);
surf(axesHandle, envelopeAx, envelopeAy, speedSurface, speedSurface, ...
    "FaceColor", "interp", ...
    "EdgeColor", [0.12, 0.12, 0.12], ...
    "EdgeAlpha", 0.38, ...
    "LineWidth", 0.25);
colormap(axesHandle, parula(256));
clim(axesHandle, [min(ggv.Speed), max(ggv.Speed)]);
colorbarHandle = colorbar(axesHandle);
colorbarHandle.Position = [0.84, 0.15, 0.025, 0.70];
colorbarHandle.Label.String = "Vehicle speed (m/s)";

grid(axesHandle, "on");
box(axesHandle, "on");
xlabel(axesHandle, "Long. accel. (m/s^2)");
ylabel(axesHandle, "Lat. accel. (m/s^2)");
zlabel(axesHandle, "Speed (m/s)");
title(axesHandle, "Current-parameter quasi-steady GGV map");
view(axesHandle, -42, 24);
axis(axesHandle, "tight");
pbaspect(axesHandle, [1.3, 1, 1.15]);
axesHandle.Projection = "perspective";
axesHandle.GridAlpha = 0.16;
axesHandle.MinorGridAlpha = 0.08;
axesHandle.FontName = "Helvetica";
axesHandle.FontSize = 11;
drawnow;

if strlength(OutputFolder) == 0
    OutputFolder = fullfile(projectRoot, "results", "quasi_static", "ggv");
end
if ~isfolder(OutputFolder)
    mkdir(OutputFolder);
end
fileTimestamp = string(datetime("now", TimeZone = "UTC", ...
    Format = "yyyyMMdd_HHmmss_SSS"));
runStem = "GGV_3D_current_1mps_" + fileTimestamp;
figPath = fullfile(OutputFolder, runStem + ".fig");
savefig(figureHandle, figPath);

if SaveMat
    matPath = fullfile(OutputFolder, runStem + ".mat");
    runMetadata = struct( ...
        "GeneratedAtUTC", string(datetime("now", TimeZone = "UTC")), ...
        "DictionaryPath", fullfile(projectRoot, "data", "VehicleData.sldd"), ...
        "TireProfileName", string(profile.Name), ...
        "TireProfileValidity", profile.Validity, ...
        "ParallelEnvironment", actualParallelPoolType, ...
        "RequestedParallelEnvironment", string(ParallelPoolType), ...
        "ParallelWorkerCount", parallelPool.NumWorkers, ...
        "ParallelDimension", "Speed layers", ...
        "TireEnvelopeMethod", string(tireEnvelopeLookup.Method), ...
        "TireEnvelopeLoadPointCount", tireEnvelopeLookup.LoadPointCount, ...
        "TireEnvelopeNormalLoadUpper", tireEnvelopeLookup.NormalLoadUpper, ...
        "TireEnvelopePrecomputeSeconds", lookupElapsedSeconds);
    save(char(matPath), "ggv", "config", "speedGrid", "runMetadata", "-v7.3");
end

fprintf("3-D GGV complete: %d speed points, 1 m/s spacing.\n", numel(speedGrid));
fprintf("Editable MATLAB figure: %s\n", figPath);

function pool = ensureParallelPool(poolType, workerCount)
poolType = upper(string(poolType));
assert(any(poolType == ["THREADS", "PROCESSES"]), ...
    "FSAE:QuasiStatic:InvalidParallelPoolType", ...
    "ParallelPoolType must be Threads or Processes.");
pool = gcp("nocreate");
if poolType == "THREADS"
    profileName = "Threads";
    expectedClass = "parallel.ThreadPool";
else
    profileName = "Processes";
    expectedClass = "parallel.ProcessPool";
end
if ~isempty(pool)
    if ~isa(pool, expectedClass)
        warning("FSAE:QuasiStatic:ExistingParallelPoolType", ...
            "Using the existing %s instead of the requested %s pool.", ...
            class(pool), profileName);
    end
    if workerCount > 0 && pool.NumWorkers ~= workerCount
        warning("FSAE:QuasiStatic:ExistingParallelPoolSize", ...
            "Using the existing %d-worker parallel pool instead of the requested %d workers.", ...
            pool.NumWorkers, workerCount);
    end
    return
end

if workerCount == 0
    try
        pool = parpool(profileName);
    catch exception
        if poolType ~= "THREADS"
            rethrow(exception)
        end
        warning("FSAE:QuasiStatic:ThreadPoolFallback", ...
            "Thread pool startup failed (%s). Falling back to Processes.", ...
            exception.message);
        pool = parpool("Processes");
    end
else
    try
        pool = parpool(profileName, workerCount);
    catch exception
        if poolType ~= "THREADS"
            rethrow(exception)
        end
        warning("FSAE:QuasiStatic:ThreadPoolFallback", ...
            "Thread pool startup failed (%s). Falling back to Processes.", ...
            exception.message);
        pool = parpool("Processes", workerCount);
    end
end
end

function ready = setupWorkerPaths(projectRoot)
addpath(fullfile(projectRoot, "scripts", "ggv"), "-begin");
addpath(fullfile(projectRoot, "scripts", "tire"), "-begin");
ready = true;
end

function printGGVProgress(message)
persistent completedCount totalCount
if isfield(message, "Reset") && message.Reset
    completedCount = 0;
    totalCount = message.Total;
    return
end
completedCount = completedCount + 1;
fprintf("GGV layer %d/%d complete: index %d, speed %.1f m/s.\n", ...
    completedCount, totalCount, message.Index, message.Speed);
end

function ggv = mergeGGVLayers(layers, config)
assert(~isempty(layers) && all(~cellfun(@isempty, layers)), ...
    "FSAE:QuasiStatic:MissingGGVLayer", ...
    "Every speed layer must return a GGV result before merging.");
ggv = layers{1};
sameFractionGrid = cellfun( ...
    @(layer) isequal(layer.LateralFraction, ggv.LateralFraction), layers);
assert(all(sameFractionGrid), "FSAE:QuasiStatic:GGVLayerContract", ...
    "All parallel GGV layers must use the same lateral-fraction grid.");

ggv.Speed = concatenateLayerField(layers, "Speed");
ggv.LateralAcceleration = concatenateLayerField( ...
    layers, "LateralAcceleration");
ggv.AyPositive = concatenateLayerField(layers, "AyPositive");
ggv.AyNegative = concatenateLayerField(layers, "AyNegative");
ggv.LateralReferenceAxPositive = concatenateLayerField( ...
    layers, "LateralReferenceAxPositive");
ggv.LateralReferenceAxNegative = concatenateLayerField( ...
    layers, "LateralReferenceAxNegative");
ggv.AxMax = concatenateLayerField(layers, "AxMax");
ggv.AxMin = concatenateLayerField(layers, "AxMin");
ggv.ActiveConstraintMax = concatenateLayerField( ...
    layers, "ActiveConstraintMax");
ggv.ActiveConstraintMin = concatenateLayerField( ...
    layers, "ActiveConstraintMin");
ggv.LateralLimitResults = concatenateLayerField( ...
    layers, "LateralLimitResults");
ggv.PointFeasible = concatenateLayerField(layers, "PointFeasible");
ggv.SpeedFeasible = concatenateLayerField(layers, "SpeedFeasible");
ggv.SearchSaturation.AyPositive = concatenateSaturationField( ...
    layers, "AyPositive");
ggv.SearchSaturation.AyNegative = concatenateSaturationField( ...
    layers, "AyNegative");
ggv.SearchSaturation.AxMax = concatenateSaturationField(layers, "AxMax");
ggv.SearchSaturation.AxMin = concatenateSaturationField(layers, "AxMin");

ggv.Config = config;
ggv.Diagnostics.SearchLimitReached = ...
    any(ggv.SearchSaturation.AyPositive) || ...
    any(ggv.SearchSaturation.AyNegative) || ...
    any(ggv.SearchSaturation.AxMax, "all") || ...
    any(ggv.SearchSaturation.AxMin, "all");
ggv.Diagnostics.InvalidPointCount = nnz(~ggv.PointFeasible);
ggv.Diagnostics.InvalidSpeedCount = nnz(~ggv.SpeedFeasible);
end

function output = concatenateLayerField(layers, fieldName)
values = cellfun(@(layer) layer.(char(fieldName)), layers, ...
    UniformOutput = false);
output = vertcat(values{:});
end

function output = concatenateSaturationField(layers, fieldName)
values = cellfun( ...
    @(layer) layer.SearchSaturation.(char(fieldName)), layers, ...
    UniformOutput = false);
output = vertcat(values{:});
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

% The QuasiStatic configuration defaults this optional limit to Inf. Adding it to
% the local copy also makes Brake.MaxTotalForce available as a scan path.
if ~isfield(parameters.Brake, "MaxTotalForce")
    parameters.Brake.MaxTotalForce = Inf;
end
end
