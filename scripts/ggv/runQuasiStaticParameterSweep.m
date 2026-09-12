function results = runQuasiStaticParameterSweep(baseConfig, sweep, options)
%RUNQuasiStaticPARAMETERSWEEP Run a reproducible Cartesian parameter sweep.
%   RESULTS includes parameter values, GGV options and failure reasons for
%   every combination. Failed cases are retained rather than zero-filled.

arguments
    baseConfig (1, 1) struct
    sweep table
    options.GenerateGGVOptions (1, 1) struct = struct( ...
        "SpeedGrid", [0, 10, 20], "LateralPointCount", 5, ...
        "MaxLateralAcceleration", 12.0, ...
        "MaxLongitudinalAcceleration", 12.0, ...
        "MaxBisectionIterations", 20)
    options.OutputFile (1, 1) string = ""
    options.RunMetadata (1, 1) struct = struct()
end

required = ["Name", "Path", "Values", "Unit", "Source"];
assert(all(ismember(required, string(sweep.Properties.VariableNames))), ...
    "FSAE:QuasiStatic:SweepContract", "Sweep table columns do not match the QuasiStatic contract.");
runMetadata = makeRunMetadata(options.RunMetadata);
valueCount = zeros(height(sweep), 1);
for index = 1:height(sweep)
    valueCount(index) = numel(sweep.Values{index});
end
combinationCount = prod(valueCount);
results = repmat(emptyResult(), combinationCount, 1);

for combination = 1:combinationCount
    indices = linearCombinationIndices(valueCount, combination);
    config = baseConfig;
    parameterValues = strings(height(sweep), 1);
    selectedValues = zeros(height(sweep), 1);
    parameterRecord = struct;
    for index = 1:height(sweep)
        value = sweep.Values{index}(indices(index));
        config = setNested(config, string(sweep.Path(index)), value);
        selectedValues(index) = value;
        parameterValues(index) = string(value);
        parameterRecord.(char(sweep.Name(index))) = value;
    end
    parameterTrace = sweep(:, ["Name", "Path", "Unit", "Source"]);
    parameterTrace.Value = selectedValues;
    results(combination).Index = combination;
    results(combination).Parameters = parameterRecord;
    results(combination).ParameterValues = strjoin(parameterValues, ", ");
    results(combination).ParameterTrace = parameterTrace;
    results(combination).TireModel = config.Tire.Model;
    results(combination).Variant = "QuasiStatic_SteadyState_" + config.Tire.Model;
    results(combination).SolverName = "QuasiStatic_" + config.Solver.AllocationMode + ...
        "_ForceAllocation";
    results(combination).SolverOptions = options.GenerateGGVOptions;
    results(combination).RunMetadata = runMetadata;
    try
        ggv = runGenerator(config, options.GenerateGGVOptions);
        assert(~ggv.Diagnostics.SearchLimitReached, ...
            "FSAE:QuasiStatic:GGVSearchSaturated", ...
            "GGV search reached its adaptive acceleration limit.");
        assert(ggv.Diagnostics.InvalidPointCount == 0, ...
            "FSAE:QuasiStatic:InvalidGGVSurface", ...
            "GGV contains %d infeasible boundary points.", ...
            ggv.Diagnostics.InvalidPointCount);
        results(combination).Success = true;
        results(combination).GGV = ggv;
        results(combination).PeakAy = max([ggv.AyPositive; ggv.AyNegative]);
        results(combination).PeakAx = max(ggv.AxMax, [], "all", "omitnan");
        results(combination).PeakBrake = min(ggv.AxMin, [], "all", "omitnan");
    catch exception
        results(combination).FailureReason = string(exception.identifier) + ": " + string(exception.message);
    end
end

if strlength(options.OutputFile) > 0
    save(options.OutputFile, "results", "sweep", "runMetadata", "-v7.3");
end
end

function result = emptyResult()
result = struct( ...
    "Index", 0, "Parameters", struct(), "ParameterValues", "", ...
    "ParameterTrace", table(), ...
    "TireModel", "", "Variant", "", "SolverName", "", ...
    "SolverOptions", struct(), "Success", false, ...
    "GGV", struct(), "PeakAy", NaN, "PeakAx", NaN, "PeakBrake", NaN, ...
    "FailureReason", "", "RunMetadata", struct());
end

function indices = linearCombinationIndices(counts, linearIndex)
indices = ones(size(counts));
remaining = linearIndex - 1;
for index = 1:numel(counts)
    indices(index) = mod(remaining, counts(index)) + 1;
    remaining = floor(remaining / counts(index));
end
end

function output = setNested(input, path, value)
output = input;
parts = split(path, ".");
assert(numel(parts) >= 2, "FSAE:QuasiStatic:SweepPath", ...
    "Sweep path must contain at least one nested field: %s.", path);
top = char(parts(1));
assert(isfield(output, top), "FSAE:QuasiStatic:SweepPath", ...
    "Unknown sweep path: %s.", path);
output.(top) = setNestedPart(output.(top), parts(2:end), value, path);
end

function cursor = setNestedPart(cursor, parts, value, fullPath)
fieldName = char(parts(1));
assert(isstruct(cursor) && isfield(cursor, fieldName), ...
    "FSAE:QuasiStatic:SweepPath", "Unknown sweep path: %s.", fullPath);
if numel(parts) > 1
    cursor.(fieldName) = setNestedPart( ...
        cursor.(fieldName), parts(2:end), value, fullPath);
else
    cursor.(fieldName) = value;
end
end

function ggv = runGenerator(config, options)
ggv = generateGGV(config, ...
    SpeedGrid = options.SpeedGrid, ...
    LateralPointCount = options.LateralPointCount, ...
    MaxLateralAcceleration = options.MaxLateralAcceleration, ...
    MaxLongitudinalAcceleration = options.MaxLongitudinalAcceleration, ...
    MaxBisectionIterations = options.MaxBisectionIterations, ...
    AllocationMode = config.Solver.AllocationMode);
end

function metadata = makeRunMetadata(overrides)
projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
gitCommand = sprintf('git -C "%s" rev-parse HEAD', projectRoot);
[commitStatus, commitText] = system(gitCommand);
if commitStatus == 0
    gitCommit = strip(string(commitText));
else
    gitCommit = "UNAVAILABLE";
end
statusCommand = sprintf('git -C "%s" status --porcelain', projectRoot);
[statusCode, statusText] = system(statusCommand);
metadata = struct( ...
    "MATLABVersion", string(version), ...
    "MATLABRelease", string(version("-release")), ...
    "GitCommit", gitCommit, ...
    "GitDirty", statusCode == 0 && strlength(strip(string(statusText))) > 0, ...
    "GitStatusAvailable", statusCode == 0, ...
    "Scenario", "QuasiStatic_GGV_PARAMETER_SWEEP", ...
    "GeneratedAtUTC", string(datetime("now", TimeZone = "UTC")));
fields = fieldnames(overrides);
for index = 1:numel(fields)
    metadata.(fields{index}) = overrides.(fields{index});
end
end
