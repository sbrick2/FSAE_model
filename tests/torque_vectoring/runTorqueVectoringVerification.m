function report = runTorqueVectoringVerification(options)
%RUNTorqueVectoringVERIFICATION Run paired TTC/MF-based TV0 and TV1 simulations.
%   The returned report quantifies yaw stability, path tracking, energy,
%   tire utilization, saturation, and boundary-violation side effects.

arguments
    options.Events (1, :) string = "Skidpad"
    options.SaveSummary (1, 1) logical = true
    options.UseFastRestart (1, 1) logical = true
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "torque_vectoring"));
addpath(fullfile(projectRoot, "scripts", "simulation"));
addpath(fullfile(projectRoot, "scripts", "track"));
addpath(fullfile(projectRoot, "scenarios", "Skidpad"));
addpath(fullfile(projectRoot, "scenarios", "Autocross"));

installTorqueVectoringControlData();
parameters = loadTorqueVectoringVehicleParameters();
matrix = createTorqueVectoringScenarioMatrix(Events = options.Events);
inputs = repmat(Simulink.SimulationInput("FSAE_TorqueVectoring_ClosedLoop"), ...
    numel(matrix), 1);
for caseIndex = 1:numel(matrix)
    inputs(caseIndex) = createTorqueVectoringSimulationInput( ...
        matrix(caseIndex), parameters);
end

if options.UseFastRestart
    outputs = sim(inputs, UseFastRestart = "on");
else
    outputs = sim(inputs);
end

metricCells = cell(numel(matrix), 1);
for caseIndex = 1:numel(matrix)
    metricCells{caseIndex} = summarizeTorqueVectoringRun( ...
        outputs(caseIndex), matrix(caseIndex));
end
metrics = vertcat(metricCells{:});

pairIDs = unique([matrix.PairID], "stable");
comparisonCells = cell(numel(pairIDs), 1);
for pairIndex = 1:numel(pairIDs)
    mask = [metrics.PairID] == pairIDs(pairIndex);
    comparisonCells{pairIndex} = compareTorqueVectoringPair(metrics(mask));
end
comparisons = vertcat(comparisonCells{:});

report = struct( ...
    "SchemaVersion", "1.0", ...
    "GeneratedAt", string(datetime("now", ...
        Format = "yyyy-MM-dd'T'HH:mm:ssXXX")), ...
    "Model", "FSAE_TorqueVectoring_ClosedLoop", ...
    "TireBasis", ...
        "Round9_43075_R20_Rim7 MF6.2-equivalent; longitudinal/combined proxy", ...
    "ControlDesignStatus", "TENTATIVE_TorqueVectoring_CONTROL_DESIGN", ...
    "Cases", matrix, ...
    "Metrics", metrics, ...
    "Comparisons", comparisons);

if options.SaveSummary
    resultFolder = fullfile(projectRoot, "tests", "torque_vectoring", "results");
    if ~isfolder(resultFolder)
        mkdir(resultFolder);
    end
    save(fullfile(resultFolder, "TorqueVectoringVerificationSummary.mat"), ...
        "report", "-v7.3");
end

for comparison = reshape(comparisons, 1, [])
    fprintf("%s: yaw RMSE improvement %.3f%%, lateral error " + ...
        "improvement %.3f%%, drive energy change %.3f%%, " + ...
        "peak tire utilization change %.5f.\n", ...
        comparison.PairID, ...
        comparison.YawRateRMSEImprovementPercent, ...
        comparison.LateralErrorImprovementPercent, ...
        comparison.DriveEnergyChangePercent, ...
        comparison.PeakTireUtilizationChange);
end
end
