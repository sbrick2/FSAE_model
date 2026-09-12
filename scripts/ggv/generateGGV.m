function ggv = generateGGV(config, options)
%GENERATEGGV Generate a speed-dependent quasi-steady GGV surface.
%   GGV = GENERATEGGV(CONFIG) samples lateral-acceleration fractions and
%   bisects the feasible longitudinal-acceleration boundary at each speed.
%   The returned surface contains both acceleration and braking bounds plus
%   active-constraint labels for interpretation and later lap-time planning.
%   SteeringAngle is fixed across the map and defaults to zero.

arguments
    config (1, 1) struct
    options.SpeedGrid (1, :) double = linspace(0, 30, 7)
    options.LateralPointCount (1, 1) double {mustBeInteger, mustBeGreaterThanOrEqual(options.LateralPointCount, 3)} = 9
    options.RoadGripScale (1, :) double = []
    options.RoadMuLimit (1, :) double = []
    options.CamberAngle (4, 1) double = zeros(4, 1)
    options.SteeringAngle (1, 1) double {mustBeFinite} = 0.0
    options.AllocationMode (1, 1) string = "Fast"
    options.MaxLateralAcceleration (1, 1) double {mustBePositive} = 20.0
    options.MaxLongitudinalAcceleration (1, 1) double {mustBePositive} = 20.0
    options.MaxBisectionIterations (1, 1) double {mustBeInteger, mustBePositive} = 32
    options.LongitudinalBracketPointCount (1, 1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(options.LongitudinalBracketPointCount, 9)} = 33
    options.MaxSearchExpansionCount (1, 1) double ...
        {mustBeInteger, mustBeNonnegative} = 3
    options.EnvelopeLoadPointCount (1, 1) double ...
        {mustBeInteger, mustBeGreaterThanOrEqual(options.EnvelopeLoadPointCount, 17)} = 81
    options.UseParallelPrecomputation (1, 1) logical = false
    options.TireEnvelopeLookup (1, 1) struct = struct()
end

speedGrid = unique(double(options.SpeedGrid(:)'));
config.Solver.AllocationMode = upper(options.AllocationMode);
assert(any(config.Solver.AllocationMode == ["LP", "FAST"]), ...
    "FSAE:QuasiStatic:AllocationMode", "AllocationMode must be LP or Fast.");
assert(all(speedGrid >= 0) && all(isfinite(speedGrid)), ...
    "FSAE:QuasiStatic:SpeedGrid", "SpeedGrid must contain finite nonnegative speeds.");
assert(mod(options.LongitudinalBracketPointCount, 2) == 1, ...
    "FSAE:QuasiStatic:LongitudinalBracketPointCount", ...
    "LongitudinalBracketPointCount must be odd so that zero is sampled.");
lookupBuiltInCall = isempty(fieldnames(options.TireEnvelopeLookup));
if lookupBuiltInCall
    tireEnvelopeLookup = buildTireEnvelopeLookup(config, ...
        SpeedGrid = speedGrid, ...
        RoadGripScale = options.RoadGripScale, ...
        RoadMuLimit = options.RoadMuLimit, ...
        CamberAngle = options.CamberAngle, ...
        LoadPointCount = options.EnvelopeLoadPointCount, ...
        UseParallel = options.UseParallelPrecomputation);
else
    tireEnvelopeLookup = options.TireEnvelopeLookup;
end
roadGripScale = resolveFour(options.RoadGripScale, ...
    config.Environment.RoadGripScale);
roadMuLimit = resolveFour(options.RoadMuLimit, ...
    config.Environment.RoadMuLimit);
validateTireEnvelopeLookup(config, tireEnvelopeLookup, ...
    roadGripScale, roadMuLimit, options.CamberAngle);
options.TireEnvelopeLookup = tireEnvelopeLookup;
lateralFraction = linspace(-1.0, 1.0, options.LateralPointCount);
speedCount = numel(speedGrid);
fractionCount = numel(lateralFraction);

ayPositive = zeros(speedCount, 1);
ayNegative = zeros(speedCount, 1);
axMax = NaN(speedCount, fractionCount);
axMin = NaN(speedCount, fractionCount);
activeMax = cell(speedCount, fractionCount);
activeMin = cell(speedCount, fractionCount);
lateralResults = cell(speedCount, 2);
lateralAcceleration = zeros(speedCount, fractionCount);
pointFeasible = false(speedCount, fractionCount);
axMaxSaturated = false(speedCount, fractionCount);
axMinSaturated = false(speedCount, fractionCount);
ayPositiveSaturated = false(speedCount, 1);
ayNegativeSaturated = false(speedCount, 1);
speedFeasible = false(speedCount, 1);
lateralReferenceAxPositive = NaN(speedCount, 1);
lateralReferenceAxNegative = NaN(speedCount, 1);

for speedIndex = 1:speedCount
    speed = speedGrid(speedIndex);
    [ayPositive(speedIndex), ayPositiveSaturated(speedIndex), ...
        positiveBaseFeasible, lateralReferenceAxPositive(speedIndex)] = ...
        findLateralLimit(config, speed, 1.0, options);
    [ayNegative(speedIndex), ayNegativeSaturated(speedIndex), ...
        negativeBaseFeasible, lateralReferenceAxNegative(speedIndex)] = ...
        findLateralLimit(config, speed, -1.0, options);
    speedFeasible(speedIndex) = positiveBaseFeasible && negativeBaseFeasible;
    lateralResults{speedIndex, 1} = ayPositive(speedIndex);
    lateralResults{speedIndex, 2} = ayNegative(speedIndex);

    for lateralIndex = 1:fractionCount
        fraction = lateralFraction(lateralIndex);
        if fraction >= 0
            ay = fraction * ayPositive(speedIndex);
        else
            ay = -abs(fraction) * ayNegative(speedIndex);
        end
        lateralAcceleration(speedIndex, lateralIndex) = ay;
        [axMin(speedIndex, lateralIndex), minResult, ...
                axMinSaturated(speedIndex, lateralIndex), ...
                axMax(speedIndex, lateralIndex), maxResult, ...
                axMaxSaturated(speedIndex, lateralIndex), ...
                pointFeasible(speedIndex, lateralIndex)] = ...
            findLongitudinalBounds(config, speed, ay, options);
        activeMin{speedIndex, lateralIndex} = constraintLabels(minResult);
        activeMax{speedIndex, lateralIndex} = constraintLabels(maxResult);
    end
end

searchLimitReached = any(ayPositiveSaturated) || any(ayNegativeSaturated) || ...
    any(axMaxSaturated, "all") || any(axMinSaturated, "all");
ggv = struct( ...
    "Speed", speedGrid(:), ...
    "LateralFraction", lateralFraction(:), ...
    "LateralAcceleration", lateralAcceleration, ...
    "AyPositive", ayPositive, ...
    "AyNegative", ayNegative, ...
    "LateralReferenceAxPositive", lateralReferenceAxPositive, ...
    "LateralReferenceAxNegative", lateralReferenceAxNegative, ...
    "AxMax", axMax, ...
    "AxMin", axMin, ...
    "ActiveConstraintMax", {activeMax}, ...
    "ActiveConstraintMin", {activeMin}, ...
    "LateralLimitResults", {lateralResults}, ...
    "PointFeasible", pointFeasible, ...
    "SpeedFeasible", speedFeasible, ...
    "SearchSaturation", struct( ...
        "AyPositive", ayPositiveSaturated, ...
        "AyNegative", ayNegativeSaturated, ...
        "AxMax", axMaxSaturated, ...
        "AxMin", axMinSaturated), ...
    "Config", config, ...
    "Options", struct( ...
        "RoadGripScale", options.RoadGripScale, ...
        "RoadMuLimit", options.RoadMuLimit, ...
        "CamberAngle", options.CamberAngle, ...
        "SteeringAngle", options.SteeringAngle, ...
        "AllocationMode", config.Solver.AllocationMode, ...
        "LateralPointCount", options.LateralPointCount, ...
        "MaxLateralAcceleration", options.MaxLateralAcceleration, ...
        "MaxLongitudinalAcceleration", options.MaxLongitudinalAcceleration, ...
        "MaxBisectionIterations", options.MaxBisectionIterations, ...
        "LongitudinalBracketPointCount", options.LongitudinalBracketPointCount, ...
        "MaxSearchExpansionCount", options.MaxSearchExpansionCount, ...
        "EnvelopeLoadPointCount", tireEnvelopeLookup.LoadPointCount, ...
        "UseParallelPrecomputation", ...
        tireEnvelopeLookup.PrecomputationParallel), ...
    "Diagnostics", struct( ...
        "SearchLimitReached", searchLimitReached, ...
        "InvalidPointCount", nnz(~pointFeasible), ...
        "InvalidSpeedCount", nnz(~speedFeasible)), ...
    "Metadata", struct( ...
        "Method", "Adaptive longitudinal bounds with neutral/coast lateral reference", ...
        "TireModel", config.Tire.Model, ...
        "TireEnvelopeSource", string(tireEnvelopeLookup.Method), ...
        "TireEnvelopeLookupBuiltInCall", lookupBuiltInCall, ...
        "TireEnvelopeNormalLoadUpper", tireEnvelopeLookup.NormalLoadUpper, ...
        "TireEnvelopeLoadPointCount", tireEnvelopeLookup.LoadPointCount, ...
        "IsPerformanceClaim", false));
end

function [limit, saturated, baseFeasible, referenceAcceleration] = findLateralLimit( ...
        config, speed, signOfAy, options)
low = 0.0;
high = options.MaxLateralAcceleration;
[referenceAcceleration, ~, baseFeasible] = locateFeasibleSeed( ...
    config, speed, 0.0, options);
if ~baseFeasible
    limit = 0.0;
    saturated = false;
    return
end

highResult = evaluate( ...
    config, speed, [referenceAcceleration, signOfAy * high], options);
highFeasible = highResult.Feasible;
expansionCount = 0;
while highFeasible && expansionCount < options.MaxSearchExpansionCount
    low = high;
    high = 2.0 * high;
    expansionCount = expansionCount + 1;
    highResult = evaluate( ...
        config, speed, [referenceAcceleration, signOfAy * high], options);
    highFeasible = highResult.Feasible;
end
if highFeasible
    limit = high;
    saturated = true;
    return
end

for iteration = 1:options.MaxBisectionIterations
    mid = 0.5 * (low + high);
    midResult = evaluate( ...
        config, speed, [referenceAcceleration, signOfAy * mid], options);
    if midResult.Feasible
        low = mid;
    else
        high = mid;
    end
    if high - low <= config.Solver.BisectionTolerance
        break
    end
end
limit = low;
saturated = false;
end

function [lowerBoundary, lowerResult, lowerSaturated, ...
        upperBoundary, upperResult, upperSaturated, feasible] = ...
        findLongitudinalBounds(config, speed, ay, options)
[seedAcceleration, seedResult, feasible] = locateFeasibleSeed( ...
    config, speed, ay, options);
if ~feasible
    lowerBoundary = NaN;
    upperBoundary = NaN;
    lowerResult = seedResult;
    upperResult = seedResult;
    lowerSaturated = false;
    upperSaturated = false;
    return
end

[lowerBoundary, lowerResult, lowerSaturated] = searchBoundary( ...
    config, speed, ay, seedAcceleration, seedResult, -1.0, options);
[upperBoundary, upperResult, upperSaturated] = searchBoundary( ...
    config, speed, ay, seedAcceleration, seedResult, 1.0, options);
end

function [seedAcceleration, seedResult, found] = locateFeasibleSeed( ...
        config, speed, ay, options)
zeroResult = evaluate(config, speed, [0.0, ay], options);
if zeroResult.Feasible
    seedAcceleration = 0.0;
    seedResult = zeroResult;
    found = true;
    return
end

coastAcceleration = 0.0;
if isfield(zeroResult, "Aero") && isfield(zeroResult.Aero, "DragForce")
    coastAcceleration = -zeroResult.Aero.DragForce / config.Vehicle.Mass;
end
maximumSpan = options.MaxLongitudinalAcceleration * ...
    2.0^options.MaxSearchExpansionCount;
[lowerAcceleration, upperAcceleration] = normalLoadAccelerationBounds( ...
    config, zeroResult.NormalLoad, maximumSpan);
candidateAcceleration = linspace(lowerAcceleration, upperAcceleration, ...
    options.LongitudinalBracketPointCount);
candidateAcceleration = [candidateAcceleration, 0.0, coastAcceleration];
candidateAcceleration = unique(candidateAcceleration);
[~, order] = sort(abs(candidateAcceleration - coastAcceleration));
candidateAcceleration = candidateAcceleration(order);

seedAcceleration = NaN;
seedResult = zeroResult;
found = false;
for index = 1:numel(candidateAcceleration)
    acceleration = candidateAcceleration(index);
    if abs(acceleration) <= eps
        continue
    end
    result = evaluate(config, speed, [acceleration, ay], options);
    seedResult = result;
    if result.Feasible
        seedAcceleration = acceleration;
        seedResult = result;
        found = true;
        return
    end
end
end

function [lower, upper] = normalLoadAccelerationBounds( ...
        config, zeroAccelerationNormalLoad, maximumSpan)
mass = config.Vehicle.Mass;
height = config.Vehicle.CGHeight;
wheelbase = config.Vehicle.Wheelbase;
if height <= eps
    lower = -maximumSpan;
    upper = maximumSpan;
    return
end

loadSlope = mass * height / (2.0 * wheelbase);
frontUpper = min(zeroAccelerationNormalLoad(1:2)) / loadSlope;
rearLower = -min(zeroAccelerationNormalLoad(3:4)) / loadSlope;
margin = 10.0 * config.Solver.BisectionTolerance;
lower = max(-maximumSpan, rearLower + margin);
upper = min(maximumSpan, frontUpper - margin);
if lower >= upper
    lower = -maximumSpan;
    upper = maximumSpan;
end
end

function [boundary, result, saturated] = searchBoundary( ...
        config, speed, ay, seedAcceleration, seedResult, direction, options)
feasibleAcceleration = seedAcceleration;
result = seedResult;
span = max(options.MaxLongitudinalAcceleration, ...
    1.25 * abs(seedAcceleration) + config.Solver.BisectionTolerance);

for expansionIndex = 0:options.MaxSearchExpansionCount
    candidateAcceleration = direction * span;
    candidateResult = evaluate( ...
        config, speed, [candidateAcceleration, ay], options);
    if ~candidateResult.Feasible
        [boundary, result] = refineBoundary(config, speed, ay, ...
            feasibleAcceleration, result, candidateAcceleration, ...
            direction, options);
        saturated = false;
        return
    end
    feasibleAcceleration = candidateAcceleration;
    result = candidateResult;
    span = 2.0 * span;
end

boundary = feasibleAcceleration;
saturated = true;
end

function [boundary, result] = refineBoundary( ...
        config, speed, ay, feasibleAcceleration, feasibleResult, ...
        infeasibleAcceleration, direction, options)
if direction < 0
    low = infeasibleAcceleration;
    high = feasibleAcceleration;
else
    low = feasibleAcceleration;
    high = infeasibleAcceleration;
end
result = feasibleResult;

for iteration = 1:options.MaxBisectionIterations
    mid = 0.5 * (low + high);
    midResult = evaluate(config, speed, [mid, ay], options);
    if direction < 0
        if midResult.Feasible
            high = mid;
            result = midResult;
        else
            low = mid;
        end
    elseif midResult.Feasible
        low = mid;
        result = midResult;
    else
        high = mid;
    end
    if high - low <= config.Solver.BisectionTolerance
        break
    end
end
if direction < 0
    boundary = high;
else
    boundary = low;
end
end

function labels = constraintLabels(result)
labels = strings(0, 1);
if isstruct(result) && isfield(result, "ConstraintActive") && ...
        isfield(result.ConstraintActive, "Labels")
    labels = result.ConstraintActive.Labels;
end
end

function result = evaluate(config, speed, targetAcceleration, options)
result = solveSteadyState(config, speed, targetAcceleration, ...
    SteeringAngle = options.SteeringAngle, ...
    CamberAngle = options.CamberAngle, ...
    RoadGripScale = options.RoadGripScale, ...
    RoadMuLimit = options.RoadMuLimit, ...
    TireEnvelopeLookup = options.TireEnvelopeLookup, ...
    AssumeTireEnvelopeLookupValidated = true);
end

function values = resolveFour(override, fallback)
if isempty(override)
    values = double(fallback(:));
else
    values = double(override(:));
end
if isscalar(values)
    values = repmat(values, 4, 1);
end
assert(numel(values) == 4, "FSAE:QuasiStatic:FourWheelInput", ...
    "The input must be scalar or contain four wheel values.");
end
