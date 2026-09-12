%% 轮胎模型本体稳态动力学特性
% 本脚本直接扫描当前 TireModel 轮胎模型，不读取整车或圈速仿真结果。输出包括纯纵滑、
% 纯侧偏、轮荷敏感性、外倾角影响、联合滑移和 Fx-Fy 力平面。

%% 轮胎特性参数选择区

% Profile：[] 读取 VehicleData.sldd 当前选择；也可填写数据库 Profile 名称，
% 例如 "Round9_43075_R20_Rim7"，或直接填写一个 TireModel Profile 结构体。
tireCfg.Profile = [];

% 模型可选 "MF62" 或 "TTCMap"
tireCfg.Model = "MF62";

% 法向载荷，单位 N；[] 表示参考轮荷的 50%/75%/100%/125%/150%
tireCfg.NormalLoads = [];

% 纯滑移扫描范围
tireCfg.SlipRatioRange = [-0.25, 0.25];
tireCfg.SlipAngleRangeDeg = [-15, 15];

% 外倾角和联合滑移工况
tireCfg.CamberAnglesDeg = [-4, -2, 0, 2, 4];
tireCfg.CombinedSlipRatios = [0, 0.05, 0.10, 0.15];
tireCfg.CombinedSlipAnglesDeg = [-9, -6, -3, 0, 3, 6, 9];

% 曲线和联合滑移等高图分辨率
tireCfg.PointCount = 201;
tireCfg.CombinedGridPointCount = 51;

% 自动平滑特性曲线；窗口按曲线点数比例计算，并自动调整为奇数
tireCfg.AutoSmooth = true;
tireCfg.SmoothingFraction = 0.1;

% 1 表示TTC/MF62基准试验路面；Inf表示不叠加绝对摩擦圆上限
tireCfg.RoadGripScale = 1;
tireCfg.RoadMuLimit = Inf;

% 图窗与可选 MATLAB FIG 保存
tireCfg.Visible = "on";
tireCfg.SaveFigure = true;
tireCfg.OutputFolder = "results/tire"; % 空 -> results/tire/characteristics

%% 计算并绘图

projectRoot = tireAnalysisProjectRoot();
addpath(fullfile(projectRoot, "scripts", "tire"), "-begin");
profile = resolveTireProfile(tireCfg.Profile, projectRoot);
model = validateOptions(tireCfg);
normalLoads = resolveNormalLoads(tireCfg.NormalLoads, profile, model);
referenceLoad = profile.Reference.NormalLoadN;
capabilities = detectTireCapabilities(profile, model);
analysisCfg = adaptConfigurationToMap(tireCfg, profile, capabilities);

kappa = linspace(analysisCfg.SlipRatioRange(1), ...
    analysisCfg.SlipRatioRange(2), analysisCfg.PointCount).';
alphaDeg = linspace(analysisCfg.SlipAngleRangeDeg(1), ...
    analysisCfg.SlipAngleRangeDeg(2), analysisCfg.PointCount).';
alpha = deg2rad(alphaDeg);

useLateralMapLayout = model == "TTCMAP" && ...
    capabilities.SupportsLateral && ...
    (~capabilities.SupportsLongitudinal || ...
    ~capabilities.SupportsCombinedSlip);
if useLateralMapLayout
    [pureLateral, camberSweep] = evaluateLateralMapCharacteristics( ...
        profile, normalLoads, referenceLoad, alpha, analysisCfg);
    pureLateral.Fy = smoothCurveMatrix(pureLateral.Fy, analysisCfg);
    camberSweep.Fy = smoothCurveMatrix(camberSweep.Fy, analysisCfg);
    pureLongitudinal = unavailableLongitudinalData( ...
        numel(kappa), numel(normalLoads));
    tireCharacteristics = summarizePureSlip(kappa, alpha, normalLoads, ...
        pureLongitudinal.Fx, pureLateral.Fy);
    tabNames = [ ...
        "Lateral force"
        "Lateral load sensitivity"
        "Camber influence"
        "Peak lateral friction"
        "Slip/load coverage"
        "Slip/camber coverage"
        "Lateral-force sample coverage"
        "Reference-map force samples"
    ];
    [tireCharacteristicFigures, axesHandles] = createTireFigureTabs( ...
        string(profile.Name) + " | TTC Map characteristics", ...
        tabNames, tireCfg.Visible);
    plotTTCMapLateralAxes(axesHandles(1:4), normalLoads, ...
        alphaDeg, pureLateral, camberSweep, tireCharacteristics);
    plotTTCMapCoverageAxes(axesHandles(5:8), profile);
    figureStems = "tire_characteristics";
    outOfDomainCount = pureLateral.OutOfDomainCount + ...
        camberSweep.OutOfDomainCount;
    queryCount = pureLateral.QueryCount + camberSweep.QueryCount;
else
    [pureLongitudinal, pureLateral, camberSweep, combinedKappa, ...
        combinedAlpha, combinedGrid] = evaluateCharacteristics( ...
        model, profile, normalLoads, referenceLoad, kappa, alpha, ...
        analysisCfg);
    pureLongitudinal.Fx = smoothCurveMatrix( ...
        pureLongitudinal.Fx, analysisCfg);
    pureLateral.Fy = smoothCurveMatrix(pureLateral.Fy, analysisCfg);
    camberSweep.Fy = smoothCurveMatrix(camberSweep.Fy, analysisCfg);
    combinedKappa.Fx = smoothCurveMatrix(combinedKappa.Fx, analysisCfg);
    combinedKappa.Fy = smoothCurveMatrix(combinedKappa.Fy, analysisCfg);
    combinedAlpha.Fx = smoothCurveMatrix(combinedAlpha.Fx, analysisCfg);
    combinedAlpha.Fy = smoothCurveMatrix(combinedAlpha.Fy, analysisCfg);
    tireCharacteristics = summarizePureSlip(kappa, alpha, normalLoads, ...
        pureLongitudinal.Fx, pureLateral.Fy);
    tabNames = [ ...
        "Pure longitudinal force"
        "Pure lateral force"
        "Longitudinal load sensitivity"
        "Lateral load sensitivity"
        "Camber influence"
        "Peak friction versus load"
        "Combined longitudinal force"
        "Combined lateral force"
        "Fx-Fy force plane"
        "Resultant force coefficient"
    ];
    [tireCharacteristicFigures, axesHandles] = createTireFigureTabs( ...
        string(profile.Name) + " | " + model + " tire characteristics", ...
        tabNames, tireCfg.Visible);
    plotPureSlipAxes(axesHandles(1:6), normalLoads, kappa, alphaDeg, ...
        pureLongitudinal, pureLateral, camberSweep, tireCharacteristics);
    plotCombinedSlipAxes(axesHandles(7:10), kappa, ...
        alphaDeg, combinedKappa, combinedAlpha, combinedGrid, analysisCfg);
    figureStems = "tire_characteristics";
    outOfDomainCount = pureLongitudinal.OutOfDomainCount + ...
        pureLateral.OutOfDomainCount + camberSweep.OutOfDomainCount + ...
        combinedKappa.OutOfDomainCount + combinedAlpha.OutOfDomainCount + ...
        combinedGrid.OutOfDomainCount;
    queryCount = pureLongitudinal.QueryCount + pureLateral.QueryCount + ...
        camberSweep.QueryCount + combinedKappa.QueryCount + ...
        combinedAlpha.QueryCount + combinedGrid.QueryCount;
end
tireModelSummary = struct( ...
    "ProfileName", string(profile.Name), ...
    "TireID", string(profile.TireID), ...
    "Model", model, ...
    "ReferenceNormalLoad_N", referenceLoad, ...
    "InputPressure_Pa", profile.InputPressurePa, ...
    "RoadGripScale", tireCfg.RoadGripScale, ...
    "RoadMuLimit", tireCfg.RoadMuLimit, ...
    "NormalLoads_N", normalLoads, ...
    "Smoothing", struct( ...
        "Enabled", tireCfg.AutoSmooth, ...
        "Method", "Savitzky-Golay", ...
        "Fraction", tireCfg.SmoothingFraction, ...
        "WindowPoints", automaticSmoothingWindow( ...
            tireCfg.PointCount, tireCfg.SmoothingFraction)), ...
    "Capabilities", capabilities, ...
    "Characteristics", tireCharacteristics, ...
    "OutOfDomainPercent", 100 * outOfDomainCount / max(queryCount, 1));
tireModelSummary.FigureFiles = finalizeTireFigures( ...
    tireCharacteristicFigures, figureStems, profile, model, ...
    tireCfg.SaveFigure, tireCfg.OutputFolder, projectRoot);

disp(tireCharacteristics);

function model = validateOptions(options)
model = upper(options.Model);
assert(any(model == ["MF62", "TTCMAP"]), ...
    "FSAE:TireAnalysis:InvalidModel", ...
    "Model must be MF62 or TTCMap.");
assert(options.SlipRatioRange(1) < options.SlipRatioRange(2), ...
    "FSAE:TireAnalysis:SlipRatioRange", ...
    "SlipRatioRange must be strictly increasing.");
assert(options.SlipAngleRangeDeg(1) < options.SlipAngleRangeDeg(2), ...
    "FSAE:TireAnalysis:SlipAngleRange", ...
    "SlipAngleRangeDeg must be strictly increasing.");
assert(any(options.Visible == ["on", "off"]), ...
    "FSAE:TireAnalysis:Visible", "Visible must be on or off.");
assert(islogical(options.AutoSmooth) && isscalar(options.AutoSmooth), ...
    "FSAE:TireAnalysis:AutoSmooth", ...
    "AutoSmooth must be a scalar logical value.");
assert(isnumeric(options.SmoothingFraction) && ...
    isscalar(options.SmoothingFraction) && ...
    isfinite(options.SmoothingFraction) && ...
    options.SmoothingFraction > 0 && options.SmoothingFraction <= 0.25, ...
    "FSAE:TireAnalysis:SmoothingFraction", ...
    "SmoothingFraction must be in the interval (0, 0.25].");
end

function capabilities = detectTireCapabilities(profile, model)
if model == "MF62"
    capabilities = struct( ...
        "SupportsLongitudinal", true, ...
        "SupportsLateral", true, ...
        "SupportsCombinedSlip", true, ...
        "Notes", "MF62 analytical model supports the configured slip sweeps.");
    return
end
points = double(profile.Map.Points);
tolerance = 1.0e-6;
hasLongitudinalSlip = abs(points(:, 1)) > tolerance;
hasLateralSlip = abs(points(:, 2)) > tolerance;
capabilities = struct( ...
    "SupportsLongitudinal", any(hasLongitudinalSlip), ...
    "SupportsLateral", any(hasLateralSlip), ...
    "SupportsCombinedSlip", any(hasLongitudinalSlip & hasLateralSlip), ...
    "Notes", "Capabilities are inferred from the TTC reference-map samples.");
if ~capabilities.SupportsLongitudinal
    capabilities.Notes = capabilities.Notes + ...
        " This map has kappa=0 only; longitudinal and combined-slip plots " + ...
        "are replaced by lateral characteristics and map-domain diagnostics.";
end
end

function config = adaptConfigurationToMap(config, profile, capabilities)
if capabilities.SupportsLongitudinal && capabilities.SupportsLateral && ...
        capabilities.SupportsCombinedSlip
    return
end
lowerBound = double(profile.Map.LowerBound);
upperBound = double(profile.Map.UpperBound);
requestedAlpha = deg2rad(config.SlipAngleRangeDeg);
effectiveAlpha = [max(requestedAlpha(1), lowerBound(2)), ...
    min(requestedAlpha(2), upperBound(2))];
assert(effectiveAlpha(1) < effectiveAlpha(2), ...
    "FSAE:TireAnalysis:AlphaOutsideMap", ...
    "SlipAngleRangeDeg does not overlap the TTC map domain [%.2f, %.2f] deg.", ...
    rad2deg(lowerBound(2)), rad2deg(upperBound(2)));
config.SlipAngleRangeDeg = rad2deg(effectiveAlpha);

camberRad = deg2rad(config.CamberAnglesDeg);
insideCamber = camberRad >= lowerBound(3) & camberRad <= upperBound(3);
config.CamberAnglesDeg = config.CamberAnglesDeg(insideCamber);
if isempty(config.CamberAnglesDeg)
    config.CamberAnglesDeg = rad2deg(min(max(0, lowerBound(3)), upperBound(3)));
end
assert(profile.InputPressurePa >= lowerBound(5) && ...
    profile.InputPressurePa <= upperBound(5), ...
    "FSAE:TireAnalysis:PressureOutsideMap", ...
    "The profile input pressure %.0f Pa lies outside the TTC map domain.", ...
    profile.InputPressurePa);
end

function profile = resolveTireProfile(profileInput, projectRoot)
if isempty(profileInput)
    profile = getSelectedTireProfile();
elseif isstruct(profileInput) && isscalar(profileInput)
    profile = profileInput;
elseif (isstring(profileInput) && isscalar(profileInput)) || ...
        (ischar(profileInput) && isrow(profileInput))
    profileName = string(profileInput);
    dataFile = fullfile(projectRoot, "data", "TireData", ...
        "TireModelData.mat");
    assert(isfile(dataFile), "FSAE:TireAnalysis:DatabaseMissing", ...
        "Tire profile database does not exist: %s", dataFile);
    stored = load(dataFile, "database");
    names = string({stored.database.Profiles.Name});
    index = find(names == profileName, 1);
    assert(~isempty(index), "FSAE:TireAnalysis:UnknownProfile", ...
        "Unknown tire profile '%s'. Available profiles: %s", ...
        profileName, strjoin(names, ", "));
    profile = stored.database.Profiles(index);
else
    error("FSAE:TireAnalysis:InvalidProfile", ...
        "Profile must be empty, a scalar TireModel profile structure, or a profile name.");
end
requiredFields = ["Name", "TireID", "Reference", "InputPressurePa", ...
    "Longitudinal", "Lateral", "Combined", "Map"];
assert(all(isfield(profile, cellstr(requiredFields))), ...
    "FSAE:TireAnalysis:IncompleteProfile", ...
    "The tire profile is missing required TireModel model fields.");
end

function normalLoads = resolveNormalLoads(requestedLoads, profile, model)
if isempty(requestedLoads)
    normalLoads = profile.Reference.NormalLoadN .* ...
        [0.50, 0.75, 1.00, 1.25, 1.50];
else
    normalLoads = sort(unique(requestedLoads));
end
if model == "TTCMAP"
    lowerLoad = profile.Map.LowerBound(4);
    upperLoad = profile.Map.UpperBound(4);
    normalLoads = normalLoads(normalLoads >= lowerLoad & ...
        normalLoads <= upperLoad);
    assert(~isempty(normalLoads), "FSAE:TireAnalysis:LoadsOutOfMap", ...
        "No requested normal load lies inside the TTC map domain [%.1f, %.1f] N.", ...
        lowerLoad, upperLoad);
end
normalLoads = normalLoads(:).';
end

function [lateral, camber] = evaluateLateralMapCharacteristics( ...
        profile, normalLoads, referenceLoad, alpha, options)
loadCount = numel(normalLoads);
alphaMatrix = repmat(alpha, 1, loadCount);
loadMatrix = repmat(normalLoads, numel(alpha), 1);
[~, lateral.Fy, lateral.OutOfDomain] = evaluateModel( ...
    "TTCMAP", zeros(size(alphaMatrix)), alphaMatrix, ...
    zeros(size(alphaMatrix)), loadMatrix, options.RoadGripScale, ...
    options.RoadMuLimit, profile);
lateral = addQueryCounts(lateral);

camber.AnglesDeg = options.CamberAnglesDeg;
camberMatrix = repmat(deg2rad(camber.AnglesDeg), numel(alpha), 1);
camberAlpha = repmat(alpha, 1, numel(camber.AnglesDeg));
[~, camber.Fy, camber.OutOfDomain] = evaluateModel( ...
    "TTCMAP", zeros(size(camberAlpha)), camberAlpha, camberMatrix, ...
    referenceLoad .* ones(size(camberAlpha)), options.RoadGripScale, ...
    options.RoadMuLimit, profile);
camber = addQueryCounts(camber);
end

function longitudinal = unavailableLongitudinalData(pointCount, loadCount)
longitudinal.Fx = NaN(pointCount, loadCount);
longitudinal.OutOfDomain = false(pointCount, loadCount);
longitudinal.OutOfDomainCount = 0;
longitudinal.QueryCount = 0;
end

function [longitudinal, lateral, camber, combinedKappa, ...
        combinedAlpha, gridData] = evaluateCharacteristics( ...
        model, profile, normalLoads, referenceLoad, kappa, alpha, options)
loadCount = numel(normalLoads);
kappaMatrix = repmat(kappa, 1, loadCount);
alphaMatrix = repmat(alpha, 1, loadCount);
loadMatrix = repmat(normalLoads, numel(kappa), 1);
[longitudinal.Fx, ~, longitudinal.OutOfDomain] = evaluateModel( ...
    model, kappaMatrix, zeros(size(kappaMatrix)), ...
    zeros(size(kappaMatrix)), loadMatrix, options.RoadGripScale, ...
    options.RoadMuLimit, profile);
longitudinal = addQueryCounts(longitudinal);

[~, lateral.Fy, lateral.OutOfDomain] = evaluateModel( ...
    model, zeros(size(alphaMatrix)), alphaMatrix, ...
    zeros(size(alphaMatrix)), loadMatrix, options.RoadGripScale, ...
    options.RoadMuLimit, profile);
lateral = addQueryCounts(lateral);

camber.AnglesDeg = options.CamberAnglesDeg;
camberMatrix = repmat(deg2rad(camber.AnglesDeg), numel(alpha), 1);
camberAlpha = repmat(alpha, 1, numel(camber.AnglesDeg));
[~, camber.Fy, camber.OutOfDomain] = evaluateModel( ...
    model, zeros(size(camberAlpha)), camberAlpha, camberMatrix, ...
    referenceLoad .* ones(size(camberAlpha)), options.RoadGripScale, ...
    options.RoadMuLimit, profile);
camber = addQueryCounts(camber);

combinedKappa.AnglesDeg = options.CombinedSlipAnglesDeg;
combinedKappaAlpha = repmat(deg2rad(combinedKappa.AnglesDeg), ...
    numel(kappa), 1);
combinedKappaInput = repmat(kappa, 1, numel(combinedKappa.AnglesDeg));
[combinedKappa.Fx, combinedKappa.Fy, ...
    combinedKappa.OutOfDomain] = evaluateModel(model, ...
    combinedKappaInput, combinedKappaAlpha, ...
    zeros(size(combinedKappaInput)), ...
    referenceLoad .* ones(size(combinedKappaInput)), ...
    options.RoadGripScale, options.RoadMuLimit, profile);
combinedKappa = addQueryCounts(combinedKappa);

combinedAlpha.SlipRatios = options.CombinedSlipRatios;
combinedAlphaInput = repmat(alpha, 1, numel(combinedAlpha.SlipRatios));
combinedAlphaKappa = repmat(combinedAlpha.SlipRatios, numel(alpha), 1);
[combinedAlpha.Fx, combinedAlpha.Fy, ...
    combinedAlpha.OutOfDomain] = evaluateModel(model, ...
    combinedAlphaKappa, combinedAlphaInput, ...
    zeros(size(combinedAlphaInput)), ...
    referenceLoad .* ones(size(combinedAlphaInput)), ...
    options.RoadGripScale, options.RoadMuLimit, profile);
combinedAlpha = addQueryCounts(combinedAlpha);

gridKappa = linspace(options.SlipRatioRange(1), ...
    options.SlipRatioRange(2), options.CombinedGridPointCount);
gridAlphaDeg = linspace(options.SlipAngleRangeDeg(1), ...
    options.SlipAngleRangeDeg(2), options.CombinedGridPointCount);
[gridData.Kappa, gridData.AlphaDeg] = meshgrid(gridKappa, gridAlphaDeg);
[gridData.Fx, gridData.Fy, gridData.OutOfDomain] = evaluateModel( ...
    model, gridData.Kappa, deg2rad(gridData.AlphaDeg), ...
    zeros(size(gridData.Kappa)), ...
    referenceLoad .* ones(size(gridData.Kappa)), ...
    options.RoadGripScale, options.RoadMuLimit, profile);
gridData.ForceCoefficient = hypot(gridData.Fx, gridData.Fy) ./ referenceLoad;
gridData = addQueryCounts(gridData);
end

function data = addQueryCounts(data)
data.OutOfDomainCount = nnz(data.OutOfDomain);
data.QueryCount = numel(data.OutOfDomain);
end

function data = smoothCurveMatrix(data, options)
if ~options.AutoSmooth || isempty(data)
    return
end
targetWindow = automaticSmoothingWindow( ...
    size(data, 1), options.SmoothingFraction);
for column = 1:size(data, 2)
    finiteMask = isfinite(data(:, column));
    transitions = diff([false; finiteMask; false]);
    runStarts = find(transitions == 1);
    runEnds = find(transitions == -1) - 1;
    for runIndex = 1:numel(runStarts)
        indices = runStarts(runIndex):runEnds(runIndex);
        window = min(targetWindow, largestOdd(numel(indices)));
        if window >= 5
            data(indices, column) = smoothdata( ...
                data(indices, column), "sgolay", window);
        end
    end
end
end

function window = automaticSmoothingWindow(pointCount, fraction)
window = max(5, round(pointCount * fraction));
if mod(window, 2) == 0
    window = window + 1;
end
window = min(window, largestOdd(pointCount));
end

function value = largestOdd(value)
value = floor(value);
if mod(value, 2) == 0
    value = value - 1;
end
end

function [forceX, forceY, outOfDomain] = evaluateModel( ...
        model, slipRatio, slipAngle, camberAngle, normalLoad, ...
        roadGripScale, roadMuLimit, profile)
roadGripScaleInput = roadGripScale .* ones(size(slipRatio));
roadMuLimitInput = roadMuLimit .* ones(size(slipRatio));
if model == "MF62"
    [forceX, forceY] = evaluateTireMF62(slipRatio, slipAngle, ...
        camberAngle, normalLoad, roadGripScaleInput, ...
        roadMuLimitInput, profile);
    outOfDomain = false(size(forceX));
else
    [forceX, forceY, ~, ~, outOfDomain] = evaluateTTCMap( ...
        slipRatio, slipAngle, camberAngle, normalLoad, ...
        roadGripScaleInput, roadMuLimitInput, profile);
    forceX(outOfDomain) = NaN;
    forceY(outOfDomain) = NaN;
end
end

function characteristics = summarizePureSlip( ...
        kappa, alpha, normalLoads, forceX, forceY)
loadCount = numel(normalLoads);
peakFx = NaN(loadCount, 1);
peakFy = NaN(loadCount, 1);
peakMuX = NaN(loadCount, 1);
peakMuY = NaN(loadCount, 1);
peakKappa = NaN(loadCount, 1);
peakAlphaDeg = NaN(loadCount, 1);
longitudinalStiffness = NaN(loadCount, 1);
corneringStiffness = NaN(loadCount, 1);
for index = 1:loadCount
    [peakFx(index), peakKappa(index)] = peakWithLocation( ...
        forceX(:, index), kappa);
    [peakFy(index), peakAlpha] = peakWithLocation( ...
        forceY(:, index), alpha);
    peakAlphaDeg(index) = rad2deg(peakAlpha);
    peakMuX(index) = peakFx(index) / normalLoads(index);
    peakMuY(index) = peakFy(index) / normalLoads(index);
    longitudinalStiffness(index) = estimateSlope(kappa, ...
        forceX(:, index), 0.02);
    corneringStiffness(index) = -estimateSlope(alpha, ...
        forceY(:, index), deg2rad(1.5));
end
characteristics = table(normalLoads(:), peakFx, peakFy, peakMuX, ...
    peakMuY, peakKappa, peakAlphaDeg, longitudinalStiffness, ...
    corneringStiffness, VariableNames=["NormalLoad_N", ...
    "PeakAbsFx_N", "PeakAbsFy_N", "PeakMuX", "PeakMuY", ...
    "SlipRatioAtPeakFx", "SlipAngleAtPeakFy_deg", ...
    "LongitudinalStiffness_N", "CorneringStiffness_N_per_rad"]);
end

function [peakValue, location] = peakWithLocation(force, coordinate)
valid = isfinite(force) & isfinite(coordinate);
if ~any(valid)
    peakValue = NaN;
    location = NaN;
    return
end
validIndices = find(valid);
[peakValue, localIndex] = max(abs(force(valid)));
location = coordinate(validIndices(localIndex));
end

function slope = estimateSlope(x, y, halfWidth)
valid = isfinite(x) & isfinite(y) & abs(x) <= halfWidth;
if nnz(valid) < 3
    slope = NaN;
    return
end
coefficients = polyfit(x(valid), y(valid), 1);
slope = coefficients(1);
end

function plotPureSlipAxes(axesHandles, normalLoads, kappa, alphaDeg, ...
        longitudinal, lateral, camber, characteristics)
loadLabels = compose("Fz = %.0f N", normalLoads);

ax = axesHandles(1);
plotMatrix(ax, kappa, longitudinal.Fx, loadLabels);
xlabel(ax, "Slip ratio (-)"); ylabel(ax, "Longitudinal force Fx (N)");
title(ax, "Pure longitudinal force");

ax = axesHandles(2);
plotMatrix(ax, alphaDeg, lateral.Fy, loadLabels);
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Lateral force Fy (N)");
title(ax, "Pure lateral force");

ax = axesHandles(3);
plotMatrix(ax, kappa, longitudinal.Fx ./ normalLoads, loadLabels);
xlabel(ax, "Slip ratio (-)"); ylabel(ax, "Fx / Fz (-)");
title(ax, "Longitudinal load sensitivity");

ax = axesHandles(4);
plotMatrix(ax, alphaDeg, lateral.Fy ./ normalLoads, loadLabels);
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Fy / Fz (-)");
title(ax, "Lateral load sensitivity");

ax = axesHandles(5);
plotMatrix(ax, alphaDeg, camber.Fy, ...
    compose("Camber = %g deg", camber.AnglesDeg));
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Lateral force Fy (N)");
title(ax, "Camber influence at reference load");

ax = axesHandles(6);
plot(ax, characteristics.NormalLoad_N, characteristics.PeakMuX, ...
    "o-", "LineWidth", 1.3, "DisplayName", "Peak |Fx| / Fz");
hold(ax, "on");
plot(ax, characteristics.NormalLoad_N, characteristics.PeakMuY, ...
    "s-", "LineWidth", 1.3, "DisplayName", "Peak |Fy| / Fz");
hold(ax, "off");
grid(ax, "on"); legend(ax, "Location", "best");
xlabel(ax, "Normal load (N)"); ylabel(ax, "Peak force coefficient (-)");
title(ax, "Peak friction versus load");
end

function plotTTCMapLateralAxes(axesHandles, normalLoads, ...
        alphaDeg, lateral, camber, characteristics)
loadLabels = compose("Fz = %.0f N", normalLoads);

ax = axesHandles(1);
plotMatrix(ax, alphaDeg, lateral.Fy, loadLabels);
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Lateral force Fy (N)");
title(ax, "Lateral force versus slip angle");

ax = axesHandles(2);
plotMatrix(ax, alphaDeg, lateral.Fy ./ normalLoads, loadLabels);
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Fy / Fz (-)");
title(ax, "Lateral load sensitivity");

ax = axesHandles(3);
plotMatrix(ax, alphaDeg, camber.Fy, ...
    compose("Camber = %g deg", camber.AnglesDeg));
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Lateral force Fy (N)");
title(ax, "Camber influence at reference load");

ax = axesHandles(4);
plot(ax, characteristics.NormalLoad_N, characteristics.PeakMuY, ...
    "o-", "LineWidth", 1.3, "MarkerFaceColor", "auto");
grid(ax, "on");
xlabel(ax, "Normal load (N)"); ylabel(ax, "Peak |Fy| / Fz (-)");
title(ax, "Peak lateral friction versus load");
end

function plotTTCMapCoverageAxes(axesHandles, profile)
points = double(profile.Map.Points);
forces = double(profile.Map.Forces);
valid = all(isfinite(points), 2) & all(isfinite(forces), 2) & ...
    points(:, 4) > 0;
points = points(valid, :);
forces = forces(valid, :);
alphaDeg = rad2deg(points(:, 2));
camberDeg = rad2deg(points(:, 3));
normalLoad = points(:, 4);
pressureKPa = points(:, 5) ./ 1000;
lateralCoefficient = forces(:, 2) ./ normalLoad;

ax = axesHandles(1);
scatter(ax, alphaDeg, normalLoad, 10, camberDeg, "filled", ...
    "MarkerFaceAlpha", 0.35);
grid(ax, "on"); xlabel(ax, "Slip angle (deg)");
ylabel(ax, "Normal load (N)"); title(ax, "Slip/load coverage");
bar = colorbar(ax); bar.Label.String = "Camber (deg)";

ax = axesHandles(2);
scatter(ax, alphaDeg, camberDeg, 10, pressureKPa, "filled", ...
    "MarkerFaceAlpha", 0.35);
grid(ax, "on"); xlabel(ax, "Slip angle (deg)");
ylabel(ax, "Camber (deg)"); title(ax, "Slip/camber coverage");
bar = colorbar(ax); bar.Label.String = "Pressure (kPa)";

ax = axesHandles(3);
scatter(ax, normalLoad, camberDeg, 10, lateralCoefficient, "filled", ...
    "MarkerFaceAlpha", 0.35);
grid(ax, "on"); xlabel(ax, "Normal load (N)");
ylabel(ax, "Camber (deg)"); title(ax, "Lateral-force sample coverage");
bar = colorbar(ax); bar.Label.String = "Fy / Fz (-)";

ax = axesHandles(4);
scatter(ax, forces(:, 1), forces(:, 2), 10, normalLoad, "filled", ...
    "MarkerFaceAlpha", 0.35);
grid(ax, "on"); axis(ax, "equal");
xlabel(ax, "Map force Fx (N)"); ylabel(ax, "Map force Fy (N)");
title(ax, "Reference-map force samples");
bar = colorbar(ax); bar.Label.String = "Normal load (N)";
end

function plotCombinedSlipAxes(axesHandles, kappa, ...
        alphaDeg, combinedKappa, combinedAlpha, gridData, options)

ax = axesHandles(1);
nonnegativeAngles = combinedKappa.AnglesDeg >= 0;
plotMatrix(ax, kappa, combinedKappa.Fx(:, nonnegativeAngles), ...
    compose("Alpha = %g deg", ...
    combinedKappa.AnglesDeg(nonnegativeAngles)));
xlabel(ax, "Slip ratio (-)"); ylabel(ax, "Longitudinal force Fx (N)");
title(ax, "Longitudinal force under lateral slip");

ax = axesHandles(2);
plotMatrix(ax, alphaDeg, combinedAlpha.Fy, ...
    compose("Kappa = %g", combinedAlpha.SlipRatios));
xlabel(ax, "Slip angle (deg)"); ylabel(ax, "Lateral force Fy (N)");
title(ax, "Lateral force under longitudinal slip");

ax = axesHandles(3);
plotMatrix(ax, combinedKappa.Fx, combinedKappa.Fy, ...
    compose("Alpha = %g deg", combinedKappa.AnglesDeg));
axis(ax, "equal");
xlabel(ax, "Longitudinal force Fx (N)");
ylabel(ax, "Lateral force Fy (N)");
title(ax, "Fx-Fy force plane");

ax = axesHandles(4);
contourf(ax, gridData.Kappa, gridData.AlphaDeg, ...
    gridData.ForceCoefficient, 18, "LineStyle", "none");
grid(ax, "on");
xlabel(ax, "Slip ratio (-)"); ylabel(ax, "Slip angle (deg)");
title(ax, "Resultant force coefficient");
colorBar = colorbar(ax);
colorBar.Label.String = "sqrt(Fx^2 + Fy^2) / Fz";
xlim(ax, options.SlipRatioRange);
ylim(ax, options.SlipAngleRangeDeg);
end

function [figureHandle, axesHandles] = createTireFigureTabs( ...
        groupName, plotNames, visibility)
plotNames = string(plotNames(:));
figureHandle = figure( ...
    "Name", groupName, "NumberTitle", "off", ...
    "Visible", visibility, "Color", "white", ...
    "WindowStyle", "normal");
tabGroup = uitabgroup("Parent", figureHandle);
axesHandles = gobjects(numel(plotNames), 1);
for index = 1:numel(plotNames)
    tabHandle = uitab(tabGroup, "Title", plotNames(index));
    axesHandles(index) = axes("Parent", tabHandle);
end
end

function plotMatrix(ax, x, data, labels)
hold(ax, "on");
lineHandles = gobjects(0);
validLabels = strings(0);
for index = 1:size(data, 2)
    if isvector(x)
        xData = x(:);
    else
        xData = x(:, index);
    end
    valid = isfinite(xData) & isfinite(data(:, index));
    if ~any(valid)
        continue
    end
    lineHandles(end + 1, 1) = plot(ax, xData(valid), ...
        data(valid, index), "LineWidth", 1.25); %#ok<AGROW>
    validLabels(end + 1) = labels(index); %#ok<AGROW>
end
hold(ax, "off");
grid(ax, "on");
if isempty(lineHandles)
    axis(ax, "off");
    text(ax, 0.5, 0.5, "No in-domain tire data", ...
        "Units", "normalized", "HorizontalAlignment", "center");
elseif numel(lineHandles) > 1
    legend(ax, lineHandles, validLabels, "Location", "best", ...
        "Interpreter", "none");
end
end

function files = finalizeTireFigures(figures, stems, profile, model, ...
        saveFigures, outputFolder, projectRoot)
for figureHandle = figures(:).'
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
files = strings(numel(figures), 1);
if ~saveFigures
    return
end
if strlength(outputFolder) == 0
    outputFolder = fullfile(projectRoot, "results", "tire", ...
        "characteristics");
end
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end
profileStem = lower(regexprep(string(profile.Name), "[^A-Za-z0-9]+", "_"));
timestamp = string(datetime("now", TimeZone="UTC", ...
    Format="yyyyMMdd_HHmmss_SSS"));
for index = 1:numel(figures)
    files(index) = fullfile(outputFolder, profileStem + "_" + ...
        lower(model) + "_" + stems(index) + "_" + timestamp + ".fig");
    savefig(figures(index), files(index));
end
end

function projectRoot = tireAnalysisProjectRoot()
projectRoot = string(fileparts(fileparts(fileparts(fileparts( ...
    mfilename("fullpath"))))));
end
