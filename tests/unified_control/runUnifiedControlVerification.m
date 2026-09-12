function report = runUnifiedControlVerification(options)
%RUNUnifiedControlVERIFICATION Run UnifiedControl function and 10DOF closed-loop MIL verification.

arguments
    options.Profiles (1, :) string = ["Baseline", "AllFeatures", "NoRegen"]
    options.StopTime (1, 1) double {mustBePositive} = 0.5
    options.UseFastRestart (1, 1) logical = true
    options.SaveSummary (1, 1) logical = true
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "initialization"));
addpath(fullfile(projectRoot, "scripts", "path_tracking"));
addpath(fullfile(projectRoot, "scripts", "torque_vectoring"));
addpath(fullfile(projectRoot, "scripts", "vehicle_10dof"));
addpath(fullfile(projectRoot, "scripts", "unified_control"));
addpath(fullfile(projectRoot, "tests", "torque_vectoring"));
addpath(fullfile(projectRoot, "scripts", "simulation"));
addpath(fullfile(projectRoot, "scripts", "track"));
addpath(fullfile(projectRoot, "scenarios", "Skidpad"));

initProject();
installE41VehicleParameters();
installVehicle10DOFSuspensionData();
installTorqueVectoringControlData();
installUnifiedControlData();
functionReport = runUnifiedControlFunctionVerification(SaveSummary = false);
vehicleParameters = loadTorqueVectoringVehicleParameters();
vehicleParameters.UnifiedControl = loadUnifiedControlRuntime( ...
    fullfile(projectRoot, "data", "VehicleData.sldd"));
cases = createUnifiedControlScenarioMatrix(Profiles = options.Profiles);

in = repmat(Simulink.SimulationInput("FSAE_UnifiedControl_ClosedLoop"), ...
    numel(cases), 1);
for index = 1:numel(cases)
    in(index) = createTorqueVectoringSimulationInput(cases(index), vehicleParameters);
    in(index) = in(index).setVariable("UnifiedControlEnableTV", cases(index).EnableTV);
    in(index) = in(index).setVariable("UnifiedControlEnableTC", cases(index).EnableTC);
    in(index) = in(index).setVariable( ...
        "UnifiedControlEnableRegen", cases(index).EnableRegen);
    in(index) = in(index).setVariable("UnifiedControlEnableEnergyManagement", ...
        cases(index).EnableEnergyManagement);
    in(index) = in(index).setModelParameter( ...
        "StopTime", string(options.StopTime), ...
        "SaveOutput", "on", ...
        "OutputSaveName", "yout", ...
        "SaveFormat", "Dataset", ...
        "ReturnWorkspaceOutputs", "on");
end

if options.UseFastRestart
    out = sim(in, "UseFastRestart", "on");
else
    out = sim(in);
end

scenarioResults = struct.empty(0, 1);
for index = 1:numel(cases)
    item = assessScenario(out(index), cases(index), ...
        vehicleParameters);
    if index == 1
        scenarioResults = item;
    else
        scenarioResults(index, 1) = item;
    end
    assert(item.Passed, "FSAE:UnifiedControl:ClosedLoopScenario", ...
        "UnifiedControl scenario %s violated a structural or physical bound.", ...
        cases(index).Name);
end

report = struct( ...
    "SchemaVersion", "1.0", ...
    "GeneratedAt", string(datetime("now", ...
        Format = "yyyy-MM-dd'T'HH:mm:ss")), ...
    "Model", "FSAE_UnifiedControl_ClosedLoop", ...
    "FunctionVerification", functionReport, ...
    "ScenarioResults", scenarioResults, ...
    "AllPassed", all([scenarioResults.Passed]), ...
    "CalibrationStatus", "TENTATIVE_UnifiedControl_CONTROL_DESIGN", ...
    "Limitation", ...
        "Vehicle10DOF inertia/damper and UnifiedControl TC/rate/friction calibrations await vehicle evidence.");

if options.SaveSummary
    resultFolder = fullfile(projectRoot, "tests", "unified_control", "results");
    if ~isfolder(resultFolder)
        mkdir(resultFolder);
    end
    save(fullfile(resultFolder, "UnifiedControlVerificationSummary.mat"), ...
        "report", "-v7.3");
end

fprintf("UnifiedControl verification: %d/%d closed-loop scenarios passed.\n", ...
    sum([scenarioResults.Passed]), numel(scenarioResults));
for index = 1:numel(scenarioResults)
    item = scenarioResults(index);
    fprintf("  %s: motor %.3f N*m, power [%.1f, %.1f] W, " + ...
        "min Fz %.3f N, max step %.3f N*m.\n", item.Name, ...
        item.MaximumAbsMotorTorque, item.MinimumPowerRequest, ...
        item.MaximumPowerRequest, item.MinimumNormalLoad, ...
        item.MaximumMotorTorqueStep);
end
end

function result = assessScenario(out, definition, parameters)
dataset = out.yout;
vehicleState = dataset.getElement(1).Values;
controller = dataset.getElement(5).Values;
actuator = dataset.getElement(7).Values;

normalLoad = timeRows(vehicleState.NormalLoad);
motorTorque = timeRows(actuator.MotorTorqueRequest);
frictionBrake = timeRows(actuator.FrictionBrakeTorqueRequest);
powerRequest = timeRows(actuator.TotalPowerRequest);
controllerMode = timeRows(actuator.ControllerMode);
tcActive = timeRows(controller.TCActive);
energyActive = timeRows(controller.EnergyManagementActive);

motorLimit = parameters.UnifiedControl.MotorTorqueLimit;
drivePowerLimit = parameters.UnifiedControl.DrivePowerLimit;
if definition.EnableEnergyManagement
    drivePowerLimit = 0.90 * drivePowerLimit;
end
regenPowerLimit = parameters.UnifiedControl.RegenPowerLimit;
rateStep = parameters.UnifiedControl.MotorTorqueRateLimit * ...
    parameters.UnifiedControl.ControlSampleTime;

maximumMotorTorqueStep = 0.0;
if size(motorTorque, 1) > 1
    maximumMotorTorqueStep = max(abs(diff(motorTorque, 1, 1)), [], "all");
end
finiteSignals = all(isfinite(normalLoad), "all") && ...
    all(isfinite(motorTorque), "all") && ...
    all(isfinite(frictionBrake), "all") && ...
    all(isfinite(powerRequest), "all");
bounded = max(abs(motorTorque), [], "all") <= motorLimit + 1.0e-6 && ...
    min(frictionBrake, [], "all") >= -1.0e-9 && ...
    max(powerRequest, [], "all") <= drivePowerLimit + 1.0 && ...
    min(powerRequest, [], "all") >= -regenPowerLimit - 1.0 && ...
    maximumMotorTorqueStep <= rateStep + 1.0e-9 && ...
    min(normalLoad, [], "all") >= -1.0e-9 && ...
    all(ismember(unique(controllerMode), uint8([0; 2; 3])));

result = struct( ...
    "Name", definition.Name, ...
    "Profile", definition.Profile, ...
    "Passed", finiteSignals && bounded, ...
    "MaximumAbsMotorTorque", max(abs(motorTorque), [], "all"), ...
    "MaximumMotorTorqueStep", maximumMotorTorqueStep, ...
    "MaximumPowerRequest", max(powerRequest, [], "all"), ...
    "MinimumPowerRequest", min(powerRequest, [], "all"), ...
    "MinimumNormalLoad", min(normalLoad, [], "all"), ...
    "TCActiveFraction", mean(double(tcActive), "all"), ...
    "EnergyActiveFraction", mean(double(energyActive), "all"), ...
    "FiniteSignals", finiteSignals, ...
    "BoundsSatisfied", bounded, ...
    "ConfigurationFingerprint", definition.ConfigurationFingerprint);
end

function data = timeRows(signal)
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(signal.Time)
    data = data.';
end
end

function runtime = loadUnifiedControlRuntime(dictionaryPath)
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
section = getSection(dictionary, "Design Data");
mapping = {
    "MotorTorqueLimit", "UnifiedControlMotorTorqueLimit";
    "DrivePowerLimit", "UnifiedControlDrivePowerLimit";
    "RegenPowerLimit", "UnifiedControlRegenPowerLimit";
    "MotorTorqueRateLimit", "UnifiedControlMotorTorqueRateLimit";
    "ControlSampleTime", "UnifiedControlSampleTime"
    };
runtime = struct;
for row = 1:size(mapping, 1)
    value = getValue(getEntry(section, mapping{row, 2}));
    if isa(value, "Simulink.Parameter")
        value = value.Value;
    end
    runtime.(mapping{row, 1}) = value;
end
end
