function modelNames = buildPathTrackingPathTrackingModels(options)
%BUILDPathTrackingPATHTRACKINGMODELS Build the PathTracking production model set.
%   This builder creates separate PathTracking models and leaves the ProjectFoundation/OpenLoopPlant/TireModel
%   production models unchanged.

arguments
    options.ForceRebuild (1, 1) logical = false
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "track"));
addpath(fullfile(projectRoot, "scripts", "path_tracking"));
addpath(fullfile(projectRoot, "scenarios", "Acceleration"));

modelNames = ["PathTrackingDriver", "PathTrackingVehicleController", ...
    "PathTrackingSensorModel", "FSAE_PathTracking_ClosedLoop"];
dictionaryPath = fullfile(projectRoot, "data", "VehicleData.sldd");
assert(isfile(dictionaryPath), "FSAE:PathTrackingMissingDictionary", ...
    "VehicleData.sldd is required before building the PathTracking models.");
dictionaryName = "VehicleData.sldd";
installPathTrackingTrackDataBus(dictionaryPath);
defaultScenario = createPathTrackingAccelerationScenario();

createPathTrackingDriver(projectRoot, dictionaryName, options.ForceRebuild);
createVehicleController(projectRoot, dictionaryName, options.ForceRebuild);
createSensorModel(projectRoot, dictionaryName, options.ForceRebuild);
createClosedLoopTop(projectRoot, dictionaryName, defaultScenario, options.ForceRebuild);

fprintf("PathTracking models ready: %s\n", strjoin(modelNames, ", "));
end

function createPathTrackingDriver(projectRoot, dictionaryName, forceRebuild)
modelName = "PathTrackingDriver";
if ~prepareModel(modelName, fullfile(projectRoot, "models", "driver"), dictionaryName, forceRebuild)
    return
end
add_block("simulink/Ports & Subsystems/In1", modelName + "/TrackReference", ...
    "Position", [30 80 60 94], "Port", "1", ...
    "OutDataTypeStr", "Bus: TrackReferenceBus");
add_block("simulink/Ports & Subsystems/In1", modelName + "/Sensor", ...
    "Position", [30 220 60 234], "Port", "2", ...
    "OutDataTypeStr", "Bus: SensorBus");
add_block("simulink/User-Defined Functions/MATLAB Function", ...
    modelName + "/PathTrackingCore", "Position", [180 90 430 260]);
setChartScript(modelName + "/PathTrackingCore", ...
    fullfile(projectRoot, "scripts", "path_tracking", "fsaePathTrackingPathTrackingCore.m"), ...
    "fsaePathTrackingPathTrackingCore");
declareChartParameters(modelName + "/PathTrackingCore", [ ...
    "PathTrackingControlSampleTime", "PathTrackingWheelbase", "PathTrackingHeadingGain", ...
    "PathTrackingCrossTrackGain", "PathTrackingMinimumSteeringSpeed", ...
    "PathTrackingMaximumSteeringAngle", "PathTrackingMaximumSteeringRate", ...
    "PathTrackingSteeringWheelRatio", "PathTrackingSpeedKp", "PathTrackingSpeedKi", ...
    "PathTrackingAntiWindupGain", "PathTrackingMaximumAcceleration", ...
    "PathTrackingMaximumDeceleration", "PathTrackingBrakePressurePerAcceleration"]);
assignPathTrackingControlParameters(get_param(modelName, "ModelWorkspace"));
add_block("simulink/Signal Routing/Bus Creator", modelName + "/DriverCommandBus", ...
    "Position", [520 95 550 255], "Inputs", "10", ...
    "OutDataTypeStr", "Bus: DriverCommandBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/DriverCommand", ...
    "Position", [620 160 650 174], "Port", "1", ...
    "OutDataTypeStr", "Bus: DriverCommandBus");
add_line(modelName, "TrackReference/1", "PathTrackingCore/1");
add_line(modelName, "Sensor/1", "PathTrackingCore/2");
for port = 1:10
    add_line(modelName, "PathTrackingCore/" + port, "DriverCommandBus/" + port);
end
add_line(modelName, "DriverCommandBus/1", "DriverCommand/1");
set_param(modelName, "StopTime", "10", "Solver", "ode4", "FixedStep", "0.01");
save_system(modelName);
close_system(modelName, 0);
end

function createVehicleController(projectRoot, dictionaryName, forceRebuild)
modelName = "PathTrackingVehicleController";
if ~prepareModel(modelName, fullfile(projectRoot, "models", "controller"), dictionaryName, forceRebuild)
    return
end
add_block("simulink/Ports & Subsystems/In1", modelName + "/DriverCommand", ...
    "Position", [30 40 60 54], "Port", "1", ...
    "OutDataTypeStr", "Bus: DriverCommandBus");
add_block("simulink/Ports & Subsystems/In1", modelName + "/Sensor", ...
    "Position", [30 160 60 174], "Port", "2", ...
    "OutDataTypeStr", "Bus: SensorBus");
add_block("simulink/Ports & Subsystems/In1", modelName + "/PowertrainState", ...
    "Position", [30 280 60 294], "Port", "3", ...
    "OutDataTypeStr", "Bus: PowertrainStateBus");
add_block("simulink/User-Defined Functions/MATLAB Function", ...
    modelName + "/VehicleControllerCore", "Position", [180 100 450 300]);
setChartScript(modelName + "/VehicleControllerCore", ...
    fullfile(projectRoot, "scripts", "path_tracking", "fsaePathTrackingVehicleControllerCore.m"), ...
    "fsaePathTrackingVehicleControllerCore");
declareChartParameters(modelName + "/VehicleControllerCore", [ ...
    "PathTrackingVehicleMass", "PathTrackingTireEffectiveRadius", "PathTrackingGearRatio", ...
    "PathTrackingGearEfficiency", "PathTrackingMotorTorqueLimit", ...
    "PathTrackingMaximumAcceleration", "PathTrackingMaximumDeceleration"]);
assignPathTrackingControlParameters(get_param(modelName, "ModelWorkspace"));
add_block("simulink/Signal Routing/Bus Creator", modelName + "/ActuatorCommandBus", ...
    "Position", [540 40 570 170], "Inputs", "7", ...
    "OutDataTypeStr", "Bus: ActuatorCommandBus");
add_block("simulink/Signal Routing/Bus Creator", modelName + "/ControllerDebugBus", ...
    "Position", [540 220 570 420], "Inputs", "11", ...
    "OutDataTypeStr", "Bus: ControllerDebugBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/ActuatorCommand", ...
    "Position", [650 90 680 104], "Port", "1", ...
    "OutDataTypeStr", "Bus: ActuatorCommandBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/ControllerDebug", ...
    "Position", [650 310 680 324], "Port", "2", ...
    "OutDataTypeStr", "Bus: ControllerDebugBus");
add_line(modelName, "DriverCommand/1", "VehicleControllerCore/1");
add_line(modelName, "Sensor/1", "VehicleControllerCore/2");
add_line(modelName, "PowertrainState/1", "VehicleControllerCore/3");
for port = 1:7
    add_line(modelName, "VehicleControllerCore/" + port, "ActuatorCommandBus/" + port);
end
for port = 8:18
    add_line(modelName, "VehicleControllerCore/" + port, "ControllerDebugBus/" + (port - 7));
end
add_line(modelName, "ActuatorCommandBus/1", "ActuatorCommand/1");
add_line(modelName, "ControllerDebugBus/1", "ControllerDebug/1");
set_param(modelName, "StopTime", "10", "Solver", "ode4", "FixedStep", "0.01");
save_system(modelName);
close_system(modelName, 0);
end

function createSensorModel(projectRoot, dictionaryName, forceRebuild)
modelName = "PathTrackingSensorModel";
if ~prepareModel(modelName, fullfile(projectRoot, "models", "components"), dictionaryName, forceRebuild)
    return
end
add_block("simulink/Ports & Subsystems/In1", modelName + "/VehicleState", ...
    "Position", [30 80 60 94], "Port", "1", ...
    "OutDataTypeStr", "Bus: VehicleStateBus");
add_block("simulink/Ports & Subsystems/In1", modelName + "/PowertrainState", ...
    "Position", [30 230 60 244], "Port", "2", ...
    "OutDataTypeStr", "Bus: PowertrainStateBus");
add_block("simulink/User-Defined Functions/MATLAB Function", ...
    modelName + "/SensorCore", "Position", [180 100 450 260]);
setChartScript(modelName + "/SensorCore", ...
    fullfile(projectRoot, "scripts", "path_tracking", "fsaePathTrackingSensorCore.m"), ...
    "fsaePathTrackingSensorCore");
add_block("simulink/Signal Routing/Bus Creator", modelName + "/SensorBus", ...
    "Position", [520 90 550 300], "Inputs", "18", ...
    "OutDataTypeStr", "Bus: SensorBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/Sensor", ...
    "Position", [620 175 650 189], "Port", "1", ...
    "OutDataTypeStr", "Bus: SensorBus");
add_line(modelName, "VehicleState/1", "SensorCore/1");
add_line(modelName, "PowertrainState/1", "SensorCore/2");
for port = 1:18
    add_line(modelName, "SensorCore/" + port, "SensorBus/" + port);
end
add_line(modelName, "SensorBus/1", "Sensor/1");
set_param(modelName, "StopTime", "10", "Solver", "ode4", "FixedStep", "0.01");
save_system(modelName);
close_system(modelName, 0);
end

function createClosedLoopTop(projectRoot, dictionaryName, defaultScenario, forceRebuild)
modelName = "FSAE_PathTracking_ClosedLoop";
if ~prepareModel(modelName, fullfile(projectRoot, "models", "top"), dictionaryName, forceRebuild)
    return
end

modelWorkspace = get_param(modelName, "ModelWorkspace");
assignin(modelWorkspace, "PathTrackingTrackData", makePathTrackingTrackData(defaultScenario.Track));
assignin(modelWorkspace, "PathTrackingEnvironment", defaultScenario.Environment);
assignin(modelWorkspace, "PathTrackingLookaheadTime", 0.10);
assignin(modelWorkspace, "PathTrackingMinLookahead", 0.5);
assignin(modelWorkspace, "PathTrackingProjectionSearchWindow", 200);
assignin(modelWorkspace, "PathTrackingControlSampleTime", 0.01);
assignin(modelWorkspace, "PathTrackingWheelbase", 2.0);
assignin(modelWorkspace, "PathTrackingHeadingGain", 1.2);
assignin(modelWorkspace, "PathTrackingCrossTrackGain", 2.0);
assignin(modelWorkspace, "PathTrackingMinimumSteeringSpeed", 1.0);
assignin(modelWorkspace, "PathTrackingMaximumSteeringAngle", 0.50);
assignin(modelWorkspace, "PathTrackingMaximumSteeringRate", 4.0);
assignin(modelWorkspace, "PathTrackingSteeringWheelRatio", 1.0);
assignin(modelWorkspace, "PathTrackingSpeedKp", 0.8);
assignin(modelWorkspace, "PathTrackingSpeedKi", 0.25);
assignin(modelWorkspace, "PathTrackingAntiWindupGain", 0.5);
assignin(modelWorkspace, "PathTrackingMaximumAcceleration", 8.0);
assignin(modelWorkspace, "PathTrackingMaximumDeceleration", 8.0);
assignin(modelWorkspace, "PathTrackingBrakePressurePerAcceleration", 1.0e4);

add_block("simulink/Sources/Constant", modelName + "/TrackDataSource", ...
    "Position", [30 60 130 90], "Value", "PathTrackingTrackData", ...
    "OutDataTypeStr", "Bus: PathTrackingTrackDataBus");
addEnvironmentSource(modelName);
add_block("simulink/User-Defined Functions/MATLAB Function", ...
    modelName + "/TrackReferenceCore", "Position", [190 60 460 210]);
setChartScript(modelName + "/TrackReferenceCore", ...
    fullfile(projectRoot, "scripts", "path_tracking", "fsaePathTrackingTrackReferenceCore.m"), ...
    "fsaePathTrackingTrackReferenceCore");
declareChartParameters(modelName + "/TrackReferenceCore", [ ...
    "PathTrackingProjectionSearchWindow", "PathTrackingMinLookahead", "PathTrackingLookaheadTime"]);
add_block("simulink/Signal Routing/Bus Creator", modelName + "/TrackReferenceBus", ...
    "Position", [520 60 550 230], "Inputs", "9", ...
    "OutDataTypeStr", "Bus: TrackReferenceBus");
add_block("simulink/Ports & Subsystems/Model", modelName + "/PathTrackingSensorModel", ...
    "Position", [460 330 600 390], "ModelName", "PathTrackingSensorModel");
add_block("simulink/Discrete/Memory", modelName + "/SensorFeedbackMemory", ...
    "Position", [640 340 690 370], ...
    "InitialCondition", "DefaultSensorBus");
add_block("simulink/Discrete/Memory", modelName + "/PowertrainFeedbackMemory", ...
    "Position", [640 400 690 430], ...
    "InitialCondition", "DefaultPowertrainStateBus");
add_block("simulink/Ports & Subsystems/Model", modelName + "/PathTrackingDriver", ...
    "Position", [640 50 790 120], "ModelName", "PathTrackingDriver");
add_block("simulink/Ports & Subsystems/Model", modelName + "/PathTrackingVehicleController", ...
    "Position", [830 50 990 140], "ModelName", "PathTrackingVehicleController");
add_block("simulink/Ports & Subsystems/Model", modelName + "/VehiclePlant", ...
    "Position", [1040 300 1200 390], "ModelName", "VehiclePlant");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/VehicleState", ...
    "Position", [1270 300 1300 314], "Port", "1", ...
    "OutDataTypeStr", "Bus: VehicleStateBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/PowertrainState", ...
    "Position", [1270 345 1300 359], "Port", "2", ...
    "OutDataTypeStr", "Bus: PowertrainStateBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/Sensor", ...
    "Position", [1270 390 1300 404], "Port", "3", ...
    "OutDataTypeStr", "Bus: SensorBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/TrackReference", ...
    "Position", [1270 435 1300 449], "Port", "4", ...
    "OutDataTypeStr", "Bus: TrackReferenceBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/ControllerDebug", ...
    "Position", [1270 480 1300 494], "Port", "5", ...
    "OutDataTypeStr", "Bus: ControllerDebugBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/DriverCommand", ...
    "Position", [1270 525 1300 539], "Port", "6", ...
    "OutDataTypeStr", "Bus: DriverCommandBus");
add_block("simulink/Ports & Subsystems/Out1", modelName + "/ActuatorCommand", ...
    "Position", [1270 570 1300 584], "Port", "7", ...
    "OutDataTypeStr", "Bus: ActuatorCommandBus");

add_line(modelName, "TrackDataSource/1", "TrackReferenceCore/2");
add_line(modelName, "TrackReferenceCore/1", "TrackReferenceBus/1");
for port = 2:9
    add_line(modelName, "TrackReferenceCore/" + port, "TrackReferenceBus/" + port);
end
add_line(modelName, "TrackReferenceBus/1", "PathTrackingDriver/1");
add_line(modelName, "PathTrackingDriver/1", "PathTrackingVehicleController/1");
add_line(modelName, "PathTrackingVehicleController/1", "VehiclePlant/1");
add_line(modelName, "EnvironmentSource/1", "VehiclePlant/2");
add_line(modelName, "VehiclePlant/1", "PathTrackingSensorModel/1");
add_line(modelName, "VehiclePlant/2", "PathTrackingSensorModel/2");
add_line(modelName, "PathTrackingSensorModel/1", "SensorFeedbackMemory/1");
add_line(modelName, "SensorFeedbackMemory/1", "TrackReferenceCore/1");
add_line(modelName, "SensorFeedbackMemory/1", "PathTrackingDriver/2");
add_line(modelName, "SensorFeedbackMemory/1", "PathTrackingVehicleController/2");
add_line(modelName, "VehiclePlant/2", "PowertrainFeedbackMemory/1");
add_line(modelName, "PowertrainFeedbackMemory/1", "PathTrackingVehicleController/3");
add_line(modelName, "VehiclePlant/1", "VehicleState/1");
add_line(modelName, "VehiclePlant/2", "PowertrainState/1");
add_line(modelName, "SensorFeedbackMemory/1", "Sensor/1");
add_line(modelName, "TrackReferenceBus/1", "TrackReference/1");
add_line(modelName, "PathTrackingVehicleController/2", "ControllerDebug/1");
add_line(modelName, "PathTrackingDriver/1", "DriverCommand/1");
add_line(modelName, "PathTrackingVehicleController/1", "ActuatorCommand/1");
set_param(modelName, "StopTime", "10", "Solver", "ode45", ...
    "MaxStep", "0.01", "ZeroCrossAlgorithm", "Adaptive", ...
    "IgnoredZcDiagnostic", "none");
save_system(modelName);
close_system(modelName, 0);
end

function created = prepareModel(modelName, folder, dictionaryName, forceRebuild)
if isfile(fullfile(folder, modelName + ".slx")) && ~forceRebuild
    created = false;
    return
end
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
modelPath = fullfile(folder, modelName + ".slx");
if isfile(modelPath)
    delete(modelPath);
end
if ~isfolder(folder)
    mkdir(folder);
end
new_system(modelName);
set_param(modelName, "DataDictionary", dictionaryName);
save_system(modelName, modelPath);
created = true;
end

function setChartScript(blockPath, sourceFile, functionName)
chart = sfroot().find("-isa", "Stateflow.EMChart", "Path", blockPath);
assert(~isempty(chart), "FSAE:PathTrackingChartMissing", ...
    "Could not locate MATLAB Function block %s.", blockPath);
script = fileread(sourceFile);
chart.Script = strrep(script, functionName, "fcn");
end

function addEnvironmentSource(modelName)
fieldNames = [ ...
    "RoadGripScale", "RoadMuLimit", "RoadGrade", "RoadBank", ...
    "RoadHeight", "AirDensity", "WindVelocityGlobal", ...
    "AmbientTemperature", "Gravity", "EnableRoadDisturbance", ...
    "EnvironmentValid"];
add_block("simulink/Signal Routing/Bus Creator", ...
    modelName + "/EnvironmentSource", ...
    "Position", [180 400 210 650], ...
    "Inputs", string(numel(fieldNames)), ...
    "OutDataTypeStr", "Bus: EnvironmentBus");
for index = 1:numel(fieldNames)
    yPosition = 390 + 24 * index;
    add_block("simulink/Sources/Constant", ...
        modelName + "/" + fieldNames(index), ...
        "Position", [30 yPosition 140 yPosition + 16], ...
        "Value", "PathTrackingEnvironment." + fieldNames(index));
    lineHandle = add_line(modelName, fieldNames(index) + "/1", ...
        "EnvironmentSource/" + index);
    set_param(lineHandle, "Name", fieldNames(index));
end
end

function declareChartParameters(blockPath, parameterNames)
chart = sfroot().find("-isa", "Stateflow.EMChart", "Path", blockPath);
assert(~isempty(chart), "FSAE:PathTrackingChartMissing", ...
    "Could not locate MATLAB Function block %s.", blockPath);
for parameterName = parameterNames
    data = chart.find("-isa", "Stateflow.Data", "Name", parameterName);
    if isempty(data)
        data = Stateflow.Data(chart);
        data.Name = char(parameterName);
    end
    data.Scope = "Parameter";
end
end

function bus = createTrackDataBus(maxSamples)
bus = Simulink.Bus;
bus.Elements = [ ...
    makeElement("SampleCount", "uint32", [1 1]), ...
    makeElement("SampleDistance", "double", [1 1]), ...
    makeElement("TrackLength", "double", [1 1]), ...
    makeElement("IsClosed", "boolean", [1 1]), ...
    makeElement("ReferencePathValid", "boolean", [1 1]), ...
    makeElement("ReferenceSampleDistance", "double", [1 1]), ...
    makeElement("X", "double", [maxSamples 1]), ...
    makeElement("Y", "double", [maxSamples 1]), ...
    makeElement("Heading", "double", [maxSamples 1]), ...
    makeElement("Curvature", "double", [maxSamples 1]), ...
    makeElement("ReferenceX", "double", [maxSamples 1]), ...
    makeElement("ReferenceY", "double", [maxSamples 1]), ...
    makeElement("ReferenceHeading", "double", [maxSamples 1]), ...
    makeElement("ReferenceCurvature", "double", [maxSamples 1]), ...
    makeElement("ReferenceEnvelopeClearance", ...
        "double", [maxSamples 1]), ...
    makeElement("ReferenceBoundaryCorrectionSign", ...
        "double", [maxSamples 1]), ...
    makeElement("ReferenceSpeed", "double", [maxSamples 1]), ...
    makeElement("LeftHalfWidth", "double", [maxSamples 1]), ...
    makeElement("RightHalfWidth", "double", [maxSamples 1])];
end

function element = makeElement(name, dataType, dimensions)
element = Simulink.BusElement;
element.Name = name;
element.DataType = dataType;
element.Dimensions = dimensions;
end

function installPathTrackingTrackDataBus(dictionaryPath)
dictionary = Simulink.data.dictionary.open(dictionaryPath);
cleanup = onCleanup(@() close(dictionary));
designData = getSection(dictionary, "Design Data");
bus = createTrackDataBus(4096);
try
    entry = getEntry(designData, "PathTrackingTrackDataBus");
    setValue(entry, bus);
catch exception
    if contains(exception.identifier, "EntryNotFound") || ...
            contains(exception.message, "does not exist", "IgnoreCase", true)
        addEntry(designData, "PathTrackingTrackDataBus", bus);
    else
        rethrow(exception);
    end
end
saveChanges(dictionary);
end

function assignPathTrackingControlParameters(modelWorkspace)
assignin(modelWorkspace, "PathTrackingControlSampleTime", 0.01);
% Stand-alone synthetic fallback. createPathTrackingSimulationInput replaces this
% value with Vehicle.Wheelbase in the PathTrackingDriver workspace.
assignin(modelWorkspace, "PathTrackingWheelbase", 2.0);
assignin(modelWorkspace, "PathTrackingHeadingGain", 1.2);
assignin(modelWorkspace, "PathTrackingCrossTrackGain", 2.0);
assignin(modelWorkspace, "PathTrackingMinimumSteeringSpeed", 1.0);
assignin(modelWorkspace, "PathTrackingMaximumSteeringAngle", 0.50);
assignin(modelWorkspace, "PathTrackingMaximumSteeringRate", 4.0);
assignin(modelWorkspace, "PathTrackingSteeringWheelRatio", 1.0);
assignin(modelWorkspace, "PathTrackingSpeedKp", 0.8);
assignin(modelWorkspace, "PathTrackingSpeedKi", 0.25);
assignin(modelWorkspace, "PathTrackingAntiWindupGain", 0.5);
assignin(modelWorkspace, "PathTrackingMaximumAcceleration", 8.0);
assignin(modelWorkspace, "PathTrackingMaximumDeceleration", 8.0);
assignin(modelWorkspace, "PathTrackingBrakePressurePerAcceleration", 1.0e4);
% Explicit synthetic defaults keep the generated reference model compilable.
% createPathTrackingSimulationInput overrides these from the shared parameter records.
assignin(modelWorkspace, "PathTrackingVehicleMass", 250.0);
assignin(modelWorkspace, "PathTrackingTireEffectiveRadius", 0.23);
assignin(modelWorkspace, "PathTrackingGearRatio", 10.0);
assignin(modelWorkspace, "PathTrackingGearEfficiency", 0.95);
assignin(modelWorkspace, "PathTrackingMotorTorqueLimit", 40.0);
end
