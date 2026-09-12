function repairOpenLoopPlantVehiclePlant()
%REPAIROpenLoopPlantVEHICLEPLANT Finalize OpenLoopPlant integration diagnostics and logging.
%   This migration is idempotent. It names component boundary signals,
%   separates unit-incompatible zero sources, enables the ResultVisualization diagnostic
%   signal logs, and tightens integration diagnostics.

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scripts", "initialization"));
project = initProject(); %#ok<NASGU>

repairVehicle7DOFDiagnostics();

referencedModels = [ ...
    "Vehicle7DOF", "WheelSlipKinematics", resolveTireModelName(), ...
    "LoadTransferModel", "AeroModel", "PowertrainModel", "BatteryModel"];
for model = referencedModels
    open_system(model);
    if model == "Vehicle7DOF"
        minimizeArtificialLoops = "on";
    else
        minimizeArtificialLoops = "off";
    end
    set_param(model, "ModelReferenceMinAlgLoopOccurrences", ...
        minimizeArtificialLoops);
    save_system(model);
    close_system(model);
end

model = "VehiclePlant";
open_system(model);

replaceSharedZeroSources(model);
insertMemoryBreaks(model);

configureOutputSignals(model, "Vehicle7DOF", [ ...
    "X", "Y", "Psi", "Ux", "Uy", "YawRate", "UxDot", "UyDot", ...
    "Ax", "Ay", "YawAcceleration", "WheelSpeed", ...
    "WheelVelocityXWheel", "WheelVelocityYWheel"], [13, 14]);
configureOutputSignals(model, "WheelSlipKinematics", [ ...
    "SlipRatio", "SlipAngle", "RegularizedSpeed", "LowSpeedBlend"], 4);
configureOutputSignals(model, resolveTireModelName(model), [ ...
    "TireFx", "TireFy", "MuUtilization", "SaturationScale"], [3, 4]);
configureOutputSignals(model, "LoadTransferModel", [ ...
    "NormalLoad", "FrontAxleLoad", "RearAxleLoad", ...
    "TotalNormalLoad"], []);
configureOutputSignals(model, "AeroModel", [ ...
    "ForceXBody", "ForceYBody", "DownforceFront", "DownforceRear", ...
    "RelativeAirSpeed", "DynamicPressure"], 1:6);
configureOutputSignals(model, "PowertrainModel", [ ...
    "WheelAppliedTorque", "MotorSpeed", "MotorTorqueActual", ...
    "MotorMechanicalPower", "ElectricalPowerUnconstrained", ...
    "ElectricalPowerActual", "SpeedLimitScale", ...
    "TorqueSaturationResidual"], [1, 5, 6, 7, 8]);
configureOutputSignals(model, "BatteryModel", [ ...
    "BatteryPower", "PowerScale", "BatteryCurrent", "BatterySOC", ...
    "DrivePowerClipped", "RegenPowerClipped"], [1, 2, 5, 6]);

configureLine(model, "ActuatorCmdSelect", 1, ...
    "MotorTorqueRequest", true);
configureLine(model, "SteeringMux", 1, "WheelSteerAngle", false);
configureLine(model, "CamberZero", 1, "CamberAngle", false);
configureLine(model, "MotorLimitFlagsFalse", 1, ...
    "MotorLimitActive", false);
configureLine(model, "BatteryVoltageConst", 1, "BatteryVoltage", false);
configureLine(model, "VehicleStateBusCreator", 1, ...
    "VehicleState", false);
configureLine(model, "PowertrainStateBusCreator", 1, ...
    "PowertrainState", false);

set_param(model, ...
    "BusObjectLabelMismatch", "error", ...
    "UnitsInconsistencyMsg", "warning", ...
    "AlgebraicLoopMsg", "error", ...
    "ArtificialAlgebraicLoopMsg", "error");

save_system(model);
close_system(model);
fprintf("OpenLoopPlant VehiclePlant integration repair completed.\n");
end

function name = resolveTireModelName(model)
if nargin == 0
    if isfile(which("TireModel"))
        name = "TireModel";
    else
        name = "TireSimple";
    end
elseif getSimulinkBlockHandle(model + "/TireModel") ~= -1
    name = "TireModel";
else
    name = "TireSimple";
end
end

function insertMemoryBreaks(model)
removeAccelerationMemory(model, 9, 10, 1, ...
    "AxLoadTransferMemory");
removeAccelerationMemory(model, 10, 11, 2, ...
    "AyLoadTransferMemory");
insertPowerScaleMemory(model);
end

function removeAccelerationMemory(model, vehicleOutput, busInput, ...
        loadInput, memoryName)
memoryPath = model + "/" + memoryName;
vehiclePorts = get_param(model + "/Vehicle7DOF", "PortHandles");
busPorts = get_param(model + "/VehicleStateBusCreator", "PortHandles");
loadPorts = get_param(model + "/LoadTransferModel", "PortHandles");

if getSimulinkBlockHandle(memoryPath) ~= -1
    line = get_param(vehiclePorts.Outport(vehicleOutput), "Line");
    if line ~= -1
        delete_line(line);
    end
    memoryPorts = get_param(memoryPath, "PortHandles");
    line = get_param(memoryPorts.Outport(1), "Line");
    if line ~= -1
        delete_line(line);
    end
    delete_block(memoryPath);
end
if get_param(busPorts.Inport(busInput), "Line") == -1
    add_line(model, vehiclePorts.Outport(vehicleOutput), ...
        busPorts.Inport(busInput), "autorouting", "on");
end
if get_param(loadPorts.Inport(loadInput), "Line") == -1
    add_line(model, vehiclePorts.Outport(vehicleOutput), ...
        loadPorts.Inport(loadInput), "autorouting", "on");
end
end

function insertPowerScaleMemory(model)
memoryName = "PowerScaleMemory";
memoryPath = model + "/" + memoryName;
switchPorts = get_param(model + "/PowerScaleStartupFallback", "PortHandles");
powertrainPorts = get_param(model + "/PowertrainModel", "PortHandles");
if getSimulinkBlockHandle(memoryPath) == -1
    line = get_param(switchPorts.Outport(1), "Line");
    if line ~= -1
        delete_line(line);
    end
    powertrainPosition = get_param(model + "/PowertrainModel", "Position");
    memoryPosition = [ ...
        powertrainPosition(1) - 120, powertrainPosition(2) + 70, ...
        powertrainPosition(1) - 70, powertrainPosition(2) + 95];
    add_block("simulink/Discrete/Memory", memoryPath, ...
        "InitialCondition", "1", ...
        "Position", memoryPosition);
    memoryPorts = get_param(memoryPath, "PortHandles");
    add_line(model, switchPorts.Outport(1), ...
        memoryPorts.Inport(1), "autorouting", "on");
    add_line(model, memoryPorts.Outport(1), ...
        powertrainPorts.Inport(3), "autorouting", "on");
    add_line(model, memoryPorts.Outport(1), ...
        powertrainPorts.Inport(4), "autorouting", "on");
end
configureLine(model, memoryName, 1, "PowerScaleApplied", false);
end

function repairVehicle7DOFDiagnostics()
model = "Vehicle7DOF";
open_system(model);
diagnostics = [ ...
    struct("Outport", "UxDot", "Memory", "UxDotDiagnosticMemory"), ...
    struct("Outport", "UyDot", "Memory", "UyDotDiagnosticMemory"), ...
    struct("Outport", "Ax", "Memory", "AxDiagnosticMemory"), ...
    struct("Outport", "Ay", "Memory", "AyDiagnosticMemory"), ...
    struct("Outport", "YawAcceleration", ...
        "Memory", "YawAccelerationDiagnosticMemory"), ...
    struct("Outport", "WheelVelocityXWheel", ...
        "Memory", "WheelVelocityXDiagnosticMemory"), ...
    struct("Outport", "WheelVelocityYWheel", ...
        "Memory", "WheelVelocityYDiagnosticMemory")];
for index = 1:numel(diagnostics)
    insertDiagnosticMemory(model, diagnostics(index));
end
save_system(model);
close_system(model);
end

function insertDiagnosticMemory(model, definition)
outportPath = model + "/" + definition.Outport;
memoryPath = model + "/" + definition.Memory;
outportPorts = get_param(outportPath, "PortHandles");
if getSimulinkBlockHandle(memoryPath) == -1
    line = get_param(outportPorts.Inport(1), "Line");
    assert(line ~= -1, "FSAE:OpenLoopPlantRepair:DiagnosticSource", ...
        "%s has no source signal.", definition.Outport);
    sourcePort = get_param(line, "SrcPortHandle");
    delete_line(line);
    outportPosition = get_param(outportPath, "Position");
    memoryPosition = [ ...
        outportPosition(1) - 100, outportPosition(2), ...
        outportPosition(1) - 50, outportPosition(2) + 25];
    add_block("simulink/Discrete/Memory", memoryPath, ...
        "InitialCondition", "0", ...
        "Position", memoryPosition);
    memoryPorts = get_param(memoryPath, "PortHandles");
    add_line(model, sourcePort, memoryPorts.Inport(1), ...
        "autorouting", "on");
    add_line(model, memoryPorts.Outport(1), outportPorts.Inport(1), ...
        "autorouting", "on");
end
memoryPorts = get_param(memoryPath, "PortHandles");
line = get_param(memoryPorts.Outport(1), "Line");
set_param(line, "Name", definition.Outport);
end

function replaceSharedZeroSources(model)
replaceWithTypedConstants(model, "RollPitchVertAzZero", [ ...
    struct("Name", "RollAngleZero", "Port", 7, "Signal", "RollAngle"), ...
    struct("Name", "PitchAngleZero", "Port", 8, "Signal", "PitchAngle"), ...
    struct("Name", "VerticalPositionZero", "Port", 9, ...
        "Signal", "VerticalPosition"), ...
    struct("Name", "VerticalAccelerationZero", "Port", 12, ...
        "Signal", "Az")]);

replaceWithTypedConstants(model, "LimitFlagsFalse", [ ...
    struct("Name", "TotalPowerLimitFalse", "Port", 9, ...
        "Signal", "TotalPowerLimitActive"), ...
    struct("Name", "RegenLimitFalse", "Port", 10, ...
        "Signal", "RegenLimitActive")]);
end

function replaceWithTypedConstants(model, oldName, definitions)
oldPath = model + "/" + oldName;
if getSimulinkBlockHandle(oldPath) ~= -1
    position = get_param(oldPath, "Position");
    portHandles = get_param(oldPath, "PortHandles");
    line = get_param(portHandles.Outport(1), "Line");
    if line ~= -1
        delete_line(line);
    end
    delete_block(oldPath);
else
    position = [30, 30, 60, 50];
end

busCreator = model + "/" + ...
    selectBusCreator(oldName);
busPorts = get_param(busCreator, "PortHandles");
for index = 1:numel(definitions)
    definition = definitions(index);
    blockPath = model + "/" + definition.Name;
    if getSimulinkBlockHandle(blockPath) == -1
        offset = [0, 45 * (index - 1), 0, 45 * (index - 1)];
        add_block("simulink/Sources/Constant", blockPath, ...
            "Value", "0", ...
            "OutDataTypeStr", constantDataType(oldName), ...
            "Position", position + offset);
    end
    sourcePorts = get_param(blockPath, "PortHandles");
    destinationPort = busPorts.Inport(definition.Port);
    if get_param(destinationPort, "Line") == -1
        add_line(model, sourcePorts.Outport(1), destinationPort, ...
            "autorouting", "on");
    end
    configureLine(model, definition.Name, 1, definition.Signal, false);
end
end

function name = selectBusCreator(oldName)
if oldName == "RollPitchVertAzZero"
    name = "VehicleStateBusCreator";
else
    name = "PowertrainStateBusCreator";
end
end

function dataType = constantDataType(oldName)
if oldName == "LimitFlagsFalse"
    dataType = "boolean";
else
    dataType = "double";
end
end

function configureOutputSignals(model, blockName, names, loggedIndices)
blockPath = model + "/" + blockName;
ports = get_param(blockPath, "PortHandles");
assert(numel(ports.Outport) == numel(names), ...
    "FSAE:OpenLoopPlantRepair:PortCount", ...
    "%s output count changed; update the signal contract.", blockName);
for index = 1:numel(names)
    line = get_param(ports.Outport(index), "Line");
    assert(line ~= -1, "FSAE:OpenLoopPlantRepair:MissingLine", ...
        "%s output %d is unconnected.", blockName, index);
    set_param(line, "Name", names(index));
    if any(loggedIndices == index)
        set_param(ports.Outport(index), ...
            "DataLogging", "on", ...
            "DataLoggingNameMode", "Custom", ...
            "DataLoggingName", names(index));
    else
        set_param(ports.Outport(index), "DataLogging", "off");
    end
end
end

function configureLine(model, blockName, outputPort, signalName, logSignal)
blockPath = model + "/" + blockName;
ports = get_param(blockPath, "PortHandles");
line = get_param(ports.Outport(outputPort), "Line");
assert(line ~= -1, "FSAE:OpenLoopPlantRepair:MissingLine", ...
    "%s output %d is unconnected.", blockName, outputPort);
if blockName ~= "ActuatorCmdSelect"
    set_param(line, "Name", signalName);
end
if logSignal
    set_param(ports.Outport(outputPort), ...
        "DataLogging", "on", ...
        "DataLoggingNameMode", "Custom", ...
        "DataLoggingName", signalName);
else
    set_param(ports.Outport(outputPort), "DataLogging", "off");
end
end
