function metrics = summarizeTorqueVectoringRun(simulationOutput, caseDefinition)
%SUMMARIZETorqueVectoringRUN Calculate TorqueVectoring verification metrics for one simulation.

arguments
    simulationOutput (1, 1) Simulink.SimulationOutput
    caseDefinition (1, 1) struct
end

runConfig = struct( ...
    "CaseID", caseDefinition.ID, ...
    "PairID", caseDefinition.PairID, ...
    "EnableTV", caseDefinition.EnableTV, ...
    "TireModel", caseDefinition.TireModel);
result = collectLapSimulationResults( ...
    simulationOutput, caseDefinition.Scenario, runConfig);
time = result.Time;

metrics = struct( ...
    "CaseID", caseDefinition.ID, ...
    "PairID", caseDefinition.PairID, ...
    "Event", caseDefinition.Event, ...
    "EnableTV", caseDefinition.EnableTV, ...
    "TireModel", caseDefinition.TireModel, ...
    "SimulationTime", terminalValue(time), ...
    "YawRateRMSE", rmseFinite(result.Controller.YawRateError), ...
    "PeakAbsYawRateError", maxAbsFinite( ...
        result.Controller.YawRateError), ...
    "LateralErrorRMSE", rmseFinite(result.Track.LateralError), ...
    "PeakAbsLateralError", maxAbsFinite(result.Track.LateralError), ...
    "DriveEnergyJ", integrateFinite(time, ...
        positivePart(result.Battery.Power)), ...
    "RecoveredEnergyJ", -integrateFinite(time, ...
        negativePart(result.Battery.Power)), ...
    "NetBatteryEnergyJ", integrateFinite(time, result.Battery.Power), ...
    "PeakTireUtilization", maxFinite(result.Tire.MuUtilization), ...
    "MeanTireUtilization", meanFinite(result.Tire.MuUtilization), ...
    "ControllerSaturationFraction", ...
        meanFinite(result.Controller.ControllerSaturated), ...
    "TVActiveFraction", meanFinite(result.Controller.TVActive), ...
    "BoundaryViolationFraction", ...
        meanFinite(result.Track.BoundaryViolation), ...
    "PeakMotorTorqueRequest", ...
        maxAbsFinite(result.Actuator.MotorTorqueRequest), ...
    "PeakAllocatedYawMoment", ...
        maxAbsFinite(result.Controller.AllocatedYawMoment), ...
    "DataValid", result.Meta.Valid, ...
    "MissingSignals", result.Meta.MissingSignals, ...
    "NonFiniteSignals", result.Meta.NonFiniteSignals);
end

function value = rmseFinite(data)
data = finiteValues(data);
if isempty(data)
    value = NaN;
else
    value = sqrt(mean(data.^2));
end
end

function value = maxAbsFinite(data)
data = finiteValues(data);
if isempty(data)
    value = NaN;
else
    value = max(abs(data));
end
end

function value = maxFinite(data)
data = finiteValues(data);
if isempty(data)
    value = NaN;
else
    value = max(data);
end
end

function value = meanFinite(data)
data = finiteValues(data);
if isempty(data)
    value = NaN;
else
    value = mean(data);
end
end

function value = terminalValue(data)
data = finiteValues(data);
if isempty(data)
    value = NaN;
else
    value = data(end);
end
end

function value = integrateFinite(time, data)
if isempty(time) || isempty(data)
    value = NaN;
    return
end
time = double(time(:));
data = double(data);
if size(data, 1) ~= numel(time)
    value = NaN;
    return
end
if size(data, 2) > 1
    data = sum(data, 2);
end
mask = isfinite(time) & isfinite(data);
if nnz(mask) < 2
    value = NaN;
else
    value = trapz(time(mask), data(mask));
end
end

function output = positivePart(data)
if isempty(data)
    output = data;
else
    output = max(double(data), 0.0);
end
end

function output = negativePart(data)
if isempty(data)
    output = data;
else
    output = min(double(data), 0.0);
end
end

function output = finiteValues(data)
if isempty(data)
    output = [];
else
    data = double(data);
    output = data(isfinite(data));
end
end
