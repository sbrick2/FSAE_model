function in = createTorqueVectoringSimulationInput(caseDefinition, parameters)
%CREATETorqueVectoringSIMULATIONINPUT Configure one paired TorqueVectoring verification run.
%   Uses the installed TTC-derived MF6.2 model by default and overrides only
%   TorqueVectoringEnableTV between the paired TV0/TV1 cases.

arguments
    caseDefinition (1, 1) struct
    parameters (1, 1) struct
end

modelMode = tireMode(caseDefinition.TireModel);
tireSelection = struct( ...
    "ModelMode", uint8(modelMode), ...
    "ModelModeMeaning", "0=TireSimple, 1=TireMF62, 2=TireTTCMap");
in = createPathTrackingSimulationInput(caseDefinition.Scenario, parameters, ...
    ModelName = caseDefinition.ModelName, ...
    TireSelection = tireSelection);
in = in.setVariable("TorqueVectoringEnableTV", logical(caseDefinition.EnableTV));
in = in.setModelParameter( ...
    SignalLogging = "on", ...
    SignalLoggingName = "logsout", ...
    ReturnWorkspaceOutputs = "on");
end

function mode = tireMode(name)
switch upper(string(name))
    case "SIMPLE"
        mode = 0;
    case "MF62"
        mode = 1;
    case "TTCMAP"
        mode = 2;
    otherwise
        error("FSAE:TorqueVectoring:UnknownTireModel", ...
            "Unknown TorqueVectoring tire model '%s'.", name);
end
end
