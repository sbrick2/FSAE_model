function matrix = createTorqueVectoringScenarioMatrix(options)
%CREATETorqueVectoringSCENARIOMATRIX Create paired no-TV/TV TorqueVectoring verification cases.
%   Within each pair, the Simulink model, track, environment, initial state,
%   solver setup, tire model, and stop time are identical. EnableTV is the
%   only intended configuration difference.

arguments
    options.Events (1, :) string = ["Skidpad", "Autocross"]
    options.IncludeTTCMapCrossCheck (1, 1) logical = false
end

projectRoot = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
addpath(fullfile(projectRoot, "scenarios", "Skidpad"));
addpath(fullfile(projectRoot, "scenarios", "Autocross"));

matrix = repmat(emptyCase(), 0, 1);
for eventName = options.Events
    scenario = createScenario(eventName);
    tireModels = "MF62";
    if options.IncludeTTCMapCrossCheck
        tireModels(end + 1) = "TTCMAP"; %#ok<AGROW>
    end
    for tireModel = tireModels
        pairID = "TorqueVectoring-" + upper(eventName) + "-" + tireModel;
        for enableTV = [false, true]
            caseDefinition = emptyCase();
            caseDefinition.ID = pairID + conditionalSuffix(enableTV);
            caseDefinition.PairID = pairID;
            caseDefinition.Event = eventName;
            caseDefinition.EnableTV = enableTV;
            caseDefinition.TireModel = tireModel;
            caseDefinition.ModelName = "FSAE_TorqueVectoring_ClosedLoop";
            caseDefinition.Scenario = scenario;
            caseDefinition.ConfigurationFingerprint = struct( ...
                "TrackID", scenario.ID, ...
                "StopTime", scenario.StopTime, ...
                "TireModel", tireModel, ...
                "ModelName", caseDefinition.ModelName);
            matrix(end + 1, 1) = caseDefinition; %#ok<AGROW>
        end
    end
end
validatePairs(matrix);
end

function scenario = createScenario(eventName)
switch lower(eventName)
    case "skidpad"
        scenario = createPathTrackingSkidpadScenario();
    case "autocross"
        scenario = createPathTrackingAutocrossScenario();
    otherwise
        error("FSAE:TorqueVectoring:UnsupportedEvent", ...
            "Unsupported TorqueVectoring event '%s'.", eventName);
end
end

function suffix = conditionalSuffix(enableTV)
if enableTV
    suffix = "-TV1";
else
    suffix = "-TV0";
end
end

function validatePairs(matrix)
pairIDs = unique([matrix.PairID], "stable");
for pairID = pairIDs
    indices = find([matrix.PairID] == pairID);
    assert(numel(indices) == 2, "FSAE:TorqueVectoring:ScenarioPair", ...
        "Pair %s must contain exactly TV0 and TV1.", pairID);
    pair = matrix(indices);
    assert(isequal(sort([pair.EnableTV]), [false, true]), ...
        "FSAE:TorqueVectoring:ScenarioSwitch", ...
        "Pair %s does not contain one disabled and one enabled case.", ...
        pairID);
    assert(isequaln(pair(1).ConfigurationFingerprint, ...
        pair(2).ConfigurationFingerprint), ...
        "FSAE:TorqueVectoring:ScenarioMismatch", ...
        "Pair %s differs in more than EnableTV.", pairID);
    assert(isequaln(pair(1).Scenario, pair(2).Scenario), ...
        "FSAE:TorqueVectoring:ScenarioDataMismatch", ...
        "Pair %s does not share identical scenario data.", pairID);
end
end

function output = emptyCase()
output = struct( ...
    "ID", "", "PairID", "", "Event", "", "EnableTV", false, ...
    "TireModel", "", "ModelName", "", "Scenario", struct, ...
    "ConfigurationFingerprint", struct);
end
